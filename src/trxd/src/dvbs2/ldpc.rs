//! DVB-S2 short-frame LDPC decoder: layered belief propagation over the
//! parity checks of EN 302 307-1 5.3.2 (information bits by the address
//! tables, parity bits by the accumulator staircase), with the exact check
//! rule (box-plus, forward/backward per check). Min-sum was tried first: fine
//! at 1/2 and 2/3, but it lost 3-4 dB on the low rates (1/4, 1/3) that are
//! there for robustness. Early stop once every check holds.
//!
//! Also `trxd --ldpc-helper --modcod N --shortframes`: leandvb's external
//! decoder protocol (16200 int8 LLRs per frame on stdin, positive = 0, the
//! corrected frame back on stdout), so the decoder can be tested behind an
//! independent demodulator.

use super::{NLDPC, Rate};

/// Only a guard against infinities. A tight clamp here (30 was tried) breaks
/// the layered bookkeeping once messages saturate: frames with a handful of
/// errors at high SNR diverged.
const LLR_MAX: f32 = 1.0e4;

/// ln(1 + e^-x) for x >= 0, from a table in steps of 1/32 (the error, under
/// 0.008, costs nothing measurable; exp and ln per edge cost the Cortex-A9 a lot).
#[inline]
fn corr(x: f32) -> f32 {
    const STEP: f32 = 32.0;
    static TABLE: std::sync::OnceLock<Vec<f32>> = std::sync::OnceLock::new();
    let t = TABLE.get_or_init(|| (0..12 * STEP as usize).map(|i| (-(i as f32 + 0.5) / STEP).exp().ln_1p()).collect());
    t.get((x * STEP) as usize).copied().unwrap_or(0.0)
}

/// phi(x) = -ln(tanh(x / 2)) = ln((e^x + 1) / (e^x - 1)), its own inverse:
/// the check rule is phi(sum of phi(|L|)) over the other edges. Tables in
/// the middle (steps of 1/1024 below 1, where phi is steep; 1/64 up to 16);
/// outside them it is computed, in f64:
/// - above 16, phi(x) = 2 e^-x: a table that returned 0 there made every
///   strong edge vanish from the sum, and a check with only strong other
///   edges then sent the cap (40) instead of about the weakest of them;
/// - below 1/1024, phi(x) = ln(2 / x): a table value capped every message
///   at 8.3.
/// Real LLRs reach 100; with either shortcut half the frames of a real
/// recording failed that box-plus decoded.
#[inline]
fn phi(x: f64) -> f64 {
    const FINE: f64 = 1024.0;
    const COARSE: f64 = 64.0;
    static TABLES: std::sync::OnceLock<(Vec<f32>, Vec<f32>)> = std::sync::OnceLock::new();
    let f = |x: f64| (-(x / 2.0).tanh().ln()) as f32;
    let (fine, coarse) = TABLES.get_or_init(|| {
        (
            (0..FINE as usize).map(|i| f((i as f64 + 0.5) / FINE)).collect(),
            (0..16 * COARSE as usize).map(|i| f((i as f64 + 0.5) / COARSE)).collect(),
        )
    });
    if x < 1.0 / FINE {
        (2.0 / x.max(1e-300)).ln().min(LLR_MAX as f64)
    } else if x < 1.0 {
        fine[(x * FINE) as usize] as f64
    } else if x < 16.0 {
        coarse[(x * COARSE) as usize] as f64
    } else {
        2.0 * (-x).exp()
    }
}

/// L(a xor b) from L(a) and L(b), exactly.
#[inline]
fn boxplus(a: f32, b: f32) -> f32 {
    let s = if (a < 0.0) != (b < 0.0) { -1.0 } else { 1.0 };
    s * a.abs().min(b.abs()) + corr((a + b).abs()) - corr((a - b).abs())
}

pub struct Decoder {
    n: usize,
    /// Variables of each check, flattened; `start[j]..start[j+1]`.
    vars: Vec<u32>,
    start: Vec<u32>,
    /// Check-to-variable messages, one per edge (same order as `vars`).
    msg: Vec<f32>,
    post: Vec<f32>,
    fwd: Vec<f32>,
    ext: Vec<f32>,
    extd: Vec<f64>,
    pub max_iter: usize,
    /// Normalized min-sum instead of the exact rule: half the work, but it
    /// costs 1.5 dB at 1/2 and 3-4 dB at 1/4 and 1/3 (measured). Off.
    pub min_sum: bool,
    /// The exact rule in its phi (tanh) form: one table look-up per edge in
    /// and one out, instead of four for the forward/backward box-plus.
    pub phi: bool,
}

/// Min-sum normalization.
const ALPHA: f32 = 0.75;

impl Decoder {
    pub fn new(rate: Rate) -> Self {
        let k = rate.kldpc();
        let nk = NLDPC - k;
        let q = nk / 360;
        let mut checks: Vec<Vec<u32>> = vec![Vec::new(); nk];
        for (row, addrs) in rate.table().iter().enumerate() {
            for m in 0..360 {
                let v = (row * 360 + m) as u32;
                for &x in addrs.iter() {
                    checks[(x as usize + m * q) % nk].push(v);
                }
            }
        }
        // Accumulator: check j also holds parity bits j and j - 1.
        for (j, c) in checks.iter_mut().enumerate() {
            c.push((k + j) as u32);
            if j > 0 {
                c.push((k + j - 1) as u32);
            }
        }
        let mut vars = Vec::new();
        let mut start = vec![0u32];
        for c in &checks {
            vars.extend_from_slice(c);
            start.push(vars.len() as u32);
        }
        let edges = vars.len();
        let dmax = start.windows(2).map(|w| (w[1] - w[0]) as usize).max().unwrap_or(0);
        Decoder {
            n: NLDPC,
            vars,
            start,
            msg: vec![0.0; edges],
            post: vec![0.0; NLDPC],
            fwd: vec![0.0; dmax],
            ext: vec![0.0; dmax],
            extd: vec![0.0; dmax],
            max_iter: 50,
            min_sum: std::env::var("DVBS2_LDPC").is_ok_and(|v| v == "minsum"),
            phi: !std::env::var("DVBS2_LDPC").is_ok_and(|v| v == "boxplus" || v == "minsum"),
        }
    }

    /// Decode `llr` (positive = 0, any scale) in place into hard bits (0/1).
    /// Returns the iterations used, or None if checks still fail.
    pub fn decode(&mut self, llr: &[f32], bits: &mut [u8]) -> Option<usize> {
        assert_eq!(llr.len(), self.n);
        self.post.copy_from_slice(llr);
        self.msg.iter_mut().for_each(|m| *m = 0.0);
        let checks = self.start.len() - 1;
        let mut result = None;
        let mut best = (usize::MAX, 0usize);
        for it in 1..=self.max_iter {
            for j in 0..checks {
                let (a, b) = (self.start[j] as usize, self.start[j + 1] as usize);
                if self.min_sum {
                    let (mut min1, mut min2, mut idx, mut neg) = (f32::MAX, f32::MAX, 0usize, false);
                    for e in a..b {
                        let v = self.vars[e] as usize;
                        let t = self.post[v] - self.msg[e];
                        self.post[v] = t;
                        let m = t.abs();
                        neg ^= t < 0.0;
                        if m < min1 {
                            min2 = min1;
                            min1 = m;
                            idx = e;
                        } else if m < min2 {
                            min2 = m;
                        }
                    }
                    let (m1, m2) = (ALPHA * min1, ALPHA * min2);
                    for e in a..b {
                        let v = self.vars[e] as usize;
                        let t = self.post[v];
                        let mag = if e == idx { m2 } else { m1 };
                        let r = if neg ^ (t < 0.0) { -mag } else { mag };
                        self.msg[e] = r;
                        self.post[v] = t + r;
                    }
                    continue;
                }
                if self.phi {
                    // The sum in f64: "sum - own" for the one weak edge among
                    // strong ones must keep the others' tiny terms (1e-7),
                    // which f32 cancels to 0 (a message of 40 instead of ~15).
                    let (mut sum, mut neg) = (0f64, false);
                    for e in a..b {
                        let v = self.vars[e] as usize;
                        let t = (self.post[v] - self.msg[e]).clamp(-LLR_MAX, LLR_MAX);
                        self.post[v] = t;
                        let f = phi(t.abs() as f64);
                        self.extd[e - a] = f;
                        sum += f;
                        neg ^= t < 0.0;
                    }
                    for e in a..b {
                        let v = self.vars[e] as usize;
                        let t = self.post[v];
                        let mag = phi((sum - self.extd[e - a]).max(0.0)) as f32;
                        let r = if neg ^ (t < 0.0) { -mag } else { mag };
                        self.msg[e] = r;
                        self.post[v] = t + r;
                    }
                    continue;
                }
                let d = b - a;
                // Variable-to-check messages, and their running box-plus from the left.
                for i in 0..d {
                    let v = self.vars[a + i] as usize;
                    let t = (self.post[v] - self.msg[a + i]).clamp(-LLR_MAX, LLR_MAX);
                    self.ext[i] = t;
                    self.fwd[i] = if i == 0 { t } else { boxplus(self.fwd[i - 1], t) };
                }
                // Each edge gets everything but itself: left part box-plus right part.
                let mut bwd = 0.0f32;
                for i in (0..d).rev() {
                    let r = match (i > 0, i + 1 < d) {
                        (true, true) => boxplus(self.fwd[i - 1], bwd),
                        (true, false) => self.fwd[i - 1],
                        (false, true) => bwd,
                        (false, false) => 0.0,
                    };
                    let t = self.ext[i];
                    bwd = if i + 1 < d { boxplus(bwd, t) } else { t };
                    let v = self.vars[a + i] as usize;
                    self.msg[a + i] = r;
                    self.post[v] = t + r;
                }
            }
            let bad = self.unsatisfied();
            if bad == 0 {
                result = Some(it);
                break;
            }
            // Hopeless frames (no signal, wrong carrier) stop early: the
            // unsatisfied checks have not improved for a while.
            if bad < best.0 {
                best = (bad, it);
            } else if it - best.1 >= 8 {
                break;
            }
        }
        for (b, &p) in bits.iter_mut().zip(&self.post) {
            *b = (p < 0.0) as u8;
        }
        result
    }

    /// Parity checks the current hard decisions violate.
    fn unsatisfied(&self) -> usize {
        (0..self.start.len() - 1)
            .filter(|&j| {
                let (a, b) = (self.start[j] as usize, self.start[j + 1] as usize);
                self.vars[a..b].iter().filter(|&&v| self.post[v as usize] < 0.0).count() % 2 == 1
            })
            .count()
    }
}

/// `trxd --ldpc-file LLRS RATE`: decode a dump of LLR frames (f32 LE, 16200
/// a frame) with each algorithm; a debugging aid.
pub fn file_cli(args: &[String]) -> Result<(), String> {
    let path = args.first().ok_or("LLRS")?;
    let rate = args.get(1).and_then(|s| Rate::parse(s)).ok_or("RATE")?;
    let raw = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let llr: Vec<f32> = raw.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
    for (ms, ph, name) in [(false, false, "box-plus"), (false, true, "phi"), (true, false, "min-sum")] {
        let mut dec = Decoder::new(rate);
        dec.min_sum = ms;
        dec.phi = ph;
        let mut bits = vec![0u8; NLDPC];
        let (mut ok, mut fails) = (0, Vec::new());
        for (i, f) in llr.chunks_exact(NLDPC).enumerate() {
            if dec.decode(f, &mut bits).is_some() {
                ok += 1;
            } else {
                fails.push(i);
            }
        }
        let big = llr.iter().fold(0f32, |m, v| m.max(v.abs()));
        eprintln!("{name:>8}: {ok} decoded, failed {:?} (max |LLR| {big:.1})", &fails[..fails.len().min(12)]);
    }
    // The phi decoder under iteration caps, and what each frame needed.
    let mut dec = Decoder::new(rate);
    let mut bits = vec![0u8; NLDPC];
    let mut need = Vec::new();
    for f in llr.chunks_exact(NLDPC) {
        need.push(dec.decode(f, &mut bits));
    }
    let n = need.len();
    let mut line = String::from("     cap:");
    for cap in [50usize, 30, 20, 12, 8] {
        line += &format!("  {cap}: {}/{n}", need.iter().filter(|x| x.is_some_and(|i| i <= cap)).count());
    }
    eprintln!("{line}");
    let mut its: Vec<usize> = need.iter().flatten().copied().collect();
    its.sort();
    if !its.is_empty() {
        eprintln!("     iterations when decoded: median {}, 90 % {}, max {}", its[its.len() / 2], its[its.len() * 9 / 10], its[its.len() - 1]);
    }
    Ok(())
}

/// `trxd --ldpc-helper --modcod N [--shortframes]` for leandvb.
pub fn helper_cli(args: &[String]) -> Result<(), String> {
    use std::io::{Read, Write};
    let modcod: u32 = args
        .iter()
        .position(|a| a == "--modcod")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .ok_or("--modcod N")?;
    if !args.iter().any(|a| a == "--shortframes") {
        return Err("only short frames".into());
    }
    let rate = match modcod {
        1 => Rate::R1_4,
        2 => Rate::R1_3,
        4 => Rate::R1_2,
        6 => Rate::R2_3,
        7 => Rate::R3_4,
        m => return Err(format!("modcod {m} not supported")),
    };
    let mut dec = Decoder::new(rate);
    let (mut inp, mut out) = (std::io::stdin().lock(), std::io::stdout().lock());
    let mut buf = vec![0u8; NLDPC];
    let mut llr = vec![0f32; NLDPC];
    let mut bits = vec![0u8; NLDPC];
    let (mut frames, mut failed, mut iters) = (0u64, 0u64, 0usize);
    let (mut sat, mut mag, mut total) = (0u64, 0f64, 0u64);
    while inp.read_exact(&mut buf).is_ok() {
        // leandvb's soft bits are received amplitudes on a fixed scale, not
        // LLRs. Read them as +-mu plus Gaussian noise (variance s2) and scale
        // each frame to LLR = 2 mu v / s2, from the frame's own statistics.
        let (mut m1, mut m2) = (0f64, 0f64);
        for &b in &buf {
            let v = (b as i8) as f64;
            m1 += v.abs();
            m2 += v * v;
            sat += (v.abs() >= 127.0) as u64;
        }
        let mu = m1 / NLDPC as f64;
        let s2 = (m2 / NLDPC as f64 - mu * mu).max(mu * mu * 1e-3).max(1e-6);
        let k = (2.0 * mu / s2) as f32;
        for (l, &b) in llr.iter_mut().zip(&buf) {
            *l = (b as i8) as f32 * k;
            mag += l.abs() as f64;
        }
        total += NLDPC as u64;
        match dec.decode(&llr, &mut bits) {
            Some(i) => iters += i,
            None => failed += 1,
        }
        frames += 1;
        for (o, &b) in buf.iter_mut().zip(&bits) {
            *o = if b == 1 { (-127i8) as u8 } else { 127 };
        }
        // leandvb may exit with frames in flight: stop quietly, say what we did.
        if out.write_all(&buf).and_then(|_| out.flush()).is_err() {
            break;
        }
    }
    eprintln!(
        "ldpc-helper {}: {frames} frames, {failed} not converged, mean {:.1} iterations; input |LLR| mean {:.1}, saturated {:.1} %",
        rate.label(),
        iters as f64 / (frames - failed).max(1) as f64,
        mag / total.max(1) as f64,
        100.0 * sat as f64 / total.max(1) as f64
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbs2::{Fec, Rate};

    fn gauss(seed: &mut u64) -> f32 {
        let mut u = || {
            *seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ((*seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12)
        };
        let (a, b) = (u(), u());
        ((-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()) as f32
    }

    /// QPSK at Es/N0 a dB above the DVB-S2 short-frame thresholds (each bit
    /// is BPSK at Es/N0 - 3 dB): every frame must decode.
    #[test]
    fn decodes_qpsk_near_the_dvb_s2_threshold() {
        for (rate, esn0_db) in [(Rate::R1_4, -1.0), (Rate::R1_2, 2.0), (Rate::R1_2, 5.0), (Rate::R1_2, 8.0), (Rate::R2_3, 8.0)] {
            let fec = Fec::new(rate);
            let mut dec = Decoder::new(rate);
            let sigma = (10f32.powf(-(esn0_db - 3.0) / 10.0) / 2.0).sqrt();
            let mut seed = 7u64;
            let mut its = Vec::new();
            for f in 0..10usize {
                let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 31 + f * 17 + 3) as u8).collect();
                let cw = fec.encode(&bb);
                let llr: Vec<f32> = cw
                    .iter()
                    .map(|&b| {
                        let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed);
                        2.0 * y / (sigma * sigma)
                    })
                    .collect();
                let mut bits = vec![0u8; NLDPC];
                let it = dec.decode(&llr, &mut bits);
                let raw = llr.iter().zip(&cw).filter(|(l, b)| (**l < 0.0) as u8 != **b).count();
                let wrong = bits.iter().zip(&cw).filter(|(a, b)| a != b).count();
                assert!(it.is_some() && bits == cw, "{} frame {f} at {esn0_db} dB: {raw} raw errors, {wrong} left, it {it:?}", rate.label());
                its.push(it.unwrap());
            }
            eprintln!("{} at Es/N0 {esn0_db} dB: iterations {its:?}", rate.label());
        }
    }

    /// Real receivers hand over large LLRs (up to 100) and now and then a
    /// strongly wrong bit; the phi decoder once capped its messages and failed
    /// half of such frames (a real recording at Es/N0 8.5 dB). Every variant
    /// that decodes these must fix them.
    #[test]
    fn strong_and_strongly_wrong_llrs_still_decode() {
        let rate = Rate::R1_2;
        let fec = Fec::new(rate);
        let sigma = (10f32.powf(-(8.5 - 3.0) / 10.0) / 2.0).sqrt();
        let mut seed = 11u64;
        for f in 0..8usize {
            let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 29 + f * 3 + 1) as u8).collect();
            let cw = fec.encode(&bb);
            let mut llr: Vec<f32> = cw
                .iter()
                .map(|&b| {
                    let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed);
                    6.0 * 2.0 * y / (sigma * sigma) // overconfident: |LLR| around 75
                })
                .collect();
            // A few bits confidently wrong (a burst, an impulse).
            for k in 0..25usize {
                let i = (k * 641 + f * 97) % NLDPC;
                llr[i] = if cw[i] == 0 { -40.0 } else { 40.0 };
            }
            for (ms, ph) in [(false, false), (false, true)] {
                let mut dec = Decoder::new(rate);
                dec.min_sum = ms;
                dec.phi = ph;
                let mut bits = vec![0u8; NLDPC];
                let ok = dec.decode(&llr, &mut bits).is_some() && bits == cw;
                assert!(ok, "frame {f}, {}", if ph { "phi" } else { "box-plus" });
            }
        }
    }

    /// Frame error rate against Es/N0 (QPSK): `cargo test fer_sweep -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn fer_sweep() {
        for rate in [Rate::R1_4, Rate::R1_3, Rate::R1_2, Rate::R2_3, Rate::R3_4] {
          for (ms, ph) in [(false, false), (false, true), (true, false)] {
            let fec = Fec::new(rate);
            let mut dec = Decoder::new(rate);
            dec.min_sum = ms;
            dec.phi = ph;
            let t0 = std::time::Instant::now();
            let mut decoded = 0usize;
            let mut line = format!("{:>4} {}:", rate.label(), if ms { "min-sum " } else if ph { "phi     " } else { "box-plus" });
            for tenth in (-30..=50).step_by(5) {
                let esn0 = tenth as f32 / 10.0;
                let sigma = (10f32.powf(-(esn0 - 3.0) / 10.0) / 2.0).sqrt();
                let mut seed = 99u64 + tenth as u64;
                let (mut bad, n) = (0, 40);
                for f in 0..n {
                    let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 131 + f * 7 + 1) as u8).collect();
                    let cw = fec.encode(&bb);
                    let llr: Vec<f32> = cw
                        .iter()
                        .map(|&b| {
                            let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed);
                            2.0 * y / (sigma * sigma)
                        })
                        .collect();
                    let mut bits = vec![0u8; NLDPC];
                    decoded += 1;
                    if dec.decode(&llr, &mut bits).is_none() || bits != cw {
                        bad += 1;
                    }
                }
                line += &format!(" {esn0:+.1}:{bad:>2}");
                if bad == 0 {
                    break;
                }
            }
            eprintln!("{line}   (frames failed of 40, by Es/N0 dB; {:.2} ms a frame)", t0.elapsed().as_secs_f64() * 1e3 / decoded as f64);
          }
        }
    }

    /// Time per converging frame, 2 dB above each threshold:
    /// `cargo test --release decoder_speed -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn decoder_speed() {
        for (rate, esn0) in [(Rate::R1_4, -0.5f32), (Rate::R1_3, 0.5), (Rate::R1_2, 2.5), (Rate::R2_3, 5.5), (Rate::R3_4, 6.5)] {
            let fec = Fec::new(rate);
            let sigma = (10f32.powf(-(esn0 - 3.0) / 10.0) / 2.0).sqrt();
            let mut seed = 3u64;
            let frames: Vec<(Vec<u8>, Vec<f32>)> = (0..30usize)
                .map(|f| {
                    let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 13 + f * 5 + 9) as u8).collect();
                    let cw = fec.encode(&bb);
                    let llr = cw.iter().map(|&b| { let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed); 2.0 * y / (sigma * sigma) }).collect();
                    (cw, llr)
                })
                .collect();
            for (ms, ph, name) in [(false, false, "box-plus"), (false, true, "phi"), (true, false, "min-sum")] {
                let mut dec = Decoder::new(rate);
                dec.min_sum = ms;
                dec.phi = ph;
                let mut bits = vec![0u8; NLDPC];
                let (t0, mut ok, mut its) = (std::time::Instant::now(), 0, 0);
                for (cw, llr) in &frames {
                    if let Some(i) = dec.decode(llr, &mut bits) {
                        its += i;
                        ok += (&bits == cw) as usize;
                    }
                }
                eprintln!("{:>4} at {esn0:+.1} dB {name:>8}: {:.2} ms a frame, {ok}/30 ok, {:.1} iterations", rate.label(), t0.elapsed().as_secs_f64() * 1e3 / 30.0, its as f64 / ok.max(1) as f64);
            }
        }
    }

    /// What an iteration cap costs and saves near threshold (run on the A9
    /// for the times): `-- --ignored iteration_budget --nocapture`.
    #[test]
    #[ignore]
    fn iteration_budget() {
        let rate = Rate::R1_2;
        let fec = Fec::new(rate);
        for esn0 in [1.0f32, 1.5, 2.0, 2.5, 3.0] {
            let sigma = (10f32.powf(-(esn0 - 3.0) / 10.0) / 2.0).sqrt();
            let mut seed = 7u64;
            let frames: Vec<(Vec<u8>, Vec<f32>)> = (0..60usize)
                .map(|f| {
                    let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 13 + f * 5 + 9) as u8).collect();
                    let cw = fec.encode(&bb);
                    let llr = cw.iter().map(|&b| { let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed); 2.0 * y / (sigma * sigma) }).collect();
                    (cw, llr)
                })
                .collect();
            let mut line = format!("1/2 at {esn0:+.1} dB:");
            for cap in [50usize, 30, 20, 12] {
                let mut dec = Decoder::new(rate);
                dec.max_iter = cap;
                let mut bits = vec![0u8; NLDPC];
                let (t0, mut ok) = (std::time::Instant::now(), 0);
                for (cw, llr) in &frames {
                    if dec.decode(llr, &mut bits).is_some() {
                        ok += (&bits == cw) as usize;
                    }
                }
                line += &format!("  cap {cap}: {ok}/60 ok {:.1} ms", t0.elapsed().as_secs_f64() * 1e3 / 60.0);
            }
            eprintln!("{line}");
        }
    }

    /// The phi table's values and a checksum: `-- --ignored phi_table --nocapture`.
    #[test]
    #[ignore]
    fn phi_table() {
        let xs = [0.0f64, 0.0004, 0.01, 0.2, 0.5, 0.9999, 1.0, 1.5, 3.0, 8.0, 15.9, 16.5, 100.0];
        let vals: Vec<String> = xs.iter().map(|&x| format!("phi({x})={:.6e}", phi(x))).collect();
        eprintln!("{}", vals.join(" "));
        let (mut sum, mut bad) = (0f64, 0);
        for i in 0..20_000 {
            let v = phi(i as f64 / 1000.0);
            if !v.is_finite() || v < 0.0 {
                bad += 1;
            }
            sum += v as f64 * (i % 7 + 1) as f64;
        }
        eprintln!("checksum {sum:.9e}, non-finite or negative: {bad}");
    }

    #[test]
    fn decodes_a_noisy_codeword_back() {
        for (rate, snr_db) in [(Rate::R1_4, -1.0), (Rate::R1_2, 2.0), (Rate::R2_3, 4.0)] {
            let fec = Fec::new(rate);
            let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 73 + 5) as u8).collect();
            let cw = fec.encode(&bb);
            // BPSK, Es/N0 = snr_db: LLR = 2 y / sigma^2.
            let sigma = (10f32.powf(-snr_db / 10.0) / 2.0).sqrt();
            let mut seed = 42u64;
            let llr: Vec<f32> = cw
                .iter()
                .map(|&b| {
                    let y = if b == 0 { 1.0 } else { -1.0 } + sigma * gauss(&mut seed);
                    2.0 * y / (sigma * sigma)
                })
                .collect();
            let raw_errors = llr.iter().zip(&cw).filter(|(l, b)| (**l < 0.0) as u8 != **b).count();
            let mut dec = Decoder::new(rate);
            let mut bits = vec![0u8; NLDPC];
            let it = dec.decode(&llr, &mut bits);
            assert!(raw_errors > 100, "{} too clean to test: {raw_errors}", rate.label());
            assert!(it.is_some(), "{} did not converge ({raw_errors} raw errors)", rate.label());
            assert_eq!(bits, cw, "{}", rate.label());
        }
    }
}
