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
    pub max_iter: usize,
}

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
            max_iter: 50,
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

    /// Frame error rate against Es/N0 (QPSK): `cargo test fer_sweep -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn fer_sweep() {
        for rate in [Rate::R1_4, Rate::R1_3, Rate::R1_2, Rate::R2_3] {
            let fec = Fec::new(rate);
            let mut dec = Decoder::new(rate);
            let mut line = format!("{:>4}:", rate.label());
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
                    if dec.decode(&llr, &mut bits).is_none() || bits != cw {
                        bad += 1;
                    }
                }
                line += &format!(" {esn0:+.1}:{bad:>2}");
                if bad == 0 {
                    break;
                }
            }
            eprintln!("{line}   (frames failed of 40, by Es/N0 dB)");
        }
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
