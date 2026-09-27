//! A DVB-T2 receiver for this modulator's profile (2K, SISO, one PLP, the
//! parameters known in advance): offline, to check transmissions end to
//! end. Samples at the elementary rate in; TS packets out.
//!
//! P1 by correlation with the known waveform, fractional frequency from the
//! guard intervals, FFT per symbol, the channel from the P2 pilots (every
//! third carrier, averaged over the P2 symbols) with each symbol's common
//! phase from its pilots, then the transmitter's permutations undone and
//! the DVB-S2 decoder chain (same LDPC, BCH, BBFRAME) on the FEC blocks.

use num_complex::Complex32;
use rustfft::FftPlanner;

use super::frame::{FrameMapper, FreqInterleaver};
use super::ofdm::Ofdm;
use super::{BitInterleaver, CellInterleaver, Constellation, Params, FFT, N_P2};
use crate::dvbs2::TS_LEN;

pub struct Report {
    pub frames: usize,
    pub packets: Vec<[u8; TS_LEN]>,
    /// MER of the L1-pre cells (BPSK, known), dB, per frame.
    pub mer_db: Vec<f32>,
    pub freq_hz: f32,
    pub ldpc_fail: u64,
}

/// Where the first frame starts (P1) and a coarse frequency: the known P1
/// correlated in 16 coherent chunks of 128 samples (magnitudes summed: a
/// few kHz of offset do not matter), the offset from the chunks' phase
/// steps.
fn find_p1(x: &[Complex32], p1: &[Complex32], frame: usize, fs: f64) -> Option<(usize, f64)> {
    find_p1_in(x, p1, 0, frame + p1.len(), fs)
}

/// [`find_p1`] over start positions `from..to`.
fn find_p1_in(x: &[Complex32], p1: &[Complex32], from: usize, to: usize, fs: f64) -> Option<(usize, f64)> {
    const CH: usize = 128;
    if x.len() < to + p1.len() {
        return None;
    }
    let chunks = |t: usize| -> Vec<Complex32> {
        (0..p1.len() / CH)
            .map(|c| {
                let mut acc = Complex32::default();
                for i in c * CH..(c + 1) * CH {
                    acc += x[t + i] * p1[i].conj();
                }
                acc
            })
            .collect()
    };
    let mut best = (from, 0f32);
    for t in from..to {
        let m: f32 = chunks(t).iter().map(|z| z.norm()).sum();
        if m > best.1 {
            best = (t, m);
        }
    }
    let c = chunks(best.0);
    let mut d = Complex32::default();
    for w in c.windows(2) {
        d += w[1] * w[0].conj();
    }
    Some((best.0, d.arg() as f64 / (std::f64::consts::TAU * CH as f64) * fs))
}

pub fn receive(p: Params, x: &[Complex32], fs: f64) -> Report {
    let ofdm = Ofdm::new(p);
    let fm = FrameMapper::new(p);
    let fi = FreqInterleaver::new(&p);
    let ci = CellInterleaver::new(&p);
    let bi = BitInterleaver::new(&p);
    let mut fec = crate::dvbs2::rx::Fec::new(crate::dvbs2::FrameSpec::long(match p.rate {
        crate::dvbs2::ldpc_fpga::LongRate::R1_2 => crate::dvbs2::fpga_tx::LongMode::Qpsk12,
        crate::dvbs2::ldpc_fpga::LongRate::R3_4 => crate::dvbs2::fpga_tx::LongMode::Qpsk34,
    }));
    let mut stats = crate::dvbs2::rx::Stats::default();
    let fft = FftPlanner::new().plan_fft_forward(FFT);
    let gi = p.guard.samples();
    let sym_len = FFT + gi;
    let frame = p.frame_samples();
    let nsym = p.symbols();
    let plans: Vec<Vec<Option<Complex32>>> = (0..nsym).map(|j| ofdm.plan(j)).collect();
    let pre_ref: Vec<f32> = fm.pre_cells().iter().map(|&c| if c == super::BPSK0 { 1.0 } else { -1.0 }).collect();
    let mut report = Report { frames: 0, packets: Vec::new(), mer_db: Vec::new(), freq_hz: 0.0, ldpc_fail: 0 };
    let Some((mut start, coarse)) = find_p1(x, ofdm.p1(), frame, fs) else {
        return report;
    };
    let scale = 1.0 / (FFT as f32 * ofdm.norm());
    let mut coarse = coarse;
    let mut first = true;
    while start + frame <= x.len() {
        // Each frame: P1 again near where it should be (a gap in the
        // transmission, or the receiver's, moves it), the frequency from
        // this frame's guard intervals (modulo a carrier spacing; P1's
        // coarse estimate picks the multiple).
        if !first {
            let lo = start.saturating_sub(4096);
            match find_p1_in(x, ofdm.p1(), lo, start + 4096, fs) {
                Some((s, c)) => {
                    start = s;
                    coarse = c;
                }
                None => break,
            }
            if start + frame > x.len() {
                break;
            }
        }
        first = false;
        let mut cp = Complex32::default();
        for j in 0..nsym {
            let s = start + 2048 + j * sym_len;
            for n in 0..gi {
                cp += x[s + n].conj() * x[s + n + FFT];
            }
        }
        let spacing = fs / FFT as f64;
        let frac = cp.arg() as f64 / (std::f64::consts::TAU * FFT as f64) * fs;
        let f_off = frac + ((coarse - frac) / spacing).round() * spacing;
        report.freq_hz = f_off as f32;
        let w = -std::f64::consts::TAU * f_off / fs;
        // Each symbol: FFT (a few samples early into the guard interval,
        // the common phase takes the shift), carriers 0..1705.
        let early = if std::env::var_os("T2EARLY0").is_some() { 0 } else { gi / 4 };
        let mut carriers: Vec<Vec<Complex32>> = Vec::with_capacity(nsym);
        for j in 0..nsym {
            let s = start + 2048 + j * sym_len + gi - early;
            let mut buf: Vec<Complex32> = (0..FFT)
                .map(|n| {
                    let t = (s + n) as f64 * w;
                    x[s + n] * Complex32::new(t.cos() as f32, t.sin() as f32)
                })
                .collect();
            fft.process(&mut buf);
            // A window `early` samples before the symbol: carrier k turns
            // by exp(-j 2 pi (bin freq) early / N); undo it.
            carriers.push(
                (0..1705)
                    .map(|k| {
                        let b = ofdm.bin(k);
                        let f = if b >= FFT / 2 { b as f64 - FFT as f64 } else { b as f64 };
                        let t = std::f64::consts::TAU * f * early as f64 / FFT as f64;
                        buf[b] * scale * Complex32::new(t.cos() as f32, t.sin() as f32)
                    })
                    .collect(),
            );
        }
        // Channel from the P2 pilots, linear between them, averaged.
        let mut h = vec![Complex32::default(); 1705];
        for j in 0..N_P2 {
            let pil: Vec<(usize, Complex32)> =
                plans[j].iter().enumerate().filter_map(|(k, v)| v.filter(|z| z.norm() > 0.0).map(|z| (k, carriers[j][k] / z))).collect();
            // Each carrier once: [k0, k1) per pair of neighbouring pilots,
            // the last pilot itself at the end (visiting both ends of every
            // pair counted the pilot carriers twice: data cells on them came
            // out at half amplitude, harmless to QPSK, fatal to 16QAM).
            for w2 in pil.windows(2) {
                let ((k0, h0), (k1, h1)) = (w2[0], w2[1]);
                for k in k0..k1 {
                    let a = (k - k0) as f32 / (k1 - k0) as f32;
                    h[k] += (h0 * (1.0 - a) + h1 * a) / N_P2 as f32;
                }
            }
            if let Some(&(kl, hl)) = pil.last() {
                h[kl] += hl / N_P2 as f32;
            }
        }
        // Equalize; each symbol's common phase from its pilots.
        let mut syms: Vec<Vec<Complex32>> = Vec::with_capacity(nsym);
        for j in 0..nsym {
            let mut c = Complex32::default();
            for (k, v) in plans[j].iter().enumerate() {
                if let Some(z) = v.filter(|z| z.norm() > 0.0) {
                    c += carriers[j][k] / h[k] * z.conj();
                }
            }
            let rot = if c.norm() > 0.0 { c.conj() / c.norm() } else { Complex32::new(1.0, 0.0) };
            syms.push(plans[j].iter().enumerate().filter(|(_, v)| v.is_none()).map(|(k, _)| carriers[j][k] / h[k] * rot).collect());
        }
        if std::env::var_os("T2DEBUG").is_some() && report.frames == 0 && p.constellation == Constellation::Qam16 {
            let a = 1.0 / 10f32.sqrt();
            let q = |x: f32| {
                let l = [-3.0 * a, -a, a, 3.0 * a];
                *l.iter().min_by(|u, v| (x - **u).abs().partial_cmp(&(x - **v).abs()).unwrap()).unwrap()
            };
            let mut line = String::new();
            for (j, sy) in syms.iter().enumerate() {
                let e: f32 = sy.iter().map(|z| (z - Complex32::new(q(z.re), q(z.im))).norm_sqr()).sum::<f32>() / sy.len() as f32;
                let pw: f32 = sy.iter().map(|z| z.norm_sqr()).sum::<f32>() / sy.len() as f32;
                if j < 10 || j % 20 == 0 || j + 2 >= syms.len() {
                    line += &format!(" {j}:{:.1}", 10.0 * (pw / e.max(1e-9)).log10());
                }
            }
            eprintln!("DD MER per symbol:{line}");
        }
        let cells = fi.unframe(&syms);
        let (pre, _post, data) = fm.unmap(&cells);
        let (mut sig, mut err) = (0f32, 0f32);
        for (z, &r) in pre.iter().zip(&pre_ref) {
            sig += r * r;
            err += (z - Complex32::new(r, 0.0)).norm_sqr();
        }
        report.mer_db.push(10.0 * (sig / err.max(1e-12)).log10());
        let blocks = ci.deinterleave(&data, p.fec_blocks);
        // QPSK LLRs (positive = 0), noise from the L1-pre error.
        let sigma2 = (err / pre.len() as f32).max(1e-6);
        // Rotated constellations: word j's I is in cell j, its Q in cell
        // j + 1 (cyclically in the block); rotate back.
        let angle: f32 = match p.constellation {
            Constellation::Qpsk => 29.0,
            Constellation::Qam16 => 16.8,
        };
        let derot = Complex32::from_polar(1.0, -angle.to_radians());
        if std::env::var_os("T2DEBUG").is_some() {
            let b = &blocks[0];
            let pw = b.iter().map(|z| z.norm_sqr()).sum::<f32>() / b.len() as f32;
            let mre = b.iter().map(|z| z.re.abs()).sum::<f32>() / b.len() as f32;
            eprintln!("block 0: mean power {pw:.3}, mean |re| {mre:.3}, first cells {:?}", &b[..4]);
        }
        for blk in &blocks {
            let n = blk.len();
            let z = |j: usize| if p.rotation { Complex32::new(blk[j].re, blk[(j + 1) % n].im) * derot } else { blk[j] };
            let cell_llr: Vec<f32> = match p.constellation {
                Constellation::Qpsk => (0..n)
                    .flat_map(|j| {
                        let s = 2.0 * std::f32::consts::FRAC_1_SQRT_2 * 2.0 / sigma2;
                        let z = z(j);
                        [z.re * s, z.im * s]
                    })
                    .collect(),
                Constellation::Qam16 => (0..n)
                    .flat_map(|j| {
                        // Word bits 3, 2: signs of I, Q (0 positive); 1, 0:
                        // outer (0) or inner level. Max-log, per axis.
                        let a = 1.0 / 10f32.sqrt();
                        let s = 4.0 * a / sigma2;
                        let z = z(j);
                        [z.re * s, z.im * s, (z.re.abs() - 2.0 * a) * s, (z.im.abs() - 2.0 * a) * s]
                    })
                    .collect(),
            };
            let llr = bi.deinterleave_llr(&cell_llr);
            if let Some(path) = std::env::var_os("T2DUMPC") {
                if report.frames == 0 && std::ptr::eq(blk, &blocks[0]) {
                    let b: Vec<u8> = blk.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
                    std::fs::write(path, b).unwrap();
                }
            }
            if let Some(path) = std::env::var_os("T2DUMP") {
                if report.frames == 0 && std::ptr::eq(blk, &blocks[0]) {
                    let b: Vec<u8> = llr.iter().map(|&l| (l < 0.0) as u8).collect();
                    std::fs::write(path, b).unwrap();
                }
            }
            fec.frame(&llr, &mut stats, &mut report.packets);
        }
        report.frames += 1;
        start += frame;
    }
    report.ldpc_fail = stats.ldpc_fail;
    report
}

#[cfg(test)]
mod tests {
    use super::super::Modulator;
    use super::*;

    /// Modulator -> offset, noise -> receiver: every packet back in order
    /// (plain and rotated QPSK).
    #[test]
    fn t2_loopback() {
        for rotation in [false, true] {
            let mut p = Params::amateur();
            p.rotation = rotation;
            loopback(p, 15.0);
        }
    }

    #[test]
    fn t2_loopback_16qam() {
        for rotation in [false, true] {
            let mut p = Params::amateur();
            p.constellation = super::super::Constellation::Qam16;
            p.fec_blocks = 18;
            p.rotation = rotation;
            loopback(p, 22.0);
        }
    }

    fn loopback(p: Params, snr_db: f32) {
        let mut m = Modulator::new(p);
        let mut n = 0u32;
        let mut next = || {
            let mut pkt = [0u8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1] = 0x01;
            pkt[4..8].copy_from_slice(&n.to_be_bytes());
            for (i, b) in pkt[8..].iter_mut().enumerate() {
                *b = (n as usize * 7 + i) as u8;
            }
            n += 1;
            pkt
        };
        let mut x = vec![Complex32::default(); 5000];
        for _ in 0..3 {
            m.frame(&mut next, &mut x);
        }
        let fs = 131e6 / 71.0;
        // T2CLEAN=1: no offset, no noise (what is left is the receiver's).
        let clean = std::env::var_os("T2CLEAN").is_some();
        let off = if clean { 0.0 } else { 3000.0 };
        let snr_db = if clean { 200.0 } else { snr_db };
        let sigma = (10f32.powf(-snr_db / 10.0) / 2.0).sqrt();
        let mut seed = 5u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        for (k, z) in x.iter_mut().enumerate() {
            let ph = std::f64::consts::TAU * off * k as f64 / fs;
            *z = *z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g());
        }
        let r = receive(p, &x, fs);
        eprintln!("frames {}, packets {}, MER {:?}, freq {:.0} Hz, LDPC failures {}", r.frames, r.packets.len(), r.mer_db, r.freq_hz, r.ldpc_fail);
        if let Some(path) = std::env::var_os("T2DUMP") {
            // The first FEC block's hard decisions against what was sent.
            let mut m2 = Modulator::new(p);
            let mut n2 = 0u32;
            let mut next2 = || {
                let mut pkt = [0u8; TS_LEN];
                pkt[0] = 0x47;
                pkt[1] = 0x01;
                pkt[4..8].copy_from_slice(&n2.to_be_bytes());
                for (i, b) in pkt[8..].iter_mut().enumerate() {
                    *b = (n2 as usize * 7 + i) as u8;
                }
                n2 += 1;
                pkt
            };
            let cw = m2.codeword(&mut next2);
            if let Some(pc) = std::env::var_os("T2DUMPC") {
                let tx = super::super::map_cells(&p, &cw);
                let rx: Vec<Complex32> = std::fs::read(pc).unwrap().chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
                let bad: Vec<usize> = (0..tx.len()).filter(|&i| (tx[i] - rx[i]).norm() > 1e-3).collect();
                eprintln!("block 0 cells: {} of {} differ; first {:?}; tx {:?} rx {:?}", bad.len(), tx.len(), &bad[..bad.len().min(6)], bad.first().map(|&i| tx[i]), bad.first().map(|&i| rx[i]));
            }
            let got = std::fs::read(path).unwrap();
            let bad: Vec<usize> = (0..cw.len()).filter(|&i| got[i] != cw[i]).collect();
            eprintln!("block 0: {} of {} bits wrong; first {:?}; info part {}, parity part {}", bad.len(), cw.len(), &bad[..bad.len().min(8)], bad.iter().filter(|&&i| i < 32400).count(), bad.iter().filter(|&&i| i >= 32400).count());
        }
        assert!(r.frames >= 2 && r.ldpc_fail == 0, "{} frames, {} failures", r.frames, r.ldpc_fail);
        let data: Vec<_> = r.packets.iter().filter(|p| p[1] == 0x01).collect();
        assert!(data.len() > 150 * p.fec_blocks / 9, "{} packets", data.len());
        let f0 = u32::from_be_bytes(data[0][4..8].try_into().unwrap());
        for (i, pkt) in data.iter().enumerate() {
            assert_eq!(u32::from_be_bytes(pkt[4..8].try_into().unwrap()), f0 + i as u32, "packet {i}");
        }
    }
}

/// Resample `x` from `fs_in` to `fs_out` (below it): windowed sinc
/// (Kaiser 8, 48 taps at the input rate), cut off at 0.45 of `fs_out`.
pub fn resample(x: &[Complex32], fs_in: f64, fs_out: f64) -> Vec<Complex32> {
    const TAPS: usize = 48;
    const PH: usize = 512;
    let fc = 0.45 * fs_out / fs_in; // cycles a sample at the input rate
    let i0 = |v: f64| {
        let (mut s, mut t) = (1.0, 1.0);
        for k in 1..50 {
            t *= (v / 2.0) / k as f64;
            s += t * t;
        }
        s
    };
    let beta = 8.0;
    // table[p][k]: tap k for fractional delay p / PH
    let table: Vec<Vec<f32>> = (0..=PH)
        .map(|p| {
            let frac = p as f64 / PH as f64;
            let h: Vec<f64> = (0..TAPS)
                .map(|k| {
                    let t = k as f64 - (TAPS / 2) as f64 + 1.0 - frac;
                    let s = if t.abs() < 1e-12 { 2.0 * fc } else { (std::f64::consts::TAU * fc * t).sin() / (std::f64::consts::PI * t) };
                    let r = t / (TAPS / 2) as f64;
                    s * if r.abs() <= 1.0 { i0(beta * (1.0 - r * r).sqrt()) / i0(beta) } else { 0.0 }
                })
                .collect();
            let g: f64 = h.iter().sum();
            h.iter().map(|v| (v / g) as f32).collect()
        })
        .collect();
    let ratio = fs_in / fs_out;
    let n_out = ((x.len() - TAPS) as f64 / ratio) as usize;
    (0..n_out)
        .map(|m| {
            let t = m as f64 * ratio;
            let i = t.floor() as usize;
            let p = ((t - i as f64) * PH as f64).round() as usize;
            let h = &table[p];
            let mut acc = Complex32::default();
            for k in 0..TAPS {
                acc += x[i + k] * h[k];
            }
            acc
        })
        .collect()
}

/// A board recording (`trxd --capture-iq`, cf32 at 3.072 MS/s) of this
/// modulator's T2 through the receiver:
/// `T2CAP=<file> cargo test --release t2_capture -- --ignored --nocapture`
#[test]
#[ignore]
fn t2_capture() {
    let path = std::env::var("T2CAP").expect("T2CAP=<cf32 at 3.072 MS/s>");
    let raw = std::fs::read(path).unwrap();
    let x: Vec<Complex32> = raw.chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
    let fs = 131e6 / 71.0;
    let mut y = resample(&x, 3_072_000.0, fs);
    // T2CONJ=1: the spectrum mirrored (I/Q the other way round somewhere).
    if std::env::var_os("T2CONJ").is_some() {
        y.iter_mut().for_each(|z| *z = z.conj());
    }
    let mut p = Params::amateur();
    // T2ROT=1: the transmitter rotated its constellation (trxd's default).
    p.rotation = std::env::var_os("T2ROT").is_some();
    let r = receive(p, &y, fs);
    let mut pids = std::collections::BTreeMap::new();
    let mut cc_err = 0;
    let mut last_cc = std::collections::HashMap::new();
    for pkt in &r.packets {
        let pid = ((pkt[1] as u16 & 0x1F) << 8) | pkt[2] as u16;
        *pids.entry(pid).or_insert(0) += 1;
        if pid != 0x1FFF && pkt[3] & 0x10 != 0 {
            let cc = pkt[3] & 15;
            if let Some(&prev) = last_cc.get(&pid) {
                if cc != (prev + 1) & 15 {
                    cc_err += 1;
                }
            }
            last_cc.insert(pid, cc);
        }
    }
    eprintln!("{} samples: {} frames, {} packets, LDPC failures {}, carrier {:.0} Hz, L1-pre MER {:?} dB", x.len(), r.frames, r.packets.len(), r.ldpc_fail, r.freq_hz, r.mer_db);
    eprintln!("PIDs {pids:?}, continuity errors {cc_err}");
}

/// Timing and pilot coherence per symbol of a capture:
/// `T2CAP=<file> cargo test --release t2_diag -- --ignored --nocapture`
#[test]
#[ignore]
fn t2_diag() {
    let path = std::env::var("T2CAP").expect("T2CAP");
    let raw = std::fs::read(path).unwrap();
    let x: Vec<Complex32> = raw.chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
    let fs = 131e6 / 71.0;
    let y = resample(&x, 3_072_000.0, fs);
    let p = Params::amateur();
    let ofdm = Ofdm::new(p);
    let frame = p.frame_samples();
    let (start, coarse) = find_p1(&y, ofdm.p1(), frame, fs).unwrap();
    let pw: f32 = y.iter().take(200_000).map(|z| z.norm_sqr()).sum::<f32>() / 200_000.0;
    eprintln!("P1 at {start}, coarse {coarse:.0} Hz, mean power {pw:.3e}");
    let fft = FftPlanner::new().plan_fft_forward(FFT);
    let gi = p.guard.samples();
    let w = -std::f64::consts::TAU * coarse / fs;
    for f in 0..3 {
        for j in [0usize, 1, 7, 8, 9, 50, 100, 150, 197] {
            let s = start + f * frame + 2048 + j * (FFT + gi) + gi;
            if s + FFT > y.len() {
                break;
            }
            let mut buf: Vec<Complex32> = (0..FFT).map(|n| { let t = (s + n) as f64 * w; y[s + n] * Complex32::new(t.cos() as f32, t.sin() as f32) }).collect();
            fft.process(&mut buf);
            let plan = ofdm.plan(j);
            let r: Vec<(usize, Complex32)> = plan.iter().enumerate().filter_map(|(k, v)| v.filter(|z| z.norm() > 0.0).map(|z| (k, buf[ofdm.bin(k)] / z))).collect();
            // phase step between neighbouring pilots, per carrier
            let mut d = Complex32::default();
            for w2 in r.windows(2) {
                d += w2[1].1 * w2[0].1.conj() / (w2[1].0 - w2[0].0) as f32;
            }
            let coh = r.iter().map(|x| x.1).sum::<Complex32>().norm() / r.iter().map(|x| x.1.norm()).sum::<f32>();
            eprintln!("frame {f} sym {j:3}: timing {:+.2} samples, pilot coherence {coh:.2}, |pilots| {:.3e}", -d.arg() as f64 / (std::f64::consts::TAU / FFT as f64), r.iter().map(|x| x.1.norm()).sum::<f32>() / r.len() as f32);
        }
    }
}

/// The on-air sample path in simulation: the modulator at x1.25, the
/// FPGA's resampler (its integer model and low-pass table) to 3.072 MS/s,
/// then this receiver's resampler back and the receiver.
#[test]
fn t2_through_the_fpga_resampler() {
    let p = Params::amateur();
    let mut m = super::Modulator::oversampled(p, 5);
    let mut n = 0u32;
    let mut next = || {
        let mut pkt = [0u8; TS_LEN];
        pkt[0] = 0x47;
        pkt[1] = 0x01;
        pkt[4..8].copy_from_slice(&n.to_be_bytes());
        n += 1;
        pkt
    };
    let mut x = vec![Complex32::default(); 3000];
    for _ in 0..3 {
        m.frame(&mut next, &mut x);
    }
    // 8-bit samples as sent, then the interpolator's integer model.
    let fs_in = 131e6 / 71.0 * 1.25;
    let q = |v: f32| (((v * 48.0) as i32).clamp(-127, 127) * 256) as i64;
    let coeffs: Vec<i64> = crate::dvbs2::fpga_tx::lowpass_table().iter().map(|&c| c as i64).collect();
    let step = crate::dvbs2::fpga_tx::step(fs_in) as u64;
    let (span, ph_bits) = (16usize, 8u32);
    let mut hist = vec![(0i64, 0i64); span];
    let (mut acc, mut k) = (0u64, 0usize);
    let nout = ((x.len() - 20) as f64 * 3_072_000.0 / fs_in) as usize;
    let mut dac = Vec::with_capacity(nout);
    for _ in 0..nout {
        acc += step;
        if acc >= 1 << 32 {
            acc -= 1 << 32;
            hist.rotate_right(1);
            hist[0] = (q(x[k].re), q(x[k].im));
            k += 1;
        }
        let ph = (acc >> (32 - ph_bits)) as usize;
        let (mut sr, mut si) = (1i64 << 16, 1i64 << 16);
        for (t, &(re, im)) in hist.iter().enumerate() {
            let h = coeffs[t * 256 + ph];
            sr += h * re;
            si += h * im;
        }
        dac.push(Complex32::new(((sr >> 17).clamp(-32768, 32767)) as f32 / 32768.0, ((si >> 17).clamp(-32768, 32767)) as f32 / 32768.0));
    }
    let fs = 131e6 / 71.0;
    let y = resample(&dac, 3_072_000.0, fs);
    let r = receive(p, &y, fs);
    eprintln!("frames {}, packets {}, MER {:?}, LDPC failures {}", r.frames, r.packets.len(), r.mer_db, r.ldpc_fail);
    assert!(r.frames >= 2 && r.ldpc_fail == 0 && r.mer_db.iter().all(|&m| m > 20.0));
}

#[test]
#[ignore]
fn t2_16qam_debug() {
    use super::{codes, Constellation, Modulator, Palette};
    let mut p = Params::amateur();
    p.constellation = Constellation::Qam16;
    p.fec_blocks = 18;
    let mut m = Modulator::new(p);
    let mut next = || [0x47u8; TS_LEN];
    let cw = m.codeword(&mut next);
    let bi = BitInterleaver::new(&p);
    let words = bi.words(&cw);
    let pal = Palette::new(&p);
    let cells: Vec<Complex32> = codes(&p, &words).iter().map(|&c| pal.get(c)).collect();
    // demap noise-free cells the receiver's way
    let a = 1.0 / 10f32.sqrt();
    let cell_llr: Vec<f32> = cells.iter().flat_map(|z| [z.re, z.im, z.re.abs() - 2.0 * a, z.im.abs() - 2.0 * a]).collect();
    // hard word bits vs words
    let mut bad_words = 0;
    for (j, w) in words.iter().enumerate() {
        let hb: u8 = (0..4).fold(0, |acc, i| (acc << 1) | (cell_llr[4 * j + i] < 0.0) as u8);
        if hb != *w {
            bad_words += 1;
        }
    }
    let llr = bi.deinterleave_llr(&cell_llr);
    let bad_bits = llr.iter().zip(&cw).filter(|(l, b)| ((**l < 0.0) as u8) != **b).count();
    eprintln!("words wrong {bad_words} of {}, codeword bits wrong {bad_bits} of {}", words.len(), cw.len());
}
