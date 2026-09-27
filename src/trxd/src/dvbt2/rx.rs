//! A DVB-T2 receiver for this modulator's profile, offline: the streaming
//! demodulator ([`super::stream::Demod`]) and the DVB-S2 decoder chain over
//! a whole recording, to check transmissions end to end. Samples at the
//! elementary rate in; TS packets out.

use num_complex::Complex32;

use super::Params;
use crate::dvbs2::TS_LEN;

pub struct Report {
    pub frames: usize,
    pub packets: Vec<[u8; TS_LEN]>,
    /// MER of the L1-pre cells (BPSK, known), dB, per frame.
    pub mer_db: Vec<f32>,
    pub freq_hz: f32,
    pub ldpc_fail: u64,
}

pub fn receive(p: Params, x: &[Complex32], fs: f64) -> Report {
    let mut d = super::stream::Demod::new(p, fs);
    let mut fec = crate::dvbs2::rx::Fec::new(crate::dvbs2::FrameSpec::long(match p.rate {
        crate::dvbs2::ldpc_fpga::LongRate::R1_2 => crate::dvbs2::fpga_tx::LongMode::Qpsk12,
        crate::dvbs2::ldpc_fpga::LongRate::R3_4 => crate::dvbs2::fpga_tx::LongMode::Qpsk34,
    }));
    let mut stats = crate::dvbs2::rx::Stats::default();
    let mut report = Report { frames: 0, packets: Vec::new(), mer_db: Vec::new(), freq_hz: 0.0, ldpc_fail: 0 };
    let mut blocks = Vec::new();
    // In pieces, as from the ring: one MER per frame.
    for chunk in x.chunks(20_000) {
        let before = d.stats.frames;
        d.push(chunk, &mut blocks);
        if d.stats.frames != before {
            report.mer_db.push(d.stats.mer_db);
        }
        for llr in blocks.drain(..) {
            fec.frame(&llr, &mut stats, &mut report.packets);
        }
    }
    report.frames = d.stats.frames as usize;
    report.freq_hz = d.stats.freq_hz;
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

    /// Joining mid-frame, a sample clock 20 ppm off (the timing drifts
    /// about 9 samples a frame) and a carrier 2 kHz off: every frame after
    /// the first P1 found, P1 tracked frame to frame.
    #[test]
    fn t2_drift() {
        let p = Params::amateur();
        let mut m = Modulator::new(p);
        let mut n = 0u32;
        let mut next = || {
            let mut pkt = [0u8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1] = 0x01;
            pkt[4..8].copy_from_slice(&n.to_be_bytes());
            n += 1;
            pkt
        };
        let mut x = Vec::new();
        for _ in 0..6 {
            m.frame(&mut next, &mut x);
        }
        let fs = 131e6 / 71.0;
        let y = super::resample(&x[150_000..], fs, fs / (1.0 + 20e-6));
        let y: Vec<Complex32> = y
            .iter()
            .enumerate()
            .map(|(k, z)| {
                let ph = std::f64::consts::TAU * 2000.0 * k as f64 / fs;
                z * Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let r = receive(p, &y, fs);
        eprintln!("frames {}, packets {}, MER {:?}, freq {:.0} Hz, LDPC failures {}", r.frames, r.packets.len(), r.mer_db, r.freq_hz, r.ldpc_fail);
        // The same 25 kHz further off, told to the receiver (the LO's offset).
        let y2: Vec<Complex32> = y
            .iter()
            .enumerate()
            .map(|(k, z)| {
                let ph = std::f64::consts::TAU * 25_000.0 * k as f64 / fs;
                z * Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut d = super::super::stream::Demod::new(p, fs);
        d.set_center(25_000.0);
        let mut blocks = Vec::new();
        for c in y2.chunks(9000) {
            d.push(c, &mut blocks);
        }
        eprintln!("with 25 kHz told: frames {}, freq {:.0} Hz, MER {:.1}", d.stats.frames, d.stats.freq_hz, d.stats.mer_db);
        assert!(d.stats.frames >= 4 && d.stats.mer_db > 20.0);
        assert!(r.frames >= 4 && r.ldpc_fail == 0 && r.mer_db.iter().all(|&m| m > 20.0));
        assert!((r.freq_hz - 2000.0).abs() < 50.0);
    }

    /// Demodulator time per stage for a frame (run it on the board:
    /// `trxd-test t2_demod_speed --ignored --nocapture`).
    #[test]
    #[ignore]
    fn t2_demod_speed() {
        let p = Params::amateur();
        let mut m = Modulator::new(p);
        let mut next = || [0x47u8; TS_LEN];
        let mut x = Vec::new();
        for _ in 0..6 {
            m.frame(&mut next, &mut x);
        }
        let fs = 131e6 / 71.0;
        let mut d = super::super::stream::Demod::new(p, fs);
        let mut blocks = Vec::new();
        let t = std::time::Instant::now();
        for c in x.chunks(9225) {
            d.push(c, &mut blocks);
        }
        let frames = d.stats.frames as f64;
        eprintln!("{} frames in {:.3} s ({:.1} ms a frame; real time is {:.0} ms)", frames, t.elapsed().as_secs_f64(), 1e3 * t.elapsed().as_secs_f64() / frames, 1e3 * p.frame_samples() as f64 / fs);
        for (n, v) in super::super::stream::PROF_NAMES.iter().zip(d.prof) {
            eprintln!("  {n:>12}: {:.1} ms a frame", 1e3 * v / frames);
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

/// The on-air sample path in simulation: the modulator at x1.25, the
/// FPGA's resampler (its integer model and low-pass table) to 3.072 MS/s,
/// then the 12-bit ADC, the FPGA's T2 receive resampler (its integer model)
/// and the receiver.
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
    // Received: the 12-bit ADC and the FPGA's T2 resampler (integer model).
    let fs = 131e6 / 71.0;
    let step = super::resamp::step(3_072_000.0, fs);
    let mut rs = super::resamp::Resampler::new(super::resamp::t2_table(3_072_000.0, fs), step);
    let y: Vec<Complex32> = rs.process(&super::resamp::adc12(&dac, 1.0)).iter().map(|v| Complex32::new(v[0] as f32 / 32768.0, v[1] as f32 / 32768.0)).collect();
    let r = receive(p, &y, super::resamp::rate_out(3_072_000.0, step));
    eprintln!("frames {}, packets {}, MER {:?}, LDPC failures {}", r.frames, r.packets.len(), r.mer_db, r.ldpc_fail);
    assert!(r.frames >= 2 && r.ldpc_fail == 0 && r.mer_db.iter().all(|&m| m > 20.0));
}

#[test]
#[ignore]
fn t2_16qam_debug() {
    use super::{codes, BitInterleaver, Constellation, Modulator, Palette};
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
