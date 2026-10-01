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
    receive_as(p, x, fs, false)
}

/// `cells`: the blocks go out as cells ([`super::stream::T2Block::Cells`],
/// what the FPGA's LDPC engine takes; their LLRs made here the same way).
pub fn receive_as(p: Params, x: &[Complex32], fs: f64, cells: bool) -> Report {
    let mut d = super::stream::Demod::new(p, fs);
    d.cells_out = cells;
    d.cells16_out = cells;
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
            fec.frame_q(&llr.llrs(), &mut stats, &mut report.packets);
        }
    }
    if let Some(se) = &d.sym_err {
        let v: Vec<String> = se.chunks(16).enumerate().filter(|(_, c)| c.iter().any(|x| x.1 > 0.0)).map(|(i, c)| {
            let (e, n) = c.iter().fold((0.0, 0.0), |a, x| (a.0 + x.0, a.1 + x.1));
            format!("{}-{}: {:.1} dB", i * 16, i * 16 + 15, -10.0 * (e / n).log10())
        }).collect();
        eprintln!("software receiver, pilot MER by symbol index: {}", v.join(", "));
    }
    if let Some(ce) = &d.car_err {
        let v: Vec<String> = ce.iter().enumerate().map(|(i, x)| format!("{}: {:.1}", i * 64, -10.0 * (x.0 / x.1.max(1.0)).log10())).collect();
        eprintln!("software receiver, pilot MER by carrier (dB): {}", v.join(", "));
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
            loopback(p, 15.0, false);
        }
    }

    #[test]
    fn t2_loopback_16qam() {
        for rotation in [false, true] {
            let mut p = Params::amateur();
            p.constellation = super::super::Constellation::Qam16;
            p.fec_blocks = 18;
            p.rotation = rotation;
            loopback(p, 22.0, false);
        }
    }

    /// The same with the blocks as cells (QPSK and 16QAM, rotated): their
    /// LLRs from [`super::super::stream::cell_llrs`], the FPGA engine's model.
    #[test]
    fn t2_loopback_cells() {
        for qam16 in [false, true] {
            let mut p = Params::amateur();
            p.rotation = true;
            if qam16 {
                p.constellation = super::super::Constellation::Qam16;
                p.fec_blocks = 18;
            }
            loopback(p, if qam16 { 22.0 } else { 15.0 }, true);
        }
    }

    /// Every mode: a frame within T2's 250 ms, the FEC blocks within its
    /// cells; the 1.35 MHz one (145 data symbols) through the loop.
    #[test]
    fn t2_modes_fit_and_short_frames_decode() {
        for bw in ["1.7", "2.0", "1.35"] {
            for c in ["QPSK", "16QAM"] {
                for r in ["1/2", "3/4"] {
                    let m = super::super::tx::Mode::parse(&format!("T2-{bw}-{c}-{r}")).unwrap();
                    let frame_s = m.p.frame_samples() as f64 / m.fs();
                    assert!(frame_s <= 0.250, "T2-{bw}-{c}-{r}: {frame_s} s");
                    // data cells: the frame's, less L1 in P2 and the frame
                    // closing symbol's unused cells (as FrameMapper counts)
                    let (_, n_fc, c_fc) = m.p.data_cells();
                    let data = m.p.frame_cells() - (1840 + super::super::l1::post_cells() + n_fc - c_fc);
                    assert!(m.p.fec_blocks * m.p.cells() <= data, "T2-{bw}-{c}-{r}: {} blocks in {data} cells", m.p.fec_blocks);
                }
            }
        }
        let m = super::super::tx::Mode::parse("T2-1.35-QPSK-1/2").unwrap();
        loopback(m.p, 15.0, false);
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
        // every frame's L1-pre decoded and as configured
        assert_eq!((d.stats.l1_ok, d.stats.l1_mismatch), (d.stats.frames, 0), "{:?}", d.stats);
        assert!(r.frames >= 4 && r.ldpc_fail == 0 && r.mer_db.iter().all(|&m| m > 20.0));
        assert!((r.freq_hz - 2000.0).abs() < 50.0);
    }

    /// Through the FPGA front end's model, closed loop: raw samples while
    /// searching, then the schedule and NCO the receiver sets (applied
    /// 10 ms late, as through the ring), FFTs of the model, P1 and
    /// frequency tracked from the raw windows. 20 ppm sample clock, 2 kHz
    /// + the LO's 25 kHz off, noise; every packet after acquisition.
    #[test]
    fn t2_through_the_front_end() {
        use super::super::fe::{model::Model, Ctl};
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
        for _ in 0..8 {
            m.frame(&mut next, &mut x);
        }
        let fs = 131e6 / 71.0;
        let y = super::resample(&x[100_000..], fs, fs / (1.0 + 20e-6));
        let mut seed = 7u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.05
        };
        let q = |v: f32| (v * 3000.0).round().clamp(-32768.0, 32767.0) as i16;
        let samples: Vec<[i16; 2]> = y
            .iter()
            .enumerate()
            .map(|(k, z)| {
                let ph = std::f64::consts::TAU * 27_000.0 * k as f64 / fs;
                let v = z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(g(), g());
                [q(v.re), q(v.im)]
            })
            .collect();
        let bins: Vec<usize> = (0..1705).map(|k| super::super::ofdm::Ofdm::new(p).bin(k)).collect();
        let mut fe = Model::new(p.frame_samples() as u64, p.symbols() as u64, p.guard.samples() as u64, &bins);
        // T2EQ=0: without the FPGA's equalizer (in the model)
        if std::env::var("T2EQ").map_or(true, |v| v != "0") {
            let (dx, dy) = p.pilots.dxdy();
            fe.enable_eq(super::super::N_P2, dx, dy, p.symbols() - 1);
        }
        let mut d = super::super::stream::Demod::new(p, fs);
        d.set_center(25_000.0);
        let mut fec = crate::dvbs2::rx::Fec::new(crate::dvbs2::FrameSpec::long(crate::dvbs2::fpga_tx::LongMode::Qpsk12));
        let mut stats = crate::dvbs2::rx::Stats::default();
        let (mut words, mut blocks, mut ctl, mut packets) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut late: Vec<(usize, Ctl)> = Vec::new();
        let chunk = 9225; // 5 ms
        for (i, c) in samples.chunks(chunk).enumerate() {
            // Commands reach the front end two chunks (10 ms) later.
            for (_, cmd) in late.iter().filter(|(at, _)| *at == i) {
                fe.apply(cmd.clone());
            }
            words.clear();
            fe.run(c, &mut words);
            d.push_words(&words, &mut blocks, &mut ctl);
            late.extend(ctl.drain(..).map(|cmd| (i + 2, cmd)));
            for llr in blocks.drain(..) {
                fec.frame_q(&llr.llrs(), &mut stats, &mut packets);
            }
        }
        eprintln!("frames {}, blocks {}, packets {}, MER {:.1} dB, freq {:.0} Hz, P1 missed {}, LDPC failures {}", d.stats.frames, d.stats.blocks, packets.len(), d.stats.mer_db, d.stats.freq_hz, d.stats.p1_missed, stats.ldpc_fail);
        assert!(d.stats.frames >= 4, "{} frames", d.stats.frames);
        assert_eq!(stats.ldpc_fail, 0);
        assert_eq!((d.stats.l1_ok, d.stats.l1_mismatch), (d.stats.frames, 0), "{:?}", d.stats);
        assert!((d.stats.freq_hz - 2000.0).abs() < 30.0);
        let data: Vec<_> = packets.iter().filter(|p| p[1] == 0x01).collect();
        let f0 = u32::from_be_bytes(data[0][4..8].try_into().unwrap());
        for (i, pkt) in data.iter().enumerate() {
            assert_eq!(u32::from_be_bytes(pkt[4..8].try_into().unwrap()), f0 + i as u32, "packet {i}");
        }
    }

    /// A board's front-end words (`touch /tmp/t2-words` on it) through the
    /// receiver: `T2WORDS=<file> cargo test --release t2_words -- --ignored
    /// --nocapture` (T2CENTER=<Hz>: the LO offset trxd told it).
    #[test]
    #[ignore]
    fn t2_words() {
        let raw = std::fs::read(std::env::var("T2WORDS").expect("T2WORDS=<u32 file>")).unwrap();
        let words: Vec<u32> = raw.chunks_exact(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect();
        let mut p = Params::amateur();
        p.rotation = std::env::var_os("T2PLAIN").is_none();
        let fs = 131e6 / 71.0;
        let mut d = super::super::stream::Demod::new(p, fs);
        d.set_center(std::env::var("T2CENTER").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0));
        let mut fec = crate::dvbs2::rx::Fec::new(crate::dvbs2::FrameSpec::long(crate::dvbs2::fpga_tx::LongMode::Qpsk12));
        let mut stats = crate::dvbs2::rx::Stats::default();
        let (mut blocks, mut ctl, mut packets) = (Vec::new(), Vec::new(), Vec::new());
        let mut mers = Vec::new();
        for c in words.chunks(20_000) {
            let f0 = d.stats.frames;
            d.push_words(c, &mut blocks, &mut ctl);
            if d.stats.frames != f0 {
                mers.push((d.stats.mer_db * 10.0).round() / 10.0);
            }
            for cmd in ctl.drain(..) {
                eprintln!("ctl {cmd:?}");
            }
            for llr in blocks.drain(..) {
                fec.frame_q(&llr.llrs(), &mut stats, &mut packets);
            }
        }
        if let Some(se) = &d.sym_err {
            let v: Vec<String> = se.chunks(16).enumerate().filter(|(_, c)| c.iter().any(|x| x.1 > 0.0)).map(|(i, c)| {
                let (e, n) = c.iter().fold((0.0, 0.0), |a, x| (a.0 + x.0, a.1 + x.1));
                format!("{}-{}: {:.1} dB", i * 16, i * 16 + 15, -10.0 * (e / n).log10())
            }).collect();
            eprintln!("pilot MER by symbol index: {}", v.join(", "));
        }
        eprintln!("BCH: fixed {} bits, failed on {} converged frames", stats.bch_fixed, stats.bch_fail);
        eprintln!("{} words: frames {}, blocks {}, packets {}, LDPC failures {}, freq {:.0} Hz, P1 missed {}, MER {:?}", words.len(), d.stats.frames, d.stats.blocks, packets.len(), stats.ldpc_fail, d.stats.freq_hz, d.stats.p1_missed, mers);
        if std::env::var_os("T2DBG").is_none() {
            return;
        }
        // Raw runs and FFTs, independent of the receiver.
        use super::super::fe::{decode, Word};
        let (mut runs, mut ffts): (Vec<(u64, Vec<Complex32>)>, Vec<(u8, u32, Vec<Complex32>)>) = (Vec::new(), Vec::new());
        let mut next: Option<u64> = None;
        for &w in &words {
            match decode(w) {
                Word::Gap => next = None,
                Word::RawHeader(c) => {
                    if next != Some(c as u64) {
                        runs.push((c as u64, Vec::new()));
                    }
                    next = Some(c as u64);
                }
                Word::Raw(v) => {
                    if let Some(r) = runs.last_mut() {
                        r.1.push(Complex32::new(v[0] as f32, v[1] as f32) / 32768.0);
                        next = next.map(|n| n + 1);
                    }
                }
                Word::CarHeader { j, f21, .. } => ffts.push((j, f21, Vec::new())),
                Word::Car(v) => {
                    if let Some(f) = ffts.last_mut() {
                        f.2.push(Complex32::new(v[0] as f32, v[1] as f32));
                    }
                }
            }
        }
        let lens: Vec<usize> = runs.iter().map(|r| r.1.len()).take(12).collect();
        eprintln!("{} raw runs (first lengths {:?}), {} FFTs (first {:?})", runs.len(), lens, ffts.len(), ffts.iter().take(12).map(|f| (f.0, f.1, f.2.len())).collect::<Vec<_>>());
        let ofdm = super::super::ofdm::Ofdm::new(p);
        let p1 = ofdm.p1();
        if let Ok(path) = std::env::var("T2P1OUT") {
            let b: Vec<u8> = p1.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
            std::fs::write(path, b).unwrap();
        }
        let e1: f32 = p1.iter().map(|z| z.norm_sqr()).sum();
        for (c, r) in runs.iter().filter(|r| r.1.len() >= 2048 + 128).take(6) {
            let r: Vec<Complex32> = r.iter().take(600_000).copied().collect();
            let rc: Vec<Complex32> = r.iter().map(|z| z.conj()).collect();
            for (name, x) in [("as is", &r), ("conj", &rc)] {
                if let Some((s, f, q)) = super::super::stream::find_p1_in(x, p1, e1, 0, x.len() - 2048 + 1, fs) {
                    eprintln!("P1 run at {c} len {} ({name}): best offset {s}, coarse {f:.0} Hz, q {q:.2}", x.len());
                }
            }
        }
        // The software receiver on the longest raw run (`/tmp/t2-rawall`):
        // the same signal without the front end's FFTs.
        if let Some((c0, big)) = runs.iter().max_by_key(|r| r.1.len()).filter(|r| r.1.len() > 1_500_000) {
            let r = receive(p, big, fs);
            eprintln!("software receiver on the raw run at {c0} ({} samples): frames {}, packets {}, LDPC failures {}, freq {:.0} Hz, MER {:?}", big.len(), r.frames, r.packets.len(), r.ldpc_fail, r.freq_hz, r.mer_db);
        }
        // P1s along the longest raw run, a frame's worth at a time.
        if let Some((c0, big)) = runs.iter().max_by_key(|r| r.1.len()) {
            let fl = p.frame_samples();
            let mut at = 0;
            while at + fl + 4096 <= big.len() {
                let x = &big[at..at + fl + 4096];
                if let Some((s, f, q)) = super::super::stream::find_p1_in(x, p1, e1, 0, x.len() - 2048 + 1, fs) {
                    eprintln!("long run P1 at {} (+{s} in chunk), coarse {f:.0} Hz, q {q:.2}", *c0 + (at + s) as u64);
                }
                at += fl;
            }
        }
        // With every sample raw too (`/tmp/t2-rawall` on the board): each FFT
        // against a software FFT of the same window, by carrier magnitude,
        // at a few window shifts and in two orders.
        if let Some((c0, big)) = runs.iter().max_by_key(|r| r.1.len()).filter(|r| r.1.len() > 3_000_000) {
            let fs_ = fs;
            let f_off: f64 = std::env::var("T2FOFF").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let bins: Vec<usize> = (0..1705).map(|k| ofdm.bin(k)).collect();
            let order = super::super::fe::carrier_order(&bins);
            let mut planner = rustfft::FftPlanner::<f32>::new();
            let fft = planner.plan_fft_forward(2048);
            let mut done = 0;
            for (j, f22, v) in ffts.iter().filter(|f| f.2.len() == 1705).skip(8) {
                let fr = super::super::fe::extend(*f22, 21, *c0 + 1_000_000);
                let start = fr + 2048 + *j as u64 * 2304 + 256 - 64;
                if start < *c0 || start + 2100 > c0 + big.len() as u64 {
                    continue;
                }
                let hw: Vec<f32> = {
                    let mut c = vec![0f32; 1705];
                    for (pos, &k) in order.iter().enumerate() {
                        c[k] = v[pos].norm();
                    }
                    c
                };
                let mut best = (0f32, 0i64);
                for sh in -40i64..=40 {
                    let s0 = (start as i64 + sh - *c0 as i64) as usize;
                    let mut w: Vec<Complex32> = (0..2048)
                        .map(|n| {
                            let ph = -std::f64::consts::TAU * f_off * n as f64 / fs_;
                            big[s0 + n] * Complex32::new(ph.cos() as f32, ph.sin() as f32)
                        })
                        .collect();
                    fft.process(&mut w);
                    let sw: Vec<f32> = bins.iter().map(|&b| w[b].norm()).collect();
                    let (ma, mb) = (hw.iter().sum::<f32>() / 1705.0, sw.iter().sum::<f32>() / 1705.0);
                    let (mut num, mut da, mut db) = (0f32, 0f32, 0f32);
                    for (a, b) in hw.iter().zip(&sw) {
                        num += (a - ma) * (b - mb);
                        da += (a - ma).powi(2);
                        db += (b - mb).powi(2);
                    }
                    let rho = num / (da * db).sqrt().max(1e-20);
                    if rho > best.0 {
                        best = (rho, sh);
                    }
                }
                eprintln!("FFT j {j} F {fr}: best magnitude correlation {:.3} at window shift {}", best.0, best.1);
                if done == 0 {
                    let seq: Vec<String> = ffts.iter().take(40).map(|f| format!("{}@{}", f.0, f.1)).collect();
                    eprintln!("FFT sequence: {}", seq.join(" "));
                }
                done += 1;
                if done >= 6 {
                    break;
                }
            }
        }
        // Each FFT back to time (inverse FFT of its carriers) against the raw
        // samples its window shares with the raw windows: [g+192, g+256)
        // (guard interval) and [g+2048, g+2240) (the tail).
        {
            let bins: Vec<usize> = (0..1705).map(|k| ofdm.bin(k)).collect();
            let order = super::super::fe::carrier_order(&bins);
            let mut planner = rustfft::FftPlanner::<f32>::new();
            let ifft = planner.plan_fft_inverse(2048);
            let find = |at: u64, len: usize| -> Option<&[Complex32]> {
                runs.iter().rev().find_map(|(c, v)| (*c <= at && at + len as u64 <= c + v.len() as u64).then(|| &v[(at - c) as usize..(at - c) as usize + len]))
            };
            let near0 = runs.first().map_or(0, |r| r.0);
            let mut shown = 0;
            let mut summary: Vec<(u8, u64, f32)> = Vec::new();
            let skip: usize = std::env::var("T2SKIP").ok().and_then(|v| v.parse().ok()).unwrap_or(8);
            let mut near = near0 + 1_000_000;
            for (idx, (j, f22, v)) in ffts.iter().filter(|f| f.2.len() == 1705).enumerate() {
                let fr = super::super::fe::extend(*f22, 21, near);
                near = fr;
                if idx < skip {
                    continue;
                }
                let g = fr + 2048 + *j as u64 * 2304;
                // T2PERM: 1 = carrier words swapped in pairs, 2 = reversed
                let perm: u32 = std::env::var("T2PERM").ok().and_then(|x| x.parse().ok()).unwrap_or(0);
                let vv: Vec<Complex32> = match perm {
                    1 => (0..1705).map(|i| if i ^ 1 < 1705 { v[i ^ 1] } else { v[i] }).collect(),
                    2 => v.iter().rev().copied().collect(),
                    _ => v.clone(),
                };
                let mut w = vec![Complex32::default(); 2048];
                for (pos, &k) in order.iter().enumerate() {
                    w[bins[k]] = vv[pos];
                }
                ifft.process(&mut w);
                // window sample n is raw sample g + 192 + n
                let mut out = String::new();
                for (lo, len) in [(0usize, 64usize), (1856, 192)] {
                    if let Some(r) = find(g + 192 + lo as u64, len) {
                        let a = &w[lo..lo + len];
                        let num: Complex32 = a.iter().zip(r).map(|(x, y)| x * y.conj()).sum();
                        let den = (a.iter().map(|x| x.norm_sqr()).sum::<f32>() * r.iter().map(|x| x.norm_sqr()).sum::<f32>()).sqrt();
                        out += &format!(" [{lo}+{len}]: {:.3}", num.norm() / den.max(1e-20));
                        if lo == 1856 {
                            summary.push((*j, fr, num.norm() / den.max(1e-20)));
                        }
                    } else {
                        out += &format!(" [{lo}+{len}]: no raw");
                    }
                }
                // the frequency that lines the tail up with the raw samples
                if let Some(r) = find(g + 192 + 1856, 192) {
                    let a = &w[1856..2048];
                    let mut prod: Vec<Complex32> = a.iter().zip(r).map(|(x, y)| x * y.conj()).collect();
                    prod.resize(8192, Complex32::default());
                    let f8 = planner.plan_fft_forward(8192);
                    f8.process(&mut prod);
                    let (bi, bv) = prod.iter().enumerate().map(|(i, z)| (i, z.norm())).fold((0, 0f32), |m, (i, v)| if v > m.1 { (i, v) } else { m });
                    let bf = if bi >= 4096 { bi as f64 - 8192.0 } else { bi as f64 } * fs / 8192.0;
                    let den = (a.iter().map(|x| x.norm_sqr()).sum::<f32>() * r.iter().map(|x| x.norm_sqr()).sum::<f32>()).sqrt();
                    out += &format!(" | best at {:.0} Hz: {:.3}", -bf, bv / den.max(1e-20));
                }
                // Is it another window's FFT (a wrong label)? Window d symbols on.
                let mut best = (0f32, 0i64);
                for d in -12i64..=12 {
                    let gd = (g as i64 + d * 2304) as u64;
                    if let Some(r) = find(gd + 192 + 1856, 192) {
                        let a = &w[1856..2048];
                        let num: Complex32 = a.iter().zip(r).map(|(x, y)| x * y.conj()).sum();
                        let den = (a.iter().map(|x| x.norm_sqr()).sum::<f32>() * r.iter().map(|x| x.norm_sqr()).sum::<f32>()).sqrt();
                        let c = num.norm() / den.max(1e-20);
                        if c > best.0 {
                            best = (c, d);
                        }
                    }
                }
                out += &format!(" | best window offset {} symbols: {:.3}", best.1, best.0);
                // Misframed? The raw tail of a nearby window anywhere in the
                // FFT's 2048 samples.
                let mut bestm = (0f32, 0i64, 0usize);
                for d in -3i64..=3 {
                    let gd = (g as i64 + d * 2304) as u64;
                    if let Some(r) = find(gd + 192 + 1856, 192) {
                        let er: f32 = r.iter().map(|x| x.norm_sqr()).sum();
                        for lag in 0..(2048 - 192) {
                            let a = &w[lag..lag + 192];
                            let num: Complex32 = a.iter().zip(r).map(|(x, y)| x * y.conj()).sum();
                            let ea: f32 = a.iter().map(|x| x.norm_sqr()).sum();
                            let c = num.norm() / (ea * er).sqrt().max(1e-20);
                            if c > bestm.0 {
                                bestm = (c, d, lag);
                            }
                        }
                    }
                }
                out += &format!(" | misframed? window {} tail at lag {}: {:.3}", bestm.1, bestm.2, bestm.0);
                if std::env::var_os("T2SUMMARY").is_none() {
                    eprintln!("FFT j {j} F {fr} vs raw:{out}");
                }
                shown += 1;
                let lim: usize = std::env::var("T2NFFT").ok().and_then(|x| x.parse().ok()).unwrap_or(6);
                if shown >= lim {
                    break;
                }
            }
            if !summary.is_empty() {
                let good = summary.iter().filter(|x| x.2 > 0.8).count();
                let frames: std::collections::BTreeMap<u64, (usize, usize)> = summary.iter().fold(Default::default(), |mut m, x| {
                    let e = m.entry(x.1).or_insert((0, 0));
                    e.1 += 1;
                    if x.2 > 0.8 {
                        e.0 += 1;
                    }
                    m
                });
                eprintln!("FFT vs raw summary: {good} of {} good; per frame (good/all): {:?}", summary.len(), frames);
            }
        }
        // Continual pilots (the same carriers in every data symbol): their
        // coherence does not depend on which symbol the FFT is.
        {
            let bins: Vec<usize> = (0..1705).map(|k| ofdm.bin(k)).collect();
            let order = super::super::fe::carrier_order(&bins);
            let plans: Vec<Vec<Option<Complex32>>> = (8..p.symbols()).map(|j| ofdm.plan(j)).collect();
            let cont: Vec<usize> = (0..1705).filter(|&k| plans.iter().all(|pl| pl[k].is_some_and(|z| z.norm() > 0.0))).collect();
            let mut shown = 0;
            for (j, _f22, v) in ffts.iter().filter(|f| f.2.len() == 1705 && f.0 >= 10).skip(20) {
                let mut c = vec![Complex32::default(); 1705];
                for (pos, &k) in order.iter().enumerate() {
                    c[k] = v[pos];
                }
                let pl = &plans[0];
                let z: Vec<Complex32> = cont.iter().map(|&k| c[k] / pl[k].unwrap()).collect();
                // the pilot sign pattern depends on the symbol (pn[j]):
                // compare magnitudes of neighbour products instead
                let num: Complex32 = z.windows(2).map(|w| w[1] * w[0].conj()).map(|x| x * x).sum();
                let den: f32 = z.windows(2).map(|w| (w[1] * w[0].conj()).norm_sqr()).sum();
                let pw_p: f32 = z.iter().map(|x| x.norm_sqr()).sum::<f32>() / z.len() as f32;
                let pw_all: f32 = c.iter().map(|x| x.norm_sqr()).sum::<f32>() / 1705.0;
                eprintln!("FFT j {j}: {} continual pilots, squared-product coherence {:.2}, pilot/mean power {:.2}", cont.len(), num.norm() / den.max(1e-20), pw_p / pw_all);
                shown += 1;
                if shown >= 4 {
                    break;
                }
            }
        }
        // Pilot coherence of the P2 symbols' FFTs: neighbouring pilots after
        // removing the known values.
        let bins: Vec<usize> = (0..1705).map(|k| ofdm.bin(k)).collect();
        let order = super::super::fe::carrier_order(&bins);
        for (j, f22, v) in ffts.iter().filter(|f| f.0 < 8 && f.2.len() == 1705).take(6) {
            let mut c = vec![Complex32::default(); 1705];
            for (pos, &k) in order.iter().enumerate() {
                c[k] = v[pos];
            }
            let plan = ofdm.plan(*j as usize);
            let z: Vec<Complex32> = plan.iter().enumerate().filter_map(|(k, pv)| pv.filter(|x| x.norm() > 0.0).map(|x| c[k] / x)).collect();
            let num: Complex32 = z.windows(2).map(|w| w[1] * w[0].conj()).sum();
            let den: f32 = z.windows(2).map(|w| w[1].norm() * w[0].norm()).sum();
            let pw: f32 = c.iter().map(|x| x.norm_sqr()).sum::<f32>() / 1705.0;
            eprintln!("FFT j {j} F {f22}: pilot coherence {:.2} (phase step {:.3} rad), mean power {pw:.0}", num.norm() / den, num.arg());
        }
    }

    /// The front-end path's time per stage (run it on the board:
    /// `trxd-test t2_fe_speed --ignored --nocapture`): words made by the
    /// model first (closed loop), then only the receiver timed on them.
    #[test]
    #[ignore]
    fn t2_fe_speed() {
        use super::super::fe::model::Model;
        let p = Params::amateur();
        let mut m = Modulator::new(p);
        let mut next = || [0x47u8; TS_LEN];
        let mut x = Vec::new();
        for _ in 0..7 {
            m.frame(&mut next, &mut x);
        }
        let fs = 131e6 / 71.0;
        let q = |v: f32| (v * 3000.0).round().clamp(-32768.0, 32767.0) as i16;
        let samples: Vec<[i16; 2]> = x.iter().map(|z| [q(z.re), q(z.im)]).collect();
        let bins: Vec<usize> = (0..1705).map(|k| super::super::ofdm::Ofdm::new(p).bin(k)).collect();
        let mut fe = Model::new(p.frame_samples() as u64, p.symbols() as u64, p.guard.samples() as u64, &bins);
        // T2EQ=1: with the FPGA's equalizer (in the model)
        if std::env::var_os("T2EQ").is_some() {
            let (dx, dy) = p.pilots.dxdy();
            fe.enable_eq(super::super::N_P2, dx, dy, p.symbols() - 1);
        }
        let mut d = super::super::stream::Demod::new(p, fs);
        let (mut blocks, mut ctl) = (Vec::new(), Vec::new());
        let mut chunks = Vec::new();
        for c in samples.chunks(9225) {
            let mut w = Vec::new();
            fe.run(c, &mut w);
            d.push_words(&w, &mut blocks, &mut ctl);
            for cmd in ctl.drain(..) {
                fe.apply(cmd);
            }
            chunks.push(w);
        }
        blocks.clear();
        let mut d = super::super::stream::Demod::new(p, fs);
        // T2CELLS=1: QPSK blocks out as cells (the FPGA decoder's LLRs)
        d.cells_out = std::env::var_os("T2CELLS").is_some();
        let t = std::time::Instant::now();
        // Steady state: from the end of the first frame decoded.
        let (mut t1, mut prof1, mut f1) = (None, [0.0; 6], 0u64);
        for w in &chunks {
            d.push_words(w, &mut blocks, &mut ctl);
            if t1.is_none() && d.stats.frames >= 1 {
                t1 = Some(std::time::Instant::now());
                prof1 = d.prof;
                f1 = d.stats.frames;
            }
        }
        let el = t.elapsed().as_secs_f64();
        let frames = (d.stats.frames - f1) as f64;
        eprintln!("{} frames, {} blocks in {:.3} s; MER {:.1} dB; steady state {:.1} ms a frame", d.stats.frames, blocks.len(), el, d.stats.mer_db,
            1e3 * t1.map_or(0.0, |t| t.elapsed().as_secs_f64()) / frames.max(1.0));
        for ((n, v), v1) in super::super::stream::PROF_NAMES.iter().zip(d.prof).zip(prof1) {
            eprintln!("  {n:>12}: {:.1} ms a frame", 1e3 * (v - v1) / frames.max(1.0));
        }
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

    fn loopback(p: Params, snr_db: f32, cells: bool) {
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
        let r = receive_as(p, &x, fs, cells);
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
    let mut x: Vec<Complex32> = raw.chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
    // T2SHIFT=<Hz>: the signal sits that far from the centre (moved to it).
    if let Some(sh) = std::env::var("T2SHIFT").ok().and_then(|v| v.parse::<f64>().ok()) {
        for (k, z) in x.iter_mut().enumerate() {
            let ph = -std::f64::consts::TAU * sh * k as f64 / 3_072_000.0;
            *z *= Complex32::new(ph.cos() as f32, ph.sin() as f32);
        }
    }
    let fs = 131e6 / 71.0;
    // T2FPGA=<scale>: the FPGA's resampler model instead (12-bit ADC
    // samples, full scale 1.0 * scale), its integer output.
    let mut y = match std::env::var("T2FPGA").ok().and_then(|v| v.parse::<f32>().ok()) {
        Some(scale) => {
            let step = super::resamp::step(3_072_000.0, fs);
            let mut rs = super::resamp::Resampler::new(super::resamp::t2_table(3_072_000.0, fs), step);
            let adc = super::resamp::adc12(&x, scale);
            let clip = adc.iter().filter(|v| v[0].abs() >= 2047 || v[1].abs() >= 2047).count();
            eprintln!("12-bit ADC: {clip} clipped of {}", adc.len());
            rs.process(&adc).iter().map(|v| Complex32::new(v[0] as f32 / 32768.0, v[1] as f32 / 32768.0)).collect()
        }
        None => resample(&x, 3_072_000.0, fs),
    };
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
