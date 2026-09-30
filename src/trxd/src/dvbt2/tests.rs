//! The modulator against GNU Radio gr-dtv, stage by stage: run
//! datv-ref/t2/ref/t2ref.py (OUTDIR 190 9: QPSK 1/2, PP2, GI 1/8, 2K,
//! 1.7 MHz), then `T2REF=<OUTDIR> cargo test t2_matches_gr_dtv -- --ignored --nocapture`.

use num_complex::Complex32;

use super::*;
use crate::dvbs2::TS_LEN;

fn read_c32(path: &str) -> Vec<Complex32> {
    std::fs::read(path)
        .unwrap()
        .chunks_exact(8)
        .map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap())))
        .collect()
}

fn max_err(a: &[Complex32], b: &[Complex32]) -> f32 {
    a.iter().zip(b).map(|(x, y)| (x - y).norm()).fold(0.0, f32::max)
}

#[test]
#[ignore]
fn t2_matches_gr_dtv() {
    let dir = std::env::var("T2REF").expect("T2REF=<t2ref.py output dir>");
    let ts = std::fs::read(format!("{dir}/in.ts")).unwrap();
    let mut pkts = ts.chunks_exact(TS_LEN).map(|c| <[u8; TS_LEN]>::try_from(c).unwrap());
    let mut next = || pkts.next().expect("out of TS");
    let mut p = Params::amateur();
    // T2RATE=34: a reference made with C3_4.
    if std::env::var("T2RATE").is_ok_and(|v| v == "34") {
        p.rate = crate::dvbs2::ldpc_fpga::LongRate::R3_4;
    }
    // T2ROT=1: a reference made with ROTATION_ON.
    p.rotation = std::env::var_os("T2ROT").is_some();
    // T2CONST=16: made with MOD_16QAM (and 18 FEC blocks).
    if std::env::var("T2CONST").is_ok_and(|v| v == "16") {
        p.constellation = Constellation::Qam16;
        p.fec_blocks = 18;
    }
    let bi = BitInterleaver::new(&p);
    let mut m = Modulator::new(p);
    let ldpc_ref = std::fs::read(format!("{dir}/ldpc.bin")).unwrap();
    let cells_ref = read_c32(&format!("{dir}/modulator.bin"));
    let ci_ref = read_c32(&format!("{dir}/cellinterleaver.bin"));
    let fm_ref = read_c32(&format!("{dir}/framemapper.bin"));
    let fi_ref = read_c32(&format!("{dir}/freqinterleaver.bin"));
    let pg_ref = read_c32(&format!("{dir}/pilotgenerator.bin"));
    let out_ref = read_c32(&format!("{dir}/p1.bin"));
    let frames = 3;
    let n = p.cells();
    let mut fm = frame::FrameMapper::new(p);
    let fi = frame::FreqInterleaver::new(&p);
    let ofdm = ofdm::Ofdm::new(p);
    let fcells = p.frame_cells();
    let fsamples = p.frame_samples();
    let pal = Palette::new(&p);
    let cx = |v: &[Cell]| v.iter().map(|&c| pal.get(c)).collect::<Vec<Complex32>>();
    for f in 0..frames {
        let mut blocks = Vec::new();
        for b in 0..p.fec_blocks {
            let k = f * p.fec_blocks + b;
            let cw = m.codeword(&mut next);
            let bad = cw.iter().zip(&ldpc_ref[k * 64_800..(k + 1) * 64_800]).filter(|(a, b)| a != b).count();
            assert_eq!(bad, 0, "frame {f} block {b}: LDPC codeword differs in {bad} bits");
            let cells = codes(&p, &bi.words(&cw));
            let e = max_err(&cx(&cells), &cells_ref[k * n..(k + 1) * n]);
            assert!(e < 1e-6, "frame {f} block {b}: cells differ by {e}");
            blocks.push(cells);
        }
        let data = interleave(&p, &blocks);
        let e = max_err(&cx(&data), &ci_ref[f * data.len()..(f + 1) * data.len()]);
        assert!(e < 1e-6, "frame {f}: cell/time interleaver differs by {e}");
        let cells = cx(&fm.frame(&data));
        let r = &fm_ref[f * fcells..(f + 1) * fcells];
        let bad: Vec<usize> = (0..fcells).filter(|&i| (cells[i] - r[i]).norm() > 1e-5).collect();
        assert!(bad.is_empty(), "frame {f}: frame mapper differs at {} cells, first {:?}: ours {:?} gr {:?}", bad.len(), &bad[..bad.len().min(5)], cells[bad[0]], r[bad[0]]);
        let codes = { let mut fm2 = frame::FrameMapper::new(p); for _ in 0..f { fm2.frame(&data); } fm2.frame(&data) };
        let syms = fi.frame(&codes);
        let flat: Vec<Complex32> = syms.iter().flat_map(|s| cx(s)).collect();
        let e = max_err(&flat, &fi_ref[f * fcells..(f + 1) * fcells]);
        assert!(e < 1e-6, "frame {f}: frequency interleaver differs by {e}");
        let mut out = Vec::new();
        ofdm.frame(&syms, &mut out);
        let nsym = p.symbols();
        // Before the guard interval: compare the pilot generator's symbols.
        let gi = p.guard.samples();
        for j in 0..nsym {
            let ours = &out[2048 + j * (FFT + gi) + gi..2048 + (j + 1) * (FFT + gi)];
            let theirs = &pg_ref[(f * nsym + j) * FFT..(f * nsym + j + 1) * FFT];
            let e = max_err(ours, theirs);
            if e >= 1e-4 {
                // Back to carriers: which differ?
                let fwd = rustfft::FftPlanner::new().plan_fft_forward(FFT);
                let car = |x: &[Complex32]| {
                    let mut b = x.to_vec();
                    fwd.process(&mut b);
                    let mut c = vec![Complex32::default(); FFT];
                    c[..FFT / 2].copy_from_slice(&b[FFT / 2..]);
                    c[FFT / 2..].copy_from_slice(&b[..FFT / 2]);
                    c
                };
                let (a, b) = (car(ours), car(theirs));
                let diff: Vec<(usize, Complex32, Complex32)> = (0..FFT).filter(|&k| (a[k] - b[k]).norm() > 1e-2).map(|k| (k, a[k], b[k])).take(12).collect();
                let nd = (0..FFT).filter(|&k| (a[k] - b[k]).norm() > 1e-2).count();
                panic!("frame {f} symbol {j}: OFDM differs by {e}; {nd} carriers, first (index, ours, gr): {diff:?}");
            }
        }
        let e = max_err(&out, &out_ref[f * fsamples..(f + 1) * fsamples]);
        eprintln!("frame {f}: bit-exact to the cells, output max error {e:.2e}");
        assert!(e < 1e-4, "frame {f}: output differs by {e}");
    }
}

/// Oversampled by 2, every other sample is the plain frame (zero-padded
/// IFFT, P1 evaluated as a waveform).
#[test]
fn oversampling_keeps_the_waveform() {
    let p = Params::amateur();
    let mut m = Modulator::new(p);
    let mut n = 0u32;
    let mut next = || {
        let mut pkt = [0u8; TS_LEN];
        pkt[0] = 0x47;
        pkt[4..8].copy_from_slice(&n.to_be_bytes());
        n += 1;
        pkt
    };
    let p2 = p;
    let blocks: Vec<Vec<Cell>> = (0..p.fec_blocks).map(|_| cell_codes(&m.codeword(&mut next))).collect();
    let data = interleave(&p, &blocks);
    let cells = frame::FrameMapper::new(p).frame(&data);
    let syms = frame::FreqInterleaver::new(&p).frame(&cells);
    let (mut a, mut b) = (Vec::new(), Vec::new());
    ofdm::Ofdm::new(p).frame(&syms, &mut a);
    ofdm::Ofdm::oversampled(p2, 8).frame(&syms, &mut b);
    assert_eq!(b.len(), 2 * a.len());
    let e = a.iter().enumerate().map(|(i, z)| (z - b[2 * i]).norm()).fold(0f32, f32::max);
    assert!(e < 1e-4, "max error {e}");
}

/// Frames a second the modulator makes (the A9 must beat 4 at 1.7 MHz):
/// `cargo test --release t2_speed -- --ignored --nocapture`
#[test]
#[ignore]
fn t2_speed() {
    let mut p = Params::amateur();
    // T2CONST=16: 16QAM, 18 FEC blocks.
    if std::env::var("T2CONST").is_ok_and(|v| v == "16") {
        p.constellation = Constellation::Qam16;
        p.fec_blocks = 18;
    }
    let mut m = Modulator::oversampled(p, 4);
    let mut next = || {
        let mut pkt = [0u8; TS_LEN];
        pkt[0] = 0x47;
        pkt
    };
    let mut out = Vec::new();
    let t0 = std::time::Instant::now();
    for _ in 0..8 {
        out.clear();
        m.frame(&mut next, &mut out);
    }
    let dt = t0.elapsed().as_secs_f64() / 8.0;
    eprintln!("{:.1} ms a T2 frame ({:?}, {:.0} ms of signal)", dt * 1e3, p.constellation, p.frame_samples() as f64 / (131e6 / 71.0) * 1e3);
}

#[test]
#[ignore]
fn t2_profile() {
    let p = Params::amateur();
    let mut m = Modulator::new(p);
    let mut next = || {
        let mut pkt = [0u8; TS_LEN];
        pkt[0] = 0x47;
        pkt
    };
    let mut fm = frame::FrameMapper::new(p);
    let fi = frame::FreqInterleaver::new(&p);
    let of = ofdm::Ofdm::oversampled(p, 5);
    let ci = CellInterleaver::new(&p);
    let mut t = [0f64; 6];
    for _ in 0..8 {
        let t0 = std::time::Instant::now();
        let cws: Vec<Vec<u8>> = (0..p.fec_blocks).map(|_| m.codeword(&mut next)).collect();
        let t1 = std::time::Instant::now();
        let blocks: Vec<Vec<Cell>> = cws.iter().map(|c| cell_codes(c)).collect();
        let data = ci.frame(&blocks);
        let t2 = std::time::Instant::now();
        let cells = fm.frame(&data);
        let t3 = std::time::Instant::now();
        let syms = fi.frame(&cells);
        let t4 = std::time::Instant::now();
        let mut out = Vec::new();
        of.frame(&syms, &mut out);
        let t5 = std::time::Instant::now();
        for (i, d) in [t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4].iter().enumerate() {
            t[i] += d.as_secs_f64() * 1e3 / 8.0;
        }
    }
    eprintln!("ms a frame: codewords {:.2}, cells+interleave {:.2}, frame mapper {:.2}, freq il {:.2}, ofdm x1.25 {:.2}", t[0], t[1], t[2], t[3], t[4]);
    let fft = rustfft::FftPlanner::<f32>::new().plan_fft_inverse(2560);
    let mut b = vec![Complex32::new(0.1, 0.2); 2560];
    let t0 = std::time::Instant::now();
    for _ in 0..198 {
        fft.process(&mut b);
    }
    eprintln!("198 IFFTs of 2560: {:.1} ms", t0.elapsed().as_secs_f64() * 1e3);
    let t0 = std::time::Instant::now();
    std::thread::scope(|sc| {
        for _ in 0..2 {
            sc.spawn(|| {
                let mut b = vec![Complex32::new(0.1, 0.2); 2560];
                for _ in 0..99 {
                    fft.process(&mut b);
                }
            });
        }
    });
    eprintln!("the same on two threads: {:.1} ms", t0.elapsed().as_secs_f64() * 1e3);
}

/// Where the transmitter's FEC stage spends its time, per T2 frame (run it
/// on the board: `trxd-test --ignored --nocapture t2_fec_split`).
/// The packed FEC path (FecStage::frame) against the bit-a-byte stages.
#[test]
fn t2_fec_fast_matches_ref() {
    for (qam, rate, rot) in [(false, LongRate::R1_2, false), (true, LongRate::R1_2, true), (false, LongRate::R3_4, true), (true, LongRate::R3_4, false)] {
        let mut p = Params::amateur();
        p.rate = rate;
        p.rotation = rot;
        p.fec_blocks = if rate == LongRate::R3_4 { 6 } else { 9 };
        if qam {
            p.constellation = Constellation::Qam16;
            p.fec_blocks *= 2;
        }
        let pk = |n: &mut u8| {
            *n = n.wrapping_add(7);
            let mut pk = [*n; TS_LEN];
            pk[0] = 0x47;
            pk[5] = n.wrapping_mul(13);
            pk
        };
        let (mut a, mut b) = (FecStage::new(p), FecStage::new(p));
        let (mut na, mut nb) = (0u8, 0u8);
        for f in 0..3 {
            let fast = a.frame(&mut || pk(&mut na));
            let slow = b.frame_ref(&mut || pk(&mut nb));
            assert!(fast == slow, "{p:?} frame {f}");
        }
    }
}

/// FastFrame's DMA bytes against the stage-by-stage path's frame_fpga.
#[test]
fn t2_fast_frame_matches() {
    for (qam, rot) in [(false, true), (true, false), (true, true), (false, false)] {
        let mut p = Params::amateur();
        p.rotation = rot;
        if qam {
            p.constellation = Constellation::Qam16;
            p.fec_blocks *= 2;
        }
        let pk = |n: &mut u8| {
            *n = n.wrapping_add(3);
            let mut pk = [*n; TS_LEN];
            pk[0] = 0x47;
            pk[9] = n.wrapping_mul(29);
            pk
        };
        let o = OfdmStage::new(p, 4);
        let ff = FastFrame::new(p, o.ofdm(), 48.0);
        let (mut a, mut b) = (FecStage::new(p), FecStage::new(p));
        let (mut na, mut nb) = (0u8, 0u8);
        for f in 0..3 {
            let codes = a.frame_codes(&mut || pk(&mut na));
            let mut fast = Vec::new();
            ff.frame(&codes, f, &mut fast);
            let cells = b.frame_ref(&mut || pk(&mut nb));
            let mut slow = Vec::new();
            o.frame_fpga(&cells, 48.0, &mut slow);
            assert_eq!(fast.len(), slow.len());
            let bad = fast.chunks(4).zip(slow.chunks(4)).position(|(x, y)| x != y);
            assert!(bad.is_none(), "{p:?} frame {f}: first different word {bad:?}");
            // in TX-block pieces
            let mut pieces = vec![0u8; fast.len()];
            for (i, c) in pieces.chunks_mut(4 * 1777).enumerate() {
                ff.fill(&codes, f, i * 4 * 1777, c);
            }
            assert!(pieces == fast, "{p:?} frame {f}: fill in pieces differs");
        }
    }
}

#[test]
#[ignore]
fn t2_fec_split() {
    for qam in [false, true] {
        let mut p = Params::amateur();
        if qam {
            p.constellation = Constellation::Qam16;
            p.fec_blocks *= 2;
        }
        let mut f = FecStage::new(p);
        let mut n = 0u8;
        let mut next = || {
            n = n.wrapping_add(1);
            let mut pk = [n; TS_LEN];
            pk[0] = 0x47;
            pk
        };
        let frames = 8;
        let mut t = [0f64; 6];
        for _ in 0..frames {
            let mut blocks = Vec::new();
            for _ in 0..p.fec_blocks {
                let t0 = std::time::Instant::now();
                let kbch = p.kbch();
                let mut info = f.framer.frame_bytes(kbch / 8, 0, &mut next);
                for (b, s) in info.iter_mut().zip(&f.bbscr) {
                    *b ^= s;
                }
                let t1 = std::time::Instant::now();
                let par = f.bch.parity_bytes(&info);
                info.extend_from_slice(&par);
                let t2 = std::time::Instant::now();
                let cw = if qam { crate::dvbs2::ldpc_fpga::encode_packed_pi(p.rate, &info) } else { crate::dvbs2::ldpc_fpga::encode_packed(p.rate, &info) };
                let t3 = std::time::Instant::now();
                blocks.push(if qam { f.bi.words_pi(&cw) } else { cw });
                let t4 = std::time::Instant::now();
                t[0] += (t1 - t0).as_secs_f64();
                t[1] += (t2 - t1).as_secs_f64() - t[4] + t[4];
                t[2] += (t3 - t2).as_secs_f64();
                t[3] += (t4 - t3).as_secs_f64();
            }
            let t4 = std::time::Instant::now();
            let data = f.ci.frame_words(p.constellation.bits(), &blocks);
            let t5 = std::time::Instant::now();
            let _cells = f.mapper.frame(&data);
            t[4] += (t5 - t4).as_secs_f64();
            t[5] += t5.elapsed().as_secs_f64();
        }
        let ms = |v: f64| v / frames as f64 * 1e3;
        eprintln!(
            "{}: per frame ({} blocks): framing+scramble {:.1} ms, BCH {:.1}, LDPC {:.1}, bit interleave+cells {:.1}, cell+time interleave {:.1}, frame map {:.1}",
            if qam { "16QAM" } else { "QPSK" }, p.fec_blocks, ms(t[0]), ms(t[1]), ms(t[2]), ms(t[3]), ms(t[4]), ms(t[5])
        );
    }
}

/// One frame for the FPGA's IFFT and the CPU's samples of the same frame
/// (x scale x 256, as 16-bit): T2FPGA_OUT=<dir> writes fpga_in.bin (the DMA
/// bytes) and cpu_out.bin (f32 I/Q), for maia-hdl's t2ifft model to compare.
#[test]
#[ignore]
fn t2_fpga_frame_dump() {
    let dir = std::env::var("T2FPGA_OUT").expect("T2FPGA_OUT");
    let p = Params::amateur();
    let mut f = FecStage::new(p);
    let mut n = 0u8;
    let mut next = || {
        n = n.wrapping_add(7);
        let mut pk = [n; TS_LEN];
        pk[0] = 0x47;
        pk
    };
    let cells = f.frame(&mut next);
    let o = OfdmStage::new(p, 4);
    let scale = 48.0;
    let mut bytes = Vec::new();
    o.frame_fpga(&cells, scale, &mut bytes);
    let mut iq = Vec::new();
    o.frame(&cells, &mut iq);
    let cpu: Vec<u8> = iq.iter().flat_map(|z| [(z.re * scale * 256.0).to_le_bytes(), (z.im * scale * 256.0).to_le_bytes()]).flatten().collect();
    std::fs::write(format!("{dir}/fpga_in.bin"), &bytes).unwrap();
    std::fs::write(format!("{dir}/cpu_out.bin"), &cpu).unwrap();
    eprintln!("fpga words {}, cpu samples {}", bytes.len() / 4, iq.len());
}

/// Per-frame time of the two TX threads' work, FastFrame against the
/// stage-by-stage path (run on the board).
#[test]
#[ignore]
fn t2_fast_timing() {
    for qam in [false, true] {
        let mut p = Params::amateur();
        p.rotation = true;
        if qam {
            p.constellation = Constellation::Qam16;
            p.fec_blocks *= 2;
        }
        let mut n = 0u8;
        let mut next = || {
            n = n.wrapping_add(1);
            let mut pk = [n; TS_LEN];
            pk[0] = 0x47;
            pk
        };
        let o = OfdmStage::new(p, 4);
        let ff = FastFrame::new(p, o.ofdm(), 48.0);
        let mut f = FecStage::new(p);
        let frames = 8;
        let mut t = [0f64; 5];
        let mut reuse = Vec::with_capacity(4 * (1 + 2048 + p.symbols() * 1705));
        for _ in 0..frames {
            let t0 = std::time::Instant::now();
            let codes = f.frame_codes(&mut next);
            let t1 = std::time::Instant::now();
            let mut out = Vec::with_capacity(4 * (1 + 2048 + p.symbols() * 1705));
            ff.frame(&codes, 0, &mut out);
            let t2 = std::time::Instant::now();
            reuse.clear();
            ff.frame(&codes, 0, &mut reuse);
            let t3 = std::time::Instant::now();
            let cells = f.frame(&mut next);
            let t4 = std::time::Instant::now();
            let mut out = Vec::with_capacity(4 * (1 + 2048 + p.symbols() * 1705));
            o.frame_fpga(&cells, 48.0, &mut out);
            t[0] += (t1 - t0).as_secs_f64();
            t[1] += (t2 - t1).as_secs_f64();
            t[4] += (t3 - t2).as_secs_f64();
            t[2] += (t4 - t3).as_secs_f64();
            t[3] += t4.elapsed().as_secs_f64();
        }
        let ms = |v: f64| v / frames as f64 * 1e3;
        eprintln!(
            "{}: per frame: fast FEC {:.1} ms + gather {:.1} ms (reused buffer {:.1}); staged FEC {:.1} ms + frame_fpga {:.1} ms",
            if qam { "16QAM" } else { "QPSK" }, ms(t[0]), ms(t[1]), ms(t[4]), ms(t[2]), ms(t[3])
        );
    }
}

#[test]
#[ignore]
fn tmp_eq_bench() {
    let n = 1532usize;
    let data: Vec<usize> = (0..1705).filter(|k| k % 12 != 3 && k % 37 != 0).take(n).collect();
    let cj: Vec<[i16; 2]> = (0..1705).map(|k| [(k * 7 % 2000) as i16 - 1000, (k * 13 % 2000) as i16 - 1000]).collect();
    let g: Vec<[i32; 2]> = (0..1705).map(|k| [(k % 4000) as i32, 4000 - (k % 3000) as i32]).collect();
    let fine: Vec<[i32; 2]> = (0..64).map(|m| [16384 - m, m * 10]).collect();
    let coarse: Vec<[i32; 2]> = (0..27).map(|m| [16384 - m, m * 10]).collect();
    let scatter: Vec<u32> = (0..n as u32).map(|i| (i.wrapping_mul(2654435761) >> 8) % 291600).collect();
    let mut cells = vec![[0i8; 2]; 291600];
    let mut flat = vec![[0i8; 2]; n];
    let s = 16;
    let q = |x: i64| ((x + (1 << (s - 1))) >> s).clamp(-127, 127) as i8;
    let syms = 190 * 10;
    let t = std::time::Instant::now();
    for _ in 0..syms {
        for ((o, &d), &k) in flat.iter_mut().zip(&scatter).zip(&data) {
            let (f, c) = (fine[k & 63], coarse[k >> 6]);
            let r = [(f[0] * c[0] - f[1] * c[1]) >> 14, (f[0] * c[1] + f[1] * c[0]) >> 14];
            let gk = g[k];
            let gr = [(gk[0] * r[0] - gk[1] * r[1]) >> 14, (gk[0] * r[1] + gk[1] * r[0]) >> 14];
            let (x, y) = (cj[k][0] as i64, cj[k][1] as i64);
            let (a, b) = (gr[0] as i64, gr[1] as i64);
            let c = [q(x * a - y * b), q(x * b + y * a)];
            if d != u32::MAX { cells[d as usize] = c; } else { *o = c; }
        }
    }
    let full = t.elapsed().as_secs_f64() * 1e3 / 10.0;
    let t = std::time::Instant::now();
    for _ in 0..syms {
        for (o, &k) in flat.iter_mut().zip(&data) {
            let (f, c) = (fine[k & 63], coarse[k >> 6]);
            let r = [(f[0] * c[0] - f[1] * c[1]) >> 14, (f[0] * c[1] + f[1] * c[0]) >> 14];
            let gk = g[k];
            let gr = [(gk[0] * r[0] - gk[1] * r[1]) >> 14, (gk[0] * r[1] + gk[1] * r[0]) >> 14];
            let (x, y) = (cj[k][0] as i64, cj[k][1] as i64);
            let (a, b) = (gr[0] as i64, gr[1] as i64);
            *o = [q(x * a - y * b), q(x * b + y * a)];
        }
        std::hint::black_box(&flat);
    }
    let seq = t.elapsed().as_secs_f64() * 1e3 / 10.0;
    let t = std::time::Instant::now();
    let mut acc = 0i32;
    for _ in 0..syms {
        for &k in &data {
            acc = acc.wrapping_add(cj[k][0] as i32 + g[k][1]);
        }
    }
    std::hint::black_box(acc);
    let loads = t.elapsed().as_secs_f64() * 1e3 / 10.0;
    eprintln!("per frame (190 symbols): full {full:.1} ms, sequential stores {seq:.1} ms, loads only {loads:.1} ms");
}

/// The cell router's mapping (`deinterleaved_index`) against the receiver's
/// per-block gather: every time-deinterleaved position lands where
/// `block_gather` puts it.
#[test]
fn deinterleaved_index_matches_gather() {
    for p in [Params::amateur(), {
        let mut q = Params::amateur();
        q.constellation = Constellation::Qam16;
        q.fec_blocks = 18;
        q
    }] {
        let ci = CellInterleaver::new(&p);
        let n = p.cells();
        let ti: Vec<u32> = (0..(n * p.fec_blocks) as u32).collect();
        let mut out = Vec::new();
        for (r, blk) in ti.chunks(n).enumerate() {
            ci.block_gather(r, blk, &mut out);
            for (q, &g) in out.iter().enumerate() {
                assert_eq!(ci.deinterleaved_index(g as usize), r * n + q, "block {r} cell {q}");
            }
        }
    }
}
