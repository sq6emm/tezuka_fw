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
            let cells = cell_codes(&cw);
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
    let p = Params::amateur();
    let mut m = Modulator::oversampled(p, 5);
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
    eprintln!("{:.1} ms a T2 frame x1.25 ({:.0} ms of signal)", dt * 1e3, p.frame_samples() as f64 / (131e6 / 71.0) * 1e3);
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
