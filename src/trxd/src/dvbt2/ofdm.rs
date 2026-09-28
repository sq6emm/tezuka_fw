//! Pilots and OFDM (gr-dtv dvbt2_pilotgenerator_cc, ofdm_cyclic_prefixer,
//! dvbt2_p1insertion_cc), 2K SISO, normal carriers.
//!
//! Optionally oversampled by `os4 / 4` (a larger IFFT with the same
//! carriers, guard intervals and P1 scaled with it) so that the FPGA's
//! 16-tap resampler sees the spectrum well inside its pass band; `os4 = 4`
//! is gr-dtv's output exactly.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

use super::tables::{P1_ACTIVE, P2_PAPR_2K, PN_SEQUENCE, PP1_CP, PP2_CP, PP3_CP, PP4_CP, PP5_CP, PP7_CP, S1_PATTERNS, S2_PATTERNS};
use super::{Cell, Palette, Params, Pilots, FFT, N_P2};

/// Carriers in use (2K).
const C_PS: usize = 1705;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Carrier {
    Data,
    Scattered,
    Continual,
    P2Pilot,
    Reserved,
}

pub struct Ofdm {
    p: Params,
    prbs: Vec<u8>,
    pn: Vec<u8>,
    p2_map: Vec<Carrier>,
    fc_map: Vec<Carrier>,
    data_maps: Vec<Vec<Carrier>>,
    sp: f32,
    cp: f32,
    p2: f32,
    norm: f32,
    fft: Arc<dyn Fft<f32>>,
    p1: Vec<Complex32>,
    /// IFFT size (2048 x os4 / 4) and where carrier 0 sits in it.
    n: usize,
    left: usize,
    pal: Palette,
}

impl Ofdm {
    pub fn new(p: Params) -> Ofdm {
        Self::oversampled(p, 4)
    }

    /// Oversampled by `os4 / 4` (4 to 8).
    pub fn oversampled(p: Params, os4: usize) -> Ofdm {
        assert!((4..=8).contains(&os4));
        let n = FFT * os4 / 4;
        let mut sr: u32 = 0x7ff;
        let prbs: Vec<u8> = (0..C_PS)
            .map(|_| {
                let b = (sr ^ (sr >> 2)) & 1;
                let out = (sr & 1) as u8;
                sr >>= 1;
                if b != 0 {
                    sr |= 0x400;
                }
                out
            })
            .collect();
        let pn: Vec<u8> = PN_SEQUENCE.iter().flat_map(|&b| (0..8).rev().map(move |k| (b >> k) & 1)).collect();
        let mut p2_map = vec![Carrier::Data; C_PS];
        for i in (0..C_PS).step_by(3) {
            p2_map[i] = Carrier::P2Pilot;
        }
        for &k in &P2_PAPR_2K {
            p2_map[k] = Carrier::Reserved;
        }
        let (dx, dy) = p.pilots.dxdy();
        let mut fc_map = vec![Carrier::Data; C_PS];
        for i in (0..C_PS).step_by(dx) {
            fc_map[i] = Carrier::Scattered;
        }
        if p.pilots == Pilots::PP7 {
            fc_map[C_PS - 2] = Carrier::Scattered;
        }
        fc_map[0] = Carrier::Scattered;
        fc_map[C_PS - 1] = Carrier::Scattered;
        let cps: &[usize] = match p.pilots {
            Pilots::PP1 => &PP1_CP,
            Pilots::PP2 => &PP2_CP,
            Pilots::PP3 => &PP3_CP,
            Pilots::PP4 => &PP4_CP,
            Pilots::PP5 => &PP5_CP,
            Pilots::PP7 => &PP7_CP,
        };
        let data_maps = (0..p.symbols())
            .map(|symbol| {
                let mut m = vec![Carrier::Data; C_PS];
                for &k in cps {
                    m[k % 1632] = Carrier::Continual;
                }
                for (i, c) in m.iter_mut().enumerate() {
                    if i % (dx * dy) == dx * (symbol % dy) {
                        *c = Carrier::Scattered;
                    }
                }
                m[0] = Carrier::Scattered;
                m[C_PS - 1] = Carrier::Scattered;
                m
            })
            .collect();
        let sp = match p.pilots {
            Pilots::PP1 | Pilots::PP2 => 4.0 / 3.0,
            Pilots::PP3 | Pilots::PP4 => 7.0 / 4.0,
            Pilots::PP5 | Pilots::PP7 => 7.0 / 3.0,
        };
        let fft = FftPlanner::new().plan_fft_inverse(n);
        let p1 = p1_symbol(os4);
        Ofdm {
            p,
            prbs,
            pn,
            p2_map,
            fc_map,
            data_maps,
            sp,
            // 1K and 2K: 4/3 (4K: 4 sqrt2 / 3, 8K and up: 8/3).
            cp: 4.0 / 3.0,
            p2: (31f64.sqrt() / 5.0) as f32,
            norm: (5.0 / (27.0 * C_PS as f64).sqrt()) as f32,
            fft,
            p1,
            n,
            left: (n - C_PS) / 2 + 1,
            pal: Palette::new(&p),
        }
    }

    /// Symbol `j`'s carriers (0..1705): Some(value) for a pilot (0 for a
    /// reserved carrier), None for a data cell.
    pub fn plan(&self, j: usize) -> Vec<Option<Complex32>> {
        let (_, n_fc, _) = self.p.data_cells();
        let n = self.p.symbols();
        let (map, boost): (&[Carrier], f32) = if j < N_P2 {
            (&self.p2_map, self.p2)
        } else if n_fc != 0 && j == n - 1 {
            (&self.fc_map, self.sp)
        } else {
            (&self.data_maps[j], self.sp)
        };
        map.iter()
            .enumerate()
            .map(|(k, &c)| {
                let bit = self.prbs[k] ^ self.pn[j];
                let pilot = |a: f32| Some(Complex32::new(if bit == 0 { a } else { -a }, 0.0));
                match c {
                    Carrier::Data => None,
                    Carrier::Scattered | Carrier::P2Pilot => pilot(boost),
                    Carrier::Continual => pilot(self.cp),
                    Carrier::Reserved => Some(Complex32::default()),
                }
            })
            .collect()
    }

    /// FFT bin (of `self.n`) of carrier k, and the IFFT scaling.
    pub fn bin(&self, k: usize) -> usize {
        let b = self.left + k;
        let half = self.n / 2;
        if b >= half { b - half } else { b + half }
    }
    pub fn norm(&self) -> f32 {
        self.norm
    }
    pub fn p1(&self) -> &[Complex32] {
        &self.p1
    }

    /// Samples of one T2 frame at this oversampling.
    pub fn frame_samples(&self) -> usize {
        self.p.frame_samples() * self.n / FFT
    }

    /// One T2 frame from its frequency-interleaved symbols: P1, then each
    /// symbol with its guard interval, time domain at the elementary rate
    /// times the oversampling. The symbols are split over two threads (the
    /// IFFTs are most of the work on the A9).
    pub fn frame(&self, syms: &[Vec<Cell>], out: &mut Vec<Complex32>) {
        let n = self.p.symbols();
        assert_eq!(syms.len(), n);
        out.extend_from_slice(&self.p1);
        let nn = self.n;
        let gi = self.p.guard.samples() * nn / FFT;
        let start = out.len();
        out.resize(start + n * (nn + gi), Complex32::default());
        let (first, second) = out[start..].split_at_mut((n / 2) * (nn + gi));
        std::thread::scope(|sc| {
            sc.spawn(|| self.symbols(syms, 0, first));
            self.symbols(syms, n / 2, second);
        });
    }

    /// Symbol `j`'s 1705 carriers (data cells and pilots), before the IFFT.
    fn carriers(&self, syms: &[Vec<Cell>], j: usize, out: &mut [Complex32]) {
        let (_, n_fc, _) = self.p.data_cells();
        let n = self.p.symbols();
        let (map, boost): (&[Carrier], f32) = if j < N_P2 {
            (&self.p2_map, self.p2)
        } else if n_fc != 0 && j == n - 1 {
            (&self.fc_map, self.sp)
        } else {
            (&self.data_maps[j], self.sp)
        };
        let mut it = syms[j].iter();
        for ((k, &c), o) in map.iter().enumerate().zip(out.iter_mut()) {
            let bit = self.prbs[k] ^ self.pn[j];
            let pilot = |a: f32| Complex32::new(if bit == 0 { a } else { -a }, 0.0);
            *o = match c {
                Carrier::Data => self.pal.get(*it.next().expect("too few cells")),
                Carrier::Scattered | Carrier::P2Pilot => pilot(boost),
                Carrier::Continual => pilot(self.cp),
                Carrier::Reserved => Complex32::default(),
            };
        }
        assert!(it.next().is_none(), "symbol {j}: too many cells");
    }

    /// One T2 frame for the FPGA's transmit IFFT (maia-hdl t2ifft.py), as
    /// the DMA's 16-bit I/Q words (I low, little-endian): the sync word, P1
    /// (samples x `scale` x 256, as the 8-bit samples of [`Self::frame`]
    /// times `scale` became in the FPGA), then each symbol's carriers in its
    /// bin order (bins 0..852: carriers 852..1704; bins 1196..2047: carriers
    /// 0..851) scaled so that the FPGA's sum(bins) / 8 gives the same
    /// samples. 2K without oversampling only.
    pub fn frame_fpga(&self, syms: &[Vec<Cell>], scale: f32, out: &mut Vec<u8>) {
        assert_eq!(self.n, FFT, "the FPGA's IFFT is 2K at the elementary rate");
        let n = self.p.symbols();
        assert_eq!(syms.len(), n);
        let word = |re: f32, im: f32| -> [u8; 4] {
            let q = |v: f32| (v + if v >= 0.0 { 0.5 } else { -0.5 }).clamp(-32767.0, 32767.0) as i16;
            let (a, b) = (q(re).to_le_bytes(), q(im).to_le_bytes());
            [a[0], a[1], b[0], b[1]]
        };
        // sync: I = 0x7FFF, Q = -0x7FFF
        out.extend_from_slice(&word(32767.0, -32767.0));
        let g1 = scale * 256.0;
        for z in &self.p1 {
            out.extend_from_slice(&word(z.re * g1, z.im * g1));
        }
        // The words, scaled once: every cell code, and the pilots' values
        // (a table lookup a carrier instead of a multiply and two roundings:
        // this took 90 ms a frame on the A9).
        let g = self.norm * scale * 2048.0;
        let data: Vec<[u8; 4]> = self.pal.0.iter().map(|z| word(z.re * g, z.im * g)).collect();
        let pil = |a: f32| [word(a * g, 0.0), word(-a * g, 0.0)];
        let (p2, sp, cp) = (pil(self.p2), pil(self.sp), pil(self.cp));
        let zero = word(0.0, 0.0);
        let (_, n_fc, _) = self.p.data_cells();
        let mut sym = vec![[0u8; 4]; C_PS];
        let low = C_PS / 2; // 852: bins 0..852 hold carriers 852..1704
        let start = out.len();
        out.resize(start + n * C_PS * 4, 0);
        let mut at = start;
        for j in 0..n {
            let (map, boost) = if j < N_P2 {
                (&self.p2_map, &p2)
            } else if n_fc != 0 && j == n - 1 {
                (&self.fc_map, &sp)
            } else {
                (&self.data_maps[j], &sp)
            };
            let mut it = syms[j].iter();
            for (k, (&c, w)) in map.iter().zip(sym.iter_mut()).enumerate() {
                let neg = (self.prbs[k] ^ self.pn[j]) as usize;
                *w = match c {
                    Carrier::Data => data[*it.next().expect("too few cells") as usize],
                    Carrier::Scattered | Carrier::P2Pilot => boost[neg],
                    Carrier::Continual => cp[neg],
                    Carrier::Reserved => zero,
                };
            }
            assert!(it.next().is_none(), "symbol {j}: too many cells");
            for w in sym[low..].iter().chain(&sym[..low]) {
                out[at..at + 4].copy_from_slice(w);
                at += 4;
            }
        }
    }

    /// [`Self::frame_fpga`]'s carrier order for any cell type: every
    /// symbol's 1705 carriers in bin order, data from `syms`, pilots as
    /// `pilot(kind, negative)` (kind 0: P2, 1: scattered, 2: continual),
    /// reserved carriers `zero`.
    pub fn fpga_slots<T: Copy>(&self, syms: &[Vec<T>], pilot: &dyn Fn(usize, usize) -> T, zero: T) -> Vec<T> {
        let n = self.p.symbols();
        assert_eq!(syms.len(), n);
        let (_, n_fc, _) = self.p.data_cells();
        let low = C_PS / 2;
        let mut out = Vec::with_capacity(n * C_PS);
        let mut sym = vec![zero; C_PS];
        for j in 0..n {
            let (map, boost) = if j < N_P2 {
                (&self.p2_map, 0)
            } else if n_fc != 0 && j == n - 1 {
                (&self.fc_map, 1)
            } else {
                (&self.data_maps[j], 1)
            };
            let mut it = syms[j].iter();
            for (k, (&c, w)) in map.iter().zip(sym.iter_mut()).enumerate() {
                let neg = (self.prbs[k] ^ self.pn[j]) as usize;
                *w = match c {
                    Carrier::Data => *it.next().expect("too few cells"),
                    Carrier::Scattered | Carrier::P2Pilot => pilot(boost, neg),
                    Carrier::Continual => pilot(2, neg),
                    Carrier::Reserved => zero,
                };
            }
            assert!(it.next().is_none(), "symbol {j}: too many cells");
            out.extend_from_slice(&sym[low..]);
            out.extend_from_slice(&sym[..low]);
        }
        out
    }

    /// The DMA words [`Self::frame_fpga`] sends: sync, P1 at `scale`, and
    /// per code (the palette's, then 263 + 2 kind + negative for pilots,
    /// as [`Self::fpga_slots`]) its carrier word.
    pub fn fpga_words(&self, scale: f32) -> (Vec<[u8; 4]>, Vec<[u8; 4]>) {
        let word = |re: f32, im: f32| -> [u8; 4] {
            let q = |v: f32| (v + if v >= 0.0 { 0.5 } else { -0.5 }).clamp(-32767.0, 32767.0) as i16;
            let (a, b) = (q(re).to_le_bytes(), q(im).to_le_bytes());
            [a[0], a[1], b[0], b[1]]
        };
        let mut head = vec![word(32767.0, -32767.0)];
        let g1 = scale * 256.0;
        head.extend(self.p1.iter().map(|z| word(z.re * g1, z.im * g1)));
        let g = self.norm * scale * 2048.0;
        let mut words: Vec<[u8; 4]> = self.pal.0.iter().map(|z| word(z.re * g, z.im * g)).collect();
        for a in [self.p2, self.sp, self.cp] {
            words.push(word(a * g, 0.0));
            words.push(word(-a * g, 0.0));
        }
        (head, words)
    }

    /// Symbols `from..` into `out` (guard interval, then the symbol).
    fn symbols(&self, syms: &[Vec<Cell>], from: usize, out: &mut [Complex32]) {
        let (_, n_fc, _) = self.p.data_cells();
        let n = self.p.symbols();
        let nn = self.n;
        let gi = self.p.guard.samples() * nn / FFT;
        let mut buf = vec![Complex32::default(); nn];
        let mut scratch = vec![Complex32::default(); self.fft.get_inplace_scratch_len()];
        for (i, slot) in out.chunks_exact_mut(nn + gi).enumerate() {
            let j = from + i;
            let cells = &syms[j];
            let (map, boost): (&[Carrier], f32) = if j < N_P2 {
                (&self.p2_map, self.p2)
            } else if n_fc != 0 && j == n - 1 {
                (&self.fc_map, self.sp)
            } else {
                (&self.data_maps[j], self.sp)
            };
            // Carrier k sits at left + k; swapping halves puts DC at 0.
            buf.fill(Complex32::default());
            let half = nn / 2;
            let mut it = cells.iter();
            for (k, &c) in map.iter().enumerate() {
                let bit = self.prbs[k] ^ self.pn[j];
                let pilot = |a: f32| Complex32::new(if bit == 0 { a } else { -a }, 0.0);
                let v = match c {
                    Carrier::Data => self.pal.get(*it.next().expect("too few cells")),
                    Carrier::Scattered | Carrier::P2Pilot => pilot(boost),
                    Carrier::Continual => pilot(self.cp),
                    Carrier::Reserved => Complex32::default(),
                };
                let bin = self.left + k;
                buf[if bin >= half { bin - half } else { bin + half }] = v;
            }
            assert!(it.next().is_none(), "symbol {j}: too many cells");
            self.fft.process_with_scratch(&mut buf, &mut scratch);
            for (o, z) in slot[gi..].iter_mut().zip(&buf) {
                *o = z * self.norm;
            }
            let (g, body) = slot.split_at_mut(gi);
            g.copy_from_slice(&body[nn - gi..]);
        }
    }
}

/// The P1 symbol (T2 SISO, 2K): C-A-B, 2048 elementary periods, sampled
/// at `os4 / 4` samples a period. A is the 1K IFFT of the 384 DBPSK
/// carriers (scaled by 1/sqrt 384); C (542 periods) and B (482) are A's
/// first and last parts shifted up one carrier, as gr-dtv builds them. The
/// waveform is evaluated directly, so any oversampling works.
fn p1_symbol(os4: usize) -> Vec<Complex32> {
    let (s1, s2) = (0usize, 0usize << 1); // preamble T2 SISO; S2 = (fft & 7) << 1, 2K = 0
    let mut seq = Vec::with_capacity(384);
    for &b in S1_PATTERNS[s1].iter() {
        seq.extend((0..8).rev().map(|j| (b >> j) & 1));
    }
    for &b in S2_PATTERNS[s2].iter() {
        seq.extend((0..8).rev().map(|j| (b >> j) & 1));
    }
    for &b in S1_PATTERNS[s1].iter() {
        seq.extend((0..8).rev().map(|j| (b >> j) & 1));
    }
    let mut dbpsk = vec![0i32; 385];
    dbpsk[0] = 1;
    for i in 1..385 {
        dbpsk[i] = if seq[i - 1] == 1 { -dbpsk[i - 1] } else { dbpsk[i - 1] };
    }
    let mut sr: u32 = 0x4e46;
    let rnd: Vec<i32> = (0..384)
        .map(|_| {
            let b = (sr ^ (sr >> 1)) & 1;
            sr >>= 1;
            if b != 0 {
                sr |= 0x4000;
            }
            if b == 0 { 1 } else { -1 }
        })
        .collect();
    // Carriers: bin P1_ACTIVE[i] + 86 of 1024, DC at 512.
    let carriers: Vec<(f64, f64)> = (0..384).map(|i| ((P1_ACTIVE[i] + 86) as f64 - 512.0, (dbpsk[i + 1] * rnd[i]) as f64)).collect();
    let wave = |t: f64, shift: f64| -> Complex32 {
        let mut z = num_complex::Complex64::default();
        for &(k, a) in &carriers {
            z += num_complex::Complex64::from_polar(a, std::f64::consts::TAU * (k + shift) * t / 1024.0);
        }
        let z = z / 384f64.sqrt();
        Complex32::new(z.re as f32, z.im as f32)
    };
    (0..2048 * os4 / 4)
        .map(|n| {
            let t = n as f64 * 4.0 / os4 as f64;
            if t < 542.0 {
                wave(t, 1.0)
            } else if t < 1566.0 {
                wave(t - 542.0, 0.0)
            } else {
                wave(t - 1024.0, 1.0)
            }
        })
        .collect()
}
