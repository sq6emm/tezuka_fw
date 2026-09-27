//! DVB-T2 receiver for this modulator's profile (2K, SISO, one PLP, the
//! parameters known in advance), streaming: samples at the elementary rate
//! in (from the FPGA's T2 resampler), one LLR vector per FEC block out for
//! the DVB-S2 decoder chain (the same LDPC codes, BCH, BBFRAME).
//!
//! - Acquisition: P1 by its structure (C and B are frequency-shifted copies
//!   of parts of A, 1024 and 482 samples away: two running delayed
//!   correlations over a frame), then the known P1 waveform correlated in
//!   16 coherent chunks around the peak for the exact start and a coarse
//!   frequency.
//! - Locked: each frame's P1 again near where it should be (structure
//!   within +-64 samples, the waveform within +-4 of that); two misses in a
//!   row and it searches again.
//! - Symbol by symbol, so a frame's worth never has to sit in the A9's
//!   caches: once the 8 P2 symbols are in, the frequency from their guard
//!   intervals (and the previous frame's), their FFTs, the channel from
//!   their pilots (every third carrier, averaged); then each symbol as it
//!   arrives: FFT, equalization with its own common phase and phase slope
//!   (timing drift) from its pilots, the data cells as 8-bit pairs. At the
//!   frame's end one gather undoes the frequency interleaver, the frame
//!   layout and the cell interleaver; LLRs per FEC block.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

use super::frame::{FrameMapper, FreqInterleaver};
use super::ofdm::Ofdm;
use super::{BitInterleaver, CellInterleaver, Constellation, Params, FFT, N_P2};

const P1_LEN: usize = 2048;
/// Frame to frame, P1 is looked for this far either side of where it
/// should be.
const TRACK: usize = 64;
/// Known-P1 correlation (0..1) above which a P1 counts as found.
const P1_OK: f32 = 0.2;
/// Equalized cells to 8 bits: unit amplitude = CELL_SCALE.
const CELL_SCALE: f32 = 40.0;
const CARRIERS: usize = 1705;

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub frames: u64,
    /// P1 not found where the frame timing put it.
    pub p1_missed: u64,
    pub locked: bool,
    /// MER of the last frame's L1-pre cells (BPSK, known), dB.
    pub mer_db: f32,
    /// Carrier offset from the nominal (`set_center`), Hz.
    pub freq_hz: f32,
    /// FEC blocks handed on.
    pub blocks: u64,
}

pub const PROF_NAMES: [&str; 6] = ["p1", "fft", "equalize", "deinterleave", "llr", "input"];

enum State {
    Search,
    /// In a frame whose P1 starts at `start` (absolute sample index);
    /// `sym` is the next symbol to take (0: the P2 symbols, together).
    Frame { start: u64, coarse: f64, sym: usize },
}

pub struct Demod {
    p: Params,
    fs: f64,
    bi: BitInterleaver,
    fft: Arc<dyn Fft<f32>>,
    p1: Vec<Complex32>,
    p1_energy: f32,
    /// Each symbol's pilots: (carrier, value).
    pilots: Vec<Vec<(usize, Complex32)>>,
    /// Each symbol's data carriers.
    data: Vec<Vec<usize>>,
    /// Where each symbol's data cells start in `flat`.
    data_at: Vec<usize>,
    pre_ref: Vec<f32>,
    bins: Vec<usize>,
    /// Per carrier: undoes the FFT window's early start, and the scaling.
    early_rot: Vec<Complex32>,
    early: usize,
    /// P1 rotation table e^(-j 2 pi m / 1024).
    shift: Vec<Complex32>,
    /// P1 moved to the signal's nominal offset (`center_hz`).
    p1c: Vec<Complex32>,
    /// Flat data-cell index (symbols' data carriers in order) of each
    /// FEC block's cells and of the L1-pre cells: the frequency
    /// deinterleaver, the frame's layout and the cell deinterleaver in one.
    gather: Vec<Vec<u32>>,
    pre_gather: Vec<u32>,
    /// The frame's data cells, equalized, 8-bit I/Q.
    flat: Vec<[i8; 2]>,
    /// Samples from absolute index `base` on.
    buf: Vec<Complex32>,
    base: u64,
    /// The signal's nominal offset from the input's centre (the LO sits
    /// off the signal): P1's reference is moved there, the derotation takes
    /// it out.
    center_hz: f64,
    state: State,
    misses: u32,
    /// This frame's derotation (rad a sample) and channel inverse.
    w: f64,
    hinv: Vec<Complex32>,
    /// Guard-interval correlation of the previous frame's data symbols.
    cp_prev: Complex32,
    cp_acc: Complex32,
    work: Vec<Complex32>,
    scratch: Vec<Complex32>,
    carriers: Vec<Complex32>,
    pub stats: Stats,
    /// Some equalized data cells of the last frame, for the browser.
    pub constellation: Vec<[i8; 2]>,
    pub constellation_seq: u64,
    /// Seconds spent per stage ([`PROF_NAMES`]).
    pub prof: [f64; 6],
}

impl Demod {
    pub fn new(p: Params, fs: f64) -> Demod {
        let ofdm = Ofdm::new(p);
        let fm = FrameMapper::new(p);
        let nsym = p.symbols();
        let plans: Vec<Vec<Option<Complex32>>> = (0..nsym).map(|j| ofdm.plan(j)).collect();
        let pilots = plans
            .iter()
            .map(|pl| pl.iter().enumerate().filter_map(|(k, v)| v.filter(|z| z.norm() > 0.0).map(|z| (k, z))).collect())
            .collect();
        let data: Vec<Vec<usize>> =
            plans.iter().map(|pl| pl.iter().enumerate().filter(|(_, v)| v.is_none()).map(|(k, _)| k).collect()).collect();
        let mut data_at = Vec::with_capacity(nsym + 1);
        let mut at = 0;
        for d in &data {
            data_at.push(at);
            at += d.len();
        }
        data_at.push(at);
        let pre_ref = fm.pre_cells().iter().map(|&c| if c == super::BPSK0 { 1.0 } else { -1.0 }).collect();
        let gi = p.guard.samples();
        let early = gi / 4;
        let scale = 1.0 / (FFT as f32 * ofdm.norm());
        let bins: Vec<usize> = (0..CARRIERS).map(|k| ofdm.bin(k)).collect();
        // A window `early` samples before the symbol: carrier k turns by
        // exp(-j 2 pi (bin freq) early / N); undo it.
        let early_rot = bins
            .iter()
            .map(|&b| {
                let f = if b >= FFT / 2 { b as f64 - FFT as f64 } else { b as f64 };
                let t = std::f64::consts::TAU * f * early as f64 / FFT as f64;
                Complex32::new(t.cos() as f32, t.sin() as f32) * scale
            })
            .collect();
        let p1: Vec<Complex32> = ofdm.p1().to_vec();
        let p1_energy = p1.iter().map(|z| z.norm_sqr()).sum();
        let shift = (0..1024)
            .map(|m| {
                let t = -std::f64::consts::TAU * m as f64 / 1024.0;
                Complex32::new(t.cos() as f32, t.sin() as f32)
            })
            .collect();
        // The deinterleavers applied to indices once.
        let fi = FreqInterleaver::new(&p);
        let ci = CellInterleaver::new(&p);
        let idx: Vec<Vec<u32>> = (0..nsym).map(|j| (data_at[j] as u32..data_at[j + 1] as u32).collect()).collect();
        let cells = fi.unframe(&idx);
        let (pre_gather, _post, dcells) = fm.unmap(&cells);
        let gather = ci.deinterleave(&dcells, p.fec_blocks);
        let fft = FftPlanner::new().plan_fft_forward(FFT);
        let scratch = vec![Complex32::default(); fft.get_inplace_scratch_len()];
        Demod {
            p,
            fs,
            bi: BitInterleaver::new(&p),
            fft,
            p1c: p1.clone(),
            p1,
            p1_energy,
            pilots,
            data,
            flat: vec![[0; 2]; at],
            data_at,
            pre_ref,
            bins,
            early_rot,
            early,
            shift,
            gather,
            pre_gather,
            buf: Vec::new(),
            base: 0,
            center_hz: 0.0,
            state: State::Search,
            misses: 0,
            w: 0.0,
            hinv: vec![Complex32::default(); CARRIERS],
            cp_prev: Complex32::default(),
            cp_acc: Complex32::default(),
            work: vec![Complex32::default(); FFT],
            scratch,
            carriers: vec![Complex32::default(); N_P2 * CARRIERS],
            stats: Stats::default(),
            constellation: Vec::new(),
            constellation_seq: 0,
            prof: [0.0; 6],
        }
    }

    /// The signal sits `hz` from the centre of the samples pushed.
    pub fn set_center(&mut self, hz: f64) {
        if (hz - self.center_hz).abs() < 0.5 {
            return;
        }
        self.center_hz = hz;
        let w = std::f64::consts::TAU * hz / self.fs;
        self.p1c = self.p1.iter().enumerate().map(|(i, &z)| z * Complex32::from_polar(1.0, (w * i as f64) as f32)).collect();
    }

    fn end(&self) -> u64 {
        self.base + self.buf.len() as u64
    }

    fn at(&self, abs: u64) -> usize {
        (abs - self.base) as usize
    }

    /// Drop samples before `abs` (in large steps: the move costs).
    fn release(&mut self, abs: u64) {
        let k = abs.saturating_sub(self.base) as usize;
        if k > 200_000 || k == self.buf.len() {
            let k = k.min(self.buf.len());
            self.buf.drain(..k);
            self.base += k as u64;
        }
    }

    /// Samples in; the FEC blocks of every frame completed, as LLRs
    /// (positive = 0, 64800 each), onto `out`.
    pub fn push(&mut self, x: &[Complex32], out: &mut Vec<Vec<f32>>) {
        let t_in = std::time::Instant::now();
        self.buf.extend_from_slice(x);
        self.prof[5] += t_in.elapsed().as_secs_f64();
        let frame = self.p.frame_samples() as u64;
        let gi = self.p.guard.samples() as u64;
        let sym_len = FFT as u64 + gi;
        let nsym = self.p.symbols();
        loop {
            match self.state {
                State::Search => {
                    if self.end() < self.base + frame + (P1_LEN + 1024) as u64 {
                        return;
                    }
                    let t0 = std::time::Instant::now();
                    let peak = self.structure_peak(self.base, self.base + frame);
                    let found = self.p1_near(peak, 32).filter(|f| f.2 >= P1_OK);
                    self.prof[0] += t0.elapsed().as_secs_f64();
                    match found {
                        Some((s, coarse, _)) => {
                            self.state = State::Frame { start: s, coarse, sym: 0 };
                            self.misses = 0;
                            self.cp_prev = Complex32::default();
                            self.stats.locked = true;
                        }
                        None => {
                            self.stats.locked = false;
                            self.buf.drain(..frame as usize);
                            self.base += frame;
                        }
                    }
                }
                State::Frame { start, coarse, sym } if sym == 0 => {
                    // The P2 symbols, all of them.
                    let s0 = start + P1_LEN as u64;
                    if self.end() < s0 + N_P2 as u64 * sym_len {
                        return;
                    }
                    self.p2(start, coarse);
                    self.state = State::Frame { start, coarse, sym: N_P2 };
                }
                State::Frame { start, coarse, sym } if sym < nsym => {
                    let s = start + P1_LEN as u64 + sym as u64 * sym_len;
                    if self.end() < s + sym_len {
                        return;
                    }
                    self.data_symbol(start, sym);
                    self.state = State::Frame { start, coarse, sym: sym + 1 };
                }
                State::Frame { start, coarse, .. } => {
                    // The frame is in: decode it, then find the next P1.
                    let predicted = start + frame;
                    if self.end() < predicted + (TRACK + P1_LEN + 8) as u64 {
                        return;
                    }
                    self.finish(out);
                    let t0 = std::time::Instant::now();
                    let peak = self.structure_peak(predicted - TRACK as u64, predicted + TRACK as u64);
                    let found = self.p1_near(peak, 4).filter(|f| f.2 >= P1_OK);
                    self.prof[0] += t0.elapsed().as_secs_f64();
                    let next = match found {
                        Some((s, c, _)) => {
                            self.misses = 0;
                            Some((s, c))
                        }
                        None => {
                            self.misses += 1;
                            self.stats.p1_missed += 1;
                            (self.misses < 2).then_some((predicted, coarse))
                        }
                    };
                    match next {
                        Some((s, c)) => {
                            self.cp_prev = self.cp_acc;
                            self.state = State::Frame { start: s, coarse: c, sym: 0 };
                            self.release(s - TRACK as u64);
                        }
                        None => {
                            self.state = State::Search;
                            self.stats.locked = false;
                            self.release(predicted);
                        }
                    }
                }
            }
        }
    }

    /// One symbol's carriers (0..1705) into `self.carriers[slot]`: derotated,
    /// FFT a few samples early into the guard interval.
    fn fft_symbol(&mut self, start: u64, j: usize, slot: usize) {
        let gi = self.p.guard.samples() as u64;
        let s = start + P1_LEN as u64 + j as u64 * (FFT as u64 + gi) + gi - self.early as u64;
        let ph = (s as f64 * self.w).rem_euclid(std::f64::consts::TAU);
        let mut r = Complex32::new(ph.cos() as f32, ph.sin() as f32);
        let step = Complex32::new(self.w.cos() as f32, self.w.sin() as f32);
        let i = self.at(s);
        for (o, &v) in self.work.iter_mut().zip(&self.buf[i..i + FFT]) {
            *o = v * r;
            r *= step;
        }
        self.fft.process_with_scratch(&mut self.work, &mut self.scratch);
        let c = &mut self.carriers[slot * CARRIERS..(slot + 1) * CARRIERS];
        for ((o, &b), &e) in c.iter_mut().zip(&self.bins).zip(&self.early_rot) {
            *o = self.work[b] * e;
        }
    }

    /// Guard-interval correlation of symbol `j` (its prefix against the
    /// symbol's end).
    fn gi_corr(&self, start: u64, j: usize) -> Complex32 {
        let gi = self.p.guard.samples();
        let i = self.at(start + (P1_LEN + j * (FFT + gi)) as u64);
        let mut cp = Complex32::default();
        for n in 0..gi {
            cp += self.buf[i + n].conj() * self.buf[i + n + FFT];
        }
        cp
    }

    /// The P2 symbols: frequency, FFTs, channel, their cells.
    fn p2(&mut self, start: u64, coarse: f64) {
        let t0 = std::time::Instant::now();
        let fs = self.fs;
        // Frequency: the P2 symbols' guard intervals and the previous
        // frame's (modulo a carrier spacing; P1's coarse estimate picks the
        // multiple).
        let mut cp = self.cp_prev;
        for j in 0..N_P2 {
            cp += self.gi_corr(start, j);
        }
        self.cp_acc = Complex32::default();
        let spacing = fs / FFT as f64;
        let frac = cp.arg() as f64 / (std::f64::consts::TAU * FFT as f64) * fs;
        let f_off = frac + ((coarse - frac) / spacing).round() * spacing;
        self.stats.freq_hz = (f_off - self.center_hz) as f32;
        self.w = -std::f64::consts::TAU * f_off / fs;
        for j in 0..N_P2 {
            self.fft_symbol(start, j, j);
        }
        self.prof[1] += t0.elapsed().as_secs_f64();
        let t0 = std::time::Instant::now();
        // Channel from the P2 pilots, linear between them, averaged. Each
        // carrier once: [k0, k1) per pair of neighbouring pilots, the last
        // pilot itself at the end.
        let mut h = vec![Complex32::default(); CARRIERS];
        for j in 0..N_P2 {
            let c = &self.carriers[j * CARRIERS..(j + 1) * CARRIERS];
            let pil: Vec<(usize, Complex32)> = self.pilots[j].iter().map(|&(k, z)| (k, c[k] / z)).collect();
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
        for (hi, &v) in self.hinv.iter_mut().zip(&h) {
            *hi = if v.norm_sqr() > 0.0 { v.inv() } else { Complex32::default() };
        }
        for j in 0..N_P2 {
            self.equalize(j, j);
        }
        self.prof[2] += t0.elapsed().as_secs_f64();
    }

    fn data_symbol(&mut self, start: u64, j: usize) {
        let t0 = std::time::Instant::now();
        self.cp_acc += self.gi_corr(start, j);
        self.fft_symbol(start, j, 0);
        self.prof[1] += t0.elapsed().as_secs_f64();
        let t0 = std::time::Instant::now();
        self.equalize(j, 0);
        self.prof[2] += t0.elapsed().as_secs_f64();
    }

    /// Symbol `j`'s data cells (carriers in `slot`), equalized with its own
    /// common phase and phase slope across the carriers (timing drifting
    /// against the P2 symbols) from its pilots, into `flat`.
    fn equalize(&mut self, j: usize, slot: usize) {
        let cj = &self.carriers[slot * CARRIERS..(slot + 1) * CARRIERS];
        let hinv = &self.hinv;
        let z: Vec<(usize, Complex32)> = self.pilots[j].iter().map(|&(k, r)| (k, cj[k] * hinv[k] * r.conj())).collect();
        // Slope from neighbouring pilots at the commonest spacing.
        let mut hist = [0u32; 32];
        for w2 in z.windows(2) {
            let d = w2[1].0 - w2[0].0;
            if d < 32 {
                hist[d] += 1;
            }
        }
        let dk = (1..32).max_by_key(|&d| hist[d]).unwrap_or(1);
        let mut ds = Complex32::default();
        for w2 in z.windows(2) {
            if w2[1].0 - w2[0].0 == dk {
                ds += w2[1].1 * w2[0].1.conj();
            }
        }
        let slope = if ds.norm() > 0.0 { ds.arg() / dk as f32 } else { 0.0 };
        let step = Complex32::from_polar(1.0, -slope);
        let mut c = Complex32::default();
        let (mut r, mut kk) = (Complex32::new(1.0, 0.0), 0usize);
        for &(k, v) in &z {
            while kk < k {
                r *= step;
                kk += 1;
            }
            c += v * r;
        }
        let a = if c.norm() > 0.0 { c.arg() } else { 0.0 };
        let (mut r, mut kk) = (Complex32::from_polar(CELL_SCALE, -a), 0usize);
        let q = |v: f32| v.round().clamp(-127.0, 127.0) as i8;
        let out = &mut self.flat[self.data_at[j]..self.data_at[j + 1]];
        for (o, &k) in out.iter_mut().zip(&self.data[j]) {
            while kk < k {
                r *= step;
                kk += 1;
            }
            let v = cj[k] * hinv[k] * r;
            *o = [q(v.re), q(v.im)];
        }
    }

    /// The frame's cells are all in: MER, FEC blocks' LLRs.
    fn finish(&mut self, out: &mut Vec<Vec<f32>>) {
        let t0 = std::time::Instant::now();
        let p = self.p;
        let flat = &self.flat;
        let cell = |i: u32| {
            let v = flat[i as usize];
            Complex32::new(v[0] as f32, v[1] as f32) / CELL_SCALE
        };
        // The browser's constellation: a data symbol's cells.
        let off = self.data_at[N_P2 + 5];
        let q = |v: f32| (v * 56.0 / CELL_SCALE).round().clamp(-127.0, 127.0) as i8;
        self.constellation = flat[off..].iter().step_by(6).take(256).map(|z| [q(z[0] as f32), q(z[1] as f32)]).collect();
        self.constellation_seq += 1;
        let (mut sig, mut err) = (0f32, 0f32);
        for (&i, &r) in self.pre_gather.iter().zip(&self.pre_ref) {
            sig += r * r;
            err += (cell(i) - Complex32::new(r, 0.0)).norm_sqr();
        }
        self.stats.mer_db = 10.0 * (sig / err.max(1e-12)).log10();
        self.stats.frames += 1;
        let sigma2 = (err / self.pre_gather.len() as f32).max(1e-6);
        // Rotated constellations: word j's I is in cell j, its Q in cell
        // j + 1 (cyclically in the block); rotate back.
        let angle: f32 = match p.constellation {
            Constellation::Qpsk => 29.0,
            Constellation::Qam16 => 16.8,
        };
        let derot = Complex32::from_polar(1.0, -angle.to_radians());
        self.prof[3] += t0.elapsed().as_secs_f64();
        for g in &self.gather {
            let t0 = std::time::Instant::now();
            let blk: Vec<Complex32> = g.iter().map(|&i| cell(i)).collect();
            self.prof[3] += t0.elapsed().as_secs_f64();
            let t0 = std::time::Instant::now();
            let n = blk.len();
            let z = |j: usize| if p.rotation { Complex32::new(blk[j].re, blk[(j + 1) % n].im) * derot } else { blk[j] };
            let cell_llr: Vec<f32> = match p.constellation {
                Constellation::Qpsk => {
                    let s = 2.0 * std::f32::consts::FRAC_1_SQRT_2 * 2.0 / sigma2;
                    (0..n)
                        .flat_map(|j| {
                            let z = z(j);
                            [z.re * s, z.im * s]
                        })
                        .collect()
                }
                Constellation::Qam16 => {
                    // Word bits 3, 2: signs of I, Q (0 positive); 1, 0: outer
                    // (0) or inner level. Max-log, per axis.
                    let a = 1.0 / 10f32.sqrt();
                    let s = 4.0 * a / sigma2;
                    (0..n)
                        .flat_map(|j| {
                            let z = z(j);
                            [z.re * s, z.im * s, (z.re.abs() - 2.0 * a) * s, (z.im.abs() - 2.0 * a) * s]
                        })
                        .collect()
                }
            };
            out.push(self.bi.deinterleave_llr(&cell_llr));
            self.stats.blocks += 1;
            self.prof[4] += t0.elapsed().as_secs_f64();
        }
    }

    /// The known P1 (at the nominal offset) within `r` samples of `at`
    /// (absolute): its start, the carrier's offset from the input's centre
    /// (coarse), the match.
    fn p1_near(&self, at: u64, r: usize) -> Option<(u64, f64, f32)> {
        let a = self.at(at.max(self.base));
        let (s, c, q) = find_p1_in(&self.buf, &self.p1c, self.p1_energy, a.saturating_sub(r), a + r + 1, self.fs)?;
        Some((self.base + s as u64, c + self.center_hz, q))
    }

    /// Start of the best P1 by structure among `from..=to` (absolute; offset
    /// blind).
    fn structure_peak(&self, from: u64, to: u64) -> u64 {
        let x = &self.buf;
        let from = from.max(self.base);
        let base = self.at(from);
        let n = (to - from) as usize + P1_LEN;
        // u[m] = x[m] conj(x[m + 1024]) e^(-j 2 pi m / 1024): C against the
        // A it copies; v[m] = x[m] conj(x[m - 482]) e^(..): B against A.
        // Summed over C's 542 / B's 482 samples: a common phase at the P1.
        // Prefix sums in f64: a frame's worth of them in f32 loses the
        // small differences taken below.
        type C64 = num_complex::Complex64;
        let rot = |m: usize| self.shift[m & 1023];
        let mut cu = vec![C64::default(); n + 1];
        let mut cv = vec![C64::default(); n + 1];
        let mut ce = vec![0f64; n + 1];
        for i in 0..n {
            let m = base + i;
            let u = if m + 1024 < x.len() { x[m] * x[m + 1024].conj() * rot(m) } else { Complex32::default() };
            let v = if m >= 482 { x[m] * x[m - 482].conj() * rot(m) } else { Complex32::default() };
            cu[i + 1] = cu[i] + C64::new(u.re as f64, u.im as f64);
            cv[i + 1] = cv[i] + C64::new(v.re as f64, v.im as f64);
            ce[i + 1] = ce[i] + x.get(m).map_or(0.0, |z| z.norm_sqr() as f64);
        }
        let mut best = (0usize, 0f64);
        for s in 0..=(to - from) as usize {
            let c = (cu[s + 542] - cu[s]).norm();
            let b = (cv[s + 2048] - cv[s + 1566]).norm();
            let e = ce[s + 2048] - ce[s];
            let m = (c + b) / e.max(1e-30);
            if m > best.1 {
                best = (s, m);
            }
        }
        from + best.0 as u64
    }
}

/// The known P1 correlated at start positions `from..to` in 16 coherent
/// chunks of 128 samples (magnitudes summed: a few kHz of offset do not
/// matter): the best start, a coarse frequency from the chunks' phase steps,
/// and the match (0..1, normalized by both energies).
pub fn find_p1_in(x: &[Complex32], p1: &[Complex32], p1_energy: f32, from: usize, to: usize, fs: f64) -> Option<(usize, f64, f32)> {
    const CH: usize = 128;
    let to = to.min(x.len().saturating_sub(p1.len()) + 1);
    if from >= to {
        return None;
    }
    let chunks = |t: usize| -> [Complex32; 16] {
        let mut c = [Complex32::default(); 16];
        for (ci, acc) in c.iter_mut().enumerate() {
            for i in ci * CH..(ci + 1) * CH {
                *acc += x[t + i] * p1[i].conj();
            }
        }
        c
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
    let e: f32 = x[best.0..best.0 + p1.len()].iter().map(|z| z.norm_sqr()).sum();
    let q = best.1 / (e * p1_energy).sqrt().max(1e-20);
    Some((best.0, d.arg() as f64 / (std::f64::consts::TAU * CH as f64) * fs, q))
}
