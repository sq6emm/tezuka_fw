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
/// ... when tracking (within +-4 samples of where it should be).
const P1_TRACK_OK: f32 = 0.15;
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
    /// Places the input had words missing (the ring reader was behind).
    pub gaps: u64,
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
    /// The same the other way: for each flat cell, its place in `cells`
    /// (FEC block * block cells + cell), or u32::MAX (L1, dummy cells).
    scatter: Vec<u32>,
    /// The frame's FEC block cells, cell-interleaved (time deinterleaved
    /// only), 8-bit I/Q.
    cells: Vec<[i8; 2]>,
    /// One FEC block's cells in order (the cell deinterleaver's output).
    blk: Vec<[i8; 2]>,
    ci: CellInterleaver,
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
    /// hinv with the early window's rotation (for raw carriers).
    hinv_e: Vec<Complex32>,
    /// hinv_e x CELL_SCALE in fixed point (2^g_shift), for equalize_int.
    g_int: Vec<[i32; 2]>,
    g_shift: u32,
    /// Guard-interval correlation of the previous frame's data symbols.
    cp_prev: Complex32,
    cp_acc: Complex32,
    work: Vec<Complex32>,
    scratch: Vec<Complex32>,
    carriers: Vec<Complex32>,
    /// With the FPGA's OFDM front end ([`Self::push_words`]).
    fe: Option<Box<FeState>>,
    /// Carrier index of each carrier word of an FPGA FFT.
    order: Vec<usize>,
    pub stats: Stats,
    /// Some equalized data cells of the last frame, for the browser.
    pub constellation: Vec<[i8; 2]>,
    pub constellation_seq: u64,
    /// Seconds spent per stage ([`PROF_NAMES`]).
    pub prof: [f64; 6],
    /// Debug (T2SYMERR set): pilot error and power by symbol index after
    /// the equalizer, to see the channel estimate age through a frame.
    pub sym_err: Option<Vec<(f64, f64)>>,
    sym_err_frame: Vec<(f64, f64)>,
    /// Debug (T2SYMERR): pilot error and count by carrier (64-carrier buckets).
    pub car_err: Option<Vec<(f64, f64)>>,
    /// This frame's pilot error and count (the MER shown).
    pil_err: (f64, f64),
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
        let order = super::fe::carrier_order(&bins);
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
        // Cells go only through the time deinterleaver here: data cell d
        // (row d / cols, column d % cols of it) to rows column + row, FEC
        // block r's cell-interleaved cells at r cells..: a symbol's cells
        // land in about 45 short runs. The cell deinterleaver runs per block
        // with the LLRs, in its 64 KB. (Stores straight to FEC block order
        // hit the whole 580 KB frame at random: 30 ms a frame on the A9.)
        let (rows, cols) = (p.cells() / 5, 5 * p.fec_blocks);
        let mut scatter = vec![u32::MAX; at];
        for (d, &i) in dcells.iter().enumerate() {
            scatter[i as usize] = (rows * (d % cols) + d / cols) as u32;
        }
        let ncells = gather.iter().map(|g| g.len()).sum();
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
            scatter,
            cells: vec![[0; 2]; ncells],
            blk: Vec::new(),
            ci,
            buf: Vec::new(),
            base: 0,
            center_hz: 0.0,
            state: State::Search,
            misses: 0,
            w: 0.0,
            hinv: vec![Complex32::default(); CARRIERS],
            hinv_e: vec![Complex32::default(); CARRIERS],
            g_int: vec![[0; 2]; CARRIERS],
            g_shift: 16,
            cp_prev: Complex32::default(),
            cp_acc: Complex32::default(),
            work: vec![Complex32::default(); FFT],
            scratch,
            carriers: vec![Complex32::default(); N_P2 * CARRIERS],
            fe: None,
            order,
            stats: Stats::default(),
            constellation: Vec::new(),
            constellation_seq: 0,
            prof: [0.0; 6],
            sym_err: std::env::var_os("T2SYMERR").map(|_| vec![(0.0, 0.0); 256]),
            car_err: std::env::var_os("T2SYMERR").map(|_| vec![(0.0, 0.0); CARRIERS / 64 + 1]),
            sym_err_frame: vec![(0.0, 0.0); 256],
            pil_err: (0.0, 0.0),
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
    pub fn push(&mut self, x: &[Complex32], out: &mut Vec<Vec<i8>>) {
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
        self.p2_channel();
    }

    /// The channel from the P2 symbols' carriers (in `carriers` slots
    /// 0..8), then their cells.
    fn p2_channel(&mut self) {
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
        for ((he, &hi), &e) in self.hinv_e.iter_mut().zip(&self.hinv).zip(&self.early_rot) {
            *he = hi * e;
        }
        // Fixed point for equalize_int: the mean gain at about 2^12, a
        // component at most 2^14 (a deep fade's carriers get less gain than
        // their inverse: they carry more noise than signal anyway).
        let mean = self.hinv_e.iter().map(|z| z.norm()).sum::<f32>() / CARRIERS as f32 * CELL_SCALE;
        let s = if mean > 0.0 { (4096.0 / mean).log2().floor().clamp(2.0, 30.0) as u32 } else { 16 };
        self.g_shift = s;
        let m = 2f32.powi(s as i32) * CELL_SCALE;
        for (g, &h) in self.g_int.iter_mut().zip(&self.hinv_e) {
            *g = [(h.re * m).round().clamp(-16383.0, 16383.0) as i32, (h.im * m).round().clamp(-16383.0, 16383.0) as i32];
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
        let cj = std::mem::take(&mut self.carriers);
        self.equalize_from(j, &cj[slot * CARRIERS..(slot + 1) * CARRIERS]);
        self.carriers = cj;
    }

    /// [`Self::equalize`] from the carriers `cj` (with the early window's
    /// rotation undone).
    fn equalize_from(&mut self, j: usize, cj: &[Complex32]) {
        let hinv = std::mem::take(&mut self.hinv);
        let (fine, mut coarse) = self.eq_phase(j, |k| cj[k] * hinv[k]);
        for co in coarse.iter_mut() {
            *co *= CELL_SCALE;
        }
        let (lo, hi) = (self.data_at[j], self.data_at[j + 1]);
        let out = &mut self.flat[lo..hi];
        let dest = &self.scatter[lo..hi];
        let cells = &mut self.cells;
        for ((o, &d), &k) in out.iter_mut().zip(dest).zip(&self.data[j]) {
            let v = cj[k] * hinv[k] * (coarse[k >> 6] * fine[k & 63]);
            let c = [q8f(v.re), q8f(v.im)];
            // FEC block cells straight to their place (stores miss the
            // caches more cheaply than a gather's loads); the rest (L1,
            // dummy cells) in `flat`.
            if d != u32::MAX {
                cells[d as usize] = c;
            } else {
                *o = c;
            }
        }
        self.hinv = hinv;
    }

    /// [`Self::equalize`] for the front end's words (as they come: the
    /// early window's rotation is in `hinv_e`), the data cells in fixed
    /// point: the A9's VFP took about 150 ns a cell for the three complex
    /// multiplies, its integer multipliers (32 x 32 -> 64 bits) far less.
    fn equalize_int(&mut self, j: usize, cj: &[[i16; 2]]) {
        let hinv_e = std::mem::take(&mut self.hinv_e);
        let cf = |k: usize| Complex32::new(cj[k][0] as f32, cj[k][1] as f32);
        let (fine, coarse) = self.eq_phase(j, |k| cf(k) * hinv_e[k]);
        self.hinv_e = hinv_e;
        // e^(-j (a + slope k)) in Q14
        let q14 = |z: Complex32| [(z.re * 16384.0).round() as i32, (z.im * 16384.0).round() as i32];
        let fine: Vec<[i32; 2]> = fine.iter().map(|&z| q14(z)).collect();
        let coarse: Vec<[i32; 2]> = coarse.iter().map(|&z| q14(z)).collect();
        let g = &self.g_int;
        // g is in 2^g_shift units; gain times ramp to 2^16 units (a 32-bit
        // shift), so the last shift is a constant one (a variable 64-bit
        // shift is a handful of instructions and branches on the A9)
        let gs = self.g_shift - 2;
        let q = |x: i64| ((x + (1 << 15)) >> 16).clamp(-127, 127) as i8;
        let (lo, hi) = (self.data_at[j], self.data_at[j + 1]);
        let out = &mut self.flat[lo..hi];
        let dest = &self.scatter[lo..hi];
        let cells = &mut self.cells;
        for ((o, &d), &k) in out.iter_mut().zip(dest).zip(&self.data[j]) {
            let (f, c) = (fine[k & 63], coarse[k >> 6]);
            // ramp (Q14), gain times ramp (gain's format), times the carrier
            let r = [(f[0] * c[0] - f[1] * c[1]) >> 14, (f[0] * c[1] + f[1] * c[0]) >> 14];
            let gk = g[k];
            let gr = [(gk[0] * r[0] - gk[1] * r[1]) >> gs, (gk[0] * r[1] + gk[1] * r[0]) >> gs];
            let (x, y) = (cj[k][0] as i64, cj[k][1] as i64);
            let (a, b) = (gr[0] as i64, gr[1] as i64);
            let c = [q(x * a - y * b), q(x * b + y * a)];
            if d != u32::MAX {
                cells[d as usize] = c;
            } else {
                *o = c;
            }
        }
    }

    /// Symbol `j`'s common phase and phase slope from its pilots (`zc(k)`:
    /// carrier k times the channel inverse), the pilot error for the MER:
    /// e^(-j (a + slope k)) as fine[k % 64] coarse[k / 64].
    fn eq_phase(&mut self, j: usize, zc: impl Fn(usize) -> Complex32) -> ([Complex32; 64], [Complex32; CARRIERS / 64 + 1]) {
        let z: Vec<(usize, Complex32)> = self.pilots[j].iter().map(|&(k, r)| (k, zc(k) * r.conj())).collect();
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
        // e^(-j slope k) = coarse[k / 64] fine[k % 64]: independent
        // multiplies (a running product was a chain of 1705 dependent ones,
        // twice a symbol).
        let step = Complex32::from_polar(1.0, -slope);
        let mut fine = [Complex32::new(1.0, 0.0); 64];
        for m in 1..64 {
            fine[m] = fine[m - 1] * step;
        }
        let step64 = fine[63] * step;
        let mut coarse = [Complex32::new(1.0, 0.0); CARRIERS / 64 + 1];
        for n in 1..coarse.len() {
            coarse[n] = coarse[n - 1] * step64;
        }
        let ramp = |k: usize| coarse[k >> 6] * fine[k & 63];
        let mut c = Complex32::default();
        for &(k, v) in &z {
            c += v * ramp(k);
        }
        let a = if c.norm() > 0.0 { c.arg() } else { 0.0 };
        let rot = Complex32::from_polar(1.0, -a);
        // Pilot error after the equalizer (the display's MER; per symbol
        // index too when debugging).
        let (mut e, mut n) = (0f64, 0f64);
        for (&(k, v), &(_, rf)) in z.iter().zip(&self.pilots[j]) {
            let vn = v / rf.norm_sqr() * ramp(k) * rot;
            // in data-cell units (unit power): the pilot's own boost out
            let ek = ((vn - Complex32::new(1.0, 0.0)).norm_sqr() * rf.norm_sqr()) as f64;
            e += ek;
            n += 1.0;
            if let Some(ce) = self.car_err.as_mut() {
                ce[k >> 6].0 += ek;
                ce[k >> 6].1 += 1.0;
            }
        }
        self.pil_err.0 += e;
        self.pil_err.1 += n;
        if self.sym_err.is_some() {
            self.sym_err_frame[j].0 += e;
            self.sym_err_frame[j].1 += n;
        }
        for co in coarse.iter_mut() {
            *co *= rot;
        }
        (fine, coarse)
    }

    /// The frame's cells are all in: MER, FEC blocks' LLRs.
    fn finish(&mut self, out: &mut Vec<Vec<i8>>) {
        let t0 = std::time::Instant::now();
        let p = self.p;
        let flat = &self.flat;
        let cell = |i: u32| {
            let v = flat[i as usize];
            Complex32::new(v[0] as f32, v[1] as f32) * (1.0 / CELL_SCALE)
        };
        // The browser's constellation: a data symbol's cells.
        let q = |v: f32| q8(v * 56.0 / CELL_SCALE);
        self.constellation = self.cells.iter().step_by(97).take(256).map(|z| [q(z[0] as f32), q(z[1] as f32)]).collect();
        self.constellation_seq += 1;
        let (mut sig, mut err) = (0f32, 0f32);
        for (&i, &r) in self.pre_gather.iter().zip(&self.pre_ref) {
            sig += r * r;
            err += (cell(i) - Complex32::new(r, 0.0)).norm_sqr();
        }
        self.stats.mer_db = 10.0 * (sig / err.max(1e-12)).log10();
        // The MER shown: the pilots' error after the equalizer in data-cell
        // units, what the data cells see. The L1-pre cells' (above, for the
        // LLR scale) read about 3 dB higher.
        if self.pil_err.1 > 0.0 {
            self.stats.mer_db = (-10.0 * (self.pil_err.0 / self.pil_err.1).max(1e-12).log10()) as f32;
        }
        self.pil_err = (0.0, 0.0);
        if let Some(se) = self.sym_err.as_mut() {
            if self.stats.mer_db > 0.0 {
                for (a, b) in se.iter_mut().zip(&self.sym_err_frame) {
                    a.0 += b.0;
                    a.1 += b.1;
                }
            }
            self.sym_err_frame.iter_mut().for_each(|x| *x = (0.0, 0.0));
        }
        if std::env::var_os("T2PRE").is_some() && self.stats.frames < 3 {
            let v: Vec<String> = self.pre_gather.iter().zip(&self.pre_ref).take(12).map(|(&i, &r)| format!("{:+.2}{:+.2}j/{r:+}", cell(i).re, cell(i).im)).collect();
            eprintln!("L1-pre cells: {}", v.join(" "));
            let (mut sr, mut si) = (0f32, 0f32);
            for (&i, &r) in self.pre_gather.iter().zip(&self.pre_ref) {
                sr += cell(i).re * r;
                si += cell(i).im * r;
            }
            eprintln!("  sum(cell * ref) / n = {:.3}{:+.3}j", sr / self.pre_ref.len() as f32, si / self.pre_ref.len() as f32);
            let mean_abs: f32 = self.cells.iter().take(5000).map(|c| ((c[0] as f32).powi(2) + (c[1] as f32).powi(2)).sqrt()).sum::<f32>() / 5000.0 / CELL_SCALE;
            eprintln!("  data cells mean |z| {mean_abs:.2}");
        }
        self.stats.frames += 1;
        let sigma2 = (err / self.pre_gather.len() as f32).max(1e-6);
        // Rotated constellations: word j's I is in cell j, its Q in cell
        // j + 1 (cyclically in the block); rotate back.
        let angle: f32 = match p.constellation {
            Constellation::Qpsk => 29.0,
            Constellation::Qam16 => 16.8,
        };
        self.prof[3] += t0.elapsed().as_secs_f64();
        // Gather and LLRs in one pass, straight from the 8-bit cells (the
        // A9 has no integer divide and no vectorized f32: no % or / here).
        let t0 = std::time::Instant::now();
        let (sn, cs) = (-angle.to_radians()).sin_cos();
        let bits = p.constellation.bits();
        let n = self.gather[0].len();
        // Fixed point (the A9's VFP takes about 40 ns a complex multiply):
        // the rotation in Q14, the LLR scale (x LLR_SCALE: straight into the
        // LDPC decoder's 6 bits) in fixed point too.
        let (k, a) = match p.constellation {
            Constellation::Qpsk => (2.0 * std::f32::consts::FRAC_1_SQRT_2 * 2.0 / sigma2 / CELL_SCALE, 0.0),
            Constellation::Qam16 => {
                let a = 1.0 / 10f32.sqrt();
                (4.0 * a / sigma2 / CELL_SCALE, 2.0 * a * CELL_SCALE)
            }
        };
        // (Q10, capped at 64: |cell| >= 1 saturates there; in i32 with the
        // cells in Q7, no 64-bit multiplies)
        let kq = (k * crate::dvbs2::fpga_ldpc::LLR_SCALE * 1024.0).clamp(0.0, 65536.0) as i32;
        let (c14, s14) = ((cs * 16384.0).round() as i32, (sn * 16384.0).round() as i32);
        let a14 = (a * 16384.0).round() as i32;
        let rot = p.rotation;
        let q = |z14: i32| -> i8 {
            let v = ((z14 >> 7) * kq + (1 << 16)) >> 17;
            (if v > 31 { 31 } else if v < -31 { -31 } else { v }) as i8
        };
        for (r, ti) in self.cells.chunks(n).enumerate() {
            self.ci.block_gather(r, ti, &mut self.blk);
            let blk = &self.blk;
            let mut llr = vec![0i8; n * bits];
            for j in 0..n {
                let c = blk[j];
                let (zr, zi) = if rot {
                    // word j: I from cell j, Q from cell j + 1; rotate back
                    let d = blk[if j + 1 == n { 0 } else { j + 1 }];
                    let (re, im) = (c[0] as i32, d[1] as i32);
                    (re * c14 - im * s14, re * s14 + im * c14)
                } else {
                    (c[0] as i32 * 16384, c[1] as i32 * 16384)
                };
                match p.constellation {
                    Constellation::Qpsk => {
                        llr[2 * j] = q(zr);
                        llr[2 * j + 1] = q(zi);
                    }
                    Constellation::Qam16 => {
                        // Word bits 3, 2: signs of I, Q (0 positive); 1, 0:
                        // outer (0) or inner level. Max-log, per axis.
                        llr[4 * j] = q(zr);
                        llr[4 * j + 1] = q(zi);
                        llr[4 * j + 2] = q(zr.abs() - a14);
                        llr[4 * j + 3] = q(zi.abs() - a14);
                    }
                }
            }
            out.push(match p.constellation {
                Constellation::Qpsk => llr,
                Constellation::Qam16 => self.bi.deinterleave_llr(&llr),
            });
            self.stats.blocks += 1;
        }
        self.prof[4] += t0.elapsed().as_secs_f64();
    }

    /// The known P1 (at the nominal offset) within `r` samples of `at`
    /// (absolute): its start, the carrier's offset from the input's centre
    /// (coarse), the match.
    fn p1_near(&self, at: u64, r: usize) -> Option<(u64, f64, f32)> {
        self.p1_near_in(&self.buf, self.base, at, r)
    }

    /// [`Self::p1_near`] in samples `x` starting at counter `base`.
    fn p1_near_in(&self, x: &[Complex32], base: u64, at: u64, r: usize) -> Option<(u64, f64, f32)> {
        let a = (at.max(base) - base) as usize;
        let (s, c, q) = find_p1_in(x, &self.p1c, self.p1_energy, a.saturating_sub(r), a + r + 1, self.fs)?;
        Some((base + s as u64, c + self.center_hz, q))
    }

    /// Start of the best P1 by structure among `from..=to` (absolute; offset
    /// blind).
    fn structure_peak(&self, from: u64, to: u64) -> u64 {
        self.structure_peak_in(&self.buf, self.base, from, to)
    }

    /// [`Self::structure_peak`] in samples `x` starting at counter `xbase`.
    fn structure_peak_in(&self, x: &[Complex32], xbase: u64, from: u64, to: u64) -> u64 {
        let from = from.max(xbase);
        let base = (from - xbase) as usize;
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

/// The receiver's side of the FPGA front end.
struct FeState {
    /// Searching: the front end sends every sample (into `buf`).
    acquiring: bool,
    /// Counter of the next raw sample; the newest counter seen.
    raw_next: Option<u64>,
    now: u64,
    /// Raw runs (counter of the first sample, samples), oldest first.
    runs: std::collections::VecDeque<(u64, Vec<Complex32>)>,
    /// The FFT coming in: (symbol, frame start), carriers so far.
    car: Option<(usize, u64)>,
    car_pos: usize,
    car_buf: Vec<[i16; 2]>,
    /// The symbol coming in was equalized by the FPGA (t2eq): its cells in
    /// carrier order.
    car_eq: bool,
    eq_cells: Vec<[i8; 2]>,
    /// The frame being assembled: its start, the next symbol expected,
    /// whether all came.
    frame: Option<(u64, usize, bool)>,
    /// Carrier offset (absolute, Hz), the one the NCO takes out.
    f_est: f64,
    nco_hz: f64,
    /// Frames in a row whose carrier estimate jumped from `f_est`.
    jumps: u32,
    misses: u32,
    /// The next frame's P1, to be checked once its raw samples are in.
    p1_check: Option<u64>,
    /// Every sample raw asked for: carrier words still queued from before
    /// are not a schedule to join, until a long raw run shows it took.
    raw_all_asked: bool,
    /// Samples in the raw run so far.
    run_len: usize,
}

impl Demod {
    /// Words from the FPGA's OFDM front end ([`super::fe`]) in; FEC blocks
    /// out as from [`Self::push`]; `ctl` gets what the front end must do.
    pub fn push_words(&mut self, words: &[u32], out: &mut Vec<Vec<i8>>, ctl: &mut Vec<super::fe::Ctl>) {
        use super::fe::{decode, extend, Word};
        let mut fe = self.fe.take().unwrap_or_else(|| {
            Box::new(FeState {
                acquiring: true,
                raw_next: None,
                now: 0,
                runs: Default::default(),
                car: None,
                car_pos: 0,
                car_buf: vec![[0; 2]; CARRIERS],
                car_eq: false,
                eq_cells: vec![[0; 2]; 2 * super::fe::EQ_WORDS],
                frame: None,
                f_est: 0.0,
                nco_hz: 0.0,
                jumps: 0,
                misses: 0,
                p1_check: None,
                raw_all_asked: false,
                run_len: 0,
            })
        });
        let t_in = std::time::Instant::now();
        let mut acq: Vec<Complex32> = Vec::new();
        let mut i = 0;
        while i < words.len() {
            let w = words[i];
            i += 1;
            // Fast paths (most words): a run of carrier words into the FFT
            // buffer, a run of raw samples onto the current run (header bit
            // 0 clear; bit 16 marks carriers; a gap word has bit 0 set).
            let limit = if fe.car_eq { super::fe::EQ_WORDS } else { CARRIERS };
            if w & 0x0001_0001 == 0x0001_0000 && fe.car.is_some() && fe.car_pos < limit {
                let run = &words[i - 1..(i - 1 + limit - fe.car_pos).min(words.len())];
                let n = run.iter().position(|&x| x & 0x0001_0001 != 0x0001_0000).unwrap_or(run.len());
                if fe.car_eq {
                    for (p, &x) in run[..n].iter().enumerate() {
                        let c = super::fe::eq_cells(x);
                        let at = 2 * (fe.car_pos + p);
                        fe.eq_cells[at] = c[0];
                        fe.eq_cells[at + 1] = c[1];
                    }
                } else {
                    let order = &self.order[fe.car_pos..fe.car_pos + n];
                    for (&x, &k) in run[..n].iter().zip(order) {
                        fe.car_buf[k] = [(x as u16 & 0xFFFE) as i16, ((x >> 16) as u16 & 0xFFFE) as i16];
                    }
                }
                fe.car_pos += n;
                i += n - 1;
                if fe.car_pos == limit {
                    let (j, f) = fe.car.take().unwrap();
                    if !fe.acquiring {
                        if fe.car_eq {
                            self.fe_symbol_eq(&mut fe, j, f, out, ctl);
                        } else {
                            self.fe_symbol(&mut fe, j, f, out, ctl);
                        }
                    }
                }
                continue;
            }
            if w & 0x0001_0001 == 0 && !fe.acquiring && fe.raw_next.is_some() && !fe.runs.is_empty() {
                let run = &words[i - 1..];
                let n = run.iter().position(|&x| x & 0x0001_0001 != 0).unwrap_or(run.len());
                let r = fe.runs.back_mut().unwrap();
                r.1.extend(run[..n].iter().map(|&x| {
                    let v = [(x as u16 & 0xFFFE) as i16, ((x >> 16) as u16 & 0xFFFE) as i16];
                    Complex32::new(v[0] as f32, v[1] as f32) * (1.0 / 32768.0)
                }));
                // A long run (every sample raw: before the schedule took):
                // only its end can matter.
                if r.1.len() > 3 * self.p.frame_samples() {
                    let cut = r.1.len() - 2 * self.p.frame_samples();
                    r.1.drain(..cut);
                    r.0 += cut as u64;
                }
                let c = fe.raw_next.unwrap() + n as u64;
                fe.raw_next = Some(c);
                fe.now = c;
                fe.run_len += n;
                if fe.run_len > 3 * (P1_LEN + 512) {
                    fe.raw_all_asked = false;
                }
                i += n - 1;
                continue;
            }
            match decode(w) {
                Word::Gap => {
                    // Words lost: nothing continues across it.
                    fe.raw_next = None;
                    fe.run_len = 0;
                    fe.car = None;
                    if let Some(f) = fe.frame.as_mut() {
                        f.2 = false;
                    }
                    if fe.acquiring {
                        self.buf.clear();
                        acq.clear();
                    }
                    self.stats.gaps += 1;
                }
                Word::RawHeader(p) => {
                    let c = extend(p, 30, fe.raw_next.unwrap_or(fe.now));
                    if fe.raw_next != Some(c) {
                        fe.run_len = 0;
                    }
                    if fe.acquiring {
                        // A gap: start the search buffer again.
                        if fe.raw_next != Some(c) {
                            self.buf.clear();
                            self.base = c;
                            acq.clear();
                        }
                    } else if fe.raw_next != Some(c) || fe.runs.is_empty() {
                        fe.runs.push_back((c, Vec::new()));
                    }
                    fe.raw_next = Some(c);
                }
                Word::Raw(v) => {
                    let Some(c) = fe.raw_next else { continue };
                    let z = Complex32::new(v[0] as f32, v[1] as f32) / 32768.0;
                    if fe.acquiring {
                        if self.buf.is_empty() && acq.is_empty() {
                            self.base = c;
                        }
                        acq.push(z);
                    } else if let Some(r) = fe.runs.back_mut() {
                        r.1.push(z);
                        // A long run (every sample raw: before the schedule
                        // took): only its end can matter.
                        if r.1.len() > 3 * self.p.frame_samples() {
                            let cut = self.p.frame_samples();
                            r.1.drain(..cut);
                            r.0 += cut as u64;
                        }
                    }
                    fe.raw_next = Some(c + 1);
                    fe.now = c + 1;
                    fe.run_len += 1;
                    if fe.run_len > 3 * (P1_LEN + 512) {
                        fe.raw_all_asked = false;
                    }
                }
                Word::CarHeader { j, f21, eq } => {
                    if fe.acquiring && fe.raw_next.is_some() && !fe.raw_all_asked {
                        // The front end is already scheduled (the receiver
                        // restarted, a recording): take its frames.
                        fe.acquiring = false;
                        fe.runs.clear();
                        self.buf.clear();
                        acq.clear();
                        self.stats.locked = true;
                    }
                    fe.car = Some((j as usize, extend(f21, 21, fe.now)));
                    fe.car_pos = 0;
                    fe.car_eq = eq;
                }
                Word::Car(v) => {
                    if fe.car.is_none() || fe.car_pos >= CARRIERS || fe.car_eq {
                        // (equalized words always take the fast path above)
                        continue;
                    }
                    fe.car_buf[self.order[fe.car_pos]] = v;
                    fe.car_pos += 1;
                    if fe.car_pos == CARRIERS {
                        let (j, f) = fe.car.take().unwrap();
                        if !fe.acquiring {
                            self.fe_symbol(&mut fe, j, f, out, ctl);
                        }
                    }
                }
            }
        }
        self.prof[5] += t_in.elapsed().as_secs_f64();
        if fe.acquiring {
            self.buf.extend_from_slice(&acq);
            self.fe_acquire(&mut fe, ctl);
        } else if let Some(next) = fe.p1_check {
            if fe.now >= next + (P1_LEN + 2 * super::fe::TRACK as usize) as u64 {
                fe.p1_check = None;
                self.fe_p1(&mut fe, next, ctl);
            }
        }
        // Raw runs older than two frames go.
        let keep = fe.now.saturating_sub(2 * self.p.frame_samples() as u64);
        while fe.runs.len() > 1 && fe.runs.front().is_some_and(|r| r.0 + (r.1.len() as u64) < keep) {
            fe.runs.pop_front();
        }
        self.fe = Some(fe);
    }

    /// Searching: P1 and the frequency in the raw samples, then a schedule
    /// for the front end from a frame far enough ahead.
    fn fe_acquire(&mut self, fe: &mut FeState, ctl: &mut Vec<super::fe::Ctl>) {
        let frame = self.p.frame_samples() as u64;
        let gi = self.p.guard.samples();
        let need = frame + (P1_LEN + N_P2 * (FFT + gi) + 1024) as u64;
        if (self.buf.len() as u64) < need {
            return;
        }
        let t0 = std::time::Instant::now();
        let peak = self.structure_peak(self.base, self.base + frame);
        let found = self.p1_near(peak, 32).filter(|f| f.2 >= P1_OK);
        self.prof[0] += t0.elapsed().as_secs_f64();
        let Some((s, coarse, _)) = found else {
            self.stats.locked = false;
            self.buf.drain(..frame as usize);
            self.base += frame;
            return;
        };
        let mut cp = Complex32::default();
        for j in 0..N_P2 {
            cp += self.gi_corr(s, j);
        }
        let f = self.resolve_freq(cp, coarse);
        // A frame well beyond what is in (the ring and this search take
        // their time; a start already past, the front end catches up with).
        let end = self.base + self.buf.len() as u64;
        let mut start = s;
        while start < end + (self.fs * 0.3) as u64 {
            start += frame;
        }
        ctl.push(super::fe::Ctl::Schedule { start, freq: super::fe::freq_word(f, self.fs) });
        fe.acquiring = false;
        fe.f_est = f;
        fe.nco_hz = f;
        fe.misses = 0;
        fe.frame = None;
        fe.p1_check = None;
        fe.runs.clear();
        fe.raw_next = None;
        self.stats.locked = true;
        self.stats.freq_hz = (f - self.center_hz) as f32;
        self.buf.clear();
    }

    /// The carrier offset from a guard-interval correlation (modulo a
    /// carrier spacing) and an estimate that picks the multiple.
    fn resolve_freq(&self, cp: Complex32, near: f64) -> f64 {
        let spacing = self.fs / FFT as f64;
        let frac = cp.arg() as f64 / (std::f64::consts::TAU * FFT as f64) * self.fs;
        frac + ((near - frac) / spacing).round() * spacing
    }

    /// `len` raw samples from counter `at`, if one run holds them.
    fn fe_raw<'a>(fe: &'a FeState, at: u64, len: usize) -> Option<&'a [Complex32]> {
        fe.runs.iter().rev().find_map(|(c, v)| {
            (*c <= at && at + len as u64 <= c + v.len() as u64).then(|| &v[(at - c) as usize..(at - c) as usize + len])
        })
    }

    /// One FFT from the front end: symbol `j` of the frame starting at `f`.
    /// A symbol of frame `f` is in: whether the frame has all of its
    /// symbols so far.
    fn fe_frame_ok(fe: &mut FeState, j: usize, f: u64) -> bool {
        match fe.frame {
            Some((ff, next, ok)) if ff == f => fe.frame = Some((f, j + 1, ok && j == next)),
            _ => fe.frame = Some((f, j + 1, j == 0)),
        }
        fe.frame.unwrap().2
    }

    /// A symbol the FPGA equalized (t2eq): its cells as they are.
    fn fe_symbol_eq(&mut self, fe: &mut FeState, j: usize, f: u64, out: &mut Vec<Vec<i8>>, ctl: &mut Vec<super::fe::Ctl>) {
        let nsym = self.p.symbols();
        if !Self::fe_frame_ok(fe, j, f) || j < N_P2 {
            return;
        }
        let t0 = std::time::Instant::now();
        let cells = std::mem::take(&mut fe.eq_cells);
        self.eq_symbol(j, &cells);
        fe.eq_cells = cells;
        self.prof[2] += t0.elapsed().as_secs_f64();
        if j == nsym - 1 {
            self.finish(out);
            self.fe_track(fe, f, ctl);
        }
    }

    /// Symbol `j`'s cells from the FPGA (unit EQ_UNIT, carrier order): the
    /// pilot error for the MER, the data cells (to CELL_SCALE) to their
    /// places.
    fn eq_symbol(&mut self, j: usize, cells: &[[i8; 2]]) {
        let u = 1.0 / super::fe::EQ_UNIT as f32;
        let (mut e, mut n) = (0f64, 0f64);
        for &(k, rf) in &self.pilots[j] {
            let c = cells[k];
            e += (Complex32::new(c[0] as f32 * u, c[1] as f32 * u) - rf).norm_sqr() as f64;
            n += 1.0;
        }
        self.pil_err.0 += e;
        self.pil_err.1 += n;
        if self.sym_err.is_some() {
            self.sym_err_frame[j].0 += e;
            self.sym_err_frame[j].1 += n;
        }
        // (CELL_SCALE is twice EQ_UNIT)
        let (lo, hi) = (self.data_at[j], self.data_at[j + 1]);
        let out = &mut self.flat[lo..hi];
        let dest = &self.scatter[lo..hi];
        let dst = &mut self.cells;
        for ((o, &d), &k) in out.iter_mut().zip(dest).zip(&self.data[j]) {
            let c = cells[k];
            let c = [c[0] << 1, c[1] << 1];
            if d != u32::MAX {
                dst[d as usize] = c;
            } else {
                *o = c;
            }
        }
    }

    /// The channel inverse for the FPGA's equalizer (from `hinv_e`, with the
    /// early window's rotation, as the carrier words come).
    fn eq_table(&self) -> super::fe::Ctl {
        let z = super::fe::EQ_Z_UNIT;
        let mean = self.hinv_e.iter().map(|h| h.norm()).sum::<f32>() / CARRIERS as f32 * z;
        let gshift = if mean > 0.0 { (8192.0 / mean).log2().floor().clamp(1.0, 30.0) as u32 } else { 16 };
        let m = z * 2f32.powi(gshift as i32);
        let q = |v: f32| ((v * m).round().clamp(-32767.0, 32767.0) as i32 as u32) & 0xFFFF;
        let g = self.hinv_e.iter().map(|h| q(h.re) | q(h.im) << 16).collect();
        super::fe::Ctl::EqTable { g, gshift }
    }

    fn fe_symbol(&mut self, fe: &mut FeState, j: usize, f: u64, out: &mut Vec<Vec<i8>>, ctl: &mut Vec<super::fe::Ctl>) {
        let nsym = self.p.symbols();
        if !Self::fe_frame_ok(fe, j, f) {
            return;
        }
        if j < N_P2 {
            let t0 = std::time::Instant::now();
            let e = &self.early_rot;
            for ((o, &v), &r) in self.carriers[j * CARRIERS..(j + 1) * CARRIERS].iter_mut().zip(&fe.car_buf).zip(e) {
                *o = Complex32::new(v[0] as f32, v[1] as f32) * r;
            }
            self.prof[1] += t0.elapsed().as_secs_f64();
            if j == N_P2 - 1 {
                self.p2_channel();
                // the FPGA's equalizer takes it from the next symbol on
                ctl.push(self.eq_table());
            }
        } else {
            // data symbols straight from the front end's words
            let t0 = std::time::Instant::now();
            self.equalize_int(j, &fe.car_buf);
            self.prof[2] += t0.elapsed().as_secs_f64();
        }
        if j == nsym - 1 {
            self.finish(out);
            self.fe_track(fe, f, ctl);
        }
    }

    /// After a frame: its frequency from the guard intervals, the next
    /// frame's P1 against where the front end put it.
    fn fe_track(&mut self, fe: &mut FeState, f: u64, ctl: &mut Vec<super::fe::Ctl>) {
        use super::fe::{freq_word, Ctl};
        let gi = self.p.guard.samples();
        let sl = FFT + gi;
        let mut cp = Complex32::default();
        for j in 0..self.p.symbols() {
            let g = f + (P1_LEN + j * sl) as u64;
            if let (Some(a), Some(b)) = (Self::fe_raw(fe, g, gi), Self::fe_raw(fe, g + FFT as u64, gi)) {
                for (x, y) in a.iter().zip(b) {
                    cp += x.conj() * y;
                }
            }
        }
        if cp.norm() > 0.0 {
            // The carrier moves slowly: follow small changes halfway a
            // frame; a jump (a frame hit by a fade or a slip, its guard
            // intervals' phase on the other side of the wrap) only when
            // three frames in a row agree. One bad frame retuned the NCO and
            // lost the frames after it.
            let fnew = self.resolve_freq(cp, fe.f_est);
            let d = fnew - fe.f_est;
            if d.abs() <= 100.0 {
                fe.f_est += 0.5 * d;
                fe.jumps = 0;
            } else {
                fe.jumps += 1;
                if fe.jumps >= 3 {
                    fe.f_est = fnew;
                    fe.jumps = 0;
                }
            }
            let fnew = fe.f_est;
            self.stats.freq_hz = (fnew - self.center_hz) as f32;
            if (fnew - fe.nco_hz).abs() > 2.0 {
                fe.nco_hz = fnew;
                ctl.push(Ctl::Freq(freq_word(fnew, self.fs)));
            }
        }
        fe.p1_check = Some(f + self.p.frame_samples() as u64);
    }

    /// The P1 of the frame the front end starts at `next`: where it really
    /// is; the frame after moves to match.
    fn fe_p1(&mut self, fe: &mut FeState, next: u64, ctl: &mut Vec<super::fe::Ctl>) {
        use super::fe::{freq_word, Ctl, TRACK};
        let frame = self.p.frame_samples() as u64;
        let t0 = std::time::Instant::now();
        let found = Self::fe_raw(fe, next - TRACK as u64, P1_LEN + 2 * TRACK as usize).and_then(|x| {
            let base = next - TRACK as u64;
            let peak = self.structure_peak_in(x, base, next - TRACK as u64, next + TRACK as u64 - 1);
            let peak = peak.min(next + TRACK as u64 - 5).max(base + 4);
            self.p1_near_in(x, base, peak, 4)
        });
        self.prof[0] += t0.elapsed().as_secs_f64();
        // Where it should be, within a few samples: a lower bar than the
        // search's (noise alone reaches about 0.1 here).
        match found.filter(|v| v.2 >= P1_TRACK_OK) {
            Some((s, _, _)) => {
                fe.misses = 0;
                // Off by more than a couple of samples: move the frame after
                // it, while it is still ahead of the front end.
                if s.abs_diff(next) > 2 && fe.now + ((self.fs * 0.03) as u64) < next + frame {
                    ctl.push(Ctl::Schedule { start: s + frame, freq: freq_word(fe.nco_hz, self.fs) });
                }
            }
            None => {
                fe.misses += 1;
                self.stats.p1_missed += 1;
                if fe.misses >= 6 {
                    ctl.push(Ctl::RawAll);
                    fe.raw_all_asked = true;
                    fe.acquiring = true;
                    fe.frame = None;
                    fe.raw_next = None;
                    fe.p1_check = None;
                    self.stats.locked = false;
                    self.buf.clear();
                }
            }
        }
    }
}

/// [`q8`] with plain compares (f32::max/min are libm calls on armv7).
#[inline]
fn q8f(v: f32) -> i8 {
    let x = if v > 127.0 { 127.0 } else if v < -127.0 { -127.0 } else { v };
    (if x >= 0.0 { x + 0.5 } else { x - 0.5 }) as i32 as i8
}

/// Round to 8 bits (saturating) without libm's round().
fn q8(v: f32) -> i8 {
    (v + if v >= 0.0 { 0.5 } else { -0.5 }) as i8
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
