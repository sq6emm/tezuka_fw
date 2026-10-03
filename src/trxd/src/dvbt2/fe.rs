//! The FPGA's DVB-T2 OFDM front end (maia-hdl `t2ofdm.py`): its ring word
//! format, the commands the receiver gives it, and (for tests) a model.
//!
//! After the T2 resampler the FPGA counts samples (32 bits) and, once the
//! receiver has given it a frame start and a frequency, mixes the signal
//! down, takes each symbol's FFT and sends the active carriers; raw samples
//! go along only where the receiver needs them (around P1, each guard
//! interval and the tail it copies). Before that, every sample goes raw
//! (acquisition).
//!
//! Ring words (re in 15:0, im in 31:16): bit 0 set on headers, bit 16 set on
//! the carrier stream; values lose their LSB. Header payload (30 bits) =
//! re[15:1] | im[15:1] << 15: raw, the counter of the next sample; carriers,
//! symbol j in 7:0, the low 21 bits of the frame's start in 28:8 and bit 29
//! set when the FPGA's equalizer (maia-hdl `t2eq.py`) did the symbol: then
//! 853 words of two 7-bit cells each (carriers in order, a unit of
//! [`EQ_UNIT`]: I 7:1, Q 14:8 of carrier 2i; 23:17, 30:24 of 2i + 1) instead
//! of the 1705 carriers.

use num_complex::Complex32;

pub const N: usize = 2048;
pub const CARRIERS: usize = 1705;
/// Raw samples either side of each P1.
pub const TRACK: u32 = 64;
/// FFT windows start this far before each symbol's useful part.
pub const EARLY: u32 = 64;

/// [`EARLY`] for a guard interval of `gi` samples: never past half the
/// guard (t2ofdm.py places the window at gi - early: a guard shorter than
/// EARLY would put it before the symbol, in the previous one).
pub fn early(gi: usize) -> u32 {
    EARLY.min(gi as u32 / 2)
}
/// FFT truncation stages (1/2 each): the carriers come out scaled by 1/64.
pub const FFT_SCALE: f32 = 1.0 / 64.0;
/// An equalized cell's unit (t2eq.py: z unit 1280 >> 6).
pub const EQ_UNIT: i32 = 20;
/// The equalizer's z unit: the channel inverse is loaded as G = hinv 1280 2^gshift.
pub const EQ_Z_UNIT: f32 = 1280.0;
/// Words of an equalized symbol.
pub const EQ_WORDS: usize = CARRIERS.div_ceil(2);

/// Put in the word stream by the ring reader where words were lost (the
/// front end never sends it: a carrier header for symbol 255).
pub const GAP: u32 = 0xFFFF_FFFF;

/// Report records (maia-hdl t2p1.py, t2eq.py): a carrier header with one of
/// these symbol numbers, then [`REPORT_WORDS`] words of 16 bits each
/// ([`report_value`]). P1: the best P1 window's start (2 words), |C| + |B|
/// (2), E (2) per frame-length block; GI: the frame's guard-interval
/// correlation re (2), im (2), the frame's start (2); MER: the pilots'
/// error sum >> 4 (2), their number, the frame's start's low 21 bits (2), 0.
/// A record's words come together but may sit inside a symbol's carriers.
pub const P1_J: u8 = 253;
pub const GI_J: u8 = 252;
pub const MER_J: u8 = 251;
pub const REPORT_WORDS: usize = 6;
/// The t2_features bits a bitstream with the reports has.
pub const FEATURES_REPORTS: u32 = 0b111;

/// The 16 bits a report word carries.
pub fn report_value(w: u32) -> u16 {
    (((w >> 1) & 0x7FFF) | ((w >> 17) & 1) << 15) as u16
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Word {
    Gap,
    RawHeader(u32),
    Raw([i16; 2]),
    CarHeader { j: u8, f21: u32, eq: bool },
    Car([i16; 2]),
}

pub fn decode(w: u32) -> Word {
    if w == GAP {
        return Word::Gap;
    }
    let header = w & 1 != 0;
    let car = w & (1 << 16) != 0;
    if header {
        let p = ((w >> 1) & 0x7FFF) | ((w >> 17) & 0x7FFF) << 15;
        if car { Word::CarHeader { j: p as u8, f21: (p >> 8) & 0x1F_FFFF, eq: p >> 29 & 1 == 1 } } else { Word::RawHeader(p) }
    } else {
        let v = [(w as u16 & 0xFFFE) as i16, ((w >> 16) as u16 & 0xFFFE) as i16];
        if car { Word::Car(v) } else { Word::Raw(v) }
    }
}

/// What the receiver wants of the front end.
/// Two equalized cells from a carrier word of an equalized symbol.
pub fn eq_cells(w: u32) -> [[i8; 2]; 2] {
    let c = |shift: u32| (((w >> shift) as u8) << 1) as i8 >> 1;
    [[c(1), c(8)], [c(17), c(24)]]
}

#[derive(Clone, Debug, PartialEq)]
pub enum Ctl {
    /// Every sample raw (searching).
    RawAll,
    /// Frames from `start` (counter) on (at once when none is running, else
    /// from the end of the current one), the NCO at `freq`.
    Schedule { start: u64, freq: u32 },
    /// The NCO's phase step a sample.
    Freq(u32),
    /// The equalizer's channel inverse for the symbols to come: per carrier
    /// re | im << 16 (16 bits each, a z unit of [`EQ_Z_UNIT`] `gshift` bits
    /// up).
    EqTable { g: Vec<u32>, gshift: u32 },
    /// Equalized symbols into the ring: all (0) or only symbol j (the
    /// router takes them all).
    EqRing(u8),
}

/// NCO step removing `hz` at `fs`.
pub fn freq_word(hz: f64, fs: f64) -> u32 {
    (-hz / fs * 4_294_967_296.0).round() as i64 as u32
}

fn bitrev11(i: usize) -> usize {
    (i as u16).reverse_bits() as usize >> 5
}

/// For each carrier word of an FFT (in the order they come), its carrier
/// index k, from the carriers' FFT bins.
pub fn carrier_order(bins: &[usize]) -> Vec<usize> {
    let mut of_bin = vec![usize::MAX; N];
    for (k, &b) in bins.iter().enumerate() {
        of_bin[b] = k;
    }
    (0..N).map(|i| of_bin[bitrev11(i)]).filter(|&k| k != usize::MAX).collect()
}

/// Extend a counter's low `bits` to the full count nearest `near`.
pub fn extend(low: u32, bits: u32, near: u64) -> u64 {
    let m = 1u64 << bits;
    let d = (low as u64).wrapping_sub(near) & (m - 1);
    let d = if d >= m / 2 { d as i64 - m as i64 } else { d as i64 };
    (near as i64 + d) as u64
}

#[cfg(test)]
pub mod model {
    //! The front end in software for the receiver's tests: the same
    //! schedule and word streams; a float FFT (not the FPGA's integer one).
    use super::*;
    use rustfft::FftPlanner;

    pub struct Model {
        pub frame_len: u64,
        pub nsym: u64,
        pub gi: u64,
        pub shift: u32,
        bins_order: Vec<usize>,
        active: Vec<bool>,
        counter: u64,
        scheduled: bool,
        running: bool,
        f: u64,
        pending: Option<u64>,
        phase: u32,
        freq: u32,
        run_len: u64,
        win: Vec<Complex32>,
        win_tag: u32,
        skip: bool,
        /// k of each FFT bin (usize::MAX: not a carrier).
        k_of_bin: Vec<usize>,
        /// The equalizer (t2eq.py in float) when enabled: p2, dx, dy, fc_j,
        /// prbs, pn; the table once loaded.
        eq: Option<(usize, usize, usize, usize, Vec<u8>, Vec<u8>)>,
        eq_g: Option<(Vec<Complex32>, u32)>,
        /// Reports (t2p1.py, t2eq.py) when enabled: P1 / GI model, the MER
        /// sum of the frame, which equalized symbols go out.
        reports: Option<P1Model>,
        mer: (u64, u32),
        ring_j: u8,
    }

    /// maia-hdl t2p1.Model in Rust (bit-exact).
    pub struct P1Model {
        block_len: u64,
        k: i64,
        cos: Vec<i64>,
        sin: Vec<i64>,
        mem: std::collections::VecDeque<([i64; 2], [i64; 2])>,
        count: u64,
        c: [i64; 2],
        b: [i64; 2],
        e: i64,
        gi: [i64; 2],
        blk: u64,
        best: Option<(i64, u32, u32, u32)>,
    }

    fn wrap48(v: i64) -> i64 {
        (v << 16) >> 16
    }

    impl P1Model {
        pub fn new(block_len: u64, k_q8: u32) -> P1Model {
            let ph = |i: usize| std::f64::consts::TAU * i as f64 / 1024.0;
            P1Model {
                block_len,
                k: k_q8 as i64,
                cos: (0..1024).map(|i| (32767.0 * ph(i).cos()).round() as i64).collect(),
                sin: (0..1024).map(|i| (32767.0 * ph(i).sin()).round() as i64).collect(),
                mem: Default::default(),
                count: 0,
                c: [0; 2],
                b: [0; 2],
                e: 0,
                gi: [0; 2],
                blk: 0,
                best: None,
            }
        }

        fn tap(&self, d: u64) -> ([i64; 2], [i64; 2]) {
            if self.count < d {
                ([0; 2], [0; 2])
            } else {
                self.mem[self.mem.len() - d as usize]
            }
        }

        fn words(j: u8, v: [u32; 3]) -> Vec<u32> {
            let mut out = vec![Model::header(j as u32, true)];
            for x in v {
                for h in [x & 0xFFFF, x >> 16] {
                    out.push(((h & 0x7FFF) << 1) | (((h >> 15) & 1) << 1 | 1) << 16);
                }
            }
            out
        }

        pub fn push(&mut self, x: [i16; 2], t: u64, tail: bool, last: bool, frame_start: u64) -> Vec<u32> {
            let mut out = Vec::new();
            let sat = |v: i64| v.clamp(-32768, 32767);
            let x0 = [x[0] as i64, x[1] as i64];
            let (c, s) = (self.cos[(t & 1023) as usize], self.sin[(t & 1023) as usize]);
            let y0 = [sat((x0[0] * c + x0[1] * s + (1 << 14)) >> 15), sat((x0[1] * c - x0[0] * s + (1 << 14)) >> 15)];
            let mc = |a: [i64; 2], b: [i64; 2]| [a[0] * b[0] + a[1] * b[1], a[1] * b[0] - a[0] * b[1]];
            let (x482, y482) = self.tap(482);
            let (x964, _) = self.tap(964);
            let (x1024, _) = self.tap(1024);
            let (_, y1506) = self.tap(1506);
            let (x2048, y2048) = self.tap(2048);
            self.e = wrap48(self.e + x0[0] * x0[0] + x0[1] * x0[1] - x2048[0] * x2048[0] - x2048[1] * x2048[1]);
            let (ba, br, ca, cr) = (mc(y0, x482), mc(y482, x964), mc(y1506, x482), mc(y2048, x1024));
            for i in 0..2 {
                self.b[i] = wrap48(self.b[i] + ba[i] - br[i]);
                self.c[i] = wrap48(self.c[i] + ca[i] - cr[i]);
            }
            if tail && self.count >= N as u64 {
                let g = mc(x0, x2048);
                for i in 0..2 {
                    self.gi[i] = wrap48(self.gi[i] + g[i]);
                }
            }
            self.mem.push_back((x0, y0));
            if self.mem.len() > 2048 {
                self.mem.pop_front();
            }
            self.count += 1;
            let mag = |z: [i64; 2]| {
                let (a, b) = (z[0].abs(), z[1].abs());
                a.max(b) + ((3 * a.min(b)) >> 3)
            };
            if self.count > N as u64 {
                let cb = ((mag(self.c) + mag(self.b)) >> 10) & 0xFFFF_FFFF;
                let e = (self.e >> 10) & 0xFFFF_FFFF;
                let score = cb - ((self.k * e) >> 8);
                if self.best.is_none_or(|b| score > b.0) {
                    self.best = Some((score, (t.wrapping_sub(N as u64 - 1)) as u32, cb as u32, e as u32));
                }
            }
            self.blk += 1;
            if self.blk == self.block_len {
                self.blk = 0;
                if let Some((_, s, cb, e)) = self.best.take() {
                    out.extend(Self::words(P1_J, [s, cb, e]));
                }
            }
            if last {
                let g = |v: i64| (v >> 16) as u32;
                out.extend(Self::words(GI_J, [g(self.gi[0]), g(self.gi[1]), frame_start as u32]));
                self.gi = [0; 2];
            }
            out
        }
    }

    impl Model {
        pub fn new(frame_len: u64, nsym: u64, gi: u64, bins: &[usize]) -> Model {
            let mut active = vec![false; N];
            for &b in bins {
                active[b] = true;
            }
            Model {
                frame_len,
                nsym,
                gi,
                shift: 0,
                bins_order: (0..N).map(bitrev11).collect(),
                active,
                counter: 0,
                scheduled: false,
                running: false,
                f: 0,
                pending: None,
                phase: 0,
                freq: 0,
                run_len: 0,
                win: Vec::new(),
                win_tag: 0,
                skip: false,
                k_of_bin: {
                    let mut v = vec![usize::MAX; N];
                    for (k, &b) in bins.iter().enumerate() {
                        v[b] = k;
                    }
                    v
                },
                eq: None,
                eq_g: None,
                reports: None,
                mer: (0, 0),
                ring_j: 0,
            }
        }

        /// Reports on (as trxd turns them on: P1, GI, MER, no raw samples
        /// while searching or around the guard intervals).
        pub fn enable_reports(&mut self, k_q8: u32) {
            self.reports = Some(P1Model::new(self.frame_len, k_q8));
        }

        /// Equalize data symbols (from `p2` on) as the FPGA's t2eq does, once
        /// a table comes (float here, not bit-exact).
        pub fn enable_eq(&mut self, p2: usize, dx: usize, dy: usize, fc_j: usize) {
            let mut sr: u32 = 0x7ff;
            let prbs = (0..CARRIERS)
                .map(|_| {
                    let b = (sr ^ (sr >> 2)) & 1;
                    let o = (sr & 1) as u8;
                    sr >>= 1;
                    if b != 0 {
                        sr |= 0x400;
                    }
                    o
                })
                .collect();
            let pn = crate::dvbt2::tables::PN_SEQUENCE.iter().flat_map(|&b| (0..8).rev().map(move |k| (b >> k) & 1)).collect();
            self.eq = Some((p2, dx, dy, fc_j, prbs, pn));
        }

        /// The equalized words of symbol `j` from its carriers (by k), and
        /// its pilots' MER terms (t2eq.py: (8 I - s 213)^2 + (8 Q)^2; sum,
        /// count).
        fn equalize(&self, j: usize, c: &[Complex32]) -> (Vec<u32>, (u64, u32)) {
            let (_, dx, dy, fc_j, prbs, pn) = self.eq.as_ref().unwrap();
            let (g, gs) = self.eq_g.as_ref().unwrap();
            let sc = 1.0 / (1u64 << gs) as f32;
            let z: Vec<Complex32> = c.iter().zip(g).map(|(&c, &g)| c * g * sc).collect();
            let (d, k0) = if j == *fc_j { (*dx, 0) } else { (dx * dy, dx * (j % dy)) };
            let pil: Vec<(usize, Complex32)> =
                (k0..CARRIERS).step_by(d).map(|k| (k, if prbs[k] ^ pn[j] == 1 { -z[k] } else { z[k] })).collect();
            let ds: Complex32 = pil.windows(2).map(|w| w[1].1 * w[0].1.conj()).sum();
            let slope = ds.arg() / d as f32;
            let cp: Complex32 = pil.iter().map(|&(k, v)| v * Complex32::from_polar(1.0, -slope * k as f32)).sum();
            let a = cp.arg();
            let u = EQ_UNIT as f32 / EQ_Z_UNIT;
            let qi = |v: f32| (v * u).round().clamp(-63.0, 63.0) as i32;
            let ci: Vec<(i32, i32)> = (0..CARRIERS)
                .map(|k| {
                    let v = z[k] * Complex32::from_polar(1.0, -(a + slope * k as f32));
                    (qi(v.re), qi(v.im))
                })
                .collect();
            let mut mer = (0u64, 0u32);
            for k in (k0..CARRIERS).step_by(d) {
                let r = 8 * ci[k].0 as i64 - if prbs[k] ^ pn[j] == 1 { -213 } else { 213 };
                mer.0 += (r * r + (8 * ci[k].1 as i64).pow(2)) as u64;
                mer.1 += 1;
            }
            let m7 = |v: i32| v as u32 & 0x7F;
            let cells: Vec<(u32, u32)> = ci.iter().map(|&(a, b)| (m7(a), m7(b))).chain(std::iter::once((0, 0))).collect();
            (cells.chunks(2).map(|p| p[0].0 << 1 | p[0].1 << 8 | 1 << 16 | p[1].0 << 17 | p[1].1 << 24).collect(), mer)
        }

        pub fn apply(&mut self, c: Ctl) {
            match c {
                Ctl::RawAll => {
                    self.scheduled = false;
                    self.running = false;
                    self.pending = None;
                }
                Ctl::Schedule { start, freq } => {
                    self.scheduled = true;
                    self.pending = Some(start);
                    self.freq = freq;
                }
                Ctl::Freq(f) => self.freq = f,
                Ctl::EqTable { g, gshift } => {
                    let v = |x: u32| x as u16 as i16 as f32;
                    self.eq_g = Some((g.iter().map(|&w| Complex32::new(v(w), v(w >> 16))).collect(), gshift));
                }
                Ctl::EqRing(j) => self.ring_j = j,
            }
        }

        fn header(p: u32, car: bool) -> u32 {
            let re = ((p & 0x7FFF) << 1) | 1;
            let im = (((p >> 15) & 0x7FFF) << 1) | car as u32;
            re | im << 16
        }

        fn data(v: [i16; 2], car: bool) -> u32 {
            (v[0] as u16 as u32 & 0xFFFE) | ((v[1] as u16 as u32 & 0xFFFE) | car as u32) << 16
        }

        /// Samples in, words out (each stream in order; carriers of an FFT
        /// come out whole when its window is complete).
        pub fn run(&mut self, x: &[[i16; 2]], out: &mut Vec<u32>) {
            for &v in x {
                if self.scheduled && !self.running {
                    if let Some(p) = self.pending.take() {
                        self.f = p;
                        self.running = true;
                        // a start already past: the rest of that frame left out
                        self.skip = self.counter > p;
                    }
                }
                if !(self.scheduled && self.running) {
                    // no schedule: the FFT restarts (as the FPGA's does)
                    self.win.clear();
                }
                let ph = self.phase as f64 / 4_294_967_296.0 * std::f64::consts::TAU;
                let y = Complex32::new(v[0] as f32, v[1] as f32) * Complex32::new(ph.cos() as f32, ph.sin() as f32);
                self.phase = self.phase.wrapping_add(self.freq);
                let hw = self.reports.is_some();
                let (mut raw, mut in_win, mut j) = (!hw, false, 0u64);
                let (mut tail, mut last, fs) = (false, false, self.f);
                if self.scheduled && self.running {
                    let r = self.counter as i64 - self.f as i64;
                    let (fl, gi) = (self.frame_len as i64, self.gi as i64);
                    let sl = N as i64 + gi;
                    raw = false;
                    if r < 0 {
                        raw = r >= -(TRACK as i64);
                    } else if r < fl {
                        if (r < N as i64 + TRACK as i64 && !self.skip) || r >= fl - TRACK as i64 {
                            raw = true;
                        }
                        if r >= N as i64 && !self.skip {
                            let u = r - N as i64;
                            let (jj, q) = (u / sl, u % sl);
                            if (jj as u64) < self.nsym {
                                j = jj as u64;
                                if (q < gi || q >= N as i64) && !hw {
                                    raw = true;
                                }
                                tail = q >= N as i64;
                                let lo = gi - early(gi as usize) as i64;
                                if q >= lo && q < lo + N as i64 {
                                    in_win = true;
                                }
                            }
                        }
                    }
                    if r == fl - 1 {
                        last = true;
                        self.f = self.pending.take().unwrap_or(self.f + self.frame_len);
                        self.skip = false;
                    } else if r >= fl {
                        // A start already past: catch up, leave that frame out.
                        self.f += self.frame_len;
                        self.skip = true;
                    }
                } else if self.scheduled {
                    raw = false;
                }
                if let Some(p1) = self.reports.as_mut() {
                    out.extend(p1.push(v, self.counter, tail, last, fs));
                }
                if raw {
                    if self.run_len % 65536 == 0 {
                        out.push(Self::header((self.counter & 0x3FFF_FFFF) as u32, false));
                    }
                    out.push(Self::data(v, false));
                    self.run_len += 1;
                } else {
                    self.run_len = 0;
                }
                if in_win {
                    if self.win.is_empty() {
                        self.win_tag = j as u32 | ((self.f as u32) & 0x1F_FFFF) << 8;
                    }
                    self.win.push(y);
                    if self.win.len() == N {
                        let fft = FftPlanner::new().plan_fft_forward(N);
                        fft.process(&mut self.win);
                        let jj = (self.win_tag & 0xFF) as usize;
                        if self.eq_g.is_some() && self.eq.as_ref().is_some_and(|e| jj >= e.0) {
                            // carriers by k, as the words would carry them
                            let mut c = vec![Complex32::default(); CARRIERS];
                            for (b, &k) in self.k_of_bin.iter().enumerate() {
                                if k != usize::MAX {
                                    let z = self.win[b] * FFT_SCALE / (1 << self.shift) as f32;
                                    let q = |v: f32| (v.round().clamp(-32768.0, 32767.0) as i16 & !1) as f32;
                                    c[k] = Complex32::new(q(z.re), q(z.im));
                                }
                            }
                            let (words, mer) = self.equalize(jj, &c);
                            if self.ring_j == 0 || self.ring_j as usize == jj {
                                out.push(Self::header(self.win_tag | 1 << 29, true));
                                out.extend(words);
                            }
                            if self.reports.is_some() {
                                self.mer.0 += mer.0;
                                self.mer.1 += mer.1;
                                if jj as u64 == self.nsym - 1 {
                                    let f21 = (self.win_tag >> 8) & 0x1F_FFFF;
                                    let err = (self.mer.0 >> 4) as u32;
                                    out.extend(P1Model::words(MER_J, [err, self.mer.1 | (f21 & 0xFFFF) << 16, f21 >> 16]));
                                    self.mer = (0, 0);
                                }
                            }
                            self.win.clear();
                            self.counter += 1;
                            continue;
                        }
                        out.push(Self::header(self.win_tag, true));
                        for i in 0..N {
                            let b = self.bins_order[i];
                            if self.active[b] {
                                let z = self.win[b] * FFT_SCALE / (1 << self.shift) as f32;
                                let q = |v: f32| v.round().clamp(-32768.0, 32767.0) as i16;
                                out.push(Self::data([q(z.re), q(z.im)], true));
                            }
                        }
                        self.win.clear();
                    }
                }
                self.counter += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::model::P1Model;
    use super::{report_value, GI_J, P1_J};

    /// P1Model against maia-hdl t2p1.Model: the same input (an LCG and two
    /// P1-like C A B structures, as made by the Python below) gives the same
    /// words. The words came from:
    /// md = Model(5000, 64); md.push(re, im, t, t % 2300 >= 2100, t in
    /// {6000, 11000}, {6000: 1000, 11000: 6000}.get(t, 0)) for each sample.
    #[test]
    fn p1_model_matches_the_python_one() {
        let want: [u32; 28] = [0x000101fb, 0x00011388, 0x00010000, 0x0003052a, 0x000100c0, 0x0003a046, 0x0001016a, 0x000101f9, 0x00010354, 0x00010000, 0x0001017a, 0x00010000, 0x000107d0, 0x00010000, 0x000101fb, 0x00013a98, 0x00010000, 0x00011c4e, 0x000100ca, 0x0003f87c, 0x0001017a, 0x000101f9, 0x000104be, 0x00010000, 0x0003ffd4, 0x0003fffe, 0x00012ee0, 0x00010000];
        let mut st: u64 = 12345;
        let mut rnd = || {
            st = (st * 1103515245 + 12345) & 0x7FFF_FFFF;
            ((st >> 8) % 2001) as i64 - 1000
        };
        let n = 12000;
        let mut x: Vec<(i64, i64)> = (0..n).map(|_| (rnd(), rnd())).collect();
        let rot = |i: usize| {
            let p = std::f64::consts::TAU * i as f64 / 1024.0;
            (p.cos(), p.sin())
        };
        for s0 in [2500usize, 7500] {
            let a: Vec<(i64, i64)> = (0..1024).map(|i| (x[s0 + 542 + i].0 * 3, x[s0 + 542 + i].1 * 3)).collect();
            for i in 0..1024 {
                x[s0 + 542 + i] = a[i];
            }
            let shifted = |v: (i64, i64), i: usize| {
                let (c, s) = rot(i);
                let (re, im) = (v.0 as f64 * c - v.1 as f64 * s, v.0 as f64 * s + v.1 as f64 * c);
                (re.round() as i64, im.round() as i64)
            };
            for i in 0..542 {
                x[s0 + i] = shifted(a[482 + i], i);
            }
            for i in 0..482 {
                x[s0 + 1566 + i] = shifted(a[542 + i], i);
            }
        }
        let mut md = P1Model::new(5000, 64);
        let mut got = Vec::new();
        for (t, &(re, im)) in x.iter().enumerate() {
            let v = [re.clamp(-32768, 32767) as i16, im.clamp(-32768, 32767) as i16];
            let fs = match t {
                6000 => 1000,
                11000 => 6000,
                _ => 0,
            };
            got.extend(md.push(v, t as u64, t % 2300 >= 2100, t == 6000 || t == 11000, fs));
        }
        assert_eq!(got, want);
        // the P1s where they were put, the GI reports with their frames
        let recs: Vec<(u8, u32)> = got.chunks(7).map(|r| (((r[0] >> 1) & 0xFF) as u8, report_value(r[5]) as u32 | (report_value(r[6]) as u32) << 16)).collect();
        let starts: Vec<(u8, u32)> = got.chunks(7).map(|r| (((r[0] >> 1) & 0xFF) as u8, report_value(r[1]) as u32 | (report_value(r[2]) as u32) << 16)).collect();
        assert_eq!(starts.iter().filter(|r| r.0 == P1_J).map(|r| r.1).collect::<Vec<_>>(), [2500, 7500]);
        assert_eq!(recs.iter().filter(|r| r.0 == GI_J).map(|r| r.1).collect::<Vec<_>>(), [1000, 6000]);
    }
}
