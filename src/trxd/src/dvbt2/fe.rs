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
//! symbol j in 7:0 and the low 22 bits of the frame's start in 29:8.

use num_complex::Complex32;

pub const N: usize = 2048;
pub const CARRIERS: usize = 1705;
/// Raw samples either side of each P1.
pub const TRACK: u32 = 64;
/// FFT windows start this far before each symbol's useful part.
pub const EARLY: u32 = 64;
/// FFT truncation stages (1/2 each): the carriers come out scaled by 1/64.
pub const FFT_SCALE: f32 = 1.0 / 64.0;

/// Put in the word stream by the ring reader where words were lost (the
/// front end never sends it: a carrier header for symbol 255).
pub const GAP: u32 = 0xFFFF_FFFF;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Word {
    Gap,
    RawHeader(u32),
    Raw([i16; 2]),
    CarHeader { j: u8, f22: u32 },
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
        if car { Word::CarHeader { j: p as u8, f22: p >> 8 } } else { Word::RawHeader(p) }
    } else {
        let v = [(w as u16 & 0xFFFE) as i16, ((w >> 16) as u16 & 0xFFFE) as i16];
        if car { Word::Car(v) } else { Word::Raw(v) }
    }
}

/// What the receiver wants of the front end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ctl {
    /// Every sample raw (searching).
    RawAll,
    /// Frames from `start` (counter) on (at once when none is running, else
    /// from the end of the current one), the NCO at `freq`.
    Schedule { start: u64, freq: u32 },
    /// The NCO's phase step a sample.
    Freq(u32),
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
            }
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
                let (mut raw, mut in_win, mut j) = (true, false, 0u64);
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
                                if q < gi || q >= N as i64 {
                                    raw = true;
                                }
                                let lo = gi - EARLY as i64;
                                if q >= lo && q < lo + N as i64 {
                                    in_win = true;
                                }
                            }
                        }
                    }
                    if r == fl - 1 {
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
                        self.win_tag = j as u32 | ((self.f as u32) & 0x3F_FFFF) << 8;
                    }
                    self.win.push(y);
                    if self.win.len() == N {
                        let fft = FftPlanner::new().plan_fft_forward(N);
                        fft.process(&mut self.win);
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
