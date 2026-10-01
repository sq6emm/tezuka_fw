//! CW copy through rain scatter (10 GHz and up): one station's keying when
//! its carrier arrives Doppler-spread over a few hundred Hz to more than a
//! kHz and fading fast. A narrow filter or a tone-tracking decoder catches a
//! sliver of that; this collects all of it (docs/RSCW.md).
//!
//! Live ([`RsNnStream`], the CW box's "rs" engine and the CW-RS mode): a
//! small neural network ([`crate::rsnn`]) gives each frame's likelihood
//! that the key is down; a character-level Viterbi ([`CharModel`]: duration
//! chains and a trie of the Morse codes) at six speeds turns that into text.
//!
//! The first engine is kept for the tests' comparison: a matched energy
//! detector over the spectrum (bins weighted by their average excess SNR,
//! S / (1 + S)), its score as a likelihood, an element-level Viterbi bank
//! ([`RsStream`]); and before that, thresholds with hysteresis ([`RsCw`]).
#![cfg_attr(not(test), allow(dead_code))]

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

const N: usize = 256;
const HOP: usize = 64;
/// Quantile the noise tracker aims at, and the exponential distribution's
/// value there (noise power in a bin is exponential): mean = q-value / Q_VAL.
const Q: f32 = 0.25;
const Q_VAL: f32 = 0.2877; // -ln(1 - 0.25)

/// The decoder's tunables.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// Integration of the detector score, in dots.
    pub integ: f32,
    /// Thresholds in noise sigmas (key on, stay on, squelch open).
    pub on_sigma: f32,
    pub off_sigma: f32,
    pub squelch_sigma: f32,
    /// ... and as fractions of the signal's level.
    pub on_frac: f32,
    pub off_frac: f32,
    /// Marks shorter than this (dots) are dropped; gaps shorter than
    /// `bridge` (dots) inside a mark are fades.
    pub min_mark: f32,
    pub bridge: f32,
}

impl Default for Params {
    fn default() -> Params {
        Params { integ: 0.7, on_sigma: 6.0, off_sigma: 3.0, squelch_sigma: 8.0, on_frac: 0.45, off_frac: 0.25, min_mark: 0.5, bridge: 0.35 }
    }
}

pub struct RsCw {
    pub p: Params,
    rate: f32,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<f32>,
    work: Vec<Complex32>,
    /// Bins searched (audio band).
    lo: usize,
    hi: usize,
    /// Per bin: noise quantile (log), average excess SNR.
    noise_log: Vec<f32>,
    shape: Vec<f32>,
    frames: u64,
    /// Detector output history for smoothing.
    zs: Vec<f32>,
    /// Signal level tracker (the score's upper quantile, log).
    level_log: f32,
    /// The score's noise statistics: its 10 % and 30 % quantiles (the key is
    /// up less than 70 % of the time, so these are noise).
    q10: f32,
    q30: f32,
    key: bool,
    /// Length of the current mark or space, frames.
    run: u32,
    /// Pending mark cut short by a fade: its length and the gap after it.
    pending_mark: Option<(u32, u32)>,
    dot: f32,
    /// Elements of the character being read ('.' '-').
    symbol: String,
    pub text: String,
    /// The last frame's detector score (for tests and displays).
    pub score: f32,
    /// Every frame's raw score, when asked for (batch decoding).
    pub history: Option<Vec<f32>>,
    /// Every frame's log-likelihood ratio (keyed or not), when asked for
    /// (the streaming Viterbi): the score lightly smoothed, noise centre and
    /// sigma from its 10 % and 30 % quantiles, the signal's level from its
    /// 90 % quantile (a few seconds), at least 5 sigma.
    pub llr_out: Option<Vec<f32>>,
    ys: [f32; 3],
    yq10: f32,
    yq30: f32,
    yq90: f32,
    /// The signal's level above the noise, in noise sigmas (0 until known).
    pub level_sigmas: f32,
}

impl RsCw {
    pub fn new(rate: f32) -> RsCw {
        let fft = FftPlanner::new().plan_fft_forward(N);
        let window = (0..N).map(|n| (std::f32::consts::PI * n as f32 / N as f32).sin().powi(2)).collect();
        let bin = |hz: f32| ((hz / rate * N as f32).round() as usize).min(N / 2);
        RsCw {
            p: Params::default(),
            rate,
            fft,
            window,
            buf: Vec::new(),
            work: vec![Complex32::default(); N],
            lo: bin(200.0),
            hi: bin(3000.0),
            noise_log: vec![0.0; N / 2 + 1],
            shape: vec![0.0; N / 2 + 1],
            frames: 0,
            zs: Vec::new(),
            level_log: 0.0,
            q10: -1.0,
            q30: 0.0,
            key: false,
            run: 0,
            pending_mark: None,
            dot: 0.06 * rate / HOP as f32,
            symbol: String::new(),
            text: String::new(),
            score: 0.0,
            history: None,
            llr_out: None,
            ys: [0.0; 3],
            yq10: -1.0,
            yq30: 0.0,
            yq90: 3.0,
            level_sigmas: 0.0,
        }
    }

    /// Only look between these audio frequencies (the CW filter).
    pub fn set_band(&mut self, lo: f32, hi: f32) {
        let bin = |hz: f32| ((hz / self.rate * N as f32).round() as usize).clamp(1, N / 2);
        self.lo = bin(lo.min(hi).max(100.0));
        self.hi = bin(lo.max(hi).min(self.rate / 2.0 - 100.0)).max(self.lo + 4);
    }

    /// Frames a second.
    fn fps(&self) -> f32 {
        self.rate / HOP as f32
    }

    pub fn wpm(&self) -> f32 {
        1.2 * self.fps() / self.dot
    }

    pub fn process(&mut self, audio: &[f32]) {
        self.buf.extend_from_slice(audio);
        let mut at = 0;
        while at + N <= self.buf.len() {
            for (o, (x, w)) in self.work.iter_mut().zip(self.buf[at..at + N].iter().zip(&self.window)) {
                *o = Complex32::new(x * w, 0.0);
            }
            self.fft.process(&mut self.work);
            self.frame();
            at += HOP;
        }
        self.buf.drain(..at);
    }

    fn frame(&mut self) {
        self.frames += 1;
        let warm = self.frames < 40;
        // Noise per bin (quantile in the log domain), excess SNR, the shape.
        let eta = if warm { 0.2 } else { 0.004 };

        let shape_a = 1.0 / (4.0 * self.fps());
        let mut num = 0f32;
        let mut wsum2 = 0f32;
        // The frame's gain against the noise model: the median bin's ratio
        // (most of the band is noise even under a wide signal). A receiver's
        // AGC moves the whole spectrum within a character; this takes it out.
        let gain = if self.frames > 1 && std::env::var_os("RSCW_AGC").is_some() {
            // bins the signal does not reach (by its learned shape) only
            let mut v: Vec<f32> = (self.lo..=self.hi)
                .filter(|&k| self.shape[k] < 0.1)
                .map(|k| self.work[k].norm_sqr().max(1e-20) / (self.noise_log[k].exp() / Q_VAL))
                .collect();
            if v.len() >= 12 {
                let mid = v.len() / 2;
                v.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
                // median of an exponential is ln 2 of its mean
                (v[mid] / std::f32::consts::LN_2).max(1e-6)
            } else {
                1.0
            }
        } else {
            1.0
        };
        for k in self.lo..=self.hi {
            let p = (self.work[k].norm_sqr() / gain).max(1e-20);
            let lp = p.ln();
            if self.frames == 1 {
                self.noise_log[k] = lp;
            }
            if lp < self.noise_log[k] {
                self.noise_log[k] -= eta * (1.0 - Q);
            } else {
                self.noise_log[k] += eta * Q;
            }
            let noise = self.noise_log[k].exp() / Q_VAL;
            let r = p / noise - 1.0;
            self.shape[k] += shape_a * (r - self.shape[k]);
        }
        // Weights from the shape (bins the signal does not reach get none).
        for k in self.lo..=self.hi {
            let s = self.shape[k].max(0.0);
            let w = s / (1.0 + s);
            if w > 0.0 {
                let p = self.work[k].norm_sqr() / gain;
                let noise = self.noise_log[k].exp() / Q_VAL;
                num += w * (p / noise - 1.0);
                wsum2 += w * w;
            }
        }
        let z = if wsum2 > 0.0 { num / wsum2.sqrt() } else { 0.0 };
        if let Some(h) = self.history.as_mut() {
            h.push(z);
        }
        if self.llr_out.is_some() && self.frames > 40 {
            self.ys = [self.ys[1], self.ys[2], z];
            let y = (self.ys[0] + self.ys[1] + self.ys[2]) / 3.0;
            let spread = (self.yq30 - self.yq10).max(0.05);
            // quick at first (seconds of history matter), then steady
            let early = self.frames < (4.0 * self.fps()) as u64;
            let kq: f32 = std::env::var("RSCW_KQ").ok().and_then(|x| x.parse().ok()).unwrap_or(0.01);
            let st = if early { 0.1 } else { kq } * spread;
            self.yq10 += if y < self.yq10 { -st * 0.9 } else { st * 0.1 };
            self.yq30 += if y < self.yq30 { -st * 0.7 } else { st * 0.3 };
            let sigma = spread / 0.757;
            let mu = self.yq30 + 0.524 * sigma;
            let k9: f32 = std::env::var("RSCW_K9").ok().and_then(|x| x.parse().ok()).unwrap_or(0.015);
            let st9 = if early { 0.05 } else { k9 } * sigma.max(0.1);
            self.yq90 += if y > self.yq90 { st9 * 0.9 } else { -st9 * 0.1 };
            let fl: f32 = std::env::var("RSCW_SFLOOR").ok().and_then(|x| x.parse().ok()).unwrap_or(7.0);
            let a = self.yq90.max(mu + fl * sigma);
            self.level_sigmas = if early { 0.0 } else { (self.yq90 - mu) / sigma };
            let s_on2 = sigma * sigma + (0.5 * (a - mu)).powi(2);
            let off = -0.5 * ((y - mu) / sigma).powi(2) - sigma.ln();
            let on = -0.5 * (y - a).powi(2) / s_on2 - 0.5 * s_on2.ln();
            let r = (on - off).clamp(-20.0, 20.0);
            self.llr_out.as_mut().unwrap().push(if r.is_finite() { r } else { 0.0 });
        }
        // Integrate over about 0.7 of a dot (matched to the element: the
        // noise grows as the root, the keyed signal in proportion).
        self.zs.push(z);
        let m = ((self.dot * self.p.integ).round() as usize).clamp(1, 60);
        if self.zs.len() > 256 {
            self.zs.drain(..self.zs.len() - 128);
        }
        let n = self.zs.len().min(m);
        let zs = self.zs[self.zs.len() - n..].iter().sum::<f32>() / (n as f32).sqrt();
        self.score = zs;
        if warm {
            return;
        }
        // Noise statistics of the score (quantiles, steps scaled to their
        // spread): centre and sigma from the 10 % and 30 % points.
        let spread = (self.q30 - self.q10).max(0.05);
        let st = 0.02 * spread;
        self.q10 += if zs < self.q10 { -st * 0.9 } else { st * 0.1 };
        self.q30 += if zs < self.q30 { -st * 0.7 } else { st * 0.3 };
        let sigma = spread / 0.757;
        let mu = self.q30 + 0.524 * sigma;
        // The signal's level: an upper quantile of the score (log), slow.
        let l = zs.max(0.5).ln();
        let lq = 0.9;
        if l > self.level_log {
            self.level_log += 0.01 * lq;
        } else {
            self.level_log -= 0.01 * (1.0 - lq);
        }
        let level = self.level_log.exp();
        // Hysteresis thresholds between noise (about 1) and the level.
        // The score is skewed (noise alone passes mu + 5 sigma about 1 % of
        // the time): 7 sigma to key, 3.5 to stay keyed, and nothing at all
        // while the signal's level is not clear of the noise (squelch).
        let p = self.p;
        let on = (p.on_frac * level).max(mu + p.on_sigma * sigma);
        let off = (p.off_frac * level).max(mu + p.off_sigma * sigma);
        let open = level > mu + p.squelch_sigma * sigma;
        let key = open && if self.key { zs > off } else { zs > on };
        if key == self.key {
            self.run += 1;
            return;
        }
        let len = self.run;
        self.run = 1;
        self.key = key;
        if key {
            self.space_ended(len);
        } else {
            self.mark_ended(len);
        }
    }

    fn mark_ended(&mut self, len: u32) {
        // A fade split this mark from the one before: join them.
        let len = match self.pending_mark.take() {
            Some((m, gap)) => m + gap + len,
            None => len,
        };
        if (len as f32) < self.p.min_mark * self.dot {
            return;
        }
        self.pending_mark = Some((len, 0));
    }

    fn space_ended(&mut self, len: u32) {
        let Some((m, _)) = self.pending_mark else {
            // A space after a dropped mark: part of the space before.
            self.flush_gap(len);
            return;
        };
        if (len as f32) < self.p.bridge * self.dot {
            // A fade inside the mark: keep it open.
            self.pending_mark = Some((m, len));
            return;
        }
        self.pending_mark = None;
        self.element(m);
        self.flush_gap(len);
    }

    fn element(&mut self, m: u32) {
        let u = m as f32 / self.dot;
        let dash = u > 2.0;
        // The dot length follows the elements (clamped to 5-60 WPM).
        let est = if dash { m as f32 / 3.0 } else { m as f32 };
        if (0.4..4.0).contains(&(est / self.dot)) {
            self.dot += 0.1 * (est - self.dot);
        }
        let fps = self.fps();
        self.dot = self.dot.clamp(1.2 * fps / 60.0, 1.2 * fps / 5.0);
        self.symbol.push(if dash { '-' } else { '.' });
        if self.symbol.len() > 8 {
            self.symbol.clear();
            self.text.push('*');
        }
    }

    fn flush_gap(&mut self, len: u32) {
        let u = len as f32 / self.dot;
        if u >= 2.0 && !self.symbol.is_empty() {
            let c = decode(&self.symbol);
            self.text.push(c);
            self.symbol.clear();
        }
        if u >= 5.0 && !self.text.ends_with(' ') && !self.text.is_empty() {
            self.text.push(' ');
        }
    }

    /// Finish what is pending (end of a recording).
    pub fn finish(&mut self) {
        if let Some((m, _)) = self.pending_mark.take() {
            self.element(m);
        }
        if !self.symbol.is_empty() {
            let c = decode(&self.symbol);
            self.text.push(c);
            self.symbol.clear();
        }
    }
}

/// The Morse model's states for one speed (see [`viterbi`]): per kind
/// (dot, dash, element gap, character gap, word gap) a chain of positions.
struct Model {
    base: [usize; 5],
    lo: [usize; 5],
    hi: [usize; 5],
    n: usize,
}

impl Model {
    fn new(dot: f32) -> Model {
        let d = [0.6f32, 1.6, 2.2, 4.2, 0.5, 1.8, 4.5, 7.5];
        let seg = |lo: f32, hi: f32| ((lo * dot).round().max(1.0) as usize, (hi * dot).round().max(2.0) as usize);
        let k = [seg(d[0], d[1]), seg(d[2], d[3]), seg(d[4], d[5]), seg(d[5], d[6]), seg(d[6], d[7])];
        let mut base = [0; 5];
        let mut n = 0;
        for i in 0..5 {
            base[i] = n;
            n += k[i].1;
        }
        Model { base, lo: k.map(|x| x.0), hi: k.map(|x| x.1), n }
    }
    fn kind(&self, s: usize) -> usize {
        (0..5).rev().find(|&i| s >= self.base[i]).unwrap()
    }
}

/// One speed's decoder, a frame at a time, keeping the last `hist`
/// frames' back pointers.
struct Bank {
    m: Model,
    score: Vec<f64>,
    next: Vec<f64>,
    back: std::collections::VecDeque<Vec<u16>>,
    hist: usize,
    /// Recent log-likelihood a frame (how well this speed explains the input).
    fit: f64,
}

impl Bank {
    fn new(dot: f32, hist: usize) -> Bank {
        let m = Model::new(dot);
        let mut score = vec![-1e30; m.n];
        score[m.base[4]] = 0.0;
        Bank { next: vec![-1e30; m.n], score, back: Default::default(), hist, fit: 0.0, m }
    }

    fn step(&mut self, l: f32) {
        const NEG: f64 = -1e29;
        let m = &self.m;
        let ln_half = (0.5f64).ln();
        let e_on = l as f64 / 2.0;
        let mut bp = vec![u16::MAX; m.n];
        self.next.iter_mut().for_each(|v| *v = -1e30);
        for ki in 0..5 {
            let e = if ki < 2 { e_on } else { -e_on };
            let (lo, hi) = (m.lo[ki], m.hi[ki]);
            let outs: &[usize] = if ki < 2 { &[2, 3, 4] } else { &[0, 1] };
            for pos in 0..hi {
                let s = m.base[ki] + pos;
                let sc = self.score[s];
                if sc < NEG {
                    continue;
                }
                let can_end = pos + 1 >= lo;
                let last = pos + 1 == hi;
                if !last {
                    let v = sc + if can_end { ln_half } else { 0.0 } + e;
                    if v > self.next[s + 1] {
                        self.next[s + 1] = v;
                        bp[s + 1] = s as u16;
                    }
                } else if ki == 4 {
                    let v = sc + e;
                    if v > self.next[s] {
                        self.next[s] = v;
                        bp[s] = s as u16;
                    }
                }
                if can_end {
                    let ln_out = if last { 0.0 } else { ln_half } - (outs.len() as f64).ln();
                    for &nk in outs {
                        let t = m.base[nk];
                        let e2 = if nk < 2 { e_on } else { -e_on };
                        let v = sc + ln_out + e2;
                        if v > self.next[t] {
                            self.next[t] = v;
                            bp[t] = s as u16;
                        }
                    }
                }
            }
        }
        std::mem::swap(&mut self.score, &mut self.next);
        // keep the numbers small; what is taken off is the frame's fit
        let best = self.score.iter().cloned().fold(f64::MIN, f64::max);
        self.score.iter_mut().for_each(|v| *v -= best);
        self.fit += 0.003 * (best - self.fit);
        self.back.push_back(bp);
        if self.back.len() > self.hist {
            self.back.pop_front();
        }
    }

    /// The best path's kinds for the last `n` frames (oldest first).
    fn path(&self, n: usize) -> Vec<u8> {
        let n = n.min(self.back.len());
        let mut s = (0..self.m.n).max_by(|&a, &b| self.score[a].total_cmp(&self.score[b])).unwrap_or(0);
        let mut out = vec![4u8; n];
        let len = self.back.len();
        for i in 0..n {
            let t = len - 1 - i;
            out[n - 1 - i] = self.m.kind(s) as u8;
            let b = self.back[t][s];
            if b == u16::MAX {
                break;
            }
            s = b as usize;
        }
        out
    }
}

/// The rain-scatter decoder, streaming: the front end's per-frame
/// likelihoods into a bank of speeds (10-28 WPM); every half second the text
/// of the best-fitting speed's path is committed up to two seconds back.
pub struct RsStream {
    front: RsCw,
    banks: Vec<Bank>,
    dots: Vec<f32>,
    frame: u64,
    /// Frames not yet committed (counted back from now).
    pending: usize,
    lag: usize,
    every: usize,
    since: usize,
    sym: String,
    prev: u8,
    pub text: String,
    /// New text since the last take.
    fresh: String,
}

impl RsStream {
    pub fn new(rate: f32) -> RsStream {
        let mut front = RsCw::new(rate);
        front.llr_out = Some(Vec::new());
        let fps = front.fps();
        let dots: Vec<f32> = [10.0f32, 12.0, 14.0, 17.0, 20.0, 24.0, 28.0].iter().map(|w| 1.2 * fps / w).collect();
        let hist = (6.0 * fps) as usize;
        RsStream {
            banks: dots.iter().map(|&d| Bank::new(d, hist)).collect(),
            dots,
            frame: 0,
            pending: 0,
            lag: (2.0 * fps) as usize,
            every: (0.5 * fps) as usize,
            since: 0,
            sym: String::new(),
            prev: 4,
            text: String::new(),
            fresh: String::new(),
            front,
        }
    }

    /// Only look for the signal between these audio frequencies.
    pub fn set_band(&mut self, lo: f32, hi: f32) {
        self.front.set_band(lo, hi);
    }

    pub fn wpm(&self) -> f32 {
        let b = (0..self.banks.len()).max_by(|&a, &b| self.banks[a].fit.total_cmp(&self.banks[b].fit)).unwrap_or(0);
        1.2 * self.front.fps() / self.dots[b]
    }

    pub fn process(&mut self, audio: &[f32]) {
        self.front.process(audio);
        let ls = std::mem::take(self.front.llr_out.as_mut().unwrap());
        for l in ls {
            for b in &mut self.banks {
                b.step(l);
            }
            self.frame += 1;
            self.pending += 1;
            self.since += 1;
            if self.since >= self.every && self.pending > self.lag {
                self.since = 0;
                self.commit(self.pending - self.lag);
            }
        }
    }

    /// Commit `n` frames (the oldest pending ones) from the best bank; with
    /// no signal to speak of (level under 6 sigma, or the first seconds),
    /// they go without text (noise decodes as a stream of E and T).
    fn commit(&mut self, n: usize) {
        let squelch: f32 = std::env::var("RSCW_SQUELCH").ok().and_then(|x| x.parse().ok()).unwrap_or(6.0);
        if self.front.level_sigmas < squelch {
            let n = n.min(self.pending);
            self.pending -= n;
            self.sym.clear();
            self.prev = 4;
            return;
        }
        let b = (0..self.banks.len()).max_by(|&a, &b| self.banks[a].fit.total_cmp(&self.banks[b].fit)).unwrap_or(0);
        let path = self.banks[b].path(self.pending);
        let n = n.min(path.len());
        let mut out = String::new();
        for &k in &path[..n] {
            if k != self.prev {
                match k {
                    0 => self.sym.push('.'),
                    1 => self.sym.push('-'),
                    3 | 4 => {
                        if !self.sym.is_empty() {
                            out.push(decode(&self.sym));
                            self.sym.clear();
                        }
                        if k == 4 && !out.ends_with(' ') && !(out.is_empty() && self.text.ends_with(' ')) {
                            out.push(' ');
                        }
                    }
                    _ => {}
                }
                if self.sym.len() > 8 {
                    self.sym.clear();
                }
                self.prev = k;
            }
        }
        self.pending -= n;
        self.text += &out;
        self.fresh += &out;
    }

    /// What was committed since the last call.
    pub fn take(&mut self) -> String {
        std::mem::take(&mut self.fresh)
    }

    /// Commit the rest (end of a recording).
    pub fn finish(&mut self) {
        let n = self.pending;
        if n > 0 {
            self.commit(n);
        }
        if !self.sym.is_empty() {
            self.text.push(decode(&self.sym));
            self.sym.clear();
        }
    }
}

/// Morse by Viterbi over the detector's scores (one a frame): elements and
/// gaps as chains of states with flexible durations (in dots: dot 0.6-1.6,
/// dash 2.2-4.2, element gap 0.5-1.8, character gap 1.8-4.5, word gap from
/// 4.5 on, staying as long as the silence lasts), Morse's grammar between
/// them, each frame scored against noise and a fading signal. `dot` in
/// frames. Returns the text and the path's log-likelihood per frame.
pub fn viterbi(llr: &[f32], dot: f32) -> (String, f64) {
    #[derive(Clone, Copy, PartialEq)]
    enum Kind {
        Dot,
        Dash,
        EGap,
        CGap,
        WGap,
    }
    let seg = |lo: f32, hi: f32| ((lo * dot).round().max(1.0) as usize, (hi * dot).round().max(2.0) as usize);
    // RSCW_DUR="dot_lo dot_hi dash_lo dash_hi egap_lo egap_hi cgap_hi wgap_hi"
    let d: Vec<f32> = std::env::var("RSCW_DUR")
        .ok()
        .map(|v| v.split_whitespace().filter_map(|x| x.parse().ok()).collect())
        .filter(|v: &Vec<f32>| v.len() == 8)
        .unwrap_or(vec![0.6, 1.6, 2.2, 4.2, 0.5, 1.8, 4.5, 7.5]);
    let kinds = [(Kind::Dot, seg(d[0], d[1])), (Kind::Dash, seg(d[2], d[3])), (Kind::EGap, seg(d[4], d[5])), (Kind::CGap, seg(d[5], d[6])), (Kind::WGap, seg(d[6], d[7]))];
    // States: for each kind, positions 1..=max (frames spent so far).
    let mut base = Vec::new();
    let mut n = 0;
    for (_, (_, hi)) in &kinds {
        base.push(n);
        n += hi;
    }
    let on = |k: Kind| matches!(k, Kind::Dot | Kind::Dash);
    let starts_after = |k: Kind| -> &'static [usize] {
        // kinds that may follow k
        match k {
            Kind::Dot | Kind::Dash => &[2, 3, 4],
            Kind::EGap => &[0, 1],
            Kind::CGap | Kind::WGap => &[0, 1],
        }
    };
    const NEG: f64 = -1e30;
    let mut score = vec![NEG; n];
    // start in a word gap
    score[base[4]] = 0.0;
    // back pointers per frame: previous state index
    let mut back: Vec<Vec<u32>> = Vec::with_capacity(llr.len());
    let ln_half = (0.5f64).ln();
    for &l in llr {
        let mut next = vec![NEG; n];
        let mut bp = vec![u32::MAX; n];
        for (ki, &(k, (lo, hi))) in kinds.iter().enumerate() {
            let e = if on(k) { l as f64 / 2.0 } else { -l as f64 / 2.0 };
            for pos in 0..hi {
                let s = base[ki] + pos;
                let sc = score[s];
                if sc <= NEG / 2.0 {
                    continue;
                }
                let can_end = pos + 1 >= lo;
                let last = pos + 1 == hi;
                // continue
                if !last {
                    let v = sc + if can_end { ln_half } else { 0.0 } + e;
                    let t = s + 1;
                    if v > next[t] {
                        next[t] = v;
                        bp[t] = s as u32;
                    }
                } else if k == Kind::WGap {
                    // silence goes on: stay
                    let v = sc + e;
                    if v > next[s] {
                        next[s] = v;
                        bp[s] = s as u32;
                    }
                }
                if can_end {
                    let outs = starts_after(k);
                    let ln_out = if last { 0.0 } else { ln_half } - (outs.len() as f64).ln();
                    for &nk in outs {
                        let t = base[nk];
                        let e2 = if on(kinds[nk].0) { l as f64 / 2.0 } else { -l as f64 / 2.0 };
                        let v = sc + ln_out + e2;
                        if v > next[t] {
                            next[t] = v;
                            bp[t] = s as u32;
                        }
                    }
                }
            }
        }
        score = next;
        back.push(bp);
    }
    // Trace back from the best end state.
    let (mut s, best) = score.iter().enumerate().fold((0, NEG), |m, (i, &v)| if v > m.1 { (i, v) } else { m });
    let kind_of = |s: usize| (0..5).rev().find(|&i| s >= base[i]).unwrap();
    let mut path = vec![0u8; llr.len()];
    for t in (0..llr.len()).rev() {
        path[t] = kind_of(s) as u8;
        let b = back[t][s];
        if b == u32::MAX {
            break;
        }
        s = b as usize;
    }
    // Segments to text: a new segment starts where the kind changes or a
    // state is the first of its chain (dot after dot has a gap between).
    let mut text = String::new();
    let mut sym = String::new();
    let mut prev = 4u8;
    for &k in &path {
        if k != prev {
            match k {
                0 => sym.push('.'),
                1 => sym.push('-'),
                3 | 4 => {
                    if !sym.is_empty() {
                        text.push(decode(&sym));
                        sym.clear();
                    }
                    if k == 4 && !text.ends_with(' ') && !text.is_empty() {
                        text.push(' ');
                    }
                }
                _ => {}
            }
            prev = k;
        }
    }
    if !sym.is_empty() {
        text.push(decode(&sym));
    }
    (text, best / llr.len().max(1) as f64)
}

/// The dot length (frames) the marks fit best: the runs where the keyed
/// hypothesis wins (a little smoothed), each in 1 or 3 dots, the dot from
/// `lo..=hi` with the least squared misfit (in dots, a mark's weight its
/// length, marks shorter than 0.4 dot not counted).
pub fn dot_from_marks(llr: &[f32], lo: f32, hi: f32) -> f32 {
    let mut marks = Vec::new();
    let mut run = 0usize;
    let mut acc = 0f32;
    for (i, &l) in llr.iter().enumerate() {
        acc = 0.7 * acc + 0.3 * l;
        let _ = i;
        if acc > 0.0 {
            run += 1;
        } else if run > 0 {
            marks.push(run as f32);
            run = 0;
        }
    }
    let mut best = (f32::MAX, lo);
    let mut d = lo;
    while d <= hi {
        let (mut e, mut w) = (0f32, 0f32);
        for &m in &marks {
            let u = m / d;
            if u < 0.4 || u > 6.0 {
                continue;
            }
            let err = (u - 1.0).powi(2).min(((u - 3.0) / 3.0).powi(2) * 9.0 / 4.0);
            e += err;
            w += 1.0;
        }
        // misfit per mark, a mild pull toward using many marks
        let v = if w >= 4.0 { e / w + 2.0 / w } else { f32::MAX };
        if v < best.0 {
            best = (v, d);
        }
        d *= 1.03;
    }
    best.1
}

/// How much a decode looks like Morse text: characters in words of three or
/// more valid ones, less twice the undecodable ones and one a stray single
/// letter.
pub fn text_quality(t: &str) -> f32 {
    let mut q = 0f32;
    for w in t.split_whitespace() {
        let bad = w.chars().filter(|c| *c == '*').count() as f32;
        let n = w.chars().count();
        if n >= 3 && bad == 0.0 {
            q += n as f32;
        } else if n == 1 {
            q -= 1.0;
        }
        q -= 2.0 * bad;
    }
    q
}

/// Per-frame log-likelihood ratios (keyed against not) from the raw
/// scores: noise centre and sigma from the scores' 10 % and 30 % quantiles,
/// the signal's level from a rolling 90 % quantile (+-2.5 s: it fades), a
/// keyed frame modelled as that level with a spread of half of it.
pub fn llrs(z: &[f32], fps: f32) -> Vec<f32> {
    let n = z.len();
    if n == 0 {
        return Vec::new();
    }
    // light smoothing (frames overlap by 3/4)
    let y: Vec<f32> = (0..n).map(|i| z[i.saturating_sub(1)..(i + 2).min(n)].iter().sum::<f32>() / (i + 2).min(n).saturating_sub(i.saturating_sub(1)) as f32).collect();
    let mut sorted = y.clone();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let q = |p: f32| sorted[((n - 1) as f32 * p) as usize];
    let sigma = ((q(0.3) - q(0.1)) / 0.757).max(1e-3);
    let mu = q(0.3) + 0.524 * sigma;
    let half = (2.5 * fps) as usize;
    let step = (0.25 * fps).max(1.0) as usize;
    let mut level = vec![0f32; n];
    let mut i = 0;
    while i < n {
        let w = &y[i.saturating_sub(half)..(i + half).min(n)];
        let mut v = w.to_vec();
        v.sort_by(|a, b| a.total_cmp(b));
        let floor: f32 = std::env::var("RSCW_FLOOR").ok().and_then(|x| x.parse().ok()).unwrap_or(5.0);
        let a = v[((v.len() - 1) as f32 * 0.9) as usize].max(mu + floor * sigma);
        for l in &mut level[i..(i + step).min(n)] {
            *l = a;
        }
        i += step;
    }
    y.iter()
        .zip(&level)
        .map(|(&v, &a)| {
            let s_on2 = sigma * sigma + (0.5 * (a - mu)).powi(2);
            let off = -0.5 * ((v - mu) / sigma).powi(2) - sigma.ln();
            let on = -0.5 * (v - a).powi(2) / s_on2 - 0.5 * s_on2.ln();
            let r = (on - off).clamp(-20.0, 20.0);
            if r.is_finite() { r } else { 0.0 }
        })
        .collect()
}

/// A trie of the Morse codes: node 0 is the root; each other node is the
/// element sequence leading to it (its last element in `dash`), with the
/// character it spells if any.
struct Trie {
    child: Vec<[usize; 2]>,
    dash: Vec<bool>,
    ch: Vec<Option<char>>,
    depth: Vec<usize>,
}

impl Trie {
    fn new() -> Trie {
        let mut t = Trie { child: vec![[0; 2]], dash: vec![false], ch: vec![None], depth: vec![0] };
        for &(code, c) in MORSE {
            let mut n = 0;
            for e in code.chars() {
                let d = (e == '-') as usize;
                if t.child[n][d] == 0 {
                    t.child.push([0; 2]);
                    t.dash.push(d == 1);
                    t.ch.push(None);
                    t.depth.push(t.depth[n] + 1);
                    t.child[n][d] = t.child.len() - 1;
                }
                n = t.child[n][d];
            }
            t.ch[n] = Some(c);
        }
        t
    }
}

/// The character-level Morse model for one speed: chains of duration
/// states, a mark per trie node, an element gap per node with children, one
/// character gap and one word gap; the elements between character gaps must
/// spell a Morse character (so one misread element cannot turn into an
/// unknown symbol).
struct CharModel {
    chains: Vec<Chain>,
    ns: usize,
    chain_of: Vec<u16>,
    wg: usize,
    ch: Vec<Option<char>>,
    /// Log bonus for ending a character at each mark chain: the element
    /// boundaries' choices make short characters cheap (noise decodes as E
    /// and T); `len_bonus` per element evens that out.
    end_bonus: Vec<f64>,
    /// Per chain: its successors (first state, key down, a gap: takes the
    /// end bonus) and the log of their count, for [`CharBank::step`].
    succ: Vec<Vec<(usize, bool, bool)>>,
    ln_nouts: Vec<f64>,
}

struct Chain {
    on: bool,
    lo: usize,
    hi: usize,
    /// 0 mark, 1 element gap (of `node`), 2 character gap, 3 word gap
    kind: u8,
    node: usize,
    base: usize,
    outs: Vec<usize>,
}

impl CharModel {
    fn new(dot: f32) -> CharModel {
        let tr = Trie::new();
        let nn = tr.child.len();
        let seg = |lo: f32, hi: f32| ((lo * dot).round().max(1.0) as usize, (hi * dot).round().max(2.0) as usize);
        let d: Vec<f32> = std::env::var("RSCW_DUR")
            .ok()
            .map(|v| v.split_whitespace().filter_map(|x| x.parse().ok()).collect())
            .filter(|v: &Vec<f32>| v.len() == 8)
            .unwrap_or(vec![0.7, 1.5, 2.4, 4.0, 0.6, 1.7, 4.0, 7.0]);
        let (dotd, dashd, egd, cgd, wgd) = (seg(d[0], d[1]), seg(d[2], d[3]), seg(d[4], d[5]), seg(d[5], d[6]), seg(d[6], d[7]));
        let mut chains: Vec<Chain> = Vec::new();
        let mut mark_of = vec![usize::MAX; nn];
        let mut egap_of = vec![usize::MAX; nn];
        let mut base = 0;
        for n in 1..nn {
            let (lo, hi) = if tr.dash[n] { dashd } else { dotd };
            mark_of[n] = chains.len();
            chains.push(Chain { on: true, lo, hi, kind: 0, node: n, base, outs: vec![] });
            base += hi;
        }
        for n in 1..nn {
            if tr.child[n] != [0, 0] {
                egap_of[n] = chains.len();
                chains.push(Chain { on: false, lo: egd.0, hi: egd.1, kind: 1, node: n, base, outs: vec![] });
                base += egd.1;
            }
        }
        let cg = chains.len();
        chains.push(Chain { on: false, lo: cgd.0, hi: cgd.1, kind: 2, node: 0, base, outs: vec![] });
        base += cgd.1;
        let wg = chains.len();
        chains.push(Chain { on: false, lo: wgd.0, hi: wgd.1, kind: 3, node: 0, base, outs: vec![] });
        base += wgd.1;
        for c in 0..chains.len() {
            let n = chains[c].node;
            let outs: Vec<usize> = match chains[c].kind {
                0 => {
                    let mut o = Vec::new();
                    if egap_of[n] != usize::MAX {
                        o.push(egap_of[n]);
                    }
                    if tr.ch[n].is_some() {
                        o.push(cg);
                        o.push(wg);
                    }
                    o
                }
                1 => tr.child[n].iter().filter(|&&k| k != 0).map(|&k| mark_of[k]).collect(),
                _ => tr.child[0].iter().filter(|&&k| k != 0).map(|&k| mark_of[k]).collect(),
            };
            chains[c].outs = outs;
        }
        let mut chain_of = vec![0u16; base];
        for (ci, c) in chains.iter().enumerate() {
            for p in 0..c.hi {
                chain_of[c.base + p] = ci as u16;
            }
        }
        let lb: f64 = std::env::var("RSCW_LEN").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
        let end_bonus = chains.iter().map(|c| if c.kind == 0 { lb * tr.depth[c.node] as f64 } else { 0.0 }).collect();
        let succ = chains.iter().map(|c| c.outs.iter().map(|&o| (chains[o].base, chains[o].on, chains[o].kind >= 2)).collect()).collect();
        let ln_nouts = chains.iter().map(|c| (c.outs.len() as f64).ln()).collect();
        CharModel { chains, ns: base, chain_of, wg, ch: tr.ch, end_bonus, succ, ln_nouts }
    }

    /// Text from consecutive frames' chains: a character where a mark ends
    /// in a character or word gap (a space after the word gap). `prev` is
    /// the chain before the first frame; the last chain comes back.
    fn text(&self, path: &[u16], mut prev: u16, out: &mut String) -> u16 {
        for &c in path {
            if c != prev {
                let (pk, ck) = (self.chains[prev as usize].kind, self.chains[c as usize].kind);
                if pk == 0 && (ck == 2 || ck == 3) {
                    if let Some(ch) = self.ch[self.chains[prev as usize].node] {
                        out.push(ch);
                    }
                    if ck == 3 {
                        out.push(' ');
                    }
                }
                prev = c;
            }
        }
        prev
    }
}

/// One speed's character-level decoder, a frame at a time, keeping the last
/// `hist` frames' back pointers (all of them when `hist` is 0).
struct CharBank {
    m: CharModel,
    // (f32: the A9 is memory bound here; scores are relative to the best)
    score: Vec<f32>,
    next: Vec<f32>,
    back: std::collections::VecDeque<Vec<u16>>,
    hist: usize,
    /// Recent log-likelihood a frame (how well this speed explains the input).
    fit: f64,
    /// Sum of what was taken off the scores (the whole path's likelihood).
    total: f64,
    beam: f64,
    /// The best score of the last frame: taken off each score as it is read
    /// (instead of a pass over all of them).
    off: f32,
    /// Chains with a state within the beam (the others are skipped whole).
    live: Vec<bool>,
    cmax: Vec<f32>,
}

impl CharBank {
    fn new(dot: f32, hist: usize) -> CharBank {
        let m = CharModel::new(dot);
        let mut score = vec![-1e30; m.ns];
        score[m.chains[m.wg].base] = 0.0;
        let beam = std::env::var("RSCW_BEAM").ok().and_then(|v| v.parse().ok()).unwrap_or(if hist > 0 { 30.0 } else { 1e29 });
        CharBank { next: vec![-1e30; m.ns], score, back: Default::default(), hist, fit: 0.0, total: 0.0, beam, off: 0.0, live: vec![true; m.chains.len()], cmax: Vec::new(), m }
    }

    fn step(&mut self, l: f32) {
        // states further than BEAM below the best are dropped
        let neg = -self.beam as f32;
        let ln_half = (0.5f32).ln();
        let (e_on, e_off) = (l / 2.0, -l / 2.0);
        // (the oldest frame's back pointers, when they go, are reused)
        let mut bp = match self.back.front() {
            Some(_) if self.hist > 0 && self.back.len() >= self.hist => {
                let mut v = self.back.pop_front().unwrap_or_default();
                v.fill(u16::MAX);
                v
            }
            _ => vec![u16::MAX; self.m.ns],
        };
        self.next.iter_mut().for_each(|v| *v = -1e30);
        for (ci, c) in self.m.chains.iter().enumerate() {
            if !self.live[ci] {
                continue;
            }
            let e = if c.on { e_on } else { e_off };
            let (succ, eb) = (&self.m.succ[ci], self.m.end_bonus[ci] as f32);
            for pos in 0..c.hi {
                let s = c.base + pos;
                let sc = self.score[s] - self.off;
                if sc < neg {
                    continue;
                }
                let can_end = pos + 1 >= c.lo;
                let last = pos + 1 == c.hi;
                if !last {
                    let v = sc + if can_end { ln_half } else { 0.0 } + e;
                    if v > self.next[s + 1] {
                        self.next[s + 1] = v;
                        bp[s + 1] = s as u16;
                    }
                } else if c.kind == 3 {
                    let v = sc + e;
                    if v > self.next[s] {
                        self.next[s] = v;
                        bp[s] = s as u16;
                    }
                }
                if can_end && !succ.is_empty() {
                    let ln_out = if last { 0.0 } else { ln_half } - self.m.ln_nouts[ci] as f32;
                    for &(t, on, gap) in succ {
                        let bonus = if gap { eb } else { 0.0 };
                        let v = sc + ln_out + bonus + if on { e_on } else { e_off };
                        if v > self.next[t] {
                            self.next[t] = v;
                            bp[t] = s as u16;
                        }
                    }
                }
            }
        }
        std::mem::swap(&mut self.score, &mut self.next);
        // the best score, and each chain's (whether any of it is in the beam)
        let mut cmax = std::mem::take(&mut self.cmax);
        cmax.clear();
        cmax.extend(self.m.chains.iter().map(|c| self.score[c.base..c.base + c.hi].iter().cloned().fold(f32::MIN, f32::max)));
        let best = cmax.iter().cloned().fold(f32::MIN, f32::max);
        for (l, &m) in self.live.iter_mut().zip(&cmax) {
            *l = !(m - best < neg);
        }
        self.cmax = cmax;
        self.off = best;
        let best = best as f64;
        self.total += best;
        self.fit += 0.003 * (best - self.fit);
        self.back.push_back(bp);
        if self.hist > 0 && self.back.len() > self.hist {
            self.back.pop_front();
        }
    }

    /// The best path's chains for the last `n` frames (oldest first).
    fn path(&self, n: usize) -> Vec<u16> {
        let n = n.min(self.back.len());
        let mut s = (0..self.m.ns).max_by(|&a, &b| self.score[a].total_cmp(&self.score[b])).unwrap_or(0);
        let mut out = vec![self.m.wg as u16; n];
        let len = self.back.len();
        for i in 0..n {
            let t = len - 1 - i;
            out[n - 1 - i] = self.m.chain_of[s];
            let b = self.back[t][s];
            if b == u16::MAX {
                break;
            }
            s = b as usize;
        }
        out
    }
}

/// Character-level Viterbi over a whole recording (see [`CharModel`]):
/// the text and the path's log-likelihood per frame.
pub fn viterbi_chars(llr: &[f32], dot: f32) -> (String, f64) {
    let mut b = CharBank::new(dot, 0);
    for &l in llr {
        b.step(l);
    }
    let path = b.path(llr.len());
    let mut text = String::new();
    let last = b.m.text(&path, b.m.wg as u16, &mut text);
    let lc = &b.m.chains[last as usize];
    if lc.kind <= 1 {
        if let Some(ch) = b.m.ch[lc.node] {
            text.push(ch);
        }
    }
    (text, b.total / llr.len().max(1) as f64)
}

/// Seconds of keying the Morse rhythm test looks at.
const MORSE_WINDOW_S: f32 = 4.0;
/// Gap stretches tried (slow fists, Farnsworth), as rain-scatter-decoder.
const SPACE_RATIOS: [f32; 14] = [1.0, 1.15, 1.35, 1.6, 1.9, 2.2, 2.6, 3.0, 3.5, 4.1, 4.8, 5.6, 6.5, 8.0];

/// How Morse-like a keying is, 0..1 (port of rain-scatter-decoder's
/// `_quant_fit`): Morse run lengths are quantised against one time unit,
/// elements 1 and 3, gaps 1, 3 and 7 (gaps one-sided: only a gap shorter
/// than its target counts against, with a fitted stretch); speech run
/// lengths are not. Units of 28-130 ms (about 9-43 WPM). The runs cut
/// by the window's edges are left out. `dt`: seconds a frame.
pub fn morse_fit(keyed: &[bool], dt: f32) -> f32 {
    let mut runs: Vec<(bool, f32)> = Vec::new();
    for &k in keyed {
        match runs.last_mut() {
            Some((v, n)) if *v == k => *n += dt,
            _ => runs.push((k, dt)),
        }
    }
    if runs.len() < 3 {
        return 0.0;
    }
    let runs = &runs[1..runs.len() - 1];
    let on: Vec<f32> = runs.iter().filter(|r| r.0).map(|r| r.1).collect();
    let off: Vec<f32> = runs.iter().filter(|r| !r.0).map(|r| r.1).collect();
    if runs.len() < 8 || on.len() < 4 || off.len() < 3 {
        return 0.0;
    }
    let n = (on.len() + off.len()) as f32;
    let (umin, umax) = (0.028f32, 0.130f32);
    let mut best = f32::MAX;
    for i in 0..80 {
        let u = (umin.ln() + (umax.ln() - umin.ln()) * i as f32 / 79.0).exp();
        let c_on: f32 = on.iter().map(|&l| {
            let k = l / u;
            let t = if k < 2.0 { 1.0 } else { 3.0 };
            (k - t).abs() / t
        }).sum();
        let mut c_gap = f32::MAX;
        for sr in SPACE_RATIOS {
            let c: f32 = off.iter().map(|&l| {
                let k = l / u;
                let (t, two) = [(1.0, true), (3.0 * sr, false), (7.0 * sr, false)]
                    .into_iter()
                    .min_by(|a, b| (k - a.0).abs().total_cmp(&(k - b.0).abs()))
                    .unwrap();
                (if two { (k - t).abs() } else { (t - k).max(0.0) }) / t
            }).sum();
            c_gap = c_gap.min(c);
        }
        best = best.min((c_on + c_gap) / n);
    }
    (1.0 - best / 0.45).max(0.0)
}

/// The rain-scatter decoder with the neural keying detector
/// ([`crate::rsnn`]) and character-level Viterbi banks (8-30 WPM), streaming:
/// every half second the best-fitting speed's text is committed up to two
/// seconds back. No text while the network has not been sure of a key-down
/// for the last few seconds (noise alone decodes as E and T).
pub struct RsNnStream {
    nn: crate::rsnn::RsNn,
    banks: Vec<CharBank>,
    dots: Vec<f32>,
    fps: f32,
    pending: usize,
    lag: usize,
    every: usize,
    since: usize,
    prev: u16,
    /// Frames since the network last gave a confident key-down.
    quiet: usize,
    llr: Vec<f32>,
    dec: usize,
    acc: (f32, usize),
    squelch_s: f32,
    /// The banks are resting (squelched).
    idle: bool,
    /// An LLR above this is a confident key-down (opens the squelch).
    sure: f32,
    pub text: String,
    fresh: String,
    /// Seconds in the network (features included) and in the banks.
    pub prof: (f64, f64),
    /// Decoder frames so far, squelched ones included (for times).
    frames: usize,
    /// The LLRs of the frames not yet committed (as the banks saw them).
    recent: std::collections::VecDeque<f32>,
    /// The character being read: its frames' agreement with the path's
    /// key states (LLR signed by key down / up), summed, and their count.
    span: (f32, usize),
    /// A character is shown only if its mean agreement per network frame
    /// is at least this (RSCW_CHARCONF; -inf: every character).
    char_conf: f32,
    /// Every shown character (and word gap) with its time in seconds.
    pub timed: Vec<(f32, char)>,
    /// A word of at most this many characters, all of the shortest codes
    /// (E T I A N M: what noise decodes as), is not shown (RSCW_JUNK; 0:
    /// off). Only such a word's start waits, until a longer or another
    /// letter shows it is real.
    junk_len: usize,
    hold: Vec<(usize, char)>,
    word_ok: bool,
    /// The network's LLRs of the last few seconds (the Morse rhythm test).
    window: std::collections::VecDeque<f32>,
    /// How Morse-like the keying in the window is, 0..1 ([`morse_fit`]),
    /// as of the last commit; text is shown only at RSCW_MORSE or above
    /// (-inf: off).
    pub morse: f32,
    morse_min: f32,
    /// Each commit's rhythm score with its time (while the test is on).
    pub morse_log: Vec<(f32, f32)>,
}

impl RsNnStream {
    pub fn new(rate: f32) -> RsNnStream {
        let nn = crate::rsnn::RsNn::new();
        // the decoder runs at about 94 frames a second (a network at hop 64:
        // pairs of its LLRs summed)
        let dec: usize = std::env::var("RSCW_DEC").ok().and_then(|v| v.parse().ok()).unwrap_or((128 / nn.hop()).max(1));
        let fps = rate / nn.hop() as f32 / dec as f32;
        let wpms: Vec<f32> = std::env::var("RSCW_WPMS")
            .ok()
            .map(|v| v.split_whitespace().filter_map(|x| x.parse().ok()).collect())
            .unwrap_or(vec![9.5f32, 11.0, 12.5, 14.0, 15.5, 17.5, 20.0, 23.0, 27.0]);
        let dots: Vec<f32> = wpms.iter().map(|w| 1.2 * fps / w).collect();
        let hist = (6.0 * fps) as usize;
        RsNnStream {
            nn,
            banks: dots.iter().map(|&d| CharBank::new(d, hist)).collect(),
            dots,
            fps,
            pending: 0,
            lag: (std::env::var("RSCW_LAG").ok().and_then(|v| v.parse().ok()).unwrap_or(2.0f32) * fps) as usize,
            every: (0.5 * fps) as usize,
            since: 0,
            prev: 0,
            quiet: usize::MAX / 2,
            llr: Vec::new(),
            dec,
            acc: (0.0, 0),
            squelch_s: std::env::var("RSCW_SQS").ok().and_then(|v| v.parse().ok()).unwrap_or(4.0),
            idle: false,
            sure: std::env::var("RSCW_SURE").ok().and_then(|v| v.parse().ok()).unwrap_or(5.0),
            text: String::new(),
            fresh: String::new(),
            prof: (0.0, 0.0),
            frames: 0,
            recent: std::collections::VecDeque::new(),
            span: (0.0, 0),
            // (the gates tuned on half of 62 fragments operators labelled,
            // 2026-10-01: with the rhythm test the character gate could
            // ease from 5 to 4; text on voice and noise 88 % -> 6 %)
            char_conf: std::env::var("RSCW_CHARCONF").ok().and_then(|v| v.parse().ok()).unwrap_or(4.0),
            timed: Vec::new(),
            junk_len: std::env::var("RSCW_JUNK").ok().and_then(|v| v.parse().ok()).unwrap_or(2),
            hold: Vec::new(),
            word_ok: false,
            window: std::collections::VecDeque::new(),
            morse: 0.0,
            morse_log: Vec::new(),
            // just above every fragment without Morse in the tuning half
            // (their highest: 0.42; Morse mostly 0.65-0.96)
            morse_min: std::env::var("RSCW_MORSE").ok().and_then(|v| v.parse().ok()).unwrap_or(0.45),
        }
    }

    pub fn wpm(&self) -> f32 {
        let b = (0..self.banks.len()).max_by(|&a, &b| self.banks[a].fit.total_cmp(&self.banks[b].fit)).unwrap_or(0);
        1.2 * self.fps / self.dots[b]
    }

    /// The network looks at 200-3000 Hz itself and finds the signal there;
    /// the CW filter only limits what reaches it.
    pub fn set_band(&mut self, _lo: f32, _hi: f32) {}

    pub fn process(&mut self, audio: &[f32]) {
        self.llr.clear();
        let mut llr = std::mem::take(&mut self.llr);
        let t0 = std::time::Instant::now();
        self.nn.process(audio, &mut llr);
        let t1 = std::time::Instant::now();
        self.prof.0 += (t1 - t0).as_secs_f64();
        for &l0 in &llr {
            self.acc = (self.acc.0 + l0, self.acc.1 + 1);
            if self.acc.1 < self.dec {
                continue;
            }
            let l = self.acc.0;
            self.acc = (0.0, 0);
            self.frames += 1;
            self.window.push_back(l);
            if self.window.len() as f32 > MORSE_WINDOW_S * self.fps {
                self.window.pop_front();
            }
            self.quiet = if l > self.sure * self.dec as f32 { 0 } else { self.quiet.saturating_add(1) };
            // Squelched (no confident key-down for a while): nothing would
            // be shown, so the banks rest (on noise the beam prunes little
            // and they cost most); they start afresh at the next key-down.
            if self.quiet as f32 > self.squelch_s * self.fps + self.lag as f32 {
                if !self.idle {
                    let hist = (6.0 * self.fps) as usize;
                    self.banks = self.dots.iter().map(|&d| CharBank::new(d, hist)).collect();
                    self.pending = 0;
                    self.since = 0;
                    self.prev = 0;
                    self.recent.clear();
                    self.span = (0.0, 0);
                    self.idle = true;
                }
                continue;
            }
            self.idle = false;
            for b in &mut self.banks {
                b.step(l);
            }
            self.recent.push_back(l);
            self.pending += 1;
            self.since += 1;
            if self.since >= self.every && self.pending > self.lag {
                self.since = 0;
                self.commit(self.pending - self.lag);
            }
        }
        self.llr = llr;
        self.prof.1 += t1.elapsed().as_secs_f64();
    }

    fn commit(&mut self, n: usize) {
        let b = (0..self.banks.len()).max_by(|&a, &b| self.banks[a].fit.total_cmp(&self.banks[b].fit)).unwrap_or(0);
        let path = self.banks[b].path(self.pending);
        let n = n.min(path.len());
        // the first committed frame's number (pending frames are the latest)
        let start = self.frames - self.pending;
        let llrs: Vec<f32> = self.recent.drain(..n.min(self.recent.len())).collect();
        let m = &self.banks[b].m;
        let mut prev = if (self.prev as usize) < m.chains.len() { self.prev } else { m.wg as u16 };
        let mut span = self.span;
        let mut events: Vec<(usize, char)> = Vec::new();
        // as CharModel::text, with each character's agreement and time
        for (i, &c) in path[..n].iter().enumerate() {
            let ch = &m.chains[c as usize];
            let l = llrs.get(i).copied().unwrap_or(0.0);
            span = (span.0 + if ch.on { l } else { -l }, span.1 + 1);
            if c != prev {
                let (pk, ck) = (m.chains[prev as usize].kind, ch.kind);
                if pk == 0 && (ck == 2 || ck == 3) {
                    if let Some(x) = m.ch[m.chains[prev as usize].node] {
                        if span.0 / (span.1.max(1) * self.dec) as f32 >= self.char_conf {
                            events.push((start + i, x));
                        }
                    }
                    span = (0.0, 0);
                    if ck == 3 {
                        events.push((start + i, ' '));
                    }
                }
                prev = c;
            }
        }
        self.prev = prev;
        self.span = span;
        self.pending -= n;
        // squelch: the network has been unsure for over 4 s
        if self.quiet as f32 > self.squelch_s * self.fps + self.lag as f32 {
            return;
        }
        // and the keying must look like Morse (speech does not)
        if self.morse_min > f32::NEG_INFINITY {
            let keyed: Vec<bool> = self.window.iter().map(|&l| l > 0.0).collect();
            self.morse = morse_fit(&keyed, 1.0 / self.fps);
            self.morse_log.push((self.frames as f32 / self.fps, self.morse));
            if self.morse < self.morse_min {
                return;
            }
        }
        self.show(events);
    }

    fn show(&mut self, events: Vec<(usize, char)>) {
        for (f, x) in events {
            if x == ' ' {
                // a word that never showed itself real is dropped
                self.hold.clear();
                self.word_ok = false;
                if !(self.text.is_empty() || self.text.ends_with(' ')) {
                    self.put(f, ' ');
                }
                continue;
            }
            if self.word_ok || self.junk_len == 0 {
                self.put(f, x);
                continue;
            }
            self.hold.push((f, x));
            if self.hold.len() > self.junk_len || !"ETIANM".contains(x) {
                self.word_ok = true;
                for (f, x) in std::mem::take(&mut self.hold) {
                    self.put(f, x);
                }
            }
        }
    }

    fn put(&mut self, f: usize, x: char) {
        self.text.push(x);
        self.fresh.push(x);
        self.timed.push((f as f32 / self.fps, x));
    }

    /// What was committed since the last call.
    pub fn take(&mut self) -> String {
        std::mem::take(&mut self.fresh)
    }

    pub fn finish(&mut self) {
        // the network's last frames wait for context after them (about
        // 2.7 s for the larger networks): silence releases them, as the
        // zero padding at a batch's end would (the endings were lost)
        let tail = vec![0f32; self.nn.latency()];
        self.process(&tail);
        let n = self.pending;
        if n > 0 {
            self.commit(n);
        }
        // the character still being read when the audio stops (as
        // viterbi_chars does): it had no gap after it to end it
        let b = (0..self.banks.len()).max_by(|&a, &b| self.banks[a].fit.total_cmp(&self.banks[b].fit)).unwrap_or(0);
        let m = &self.banks[b].m;
        if self.idle || (self.prev as usize) >= m.chains.len() {
            return;
        }
        let c = &m.chains[self.prev as usize];
        if c.kind <= 1 && self.span.1 > 0 && self.span.0 / (self.span.1 * self.dec) as f32 >= self.char_conf {
            if let Some(x) = m.ch[c.node] {
                self.span = (0.0, 0);
                self.show(vec![(self.frames, x)]);
            }
        }
    }
}

const MORSE: &[(&str, char)] = &[
    (".-", 'A'), ("-...", 'B'), ("-.-.", 'C'), ("-..", 'D'), (".", 'E'), ("..-.", 'F'), ("--.", 'G'), ("....", 'H'),
    ("..", 'I'), (".---", 'J'), ("-.-", 'K'), (".-..", 'L'), ("--", 'M'), ("-.", 'N'), ("---", 'O'), (".--.", 'P'),
    ("--.-", 'Q'), (".-.", 'R'), ("...", 'S'), ("-", 'T'), ("..-", 'U'), ("...-", 'V'), (".--", 'W'), ("-..-", 'X'),
    ("-.--", 'Y'), ("--..", 'Z'), (".----", '1'), ("..---", '2'), ("...--", '3'), ("....-", '4'), (".....", '5'),
    ("-....", '6'), ("--...", '7'), ("---..", '8'), ("----.", '9'), ("-----", '0'), ("-..-.", '/'), ("..--..", '?'),
    (".-.-.-", '.'), ("--..--", ','), ("-...-", '='), (".-.-.", '+'),
];

fn decode(s: &str) -> char {
    const T: &[(&str, char)] = &[
        (".-", 'A'), ("-...", 'B'), ("-.-.", 'C'), ("-..", 'D'), (".", 'E'), ("..-.", 'F'), ("--.", 'G'), ("....", 'H'),
        ("..", 'I'), (".---", 'J'), ("-.-", 'K'), (".-..", 'L'), ("--", 'M'), ("-.", 'N'), ("---", 'O'), (".--.", 'P'),
        ("--.-", 'Q'), (".-.", 'R'), ("...", 'S'), ("-", 'T'), ("..-", 'U'), ("...-", 'V'), (".--", 'W'), ("-..-", 'X'),
        ("-.--", 'Y'), ("--..", 'Z'), (".----", '1'), ("..---", '2'), ("...--", '3'), ("....-", '4'), (".....", '5'),
        ("-....", '6'), ("--...", '7'), ("---..", '8'), ("----.", '9'), ("-----", '0'), ("-..-.", '/'), ("..--..", '?'),
        (".-.-.-", '.'), ("--..--", ','), ("-...-", '='), (".-.-.", '+'),
    ];
    T.iter().find(|(c, _)| *c == s).map_or('*', |(_, ch)| *ch)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 16-bit PCM mono WAV samples (full scale 1.0) and the rate.
    pub fn read_wav(path: &str) -> (Vec<f32>, f32) {
        let b = std::fs::read(path).unwrap();
        let mut i = 12;
        let (mut rate, mut data) = (12_000u32, Vec::new());
        while i + 8 <= b.len() {
            let id = &b[i..i + 4];
            let len = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap()) as usize;
            if id == b"fmt " {
                rate = u32::from_le_bytes(b[i + 12..i + 16].try_into().unwrap());
            } else if id == b"data" {
                data = b[i + 8..(i + 8 + len).min(b.len())].chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0).collect();
            }
            i += 8 + len + (len & 1);
        }
        (data, rate as f32)
    }

    /// Synthetic rain scatter: keyed noise spread over `spread` Hz around
    /// 900 Hz with Rayleigh fading, in white noise; the text comes back.
    fn synthetic(text: &str, wpm: f32, spread: f32, snr_db: f32) -> String {
        let rate = 12_000.0;
        let tl = crate::morse::timeline(text);
        let dot = crate::morse::dot_seconds(wpm) * rate as f64;
        let n = ((tl.len() + 20) as f64 * dot) as usize;
        let mut seed = 11u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        // Band-limited noise around 900 Hz: complex noise through a moving
        // average (width about `spread`), moved up, real part.
        let taps = ((rate / spread) as usize).max(1);
        let mut acc = [0f32; 2];
        let mut hist: std::collections::VecDeque<[f32; 2]> = Default::default();
        let sig_gain = 10f32.powf(snr_db / 20.0);
        let mut x = Vec::with_capacity(n);
        for i in 0..n {
            let v = [g(), g()];
            hist.push_back(v);
            acc[0] += v[0];
            acc[1] += v[1];
            if hist.len() > taps {
                let o = hist.pop_front().unwrap();
                acc[0] -= o[0];
                acc[1] -= o[1];
            }
            let key = tl.get((i as f64 / dot) as usize).copied().unwrap_or(false) && i as f64 > 10.0 * dot;
            let ph = std::f32::consts::TAU * 900.0 * i as f32 / rate;
            let s = (acc[0] * ph.cos() - acc[1] * ph.sin()) / (taps as f32).sqrt();
            // SNR in 2500 Hz: signal power 1 against noise density scaled.
            let noise = g() * (rate / 2.0 / 2500.0).sqrt() * 0.5;
            x.push(if key { s * sig_gain * 0.5 } else { 0.0 } + noise);
        }
        let mut d = RsStream::new(rate);
        for c in x.chunks(1200) {
            d.process(c);
        }
        d.finish();
        d.text.trim().to_string()
    }

    #[test]
    fn noise_statistics() {
        let mut seed = 3u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        let mut d = RsCw::new(12_000.0);
        let mut zs = Vec::new();
        for _ in 0..(12_000 * 60 / 1200) {
            let x: Vec<f32> = (0..1200).map(|_| g() * 0.1).collect();
            let before = d.frames;
            d.process(&x);
            if d.frames > 400 && d.frames != before {
                zs.push(d.score);
            }
        }
        zs.sort_by(|a, b| a.total_cmp(b));
        let q = |p: f32| zs[((zs.len() - 1) as f32 * p) as usize];
        let mean = zs.iter().sum::<f32>() / zs.len() as f32;
        let sd = (zs.iter().map(|z| (z - mean).powi(2)).sum::<f32>() / zs.len() as f32).sqrt();
        eprintln!("noise score: mean {mean:.2} sd {sd:.2} q10 {:.2} q30 {:.2} q50 {:.2} q90 {:.2} q99 {:.2} q999 {:.2}; tracker mu+5s {:.2}, text {:?}",
            q(0.1), q(0.3), q(0.5), q(0.9), q(0.99), q(0.999), d.q30 + 0.524 * (d.q30 - d.q10) / 0.757 + 5.0 * (d.q30 - d.q10) / 0.757, d.text);
    }

    #[test]
    fn synthetic_rain_scatter() {
        // The streaming decoder, Doppler spread 300 and 800 Hz, Rayleigh
        // fading, 0 dB SNR in 2.5 kHz: the callsign and locator come through.
        for (spread, snr) in [(300.0, 0.0), (800.0, 0.0), (800.0, -5.0)] {
            let got = synthetic("CQ CQ DE SP6HED SP6HED JO80 K  CQ CQ DE SP6HED SP6HED JO80 K  CQ CQ DE SP6HED SP6HED JO80 K", 18.0, spread, snr);
            eprintln!("spread {spread} Hz, SNR {snr} dB in 2.5 kHz: {got:?}");
            if snr >= 0.0 {
                let flat: String = got.chars().filter(|c| *c != ' ').collect();
                assert!(flat.contains("SP6HED") && flat.contains("JO80"), "{spread} Hz: {got}");
            }
        }
    }

    /// What each recording should yield (callsigns, locators, words seen
    /// clearly in some decode), by a piece of its file name.
    const TRUTH: &[(&str, &[&str])] = &[
        ("9a2sb_250608", &["9A2SB", "CQ"]), ("dl3jan", &["DL3JAN"]), ("i4xcc_250608.", &["I4XCC"]), ("i6xck", &["I6XCK"]),
        ("iw4cjm", &["IW4CJM"]), ("9A1CAL-SP9COO", &["9A1CAL", "SP9COO"]), ("9A2SB_rs", &["9A2SB"]), ("9a1cms", &["9A1CMS", "OK2ER"]),
        ("9a2sb.wav", &["9A2SB", "CQ"]), ("9a2sb_rs2", &["9A2SB"]), ("9a2sb_rs3", &["9A2SB"]), ("OM-OK1TEH", &["OK1TEH", "SP6GWB"]),
        ("OM3CLS_rs.", &["OM3CLS"]), ("OM3CLS_rs1", &["OM3CLS", "SP6GWB", "TNX", "73"]), ("DB0ANU", &["DB0ANU", "JN59GG", "ANSBACH"]),
        ("SUZ", &["SUZ"]), ("Db0FGB", &["DB0FGB"]), ("HA8MV_rs", &["HA8MV", "SP6GWB", "TNX"]), ("OE5XBM", &["OE5XBM"]),
        ("OK0EW", &["OK0EW"]), ("JN88", &["JN88", "73"]), ("audio(17)", &["SP6GWB", "JO80HK", "55S"]), ("S51ZO", &["S51ZO"]),
        ("S56BD", &["S56BD"]), ("SP3JBI_RS", &["SP3JBI"]), ("SP3JBI_z_HA8MV", &["SP3JBI", "HA8MV"]), ("SR6KBL", &["SR6KBL"]),
        ("SR6NCI", &["SR6NCI"]), ("9A1CAL_rs", &["9A1CAL"]), ("9a2sb_rs1", &["9A2SB", "SP6GWB"]), ("IW5DHN", &["IW5DHN"]),
        ("DL6NCI", &["DL6NCI", "CQ"]), ("DL7YC", &["DL7YC", "SP6GWB", "73"]), ("SP6HED_ssb.", &["SP6HED"]),
    ];

    fn decode_file(path: &str, p: Params) -> String {
        let (x, rate) = read_wav(path);
        let mut d = RsCw::new(rate);
        d.p = p;
        for c in x.chunks(1200) {
            d.process(c);
        }
        d.finish();
        d.text
    }

    /// Tokens found over all recordings (and the total asked for).
    fn score(dir: &str, p: Params) -> (usize, usize) {
        let (mut got, mut all) = (0, 0);
        for e in std::fs::read_dir(dir).unwrap() {
            let f = e.unwrap().path();
            let name = f.file_name().unwrap().to_string_lossy().to_string();
            let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) else { continue };
            let text: String = decode_file(f.to_str().unwrap(), p).chars().filter(|c| *c != ' ').collect();
            all += toks.len();
            got += toks.iter().filter(|t| text.contains(*t)).count();
        }
        (got, all)
    }

    /// The Viterbi decoder on a whole recording: scores, LLRs, the best of
    /// a range of speeds.
    /// LLRs of a recording by the chosen detector (RS2: v2, else the
    /// batch version of the first), and the frame rate.
    fn file_llrs(path: &str) -> (Vec<f32>, f32) {
        let (x, rate) = read_wav(path);
        let hop: f32 = std::env::var("RSCW_LLRHOP").ok().and_then(|v| v.parse().ok()).unwrap_or(HOP as f32);
        let fps = rate / hop;
        // RSCW_LLRDIR: LLRs from elsewhere (the network), <dir>/<file>.f32
        if let Ok(dir) = std::env::var("RSCW_LLRDIR") {
            let name = std::path::Path::new(path).file_name().unwrap().to_string_lossy().to_string();
            let b = std::fs::read(format!("{dir}/{name}.f32")).unwrap();
            let s: f32 = std::env::var("RSCW_LLRSCALE").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0);
            return (b.chunks_exact(4).map(|c| s * f32::from_le_bytes(c.try_into().unwrap())).collect(), fps);
        }
        let mut d = RsCw::new(rate);
        d.history = Some(Vec::new());
        for c in x.chunks(1200) {
            d.process(c);
        }
        let z = d.history.take().unwrap();
        (llrs(&z[40.min(z.len())..], fps), fps)
    }

    /// Upper bound: every speed tried, the text with the most known tokens.
    #[test]
    #[ignore]
    fn rscw_oracle() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let (mut got, mut all) = (0, 0);
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        for f in files {
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) else { continue };
            let (l, fps) = file_llrs(f.to_str().unwrap());
            let mut best = (0usize, String::new(), 0f32);
            let mut dot = 1.2 * fps / 30.0;
            while dot <= 1.2 * fps / 8.0 {
                let (t, _) = if std::env::var_os("RSCW_VCHAR").is_some() { viterbi_chars(&l, dot) } else { viterbi(&l, dot) };
                let flat: String = t.chars().filter(|c| *c != ' ').collect();
                let g = toks.iter().filter(|x| flat.contains(*x)).count();
                if g > best.0 || best.1.is_empty() {
                    best = (g, t, 1.2 * fps / dot);
                }
                dot *= 1.06;
            }
            all += toks.len();
            got += best.0;
            eprintln!("{:36} {:4.0} WPM [{}/{}] | {}", &name[..name.len().min(36)], best.2, best.0, toks.len(), best.1.trim().chars().take(150).collect::<String>());
        }
        eprintln!("oracle score: {got} of {all}");
    }

    fn viterbi_file(path: &str) -> (String, f32) {
        if std::env::var_os("RSCW_LLRDIR").is_some() {
            let (l, fps) = file_llrs(path);
            let mut best = (String::new(), f64::MIN, 0f32);
            // RSCW_BOTHPOL: the LLRs inverted too (FSK beacons: which tone
            // is key down is not known)
            let inv: Vec<f32> = l.iter().map(|v| -v).collect();
            let pols: Vec<&[f32]> = if std::env::var_os("RSCW_BOTHPOL").is_some() { vec![&l, &inv] } else { vec![&l] };
            for lp in pols {
                let mut dot = 1.2 * fps / 30.0;
                while dot <= 1.2 * fps / 8.0 {
                    let (t, ll) = if std::env::var_os("RSCW_VCHAR").is_some() { viterbi_chars(lp, dot) } else { viterbi(lp, dot) };
                    if ll > best.1 {
                        best = (t, ll, dot);
                    }
                    dot *= 1.06;
                }
            }
            return (best.0, 1.2 * fps / best.2);
        }
        let (x, rate) = read_wav(path);
        let mut d = RsCw::new(rate);
        d.history = Some(Vec::new());
        for c in x.chunks(1200) {
            d.process(c);
        }
        let z = d.history.take().unwrap();
        let fps = d.fps();
        let l = llrs(&z[40.min(z.len())..], fps);
        // RSCW_WIN=<s>: windows of that length (3 s overlap), each at its own
        // speed (a QSO has two stations).
        if let Some(win) = std::env::var("RSCW_WIN").ok().and_then(|x| x.parse::<f32>().ok()) {
            let (w, hop) = ((win * fps) as usize, ((win - 3.0).max(1.0) * fps) as usize);
            let mut text = String::new();
            let mut at = 0;
            let mut wpm_last = 0.0;
            while at < l.len() {
                let part = &l[at..(at + w).min(l.len())];
                let mut best = (String::new(), f64::MIN, 0f32);
                let mut dot = 1.2 * fps / 30.0;
                while dot <= 1.2 * fps / 10.0 {
                    let (t, ll) = viterbi(part, dot);
                    if ll > best.1 {
                        best = (t, ll, dot);
                    }
                    dot *= 1.08;
                }
                text += best.0.trim();
                text.push_str(" | ");
                wpm_last = 1.2 * fps / best.2;
                if at + w >= l.len() {
                    break;
                }
                at += hop;
            }
            return (text, wpm_last);
        }
        if std::env::var_os("RSCW_MARKSPEED").is_some() {
            let d0 = dot_from_marks(&l, 1.2 * fps / 35.0, 1.2 * fps / 8.0);
            let mut best = (String::new(), f64::MIN, 0f32);
            for f in [0.92f32, 1.0, 1.08] {
                let (t, ll) = viterbi(&l, d0 * f);
                if ll > best.1 {
                    best = (t, ll, d0 * f);
                }
            }
            return (best.0, 1.2 * fps / best.2);
        }
        if std::env::var_os("RSCW_QUALITY").is_some() {
            let mut best = (String::new(), f32::MIN, 0f32);
            let mut dot = 1.2 * fps / 30.0;
            while dot <= 1.2 * fps / 9.0 {
                let (t, _) = viterbi(&l, dot);
                let q = text_quality(&t);
                if q > best.1 {
                    best = (t, q, dot);
                }
                dot *= 1.06;
            }
            return (best.0, 1.2 * fps / best.2);
        }
        let vit = |l: &[f32], dot: f32| if std::env::var_os("RSCW_VCHAR").is_some() { viterbi_chars(l, dot) } else { viterbi(l, dot) };
        let mut best = (String::new(), f64::MIN, 0f32);
        let (w_lo, w_hi): (f32, f32) = (
            std::env::var("RSCW_WLO").ok().and_then(|x| x.parse().ok()).unwrap_or(10.0),
            std::env::var("RSCW_WHI").ok().and_then(|x| x.parse().ok()).unwrap_or(30.0),
        );
        let mut dot = 1.2 * fps / w_hi;
        while dot <= 1.2 * fps / w_lo {
            let (t, ll) = vit(&l, dot);
            if ll > best.1 {
                best = (t, ll, dot);
            }
            dot *= 1.08;
        }
        (best.0, 1.2 * fps / best.2)
    }

    #[test]
    #[ignore]
    fn rscw_viterbi() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let (mut got, mut all) = (0, 0);
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        for f in files {
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            let (text, wpm) = viterbi_file(f.to_str().unwrap());
            let flat: String = text.chars().filter(|c| *c != ' ').collect();
            let mut mark = String::new();
            if let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) {
                all += toks.len();
                let g = toks.iter().filter(|t| flat.contains(*t)).count();
                got += g;
                mark = format!("[{g}/{}]", toks.len());
            }
            eprintln!("{:36} {wpm:4.0} WPM {mark:6} | {}", &name[..name.len().min(36)], text.trim());
        }
        eprintln!("viterbi score: {got} of {all}");
    }

    /// The streaming decoder (as trxd runs it) on every recording.
    #[test]
    #[ignore]
    fn rscw_stream() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let (mut got, mut all) = (0, 0);
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        let t0 = std::time::Instant::now();
        let mut audio_s = 0.0;
        for f in files {
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            let (x, rate) = read_wav(f.to_str().unwrap());
            audio_s += x.len() as f64 / rate as f64;
            let mut d = RsStream::new(rate);
            for c in x.chunks(1200) {
                d.process(c);
            }
            d.finish();
            let flat: String = d.text.chars().filter(|c| *c != ' ').collect();
            let mut mark = String::new();
            if let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) {
                all += toks.len();
                let g = toks.iter().filter(|t| flat.contains(*t)).count();
                got += g;
                mark = format!("[{g}/{}]", toks.len());
            }
            eprintln!("{:36} {:4.0} WPM {mark:6} | {}", &name[..name.len().min(36)], d.wpm(), d.text.trim());
        }
        eprintln!("stream score: {got} of {all}; {:.2} s of CPU for {:.0} s of audio", t0.elapsed().as_secs_f64(), audio_s);
    }

    /// Time of the network alone and of one speed's character bank.
    #[test]
    #[ignore]
    fn rscw_nn_cost() {
        let (x, rate) = read_wav(&std::env::var("RSCW_FILE").expect("RSCW_FILE"));
        let t0 = std::time::Instant::now();
        let mut nn = crate::rsnn::RsNn::new();
        let mut l = Vec::new();
        for c in x.chunks(1200) {
            nn.process(c, &mut l);
        }
        let t_nn = t0.elapsed().as_secs_f64();
        let fps = rate / HOP as f32;
        for w in [9.0f32, 15.0, 30.0] {
            let t0 = std::time::Instant::now();
            let mut b = CharBank::new(1.2 * fps / w, (6.0 * fps) as usize);
            for &v in &l {
                b.step(v);
            }
            eprintln!("bank {w} WPM: {} states, {:.3} s", b.m.ns, t0.elapsed().as_secs_f64());
        }
        eprintln!("audio {:.0} s: network {t_nn:.3} s", x.len() as f32 / rate);
    }

    /// The network + character-level streaming decoder on every recording.
    /// Repeat combining (beacons send the same text again and again): the
    /// period from the LLRs' autocorrelation, the repetitions summed, the
    /// folded message decoded; tokens from the plain and folded text.
    #[test]
    #[ignore]
    fn rscw_fold() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let min_r: f64 = std::env::var("RSCW_FOLD_R").ok().and_then(|v| v.parse().ok()).unwrap_or(0.15);
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        let (mut got_plain, mut got_both, mut all) = (0, 0, 0);
        for f in files {
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            let (x, rate) = read_wav(f.to_str().unwrap());
            let mut d = RsNnStream::new(rate);
            let mut nn = crate::rsnn::RsNn::new();
            let dec = (128 / nn.hop()).max(1);
            let mut raw = Vec::new();
            for c in x.chunks(1200) {
                d.process(c);
                nn.process(c, &mut raw);
            }
            d.finish();
            let llr: Vec<f32> = raw.chunks_exact(dec).map(|c| c.iter().sum()).collect();
            let fps = rate / nn.hop() as f32 / dec as f32;
            // the period: normalized autocorrelation, 8 s .. 150 s (at
            // least two whole repetitions)
            let xs: Vec<f64> = llr.iter().map(|&v| (v as f64).clamp(-10.0, 10.0)).collect();
            let m = xs.iter().sum::<f64>() / xs.len().max(1) as f64;
            let xs: Vec<f64> = xs.iter().map(|v| v - m).collect();
            let (lo, hi) = ((8.0 * fps) as usize, ((150.0 * fps) as usize).min(xs.len() / 2));
            let mut best = (0usize, 0f64);
            for lag in lo..hi {
                let (mut sxy, mut sxx, mut syy) = (0f64, 0f64, 0f64);
                for i in 0..xs.len() - lag {
                    sxy += xs[i] * xs[i + lag];
                    sxx += xs[i] * xs[i];
                    syy += xs[i + lag] * xs[i + lag];
                }
                let r = sxy / (sxx * syy).sqrt().max(1e-9);
                if r > best.1 {
                    best = (lag, r);
                }
            }
            let mut folded = String::new();
            if best.0 > 0 && best.1 >= min_r {
                let p = best.0;
                let mut sum = vec![0f32; p];
                for (i, &v) in llr.iter().enumerate() {
                    sum[i % p] += v;
                }
                let twice: Vec<f32> = sum.iter().chain(sum.iter()).cloned().collect();
                let wpms = [9.5f32, 11.0, 12.5, 14.0, 15.5, 17.5, 20.0, 23.0, 27.0];
                let mut bestv = (f64::MIN, String::new());
                for w in wpms {
                    let (t, fit) = viterbi_chars(&twice, 1.2 * fps / w);
                    if fit > bestv.0 {
                        bestv = (fit, t);
                    }
                }
                folded = bestv.1;
            }
            let flat: String = d.text.chars().filter(|c| *c != ' ').collect();
            let fflat: String = folded.chars().filter(|c| *c != ' ').collect();
            let mut mark = String::new();
            if let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) {
                all += toks.len();
                let g1 = toks.iter().filter(|t| flat.contains(*t)).count();
                let g2 = toks.iter().filter(|t| flat.contains(*t) || fflat.contains(*t)).count();
                got_plain += g1;
                got_both += g2;
                mark = format!("[{g1}->{g2}/{}]", toks.len());
            }
            eprintln!("{:34} period {:5.1} s r {:.2} {mark:10} | {}", &name[..name.len().min(34)], best.0 as f32 / fps, best.1, &folded.trim()[..folded.trim().len().min(150)]);
        }
        eprintln!("fold score: plain {got_plain}, with folding {got_both} of {all}");
    }

    #[test]
    #[ignore]
    fn rscw_banks_speed() {
        // The streaming decoder's nine speed banks on a keyed test signal
        // (PARIS at 15 WPM, LLR +-6 with noise), as trxd runs them.
        let fps = 12000.0 / 128.0;
        let dots = [9.5f32, 11.0, 12.5, 14.0, 15.5, 17.5, 20.0, 23.0, 27.0].map(|w| 1.2 * fps / w);
        let mut banks: Vec<CharBank> = dots.iter().map(|&d| CharBank::new(d, (6.0 * fps) as usize)).collect();
        let key = ".--. .- .-. .. ...  ";
        let dot = (1.2 * fps / 15.0) as usize;
        let mut llr = Vec::new();
        let mut x = 1u32;
        for _ in 0..40 {
            for c in key.chars() {
                let (on, n) = match c { '.' => (true, 1), '-' => (true, 3), _ => (false, 2) };
                for _ in 0..n * dot {
                    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
                    llr.push(if on { 6.0 } else { -6.0 } + (x % 1000) as f32 / 100.0 - 5.0);
                }
                for _ in 0..dot {
                    llr.push(-6.0);
                }
            }
        }
        let t0 = std::time::Instant::now();
        for &l in &llr {
            for b in &mut banks {
                b.step(l);
            }
        }
        let dt = t0.elapsed().as_secs_f64();
        let ns: usize = banks.iter().map(|b| b.m.ns).sum();
        eprintln!("{} frames x 9 banks ({ns} states): {:.1} us a frame, {:.1} % of a core at {fps:.0} frames/s; best fit {:.3}", llr.len(), dt / llr.len() as f64 * 1e6, dt / llr.len() as f64 * fps as f64 * 100.0, banks.iter().map(|b| b.fit).fold(f64::MIN, f64::max));
    }

    #[test]
    #[ignore]
    fn rscw_nnstream() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let (mut got, mut all) = (0, 0);
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        let t0 = std::time::Instant::now();
        let mut audio_s = 0.0;
        let mut prof = (0.0, 0.0);
        for f in files {
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            let (x, rate) = read_wav(f.to_str().unwrap());
            audio_s += x.len() as f64 / rate as f64;
            let mut d = RsNnStream::new(rate);
            for c in x.chunks(1200) {
                d.process(c);
            }
            d.finish();
            prof = (prof.0 + d.prof.0, prof.1 + d.prof.1);
            let flat: String = d.text.chars().filter(|c| *c != ' ').collect();
            let mut mark = String::new();
            if let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) {
                all += toks.len();
                let g = toks.iter().filter(|t| flat.contains(*t)).count();
                got += g;
                mark = format!("[{g}/{}]", toks.len());
            }
            eprintln!("{:36} {:4.0} WPM {mark:6} | {}", &name[..name.len().min(36)], d.wpm(), d.text.trim());
        }
        eprintln!("nn stream score: {got} of {all}; {:.2} s of CPU for {:.0} s of audio (network {:.1} s, banks {:.1} s)", t0.elapsed().as_secs_f64(), audio_s, prof.0, prof.1);
    }

    /// The Morse rhythm test: Morse keying (with timing jitter) scores
    /// clearly above speech-like runs of random length, a steady carrier 0.
    #[test]
    fn morse_fit_tells_morse_from_speech() {
        let dt = 128.0 / 12000.0;
        let mut seed = 7u64;
        let mut rnd = move || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            (seed >> 11) as f32 / (1u64 << 53) as f32
        };
        let mut push = |v: &mut Vec<bool>, k: bool, secs: f32| v.extend(std::iter::repeat(k).take((secs / dt).round() as usize));
        // "CQ DE SP6GWB K" at 15 WPM, 10 % jitter, 4 s of it
        let u = 1.2 / 15.0;
        let mut m = Vec::new();
        for w in "CQ DE SP6GWB K CQ".split(' ') {
            for c in w.chars() {
                let code = MORSE.iter().find(|x| x.1 == c).unwrap().0;
                for e in code.chars() {
                    push(&mut m, true, u * if e == '.' { 1.0 } else { 3.0 } * (0.9 + 0.2 * rnd()));
                    push(&mut m, false, u * (0.9 + 0.2 * rnd()));
                }
                push(&mut m, false, 2.0 * u);
            }
            push(&mut m, false, 4.0 * u);
        }
        m.truncate((4.0 / dt) as usize);
        let mut sp = Vec::new();
        while sp.len() < (4.0 / dt) as usize {
            let (a, b) = (0.1 + 0.4 * rnd(), 0.05 + 0.35 * rnd());
            push(&mut sp, true, a);
            push(&mut sp, false, b);
        }
        let (fm, fs, fc) = (morse_fit(&m, dt), morse_fit(&sp, dt), morse_fit(&vec![true; 400], dt));
        eprintln!("morse {fm:.2}, speech {fs:.2}, carrier {fc:.2}");
        // (synthetic speech scores about 0.5: the threshold comes from
        // labelled recordings, not from this)
        assert!(fm > 0.75 && fs < fm - 0.25 && fc == 0.0);
    }

    /// RSCW_DIR=<wavs> RSCW_OUT=<file.jsonl>: every recording's shown
    /// characters with their times (one JSON line a file), for scoring
    /// against labelled time windows.
    #[test]
    #[ignore]
    fn rscw_nntimed() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let out = std::env::var("RSCW_OUT").expect("RSCW_OUT");
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        let mut lines = String::new();
        for f in files {
            let (x, rate) = read_wav(f.to_str().unwrap());
            let mut d = RsNnStream::new(rate);
            for c in x.chunks(1200) {
                d.process(c);
            }
            d.finish();
            let timed: Vec<(f32, String)> = d.timed.iter().map(|&(t, c)| ((t * 100.0).round() / 100.0, c.to_string())).collect();
            let name = f.file_name().unwrap().to_string_lossy().to_string();
            let ml: Vec<(f32, f32)> = d.morse_log.iter().map(|&(t, m)| ((t * 100.0).round() / 100.0, (m * 1000.0).round() / 1000.0)).collect();
            lines += &serde_json::json!({"name": name, "text": d.text, "timed": timed, "morse": ml}).to_string();
            lines.push('\n');
        }
        std::fs::write(&out, lines).unwrap();
    }

    /// The classic narrowband decoder (sdroxide CwRx, as trxd's live CW
    /// box) on the same recordings, tuned to each one's spectral peak.
    #[test]
    #[ignore]
    fn rscw_baseline() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let (mut got, mut all) = (0, 0);
        for e in std::fs::read_dir(&dir).unwrap() {
            let f = e.unwrap().path();
            let name = f.file_name().unwrap().to_string_lossy().to_string();
            let (x, rate) = read_wav(f.to_str().unwrap());
            // peak of the average spectrum, 200-3000 Hz
            let n = 4096;
            let fft = FftPlanner::<f32>::new().plan_fft_forward(n);
            let mut acc = vec![0f32; n / 2];
            for c in x.chunks_exact(n) {
                let mut w: Vec<Complex32> = c.iter().map(|&v| Complex32::new(v, 0.0)).collect();
                fft.process(&mut w);
                for (a, v) in acc.iter_mut().zip(&w) {
                    *a += v.norm_sqr();
                }
            }
            let bin = |hz: f32| (hz / rate * n as f32) as usize;
            let pk = (bin(200.0)..bin(3000.0)).max_by(|&a, &b| acc[a].total_cmp(&acc[b])).unwrap_or(0);
            let pitch = pk as f32 * rate / n as f32;
            let mut rx = sdroxide_dsp::CwRx::new(rate as f64, pitch);
            let mut text = String::new();
            for c in x.chunks(1200) {
                text += &rx.process(c);
            }
            let flat: String = text.chars().filter(|c| *c != ' ').collect();
            if let Some((_, toks)) = TRUTH.iter().find(|(k, _)| name.contains(k)) {
                all += toks.len();
                got += toks.iter().filter(|t| flat.contains(*t)).count();
            }
        }
        eprintln!("baseline (CwRx) score: {got} of {all}");
    }

    /// `RSCW_DIR=... cargo test --release rscw_score -- --ignored --nocapture`
    /// (RSCW_SWEEP=1: a grid around the defaults).
    #[test]
    #[ignore]
    fn rscw_score() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let base = Params::default();
        eprintln!("defaults {:?}: {:?}", base, score(&dir, base));
        if std::env::var_os("RSCW_SWEEP").is_none() {
            return;
        }
        for integ in [0.33, 0.5, 0.7, 1.0] {
            for (on, sq) in [(4.0, 5.0), (5.0, 6.0), (6.0, 8.0), (7.0, 10.0)] {
                let p = Params { integ, on_sigma: on, off_sigma: on / 2.0, squelch_sigma: sq, ..base };
                eprintln!("integ {integ} on {on} squelch {sq}: {:?}", score(&dir, p));
            }
        }
    }

    /// Debug: the detector score and the streaming LLR of RSCW_FILE, as f32
    /// files in RSCW_OUT (…z.f32, …llr.f32).
    #[test]
    #[ignore]
    fn rscw_dump() {
        let path = std::env::var("RSCW_FILE").expect("RSCW_FILE");
        let out = std::env::var("RSCW_OUT").expect("RSCW_OUT");
        let (x, rate) = read_wav(&path);
        let mut d = RsCw::new(rate);
        d.history = Some(Vec::new());
        d.llr_out = Some(Vec::new());
        for c in x.chunks(1200) {
            d.process(c);
        }
        let w = |name: &str, v: &[f32]| {
            let b: Vec<u8> = v.iter().flat_map(|x| x.to_le_bytes()).collect();
            std::fs::write(format!("{out}{name}"), b).unwrap();
        };
        w("z.f32", d.history.as_ref().unwrap());
        w("llr.f32", d.llr_out.as_ref().unwrap());
        eprintln!("fps {}", d.fps());
    }

    /// Every recording in RSCW_DIR through the decoder:
    /// `RSCW_DIR=/data/claude/rscw/wav cargo test --release rscw_files -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn rscw_files() {
        let dir = std::env::var("RSCW_DIR").expect("RSCW_DIR");
        let mut files: Vec<_> = std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).collect();
        files.sort();
        for f in files {
            let (x, rate) = read_wav(f.to_str().unwrap());
            let mut d = RsCw::new(rate);
            for c in x.chunks(1200) {
                d.process(c);
            }
            d.finish();
            let name = f.file_name().unwrap().to_string_lossy().replace("rsonly__", "");
            eprintln!("{:36} {:4.0} WPM | {}", &name[..name.len().min(36)], d.wpm(), d.text.trim());
        }
    }
}
