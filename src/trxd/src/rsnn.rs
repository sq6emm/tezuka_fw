//! The rain-scatter keying detector as a small neural network: from the
//! audio's short-time spectrum, each frame's log-likelihood that the key is
//! down, for the Morse decoder ([`crate::rscw::RsNnStream`]).
//!
//! Features (as the training's rsgen.py): 256-point FFT every `hop` samples
//! at 12 kHz, sin^2 window, bins 200..3000 Hz (61), log power minus a
//! per-bin noise floor (its 25 % point over the first 64 frames, then a
//! quantile tracker), clipped to -6..12.
//!
//! Network (train.py `Net`, trained on synthetic rain scatter, tropo and
//! beacons, on-off and FSK): three 2-D convolutions over (time, frequency),
//! the second halving the frequency axis; attention and max pooling over
//! frequency (the carrier may sit anywhere); four dilated 1-D convolutions
//! over time (1, 2, 4, 8) with 32 channels, residual; one output per frame.
//! The weight file gives the hop and the 2-D layers' channels (RSN2), and
//! the temporal layers' channels and dilations (RSN3: larger networks). The 2-D and
//! temporal layers run in 16-bit fixed point (twice as fast as floats on
//! the A9, whose NEON does not take floats in auto-vectorized code).

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

pub const N: usize = 256;
const LO: usize = 4;
const HI: usize = 64;
pub const NB: usize = HI - LO + 1;
const Q: f32 = 0.25;
const WARM: usize = 64;
const NB2: usize = (NB + 4 - 5) / 2 + 1;

/// Streaming features: audio in, feature rows out.
pub struct Features {
    hop: usize,
    eta: f32,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<f32>,
    work: Vec<Complex32>,
    warm: Vec<[f32; NB]>,
    floor: [f32; NB],
    started: bool,
}

impl Features {
    pub fn new(hop: usize) -> Features {
        Features {
            hop,
            // the same tracking per second at any hop
            eta: 0.01 * hop as f32 / 64.0,
            fft: FftPlanner::new().plan_fft_forward(N),
            window: (0..N).map(|n| (std::f32::consts::PI * n as f32 / N as f32).sin().powi(2)).collect(),
            buf: Vec::new(),
            work: vec![Complex32::default(); N],
            warm: Vec::new(),
            floor: [0.0; NB],
            started: false,
        }
    }

    /// Audio (12 kHz) in; the feature rows of the frames completed appended
    /// to `out` (none for the first 64 frames until the floor is known,
    /// then those at once).
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<[f32; NB]>) {
        self.buf.extend_from_slice(audio);
        let mut at = 0;
        while at + N <= self.buf.len() {
            for (o, (x, w)) in self.work.iter_mut().zip(self.buf[at..at + N].iter().zip(&self.window)) {
                *o = Complex32::new(x * w, 0.0);
            }
            self.fft.process(&mut self.work);
            let mut lp = [0f32; NB];
            for (k, v) in lp.iter_mut().enumerate() {
                *v = self.work[LO + k].norm_sqr().max(1e-20).ln();
            }
            self.row(lp, out);
            at += self.hop;
        }
        self.buf.drain(..at);
    }

    fn row(&mut self, lp: [f32; NB], out: &mut Vec<[f32; NB]>) {
        if !self.started {
            self.warm.push(lp);
            if self.warm.len() < WARM {
                return;
            }
            // each bin's Q quantile over the first frames (numpy's linear
            // interpolation)
            for k in 0..NB {
                let mut v: Vec<f32> = self.warm.iter().map(|r| r[k]).collect();
                v.sort_by(|a, b| a.total_cmp(b));
                let pos = (v.len() - 1) as f32 * Q;
                let i = pos.floor() as usize;
                let fr = pos - i as f32;
                self.floor[k] = v[i] + fr * (v[(i + 1).min(v.len() - 1)] - v[i]);
            }
            self.started = true;
            let warm = std::mem::take(&mut self.warm);
            for r in warm {
                out.push(self.feature(&r));
            }
            return;
        }
        for (f, &v) in self.floor.iter_mut().zip(&lp) {
            if v < *f {
                *f -= self.eta * (1.0 - Q);
            } else {
                *f += self.eta * Q;
            }
        }
        out.push(self.feature(&lp));
    }

    fn feature(&self, lp: &[f32; NB]) -> [f32; NB] {
        let mut o = [0f32; NB];
        for k in 0..NB {
            o[k] = (lp[k] - self.floor[k]).clamp(-6.0, 12.0);
        }
        o
    }
}

/// Fixed point of the integer layers: activations x AQ, weights x WQ, sums
/// in i32. Measured on the recordings (m4): activations up to 55 (7040
/// here), weights under 1.3 (2660): both fit 16 bits; a sum is a
/// preactivation (under about 60) x AQ x WQ, some 1.6e7, far below 2^31.
pub(crate) const AQ: f32 = 128.0;
const WQ: f32 = 2048.0;

/// exp(-d) for the attention's softmax: d in steps of 1 / 2^(18 - EXP_SHIFT)
/// = 1/128 (the logits are x AQ WQ = 2^18), Q15, EXP_N entries (to 16).
pub const EXP_SHIFT: u32 = 11;
pub const EXP_N: usize = 2048;

pub fn exp_table() -> Vec<i32> {
    (0..EXP_N).map(|i| (32767.0 * (-(i as f64) / 128.0).exp()).round() as i32).collect()
}

/// The network's weights.
pub struct Net {
    /// Samples between frames.
    pub hop: usize,
    /// The 2-D layers' channels.
    c2: usize,
    /// The temporal layers' channels and dilations.
    c1: usize,
    dils: Vec<usize>,
    c1w: Vec<f32>,
    c1b: Vec<f32>,
    c2w: Vec<f32>,
    c2b: Vec<f32>,
    c3w: Vec<f32>,
    c3b: Vec<f32>,
    attw: Vec<f32>,
    attb: f32,
    inpw: Vec<f32>,
    inpb: Vec<f32>,
    tw: Vec<Vec<f32>>,
    tb: Vec<Vec<f32>>,
    outw: Vec<f32>,
    outb: f32,
    c1q: Vec<i16>,
    c2q: Vec<i16>,
    c3q: Vec<i16>,
    twq: Vec<Vec<i16>>,
    float: bool,
}

/// The trained weights shipped with trxd.
static WEIGHTS: &[u8] = include_bytes!("../rsnn.bin");

impl Net {
    /// The weights shipped in the binary (RSNN_WEIGHTS=<file>: others, for
    /// tests and trials).
    pub fn builtin() -> Net {
        if let Some(b) = std::env::var_os("RSNN_WEIGHTS").and_then(|p| std::fs::read(p).ok()) {
            match Net::from_bytes(&b) {
                Ok(n) => return n,
                Err(e) => tracing::warn!("RSNN_WEIGHTS: {e}; the built-in weights"),
            }
        }
        Net::from_bytes(WEIGHTS).expect("rsnn.bin")
    }

    /// RSN2: hop, 2-D channels, count, weights (32 temporal channels,
    /// dilations 1, 2, 4, 8); RSN3: hop, 2-D channels, temporal channels,
    /// layers, their dilations, count, weights.
    pub fn from_bytes(b: &[u8]) -> Result<Net, String> {
        let u = |i: usize| b.get(i..i + 4).map_or(0, |x| u32::from_le_bytes(x.try_into().unwrap()) as usize);
        let (hop, c2, c1, dils, start) = match b.get(..4) {
            Some(b"RSN2") => (u(4), u(8), 32, vec![1, 2, 4, 8], 12),
            Some(b"RSN3") => {
                let nl = u(16);
                if nl == 0 || nl > 16 {
                    return Err(format!("{nl} temporal layers"));
                }
                (u(4), u(8), u(12), (0..nl).map(|i| u(20 + 4 * i)).collect(), 20 + 4 * nl)
            }
            _ => return Err("not an RSN2/RSN3 weight file".into()),
        };
        let n = u(start);
        if b.len() != start + 4 + 4 * n {
            return Err(format!("{} bytes for {n} weights", b.len()));
        }
        let all: Vec<f32> = b[start + 4..].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
        #[allow(non_snake_case)]
        let C1 = c1;
        let mut at = 0;
        let mut take = |k: usize| {
            let v = all[at..at + k].to_vec();
            at += k;
            v
        };
        let c1w = take(c2 * 15);
        let c1b = take(c2);
        let c2w = take(c2 * c2 * 15);
        let c2b = take(c2);
        let c3w = take(c2 * c2 * 9);
        let c3b = take(c2);
        let attw = take(c2);
        let attb = take(1)[0];
        let inpw = take(C1 * 2 * c2);
        let inpb = take(C1);
        let mut tw = Vec::new();
        let mut tb = Vec::new();
        for _ in 0..dils.len() {
            tw.push(take(C1 * C1 * 5));
            tb.push(take(C1));
        }
        let outw = take(C1);
        let outb = take(1)[0];
        if at != n {
            return Err(format!("{n} weights, {at} used"));
        }
        let q = |v: &[f32]| v.iter().map(|x| (x * WQ).round().clamp(-32767.0, 32767.0) as i16).collect::<Vec<i16>>();
        let (c1q, c2q, c3q) = (q(&c1w), q(&c2w), q(&c3w));
        let twq = tw.iter().map(|v| q(v)).collect();
        let float = std::env::var_os("RSNN_FLOAT").is_some();
        Ok(Net { hop, c2, c1, dils, c1w, c1b, c2w, c2b, c3w, c3b, attw, attb, inpw, inpb, tw, tb, outw, outb, c1q, c2q, c3q, twq, float })
    }

    /// The front's parameters as the FPGA engine (maia-hdl rsnn_front.py)
    /// takes them: 16-bit weights c1, c2, c3 (conv2d_i layouts), attention,
    /// 1x1 input (x WQ); 32-bit biases c1, c2, c3, attention, 1x1 (x AQ WQ).
    pub fn fpga_image(&self) -> (Vec<i16>, Vec<i32>) {
        let q = |v: f32| (v * WQ).round().clamp(-32767.0, 32767.0) as i16;
        let b = |v: f32| (v * AQ * WQ).round() as i32;
        let mut w: Vec<i16> = Vec::new();
        w.extend(&self.c1q);
        w.extend(&self.c2q);
        w.extend(&self.c3q);
        w.extend(self.attw.iter().map(|&v| q(v)));
        w.extend(self.inpw.iter().map(|&v| q(v)));
        let mut bs: Vec<i32> = Vec::new();
        bs.extend(self.c1b.iter().map(|&v| b(v)));
        bs.extend(self.c2b.iter().map(|&v| b(v)));
        bs.extend(self.c3b.iter().map(|&v| b(v)));
        bs.push(b(self.attb));
        bs.extend(self.inpb.iter().map(|&v| b(v)));
        (w, bs)
    }

    /// The 2-D and 1x1 channels (the FPGA engine's configuration).
    pub fn channels(&self) -> (usize, usize) {
        (self.c2, self.c1)
    }

    /// Frames of context either side a frame's output depends on.
    pub fn reach(&self) -> usize {
        3 + 2 * self.dils.iter().sum::<usize>()
    }

    /// Weights (the network's size).
    pub fn weights(&self) -> usize {
        self.c1w.len() + self.c2w.len() + self.c3w.len() + self.inpw.len() + self.tw.iter().map(|w| w.len()).sum::<usize>() + self.outw.len()
    }

    /// Logits (key down against up) of `x` (T rows of features).
    pub fn forward(&self, x: &[[f32; NB]]) -> Vec<f32> {
        let t = x.len();
        if t == 0 {
            return Vec::new();
        }
        if self.float {
            let h = self.front(x);
            self.temporal_f(h, t)
        } else {
            let hq = self.front_q(x);
            self.temporal_iq(hq, t)
        }
    }

    /// [`Self::front`] in fixed point all through (what the FPGA's engine
    /// does, bit for bit): h x AQ ([c][t], i32). The 2-D layers as
    /// conv2d_i; attention logits x AQ WQ in i32; softmax weights exp(-d)
    /// from [`exp_table`] (Q15, steps of 1/128, d the distance to the
    /// largest logit), their sum's reciprocal (2^31 / sum) once a frame;
    /// the pooled values x AQ; the 1x1 in i64 sums of weights x WQ.
    pub fn front_q(&self, x: &[[f32; NB]]) -> Vec<i32> {
        let (c1n, c2, t) = (self.c1, self.c2, x.len());
        let mut a0 = ActI::new(1, t, NB);
        for (ti, row) in x.iter().enumerate() {
            for (o, v) in a0.row_mut(0, ti).iter_mut().zip(row) {
                *o = (v * AQ).round() as i16;
            }
        }
        let a1 = conv2d_i::<5, 1>(&a0, &self.c1q, &self.c1b, c2, NB);
        let a2 = conv2d_i::<5, 2>(&a1, &self.c2q, &self.c2b, c2, NB2);
        let a3 = conv2d_i::<3, 1>(&a2, &self.c3q, &self.c3b, c2, NB2);
        let exp = exp_table();
        let q = |v: f32, s: f32| (v * s).round().clamp(-32767.0, 32767.0) as i32;
        let attw: Vec<i32> = self.attw.iter().map(|&w| q(w, WQ)).collect();
        let attb = (self.attb * AQ * WQ).round() as i32;
        let inpw: Vec<i32> = self.inpw.iter().map(|&w| q(w, WQ)).collect();
        let inpb: Vec<i64> = self.inpb.iter().map(|&b| (b * AQ * WQ).round() as i64).collect();
        let mut h = vec![0i32; c1n * t];
        let mut z = vec![0i64; 2 * c2];
        for ti in 0..t {
            let mut lg = [attb; NB2];
            for (c, &w) in attw.iter().enumerate() {
                for (l, &v) in lg.iter_mut().zip(a3.row(c, ti)) {
                    *l += w * v as i32;
                }
            }
            let m = *lg.iter().max().unwrap();
            let mut e = [0i64; NB2];
            for (ev, &l) in e.iter_mut().zip(&lg) {
                *ev = exp[(((m - l) as u32) >> EXP_SHIFT).min(EXP_N as u32 - 1) as usize] as i64;
            }
            let den: i64 = e.iter().sum();
            let r = (1i64 << 31) / den;
            for c in 0..c2 {
                let row = a3.row(c, ti);
                let num: i64 = row.iter().zip(&e).map(|(&v, &ev)| v as i64 * ev).sum();
                // (num >> 16) x r in 48 bits: one DSP48 in the FPGA
                z[c] = ((num >> 16) * r) >> 15;
                z[c2 + c] = *row.iter().max().unwrap() as i64;
            }
            for o in 0..c1n {
                let w = &inpw[o * 2 * c2..(o + 1) * 2 * c2];
                let acc: i64 = inpb[o] + w.iter().zip(&z).map(|(&a, &b)| a as i64 * b).sum::<i64>();
                h[o * t + ti] = (acc.max(0) / WQ as i64) as i32;
            }
        }
        h
    }

    /// The 2-D layers, pooling and the 1x1 into the temporal layers: h
    /// ([c][t], C1 x T). A frame's value depends on the rows 3 either side.
    fn front(&self, x: &[[f32; NB]]) -> Vec<f32> {
        #[allow(non_snake_case)]
        let C1 = self.c1;
        let t = x.len();
        if t == 0 {
            return Vec::new();
        }
        let c2 = self.c2;
        let a3 = if self.float {
            let mut a0 = Act::new(1, t, NB);
            for (ti, row) in x.iter().enumerate() {
                a0.row_mut(0, ti).copy_from_slice(row);
            }
            let a1 = conv2d::<5, 1>(&a0, &self.c1w, &self.c1b, c2, NB);
            let a2 = conv2d::<5, 2>(&a1, &self.c2w, &self.c2b, c2, NB2);
            conv2d::<3, 1>(&a2, &self.c3w, &self.c3b, c2, NB2)
        } else {
            let mut a0 = ActI::new(1, t, NB);
            for (ti, row) in x.iter().enumerate() {
                for (o, v) in a0.row_mut(0, ti).iter_mut().zip(row) {
                    *o = (v * AQ).round() as i16;
                }
            }
            let a1 = conv2d_i::<5, 1>(&a0, &self.c1q, &self.c1b, c2, NB);
            let a2 = conv2d_i::<5, 2>(&a1, &self.c2q, &self.c2b, c2, NB2);
            let a3 = conv2d_i::<3, 1>(&a2, &self.c3q, &self.c3b, c2, NB2);
            let mut f = Act::new(c2, t, NB2);
            for c in 0..c2 {
                for ti in 0..t {
                    for (o, v) in f.row_mut(c, ti).iter_mut().zip(a3.row(c, ti)) {
                        *o = *v as f32 / AQ;
                    }
                }
            }
            f
        };
        // pooling over frequency: attention (softmax of a 1x1 conv) and max
        let mut z = vec![0f32; 2 * c2 * t];
        let mut lg = [0f32; NB2];
        for ti in 0..t {
            lg.fill(self.attb);
            for c in 0..c2 {
                let w = self.attw[c];
                for (l, v) in lg.iter_mut().zip(a3.row(c, ti)) {
                    *l += w * v;
                }
            }
            let m = lg.iter().cloned().fold(f32::MIN, f32::max);
            let mut den = 0f32;
            for l in lg.iter_mut() {
                *l = (*l - m).exp();
                den += *l;
            }
            for c in 0..c2 {
                let row = a3.row(c, ti);
                let mut s = 0f32;
                let mut mx = f32::MIN;
                for (v, l) in row.iter().zip(&lg) {
                    s += v * l;
                    mx = mx.max(*v);
                }
                z[c * t + ti] = s / den;
                z[(c2 + c) * t + ti] = mx;
            }
        }
        // 1x1 to C1 channels, [c][t]
        let mut h = vec![0f32; C1 * t];
        for o in 0..C1 {
            let w = &self.inpw[o * 2 * c2..(o + 1) * 2 * c2];
            let out = &mut h[o * t..(o + 1) * t];
            out.fill(self.inpb[o]);
            for (i, &wi) in w.iter().enumerate() {
                for (ov, zv) in out.iter_mut().zip(&z[i * t..(i + 1) * t]) {
                    *ov += wi * zv;
                }
            }
            out.iter_mut().for_each(|v| *v = v.max(0.0));
        }
        h
    }

    fn temporal_f(&self, mut h: Vec<f32>, t: usize) -> Vec<f32> {
        #[allow(non_snake_case)]
        let C1 = self.c1;
        let mut tmp = vec![0f32; C1 * t];
        for (layer, d) in self.dils.iter().enumerate() {
            let (w, b) = (&self.tw[layer], &self.tb[layer]);
            for o in 0..C1 {
                let out = &mut tmp[o * t..(o + 1) * t];
                out.fill(b[o]);
                for i in 0..C1 {
                    let inp = &h[i * t..(i + 1) * t];
                    for k in 0..5 {
                        let wk = w[(o * C1 + i) * 5 + k];
                        let sh = k as isize * *d as isize - 2 * *d as isize;
                        let (lo, hi) = ((-sh).max(0) as usize, (t as isize - sh.max(0)).max(0) as usize);
                        if hi <= lo {
                            continue;
                        }
                        let src = &inp[(lo as isize + sh) as usize..(hi as isize + sh) as usize];
                        for (ov, iv) in out[lo..hi].iter_mut().zip(src) {
                            *ov += wk * iv;
                        }
                    }
                }
            }
            for (hv, tv) in h.iter_mut().zip(&tmp) {
                *hv += tv.max(0.0);
            }
        }
        let mut out = vec![self.outb; t];
        for c in 0..C1 {
            let w = self.outw[c];
            for (ov, hv) in out.iter_mut().zip(&h[c * t..(c + 1) * t]) {
                *ov += w * hv;
            }
        }
        out
    }

    /// The temporal layers and the output in fixed point (h x AQ in i32,
    /// taken as i16 into each layer; weights x WQ).
    /// [`Self::temporal_iq`] from h in float.
    #[allow(dead_code)]
    fn temporal_i(&self, h: &[f32], t: usize) -> Vec<f32> {
        self.temporal_iq(h.iter().map(|v| (v * AQ).round() as i32).collect(), t)
    }

    fn temporal_iq(&self, mut hq: Vec<i32>, t: usize) -> Vec<f32> {
        #[allow(non_snake_case)]
        let C1 = self.c1;
        let mut inp = vec![0i16; C1 * t];
        let mut acc = vec![0i32; t];
        let mut add = vec![0i32; C1 * t];
        for (layer, d) in self.dils.iter().enumerate() {
            for (o, v) in inp.iter_mut().zip(&hq) {
                *o = (*v).clamp(-32767, 32767) as i16;
            }
            let (w, b) = (&self.twq[layer], &self.tb[layer]);
            for o in 0..C1 {
                acc.fill((b[o] * AQ * WQ).round() as i32);
                for i in 0..C1 {
                    let x = &inp[i * t..(i + 1) * t];
                    for k in 0..5 {
                        let wk = w[(o * C1 + i) * 5 + k] as i32;
                        let sh = k as isize * *d as isize - 2 * *d as isize;
                        let (lo, hi) = ((-sh).max(0) as usize, (t as isize - sh.max(0)).max(0) as usize);
                        if hi <= lo {
                            continue;
                        }
                        let src = &x[(lo as isize + sh) as usize..(hi as isize + sh) as usize];
                        for (av, &xv) in acc[lo..hi].iter_mut().zip(src) {
                            *av += wk * xv as i32;
                        }
                    }
                }
                for (a, v) in add[o * t..(o + 1) * t].iter_mut().zip(&acc) {
                    *a = (*v).max(0) / WQ as i32;
                }
            }
            for (hv, a) in hq.iter_mut().zip(&add) {
                *hv += a;
            }
        }
        let mut out = vec![self.outb; t];
        for c in 0..C1 {
            let w = self.outw[c] / AQ;
            for (ov, hv) in out.iter_mut().zip(&hq[c * t..(c + 1) * t]) {
                *ov += w * *hv as f32;
            }
        }
        out
    }
}

/// Activations [c][t][f] with a zero border: one frame before and after,
/// two bins either side.
struct Act {
    t: usize,
    f: usize,
    d: Vec<f32>,
}

impl Act {
    fn new(c: usize, t: usize, f: usize) -> Act {
        Act { t, f, d: vec![0.0; c * (t + 2) * (f + 4)] }
    }
    /// Padded row (f + 4 values) of channel c at padded time index tp.
    fn prow(&self, c: usize, tp: usize) -> &[f32] {
        let w = self.f + 4;
        let at = (c * (self.t + 2) + tp) * w;
        &self.d[at..at + w]
    }
    fn row(&self, c: usize, t: usize) -> &[f32] {
        &self.prow(c, t + 1)[2..2 + self.f]
    }
    fn row_mut(&mut self, c: usize, t: usize) -> &mut [f32] {
        let w = self.f + 4;
        let at = (c * (self.t + 2) + t + 1) * w + 2;
        &mut self.d[at..at + self.f]
    }
}

/// A 2-D convolution (time kernel 3, frequency kernel KF padded KF/2,
/// frequency stride ST) to `cout` channels of `fout` bins, ReLU.
fn conv2d<const KF: usize, const ST: usize>(a: &Act, w: &[f32], b: &[f32], cout: usize, fout: usize) -> Act {
    let cin = a.d.len() / ((a.t + 2) * (a.f + 4));
    let mut out = Act::new(cout, a.t, fout);
    let off = 2 - KF / 2;
    let mut acc = vec![0f32; fout];
    for o in 0..cout {
        for ti in 0..a.t {
            acc.fill(b[o]);
            for i in 0..cin {
                for kt in 0..3 {
                    let irow = a.prow(i, ti + kt);
                    let wk: &[f32; KF] = w[((o * cin + i) * 3 + kt) * KF..][..KF].try_into().unwrap();
                    for (f, av) in acc.iter_mut().enumerate() {
                        let x: &[f32; KF] = irow[f * ST + off..][..KF].try_into().unwrap();
                        let mut s = 0f32;
                        for k in 0..KF {
                            s += wk[k] * x[k];
                        }
                        *av += s;
                    }
                }
            }
            for (ov, av) in out.row_mut(o, ti).iter_mut().zip(&acc) {
                *ov = av.max(0.0);
            }
        }
    }
    out
}

/// [`Act`] in 16-bit fixed point (x AQ).
struct ActI {
    t: usize,
    f: usize,
    d: Vec<i16>,
}

impl ActI {
    fn new(c: usize, t: usize, f: usize) -> ActI {
        ActI { t, f, d: vec![0; c * (t + 2) * (f + 4)] }
    }
    fn prow(&self, c: usize, tp: usize) -> &[i16] {
        let w = self.f + 4;
        let at = (c * (self.t + 2) + tp) * w;
        &self.d[at..at + w]
    }
    fn row(&self, c: usize, t: usize) -> &[i16] {
        &self.prow(c, t + 1)[2..2 + self.f]
    }
    fn row_mut(&mut self, c: usize, t: usize) -> &mut [i16] {
        let w = self.f + 4;
        let at = (c * (self.t + 2) + t + 1) * w + 2;
        &mut self.d[at..at + self.f]
    }
}

/// [`conv2d`] in fixed point: i16 x i16 into i32 sums, rescaled to x AQ.
fn conv2d_i<const KF: usize, const ST: usize>(a: &ActI, w: &[i16], b: &[f32], cout: usize, fout: usize) -> ActI {
    let cin = a.d.len() / ((a.t + 2) * (a.f + 4));
    let mut out = ActI::new(cout, a.t, fout);
    let off = 2 - KF / 2;
    let mut acc = vec![0i32; fout];
    for o in 0..cout {
        let bias = (b[o] * AQ * WQ).round() as i32;
        for ti in 0..a.t {
            acc.fill(bias);
            for i in 0..cin {
                for kt in 0..3 {
                    let irow = a.prow(i, ti + kt);
                    let wk: &[i16; KF] = w[((o * cin + i) * 3 + kt) * KF..][..KF].try_into().unwrap();
                    for (f, av) in acc.iter_mut().enumerate() {
                        let x: &[i16; KF] = irow[f * ST + off..][..KF].try_into().unwrap();
                        let mut s = 0i32;
                        for k in 0..KF {
                            s += wk[k] as i32 * x[k] as i32;
                        }
                        *av += s;
                    }
                }
            }
            let sh = WQ as i32;
            for (ov, av) in out.row_mut(o, ti).iter_mut().zip(&acc) {
                *ov = ((*av).max(0) / sh).min(32767) as i16;
            }
        }
    }
    out
}

/// Streaming: audio in, each frame's LLR out (a little behind: the
/// network's reach, in chunks).
/// The network frame by frame: every frame through every layer once (the
/// batch [`Net::forward`] on overlapping chunks redid 2 x reach frames of
/// context each time: 2.3 times the work for the larger networks). The
/// same fixed-point arithmetic as `temporal_i`: equal to the batch, bit for
/// bit, on the frames it gives out.
pub struct Stream {
    net: Net,
    /// Feature rows from absolute frame `rows_at` on; `front_done`: frames
    /// whose h is out.
    rows: Vec<[f32; NB]>,
    rows_at: usize,
    front_done: usize,
    /// Per temporal layer: its input frames (i32 and clamped i16) from
    /// absolute frame `at` on, the next frame it gives out; the weights as
    /// [out][tap][in].
    layers: Vec<TLayer>,
    /// The last layer's outputs not yet turned into logits start here.
    out_next: usize,
    /// The front in the FPGA (on a board with the engine free), else
    /// [`Net::front_q`] here.
    #[cfg(target_os = "linux")]
    fpga: Option<crate::rsnn_fpga::FpgaFront>,
}

struct TLayer {
    d: usize,
    at: usize,
    next: usize,
    h: std::collections::VecDeque<Vec<i32>>,
    q: std::collections::VecDeque<Vec<i16>>,
    w: Vec<i16>,
    b: Vec<i32>,
}

impl Stream {
    pub fn new(net: Net) -> Stream {
        let c1 = net.c1;
        let layers = net
            .dils
            .iter()
            .enumerate()
            .map(|(l, &d)| {
                let src = &net.twq[l];
                let mut w = vec![0i16; c1 * 5 * c1];
                for o in 0..c1 {
                    for i in 0..c1 {
                        for k in 0..5 {
                            w[(o * 5 + k) * c1 + i] = src[(o * c1 + i) * 5 + k];
                        }
                    }
                }
                let b = net.tb[l].iter().map(|v| (v * AQ * WQ).round() as i32).collect();
                TLayer { d, at: 0, next: 0, h: Default::default(), q: Default::default(), w, b }
            })
            .collect();
        #[cfg(target_os = "linux")]
        let fpga = crate::rsnn_fpga::FpgaFront::open(&net);
        Stream {
            net,
            rows: Vec::new(),
            rows_at: 0,
            front_done: 0,
            layers,
            out_next: 0,
            #[cfg(target_os = "linux")]
            fpga,
        }
    }

    pub fn net(&self) -> &Net {
        &self.net
    }

    /// The front runs in the FPGA.
    pub fn on_fpga(&self) -> bool {
        #[cfg(target_os = "linux")]
        return self.fpga.is_some();
        #[cfg(not(target_os = "linux"))]
        false
    }

    /// Feature rows in; the logits of the frames now final appended to
    /// `out` (each frame's once all the context it takes is in).
    pub fn push(&mut self, rows: &[[f32; NB]], out: &mut Vec<f32>) {
        #[cfg(target_os = "linux")]
        if let Some(f) = self.fpga.as_mut() {
            let mut hs = Vec::new();
            for row in rows {
                if let Some(h) = f.push(row) {
                    hs.push(h.iter().map(|&v| v as i32).collect::<Vec<i32>>());
                }
            }
            for h in hs {
                self.feed(0, h, out);
                self.front_done += 1;
            }
            return;
        }
        self.rows.extend_from_slice(rows);
        let end = self.rows_at + self.rows.len();
        // the front: frames with 3 rows after them (and 3 before, or the start)
        if end >= self.front_done + 3 + 1 {
            let (s, e) = (self.front_done, end - 3);
            let lo = s.saturating_sub(3).max(self.rows_at);
            let win = &self.rows[lo - self.rows_at..end - self.rows_at];
            let h = self.net.front_q(win);
            let t = win.len();
            let c1 = self.net.c1;
            for f in s..e {
                let v: Vec<i32> = (0..c1).map(|c| h[c * t + (f - lo)]).collect();
                self.feed(0, v, out);
            }
            self.front_done = e;
            // keep 3 rows of context before the next frame
            let keep = self.front_done.saturating_sub(3);
            if keep > self.rows_at {
                self.rows.drain(..keep - self.rows_at);
                self.rows_at = keep;
            }
        }
    }

    /// Frame `v` (i32, x AQ) into temporal layer `l`; what it completes
    /// goes on.
    fn feed(&mut self, l: usize, v: Vec<i32>, out: &mut Vec<f32>) {
        if l == self.layers.len() {
            let net = &self.net;
            let mut o = net.outb;
            for (c, &hv) in v.iter().enumerate() {
                o += net.outw[c] / AQ * hv as f32;
            }
            out.push(o);
            self.out_next += 1;
            return;
        }
        let c1 = self.net.c1;
        let lay = &mut self.layers[l];
        lay.q.push_back(v.iter().map(|x| (*x).clamp(-32767, 32767) as i16).collect());
        lay.h.push_back(v);
        let mut done = Vec::new();
        loop {
            let (d, t) = (lay.d, lay.next);
            // out[t] takes in[t - 2d ..= t + 2d]
            if lay.at + lay.h.len() <= t + 2 * d {
                break;
            }
            let mut o = vec![0i32; c1];
            for (oc, ov) in o.iter_mut().enumerate() {
                let mut acc = lay.b[oc];
                for k in 0..5 {
                    let tau = t as isize + (k as isize - 2) * d as isize;
                    if tau < 0 {
                        continue;
                    }
                    let x = &lay.q[tau as usize - lay.at];
                    let w = &lay.w[(oc * 5 + k) * c1..(oc * 5 + k + 1) * c1];
                    acc += w.iter().zip(x.iter()).map(|(&a, &b)| a as i32 * b as i32).sum::<i32>();
                }
                *ov = lay.h[t - lay.at][oc] + acc.max(0) / WQ as i32;
            }
            done.push(o);
            lay.next += 1;
            // inputs older than next - 2d are not needed again
            while lay.at + 2 * d < lay.next {
                lay.h.pop_front();
                lay.q.pop_front();
                lay.at += 1;
            }
        }
        for o in done {
            self.feed(l + 1, o, out);
        }
    }
}

pub struct RsNn {
    feat: Features,
    stream: Stream,
    rows: Vec<[f32; NB]>,
    lg: Vec<f32>,
    prior: f32,
}

impl RsNn {
    pub fn new() -> RsNn {
        let net = Net::builtin();
        RsNn { feat: Features::new(net.hop), stream: Stream::new(net), rows: Vec::new(), lg: Vec::new(), prior: (0.45f32 / 0.55).ln() }
    }

    /// Samples between the frames (and LLRs) given out.
    pub fn hop(&self) -> usize {
        self.stream.net().hop
    }

    /// Audio in (12 kHz); LLRs of the frames now final appended to `out`.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<f32>) {
        self.rows.clear();
        self.feat.process(audio, &mut self.rows);
        if self.rows.is_empty() {
            return;
        }
        self.lg.clear();
        self.stream.push(&self.rows, &mut self.lg);
        out.extend(self.lg.iter().map(|v| v - self.prior));
    }
}

impl Default for RsNn {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The streamer against the batch forward pass: equal, bit for bit
    /// (RSNN_WEIGHTS: another network).
    #[test]
    fn stream_matches_batch() {
        let net = Net::builtin();
        let t = 600 + 2 * net.reach();
        let mut x = 7u32;
        let rows: Vec<[f32; NB]> = (0..t)
            .map(|_| {
                std::array::from_fn(|_| {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    (x % 1800) as f32 / 100.0 - 6.0
                })
            })
            .collect();
        let want = net.forward(&rows);
        let reach = net.reach();
        let mut st = Stream::new(Net::builtin());
        eprintln!("front on the FPGA: {}", st.on_fpga());
        // RSNN_EXPECT_FPGA=1: on a board with the engine
        assert!(st.on_fpga() || std::env::var_os("RSNN_EXPECT_FPGA").is_none());
        let mut got = Vec::new();
        let mut at = 0;
        let mut n = 1;
        while at < t {
            let e = (at + n).min(t);
            st.push(&rows[at..e], &mut got);
            at = e;
            n = n * 3 % 37 + 1;
        }
        // (the batch pads the end with zeros; the streamer waits for rows)
        let upto = t - reach;
        assert!(got.len() >= upto - 10, "{} of {upto}", got.len());
        let bad = (0..upto.min(got.len())).find(|&i| got[i] != want[i]);
        assert!(bad.is_none(), "first difference at frame {bad:?}: {} vs {}", got[bad.unwrap()], want[bad.unwrap()]);
    }

    /// Vectors for the FPGA front (maia-hdl test_rsnn_front.py):
    /// RSNN_FRONT_VEC=<file.json> (RSNN_WEIGHTS: the network).
    #[test]
    #[ignore]
    fn rsnn_front_vectors() {
        // RSNN_FRONT_SMALL=1: a small random network (c2 4, c1 8), quick
        // to simulate; else RSNN_WEIGHTS or the built-in one.
        let net = if std::env::var_os("RSNN_FRONT_SMALL").is_some() {
            let (c2, c1) = (4usize, 8usize);
            let n = c2 * 15 + c2 + c2 * c2 * 15 + c2 + c2 * c2 * 9 + c2 + c2 + 1 + c1 * 2 * c2 + c1 + c1 * c1 * 5 + c1 + c1 + 1;
            let mut r = 5u32;
            let mut rnd = || {
                r ^= r << 13;
                r ^= r >> 17;
                r ^= r << 5;
                (r % 2001) as f32 / 1000.0 - 1.0
            };
            let mut b = b"RSN3".to_vec();
            for v in [128u32, c2 as u32, c1 as u32, 1, 1, n as u32] {
                b.extend(v.to_le_bytes());
            }
            for _ in 0..n {
                b.extend((rnd() * 0.6).to_le_bytes());
            }
            Net::from_bytes(&b).unwrap()
        } else {
            Net::builtin()
        };
        let t = 24;
        let mut x = 11u32;
        let rows: Vec<[f32; NB]> = (0..t)
            .map(|_| {
                std::array::from_fn(|_| {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    ((x % 1800) as f32 / 100.0 - 6.0) * 0.7
                })
            })
            .collect();
        let h = net.front_q(&rows);
        let (c2, c1) = net.channels();
        // frames with all 3 rows after them (the stream's outputs)
        let expect: Vec<Vec<i32>> = (0..t - 3).map(|k| (0..c1).map(|c| h[c * t + k]).collect()).collect();
        let rq: Vec<Vec<i16>> = rows.iter().map(|r| r.iter().map(|v| (v * AQ).round() as i16).collect()).collect();
        let (w, b) = net.fpga_image();
        let j = serde_json::json!({"c2": c2, "c1": c1, "w": w, "b": b, "rows": rq, "h": expect});
        std::fs::write(std::env::var("RSNN_FRONT_VEC").expect("RSNN_FRONT_VEC"), j.to_string()).unwrap();
        eprintln!("front vectors: c2 {c2} c1 {c1}, {} weights, {} biases, {t} rows", w.len(), b.len());
    }

    /// The network's time a frame (run on the board: `trxd-test rsnn_speed
    /// --ignored --nocapture`; RSNN_WEIGHTS for another network).
    #[test]
    #[ignore]
    fn rsnn_speed() {
        let net = Net::builtin();
        let t = 192 + 2 * net.reach();
        let x: Vec<[f32; NB]> = (0..t).map(|i| std::array::from_fn(|k| ((i * 7 + k * 3) % 11) as f32 - 3.0)).collect();
        let t0 = std::time::Instant::now();
        let reps = 10;
        for _ in 0..reps {
            std::hint::black_box(net.forward(&x));
        }
        let per = t0.elapsed().as_secs_f64() / (reps * 192) as f64;
        let fps = 12_000.0 / net.hop as f64;
        eprintln!("{} weights: batch {:.2} ms a frame (192 a chunk, reach {}): {:.1} % of a core at {fps:.0} frames/s", net.weights(), per * 1e3, net.reach(), 100.0 * per * fps);
        let (hop, w) = (net.hop, net.weights());
        let mut st = Stream::new(net);
        let long: Vec<[f32; NB]> = (0..2000).map(|i| x[i % x.len()]).collect();
        let mut out = Vec::new();
        let cpu = || {
            let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
            // SAFETY: a valid timespec out-pointer.
            unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
            ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
        };
        let (t0, c0) = (std::time::Instant::now(), cpu());
        for c in long.chunks(8) {
            st.push(c, &mut out);
        }
        let per = (cpu() - c0) / long.len() as f64;
        let wall = t0.elapsed().as_secs_f64() / long.len() as f64;
        eprintln!(
            "{w} weights: streaming {:.2} ms CPU ({:.2} ms wall) a frame: {:.1} % of a core ({} out, front on the FPGA: {})",
            per * 1e3,
            wall * 1e3,
            100.0 * per * 12_000.0 / hop as f64,
            out.len(),
            st.on_fpga()
        );
    }

    /// Against PyTorch: RSNN_VEC=<testvec.bin> (export.py, the same weights
    /// as rsnn.bin).
    #[test]
    #[ignore]
    fn rsnn_matches_torch() {
        let b = std::fs::read(std::env::var("RSNN_VEC").expect("RSNN_VEC")).unwrap();
        let t = u32::from_le_bytes(b[..4].try_into().unwrap()) as usize;
        let f: Vec<f32> = b[4..].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
        let feat: Vec<[f32; NB]> = (0..t).map(|i| f[i * NB..(i + 1) * NB].try_into().unwrap()).collect();
        let want = &f[t * NB..t * NB + t];
        let audio = &f[t * NB + t..];
        let net = Net::builtin();
        let mut fe = Features::new(net.hop);
        let mut rows = Vec::new();
        fe.process(audio, &mut rows);
        let fd = rows.iter().zip(&feat).flat_map(|(a, b)| a.iter().zip(b).map(|(x, y)| (x - y).abs())).fold(0f32, f32::max);
        eprintln!("features: {} rows (want {t}), max diff {fd}", rows.len());
        let got = net.forward(&feat);
        let d = got.iter().zip(want).map(|(a, b)| (a - b).abs()).fold(0f32, f32::max);
        eprintln!("logits ({}): max diff {d} (range {:.1}..{:.1})", if net.float { "float" } else { "fixed point" }, want.iter().cloned().fold(f32::MAX, f32::min), want.iter().cloned().fold(f32::MIN, f32::max));
        let mut s = RsNn::new();
        let mut l = Vec::new();
        for c in audio.chunks(700) {
            s.process(c, &mut l);
        }
        let batch = net.forward(&rows);
        let sd = l.iter().zip(&batch).map(|(a, b)| (a - (b - s.prior)).abs()).fold(0f32, f32::max);
        eprintln!("streaming: {} frames out, max diff to batch {sd}", l.len());
        assert!(rows.len() == t && fd < 0.02, "features differ");
        assert!(d < if net.float { 1e-3 } else { 0.25 }, "logits differ");
        assert!(sd < 1e-4, "streaming differs");
    }
}
