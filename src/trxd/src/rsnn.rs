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
//! The weight file gives the hop and the 2-D layers' channels. The 2-D and
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
const C1: usize = 32;
const NB2: usize = (NB + 4 - 5) / 2 + 1;
/// Frames of context either side a frame's output depends on.
pub const REACH: usize = 3 + 2 * (1 + 2 + 4 + 8);

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
const AQ: f32 = 128.0;
const WQ: f32 = 2048.0;

/// The network's weights.
pub struct Net {
    /// Samples between frames.
    pub hop: usize,
    /// The 2-D layers' channels.
    c2: usize,
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

    pub fn from_bytes(b: &[u8]) -> Result<Net, String> {
        if b.len() < 16 || &b[..4] != b"RSN2" {
            return Err("not an RSN2 weight file".into());
        }
        let u = |i: usize| u32::from_le_bytes(b[i..i + 4].try_into().unwrap()) as usize;
        let (hop, c2, n) = (u(4), u(8), u(12));
        if b.len() != 16 + 4 * n {
            return Err(format!("{} bytes for {n} weights", b.len()));
        }
        let all: Vec<f32> = b[16..].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
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
        for _ in 0..4 {
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
        Ok(Net { hop, c2, c1w, c1b, c2w, c2b, c3w, c3b, attw, attb, inpw, inpb, tw, tb, outw, outb, c1q, c2q, c3q, twq, float })
    }

    /// Logits (key down against up) of `x` (T rows of features).
    pub fn forward(&self, x: &[[f32; NB]]) -> Vec<f32> {
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
        if self.float {
            self.temporal_f(h, t)
        } else {
            self.temporal_i(&h, t)
        }
    }

    fn temporal_f(&self, mut h: Vec<f32>, t: usize) -> Vec<f32> {
        let mut tmp = vec![0f32; C1 * t];
        for (layer, d) in [1usize, 2, 4, 8].iter().enumerate() {
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
    fn temporal_i(&self, h: &[f32], t: usize) -> Vec<f32> {
        let mut hq: Vec<i32> = h.iter().map(|v| (v * AQ).round() as i32).collect();
        let mut inp = vec![0i16; C1 * t];
        let mut acc = vec![0i32; t];
        let mut add = vec![0i32; C1 * t];
        for (layer, d) in [1usize, 2, 4, 8].iter().enumerate() {
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
pub struct RsNn {
    feat: Features,
    net: Net,
    rows: Vec<[f32; NB]>,
    /// Frames of `rows` whose output has been given (the rest waits for
    /// context after it).
    done: usize,
    prior: f32,
}

/// Frames run through the network at once (plus the reach either side).
const CHUNK: usize = 192;

impl RsNn {
    pub fn new() -> RsNn {
        let net = Net::builtin();
        RsNn { feat: Features::new(net.hop), net, rows: Vec::new(), done: 0, prior: (0.45f32 / 0.55).ln() }
    }

    /// Samples between the frames (and LLRs) given out.
    pub fn hop(&self) -> usize {
        self.net.hop
    }

    /// Audio in (12 kHz); LLRs of the frames now final appended to `out`.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<f32>) {
        self.feat.process(audio, &mut self.rows);
        while self.rows.len() >= self.done + CHUNK + REACH {
            let lo = self.done.saturating_sub(REACH);
            let hi = self.done + CHUNK + REACH;
            let lg = self.net.forward(&self.rows[lo..hi]);
            out.extend(lg[self.done - lo..self.done - lo + CHUNK].iter().map(|v| v - self.prior));
            self.done += CHUNK;
            let drop = self.done.saturating_sub(REACH);
            if drop > 0 {
                self.rows.drain(..drop);
                self.done -= drop;
            }
        }
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
