//! DVB-S2 receiver for the signals [`super::Modulator`] sends: QPSK short
//! frames with pilots, one known MODCOD, CCM, TS. Stream-rate IQ in, TS
//! packets out.
//!
//! ```text
//! IQ (stream rate) -> NCO (channel + AFC) -> RRC matched filter, evaluated
//!   at 2-3 samples/symbol -> Gardner timing (cubic interpolation) -> symbols
//! symbols -> PLHEADER search (chunked coherent correlation, confirmed one
//!   frame later) -> per frame: phase at the header and each pilot block,
//!   frequency from those, linear phase between them -> descramble -> LLRs
//!   scaled by the SNR of the known symbols -> LDPC -> BBFRAME (CRC-8) -> TS
//! ```
//!
//! Frequency error up to about +-1.5 kHz is pulled in by the header at
//! increasing lags; the pilots then hold it to a fraction of a hertz.

use std::collections::VecDeque;

use num_complex::Complex32;

use super::{BBHEADER, NLDPC, PILOT, Params, SLOT, TS_LEN, bb_scrambling, crc8, ldpc::Decoder, pl_scrambling, plheader, rrc_taps};

/// Header correlation (0..1) that counts as a PLHEADER.
const SYNC_MIN: f32 = 0.5;
/// Frames in a row whose header is missing before searching again.
const LOST_AFTER: u32 = 4;
/// Symbols either side of the predicted header position searched when locked.
const TRACK_SLACK: usize = 3;
/// Frames whose header lag correlations are summed before the carrier
/// estimate is trusted (about 1/sqrt(N) of the single-frame noise).
const ACQ_FRAMES: u32 = 16;
/// Header lags for the coarse carrier estimate (+-rs/2 down to +-rs/80).
const LAGS: [usize; 3] = [1, 8, 40];

#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub frames: u64,
    pub frames_bad: u64,
    /// Seconds spent in the LDPC decoder and in the rest of the receiver.
    pub ldpc_s: f64,
    pub other_s: f64,
    /// Of `frames_bad`: LDPC did not converge / the BBHEADER CRC failed.
    pub ldpc_fail: u64,
    pub crc_fail: u64,
    pub packets: u64,
    /// IQ blocks the receiving thread could not take in time.
    pub blocks_dropped: u64,
    /// Frames not decoded: hopeless (Es/N0 far too low) / decoder busy.
    pub frames_skipped: u64,
    pub frames_fec_busy: u64,
    /// Seconds since the receiver started (for CPU shares).
    pub wall_s: f64,
    pub locked: bool,
    /// Es/N0 of the last frame, dB.
    pub esn0_db: f32,
    /// Carrier offset the AFC has found, Hz.
    pub freq_hz: f32,
    /// Es/N0 of the last frame's data symbols, decision-directed (a check on
    /// phase and timing between the known blocks), dB.
    pub data_esn0_db: f32,
}

pub struct Receiver {
    p: Params,
    rs: f64,
    fs: f64,
    // Front end.
    nco: Complex32,
    nco_n: u32,
    /// Channel centre relative to the input (Hz) and the AFC's correction.
    center_hz: f64,
    afc_hz: f64,
    decim: usize,
    decim_n: usize,
    rrc: Vec<f32>,
    /// Filter history, written twice (`i` and `i + n`) so the newest `n`
    /// samples are always one contiguous slice: the dot product vectorizes.
    hist: Vec<Complex32>,
    hpos: usize,
    // Timing (on matched-filter output, `sps` samples per symbol).
    mf: Vec<Complex32>,
    t: f64,
    omega: f64,
    omega_nom: f64,
    agc: f32,
    prev_sym: Complex32,
    // Frames.
    syms: Vec<Complex32>,
    header: Vec<Complex32>,
    frame_len: usize,
    known: Vec<(usize, usize)>,
    scramble: Vec<u8>,
    locked_at: Option<usize>,
    candidate: Option<usize>,
    missed: u32,
    freq_est: f64,
    /// Frames since lock, for the frequency acquisition.
    frames_locked: u32,
    /// Header lag correlations summed over the acquisition frames.
    acq: [Complex32; 3],
    /// Frames in a row that failed to decode while tracking.
    fails: u32,
    // FEC: inline (tests, CLI) or on a thread of its own (trxd).
    fec: FecMode,
    llr: Vec<f32>,
    pub stats: Stats,
}

/// LDPC + BBFRAME -> TS: the expensive part, which can run on its own core.
pub struct Fec {
    p: Params,
    dec: Decoder,
    bbscr: Vec<u8>,
    bits: Vec<u8>,
    partial: Vec<u8>,
    have_prev: bool,
}

enum FecMode {
    Inline(Fec),
    /// Frames of LLRs to the decoding thread; it counts failures in a row
    /// (read back here for re-acquisition).
    Thread { tx: crossbeam_channel::Sender<Vec<f32>>, fails: std::sync::Arc<std::sync::atomic::AtomicU32> },
}

/// Es/N0 (dB) below which a frame of this rate is not worth decoding.
fn hopeless_below(rate: super::Rate) -> f32 {
    match rate {
        super::Rate::R1_4 => -5.0,
        super::Rate::R1_3 => -3.5,
        super::Rate::R1_2 => -2.0,
        super::Rate::R2_3 => 0.5,
    }
}

impl Fec {
    pub fn new(p: Params) -> Self {
        Fec { p, dec: Decoder::new(p.rate), bbscr: bb_scrambling(p.rate.kbch() / 8), bits: vec![0; NLDPC], partial: Vec::new(), have_prev: false }
    }

    /// Decode one frame of LLRs; TS packets out. False if it did not decode.
    pub fn frame(&mut self, llr: &[f32], stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode(llr, &mut self.bits);
        stats.ldpc_s += t0.elapsed().as_secs_f64();
        if ok.is_none() {
            stats.frames_bad += 1;
            stats.ldpc_fail += 1;
            self.have_prev = false;
            return false;
        }
        self.deframe(stats, out)
    }

    /// A frame lost before decoding: the packet straddling it is gone too.
    pub fn lost(&mut self) {
        self.have_prev = false;
    }
}

/// Cubic (Catmull-Rom) interpolation of `x` at fractional index `t`.
fn interp(x: &[Complex32], t: f64) -> Complex32 {
    let i = t.floor() as usize;
    let mu = (t - i as f64) as f32;
    let (a, b, c, d) = (x[i - 1], x[i], x[i + 1], x[i + 2]);
    let c0 = b;
    let c1 = (c - a) * 0.5;
    let c2 = a - b * 2.5 + c * 2.0 - d * 0.5;
    let c3 = (d - a) * 0.5 + (b - c) * 1.5;
    ((c3 * mu + c2) * mu + c1) * mu + c0
}

fn wrap(p: f64) -> f64 {
    (p + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

impl Receiver {
    /// `fs`: input rate; `rs`: symbol rate (fs / rs a whole number);
    /// `center_hz`: where the signal sits in the input.
    pub fn new(p: Params, fs: f64, rs: f64, center_hz: f64) -> Self {
        let sps_in = (fs / rs).round() as usize;
        // Decimate to 2 or 3 samples per symbol inside the matched filter.
        let decim = (1..=sps_in).rev().find(|d| sps_in % d == 0 && sps_in / d >= 2).unwrap_or(1);
        let sps = sps_in / decim;
        let rrc = rrc_taps(sps_in, p.rolloff, 12);
        let frame_len = p.frame_symbols();
        let n_data = frame_len - SLOT;
        // Known blocks: the header, then each pilot block (after 16 slots).
        let mut known = vec![(0usize, SLOT)];
        if p.pilots {
            let slots = NLDPC / 2 / SLOT;
            for i in 1..=(slots - 1) / 16 {
                let at = SLOT + i * 16 * SLOT + (i - 1) * PILOT;
                known.push((at, PILOT));
            }
        }
        Receiver {
            p,
            rs,
            fs,
            nco: Complex32::new(1.0, 0.0),
            nco_n: 0,
            center_hz,
            afc_hz: 0.0,
            decim,
            decim_n: 0,
            hist: vec![Complex32::default(); 2 * rrc.len()],
            hpos: 0,
            rrc,
            mf: Vec::new(),
            t: 4.0,
            omega: sps as f64,
            omega_nom: sps as f64,
            agc: 1.0,
            prev_sym: Complex32::default(),
            syms: Vec::new(),
            header: plheader(p.rate.modcod(), p.pilots),
            frame_len,
            known,
            scramble: pl_scrambling(n_data),
            locked_at: None,
            candidate: None,
            missed: 0,
            freq_est: 0.0,
            frames_locked: 0,
            acq: [Complex32::default(); 3],
            fails: 0,
            fec: FecMode::Inline(Fec::new(p)),
            llr: vec![0.0; NLDPC],
            stats: Stats::default(),
        }
    }

    /// The signal moved in the input (the LO was retuned). The carrier error
    /// the AFC found is the transmitter's, so it stays.
    pub fn set_center(&mut self, hz: f64) {
        self.center_hz = hz;
    }

    /// Feed input samples; whole TS packets come out as frames complete.
    pub fn process(&mut self, iq: &[Complex32], out: &mut Vec<[u8; TS_LEN]>) {
        let t0 = std::time::Instant::now();
        let ldpc0 = self.stats.ldpc_s;
        self.process_inner(iq, out);
        self.stats.other_s += t0.elapsed().as_secs_f64() - (self.stats.ldpc_s - ldpc0);
    }

    fn process_inner(&mut self, iq: &[Complex32], out: &mut Vec<[u8; TS_LEN]>) {
        let w = -std::f64::consts::TAU * (self.center_hz + self.afc_hz) / self.fs;
        let step = Complex32::new(w.cos() as f32, w.sin() as f32);
        let n = self.rrc.len();
        for &x in iq {
            let y = x * self.nco;
            self.hist[self.hpos] = y;
            self.hist[self.hpos + n] = y;
            self.hpos = (self.hpos + 1) % n;
            self.nco *= step;
            self.nco_n += 1;
            if self.nco_n == 1024 {
                // Keep the phasor on the unit circle.
                self.nco_n = 0;
                self.nco /= self.nco.norm();
            }
            self.decim_n += 1;
            if self.decim_n == self.decim {
                self.decim_n = 0;
                // Oldest first: hist[hpos..hpos + n] (taps are symmetric).
                let win = &self.hist[self.hpos..self.hpos + n];
                let (mut re, mut im) = (0f32, 0f32);
                for (h, s) in self.rrc.iter().zip(win) {
                    re += h * s.re;
                    im += h * s.im;
                }
                self.mf.push(Complex32::new(re, im));
            }
        }
        self.timing();
        self.frames(out);
    }

    /// Gardner timing recovery: matched-filter samples -> one per symbol.
    fn timing(&mut self) {
        let (kp, ki) = (0.01, 0.0001);
        while self.t + self.omega + 3.0 < self.mf.len() as f64 {
            let y = interp(&self.mf, self.t);
            let mid = interp(&self.mf, self.t - self.omega / 2.0);
            let p = y.norm_sqr().max(1e-20);
            self.agc += 0.001 * (p - self.agc);
            // Gardner: positive when the strobe is late (the midpoint leans
            // toward the newer symbol), so the next strobe comes sooner.
            let e = ((self.prev_sym - y) * mid.conj()).re / self.agc.max(1e-20);
            let e = e.clamp(-1.0, 1.0) as f64;
            self.prev_sym = y;
            self.syms.push(y / self.agc.sqrt());
            self.omega = (self.omega + ki * e).clamp(self.omega_nom * 0.99, self.omega_nom * 1.01);
            self.t += self.omega + kp * e;
        }
        // Keep a little history for the interpolator.
        let keep = (self.t.floor() as usize).saturating_sub(4);
        if keep > 4096 {
            self.mf.drain(..keep);
            self.t -= keep as f64;
        }
    }

    /// Normalized header correlation at symbol `k` (0..1), chunked coherent
    /// (10 symbols) so a frequency error of a few hundred Hz does not hurt.
    fn header_metric(&self, k: usize) -> f32 {
        let s = &self.syms[k..k + SLOT];
        let (mut num, mut den) = (0f32, 0f32);
        for c in 0..SLOT / 10 {
            let mut acc = Complex32::default();
            for i in c * 10..c * 10 + 10 {
                acc += s[i] * self.header[i].conj();
                den += s[i].norm();
            }
            num += acc.norm();
        }
        num / den.max(1e-20)
    }

    fn frames(&mut self, out: &mut Vec<[u8; TS_LEN]>) {
        let l = self.frame_len;
        loop {
            match self.locked_at {
                None => {
                    // Search: the best header position within one frame of symbols.
                    let start = self.candidate.map_or(0, |c| c + l - TRACK_SLACK);
                    let need = start + if self.candidate.is_some() { 2 * TRACK_SLACK } else { l } + SLOT;
                    if self.syms.len() < need {
                        return;
                    }
                    let end = need - SLOT;
                    let (best, m) = (start..end).map(|k| (k, self.header_metric(k))).fold((start, 0f32), |a, b| if b.1 > a.1 { b } else { a });
                    if m >= SYNC_MIN {
                        if self.candidate.is_some() {
                            // Two headers one frame apart: locked.
                            self.locked_at = Some(best - l);
                            self.missed = 0;
                            self.frames_locked = 0;
                            self.acq = [Complex32::default(); 3];
                            self.stats.locked = true;
                        } else {
                            self.candidate = Some(best);
                        }
                    } else {
                        self.candidate = None;
                        let drop = end.saturating_sub(SLOT);
                        self.syms.drain(..drop.min(self.syms.len()));
                    }
                }
                Some(p) => {
                    // A frame is ready once the next header can be checked.
                    if self.syms.len() < p + l + TRACK_SLACK + SLOT {
                        return;
                    }
                    let lo = p + l - TRACK_SLACK;
                    let (next, m) = (lo..=p + l + TRACK_SLACK).map(|k| (k, self.header_metric(k))).fold((lo, 0f32), |a, b| if b.1 > a.1 { b } else { a });
                    if std::env::var_os("DVBS2_DEBUG").is_some() {
                        eprintln!("frame: next header at {:+} (metric {m:.2})", next as i64 - (p + l) as i64);
                    }
                    let next = if m >= SYNC_MIN {
                        self.missed = 0;
                        next
                    } else {
                        self.missed += 1;
                        p + l
                    };
                    self.frame(p, next, out);
                    if self.missed >= LOST_AFTER {
                        self.locked_at = None;
                        self.candidate = None;
                        self.stats.locked = false;
                        match &mut self.fec {
                            FecMode::Inline(f) => f.lost(),
                            FecMode::Thread { tx, .. } => {
                                let _ = tx.try_send(Vec::new());
                            }
                        }
                        self.syms.drain(..next);
                        continue;
                    }
                    // Keep the next frame at the front.
                    self.syms.drain(..next);
                    self.locked_at = Some(0);
                }
            }
        }
    }

    /// Phase of a known block (header at `at` with the reference, or a pilot).
    fn block(&self, base: usize, at: usize, len: usize) -> Complex32 {
        let mut acc = Complex32::default();
        let a = std::f32::consts::FRAC_1_SQRT_2;
        for i in 0..len {
            let s = self.syms[base + at + i];
            let r = if at == 0 {
                self.header[i]
            } else {
                // Pilots: (1+j)/sqrt2, PL-scrambled like the data.
                super::rotate(Complex32::new(a, a), self.scramble[at - SLOT + i])
            };
            acc += s * r.conj();
        }
        acc / len as f32
    }

    fn frame(&mut self, p: usize, next: usize, out: &mut Vec<[u8; TS_LEN]>) {
        let rs = self.rs;
        // Header: frequency at increasing lags (+-rs/2, then finer).
        let z: Vec<Complex32> = (0..SLOT).map(|i| self.syms[p + i] * self.header[i].conj()).collect();
        // Acquisition: the header's lag correlations, summed over ACQ_FRAMES
        // with the NCO held, then resolved coarse to fine (lag 1: +-rs/2, lag
        // 40: +-rs/80) and applied once. One frame alone is too noisy (+-15 Hz
        // at 8 dB) to unwrap phase across the 1476 symbols between pilots.
        let acquiring = self.frames_locked < ACQ_FRAMES;
        let mut f = self.freq_est;
        if acquiring {
            for (a, &lag) in self.acq.iter_mut().zip(&LAGS) {
                for i in 0..SLOT - lag {
                    *a += z[i + lag] * z[i].conj();
                }
            }
            let mut fa = 0.0;
            for (a, &lag) in self.acq.iter().zip(&LAGS) {
                let pred = std::f64::consts::TAU * fa * lag as f64 / rs;
                fa += wrap(a.arg() as f64 - pred) / (std::f64::consts::TAU * lag as f64) * rs;
            }
            f = fa;
        }
        // Known blocks through the frame and the next header: phases unwrapped
        // against the frequency so far, then a least-squares line.
        let mut pts: Vec<(f64, f64, f32)> = Vec::new(); // (centre symbol, phase, weight)
        let with_next = next + SLOT <= self.syms.len();
        for &(at, len) in &self.known {
            let c = self.block(p, at, len);
            pts.push(((at + len / 2) as f64, c.arg() as f64, c.norm() * len as f32));
        }
        if with_next {
            let c = self.block(next, 0, SLOT);
            pts.push(((next - p + SLOT / 2) as f64, c.arg() as f64, c.norm() * SLOT as f32));
        }
        // Unwrap against a predicted frequency, fit a line (weighted least
        // squares), score by the weighted residual. Phases sampled at the
        // pilots alias every rs/1476 Hz, but the uneven spacings (header to
        // first pilot 1503, last pilot to next header 963) leave a wrong alias
        // a residual of radians: try the neighbours and keep the best fit.
        let fit = |fp: f64| -> (Vec<f64>, f64, f64) {
            let mut u = vec![pts[0].1];
            for w in 1..pts.len() {
                let dt = pts[w].0 - pts[w - 1].0;
                let pred = u[w - 1] + std::f64::consts::TAU * fp * dt / rs;
                u.push(pred + wrap(pts[w].1 - pred));
            }
            let (mut sw, mut sx, mut sy, mut sxx, mut sxy) = (0f64, 0f64, 0f64, 0f64, 0f64);
            for (i, pt) in pts.iter().enumerate() {
                let w = pt.2 as f64;
                sw += w;
                sx += w * pt.0;
                sy += w * u[i];
                sxx += w * pt.0 * pt.0;
                sxy += w * pt.0 * u[i];
            }
            let slope = (sw * sxy - sx * sy) / (sw * sxx - sx * sx);
            if !slope.is_finite() || pts.len() < 3 {
                return (u, fp, 0.0);
            }
            let icpt = (sy - slope * sx) / sw;
            let rss: f64 = pts.iter().enumerate().map(|(i, pt)| pt.2 as f64 * (u[i] - icpt - slope * pt.0).powi(2)).sum::<f64>() / sw;
            (u, slope * rs / std::f64::consts::TAU, rss)
        };
        let alias = rs / (16 * SLOT + PILOT) as f64;
        // No alias hopping while tracking: noise sometimes favours a wrong
        // neighbour, and near-aliases three steps away then hold it there.
        // Trouble shows as failed frames, which restart acquisition instead.
        let span = 0;
        let (unwrapped, f_fit, _) = (-span..=span)
            .map(|k| fit(f + k as f64 * alias))
            .fold(None::<(Vec<f64>, f64, f64)>, |best, c| match best {
                Some(b) if b.2 <= c.2 => Some(b),
                _ => Some(c),
            })
            .unwrap();
        f = f_fit;
        self.frames_locked = self.frames_locked.saturating_add(1);
        self.stats.freq_hz = (self.afc_hz + f) as f32;
        if acquiring {
            if self.frames_locked == ACQ_FRAMES {
                // Acquired: the NCO takes the averaged estimate in one step.
                self.afc_hz += f;
                self.freq_est = 0.0;
            }
        } else {
            // Tracking: move the NCO slowly toward the estimate; what remains
            // is followed inside each frame by the phases above.
            self.freq_est = 0.8 * f;
            self.afc_hz += 0.2 * f;
        }

        // Phase for every symbol: linear between neighbouring known blocks.
        let phase_at = |k: f64| -> f64 {
            let i = pts.iter().rposition(|pt| pt.0 <= k).unwrap_or(0);
            let j = (i + 1).min(pts.len() - 1);
            if i == j {
                return unwrapped[i] + std::f64::consts::TAU * f * (k - pts[i].0) / rs;
            }
            let a = (k - pts[i].0) / (pts[j].0 - pts[i].0);
            unwrapped[i] + a * (unwrapped[j] - unwrapped[i])
        };
        // Amplitude and noise from the known symbols.
        let (mut amp, mut n_known) = (0f32, 0usize);
        let mut noise = 0f32;
        let a = std::f32::consts::FRAC_1_SQRT_2;
        for &(at, len) in &self.known {
            for i in 0..len {
                let k = at + i;
                let ph = phase_at(k as f64) as f32;
                let s = self.syms[p + k] * Complex32::new(ph.cos(), -ph.sin());
                let r = if at == 0 { self.header[i] } else { super::rotate(Complex32::new(a, a), self.scramble[k - SLOT]) };
                amp += (s * r.conj()).re;
                n_known += 1;
            }
        }
        amp /= n_known as f32;
        for &(at, len) in &self.known {
            for i in 0..len {
                let k = at + i;
                let ph = phase_at(k as f64) as f32;
                let s = self.syms[p + k] * Complex32::new(ph.cos(), -ph.sin());
                let r = if at == 0 { self.header[i] } else { super::rotate(Complex32::new(a, a), self.scramble[k - SLOT]) };
                noise += (s - r * amp).norm_sqr();
            }
        }
        let sigma2 = (noise / n_known as f32).max(1e-9);
        self.stats.esn0_db = 10.0 * (amp * amp / sigma2).log10();
        // Data symbols: derotate, descramble, LLRs (positive = 0).
        let scale = 2.0 * std::f32::consts::SQRT_2 * amp / sigma2 * a * std::f32::consts::SQRT_2;
        let (mut dd_sig, mut dd_err) = (0f32, 0f32);
        let mut n = 0usize; // data symbol index
        let mut k = SLOT; // position in the frame
        let mut pilot = 1usize;
        while n < NLDPC / 2 {
            if pilot < self.known.len() && k == self.known[pilot].0 {
                k += PILOT;
                pilot += 1;
                continue;
            }
            let ph = phase_at(k as f64) as f32;
            let s = self.syms[p + k] * Complex32::new(ph.cos(), -ph.sin());
            let d = super::rotate(s, (4 - self.scramble[k - SLOT]) & 3);
            self.llr[2 * n] = scale * d.re;
            self.llr[2 * n + 1] = scale * d.im;
            let dec = Complex32::new(amp * a * d.re.signum(), amp * a * d.im.signum());
            dd_sig += dec.norm_sqr();
            dd_err += (d - dec).norm_sqr();
            n += 1;
            k += 1;
        }
        self.stats.data_esn0_db = 10.0 * (dd_sig / dd_err.max(1e-9)).log10();
        self.stats.frames += 1;
        // Far below what this rate decodes (no signal, carrier wrong): the
        // decoder would only burn CPU. It counts as a failure.
        let hopeless = self.stats.esn0_db < hopeless_below(self.p.rate);
        let fails = match &mut self.fec {
            FecMode::Inline(f) => {
                if hopeless {
                    self.stats.frames_bad += 1;
                    self.stats.frames_skipped += 1;
                    f.lost();
                    self.fails + 1
                } else if f.frame(&self.llr, &mut self.stats, out) {
                    0
                } else {
                    self.fails + 1
                }
            }
            FecMode::Thread { tx, fails } => {
                if hopeless {
                    self.stats.frames_bad += 1;
                    self.stats.frames_skipped += 1;
                    // An empty frame tells the decoder one is missing.
                    let _ = tx.try_send(Vec::new());
                } else if tx.try_send(self.llr.clone()).is_err() {
                    self.stats.frames_bad += 1;
                    self.stats.frames_fec_busy += 1;
                }
                fails.load(std::sync::atomic::Ordering::Relaxed) + hopeless as u32
            }
        };
        self.fails = fails;
        if self.fails >= 3 && self.frames_locked >= ACQ_FRAMES {
            // Carrier wrong, or gone: acquire again from where the AFC is.
            self.frames_locked = 0;
            self.acq = [Complex32::default(); 3];
            self.freq_est = 0.0;
            self.fails = 0;
            if let FecMode::Thread { fails, .. } = &self.fec {
                fails.store(0, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}

impl Fec {
    /// BBFRAME -> TS packets (the reverse of [`super::Framer`]).
    fn deframe(&mut self, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let n = self.p.rate.kbch() / 8;
        let bb: Vec<u8> = (0..n)
            .map(|i| {
                let mut v = 0u8;
                for b in 0..8 {
                    v = (v << 1) | self.bits[i * 8 + b];
                }
                v ^ self.bbscr[i]
            })
            .collect();
        if crc8(&bb[..9]) != bb[9] {
            stats.frames_bad += 1;
            stats.crc_fail += 1;
            self.have_prev = false;
            return false;
        }
        let dfl = u16::from_be_bytes([bb[4], bb[5]]) as usize / 8;
        let syncd = u16::from_be_bytes([bb[7], bb[8]]) as usize / 8;
        let data = &bb[BBHEADER..(BBHEADER + dfl).min(n)];
        // The tail of a packet that began in the previous frame.
        if self.have_prev && syncd <= data.len() && self.partial.len() + syncd == TS_LEN {
            let mut up = std::mem::take(&mut self.partial);
            up.extend_from_slice(&data[..syncd]);
            self.emit(&up, stats, out);
        }
        self.partial.clear();
        let mut i = syncd;
        while i + TS_LEN <= data.len() {
            let up = data[i..i + TS_LEN].to_vec();
            self.emit(&up, stats, out);
            i += TS_LEN;
        }
        self.partial.extend_from_slice(&data[i.min(data.len())..]);
        self.have_prev = true;
        true
    }

    /// One user packet: its first byte is the CRC-8 of the packet before.
    fn emit(&mut self, up: &[u8], stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) {
        let mut pkt = [0u8; TS_LEN];
        pkt.copy_from_slice(up);
        pkt[0] = 0x47;
        stats.packets += 1;
        out.push(pkt);
    }
}

/// What the receiving thread hands back.
#[derive(Default)]
pub struct RxShared {
    pub stats: Stats,
    /// Browser messages ([`super::ts::Demux`]), oldest first.
    pub msgs: VecDeque<Vec<u8>>,
}

/// [`Receiver`] and [`super::ts::Demux`] on their own thread, off the
/// engine's sample path: it gets copies of the stream-rate IQ and drops
/// blocks (never stalls the engine) if the CPU cannot keep up.
pub struct RxThread {
    tx: crossbeam_channel::Sender<(Vec<Complex32>, f64)>,
    shared: std::sync::Arc<std::sync::Mutex<RxShared>>,
    pub params: Params,
    pub sr: f64,
    fec_stats: std::sync::Arc<std::sync::Mutex<Stats>>,
    /// Blocks dropped because the thread was behind (CPU too slow).
    dropped: std::sync::Arc<std::sync::atomic::AtomicU64>,
    started: std::time::Instant,
}

impl RxThread {
    pub fn start(p: Params, fs: f64, sr: f64, center_hz: f64) -> Self {
        use std::sync::{Arc, Mutex, atomic::AtomicU32, atomic::Ordering};
        let (tx, rx) = crossbeam_channel::bounded::<(Vec<Complex32>, f64)>(64);
        // A few frames of slack between demodulator and decoder: if decoding
        // falls behind, whole frames are skipped and timing stays intact.
        let (ftx, frx) = crossbeam_channel::bounded::<Vec<f32>>(4);
        let fails = Arc::new(AtomicU32::new(0));
        let shared = Arc::new(Mutex::new(RxShared::default()));
        let fec_stats = Arc::new(Mutex::new(Stats::default()));
        let (sh, fs2, fl) = (shared.clone(), fec_stats.clone(), fails.clone());
        std::thread::Builder::new()
            .name("datv-fec".into())
            .spawn(move || {
                let mut fec = Fec::new(p);
                let mut dmx = super::ts::Demux::default();
                let (mut ts, mut msgs) = (Vec::new(), Vec::new());
                let mut st = Stats::default();
                for llr in frx {
                    if llr.is_empty() {
                        fec.lost();
                        continue;
                    }
                    if fec.frame(&llr, &mut st, &mut ts) {
                        fl.store(0, Ordering::Relaxed);
                    } else {
                        fl.fetch_add(1, Ordering::Relaxed);
                    }
                    for pkt in ts.drain(..) {
                        dmx.push(&pkt, &mut msgs);
                    }
                    *fs2.lock().unwrap() = st;
                    let mut s = sh.lock().unwrap();
                    s.msgs.extend(msgs.drain(..));
                    let excess = s.msgs.len().saturating_sub(300);
                    s.msgs.drain(..excess);
                }
            })
            .expect("spawn datv-fec");
        let sh = shared.clone();
        std::thread::Builder::new()
            .name("datv-rx".into())
            .spawn(move || {
                let mut r = Receiver::new(p, fs, sr, center_hz);
                r.fec = FecMode::Thread { tx: ftx, fails };
                let mut none = Vec::new();
                for (iq, center) in rx {
                    r.set_center(center);
                    r.process(&iq, &mut none);
                    sh.lock().unwrap().stats = r.stats;
                }
            })
            .expect("spawn datv-rx");
        RxThread { tx, shared, fec_stats, params: p, sr, dropped: Default::default(), started: std::time::Instant::now() }
    }

    /// A block of stream IQ; the signal sits `center_hz` from its centre.
    pub fn feed(&self, iq: &[Complex32], center_hz: f64) {
        if self.tx.try_send((iq.to_vec(), center_hz)).is_err() {
            self.dropped.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Statistics (demodulator and decoder merged) and the messages decoded
    /// since the last call.
    pub fn take(&self) -> (Stats, Vec<Vec<u8>>) {
        let f = *self.fec_stats.lock().unwrap();
        let mut s = self.shared.lock().unwrap();
        let mut st = s.stats;
        st.frames_bad += f.frames_bad;
        st.ldpc_fail = f.ldpc_fail;
        st.crc_fail = f.crc_fail;
        st.packets = f.packets;
        st.ldpc_s = f.ldpc_s;
        st.blocks_dropped = self.dropped.load(std::sync::atomic::Ordering::Relaxed);
        st.wall_s = self.started.elapsed().as_secs_f64();
        (st, s.msgs.drain(..).collect())
    }
}

/// `trxd --dvbs2-demod IN.cf32 OUT.ts FS SR RATE CENTER_HZ [nopilots]`:
/// receive a recording (complex f32 at FS, signal at CENTER_HZ).
pub fn demod_cli(input: &str, output: &str, rest: &[String]) -> Result<(), String> {
    let num = |i: usize, what: &str| -> Result<f64, String> {
        rest.get(i).ok_or(format!("{what}"))?.parse().map_err(|_| format!("{what}: a number"))
    };
    let fs = num(0, "FS")?;
    let sr = num(1, "SR")?;
    let rate = rest.get(2).and_then(|s| super::Rate::parse(s)).ok_or("RATE: 1/4, 1/3, 1/2 or 2/3")?;
    let center = num(3, "CENTER_HZ")?;
    let p = Params { rate, pilots: !rest.iter().any(|s| s == "nopilots"), rolloff: 0.35 };
    let raw = std::fs::read(input).map_err(|e| format!("{input}: {e}"))?;
    let iq: Vec<Complex32> = raw
        .chunks_exact(8)
        .map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap())))
        .collect();
    let mut rx = Receiver::new(p, fs, sr, center);
    let mut ts = Vec::new();
    let t0 = std::time::Instant::now();
    for chunk in iq.chunks(3840) {
        rx.process(chunk, &mut ts);
    }
    let secs = t0.elapsed().as_secs_f64();
    let bytes: Vec<u8> = ts.iter().flatten().copied().collect();
    std::fs::write(output, bytes).map_err(|e| format!("{output}: {e}"))?;
    let s = rx.stats;
    eprintln!(
        "{} frames ({} failed: {} LDPC, {} CRC), {} TS packets, Es/N0 {:.1} dB (data, decision-directed {:.1} dB), carrier {:+.1} Hz, {:.1}x real time (LDPC {:.0} %)",
        s.frames,
        s.frames_bad,
        s.ldpc_fail,
        s.crc_fail,
        s.packets,
        s.esn0_db,
        s.data_esn0_db,
        s.freq_hz,
        iq.len() as f64 / fs / secs,
        100.0 * s.ldpc_s / (s.ldpc_s + s.other_s).max(1e-9)
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbs2::{Modulator, Rate};

    /// Modulate numbered packets, add a carrier offset and noise, receive:
    /// what comes out (after acquisition) is exactly what went in.
    #[test]
    fn receives_what_the_modulator_sends() {
        let p = Params { rate: Rate::R1_2, pilots: true, rolloff: 0.35 };
        let (fs, rs, center) = (384_000.0, 64_000.0, 40_000.0);
        let mut m = Modulator::new(p, 6);
        let mut n = 0u32;
        let mut next = || {
            let mut pkt = [0u8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1..5].copy_from_slice(&n.to_be_bytes());
            for (i, b) in pkt[5..].iter_mut().enumerate() {
                *b = (n as usize * 7 + i) as u8;
            }
            n += 1;
            pkt
        };
        let frames = ACQ_FRAMES as usize + 12;
        let mut iq = vec![Complex32::default(); frames * p.frame_symbols() * 6];
        m.fill(&mut iq, &mut next);
        // Es/N0 4 dB, 150 Hz off the nominal centre.
        let esn0 = 10f32.powf(4.0 / 10.0);
        let sigma = (fs as f32 / (rs as f32 * esn0) / 2.0).sqrt();
        let mut seed = 5u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        for (k, z) in iq.iter_mut().enumerate() {
            let ph = std::f64::consts::TAU * (center + 150.0) * k as f64 / fs;
            *z = *z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g());
        }
        let mut rx = Receiver::new(p, fs, rs, center);
        let mut out = Vec::new();
        for c in iq.chunks(3840) {
            rx.process(c, &mut out);
        }
        let s = rx.stats;
        assert!(s.locked && (s.freq_hz - 150.0).abs() < 5.0, "{s:?}");
        let data: Vec<_> = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).collect();
        assert!(data.len() > 30, "{} packets, {s:?}", data.len());
        // Consecutive, and each exactly as sent.
        let first = u32::from_be_bytes(data[0][1..5].try_into().unwrap());
        for (i, pkt) in data.iter().enumerate() {
            let k = first + i as u32;
            assert_eq!(u32::from_be_bytes(pkt[1..5].try_into().unwrap()), k, "packet {i}");
            assert!(pkt[5..].iter().enumerate().all(|(j, &b)| b == (k as usize * 7 + j) as u8));
        }
    }
}
