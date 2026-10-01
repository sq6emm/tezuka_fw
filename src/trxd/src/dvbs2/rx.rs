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

use super::{BBHEADER, FrameSpec, PILOT, PSK8_PHASE, Params, SLOT, TS_LEN, bb_scrambling, crc8, fpga_ldpc::Ldpc, pl_scrambling, rrc_taps};
use super::bch::{Bch, Outcome};

/// Frames `/tmp/datv-dump` captures at most (about 65 kB each).
const DUMP_FRAMES: usize = 300;
/// Header correlation (0..1) that counts as a PLHEADER.
const SYNC_MIN: f32 = 0.5;
/// Frames in a row whose header is missing before searching again.
const LOST_AFTER: u32 = 4;
/// Symbols either side of the predicted header position searched when locked.
const TRACK_SLACK: usize = 3;
/// Frames whose header lag correlations are summed before the carrier
/// estimate is trusted (about 1/sqrt(N) of the single-frame noise).
const ACQ_FRAMES: u32 = 16;
/// Known-block Es/N0 (dB) that ends acquisition early (after 3 frames, two
/// of them this clean).
const ACQ_CLEAN_DB: f32 = 10.0;
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
    /// Long frames: bits BCH corrected; frames LDPC converged on that BCH
    /// rejected (counted in `ldpc_fail` too).
    pub bch_fixed: u64,
    pub bch_fail: u64,
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
    spec: FrameSpec,
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
    /// Tracking: how far the carrier moves a frame (Hz), learned.
    drift_hz: f64,
    /// Frames since lock, for the frequency acquisition.
    frames_locked: u32,
    /// Frames this acquisition takes ([`ACQ_FRAMES`], fewer for a strong
    /// signal), and acquisition frames in a row whose known blocks came out
    /// clean.
    acq_frames: u32,
    acq_good: u32,
    /// Header lag correlations summed over the acquisition frames.
    acq: [Complex32; 3],
    /// Frames in a row that failed to decode while tracking.
    fails: u32,
    /// Frames in a row with an Es/N0 of noise (false lock).
    noise_frames: u32,
    // FEC: inline (tests, CLI) or on a thread of its own (trxd).
    fec: FecMode,
    llr: Vec<f32>,
    pub stats: Stats,
    /// Constellation of the last frame: about 256 corrected, descrambled data
    /// symbols, scaled so the ideal QPSK points are (+-40, +-40).
    pub constellation: Vec<[i8; 2]>,
    /// Frames the constellation has been taken from (to see a new one).
    pub constellation_seq: u64,
    /// Symbols in (see [`Self::new_symbols_spec`]).
    symbol_input: bool,
    /// Header candidates from the FPGA ([`super::hdrdet`]), one a symbol
    /// in step with `syms` (only when `flagged`): the search looks there only.
    flags: Vec<bool>,
    flagged: bool,
}

/// LDPC + BBFRAME -> TS: the expensive part, which can run on its own core.
pub struct Fec {
    spec: FrameSpec,
    dec: Ldpc,
    bch: Option<Bch>,
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

/// Time in BCH and in deframing (ns), for the FEC thread's log.
pub static FEC_PROF_NS: [std::sync::atomic::AtomicU64; 2] = [const { std::sync::atomic::AtomicU64::new(0) }; 2];

impl Fec {
    pub fn new(spec: FrameSpec) -> Self {
        let bch = Some(if spec.is_short() { Bch::short() } else { Bch::new() });
        Fec { spec, dec: Ldpc::for_spec(&spec), bch, bbscr: bb_scrambling(spec.kbch / 8), bits: vec![0; spec.n], partial: Vec::new(), have_prev: false }
    }

    /// Decode one frame of LLRs; TS packets out. False if it did not decode.
    pub fn frame(&mut self, llr: &[f32], stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode(llr, &mut self.bits);
        self.after_decode(ok, t0, stats, out)
    }

    /// [`Self::frame`] from LLRs already in the LDPC decoder's 6 bits
    /// ([`super::fpga_ldpc::Ldpc::decode_q`]).
    pub fn frame_q(&mut self, llr: &[i8], stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode_q(llr, &mut self.bits);
        self.after_decode(ok, t0, stats, out)
    }

    /// A DVB-T2 QPSK block in DDR (the FPGA's cell router put it there).
    pub fn frame_ddr(&mut self, addr: u32, p: &crate::dvbt2::stream::CellParams, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode_ddr(addr, p, &mut self.bits);
        self.after_decode(ok, t0, stats, out)
    }

    /// A DVB-T2 QPSK block as cells (the FPGA decoder makes the LLRs).
    pub fn frame_cells(&mut self, cells: &[[i8; 2]], p: &crate::dvbt2::stream::CellParams, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode_cells(cells, p, &mut self.bits);
        self.after_decode(ok, t0, stats, out)
    }

    fn after_decode(&mut self, ok: Option<usize>, t0: std::time::Instant, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t_dec = std::time::Instant::now();
        // BCH cleans up what LDPC left (or falsely converged on); it also
        // rescues a nearly converged frame.
        let ok = match &self.bch {
            None => ok.is_some(),
            Some(bch) => match bch.decode(&mut self.bits[..self.spec.kbch + bch.parity()]) {
                Outcome::Clean => true,
                Outcome::Fixed(k) => {
                    stats.bch_fixed += k as u64;
                    true
                }
                Outcome::Failed => {
                    if ok.is_some() {
                        stats.bch_fail += 1;
                    }
                    false
                }
            },
        };
        stats.ldpc_s += t0.elapsed().as_secs_f64();
        use std::sync::atomic::Ordering::Relaxed;
        FEC_PROF_NS[0].fetch_add(t_dec.elapsed().as_nanos() as u64, Relaxed);
        if !ok {
            stats.frames_bad += 1;
            stats.ldpc_fail += 1;
            self.have_prev = false;
            return false;
        }
        let t_df = std::time::Instant::now();
        let r = self.deframe(stats, out);
        FEC_PROF_NS[1].fetch_add(t_df.elapsed().as_nanos() as u64, Relaxed);
        r
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
        Self::with_spec(FrameSpec::short(p), fs, rs, center_hz)
    }

    /// Any frame type (short QPSK, or normal QPSK / 8PSK).
    pub fn with_spec(spec: FrameSpec, fs: f64, rs: f64, center_hz: f64) -> Self {
        let sps_in = (fs / rs).round() as usize;
        // Decimate to 2 or 3 samples per symbol inside the matched filter.
        let decim = (1..=sps_in).rev().find(|d| sps_in % d == 0 && sps_in / d >= 2).unwrap_or(1);
        let sps = sps_in / decim;
        let rrc = rrc_taps(sps_in, spec.rolloff, 12);
        let frame_len = spec.frame_symbols();
        let n_data = frame_len - SLOT;
        // Known blocks: the header, then each pilot block (after 16 slots).
        let mut known = vec![(0usize, SLOT)];
        if spec.pilots {
            let slots = spec.slots();
            for i in 1..=(slots - 1) / 16 {
                let at = SLOT + i * 16 * SLOT + (i - 1) * PILOT;
                known.push((at, PILOT));
            }
        }
        Receiver {
            spec,
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
            header: spec.header(),
            frame_len,
            known,
            scramble: pl_scrambling(n_data),
            locked_at: None,
            candidate: None,
            missed: 0,
            freq_est: 0.0,
            drift_hz: 0.0,
            frames_locked: 0,
            acq_frames: ACQ_FRAMES,
            acq_good: 0,
            acq: [Complex32::default(); 3],
            fails: 0,
            noise_frames: 0,
            fec: FecMode::Inline(Fec::new(spec)),
            llr: vec![0.0; spec.n],
            stats: Stats::default(),
            constellation: Vec::new(),
            constellation_seq: 0,
            symbol_input: false,
            flags: Vec::new(),
            flagged: false,
        }
    }

    /// Input one sample a symbol, timing already recovered (the FPGA's
    /// [`super::symsync`]): only the AFC's mixer and the AGC run before
    /// frame sync.
    pub fn new_symbols_spec(spec: FrameSpec, rs: f64, center_hz: f64) -> Self {
        let mut r = Receiver::with_spec(spec, rs, rs, center_hz);
        r.symbol_input = true;
        r
    }

    /// Input already matched-filtered and decimated (the FPGA DDC, see
    /// [`super::ddc`]): `fs` may be any rate of about 2 samples per symbol
    /// or more, not necessarily a whole multiple of `rs`. Only the AFC's
    /// mixer runs here.
    pub fn new_prefiltered(p: Params, fs: f64, rs: f64, center_hz: f64) -> Self {
        Self::new_prefiltered_spec(FrameSpec::short(p), fs, rs, center_hz)
    }

    pub fn new_prefiltered_spec(spec: FrameSpec, fs: f64, rs: f64, center_hz: f64) -> Self {
        let mut r = Receiver::with_spec(spec, fs, rs, center_hz);
        r.rrc = vec![1.0];
        r.hist = vec![Complex32::default(); 2];
        r.hpos = 0;
        r.decim = 1;
        r.omega = fs / rs;
        r.omega_nom = fs / rs;
        r
    }

    /// The signal moved in the input (the LO was retuned). The carrier error
    /// the AFC found is the transmitter's, so it stays.
    pub fn set_center(&mut self, hz: f64) {
        self.center_hz = hz;
    }

    /// Feed input samples; whole TS packets come out as frames complete.
    pub fn process(&mut self, iq: &[Complex32], out: &mut Vec<[u8; TS_LEN]>) {
        self.process_flagged(iq, None, out);
    }

    /// Symbols (see [`Self::new_symbols_spec`]) with the FPGA's header
    /// candidate flags, one per symbol.
    pub fn process_flagged(&mut self, iq: &[Complex32], flags: Option<&[bool]>, out: &mut Vec<[u8; TS_LEN]>) {
        let t0 = std::time::Instant::now();
        let ldpc0 = self.stats.ldpc_s;
        self.process_inner(iq, flags, out);
        self.stats.other_s += t0.elapsed().as_secs_f64() - (self.stats.ldpc_s - ldpc0);
    }

    fn process_inner(&mut self, iq: &[Complex32], flags: Option<&[bool]>, out: &mut Vec<[u8; TS_LEN]>) {
        let w = -std::f64::consts::TAU * (self.center_hz + self.afc_hz) / self.fs;
        let step = Complex32::new(w.cos() as f32, w.sin() as f32);
        if self.symbol_input {
            self.flagged = flags.is_some();
            if let Some(f) = flags {
                self.flags.extend_from_slice(f);
            }
            for &x in iq {
                let y = x * self.nco;
                self.nco *= step;
                self.nco_n += 1;
                if self.nco_n == 1024 {
                    self.nco_n = 0;
                    self.nco /= self.nco.norm();
                }
                self.agc += 0.001 * (y.norm_sqr().max(1e-20) - self.agc);
                self.syms.push(y / self.agc.sqrt());
            }
            self.frames(out);
            return;
        }
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

    /// The best header position in `start..end`: [`Self::header_metric`],
    /// computed only where the SOF alone (its first 30 symbols, the same
    /// chunked metric) passes a loose threshold. Unlocked, this runs for
    /// every symbol: at 250 kS/s the full metric everywhere cost the A9 more
    /// than a core, the receiver fell behind the ring and never locked.
    /// Drop the first `n` symbols (and their flags).
    fn drain_syms(&mut self, n: usize) {
        let n = n.min(self.syms.len());
        self.syms.drain(..n);
        if self.flagged {
            let k = n.min(self.flags.len());
            self.flags.drain(..k);
        }
    }

    fn search(&self, start: usize, end: usize) -> (usize, f32) {
        if self.flagged {
            let mut best = (start, 0f32);
            let last = super::hdrdet::SOF_LEN - 1;
            for k in start..end {
                if self.flags.get(k + last) == Some(&true) {
                    let m = self.header_metric(k);
                    if m > best.1 {
                        best = (k, m);
                    }
                }
            }
            return best;
        }
        const SOF: usize = 30;
        // Noise gives about 0.28 +- 0.08 here, a header 0.7 at 0 dB Es/N0.
        const SOF_MIN: f32 = 0.4;
        // |s| prefix sums for the normalization.
        let mut cum = Vec::with_capacity(end + SOF - start + 1);
        cum.push(0f32);
        for z in &self.syms[start..end + SOF] {
            cum.push(cum.last().unwrap() + z.norm());
        }
        let mut best = (start, 0f32);
        for k in start..end {
            let s = &self.syms[k..k + SOF];
            let mut num = 0f32;
            for c in 0..SOF / 10 {
                let mut acc = Complex32::default();
                for i in c * 10..c * 10 + 10 {
                    acc += s[i] * self.header[i].conj();
                }
                num += acc.norm();
            }
            let den = cum[k - start + SOF] - cum[k - start];
            if num < SOF_MIN * den {
                continue;
            }
            let m = self.header_metric(k);
            if m > best.1 {
                best = (k, m);
            }
        }
        best
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
                    let (best, m) = self.search(start, end);
                    if m >= SYNC_MIN {
                        if let Some(first) = self.candidate.and(best.checked_sub(l)) {
                            // Two headers one frame apart: locked.
                            self.locked_at = Some(first);
                            self.missed = 0;
                            self.frames_locked = 0;
                            self.acq = [Complex32::default(); 3];
                            self.acq_frames = ACQ_FRAMES;
                            self.acq_good = 0;
                            self.stats.locked = true;
                        } else {
                            // (Or the second header came early, symbols lost
                            // between, and a frame back from it lies before
                            // the buffer: a candidate in the first
                            // TRACK_SLACK symbols. That wrapped, and the
                            // receiver thread died indexing at usize::MAX
                            // (seen on 2 m, 8PSK 500 kS/s in interference).
                            // It starts over from the second.)
                            self.candidate = Some(best);
                        }
                    } else {
                        self.candidate = None;
                        let drop = end.saturating_sub(SLOT);
                        self.drain_syms(drop);
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
                        self.drain_syms(next);
                        continue;
                    }
                    // Keep the next frame at the front.
                    self.drain_syms(next);
                    self.locked_at = Some(0);
                }
            }
        }
    }

    /// Phase of a known block (header at `at` with the reference, or a
    /// pilot) at its centre, `f` Hz (the frequency so far) taken out inside
    /// it. Averaged as it comes, a block turning through more than a cycle
    /// (the 90-symbol header from rs / 90 on: 367 Hz at 33 kS/s, which
    /// acquisition sees with the NCO held) comes out with its phase flipped,
    /// and the frame's fit an alias (rs / 1476) off for good.
    fn block(&self, base: usize, at: usize, len: usize, f: f64) -> Complex32 {
        let mut acc = Complex32::default();
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let w = -std::f64::consts::TAU * f / self.rs;
        let mid = (len - 1) as f64 / 2.0;
        for i in 0..len {
            let ph = w * (i as f64 - mid);
            let s = self.syms[base + at + i] * Complex32::new(ph.cos() as f32, ph.sin() as f32);
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

    /// Move the NCO by `df` Hz as of symbol `from` (the next header). The
    /// symbols after it were already mixed with the old frequency (up to an
    /// input block's worth): give them the new one too, and turn the NCO's
    /// phase so the samples still to come continue them without a step.
    fn retune(&mut self, df: f64, from: usize) {
        self.afc_hz += df;
        let w = -std::f64::consts::TAU * df / self.rs;
        for (i, s) in self.syms.iter_mut().enumerate().skip(from) {
            let ph = w * (i - from) as f64;
            *s *= Complex32::new(ph.cos() as f32, ph.sin() as f32);
        }
        // Symbol `from` to the next input sample: the buffered symbols, the
        // matched-filter samples not yet strobed, the filter's delay.
        let mut t = self.syms.len().saturating_sub(from) as f64 / self.rs;
        if !self.symbol_input {
            t += (self.mf.len() as f64 - self.t).max(0.0) * self.decim as f64 / self.fs + (self.rrc.len() / 2) as f64 / self.fs;
        }
        let ph = -std::f64::consts::TAU * df * t;
        self.nco *= Complex32::new(ph.cos() as f32, ph.sin() as f32);
    }

    fn frame(&mut self, p: usize, next: usize, out: &mut Vec<[u8; TS_LEN]>) {
        let rs = self.rs;
        // Header: frequency at increasing lags (+-rs/2, then finer).
        let z: Vec<Complex32> = (0..SLOT).map(|i| self.syms[p + i] * self.header[i].conj()).collect();
        // Acquisition: the header's lag correlations, summed over ACQ_FRAMES
        // with the NCO held, then resolved coarse to fine (lag 1: +-rs/2, lag
        // 40: +-rs/80) and applied once. One frame alone is too noisy (+-15 Hz
        // at 8 dB) to unwrap phase across the 1476 symbols between pilots.
        let acquiring = self.frames_locked < self.acq_frames;
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
            let c = self.block(p, at, len, f);
            pts.push(((at + len / 2) as f64, c.arg() as f64, c.norm() * len as f32));
        }
        if with_next {
            let c = self.block(next, 0, SLOT, f);
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
            // A strong signal need not wait for the average: once the known
            // blocks came out clean on two frames, this frame's own fit is
            // right. Waiting 16 frames (11 s at 33 kS/s 8PSK) let a drifting
            // carrier (a transmitter just keyed, a few Hz/s at 3.4 GHz) leave
            // the average an alias behind, again after every restart.
            if self.frames_locked >= self.acq_frames || (self.frames_locked >= 3 && self.acq_good >= 2) {
                // Acquired: the NCO takes the estimate in one step.
                self.retune(f, next);
                self.freq_est = 0.0;
                self.drift_hz = 0.0;
                self.acq_frames = self.frames_locked;
            }
        } else {
            // Tracking: move the NCO slowly toward the estimate; what remains
            // is followed inside each frame by the phases above. Second
            // order: it also moves by the drift learned from how far each
            // frame's estimate missed the prediction (a transmitter just
            // keyed moves a few Hz/s; at 33 kS/s a frame lasts 0.7-1 s and a
            // pilot alias is only 22 Hz away).
            let miss = f - self.freq_est;
            self.drift_hz = (self.drift_hz + 0.2 * miss).clamp(-alias / 4.0, alias / 4.0);
            self.freq_est = 0.8 * f;
            self.retune(0.2 * f + self.drift_hz, next);
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
        if acquiring {
            self.acq_good = if self.stats.esn0_db >= ACQ_CLEAN_DB { self.acq_good + 1 } else { 0 };
        }
        // Data symbols: derotate, descramble, LLRs (positive = 0).
        let scale = 2.0 * std::f32::consts::SQRT_2 * amp / sigma2 * a * std::f32::consts::SQRT_2;
        let (mut dd_sig, mut dd_err) = (0f32, 0f32);
        self.constellation.clear();
        // Constellation display: QPSK points at (+-40, +-40); 8PSK on the
        // same radius (about 57).
        let unit = 40.0 / (amp * a).max(1e-9);
        let mut n = 0usize; // data symbol index
        let mut k = SLOT; // position in the frame
        let mut pilot = 1usize;
        let nsym = self.spec.n / self.spec.bps;
        // 8PSK points (unit radius) for the soft demapper
        let psk8: [Complex32; 8] = std::array::from_fn(|v| {
            let a = std::f32::consts::FRAC_PI_4 * PSK8_PHASE[v] as f32;
            Complex32::new(a.cos(), a.sin())
        });
        while n < nsym {
            if pilot < self.known.len() && k == self.known[pilot].0 {
                k += PILOT;
                pilot += 1;
                continue;
            }
            let ph = phase_at(k as f64) as f32;
            let s = self.syms[p + k] * Complex32::new(ph.cos(), -ph.sin());
            let d = super::rotate(s, (4 - self.scramble[k - SLOT]) & 3);
            let dec = if self.spec.bps == 2 {
                self.llr[2 * n] = scale * d.re;
                self.llr[2 * n + 1] = scale * d.im;
                Complex32::new(amp * a * d.re.signum(), amp * a * d.im.signum())
            } else {
                // 8PSK, max-log: per bit the nearest point with it 1 minus
                // the nearest with it 0, over the noise; bits y0 y1 y2 go to
                // the columns of the bit interleaver (rate 3/4: 0, 1, 2).
                let dist: [f32; 8] = std::array::from_fn(|v| (d - psk8[v] * amp).norm_sqr());
                let rows = self.spec.n / 3;
                for bit in 0..3 {
                    let mask = 4 >> bit;
                    let (mut d0, mut d1) = (f32::MAX, f32::MAX);
                    for (v, &dv) in dist.iter().enumerate() {
                        if v & mask == 0 {
                            d0 = d0.min(dv);
                        } else {
                            d1 = d1.min(dv);
                        }
                    }
                    self.llr[bit * rows + n] = (d1 - d0) / sigma2;
                }
                let best = (0..8).min_by(|&x, &y| dist[x].total_cmp(&dist[y])).unwrap();
                psk8[best] * amp
            };
            dd_sig += dec.norm_sqr();
            dd_err += (d - dec).norm_sqr();
            if n % 32 == 0 {
                let q = |v: f32| (v * unit).round().clamp(-127.0, 127.0) as i8;
                self.constellation.push([q(d.re), q(d.im)]);
            }
            n += 1;
            k += 1;
        }
        self.stats.data_esn0_db = 10.0 * (dd_sig / dd_err.max(1e-9)).log10();
        self.constellation_seq += 1;
        self.stats.frames += 1;
        // Far below what this rate decodes (no signal, carrier wrong): the
        // decoder would only burn CPU. It counts as a failure.
        let hopeless = self.stats.esn0_db < self.spec.hopeless_db;
        // Far below anything decodable, twice: the "lock" was a chance
        // header match on noise. Let go (the caller sees `missed`).
        if self.stats.esn0_db < self.spec.hopeless_db - 6.0 {
            self.noise_frames += 1;
            if self.noise_frames >= 2 {
                self.missed = LOST_AFTER;
            }
        } else {
            self.noise_frames = 0;
        }
        let fails = match &mut self.fec {
            FecMode::Inline(f) => {
                // Debug: DVBS2_DUMP_LLR=<file> appends each frame's LLRs (f32 LE).
                if let Some(path) = std::env::var_os("DVBS2_DUMP_LLR") {
                    use std::io::Write;
                    if let Ok(mut fh) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                        let b: Vec<u8> = self.llr.iter().flat_map(|v| v.to_le_bytes()).collect();
                        let _ = fh.write_all(&b);
                    }
                }
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
        if self.fails >= 3 && self.frames_locked >= self.acq_frames {
            // Carrier wrong, or gone: acquire again from where the AFC is.
            self.frames_locked = 0;
            self.acq = [Complex32::default(); 3];
            self.acq_frames = ACQ_FRAMES;
            self.acq_good = 0;
            self.freq_est = 0.0;
            self.drift_hz = 0.0;
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
        let n = self.spec.kbch / 8;
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
    /// DVB service information read so far (SDT, NIT, EIT p/f, TDT).
    pub si: super::ts::Si,
}

/// [`Receiver`] and [`super::ts::Demux`] on their own thread, off the
/// engine's sample path: it gets copies of the stream-rate IQ and drops
/// blocks (never stalls the engine) if the CPU cannot keep up.
pub struct RxThread {
    tx: crossbeam_channel::Sender<(Vec<Complex32>, f64)>,
    shared: std::sync::Arc<std::sync::Mutex<RxShared>>,
    pub spec: FrameSpec,
    /// The mode as the UI names it (e.g. "1/2", "L-8PSK-3/4").
    pub label: String,
    pub sr: f64,
    fec_stats: std::sync::Arc<std::sync::Mutex<Stats>>,
    /// Blocks dropped because the thread was behind (CPU too slow).
    dropped: std::sync::Arc<std::sync::atomic::AtomicU64>,
    started: std::time::Instant,
    /// Receiving through the FPGA DDC: the signal's offset from the LO
    /// (f64 bits) for the DDC's NCO; the stream IQ is not used.
    fpga_center: Option<std::sync::Arc<std::sync::atomic::AtomicU64>>,
    /// DVB-T2: the channel bandwidth (Hz), wider than the RX filter's
    /// default.
    pub t2_bw: Option<f64>,
}

/// The receiver's statistics and constellation out to the shared state.
fn publish(r: &Receiver, sh: &std::sync::Mutex<RxShared>, seen: &mut u64) {
    let mut s = sh.lock().unwrap();
    s.stats = r.stats;
    // The constellation, for the browser: [8][re, im as i8]...
    if r.constellation_seq != *seen {
        *seen = r.constellation_seq;
        let mut m = Vec::with_capacity(1 + 2 * r.constellation.len());
        m.push(8u8);
        m.extend(r.constellation.iter().flat_map(|p| [p[0] as u8, p[1] as u8]));
        s.msgs.push_back(m);
    }
}

/// The decoding thread: LLR vectors (64800) in, LDPC (the FPGA's when
/// there), BCH, BBFRAME, TS, the demultiplexer's browser messages out.
/// A frame's LLRs for the FEC thread: floats (DVB-S2) or already the LDPC
/// decoder's 6 bits (DVB-T2); empty: a frame lost.
pub trait FecBlock: Send + 'static {
    fn is_empty(&self) -> bool;
    fn decode(&self, fec: &mut Fec, st: &mut Stats, ts: &mut Vec<[u8; TS_LEN]>) -> bool;
    /// As f32 LE (the /tmp/datv-dump debug file).
    fn dump(&self) -> Vec<u8>;
}

impl FecBlock for Vec<f32> {
    fn is_empty(&self) -> bool {
        self.as_slice().is_empty()
    }
    fn decode(&self, fec: &mut Fec, st: &mut Stats, ts: &mut Vec<[u8; TS_LEN]>) -> bool {
        fec.frame(self, st, ts)
    }
    fn dump(&self) -> Vec<u8> {
        self.iter().flat_map(|v| v.to_le_bytes()).collect()
    }
}

impl FecBlock for crate::dvbt2::stream::T2Block {
    fn is_empty(&self) -> bool {
        match self {
            crate::dvbt2::stream::T2Block::Llr(v) => v.is_empty(),
            crate::dvbt2::stream::T2Block::Cells(c, _) => c.is_empty(),
            crate::dvbt2::stream::T2Block::Ddr(..) => false,
        }
    }
    fn decode(&self, fec: &mut Fec, st: &mut Stats, ts: &mut Vec<[u8; TS_LEN]>) -> bool {
        match self {
            crate::dvbt2::stream::T2Block::Llr(v) => fec.frame_q(v, st, ts),
            crate::dvbt2::stream::T2Block::Cells(c, p) => fec.frame_cells(c, p, st, ts),
            crate::dvbt2::stream::T2Block::Ddr(a, p) => fec.frame_ddr(*a, p, st, ts),
        }
    }
    fn dump(&self) -> Vec<u8> {
        self.llrs().iter().flat_map(|&v| (v as f32 / super::fpga_ldpc::LLR_SCALE).to_le_bytes()).collect()
    }
}

impl FecBlock for Vec<i8> {
    fn is_empty(&self) -> bool {
        self.as_slice().is_empty()
    }
    fn decode(&self, fec: &mut Fec, st: &mut Stats, ts: &mut Vec<[u8; TS_LEN]>) -> bool {
        fec.frame_q(self, st, ts)
    }
    fn dump(&self) -> Vec<u8> {
        let s = 1.0 / super::fpga_ldpc::LLR_SCALE;
        self.iter().flat_map(|&v| (v as f32 * s).to_le_bytes()).collect()
    }
}

fn spawn_fec<B: FecBlock>(
    p: FrameSpec,
    frx: crossbeam_channel::Receiver<B>,
    fl: std::sync::Arc<std::sync::atomic::AtomicU32>,
    sh: std::sync::Arc<std::sync::Mutex<RxShared>>,
    fs2: std::sync::Arc<std::sync::Mutex<Stats>>,
) {
    use std::sync::atomic::Ordering;
    std::thread::Builder::new()
        .name("datv-fec".into())
        .spawn(move || {
            // Ahead of the web server and scopes, behind the engine (-10).
            crate::stream::thread_nice(-5);
            let mut fec = Fec::new(p);
            let mut dmx = super::ts::Demux::default();
            let (mut ts, mut msgs) = (Vec::new(), Vec::new());
            let mut st = Stats::default();
            let mut dumped = 0usize;
            let (mut nblk, mut dmx_ns, mut t_prof) = (0u64, 0u64, std::time::Instant::now());
            for llr in frx.iter() {
                if llr.is_empty() {
                    fec.lost();
                    continue;
                }
                // Near the threshold every frame runs long, and the ones
                // that will fail run longest: with frames waiting, give
                // each fewer iterations rather than drop the next ones
                // unread. 20 costs nothing measurable at 1/2 (ldpc.rs
                // iteration_budget), 12 a few frames at the very edge.
                fec.dec.set_max_iter(match frx.len() {
                    0..=1 => 50,
                    2..=7 => 20,
                    8..=19 => 12,
                    _ => 8,
                });
                // Debug on a board: `touch /tmp/datv-dump` appends each
                // frame's LLRs (f32 LE, 16200 a frame) to
                // /tmp/datv-llr.f32, up to DUMP_FRAMES; remove it to stop.
                if dumped < DUMP_FRAMES && std::path::Path::new("/tmp/datv-dump").exists() {
                    use std::io::Write;
                    if let Ok(mut fh) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/datv-llr.f32") {
                        let b: Vec<u8> = llr.dump();
                        let _ = fh.write_all(&b);
                        dumped += 1;
                    }
                }
                if llr.decode(&mut fec, &mut st, &mut ts) {
                    fl.store(0, Ordering::Relaxed);
                } else {
                    fl.fetch_add(1, Ordering::Relaxed);
                }
                let t_dmx = std::time::Instant::now();
                for pkt in ts.drain(..) {
                    dmx.push(&pkt, &mut msgs);
                }
                dmx_ns += t_dmx.elapsed().as_nanos() as u64;
                nblk += 1;
                if t_prof.elapsed() > std::time::Duration::from_secs(30) {
                    use super::fpga_ldpc::PROF_NS;
                    let per = nblk as f64;
                    let ms = |a: &std::sync::atomic::AtomicU64| a.swap(0, Ordering::Relaxed) as f64 / per / 1e6;
                    tracing::info!(
                        llr_in_ms = format!("{:.2}", ms(&PROF_NS[0])),
                        fpga_ms = format!("{:.2}", ms(&PROF_NS[1])),
                        bits_out_ms = format!("{:.2}", ms(&PROF_NS[2])),
                        bch_ms = format!("{:.2}", ms(&FEC_PROF_NS[0])),
                        deframe_ms = format!("{:.2}", ms(&FEC_PROF_NS[1])),
                        demux_ms = format!("{:.2}", dmx_ns as f64 / per / 1e6),
                        blocks = nblk,
                        "DATV FEC per block"
                    );
                    dmx_ns = 0;
                    nblk = 0;
                    t_prof = std::time::Instant::now();
                }
                *fs2.lock().unwrap() = st;
                let mut s = sh.lock().unwrap();
                s.si = dmx.si.clone();
                s.msgs.extend(msgs.drain(..));
                let excess = s.msgs.len().saturating_sub(300);
                s.msgs.drain(..excess);
            }
        })
        .expect("spawn datv-fec");
}

fn label_log(m: &crate::dvbt2::tx::Mode) -> String {
    format!("{:.2} MHz {:?} {:?}", m.bw_hz / 1e6, m.p.constellation, m.p.rate)
}

impl RxThread {
    pub fn start(p: Params, fs: f64, sr: f64, center_hz: f64) -> Self {
        Self::start_spec(FrameSpec::short(p), p.rate.label().to_string(), fs, sr, center_hz)
    }

    pub fn start_spec(p: FrameSpec, label: String, fs: f64, sr: f64, center_hz: f64) -> Self {
        use std::sync::{Arc, Mutex, atomic::AtomicU32, atomic::Ordering};
        let (tx, rx) = crossbeam_channel::bounded::<(Vec<Complex32>, f64)>(64);
        // Slack between demodulator and decoder: about 6 s at 64 kS/s. The
        // decoder needs ~30 % of a core on average, but on the busy A9 it
        // gets none for a while now and then; 16 frames overflowed over the
        // air (a quarter of the frames lost as "busy" that would all have
        // decoded). Beyond it whole frames are skipped and timing stays intact.
        let (ftx, frx) = crossbeam_channel::bounded::<Vec<f32>>(48);
        let fails = Arc::new(AtomicU32::new(0));
        let shared = Arc::new(Mutex::new(RxShared::default()));
        let fec_stats = Arc::new(Mutex::new(Stats::default()));
        spawn_fec(p, frx, fails.clone(), shared.clone(), fec_stats.clone());
        let sh = shared.clone();
        let dropped = Arc::new(std::sync::atomic::AtomicU64::new(0));
        // The FPGA front end when the bitstream has it and the rate suits it:
        // the DDC hands over matched-filtered samples at 2 per symbol.
        let fpga = super::fpga::available() && super::ddc::symbol_rate_ok(super::fpga::FS_IN, sr);
        let fpga_center = fpga.then(|| Arc::new(std::sync::atomic::AtomicU64::new(center_hz.to_bits())));
        let (fc, dr) = (fpga_center.clone(), dropped.clone());
        std::thread::Builder::new()
            .name("datv-rx".into())
            .spawn(move || {
                crate::stream::thread_nice(-5);
                let mut none = Vec::new();
                let mut seen = 0;
                let Some(fc) = fc else {
                    let mut r = Receiver::with_spec(p, fs, sr, center_hz);
                    r.fec = FecMode::Thread { tx: ftx, fails };
                    for (iq, center) in rx {
                        r.set_center(center);
                        r.process(&iq, &mut none);
                        publish(&r, &sh, &mut seen);
                    }
                    return;
                };
                // The ring holds 0.5 s: a reader of its own drains it every
                // few ms into a queue of seconds, so the demodulator's bursts
                // (a whole long frame at once) cannot let the DMA lap it.
                let (btx, brx) = crossbeam_channel::bounded::<(Vec<Complex32>, Option<Vec<bool>>)>(800);
                let (ftx_fs, frx_fs) = crossbeam_channel::bounded::<(f64, bool, bool)>(1);
                let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
                let (st2, dr2) = (stop.clone(), dr.clone());
                let rolloff = p.rolloff;
                let ring = std::thread::Builder::new()
                    .name("datv-ring".into())
                    .spawn(move || {
                        crate::stream::thread_nice(-5);
                        let mut fe = match super::fpga::FrontEnd::start(sr, rolloff, center_hz) {
                            Ok(fe) => fe,
                            Err(e) => {
                                tracing::warn!("DATV: FPGA front end: {e}");
                                return;
                            }
                        };
                        let _ = ftx_fs.send((fe.fs_out(), fe.symbols(), fe.flagged()));
                        let (mut reported, mut late_reported) = (false, false);
                        let mut last = std::time::Instant::now();
                        while !st2.load(Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(5));
                            fe.set_center(f64::from_bits(fc.load(Ordering::Relaxed)));
                            let mut buf = Vec::new();
                            let mut flags = fe.flagged().then(Vec::new);
                            match flags.as_mut() {
                                Some(f) => fe.read_flagged(&mut buf, f),
                                None => fe.read(&mut buf),
                            }
                            if !late_reported && last.elapsed() > std::time::Duration::from_millis(400) {
                                late_reported = true;
                                dr2.fetch_add(1, Ordering::Relaxed);
                                tracing::warn!(late = ?last.elapsed(), "DATV: the ring reader was late: samples lost");
                            }
                            last = std::time::Instant::now();
                            // Debug on a board: `touch /tmp/datv-iq` records the
                            // DDC's output (complex f32, 2+ samples per symbol)
                            // into /tmp/datv-iq.cf32, contiguously; the
                            // demodulator gets nothing meanwhile. 64 MB at most.
                            if std::path::Path::new("/tmp/datv-iq").exists() {
                                use std::io::Write;
                                if !buf.is_empty() {
                                    if let Ok(mut fh) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/datv-iq.cf32") {
                                        if fh.metadata().map_or(0, |m| m.len()) < 64_000_000 {
                                            let b: Vec<u8> = buf.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
                                            let _ = fh.write_all(&b);
                                        }
                                    }
                                }
                                continue;
                            }
                            if !reported && fe.dropped() {
                                reported = true;
                                dr2.fetch_add(1, Ordering::Relaxed);
                                tracing::warn!("DATV: the FPGA recorder dropped samples");
                            }
                            if !buf.is_empty() && btx.try_send((buf, flags)).is_err() {
                                // The demodulator is seconds behind.
                                dr2.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    })
                    .expect("spawn datv-ring");
                let Ok((fs_out, symbols, flagged)) = frx_fs.recv() else {
                    return;
                };
                tracing::info!(fs = fs_out, symbols, flagged, "DATV receive through the FPGA DDC");
                let mut r = if symbols { Receiver::new_symbols_spec(p, sr, 0.0) } else { Receiver::new_prefiltered_spec(p, fs_out, sr, 0.0) };
                r.fec = FecMode::Thread { tx: ftx, fails };
                'run: loop {
                    // Until the RxThread is dropped (its sender goes); the
                    // stream IQ it sends is not used here.
                    loop {
                        match rx.try_recv() {
                            Err(crossbeam_channel::TryRecvError::Disconnected) => break 'run,
                            Err(crossbeam_channel::TryRecvError::Empty) => break,
                            Ok(_) => {}
                        }
                    }
                    match brx.recv_timeout(std::time::Duration::from_millis(20)) {
                        Ok((buf, flags)) => {
                            r.process_flagged(&buf, flags.as_deref(), &mut none);
                            publish(&r, &sh, &mut seen);
                        }
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                    }
                }
                stop.store(true, Ordering::Relaxed);
                let _ = ring.join();
            })
            .expect("spawn datv-rx");
        RxThread { tx, shared, fec_stats, spec: p, label, sr, dropped, started: std::time::Instant::now(), fpga_center, t2_bw: None }
    }

    /// DVB-T2 through the FPGA's T2 resampler: [`crate::dvbt2::stream::Demod`]
    /// on the ring's samples, its FEC blocks to the same decoding thread.
    pub fn start_t2(mode: crate::dvbt2::tx::Mode, label: String, center_hz: f64) -> Self {
        use std::sync::{Arc, Mutex, atomic::AtomicU32, atomic::Ordering};
        use super::fpga_tx::LongMode;
        let spec = FrameSpec::long(match mode.p.rate {
            super::ldpc_fpga::LongRate::R1_2 => LongMode::Qpsk12,
            super::ldpc_fpga::LongRate::R3_4 => LongMode::Qpsk34,
        });
        let (tx, rx) = crossbeam_channel::bounded::<(Vec<Complex32>, f64)>(1);
        // Two frames of FEC blocks (18 a frame at 16QAM).
        let (ftx, frx) = crossbeam_channel::bounded::<crate::dvbt2::stream::T2Block>(40);
        let fails = Arc::new(AtomicU32::new(0));
        let shared = Arc::new(Mutex::new(RxShared::default()));
        let fec_stats = Arc::new(Mutex::new(Stats::default()));
        spawn_fec(spec, frx, fails, shared.clone(), fec_stats.clone());
        let sh = shared.clone();
        let dropped = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let center = Arc::new(std::sync::atomic::AtomicU64::new(center_hz.to_bits()));
        let (fc, dr) = (center.clone(), dropped.clone());
        std::thread::Builder::new()
            .name("datv-rx".into())
            .spawn(move || {
                crate::stream::thread_nice(-5);
                // Samples (no FFT front end) or front-end words; commands back.
                let (btx, brx) = crossbeam_channel::bounded::<Result<Vec<Complex32>, Vec<u32>>>(800);
                let (ctx, crx) = crossbeam_channel::unbounded::<crate::dvbt2::fe::Ctl>();
                let (ftx_fs, frx_fs) = crossbeam_channel::bounded::<(f64, bool)>(1);
                let params = mode.p;
                let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
                let (st2, dr2) = (stop.clone(), dr.clone());
                let fs_nominal = mode.fs();
                let ring = std::thread::Builder::new()
                    .name("datv-ring".into())
                    .spawn(move || {
                        crate::stream::thread_nice(-5);
                        let mut fe = match super::fpga::FrontEnd::start_t2(fs_nominal, &params) {
                            Ok(fe) => fe,
                            Err(e) => {
                                tracing::warn!("DVB-T2: FPGA front end: {e}");
                                return;
                            }
                        };
                        let _ = ftx_fs.send((fe.fs_out(), fe.t2_fe()));
                        let mut reported = false;
                        // (`touch /tmp/t2-words-all` too: from the start, the search included)
                        let mut rec_on = std::path::Path::new("/tmp/t2-words-all").exists();
                        let mut gap = false;
                        let (mut last_read, mut late) = (std::time::Instant::now(), 0u64);
                        let (mut read_s, mut t_log) = (0f64, std::time::Instant::now());
                        while !st2.load(Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(5));
                            for c in crx.try_iter() {
                                fe.t2_ctl(c);
                            }
                            // The ring holds about 140 ms of words: read later
                            // than 100 ms after the last time and the DMA may
                            // have lapped it (words lost, unnoticed otherwise).
                            if last_read.elapsed() > std::time::Duration::from_millis(100) {
                                gap = true;
                                late += 1;
                                if late == 1 || late % 100 == 0 {
                                    tracing::warn!(late, "DVB-T2: the ring reader was late (words lost)");
                                }
                            }
                            last_read = std::time::Instant::now();
                            let t0 = std::time::Instant::now();
                            let buf = if fe.t2_fe() {
                                let mut w = Vec::new();
                                fe.read_words(&mut w);
                                Err(w)
                            } else {
                                let mut b = Vec::new();
                                fe.read(&mut b);
                                Ok(b)
                            };
                            read_s += t0.elapsed().as_secs_f64();
                            // Debug on a board: `touch /tmp/t2-words` appends the
                            // front end's ring words (u32 LE) to /tmp/t2-words.u32
                            // once it sends carriers, 64 MB at most (the receiver
                            // still gets them).
                            if let Err(w) = &buf {
                                // (from the first carrier words on: not the search)
                                rec_on |= w.iter().any(|&v| v & 0x1_0001 == 0x1_0001);
                                if rec_on && std::path::Path::new("/tmp/t2-words").exists() {
                                    use std::io::Write;
                                    if let Ok(mut fh) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/t2-words.u32") {
                                        if fh.metadata().map_or(0, |m| m.len()) < 64_000_000 {
                                            let b: Vec<u8> = w.iter().flat_map(|v| v.to_le_bytes()).collect();
                                            let _ = fh.write_all(&b);
                                        }
                                    }
                                }
                            }
                            if t_log.elapsed() > std::time::Duration::from_secs(30) {
                                tracing::info!(ring_read_cpu = format!("{:.1} %", 100.0 * read_s / t_log.elapsed().as_secs_f64()), "DVB-T2 ring");
                                (read_s, t_log) = (0.0, std::time::Instant::now());
                            }
                            if !reported && (fe.dropped() || fe.t2_overflow()) {
                                reported = true;
                                dr2.fetch_add(1, Ordering::Relaxed);
                                tracing::warn!("DVB-T2: the FPGA recorder dropped samples (or the front end words)");
                            }
                            let mut buf = buf;
                            let empty = match &buf {
                                Ok(b) => b.is_empty(),
                                Err(w) => w.is_empty(),
                            };
                            if gap {
                                // Words were dropped before these: say so.
                                if let Err(w) = &mut buf {
                                    w.insert(0, crate::dvbt2::fe::GAP);
                                }
                            }
                            if !empty {
                                gap = btx.try_send(buf).is_err();
                                if gap {
                                    dr2.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }
                    })
                    .expect("spawn datv-ring");
                let Ok((fs, t2_fe)) = frx_fs.recv() else {
                    return;
                };
                tracing::info!(fs, t2_fe, mode = %label_log(&mode), "DVB-T2 receive through the FPGA resampler");
                let mut ctl = Vec::new();
                let mut d = crate::dvbt2::stream::Demod::new(mode.p, fs);
                // QPSK cells straight to the FPGA's LDPC decoder, which
                // makes the LLRs (LDP5 with its DDR buffers)
                d.cells_out = super::fpga_ldpc::cells_available();
                d.cells16_out = super::fpga_ldpc::cells16_available();
                let router = d.enable_router();
                tracing::info!(cells = d.cells_out, qam16 = d.cells16_out, router, "DVB-T2 LLRs in the FPGA");
                let (mut blocks, mut seen) = (Vec::new(), 0u64);
                let (mut busy_s, mut t_log) = (0f64, std::time::Instant::now());
                let mut prof_f0 = 0u64;
                'run: loop {
                    loop {
                        match rx.try_recv() {
                            Err(crossbeam_channel::TryRecvError::Disconnected) => break 'run,
                            Err(crossbeam_channel::TryRecvError::Empty) => break,
                            Ok(_) => {}
                        }
                    }
                    match brx.recv_timeout(std::time::Duration::from_millis(20)) {
                        Ok(buf) => {
                            let t0 = std::time::Instant::now();
                            d.set_center(f64::from_bits(fc.load(Ordering::Relaxed)));
                            match buf {
                                Ok(b) => d.push(&b, &mut blocks),
                                Err(w) => {
                                    d.push_words(&w, &mut blocks, &mut ctl);
                                    for c in ctl.drain(..) {
                                        let _ = ctx.send(c);
                                    }
                                }
                            }
                            let mut busy = 0;
                            for b in blocks.drain(..) {
                                if ftx.try_send(b).is_err() {
                                    busy += 1;
                                }
                            }
                            let dt = t0.elapsed().as_secs_f64();
                            busy_s += dt;
                            if t_log.elapsed() > std::time::Duration::from_secs(30) {
                                tracing::info!(demod_cpu = format!("{:.1} %", 100.0 * busy_s / t_log.elapsed().as_secs_f64()), mer_db = d.stats.mer_db, freq_hz = d.stats.freq_hz, frames = d.stats.frames, gaps = d.stats.gaps, p1_missed = d.stats.p1_missed, resched = d.stats.resched, retunes = d.stats.retunes, router = ?d.router_counters(), router_missed = d.router_missed, prof_ms = ?d.prof.map(|x| (x * 1e3 / (d.stats.frames - prof_f0).max(1) as f64 * 10.0).round() / 10.0), "DVB-T2 demodulator");
                                (busy_s, t_log) = (0.0, std::time::Instant::now());
                                d.prof = [0.0; 6];
                                prof_f0 = d.stats.frames;
                            }
                            let mut s = sh.lock().unwrap();
                            s.stats.frames = d.stats.blocks;
                            s.stats.frames_fec_busy += busy;
                            // (the web UI's demod CPU)
                            s.stats.other_s += dt;
                            s.stats.locked = d.stats.locked;
                            s.stats.esn0_db = d.stats.mer_db;
                            s.stats.data_esn0_db = d.stats.mer_db;
                            s.stats.freq_hz = d.stats.freq_hz;
                            if d.constellation_seq != seen {
                                seen = d.constellation_seq;
                                let mut m = Vec::with_capacity(1 + 2 * d.constellation.len());
                                m.push(8u8);
                                m.extend(d.constellation.iter().flat_map(|p| [p[0] as u8, p[1] as u8]));
                                s.msgs.push_back(m);
                            }
                        }
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                    }
                }
                stop.store(true, Ordering::Relaxed);
                let _ = ring.join();
            })
            .expect("spawn datv-rx");
        RxThread { tx, shared, fec_stats, spec, label, sr: 0.0, dropped, started: std::time::Instant::now(), fpga_center: Some(center), t2_bw: Some(mode.bw_hz) }
    }

    /// Receiving through the FPGA front end.
    pub fn uses_fpga(&self) -> bool {
        self.fpga_center.is_some()
    }

    /// A block of stream IQ; the signal sits `center_hz` from its centre.
    /// With the FPGA front end only the centre matters (the DDC's NCO).
    pub fn feed(&self, iq: &[Complex32], center_hz: f64) {
        if let Some(c) = &self.fpga_center {
            c.store(center_hz.to_bits(), std::sync::atomic::Ordering::Relaxed);
            // Debug (DVB-T2, radio.fpga_decimation = false so this is the
            // ADC stream the FPGA resampler gets too): `touch /tmp/t2-iq`
            // appends it to /tmp/t2-iq.cf32, 40 MB at most.
            if self.t2_bw.is_some() && std::path::Path::new("/tmp/t2-iq").exists() {
                use std::io::Write;
                if let Ok(mut fh) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/t2-iq.cf32") {
                    if fh.metadata().map_or(0, |m| m.len()) < 40_000_000 {
                        let b: Vec<u8> = iq.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
                        let _ = fh.write_all(&b);
                    }
                }
            }
            return;
        }
        if self.tx.try_send((iq.to_vec(), center_hz)).is_err() {
            self.dropped.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Through the FPGA's DDC (not the stream IQ).
    pub fn through_fpga(&self) -> bool {
        self.fpga_center.is_some()
    }

    /// The DVB service information received so far.
    pub fn si(&self) -> super::ts::Si {
        self.shared.lock().unwrap().si.clone()
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
pub(crate) mod tests {
    /// Long-frame PLFRAMEs as the FPGA transmitter makes them (normal frames,
    /// pilots, QPSK or 8PSK): BBFRAME, BB scrambling, BCH, LDPC, bit interleaving (8PSK), mapping,
    /// header, pilots, PL scrambling. Unit-power symbols.
    pub(crate) fn long_symbols(mode: super::super::fpga_tx::LongMode, frames: usize, next: &mut dyn FnMut() -> [u8; TS_LEN]) -> Vec<Complex32> {
        use super::super::{Framer, PILOT as PIL, PSK8_PHASE, SLOT as SL, bb_scrambling, pl_scrambling, rotate};
        use super::super::ldpc_fpga::{encode, N};
        let spec = FrameSpec::long(mode);
        let rate = spec.long_rate.unwrap();
        let mut framer = Framer::new();
        let bch = super::super::bch::Bch::new();
        let bbscr = bb_scrambling(spec.kbch / 8);
        let header = spec.header();
        let scr = pl_scrambling(spec.frame_symbols() - SL);
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let mut out = Vec::new();
        for _ in 0..frames {
            let bb = framer.frame_bytes(spec.kbch / 8, 0, next);
            let mut info = vec![0u8; rate.k()];
            for (i, byte) in bb.iter().enumerate() {
                for b in 0..8 {
                    info[i * 8 + b] = ((byte ^ bbscr[i]) >> (7 - b)) & 1;
                }
            }
            bch.encode(&mut info);
            let cw = encode(rate, &info);
            let nsym = N / spec.bps;
            let syms: Vec<Complex32> = (0..nsym)
                .map(|i| {
                    if spec.bps == 2 {
                        let (b0, b1) = (cw[2 * i], cw[2 * i + 1]);
                        Complex32::new(if b0 == 0 { a } else { -a }, if b1 == 0 { a } else { -a })
                    } else {
                        let rows = N / 3;
                        let v = (cw[i] << 2 | cw[rows + i] << 1 | cw[2 * rows + i]) as usize;
                        let ph = std::f32::consts::FRAC_PI_4 * PSK8_PHASE[v] as f32;
                        Complex32::new(ph.cos(), ph.sin())
                    }
                })
                .collect();
            out.extend_from_slice(&header);
            let mut k = 0;
            for (n, s) in syms.iter().enumerate() {
                if n > 0 && n % (16 * SL) == 0 {
                    for _ in 0..PIL {
                        out.push(rotate(Complex32::new(a, a), scr[k]));
                        k += 1;
                    }
                }
                out.push(rotate(*s, scr[k]));
                k += 1;
            }
        }
        out
    }

    /// A candidate header right at the start of the buffer and the next one
    /// two symbols early (symbols lost between): the frame before the second
    /// would start before the buffer. It used to wrap and index at
    /// usize::MAX; now the second header is the new candidate, and the
    /// frames after it decode.
    #[test]
    fn early_second_header_at_the_buffer_start() {
        use super::super::fpga_tx::LongMode;
        let spec = FrameSpec::long(LongMode::Qpsk12);
        let mut next = counter_packets();
        let syms = long_symbols(LongMode::Qpsk12, 5, &mut next);
        let mut x = vec![Complex32::new(0.01, -0.01)];
        x.extend_from_slice(&syms[..1000]);
        x.extend_from_slice(&syms[1002..]);
        let mut rx = Receiver::new_symbols_spec(spec, 250e3, 0.0);
        let mut out = Vec::new();
        for c in x.chunks(4096) {
            rx.process(c, &mut out);
        }
        assert!(rx.stats.locked && out.len() >= 21, "{} packets, {:?}", out.len(), rx.stats);
    }

    pub(crate) fn counter_packets() -> impl FnMut() -> [u8; TS_LEN] {
        let mut n = 0u32;
        move || {
            let mut pkt = [0u8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1..5].copy_from_slice(&n.to_be_bytes());
            for (i, b) in pkt[5..].iter_mut().enumerate() {
                *b = (n as usize * 7 + i) as u8;
            }
            n += 1;
            pkt
        }
    }

    /// Long frames through RRC, noise and a carrier offset into the receiver,
    /// decoded by the FPGA decoder's model: every packet back, in order.
    fn long_link(mode: super::super::fpga_tx::LongMode, esn0_db: f32, frames: usize) -> (Stats, usize) {
        let (sps, rs) = (4usize, 64_000.0);
        let fs = rs * sps as f64;
        let mut next = counter_packets();
        let syms = long_symbols(mode, frames, &mut next);
        let spec = FrameSpec::long(mode);
        let h = super::super::rrc_taps(sps, spec.rolloff, 12);
        let mut iq = vec![Complex32::default(); syms.len() * sps + h.len()];
        for (i, s) in syms.iter().enumerate() {
            for (t, &c) in h.iter().enumerate() {
                iq[i * sps + t] += s * c;
            }
        }
        let esn0 = 10f32.powf(esn0_db / 10.0);
        let sigma = (sps as f32 / esn0 / 2.0).sqrt();
        let mut seed = 17u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        let off = 120.0;
        for (k, z) in iq.iter_mut().enumerate() {
            let ph = std::f64::consts::TAU * off * k as f64 / fs;
            *z = *z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g());
        }
        let mut rx = Receiver::with_spec(spec, fs, rs, 0.0);
        let mut out = Vec::new();
        for c in iq.chunks(8192) {
            rx.process(c, &mut out);
        }
        let data: Vec<_> = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).collect();
        if let Some(first) = data.first() {
            let f0 = u32::from_be_bytes(first[1..5].try_into().unwrap());
            for (i, pkt) in data.iter().enumerate() {
                let k = f0 + i as u32;
                assert_eq!(u32::from_be_bytes(pkt[1..5].try_into().unwrap()), k, "{mode:?}: packet {i} of {}; {:?}", data.len(), rx.stats);
                let bad: Vec<usize> = pkt[5..].iter().enumerate().filter(|(j, b)| **b != (k as usize * 7 + j) as u8).map(|(j, _)| j + 5).collect();
                assert!(bad.is_empty(), "{mode:?}: packet {i} of {}: {} bytes wrong, from {:?} to {:?}; {:?}", data.len(), bad.len(), bad.first(), bad.last(), rx.stats);
            }
        }
        (rx.stats, data.len())
    }

    #[test]
    fn long_frames_qpsk_1_2() {
        let (s, n) = long_link(super::super::fpga_tx::LongMode::Qpsk12, 3.0, 22);
        assert!(s.locked && (s.freq_hz - 120.0).abs() < 5.0, "{s:?}");
        // 21.4 packets a frame; all but the first frame or two.
        assert!(n >= 19 * 21 && s.ldpc_fail == 0, "{n} packets, {s:?}");
    }

    #[test]
    fn long_frames_qpsk_3_4() {
        let (s, n) = long_link(super::super::fpga_tx::LongMode::Qpsk34, 6.5, 22);
        assert!(s.locked, "{s:?}");
        // 32.1 packets a frame; all but the first frame or two.
        assert!(n >= 19 * 32 && s.ldpc_fail == 0, "{n} packets, {s:?}");
    }

    #[test]
    fn long_frames_8psk_3_4() {
        let (s, n) = long_link(super::super::fpga_tx::LongMode::Psk8_34, 10.0, 22);
        assert!(s.locked, "{s:?}");
        // 32.1 packets a frame; all but the first frame or two.
        assert!(n >= 19 * 32 && s.ldpc_fail == 0, "{n} packets, {s:?}");
    }

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
