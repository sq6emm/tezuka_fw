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
//!
//! Other PLFRAMEs on the carrier (dummy frames, EN 302 307-1 5.5.1, and
//! frames of other MODCODs or sizes, VCM) are recognised by their PLS code
//! when locked and stepped over by their own length; only the configured
//! MODCOD is decoded (a frame of another one counts as lost for the TS).

use std::borrow::Cow;
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
    /// CPU seconds of the decoder thread (`ldpc_s` is wall time: with the
    /// FPGA decoder mostly its wait).
    pub fec_cpu_s: f64,
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
    /// Frames stepped over: dummy PLFRAMEs / frames of another MODCOD or size.
    pub frames_dummy: u64,
    pub frames_other: u64,
    /// BBFRAMEs decoded but not taken: MATYPE other than TS, single stream,
    /// no ISSY, no null-packet deletion, UPL 1504 (or a bad DFL / SYNCD).
    pub bb_unsupported: u64,
    /// User packets whose CRC-8 (EN 302 307-1 5.1.4) failed: passed on with
    /// transport_error_indicator set.
    pub ts_crc_bad: u64,
    /// DVB-T2 blocks in DDR whose router buffer was reused before they were
    /// decoded (the decoder more than three frames behind): dropped.
    pub blocks_lapped: u64,
    /// DVB-S2 frames from the ring the engine could not read in time (the
    /// recorder had overwritten them): lost.
    pub ring_lapped: u64,
    /// Ring mode, tracking: known blocks from the FPGA's accumulator /
    /// made from the ring's words (s2trk).
    pub trk_hw: u64,
    pub trk_model: u64,
}

/// Length (symbols) of a PLFRAME with this PLS; None for reserved MODCODs.
pub fn plframe_len(modcod: u8, short: bool, pilots: bool) -> Option<usize> {
    if modcod == 0 {
        // Dummy PLFRAME: header and 36 slots, no pilots.
        return Some(SLOT + 36 * SLOT);
    }
    let bps = match modcod {
        1..=11 => 2,
        12..=17 => 3,
        18..=23 => 4,
        24..=28 => 5,
        _ => return None,
    };
    let slots = if short { 16_200 } else { 64_800 } / bps / SLOT;
    Some(SLOT + slots * SLOT + if pilots { (slots - 1) / 16 * PILOT } else { 0 })
}

/// Ring mode ([`Receiver::new_ring_spec`]): the receiver keeps the ring's
/// raw words and makes a symbol of one (i16 I/Q over 32768, times the
/// AFC's mixer) only where it looks: headers, pilots, a sample of the data
/// for the constellation. The data symbols go to the LDPC engine as a
/// [`super::s2ring::Job`]: the engine reads them from the ring itself.
struct RingRx {
    raw: Vec<u32>,
    /// Absolute ring word of `raw[0]`.
    abs0: u64,
    mix: Mix,
    /// The mixer before the last retune (for symbols before it).
    prev: Option<Mix>,
    /// The engine reads the ring (otherwise the job carries the words and
    /// the model makes the cells: tests, a PC).
    engine: bool,
    watch: Option<std::sync::Arc<super::fpga::RingWatch>>,
    /// The FPGA's known-block entries (s2trk) by absolute first symbol.
    hw: std::collections::BTreeMap<u64, super::s2trk::Entry>,
    tabs: super::s2trk::Tables,
    /// Our header's quarter turns (the unit's table).
    hdr_q: Vec<u8>,
    /// The unit is there ([`Receiver::set_trk`]); commands for it.
    trk: bool,
    ctl: Vec<super::s2trk::Ctl>,
    trk_loaded: Option<u64>,
    dth_sent: Option<u32>,
}

/// The AFC's mixer as a function of the absolute symbol: th0 + w (k - k0).
#[derive(Clone, Copy, Debug)]
struct Mix {
    k0: u64,
    th0: f64,
    w: f64,
}

impl Mix {
    fn at(&self, k: u64) -> f64 {
        self.th0 + self.w * (k as i64 - self.k0 as i64) as f64
    }
}

impl RingRx {
    fn phase(&self, k: u64) -> f64 {
        match self.prev {
            Some(p) if k < self.mix.k0 => p.at(k),
            _ => self.mix.at(k),
        }
    }
    /// The mixer's step a symbol at `k`.
    fn w_at(&self, k: u64) -> f64 {
        match self.prev {
            Some(p) if k < self.mix.k0 => p.w,
            _ => self.mix.w,
        }
    }
    fn raw_sym(w: u32) -> Complex32 {
        Complex32::new((w as u16 as i16) as f32 / 32768.0, ((w >> 16) as u16 as i16) as f32 / 32768.0)
    }
}

/// A PLFRAME the receiver steps over: its PLS, length and header.
struct Other {
    modcod: u8,
    len: usize,
    header: Vec<Complex32>,
    /// Its PLS index (modcod << 2 | short << 1 | pilots).
    pls: u8,
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
    /// Long frames: the data symbols as cells for the LDPC engine.
    cells: Vec<[i8; 2]>,
    /// Long frames with float LLRs made here instead (never on a board:
    /// the path before the engine's demapper, for comparisons).
    pub float_llr: bool,
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
    /// Every other PLFRAME type (dummy, other MODCODs and sizes).
    others: Vec<Other>,
    /// The frame at `locked_at`: ours (None) or `others[i]`.
    cur: Option<usize>,
    /// Ring mode, tracking: the carrier fit, amplitude and noise from
    /// per-block sums (s2trk; false: from every known symbol
    /// as before).
    sums: bool,
    ring: Option<RingRx>,
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
    /// BBFRAMEs with an unsupported MATYPE so far (logged once).
    matype_warned: bool,
    /// The PL scrambling sequence (ring frames the model turns).
    pl_scr: Vec<u8>,
}

enum FecMode {
    Inline(Fec),
    /// Frames of LLRs to the decoding thread; it counts failures in a row
    /// (read back here for re-acquisition).
    Thread { tx: crossbeam_channel::Sender<S2Block>, fails: std::sync::Arc<std::sync::atomic::AtomicU32> },
}

/// One DVB-S2 frame for the decoding thread: float LLRs (short frames), or a
/// long frame's data symbols as cells with its noise scale `kq` and bits
/// per symbol (the FPGA's LDPC engine makes the LLRs, [`super::s2cells`]).
pub enum S2Block {
    Llr(Vec<f32>),
    Cells(Vec<[i8; 2]>, i32, u8),
    /// A long frame the engine reads from the ring (s2ring).
    Ring(Box<super::s2ring::Job>),
}

impl S2Block {
    /// A frame missing.
    fn lost() -> Self {
        S2Block::Llr(Vec::new())
    }
}

impl FecBlock for S2Block {
    fn is_empty(&self) -> bool {
        match self {
            S2Block::Llr(v) => v.is_empty(),
            S2Block::Cells(c, ..) => c.is_empty(),
            S2Block::Ring(_) => false,
        }
    }
    fn decode(&self, fec: &mut Fec, st: &mut Stats, ts: &mut Vec<[u8; TS_LEN]>) -> bool {
        match self {
            S2Block::Llr(v) => fec.frame(v, st, ts),
            S2Block::Cells(c, kq, bps) => fec.frame_cells(c, &super::s2cells::params(*bps as usize, *kq), st, ts),
            S2Block::Ring(job) => fec.frame_ring(job, st, ts),
        }
    }
    fn dump(&self) -> Vec<u8> {
        match self {
            S2Block::Llr(v) => v.dump(),
            S2Block::Cells(c, kq, bps) => {
                let p = super::s2cells::params(*bps as usize, *kq);
                let l = crate::dvbt2::stream::cell_llrs(c, &p);
                l.iter().flat_map(|&v| (v as f32 / super::fpga_ldpc::LLR_SCALE).to_le_bytes()).collect()
            }
            // (the words are in the ring only, unless the job carries them)
            S2Block::Ring(job) => match &job.words {
                Some(w) => {
                    let scr = pl_scrambling(w.len());
                    let c = super::s2ring::cells(w, job.n_cells, job.pilots, &job.segs, job.gain, &scr);
                    S2Block::Cells(c, job.kq, job.bps).dump()
                }
                None => Vec::new(),
            },
        }
    }
}

/// Time in BCH and in deframing (ns), for the FEC thread's log.
pub static FEC_PROF_NS: [std::sync::atomic::AtomicU64; 2] = [const { std::sync::atomic::AtomicU64::new(0) }; 2];

impl Fec {
    pub fn new(spec: FrameSpec) -> Self {
        let bch = Some(if spec.is_short() { Bch::short() } else { Bch::new() });
        let mut dec = Ldpc::for_spec(&spec);
        // normal frames: BBFRAME bytes and the BCH remainder from the fabric
        if !spec.is_short() {
            dec.want_bb();
        }
        Fec { spec, dec, bch, bbscr: bb_scrambling(spec.kbch / 8), bits: vec![0; spec.n], partial: Vec::new(), have_prev: false, matype_warned: false, pl_scr: Vec::new() }
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

    /// A DVB-S2 long frame from the receive ring (s2ring): the engine reads
    /// and turns its symbols; without one (the job carries the words) the
    /// model makes the cells here. A frame no longer in the ring is lost.
    pub fn frame_ring(&mut self, job: &super::s2ring::Job, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        if let Some(w) = &job.words {
            if self.pl_scr.len() < w.len() {
                self.pl_scr = pl_scrambling(w.len());
            }
            let c = super::s2ring::cells(w, job.n_cells, job.pilots, &job.segs, job.gain, &self.pl_scr);
            return self.frame_cells(&c, &super::s2cells::params(job.bps as usize, job.kq), stats, out);
        }
        let Some(watch) = &job.watch else {
            stats.frames_bad += 1;
            self.have_prev = false;
            return false;
        };
        match self.dec.decode_ring(job, watch, &mut self.bits) {
            Ok(ok) => self.after_decode(ok, t0, stats, out),
            Err(()) => {
                // lapped (the decoder far behind) or no engine: lost
                stats.frames_bad += 1;
                stats.ring_lapped += 1;
                self.have_prev = false;
                false
            }
        }
    }

    /// A DVB-T2 QPSK block as cells (the FPGA decoder makes the LLRs).
    pub fn frame_cells(&mut self, cells: &[[i8; 2]], p: &crate::dvbt2::stream::CellParams, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t0 = std::time::Instant::now();
        let ok = self.dec.decode_cells(cells, p, &mut self.bits);
        self.after_decode(ok, t0, stats, out)
    }

    fn after_decode(&mut self, ok: Option<usize>, t0: std::time::Instant, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let t_dec = std::time::Instant::now();
        // The fabric's BBFRAME (descrambled bytes, the BCH remainder): no
        // bits to unpack, divide or descramble here.
        if let Some((mut bytes, rem)) = self.dec.take_bb() {
            let n = self.spec.kbch + 192;
            let good = match self.bch.as_ref().map(|b| b.correct_bytes(&mut bytes, rem, n, self.spec.kbch)) {
                Some(Outcome::Clean) | None => true,
                Some(Outcome::Fixed(k)) => {
                    stats.bch_fixed += k as u64;
                    true
                }
                Some(Outcome::Failed) => {
                    if ok.is_some() {
                        stats.bch_fail += 1;
                    }
                    false
                }
            };
            stats.ldpc_s += t0.elapsed().as_secs_f64();
            use std::sync::atomic::Ordering::Relaxed;
            FEC_PROF_NS[0].fetch_add(t_dec.elapsed().as_nanos() as u64, Relaxed);
            let r = if good {
                let t_df = std::time::Instant::now();
                let r = self.deframe_bytes(&bytes[..self.spec.kbch / 8], stats, out);
                FEC_PROF_NS[1].fetch_add(t_df.elapsed().as_nanos() as u64, Relaxed);
                r
            } else {
                stats.frames_bad += 1;
                stats.ldpc_fail += 1;
                self.have_prev = false;
                false
            };
            self.dec.return_bb(bytes);
            return r;
        }
        if self.dec.bb_lost() {
            // (a time-out: neither bytes nor this frame's bits)
            stats.ldpc_s += t0.elapsed().as_secs_f64();
            stats.frames_bad += 1;
            stats.ldpc_fail += 1;
            self.have_prev = false;
            return false;
        }
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

/// Normalized header correlation (0..1) of the symbols `s`, chunked
/// coherent (10 symbols) so a frequency error of a few hundred Hz does not hurt.
fn header_metric_of(s: &[Complex32], header: &[Complex32]) -> f32 {
    let (mut num, mut den) = (0f32, 0f32);
    for c in 0..SLOT / 10 {
        let mut acc = Complex32::default();
        for i in c * 10..c * 10 + 10 {
            acc += s[i] * header[i].conj();
            den += s[i].norm();
        }
        num += acc.norm();
    }
    num / den.max(1e-20)
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
            cells: Vec::with_capacity(spec.n / 2),
            float_llr: false,
            stats: Stats::default(),
            constellation: Vec::new(),
            constellation_seq: 0,
            symbol_input: false,
            flags: Vec::new(),
            flagged: false,
            others: {
                let mut v = Vec::new();
                for modcod in 0..=28u8 {
                    for (short, pilots) in [(false, false), (false, true), (true, false), (true, true)] {
                        if modcod == spec.modcod && short == spec.is_short() && pilots == spec.pilots {
                            continue;
                        }
                        let len = plframe_len(modcod, short, pilots).unwrap();
                        v.push(Other { modcod, len, header: super::plheader_typed(modcod, pilots, short), pls: modcod << 2 | (short as u8) << 1 | pilots as u8 });
                    }
                }
                v
            },
            cur: None,
            sums: true,
            ring: None,
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

    /// Ring mode: the FPGA's symbols as the ring's raw words
    /// ([`Self::process_ring`], from absolute word `at`), header flags in
    /// them; long frames go to the engine as ring jobs (`engine`: it reads
    /// the ring; otherwise the jobs carry the words for the model).
    pub fn new_ring_spec(spec: FrameSpec, rs: f64, at: u64, engine: bool, watch: Option<std::sync::Arc<super::fpga::RingWatch>>) -> Self {
        let mut r = Receiver::with_spec(spec, rs, rs, 0.0);
        r.symbol_input = true;
        r.flagged = true;
        let hdr_q = super::s2trk::quarters(&r.header);
        r.ring = Some(RingRx {
            raw: Vec::new(),
            abs0: at,
            mix: Mix { k0: at, th0: 0.0, w: 0.0 },
            prev: None,
            engine,
            watch,
            hw: Default::default(),
            tabs: Default::default(),
            hdr_q,
            trk: false,
            ctl: Vec::new(),
            trk_loaded: None,
            dth_sent: None,
        });
        r
    }

    /// Ring mode: the FPGA has the known-symbol accumulator (s2trk); its
    /// commands then come out of [`Self::trk_ctl`].
    pub fn set_trk(&mut self, on: bool) {
        if let Some(r) = self.ring.as_mut() {
            r.trk = on;
            if on {
                r.ctl.push(super::s2trk::Ctl::Header(r.hdr_q.clone()));
            }
        }
    }

    /// Commands for the FPGA's accumulator since the last call.
    pub fn trk_ctl(&mut self) -> Vec<super::s2trk::Ctl> {
        self.ring.as_mut().map_or_else(Vec::new, |r| std::mem::take(&mut r.ctl))
    }

    /// The accumulator's entries (as read from its FIFO).
    pub fn push_trk(&mut self, ents: &[[u32; super::s2trk::ENTRY_WORDS]]) {
        let Some(r) = self.ring.as_mut() else { return };
        let now = r.abs0 + r.raw.len() as u64;
        for w in ents {
            let e = super::s2trk::Entry::from_words(w);
            let k = crate::dvbt2::fe::extend(e.k0, 32, now);
            r.hw.insert(k, e);
        }
        // (what fell out of the buffer)
        let keep = r.hw.split_off(&r.abs0);
        r.hw = keep;
    }

    /// Ring words from absolute word `at` (see [`Self::new_ring_spec`]);
    /// a gap (the reader lapped) loses lock and starts afresh there.
    pub fn process_ring(&mut self, at: u64, words: &[u32], out: &mut Vec<[u8; TS_LEN]>) {
        let t0 = std::time::Instant::now();
        let ldpc0 = self.stats.ldpc_s;
        let r = self.ring.as_mut().expect("ring mode");
        if at != r.abs0 + r.raw.len() as u64 {
            r.raw.clear();
            r.abs0 = at;
            r.prev = None;
            r.mix = Mix { k0: at, th0: 0.0, w: r.mix.w };
            self.stats.blocks_dropped += 1;
            if self.locked_at.is_some() || self.candidate.is_some() {
                self.locked_at = None;
                self.candidate = None;
                self.cur = None;
                self.stats.locked = false;
                self.fec_lost();
            }
        }
        if let Some(r) = self.ring.as_mut() {
            r.raw.extend_from_slice(words);
        }
        self.frames(out);
        self.stats.other_s += t0.elapsed().as_secs_f64() - (self.stats.ldpc_s - ldpc0);
    }

    /// The TS loses the packet straddling a frame that will not come.
    fn fec_lost(&mut self) {
        match &mut self.fec {
            FecMode::Inline(f) => f.lost(),
            FecMode::Thread { tx, .. } => {
                let _ = tx.try_send(S2Block::lost());
            }
        }
    }

    /// Symbols buffered.
    fn nsyms(&self) -> usize {
        match &self.ring {
            Some(r) => r.raw.len(),
            None => self.syms.len(),
        }
    }

    /// Buffered symbol `k` (ring mode: made from its word now).
    fn sym(&self, k: usize) -> Complex32 {
        match &self.ring {
            Some(r) => {
                let ph = r.phase(r.abs0 + k as u64);
                RingRx::raw_sym(r.raw[k]) * Complex32::new(ph.cos() as f32, ph.sin() as f32)
            }
            None => self.syms[k],
        }
    }

    /// Buffered symbols `k..k + n` (ring mode: made now, the mixer's
    /// phasor stepped along).
    fn win(&self, k: usize, n: usize) -> Cow<'_, [Complex32]> {
        let Some(r) = &self.ring else { return Cow::Borrowed(&self.syms[k..k + n]) };
        let a = r.abs0 + k as u64;
        let m = match r.prev {
            Some(p) if a < r.mix.k0 => {
                if a + n as u64 > r.mix.k0 {
                    return Cow::Owned((k..k + n).map(|i| self.sym(i)).collect());
                }
                p
            }
            _ => r.mix,
        };
        let ph = m.at(a);
        let mut z = Complex32::new(ph.cos() as f32, ph.sin() as f32);
        let st = Complex32::new(m.w.cos() as f32, m.w.sin() as f32);
        Cow::Owned(
            r.raw[k..k + n]
                .iter()
                .map(|&w| {
                    let y = RingRx::raw_sym(w) * z;
                    z *= st;
                    y
                })
                .collect(),
        )
    }

    /// The FPGA's header candidate flag at symbol `k`.
    fn flag(&self, k: usize) -> bool {
        match &self.ring {
            Some(r) => r.raw.get(k).is_some_and(|w| w & 0x1_0000 != 0),
            None => self.flags.get(k) == Some(&true),
        }
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
        self.header_metric_with(k, &self.header)
    }

    fn header_metric_with(&self, k: usize, header: &[Complex32]) -> f32 {
        header_metric_of(&self.win(k, SLOT), header)
    }

    /// After a frame of ours at `p` (tracking): the FPGA's accumulator gets
    /// the mixer's step when it changed (it takes it at its next frame
    /// start) and, when it did not have this frame's header, the next
    /// frame to follow from.
    fn trk_commands(&mut self, p: usize, next: usize, kind: Option<usize>) {
        let (len, pilots, npil) = (self.frame_len as u32, self.spec.pilots, (self.known.len() - 1) as u8);
        let Some(r) = self.ring.as_mut().filter(|r| r.trk) else { return };
        let d = super::s2trk::step(r.mix.w);
        if r.dth_sent != Some(d) {
            r.ctl.push(super::s2trk::Ctl::Dth(d));
            r.dth_sent = Some(d);
        }
        let at = r.abs0 + p as u64;
        if kind.is_none() && !r.hw.contains_key(&at) && r.trk_loaded.is_none_or(|l| at >= l + 2 * len as u64) {
            r.ctl.push(super::s2trk::Ctl::Load { base: r.abs0 + next as u64, len, pilots, npil });
            r.trk_loaded = Some(at);
        }
    }

    /// Ring mode: the known block at buffer index `k` (`len` symbols; a
    /// header with quarter turns `hdr`, ours when None and `pilot_at` None;
    /// pilots from frame position `pilot_at` on): its correlation with the
    /// references per symbol, mixed as the receiver mixes, and its power.
    /// The FPGA's entry when it has one made with our mixer step, else the
    /// same sums from the words (s2trk::block).
    fn known_block(&mut self, k: usize, len: usize, hdr: Option<&[u8]>, pilot_at: Option<usize>) -> (Complex32, f64) {
        let r = self.ring.as_ref().expect("ring mode");
        let abs = r.abs0 + k as u64;
        let ph0 = r.phase(abs);
        let w = r.w_at(abs);
        let dth = super::s2trk::step(w);
        let mid = ph0 + w * (len - 1) as f64 / 2.0;
        let ours = hdr.is_none();
        // (its own mixer's step may lag ours by a retune: its mean phase is
        // corrected by corr(); the rest turns the block's ends by
        // dw len / 2 at most, taken when under 0.05 rad)
        let close = |e: &&super::s2trk::Entry| {
            let dw = e.dth.wrapping_sub(dth) as i32 as f64 * std::f64::consts::TAU / 4_294_967_296.0;
            dw.abs() * len as f64 / 2.0 < 0.05
        };
        if let Some(e) = r.hw.get(&abs).filter(|e| ours && close(e)) {
            let c = (e.corr(len, mid), e.power());
            self.stats.trk_hw += 1;
            return c;
        }
        let words = &r.raw[k..k + len];
        let e = match pilot_at {
            Some(at) => super::s2trk::block(&r.tabs, words, abs, self.scramble[at - SLOT..at - SLOT + len].iter().copied(), super::s2trk::turns(ph0), dth),
            None => super::s2trk::block(&r.tabs, words, abs, hdr.unwrap_or(&r.hdr_q).iter().copied(), super::s2trk::turns(ph0), dth),
        };
        self.stats.trk_model += 1;
        (e.corr(len, mid), e.power())
    }

    /// Another PLFRAME's header in `lo..=hi`: the best SOF position (two
    /// coherent chunks of 13, as the FPGA's [`super::hdrdet`]), then every
    /// other PLS code there. (position, metric, index into `others`) when
    /// one reaches [`SYNC_MIN`].
    fn other_at(&self, lo: usize, hi: usize) -> Option<(usize, f32, usize)> {
        let sof = |k: usize| -> f32 {
            let s = self.win(k, 26);
            let (mut num, mut den) = (0f32, 0f32);
            for c in 0..2 {
                let mut acc = Complex32::default();
                for i in c * 13..c * 13 + 13 {
                    acc += s[i] * self.header[i].conj();
                    den += s[i].norm();
                }
                num += acc.norm();
            }
            num / den.max(1e-20)
        };
        let k = (lo..=hi).max_by(|&a, &b| sof(a).total_cmp(&sof(b)))?;
        let w = self.win(k, SLOT);
        let (i, m) = if self.sums {
            // the PLS there (one decode, not 115 correlations)
            let i = self.pls_other(&w)?;
            (i, header_metric_of(&w, &self.others[i].header))
        } else {
            self.others.iter().enumerate().map(|(i, o)| (i, header_metric_of(&w, &o.header))).max_by(|a, b| a.1.total_cmp(&b.1))?
        };
        (m >= SYNC_MIN).then_some((k, m, i))
    }

    /// The other frame type whose PLS the header `w` carries (None: ours,
    /// or none of the others). Its phase from the SOF, then
    /// [`super::s2trk::pls_decode`].
    fn pls_other(&self, w: &[Complex32]) -> Option<usize> {
        let c: Complex32 = (0..26).map(|i| w[i] * self.header[i].conj()).sum();
        let rot = c.conj() / c.norm().max(1e-20);
        let h: Vec<Complex32> = w.iter().map(|z| z * rot).collect();
        let (idx, _) = super::s2trk::pls_decode(&h);
        self.others.iter().position(|o| o.pls == idx)
    }

    /// A frame of another type was stepped over.
    fn skip_other(&mut self, i: usize) {
        if self.others[i].modcod == 0 {
            // No BBFRAME: the TS continues in the next frame.
            self.stats.frames_dummy += 1;
            return;
        }
        // Part of the stream we cannot decode: the packet straddling it is lost.
        self.stats.frames_other += 1;
        match &mut self.fec {
            FecMode::Inline(f) => f.lost(),
            FecMode::Thread { tx, .. } => {
                let _ = tx.try_send(S2Block::lost());
            }
        }
    }

    /// The best header position in `start..end`: [`Self::header_metric`],
    /// computed only where the SOF alone (its first 30 symbols, the same
    /// chunked metric) passes a loose threshold. Unlocked, this runs for
    /// every symbol: at 250 kS/s the full metric everywhere cost the A9 more
    /// than a core, the receiver fell behind the ring and never locked.
    /// Drop the first `n` symbols (and their flags).
    fn drain_syms(&mut self, n: usize) {
        if let Some(r) = self.ring.as_mut() {
            let n = n.min(r.raw.len());
            r.raw.drain(..n);
            r.abs0 += n as u64;
            if r.prev.is_some() && r.abs0 >= r.mix.k0 {
                r.prev = None;
            }
            // Keep the mixer's reference near (f64 phase over hours).
            if r.prev.is_none() && r.abs0 > r.mix.k0 + (1 << 20) {
                r.mix = Mix { k0: r.abs0, th0: wrap(r.mix.at(r.abs0)), w: r.mix.w };
            }
            return;
        }
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
                if self.flag(k + last) {
                    let m = self.header_metric(k);
                    if m > best.1 && !self.other_beats(k, m) {
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
            if m > best.1 && !self.other_beats(k, m) {
                best = (k, m);
            }
        }
        best
    }

    /// Another PLS code fits the header at `k` better than ours (`m`): all
    /// share the SOF, which alone makes over a quarter of our metric.
    fn other_beats(&self, k: usize, m: f32) -> bool {
        if m < SYNC_MIN {
            return false;
        }
        let w = self.win(k, SLOT);
        if self.sums {
            return self.pls_other(&w).is_some_and(|i| header_metric_of(&w, &self.others[i].header) > m);
        }
        self.others.iter().any(|o| header_metric_of(&w, &o.header) > m)
    }

    fn frames(&mut self, out: &mut Vec<[u8; TS_LEN]>) {
        let l = self.frame_len;
        loop {
            match self.locked_at {
                None => {
                    // Search: the best header position within one frame of symbols.
                    let start = self.candidate.map_or(0, |c| c + l - TRACK_SLACK);
                    let need = start + if self.candidate.is_some() { 2 * TRACK_SLACK } else { l } + SLOT;
                    if self.nsyms() < need {
                        return;
                    }
                    let end = need - SLOT;
                    let (mut best, mut m) = self.search(start, end);
                    if m < SYNC_MIN && self.candidate.is_some() {
                        // One frame after ours a dummy or another MODCOD's
                        // header confirms it as well.
                        if let Some((k, mo, _)) = self.other_at(start, end - 1) {
                            (best, m) = (k, mo);
                        }
                    }
                    if m >= SYNC_MIN {
                        if let Some(first) = self.candidate.and(best.checked_sub(l)) {
                            // Two headers one frame apart: locked.
                            self.locked_at = Some(first);
                            self.cur = None;
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
                    // This frame's length: ours, or the one stepped over.
                    let l = self.cur.map_or(l, |i| self.others[i].len);
                    // A frame is ready once the next header can be checked.
                    if self.nsyms() < p + l + TRACK_SLACK + SLOT {
                        return;
                    }
                    let lo = p + l - TRACK_SLACK;
                    let (mut next, mut m) = (lo..=p + l + TRACK_SLACK).map(|k| (k, self.header_metric(k))).fold((lo, 0f32), |a, b| if b.1 > a.1 { b } else { a });
                    // Not ours: a dummy frame or another MODCOD (by its PLS code).
                    // (All share the SOF: a header of another PLS can pass
                    // ours; the better fit wins.)
                    let mut kind = None;
                    if let Some((k, mo, i)) = self.other_at(lo, p + l + TRACK_SLACK) {
                        if mo > m {
                            (next, m, kind) = (k, mo, Some(i));
                        }
                    }
                    if std::env::var_os("DVBS2_DEBUG").is_some() {
                        eprintln!("frame: next header at {:+} (metric {m:.2}, {})", next as i64 - (p + l) as i64, kind.map_or("ours".to_string(), |i| format!("modcod {}", self.others[i].modcod)));
                    }
                    let next = if m >= SYNC_MIN {
                        self.missed = 0;
                        next
                    } else {
                        self.missed += 1;
                        kind = None;
                        p + l
                    };
                    match self.cur {
                        None => self.frame(p, next, kind, out),
                        Some(i) => self.skip_other(i),
                    }
                    self.cur = kind;
                    if self.missed >= LOST_AFTER {
                        self.cur = None;
                        self.locked_at = None;
                        self.candidate = None;
                        self.stats.locked = false;
                        self.fec_lost();
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
        self.block_with(base, at, len, f, &self.header)
    }

    /// [`Self::block`] with the header of the frame at `base` given.
    fn block_with(&self, base: usize, at: usize, len: usize, f: f64, header: &[Complex32]) -> Complex32 {
        let mut acc = Complex32::default();
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let w = -std::f64::consts::TAU * f / self.rs;
        let mid = (len - 1) as f64 / 2.0;
        let x = self.win(base + at, len);
        for i in 0..len {
            let ph = w * (i as f64 - mid);
            let s = x[i] * Complex32::new(ph.cos() as f32, ph.sin() as f32);
            let r = if at == 0 {
                header[i]
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
        if let Some(r) = self.ring.as_mut() {
            // The mixer is a function of the symbol: from `from` on it turns
            // at the new rate, continuing the phase.
            let a = r.abs0 + from as u64;
            let th = wrap(r.phase(a));
            r.prev = Some(r.mix);
            r.mix = Mix { k0: a, th0: th, w: r.mix.w + w };
            return;
        }
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

    /// One frame of ours at `p`; the next header at `next` is ours (`kind`
    /// None) or `others[i]`'s.
    fn frame(&mut self, p: usize, next: usize, kind: Option<usize>, out: &mut Vec<[u8; TS_LEN]>) {
        let rs = self.rs;
        // Header: frequency at increasing lags (+-rs/2, then finer).
        let hw = self.win(p, SLOT);
        let z: Vec<Complex32> = (0..SLOT).map(|i| hw[i] * self.header[i].conj()).collect();
        drop(hw);
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
        let with_next = next + SLOT <= self.nsyms();
        // Ring mode, tracking: per-block sums (the FPGA's, or made from the
        // words), the frequency left inside a block ignored (a few Hz when
        // tracking); (correlation per symbol, power) of each known block.
        let sums = self.sums && !acquiring && self.ring.is_some();
        let mut kb: Vec<(Complex32, f64)> = Vec::new();
        if sums {
            for (b, &(at, len)) in self.known.clone().iter().enumerate() {
                let c = self.known_block(p + at, len, None, (b > 0).then_some(at));
                pts.push(((at + len / 2) as f64, c.0.arg() as f64, c.0.norm() * len as f32));
                kb.push(c);
            }
            if with_next {
                let q = kind.map(|i| super::s2trk::quarters(&self.others[i].header));
                let c = self.known_block(next, SLOT, q.as_deref(), None).0;
                pts.push(((next - p + SLOT / 2) as f64, c.arg() as f64, c.norm() * SLOT as f32));
            }
        } else {
            for &(at, len) in &self.known {
                let c = self.block(p, at, len, f);
                pts.push(((at + len / 2) as f64, c.arg() as f64, c.norm() * len as f32));
            }
            if with_next {
                let h = kind.map_or(&self.header, |i| &self.others[i].header);
                let c = self.block_with(next, 0, SLOT, f, h);
                pts.push(((next - p + SLOT / 2) as f64, c.arg() as f64, c.norm() * SLOT as f32));
            }
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
        if sums {
            // from the block sums: amp = mean Re(c e^(-j phase)); noise =
            // mean |s|^2 - amp^2 (the phase constant inside a block)
            let mut pw = 0f64;
            for (&(at, len), &(c, p_b)) in self.known.iter().zip(&kb) {
                let ph = phase_at((at + len / 2) as f64) as f32;
                amp += (c * Complex32::new(ph.cos(), -ph.sin())).re * len as f32;
                pw += p_b;
                n_known += len;
            }
            amp /= n_known as f32;
            noise = (pw as f32 - n_known as f32 * amp * amp).max(0.0);
        }
        // (the known symbols, made once)
        let kw: Vec<Cow<'_, [Complex32]>> = if sums { Vec::new() } else { self.known.iter().map(|&(at, len)| self.win(p + at, len)).collect() };
        for (b, &(at, len)) in self.known.iter().enumerate().filter(|_| !sums) {
            for i in 0..len {
                let k = at + i;
                let ph = phase_at(k as f64) as f32;
                let s = kw[b][i] * Complex32::new(ph.cos(), -ph.sin());
                let r = if at == 0 { self.header[i] } else { super::rotate(Complex32::new(a, a), self.scramble[k - SLOT]) };
                amp += (s * r.conj()).re;
                n_known += 1;
            }
        }
        if !sums {
            amp /= n_known as f32;
        }
        for (b, &(at, len)) in self.known.iter().enumerate().filter(|_| !sums) {
            for i in 0..len {
                let k = at + i;
                let ph = phase_at(k as f64) as f32;
                let s = kw[b][i] * Complex32::new(ph.cos(), -ph.sin());
                let r = if at == 0 { self.header[i] } else { super::rotate(Complex32::new(a, a), self.scramble[k - SLOT]) };
                noise += (s - r * amp).norm_sqr();
            }
        }
        drop(kw);
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
        // Long frames: the data symbols go to the FPGA's LDPC engine as
        // cells (it makes the LLRs; s2cells is its model); short frames:
        // float LLRs here.
        let cells = !self.spec.is_short() && !self.float_llr;
        let g = super::s2cells::gain(amp);
        // Ring mode: the engine turns the data symbols itself (s2ring);
        // here only every 32nd, for the MER and the constellation.
        let ring_job = cells && self.ring.is_some();
        if cells && !ring_job {
            self.cells.clear();
        }
        // The phase is linear between known blocks: a rotator stepped once a
        // symbol, set afresh (cos, sin) at each block and after each pilot.
        let mut rot = Complex32::new(1.0, 0.0);
        let mut step = Complex32::new(1.0, 0.0);
        let mut seg_end = 0usize;
        if ring_job && sums {
            // Every 128th data symbol (the MER, the constellation: 169 to
            // 253 points a long frame): per data group one rotator for the
            // mixer less the carrier fit (both linear in a group), stepped
            // SAMPLE symbols at a time.
            const SAMPLE: usize = 128;
            let r = self.ring.as_ref().expect("ring mode");
            let base = r.abs0 + p as u64;
            let glen = if self.spec.pilots { super::s2ring::GROUP } else { nsym };
            for gi in 0..nsym.div_ceil(glen) {
                let n0 = gi * glen;
                let k_s = SLOT + gi * (glen + if self.spec.pilots { PILOT } else { 0 });
                let ab = base + k_s as u64;
                let th = r.phase(ab) - phase_at(k_s as f64);
                let w = (r.phase(ab + 1) - r.phase(ab)) - (phase_at(k_s as f64 + 1.0) - phase_at(k_s as f64));
                let mut n = n0.div_ceil(SAMPLE) * SAMPLE;
                let a0 = th + w * (n - n0) as f64;
                let mut rot = Complex32::new(a0.cos() as f32, a0.sin() as f32);
                let sw = SAMPLE as f64 * w;
                let st = Complex32::new(sw.cos() as f32, sw.sin() as f32);
                while n < (n0 + glen).min(nsym) {
                    let k = k_s + (n - n0);
                    let s = RingRx::raw_sym(r.raw[p + k]) * rot;
                    rot *= st;
                    let d = super::rotate(s, (4 - self.scramble[k - SLOT]) & 3);
                    let dec = if self.spec.bps == 2 {
                        Complex32::new(amp * a * d.re.signum(), amp * a * d.im.signum())
                    } else {
                        let best = (0..8).min_by(|&x, &y| (d - psk8[x] * amp).norm_sqr().total_cmp(&(d - psk8[y] * amp).norm_sqr())).unwrap();
                        psk8[best] * amp
                    };
                    dd_sig += dec.norm_sqr();
                    dd_err += (d - dec).norm_sqr();
                    let q = |v: f32| (v * unit).round().clamp(-127.0, 127.0) as i8;
                    self.constellation.push([q(d.re), q(d.im)]);
                    n += SAMPLE;
                }
            }
            n = nsym;
        }
        while n < nsym {
            if pilot < self.known.len() && k == self.known[pilot].0 {
                k += PILOT;
                pilot += 1;
                seg_end = 0;
                continue;
            }
            if ring_job {
                if n % 32 == 0 {
                    let ph = phase_at(k as f64);
                    let s = self.sym(p + k) * Complex32::new(ph.cos() as f32, -ph.sin() as f32);
                    let d = super::rotate(s, (4 - self.scramble[k - SLOT]) & 3);
                    let dec = if self.spec.bps == 2 {
                        Complex32::new(amp * a * d.re.signum(), amp * a * d.im.signum())
                    } else {
                        let best = (0..8).min_by(|&x, &y| (d - psk8[x] * amp).norm_sqr().total_cmp(&(d - psk8[y] * amp).norm_sqr())).unwrap();
                        psk8[best] * amp
                    };
                    dd_sig += dec.norm_sqr();
                    dd_err += (d - dec).norm_sqr();
                    let q = |v: f32| (v * unit).round().clamp(-127.0, 127.0) as i8;
                    self.constellation.push([q(d.re), q(d.im)]);
                }
                n += 1;
                k += 1;
                continue;
            }
            if k >= seg_end {
                let (p0, p1) = (phase_at(k as f64), phase_at(k as f64 + 1.0));
                rot = Complex32::new(p0.cos() as f32, -p0.sin() as f32);
                let dp = (p1 - p0) as f32;
                step = Complex32::new(dp.cos(), -dp.sin());
                seg_end = pts.iter().map(|pt| pt.0).find(|&b| b > k as f64).map_or(usize::MAX, |b| b.ceil() as usize);
            }
            let s = self.sym(p + k) * rot;
            rot *= step;
            let d = super::rotate(s, (4 - self.scramble[k - SLOT]) & 3);
            if cells {
                self.cells.push(super::s2cells::cell(d, g));
            }
            let dec = if self.spec.bps == 2 {
                if !cells {
                    self.llr[2 * n] = scale * d.re;
                    self.llr[2 * n + 1] = scale * d.im;
                }
                Complex32::new(amp * a * d.re.signum(), amp * a * d.im.signum())
            } else if cells {
                // The decision only for the MER (every 4th symbol: the
                // nearest of 8 points is most of the work left here).
                if n % 4 != 0 {
                    n += 1;
                    k += 1;
                    continue;
                }
                let best = (0..8).min_by(|&x, &y| (d - psk8[x] * amp).norm_sqr().total_cmp(&(d - psk8[y] * amp).norm_sqr())).unwrap();
                psk8[best] * amp
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
        if sums {
            self.trk_commands(p, next, kind);
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
        let mut kq = super::s2cells::kq(self.spec.bps, amp, sigma2);
        // The ring job: per data group the angle the engine turns its first
        // symbol by (the AFC's mixer less the carrier fit, both linear in
        // the group) and the step a symbol.
        let job = if ring_job {
            let r = self.ring.as_ref().expect("ring mode");
            let base = r.abs0 + p as u64;
            let groups = if self.spec.pilots { nsym.div_ceil(super::s2ring::GROUP) } else { 1 };
            let segs: Vec<(u32, u32)> = (0..groups)
                .map(|gi| {
                    let k_s = SLOT + gi * (super::s2ring::GROUP + PILOT);
                    let ab = base + k_s as u64;
                    let th = r.phase(ab) - phase_at(k_s as f64);
                    let w = (r.phase(ab + 1) - r.phase(ab)) - (phase_at(k_s as f64 + 1.0) - phase_at(k_s as f64));
                    (super::s2ring::angle(th), super::s2ring::step(w))
                })
                .collect();
            let (gq, g_eff) = super::s2ring::gain(g);
            kq = super::s2cells::kq_at(self.spec.bps, amp, sigma2, g_eff);
            let fsym = super::s2ring::frame_symbols(nsym, self.spec.pilots);
            Some(Box::new(super::s2ring::Job {
                at: base + SLOT as u64,
                n_cells: nsym,
                pilots: self.spec.pilots,
                segs,
                gain: gq,
                kq,
                bps: self.spec.bps as u8,
                words: (!r.engine).then(|| r.raw[p + SLOT..p + SLOT + fsym].to_vec()),
                watch: r.watch.clone(),
            }))
        } else {
            None
        };
        let fails = match &mut self.fec {
            FecMode::Inline(f) if cells => {
                let ok = |f: &mut Fec, st: &mut Stats, out: &mut Vec<[u8; TS_LEN]>, cells: &[[i8; 2]]| match &job {
                    Some(j) => f.frame_ring(j, st, out),
                    None => f.frame_cells(cells, &super::s2cells::params(self.spec.bps, kq), st, out),
                };
                if hopeless {
                    self.stats.frames_bad += 1;
                    self.stats.frames_skipped += 1;
                    f.lost();
                    self.fails + 1
                } else if ok(f, &mut self.stats, out, &self.cells) {
                    0
                } else {
                    self.fails + 1
                }
            }
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
                    let _ = tx.try_send(S2Block::lost());
                } else if tx
                    .try_send(match job {
                        Some(j) => S2Block::Ring(j),
                        None if cells => S2Block::Cells(self.cells.clone(), kq, self.spec.bps as u8),
                        None => S2Block::Llr(self.llr.clone()),
                    })
                    .is_err()
                {
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
        self.deframe_bytes(&bb, stats, out)
    }

    /// One BBFRAME (descrambled, Kbch / 8 bytes): TS packets out.
    fn deframe_bytes(&mut self, bb: &[u8], stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) -> bool {
        let n = self.spec.kbch / 8;
        if crc8(&bb[..9]) != bb[9] {
            stats.frames_bad += 1;
            stats.crc_fail += 1;
            self.have_prev = false;
            return false;
        }
        // MATYPE-1 (EN 302 307-1 5.1.6): TS, single input stream, no ISSY,
        // no null-packet deletion (CCM/ACM and roll-off free); UPL 188 bytes.
        // Anything else is a stream this deframer would misparse.
        let upl = u16::from_be_bytes([bb[2], bb[3]]);
        let dfl_bits = u16::from_be_bytes([bb[4], bb[5]]) as usize;
        let syncd_bits = u16::from_be_bytes([bb[7], bb[8]]);
        let reason = if bb[0] & 0xEC != 0xE0 {
            Some("MATYPE")
        } else if upl as usize != TS_LEN * 8 {
            Some("UPL")
        } else if dfl_bits % 8 != 0 || dfl_bits / 8 > n - BBHEADER {
            Some("DFL")
        } else if syncd_bits != 0xFFFF && (syncd_bits % 8 != 0 || syncd_bits as usize > dfl_bits) {
            Some("SYNCD")
        } else {
            None
        };
        if let Some(what) = reason {
            stats.bb_unsupported += 1;
            if !self.matype_warned {
                self.matype_warned = true;
                tracing::warn!(matype = format!("{:02x} {:02x}", bb[0], bb[1]), upl, dfl = dfl_bits, syncd = syncd_bits, "DATV: BBFRAME {what} not supported (TS, single stream, no ISSY/NPD only): not taken");
            }
            self.partial.clear();
            self.have_prev = false;
            // Decoded fine (the carrier is right): not a failure to re-acquire on.
            return true;
        }
        let data = &bb[BBHEADER..BBHEADER + dfl_bits / 8];
        if syncd_bits == 0xFFFF {
            // No packet starts in this data field: all of it continues one.
            if self.have_prev && self.partial.len() + data.len() < TS_LEN {
                self.partial.extend_from_slice(data);
            } else {
                self.partial.clear();
                self.have_prev = false;
            }
            return true;
        }
        let syncd = syncd_bits as usize / 8;
        // The tail of a packet that began in the previous frame; its CRC-8 is
        // in the next packet's first byte (when that is in this frame).
        if self.have_prev && syncd <= data.len() && self.partial.len() + syncd == TS_LEN {
            let mut up = std::mem::take(&mut self.partial);
            up.extend_from_slice(&data[..syncd]);
            self.emit(&up, data.get(syncd).copied(), stats, out);
        }
        self.partial.clear();
        let mut i = syncd;
        while i + TS_LEN <= data.len() {
            self.emit(&data[i..i + TS_LEN], data.get(i + TS_LEN).copied(), stats, out);
            i += TS_LEN;
        }
        self.partial.extend_from_slice(&data[i.min(data.len())..]);
        self.have_prev = true;
        true
    }

    /// One user packet: its first byte is the CRC-8 of the packet before;
    /// `check` (the next packet's first byte, when known) is this one's. A
    /// mismatch sets the transport_error_indicator (TS packets passed on as
    /// a demodulator does, EN 302 307-1 5.1.4).
    fn emit(&mut self, up: &[u8], check: Option<u8>, stats: &mut Stats, out: &mut Vec<[u8; TS_LEN]>) {
        let mut pkt = [0u8; TS_LEN];
        pkt.copy_from_slice(up);
        pkt[0] = 0x47;
        if check.is_some_and(|c| c != crc8(&pkt[1..])) {
            pkt[1] |= 0x80;
            stats.ts_crc_bad += 1;
        }
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
            crate::dvbt2::stream::T2Block::Ddr(a, p, tag) => {
                // The router reuses its buffers about a second later: a
                // block whose buffer was taken before or while it was
                // decoded is lost, not decoded from another frame's cells.
                let lapped = |st: &mut Stats| {
                    st.frames_bad += 1;
                    st.blocks_lapped += 1;
                };
                if tag.as_ref().is_some_and(|t| !t.holds()) {
                    lapped(st);
                    fec.lost();
                    return false;
                }
                let n0 = ts.len();
                let ok = fec.frame_ddr(*a, p, st, ts);
                if tag.as_ref().is_some_and(|t| !t.holds()) {
                    ts.truncate(n0);
                    lapped(st);
                    fec.lost();
                    return false;
                }
                ok
            }
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
            // this thread's CPU time (the shares above are wall time, the
            // engine's decode included, which this thread sleeps through)
            let cpu_s = || {
                let mut t = libc::timespec { tv_sec: 0, tv_nsec: 0 };
                // SAFETY: a valid timespec out-pointer.
                unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut t) };
                t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
            };
            let cpu_base = cpu_s();
            let mut cpu0 = cpu_base;
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
                // The FPGA decoder takes under a millisecond an iteration:
                // a DVB-T2 frame's 9 blocks arrive at once, and the queue
                // rule for the software decoders cut them to 12 (276 of
                // 730 blocks unconverged at 1.7 MHz, BCH cleaning up).
                fec.dec.set_max_iter(if fec.dec.is_fpga() {
                    match frx.len() {
                        0..=12 => 50,
                        13..=24 => 30,
                        _ => 16,
                    }
                } else {
                    match frx.len() {
                        0..=1 => 50,
                        2..=7 => 20,
                        8..=19 => 12,
                        _ => 8,
                    }
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
                        // iterations a decode, and decodes that did not converge (at the limit)
                        iters = {
                            use super::fpga_ldpc::PROF_ITER;
                            let n = PROF_ITER[0].swap(0, Ordering::Relaxed).max(1);
                            format!("{:.1}", PROF_ITER[1].swap(0, Ordering::Relaxed) as f64 / n as f64)
                        },
                        unconverged = super::fpga_ldpc::PROF_ITER[2].swap(0, Ordering::Relaxed),
                        bits_out_ms = format!("{:.2}", ms(&PROF_NS[2])),
                        bch_ms = format!("{:.2}", ms(&FEC_PROF_NS[0])),
                        deframe_ms = format!("{:.2}", ms(&FEC_PROF_NS[1])),
                        demux_ms = format!("{:.2}", dmx_ns as f64 / per / 1e6),
                        blocks = nblk,
                        cpu_pct = format!("{:.1}", 100.0 * (cpu_s() - cpu0) / t_prof.elapsed().as_secs_f64()),
                        "DATV FEC per block"
                    );
                    cpu0 = cpu_s();
                    dmx_ns = 0;
                    nblk = 0;
                    t_prof = std::time::Instant::now();
                }
                st.fec_cpu_s = cpu_s() - cpu_base;
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
        let (ftx, frx) = crossbeam_channel::bounded::<S2Block>(48);
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
                // Ring mode (stage 2, s2ring): raw words from an absolute
                // position (None: the reader was lapped, words lost).
                // (with the known-symbol accumulator's entries read with them)
                type Ents = Vec<[u32; super::s2trk::ENTRY_WORDS]>;
                let (wtx, wrx) = crossbeam_channel::bounded::<(Option<u64>, Vec<u32>, Ents)>(800);
                // the receiver's commands for the accumulator
                let (ctx_trk, crx_trk) = crossbeam_channel::bounded::<super::s2trk::Ctl>(64);
                type Ring = Option<(u64, std::sync::Arc<super::fpga::RingWatch>, bool)>;
                let (ftx_fs, frx_fs) = crossbeam_channel::bounded::<(f64, bool, bool, Ring)>(1);
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
                        // The engine reads long frames from the ring itself
                        // when it can (and the ring tells absolute positions).
                        let ring_mode = fe.symbols() && fe.flagged() && fe.wraps_hw() && !p.is_short() && super::fpga_ldpc::ring_available();
                        let ring = if ring_mode {
                            let mut first = Vec::new();
                            let at = fe.read_raw(&mut first).unwrap_or(0) + first.len() as u64;
                            fe.watch().map(|w| (at, std::sync::Arc::new(w), fe.s2trk()))
                        } else {
                            None
                        };
                        let ring_mode = ring.is_some();
                        let _ = ftx_fs.send((fe.fs_out(), fe.symbols(), fe.flagged(), ring));
                        let (mut reported, mut late_reported) = (false, false);
                        let mut last = std::time::Instant::now();
                        while !st2.load(Ordering::Relaxed) {
                            std::thread::sleep(std::time::Duration::from_millis(5));
                            fe.set_center(f64::from_bits(fc.load(Ordering::Relaxed)));
                            if ring_mode {
                                for c in crx_trk.try_iter() {
                                    fe.trk_ctl(&c);
                                }
                                // the entries first: those of blocks whose
                                // words come now or came before
                                let mut ents = Vec::new();
                                fe.read_trk(&mut ents);
                                let mut words = Vec::new();
                                let at = fe.read_raw(&mut words);
                                last = std::time::Instant::now();
                                if !reported && fe.dropped() {
                                    reported = true;
                                    dr2.fetch_add(1, Ordering::Relaxed);
                                    tracing::warn!("DATV: the FPGA recorder dropped samples");
                                }
                                if (at.is_none() || !words.is_empty() || !ents.is_empty()) && wtx.try_send((at, words, ents)).is_err() {
                                    dr2.fetch_add(1, Ordering::Relaxed);
                                }
                                continue;
                            }
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
                let Ok((fs_out, symbols, flagged, ring_rx)) = frx_fs.recv() else {
                    return;
                };
                tracing::info!(fs = fs_out, symbols, flagged, ring = ring_rx.is_some(), "DATV receive through the FPGA DDC");
                let mut ring_at = ring_rx.as_ref().map(|r| r.0);
                let mut r = match ring_rx {
                    Some((at, watch, trk)) => {
                        let mut r = Receiver::new_ring_spec(p, sr, at, true, Some(watch));
                        r.set_trk(trk);
                        r
                    }
                    None if symbols => Receiver::new_symbols_spec(p, sr, 0.0),
                    None => Receiver::new_prefiltered_spec(p, fs_out, sr, 0.0),
                };
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
                    if let Some(at) = ring_at.as_mut() {
                        match wrx.recv_timeout(std::time::Duration::from_millis(20)) {
                            Ok((pos, words, ents)) => {
                                r.push_trk(&ents);
                                // a lap: the next words start at the DMA (pos)
                                if let Some(pos) = pos {
                                    *at = pos;
                                } else {
                                    continue;
                                }
                                r.process_ring(*at, &words, &mut none);
                                *at += words.len() as u64;
                                for c in r.trk_ctl() {
                                    let _ = ctx_trk.try_send(c);
                                }
                                publish(&r, &sh, &mut seen);
                            }
                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                        }
                        continue;
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
                let (ftx_fs, frx_fs) = crossbeam_channel::bounded::<(f64, bool, bool)>(1);
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
                        let _ = ftx_fs.send((fe.fs_out(), fe.t2_fe(), fe.t2_hw()));
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
                let Ok((fs, t2_fe, t2_hw)) = frx_fs.recv() else {
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
                // the FPGA's P1 / GI / MER reports (searching and tracking
                // without raw samples, the frame's end with its MER)
                d.hw = t2_fe && t2_hw;
                tracing::info!(cells = d.cells_out, qam16 = d.cells16_out, router, reports = d.hw, "DVB-T2 LLRs in the FPGA");
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
                                tracing::info!(demod_cpu = format!("{:.1} %", 100.0 * busy_s / t_log.elapsed().as_secs_f64()), mer_db = d.stats.mer_db, freq_hz = d.stats.freq_hz, frames = d.stats.frames, gaps = d.stats.gaps, p1_missed = d.stats.p1_missed, resched = d.stats.resched, retunes = d.stats.retunes, l1_ok_unchecked_failed = ?(d.stats.l1_ok, d.stats.l1_unchecked, d.stats.l1_failed), router = ?d.router_counters(), router_missed = d.router_missed, prof_ms = ?d.prof.map(|x| (x * 1e3 / (d.stats.frames - prof_f0).max(1) as f64 * 10.0).round() / 10.0), "DVB-T2 demodulator");
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
        st.bb_unsupported = f.bb_unsupported;
        st.ts_crc_bad = f.ts_crc_bad;
        st.blocks_lapped = f.blocks_lapped;
        st.ldpc_s = f.ldpc_s;
        st.fec_cpu_s = f.fec_cpu_s;
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

    /// Dummy PLFRAMEs and a frame of another MODCOD between ours (5.5.1,
    /// VCM): stepped over by their PLS, lock held, every packet of ours
    /// back except the one straddling the foreign frame.
    #[test]
    fn steps_over_dummy_and_other_modcod_frames() {
        use super::super::fpga_tx::LongMode;
        use super::super::{pl_scrambling, plheader_typed, rotate};
        let spec = FrameSpec::long(LongMode::Qpsk12);
        let l = spec.frame_symbols();
        let mut next = counter_packets();
        let ours = long_symbols(LongMode::Qpsk12, 9, &mut next);
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let dummy = {
            let mut v = plheader_typed(0, false, false);
            let scr = pl_scrambling(36 * SLOT);
            v.extend((0..36 * SLOT).map(|k| rotate(Complex32::new(a, a), scr[k])));
            assert_eq!(v.len(), plframe_len(0, false, false).unwrap());
            v
        };
        let other = {
            // QPSK 3/5 short with pilots: random data, right length.
            let len = plframe_len(5, true, true).unwrap();
            let mut v = plheader_typed(5, true, true);
            let mut x = 7u32;
            while v.len() < len {
                x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                v.push(Complex32::new(if x & 0x10000 != 0 { a } else { -a }, if x & 0x20000 != 0 { a } else { -a }));
            }
            v
        };
        // ours x3, dummy, ours, dummy x2, ours, other, ours x4
        let mut syms = Vec::new();
        let mut f = ours.chunks(l);
        for _ in 0..3 {
            syms.extend_from_slice(f.next().unwrap());
        }
        syms.extend_from_slice(&dummy);
        syms.extend_from_slice(f.next().unwrap());
        syms.extend_from_slice(&dummy);
        syms.extend_from_slice(&dummy);
        syms.extend_from_slice(f.next().unwrap());
        syms.extend_from_slice(&other);
        for c in f {
            syms.extend_from_slice(c);
        }
        // a little noise and a carrier phase
        let mut seed = 3u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.3
        };
        let rot = Complex32::from_polar(1.0, 0.7);
        let x: Vec<Complex32> = syms.iter().map(|z| z * rot + Complex32::new(g(), g())).collect();
        let mut rx = Receiver::new_symbols_spec(spec, 250e3, 0.0);
        let mut out = Vec::new();
        for c in x.chunks(4096) {
            rx.process(c, &mut out);
        }
        let s = rx.stats;
        assert!(s.locked, "{s:?}");
        assert_eq!((s.frames_dummy, s.frames_other), (3, 1), "{s:?}");
        let data: Vec<u32> = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).map(|p| u32::from_be_bytes(p[1..5].try_into().unwrap())).collect();
        let gaps: Vec<u32> = data.windows(2).map(|w| w[1] - w[0]).filter(|&d| d != 1).collect();
        assert_eq!(gaps, vec![2], "packets {} first {:?} {s:?}", data.len(), data.first());
        // 9 frames of ours, about 21.4 packets each: the first two or three
        // go to acquisition.
        assert!(data.len() >= 6 * 21, "{} packets {s:?}", data.len());
        assert_eq!(s.ts_crc_bad, 0);
    }

    /// MATYPE: a BBFRAME that is not TS / single stream / no ISSY / no NPD
    /// is not taken (counted), the stream continues after it.
    #[test]
    fn rejects_unsupported_matype() {
        use super::super::fpga_tx::LongMode;
        use super::super::{Framer, bb_scrambling};
        let spec = FrameSpec::long(LongMode::Qpsk12);
        let mut fec = Fec::new(spec);
        let mut framer = Framer::new();
        let mut next = counter_packets();
        let bbscr = bb_scrambling(spec.kbch / 8);
        let mut st = Stats::default();
        let mut out = Vec::new();
        for (i, npd) in [false, true, false].into_iter().enumerate() {
            let mut bb = framer.frame_bytes(spec.kbch / 8, 0, &mut next);
            if npd {
                bb[0] |= 0x04;
                bb[9] = crc8(&bb[..9]);
            }
            for (k, byte) in bb.iter().enumerate() {
                for b in 0..8 {
                    fec.bits[k * 8 + b] = ((byte ^ bbscr[k]) >> (7 - b)) & 1;
                }
            }
            assert!(fec.deframe(&mut st, &mut out), "frame {i}");
        }
        assert_eq!(st.bb_unsupported, 1);
        // first frame's packets, then the third's (its first whole one on)
        assert!(out.len() >= 2 * 20, "{}", out.len());
        // a corrupted packet byte: the TEI set on it
        let mut bb = framer.frame_bytes(spec.kbch / 8, 0, &mut next);
        bb[BBHEADER + 300] ^= 0x10;
        for (k, byte) in bb.iter().enumerate() {
            for b in 0..8 {
                fec.bits[k * 8 + b] = ((byte ^ bbscr[k]) >> (7 - b)) & 1;
            }
        }
        out.clear();
        fec.deframe(&mut st, &mut out);
        assert_eq!(st.ts_crc_bad, 1);
        assert_eq!(out.iter().filter(|p| p[1] & 0x80 != 0).count(), 1);
    }

    /// Decoding margin of the LDPC engine's cells (s2cells) against the
    /// float LLRs they replaced: the same noisy frames through both, near
    /// each mode's threshold. Run by hand:
    /// cargo test --release cells_margin -- --ignored --nocapture
    #[test]
    #[ignore]
    fn cells_margin() {
        use super::super::fpga_tx::LongMode;
        let frames = std::env::var("MARGIN_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(40usize);
        for (mode, snrs) in [(LongMode::Qpsk12, [0.6f32, 1.0, 1.4]), (LongMode::Qpsk34, [3.6, 4.0, 4.4]), (LongMode::Psk8_34, [7.4, 7.8, 8.2])] {
            let spec = FrameSpec::long(mode);
            let mut next = counter_packets();
            let clean = long_symbols(mode, frames, &mut next);
            for esn0_db in snrs {
                let sigma = (0.5 / 10f32.powf(esn0_db / 10.0)).sqrt();
                let mut seed = 99u64 + (esn0_db * 10.0) as u64;
                let mut g = || {
                    seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                    let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
                    seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                    let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
                    ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
                };
                let noisy: Vec<Complex32> = clean.iter().map(|&z| z + Complex32::new(sigma * g(), sigma * g())).collect();
                let mut res = Vec::new();
                for float in [true, false] {
                    let mut rx = Receiver::new_symbols_spec(spec, 250e3, 0.0);
                    rx.float_llr = float;
                    let mut out = Vec::new();
                    for c in noisy.chunks(8192) {
                        rx.process(c, &mut out);
                    }
                    res.push((rx.stats.frames.saturating_sub(rx.stats.frames_bad), rx.stats.frames, out.len()));
                }
                println!("{mode:?} Es/N0 {esn0_db:.1} dB: float {}/{} frames ({} packets), cells {}/{} ({} packets)", res[0].0, res[0].1, res[0].2, res[1].0, res[1].1, res[1].2);
            }
        }
    }

    /// The demodulator's own time a frame (the decoder thread excluded:
    /// its frames go to a channel), float LLRs against cells. By hand:
    /// cargo test --release demod_time -- --ignored --nocapture
    #[test]
    #[ignore]
    fn demod_time() {
        use super::super::fpga_tx::LongMode;
        for mode in [LongMode::Qpsk12, LongMode::Psk8_34] {
            let spec = FrameSpec::long(mode);
            let mut next = counter_packets();
            let syms: Vec<Complex32> = long_symbols(mode, 30, &mut next).iter().map(|&z| z * Complex32::new(0.9, 0.2)).collect();
            for float in [true, false] {
                let (tx, rx_c) = crossbeam_channel::unbounded::<S2Block>();
                let mut rx = Receiver::new_symbols_spec(spec, 250e3, 0.0);
                rx.float_llr = float;
                rx.fec = FecMode::Thread { tx, fails: std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0)) };
                let mut out = Vec::new();
                let t0 = std::time::Instant::now();
                for c in syms.chunks(8192) {
                    rx.process(c, &mut out);
                }
                let dt = t0.elapsed().as_secs_f64();
                let n = rx_c.len().max(1);
                println!("{mode:?} {}: {:.2} ms a frame ({n} frames)", if float { "float" } else { "cells" }, 1e3 * dt / n as f64);
            }
        }
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
    /// Ring words as the FPGA leaves them: symbols at about -14 dBFS, a
    /// carrier offset, noise, the header detector's flag in bit 16.
    fn ring_words(mode: super::super::fpga_tx::LongMode, esn0_db: f32, frames: usize, off_hz: f64, rs: f64) -> Vec<u32> {
        let mut next = counter_packets();
        let syms = long_symbols(mode, frames, &mut next);
        let sigma = (1.0 / 10f32.powf(esn0_db / 10.0) / 2.0).sqrt();
        let mut seed = 23u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        let mut det = super::super::hdrdet::HdrDet::default();
        let q = |v: f32| (v * 6000.0).round().clamp(-32768.0, 32767.0) as i16;
        // a little noise first, so the receiver searches
        let lead: Vec<Complex32> = (0..5000).map(|_| Complex32::new(sigma * g(), sigma * g())).collect();
        lead.iter()
            .chain(syms.iter())
            .enumerate()
            .map(|(k, &z)| {
                let ph = std::f64::consts::TAU * off_hz * k as f64 / rs;
                let y = z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g());
                let y = [q(y.re), q(y.im)];
                let f = det.push(y);
                let y = super::super::hdrdet::HdrDet::mark(y, f);
                (y[0] as u16 as u32) | (y[1] as u16 as u32) << 16
            })
            .collect()
    }

    /// Ring mode (the engine reading the ring; here its model, s2ring):
    /// every packet back, in order, with the carrier off and drifting
    /// through the AFC's retunes.
    fn ring_link(mode: super::super::fpga_tx::LongMode, esn0_db: f32, frames: usize) -> (Stats, usize, f64) {
        let rs = 250e3;
        let words = ring_words(mode, esn0_db, frames, 900.0, rs);
        let mut rx = Receiver::new_ring_spec(FrameSpec::long(mode), rs, 1_000, false, None);
        let mut out = Vec::new();
        let mut at = 1_000u64;
        let t0 = std::time::Instant::now();
        for c in words.chunks(4096) {
            rx.process_ring(at, c, &mut out);
            at += c.len() as u64;
        }
        let dt = t0.elapsed().as_secs_f64();
        let data: Vec<_> = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).collect();
        if let Some(first) = data.first() {
            let f0 = u32::from_be_bytes(first[1..5].try_into().unwrap());
            for (i, pkt) in data.iter().enumerate() {
                let k = f0 + i as u32;
                assert_eq!(u32::from_be_bytes(pkt[1..5].try_into().unwrap()), k, "{mode:?}: packet {i} of {}; {:?}", data.len(), rx.stats);
                assert!(pkt[5..].iter().enumerate().all(|(j, b)| *b == (k as usize * 7 + j) as u8), "{mode:?}: packet {i} payload");
            }
        }
        (rx.stats, data.len(), dt)
    }

    /// With the FPGA's known-symbol accumulator (its model, s2trk::Unit):
    /// it sees the words as they come, the receiver's commands reach it a
    /// chunk late; once it follows the frames the receiver takes its
    /// entries (with the AFC retuning the mixer through it) and decodes
    /// every frame as without it.
    #[test]
    fn ring_with_the_fpga_accumulator() {
        use super::super::fpga_tx::LongMode;
        for mode in [LongMode::Qpsk12, LongMode::Psk8_34] {
            let rs = 250e3;
            let words = ring_words(mode, 8.0, 24, 900.0, rs);
            let mut rx = Receiver::new_ring_spec(FrameSpec::long(mode), rs, 1_000, false, None);
            rx.set_trk(true);
            let mut unit = super::super::s2trk::Unit::new(1_000);
            let mut out = Vec::new();
            let mut at = 1_000u64;
            let mut late = Vec::new();
            for c in words.chunks(4096) {
                for cmd in late.drain(..) {
                    unit.ctl(&cmd);
                }
                unit.push(c);
                let ents = std::mem::take(&mut unit.entries);
                rx.push_trk(&ents);
                rx.process_ring(at, c, &mut out);
                at += c.len() as u64;
                late = rx.trk_ctl();
            }
            let s = &rx.stats;
            eprintln!("{mode:?}: frames {} bad {} esn0 {:.1} dB, blocks from the unit {} / made {}", s.frames, s.frames_bad, s.esn0_db, s.trk_hw, s.trk_model);
            assert!(s.locked && s.frames >= 20 && s.ldpc_fail == 0, "{s:?}");
            // all but the first two tracked frames' blocks from the unit (it
            // follows from the frame after the load)
            assert!(s.trk_hw >= 2 * s.trk_model, "{} / {}", s.trk_hw, s.trk_model);
        }
    }

    #[test]
    fn ring_frames_8psk_3_4() {
        let (s, n, _) = ring_link(super::super::fpga_tx::LongMode::Psk8_34, 12.0, 20);
        assert!(s.locked, "{s:?}");
        assert!(n >= 17 * 32 && s.ldpc_fail == 0, "{n} packets, {s:?}");
    }

    #[test]
    fn ring_frames_qpsk() {
        for mode in [super::super::fpga_tx::LongMode::Qpsk12, super::super::fpga_tx::LongMode::Qpsk34] {
            let (s, n, _) = ring_link(mode, 7.0, 14);
            assert!(s.locked, "{mode:?} {s:?}");
            assert!(n >= 10 * 20 && s.ldpc_fail == 0, "{mode:?}: {n} packets, {s:?}");
        }
    }

    /// Ring mode and the symbol path (stage 1) on the same noisy words:
    /// the same frames decode (no margin lost to the fixed-point turn).
    #[test]
    fn ring_matches_the_symbol_path() {
        use super::super::fpga_tx::LongMode;
        for (mode, esn0) in [(LongMode::Psk8_34, 8.2), (LongMode::Psk8_34, 7.7), (LongMode::Qpsk12, 1.4), (LongMode::Qpsk12, 1.0), (LongMode::Qpsk34, 4.0)] {
            let rs = 250e3;
            let words = ring_words(mode, esn0, 30, 400.0, rs);
            let spec = FrameSpec::long(mode);
            let mut a = Receiver::new_ring_spec(spec, rs, 0, false, None);
            let mut b = Receiver::new_symbols_spec(spec, rs, 0.0);
            let (mut oa, mut ob) = (Vec::new(), Vec::new());
            let mut at = 0u64;
            for c in words.chunks(4096) {
                a.process_ring(at, c, &mut oa);
                at += c.len() as u64;
                let syms: Vec<Complex32> = c.iter().map(|&w| RingRx::raw_sym(w)).collect();
                let flags: Vec<bool> = c.iter().map(|w| w & 0x1_0000 != 0).collect();
                b.process_flagged(&syms, Some(&flags), &mut ob);
            }
            let good = |s: &Stats| s.frames - s.frames_bad;
            println!("{mode:?} at {esn0} dB: ring {}/{} frames, symbols {}/{}", good(&a.stats), a.stats.frames, good(&b.stats), b.stats.frames);
            assert!(good(&a.stats) + 1 >= good(&b.stats), "ring {:?}\nsymbols {:?}", a.stats, b.stats);
        }
    }

    /// A gap in the ring (the reader lapped): lock is lost, then found again.
    #[test]
    fn ring_survives_a_gap() {
        use super::super::fpga_tx::LongMode;
        let rs = 250e3;
        let words = ring_words(LongMode::Qpsk12, 8.0, 24, 200.0, rs);
        let mut rx = Receiver::new_ring_spec(FrameSpec::long(LongMode::Qpsk12), rs, 0, false, None);
        let mut out = Vec::new();
        let cut = words.len() / 3;
        rx.process_ring(0, &words[..cut], &mut out);
        let before = out.len();
        rx.process_ring((cut + 70_000) as u64, &words[cut + 70_000..], &mut out);
        assert!(before > 0 && out.len() > before + 100, "{before} then {}; {:?}", out.len(), rx.stats);
        assert!(rx.stats.locked && rx.stats.blocks_dropped == 1, "{:?}", rx.stats);
    }

    /// The receiver's own time per frame, ring mode against the symbol
    /// path (cargo test --release ring_demod_time -- --ignored --nocapture).
    #[test]
    #[ignore]
    fn ring_demod_time() {
        use super::super::fpga_tx::LongMode;
        for mode in [LongMode::Qpsk12, LongMode::Psk8_34] {
            let rs = 250e3;
            let words = ring_words(mode, 20.0, 30, 300.0, rs);
            let spec = FrameSpec::long(mode);
            let (tx, rx_c) = crossbeam_channel::unbounded::<S2Block>();
            let mut a = Receiver::new_ring_spec(spec, rs, 0, true, None);
            a.fec = FecMode::Thread { tx, fails: std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0)) };
            let mut out = Vec::new();
            let t0 = std::time::Instant::now();
            let mut at = 0u64;
            for c in words.chunks(8192) {
                a.process_ring(at, c, &mut out);
                at += c.len() as u64;
            }
            let ta = t0.elapsed().as_secs_f64() / rx_c.len().max(1) as f64;
            let (tx, rx_c) = crossbeam_channel::unbounded::<S2Block>();
            let mut b = Receiver::new_symbols_spec(spec, rs, 0.0);
            b.fec = FecMode::Thread { tx, fails: std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0)) };
            let t0 = std::time::Instant::now();
            for c in words.chunks(8192) {
                let syms: Vec<Complex32> = c.iter().map(|&w| RingRx::raw_sym(w)).collect();
                let flags: Vec<bool> = c.iter().map(|w| w & 0x1_0000 != 0).collect();
                b.process_flagged(&syms, Some(&flags), &mut out);
            }
            let tb = t0.elapsed().as_secs_f64() / rx_c.len().max(1) as f64;
            println!("{mode:?}: ring {:.3} ms a frame, symbols (stage 1, conversion included) {:.3} ms", ta * 1e3, tb * 1e3);
        }
    }

    /// The FPGA recorder in a model: a stream of ring words committed in
    /// 128-byte bursts at the symbol rate, simulated time passing between
    /// register reads (the DMA moves while the reader reads).
    struct FakeRing {
        mem: Vec<u32>,
        src: Vec<u32>,
        rate: f64,
        t: f64,
        reg_dt: f64,
        committed: u64,
    }

    impl FakeRing {
        fn new(src: Vec<u32>, rate: f64) -> Self {
            FakeRing { mem: vec![0; super::super::fpga::RING_WORDS as usize], src, rate, t: 0.0, reg_dt: 4e-6, committed: 0 }
        }
        fn advance(&mut self, dt: f64) {
            self.t += dt;
            let want = (((self.t * self.rate) as u64) / 32 * 32).min(self.src.len() as u64 / 32 * 32);
            while self.committed < want {
                let k = self.committed as usize;
                let n = self.mem.len();
                self.mem[k % n] = self.src[k];
                self.committed += 1;
            }
        }
        fn addr(&self) -> u32 {
            super::super::fpga::RING_START + ((self.committed % super::super::fpga::RING_WORDS) * 4) as u32
        }
        fn done(&self) -> bool {
            self.committed >= self.src.len() as u64 / 32 * 32
        }
    }

    impl super::super::fpga::RingHw for FakeRing {
        fn position(&mut self) -> Option<(u64, u32)> {
            self.advance(self.reg_dt);
            Some((self.committed * 4, self.addr()))
        }
        fn copy(&mut self, from: u32, to: u32, out: &mut Vec<u32>) {
            let (s, e) = (((from - super::super::fpga::RING_START) / 4) as usize, ((to - super::super::fpga::RING_START) / 4) as usize);
            out.extend_from_slice(&self.mem[s..e]);
            self.advance((e - s) as f64 * 4.0 / 185e6);
        }
    }

    /// The whole ring path at a real rate: the recorder, the ring reader
    /// thread's reads every 5 ms (some late), the receiver in ring mode, and
    /// the decoder thread taking each job after its queue and decode time,
    /// reading the frame from the ring as the engine would. Every frame
    /// sent must decode and every TS packet come out once, in order.
    fn ring_stream(mode: super::super::fpga_tx::LongMode, rs: f64, frames: usize) -> (usize, usize, u64, Stats) {
        use super::super::fpga::{RingCursor, RING_START, RING_WORDS};
        let words = ring_words(mode, 25.0, frames, 517.0, rs);
        let spec = FrameSpec::long(mode);
        let mut hw = FakeRing::new(words, rs);
        let mut cur = RingCursor { rd: RING_START, total: 0 };
        let (tx, rx_c) = crossbeam_channel::unbounded::<S2Block>();
        let mut rx = Receiver::new_ring_spec(spec, rs, 0, true, None);
        rx.fec = FecMode::Thread { tx, fails: std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0)) };
        let mut fec = Fec::new(spec);
        let mut fst = Stats::default();
        let mut ts = Vec::new();
        let mut none = Vec::new();
        let mut at: Option<u64> = None;
        // the decoder: (job, the time it was queued)
        let mut queue: VecDeque<(S2Block, f64)> = VecDeque::new();
        let mut busy_until = 0.0f64;
        let (mut decoded, mut lost) = (0usize, 0usize);
        let mut n_read = 0u32;
        let mut seed = 3u32;
        loop {
            // the reader thread: 5 ms sleeps, now and then 30 ms late
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            hw.advance(if seed >> 28 == 0 { 0.030 } else { 0.005 });
            let mut buf = Vec::new();
            match cur.read(&mut hw, &mut buf) {
                Some(pos) => {
                    let a = at.get_or_insert(pos);
                    *a = pos;
                    rx.process_ring(pos, &buf, &mut none);
                    *a += buf.len() as u64;
                }
                None => {}
            }
            n_read += 1;
            while let Ok(b) = rx_c.try_recv() {
                queue.push_back((b, hw.t));
            }
            // the decoder: one frame at a time, 12 ms each
            while let Some((_, q_t)) = queue.front() {
                let start = busy_until.max(*q_t);
                if start + 0.012 > hw.t {
                    break;
                }
                let (b, _) = queue.pop_front().unwrap();
                busy_until = start + 0.012;
                match b {
                    S2Block::Ring(mut job) => {
                        let nsym = super::super::s2ring::frame_symbols(job.n_cells, job.pilots) as u64;
                        let ahead = hw.committed - job.at;
                        if super::super::s2ring::in_ring(ahead, nsym) {
                            job.words = Some((0..nsym).map(|k| hw.mem[((job.at + k) % RING_WORDS) as usize]).collect());
                            if fec.frame_ring(&job, &mut fst, &mut ts) {
                                decoded += 1;
                            }
                        } else {
                            lost += 1;
                            fec.lost();
                        }
                    }
                    _ => fec.lost(),
                }
            }
            if hw.done() && n_read > 10 && queue.is_empty() && rx_c.is_empty() {
                // drain what is left in the ring
                let mut buf = Vec::new();
                if let Some(pos) = cur.read(&mut hw, &mut buf) {
                    rx.process_ring(pos, &buf, &mut none);
                }
                if rx_c.is_empty() {
                    break;
                }
            }
        }
        let data: Vec<_> = ts.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).collect();
        let mut in_order = 0usize;
        if let Some(first) = data.first() {
            let f0 = u32::from_be_bytes(first[1..5].try_into().unwrap());
            for (i, pkt) in data.iter().enumerate() {
                if u32::from_be_bytes(pkt[1..5].try_into().unwrap()) == f0 + i as u32 {
                    in_order += 1;
                }
            }
        }
        println!("{mode:?} {rs}: {decoded} frames decoded of {frames} sent, {lost} lost to the ring, {} packets ({in_order} in order); receiver {:?}", data.len(), rx.stats);
        let _ = lost;
        (decoded, data.len(), rx.stats.blocks_dropped, rx.stats)
    }

    #[test]
    fn ring_stream_at_real_rates() {
        use super::super::fpga_tx::LongMode;
        for (mode, rs, frames) in [(LongMode::Psk8_34, 500e3, 60), (LongMode::Qpsk12, 250e3, 30), (LongMode::Qpsk34, 33e3, 6)] {
            let (decoded, packets, gaps, st) = ring_stream(mode, rs, frames);
            // all but the frame or two before lock
            assert!(decoded + 2 >= frames, "{mode:?} {rs}: {decoded} of {frames} frames; {st:?}");
            assert_eq!(gaps, 0, "{mode:?} {rs}: the receiver saw gaps in the ring positions");
            let per = (FrameSpec::long(mode).kbch / 8 - 10) as f64 / 188.0;
            assert!(packets as f64 >= (frames - 2) as f64 * per - 2.0, "{mode:?} {rs}: {packets} packets");
        }
    }

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
