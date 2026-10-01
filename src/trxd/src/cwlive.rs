//! Live CW copy of the station the operator is tuned to, as sdroxide's CW panel
//! does it: the demodulated audio's tone at `pitch` above the dial is followed
//! by the classic [`CwRx`] (AFC, speed, SNR) and read by DeepCW as a running
//! transcript. Text arrives about once a second: the settled part is appended
//! for good, the last word or two stays "pending" and firms up in place.
//!
//! It reads the one station under the cursor, letter by letter. The classic
//! decoder's AFC only reaches +/-35 Hz, so a [`Finder`] watches the whole CW
//! filter: when nothing is locked and a keyed tone stands out elsewhere in the
//! passband, the decoder is moved onto it (as the IC-705 copies anything in
//! the filter, not only what sits exactly on the pitch).

use std::sync::{Arc, Mutex};

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use sdroxide_deepcw::{Tuner, Worker};
use sdroxide_dsp::CwRx;
use tracing::warn;

/// Receive text kept for the web UI (a late browser gets the recent past).
const TEXT_CAP: usize = 4000;
/// DeepCW window. sdroxide reads up to 20 s every second, which two Cortex-A9
/// cores cannot do (measured: 130 % CPU and still falling behind); 6 s keeps
/// words in context at 10-30 WPM and the preview still updates every second.
const WINDOW_S: f64 = 6.0;

/// Finder FFT: 8192 points at 12 kHz = 1.5 Hz bins over 0.68 s.
const FIND_N: usize = 8192;
/// Look for a tone this often.
const FIND_HOP_S: f64 = 0.5;
/// A tone this far above the passband's median (per bin, after averaging) is
/// a signal. Keyed CW at a readable SNR stands 25-50 dB out of 1.5 Hz bins.
const FIND_MIN_DB: f32 = 15.0;
/// Two looks this close together are the same tone.
const FIND_SAME_HZ: f32 = 12.0;
/// Closer than this to where the decoder listens: its own AFC pulls it in.
const FIND_AFC_HZ: f32 = 25.0;
/// After a move, give the decoder this long to lock before looking again.
const FIND_SETTLE_S: f64 = 3.0;
/// Stay this far inside the filter edges (their skirts are not signals).
const FIND_EDGE_HZ: f32 = 30.0;

/// Finds the strongest steady tone in the CW passband.
struct Finder {
    rate: f64,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    ring: Vec<f32>,
    pos: usize,
    filled: usize,
    since: usize,
    hop: usize,
    /// Power spectrum, averaged over a few looks (keying comes and goes).
    avg: Vec<f32>,
    buf: Vec<Complex32>,
    band: (f32, f32),
    /// Tone seen at the last look, to require two in a row.
    last: Option<f32>,
    /// Samples left before the next move is allowed.
    settle: usize,
}

impl Finder {
    fn new(rate: f64) -> Self {
        let window = (0..FIND_N)
            .map(|i| 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / FIND_N as f32).cos())
            .collect();
        Finder {
            rate,
            fft: FftPlanner::new().plan_fft_forward(FIND_N),
            window,
            ring: vec![0.0; FIND_N],
            pos: 0,
            filled: 0,
            since: 0,
            hop: (FIND_HOP_S * rate) as usize,
            avg: vec![0.0; FIND_N / 2],
            buf: vec![Complex32::default(); FIND_N],
            band: (300.0, 1_100.0),
            last: None,
            settle: 0,
        }
    }

    fn reset(&mut self) {
        self.filled = 0;
        self.since = 0;
        self.avg.iter_mut().for_each(|x| *x = 0.0);
        self.last = None;
        self.settle = 0;
    }

    /// Feed audio; `Some(hz)` when a tone has stood out twice in a row.
    fn push(&mut self, audio: &[f32]) -> Option<f32> {
        for &a in audio {
            self.ring[self.pos] = if a.is_finite() { a } else { 0.0 };
            self.pos = (self.pos + 1) % FIND_N;
        }
        self.filled = (self.filled + audio.len()).min(FIND_N);
        self.since += audio.len();
        self.settle = self.settle.saturating_sub(audio.len());
        if self.filled < FIND_N || self.since < self.hop {
            return None;
        }
        self.since = 0;
        for i in 0..FIND_N {
            self.buf[i] = Complex32::new(self.ring[(self.pos + i) % FIND_N] * self.window[i], 0.0);
        }
        self.fft.process(&mut self.buf);
        for (a, z) in self.avg.iter_mut().zip(&self.buf) {
            *a = 0.6 * *a + 0.4 * z.norm_sqr();
        }
        let bin = self.rate as f32 / FIND_N as f32;
        // At least bin 1: the peak's interpolation reads the bin below it.
        let lo = (((self.band.0 + FIND_EDGE_HZ) / bin).ceil() as usize).max(1);
        let hi = (((self.band.1 - FIND_EDGE_HZ) / bin).floor() as usize).min(FIND_N / 2 - 2);
        if hi <= lo + 8 {
            return None;
        }
        let mut sorted: Vec<f32> = self.avg[lo..=hi].to_vec();
        sorted.sort_by(f32::total_cmp);
        let median = sorted[sorted.len() / 2].max(1e-20);
        let (k, peak) = (lo..=hi).map(|k| (k, self.avg[k])).max_by(|a, b| a.1.total_cmp(&b.1))?;
        let tone = if 10.0 * (peak / median).log10() >= FIND_MIN_DB {
            // Parabolic interpolation between the neighbouring bins.
            let (a, b, c) = (self.avg[k - 1].sqrt(), peak.sqrt(), self.avg[k + 1].sqrt());
            let d = a - 2.0 * b + c;
            let frac = if d.abs() > 1e-20 { 0.5 * (a - c) / d } else { 0.0 };
            Some((k as f32 + frac.clamp(-0.5, 0.5)) * bin)
        } else {
            None
        };
        let seen = std::mem::replace(&mut self.last, tone);
        match (seen, tone) {
            (Some(a), Some(b)) if (a - b).abs() < FIND_SAME_HZ && self.settle == 0 => Some(b),
            _ => None,
        }
    }
}

pub struct CwLive {
    rx: CwRx,
    /// The operator's pitch (where the finder's moves are undone to on retune).
    pitch: f32,
    finder: Finder,
    tuner: Tuner,
    deep: Option<Worker>,
    scratch: Vec<f32>,
    text: String,
    pending: String,
    /// Text committed since the last [`CwLive::take_committed`].
    fresh: String,
    dirty: bool,
    /// The rain-scatter decoder, when that is the engine ([`crate::rscw`]).
    rs: Option<crate::rscw::RsNnStream>,
    rate: f64,
    band: (f32, f32),
}

/// What the panel shows next to the text.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Readout {
    pub tone_hz: f32,
    pub wpm: f32,
    pub snr_db: f32,
    pub locked: bool,
}

impl CwLive {
    /// `rate`: the audio rate; `pitch_hz`: where the CW tone sits in it.
    /// `neural`: DeepCW reads the text; otherwise the classic timing decoder
    /// (a character as soon as it is complete, a few % of a core).
    pub fn new(rate: f64, pitch_hz: f32, neural: bool) -> Self {
        let deep = if neural { start_deep() } else { None };
        CwLive {
            rx: CwRx::new(rate, pitch_hz),
            pitch: pitch_hz,
            finder: Finder::new(rate),
            tuner: Tuner::new(rate, pitch_hz as f64),
            deep,
            scratch: Vec::new(),
            text: String::new(),
            pending: String::new(),
            fresh: String::new(),
            dirty: false,
            rs: None,
            rate,
            band: (300.0, 2700.0),
        }
    }

    /// The rain-scatter decoder on or off (the timing decoder again when off).
    pub fn set_rs(&mut self, on: bool) {
        if on == self.rs.is_some() {
            return;
        }
        if on {
            self.set_neural(false);
            let mut r = crate::rscw::RsNnStream::new(self.rate as f32);
            r.set_band(self.band.0, self.band.1);
            self.rs = Some(r);
        } else {
            self.rs = None;
        }
        self.pending.clear();
        self.dirty = true;
    }

    /// "rs", "neural" or "timing".
    pub fn engine(&self) -> &'static str {
        if self.rs.is_some() {
            "rs"
        } else if self.deep.is_some() {
            "neural"
        } else {
            "timing"
        }
    }

    /// The CW filter's audio passband: where the finder looks.
    pub fn set_band(&mut self, lo: f32, hi: f32) {
        let (lo, hi) = if lo.is_finite() && hi.is_finite() { (lo, hi) } else { (300.0, 2700.0) };
        self.finder.band = (lo.min(hi), lo.max(hi));
        self.band = self.finder.band;
        if let Some(r) = self.rs.as_mut() {
            r.set_band(self.band.0, self.band.1);
        }
    }

    /// Switch between DeepCW and the timing decoder; the text so far stays.
    pub fn set_neural(&mut self, on: bool) {
        if on {
            self.rs = None;
        }
        if on == self.deep.is_some() {
            return;
        }
        self.deep = if on { start_deep() } else { None };
        self.pending.clear();
        self.tuner.reset();
        self.dirty = true;
    }

    pub fn neural(&self) -> bool {
        self.deep.is_some()
    }

    /// Feed demodulated audio (not while transmitting: we would copy ourselves).
    pub fn process(&mut self, audio: &[f32]) {
        if let Some(r) = self.rs.as_mut() {
            r.process(audio);
            let t = r.take();
            if !t.is_empty() {
                self.append(&t);
            }
            return;
        }
        if let Some(tone) = self.finder.push(audio) {
            if !self.rx.locked() && (tone - self.rx.tone_hz()).abs() > FIND_AFC_HZ {
                self.rx.set_pitch(tone);
                self.finder.settle = (FIND_SETTLE_S * self.finder.rate) as usize;
            }
        }
        let classic = self.rx.process(audio);
        let Some(deep) = self.deep.as_ref() else {
            self.append(&classic);
            return;
        };
        self.tuner.set_tone(self.rx.tone_hz() as f64);
        self.scratch.clear();
        self.tuner.push(audio, &mut self.scratch);
        deep.push(&self.scratch);
    }

    /// Collect what the model finished; true when the display changed.
    pub fn poll(&mut self) -> bool {
        let updates = self.deep.as_ref().map(Worker::poll).unwrap_or_default();
        for u in updates {
            match u {
                Ok(u) => {
                    self.append(&u.committed);
                    if self.pending != u.pending {
                        self.pending = u.pending;
                        self.dirty = true;
                    }
                }
                Err(e) => warn!("live CW: {e}"),
            }
        }
        std::mem::take(&mut self.dirty)
    }

    /// A new station (the operator retuned): settle nothing more of the old
    /// one, and start it on a new line.
    pub fn restart(&mut self) {
        if let Some(d) = self.deep.as_ref() {
            d.reset();
        }
        self.tuner.reset();
        // A new station starts on the pitch again.
        self.rx.set_pitch(self.pitch);
        self.finder.reset();
        self.pending.clear();
        if !self.text.is_empty() && !self.text.ends_with('\n') {
            self.text.push('\n');
            self.fresh.push('\n');
        }
        self.dirty = true;
    }

    /// Nothing more will come for now (keying down): settle the tail.
    pub fn flush(&self) {
        if let Some(d) = self.deep.as_ref() {
            d.flush();
        }
    }

    pub fn clear(&mut self) {
        self.text.clear();
        self.pending.clear();
        self.dirty = true;
    }

    pub fn text(&self) -> &str {
        &self.text
    }

    pub fn pending(&self) -> &str {
        &self.pending
    }

    /// Committed text since the last call.
    pub fn take_committed(&mut self) -> String {
        std::mem::take(&mut self.fresh)
    }

    pub fn readout(&self) -> Readout {
        if let Some(r) = &self.rs {
            return Readout { tone_hz: (self.band.0 + self.band.1) / 2.0, wpm: r.wpm(), snr_db: 0.0, locked: !r.text.is_empty() };
        }
        Readout { tone_hz: self.rx.tone_hz(), wpm: self.rx.wpm(), snr_db: self.rx.snr_db(), locked: self.rx.locked() }
    }

    fn append(&mut self, s: &str) {
        if s.is_empty() {
            return;
        }
        // DeepCW commits whole words with the edges trimmed: put the separator back.
        let sep = !self.text.is_empty() && !self.text.ends_with([' ', '\n']) && !s.starts_with(' ') && self.deep.is_some();
        if sep {
            self.text.push(' ');
            self.fresh.push(' ');
        }
        self.text.push_str(s);
        self.fresh.push_str(s);
        if self.text.len() > TEXT_CAP {
            let cut = self.text.len() - TEXT_CAP;
            let cut = (cut..self.text.len()).find(|&i| self.text.is_char_boundary(i)).unwrap_or(self.text.len());
            self.text.drain(..cut);
        }
        self.dirty = true;
    }
}

fn start_deep() -> Option<Worker> {
    crate::model::wait_installed();
    match Worker::with_window(WINDOW_S) {
        Ok(w) => Some(w),
        Err(e) => {
            warn!("live CW: DeepCW unavailable, using the timing decoder: {e}");
            None
        }
    }
}

enum Msg {
    Audio(Vec<f32>),
    Restart,
    Flush,
    Clear,
    Neural(bool),
    Rs(bool),
    Band(f32, f32),
}

/// What the engine reads back: the display and what settled since last time.
#[derive(Default)]
struct Shared {
    version: u64,
    text: String,
    pending: String,
    committed: String,
    readout: Option<Readout>,
    neural: bool,
    engine: &'static str,
}

/// [`CwLive`] on its own thread, off the sample path: the engine hands over
/// audio and control without waiting, and reads the result back when it has
/// changed. The tone tracking and resampling cost a third of a Cortex-A9 core
/// at 48 kHz, more than the engine can spare.
pub struct CwLiveThread {
    /// Audio, bounded: dropped when the decoder is behind.
    tx: Sender<Msg>,
    /// Control, unbounded and read first: a restart or engine switch is
    /// never lost behind queued audio.
    ctl: Sender<Msg>,
    gone: bool,
    shared: Arc<Mutex<Shared>>,
    seen: u64,
    warned: bool,
    band: Option<(f32, f32)>,
}

impl CwLiveThread {
    /// `engine`: "timing", "neural" (DeepCW) or "rs" (rain scatter).
    pub fn start(rate: f64, pitch_hz: f32, engine: &str) -> Self {
        let neural = engine == "neural";
        // 5 s of 10 ms blocks: a busy moment on the CPU must not cost audio.
        let (tx, rx) = bounded::<Msg>(512);
        let (ctl, ctl_rx) = crossbeam_channel::unbounded::<Msg>();
        let shared = Arc::new(Mutex::new(Shared::default()));
        let sh = shared.clone();
        std::thread::Builder::new()
            .name("cw-live".into())
            .spawn(move || {
                // Above the other decoders, below the sample path (nice -10).
                // SAFETY: setpriority on our own thread id, no pointers.
                unsafe {
                    libc::setpriority(libc::PRIO_PROCESS, libc::syscall(libc::SYS_gettid) as libc::id_t, -5);
                }
                run(CwLive::new(rate, pitch_hz, neural), rx, ctl_rx, sh)
            })
            .expect("spawn cw-live");
        if engine == "rs" {
            let _ = ctl.send(Msg::Rs(true));
        }
        CwLiveThread { tx, ctl, gone: false, shared, seen: 0, warned: false, band: None }
    }

    /// Audio in; dropped (not queued, and said once) if the decoder falls
    /// seconds behind.
    pub fn audio(&mut self, a: &[f32]) {
        if self.gone {
            return;
        }
        let a = a.iter().map(|&x| if x.is_finite() { x } else { 0.0 }).collect();
        match self.tx.try_send(Msg::Audio(a)) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                if !self.warned {
                    self.warned = true;
                    warn!("live CW decoder behind: audio dropped");
                }
            }
            Err(TrySendError::Disconnected(_)) => {
                self.gone = true;
                warn!("live CW thread gone");
            }
        }
    }
    pub fn restart(&self) {
        let _ = self.ctl.send(Msg::Restart);
    }
    pub fn flush(&self) {
        let _ = self.ctl.send(Msg::Flush);
    }
    pub fn clear(&self) {
        let _ = self.ctl.send(Msg::Clear);
    }
    pub fn set_neural(&self, on: bool) {
        let _ = self.ctl.send(Msg::Neural(on));
    }
    /// "rs" (rain scatter), "neural" (DeepCW) or anything else (timing).
    pub fn set_engine(&self, engine: &str) {
        match engine {
            "rs" => {
                let _ = self.ctl.send(Msg::Rs(true));
            }
            e => {
                let _ = self.ctl.send(Msg::Rs(false));
                let _ = self.ctl.send(Msg::Neural(e == "neural"));
            }
        }
    }
    pub fn engine(&self) -> &'static str {
        let e = self.shared.lock().unwrap().engine;
        if e.is_empty() { "timing" } else { e }
    }
    /// The CW filter passband (audio Hz); sent on only when it changed.
    pub fn set_band(&mut self, lo: f32, hi: f32) {
        let b = (lo, hi);
        if self.band != Some(b) && self.ctl.send(Msg::Band(b.0, b.1)).is_ok() {
            self.band = Some(b);
        }
    }
    /// Whether DeepCW is the engine in force (false also when it failed to start).
    pub fn neural(&self) -> bool {
        self.shared.lock().unwrap().neural
    }

    /// `(text, pending, newly committed)` when the display changed since the
    /// last call.
    pub fn changed(&mut self) -> Option<(String, String, String)> {
        let mut s = self.shared.lock().unwrap();
        if s.version == self.seen {
            return None;
        }
        self.seen = s.version;
        Some((s.text.clone(), s.pending.clone(), std::mem::take(&mut s.committed)))
    }

    pub fn snapshot(&self) -> (String, String) {
        let s = self.shared.lock().unwrap();
        (s.text.clone(), s.pending.clone())
    }

    pub fn readout(&self) -> Option<Readout> {
        self.shared.lock().unwrap().readout
    }
}

fn handle(c: &mut CwLive, m: Msg) {
    match m {
        Msg::Audio(a) => c.process(&a),
        Msg::Restart => c.restart(),
        Msg::Flush => c.flush(),
        Msg::Clear => c.clear(),
        Msg::Neural(on) => c.set_neural(on),
        Msg::Rs(on) => c.set_rs(on),
        Msg::Band(lo, hi) => c.set_band(lo, hi),
    }
}

fn run(mut c: CwLive, rx: Receiver<Msg>, ctl: Receiver<Msg>, shared: Arc<Mutex<Shared>>) {
    let mut n = 0u32;
    // Poll the model at least every 250 ms even with no audio (it finishes
    // on its own worker thread).
    loop {
        // Control first, so it is never stuck behind seconds of audio.
        while let Ok(m) = ctl.try_recv() {
            handle(&mut c, m);
        }
        crossbeam_channel::select! {
            recv(ctl) -> m => match m {
                Ok(m) => handle(&mut c, m),
                Err(_) => return,
            },
            recv(rx) -> m => match m {
                Ok(m) => handle(&mut c, m),
                Err(_) => return,
            },
            default(std::time::Duration::from_millis(250)) => {}
        }
        n = n.wrapping_add(1);
        let changed = c.poll();
        let mut s = shared.lock().unwrap();
        s.neural = c.neural();
        s.engine = c.engine();
        if n % 8 == 0 {
            s.readout = Some(c.readout());
        }
        if changed {
            s.version += 1;
            s.text = c.text().to_string();
            s.pending = c.pending().to_string();
            let fresh = c.take_committed();
            s.committed.push_str(&fresh);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::keyer::CwKeyer;

    #[test]
    fn finder_survives_odd_bands_and_nan() {
        let mut f = Finder::new(12_000.0);
        // A band reaching below 0 Hz from a client: no index underflow.
        f.band = (-500.0, 800.0);
        let tone: Vec<f32> = (0..24_000).map(|i| (std::f32::consts::TAU * 600.0 * i as f32 / 12_000.0).sin()).collect();
        for c in tone.chunks(600) {
            f.push(c);
        }
        // NaN samples do not stick in the average.
        let mut bad = tone.clone();
        bad[10] = f32::NAN;
        for c in bad.chunks(600) {
            f.push(c);
        }
        assert!(f.avg.iter().all(|x| x.is_finite()));
        let mut c = timing(12_000.0, 600.0);
        c.set_band(f32::NAN, 900.0);
        assert!(c.band.0.is_finite());
    }

    #[test]
    fn control_is_not_dropped_when_audio_is_full() {
        let mut t = CwLiveThread::start(12_000.0, 600.0, "timing");
        for _ in 0..2000 {
            t.audio(&[0.0; 4800]);
        }
        t.set_engine("rs");
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
        while t.engine() != "rs" && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        assert_eq!(t.engine(), "rs");
    }

    /// The timing decoder alone (tests carry no DeepCW model).
    fn timing(rate: f64, pitch: f32) -> CwLive {
        CwLive::new(rate, pitch, false)
    }

    /// Keyed audio at `tone` Hz, with a little noise, as the demodulator hands it over.
    fn keyed(text: &str, tone: f64, wpm: f32) -> Vec<f32> {
        let mut k = CwKeyer::new(48_000.0, tone, wpm);
        k.send(text);
        let mut iq = Vec::new();
        while k.busy() {
            k.render(480, &mut iq);
        }
        iq.extend(std::iter::repeat_n(num_complex::Complex32::default(), 48_000));
        let mut seed = 7u32;
        iq.iter()
            .map(|z| {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                z.re * 0.5 + ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.02
            })
            .collect()
    }

    #[test]
    fn copies_the_station_at_the_pitch_and_follows_a_mistuned_tone() {
        let mut c = timing(48_000.0, 700.0);
        // 20 Hz off the pitch: inside the AFC's reach.
        for chunk in keyed("CQ CQ DE SQ6EMM SQ6EMM K", 720.0, 20.0).chunks(480) {
            c.process(chunk);
        }
        c.poll();
        assert!(c.text().contains("SQ6EMM"), "{:?}", c.text());
        assert!((c.readout().tone_hz - 720.0).abs() < 8.0, "{:?}", c.readout());
        assert!(c.take_committed().contains("SQ6EMM"));
        assert!(c.take_committed().is_empty());
    }

    #[test]
    fn finds_a_station_off_the_pitch_anywhere_in_the_filter() {
        // 12 kHz as in trxd; 130 Hz off the pitch, far outside the AFC's reach
        // (two boards 0.1 ppm apart at 23 cm).
        let mut c = timing(12_000.0, 700.0);
        c.set_band(450.0, 950.0);
        let audio: Vec<f32> = keyed("VVV CQ TEST DE SQ6EMM SQ6EMM K", 830.0, 20.0).chunks(4).map(|q| q[0]).collect();
        for chunk in audio.chunks(120) {
            c.process(chunk);
        }
        c.poll();
        assert!(c.text().contains("SQ6EMM"), "{:?}", c.text());
        assert!((c.readout().tone_hz - 830.0).abs() < 10.0, "{:?}", c.readout());
        // A retune puts the decoder back on the pitch.
        c.restart();
        assert_eq!(c.readout().tone_hz, 700.0);
    }

    #[test]
    fn noise_alone_does_not_move_the_decoder() {
        let mut c = timing(12_000.0, 700.0);
        c.set_band(450.0, 950.0);
        let mut seed = 3u32;
        for _ in 0..200 {
            let chunk: Vec<f32> = (0..120)
                .map(|_| {
                    seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.1
                })
                .collect();
            c.process(&chunk);
        }
        // (The AFC itself may wander a fraction of a hertz.)
        assert!((c.readout().tone_hz - 700.0).abs() < 5.0, "{:?}", c.readout());
    }

    #[test]
    fn a_retune_starts_a_new_line_and_clear_empties() {
        let mut c = timing(48_000.0, 700.0);
        for chunk in keyed("CQ TEST DE SQ6EMM K", 700.0, 20.0).chunks(480) {
            c.process(chunk);
        }
        c.restart();
        assert!(c.text().ends_with('\n'), "{:?}", c.text());
        c.restart();
        assert_eq!(c.text().matches('\n').count(), 1, "one line break per station, not per retune");
        c.clear();
        assert!(c.text().is_empty());
    }
}
