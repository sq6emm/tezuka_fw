//! Live CW copy of the station the operator is tuned to, as sdroxide's CW panel
//! does it: the demodulated audio's tone at `pitch` above the dial is followed
//! by the classic [`CwRx`] (AFC, speed, SNR) and read by DeepCW as a running
//! transcript. Text arrives about once a second: the settled part is appended
//! for good, the last word or two stays "pending" and firms up in place.
//!
//! It reads the one station under the cursor, letter by letter.

use std::sync::{Arc, Mutex};

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

pub struct CwLive {
    rx: CwRx,
    tuner: Tuner,
    deep: Option<Worker>,
    scratch: Vec<f32>,
    text: String,
    pending: String,
    /// Text committed since the last [`CwLive::take_committed`] (for MQTT).
    fresh: String,
    dirty: bool,
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
            tuner: Tuner::new(rate, pitch_hz as f64),
            deep,
            scratch: Vec::new(),
            text: String::new(),
            pending: String::new(),
            fresh: String::new(),
            dirty: false,
        }
    }

    /// Switch between DeepCW and the timing decoder; the text so far stays.
    pub fn set_neural(&mut self, on: bool) {
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
}

/// [`CwLive`] on its own thread, off the sample path: the engine hands over
/// audio and control without waiting, and reads the result back when it has
/// changed. The tone tracking and resampling cost a third of a Cortex-A9 core
/// at 48 kHz, more than the engine can spare.
pub struct CwLiveThread {
    tx: Sender<Msg>,
    shared: Arc<Mutex<Shared>>,
    seen: u64,
    warned: bool,
}

impl CwLiveThread {
    pub fn start(rate: f64, pitch_hz: f32, neural: bool) -> Self {
        // 5 s of 10 ms blocks: a busy moment on the CPU must not cost audio.
        let (tx, rx) = bounded::<Msg>(512);
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
                run(CwLive::new(rate, pitch_hz, neural), rx, sh)
            })
            .expect("spawn cw-live");
        CwLiveThread { tx, shared, seen: 0, warned: false }
    }

    /// Audio in; dropped (not queued, and said once) if the decoder falls
    /// seconds behind.
    pub fn audio(&mut self, a: &[f32]) {
        match self.tx.try_send(Msg::Audio(a.to_vec())) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                if !self.warned {
                    self.warned = true;
                    warn!("live CW decoder behind: audio dropped");
                }
            }
            Err(TrySendError::Disconnected(_)) => warn!("live CW thread gone"),
        }
    }
    pub fn restart(&self) {
        let _ = self.tx.try_send(Msg::Restart);
    }
    pub fn flush(&self) {
        let _ = self.tx.try_send(Msg::Flush);
    }
    pub fn clear(&self) {
        let _ = self.tx.try_send(Msg::Clear);
    }
    pub fn set_neural(&self, on: bool) {
        let _ = self.tx.try_send(Msg::Neural(on));
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

fn run(mut c: CwLive, rx: Receiver<Msg>, shared: Arc<Mutex<Shared>>) {
    let mut n = 0u32;
    // Poll the model at least every 250 ms even with no audio (it finishes
    // on its own worker thread).
    loop {
        match rx.recv_timeout(std::time::Duration::from_millis(250)) {
            Ok(Msg::Audio(a)) => c.process(&a),
            Ok(Msg::Restart) => c.restart(),
            Ok(Msg::Flush) => c.flush(),
            Ok(Msg::Clear) => c.clear(),
            Ok(Msg::Neural(on)) => c.set_neural(on),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
        }
        n = n.wrapping_add(1);
        let changed = c.poll();
        let mut s = shared.lock().unwrap();
        s.neural = c.neural();
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
