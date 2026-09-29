//! Period decoders: each takes one T/R slot of 12 kHz audio (Q65, PI4)
//! or 3.2 kHz audio (DeepCW) and returns [`Decode`] records ready for MQTT.
//!
//! All run on a worker thread ([`DecodeWorker`]) so a decode that takes a
//! second or two on the Cortex-A9 never stalls the receive chain.

use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use mfsk_core::q65::{DecodeRequest as Q65Request, Q65a60, Q65b60, Q65c60, Q65d60, Q65e60, SearchParams};
use serde::Serialize;
use tracing::{debug, warn};

use crate::pi4;

/// The rate every period decoder here works at.
pub const DECODE_RATE: u32 = 12_000;

/// One decoded transmission, as published.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Decode {
    /// "Q65-60D", "PI4", "CW".
    pub mode: String,
    /// UTC start of the slot / window the decode came from, Unix seconds.
    pub utc: i64,
    /// Absolute RF frequency of the signal (tone 0 / carrier), Hz.
    pub freq_hz: f64,
    /// Audio frequency in the receiver passband, Hz.
    pub audio_hz: f32,
    /// Time offset from the slot start, s.
    pub dt: f32,
    pub snr_db: f32,
    pub message: String,
    /// Callsign picked out of the message, where the decoder knows one
    /// (none of the current decoders fills it in).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub call: Option<String>,
}

/// Q65 sub-mode letter for the 60 s T/R period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Q65Letter {
    A,
    B,
    C,
    D,
    E,
}

impl Q65Letter {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_uppercase().as_str() {
            "A" => Some(Self::A),
            "B" => Some(Self::B),
            "C" => Some(Self::C),
            "D" => Some(Self::D),
            "E" => Some(Self::E),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::A => "Q65-60A",
            Self::B => "Q65-60B",
            Self::C => "Q65-60C",
            Self::D => "Q65-60D",
            Self::E => "Q65-60E",
        }
    }
}

/// Q65-60x over one 60 s slot. `audio` starts at the slot boundary.
pub fn q65(audio: &[f32], letter: Q65Letter, slot_utc: i64, dial_hz: f64, freq_range: (f32, f32)) -> Vec<Decode> {
    let params = SearchParams { freq_min_hz: freq_range.0, freq_max_hz: freq_range.1, ..SearchParams::default() };
    let results = match letter {
        Q65Letter::A => Q65Request::<Q65a60>::new(audio, DECODE_RATE, 0, params).decode(),
        Q65Letter::B => Q65Request::<Q65b60>::new(audio, DECODE_RATE, 0, params).decode(),
        Q65Letter::C => Q65Request::<Q65c60>::new(audio, DECODE_RATE, 0, params).decode(),
        Q65Letter::D => Q65Request::<Q65d60>::new(audio, DECODE_RATE, 0, params).decode(),
        Q65Letter::E => Q65Request::<Q65e60>::new(audio, DECODE_RATE, 0, params).decode(),
    };
    results
        .into_iter()
        .map(|r| Decode {
            mode: letter.label().into(),
            utc: slot_utc,
            freq_hz: dial_hz + r.freq_hz as f64,
            audio_hz: r.freq_hz,
            dt: r.dt_sec,
            snr_db: r.snr_db.round(),
            message: r.message,
            call: None,
        })
        .collect()
}

/// PI4 over one minute. The minute boundary falls `boundary` samples into
/// `audio`: give the time search (±2.5 s) lead-in before it.
pub fn pi4(audio: &[f32], boundary: usize, slot_utc: i64, dial_hz: f64) -> Vec<Decode> {
    pi4::decode_window(audio, DECODE_RATE, boundary as i64)
        .into_iter()
        .map(|d| Decode {
            mode: d.variant.label().into(),
            utc: slot_utc,
            freq_hz: dial_hz + d.tone0_hz as f64,
            audio_hz: d.tone0_hz,
            dt: d.dt_sec,
            snr_db: d.snr_db.round(),
            message: d.text,
            call: None,
        })
        .collect()
}

/// DeepCW over a long window of 3.2 kHz audio (signal inside 400..1200 Hz).
/// Fed through DeepCW's own rolling [`sdroxide_deepcw::Stream`], which cuts
/// its windows at word gaps, so no character is split between two windows.
pub fn cw_window(stream: &mut sdroxide_deepcw::Stream, audio_3k2: &[f32]) -> String {
    let mut text = String::new();
    let mut take = |r: Option<Result<sdroxide_deepcw::Update, sdroxide_deepcw::Error>>| match r {
        Some(Ok(u)) if !u.committed.trim().is_empty() => {
            if !text.is_empty() {
                text.push(' ');
            }
            text.push_str(u.committed.trim());
        }
        Some(Err(e)) => warn!("DeepCW: {e}"),
        _ => {}
    };
    stream.reset();
    for piece in audio_3k2.chunks(sdroxide_deepcw::SAMPLE_RATE as usize) {
        stream.push(piece);
        while let Some(r) = stream.poll() {
            take(Some(r));
        }
    }
    take(stream.flush());
    text.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// A unit of decode work.
pub enum Job {
    Q65 { audio: Vec<f32>, letter: Q65Letter, slot_utc: i64, dial_hz: f64, range: (f32, f32) },
    Pi4 { audio: Vec<f32>, boundary: usize, slot_utc: i64, dial_hz: f64 },
    /// `carrier_hz` is the absolute frequency of the keyed tone.
    Cw { audio_3k2: Vec<f32>, slot_utc: i64, carrier_hz: f64, audio_hz: f32, snr_db: f32 },
}

/// Runs [`Job`]s one after another on its own thread; results come back on
/// [`DecodeWorker::poll`]. The queue is short: if decoding falls a whole
/// period behind, newer audio is worth more than old.
pub struct DecodeWorker {
    tx: Option<Sender<Job>>,
    rx: Receiver<Vec<Decode>>,
    handle: Option<JoinHandle<()>>,
}

impl DecodeWorker {
    pub fn new() -> Self {
        let (tx, jobs) = bounded::<Job>(4);
        let (res_tx, rx) = unbounded();
        let handle = std::thread::Builder::new()
            .name("decode".into())
            .spawn(move || {
                let mut deepcw: Option<sdroxide_deepcw::Stream> = None;
                for job in jobs {
                    let started = std::time::Instant::now();
                    let out = match job {
                        Job::Q65 { audio, letter, slot_utc, dial_hz, range } => {
                            q65(&audio, letter, slot_utc, dial_hz, range)
                        }
                        Job::Pi4 { audio, boundary, slot_utc, dial_hz } => pi4(&audio, boundary, slot_utc, dial_hz),
                        Job::Cw { audio_3k2, slot_utc, carrier_hz, audio_hz, snr_db } => {
                            if deepcw.is_none() {
                                crate::model::wait_installed();
                                match sdroxide_deepcw::Stream::new() {
                                    Ok(d) => deepcw = Some(d),
                                    Err(e) => warn!("DeepCW model: {e}"),
                                }
                            }
                            match deepcw.as_mut() {
                                Some(d) => {
                                    let text = cw_window(d, &audio_3k2);
                                    if text.is_empty() {
                                        Vec::new()
                                    } else {
                                        vec![Decode {
                                            mode: "CW".into(),
                                            utc: slot_utc,
                                            freq_hz: carrier_hz,
                                            audio_hz,
                                            dt: 0.0,
                                            snr_db: snr_db.round(),
                                            message: text,
                                            call: None,
                                        }]
                                    }
                                }
                                None => Vec::new(),
                            }
                        }
                    };
                    debug!(n = out.len(), ms = started.elapsed().as_millis() as u64, "decode job");
                    if res_tx.send(out).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn decode worker");
        DecodeWorker { tx: Some(tx), rx, handle: Some(handle) }
    }

    /// Queue a job; dropped (with a warning) if the worker is backed up.
    pub fn submit(&self, job: Job) {
        if let Some(tx) = &self.tx {
            if tx.try_send(job).is_err() {
                warn!("decoder backlog: dropping a slot");
            }
        }
    }

    pub fn poll(&self) -> Vec<Decode> {
        self.rx.try_iter().flatten().collect()
    }

    /// Block until everything submitted so far is decoded (tests, shutdown).
    pub fn finish(mut self) -> Vec<Decode> {
        drop(self.tx.take());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        self.rx.try_iter().flatten().collect()
    }
}

impl Drop for DecodeWorker {
    fn drop(&mut self) {
        drop(self.tx.take());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q65_60d_round_trip() {
        let sig = mfsk_core::q65::synthesize_standard_for::<Q65d60>("DE", "SR3LES", "JO81", DECODE_RATE, 800.0, 0.1)
            .unwrap();
        let mut audio = vec![0.0f32; 60 * DECODE_RATE as usize];
        let start = DECODE_RATE as usize; // 1 s late, like a typical beacon
        for (i, s) in sig.iter().enumerate() {
            audio[start + i] += s;
        }
        let d = q65(&audio, Q65Letter::D, 0, 1_296_871_200.0, (600.0, 1_000.0));
        assert_eq!(d.len(), 1, "{d:?}");
        assert_eq!(d[0].message, "DE SR3LES JO81");
        assert!((d[0].freq_hz - 1_296_872_000.0).abs() < 5.0);
    }

    #[test]
    fn pi4_round_trip() {
        let tones = pi4::spec::encode_reference("SR3LES").unwrap();
        let spacing = pi4::Variant::Pi4.tone_spacing_hz();
        let tone0 = pi4::Variant::Pi4.conventional_tone0_hz();
        let sps = pi4::demod::SYMBOL_SAMPLES;
        let mut audio = vec![0.0f32; 32 * DECODE_RATE as usize];
        let mut phase = 0.0f32;
        let lead = 2 * DECODE_RATE as usize; // audio captured from 2 s before the minute
        let start = lead + DECODE_RATE as usize; // beacon 1 s late
        for (k, &t) in tones.iter().enumerate() {
            let f = tone0 + t as f32 * spacing;
            for n in 0..sps {
                phase = (phase + std::f32::consts::TAU * f / DECODE_RATE as f32) % std::f32::consts::TAU;
                audio[start + k * sps + n] = 0.1 * phase.sin();
            }
        }
        let d = pi4(&audio, lead, 0, 1_296_871_200.0);
        assert_eq!(d.len(), 1, "{d:?}");
        assert_eq!(d[0].message, "SR3LES");
        assert!((d[0].dt - 1.0).abs() < 0.05, "dt {}", d[0].dt);
    }
}
