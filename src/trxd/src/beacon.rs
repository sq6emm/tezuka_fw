//! The two beacon roles.
//!
//! **beacon-tx** reproduces MGMBeacon's minute cycle on the AD936x:
//!
//! * even minute: digital mode (PI4 / Q65-60x) from second 0, then carrier;
//! * odd minute: CW identification (F1A: key-up shifted below the carrier),
//!   then carrier — "HNY HNY" first on 31 December;
//! * without a synchronised clock: "NOTIME" + CW + carrier, untimed.
//!
//! Tone 0 of the digital modes and CW key-down sit on the carrier, exactly as
//! BeaconModes does. The waveform is synthesised straight at the stream rate
//! by a phase-continuous f64 oscillator, so tone changes are sample-exact.
//!
//! **beacon-rx** listens on a beacon, records each UTC minute (with lead-in
//! for the decoders' time search), and every minute publishes to MQTT:
//! PI4 / Q65 decodes, a DeepCW read of the CW identification, and a carrier
//! measurement (frequency offset and SNR) from the minute's carrier tail.

use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use num_complex::Complex32;
use rustfft::FftPlanner;
use sdroxide_dsp::{ComplexFir, Ddc, RealFirDecim, bandpass_taps};
use serde::Serialize;
use tracing::{info, warn};

use crate::config::{BeaconDigital, Config, DecoderKind};
use crate::decode::{DECODE_RATE, Decode, DecodeWorker, Job, Q65Letter};
use crate::morse;
use crate::mqtt::Mqtt;
use crate::radio::RadioControl;
use crate::slots::SlotRecorder;
use crate::stream::{RxBlock, time_synced, unix_now};

// ------------------------------------------------------------------ TX

/// One segment of a beacon transmission: frequency offsets from the carrier,
/// each held for `step_s`.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    pub label: String,
    pub step_s: f64,
    /// Offset from the carrier per step, Hz; `None` = carrier off.
    pub offsets: Vec<Option<f64>>,
}

impl Segment {
    pub fn duration_s(&self) -> f64 {
        self.step_s * self.offsets.len() as f64
    }
}

/// PI4 symbol period (6 baud) and tone spacing (PI4 = 40 * 12000 / 2048).
const PI4_SYMBOL_S: f64 = 1.0 / 6.0;

pub fn pi4_segment(message: &str, tone0_offset_hz: f64) -> Result<Segment, String> {
    let tones = crate::pi4::spec::encode_reference(message).ok_or_else(|| format!("PI4 cannot carry '{message}'"))?;
    let spacing = crate::pi4::spec::Variant::Pi4.tone_spacing_hz() as f64;
    Ok(Segment {
        label: "PI4".into(),
        step_s: PI4_SYMBOL_S,
        offsets: tones.iter().map(|&t| Some(tone0_offset_hz + t as f64 * spacing)).collect(),
    })
}

fn q65_params(d: BeaconDigital) -> Option<(f64, f64, &'static str)> {
    use mfsk_core::engine::protocol::ModulationParams;
    use mfsk_core::q65::{Q65a60, Q65b60, Q65c60, Q65d60, Q65e60};
    fn p<P: ModulationParams>(label: &'static str) -> (f64, f64, &'static str) {
        (P::NSPS as f64 / 12_000.0, P::TONE_SPACING_HZ as f64, label)
    }
    Some(match d {
        BeaconDigital::Q65_60A => p::<Q65a60>("Q65-60A"),
        BeaconDigital::Q65_60B => p::<Q65b60>("Q65-60B"),
        BeaconDigital::Q65_60C => p::<Q65c60>("Q65-60C"),
        BeaconDigital::Q65_60D => p::<Q65d60>("Q65-60D"),
        BeaconDigital::Q65_60E => p::<Q65e60>("Q65-60E"),
        _ => return None,
    })
}

/// Q65 "DE <call> <loc4>" — the message MGMBeacon sends, which WSJT-X also
/// forwards to PSK Reporter.
pub fn q65_segment(d: BeaconDigital, call: &str, loc4: &str, tone0_offset_hz: f64) -> Result<Segment, String> {
    let (symbol_s, spacing, label) = q65_params(d).ok_or("not a Q65 mode")?;
    let bits = mfsk_core::msg::wsjt77::pack77("DE", call, loc4)
        .ok_or_else(|| format!("Q65 cannot pack 'DE {call} {loc4}'"))?;
    let tones = mfsk_core::q65::encode_channel_symbols(&bits);
    Ok(Segment {
        label: label.into(),
        step_s: symbol_s,
        offsets: tones.iter().map(|&t| Some(tone0_offset_hz + t as f64 * spacing)).collect(),
    })
}

/// CW in dot steps. `space_shift_hz > 0` keys F1A (carrier moves below on
/// key-up); 0 keys the carrier on and off.
pub fn cw_segment(text: &str, wpm: u8, space_shift_hz: f64) -> Segment {
    let up = if space_shift_hz > 0.0 { Some(-space_shift_hz) } else { None };
    Segment {
        label: "CW".into(),
        step_s: morse::dot_seconds(wpm as f32),
        offsets: morse::timeline(&text.to_ascii_uppercase())
            .into_iter()
            .map(|k| if k { Some(0.0) } else { up })
            .collect(),
    }
}

pub fn carrier_segment(seconds: f64) -> Segment {
    Segment { label: "carrier".into(), step_s: seconds, offsets: vec![Some(0.0)] }
}

/// What to send in the minute starting at `minute_utc` (Unix seconds).
pub fn plan_minute(cfg: &Config, minute_utc: i64, synced: bool) -> Result<Vec<Segment>, String> {
    let b = &cfg.beacon;
    let call = cfg.callsign.trim().to_ascii_uppercase();
    let cw_text = cfg.beacon_cw_text();
    let mut segs = Vec::new();
    if !synced && b.require_time_sync {
        segs.push(cw_segment("NOTIME ", b.cw_wpm, b.cw_space_shift_hz));
        segs.push(cw_segment(&cw_text, b.cw_wpm, b.cw_space_shift_hz));
    } else {
        let minute = minute_utc.div_euclid(60);
        let even = minute % 2 == 0;
        if even && b.digital != BeaconDigital::None {
            segs.push(match b.digital {
                BeaconDigital::Pi4 => pi4_segment(&call, b.tone0_offset_hz)?,
                d => q65_segment(d, &call, &cfg.locator4(), b.tone0_offset_hz)?,
            });
        } else {
            if b.hny && is_new_years_eve(minute_utc) {
                segs.push(cw_segment("HNY HNY ", b.cw_wpm, b.cw_space_shift_hz));
            }
            segs.push(cw_segment(&cw_text, b.cw_wpm, b.cw_space_shift_hz));
        }
    }
    let used: f64 = segs.iter().map(Segment::duration_s).sum();
    if used > 59.5 {
        return Err(format!("beacon sequence is {used:.1} s, longer than a minute"));
    }
    segs.push(carrier_segment(60.0 - used));
    Ok(segs)
}

fn is_new_years_eve(utc: i64) -> bool {
    // Civil date from days since the epoch (Howard Hinnant's algorithm).
    let z = utc.div_euclid(86_400) + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    m == 12 && d == 31
}

/// Renders segments at the stream rate: a phase-continuous oscillator at
/// `carrier_offset` (carrier relative to the LO) plus each step's offset.
pub struct Synth {
    rate: f64,
    carrier_offset: f64,
    amplitude: f32,
    phase: f64,
    /// Remaining (offset, samples) steps.
    steps: std::collections::VecDeque<(Option<f64>, u64)>,
    /// Fractional-sample carry, so step boundaries don't drift.
    t_acc: f64,
    env: f64,
}

impl Synth {
    pub fn new(rate: f64, carrier_offset: f64, amplitude: f32) -> Self {
        Synth {
            rate,
            carrier_offset,
            amplitude,
            phase: 0.0,
            steps: Default::default(),
            t_acc: 0.0,
            env: 0.0,
        }
    }

    pub fn queue(&mut self, segs: &[Segment]) {
        for s in segs {
            for &o in &s.offsets {
                self.t_acc += s.step_s * self.rate;
                let n = self.t_acc.floor();
                self.t_acc -= n;
                self.steps.push_back((o, n as u64));
            }
        }
    }

    pub fn queued_s(&self) -> f64 {
        self.steps.iter().map(|(_, n)| *n as f64).sum::<f64>() / self.rate
    }

    pub fn render(&mut self, out: &mut [Complex32]) {
        let edge = 1.0 / (0.004 * self.rate);
        for z in out.iter_mut() {
            let off = loop {
                match self.steps.front_mut() {
                    Some((_, 0)) => {
                        self.steps.pop_front();
                    }
                    Some((o, n)) => {
                        *n -= 1;
                        break *o;
                    }
                    None => break None,
                }
            };
            let on = off.is_some();
            self.env = if on { (self.env + edge).min(1.0) } else { (self.env - edge).max(0.0) };
            let f = self.carrier_offset + off.unwrap_or(0.0);
            self.phase = (self.phase + std::f64::consts::TAU * f / self.rate) % std::f64::consts::TAU;
            let a = self.amplitude as f64 * (0.5 - 0.5 * (std::f64::consts::PI * self.env).cos());
            *z = Complex32::new((a * self.phase.cos()) as f32, (a * self.phase.sin()) as f32);
        }
    }
}

#[derive(Serialize)]
struct BeaconTxState<'a> {
    role: &'static str,
    freq_hz: f64,
    minute_utc: i64,
    sequence: Vec<&'a str>,
    time_synced: bool,
}

pub fn run_tx(cfg: Config, mut radio: Box<dyn RadioControl>, rx: Receiver<RxBlock>, tx_sink: Sender<Vec<Complex32>>, mqtt: Mqtt) {
    let rate = radio.stream_rate();
    let block = cfg.radio.buffer_samples;
    let lo = cfg.beacon.freq_hz - cfg.radio.lo_offset_hz;
    if let Err(e) = radio.set_lo(lo) {
        warn!("beacon LO: {e}");
    }
    if let Err(e) = radio.set_tx_rf(true) {
        warn!("beacon TX on: {e}");
    }
    info!(freq = cfg.beacon.freq_hz, digital = ?cfg.beacon.digital, "beacon transmitter running");
    let mut synth = Synth::new(rate, cfg.radio.lo_offset_hz, 1.0);
    let latency = cfg.beacon.tx_latency_ms as f64 / 1e3;
    let mut planned_until: i64 = 0;
    let mut notime_busy_until = 0.0;

    for b in rx {
        // Time at which the sample we render now reaches the antenna.
        let air_t = b.t0 + latency;
        let synced = time_synced();
        if !synced && cfg.beacon.require_time_sync {
            // Untimed: back-to-back NOTIME sequences with 20 s of carrier.
            if synth.queued_s() < 0.5 && air_t >= notime_busy_until {
                match plan_minute(&cfg, 0, false) {
                    Ok(mut segs) => {
                        if let Some(c) = segs.last_mut() {
                            *c = carrier_segment(20.0);
                        }
                        let len: f64 = segs.iter().map(Segment::duration_s).sum();
                        notime_busy_until = air_t + len;
                        synth.queue(&segs);
                        warn!("no time sync: sending NOTIME sequence");
                    }
                    Err(e) => warn!("{e}"),
                }
            }
            planned_until = 0;
        } else {
            // Plan the next minute a little ahead of time, aligned so its first
            // sample reaches the air on second 0.
            let next_minute = ((air_t / 60.0).floor() as i64 + 1) * 60;
            if planned_until < next_minute && next_minute as f64 - air_t < 1.0 {
                let queued_end = air_t + synth.queued_s();
                let gap = next_minute as f64 - queued_end;
                if gap > 0.0 {
                    // Fill with carrier up to the boundary.
                    synth.queue(&[carrier_segment(gap)]);
                } else if gap < -0.02 {
                    warn!(overrun_ms = -gap * 1e3, "previous sequence overran the minute; trimming");
                    synth.steps.clear();
                    synth.queue(&[carrier_segment(next_minute as f64 - air_t)]);
                }
                match plan_minute(&cfg, next_minute, true) {
                    Ok(segs) => {
                        let labels: Vec<&str> = segs.iter().map(|s| s.label.as_str()).collect();
                        info!(minute = next_minute, ?labels, "beacon minute");
                        mqtt.publish_json(
                            "state",
                            &BeaconTxState {
                                role: "beacon-tx",
                                freq_hz: cfg.beacon.freq_hz,
                                minute_utc: next_minute,
                                sequence: labels,
                                time_synced: synced,
                            },
                            true,
                        );
                        synth.queue(&segs);
                    }
                    Err(e) => warn!("{e}"),
                }
                planned_until = next_minute;
            }
            if planned_until == 0 && synth.queued_s() < 0.2 {
                // Just started: carrier until the first boundary.
                synth.queue(&[carrier_segment(0.5)]);
            }
        }
        let mut out = vec![Complex32::default(); block];
        synth.render(&mut out);
        if tx_sink.send(out).is_err() {
            break;
        }
        for (name, payload) in mqtt.poll_cmds() {
            warn!("beacon-tx ignores MQTT command {name}={payload}");
        }
    }
    let _ = radio.set_tx_rf(false);
}

// ------------------------------------------------------------------ RX

/// Carrier measurement from a stretch of the minute known to be carrier only.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct CarrierReport {
    pub utc: i64,
    /// Measured absolute carrier frequency, Hz.
    pub freq_hz: f64,
    /// Measured minus nominal, Hz.
    pub offset_hz: f64,
    /// Carrier power over the noise in 2.5 kHz, dB.
    pub snr_db: f32,
    /// Carrier level, dBFS at the channel.
    pub level_dbfs: f32,
}

/// Find the carrier near `expect_hz` (audio) in 12 kHz `audio`, with
/// sub-bin interpolation.
pub fn measure_carrier(audio: &[f32], expect_hz: f64, search_hz: f64) -> Option<(f64, f32, f32)> {
    let n = audio.len().next_power_of_two() / 2;
    if n < 4096 {
        return None;
    }
    let seg = &audio[audio.len() - n..];
    let mut buf: Vec<rustfft::num_complex::Complex32> = seg
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let w = 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / n as f32).cos();
            rustfft::num_complex::Complex32::new(s * w, 0.0)
        })
        .collect();
    FftPlanner::new().plan_fft_forward(n).process(&mut buf);
    let bin_hz = DECODE_RATE as f64 / n as f64;
    let p: Vec<f32> = buf[..n / 2].iter().map(|z| z.norm_sqr()).collect();
    let lo = ((expect_hz - search_hz) / bin_hz).max(1.0) as usize;
    let hi = (((expect_hz + search_hz) / bin_hz) as usize).min(n / 2 - 2);
    let (k, &pk) = p[lo..=hi].iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1))?;
    let k = k + lo;
    // Parabolic interpolation on log power.
    let (a, b, c) = (p[k - 1].max(1e-30).ln(), pk.max(1e-30).ln(), p[k + 1].max(1e-30).ln());
    let den = a - 2.0 * b + c;
    let delta = if den.abs() > 1e-12 { 0.5 * (a - c) / den } else { 0.0 };
    let freq = (k as f64 + delta as f64) * bin_hz;
    // Noise: median bin power over 300..2800 Hz, scaled to 2.5 kHz.
    let mut noise: Vec<f32> = p[(300.0 / bin_hz) as usize..(2800.0 / bin_hz) as usize].to_vec();
    noise.sort_by(|a, b| a.total_cmp(b));
    let median = noise[noise.len() / 2].max(1e-30);
    // Hann window: the carrier's energy spreads over ~1.5 bins.
    let sig: f32 = p[k - 1..=k + 1].iter().sum();
    let snr = 10.0 * (sig / (median * (2500.0 / bin_hz) as f32)).log10();
    let w_gain = (n as f32 / 2.0).powi(2);
    let level = 10.0 * (sig / w_gain).max(1e-30).log10();
    Some((freq, snr, level))
}

pub fn run_rx(cfg: Config, mut radio: Box<dyn RadioControl>, rx: Receiver<RxBlock>, tx_sink: Sender<Vec<Complex32>>, mqtt: Mqtt) {
    let rate = radio.stream_rate();
    let br = &cfg.beacon_rx;
    let dial = br.freq_hz - br.audio_offset_hz;
    let lo = dial - cfg.radio.lo_offset_hz;
    if let Err(e) = radio.set_lo(lo) {
        warn!("beacon-rx LO: {e}");
    }
    let _ = radio.set_tx_rf(false);
    info!(beacon = br.freq_hz, dial, "beacon receiver running");

    let mut ddc = Ddc::new(rate, 48_000.0);
    ddc.set_offset_hz(dial - lo);
    // USB audio: 100..3000 Hz of the channel, real part.
    let mut usb = ComplexFir::new(bandpass_taps(255, 100.0, 3_000.0, 48_000.0));
    // The CW mark only: key-up (400 Hz lower) must be silence for DeepCW.
    let mut mark = ComplexFir::new(bandpass_taps(511, br.audio_offset_hz - 150.0, br.audio_offset_hz + 150.0, 48_000.0));
    let mut dec12 = RealFirDecim::new(63, 5_000.0, 48_000.0, 4);
    let mut dec3k2 = RealFirDecim::new(127, 1_400.0, 48_000.0, 15);
    let mut minute12 = SlotRecorder::new(12_000.0, 60, 2.5, 59.5);
    let mut minute3k2 = SlotRecorder::new(3_200.0, 60, 0.0, 59.5);
    let decoder = DecodeWorker::new();
    let letter = Q65Letter::parse(&br.q65_submode).unwrap_or(Q65Letter::D);
    let silence = vec![Complex32::default(); cfg.radio.buffer_samples];

    let (mut chan, mut a_usb, mut a_mark, mut a12, mut a3k2) = (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut filtered = Vec::new();
    let mut last_state = Instant::now() - Duration::from_secs(60);
    for b in rx {
        // Keep the DAC fed with silence (TX RF is off).
        let _ = tx_sink.try_send(silence.clone());
        chan.clear();
        ddc.process(&b.iq, &mut chan);
        filtered.clear();
        usb.process(&chan, &mut filtered);
        a_usb.clear();
        a_usb.extend(filtered.iter().map(|z| z.re * 2.0));
        filtered.clear();
        mark.process(&chan, &mut filtered);
        a_mark.clear();
        a_mark.extend(filtered.iter().map(|z| z.re * 2.0));
        a12.clear();
        dec12.process(&a_usb, &mut a12);
        a3k2.clear();
        dec3k2.process(&a_mark, &mut a3k2);

        for slot in minute12.push(b.t0, &a12) {
            let digital = &slot.audio[slot.boundary..];
            if br.decoders.contains(&DecoderKind::Pi4) {
                decoder.submit(Job::Pi4 { audio: slot.audio.clone(), boundary: slot.boundary, slot_utc: slot.utc, dial_hz: dial });
            }
            if br.decoders.contains(&DecoderKind::Q65) {
                let lo_f = (br.audio_offset_hz - 300.0) as f32;
                decoder.submit(Job::Q65 {
                    audio: digital.to_vec(),
                    letter,
                    slot_utc: slot.utc,
                    dial_hz: dial,
                    range: (lo_f.max(100.0), (br.audio_offset_hz + 1_200.0) as f32),
                });
            }
            // The last 6 s of every minute are carrier in both the PI4/NGNB and
            // the MGMBeacon cycles.
            let tail_len = (6.0 * DECODE_RATE as f64) as usize;
            if digital.len() > tail_len {
                if let Some((f, snr, level)) =
                    measure_carrier(&digital[digital.len() - tail_len..], br.audio_offset_hz, 400.0)
                {
                    let r = CarrierReport {
                        utc: slot.utc,
                        freq_hz: dial + f,
                        offset_hz: dial + f - br.freq_hz,
                        snr_db: (snr * 10.0).round() / 10.0,
                        level_dbfs: (level * 10.0).round() / 10.0,
                    };
                    info!(offset = r.offset_hz, snr = r.snr_db, "carrier");
                    mqtt.publish_json("carrier", &r, false);
                }
            }
            if !br.wav_dir.is_empty() {
                let path = std::path::Path::new(&br.wav_dir).join(format!("{}.wav", slot.utc));
                if let Err(e) = write_wav(&path, &slot.audio[slot.boundary..], DECODE_RATE) {
                    warn!("{}: {e}", path.display());
                }
            }
        }
        for slot in minute3k2.push(b.t0, &a3k2) {
            if br.decoders.contains(&DecoderKind::Cw) {
                decoder.submit(Job::Cw {
                    audio_3k2: slot.audio,
                    slot_utc: slot.utc,
                    carrier_hz: br.freq_hz,
                    audio_hz: br.audio_offset_hz as f32,
                    snr_db: 0.0,
                });
            }
        }
        for d in decoder.poll() {
            info!(mode = %d.mode, snr = d.snr_db, "{}", d.message);
            mqtt.publish_decode(&d);
        }
        if last_state.elapsed() >= Duration::from_secs(30) {
            mqtt.publish_json(
                "state",
                &serde_json::json!({
                    "role": "beacon-rx", "beacon_hz": br.freq_hz, "dial_hz": dial,
                    "time_synced": time_synced(), "utc": unix_now() as i64,
                }),
                true,
            );
            last_state = Instant::now();
        }
    }
}

/// 16-bit mono WAV, WSJT-X's own layout (plain 44-byte header).
pub fn write_wav(path: &std::path::Path, audio: &[f32], rate: u32) -> std::io::Result<()> {
    let data_len = (audio.len() * 2) as u32;
    let mut b = Vec::with_capacity(44 + data_len as usize);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data_len).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&rate.to_le_bytes());
    b.extend_from_slice(&(rate * 2).to_le_bytes());
    b.extend_from_slice(&2u16.to_le_bytes());
    b.extend_from_slice(&16u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data_len.to_le_bytes());
    for s in audio {
        b.extend_from_slice(&((s * 16_000.0).clamp(-32_767.0, 32_767.0) as i16).to_le_bytes());
    }
    std::fs::write(path, b)
}

/// Decodes everything a beacon receiver would from one minute of 48 kHz
/// complex baseband with the carrier at `audio_offset_hz` — the RX chain of
/// [`run_rx`] without the radio, for tests.
#[cfg(test)]
fn decode_minute(chan48: &[Complex32], audio_offset_hz: f64, cfg: &Config) -> (Vec<Decode>, Option<(f64, f32, f32)>) {
    let mut usb = ComplexFir::new(bandpass_taps(255, 100.0, 3_000.0, 48_000.0));
    let mut mark = ComplexFir::new(bandpass_taps(511, audio_offset_hz - 150.0, audio_offset_hz + 150.0, 48_000.0));
    let (mut f1, mut f2, mut a12, mut a3) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    usb.process(chan48, &mut f1);
    mark.process(chan48, &mut f2);
    let u: Vec<f32> = f1.iter().map(|z| z.re * 2.0).collect();
    let m: Vec<f32> = f2.iter().map(|z| z.re * 2.0).collect();
    RealFirDecim::new(63, 5_000.0, 48_000.0, 4).process(&u, &mut a12);
    RealFirDecim::new(127, 1_400.0, 48_000.0, 15).process(&m, &mut a3);
    let w = DecodeWorker::new();
    let lead = (2.5 * DECODE_RATE as f64) as usize;
    let mut padded = vec![0.0f32; lead];
    padded.extend_from_slice(&a12);
    w.submit(Job::Pi4 { audio: padded, boundary: lead, slot_utc: 0, dial_hz: 0.0 });
    let letter = Q65Letter::parse(&cfg.beacon_rx.q65_submode).unwrap();
    w.submit(Job::Q65 { audio: a12.clone(), letter, slot_utc: 0, dial_hz: 0.0, range: (500.0, 2_000.0) });
    w.submit(Job::Cw { audio_3k2: a3, slot_utc: 0, carrier_hz: 0.0, audio_hz: 800.0, snr_db: 0.0 });
    let tail = (6.0 * DECODE_RATE as f64) as usize;
    let carrier = measure_carrier(&a12[a12.len() - tail..], audio_offset_hz, 400.0);
    (w.finish(), carrier)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    fn cfg(digital: &str) -> Config {
        Config::parse(&format!(
            r#"
            callsign = "SR3LES"
            locator = "JO81HU"
            [beacon]
            digital = "{digital}"
            [beacon_rx]
            q65_submode = "D"
            "#
        ))
        .unwrap()
    }

    /// Render a planned minute through [`Synth`] at 48 kHz with the carrier at
    /// +800 Hz (the receiver's audio offset), add noise, decode.
    fn loop_minute(cfg: &Config, minute: i64) -> (Vec<Decode>, Option<(f64, f32, f32)>) {
        let segs = plan_minute(cfg, minute, true).unwrap();
        let total: f64 = segs.iter().map(Segment::duration_s).sum();
        assert!((total - 60.0).abs() < 1e-6, "{total}");
        let rate = 48_000.0;
        let mut s = Synth::new(rate, 800.0, 0.05);
        s.queue(&segs);
        let mut iq = vec![Complex32::default(); 60 * 48_000];
        s.render(&mut iq);
        let mut seed = 7u32;
        for z in iq.iter_mut() {
            let mut r = || {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
            };
            *z += Complex32::new(r(), r()) * 0.01;
        }
        decode_minute(&iq, 800.0, cfg)
    }

    #[test]
    fn q65_minute_decodes_back() {
        let c = cfg("q65-60d");
        let (d, carrier) = loop_minute(&c, 0); // even minute: Q65
        let q = d.iter().find(|d| d.mode == "Q65-60D").unwrap_or_else(|| panic!("{d:?}"));
        assert_eq!(q.message, "DE SR3LES JO81");
        assert!((q.audio_hz - 800.0).abs() < 3.0, "{}", q.audio_hz);
        let (f, snr, _) = carrier.unwrap();
        assert!((f - 800.0).abs() < 0.2, "carrier {f}");
        assert!(snr > 20.0, "snr {snr}");
    }

    #[test]
    fn pi4_minute_decodes_back() {
        let c = cfg("pi4");
        let (d, _) = loop_minute(&c, 120);
        let p = d.iter().find(|d| d.mode == "PI4").unwrap_or_else(|| panic!("{d:?}"));
        assert_eq!(p.message, "SR3LES");
    }

    #[test]
    fn cw_minute_decodes_back() {
        let c = cfg("pi4");
        let (d, _) = loop_minute(&c, 60); // odd minute: CW
        let cw = d.iter().find(|d| d.mode == "CW").unwrap_or_else(|| panic!("{d:?}"));
        assert!(cw.message.contains("SR3LES"), "{}", cw.message);
        assert!(cw.message.contains("JO81HU"), "{}", cw.message);
    }

    #[test]
    fn plans_follow_the_mgmbeacon_cycle() {
        let c = cfg("q65-60d");
        let even = plan_minute(&c, 1_800_000_000 - 1_800_000_000 % 120, true).unwrap();
        assert_eq!(even[0].label, "Q65-60D");
        assert!((even[0].duration_s() - 51.0).abs() < 0.1, "{}", even[0].duration_s());
        let odd = plan_minute(&c, 1_800_000_000 - 1_800_000_000 % 120 + 60, true).unwrap();
        assert_eq!(odd[0].label, "CW");
        let notime = plan_minute(&c, 0, false).unwrap();
        assert_eq!(notime.len(), 3);
        // 31 Dec 2025 12:01 UTC (odd minute) gets the HNY prefix.
        let hny = plan_minute(&c, 1_767_182_460, true).unwrap();
        assert_eq!(hny.len(), 3);
        assert!(is_new_years_eve(1_767_182_460));
        assert!(!is_new_years_eve(1_767_182_460 + 86_400));
    }
}
