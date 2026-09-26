//! A radio made of arithmetic, for running the whole daemon on a PC: receive
//! noise plus a keyed CW test signal, and hear whatever is transmitted (TX is
//! looped back into RX, 40 dB down). Blocks are paced to real time unless the
//! config says otherwise, so a TCI client sees the same cadence as on a board.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use num_complex::Complex32;

use super::{Radio, RadioControl, RxStream, TxStream};
use crate::config::{GainMode, RadioConfig};

/// The test signal's offset above the first LO the daemon tunes, chosen so it
/// lands 700 Hz into a USB/CW passband at the configured VFO.
const TEST_SIGNAL_TEXT: &str = "CQ CQ DE SIM SIM K   ";
const TEST_SIGNAL_WPM: f32 = 22.0;
const NOISE_RMS: f32 = 3e-4;
const TEST_SIGNAL_AMP: f32 = 3e-3;
const LOOPBACK_GAIN: f32 = 0.01;

#[derive(Default)]
struct Shared {
    lo_hz: f64,
    /// The test signal's absolute frequency, fixed by the first tune.
    signal_hz: Option<f64>,
    tx_on: bool,
    loopback: VecDeque<Complex32>,
}

pub struct SimControl {
    shared: Arc<Mutex<Shared>>,
    rate: f64,
    lo_offset_hz: f64,
    gain_db: f64,
}

impl RadioControl for SimControl {
    fn stream_rate(&self) -> f64 {
        self.rate
    }
    fn set_lo(&mut self, hz: f64) -> Result<(), String> {
        let mut s = self.shared.lock().unwrap();
        s.lo_hz = hz;
        if s.signal_hz.is_none() {
            s.signal_hz = Some(hz + self.lo_offset_hz + 700.0);
        }
        Ok(())
    }
    fn set_rx_gain(&mut self, _mode: GainMode, db: f64) -> Result<(), String> {
        self.gain_db = db;
        Ok(())
    }
    fn set_tx_attenuation(&mut self, _db: f64) -> Result<(), String> {
        Ok(())
    }
    fn set_tx_rf(&mut self, on: bool) -> Result<(), String> {
        let mut s = self.shared.lock().unwrap();
        s.tx_on = on;
        if !on {
            s.loopback.clear();
        }
        Ok(())
    }
    fn rx_gain_db(&mut self) -> f64 {
        self.gain_db
    }
}

pub struct SimRx {
    shared: Arc<Mutex<Shared>>,
    rate: f64,
    realtime: bool,
    started: Option<Instant>,
    produced: u64,
    rng: u64,
    phase: f64,
    keying: Vec<bool>,
    envelope: f32,
}

impl SimRx {
    /// xorshift64* -> two uniform floats -> Box-Muller: cheap, deterministic,
    /// and good enough noise for a receiver to chew on.
    fn gauss(&mut self) -> f32 {
        let mut next = || {
            self.rng ^= self.rng >> 12;
            self.rng ^= self.rng << 25;
            self.rng ^= self.rng >> 27;
            let v = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
            ((v >> 40) as f32 + 0.5) / (1u64 << 24) as f32
        };
        let (u1, u2) = (next(), next());
        (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos()
    }
}

impl RxStream for SimRx {
    fn read(&mut self, out: &mut [Complex32]) -> Result<(), String> {
        if self.realtime {
            let start = *self.started.get_or_insert_with(Instant::now);
            let due = start + Duration::from_secs_f64((self.produced + out.len() as u64) as f64 / self.rate);
            if let Some(wait) = due.checked_duration_since(Instant::now()) {
                std::thread::sleep(wait);
            }
        }
        let (lo, signal_hz, loop_samples) = {
            let mut s = self.shared.lock().unwrap();
            let n = out.len().min(s.loopback.len());
            let lb: Vec<Complex32> = s.loopback.drain(..n).collect();
            // The test station stands by while we transmit (our own signal is
            // what the loopback is for, e.g. a wideband DATV check).
            (s.lo_hz, s.signal_hz.filter(|_| !s.tx_on), lb)
        };
        let dot_samples = (crate::morse::dot_seconds(TEST_SIGNAL_WPM) * self.rate) as u64;
        let rise = 1.0 / (0.005 * self.rate as f32);
        let step = std::f64::consts::TAU * signal_hz.map_or(0.0, |f| f - lo) / self.rate;
        for (i, z) in out.iter_mut().enumerate() {
            let t = self.produced + i as u64;
            let key = self.keying[((t / dot_samples) as usize) % self.keying.len()];
            let target = if key && signal_hz.is_some() { 1.0 } else { 0.0 };
            self.envelope += (target - self.envelope).clamp(-rise, rise);
            self.phase = (self.phase + step) % std::f64::consts::TAU;
            let a = TEST_SIGNAL_AMP * self.envelope;
            let (n1, n2) = (self.gauss(), self.gauss());
            *z = Complex32::new(
                a * self.phase.cos() as f32 + NOISE_RMS * n1,
                a * self.phase.sin() as f32 + NOISE_RMS * n2,
            );
            if let Some(l) = loop_samples.get(i) {
                *z += l * LOOPBACK_GAIN;
            }
        }
        self.produced += out.len() as u64;
        Ok(())
    }
}

pub struct SimTx {
    shared: Arc<Mutex<Shared>>,
    cap: usize,
    /// `TRXD_SIM_TX_DUMP=<file>`: everything sent while keyed, as complex
    /// f32 at the stream rate (what would go to the DAC), for offline checks.
    dump: Option<std::fs::File>,
}

impl TxStream for SimTx {
    fn write(&mut self, iq: &[Complex32]) -> Result<(), String> {
        let mut s = self.shared.lock().unwrap();
        if !s.tx_on {
            return Ok(());
        }
        if let Some(f) = &mut self.dump {
            use std::io::Write;
            let bytes: Vec<u8> = iq.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
            let _ = f.write_all(&bytes);
        }
        s.loopback.extend(iq.iter().copied());
        let excess = s.loopback.len().saturating_sub(self.cap);
        s.loopback.drain(..excess);
        Ok(())
    }
}

pub fn open(cfg: &RadioConfig) -> Radio {
    let shared = Arc::new(Mutex::new(Shared::default()));
    let rate = cfg.stream_rate();
    Radio {
        control: Box::new(SimControl {
            shared: shared.clone(),
            rate,
            lo_offset_hz: cfg.lo_offset_hz,
            gain_db: cfg.rx_gain_db,
        }),
        rx: Box::new(SimRx {
            shared: shared.clone(),
            rate,
            realtime: cfg.sim_realtime,
            started: None,
            produced: 0,
            rng: 0x9E37_79B9_7F4A_7C15,
            phase: 0.0,
            keying: crate::morse::timeline(TEST_SIGNAL_TEXT),
            envelope: 0.0,
        }),
        tx: Box::new(SimTx {
            shared,
            cap: cfg.buffer_samples * 16,
            dump: std::env::var_os("TRXD_SIM_TX_DUMP").and_then(|p| std::fs::File::create(p).ok()),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_loops_back_into_rx() {
        let cfg = RadioConfig { sim_realtime: false, ..RadioConfig::default() };
        let mut r = open(&cfg);
        r.control.set_lo(100e6).unwrap();
        r.control.set_tx_rf(true).unwrap();
        r.tx.write(&vec![Complex32::new(1.0, 0.0); 1000]).unwrap();
        let mut buf = vec![Complex32::default(); 1000];
        r.rx.read(&mut buf).unwrap();
        let mean: f32 = buf.iter().map(|z| z.re).sum::<f32>() / 1000.0;
        assert!((mean - LOOPBACK_GAIN).abs() < 0.003, "mean {mean}");
    }
}
