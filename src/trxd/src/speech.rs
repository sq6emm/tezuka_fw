//! Speech processing for voice transmit (48 kHz audio, before the modulator):
//! a level meter for the operator, and when COMP is on a compressor with a
//! noise gate, ahead of the CESSB envelope limiter.
//!
//! ```text
//! mic -> high-pass 150 Hz -> RMS detector -> gain: compress above the
//!        threshold (4:1), expand below the gate, make-up gain -> soft limit
//! ```
//!
//! CESSB (after the modulator) keeps the RF envelope under the ceiling without
//! splatter but does not level the voice: quiet syllables stay quiet. This
//! stage does the levelling; CESSB then only trims the peaks. The gate keeps
//! the compressor from pulling the room noise up in the pauses.

/// Sample rate of the transmit audio.
const RATE: f32 = 48_000.0;
/// Compression ratio above the threshold.
const RATIO: f32 = 4.0;
/// Below this (dBFS, detector level) the expander closes...
const GATE_DB: f32 = -40.0;
/// ...at this ratio, by at most this much.
const GATE_RATIO: f32 = 3.0;
const GATE_MAX_DB: f32 = 20.0;

fn coef(ms: f32) -> f32 {
    1.0 - (-1.0 / (RATE * ms / 1000.0)).exp()
}

pub struct SpeechProc {
    /// High-pass state (one pole, 150 Hz).
    hp_x: f32,
    hp_y: f32,
    hp_a: f32,
    /// Mean square, attack/release smoothed.
    ms: f32,
    /// Gain in dB actually applied (smoothed), and its target.
    gain_db: f32,
    /// Peak |x| before processing since the meter last read it.
    peak: f32,
    /// Largest compression (gain reduction above the threshold) since the
    /// meter last read it, dB.
    gr_max: f32,
}

impl Default for SpeechProc {
    fn default() -> Self {
        let rc = 1.0 / (std::f32::consts::TAU * 150.0);
        SpeechProc { hp_x: 0.0, hp_y: 0.0, hp_a: rc / (rc + 1.0 / RATE), ms: 1e-10, gain_db: 0.0, peak: 0.0, gr_max: 0.0 }
    }
}

impl SpeechProc {
    /// Meter `audio` and, with `comp_db` (0..20, the COMP setting), compress
    /// it in place: threshold at -comp_db dBFS, 4:1, make-up of 3/4 of it, so
    /// full-scale peaks stay near full scale and the rest comes up.
    pub fn process(&mut self, audio: &mut [f32], comp_db: Option<f32>) {
        for x in audio.iter() {
            self.peak = self.peak.max(x.abs());
        }
        let Some(depth) = comp_db else {
            return;
        };
        let depth = depth.clamp(0.0, 20.0);
        let thr = -depth;
        let makeup = depth * (1.0 - 1.0 / RATIO);
        let (att, rel) = (coef(5.0), coef(200.0));
        let (g_att, g_rel) = (coef(2.0), coef(120.0));
        for x in audio.iter_mut() {
            // High-pass: rumble and handling noise only waste transmitter power.
            let y = self.hp_a * (self.hp_y + *x - self.hp_x);
            self.hp_x = *x;
            self.hp_y = y;
            let p = y * y;
            self.ms += (p - self.ms) * if p > self.ms { att } else { rel };
            let level = 10.0 * self.ms.max(1e-12).log10() + 3.0; // RMS -> about the peak of speech
            let squeeze = (level - thr).max(0.0) * (1.0 - 1.0 / RATIO);
            let mut target = makeup - squeeze;
            if level < GATE_DB {
                target -= ((GATE_DB - level) * (GATE_RATIO - 1.0)).min(GATE_MAX_DB);
            }
            // Gain moves fast down, slowly up: no pumping on every syllable.
            self.gain_db += (target - self.gain_db) * if target < self.gain_db { g_att } else { g_rel };
            let out = y * 10f32.powf(self.gain_db / 20.0);
            // A soft ceiling for the rare overshoot before the detector catches up.
            *x = if out.abs() < 0.9 { out } else { out.signum() * (0.9 + 0.1 * ((out.abs() - 0.9) / 0.1).tanh()) };
            // What the compressor takes off the voice (the gate in the pauses is not that).
            self.gr_max = self.gr_max.max(squeeze);
        }
    }

    /// Peak level since the last call, dBFS (for the MIC meter).
    pub fn take_peak_db(&mut self) -> f32 {
        let p = std::mem::take(&mut self.peak);
        20.0 * p.max(1e-6).log10()
    }

    /// Largest gain reduction since the last call, dB (for the COMP meter).
    pub fn take_gr_db(&mut self) -> f32 {
        std::mem::take(&mut self.gr_max).max(0.0)
    }

    /// A new transmission: forget the last one's level.
    pub fn reset(&mut self) {
        *self = SpeechProc::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(amp: f32, n: usize) -> Vec<f32> {
        (0..n).map(|i| amp * (std::f32::consts::TAU * 1000.0 * i as f32 / RATE).sin()).collect()
    }
    fn rms_db(x: &[f32]) -> f32 {
        10.0 * (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).log10()
    }

    #[test]
    fn quiet_speech_comes_up_loud_speech_stays() {
        let mut p = SpeechProc::default();
        let mut loud = tone(0.9, 48_000);
        p.process(&mut loud, Some(12.0));
        let mut p = SpeechProc::default();
        let mut quiet = tone(0.09, 48_000); // 20 dB down
        p.process(&mut quiet, Some(12.0));
        let (l, q) = (rms_db(&loud[24_000..]), rms_db(&quiet[24_000..]));
        assert!(l - q < 12.0, "a 20 dB difference should shrink to well under that: {l:.1} vs {q:.1}");
        assert!(loud.iter().all(|v| v.abs() <= 1.0), "never above full scale");
    }

    #[test]
    fn the_gate_keeps_room_noise_down_and_off_is_untouched() {
        let mut p = SpeechProc::default();
        let mut hiss = tone(0.002, 48_000); // -54 dBFS
        let before = rms_db(&hiss);
        p.process(&mut hiss, Some(12.0));
        assert!(rms_db(&hiss[24_000..]) <= before + 1.0, "no pumping up of the pauses");
        let mut p = SpeechProc::default();
        let orig = tone(0.3, 4_800);
        let mut x = orig.clone();
        p.process(&mut x, None);
        assert_eq!(x, orig, "COMP off: metering only");
        assert!((p.take_peak_db() - 20.0 * 0.3f32.log10()).abs() < 0.1);
    }
}
