//! Text-to-CW keyer: queued text out as a keyed complex tone, with
//! raised-cosine edges so the keying clicks stay inside a few hundred Hz.

use std::collections::VecDeque;

use num_complex::Complex32;

use crate::morse;

/// Rise / fall time of the keying envelope.
const EDGE_S: f64 = 0.005;

pub struct CwKeyer {
    rate: f64,
    offset_hz: f64,
    wpm: f32,
    /// Key state per sample still to send.
    pending: VecDeque<bool>,
    dot_samples: usize,
    phase: f64,
    env: f64,
}

impl CwKeyer {
    pub fn new(rate: f64, offset_hz: f64, wpm: f32) -> Self {
        let mut k = CwKeyer {
            rate,
            offset_hz,
            wpm,
            pending: VecDeque::new(),
            dot_samples: 0,
            phase: 0.0,
            env: 0.0,
        };
        k.set_wpm(wpm);
        k
    }

    pub fn set_wpm(&mut self, wpm: f32) {
        self.wpm = wpm.clamp(5.0, 60.0);
        self.dot_samples = (morse::dot_seconds(self.wpm) * self.rate).round() as usize;
    }

    pub fn wpm(&self) -> f32 {
        self.wpm
    }

    pub fn set_offset_hz(&mut self, hz: f64) {
        self.offset_hz = hz;
    }

    pub fn send(&mut self, text: &str) {
        for key in morse::timeline(&text.to_ascii_uppercase()) {
            self.pending.extend(std::iter::repeat_n(key, self.dot_samples));
        }
    }

    pub fn abort(&mut self) {
        self.pending.clear();
    }

    /// True while there is text left or the envelope has not decayed.
    pub fn busy(&self) -> bool {
        !self.pending.is_empty() || self.env > 1e-4
    }

    /// Append `n` samples of keyed tone (silence when idle).
    pub fn render(&mut self, n: usize, out: &mut Vec<Complex32>) {
        let step = std::f64::consts::TAU * self.offset_hz / self.rate;
        let edge = 1.0 / (EDGE_S * self.rate);
        for _ in 0..n {
            let key = self.pending.pop_front().unwrap_or(false);
            // Linear ramp on a raised-cosine map: smooth at both ends.
            self.env = if key { (self.env + edge).min(1.0) } else { (self.env - edge).max(0.0) };
            let shaped = 0.5 - 0.5 * (std::f64::consts::PI * self.env).cos();
            self.phase = (self.phase + step) % std::f64::consts::TAU;
            out.push(Complex32::new((shaped * self.phase.cos()) as f32, (shaped * self.phase.sin()) as f32));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sends_the_right_duration_and_goes_idle() {
        let rate = 48_000.0;
        let mut k = CwKeyer::new(rate, 700.0, 20.0);
        k.send("E");
        assert!(k.busy());
        let mut out = Vec::new();
        k.render(48_000, &mut out);
        assert!(!k.busy());
        // "E" = 1 dot of 60 ms at 20 WPM: energy ~ 60 ms worth of unit power.
        let energy: f32 = out.iter().map(|z| z.norm_sqr()).sum::<f32>() / rate as f32;
        assert!((energy - 0.06).abs() < 0.01, "{energy}");
    }
}
