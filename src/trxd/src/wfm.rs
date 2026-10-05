//! Wide FM (broadcast) for the Cortex-A9: mono, sized for this board.
//!
//! sdroxide's `WfmDemod` is a PC demodulator (a 63-tap channel filter at the
//! channel rate, two 383-tap 15 kHz filters, the stereo pilot PLL): on the A9
//! it took 90 % of a core. Here the FPGA DDC has already cut the channel to
//! +/-85 kHz at 192 kS/s, and the page gets 24 kHz mono audio, so the chain is
//! a discriminator, one short decimating filter to 48 kHz (it only has to keep
//! the 38 kHz stereo and 57 kHz RDS subcarriers out of 0-12 kHz), de-emphasis,
//! and sdroxide's RDS receiver on the same multiplex.

use num_complex::Complex32;
use sdroxide_dsp::{Demodulator, RdsRx, RealFirDecim};
use sdroxide_types::RdsData;

/// Broadcast FM's nominal peak deviation.
const DEVIATION_HZ: f64 = 75_000.0;
/// Broadcast processing pushes peaks past nominal deviation: room below 1.0.
const HEADROOM: f32 = 0.7;
/// The audio filter: passband to 13 kHz (the page's 24 kHz audio ends at 12),
/// stopband from 36 kHz, where the stereo subcarrier would fold onto it.
const AUDIO_TAPS: usize = 47;
const AUDIO_CUTOFF_HZ: f64 = 15_000.0;
/// European de-emphasis.
const DEEMPH_S: f64 = 50e-6;

pub struct WfmLite {
    rate: f64,
    prev: Complex32,
    scale: f32,
    lpf: RealFirDecim,
    deemph: f32,
    alpha: f32,
    dc_x: f32,
    dc_y: f32,
    rds: Option<RdsRx>,
    power: f32,
    mpx: Vec<f32>,
    audio: Vec<f32>,
}

impl WfmLite {
    pub fn new(rate: f64) -> Self {
        let out = rate / 4.0;
        WfmLite {
            rate,
            prev: Complex32::new(1.0, 0.0),
            scale: (rate / (std::f64::consts::TAU * DEVIATION_HZ)) as f32,
            lpf: RealFirDecim::new(AUDIO_TAPS, AUDIO_CUTOFF_HZ, rate, 4),
            deemph: 0.0,
            alpha: (1.0 - (-1.0 / (out * DEEMPH_S)).exp()) as f32,
            dc_x: 0.0,
            dc_y: 0.0,
            rds: RdsRx::new(rate),
            power: 0.0,
            mpx: Vec::new(),
            audio: Vec::new(),
        }
    }
}

/// atan2 to about 2e-4 rad (a polynomial in min/max of |x|, |y|), a
/// fraction of libm's cost: the discriminator runs at 192 kS/s.
#[inline]
pub fn fast_atan2(y: f32, x: f32) -> f32 {
    use std::f32::consts::{FRAC_PI_2, PI};
    let (ax, ay) = (x.abs(), y.abs());
    let (mn, mx) = if ax < ay { (ax, ay) } else { (ay, ax) };
    let a = mn / (mx + 1e-30);
    let s = a * a;
    let mut r = ((-0.046_496_475 * s + 0.159_314_22) * s - 0.327_622_76) * s * a + a;
    if ay > ax {
        r = FRAC_PI_2 - r;
    }
    if x < 0.0 {
        r = PI - r;
    }
    if y < 0.0 { -r } else { r }
}

impl Demodulator for WfmLite {
    fn process(&mut self, iq: &[Complex32], out: &mut Vec<f32>) {
        self.mpx.clear();
        let mut p = 0.0f32;
        for &z in iq {
            let d = z * self.prev.conj();
            self.prev = z;
            p += z.norm_sqr();
            self.mpx.push(fast_atan2(d.im, d.re) * self.scale);
        }
        if !iq.is_empty() {
            self.power += (p / iq.len() as f32 - self.power) * 0.2;
        }
        if let Some(r) = &mut self.rds {
            r.process(&self.mpx);
        }
        self.audio.clear();
        self.lpf.process(&self.mpx, &mut self.audio);
        for &x in &self.audio {
            self.deemph += self.alpha * (x - self.deemph);
            // DC out (a station off the dial reads as an offset).
            let y = self.deemph - self.dc_x + 0.9995 * self.dc_y;
            self.dc_x = self.deemph;
            self.dc_y = y;
            out.push(y * HEADROOM);
        }
    }

    fn set_filter(&mut self, _lo_hz: f32, _hi_hz: f32) {}

    fn audio_rate(&self) -> f64 {
        self.rate / 4.0
    }

    fn power_dbfs(&self) -> f32 {
        10.0 * self.power.max(1e-20).log10()
    }

    fn take_rds(&mut self) -> Option<RdsData> {
        self.rds.as_mut().and_then(|r| r.take())
    }

    fn reset_rds(&mut self) {
        if let Some(r) = &mut self.rds {
            r.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fast_atan2_is_close() {
        let mut worst = 0.0f32;
        for i in 0..3600 {
            let a = (i as f32 - 1800.0) * std::f32::consts::PI / 1800.0;
            for m in [0.01f32, 1.0, 300.0] {
                let e = fast_atan2(m * a.sin(), m * a.cos()) - a.sin().atan2(a.cos());
                let e = (e + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
                worst = worst.max(e.abs());
            }
        }
        assert!(worst < 5e-4, "{worst} rad");
    }

    #[test]
    fn a_tone_comes_out_at_its_pitch_and_level() {
        let rate = 192_000.0;
        let mut d = WfmLite::new(rate);
        let mut ph = 0.0f64;
        let iq: Vec<Complex32> = (0..192_000)
            .map(|i| {
                ph += std::f64::consts::TAU * 75_000.0 * (std::f64::consts::TAU * 1_000.0 * i as f64 / rate).sin() / rate;
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut out = Vec::new();
        d.process(&iq, &mut out);
        // (less the decimating filter's start-up)
        assert!((47_950..=48_000).contains(&out.len()), "{}", out.len());
        let tail = &out[24_000..];
        let crossings = tail.windows(2).filter(|w| w[0] < 0.0 && w[1] >= 0.0).count() as f64;
        assert!((crossings * 2.0 - 1_000.0).abs() < 10.0, "{} Hz", crossings * 2.0);
        // Full deviation, de-emphasised at 1 kHz (about -0.4 dB): close to the headroom.
        let peak = tail.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
        assert!((0.55..0.72).contains(&peak), "peak {peak}");
    }
}
