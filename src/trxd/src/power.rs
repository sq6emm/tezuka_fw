//! Receive power measured on the stream, independent of the demodulator.
//!
//! The level meter (S-meter, dBm readout, calibration) reads IQ before any
//! mode-specific filter, AGC or decimation: the 48 kS/s channel every mode
//! shares (the stream mixed to the dial and decimated, one fixed filter) for
//! bands up to about +-20 kHz, the 384 kS/s stream after the FPGA decimator
//! for wider ones. Windowed FFTs (Blackman-Harris), averaged over about a
//! third of a second, normalised by Parseval so that
//!
//! * the sum of all bins is the mean power of the samples, |z|^2, i.e. a
//!   complex tone of amplitude A reads 20 log10 A dBFS (0 dBFS = full scale,
//!   |z| = 1), wherever it falls between bins, and
//! * white noise of variance s^2 reads s^2 / fs per Hz, whatever the
//!   measurement bandwidth.
//!
//! From one averaged spectrum: power in a band (tone and noise alike, the
//! edge bins weighted by the part inside), noise density from the median of
//! the bins around the band (the band itself, its window skirts and DC kept
//! out). Clipping is judged on the stream samples by the engine ([`CLIP`]).
//!
//! Wide signals (DATV) can be wider than the usable stream: their channel
//! power comes from a Maia spectrometer row ([`maia_band`]) instead, whose own
//! scale is tied to this one by a calibration constant (calib.rs).

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// FFT length: 23.4 Hz bins on the 48 kS/s channel, 187.5 Hz on the stream.
pub const N: usize = 2048;
/// Spectra averaged per reading.
const AVERAGES: usize = 8;
/// Usable part of the stream (the FPGA decimator's passband), as a share of
/// the rate either side of the centre.
pub const USABLE: f64 = 0.42;
/// Bins either side of a band edge kept out of the noise estimate (the
/// window's main lobe is 4 bins wide each way).
const GUARD_BINS: f64 = 6.0;
/// Bins either side of DC kept out of the noise estimate (LO leakage).
const DC_BINS: i64 = 4;
/// A sample magnitude at or above this share of full scale counts as
/// clipping (the 12-bit ADC tops out at 2047/2048).
pub const CLIP: f32 = 0.9;

/// One averaged, FFT-shifted power spectrum of the stream (DC in the middle).
#[derive(Debug, Clone)]
pub struct Spectrum {
    /// Linear power per bin; the sum over all bins is the mean |z|^2.
    pub bins: Vec<f64>,
    pub rate: f64,
    pub averages: usize,
    /// Running number of this spectrum (0, 1, ...).
    pub seq: u64,
}

impl Spectrum {
    pub fn bin_hz(&self) -> f64 {
        self.rate / self.bins.len() as f64
    }

    /// Fractional bin index of an offset from the centre, Hz.
    fn pos(&self, off_hz: f64) -> f64 {
        off_hz / self.bin_hz() + (self.bins.len() / 2) as f64
    }

    /// Is `[lo, hi]` (offsets from the centre, Hz) inside the usable stream?
    pub fn covers(&self, lo: f64, hi: f64) -> bool {
        let lim = self.rate * USABLE;
        lo.min(hi) >= -lim && lo.max(hi) <= lim
    }

    /// Power in `[lo, hi]` (offsets from the centre, Hz): linear, |z|^2
    /// units; edge bins weighted by the share of them inside.
    pub fn band_power(&self, lo: f64, hi: f64) -> f64 {
        let (lo, hi) = (lo.min(hi), lo.max(hi));
        // Bin k covers [k - 0.5, k + 0.5) in index units.
        let (a, b) = (self.pos(lo), self.pos(hi));
        let n = self.bins.len() as i64;
        let mut p = 0.0;
        let k0 = (a + 0.5).floor() as i64;
        let k1 = (b + 0.5).floor() as i64;
        for k in k0.max(0)..=k1.min(n - 1) {
            let (l, r) = (k as f64 - 0.5, k as f64 + 0.5);
            let w = (r.min(b) - l.max(a)).clamp(0.0, 1.0);
            p += self.bins[k as usize] * w;
        }
        p
    }

    /// Power of whatever narrow signal is strongest in `[lo, hi]`: the bins of
    /// the window's main lobe around the largest one (a carrier reads its
    /// full power, noise next to it does not count).
    pub fn peak_power(&self, lo: f64, hi: f64) -> f64 {
        let (a, b) = (self.pos(lo.min(hi)).round() as i64, self.pos(lo.max(hi)).round() as i64);
        let n = self.bins.len() as i64;
        let (a, b) = (a.max(0), b.min(n - 1));
        if a > b {
            return 0.0;
        }
        let k = (a..=b).max_by(|&x, &y| self.bins[x as usize].total_cmp(&self.bins[y as usize])).unwrap_or(a);
        ((k - 3).max(0)..=(k + 3).min(n - 1)).map(|i| self.bins[i as usize]).sum()
    }

    /// Noise density, linear |z|^2 per Hz: the median of the bins within
    /// `span` Hz of the band `[lo, hi]` but outside it (with a guard), away
    /// from DC and inside the usable stream, corrected from median to mean
    /// for the averaged chi-square spread. `None` with too few bins.
    pub fn noise_density(&self, lo: f64, hi: f64, span: f64) -> Option<f64> {
        let (lo, hi) = (lo.min(hi), lo.max(hi));
        let lim = self.rate * USABLE;
        let n = self.bins.len() as i64;
        let dc = (n / 2) as i64;
        let (a, b) = (self.pos(lo) - GUARD_BINS, self.pos(hi) + GUARD_BINS);
        let (s0, s1) = (self.pos((lo - span).max(-lim)), self.pos((hi + span).min(lim)));
        let mut v: Vec<f64> = (s0.ceil() as i64..=s1.floor() as i64)
            .filter(|&k| k >= 0 && k < n)
            .filter(|&k| (k as f64) < a || (k as f64) > b)
            .filter(|&k| (k - dc).abs() > DC_BINS)
            .map(|k| self.bins[k as usize])
            .collect();
        if v.len() < 16 {
            return None;
        }
        v.sort_by(|x, y| x.total_cmp(y));
        let median = v[v.len() / 2];
        // Each bin is an average of `averages` exponential (chi-square, 2
        // degrees of freedom each) powers: median = mean (1 - 2/(9 nu))^3,
        // nu = 2 x averages (Wilson-Hilferty).
        let nu = 2.0 * self.averages.max(1) as f64;
        let ratio = (1.0 - 2.0 / (9.0 * nu)).powi(3);
        Some(median / ratio / self.bin_hz())
    }
}

/// The averaging FFT the engine feeds with every stream block.
pub struct PowerMeter {
    rate: f64,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    /// Sum of window^2 x N: Parseval's normalisation.
    norm: f64,
    buf: Vec<Complex32>,
    acc: Vec<f64>,
    ffts: usize,
    skipped: usize,
    /// Samples left out after each FFT (CPU bound on the stream).
    skip: usize,
    seq: u64,
}

impl PowerMeter {
    /// `skip`: samples left out between transforms (0 on the channel: about
    /// 3 readings a second; 2N on the 384 kS/s stream: about 8).
    pub fn new(rate: f64, skip: usize) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(N);
        let window: Vec<f32> = (0..N)
            .map(|i| {
                let x = 2.0 * std::f64::consts::PI * i as f64 / N as f64;
                (0.35875 - 0.48829 * x.cos() + 0.14128 * (2.0 * x).cos() - 0.01168 * (3.0 * x).cos()) as f32
            })
            .collect();
        let wsq: f64 = window.iter().map(|w| (*w as f64) * (*w as f64)).sum();
        PowerMeter {
            rate,
            fft,
            window,
            norm: wsq * N as f64,
            buf: Vec::with_capacity(N),
            acc: vec![0.0; N],
            ffts: 0,
            skipped: skip,
            skip,
            seq: 0,
        }
    }

    /// Forget everything averaged so far (after a retune or gain change).
    pub fn reset(&mut self) {
        self.buf.clear();
        self.acc.iter_mut().for_each(|a| *a = 0.0);
        self.ffts = 0;
        self.skipped = self.skip;
    }

    /// Feed stream samples; a finished spectrum when one is due.
    pub fn feed(&mut self, iq: &[Complex32]) -> Option<Spectrum> {
        let mut out = None;
        let mut i = 0;
        while i < iq.len() {
            if self.skipped < self.skip {
                let k = (self.skip - self.skipped).min(iq.len() - i);
                self.skipped += k;
                i += k;
                continue;
            }
            let k = (N - self.buf.len()).min(iq.len() - i);
            self.buf.extend_from_slice(&iq[i..i + k]);
            i += k;
            if self.buf.len() == N {
                for (z, w) in self.buf.iter_mut().zip(&self.window) {
                    *z *= *w;
                    if !z.re.is_finite() || !z.im.is_finite() {
                        *z = Complex32::default();
                    }
                }
                self.fft.process(&mut self.buf);
                for (a, z) in self.acc.iter_mut().zip(&self.buf) {
                    *a += z.norm_sqr() as f64;
                }
                self.buf.clear();
                self.skipped = 0;
                self.ffts += 1;
                if self.ffts >= AVERAGES {
                    let half = N / 2;
                    let k = 1.0 / (self.norm * self.ffts as f64);
                    let mut bins = vec![0.0; N];
                    for (j, a) in self.acc.iter().enumerate() {
                        bins[(j + half) % N] = a * k;
                    }
                    out = Some(Spectrum { bins, rate: self.rate, averages: self.ffts, seq: self.seq });
                    self.seq += 1;
                    self.acc.iter_mut().for_each(|a| *a = 0.0);
                    self.ffts = 0;
                }
            }
        }
        out
    }
}

/// Band power and noise density from a Maia spectrometer row (FFT-shifted
/// linear powers over the full ADC rate, `adc` S/s), in the row's own
/// units: (power in `[lo, hi]`, density per Hz). Offsets from the row's
/// centre, Hz. The row's normalisation is the IP's, tied to the stream's
/// by `calib.maia_db`.
pub fn maia_band(row: &[f32], adc: f64, lo: f64, hi: f64) -> Option<(f64, Option<f64>)> {
    if row.is_empty() {
        return None;
    }
    let lim = adc * 0.45;
    if lo.min(hi) < -lim || lo.max(hi) > lim {
        return None;
    }
    let s = Spectrum { bins: row.iter().map(|&x| x as f64).collect(), rate: adc, averages: 8, seq: 0 };
    let p = s.band_power(lo, hi);
    let span = (hi - lo).abs().max(50e3);
    Some((p, s.noise_density(lo, hi, span)))
}

pub fn db(p: f64) -> f64 {
    10.0 * p.max(1e-30).log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    /// Deterministic Gaussian-ish noise (sum of uniforms), complex, variance s2.
    fn noise(n: usize, s2: f64, seed: u64) -> Vec<Complex32> {
        let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let mut u = || {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((x >> 33) as f64 / (1u64 << 31) as f64) - 0.5
        };
        // Sum of 12 uniforms: variance 1.
        let sd = (s2 / 2.0).sqrt();
        (0..n)
            .map(|_| {
                let a: f64 = (0..12).map(|_| u()).sum();
                let b: f64 = (0..12).map(|_| u()).sum();
                Complex32::new((a * sd) as f32, (b * sd) as f32)
            })
            .collect()
    }

    fn tone(n: usize, amp: f64, off: f64, phase0: f64) -> Vec<Complex32> {
        (0..n)
            .map(|i| {
                let p = phase0 + 2.0 * std::f64::consts::PI * off * i as f64 / RATE;
                Complex32::new((amp * p.cos()) as f32, (amp * p.sin()) as f32)
            })
            .collect()
    }

    fn spectrum(iq: &[Complex32]) -> Spectrum {
        let mut m = PowerMeter::new(RATE, 0);
        let mut last = None;
        for blk in iq.chunks(3840) {
            if let Some(s) = m.feed(blk) {
                last = Some(s);
            }
        }
        last.expect("a spectrum")
    }

    fn len() -> usize {
        // Enough for one averaged spectrum, and some slack.
        AVERAGES * N + 4 * N
    }

    #[test]
    fn a_tone_reads_its_dbfs_in_any_band_and_position() {
        // -30 dBFS, at bin centres and between bins, in every mode's filter.
        let amp = 10f64.powf(-30.0 / 20.0);
        for off in [700.0, 1_011.7, 1_500.0, -1_234.5, 9_000.0, -15_000.0] {
            let s = spectrum(&tone(len(), amp, off, 0.3));
            // Filters around it (relative to it): CW 500 Hz, SSB 2.7 kHz, AM 10 kHz, FM 16 kHz.
            for (a, b) in [(-250.0, 250.0), (-1350.0, 1350.0), (-5000.0, 5000.0), (-8000.0, 8000.0), (-2000.0, 700.0)] {
                let p = db(s.band_power(off + a, off + b));
                assert!((p - -30.0).abs() < 0.05, "off {off} band {a}..{b}: {p}");
            }
            let pk = db(s.peak_power(off - 1000.0, off + 1000.0));
            assert!((pk - -30.0).abs() < 0.05, "peak {pk}");
        }
    }

    #[test]
    fn noise_density_does_not_depend_on_the_bandwidth() {
        // Variance 1e-6: -60 dBFS total, -60 - 10 log10(48000) = -106.8 dBFS/Hz.
        let iq = noise(len() * 4, 1e-6, 7);
        let want = -60.0 - 10.0 * RATE.log10();
        let mut m = PowerMeter::new(RATE, 0);
        let mut sp = Vec::new();
        for blk in iq.chunks(3840) {
            if let Some(s) = m.feed(blk) {
                sp.push(s);
            }
        }
        let s = &sp[sp.len() - 1];
        for (lo, hi) in [(450.0, 950.0), (150.0, 2850.0), (-5e3, 5e3), (-8e3, 8e3)] {
            let d = db(s.noise_density(lo, hi, 10e3).unwrap());
            assert!((d - want).abs() < 0.6, "{lo}..{hi}: {d} vs {want}");
            // And the band power of noise is density x bandwidth.
            let p = db(s.band_power(lo, hi));
            let bw = hi - lo;
            assert!((p - (want + 10.0 * bw.log10())).abs() < 1.0, "{lo}..{hi}: band {p}");
        }
    }

    #[test]
    fn noise_estimate_ignores_a_strong_signal_next_to_it() {
        let amp = 10f64.powf(-20.0 / 20.0);
        let mut iq = noise(len(), 1e-8, 3);
        for (z, t) in iq.iter_mut().zip(tone(len(), amp, 700.0, 0.0)) {
            *z += t;
        }
        let s = spectrum(&iq);
        let want = -80.0 - 10.0 * RATE.log10();
        let d = db(s.noise_density(450.0, 950.0, 8e3).unwrap());
        assert!((d - want).abs() < 1.0, "{d} vs {want}");
    }

    #[test]
    fn reset_starts_a_fresh_average() {
        // Half an average of a loud tone, a reset, then a quiet one: the
        // reading is the quiet one alone.
        let mut m = PowerMeter::new(RATE, 0);
        let _ = m.feed(&tone(3 * N, 0.5, 1_000.0, 0.0));
        m.reset();
        let mut last = None;
        for blk in tone(len(), 0.01, 1_000.0, 0.0).chunks(960) {
            if let Some(s) = m.feed(blk) {
                last = Some(s);
            }
        }
        let p = db(last.unwrap().band_power(500.0, 1_500.0));
        assert!((p - -40.0).abs() < 0.05, "{p}");
    }

    #[test]
    fn usable_range() {
        let s = spectrum(&tone(len(), 0.01, 1000.0, 0.0));
        assert!(s.covers(-20e3, 20e3));
        assert!(!s.covers(10e3, 21e3));
    }

    #[test]
    fn maia_rows_band_power() {
        // A flat row of 1e-9 per bin over 3.072 MS/s / 4096 = 750 Hz bins:
        // 10 kHz holds 13.33 bins.
        let row = vec![1e-9f32; 4096];
        let (p, d) = maia_band(&row, 3.072e6, 100e3, 110e3).unwrap();
        assert!((p / 1e-9 - 10e3 / 750.0).abs() < 0.01, "{p}");
        let d = d.unwrap();
        assert!((d * 750.0 / 1e-9 - 1.0 / (1.0 - 2.0 / 144.0f64).powi(3)).abs() < 0.01, "{d}");
        assert!(maia_band(&row, 3.072e6, 1.5e6, 1.6e6).is_none());
    }
}
