//! Spectrum scope for the web UI: averaged, windowed FFT power spectra,
//! reduced to a fixed number of display columns (peak per column, so a
//! narrow carrier never falls between pixels).
//!
//! Two instances in the transceiver: one on the full stream (wide spans) and
//! one on the 48 kS/s channel (spans up to 20 kHz, around the VFO). Only
//! the one the UI is looking at runs, and neither runs with no browser
//! connected.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// Columns sent to the browser per row.
pub const COLUMNS: usize = 1024;

pub struct Scope {
    n: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<Complex32>,
    acc: Vec<f32>,
    ffts: usize,
    /// FFTs averaged per output row.
    per_row: usize,
    /// Input samples to skip between FFTs, to bound the CPU cost.
    skip: usize,
    skipped: usize,
}

impl Scope {
    /// `n`-point FFTs over a `rate` stream, `rows_per_s` rows out, averaging
    /// at most `ffts_per_s` transforms a second.
    pub fn new(n: usize, rate: f64, rows_per_s: f64, ffts_per_s: f64) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(n);
        // Blackman-Harris: -92 dB sidelobes, so a strong signal does not
        // smear a skirt across the waterfall.
        let window: Vec<f32> = (0..n)
            .map(|i| {
                let x = std::f32::consts::TAU * i as f32 / n as f32;
                0.35875 - 0.48829 * x.cos() + 0.14128 * (2.0 * x).cos() - 0.01168 * (3.0 * x).cos()
            })
            .collect();
        let available = rate / n as f64;
        let used = ffts_per_s.min(available);
        let skip = ((rate - used * n as f64) / used).max(0.0) as usize;
        Scope {
            n,
            fft,
            window,
            buf: Vec::with_capacity(n),
            acc: vec![0.0; n],
            ffts: 0,
            per_row: (used / rows_per_s).round().max(1.0) as usize,
            skip,
            skipped: 0,
        }
    }

    pub fn bin_hz(&self, rate: f64) -> f64 {
        rate / self.n as f64
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.acc.iter_mut().for_each(|a| *a = 0.0);
        self.ffts = 0;
        self.skipped = 0;
    }

    /// Feed samples; returns a finished row of `n` powers (linear, DC in the
    /// middle, i.e. already fft-shifted) when one is due.
    pub fn process(&mut self, iq: &[Complex32]) -> Option<Vec<f32>> {
        let mut row = None;
        let mut i = 0;
        while i < iq.len() {
            if self.skipped < self.skip {
                let k = (self.skip - self.skipped).min(iq.len() - i);
                self.skipped += k;
                i += k;
                continue;
            }
            let k = (self.n - self.buf.len()).min(iq.len() - i);
            self.buf.extend_from_slice(&iq[i..i + k]);
            i += k;
            if self.buf.len() == self.n {
                for (z, w) in self.buf.iter_mut().zip(&self.window) {
                    *z *= *w;
                }
                self.fft.process(&mut self.buf);
                for (a, z) in self.acc.iter_mut().zip(&self.buf) {
                    *a += z.norm_sqr();
                }
                self.buf.clear();
                self.skipped = 0;
                self.ffts += 1;
                if self.ffts >= self.per_row {
                    let half = self.n / 2;
                    // Window power gain and FFT length, so a full-scale tone
                    // reads about 0 dBFS.
                    let norm = 1.0 / (self.ffts as f32 * (self.n as f32 * 0.35875).powi(2));
                    let mut out = vec![0.0; self.n];
                    for k in 0..self.n {
                        out[(k + half) % self.n] = self.acc[k] * norm;
                    }
                    self.acc.iter_mut().for_each(|a| *a = 0.0);
                    self.ffts = 0;
                    row = Some(out);
                }
            }
        }
        row
    }
}

/// Cut `[center - span/2, center + span/2]` out of a shifted power row that
/// covers `[row_center - rate/2, row_center + rate/2]`, and reduce it to
/// [`COLUMNS`] display values (peak per column): `v = (dBFS + 160) * 1.5`,
/// i.e. 2/3 dB steps from -160 dBFS (1) to +10 dBFS (255). Columns outside
/// the row read 0.
pub fn render(row: &[f32], row_center: f64, rate: f64, center: f64, span: f64) -> Vec<u8> {
    let n = row.len();
    let bin = rate / n as f64;
    let first = (center - span / 2.0 - (row_center - rate / 2.0)) / bin;
    let per_col = span / bin / COLUMNS as f64;
    (0..COLUMNS)
        .map(|c| {
            let a = first + c as f64 * per_col;
            let b = a + per_col.max(1.0);
            let (lo, hi) = (a.floor() as i64, (b.ceil() as i64).max(a.floor() as i64 + 1));
            let mut peak = 0.0f32;
            let mut any = false;
            for k in lo..hi {
                if k >= 0 && (k as usize) < n {
                    peak = peak.max(row[k as usize]);
                    any = true;
                }
            }
            if !any {
                return 0;
            }
            let db = 10.0 * (peak + 1e-20).log10();
            ((db + 160.0) * 1.5).round().clamp(1.0, 255.0) as u8
        })
        .collect()
}

/// Paint over the LO's DC spike in a rendered row (views too wide to keep the
/// LO out of): the columns within `half_hz` of `lo` get the line between their
/// neighbours. Nothing to do when the LO is off the view.
pub fn blank_dc(cols: &mut [u8], lo: f64, center: f64, span: f64, half_hz: f64) {
    let n = cols.len();
    let col_hz = span / n as f64;
    let at = (lo - (center - span / 2.0)) / col_hz;
    let w = half_hz / col_hz + 0.5;
    let (a, b) = ((at - w).floor() as i64, (at + w).ceil() as i64);
    if b < 0 || a >= n as i64 {
        return;
    }
    let left = if a > 0 { Some(cols[a as usize - 1]) } else { None };
    let right = if b + 1 < n as i64 { Some(cols[b as usize + 1]) } else { None };
    let (l, r) = match (left, right) {
        (Some(l), Some(r)) => (l, r),
        (Some(x), None) | (None, Some(x)) => (x, x),
        (None, None) => return,
    };
    let (lo_i, hi_i) = (a.max(0) as usize, (b.min(n as i64 - 1)) as usize);
    for i in lo_i..=hi_i {
        let t = (i as f64 - (a - 1) as f64) / ((b + 1 - (a - 1)) as f64);
        cols[i] = (l as f64 + (r as f64 - l as f64) * t).round() as u8;
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn dc_is_painted_over_and_only_in_view() {
        let mut cols = vec![10u8; COLUMNS];
        let (center, span) = (1_000_000.0, 1_024_000.0); // 1 kHz a column
        cols[512] = 200;
        cols[513] = 150;
        super::blank_dc(&mut cols, center, center, span, 2_000.0);
        assert!(cols.iter().all(|&c| c == 10), "{:?}", &cols[505..520]);
        let mut far = vec![10u8; COLUMNS];
        far[0] = 99;
        super::blank_dc(&mut far, center + 3e6, center, span, 2_000.0);
        assert_eq!(far[0], 99);
    }

    use super::*;

    #[test]
    fn a_tone_lands_in_the_right_column() {
        let rate = 48_000.0;
        let mut s = Scope::new(2048, rate, 10.0, 40.0);
        let f = 3_000.0;
        let iq: Vec<Complex32> = (0..48_000)
            .map(|i| {
                let p = std::f32::consts::TAU * f as f32 * i as f32 / rate as f32;
                Complex32::new(p.cos(), p.sin()) * 0.5
            })
            .collect();
        let mut last = None;
        for chunk in iq.chunks(480) {
            if let Some(r) = s.process(chunk) {
                last = Some(r);
            }
        }
        let row = last.expect("a row");
        let cols = render(&row, 0.0, rate, 0.0, rate);
        let (peak_col, &peak) = cols.iter().enumerate().max_by_key(|(_, v)| **v).unwrap();
        let expect = ((f + rate / 2.0) / rate * COLUMNS as f64) as usize;
        assert!((peak_col as i64 - expect as i64).abs() <= 1, "{peak_col} vs {expect}");
        // 0.5 amplitude = -6 dBFS
        let db = peak as f64 / 1.5 - 160.0;
        assert!((db + 6.0).abs() < 2.0, "{db}");
        // Outside a narrower span centred elsewhere: blank columns.
        let edge = render(&row, 0.0, rate, 40_000.0, 20_000.0);
        assert_eq!(edge[COLUMNS - 1], 0);
    }
}
