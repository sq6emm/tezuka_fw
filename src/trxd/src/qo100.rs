//! QO-100 beacon lock: the narrowband transponder's beacons as the
//! frequency reference, so the dial reads true downlink frequencies while
//! the LNB drifts (what SDR Console calls the geostationary beacon, and
//! others beacon lock or beacon tracking).
//!
//! The lock measures where a beacon really sits in the stream against where
//! it should be, and the engine moves the transverter's effective LO by the
//! difference (`Settings::xvtr_corr`, remembered across restarts). The LO
//! on the AD936x never moves for it: the channel follows inside the stream.
//!
//! Beacons (AMSAT-DL bandplan): the lower CW beacon on 10489.500 MHz and
//! the upper on 10490.000 MHz are F1A (key-up 400 Hz below the carrier, so
//! a carrier is always there), the middle beacon on 10489.750 MHz is
//! 400 bit/s BPSK. Whichever lies in the stream is used, the BPSK one first
//! (its guard band is the widest): squaring BPSK gives a line at twice its
//! carrier offset, which is read like a CW line. The CW lines come as a
//! pair 400 Hz apart; the nominal is the upper one.
//!
//! Every `CAPTURE_GAP_S` a capture of `n` samples is windowed and
//! transformed (and squared and transformed); `CAPTURES` of them are
//! averaged, then the peak inside the search window around the expected
//! offset is interpolated to a fraction of a bin (about 12 Hz bins at
//! 384 kS/s). The result is accepted when the peak stands well above the
//! window's noise.
//!
//! Before anything is known (a DRO LNB sits up to a few MHz off and
//! drifts), the beacons' pattern is searched for: the lower CW line, the
//! PSK hump 250 kHz up, the multimedia beacon's 7 kHz block 493.5 kHz up
//! and the upper CW line 500 kHz up, two or more of them in view at once.
//! The engine feeds the FPGA spectrometer's full-rate rows for that
//! (`push_wide`: +/-12 MHz on the wide images, with the analog filter
//! opened), and the averaged stream spectrum is searched too. The offset
//! where the beacons line up best starts the fine tracking. Past that
//! reach the operator marks the beacon on the scope (`mark`), as SDR
//! Console asks.

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

use crate::settings::Transverter;

pub const NB_LOWER_HZ: f64 = 10_489_500_000.0;
pub const NB_MIDDLE_HZ: f64 = 10_489_750_000.0;
pub const NB_UPPER_HZ: f64 = 10_490_000_000.0;
/// The multimedia beacon (8APSK, 7 kHz wide), part of the pattern only.
pub const NB_MULTIMEDIA_HZ: f64 = 10_489_993_500.0;
/// F1A: the key-up tone this far below the carrier.
const F1A_SHIFT_HZ: f64 = 400.0;
/// Captures per estimate, and the time between captures.
const CAPTURES: u32 = 8;
const CAPTURE_GAP_S: f64 = 0.25;
/// The search window around the expected offset once a beacon is known:
/// a station next to the beacon must not take over, but a DRO LNB drifts,
/// so it doubles on every miss up to the wide one.
const SEARCH_HZ: f64 = 5_000.0;
const SEARCH_WIDE_HZ: f64 = 20_000.0;
/// The pattern search's step, the least a beacon must stand above the
/// band's own noise around it to count, the least the pattern must sum
/// to, and how far off a DRO LNB is looked for.
const PATTERN_STEP_HZ: f64 = 100.0;
const PATTERN_LINE_DB: f64 = 6.0;
const PATTERN_MIN_DB: f64 = 24.0;
const PATTERN_REACH_HZ: f64 = 5.0e6;
/// The best alignment must beat every other one this far away by this
/// much: two stations the right distance apart do not line up uniquely.
const PATTERN_APART_HZ: f64 = 20_000.0;
const PATTERN_UNIQUE_DB: f64 = 6.0;
/// The local floor: the median over this much spectrum around a beacon.
const PATTERN_FLOOR_HZ: f64 = 60_000.0;
/// The peak against the window's median power.
const MIN_SNR_DB: f64 = 16.0;
/// Misses in a row before the lock is reported lost (and the search widens).
const MISSES_UNLOCK: u32 = 3;

/// The beacon nearest a frequency on the air (where the operator clicked).
pub fn nearest(hz: f64) -> Beacon {
    [Beacon::Lower, Beacon::Middle, Beacon::Upper]
        .into_iter()
        .min_by(|a, b| (a.hz() - hz).abs().total_cmp(&(b.hz() - hz).abs()))
        .unwrap()
}

/// Whether a transverter brings the narrowband transponder.
pub fn covers_nb(t: &Transverter) -> bool {
    t.rf_min <= NB_MIDDLE_HZ && NB_MIDDLE_HZ <= t.rf_max
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Beacon {
    Lower,
    Middle,
    Upper,
}

impl Beacon {
    pub fn hz(self) -> f64 {
        match self {
            Beacon::Lower => NB_LOWER_HZ,
            Beacon::Middle => NB_MIDDLE_HZ,
            Beacon::Upper => NB_UPPER_HZ,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Beacon::Lower => "CW 10489.500",
            Beacon::Middle => "PSK 10489.750",
            Beacon::Upper => "CW 10490.000",
        }
    }
}

/// What one estimate found.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub beacon: Beacon,
    /// Measured minus nominal, Hz (positive: the beacon reads high, the
    /// effective LO is too high).
    pub err_hz: f64,
    pub snr_db: f64,
    /// From the pattern search (100 Hz steps): to be applied whole, the
    /// fine stage looks where it points.
    pub coarse: bool,
}

pub struct Lock {
    rate: f64,
    n: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<Complex32>,
    /// Samples until the next capture starts (while not capturing).
    wait: usize,
    acc: Vec<f32>,
    acc_sq: Vec<f32>,
    captures: u32,
    pub beacon: Option<Beacon>,
    pub last: Option<Reading>,
    pub locked: bool,
    /// A beacon was found or marked once: the search stays narrow (the
    /// LNB drifts slowly; a wide window would take a station instead).
    pub acquired: bool,
    /// Full-rate spectrometer rows summed for the wide pattern search.
    wide: Vec<f32>,
    wide_rows: u32,
    /// The search window now (grows on misses).
    search: f64,
    misses: u32,
    /// What the pattern search last scored, dB (for the page).
    pub pattern_db: Option<f64>,
    /// The last spectrum the wide search looked at and what it made of
    /// it (cells, Hz per cell, the best (error, score, runner-up)), for
    /// the `beacon dump` command.
    pub last_wide: Option<(Vec<f32>, f64, Option<(f64, f64, f64)>)>,
    /// Estimates accepted since the start.
    pub updates: u32,
}

impl Lock {
    pub fn new(rate: f64) -> Self {
        // ~12 Hz bins at 384 kS/s; a power of two near rate / 12.
        let n = (rate / 12.0).log2().round().exp2() as usize;
        let n = n.clamp(4096, 65536);
        let window = (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()).collect();
        Lock {
            rate,
            n,
            fft: FftPlanner::new().plan_fft_forward(n),
            window,
            buf: Vec::with_capacity(n),
            wait: 0,
            acc: vec![0.0; n],
            acc_sq: vec![0.0; n],
            captures: 0,
            beacon: None,
            last: None,
            locked: false,
            acquired: false,
            wide: Vec::new(),
            wide_rows: 0,
            search: SEARCH_HZ,
            misses: 0,
            pattern_db: None,
            last_wide: None,
            updates: 0,
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// The operator marked a beacon (the correction already moved it onto
    /// its nominal): track it from here with the narrow window.
    pub fn mark(&mut self, b: Beacon) {
        self.beacon = Some(b);
        self.locked = true;
        self.acquired = true;
        self.search = SEARCH_HZ;
        self.misses = 0;
        self.acc.iter_mut().for_each(|p| *p = 0.0);
        self.acc_sq.iter_mut().for_each(|p| *p = 0.0);
        self.captures = 0;
    }

    /// Hz per bin.
    fn bin_hz(&self) -> f64 {
        self.rate / self.n as f64
    }

    /// Whether a beacon can be read with the stream centred on `center_hz`.
    fn fits(&self, b: Beacon, center_hz: f64) -> bool {
        let half = self.rate / 2.0;
        let margin = SEARCH_WIDE_HZ + 2_000.0;
        let off = b.hz() - center_hz;
        match b {
            Beacon::Middle => (2.0 * off).abs() + 2.0 * margin < half,
            _ => off.abs() + margin < half,
        }
    }

    /// The beacon to use with the stream centred on `center_hz` (on the
    /// air): the one in use (or marked) while it still fits, else the
    /// middle one when its doubled offset fits, else the nearer CW beacon
    /// that fits. None: no beacon in the stream.
    pub fn pick(&self, center_hz: f64) -> Option<Beacon> {
        if let Some(b) = self.beacon.filter(|b| self.fits(*b, center_hz)) {
            return Some(b);
        }
        let half = self.rate / 2.0;
        let margin = SEARCH_WIDE_HZ + 2_000.0;
        let off = |b: Beacon| b.hz() - center_hz;
        if self.fits(Beacon::Middle, center_hz) {
            return Some(Beacon::Middle);
        }
        [Beacon::Lower, Beacon::Upper]
            .into_iter()
            .filter(|b| off(*b).abs() + margin < half)
            .min_by(|a, b| off(*a).abs().total_cmp(&off(*b).abs()))
    }

    /// Feed stream samples (in the on-air sense: offsets up are frequencies
    /// up). `center_hz` is where the stream's centre sits on the air with
    /// the correction so far. A reading comes back once per averaging
    /// period; None in between, and for a period without a usable peak.
    pub fn push(&mut self, iq: &[Complex32], center_hz: f64) -> Option<Reading> {
        let mut out = None;
        let mut i = 0;
        while i < iq.len() {
            if self.wait > 0 {
                let skip = self.wait.min(iq.len() - i);
                self.wait -= skip;
                i += skip;
                continue;
            }
            let take = (self.n - self.buf.len()).min(iq.len() - i);
            self.buf.extend_from_slice(&iq[i..i + take]);
            i += take;
            if self.buf.len() == self.n {
                self.capture();
                self.buf.clear();
                self.wait = ((CAPTURE_GAP_S * self.rate) as usize).saturating_sub(self.n);
                if self.captures >= CAPTURES {
                    out = self.estimate(center_hz);
                    self.acc.iter_mut().for_each(|p| *p = 0.0);
                    self.acc_sq.iter_mut().for_each(|p| *p = 0.0);
                    self.captures = 0;
                }
            }
        }
        out
    }

    /// One capture into the averages: the spectrum, and the spectrum of
    /// the squared signal (BPSK's carrier comes back at twice the offset).
    fn capture(&mut self) {
        let n = self.n;
        let mut a: Vec<Complex32> = self.buf.iter().zip(&self.window).map(|(z, w)| z * *w).collect();
        let mut b: Vec<Complex32> = self.buf.iter().zip(&self.window).map(|(z, w)| z * z * *w).collect();
        self.fft.process(&mut a);
        self.fft.process(&mut b);
        // fftshift: bin k holds offset (k - n/2) * bin_hz
        for k in 0..n {
            let src = (k + n / 2) % n;
            self.acc[k] += a[src].norm_sqr();
            self.acc_sq[k] += b[src].norm_sqr();
        }
        self.captures += 1;
    }

    /// The averaged spectra read for the chosen beacon; the pattern search
    /// first while nothing is known.
    fn estimate(&mut self, center_hz: f64) -> Option<Reading> {
        if !self.acquired {
            let Some(found) = self.pattern(center_hz) else {
                self.miss();
                return None;
            };
            return self.found(found, center_hz);
        }
        let Some(b) = self.pick(center_hz) else {
            self.beacon = None;
            self.miss();
            return None;
        };
        if self.beacon != Some(b) {
            self.beacon = Some(b);
            self.locked = false;
            self.misses = MISSES_UNLOCK;
        }
        let search = self.search;
        let expect = b.hz() - center_hz;
        let found = match b {
            Beacon::Middle => {
                // The squared spectrum: the line at 2 x offset.
                let acc = std::mem::take(&mut self.acc_sq);
                let r = peak(&acc, self.bin_hz(), 2.0 * expect, 2.0 * search).map(|(f, snr)| (f / 2.0, snr));
                self.acc_sq = acc;
                r
            }
            _ => {
                let acc = std::mem::take(&mut self.acc);
                let r = peak(&acc, self.bin_hz(), expect, search).map(|(f, snr)| {
                    // F1A: a second line 400 Hz up means this is the key-up
                    // tone; the nominal is the carrier above it.
                    let up = line_power(&acc, self.bin_hz(), f + F1A_SHIFT_HZ);
                    let here = line_power(&acc, self.bin_hz(), f);
                    if up > 0.25 * here { (f + F1A_SHIFT_HZ, snr) } else { (f, snr) }
                });
                self.acc = acc;
                r
            }
        };
        match found {
            Some((f, snr)) if snr >= MIN_SNR_DB => {
                let r = Reading { beacon: b, err_hz: f - expect, snr_db: snr, coarse: false };
                self.last = Some(r);
                self.locked = true;
                self.acquired = true;
                self.search = SEARCH_HZ;
                self.misses = 0;
                self.updates += 1;
                Some(r)
            }
            _ => {
                self.miss();
                None
            }
        }
    }

    fn miss(&mut self) {
        self.misses += 1;
        self.search = (self.search * 2.0).min(SEARCH_WIDE_HZ);
        if self.misses >= MISSES_UNLOCK {
            self.locked = false;
        }
        // Lost for a while (the LNB ran away): the pattern search again.
        if self.misses >= 2 * MISSES_UNLOCK {
            self.acquired = false;
            self.beacon = None;
        }
    }

    /// A full-rate spectrometer row (linear power, DC in the middle, the
    /// whole converter rate `span_hz` wide, centred on `center_hz` on the
    /// air) for the wide pattern search while nothing is known; about a
    /// second of rows is summed, then searched. A coarse reading comes
    /// back when the beacons were found.
    pub fn push_wide(&mut self, bins: &[f32], span_hz: f64, center_hz: f64) -> Option<Reading> {
        if self.acquired || bins.len() < 64 {
            return None;
        }
        if self.wide.len() != bins.len() {
            self.wide = vec![0.0; bins.len()];
            self.wide_rows = 0;
        }
        for (w, b) in self.wide.iter_mut().zip(bins) {
            *w += b;
        }
        self.wide_rows += 1;
        if self.wide_rows < 15 {
            return None;
        }
        let n = self.wide.len();
        let cell_hz = span_hz / n as f64;
        // The DC spike must not pass for a beacon line.
        let mut cells = std::mem::take(&mut self.wide);
        self.wide_rows = 0;
        let mut sorted = cells.clone();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let med = sorted[n / 2];
        for c in &mut cells[n / 2 - 3..n / 2 + 4] {
            *c = med;
        }
        let (found, best) = pattern(&cells, cell_hz, center_hz, 2);
        self.last_wide = Some((cells, cell_hz, best));
        self.wide = vec![0.0; n];
        self.found(found?, center_hz)
    }

    /// The beacons' pattern across the averaged stream spectrum, in
    /// ~100 Hz cells.
    fn pattern(&self, center_hz: f64) -> Option<(f64, f64)> {
        let bin = self.bin_hz();
        let per = (PATTERN_STEP_HZ / bin).round().max(1.0) as usize;
        let cells: Vec<f32> = self.acc.chunks(per).map(|c| c.iter().sum::<f32>() / c.len() as f32).collect();
        pattern(&cells, bin * per as f64, center_hz, 2).0
    }

    /// The pattern search's answer taken: the fine stage next, from where
    /// it points.
    fn found(&mut self, (e, db): (f64, f64), center_hz: f64) -> Option<Reading> {
        self.pattern_db = Some(db);
        let b = self.pick(center_hz + e).unwrap_or(Beacon::Middle);
        self.beacon = Some(b);
        self.acquired = true;
        self.search = SEARCH_HZ;
        self.misses = 0;
        self.updates += 1;
        let r = Reading { beacon: b, err_hz: e, snr_db: db, coarse: true };
        self.last = Some(r);
        Some(r)
    }
}

/// The beacons' pattern across a power spectrum in cells of `cell_hz`
/// (DC in the middle, centred on `center_hz` on the air): the error
/// (measured minus nominal) at which the beacons in view line up best,
/// and how far the pattern stands above the band's own noise, dB summed.
/// Each beacon is judged against the floor around it (the transponder's
/// noise, not the quiet outside it), `need` of them have to stand out,
/// and two at least have to be in view. Also the best alignment, its
/// score and the runner-up's, whether or not it was accepted.
fn pattern(cells: &[f32], cell_hz: f64, center_hz: f64, need: usize) -> (Option<(f64, f64)>, Option<(f64, f64, f64)>) {
    let m = cells.len();
    // The local floor: the median over PATTERN_FLOOR_HZ around each cell
    // (small windows: around the cell itself; the stream's many cells: per
    // block).
    let half_w = ((PATTERN_FLOOR_HZ / 2.0 / cell_hz).round() as usize).clamp(4, m / 2);
    let median = |c: &[f32]| {
        let mut v = c.to_vec();
        v.sort_by(|a, b| a.total_cmp(b));
        v[v.len() / 2].max(1e-30)
    };
    let floors: Vec<f32> = if half_w <= 32 {
        (0..m).map(|k| median(&cells[k.saturating_sub(half_w)..(k + half_w + 1).min(m)])).collect()
    } else {
        cells.chunks(2 * half_w).flat_map(|c| std::iter::repeat_n(median(c), c.len())).collect()
    };
    let at = |hz: f64| -> Option<usize> {
        let k = (hz / cell_hz).round() as i64 + m as i64 / 2;
        (k >= 0 && (k as usize) < m).then_some(k as usize)
    };
    let db = |p: f32, k: usize| 10.0 * (p as f64 / floors[k] as f64).log10();
    // A line: the best cell within +/-1, which must also stand 4 dB above
    // its neighbours two and three cells out (a plateau's edge does not).
    let line = |hz: f64| {
        at(hz).map(|k| {
            let (lo, hi) = (k.saturating_sub(1), (k + 2).min(m));
            let (peak, _) = cells[lo..hi].iter().enumerate().fold((0.0f32, 0), |(p, i), (j, &c)| if c > p { (c, lo + j) } else { (p, i) });
            let near: Vec<f32> = [2usize, 3].iter().flat_map(|d| [k.checked_sub(*d), Some(k + d)]).flatten().filter(|j| *j < m).map(|j| cells[j]).collect();
            let near = near.iter().sum::<f32>() / near.len().max(1) as f32;
            if peak < 2.5 * near { 0.0 } else { db(peak, k) }
        })
    };
    // A hump: the mean over +/-w Hz (a line when w is under a cell).
    let block = |hz: f64, w_hz: f64| {
        let w = (w_hz / cell_hz).round().max(0.0) as usize;
        if w == 0 {
            return line(hz);
        }
        at(hz).map(|k| db(cells[k.saturating_sub(w)..(k + w + 1).min(m)].iter().sum::<f32>() / (2 * w + 1) as f32, k))
    };
    let edge = (m as f64 / 2.0 - 2.0) * cell_hz;
    let inside = |hz: f64| hz.abs() < edge;
    type Shape = fn(f64, &dyn Fn(f64) -> Option<f64>, &dyn Fn(f64, f64) -> Option<f64>) -> Option<f64>;
    let shapes: [(f64, Shape); 4] = [
        (NB_LOWER_HZ, |hz, l, _| l(hz)),
        (NB_MIDDLE_HZ, |hz, _, b| b(hz, 600.0)),
        (NB_MULTIMEDIA_HZ, |hz, _, b| b(hz, 3_500.0)),
        (NB_UPPER_HZ, |hz, l, _| l(hz)),
    ];
    let step = cell_hz.min(PATTERN_STEP_HZ).max(cell_hz / 2.0);
    let emax = (m as f64 / 2.0 * cell_hz).min(PATTERN_REACH_HZ);
    let steps = (2.0 * emax / step) as i64;
    let mut scores: Vec<(f64, f64)> = Vec::with_capacity(steps as usize + 1);
    for i in 0..=steps {
        let e = -emax + i as f64 * step;
        let (mut score, mut seen, mut strong) = (0.0, 0, 0);
        for (nominal, shape) in &shapes {
            let off = nominal - center_hz + e;
            if !inside(off) {
                continue;
            }
            if let Some(d) = shape(off, &line, &block) {
                seen += 1;
                if d >= PATTERN_LINE_DB {
                    strong += 1;
                    score += d;
                }
            }
        }
        if seen >= 2 && strong >= need.min(seen) {
            scores.push((e, score));
        }
    }
    let Some(best) = scores.iter().cloned().fold(None, |b: Option<(f64, f64)>, x| if b.is_none_or(|(_, s)| x.1 > s) { Some(x) } else { b }) else {
        return (None, None);
    };
    let runner_up = scores.iter().filter(|(e, _)| (e - best.0).abs() > PATTERN_APART_HZ).map(|(_, s)| *s).fold(0.0, f64::max);
    let ok = best.1 >= PATTERN_MIN_DB && best.1 - runner_up >= PATTERN_UNIQUE_DB;
    (ok.then_some(best), Some((best.0, best.1, runner_up)))
}

/// The strongest line within `search` of `expect` in an fftshifted power
/// spectrum: its offset (parabolic interpolation on log power) and how far
/// it stands above the window's median, dB.
fn peak(acc: &[f32], bin_hz: f64, expect: f64, search: f64) -> Option<(f64, f64)> {
    let n = acc.len();
    let k = |hz: f64| ((hz / bin_hz).round() as i64 + n as i64 / 2).clamp(1, n as i64 - 2) as usize;
    let (lo, hi) = (k(expect - search), k(expect + search));
    if hi <= lo + 2 {
        return None;
    }
    let (mut best, mut best_p) = (lo, 0.0f32);
    for (i, &p) in acc.iter().enumerate().take(hi + 1).skip(lo) {
        if p > best_p {
            best = i;
            best_p = p;
        }
    }
    if best_p <= 0.0 {
        return None;
    }
    let mut w: Vec<f32> = acc[lo..=hi].to_vec();
    w.sort_by(|a, b| a.total_cmp(b));
    let med = w[w.len() / 2].max(1e-30);
    let snr = 10.0 * (best_p as f64 / med as f64).log10();
    // Three-point parabolic interpolation on log power.
    let (a, b, c) = (acc[best - 1].max(1e-30).ln(), best_p.ln(), acc[best + 1].max(1e-30).ln());
    let d = (a - c) / (2.0 * (a - 2.0 * b + c));
    let d = if d.is_finite() { d.clamp(-0.5, 0.5) } else { 0.0 };
    Some(((best as f64 - n as f64 / 2.0 + d as f64) * bin_hz, snr))
}

/// Power around an offset (the bin and its neighbours).
fn line_power(acc: &[f32], bin_hz: f64, hz: f64) -> f32 {
    let n = acc.len();
    let k = ((hz / bin_hz).round() as i64 + n as i64 / 2).clamp(1, n as i64 - 2) as usize;
    acc[k - 1].max(acc[k]).max(acc[k + 1])
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 384_000.0;

    /// A tone that keys between two frequencies (F1A), in noise.
    fn f1a(off_hz: f64, secs: f64, seed: &mut u64) -> Vec<Complex32> {
        let n = (secs * RATE) as usize;
        let mut ph = 0.0f64;
        (0..n)
            .map(|i| {
                // 60 ms elements: on, off, on, on, off ...
                let down = (i / (RATE * 0.06) as usize) % 5 != 1;
                let f = if down { off_hz } else { off_hz - F1A_SHIFT_HZ };
                ph += 2.0 * std::f64::consts::PI * f / RATE;
                let s = Complex32::new(ph.cos() as f32, ph.sin() as f32);
                s + noise(seed) * 0.5
            })
            .collect()
    }

    fn noise(seed: &mut u64) -> Complex32 {
        let mut r = || {
            *seed ^= *seed << 13;
            *seed ^= *seed >> 7;
            *seed ^= *seed << 17;
            (*seed as f64 / u64::MAX as f64) as f32 - 0.5
        };
        Complex32::new(r(), r())
    }

    /// 400 bit/s BPSK on a carrier at `off_hz`, in noise.
    fn bpsk(off_hz: f64, secs: f64, seed: &mut u64) -> Vec<Complex32> {
        let n = (secs * RATE) as usize;
        let sps = RATE / 400.0;
        let mut ph = 0.0f64;
        let mut bit = 1.0f32;
        (0..n)
            .map(|i| {
                if i % sps as usize == 0 {
                    *seed ^= *seed << 13;
                    *seed ^= *seed >> 7;
                    *seed ^= *seed << 17;
                    bit = if *seed & 1 == 1 { 1.0 } else { -1.0 };
                }
                ph += 2.0 * std::f64::consts::PI * off_hz / RATE;
                Complex32::new(ph.cos() as f32, ph.sin() as f32) * bit + noise(seed) * 0.5
            })
            .collect()
    }

    fn run(lock: &mut Lock, iq: &[Complex32], center: f64) -> Option<Reading> {
        let mut out = None;
        for chunk in iq.chunks(4096) {
            if let Some(r) = lock.push(chunk, center) {
                out = Some(r);
            }
        }
        out
    }

    #[test]
    fn a_cw_beacon_reads_its_error() {
        // The stream centred 100 kHz below the lower beacon; the LNB puts
        // the beacon 1234.5 Hz high.
        let center = NB_LOWER_HZ - 100_000.0;
        let mut seed = 7;
        let iq = f1a(100_000.0 + 1234.5, 2.5, &mut seed);
        let mut lock = Lock::new(RATE);
        lock.mark(Beacon::Lower);
        let r = run(&mut lock, &iq, center).expect("a reading");
        assert_eq!(r.beacon, Beacon::Lower);
        assert!((r.err_hz - 1234.5).abs() < 5.0, "err {}", r.err_hz);
        assert!(r.snr_db > MIN_SNR_DB);
        assert!(lock.locked);
    }

    #[test]
    fn the_key_up_tone_does_not_fool_it() {
        // Mostly key-up: the lower line is the strong one.
        let center = NB_UPPER_HZ + 50_000.0;
        let mut seed = 11;
        let n = (2.5 * RATE) as usize;
        let mut ph = 0.0f64;
        let iq: Vec<Complex32> = (0..n)
            .map(|i| {
                let down = (i / (RATE * 0.06) as usize) % 5 < 2;
                let f = -50_000.0 - 300.0 - if down { 0.0 } else { F1A_SHIFT_HZ };
                ph += 2.0 * std::f64::consts::PI * f / RATE;
                Complex32::new(ph.cos() as f32, ph.sin() as f32) + noise(&mut seed) * 0.5
            })
            .collect();
        let mut lock = Lock::new(RATE);
        lock.mark(Beacon::Upper);
        let r = run(&mut lock, &iq, center).expect("a reading");
        assert_eq!(r.beacon, Beacon::Upper);
        assert!((r.err_hz + 300.0).abs() < 5.0, "err {}", r.err_hz);
    }

    #[test]
    fn the_psk_beacon_by_squaring() {
        let center = NB_MIDDLE_HZ - 40_000.0;
        let mut seed = 5;
        let iq = bpsk(40_000.0 - 876.0, 2.5, &mut seed);
        let mut lock = Lock::new(RATE);
        lock.mark(Beacon::Middle);
        let r = run(&mut lock, &iq, center).expect("a reading");
        assert_eq!(r.beacon, Beacon::Middle);
        assert!((r.err_hz + 876.0).abs() < 5.0, "err {}", r.err_hz);
    }

    #[test]
    fn the_pattern_finds_a_big_offset() {
        // The stream centred between the lower and the middle beacon, the
        // LNB 50 kHz off: both beacons in the stream, nothing known yet.
        let center = NB_LOWER_HZ + 125_000.0;
        let err = 50_000.0;
        let mut seed = 9;
        let cw = f1a(NB_LOWER_HZ - center + err, 2.5, &mut seed);
        let psk = bpsk(NB_MIDDLE_HZ - center + err, 2.5, &mut seed);
        let iq: Vec<Complex32> = cw.iter().zip(&psk).map(|(a, b)| a + b).collect();
        let mut lock = Lock::new(RATE);
        let r = run(&mut lock, &iq, center).expect("a reading");
        assert!((r.err_hz - err).abs() < 200.0, "coarse err {}", r.err_hz);
        assert!(lock.acquired);
        assert!(r.coarse);
        // The fine stage after the correction is applied (the frame moved
        // down by the error: the beacons now read nominal).
        let r2 = run(&mut lock, &iq, center - r.err_hz).expect("a fine reading");
        assert!(!r2.coarse);
        assert!((r2.err_hz + (r.err_hz - err)).abs() < 5.0, "fine err {} (coarse left {})", r2.err_hz, err - r.err_hz);
    }

    #[test]
    fn the_wide_rows_find_a_dro_offset() {
        // 24.576 MS/s rows (6 kHz cells), the LNB 1.3 MHz off, all four
        // beacons in view around the stream centred in the band.
        let span = 24_576_000.0;
        let n = 4096;
        let cell = span / n as f64;
        let center = NB_LOWER_HZ + 200_000.0;
        let err = 1_300_000.0;
        let mut seed = 21;
        let mut lock = Lock::new(RATE);
        let mut out = None;
        for _ in 0..15 {
            let mut bins: Vec<f32> = (0..n).map(|_| 1.0 + 0.3 * (noise(&mut seed).re + 0.5)).collect();
            let put = |bins: &mut Vec<f32>, hz: f64, w: f64, p: f32| {
                let k = ((hz - center + err) / cell).round() as i64 + n as i64 / 2;
                let w = (w / cell).round() as i64;
                for j in (k - w)..=(k + w) {
                    if j >= 0 && (j as usize) < n {
                        bins[j as usize] += p;
                    }
                }
            };
            put(&mut bins, NB_LOWER_HZ, 0.0, 30.0);
            put(&mut bins, NB_MIDDLE_HZ, 0.0, 40.0);
            put(&mut bins, NB_MULTIMEDIA_HZ, 3_500.0, 8.0);
            put(&mut bins, NB_UPPER_HZ, 0.0, 30.0);
            bins[n / 2] += 1000.0; // the DC spike
            if let Some(r) = lock.push_wide(&bins, span, center) {
                out = Some(r);
            }
        }
        let r = out.expect("a coarse reading from the rows");
        assert!(r.coarse);
        assert!((r.err_hz - err).abs() <= cell, "err {} vs {}", r.err_hz, err);
        assert!(lock.acquired);
    }

    #[test]
    fn a_noisy_band_without_the_pattern_is_not_a_match() {
        // A transponder-like 8 dB plateau with a dozen stations and a DC
        // spike, no beacons: the wide search must stay quiet.
        let span = 24_576_000.0;
        let n = 4096;
        let center = NB_LOWER_HZ + 200_000.0;
        let mut seed = 33;
        let mut lock = Lock::new(RATE);
        for _ in 0..15 {
            let mut bins: Vec<f32> = (0..n).map(|_| 1.0 + 0.3 * (noise(&mut seed).re + 0.5)).collect();
            for b in &mut bins[n / 2 - 40..n / 2 + 45] {
                *b *= 6.3;
            }
            for j in [-33i64, -20, -11, -4, 3, 9, 17, 25, 31, 38, 42, -27] {
                bins[(n as i64 / 2 + j) as usize] += 60.0;
            }
            bins[n / 2] += 1000.0;
            assert!(lock.push_wide(&bins, span, center).is_none());
        }
        assert!(!lock.acquired);
    }

    #[test]
    fn noise_alone_gives_nothing() {
        let mut seed = 3;
        let iq: Vec<Complex32> = (0..(2.5 * RATE) as usize).map(|_| noise(&mut seed)).collect();
        let mut lock = Lock::new(RATE);
        assert!(run(&mut lock, &iq, NB_LOWER_HZ - 100_000.0).is_none());
        assert!(!lock.locked);
    }

    #[test]
    fn picks_the_beacon_in_the_stream() {
        let lock = Lock::new(RATE);
        assert_eq!(lock.pick(NB_MIDDLE_HZ - 50_000.0), Some(Beacon::Middle));
        assert_eq!(lock.pick(NB_LOWER_HZ + 100_000.0), Some(Beacon::Lower));
        assert_eq!(lock.pick(NB_UPPER_HZ - 120_000.0), Some(Beacon::Upper));
        assert_eq!(lock.pick(NB_LOWER_HZ - 400_000.0), None);
    }
}
