//! A total-power radiometer on the Maia spectrometer's rows, for radio
//! astronomy: the Sun, the Moon, the strong continuum sources and the
//! galactic hydrogen line, with a dish or a yagi in front of the board (and
//! an LNA, or a transverter: the board's own noise figure is far too high).
//!
//! Every row the FPGA integrates (4096 bins of linear power, DC in the
//! middle) is summed here per bin, weighted by its integration time, into
//! the accumulator of the phase the operator marked: HOT (an absorber at
//! ambient temperature over the feed), COLD (empty sky), ON (the object in
//! the beam) and OFF (the sky beside it). Rows arrive at a pinned manual
//! gain; a row read at another gain is dropped.
//!
//! Kelvin come only from the hot/cold pair (the Y factor), per bin and for
//! the band: the dBm calibration, the transverter offsets and the gain
//! tables play no part. With `g = (P_hot - P_cold) / (T_hot - T_cold)`
//! counts per kelvin and `T_rx = P_cold / g - T_cold` (everything behind the
//! feed: LNA, transverter, cable, the AD936x), any power reads as an antenna
//! temperature `P / g - T_rx`: the sky in OFF, the sky plus the object in
//! ON, their difference the object's. Through a transverter the hot load
//! sits at its input, so its gain and drift are inside `g`, which is why the
//! operator re-marks OFF (or HOT) every few minutes: gain drift, not the
//! radiometer equation, limits what a long integration shows.
//!
//! Interference: a row whose band power stands out of the running median
//! of the rows before it (by the scatter they show, the radiometer's own
//! as the floor: a band with traffic in it scatters more than white noise)
//! is left out; at every integration the bins that
//! stand out of the spectrum around them are left out, and bins that do so
//! again and again are masked for the session. The DC region (the LO leak,
//! the converter's 1/f) and bin 0 are always masked.
//!
//! The engine (trx.rs) owns the radio: it places the LO so the band sits
//! beside DC where it fits, pins the gain and the analog filter, drains
//! every row into [`Radiometer::push_row`] and refuses whatever would move
//! the LO or key the transmitter meanwhile. The band here is in offsets
//! from the LO as the spectrometer sees them (mirrored by the engine through
//! an inverting transverter).

use std::collections::VecDeque;

use serde::Serialize;

pub const BINS: usize = crate::maia::BINS;

/// Rows kept for the running median of the row flag (about 2 s at 15 rows/s).
const ROW_MEDIAN_ROWS: usize = 31;
/// Bins per block of the bin flag's baseline (block medians, interpolated).
const BASELINE_BLOCK: usize = 64;
/// A bin flagged in more than this share of the integrations (after a
/// few of them) is masked for the session.
const STICKY_SHARE: f64 = 0.2;
const STICKY_MIN_SAMPLES: u32 = 10;
/// Rows whose gain reads this far from the pinned one are dropped.
const GAIN_TOL_DB: f64 = 0.5;
/// Boltzmann's constant, J/K.
const K_B: f64 = 1.380_649e-23;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Phase {
    /// Rows are measured (the strip chart, the Allan deviation) but not
    /// accumulated.
    Idle,
    Hot,
    Cold,
    On,
    Off,
}

impl Phase {
    pub fn parse(s: &str) -> Option<Phase> {
        Some(match s {
            "idle" => Phase::Idle,
            "hot" => Phase::Hot,
            "cold" => Phase::Cold,
            "on" => Phase::On,
            "off" => Phase::Off,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            Phase::Idle => "idle",
            Phase::Hot => "hot",
            Phase::Cold => "cold",
            Phase::On => "on",
            Phase::Off => "off",
        }
    }

    fn index(self) -> Option<usize> {
        match self {
            Phase::Idle => None,
            Phase::Hot => Some(0),
            Phase::Cold => Some(1),
            Phase::On => Some(2),
            Phase::Off => Some(3),
        }
    }
}

/// What an observation is set up with; fixed for its life.
#[derive(Debug, Clone, Serialize)]
pub struct Setup {
    /// The spectrometer's input rate (a bin is `adc_rate / BINS` wide).
    pub adc_rate: f64,
    /// The band as the spectrometer sees it: offsets from the LO, Hz.
    pub band: (f64, f64),
    /// Bins this close to DC are masked.
    pub dc_exclude_hz: f64,
    /// One sample's integration, s.
    pub integration_s: f64,
    /// The loads, K: the absorber (ambient) and the cold sky.
    pub t_hot: f64,
    pub t_cold: f64,
    /// The pinned front-end gain, dB.
    pub gain_db: f64,
    /// A row this many sigma (the rows' own scatter) over the running
    /// median is interference (0: off).
    pub rfi_row_sigma: f64,
    /// A bin this many sigma over the baseline around it, likewise (0: off).
    pub rfi_bin_sigma: f64,
    /// Effective aperture, m^2, for the flux density (0: not given).
    pub a_eff_m2: f64,
}

impl Default for Setup {
    fn default() -> Self {
        Setup {
            adc_rate: 3_072_000.0,
            band: (100e3, 1_100e3),
            dc_exclude_hz: 50e3,
            integration_s: 1.0,
            t_hot: 290.0,
            t_cold: 10.0,
            gain_db: 40.0,
            rfi_row_sigma: 8.0,
            rfi_bin_sigma: 6.0,
            a_eff_m2: 0.0,
        }
    }
}

/// Per-bin power summed over rows, weighted by their integration time.
#[derive(Debug, Clone, Serialize)]
pub struct Acc {
    #[serde(skip)]
    sum: Vec<f64>,
    pub rows: u64,
    pub secs: f64,
}

impl Acc {
    fn new() -> Acc {
        Acc { sum: vec![0.0; BINS], rows: 0, secs: 0.0 }
    }

    fn add(&mut self, bins: &[f32], secs: f64) {
        for (s, &b) in self.sum.iter_mut().zip(bins) {
            *s += b as f64 * secs;
        }
        self.rows += 1;
        self.secs += secs;
    }

    fn clear(&mut self) {
        self.sum.fill(0.0);
        self.rows = 0;
        self.secs = 0.0;
    }

    /// Mean power per bin, when anything was accumulated.
    fn mean(&self) -> Option<Vec<f64>> {
        (self.secs > 0.0).then(|| self.sum.iter().map(|s| s / self.secs).collect())
    }
}

/// One integration: the band's power and, once calibrated, the antenna
/// temperature it corresponds to.
#[derive(Debug, Clone, Serialize)]
pub struct Sample {
    /// Unix time at the middle of the integration.
    pub t: f64,
    pub phase: Phase,
    /// Band power in row units (the mean over the band's unmasked bins
    /// times the band's bin count: a flagged bin does not lower it).
    pub p: f64,
    /// `p / g - T_rx`, K, once hot and cold are in.
    pub k: Option<f64>,
    pub rows: u32,
    pub rows_flagged: u32,
    pub bins_flagged: u32,
    pub gain_db: f64,
    pub temp_ad936x: Option<f64>,
    pub temp_fpga: Option<f64>,
}

/// What the hot/cold pair gives: counts per kelvin per bin, and for the
/// band with its receiver temperature.
#[derive(Debug, Clone)]
struct Cal {
    g: Vec<f64>,
    g_band: f64,
    t_rx_band: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct PhaseSecs {
    pub hot: f64,
    pub cold: f64,
    pub on: f64,
    pub off: f64,
}

/// The observation's figures for the page and the log.
#[derive(Debug, Clone, Serialize)]
pub struct Results {
    pub phase: Phase,
    pub samples: usize,
    pub secs: PhaseSecs,
    pub calibrated: bool,
    /// Why there is no kelvin scale although hot and cold are in.
    pub note: Option<&'static str>,
    /// Receiver temperature behind the feed, K.
    pub t_rx: Option<f64>,
    /// System temperature on the OFF sky (or the cold sky), K.
    pub t_sys: Option<f64>,
    /// Antenna temperature in OFF (the sky) and ON, and their difference
    /// (the object), K.
    pub t_off: Option<f64>,
    pub t_on: Option<f64>,
    pub t_a: Option<f64>,
    /// The radiometer equation's rms of `t_a`, and the rms the OFF samples
    /// actually show, K.
    pub sigma_exp: Option<f64>,
    pub sigma_meas: Option<f64>,
    /// Flux density, Jy, with an aperture given.
    pub jy: Option<f64>,
    /// The latest sample: its time, power and temperature.
    pub t_last: Option<f64>,
    pub p_last: Option<f64>,
    pub k_last: Option<f64>,
    /// The band after the masks: Hz and bins.
    pub band_hz: f64,
    pub band_bins: usize,
    pub bins_masked: usize,
    pub rows_flagged: u64,
    pub rows_dropped_gain: u64,
    /// Allan deviation of the latest run of same-phase samples, relative
    /// to the mean: (tau s, adev).
    pub allan: Vec<(f64, f64)>,
}

/// One bin of the accumulated spectra, for the export.
#[derive(Debug, Clone, Serialize)]
pub struct SpectrumBin {
    pub if_hz: f64,
    pub masked: bool,
    pub hot: Option<f64>,
    pub cold: Option<f64>,
    pub on: Option<f64>,
    pub off: Option<f64>,
    /// Counts per kelvin, and ON minus OFF in kelvin.
    pub g: Option<f64>,
    pub t_a: Option<f64>,
}

pub struct Radiometer {
    pub setup: Setup,
    phase: Phase,
    accs: [Acc; 4],
    /// Bins inside the band.
    band_lo: usize,
    band_hi: usize,
    /// Masked for the session: outside the band, DC, bin 0, sticky flags.
    mask: Vec<bool>,
    /// How often each bin was flagged, and over how many integrations.
    flag_count: Vec<u32>,
    flag_samples: u32,
    /// Band powers of the last rows, for the row flag.
    recent_rows: VecDeque<f64>,
    /// The integration under way.
    tick: Acc,
    tick_rows_flagged: u32,
    tick_gain_sum: f64,
    tick_t0: Option<f64>,
    tick_temps: (Option<f64>, Option<f64>),
    /// The sample log (a ring).
    log: VecDeque<Sample>,
    log_cap: usize,
    rows_flagged: u64,
    rows_dropped_gain: u64,
}

impl Radiometer {
    pub fn new(setup: Setup, log_cap: usize) -> Radiometer {
        let bin_hz = setup.adc_rate / BINS as f64;
        let (lo, hi) = (setup.band.0.min(setup.band.1), setup.band.0.max(setup.band.1));
        let half = BINS as f64 / 2.0;
        let band_lo = ((lo / bin_hz + half).ceil().max(1.0) as usize).min(BINS - 1);
        let band_hi = ((hi / bin_hz + half).floor().max(1.0) as usize).min(BINS - 1);
        let dc = (setup.dc_exclude_hz / bin_hz).ceil() as i64;
        let mask: Vec<bool> = (0..BINS)
            .map(|k| k == 0 || k < band_lo || k > band_hi || (k as i64 - BINS as i64 / 2).abs() <= dc)
            .collect();
        Radiometer {
            setup,
            phase: Phase::Idle,
            accs: [Acc::new(), Acc::new(), Acc::new(), Acc::new()],
            band_lo,
            band_hi,
            mask,
            flag_count: vec![0; BINS],
            flag_samples: 0,
            recent_rows: VecDeque::with_capacity(ROW_MEDIAN_ROWS + 1),
            tick: Acc::new(),
            tick_rows_flagged: 0,
            tick_gain_sum: 0.0,
            tick_t0: None,
            tick_temps: (None, None),
            log: VecDeque::with_capacity(log_cap.min(1 << 16)),
            log_cap: log_cap.max(16),
            rows_flagged: 0,
            rows_dropped_gain: 0,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// The operator marks what the feed looks at from now on. The row
    /// median restarts: the power may step.
    pub fn set_phase(&mut self, p: Phase) {
        if p != self.phase {
            self.phase = p;
            self.recent_rows.clear();
            // (an integration across the mark would mix two phases)
            self.tick_reset();
        }
    }

    /// Forget one phase's accumulation, or everything (the masks too).
    pub fn clear(&mut self, phase: Option<Phase>) {
        match phase.and_then(Phase::index) {
            Some(i) => self.accs[i].clear(),
            None => {
                for a in &mut self.accs {
                    a.clear();
                }
                self.log.clear();
                self.flag_count.fill(0);
                self.flag_samples = 0;
                let keep = Self::new(self.setup.clone(), self.log_cap).mask;
                self.mask = keep;
                self.rows_flagged = 0;
                self.rows_dropped_gain = 0;
                self.recent_rows.clear();
                self.tick_reset();
            }
        }
    }

    fn tick_reset(&mut self) {
        self.tick.clear();
        self.tick_rows_flagged = 0;
        self.tick_gain_sum = 0.0;
        self.tick_t0 = None;
    }

    /// Band width after the masks, Hz, and the bins it is made of.
    fn band_used(&self) -> (f64, usize) {
        let n = (self.band_lo..=self.band_hi).filter(|&k| !self.mask[k]).count();
        (n as f64 * self.setup.adc_rate / BINS as f64, n)
    }

    /// Band power of one spectrum: the mean of the unmasked (and unflagged)
    /// bins times the band's bin count.
    fn band_power(&self, bins: &[f64], flagged: Option<&[bool]>) -> f64 {
        let mut sum = 0.0;
        let mut n = 0usize;
        for k in self.band_lo..=self.band_hi {
            if self.mask[k] || flagged.is_some_and(|f| f[k]) {
                continue;
            }
            sum += bins[k];
            n += 1;
        }
        if n == 0 {
            return 0.0;
        }
        sum / n as f64 * (self.band_hi - self.band_lo + 1) as f64
    }

    fn band_power_f32(&self, bins: &[f32]) -> f64 {
        let mut sum = 0.0;
        let mut n = 0usize;
        for k in self.band_lo..=self.band_hi {
            if !self.mask[k] {
                sum += bins[k] as f64;
                n += 1;
            }
        }
        if n == 0 {
            return 0.0;
        }
        sum / n as f64 * (self.band_hi - self.band_lo + 1) as f64
    }

    /// One spectrometer row: `bins` linear power (DC in the middle),
    /// integrated over `secs`; the gain it was read at; the chip
    /// temperatures; the time it ended. A finished integration comes back
    /// as a sample (also kept in the log).
    pub fn push_row(&mut self, bins: &[f32], secs: f64, gain_db: f64, temps: (Option<f64>, Option<f64>), now: f64) -> Option<Sample> {
        if bins.len() != BINS || !(secs > 0.0) {
            return None;
        }
        if (gain_db - self.setup.gain_db).abs() > GAIN_TOL_DB {
            self.rows_dropped_gain += 1;
            return None;
        }
        // The row flag: the band power against the median of the rows
        // before it, in units of the radiometer sigma of one row.
        let p = self.band_power_f32(bins);
        let (bw, _) = self.band_used();
        let flagged = if self.setup.rfi_row_sigma > 0.0 && self.recent_rows.len() >= ROW_MEDIAN_ROWS / 2 {
            let med = median(self.recent_rows.iter().copied());
            // The scatter the rows show (a robust estimate: 1.4826 x the
            // median absolute deviation), the radiometer's own at least.
            let mad = median(self.recent_rows.iter().map(|&x| (x - med).abs()));
            let sigma = (1.4826 * mad).max(med / (bw * secs).sqrt());
            p > med + self.setup.rfi_row_sigma * sigma
        } else {
            false
        };
        if !flagged {
            self.recent_rows.push_back(p);
            while self.recent_rows.len() > ROW_MEDIAN_ROWS {
                self.recent_rows.pop_front();
            }
        }
        if self.tick_t0.is_none() {
            self.tick_t0 = Some(now - secs);
            self.tick_temps = temps;
        }
        if flagged {
            self.rows_flagged += 1;
            self.tick_rows_flagged += 1;
        } else {
            self.tick.add(bins, secs);
            self.tick_gain_sum += gain_db;
        }
        let elapsed = now - self.tick_t0.unwrap_or(now);
        if elapsed + 1e-6 < self.setup.integration_s {
            return None;
        }
        let sample = self.finish_tick(now);
        self.tick_reset();
        sample
    }

    /// The integration is over: flag bins, accumulate into the phase, log.
    fn finish_tick(&mut self, now: f64) -> Option<Sample> {
        let mean = self.tick.mean()?;
        let rows = self.tick.rows as u32;
        // The bin flag: each bin against the interpolated block medians of
        // the spectrum, in units of its own radiometer sigma (the number of
        // FFTs averaged into it).
        let mut flagged = vec![false; BINS];
        let mut bins_flagged = 0u32;
        if self.setup.rfi_bin_sigma > 0.0 {
            let ffts = self.tick.secs * self.setup.adc_rate / BINS as f64;
            let base = baseline(&mean, &self.mask);
            for k in self.band_lo..=self.band_hi {
                if self.mask[k] {
                    continue;
                }
                let sigma = base[k] / ffts.max(1.0).sqrt();
                if mean[k] > base[k] + self.setup.rfi_bin_sigma * sigma {
                    flagged[k] = true;
                    bins_flagged += 1;
                    self.flag_count[k] += 1;
                }
            }
            self.flag_samples += 1;
            if self.flag_samples >= STICKY_MIN_SAMPLES {
                for k in self.band_lo..=self.band_hi {
                    if !self.mask[k] && self.flag_count[k] as f64 > STICKY_SHARE * self.flag_samples as f64 {
                        self.mask[k] = true;
                    }
                }
            }
        }
        // The phase takes the rows (flagged bins stay in the sums: the
        // masks apply when the sums are read, the same bins in every phase).
        if let Some(i) = self.phase.index() {
            let a = &mut self.accs[i];
            for (s, m) in a.sum.iter_mut().zip(&mean) {
                *s += m * self.tick.secs;
            }
            a.rows += self.tick.rows;
            a.secs += self.tick.secs;
        }
        let p = self.band_power(&mean, Some(&flagged));
        let k = self.cal().map(|c| p / c.g_band - c.t_rx_band);
        let t0 = self.tick_t0.unwrap_or(now);
        let s = Sample {
            t: (t0 + now) / 2.0,
            phase: self.phase,
            p,
            k,
            rows: rows + self.tick_rows_flagged,
            rows_flagged: self.tick_rows_flagged,
            bins_flagged,
            gain_db: if rows > 0 { self.tick_gain_sum / rows as f64 } else { self.setup.gain_db },
            temp_ad936x: self.tick_temps.0,
            temp_fpga: self.tick_temps.1,
        };
        if self.log.len() >= self.log_cap {
            self.log.pop_front();
        }
        self.log.push_back(s.clone());
        Some(s)
    }

    /// The hot/cold calibration from what the two phases hold.
    fn cal(&self) -> Option<Cal> {
        let hot = self.accs[0].mean()?;
        let cold = self.accs[1].mean()?;
        let dt = self.setup.t_hot - self.setup.t_cold;
        if dt <= 0.0 {
            return None;
        }
        let g: Vec<f64> = hot.iter().zip(&cold).map(|(h, c)| (h - c) / dt).collect();
        let ph = self.band_power(&hot, None);
        let pc = self.band_power(&cold, None);
        let g_band = (ph - pc) / dt;
        if !(g_band > 0.0) {
            return None;
        }
        Some(Cal { g, g_band, t_rx_band: pc / g_band - self.setup.t_cold })
    }

    pub fn results(&self) -> Results {
        let (band_hz, band_bins) = self.band_used();
        let cal = self.cal();
        let temp = |i: usize| -> Option<f64> {
            let c = cal.as_ref()?;
            let m = self.accs[i].mean()?;
            Some(self.band_power(&m, None) / c.g_band - c.t_rx_band)
        };
        let t_rx = cal.as_ref().map(|c| c.t_rx_band);
        let t_on = temp(2);
        let t_off = temp(3);
        let t_sys = t_rx.map(|r| r + t_off.unwrap_or(self.setup.t_cold));
        let t_a = t_on.zip(t_off).map(|(a, b)| a - b);
        let secs = PhaseSecs { hot: self.accs[0].secs, cold: self.accs[1].secs, on: self.accs[2].secs, off: self.accs[3].secs };
        let sigma_exp = t_sys.filter(|_| secs.on > 0.0 && secs.off > 0.0 && band_hz > 0.0).map(|t| {
            t * (1.0 / (band_hz * secs.on) + 1.0 / (band_hz * secs.off)).sqrt()
        });
        let off_k: Vec<f64> = self.log.iter().filter(|s| s.phase == Phase::Off).filter_map(|s| s.k).collect();
        let sigma_meas = (off_k.len() >= 4).then(|| {
            let m = off_k.iter().sum::<f64>() / off_k.len() as f64;
            (off_k.iter().map(|k| (k - m).powi(2)).sum::<f64>() / (off_k.len() - 1) as f64).sqrt()
        });
        let jy = t_a.filter(|_| self.setup.a_eff_m2 > 0.0).map(|t| 2.0 * K_B * t / self.setup.a_eff_m2 / 1e-26);
        let last = self.log.back();
        let note = (cal.is_none() && secs.hot > 0.0 && secs.cold > 0.0)
            .then_some("hot reads no more than cold: no kelvin scale (the absorber on the feed? T hot above T cold?)");
        Results {
            phase: self.phase,
            samples: self.log.len(),
            secs,
            calibrated: cal.is_some(),
            note,
            t_rx,
            t_sys,
            t_off,
            t_on,
            t_a,
            sigma_exp,
            sigma_meas,
            jy,
            t_last: last.map(|s| s.t),
            p_last: last.map(|s| s.p),
            k_last: last.and_then(|s| s.k),
            band_hz,
            band_bins,
            bins_masked: (self.band_lo..=self.band_hi).filter(|&k| self.mask[k]).count(),
            rows_flagged: self.rows_flagged,
            rows_dropped_gain: self.rows_dropped_gain,
            allan: self.allan(),
        }
    }

    /// Allan deviation of the latest run of samples in one phase, relative
    /// to their mean, at 1, 2, 4, ... integrations.
    fn allan(&self) -> Vec<(f64, f64)> {
        let Some(last) = self.log.back() else { return Vec::new() };
        let run: Vec<f64> = self.log.iter().rev().take_while(|s| s.phase == last.phase).map(|s| s.p).collect();
        allan_deviation(&run, self.setup.integration_s)
    }

    /// The accumulated spectra per bin, for the export.
    pub fn spectra(&self) -> Vec<SpectrumBin> {
        let means: Vec<Option<Vec<f64>>> = self.accs.iter().map(Acc::mean).collect();
        let cal = self.cal();
        let bin_hz = self.setup.adc_rate / BINS as f64;
        (self.band_lo..=self.band_hi)
            .map(|k| {
                let at = |i: usize| means[i].as_ref().map(|m| m[k]);
                let g = cal.as_ref().map(|c| c.g[k]);
                let t_a = cal.as_ref().zip(at(2)).zip(at(3)).map(|((c, on), off)| (on - off) / c.g[k]);
                SpectrumBin { if_hz: (k as f64 - BINS as f64 / 2.0) * bin_hz, masked: self.mask[k], hot: at(0), cold: at(1), on: at(2), off: at(3), g, t_a }
            })
            .collect()
    }

    pub fn samples(&self) -> impl DoubleEndedIterator<Item = &Sample> {
        self.log.iter()
    }

    /// The sample log as CSV, the setup and the figures in comment lines.
    pub fn csv(&self) -> String {
        let r = self.results();
        let mut s = String::new();
        s.push_str(&format!(
            "# trxd radiometer; adc_rate={} band_if_hz={},{} integration_s={} t_hot={} t_cold={} gain_db={} a_eff_m2={}\n",
            self.setup.adc_rate, self.setup.band.0, self.setup.band.1, self.setup.integration_s, self.setup.t_hot, self.setup.t_cold, self.setup.gain_db, self.setup.a_eff_m2
        ));
        s.push_str(&format!(
            "# band_hz={:.0} band_bins={} bins_masked={} rows_flagged={} rows_dropped_gain={}\n",
            r.band_hz, r.band_bins, r.bins_masked, r.rows_flagged, r.rows_dropped_gain
        ));
        s.push_str(&format!(
            "# secs hot={:.1} cold={:.1} on={:.1} off={:.1}; t_rx={} t_sys={} t_off={} t_on={} t_a={} sigma_exp={} sigma_meas={} jy={}\n",
            r.secs.hot, r.secs.cold, r.secs.on, r.secs.off, opt(r.t_rx), opt(r.t_sys), opt(r.t_off), opt(r.t_on), opt(r.t_a), opt(r.sigma_exp), opt(r.sigma_meas), opt(r.jy)
        ));
        s.push_str("t_unix,phase,p,k,rows,rows_flagged,bins_flagged,gain_db,temp_ad936x,temp_fpga\n");
        for x in &self.log {
            s.push_str(&format!(
                "{:.3},{},{:.6e},{},{},{},{},{:.1},{},{}\n",
                x.t, x.phase.name(), x.p, opt(x.k), x.rows, x.rows_flagged, x.bins_flagged, x.gain_db, opt(x.temp_ad936x), opt(x.temp_fpga)
            ));
        }
        s
    }

    /// The accumulated spectra as CSV.
    pub fn spectra_csv(&self) -> String {
        let mut s = String::from("if_hz,masked,hot,cold,on,off,g_per_k,t_a_k\n");
        for b in self.spectra() {
            s.push_str(&format!(
                "{:.1},{},{},{},{},{},{},{}\n",
                b.if_hz, b.masked as u8, opt_e(b.hot), opt_e(b.cold), opt_e(b.on), opt_e(b.off), opt_e(b.g), opt(b.t_a)
            ));
        }
        s
    }
}

fn opt(v: Option<f64>) -> String {
    v.filter(|x| x.is_finite()).map_or(String::new(), |x| format!("{x:.3}"))
}

fn opt_e(v: Option<f64>) -> String {
    v.filter(|x| x.is_finite()).map_or(String::new(), |x| format!("{x:.6e}"))
}

fn median(it: impl Iterator<Item = f64>) -> f64 {
    let mut v: Vec<f64> = it.collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2.0 }
}

/// A smooth baseline under a spectrum: the median of each block of
/// unmasked bins, interpolated between block centres.
fn baseline(bins: &[f64], mask: &[bool]) -> Vec<f64> {
    let n = bins.len();
    let nb = n.div_ceil(BASELINE_BLOCK);
    let mut centres = Vec::with_capacity(nb);
    let mut meds = Vec::with_capacity(nb);
    for b in 0..nb {
        let (a, z) = (b * BASELINE_BLOCK, ((b + 1) * BASELINE_BLOCK).min(n));
        let vals = (a..z).filter(|&k| !mask[k]).map(|k| bins[k]);
        let m = median(vals);
        if m > 0.0 {
            centres.push((a + z) as f64 / 2.0);
            meds.push(m);
        }
    }
    let mut out = vec![0.0; n];
    if meds.is_empty() {
        return out;
    }
    for (k, o) in out.iter_mut().enumerate() {
        let x = k as f64;
        let j = centres.partition_point(|&c| c <= x);
        *o = if j == 0 {
            meds[0]
        } else if j >= meds.len() {
            meds[meds.len() - 1]
        } else {
            let (x0, x1) = (centres[j - 1], centres[j]);
            let f = (x - x0) / (x1 - x0);
            meds[j - 1] * (1.0 - f) + meds[j] * f
        };
    }
    out
}

/// Non-overlapping Allan deviation of `y` (oldest last), relative to the
/// mean, at 1, 2, 4, ... samples while at least 4 averages remain.
pub fn allan_deviation(y: &[f64], step_s: f64) -> Vec<(f64, f64)> {
    let n = y.len();
    if n < 8 {
        return Vec::new();
    }
    let mean = y.iter().sum::<f64>() / n as f64;
    if !(mean > 0.0) {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut m = 1usize;
    while n / m >= 4 {
        let avgs: Vec<f64> = y.chunks_exact(m).map(|c| c.iter().sum::<f64>() / m as f64 / mean).collect();
        let d: f64 = avgs.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>() / (avgs.len() - 1) as f64;
        out.push((m as f64 * step_s, (d / 2.0).sqrt()));
        m *= 2;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A row of radiometer noise: per bin a mean power (a temperature in
    /// counts) with the relative scatter of `ffts` averaged FFTs.
    struct Rows {
        seed: u64,
        ffts: f64,
    }

    impl Rows {
        fn gauss(&mut self) -> f64 {
            let mut s = 0.0;
            for _ in 0..12 {
                self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                s += (self.seed >> 33) as f64 / (1u64 << 31) as f64;
            }
            s - 6.0
        }

        fn row(&mut self, counts: impl Fn(usize) -> f64) -> Vec<f32> {
            let rel = 1.0 / self.ffts.sqrt();
            (0..BINS).map(|k| (counts(k) * (1.0 + rel * self.gauss())) as f32).collect()
        }
    }

    const SECS: f64 = 1.0 / 15.0;
    const FFTS: f64 = 50.0;

    fn setup() -> Setup {
        Setup { integration_s: 1.0, t_hot: 290.0, t_cold: 10.0, gain_db: 40.0, ..Setup::default() }
    }

    /// `n` seconds of rows at a temperature (counts per kelvin 1e-6).
    fn feed(r: &mut Radiometer, rows: &mut Rows, secs: f64, t_k: impl Fn(usize) -> f64, t0: f64) -> f64 {
        let mut t = t0;
        let n = (secs / SECS).round() as usize;
        for _ in 0..n {
            t += SECS;
            let row = rows.row(|k| 1e-6 * (100.0 + t_k(k)));
            r.push_row(&row, SECS, 40.0, (Some(50.0), Some(45.0)), t);
        }
        t
    }

    #[test]
    fn hot_cold_gives_kelvin_and_on_off_the_object() {
        let mut r = Radiometer::new(setup(), 1000);
        let mut rows = Rows { seed: 7, ffts: FFTS };
        r.set_phase(Phase::Hot);
        let t = feed(&mut r, &mut rows, 20.0, |_| 290.0, 1000.0);
        r.set_phase(Phase::Cold);
        let t = feed(&mut r, &mut rows, 20.0, |_| 10.0, t);
        let res = r.results();
        assert!(res.calibrated);
        let t_rx = res.t_rx.unwrap();
        assert!((t_rx - 100.0).abs() < 2.0, "t_rx {t_rx}");
        r.set_phase(Phase::Off);
        let t = feed(&mut r, &mut rows, 20.0, |_| 12.0, t);
        r.set_phase(Phase::On);
        let _ = feed(&mut r, &mut rows, 20.0, |_| 12.0 + 30.0, t);
        let res = r.results();
        let (t_off, t_on, t_a) = (res.t_off.unwrap(), res.t_on.unwrap(), res.t_a.unwrap());
        assert!((t_off - 12.0).abs() < 1.0, "t_off {t_off}");
        assert!((t_on - 42.0).abs() < 1.0, "t_on {t_on}");
        assert!((t_a - 30.0).abs() < 1.0, "t_a {t_a}");
        assert!((res.t_sys.unwrap() - 112.0).abs() < 2.0);
        let sx = res.sigma_exp.unwrap();
        // 112 K over 1 MHz x 20 s, ON and OFF: about 0.035 K
        assert!(sx > 0.02 && sx < 0.06, "sigma_exp {sx}");
        assert_eq!(res.samples, 80);
        assert!(res.k_last.unwrap() > 40.0);
        let csv = r.csv();
        assert!(csv.lines().count() > 80, "{}", csv.lines().count());
        assert!(csv.contains(",on,"));
        let sp = r.spectra();
        assert!(sp.iter().filter(|b| !b.masked).all(|b| (b.t_a.unwrap() - 30.0).abs() < 15.0));
        assert!(r.spectra_csv().lines().count() == sp.len() + 1);
    }

    #[test]
    fn flux_from_aperture() {
        let mut r = Radiometer::new(Setup { a_eff_m2: 0.5, ..setup() }, 1000);
        let mut rows = Rows { seed: 3, ffts: FFTS };
        r.set_phase(Phase::Hot);
        let t = feed(&mut r, &mut rows, 10.0, |_| 290.0, 0.0);
        r.set_phase(Phase::Cold);
        let t = feed(&mut r, &mut rows, 10.0, |_| 10.0, t);
        r.set_phase(Phase::Off);
        let t = feed(&mut r, &mut rows, 10.0, |_| 10.0, t);
        r.set_phase(Phase::On);
        let _ = feed(&mut r, &mut rows, 10.0, |_| 10.0 + 1000.0, t);
        // 1000 K in 0.5 m^2: 2 k T / A = 5.5e-20 W/m^2/Hz = 5.5e6 Jy
        let jy = r.results().jy.unwrap();
        assert!((jy / 5.52e6 - 1.0).abs() < 0.05, "{jy}");
    }

    #[test]
    fn a_burst_row_is_left_out() {
        let mut r = Radiometer::new(setup(), 1000);
        let mut rows = Rows { seed: 11, ffts: FFTS };
        r.set_phase(Phase::Off);
        let mut t = feed(&mut r, &mut rows, 3.0, |_| 10.0, 0.0);
        let before = r.results().rows_flagged;
        assert_eq!(before, 0);
        // one row 30 % up: 0.3 / (1 / sqrt(1 MHz x 67 ms)) = 77 sigma
        t += SECS;
        let burst = rows.row(|_| 1e-6 * 110.0 * 1.3);
        r.push_row(&burst, SECS, 40.0, (None, None), t);
        assert_eq!(r.results().rows_flagged, 1);
        let _ = feed(&mut r, &mut rows, 1.0, |_| 10.0, t);
        let p: Vec<f64> = r.samples().map(|s| s.p).collect();
        let m = p.iter().sum::<f64>() / p.len() as f64;
        assert!(p.iter().all(|x| (x / m - 1.0).abs() < 0.02), "{p:?}");
    }

    #[test]
    fn a_busy_band_is_not_flagged_wholesale() {
        let mut r = Radiometer::new(setup(), 1000);
        let mut rows = Rows { seed: 13, ffts: FFTS };
        r.set_phase(Phase::Off);
        let mut t = 0.0;
        // rows scattering 3 % (traffic), 7 x the white-noise sigma
        for i in 0..300 {
            t += SECS;
            let scale = 1.0 + 0.03 * rows.gauss();
            let row = rows.row(|_| 1e-6 * 110.0 * scale);
            r.push_row(&row, SECS, 40.0, (None, None), t);
            if i == 150 {
                let burst = rows.row(|_| 1e-6 * 110.0 * 1.5);
                t += SECS;
                r.push_row(&burst, SECS, 40.0, (None, None), t);
            }
        }
        let flagged = r.results().rows_flagged;
        assert!((1..=6).contains(&flagged), "{flagged} rows flagged");
    }

    #[test]
    fn a_carrier_bin_is_flagged_then_masked() {
        let mut r = Radiometer::new(setup(), 1000);
        let mut rows = Rows { seed: 5, ffts: FFTS };
        r.set_phase(Phase::Off);
        let k = BINS / 2 + 800;
        let _ = feed(&mut r, &mut rows, 15.0, move |b| if b == k { 10.0 + 50.0 } else { 10.0 }, 0.0);
        let res = r.results();
        assert!(res.bins_masked >= 1, "{}", res.bins_masked);
        let first = r.samples().next().unwrap();
        assert!(first.bins_flagged >= 1);
        assert!(r.spectra().iter().any(|b| b.masked && (b.if_hz - 800.0 * 750.0).abs() < 1.0));
    }

    #[test]
    fn rows_at_another_gain_are_dropped() {
        let mut r = Radiometer::new(setup(), 100);
        let mut rows = Rows { seed: 9, ffts: FFTS };
        r.set_phase(Phase::Off);
        let row = rows.row(|_| 1e-6 * 110.0);
        assert!(r.push_row(&row, SECS, 38.0, (None, None), 1.0).is_none());
        assert_eq!(r.results().rows_dropped_gain, 1);
    }

    #[test]
    fn the_band_sits_in_its_bins_and_off_dc() {
        let s = Setup { adc_rate: 3_072_000.0, band: (100e3, 1_100e3), dc_exclude_hz: 50e3, ..setup() };
        let r = Radiometer::new(s, 10);
        // 750 Hz bins: +100 kHz is bin 2048 + 133.3 -> 2182, +1.1 MHz -> 2048 + 1466
        assert_eq!(r.band_lo, 2182);
        assert_eq!(r.band_hi, 2048 + 1466);
        let (hz, n) = r.band_used();
        assert_eq!(n, r.band_hi - r.band_lo + 1);
        assert!((hz - 1e6).abs() < 2000.0, "{hz}");
        // a band across DC loses the DC bins
        let r = Radiometer::new(Setup { band: (-500e3, 500e3), ..Setup::default() }, 10);
        let (_, n) = r.band_used();
        assert!(n < r.band_hi - r.band_lo + 1);
        assert!(r.mask[BINS / 2] && r.mask[BINS / 2 + 60] && !r.mask[BINS / 2 + 70]);
        assert!(r.mask[0]);
    }

    #[test]
    fn allan_deviation_falls_as_white_noise() {
        let mut rows = Rows { seed: 1, ffts: 1.0 };
        let y: Vec<f64> = (0..1024).map(|_| 1.0 + 0.1 * rows.gauss()).collect();
        let a = allan_deviation(&y, 1.0);
        assert!(a.len() >= 6);
        let (t1, d1) = a[0];
        let (t4, d4) = a[2];
        assert_eq!((t1, t4), (1.0, 4.0));
        assert!((d1 - 0.1).abs() < 0.02, "{d1}");
        assert!((d4 / d1 - 0.5).abs() < 0.15, "{} {}", d1, d4);
    }

    #[test]
    fn phase_words() {
        for p in [Phase::Idle, Phase::Hot, Phase::Cold, Phase::On, Phase::Off] {
            assert_eq!(Phase::parse(p.name()), Some(p));
        }
        assert_eq!(Phase::parse("warm"), None);
    }
}
