//! Per-board receive level calibration: dBFS at the measurement point
//! (power.rs) to dBm at the antenna socket, independent of mode, filter,
//! bandwidth and AGC state.
//!
//!     dBm = dBFS - G - E(G, f) + K(f) + c (T - T0)  [+ transverter offset]
//!
//! * dBFS: power in the measured band at the 48 kS/s channel (the 384 kS/s
//!   stream and the Maia spectrometer, used for wide bands, are brought to
//!   the channel's scale by `stream_db` / `maia_db`);
//! * G: the AD936x RX gain as the chip reports it (manual or AGC);
//! * E(G, f): the chip's error against that report (gain table of the band
//!   holding f, interpolated over G; 0 where there is none);
//! * K(f): the board's conversion at the reference gain, interpolated
//!   linearly over the AD936x frequency (the IF through a transverter);
//! * c, T0: optional temperature coefficient (dB/degC) and reference.
//!
//! The table is measured by tools/sqtrx-cal (Siglent tracking generator,
//! HP signal generator), one per RX socket pair (the AD936x's RX1 and RX2
//! inputs differ), and stored on the board as `<state_dir>/calib-rx1.json`
//! and `calib-rx2.json` (jffs2: they survive firmware updates). The table
//! of the socket in use applies. Above the highest measured frequency
//! (3.2 GHz with the Siglent's tracking generator) K is the highest point's,
//! marked extrapolated. Without a table the old per-band
//! S-meter points (settings.json) still apply where a band has them; with a
//! table they are ignored (docs/DBM.md).

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tracing::warn;

/// K with no table: the old uncalibrated assumption (noise in 2.7 kHz near
/// -135 dBm on the LibreSDR), same number as settings::UNCALIBRATED_OFFSET_DB.
pub const K_DEFAULT_DB: f64 = 12.0;

/// What the reported gain misses on the AD936x below a few hundred MHz
/// (its front end loses gain there; the AD9363 is specified from 325 MHz):
/// dB to add to K, measured on an ADALM-Pluto with an HP 8642B into RX1,
/// S-meter against the generator level in USB, 2026-10-08. Between the
/// points by log frequency, held outside them.
const K_CORRECTION: [(f64, f64); 6] = [
    (50.15e6, 11.3),
    (70.2e6, 7.8),
    (144.3e6, 3.9),
    (435e6, 2.2),
    (1296e6, 0.8),
    (2100e6, 0.7),
];

/// ... but that loss is in the top gain steps: below them the correction
/// is smaller by this much (dB, by AGC gain), measured the same way at
/// -30..-90 dBm (2026-10-08). None above 1296 MHz.
const K_EXCESS: [(f64, &[(f64, f64)]); 5] = [
    (50.15e6, &[(30.0, 4.6), (50.0, 5.0), (57.0, 5.0), (63.0, 4.3), (70.0, 2.0), (73.0, 0.0)]),
    (70.2e6, &[(30.0, 4.6), (49.0, 4.5), (55.0, 3.4), (60.0, 3.3), (66.0, 2.5), (73.0, 0.0)]),
    (144.3e6, &[(30.0, 3.0), (40.0, 3.9), (45.0, 3.9), (53.0, 1.4), (58.0, 1.3), (64.0, 0.9), (69.0, 0.6), (73.0, 0.0)]),
    (435e6, &[(30.0, 1.5), (46.0, 1.3), (52.0, 0.5), (57.0, 0.6), (63.0, 0.0), (73.0, 0.0)]),
    (1296e6, &[(30.0, 0.0), (73.0, 0.0)]),
];

/// Piecewise-linear in x over sorted (x, y) points, held outside them.
fn lerp(pts: &[(f64, f64)], x: f64) -> f64 {
    if x <= pts[0].0 {
        return pts[0].1;
    }
    if x >= pts[pts.len() - 1].0 {
        return pts[pts.len() - 1].1;
    }
    let i = pts.iter().position(|p| p.0 >= x).unwrap_or(pts.len() - 1).max(1);
    let ((xa, ya), (xb, yb)) = (pts[i - 1], pts[i]);
    ya + (x - xa) / (xb - xa) * (yb - ya)
}

/// K with no measured table for this board, at RX gain `gain_db`:
/// K_DEFAULT_DB plus the frequency correction above, less the excess
/// below the top gain steps.
pub fn k_default(f: f64, gain_db: f64) -> f64 {
    let lf = f.max(1.0).ln();
    let corr: Vec<(f64, f64)> = K_CORRECTION.iter().map(|&(fr, k)| (fr.ln(), k)).collect();
    let rows: Vec<(f64, f64)> = K_EXCESS.iter().map(|&(fr, row)| (fr.ln(), lerp(row, gain_db))).collect();
    K_DEFAULT_DB + lerp(&corr, lf) - lerp(&rows, lf)
}

/// One measured conversion point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KPoint {
    /// AD936x frequency, Hz.
    pub f: f64,
    /// dBm = dBFS - G + K at the reference gain.
    pub k: f64,
    /// "siglent", "hp", ...
    #[serde(default)]
    pub src: String,
    /// ISO date of the measurement.
    #[serde(default)]
    pub date: String,
    /// AD936x temperature when measured, degC.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t: Option<f64>,
}

/// The gain error over a frequency range: points (reported gain dB, error dB).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GainTable {
    pub f_min: f64,
    pub f_max: f64,
    pub pts: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Calib {
    #[serde(default)]
    pub version: u32,
    /// Free text: board, tool and version, instruments.
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub k: Vec<KPoint>,
    #[serde(default)]
    pub gain: Vec<GainTable>,
    /// dB to add to a stream-measured dBFS to get the channel's scale.
    #[serde(default)]
    pub stream_db: f64,
    /// dB to add to a Maia-measured dBFS to get the channel's scale.
    #[serde(default)]
    pub maia_db: f64,
    /// dB per degC above the measurement temperature (0: none).
    #[serde(default)]
    pub temp_coef: f64,
    /// Per transverter (its name): gain from the antenna to the IF, dB.
    #[serde(default)]
    pub xvtr: std::collections::BTreeMap<String, f64>,
}

/// How far a reading can be trusted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Quality {
    /// Inside the measured frequencies.
    Calibrated,
    /// Outside them (nearest point's K), or an unknown transverter.
    Extrapolated,
    /// No table: the band's old S-meter points.
    Legacy,
    /// Nothing: the nominal K.
    None,
}

/// Bands whose status SET shows (label, low, high, Hz).
pub const BANDS: &[(&str, f64, f64)] = &[
    ("6m", 50e6, 54e6),
    ("4m", 70e6, 70.5e6),
    ("2m", 144e6, 146e6),
    ("70cm", 430e6, 440e6),
    ("23cm", 1240e6, 1300e6),
    ("13cm", 2300e6, 2450e6),
    ("9cm", 3400e6, 3475e6),
    ("6cm", 5650e6, 5850e6),
];

/// Points closer than this to a frequency count it as measured (between two
/// points further apart it is still interpolated, and called calibrated if
/// both are within reach): max(20 MHz, 5 %).
fn reach(f: f64) -> f64 {
    (0.05 * f).max(20e6)
}

impl Calib {
    /// The table file of RX socket pair `port` (1 or 2).
    pub fn path(dir: &Path, port: u8) -> PathBuf {
        dir.join(format!("calib-rx{}.json", port.clamp(1, 2)))
    }

    /// Both sockets' tables (index 0: RX1).
    pub fn load_both(dir: &Path) -> [Option<Calib>; 2] {
        [Self::load(dir, 1), Self::load(dir, 2)]
    }

    /// The board's table for `port`, or `None` (no file, or one that does
    /// not parse or validate).
    pub fn load(dir: &Path, port: u8) -> Option<Calib> {
        let p = Self::path(dir, port);
        let s = std::fs::read_to_string(&p).ok()?;
        match serde_json::from_str::<Calib>(&s) {
            Ok(c) => match c.validate() {
                Ok(()) => Some(c),
                Err(e) => {
                    warn!(file = %p.display(), "calibration table ignored: {e}");
                    None
                }
            },
            Err(e) => {
                warn!(file = %p.display(), "calibration table ignored: {e}");
                None
            }
        }
    }

    pub fn save(&self, dir: &Path, port: u8) -> Result<(), String> {
        let p = Self::path(dir, port);
        let tmp = dir.join(format!("calib-rx{}.json.new", port.clamp(1, 2)));
        std::fs::create_dir_all(dir)
            .and_then(|_| std::fs::write(&tmp, serde_json::to_vec_pretty(self).unwrap_or_default()))
            .and_then(|_| std::fs::rename(&tmp, &p))
            .map_err(|e| format!("{}: {e}", p.display()))
    }

    pub fn remove(dir: &Path, port: u8) -> Result<(), String> {
        match std::fs::remove_file(Self::path(dir, port)) {
            Ok(()) => Ok(()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }

    /// Sane numbers only (it comes over the network).
    pub fn validate(&self) -> Result<(), String> {
        let ok = |x: f64, lo: f64, hi: f64| x.is_finite() && x >= lo && x <= hi;
        if self.k.is_empty() {
            return Err("no K points".into());
        }
        if self.k.len() > 2000 || self.gain.len() > 64 || self.note.len() > 500 || self.xvtr.len() > 32 {
            return Err("table too large".into());
        }
        for p in &self.k {
            if !ok(p.f, 1e6, 7e9) || !ok(p.k, -100.0, 100.0) || p.t.is_some_and(|t| !ok(t, -40.0, 150.0)) {
                return Err(format!("bad K point at {} Hz", p.f));
            }
            if p.src.len() > 32 || p.date.len() > 32 {
                return Err("K point text too long".into());
            }
        }
        for g in &self.gain {
            if !ok(g.f_min, 1e6, 7e9) || !ok(g.f_max, g.f_min, 7e9) || g.pts.is_empty() || g.pts.len() > 200 {
                return Err("bad gain table".into());
            }
            if g.pts.iter().any(|[gn, e]| !ok(*gn, -10.0, 80.0) || !ok(*e, -20.0, 20.0)) {
                return Err("bad gain point".into());
            }
        }
        if !ok(self.stream_db, -40.0, 40.0) || !ok(self.maia_db, -100.0, 100.0) || !ok(self.temp_coef, -1.0, 1.0) {
            return Err("bad stream/maia/temperature constants".into());
        }
        if self.xvtr.iter().any(|(n, v)| n.len() > 24 || !ok(*v, -60.0, 80.0)) {
            return Err("bad transverter offset".into());
        }
        Ok(())
    }

    /// Sorted by frequency, as interpolation wants it.
    pub fn normalise(&mut self) {
        self.k.sort_by(|a, b| a.f.total_cmp(&b.f));
        for g in &mut self.gain {
            g.pts.sort_by(|a, b| a[0].total_cmp(&b[0]));
        }
    }

    /// K at `f` and whether `f` is inside the measured points.
    pub fn k_at(&self, f: f64) -> (f64, bool) {
        let p = &self.k;
        match p.len() {
            0 => (k_default(f, 73.0), false),
            1 => (p[0].k, (p[0].f - f).abs() <= reach(f)),
            _ => {
                if f <= p[0].f {
                    return (p[0].k, p[0].f - f <= reach(f));
                }
                if f >= p[p.len() - 1].f {
                    let l = &p[p.len() - 1];
                    return (l.k, f - l.f <= reach(f));
                }
                let i = p.iter().position(|q| q.f >= f).unwrap_or(p.len() - 1).max(1);
                let (a, b) = (&p[i - 1], &p[i]);
                let x = if b.f > a.f { (f - a.f) / (b.f - a.f) } else { 0.0 };
                let inside = (f - a.f) <= reach(f).max(0.5 * (b.f - a.f)) || (b.f - f) <= reach(f);
                (a.k + x * (b.k - a.k), inside)
            }
        }
    }

    /// The temperature the points around `f` were measured at.
    fn t_at(&self, f: f64) -> Option<f64> {
        self.k.iter().filter(|p| p.t.is_some()).min_by(|a, b| (a.f - f).abs().total_cmp(&(b.f - f).abs())).and_then(|p| p.t)
    }

    /// The chip's gain error at reported gain `g`, frequency `f`: the table
    /// covering `f` (else the nearest one), interpolated over `g`, flat
    /// beyond its ends; 0 with none.
    pub fn gain_err(&self, g: f64, f: f64) -> f64 {
        let t = self.gain.iter().find(|t| f >= t.f_min && f <= t.f_max).or_else(|| {
            self.gain.iter().min_by(|a, b| {
                let d = |t: &GainTable| if f < t.f_min { t.f_min - f } else { f - t.f_max };
                d(a).total_cmp(&d(b))
            })
        });
        let Some(t) = t else { return 0.0 };
        let p = &t.pts;
        if p.is_empty() {
            return 0.0;
        }
        if g <= p[0][0] {
            return p[0][1];
        }
        if g >= p[p.len() - 1][0] {
            return p[p.len() - 1][1];
        }
        let i = p.iter().position(|q| q[0] >= g).unwrap_or(p.len() - 1).max(1);
        let (a, b) = (p[i - 1], p[i]);
        let x = if b[0] > a[0] { (g - a[0]) / (b[0] - a[0]) } else { 0.0 };
        a[1] + x * (b[1] - a[1])
    }

    /// dB to add to a channel-scale dBFS for dBm at the antenna (the whole
    /// right-hand side above but the dBFS), and its quality.
    pub fn offset(&self, g: f64, hw_freq: f64, temp: Option<f64>, xvtr: Option<&str>) -> (f64, Quality) {
        let (k, inside) = self.k_at(hw_freq);
        let mut off = -g - self.gain_err(g, hw_freq) + k;
        if let (Some(t), Some(t0)) = (temp, self.t_at(hw_freq)) {
            off += self.temp_coef * (t - t0);
        }
        let mut q = if inside { Quality::Calibrated } else { Quality::Extrapolated };
        if let Some(name) = xvtr {
            match self.xvtr.get(name) {
                Some(db) => off -= db,
                None => q = Quality::Extrapolated,
            }
        }
        (off, q)
    }

    /// Per band: ("calibrated" | "extrapolated", newest date) for SET.
    pub fn status(&self) -> Vec<serde_json::Value> {
        BANDS
            .iter()
            .map(|&(name, lo, hi)| {
                let pts: Vec<&KPoint> = self.k.iter().filter(|p| p.f >= lo - reach(lo) && p.f <= hi + reach(hi)).collect();
                let mid = 0.5 * (lo + hi);
                let (_, inside) = self.k_at(mid);
                let date = pts.iter().map(|p| p.date.as_str()).max().unwrap_or("");
                let src: Vec<&str> = {
                    let mut s: Vec<&str> = pts.iter().map(|p| p.src.as_str()).filter(|s| !s.is_empty()).collect();
                    s.sort();
                    s.dedup();
                    s
                };
                let gain = self.gain.iter().any(|t| mid >= t.f_min && mid <= t.f_max);
                serde_json::json!({"band": name, "status": if inside && !pts.is_empty() { "calibrated" } else { "extrapolated" },
                    "points": pts.len(), "date": date, "src": src.join(","), "gain_table": gain})
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn default_k_follows_the_measured_low_band_correction() {
        // at the top gain step: the whole frequency correction
        assert!((k_default(50.15e6, 73.0) - (K_DEFAULT_DB + 11.3)).abs() < 1e-9);
        assert!((k_default(30e6, 73.0) - (K_DEFAULT_DB + 11.3)).abs() < 1e-9);
        assert!((k_default(144.3e6, 73.0) - (K_DEFAULT_DB + 3.9)).abs() < 1e-9);
        assert!((k_default(5.7e9, 73.0) - (K_DEFAULT_DB + 0.7)).abs() < 1e-9);
        let k = k_default(100e6, 73.0) - K_DEFAULT_DB;
        assert!(k < 7.8 && k > 3.9, "{k}");
        // lower gain: less (50 MHz at 40 dB: 11.3 - 4.8)
        assert!((k_default(50.15e6, 40.0) - (K_DEFAULT_DB + 11.3 - 4.8)).abs() < 1e-9);
        // none of that above 1296 MHz
        assert!((k_default(2.1e9, 40.0) - (K_DEFAULT_DB + 0.7)).abs() < 1e-9);
    }

    use super::*;

    fn table() -> Calib {
        let mut c = Calib {
            version: 1,
            k: vec![
                KPoint { f: 144e6, k: 10.0, src: "siglent".into(), date: "2026-10-02".into(), t: Some(40.0) },
                KPoint { f: 432e6, k: 12.0, src: "siglent".into(), date: "2026-10-02".into(), t: Some(40.0) },
                KPoint { f: 1296e6, k: 16.0, src: "hp".into(), date: "2026-10-03".into(), t: Some(40.0) },
            ],
            gain: vec![GainTable { f_min: 100e6, f_max: 500e6, pts: vec![[0.0, 1.0], [40.0, 0.0], [70.0, -2.0]] }],
            ..Default::default()
        };
        c.normalise();
        c
    }

    #[test]
    fn k_interpolates_and_says_where_it_extrapolates() {
        let c = table();
        assert_eq!(c.k_at(144e6), (10.0, true));
        let (k, inside) = c.k_at(288e6);
        assert!((k - 11.0).abs() < 1e-9 && inside);
        let (k, inside) = c.k_at(5760e6);
        assert_eq!(k, 16.0);
        assert!(!inside);
        let (k, inside) = c.k_at(50e6);
        assert_eq!(k, 10.0);
        assert!(!inside);
    }

    #[test]
    fn gain_error_interpolates_over_gain_and_uses_the_nearest_table() {
        let c = table();
        assert!((c.gain_err(20.0, 145e6) - 0.5).abs() < 1e-9);
        assert!((c.gain_err(55.0, 145e6) - -1.0).abs() < 1e-9);
        assert_eq!(c.gain_err(75.0, 145e6), -2.0);
        // 1296 MHz has no table: the nearest (100-500 MHz) one.
        assert!((c.gain_err(20.0, 1296e6) - 0.5).abs() < 1e-9);
        assert_eq!(Calib::default().gain_err(20.0, 145e6), 0.0);
    }

    #[test]
    fn the_same_signal_reads_the_same_at_any_gain() {
        // A -80 dBm carrier at 145 MHz: the chip's real gain is the reported
        // one plus the error, so dBFS = -80 + (g + E(g)) - K.
        let c = table();
        let (k, _) = c.k_at(145e6);
        for g in [0.0, 13.0, 40.0, 61.0, 70.0] {
            let dbfs = -80.0 + g + c.gain_err(g, 145e6) - k;
            let (off, q) = c.offset(g, 145e6, None, None);
            assert!((dbfs + off - -80.0).abs() < 1e-9, "{g}");
            assert_eq!(q, Quality::Calibrated);
        }
    }

    #[test]
    fn temperature_and_transverters() {
        let mut c = table();
        c.temp_coef = 0.02;
        let (a, _) = c.offset(40.0, 144e6, Some(40.0), None);
        let (b, _) = c.offset(40.0, 144e6, Some(50.0), None);
        assert!((b - a - 0.2).abs() < 1e-9);
        let (_, q) = c.offset(40.0, 144e6, None, Some("3cm"));
        assert_eq!(q, Quality::Extrapolated);
        c.xvtr.insert("3cm".into(), 20.0);
        let (x, q) = c.offset(40.0, 144e6, None, Some("3cm"));
        assert_eq!(q, Quality::Calibrated);
        assert!((x - (a - 20.0)).abs() < 1e-9);
    }

    #[test]
    fn validation_and_storage() {
        let c = table();
        assert!(c.validate().is_ok());
        let mut bad = c.clone();
        bad.k[0].k = f64::NAN;
        assert!(bad.validate().is_err());
        assert!(Calib::default().validate().is_err());
        let dir = std::env::temp_dir().join(format!("trxd-calib-{}", std::process::id()));
        c.save(&dir, 2).unwrap();
        assert_eq!(Calib::load(&dir, 2), Some(c.clone()));
        assert_eq!(Calib::load_both(&dir), [None, Some(c.clone())]);
        std::fs::write(Calib::path(&dir, 2), "{nonsense").unwrap();
        assert_eq!(Calib::load(&dir, 2), None);
        Calib::remove(&dir, 2).unwrap();
        Calib::remove(&dir, 2).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn band_status() {
        let c = table();
        let s = c.status();
        let get = |b: &str| s.iter().find(|v| v["band"] == b).unwrap().clone();
        assert_eq!(get("2m")["status"], "calibrated");
        assert_eq!(get("2m")["gain_table"], true);
        assert_eq!(get("23cm")["status"], "calibrated");
        assert_eq!(get("23cm")["src"], "hp");
        assert_eq!(get("6cm")["status"], "extrapolated");
    }
}
