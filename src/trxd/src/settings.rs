//! Operator settings the web UI edits and the board keeps across reboots and
//! firmware updates (`<web.state_dir>/settings.json`, on jffs2): the station
//! callsign, transverters and the S-meter calibration.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tracing::warn;

/// A transverter: RF `rf_min..=rf_max` is reached through the AD936x tuned to
/// the IF `rf - lo_hz` (or `lo_hz - rf` with the local oscillator above the
/// band, which also mirrors the spectrum).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transverter {
    pub name: String,
    pub rf_min: f64,
    pub rf_max: f64,
    pub lo_hz: f64,
    #[serde(default)]
    pub inverted: bool,
    /// Whether this transverter may be keyed (its IF drive is low: mind TX att).
    #[serde(default = "yes")]
    pub tx: bool,
}

fn yes() -> bool {
    true
}

impl Transverter {
    pub fn covers(&self, rf: f64) -> bool {
        rf >= self.rf_min && rf <= self.rf_max
    }
    /// RF -> the AD936x frequency.
    pub fn to_if(&self, rf: f64) -> f64 {
        if self.inverted { self.lo_hz - rf } else { rf - self.lo_hz }
    }
}

/// One S-meter calibration point: the receiver's gain-compensated reading
/// (channel dBFS minus the front-end gain) against the true level in dBm.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CalPoint {
    pub reading: f64,
    pub dbm: f64,
}

/// Reading -> dBm with no calibration: the level is taken as 12 dB above the
/// gain-compensated channel reading, which puts the noise in 2.7 kHz near
/// -135 dBm on the LibreSDR (measured 2026-09-26). Calibration does the rest.
pub const UNCALIBRATED_OFFSET_DB: f64 = 12.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Settings {
    /// The operator's callsign; overrides `callsign` in trxd.toml when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub callsign: Option<String>,
    /// Mute the receiver while transmitting (its AGC and S-meter held). Off:
    /// keep receiving during TX (satellites, crossband).
    #[serde(default = "yes")]
    pub mute_at_tx: bool,
    #[serde(default)]
    pub transverters: Vec<Transverter>,
    /// Per band (band label, or transverter name): points sorted by reading.
    #[serde(default)]
    pub smeter: std::collections::BTreeMap<String, Vec<CalPoint>>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { callsign: None, mute_at_tx: true, transverters: Vec::new(), smeter: Default::default() }
    }
}

impl Settings {
    pub fn load(dir: &Path) -> Self {
        let p = dir.join("settings.json");
        match std::fs::read_to_string(&p) {
            Ok(s) => serde_json::from_str(&s).unwrap_or_else(|e| {
                warn!(file = %p.display(), "settings: {e}; starting empty");
                Settings::default()
            }),
            Err(_) => Settings::default(),
        }
    }

    pub fn save(&self, dir: &Path) {
        let p: PathBuf = dir.join("settings.json");
        let tmp = dir.join("settings.json.new");
        let res = std::fs::create_dir_all(dir)
            .and_then(|_| std::fs::write(&tmp, serde_json::to_vec_pretty(self).unwrap_or_default()))
            .and_then(|_| std::fs::rename(&tmp, &p));
        if let Err(e) = res {
            warn!(file = %p.display(), "settings not saved: {e}");
        }
    }

    /// A callsign as it is stored: upper case, 3..=12 of A-Z, 0-9 and `/`,
    /// with at least one digit and one letter. `None` for anything else.
    pub fn clean_call(s: &str) -> Option<String> {
        let c = s.trim().to_ascii_uppercase();
        let ok = (3..=12).contains(&c.len())
            && c.chars().all(|ch| ch.is_ascii_alphanumeric() || ch == '/')
            && c.chars().any(|ch| ch.is_ascii_digit())
            && c.chars().any(|ch| ch.is_ascii_alphabetic());
        ok.then_some(c)
    }

    pub fn transverter(&self, rf: f64) -> Option<&Transverter> {
        self.transverters.iter().find(|t| t.covers(rf))
    }

    /// Add a calibration point (replacing one within 1 dB of the same level).
    pub fn add_cal(&mut self, band: &str, reading: f64, dbm: f64) {
        let v = self.smeter.entry(band.to_string()).or_default();
        v.retain(|p| (p.dbm - dbm).abs() >= 1.0);
        v.push(CalPoint { reading, dbm });
        v.sort_by(|a, b| a.reading.total_cmp(&b.reading));
    }

    /// dBm for a reading on `band`: piecewise linear through the points,
    /// extended beyond them with the nearest segment's slope (1 dB/dB with a
    /// single point, which is then a plain offset).
    pub fn dbm(&self, band: &str, reading: f64) -> f64 {
        let Some(p) = self.smeter.get(band).filter(|v| !v.is_empty()) else {
            return reading + UNCALIBRATED_OFFSET_DB;
        };
        if p.len() == 1 {
            return reading + (p[0].dbm - p[0].reading);
        }
        let seg = match p.iter().position(|q| q.reading >= reading) {
            Some(0) => 0,
            Some(i) => i - 1,
            None => p.len() - 2,
        };
        let (a, b) = (p[seg], p[seg + 1]);
        let dr = b.reading - a.reading;
        if dr.abs() < 1e-6 {
            return a.dbm + (reading - a.reading);
        }
        a.dbm + (reading - a.reading) * (b.dbm - a.dbm) / dr
    }
}

/// S9 level per the IARU R1 recommendation: 50 uV (-73 dBm) below 30 MHz,
/// 5 uV (-93 dBm) above; 6 dB per S unit either way.
pub fn s9_dbm(freq_hz: f64) -> f64 {
    if freq_hz < 30e6 { -73.0 } else { -93.0 }
}

/// "S7", "S9+12" for a level at a frequency.
pub fn s_units(freq_hz: f64, dbm: f64) -> String {
    let over = dbm - s9_dbm(freq_hz);
    if over > 0.5 {
        format!("S9+{}", over.round() as i64)
    } else {
        let s = (9.0 + over / 6.0).clamp(0.0, 9.0);
        format!("S{}", s.floor() as i64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_interpolates_and_extends() {
        let mut s = Settings::default();
        assert_eq!(s.dbm("2m", -100.0), -88.0);
        s.add_cal("2m", -100.0, -93.0);
        assert_eq!(s.dbm("2m", -106.0), -99.0);
        s.add_cal("2m", -130.0, -129.0); // compressed at the bottom
        s.add_cal("2m", -40.0, -33.0);
        assert!((s.dbm("2m", -115.0) - -111.0).abs() < 1e-9);
        assert!((s.dbm("2m", -140.0) - -141.0).abs() < 1e-9);
        assert!((s.dbm("2m", -30.0) - -23.0).abs() < 1e-9);
        // Re-storing a level replaces it.
        s.add_cal("2m", -101.0, -93.2);
        assert_eq!(s.smeter["2m"].len(), 3);
        assert_eq!(s.dbm("70cm", -100.0), -88.0);
    }

    #[test]
    fn s_units_follow_the_iaru_levels() {
        assert_eq!(s_units(144e6, -93.0), "S9");
        assert_eq!(s_units(144e6, -99.0), "S8");
        assert_eq!(s_units(144e6, -80.0), "S9+13");
        assert_eq!(s_units(14e6, -73.0), "S9");
        assert_eq!(s_units(144e6, -150.0), "S0");
    }

    #[test]
    fn transverters_map_and_persist() {
        let t = Transverter { name: "10G".into(), rf_min: 10_368e6, rf_max: 10_370e6, lo_hz: 9_936e6, inverted: false, tx: true };
        assert_eq!(t.to_if(10_368.1e6), 432.1e6);
        let inv = Transverter { inverted: true, lo_hz: 10_800e6, ..t.clone() };
        assert_eq!(inv.to_if(10_368.1e6), 431.9e6);
        let dir = tempfile::tempdir().unwrap();
        let mut s = Settings::default();
        s.transverters.push(t);
        s.add_cal("10G", -90.0, -100.0);
        s.save(dir.path());
        let back = Settings::load(dir.path());
        assert_eq!(back, s);
        assert!(back.transverter(10_369e6).is_some() && back.transverter(432e6).is_none());
    }

    #[test]
    fn callsign_is_checked_and_kept() {
        assert_eq!(Settings::clean_call(" sq6emm "), Some("SQ6EMM".into()));
        assert_eq!(Settings::clean_call("SQ6EMM/P"), Some("SQ6EMM/P".into()));
        for bad in ["", "AB", "SQ6 EMM", "CALLSIGN", "12345", "SQ6EMM-1"] {
            assert_eq!(Settings::clean_call(bad), None, "{bad}");
        }
        let dir = tempfile::tempdir().unwrap();
        let s = Settings { callsign: Some("SQ6EMM".into()), ..Settings::default() };
        s.save(dir.path());
        assert_eq!(Settings::load(dir.path()).callsign.as_deref(), Some("SQ6EMM"));
        // Files from before MUTE AT TX existed mute at TX.
        assert!(serde_json::from_str::<Settings>("{}").unwrap().mute_at_tx);
    }
}
