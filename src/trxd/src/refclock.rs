//! Reference-oscillator disciplining.
//!
//! The AD936x runs from a 40 MHz XO/VCTCXO. How it is kept on frequency:
//!
//! * **Hardware lock** — an external 10 MHz on PlutoSky R2 (ADF4001 PLL,
//!   source chosen by S22refclk), or 10 MHz / 1PPS on Libre (`vctcxo_lock`:
//!   frequency counter, PI loop, DAC, VCTCXO — set up by S22gpsdo). trxd
//!   leaves both alone and only reports.
//! * **Measure and correct** — the simple bitstream's `refmeter` counts the
//!   40 MHz reference against a GPS 1PPS, or against chrony-disciplined
//!   system time when there is no PPS wire. The measured frequency goes into
//!   the AD936x driver's `xo_correction`, which recomputes the LO and sample
//!   clock PLLs from it, so RF and sample rate come out right even though
//!   the oscillator itself is not steered.
//!
//! The measurement also runs when a hardware lock exists: then it simply
//! reports ~0 ppb, and a correction is never needed.

use std::collections::VecDeque;
use std::fs::OpenOptions;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use tracing::{info, warn};

use crate::stream::{time_synced, unix_now};

const REFMETER_BASE: u64 = 0x43C1_0000;
const VCTCXO_BASE: u64 = 0x43C0_0000;
const REFMETER_ID: u32 = 0x5246_4D31;
const NOMINAL_HZ: f64 = 40_000_000.0;
/// Anything further off than this is a counting fault or a missing clock.
const SANE_PPM: f64 = 50.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum RefMode {
    /// PPS if the refmeter sees one, else chrony time if synchronised.
    Auto,
    Off,
    /// Only GPS 1PPS (EXT_IO0 on R2, PPS_IN on Libre).
    Pps,
    /// Only chrony-disciplined system time.
    Chrony,
    /// Libre: external 10 MHz through vctcxo_lock (hardware). R2: the ADF4001
    /// handles 10 MHz on its own; this only disables software correction.
    #[serde(rename = "10mhz")]
    TenMhz,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RefConfig {
    pub mode: RefMode,
    /// PPS: seconds averaged per estimate.
    pub pps_window_s: u32,
    /// Chrony: seconds of snapshots fitted per estimate.
    pub chrony_window_s: u32,
    /// Only rewrite xo_correction when the estimate moved this much (Hz at
    /// 40 MHz; 0.4 Hz = 10 ppb = 13 Hz at 1296 MHz).
    pub min_step_hz: f64,
}

impl Default for RefConfig {
    fn default() -> Self {
        RefConfig { mode: RefMode::Auto, pps_window_s: 64, chrony_window_s: 1_800, min_step_hz: 0.2 }
    }
}

/// A mapped 4 KiB register window.
struct Regs {
    ptr: *mut u32,
}

// SAFETY: the mapping is only touched through volatile 32-bit accesses from
// the one thread that owns the Regs.
unsafe impl Send for Regs {}

impl Regs {
    fn map(base: u64) -> Result<Regs, String> {
        let f = OpenOptions::new().read(true).write(true).open("/dev/mem").map_err(|e| format!("/dev/mem: {e}"))?;
        // SAFETY: MAP_SHARED of one page of device registers; the fd may be
        // closed afterwards, the mapping stays valid until munmap.
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                4096,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                f.as_raw_fd(),
                base as libc::off_t,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(format!("mmap {base:#x}: {}", std::io::Error::last_os_error()));
        }
        Ok(Regs { ptr: p.cast() })
    }
    fn rd(&self, off: usize) -> u32 {
        // SAFETY: off < 4096, word aligned, inside the mapping.
        unsafe { std::ptr::read_volatile(self.ptr.add(off / 4)) }
    }
    fn wr(&self, off: usize, v: u32) {
        // SAFETY: as rd.
        unsafe { std::ptr::write_volatile(self.ptr.add(off / 4), v) }
    }
}

impl Drop for Regs {
    fn drop(&mut self) {
        // SAFETY: unmapping exactly what map() mapped.
        unsafe {
            libc::munmap(self.ptr.cast(), 4096);
        }
    }
}

/// Frequency from PPS captures: (pps_seq, counter at the edge) pairs.
/// Differences over the window are exact (no per-second rounding builds up).
#[derive(Default)]
pub struct PpsEstimator {
    hist: VecDeque<(u32, u64)>,
    window: usize,
}

impl PpsEstimator {
    pub fn new(window_s: u32) -> Self {
        PpsEstimator { hist: VecDeque::new(), window: window_s.max(4) as usize }
    }

    /// Add a capture; returns the frequency once the window is full.
    pub fn push(&mut self, seq: u32, count: u64) -> Option<f64> {
        if let Some(&(ls, lc)) = self.hist.back() {
            let ds = seq.wrapping_sub(ls);
            if ds == 0 {
                return self.estimate();
            }
            let per = (count.wrapping_sub(lc)) as f64 / ds as f64;
            if ds > 4 || ((per - NOMINAL_HZ) / NOMINAL_HZ * 1e6).abs() > SANE_PPM {
                // A missed, doubled or glitched pulse: start again.
                self.hist.clear();
            }
        }
        self.hist.push_back((seq, count));
        while self.hist.len() > self.window + 1 {
            self.hist.pop_front();
        }
        self.estimate()
    }

    fn estimate(&self) -> Option<f64> {
        if self.hist.len() <= self.window {
            return None;
        }
        let (s0, c0) = self.hist.front()?;
        let (s1, c1) = self.hist.back()?;
        Some(c1.wrapping_sub(*c0) as f64 / s1.wrapping_sub(*s0) as f64)
    }

    pub fn reset(&mut self) {
        self.hist.clear();
    }
}

/// Frequency from (system time, counter) snapshots by least squares. The
/// system clock is chrony's: this measures the reference against whatever
/// chrony disciplines it to (NTP servers, or GPS).
pub struct TimeEstimator {
    pts: VecDeque<(f64, u64)>,
    window_s: f64,
}

impl TimeEstimator {
    pub fn new(window_s: u32) -> Self {
        TimeEstimator { pts: VecDeque::new(), window_s: window_s.max(60) as f64 }
    }

    pub fn push(&mut self, t: f64, count: u64) -> Option<f64> {
        if let Some(&(lt, lc)) = self.pts.back() {
            let f = count.wrapping_sub(lc) as f64 / (t - lt);
            if t <= lt || ((f - NOMINAL_HZ) / NOMINAL_HZ * 1e6).abs() > SANE_PPM {
                // Clock step or counter reset.
                self.pts.clear();
            }
        }
        self.pts.push_back((t, count));
        while self.pts.front().is_some_and(|(t0, _)| t - t0 > self.window_s) {
            self.pts.pop_front();
        }
        let (t0, c0) = *self.pts.front()?;
        if t - t0 < self.window_s * 0.5 || self.pts.len() < 8 {
            return None;
        }
        // Least-squares slope, centred (two passes): counts reach 1e10, and
        // the textbook n*Sxy - Sx*Sy form cancels away the ppb we are after.
        let n = self.pts.len() as f64;
        let xs = |&(t, _): &(f64, u64)| t - t0;
        let ys = |&(_, c): &(f64, u64)| c.wrapping_sub(c0) as f64;
        let mx = self.pts.iter().map(xs).sum::<f64>() / n;
        let my = self.pts.iter().map(ys).sum::<f64>() / n;
        let (mut sxx, mut sxy) = (0.0, 0.0);
        for p in &self.pts {
            let (dx, dy) = (xs(p) - mx, ys(p) - my);
            sxx += dx * dx;
            sxy += dx * dy;
        }
        (sxx > 0.0).then(|| sxy / sxx)
    }

    pub fn reset(&mut self) {
        self.pts.clear();
    }
}

#[derive(Debug, Clone, Serialize)]
struct RefState {
    source: &'static str,
    pps_present: bool,
    measured_hz: Option<f64>,
    error_ppb: Option<f64>,
    xo_correction: Option<f64>,
    libre_vctcxo_locked: Option<bool>,
    time_synced: bool,
}

fn phy_dir() -> Option<PathBuf> {
    let root = Path::new("/sys/bus/iio/devices");
    std::fs::read_dir(root).ok()?.flatten().map(|e| e.path()).find(|p| {
        std::fs::read_to_string(p.join("name")).map(|n| n.trim() == "ad9361-phy").unwrap_or(false)
    })
}

/// Rewrite xo_correction, then re-issue the LO and rate settings so every
/// PLL is recomputed from it whatever the driver version does on its own.
fn apply_xo(phy: &Path, hz: f64) -> Result<(), String> {
    let w = |a: &str, v: &str| std::fs::write(phy.join(a), v).map_err(|e| format!("{a}: {e}"));
    let r = |a: &str| std::fs::read_to_string(phy.join(a)).map(|s| s.trim().to_string()).map_err(|e| format!("{a}: {e}"));
    let rate = r("in_voltage_sampling_frequency")?;
    let rx = r("out_altvoltage0_RX_LO_frequency")?;
    let tx = r("out_altvoltage1_TX_LO_frequency")?;
    w("xo_correction", &format!("{}", hz.round() as u64))?;
    w("in_voltage_sampling_frequency", &rate)?;
    w("out_altvoltage0_RX_LO_frequency", &rx)?;
    w("out_altvoltage1_TX_LO_frequency", &tx)
}

/// Is `vctcxo_lock` (Libre) present? Its status register has only bits 0..1.
fn libre_vctcxo() -> Option<Regs> {
    let exists = std::fs::read_dir("/sys/firmware/devicetree/base")
        .ok()?
        .flatten()
        .chain(std::fs::read_dir("/sys/firmware/devicetree/base/amba_pl").into_iter().flatten().flatten())
        .chain(std::fs::read_dir("/sys/firmware/devicetree/base/fpga-axi@0").into_iter().flatten().flatten())
        .any(|e| {
            let n = e.file_name().to_string_lossy().to_string();
            n.ends_with("@43c00000") && (n.starts_with("vcxo") || n.starts_with("mwipcore"))
        });
    if !exists {
        return None;
    }
    Regs::map(VCTCXO_BASE).ok()
}

/// How a correction reaches the AD936x.
#[derive(Clone, Copy)]
pub enum Apply {
    /// Written from this thread when `quiet()` says a PLL recalculation (a
    /// few ms of LO settling) is acceptable (the beacon roles, whose engines
    /// never retune).
    Direct(fn() -> bool),
    /// Handed to the engine thread ([`take_pending`]), which owns the LOs and
    /// applies it only while not transmitting (the trx role).
    Engine,
}

static PENDING: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// A correction waiting for the engine (the newest wins), Hz of the reference.
pub fn take_pending() -> Option<f64> {
    PENDING.lock().ok()?.take()
}

/// Spawn the disciplining thread.
pub fn spawn(cfg: RefConfig, apply: Apply) {
    if cfg.mode == RefMode::Off {
        return;
    }
    std::thread::Builder::new()
        .name("refclock".into())
        .spawn(move || {
            if let Err(e) = run(cfg, apply) {
                warn!("reference disciplining disabled: {e}");
            }
        })
        .expect("spawn refclock");
}

fn run(cfg: RefConfig, apply: Apply) -> Result<(), String> {
    let meter = Regs::map(REFMETER_BASE)?;
    if meter.rd(0x00) != REFMETER_ID {
        return Err("no refmeter in this bitstream".into());
    }
    let phy = phy_dir().ok_or("ad9361-phy not found")?;
    let vctcxo = libre_vctcxo();

    // Libre: vctcxo_lock is driven by the board's own gpsdo_boot.sh
    // (S22gpsdo: reference choice, calibrated DAC centre, acquisition). trxd
    // only reports it, and corrects in software only when told to use chrony.
    if vctcxo.is_some() {
        info!("Libre vctcxo_lock present: hardware discipline left to S22gpsdo");
    }
    let hw_locked = |v: &Option<Regs>| v.as_ref().map(|r| r.rd(0x10) & 1 != 0);
    let software = vctcxo.is_none() || matches!(cfg.mode, RefMode::Chrony);
    let software = software && cfg.mode != RefMode::TenMhz;

    let mut pps = PpsEstimator::new(cfg.pps_window_s);
    let mut tim = TimeEstimator::new(cfg.chrony_window_s);
    let mut applied = NOMINAL_HZ;
    let mut last_snap = 0.0f64;
    let mut last_pub = 0.0f64;
    let mut last_seq = meter.rd(0x1C);
    info!(software, mode = ?cfg.mode, "reference disciplining");

    loop {
        std::thread::sleep(Duration::from_millis(250));
        let pps_present = meter.rd(0x24) & 1 != 0;
        let use_pps = pps_present && matches!(cfg.mode, RefMode::Auto | RefMode::Pps);
        let use_time = !use_pps && matches!(cfg.mode, RefMode::Auto | RefMode::Chrony) && time_synced();

        let mut est = None;
        let mut source = "none";
        if use_pps {
            source = "pps";
            let seq = meter.rd(0x1C);
            if seq != last_seq {
                // Read the 64-bit capture consistently: seq must not move.
                let lo = meter.rd(0x14) as u64;
                let hi = meter.rd(0x18) as u64;
                if meter.rd(0x1C) == seq {
                    est = pps.push(seq, (hi << 32) | lo);
                }
                last_seq = seq;
            } else {
                est = pps.estimate();
            }
            tim.reset();
        } else if use_time {
            source = "chrony";
            pps.reset();
            let now = unix_now();
            if now - last_snap >= 10.0 {
                last_snap = now;
                if let Some((t, c)) = snapshot(&meter) {
                    est = tim.push(t, c);
                }
            }
        } else {
            pps.reset();
            tim.reset();
        }

        if let (true, Some(f)) = (software, est) {
            if (f - applied).abs() >= cfg.min_step_hz && matches!(apply, Apply::Engine) {
                if let Ok(mut p) = PENDING.lock() {
                    *p = Some(f);
                }
                info!(source, measured = f, ppb = (f - NOMINAL_HZ) / NOMINAL_HZ * 1e9, "xo_correction (to the engine)");
                applied = f;
            } else if (f - applied).abs() >= cfg.min_step_hz && matches!(apply, Apply::Direct(q) if q()) {
                match apply_xo(&phy, f) {
                    Ok(()) => {
                        info!(source, measured = f, ppb = (f - NOMINAL_HZ) / NOMINAL_HZ * 1e9, "xo_correction");
                        applied = f;
                    }
                    Err(e) => warn!("xo_correction: {e}"),
                }
            }
        }

        let now = unix_now();
        // the state in the log every 5 minutes
        if now - last_pub >= 300.0 {
            last_pub = now;
            let state = RefState {
                source: if vctcxo.is_some() && !software { "libre-vctcxo" } else { source },
                pps_present,
                measured_hz: est,
                error_ppb: est.map(|f| ((f - NOMINAL_HZ) / NOMINAL_HZ * 1e9 * 10.0).round() / 10.0),
                xo_correction: software.then_some(applied),
                libre_vctcxo_locked: hw_locked(&vctcxo),
                time_synced: time_synced(),
            };
            info!(state = %serde_json::to_string(&state).unwrap_or_default(), "reference");
        }
    }
}

/// Snapshot the counter, stamped with the midpoint of the request window.
fn snapshot(meter: &Regs) -> Option<(f64, u64)> {
    let seq = meter.rd(0x10);
    let t0 = unix_now();
    meter.wr(0x04, 1);
    for _ in 0..1000 {
        if meter.rd(0x10) != seq {
            let t1 = unix_now();
            if t1 - t0 > 200e-6 {
                return None; // preempted: the stamp would be too loose
            }
            let lo = meter.rd(0x08) as u64;
            let hi = meter.rd(0x0C) as u64;
            return Some(((t0 + t1) / 2.0, (hi << 32) | lo));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pps_estimate_is_exact_over_the_window() {
        let mut e = PpsEstimator::new(8);
        let f = 40_000_001.25; // +31 ppb
        let mut last = None;
        for s in 0..20u32 {
            last = e.push(s, (s as f64 * f).floor() as u64);
        }
        assert!((last.unwrap() - f).abs() < 0.2, "{last:?}");
    }

    #[test]
    fn pps_glitch_restarts_the_window() {
        let mut e = PpsEstimator::new(4);
        for s in 0..10u32 {
            e.push(s, s as u64 * 40_000_000);
        }
        assert!(e.estimate().is_some());
        // A spurious extra edge half a second later.
        assert!(e.push(10, 9 * 40_000_000 + 20_000_000).is_none());
    }

    /// Fit `f` from 10 s snapshots over a 600 s window whose timestamps
    /// carry uniform noise of +-`noise_s`.
    fn fit_with_time_noise(noise_s: f64) -> f64 {
        let mut e = TimeEstimator::new(600);
        let f = 39_999_998.0; // -50 ppb
        let mut seed = 3u32;
        let mut out = None;
        for i in 0..70 {
            let t = 1_000.0 + i as f64 * 10.0;
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let jitter = ((seed >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 2.0 * noise_s;
            out = e.push(t + jitter, ((t - 1_000.0) * f) as u64);
        }
        (out.unwrap() - f) / f * 1e9
    }

    /// The frequency error of a time-based fit is set by the time noise over
    /// the window: sigma_t / (T * sqrt(n / 12)). This is why chrony-over-NTP
    /// disciplining is a long-window, tens-of-ppb affair, and GPS PPS (tens
    /// of ns) or LAN PTP/NTP is what gets it to single ppb.
    #[test]
    fn time_fit_error_scales_with_clock_noise() {
        let internet = fit_with_time_noise(100e-6).abs(); // +-100 us: good internet NTP
        let lan = fit_with_time_noise(5e-6).abs(); // +-5 us: GPS-disciplined chrony
        assert!(internet < 100.0, "internet NTP: {internet} ppb");
        assert!(lan < 5.0, "PPS-grade time: {lan} ppb");
        assert!(lan < internet);
    }
}
