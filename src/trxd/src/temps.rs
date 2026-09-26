//! Chip temperatures for the web header: the Zynq's XADC (FPGA die) and the
//! AD936x's own sensor, read from IIO sysfs every few seconds on a thread of
//! their own (the AD936x one is an SPI round trip).

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Latest readings, degrees C; `None` where the board has no such sensor.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Temps {
    pub fpga: Option<f64>,
    pub ad936x: Option<f64>,
}

const PERIOD: Duration = Duration::from_secs(5);

/// Start the reader; on a PC (no IIO devices) the readings stay `None`.
pub fn start() -> Arc<Mutex<Temps>> {
    let out = Arc::new(Mutex::new(Temps::default()));
    let xadc = find_iio("xadc");
    let phy = find_iio("ad9361-phy");
    if xadc.is_none() && phy.is_none() {
        return out;
    }
    let shared = out.clone();
    let _ = std::thread::Builder::new().name("temps".into()).spawn(move || loop {
        let t = Temps { fpga: xadc.as_deref().and_then(read_xadc), ad936x: phy.as_deref().and_then(read_ad936x) };
        *shared.lock().unwrap() = t;
        std::thread::sleep(PERIOD);
    });
    out
}

fn find_iio(name: &str) -> Option<PathBuf> {
    std::fs::read_dir("/sys/bus/iio/devices").ok()?.flatten().map(|e| e.path()).find(|p| {
        std::fs::read_to_string(p.join("name")).is_ok_and(|n| n.trim() == name)
    })
}

fn num(dev: &Path, attr: &str) -> Option<f64> {
    std::fs::read_to_string(dev.join(attr)).ok()?.trim().parse().ok()
}

/// XADC: (raw + offset) * scale, in millidegrees.
fn read_xadc(dev: &Path) -> Option<f64> {
    Some(xadc_celsius(num(dev, "in_temp0_raw")?, num(dev, "in_temp0_offset")?, num(dev, "in_temp0_scale")?))
}

fn xadc_celsius(raw: f64, offset: f64, scale: f64) -> f64 {
    (raw + offset) * scale / 1000.0
}

/// AD936x: `in_temp0_input`, millidegrees.
fn read_ad936x(dev: &Path) -> Option<f64> {
    Some(num(dev, "in_temp0_input")? / 1000.0)
}

#[cfg(test)]
mod tests {
    #[test]
    fn xadc_conversion() {
        // Read off a LibreSDR on 2026-09-26.
        let c = super::xadc_celsius(2620.0, -2219.0, 123.040771484);
        assert!((c - 49.34).abs() < 0.01, "{c}");
    }
}
