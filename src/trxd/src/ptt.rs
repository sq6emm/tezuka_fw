//! The PTT output lines: a sysfs GPIO value file (`radio.ptt_gpio`) and/or
//! the AD9361's GPO pins through the driver's debugfs `gpo_set`
//! (`radio.ptt_gpo`, the ADALM-Pluto).
//!
//! GPO scheme (AMSAT-DL / BATC relay circuit, as the F5OEO firmware drives
//! it): GPO0 high keys the relay through a transistor; GPO1 high inhibits it.
//! The driver brings GPO1 up at probe (device tree: manual mode, mask bit 1)
//! so a relay cannot energize while the board boots; trxd releases it once
//! the radio is open and raises it again when it stops or dies.
//! `gpo_set` takes `<pin> <value>`.
//!
//! Order of a key-down is the backend's: lines up, `ptt_delay_ms`, then the
//! TX LO and samples; key-up the reverse.

use std::path::PathBuf;

#[derive(Debug, Clone, Default)]
pub struct PttLines {
    /// `/sys/class/gpio/gpioN/value`.
    pub gpio: Option<PathBuf>,
    /// The phy's debugfs `gpo_set`.
    pub gpo: Option<PathBuf>,
}

impl PttLines {
    pub fn any(&self) -> bool {
        self.gpio.is_some() || self.gpo.is_some()
    }

    /// The PTT line(s) up or down (GPO0).
    pub fn key(&self, on: bool) -> Result<(), String> {
        if let Some(p) = &self.gpio {
            std::fs::write(p, if on { "1" } else { "0" }).map_err(|e| format!("PTT GPIO {}: {e}", p.display()))?;
        }
        if let Some(p) = &self.gpo {
            std::fs::write(p, if on { "0 1" } else { "0 0" }).map_err(|e| format!("PTT GPO0 {}: {e}", p.display()))?;
        }
        Ok(())
    }

    /// The relay inhibit (GPO1) up or down. Nothing for a sysfs line.
    pub fn inhibit(&self, on: bool) -> Result<(), String> {
        if let Some(p) = &self.gpo {
            std::fs::write(p, if on { "1 1" } else { "1 0" }).map_err(|e| format!("PTT GPO1 {}: {e}", p.display()))?;
        }
        Ok(())
    }

    /// The safe state: keyed off, inhibited. Errors are of no use here (the
    /// watchdog, a panic, a signal).
    pub fn off(&self) {
        let _ = self.key(false);
        let _ = self.inhibit(true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpo_words_and_safe_state() {
        let dir = std::env::temp_dir().join(format!("trxd-ptt-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let gpio = dir.join("value");
        let gpo = dir.join("gpo_set");
        let l = PttLines { gpio: Some(gpio.clone()), gpo: Some(gpo.clone()) };
        l.key(true).unwrap();
        assert_eq!(std::fs::read_to_string(&gpio).unwrap(), "1");
        assert_eq!(std::fs::read_to_string(&gpo).unwrap(), "0 1");
        l.inhibit(false).unwrap();
        assert_eq!(std::fs::read_to_string(&gpo).unwrap(), "1 0");
        l.off();
        assert_eq!(std::fs::read_to_string(&gpio).unwrap(), "0");
        assert_eq!(std::fs::read_to_string(&gpo).unwrap(), "1 1");
        assert!(PttLines::default().key(true).is_ok());
        assert!(!PttLines::default().any());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
