//! AD936x through the Linux IIO interface, without libiio: attributes are
//! sysfs files, samples are `read(2)`/`write(2)` on the buffer character
//! devices. That is all the local libiio backend does underneath, and it keeps
//! trxd free of a C dependency and testable against a fake sysfs tree.
//!
//! Devices (ADI kernel names):
//! * `ad9361-phy` — LOs, rates, gains, ENSM.
//! * `cf-ad9361-lpc` — RX capture: `in_voltage0/1` = RX1 I/Q, `le:S12/16`.
//! * `cf-ad9361-dds-core-lpc` — TX: `out_voltage0/1` = TX1 I/Q, 12 bits
//!   MSB-aligned in 16.
//!
//! The simple bitstream's x8 FIR decimator / interpolator are switched by bit
//! 0 of the axi_ad9361 ADC / DAC GPIO-out registers, reached through
//! `/dev/mem` because the stock driver has no attribute for them.

use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use num_complex::Complex32;
use tracing::{info, warn};

use super::{Radio, RadioControl, RxStream, TxStream};
use crate::config::{GainMode, RadioConfig};

const AXI_AD9361_BASE: u64 = 0x7902_0000;
const ADC_GPIO_OUT: usize = 0x00BC;
const DAC_GPIO_OUT: usize = 0x40BC;

/// Full scale of the 12-bit RX samples.
const RX_SCALE: f32 = 1.0 / 2048.0;
/// 12-bit TX samples sit in the top of a 16-bit word; keep 1 dB of headroom.
const TX_SCALE: f32 = 2047.0 * 16.0 * 0.89;

fn find_device(root: &Path, name: &str) -> Result<PathBuf, String> {
    let entries = fs::read_dir(root).map_err(|e| format!("{}: {e}", root.display()))?;
    for e in entries.flatten() {
        let p = e.path();
        if let Ok(n) = fs::read_to_string(p.join("name")) {
            if n.trim() == name {
                return Ok(p);
            }
        }
    }
    Err(format!("IIO device '{name}' not found under {}", root.display()))
}

fn write_attr(dev: &Path, attr: &str, value: &str) -> Result<(), String> {
    let p = dev.join(attr);
    fs::write(&p, value).map_err(|e| format!("write {} = {value}: {e}", p.display()))
}

fn read_attr(dev: &Path, attr: &str) -> Result<String, String> {
    let p = dev.join(attr);
    fs::read_to_string(&p).map(|s| s.trim().to_string()).map_err(|e| format!("read {}: {e}", p.display()))
}

/// `/dev/iio:deviceN` for a sysfs `.../iio:deviceN`.
fn char_dev(dev_root: &Path, sysfs: &Path) -> PathBuf {
    dev_root.join(sysfs.file_name().expect("iio device dir has a name"))
}

pub struct IioControl {
    phy: PathBuf,
    ptt_gpio: Option<PathBuf>,
    stream_rate: f64,
    tx_rf: Option<bool>,
}

impl RadioControl for IioControl {
    fn stream_rate(&self) -> f64 {
        self.stream_rate
    }

    fn set_lo(&mut self, hz: f64) -> Result<(), String> {
        let v = format!("{}", hz.round() as u64);
        write_attr(&self.phy, "out_altvoltage0_RX_LO_frequency", &v)?;
        write_attr(&self.phy, "out_altvoltage1_TX_LO_frequency", &v)
    }

    fn set_rx_gain(&mut self, mode: GainMode, db: f64) -> Result<(), String> {
        write_attr(&self.phy, "in_voltage0_gain_control_mode", mode.iio_name())?;
        if mode == GainMode::Manual {
            write_attr(&self.phy, "in_voltage0_hardwaregain", &format!("{db:.0}"))?;
        }
        Ok(())
    }

    fn set_tx_attenuation(&mut self, db: f64) -> Result<(), String> {
        // Attenuation in 0.25 dB steps, written as a negative gain.
        let q = (db.clamp(0.0, 89.75) * 4.0).round() / 4.0;
        write_attr(&self.phy, "out_voltage0_hardwaregain", &format!("{:.2}", -q))
    }

    fn set_tx_rf(&mut self, on: bool) -> Result<(), String> {
        if self.tx_rf == Some(on) {
            return Ok(());
        }
        // LO first on key-down, PTT line first on key-up: the PA never sees an
        // unlocked LO, and the relay never switches with RF on it.
        if on {
            write_attr(&self.phy, "out_altvoltage1_TX_LO_powerdown", "0")?;
            self.set_ptt_line(true)?;
        } else {
            self.set_ptt_line(false)?;
            write_attr(&self.phy, "out_altvoltage1_TX_LO_powerdown", "1")?;
        }
        self.tx_rf = Some(on);
        Ok(())
    }

    fn rx_gain_db(&mut self) -> f64 {
        read_gain(&self.phy).unwrap_or(0.0)
    }

    fn rx_gain_reader(&self) -> Option<Box<dyn FnMut() -> Option<f64> + Send>> {
        let phy = self.phy.clone();
        Some(Box::new(move || read_gain(&phy)))
    }
}

fn read_gain(phy: &Path) -> Option<f64> {
    read_attr(phy, "in_voltage0_hardwaregain")
        .ok()
        .and_then(|s| s.split_whitespace().next().and_then(|v| v.parse().ok()))
}

impl IioControl {
    fn set_ptt_line(&self, on: bool) -> Result<(), String> {
        match &self.ptt_gpio {
            Some(p) => fs::write(p, if on { "1" } else { "0" })
                .map_err(|e| format!("PTT GPIO {}: {e}", p.display())),
            None => Ok(()),
        }
    }
}

/// Enable exactly the listed scan elements of a buffered device, size and
/// start its buffer.
fn start_buffer(dev: &Path, prefix: &str, enable: &[usize], len: usize) -> Result<(), String> {
    // A buffer left running by a previous instance refuses reconfiguration.
    let _ = write_attr(dev, "buffer/enable", "0");
    let scan = dev.join("scan_elements");
    if let Ok(entries) = fs::read_dir(&scan) {
        for e in entries.flatten() {
            let name = e.file_name().to_string_lossy().to_string();
            if let Some(idx) = name
                .strip_prefix(prefix)
                .and_then(|s| s.strip_suffix("_en"))
                .and_then(|s| s.parse::<usize>().ok())
            {
                let on = if enable.contains(&idx) { "1" } else { "0" };
                write_attr(&scan, &name, on)?;
            }
        }
    }
    for i in enable {
        write_attr(&scan, &format!("{prefix}{i}_en"), "1")?;
    }
    write_attr(dev, "buffer/length", &len.to_string())?;
    write_attr(dev, "buffer/enable", "1")
}

/// Set or clear bit 0 of the ADC and DAC GPIO-out registers, switching the
/// simple bitstream's x8 decimator / interpolator in or out.
fn set_fpga_filters(on: bool) -> Result<(), String> {
    use std::os::unix::io::AsRawFd;
    let f = OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/mem")
        .map_err(|e| format!("/dev/mem: {e}"))?;
    let len = 0x1_0000usize;
    // SAFETY: a MAP_SHARED mapping of the axi_ad9361 register window, which
    // the simple bitstream places at AXI_AD9361_BASE; every access below is an
    // aligned 32-bit volatile load/store inside `len`.
    unsafe {
        let p = libc::mmap(
            std::ptr::null_mut(),
            len,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            f.as_raw_fd(),
            AXI_AD9361_BASE as libc::off_t,
        );
        if p == libc::MAP_FAILED {
            return Err(format!("mmap axi_ad9361: {}", std::io::Error::last_os_error()));
        }
        for off in [ADC_GPIO_OUT, DAC_GPIO_OUT] {
            let r = p.cast::<u8>().add(off).cast::<u32>();
            let v = std::ptr::read_volatile(r);
            std::ptr::write_volatile(r, if on { v | 1 } else { v & !1 });
        }
        libc::munmap(p, len);
    }
    Ok(())
}

pub struct IioRx {
    dev: File,
    raw: Vec<i16>,
}

impl RxStream for IioRx {
    fn read(&mut self, out: &mut [Complex32]) -> Result<(), String> {
        let want = out.len() * 2;
        if self.raw.len() != want {
            self.raw.resize(want, 0);
        }
        // SAFETY: viewing an i16 buffer as its bytes; u8 has no alignment
        // requirement and the length is exactly the buffer's size in bytes.
        let bytes = unsafe {
            std::slice::from_raw_parts_mut(self.raw.as_mut_ptr().cast::<u8>(), want * 2)
        };
        self.dev.read_exact(bytes).map_err(|e| format!("RX read: {e}"))?;
        for (z, iq) in out.iter_mut().zip(self.raw.chunks_exact(2)) {
            *z = Complex32::new(
                i16::from_le(iq[0]) as f32 * RX_SCALE,
                i16::from_le(iq[1]) as f32 * RX_SCALE,
            );
        }
        Ok(())
    }
}

pub struct IioTx {
    dev: File,
    raw: Vec<i16>,
}

impl TxStream for IioTx {
    fn write(&mut self, iq: &[Complex32]) -> Result<(), String> {
        self.raw.clear();
        for z in iq {
            let i = (z.re.clamp(-1.0, 1.0) * TX_SCALE) as i16;
            let q = (z.im.clamp(-1.0, 1.0) * TX_SCALE) as i16;
            self.raw.push(i.to_le());
            self.raw.push(q.to_le());
        }
        // SAFETY: as in IioRx::read, an i16 buffer seen as its bytes.
        let bytes = unsafe {
            std::slice::from_raw_parts(self.raw.as_ptr().cast::<u8>(), self.raw.len() * 2)
        };
        self.dev.write_all(bytes).map_err(|e| format!("TX write: {e}"))
    }
}

pub fn open(cfg: &RadioConfig) -> Result<Radio, String> {
    let root = Path::new(&cfg.iio_root);
    let dev_root = Path::new(&cfg.dev_root);
    let phy = find_device(root, "ad9361-phy")?;
    let rx_dev = find_device(root, "cf-ad9361-lpc")?;
    let tx_dev = find_device(root, "cf-ad9361-dds-core-lpc")?;

    let _ = write_attr(&phy, "ensm_mode", "fdd");
    write_attr(&phy, "in_voltage_sampling_frequency", &cfg.adc_rate.to_string())?;
    write_attr(&phy, "in_voltage_rf_bandwidth", &cfg.rf_bandwidth.to_string())?;
    write_attr(&phy, "out_voltage_rf_bandwidth", &cfg.rf_bandwidth.to_string())?;

    if cfg.iio_root == "/sys/bus/iio/devices" {
        match set_fpga_filters(cfg.fpga_decimation) {
            Ok(()) => info!(on = cfg.fpga_decimation, "FPGA x8 decimator/interpolator"),
            Err(e) if cfg.fpga_decimation => {
                return Err(format!("cannot enable the FPGA decimator ({e}); set radio.fpga_decimation = false"));
            }
            Err(e) => warn!("FPGA filter bits: {e}"),
        }
    }

    start_buffer(&rx_dev, "in_voltage", &[0, 1], cfg.buffer_samples)?;
    start_buffer(&tx_dev, "out_voltage", &[0, 1], cfg.buffer_samples)?;
    let rx = File::open(char_dev(dev_root, &rx_dev)).map_err(|e| format!("RX buffer: {e}"))?;
    let tx = OpenOptions::new()
        .write(true)
        .open(char_dev(dev_root, &tx_dev))
        .map_err(|e| format!("TX buffer: {e}"))?;

    let ptt_gpio = (!cfg.ptt_gpio.is_empty()).then(|| PathBuf::from(&cfg.ptt_gpio));
    info!(phy = %phy.display(), rate = cfg.stream_rate(), "AD936x ready");
    Ok(Radio {
        control: Box::new(IioControl { phy, ptt_gpio, stream_rate: cfg.stream_rate(), tx_rf: None }),
        rx: Box::new(IioRx { dev: rx, raw: Vec::new() }),
        tx: Box::new(IioTx { dev: tx, raw: Vec::new() }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Backend;

    /// A fake IIO tree: three device directories with the attribute files the
    /// driver would have, and regular files standing in for the buffer nodes.
    fn fake_tree() -> (tempfile::TempDir, RadioConfig) {
        let t = tempfile::tempdir().unwrap();
        let sys = t.path().join("sys");
        let dev = t.path().join("dev");
        fs::create_dir_all(&dev).unwrap();
        for (i, name, prefix) in [
            (0, "ad9361-phy", ""),
            (1, "cf-ad9361-dds-core-lpc", "out_voltage"),
            (2, "cf-ad9361-lpc", "in_voltage"),
        ] {
            let d = sys.join(format!("iio:device{i}"));
            fs::create_dir_all(d.join("buffer")).unwrap();
            fs::create_dir_all(d.join("scan_elements")).unwrap();
            fs::write(d.join("name"), format!("{name}\n")).unwrap();
            if !prefix.is_empty() {
                for ch in 0..4 {
                    fs::write(d.join("scan_elements").join(format!("{prefix}{ch}_en")), "0").unwrap();
                }
            }
            fs::write(dev.join(format!("iio:device{i}")), []).unwrap();
        }
        // Two RX samples: (1024, -2048) and (0, 2047).
        let mut rx = Vec::new();
        for v in [1024i16, -2048, 0, 2047] {
            rx.extend_from_slice(&v.to_le_bytes());
        }
        fs::write(dev.join("iio:device2"), rx).unwrap();
        let cfg = RadioConfig {
            backend: Backend::Iio,
            iio_root: sys.to_string_lossy().into(),
            dev_root: dev.to_string_lossy().into(),
            ..RadioConfig::default()
        };
        (t, cfg)
    }

    #[test]
    fn configures_the_phy_and_streams() {
        let (t, cfg) = fake_tree();
        let mut radio = open(&cfg).unwrap();
        radio.control.set_lo(144_150_000.0).unwrap();
        radio.control.set_tx_attenuation(10.1).unwrap();
        radio.control.set_tx_rf(true).unwrap();

        let phy = t.path().join("sys/iio:device0");
        let rd = |a: &str| fs::read_to_string(phy.join(a)).unwrap();
        assert_eq!(rd("out_altvoltage0_RX_LO_frequency"), "144150000");
        assert_eq!(rd("out_altvoltage1_TX_LO_frequency"), "144150000");
        assert_eq!(rd("in_voltage_sampling_frequency"), "3072000");
        assert_eq!(rd("out_voltage0_hardwaregain"), "-10.00");
        assert_eq!(rd("out_altvoltage1_TX_LO_powerdown"), "0");

        let rxd = t.path().join("sys/iio:device2");
        assert_eq!(fs::read_to_string(rxd.join("scan_elements/in_voltage0_en")).unwrap(), "1");
        assert_eq!(fs::read_to_string(rxd.join("scan_elements/in_voltage2_en")).unwrap(), "0");
        assert_eq!(fs::read_to_string(rxd.join("buffer/enable")).unwrap(), "1");

        let mut buf = [Complex32::default(); 2];
        radio.rx.read(&mut buf).unwrap();
        assert_eq!(buf[0], Complex32::new(0.5, -1.0));
        assert!((buf[1].im - 2047.0 / 2048.0).abs() < 1e-6);

        radio.tx.write(&[Complex32::new(1.0, -1.0)]).unwrap();
        let tx = fs::read(t.path().join("dev/iio:device1")).unwrap();
        let i = i16::from_le_bytes([tx[0], tx[1]]);
        let q = i16::from_le_bytes([tx[2], tx[3]]);
        assert_eq!(i, (TX_SCALE) as i16);
        assert_eq!(q, -(TX_SCALE as i16));
    }
}
