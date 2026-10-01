//! The radio hardware, behind three small traits so the engine never learns
//! whether it drives an AD936x or the simulator:
//!
//! * [`RadioControl`] — tuning, gains, TX RF on/off. Owned by the engine.
//! * [`RxStream`] / [`TxStream`] — blocking sample I/O, each moved onto a
//!   thread of its own so a slow write can never stall a read.
//!
//! RX and TX share the converter clock, so the engine paces transmit by
//! receive: one TX block out for every RX block in keeps the DAC queue at a
//! constant depth without any timer.

pub mod iio;
pub mod sim;

use num_complex::Complex32;

use crate::config::{Backend, GainMode, RadioConfig};

pub trait RadioControl: Send {
    /// Complex sample rate of both streams.
    fn stream_rate(&self) -> f64;
    /// Tune the RX and TX LOs together.
    fn set_lo(&mut self, hz: f64) -> Result<(), String>;
    /// Tune them apart (cross band: the AD936x has a synthesizer each).
    fn set_los(&mut self, rx_hz: f64, tx_hz: f64) -> Result<(), String> {
        if rx_hz == tx_hz { self.set_lo(rx_hz) } else { Err("this radio has one LO".into()) }
    }
    /// Move only the RX LO (the TX LO stays as it is: transmitting).
    fn set_rx_lo(&mut self, hz: f64) -> Result<(), String> {
        self.set_lo(hz)
    }
    /// A new reference correction (Hz of the 40 MHz reference): the clock
    /// chain and the LOs recomputed from it, at the frequencies already set.
    fn apply_xo(&mut self, _hz: f64) -> Result<(), String> {
        Ok(())
    }
    fn set_rx_gain(&mut self, mode: GainMode, db: f64) -> Result<(), String>;
    fn set_tx_attenuation(&mut self, db: f64) -> Result<(), String>;
    /// Power the TX LO (and the external PTT line) up or down. While down the
    /// DAC keeps being fed but nothing reaches the antenna.
    fn set_tx_rf(&mut self, on: bool) -> Result<(), String>;
    /// Current RX hardware gain in dB, for the S-meter. Slow on the AD936x
    /// (an SPI round trip, ~70 ms): never call it on the sample path; see
    /// [`RadioControl::rx_gain_reader`].
    fn rx_gain_db(&mut self) -> f64;
    /// The RX/TX socket pair in use (1 or 2); `None` when it cannot be switched.
    fn port(&self) -> Option<u8> {
        None
    }
    /// Move to the RX1/TX1 (1) or RX2/TX2 (2) sockets. Slow (a chip
    /// re-initialisation on the AD936x): never while transmitting.
    fn set_port(&mut self, _n: u8) -> Result<(), String> {
        Err("this radio has one port pair".into())
    }
    /// TX analog filter bandwidth (RF, Hz): wider for DVB-T2 than the
    /// configured default. No-op where it does not apply.
    fn set_tx_bandwidth(&mut self, _hz: u32) -> Result<(), String> {
        Ok(())
    }
    /// RX analog filter bandwidth (RF, Hz), likewise (DVB-T2 reception).
    fn set_rx_bandwidth(&mut self, _hz: u32) -> Result<(), String> {
        Ok(())
    }
    /// A way to read the gain from another thread, if the backend has one.
    fn rx_gain_reader(&self) -> Option<Box<dyn FnMut() -> Option<f64> + Send>> {
        None
    }
}

pub trait RxStream: Send {
    /// Fill `out` completely, blocking until the samples exist.
    fn read(&mut self, out: &mut [Complex32]) -> Result<(), String>;
}

pub trait TxStream: Send {
    /// Queue `iq` for transmission, blocking while the DAC queue is full.
    fn write(&mut self, iq: &[Complex32]) -> Result<(), String>;
    /// Bytes straight into the DAC DMA buffer (DATV in the FPGA: the
    /// encoder's input), blocking likewise.
    fn write_raw(&mut self, _bytes: &[u8]) -> Result<(), String> {
        Ok(())
    }
}

pub struct Radio {
    pub control: Box<dyn RadioControl>,
    pub rx: Box<dyn RxStream>,
    pub tx: Box<dyn TxStream>,
}

pub fn open(cfg: &RadioConfig, initial_lo_hz: f64) -> Result<Radio, String> {
    let mut radio = match cfg.backend {
        Backend::Iio => iio::open(cfg)?,
        Backend::Sim => sim::open(cfg),
    };
    radio.control.set_lo(initial_lo_hz)?;
    radio.control.set_rx_gain(cfg.rx_gain_mode, cfg.rx_gain_db)?;
    radio.control.set_tx_attenuation(cfg.tx_attenuation_db)?;
    radio.control.set_tx_rf(false)?;
    Ok(radio)
}
