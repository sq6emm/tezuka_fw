//! `trxd.toml`: one file, three roles. Every field has a default so a config
//! only needs what differs from it — an empty file is a working receive-only
//! transceiver on the simulated radio's frequency.

use std::path::Path;

use serde::Deserialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Role {
    /// Remote transceiver: TCI + rigctld + on-board decoders.
    Trx,
    /// MGM beacon transmitter (digital mode / CW / carrier, minute cycle).
    BeaconTx,
    /// Beacon receiver: decode each minute, publish to MQTT.
    BeaconRx,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub role: Role,
    pub callsign: String,
    /// Maidenhead locator, 4 or 6 characters.
    pub locator: String,
    /// DeepCW model: "auto" = the `model` flash partition, or a path to a
    /// packed blob / plain .onnx file (for running on a PC).
    pub cw_model: String,
    pub radio: RadioConfig,
    pub trx: TrxConfig,
    pub beacon: BeaconConfig,
    pub beacon_rx: BeaconRxConfig,
    pub mqtt: MqttConfig,
    pub reference: crate::refclock::RefConfig,
    pub web: crate::web::WebConfig,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            role: Role::Trx,
            callsign: "N0CALL".into(),
            locator: "JO81".into(),
            cw_model: "auto".into(),
            radio: RadioConfig::default(),
            trx: TrxConfig::default(),
            beacon: BeaconConfig::default(),
            beacon_rx: BeaconRxConfig::default(),
            mqtt: MqttConfig::default(),
            reference: crate::refclock::RefConfig::default(),
            web: crate::web::WebConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Backend {
    /// The AD936x through the kernel's IIO sysfs and buffer devices.
    Iio,
    /// A software radio: noise, a CW test signal, and TX looped back to RX.
    Sim,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GainMode {
    Manual,
    SlowAttack,
    FastAttack,
    Hybrid,
}

impl GainMode {
    pub fn iio_name(self) -> &'static str {
        match self {
            GainMode::Manual => "manual",
            GainMode::SlowAttack => "slow_attack",
            GainMode::FastAttack => "fast_attack",
            GainMode::Hybrid => "hybrid",
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RadioConfig {
    pub backend: Backend,
    /// AD936x converter rate. 3.072 MS/s needs no AD936x FIR and divides to
    /// 384 kS/s through the FPGA decimator and to 48 kS/s after that.
    pub adc_rate: u32,
    /// Use the simple bitstream's x8 FIR decimator / interpolator. Off streams
    /// the full `adc_rate` to the ARM (works with any ADI bitstream, costs CPU).
    pub fpga_decimation: bool,
    pub rf_bandwidth: u32,
    pub rx_gain_mode: GainMode,
    /// Manual RX gain, dB (0..73 on the AD9363).
    pub rx_gain_db: f64,
    /// TX attenuation, dB (0..89.75). Output power goes *down* as this goes up.
    pub tx_attenuation_db: f64,
    /// The LO sits this far below the VFO so the wanted signal never lands on
    /// the converter's DC spike / LO leakage.
    pub lo_offset_hz: f64,
    pub freq_min_hz: f64,
    pub freq_max_hz: f64,
    /// Optional sysfs GPIO value file driven high while transmitting (PA
    /// enable / T/R relay), e.g. `/sys/class/gpio/gpio960/value`.
    pub ptt_gpio: String,
    /// Time between raising the PTT line and the first RF sample, for a relay
    /// to settle. Also held after the last sample before the line drops.
    pub ptt_delay_ms: u32,
    /// Driver-level IIO context root, for tests (`/sys/bus/iio/devices`).
    pub iio_root: String,
    /// Where the IIO buffer character devices live (`/dev`).
    pub dev_root: String,
    /// Samples per DMA block at the post-FPGA rate.
    pub buffer_samples: usize,
    /// Simulated radio: pace blocks to real time (off only for tests).
    pub sim_realtime: bool,
}

impl Default for RadioConfig {
    fn default() -> Self {
        RadioConfig {
            backend: Backend::Iio,
            adc_rate: 3_072_000,
            fpga_decimation: true,
            rf_bandwidth: 1_000_000,
            rx_gain_mode: GainMode::SlowAttack,
            rx_gain_db: 40.0,
            tx_attenuation_db: 20.0,
            lo_offset_hz: 25_000.0,
            freq_min_hz: 47e6,
            freq_max_hz: 6e9,
            ptt_gpio: String::new(),
            ptt_delay_ms: 20,
            iio_root: "/sys/bus/iio/devices".into(),
            dev_root: "/dev".into(),
            buffer_samples: 3_840,
            sim_realtime: true,
        }
    }
}

impl RadioConfig {
    /// Sample rate the ARM sees.
    pub fn stream_rate(&self) -> f64 {
        if self.fpga_decimation { self.adc_rate as f64 / 8.0 } else { self.adc_rate as f64 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DecoderKind {
    Q65,
    Cw,
    Pi4,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TrxConfig {
    pub freq_hz: f64,
    /// Hamlib-style mode name: USB, LSB, CW, CWR, PKTUSB, AM, FM.
    pub mode: String,
    pub tci_bind: String,
    pub tci_port: u16,
    pub rigctl_bind: String,
    pub rigctl_port: u16,
    /// Whether network clients may key the transmitter at all.
    pub allow_tx: bool,
    /// Transmit time-out: unkey after this many seconds of continuous TX.
    pub max_tx_seconds: u32,
    /// Slot decoders running at start (q65 / pi4); the web UI switches
    /// them at run time. CW is the live CW box, always on; "cw" here is ignored.
    pub decoders: Vec<DecoderKind>,
    /// Q65 sub-mode the decoder listens for (T/R 60 s): A..E.
    pub q65_submode: String,
    /// Ignored: the CW skimmer these configured is gone (the live CW box reads
    /// the tuned station instead). Still accepted so older trxd.toml files load.
    pub cw_slots: u8,
    /// Ignored, as `cw_slots`.
    pub cw_squelch_db: i16,
    /// Live CW box engine: "timing" (classic, a character as soon as it is
    /// keyed, a few % CPU) or "neural" (DeepCW: copies weaker signals, but two
    /// Cortex-A9 cores cannot run it in real time). The web UI switches it.
    pub cw_engine: String,
    /// CW keyer speed for `KY`/`send_morse` from rigctl.
    pub cw_wpm: u8,
}

impl Default for TrxConfig {
    fn default() -> Self {
        TrxConfig {
            freq_hz: 144_174_000.0,
            mode: "USB".into(),
            tci_bind: "0.0.0.0".into(),
            tci_port: 40001,
            rigctl_bind: "0.0.0.0".into(),
            rigctl_port: 4532,
            allow_tx: true,
            max_tx_seconds: 180,
            decoders: Vec::new(),
            q65_submode: "D".into(),
            cw_slots: 2,
            cw_squelch_db: 6,
            cw_engine: "timing".into(),
            cw_wpm: 20,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BeaconDigital {
    None,
    Pi4,
    #[serde(rename = "q65-60a")]
    Q65_60A,
    #[serde(rename = "q65-60b")]
    Q65_60B,
    #[serde(rename = "q65-60c")]
    Q65_60C,
    #[serde(rename = "q65-60d")]
    Q65_60D,
    #[serde(rename = "q65-60e")]
    Q65_60E,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BeaconConfig {
    /// The carrier ("mark") frequency. Digital tone 0 and CW key-down sit here.
    pub freq_hz: f64,
    /// Even-minute digital mode.
    pub digital: BeaconDigital,
    pub cw_wpm: u8,
    /// Key-up shift below the carrier (F1A, like MGMBeacon). 0 = on/off keying.
    pub cw_space_shift_hz: f64,
    /// CW text; empty builds "<CALL> <CALL> LOC <LOC> <LOC> " like MGMBeacon.
    pub cw_text: String,
    /// Digital tone 0 offset from the carrier, Hz.
    pub tone0_offset_hz: f64,
    /// Refuse to key a timed sequence without a synchronised clock; send
    /// "NOTIME" + CW + carrier instead, like MGMBeacon.
    pub require_time_sync: bool,
    /// Samples queued between trxd and the antenna, expressed as time: the
    /// sequence is started this much early so it reaches the air on second 0.
    pub tx_latency_ms: u32,
    /// Send "HNY HNY" before the CW on 31 December.
    pub hny: bool,
}

impl Default for BeaconConfig {
    fn default() -> Self {
        BeaconConfig {
            freq_hz: 1_296_872_000.0,
            digital: BeaconDigital::Q65_60D,
            cw_wpm: 12,
            cw_space_shift_hz: 400.0,
            cw_text: String::new(),
            tone0_offset_hz: 0.0,
            require_time_sync: true,
            tx_latency_ms: 40,
            hny: true,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BeaconRxConfig {
    /// The beacon's carrier frequency.
    pub freq_hz: f64,
    /// Receiver dial sits this far below the carrier, so the carrier and CW
    /// land at this audio frequency (the network's 800 Hz convention).
    pub audio_offset_hz: f64,
    pub decoders: Vec<DecoderKind>,
    /// Q65 sub-mode letter (T/R 60 s).
    pub q65_submode: String,
    /// Keep each minute's 12 kHz audio as a WAV in this directory ("" = off).
    pub wav_dir: String,
}

impl Default for BeaconRxConfig {
    fn default() -> Self {
        BeaconRxConfig {
            freq_hz: 1_296_872_000.0,
            audio_offset_hz: 800.0,
            decoders: vec![DecoderKind::Pi4, DecoderKind::Q65, DecoderKind::Cw],
            q65_submode: "D".into(),
            wav_dir: String::new(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MqttConfig {
    pub enabled: bool,
    pub host: String,
    pub port: u16,
    pub username: String,
    pub password: String,
    /// `{hostname}` is replaced with the device hostname.
    pub topic_prefix: String,
    pub client_id: String,
}

impl Default for MqttConfig {
    fn default() -> Self {
        MqttConfig {
            enabled: true,
            host: "127.0.0.1".into(),
            port: 1883,
            username: String::new(),
            password: String::new(),
            topic_prefix: "trxd/{hostname}".into(),
            client_id: String::new(),
        }
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Config, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Config, String> {
        let cfg: Config = toml::from_str(text).map_err(|e| e.to_string())?;
        cfg.validate()?;
        Ok(cfg)
    }

    fn validate(&self) -> Result<(), String> {
        let r = &self.radio;
        if r.fpga_decimation && r.adc_rate % 8 != 0 {
            return Err("radio.adc_rate must be a multiple of 8 with fpga_decimation".into());
        }
        let stream = r.stream_rate();
        if stream < 96_000.0 || (stream / 48_000.0).fract() != 0.0 {
            return Err(format!(
                "stream rate {stream} S/s must be a multiple of 48 kS/s, at least 96 kS/s"
            ));
        }
        if r.lo_offset_hz.abs() * 2.0 > stream * 0.8 {
            return Err("radio.lo_offset_hz does not fit inside the stream bandwidth".into());
        }
        let loc = self.locator.trim();
        if !(loc.len() == 4 || loc.len() == 6) {
            return Err("locator must be 4 or 6 characters".into());
        }
        for sub in [&self.trx.q65_submode, &self.beacon_rx.q65_submode] {
            if !matches!(sub.to_ascii_uppercase().as_str(), "A" | "B" | "C" | "D" | "E") {
                return Err(format!("q65 sub-mode '{sub}' is not one of A..E"));
            }
        }
        Ok(())
    }

    /// Hostname, for the MQTT topic prefix.
    pub fn hostname() -> String {
        std::fs::read_to_string("/etc/hostname")
            .or_else(|_| std::fs::read_to_string("/proc/sys/kernel/hostname"))
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|_| "trxd".into())
    }

    pub fn topic_prefix(&self) -> String {
        self.mqtt.topic_prefix.replace("{hostname}", &Self::hostname())
    }

    /// The CW identification text, MGMBeacon style, unless configured.
    pub fn beacon_cw_text(&self) -> String {
        if !self.beacon.cw_text.trim().is_empty() {
            return self.beacon.cw_text.clone();
        }
        let call = self.callsign.trim().to_ascii_uppercase();
        let loc = self.locator.trim().to_ascii_uppercase();
        format!("{call} {call} LOC {loc} {loc} ")
    }

    /// 4-character locator, for WSJT-style messages.
    pub fn locator4(&self) -> String {
        self.locator.trim().to_ascii_uppercase().chars().take(4).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_config_is_valid() {
        let c = Config::parse("").unwrap();
        assert_eq!(c.role, Role::Trx);
        assert_eq!(c.radio.stream_rate(), 384_000.0);
    }

    #[test]
    fn beacon_config_parses() {
        let c = Config::parse(
            r#"
            role = "beacon-tx"
            callsign = "SR3LES"
            locator = "JO81HU"
            [beacon]
            freq_hz = 1296872000
            digital = "pi4"
            "#,
        )
        .unwrap();
        assert_eq!(c.role, Role::BeaconTx);
        assert_eq!(c.beacon.digital, BeaconDigital::Pi4);
        assert_eq!(c.beacon_cw_text(), "SR3LES SR3LES LOC JO81HU JO81HU ");
    }

    #[test]
    fn rejects_unknown_keys_and_bad_rates() {
        assert!(Config::parse("bogus = 1").is_err());
        assert!(Config::parse("[radio]\nadc_rate = 1000000").is_err());
    }
}
