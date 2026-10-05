//! `role = "trx"`: the remote transceiver.
//!
//! One engine thread, driven by receive blocks (10 ms each):
//!
//! ```text
//! RX 384k ─┬─ DDC(vfo) ─ 48k IQ ─┬─ demod ─ AGC ─ audio ─┬─ TCI RX audio
//!          │                     │                      └─ 12k ─┬─ Q65/PI4 slots ─ decoders ─ MQTT
//!          │                     │                              └─ live CW (DeepCW at the pitch)
//!          └─ DDC(0) ─ TCI IQ (48/96/192/384k)
//!
//! TCI TX audio / CW keyer / tune ─ modulator 48k ─ DUC ─ NCO(vfo) ─ TX 384k
//! ```
//!
//! Transmit is paced by receive: every RX block produces exactly one TX block
//! (silence when not keyed), so the DAC queue never drifts.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use num_complex::Complex32;
use sdroxide_dsp::{
    Agc, AutoNotch, Cessb, Ddc, Decimator, Demodulator, Duc, Modulator, Nco, NoiseBlanker, RealFir, RealFirDecim, SpectralNr,
    SsbMod,
    lowpass_taps, make_demod, make_modulator,
};
use sdroxide_rigctld::{RigState, RigctldController, ServerRequest as RigRequest};
use sdroxide_tci::server::{ServerRequest as TciRequest, TciServerController, TciStateSnapshot};
use sdroxide_types::{
    AgcMode, Band, Command, DeviceCaps, Mode, RigctldConfig, TciServerConfig, TxTelemetry, Vfo,
};
use tracing::{debug, info, warn};

use crate::config::{Config, DecoderKind, GainMode};
use crate::cwlive::CwLiveThread;
use crate::settings::{Settings, Transverter};
use crate::decode::{Decode, DecodeWorker, Job, Q65Letter};
use crate::keyer::CwKeyer;
use crate::pace::TxPace;
use crate::radio::RadioControl;
use crate::scope::{self, Scope};
use crate::web::WebHandle;
use crate::slots::SlotRecorder;
use crate::stream::{RxBlock, time_synced};

/// Channel (audio) rate.
const CH_RATE: f64 = 48_000.0;
/// What the FPGA channel DDC passes flat each side of the VFO (the widest
/// filter is narrower).
const CHAN_PASS_HZ: f64 = 12_000.0;
/// Demodulator rate for the narrow modes (SSB, CW, data): a quarter of the
/// channel, where their 3 kHz fits with room to spare. The demodulator's
/// passband FIR is the engine's biggest cost at 48 kHz (measured ~45 % of a
/// Cortex-A9 core); here it is a quarter of that.
const NARROW_RATE: f64 = 12_000.0;
/// Wide FM's channel: +/-96 kHz holds a broadcast station's +/-75 kHz
/// deviation and its 57 kHz RDS; the FPGA DDC makes it (3.072 MS/s / 16)
/// with this passband (its last stage halves the rate, so under 96 kHz).
const WFM_RATE: f64 = 192_000.0;
const WFM_PASS_HZ: f64 = 85_000.0;
/// How far the WFM channel reaches either side of the dial (retune keeps it
/// inside the stream).
const WFM_HALF_HZ: f64 = 100_000.0;

/// Whether a mode demodulates at [`NARROW_RATE`].
fn narrow_mode(m: Mode) -> bool {
    !matches!(m, Mode::Am | Mode::Sam | Mode::Nfm | Mode::Wfm | Mode::Dsb)
}
/// CW sidetone pitch: CW sits this far above the dial, as in sdroxide.
const CW_PITCH_HZ: f64 = 700.0;
/// Tune carrier offset in the sideband modes.
const TUNE_TONE_HZ: f64 = 1_000.0;
/// Keep the VFO at least this far from the LO (DC spike) and from the band edge.
const EDGE_MARGIN_HZ: f64 = 5_000.0;
/// MUTE AT TX: the receiver stays muted this long after unkeying (T/R switching,
/// the tail of our own signal).
const RX_RECOVER: Duration = Duration::from_millis(150);
/// DATV: baseband amplitude (RMS) before Drive; the RRC-shaped QPSK peaks
/// stay under full scale.
/// DATV: unkey when the browser sent no video or audio for this long.
const DATV_STARVE: Duration = Duration::from_secs(10);
/// DATV symbol rates offered: whole samples per symbol at 384 kS/s.
/// Unkey after this long without TX audio from the keying TCI client.
const TCI_STARVE: Duration = Duration::from_millis(1_500);
/// CW keyer: stay keyed this long after the last element (semi break-in).
const CW_HANG: Duration = Duration::from_millis(400);
/// Web microphone: audio buffered before the first sample goes out (12 kHz).
const MIC_PREROLL: usize = 1_200;
/// Longest a released PTT waits for the microphone queue (0.5 s deep).
const MIC_DRAIN_MAX: Duration = Duration::from_millis(1_500);
/// Web microphone silent this long while keyed: the browser is gone.
const MIC_STARVE: Duration = Duration::from_secs(2);
/// Scope spans served from the 48 kS/s channel (finer bins) up to this. The
/// view stays put while the VFO moves inside it, so the channel must cover a
/// whole span either side of the VFO: 2 x 10 kHz fits in its +/-21.6 kHz.
const NARROW_SPAN_MAX: f64 = 20_000.0;
/// Up to this the ARM FFT of the decimated stream serves the scope; wider
/// spans come from Maia's spectrometer in the FPGA (full ADC rate).
const STREAM_SPAN_MAX: f64 = 300_000.0;
/// Web scope spans up to this keep the LO (the AD936x DC spike) outside the
/// view: the LO goes beside the view, at most `rate * 0.4` from the VFO, which
/// leaves tuning room either way up to +/-50 kHz. Wider views blank it instead.
const DC_AVOID_SPAN_MAX: f64 = 100_000.0;
/// Room between the edge of the view and the LO.
const DC_GUARD_HZ: f64 = 5_000.0;

/// One source's raw level of a band (power.rs), its own dBFS.
#[derive(Debug, Clone)]
struct RawLevel {
    dbfs: f64,
    /// The strongest narrow signal in the band alone.
    peak_dbfs: f64,
    noise_dbfs_hz: Option<f64>,
    /// The spectrum's running number (a fresh one after a change: +2).
    seq: u64,
}

/// The level meter's latest reading (power.rs, calib.rs).
#[derive(Debug, Clone)]
struct Meter {
    /// Power in the measured band at the antenna socket, dBm.
    dbm: f64,
    /// Noise density there, dBm/Hz.
    dbm_hz: Option<f64>,
    /// The same at the measurement point, channel-scale dBFS.
    dbfs: f64,
    quality: crate::calib::Quality,
    /// A stream sample at or near full scale since the last reading.
    clip: bool,
    /// The band holds less than twice the noise in it: mostly noise.
    noise: bool,
    /// "channel", "stream", "maia" or "none".
    src: &'static str,
    /// dB to add to the scope's dBFS for dBm (calibrated boards only).
    scope_off: Option<f64>,
}

impl Default for Meter {
    fn default() -> Self {
        Meter { dbm: -160.0, dbm_hz: None, dbfs: -160.0, quality: crate::calib::Quality::None, clip: false, noise: true, src: "none", scope_off: None }
    }
}

/// Where the transmit audio is coming from right now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxSource {
    /// A TCI client holds the key and streams audio.
    Tci,
    /// PTT from rigctl / MQTT: TCI audio if any client streams it, else silence.
    Ptt,
    /// Keyer text from `cmd/cw`: unkeys by itself when the text is sent.
    Cw,
    /// PTT held in CW mode: the keyer sends whatever is queued, and the
    /// transmitter stays up until PTT is released.
    CwPtt,
    Tune,
    /// The web UI's PTT in CW mode: a straight key, carrier at the pitch
    /// while it is held, with shaped edges.
    Key,
    /// A browser's microphone (web UI), identified by its connection.
    Web(u64),
    /// DATV from a browser's camera and microphone (DVB-S2).
    Datv(u64),
}

/// A DATV transmission in progress: the browser's H.264/Opus multiplexed
/// into MPEG-TS and modulated as DVB-S2 at the stream rate.
struct Datv {
    sr: f64,
    profile: crate::dvbs2::ts::Profile,
    video_bps: f64,
    /// Software modulator (short frames at the stream rate), or None when
    /// the FPGA transmits (`fpga`).
    fpga: Option<crate::dvbs2::fpga_tx::Transmitter>,
    /// DVB-T2 (its own thread; raw IQ through the FPGA interpolator).
    t2: Option<crate::dvbt2::tx::T2Tx>,
    /// Encoder bytes not yet written (FPGA).
    pending: Vec<u8>,
    rate_label: String,
    pilots: bool,
    rolloff: f32,
    ts_rate: f64,
    mux: crate::dvbs2::ts::Mux,
    last_media: Instant,
}

pub struct Trx {
    cfg: Config,
    radio: Box<dyn RadioControl>,
    rate: f64,
    block: usize,
    /// TX analog bandwidth now set (DVB-T2 needs more than the default).
    tx_bw: u32,
    /// The RX filter's bandwidth now (wider while receiving DVB-T2).
    rx_bw: u32,

    // Radio state
    vfo_a: f64,
    vfo_b: f64,
    active: Vfo,
    split: bool,
    /// RX and TX LOs apart (cross-band DATV): (RX, TX).
    lo_split: Option<(f64, f64)>,
    mode: Mode,
    filter: (f32, f32),
    volume: f32,
    muted: bool,
    drive: f32,
    center: f64,
    rx_gain_mode: GainMode,
    rx_gain_db: f64,
    tx_att_db: f64,
    /// RIT / XIT: (on, offset Hz) added to the receive / transmit frequency.
    rit: (bool, f64),
    xit: (bool, f64),
    /// Mode and filter of VFO A and B (the active one's are `mode`/`filter`).
    vfo_mode: [(Mode, (f32, f32)); 2],
    /// Transverters and S-meter calibration, kept on the board.
    settings: Settings,
    settings_dir: std::path::PathBuf,
    /// The transverter the LO is currently tuned through (None: direct).
    xvtr: Option<Transverter>,

    // Receive DSP
    ddc: Ddc,
    /// The channel from the trx bitstream's DDC (a ring in DDR) instead:
    /// the NCO and the decimation off the ARM (docs/PERFORMANCE.md).
    chan_fpga: Option<crate::dvbs2::fpga::FrontEnd>,
    chan_words: Vec<u32>,
    demod: Box<dyn Demodulator>,
    agc: Agc,
    agc_mode: AgcMode,
    nb: Option<NoiseBlanker>,
    notch: Option<AutoNotch>,
    nr: Option<SpectralNr>,
    /// Squelch threshold (channel dBFS; None = open) and whether it is open.
    squelch_db: Option<f32>,
    squelch_open: bool,
    /// Gain-compensated demodulator level, smoothed: only for the old
    /// per-band S-meter points (settings.json), used where no calibration
    /// table exists.
    reading_db: f64,
    /// Level meter (power.rs): the 48 kS/s channel every mode shares, and
    /// the stream for bands wider than it; their latest averaged spectra.
    chan_meter: crate::power::PowerMeter,
    stream_meter: crate::power::PowerMeter,
    chan_spec: Option<crate::power::Spectrum>,
    stream_spec: Option<crate::power::Spectrum>,
    /// The stream meter runs until then (or while DATV is received).
    stream_meter_until: Instant,
    /// The latest Maia spectrometer row (wide bands).
    maia_last: Option<Vec<f32>>,
    /// Largest stream sample magnitude since the last reading.
    stream_peak: f32,
    /// (dial, LO, socket) the meters last averaged at: a change restarts them.
    meter_at: (f64, f64, u8),
    /// Per RX socket pair (index 0: RX1): the board's level calibration.
    calib: [Option<crate::calib::Calib>; 2],
    meter: Meter,
    /// Front-end gain actually in force (the AD936x AGC moves it), read by a
    /// background thread (an SPI round trip takes ~70 ms) as f64 bits.
    hw_gain_db: f64,
    hw_gain: Option<std::sync::Arc<std::sync::atomic::AtomicU64>>,
    /// FPGA and AD936x temperatures (web header).
    temps: std::sync::Arc<std::sync::Mutex<crate::temps::Temps>>,
    chan: Vec<Complex32>,
    /// The channel's rate: CH_RATE, WFM_RATE in wide FM.
    chan_rate: f64,
    /// DATV mode, DDC skipped: the fraction of a channel sample carried over.
    chan_frac: f64,
    /// The mode demodulates at 12 kHz (see [`NARROW_RATE`]).
    narrow: bool,
    dec4: Decimator,
    chan12: Vec<Complex32>,
    /// 12 -> 48 kHz for TCI clients' receive audio in the narrow modes.
    tci_up: RealFir,
    audio: Vec<f32>,
    iq_tap: Option<(u32, Ddc)>,
    iq_buf: Vec<Complex32>,
    dec12: RealFirDecim,
    audio12: Vec<f32>,
    slots: Vec<(DecoderKind, SlotRecorder)>,
    decoder: DecodeWorker,
    /// Live CW copy of the station at the CW pitch (web CW box, MQTT cw/text).
    cwlive: CwLiveThread,
    /// The CW decoder's own slow AGC, on the audio before the listening AGC:
    /// a fast AGC pumps up the noise in every key-up (F1A beacons leave the
    /// passband silent then) and the pops read as dits.
    cw_agc: Agc,
    cw_audio: Vec<f32>,
    s_dbfs: f32,

    // Transmit DSP
    tx_on: Option<TxSource>,
    tx_since: Option<Instant>,
    /// RF is on (from set_tx_rf(true) until unkey): the TX LO stays put.
    rf_live: bool,
    /// The AD936x TX LO as last written (its own frequency, not the air's).
    hw_tx_lo: Option<f64>,
    /// MUTE AT TX: the receiver is muted until then after TX.
    rx_quiet_until: Option<Instant>,
    modulator: Option<Box<dyn Modulator>>,
    datv: Option<Datv>,
    /// DATV mode (the web UI's DATV button): no voice chain; the DVB-S2
    /// receiver runs, and a browser may send.
    datv_mode: bool,
    /// DATV reception (DVB-S2 receiver thread), when switched on.
    datv_rx: Option<crate::dvbs2::rx::RxThread>,
    /// The receiver the browser asked for (symbol rate, mode, pilots): it
    /// pauses while this board sends DATV (the A9 cannot do both, a DVB-T2
    /// receiver starves the modulator) and resumes afterwards.
    datv_rx_req: Option<(f64, String)>,
    datv_rx_stats: crate::dvbs2::rx::Stats,
    /// Frames decoded, and frames seen, when the decoded count last moved.
    datv_rx_watch: (u64, u64, Instant),
    /// Automatic receive (symbol rate "0"): the blind scan while no
    /// receiver runs, what it found last, the receiver's last progress.
    datv_scan: Option<crate::dvbs2::scan::Scanner>,
    datv_auto: bool,
    datv_auto_sr: Option<f64>,
    datv_auto_note: String,
    datv_auto_good: (u64, Instant),
    datv_rx_log: Instant,
    duc: Duc,
    tx_nco: Nco,
    keyer: CwKeyer,
    cw_idle_since: Option<Instant>,
    tune_phase: f64,
    tx_fifo: VecDeque<f32>,
    tx_bb: Vec<Complex32>,
    tx_up: Vec<Complex32>,
    tx_out: VecDeque<Complex32>,
    pace: TxPace,
    tci_last_audio: Instant,
    tx_sink: Sender<crate::stream::TxBlock>,
    /// Speech compressor (controlled-envelope SSB) and its drive, dB.
    comp: Option<Cessb>,
    /// The band whose socket mapping was last applied (see `follow_port`).
    port_band: Option<String>,
    /// Mic meter and speech compressor (SSB only) ahead of the modulator.
    speech: crate::speech::SpeechProc,
    /// Peak TX envelope sent since the meter last read it (1.0 = DAC full scale).
    tx_peak: f32,
    comp_db: f32,
    mic_gain: f32,
    /// CW semi break-in: stay keyed this long after the last element.
    cw_hang: Duration,
    /// Straight key: held down, and the envelope (0..1) of its carrier.
    key_down: bool,
    key_env: f32,

    // Web UI
    web: Option<WebHandle>,
    scope_wide: Scope,
    scope_narrow: Scope,
    maia: Option<Receiver<Vec<f32>>>,
    web_span: f64,
    /// Centre of the web scope: fixed while the VFO moves inside the view,
    /// recentred on the VFO when it leaves (0 = recentre on the next row).
    web_center: f64,
    /// Scope keeps the VFO in the middle (CTR) instead of a fixed view.
    scope_center: bool,
    web_audio: Vec<f32>,
    /// WFM: the 48 kHz audio halved for the web (24 kHz, frame type 9).
    dec24: RealFirDecim,
    web_audio24: Vec<f32>,
    /// The last RDS sent to the web (only changes go out).
    rds_sent: Option<String>,
    rds_freq: f64,
    web_state: Option<String>,
    web_state_at: Instant,
    web_meter_at: Instant,
    mic_up: RealFir,
    mic_started: bool,
    mic_last: Instant,
    mic_buf: Vec<f32>,
    /// PTT released with `drain`: transmit what the microphone queue still
    /// holds (RADE's end-of-over frame), then unkey; at the latest then.
    mic_drain_until: Option<Instant>,
    /// RADE V2 in the page (src/rade.rs): DATA mode with the browsers'
    /// decoder on; shared so that every page shows and decodes it.
    rade: bool,

    // Control surfaces
    tci: Option<TciServerController>,
    rig: Option<RigctldController>,
    last_rig: Option<RigState>,
    last_tci: Option<TciStateSnapshot>,
    /// Engine time per stage since the last load report (see [`STAGES`]).
    prof: [Duration; STAGES.len()],
}

/// Stages of one engine block, for the per-minute profile.
const STAGES: [&str; 9] = ["tci_iq", "ddc", "rx_dsp", "audio12", "scope", "slots", "tci_audio", "transmit", "rest"];

fn mode_name(m: Mode) -> &'static str {
    sdroxide_rigctld::to_hamlib_mode(m)
}

/// RX and TX in two bands (calib::BANDS; off the table, more than a
/// twentieth of the frequency apart). Full duplex (the DATV receiver on
/// while sending) only across bands: on one band the board's own
/// transmitter is all its receiver hears.
fn cross_band(rx: f64, tx: f64) -> bool {
    let band = |f: f64| crate::calib::BANDS.iter().position(|b| f >= b.1 && f <= b.2);
    match (band(rx), band(tx)) {
        (Some(a), Some(b)) => a != b,
        _ => (rx - tx).abs() > 0.05 * rx.max(tx),
    }
}

impl Trx {
    pub fn new(
        cfg: Config,
        radio: Box<dyn RadioControl>,
        tx_sink: Sender<crate::stream::TxBlock>,
        web: Option<WebHandle>,
    ) -> Self {
        let rate = radio.stream_rate();
        let block = cfg.radio.buffer_samples;
        let mode = sdroxide_rigctld::from_hamlib_mode(&cfg.trx.mode).unwrap_or(Mode::Usb);
        let filter = mode.default_filter();
        let vfo = cfg.trx.freq_hz;
        let settings_dir = std::path::PathBuf::from(&cfg.web.state_dir);
        let settings = Settings::load(&settings_dir);

        let tci = {
            let tc = TciServerConfig {
                enabled: true,
                bind: cfg.trx.tci_bind.clone(),
                port: cfg.trx.tci_port,
                device_name: "tezuka-trxd".into(),
                allow_tx: cfg.trx.allow_tx,
                max_clients: 4,
            };
            let snap = TciStateSnapshot::default();
            match TciServerController::start(&tc, &Self::caps(&cfg, rate), snap) {
                Ok(t) => {
                    info!(addr = t.addr(), "TCI server");
                    Some(t)
                }
                Err(e) => {
                    warn!("TCI server: {e}");
                    None
                }
            }
        };
        let rig = {
            let rc = RigctldConfig {
                enabled: true,
                bind: cfg.trx.rigctl_bind.clone(),
                port: cfg.trx.rigctl_port,
                allow_tx: cfg.trx.allow_tx,
                max_clients: 4,
                rig_name: "tezuka-trxd".into(),
            };
            match RigctldController::start(&rc, RigState::default()) {
                Ok(r) => {
                    info!(addr = r.addr(), "rigctld server");
                    Some(r)
                }
                Err(e) => {
                    warn!("rigctld server: {e}");
                    None
                }
            }
        };

        let slots = cfg.trx.decoders.iter().filter_map(|d| slot_recorder(*d).map(|r| (*d, r))).collect();

        let mut agc = Agc::new(CH_RATE);
        agc.set_mode(AgcMode::Med);

        let mut t = Trx {
            radio,
            rate,
            block,
            tx_bw: cfg.radio.rf_bandwidth,
            rx_bw: cfg.radio.rf_bandwidth,
            vfo_a: vfo,
            vfo_b: vfo,
            active: Vfo::A,
            split: false,
            lo_split: None,
            mode,
            filter,
            volume: 0.5,
            muted: false,
            drive: 1.0,
            center: 0.0,
            rx_gain_mode: cfg.radio.rx_gain_mode,
            rx_gain_db: cfg.radio.rx_gain_db,
            tx_att_db: cfg.radio.tx_attenuation_db,
            rit: (false, 0.0),
            xit: (false, 0.0),
            vfo_mode: [(mode, filter); 2],
            settings,
            settings_dir,
            xvtr: None,
            nb: None,
            notch: None,
            nr: None,
            squelch_db: None,
            squelch_open: true,
            reading_db: -120.0,
            chan_meter: crate::power::PowerMeter::new(CH_RATE, 0),
            stream_meter: crate::power::PowerMeter::new(rate, 2 * crate::power::N),
            chan_spec: None,
            stream_spec: None,
            stream_meter_until: Instant::now(),
            maia_last: None,
            stream_peak: 0.0,
            meter_at: (0.0, 0.0, 0),
            calib: crate::calib::Calib::load_both(&std::path::PathBuf::from(&cfg.web.state_dir)),
            meter: Meter::default(),
            hw_gain_db: cfg.radio.rx_gain_db,
            hw_gain: None,
            temps: Default::default(),
            comp: None,
            port_band: None,
            speech: Default::default(),
            tx_peak: 0.0,
            comp_db: 10.0,
            mic_gain: 1.0,
            cw_hang: CW_HANG,
            key_down: false,
            key_env: 0.0,
            scope_center: false,
            ddc: Ddc::new(rate, CH_RATE),
            chan_fpga: None,
            chan_words: Vec::new(),
            demod: make_demod(mode, CH_RATE).expect("sideband demod"),
            agc,
            agc_mode: AgcMode::Med,
            chan: Vec::new(),
            chan_frac: 0.0,
            chan_rate: CH_RATE,
            narrow: false,
            dec4: Decimator::new(4),
            chan12: Vec::new(),
            tci_up: RealFir::new(lowpass_taps(95, 3_600.0 / CH_RATE)),
            audio: Vec::new(),
            iq_tap: None,
            iq_buf: Vec::new(),
            dec12: RealFirDecim::new(63, 5_000.0, CH_RATE, 4),
            audio12: Vec::new(),
            slots,
            decoder: DecodeWorker::new(),
            cw_agc: {
                let mut a = Agc::new(NARROW_RATE);
                a.set_mode(AgcMode::Slow);
                a
            },
            cw_audio: Vec::new(),
            cwlive: CwLiveThread::start(12_000.0, CW_PITCH_HZ as f32),
            s_dbfs: -120.0,
            tx_on: None,
            tx_since: None,
            rf_live: false,
            hw_tx_lo: None,
            rx_quiet_until: None,
            modulator: None,
            datv: None,
            datv_mode: false,
            datv_rx: None,
            datv_rx_req: None,
            datv_rx_stats: Default::default(),
            datv_rx_watch: (0, 0, Instant::now()),
            datv_scan: None,
            datv_auto: false,
            datv_auto_sr: None,
            datv_auto_note: String::new(),
            datv_auto_good: (0, Instant::now()),
            datv_rx_log: Instant::now(),
            duc: Duc::new(CH_RATE, rate),
            tx_nco: Nco::new(0.0, rate),
            keyer: CwKeyer::new(CH_RATE, CW_PITCH_HZ, cfg.trx.cw_wpm as f32),
            cw_idle_since: None,
            tune_phase: 0.0,
            tx_fifo: VecDeque::new(),
            tx_bb: Vec::new(),
            tx_up: Vec::new(),
            tx_out: VecDeque::new(),
            pace: TxPace::default(),
            tci_last_audio: Instant::now(),
            tx_sink,
            web,
            scope_wide: Scope::new(4096, rate, 15.0, 45.0),
            scope_narrow: Scope::new(2048, CH_RATE, 15.0, 23.0),
            maia: None,
            web_span: 96_000.0,
            web_center: 0.0,
            web_audio: Vec::new(),
            dec24: RealFirDecim::new(63, 11_000.0, CH_RATE, 2),
            web_audio24: Vec::new(),
            rds_sent: None,
            rds_freq: 0.0,
            web_state: None,
            web_state_at: Instant::now(),
            web_meter_at: Instant::now(),
            // x4 interpolation filter for 12 -> 48 kHz microphone audio.
            mic_up: RealFir::new(lowpass_taps(95, 3_600.0 / CH_RATE)),
            mic_started: false,
            mic_last: Instant::now(),
            mic_buf: Vec::new(),
            mic_drain_until: None,
            rade: false,
            tci,
            rig,
            last_rig: None,
            last_tci: None,
            prof: [Duration::ZERO; STAGES.len()],
            cfg,
        };
        t.retune(true);
        t.rebuild_demod();
        if let Some(mut read) = t.radio.rx_gain_reader() {
            let g = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(t.rx_gain_db.to_bits()));
            let out = g.clone();
            let _ = std::thread::Builder::new().name("rx-gain".into()).spawn(move || loop {
                if let Some(db) = read() {
                    out.store(db.to_bits(), std::sync::atomic::Ordering::Relaxed);
                }
                std::thread::sleep(Duration::from_secs(1));
            });
            t.hw_gain = Some(g);
        }
        if t.web.is_some() && t.cfg.radio.backend == crate::config::Backend::Iio {
            t.temps = crate::temps::start();
            t.maia = crate::maia::start(t.cfg.radio.adc_rate as f64, 15.0);
        }
        if t.cfg.radio.fpga_ddc && t.cfg.radio.backend == crate::config::Backend::Iio && !t.datv_mode {
            match crate::dvbs2::fpga::FrontEnd::start_channel(CH_RATE, CHAN_PASS_HZ, t.chan_offset()) {
                Ok(fe) => t.chan_fpga = Some(fe),
                Err(e) => info!("the channel on the ARM: {e}"),
            }
        }
        t
    }

    /// The channel's offset from the LO as the DDC (either) mixes it down:
    /// through an inverting transverter the AD936x sees the mirror image,
    /// so the FPGA's DDC takes the negated offset and its output is
    /// conjugated (the software one gets the stream mirrored first).
    fn chan_offset(&self) -> f64 {
        let off = self.rx_eff() - self.center;
        if self.chan_fpga.is_some() && self.xvtr.as_ref().is_some_and(|t| t.inverted) { -off } else { off }
    }

    fn caps(cfg: &Config, rate: f64) -> DeviceCaps {
        let range = (cfg.radio.freq_min_hz, cfg.radio.freq_max_hz);
        DeviceCaps {
            driver: "tezuka".into(),
            label: "AD936x".into(),
            rx_channels: 1,
            tx_channels: 1,
            full_duplex: true,
            tx_audio: true,
            freq_ranges_rx: vec![range],
            freq_ranges_tx: if cfg.trx.allow_tx { vec![range] } else { Vec::new() },
            sample_rates: [48_000.0, 96_000.0, 192_000.0, 384_000.0].into_iter().filter(|r| *r <= rate).collect(),
            ..DeviceCaps::default()
        }
    }

    fn rx_vfo(&self) -> f64 {
        match self.active {
            Vfo::A => self.vfo_a,
            Vfo::B => self.vfo_b,
        }
    }

    fn tx_vfo(&self) -> f64 {
        match (self.active, self.split) {
            (Vfo::A, false) | (Vfo::B, true) => self.vfo_a,
            _ => self.vfo_b,
        }
    }

    /// Where the receiver listens: the active VFO plus RIT.
    fn rx_eff(&self) -> f64 {
        self.rx_vfo() + if self.rit.0 { self.rit.1 } else { 0.0 }
    }

    /// Where the transmitter sends: the TX VFO plus XIT.
    fn tx_eff(&self) -> f64 {
        self.tx_vfo() + if self.xit.0 { self.xit.1 } else { 0.0 }
    }

    /// Clamp a frequency to what can be reached: the AD936x itself or a
    /// transverter's band.
    fn reachable(&self, hz: f64) -> f64 {
        if self.settings.transverter(hz).is_some() {
            hz
        } else {
            hz.clamp(self.cfg.radio.freq_min_hz, self.cfg.radio.freq_max_hz)
        }
    }

    /// Lowest and highest frequency the operator can tune (the transverter
    /// bands included).
    fn freq_range(&self) -> (f64, f64) {
        self.settings.transverters.iter().fold((self.cfg.radio.freq_min_hz, self.cfg.radio.freq_max_hz), |(a, b), t| {
            (a.min(t.rf_min), b.max(t.rf_max))
        })
    }

    /// Keep the LO so both VFOs sit inside the usable stream, clear of DC.
    /// Retunes the hardware only when a VFO has walked out of the window, so
    /// a TCI client's IQ panorama stays put while the operator tunes.
    ///
    /// Every frequency here is the one on the air; through a transverter the
    /// AD936x is set to its IF, and an LO above the band mirrors the spectrum
    /// (undone by conjugating the samples both ways).
    ///
    /// While RF is on the TX LO never moves (nor the transverter or the
    /// sockets): a change that would need that ends the transmission first,
    /// and so does one that takes the TX frequency out of where it may send.
    fn retune(&mut self, force: bool) {
        if self.rf_live {
            if let Err(e) = self.tx_check(self.tx_eff(), self.tx_half_bw()) {
                warn!("transmission ended: {e}");
                self.unkey();
                if force {
                    self.retune(true);
                }
                return;
            }
        }
        self.follow_port();
        let half = self.rate * 0.4;
        let rx = self.rx_eff();
        let tx = self.tx_eff();
        // (Transmitting, the DC spike in the scope does not matter.)
        let keepout = if self.rf_live { None } else { self.dc_keepout() };
        let clear = |c: f64| keepout.is_none_or(|(a, b)| c < a || c > b);
        let fits = |f: f64, c: f64| {
            let off = f - c;
            off.abs() < half - EDGE_MARGIN_HZ && off.abs() > EDGE_MARGIN_HZ
        };
        // Wide FM's channel reaches WFM_HALF_HZ either side of the dial.
        let rx_half = if self.mode == Mode::Wfm { WFM_HALF_HZ } else { 0.0 };
        let fits_rx = |c: f64| {
            let off = rx - c;
            off.abs() < half - EDGE_MARGIN_HZ - rx_half && off.abs() > EDGE_MARGIN_HZ
        };
        let fits_all = |c: f64| fits_rx(c) && (self.tx_on.is_none() || fits(tx, c));
        let datv = self.datv_lo_offset();
        if datv.is_none() && !force && self.lo_split.is_none() && fits_all(self.center) && clear(self.center) {
            self.apply_offsets();
            return;
        }
        let anchor = if datv.is_some() && self.tx_on.is_none() {
            rx
        } else if datv.is_some() || (self.tx_on.is_some() && !fits(tx, self.center)) {
            tx
        } else {
            rx
        };
        // Sending DATV from the FPGA (LO on the signal) and receiving DATV
        // too far from it for one LO (split across bands): the AD936x's RX
        // and TX synthesizers apart, TX on the signal, RX where it would
        // be alone. Not through transverters.
        if datv == Some(0.0)
            && matches!(self.tx_on, Some(TxSource::Datv(_)))
            && self.datv_rx.is_some()
            && !fits(rx, tx)
            && self.settings.transverter(rx).is_none()
            && self.settings.transverter(tx).is_none()
        {
            let rx_off = if self.datv_rx.as_ref().is_some_and(|r| r.t2_bw.is_some()) { 0.0 } else { self.cfg.radio.lo_offset_hz };
            let (min, max) = (self.cfg.radio.freq_min_hz, self.cfg.radio.freq_max_hz);
            let (rx_lo, tx_lo) = ((rx - rx_off).clamp(min, max), tx.clamp(min, max));
            if !force && self.lo_split == Some((rx_lo, tx_lo)) {
                self.apply_offsets();
                return;
            }
            let moved = if self.rf_live {
                if self.hw_tx_lo.is_none_or(|t| (t - tx_lo).abs() >= 1.0) {
                    return self.end_tx_to_retune(force);
                }
                self.radio.set_rx_lo(rx_lo)
            } else {
                self.radio.set_los(rx_lo, tx_lo)
            };
            if let Err(e) = moved {
                warn!("tune RX {rx_lo} TX {tx_lo}: {e}");
                return;
            }
            self.hw_tx_lo = Some(tx_lo);
            info!(rx_lo, tx_lo, "LOs apart (cross band)");
            self.lo_split = Some((rx_lo, tx_lo));
            self.xvtr = None;
            self.center = rx_lo;
            self.scope_wide.reset();
            self.apply_offsets();
            return;
        }
        let had_split = self.lo_split.take().is_some();
        let xvtr = self.settings.transverter(anchor).cloned();
        // The LO sits `lo_offset` below the signal on the air, which through an
        // inverting transverter is above it on the AD936x.
        let mut lo = match datv {
            Some(off) => anchor - off,
            None => anchor - self.cfg.radio.lo_offset_hz * if xvtr.as_ref().is_some_and(|t| t.inverted) { -1.0 } else { 1.0 },
        };
        // With the web scope open, the DC spike goes beside the view: halfway
        // between the view and the farthest the VFO (and TX) may be from the
        // LO, for the most tuning before the next move.
        if let Some((a, b)) = keepout.filter(|_| datv.is_none() && !clear(lo)) {
            let reach = half - EDGE_MARGIN_HZ;
            let (lo_min, lo_max) = if self.tx_on.is_some() {
                (rx.max(tx) - reach, rx.min(tx) + reach)
            } else {
                (rx - reach, rx + reach)
            };
            let below = (lo_min + a) / 2.0;
            let above = (b + lo_max) / 2.0;
            if lo_min < a && fits_all(below) {
                lo = below;
            } else if lo_max > b && fits_all(above) {
                lo = above;
            } else if !force && !had_split && fits_all(self.center) {
                // No room beside the view (split far apart): stay put.
                self.apply_offsets();
                return;
            }
        }
        if !force && !had_split && (lo - self.center).abs() < 1.0 {
            self.apply_offsets();
            return;
        }
        let (min, max) = (self.cfg.radio.freq_min_hz, self.cfg.radio.freq_max_hz);
        let (lo, hw) = match &xvtr {
            Some(t) => {
                let hw = t.to_if(lo).clamp(min, max);
                (if t.inverted { t.lo_hz - hw } else { hw + t.lo_hz }, hw)
            }
            None => {
                let lo = lo.clamp(min, max);
                (lo, lo)
            }
        };
        let moved = if self.rf_live {
            if xvtr != self.xvtr || self.hw_tx_lo.is_none_or(|t| (t - hw).abs() >= 1.0) {
                return self.end_tx_to_retune(force);
            }
            // Only the RX LO comes back (from a cross-band split).
            self.radio.set_rx_lo(hw)
        } else {
            self.radio.set_lo(hw)
        };
        if let Err(e) = moved {
            warn!("tune {hw}: {e}");
            return;
        }
        self.hw_tx_lo = Some(hw);
        if xvtr != self.xvtr {
            info!(xvtr = xvtr.as_ref().map(|t| t.name.as_str()), "transverter");
            self.xvtr = xvtr;
        }
        debug!(lo, hw, "LO retuned");
        self.center = lo;
        // A row averaged across the move would smear the spectrum.
        self.scope_wide.reset();
        self.apply_offsets();
    }

    /// A frequency change that needs the TX LO elsewhere while RF is on:
    /// the transmission ends, then the receiver retunes.
    fn end_tx_to_retune(&mut self, force: bool) {
        warn!(tx = self.tx_eff(), "frequency change moves the TX LO: transmission ended");
        self.unkey();
        self.retune(force);
    }

    /// May the transmitter send at `f` (Hz on the air), `half` either side?
    /// trx.allow_tx, a transverter's own TX flag, else trx.tx_ranges.
    fn tx_check(&self, f: f64, half: f64) -> Result<(), String> {
        if !self.cfg.trx.allow_tx {
            return Err("trx.allow_tx = false".into());
        }
        if let Some(t) = self.settings.transverter(f) {
            return if t.tx { Ok(()) } else { Err(format!("transverter {} is receive-only", t.name)) };
        }
        if self.cfg.trx.tx_range_ok(f - half, f + half) {
            Ok(())
        } else {
            Err(format!("{:.4} MHz (+-{:.1} kHz) is outside trx.tx_ranges", f / 1e6, half / 1e3))
        }
    }

    /// Half the bandwidth of what is (about to be) sent, Hz.
    fn tx_half_bw(&self) -> f64 {
        match &self.datv {
            // (DVB-T2 keeps its channel width in `sr`.)
            Some(d) if d.t2.is_some() => d.sr / 2.0,
            Some(d) => d.sr * (1.0 + d.rolloff as f64) / 2.0,
            None => (self.filter.0.abs().max(self.filter.1.abs()) as f64).max(1_000.0),
        }
    }

    /// The TX analog filter for what is sent: DVB-T2 is wider than the
    /// default (1 MHz). Set before RF goes on.
    fn datv_tx_bw(&mut self) {
        let want = self.datv.as_ref().and_then(|d| d.t2.as_ref()).map_or(self.cfg.radio.rf_bandwidth, |t| {
            self.cfg.radio.rf_bandwidth.max((t.mode.bw_hz * 1.3) as u32)
        });
        if want != self.tx_bw {
            match self.radio.set_tx_bandwidth(want) {
                Ok(()) => info!(hz = want, "TX bandwidth"),
                Err(e) => warn!("TX bandwidth: {e}"),
            }
            self.tx_bw = want;
        }
    }

    /// Into a band wired to the other sockets (SET > ports): switch to them.
    /// Inside a band the hand-picked pair stays.
    fn follow_port(&mut self) {
        if self.tx_on.is_some() {
            return;
        }
        let band = self.cal_band();
        if self.port_band.as_deref() == Some(band.as_str()) {
            return;
        }
        if let Some(&p) = self.settings.ports.get(&band) {
            self.set_port(p);
        }
        self.port_band = Some(band);
    }

    /// Move to the RX1/TX1 (1) or RX2/TX2 (2) sockets. The AD936x keeps the
    /// LO and gains across the switch (the backend restores them).
    fn set_port(&mut self, p: u8) {
        let p = p.clamp(1, 2);
        if self.radio.port().is_none_or(|now| now == p) {
            return;
        }
        if self.tx_on.is_some() {
            warn!(port = p, "port switch refused while transmitting");
            return;
        }
        match self.radio.set_port(p) {
            Ok(()) => {
                info!(port = p, "RX/TX sockets switched");
                self.settings.port = p;
                self.settings.save(&self.settings_dir);
            }
            Err(e) => warn!(port = p, "port switch: {e}"),
        }
    }

    fn apply_offsets(&mut self) {
        self.ddc.set_offset_hz(self.rx_eff() - self.center);
        let off = self.chan_offset();
        if let Some(fe) = &mut self.chan_fpga {
            fe.set_center(off);
        }
        let tx_off = self.tx_eff() - self.center;
        self.tx_nco.set_freq(tx_off, self.rate);
    }

    /// RDS from the wide-FM demodulator: the station name and RadioText to
    /// the web when they change.
    fn rds_poll(&mut self) {
        // Another station: its RDS starts afresh.
        let vfo = self.rx_vfo();
        if (vfo - self.rds_freq).abs() > 1_000.0 {
            self.rds_freq = vfo;
            self.demod.reset_rds();
            if self.rds_sent.take().is_some() {
                if let Some(w) = &self.web {
                    w.send_json(&serde_json::json!({ "type": "rds", "pi": null, "ps": null, "rt": null }));
                }
            }
            return;
        }
        let Some(r) = self.demod.take_rds() else { return };
        let msg = serde_json::json!({
            "type": "rds",
            "pi": r.pi.map(|p| format!("{p:04X}")),
            "ps": r.ps.as_deref().map(str::trim_end),
            "rt": r.radiotext.as_deref().map(str::trim_end),
        });
        let text = msg.to_string();
        if self.rds_sent.as_deref() != Some(text.as_str()) {
            if let Some(w) = &self.web {
                w.send_json(&msg);
            }
            self.rds_sent = Some(text);
        }
    }

    /// The channel for the mode: 48 kHz, or 192 kHz for wide FM (the FPGA
    /// DDC reprogrammed, else the software DDC from the stream).
    fn rebuild_channel(&mut self) {
        let r = if self.mode == Mode::Wfm { WFM_RATE } else { CH_RATE };
        if r == self.chan_rate {
            return;
        }
        self.chan_rate = r;
        self.ddc = Ddc::new(self.rate, r);
        self.chan_meter = crate::power::PowerMeter::new(r, 0);
        self.scope_narrow = Scope::new(2048, r, 15.0, 23.0);
        if self.chan_fpga.is_some() {
            let inverted = self.xvtr.as_ref().is_some_and(|t| t.inverted);
            let off = self.rx_eff() - self.center;
            // (the old one stops the recorder when dropped)
            self.chan_fpga = None;
            let pass = if r == WFM_RATE { WFM_PASS_HZ } else { CHAN_PASS_HZ };
            match crate::dvbs2::fpga::FrontEnd::start_channel(r, pass, if inverted { -off } else { off }) {
                Ok(fe) => self.chan_fpga = Some(fe),
                Err(e) => info!("the {r} S/s channel on the ARM: {e}"),
            }
        }
        self.rds_sent = None;
        // The LO where the new channel fits (wide FM needs room either side).
        self.retune(false);
        self.apply_offsets();
    }

    fn rebuild_demod(&mut self) {
        self.rebuild_channel();
        let narrow = narrow_mode(self.mode);
        // (wide FM: a 192 kHz channel, 48 kHz audio like the other wide modes)
        let rate = if narrow { NARROW_RATE } else { CH_RATE };
        let demod_rate = if narrow { NARROW_RATE } else { self.chan_rate };
        self.demod = if self.mode == Mode::Wfm {
            // (src/wfm.rs: sdroxide's PC demodulator took 90 % of an A9 core)
            Box::new(crate::wfm::WfmLite::new(demod_rate))
        } else {
            make_demod(self.mode, demod_rate).unwrap_or_else(|| make_demod(Mode::Usb, rate).unwrap())
        };
        self.demod.set_filter(self.filter.0, self.filter.1);
        if narrow != self.narrow {
            self.narrow = narrow;
            self.agc = Agc::new(rate);
            self.agc.set_mode(self.agc_mode);
            self.notch = self.notch.take().map(|_| AutoNotch::new());
            self.nr = self.nr.take().map(|_| SpectralNr::new());
        }
        // CW from a TCI client arrives as a keyed sidetone: single-sideband it
        // around the pitch so it lands at dial + pitch, where CW is received.
        self.modulator = make_modulator(self.mode, CH_RATE, self.filter).or_else(|| {
            (self.mode == Mode::Cw).then(|| Box::new(SsbMod::new(CH_RATE, 300.0, 1_100.0)) as Box<dyn Modulator>)
        });
        if self.comp.is_some() {
            self.comp = Some(self.make_comp());
        }
    }

    /// The COMP depth when it applies: on, and SSB voice (never data modes).
    fn ssb_comp(&self) -> Option<f32> {
        let ssb = matches!(self.mode, Mode::Usb | Mode::Lsb) && !self.datv_mode;
        self.comp.as_ref().filter(|_| ssb).map(|_| self.comp_db)
    }

    /// The speech compressor for the current filter, at the set drive.
    fn make_comp(&self) -> Cessb {
        let (a, b) = (self.filter.0.abs(), self.filter.1.abs());
        let mut c = Cessb::new(CH_RATE, a.min(b).max(100.0), a.max(b).max(300.0));
        // SpeechProc does the compression; CESSB only trims the peaks.
        c.set_compression_db(crate::speech::cessb_drive_db(self.comp_db));
        c
    }

    fn set_vfo(&mut self, vfo: Vfo, hz: f64) {
        let hz = self.reachable(hz);
        let before = self.rx_vfo();
        match vfo {
            Vfo::A => self.vfo_a = hz,
            Vfo::B => self.vfo_b = hz,
        }
        // More than the AFC can follow: another station.
        if (self.rx_vfo() - before).abs() > 30.0 {
            self.cwlive.restart();
        }
        self.retune(false);
    }

    fn set_mode(&mut self, mode: Mode) {
        if mode == self.mode {
            return;
        }
        if self.mode == Mode::Digu || mode == Mode::Digu {
            // The slot decoders pause outside DATA: start their slots afresh.
            for (kind, rec) in &mut self.slots {
                if let Some(r) = slot_recorder(*kind) {
                    *rec = r;
                }
            }
        }
        self.mode = mode;
        if mode != Mode::Digu {
            self.rade = false;
        }
        self.filter = mode.default_filter();
        self.rebuild_demod();
        self.cwlive.restart();
        self.vfo_mode[self.active as usize] = (self.mode, self.filter);
    }

    fn set_filter(&mut self, lo: f32, hi: f32) {
        self.filter = (lo, hi);
        self.demod.set_filter(lo, hi);
        if let Some(m) = &mut self.modulator {
            m.set_filter(lo, hi);
        }
        if let Some(c) = &mut self.comp {
            c.set_filter(lo.abs().min(hi.abs()), lo.abs().max(hi.abs()));
        }
        self.vfo_mode[self.active as usize] = (self.mode, self.filter);
    }

    /// Make `v` the receive VFO, with the mode and filter it was left in.
    fn select_vfo(&mut self, v: Vfo) {
        if v == self.active {
            return;
        }
        self.vfo_mode[self.active as usize] = (self.mode, self.filter);
        self.active = v;
        let (m, f) = self.vfo_mode[v as usize];
        if m != self.mode {
            self.mode = m;
            self.filter = f;
            self.rebuild_demod();
        } else {
            self.set_filter(f.0, f.1);
        }
        self.cwlive.restart();
        self.retune(false);
    }

    /// Switch one of the slot decoders (Q65 / PI4) on or off. The live
    /// CW box is always on.
    fn set_decoder(&mut self, kind: DecoderKind, on: bool) {
        let have = self.slots.iter().any(|(k, _)| *k == kind);
        if on && !have {
            if let Some(r) = slot_recorder(kind) {
                self.slots.push((kind, r));
                info!(?kind, "decoder on");
            }
        } else if !on && have {
            self.slots.retain(|(k, _)| *k != kind);
            info!(?kind, "decoder off");
        }
    }

    /// The station callsign: the one set in the web UI, else trxd.toml's.
    fn callsign(&self) -> String {
        self.settings.callsign.clone().unwrap_or_else(|| self.cfg.callsign.trim().to_ascii_uppercase())
    }

    /// The S-meter calibration table in force: the transverter's name, or the band.
    fn cal_band(&self) -> String {
        let f = self.rx_eff();
        match self.settings.transverter(f) {
            Some(t) => t.name.clone(),
            None => Band::containing(f).label().to_string(),
        }
    }

    /// Frequencies the LO must keep out of so the DC spike stays off the web
    /// scope: the view plus a guard, for spans up to [`DC_AVOID_SPAN_MAX`]
    /// while a browser is watching. `None`: anywhere will do.
    fn dc_keepout(&self) -> Option<(f64, f64)> {
        let watching = self.web.as_ref().is_some_and(|w| w.clients() > 0);
        // (Wide FM fills more than the narrow views; its channel has to stay
        // near the LO, so the spike stays where it falls.)
        if !watching || self.web_span > DC_AVOID_SPAN_MAX || self.datv_lo_offset().is_some() || self.mode == Mode::Wfm {
            return None;
        }
        let c = if self.web_center == 0.0 { self.rx_vfo() } else { self.web_center };
        let h = self.web_span / 2.0 + DC_GUARD_HZ;
        Some((c - h, c + h))
    }

    /// Sending DATV: how far below the DVB-S2 signal's centre the LO goes, so
    /// that the TX carrier leak sits just outside the signal where the stream
    /// has room for that, and the signal stays inside the TX passband.
    fn datv_lo_offset(&self) -> Option<f64> {
        // Receiving DVB-T2 (and not sending): the LO on the signal too. The
        // channel fills the FPGA resampler's passband; the usual offset
        // (100 kHz here) put its outer carriers past it.
        if self.tx_on.is_none() && self.datv_rx.as_ref().is_some_and(|r| r.t2_bw.is_some()) {
            return Some(0.0);
        }
        // Receiving DVB-S2 through the FPGA's DDC (and not sending): the LO
        // just below the signal, as for sending. Straddling the LO, a wide
        // signal meets its own mirror image: the AD936x's image rejection
        // falls off away from the LO (frequency-dependent I/Q mismatch that
        // quadrature tracking does not remove), below 1 GHz badly: 500 kS/s
        // at 436 MHz gave MER 6 dB with the LO in the middle, 15 dB 100 kHz
        // below, 27 dB with the signal on one side (docs/DATV-OTA.md).
        if let Some(half) = self.datv_rx_half() {
            return Some((half + 10e3).min(crate::dvbs2::fpga::FS_IN * 0.4 - half).max(0.0));
        }
        let d = self.datv.as_ref().filter(|_| matches!(self.tx_on, Some(TxSource::Datv(_))))?;
        if d.fpga.is_some() || d.t2.is_some() {
            // The FPGA's samples go to the DAC as they are: LO on the signal.
            return Some(0.0);
        }
        let half = d.sr * (1.0 + d.rolloff as f64) / 2.0;
        Some((half + 10e3).min(self.rate * 0.38 - half).max(0.0))
    }

    /// Receiving DVB-S2 through the FPGA's DDC and not sending: half the
    /// widest signal it may meet (the scan's fastest rate while it looks
    /// for one, and in automatic mode throughout, so the LO stays put).
    fn datv_rx_half(&self) -> Option<f64> {
        if self.tx_on.is_some() || !self.datv_mode {
            return None;
        }
        let widest = crate::dvbs2::scan::RATES.iter().cloned().fold(0.0, f64::max);
        let sr = if self.datv_scan.is_some() || (self.datv_auto && self.datv_rx.is_some()) {
            widest
        } else {
            self.datv_rx.as_ref().filter(|r| r.t2_bw.is_none() && r.through_fpga())?.sr
        };
        Some(sr * 1.35 / 2.0)
    }

    /// DATV reception can run now: nothing sent, or sent by the FPGA's
    /// DVB-S2 transmitter, or DVB-T2 with the FPGA's IFFT (the software
    /// modulator, or DVB-T2's OFDM on the A9, take its cores; a receiver
    /// beside them made the transmission stall).
    fn datv_rx_alongside_tx(&self) -> bool {
        // (TRXD_TX_ALONE=1: never, for tests)
        self.datv.as_ref().is_none_or(|d| {
            std::env::var_os("TRXD_TX_ALONE").is_none()
                && (d.fpga.is_some() || d.t2.as_ref().is_some_and(|t| t.fpga_ifft))
                && cross_band(self.rx_eff(), self.tx_eff())
        })
    }

    /// Why the receiver is not running while sending, for the page.
    fn datv_rx_paused(&self) -> Option<&'static str> {
        if self.datv.is_none() || self.datv_rx_req.is_none() || self.datv_rx.is_some() {
            return None;
        }
        Some(if !cross_band(self.rx_eff(), self.tx_eff()) { "same band" } else { "the sender needs the cores" })
    }

    /// Tell the browser why DATV did not start.
    fn datv_refuse(&self, client: u64, why: &str) {
        if let Some(w) = &self.web {
            w.send_json_to(client, &serde_json::json!({"type": "datv_error", "msg": why}));
        }
    }

    /// Start DATV for `client`: validate, build the modulator and mux, key.
    fn datv_start(&mut self, client: u64, sr: f64, rate: &str) {
        if sr == 0.0 && !rate.starts_with("T2-") {
            self.datv_refuse(client, "choose a symbol rate to send (Auto is for receiving)");
            return;
        }
        use crate::dvbs2::{ts::Mux, ts::Profile};
        use crate::dvbs2::fpga_tx::{LongMode, Transmitter};
        // The FPGA's samples go to the DAC untouched: through an inverting
        // transverter they would go out mirrored (only the software path
        // can conjugate them).
        let fpga_path = crate::dvbt2::tx::Mode::parse(rate).is_some() || LongMode::parse(rate).is_some();
        if fpga_path && self.settings.transverter(self.tx_eff()).is_some_and(|t| t.inverted) {
            warn!("DATV: the FPGA transmitter cannot send through an inverting transverter");
            self.datv_refuse(client, "DATV is generated in the FPGA and would go out mirrored through an inverting transverter");
            return;
        }
        if let Some(mode) = crate::dvbt2::tx::Mode::parse(rate) {
            // DVB-T2: modulated here, resampled to the DAC rate in the FPGA.
            let t2 = match crate::dvbt2::tx::T2Tx::start(mode.with_frequency(self.tx_eff()), self.tx_sink.clone(), self.block * 4, self.cfg.trx.t2_drive_db) {
                Ok(t2) => t2,
                Err(e) => {
                    warn!("DATV: DVB-T2: {e}");
                    self.datv_refuse(client, &e);
                    return;
                }
            };
            let ts_rate = mode.ts_rate();
            let profile = Profile::for_rate(ts_rate);
            let video_bps = profile.video_budget(ts_rate);
            self.datv = Some(Datv {
                sr: mode.bw_hz,
                profile,
                video_bps,
                fpga: None,
                t2: Some(t2),
                pending: Vec::new(),
                rate_label: rate.into(),
                pilots: true,
                rolloff: 0.0,
                ts_rate,
                mux: Mux::new(ts_rate, &self.callsign()).with_delivery(crate::dvbs2::ts::Delivery::T2 { freq_hz: self.tx_eff(), bw_hz: mode.bw_hz, plp_id: 0, t2_system_id: mode.p.t2_system_id }),
                last_media: Instant::now(),
            });
            self.key(TxSource::Datv(client));
            if self.tx_on != Some(TxSource::Datv(client)) {
                self.datv = None;
                return;
            }
            info!(bw = mode.bw_hz, mode = rate, ts_rate = ts_rate.round(), video_bps = video_bps.round(), ?profile, "DATV on (DVB-T2)");
            return;
        }
        if let Some(mode) = LongMode::parse(rate) {
            // Long frames, pilots: the FPGA encoder and interpolator.
            if !(8_000.0..=1_200_000.0).contains(&sr) {
                warn!(sr, "DATV: symbol rate out of range for the FPGA transmitter");
                self.datv_refuse(client, "symbol rate out of range for the FPGA transmitter");
                return;
            }
            let fx = match Transmitter::start(mode, sr, 0.35) {
                Ok(fx) => fx,
                Err(e) => {
                    warn!("DATV: {e}");
                    self.datv_refuse(client, &e);
                    return;
                }
            };
            let ts_rate = mode.ts_rate(sr);
            let profile = Profile::for_rate(ts_rate);
            let video_bps = profile.video_budget(ts_rate);
            self.datv = Some(Datv {
                sr,
                profile,
                video_bps,
                fpga: Some(fx),
                t2: None,
                pending: Vec::new(),
                rate_label: mode.label().into(),
                pilots: true,
                rolloff: 0.35,
                ts_rate,
                mux: Mux::new(ts_rate, &self.callsign()).with_delivery(crate::dvbs2::ts::Delivery::S2 { freq_hz: self.tx_eff(), symbol_rate: sr, rolloff: 0.35, modcod: mode.modcod() }),
                last_media: Instant::now(),
            });
            self.key(TxSource::Datv(client));
            if self.tx_on != Some(TxSource::Datv(client)) {
                self.datv = None;
                return;
            }
            info!(sr, mode = mode.label(), ts_rate = ts_rate.round(), video_bps = video_bps.round(), ?profile, "DATV on (FPGA)");
            return;
        }
        warn!(rate, "DATV: not a mode this transmitter has (DVB-S2 long frames or DVB-T2)");
        self.datv_refuse(client, "choose a DVB-S2 long-frame or DVB-T2 mode");
    }

    /// Start (or restart) the DVB-S2 receiver on the RX frequency: short
    /// frames at the stream rates (software) or anything the FPGA DDC takes;
    /// long frames (LDPC in the FPGA) through the DDC.
    fn datv_rx_start(&mut self, sr: f64, rate: &str) {
        use crate::dvbs2::{FrameSpec, ddc, fpga, fpga_tx::LongMode};
        self.datv_rx = None;
        self.datv_scan = None;
        self.datv_rx_stats = Default::default();
        // DVB-T2 (the symbol rate does not apply).
        if let Some(mode) = crate::dvbt2::tx::Mode::parse(rate) {
            self.datv_auto = false;
            if !fpga::available() {
                warn!(rate, "DVB-T2 receive needs the FPGA front end");
                return;
            }
            info!(rate, "DVB-T2 receive on");
            self.datv_rx = Some(crate::dvbs2::rx::RxThread::start_t2(mode, rate.to_string(), self.rx_eff() - self.center));
            // The LO onto the signal (datv_lo_offset).
            self.retune(true);
            return;
        }
        // Symbol rate 0: find rate and mode by themselves (blind scan).
        self.datv_auto = sr == 0.0;
        if self.datv_auto {
            self.datv_auto_note = "scanning".into();
            self.datv_scan = Some(crate::dvbs2::scan::Scanner::start(self.rx_eff() - self.center, self.datv_auto_sr));
            info!("DATV receive: automatic (scanning the standard symbol rates)");
            // The LO onto the signal (datv_lo_offset).
            self.retune(false);
            return;
        }
        let ddc = fpga::available() && ddc::symbol_rate_ok(fpga::FS_IN, sr);
        let center = self.rx_eff() - self.center;
        if let Some(mode) = LongMode::parse(rate) {
            if !ddc {
                warn!(sr, rate, "DATV receive: long frames need the FPGA front end at a rate it takes");
                return;
            }
            info!(sr, rate, "DATV receive on");
            self.datv_rx = Some(crate::dvbs2::rx::RxThread::start_spec(FrameSpec::long(mode), mode.label().to_string(), sr, center));
            self.retune(false);
            return;
        }
        warn!(sr, rate, "DATV receive: not a mode this receiver has (DVB-S2 long frames or DVB-T2)");
    }

    /// Out of DATV mode: receiver off, a DATV transmission ends.
    fn datv_leave(&mut self) {
        if !self.datv_mode {
            return;
        }
        if matches!(self.tx_on, Some(TxSource::Datv(_))) {
            self.unkey();
        }
        self.datv_mode = false;
        self.datv_rx = None;
        self.datv_rx_req = None;
        self.datv_scan = None;
        self.datv_auto = false;
        self.datv_rx_stats = Default::default();
        // The slot decoders stood still through DATV: start their slots afresh.
        for (_, rec) in &mut self.slots {
            rec.reset();
        }
        self.retune(false);
        info!("DATV mode off");
    }

    /// A receiver that holds lock while nothing decodes starts again: its
    /// frame headers keep matching but the pilots do not (their Es/N0 far
    /// below the data's decision-directed figure). A safety net: the case
    /// seen (8PSK 3/4 at 33 kS/s, the carrier more than rs / 90 off, an
    /// alias off after acquisition) is fixed in the receiver. A restart also
    /// resets the FPGA's timing recovery.
    fn datv_rx_watchdog(&mut self) {
        let s = &self.datv_rx_stats;
        // (DVB-T2 counts FEC blocks and has no such figures.)
        let s2 = self.datv_rx.as_ref().is_some_and(|r| r.t2_bw.is_none());
        if !s2 || self.datv_auto || s.frames < self.datv_rx_watch.1 {
            self.datv_rx_watch = (0, 0, Instant::now());
            return;
        }
        let good = s.frames.saturating_sub(s.frames_bad);
        if !s.locked || good != self.datv_rx_watch.0 || self.datv_rx_watch.1 == 0 {
            self.datv_rx_watch = (good, s.frames.max(1), Instant::now());
            return;
        }
        if s.frames >= self.datv_rx_watch.1 + 8 && self.datv_rx_watch.2.elapsed() >= Duration::from_secs(3) && s.esn0_db < s.data_esn0_db - 3.0 {
            warn!(frames = s.frames - self.datv_rx_watch.1, esn0 = s.esn0_db, mer = s.data_esn0_db, "DATV receive: locked but nothing decodes; starting again");
            if let Some((sr, rate)) = self.datv_rx_req.clone() {
                self.datv_rx = None;
                self.datv_rx_start(sr, &rate);
            }
            self.datv_rx_watch = (0, 0, Instant::now());
        }
    }

    /// Automatic receive: a scan that found something starts the receiver
    /// for it; a receiver that decodes nothing for a while goes back to
    /// scanning (the other station may have changed its mode).
    fn datv_auto_step(&mut self) {
        use crate::dvbs2::scan::State;
        if !self.datv_auto {
            return;
        }
        if let Some(sc) = &self.datv_scan {
            match sc.state() {
                State::Scanning(sr) => self.datv_auto_note = format!("scanning {:.0} kS/s", sr / 1e3),
                State::Failed(e) => {
                    warn!("DATV scan: {e}");
                    self.datv_auto_note = format!("cannot scan: {e}");
                    self.datv_scan = None;
                    self.datv_auto = false;
                }
                State::Found { sr, pls, offset_hz } => {
                    self.datv_scan = None;
                    self.datv_auto_sr = Some(sr);
                    match pls.long_mode() {
                        Some(mode) => {
                            let label = format!("{} @ {:.0} kS/s (auto)", mode.label(), sr / 1e3);
                            info!(sr, mode = mode.label(), offset_hz = offset_hz.round(), "DATV receive: automatic, receiving");
                            self.datv_auto_note = format!("found {} at {:.0} kS/s", pls.describe(), sr / 1e3);
                            let center = self.rx_eff() - self.center;
                            self.datv_rx = Some(crate::dvbs2::rx::RxThread::start_spec(crate::dvbs2::FrameSpec::long(mode), label, sr, center));
                            self.datv_rx_stats = Default::default();
                            self.datv_auto_good = (0, Instant::now());
                        }
                        None => {
                            // Something DVB-S2 this receiver cannot decode:
                            // say what, look again later.
                            self.datv_auto_note = format!("{} at {:.0} kS/s: not receivable here (QPSK 1/2, 3/4, 8PSK 3/4 long with pilots are)", pls.describe(), sr / 1e3);
                            self.datv_auto_good = (0, Instant::now());
                        }
                    }
                }
            }
            return;
        }
        // Receiving (or showing an unsupported mode): back to scanning once
        // no frame has decoded for 5 s (or two of the rate's frames).
        let good = self.datv_rx_stats.frames.saturating_sub(self.datv_rx_stats.frames_bad);
        if good > self.datv_auto_good.0 {
            self.datv_auto_good = (good, Instant::now());
        }
        let patience = Duration::from_secs_f64((2.5 * 33_282.0 / self.datv_auto_sr.unwrap_or(250e3)).max(5.0));
        if self.datv_auto_good.1.elapsed() > patience {
            info!("DATV receive: automatic, signal lost; scanning again");
            self.datv_rx = None;
            self.datv_rx_stats = Default::default();
            self.datv_auto_note = "scanning".into();
            self.datv_scan = Some(crate::dvbs2::scan::Scanner::start(self.rx_eff() - self.center, self.datv_auto_sr));
        }
    }

    fn datv_json(&self) -> serde_json::Value {
        match &self.datv {
            Some(d) => {
                serde_json::json!({"sr": d.sr, "rate": d.rate_label, "pilots": d.pilots, "ts_rate": d.ts_rate.round(), "fpga": d.fpga.is_some() || d.t2.is_some(),
                    "video_bps": d.video_bps.round(), "audio_bps": d.profile.audio_bps, "fps": d.profile.fps, "key_every_s": d.profile.key_every_s,
                    "width": d.profile.width, "height": d.profile.height})
            }
            None => serde_json::Value::Null,
        }
    }

    /// Where the web scope is centred for this row: it stays put while the
    /// VFO is inside the view, and recentres on the VFO when it leaves (or
    /// when the rows could not cover the view any more).
    fn web_view_center(&mut self) -> f64 {
        let vfo = self.rx_vfo();
        let half = self.web_span / 2.0;
        let margin = self.web_span * 0.05;
        let c = self.web_center;
        let covered = if self.web_span <= NARROW_SPAN_MAX {
            (c - vfo).abs() + half <= self.chan_rate * 0.45
        } else if self.web_span <= STREAM_SPAN_MAX || self.maia.is_none() {
            (c - self.center).abs() + half <= self.rate / 2.0
        } else {
            true
        };
        if self.scope_center || c == 0.0 || (vfo - c).abs() > half - margin || !covered {
            self.web_center = vfo;
        }
        self.web_center
    }

    // ---- Transmit control ----

    fn key(&mut self, source: TxSource) {
        if let Some(now) = self.tx_on {
            // One source at a time: another one taking over mid-transmission
            // (rigctl T1 during DATV) would feed the wrong path.
            if now != source {
                warn!(?now, ?source, "transmit request refused: already transmitting");
            }
            return;
        }
        if let Err(e) = self.tx_check(self.tx_eff(), self.tx_half_bw()) {
            warn!("transmit refused: {e}");
            return;
        }
        self.tx_on = Some(source);
        self.tx_since = Some(Instant::now());
        self.speech.reset();
        // Nothing more of the other station while we send: settle its tail.
        self.cwlive.flush();
        if let (TxSource::Web(client) | TxSource::Datv(client), Some(w)) = (source, &self.web) {
            w.set_mic_owner(Some(client));
            self.mic_started = false;
            self.mic_last = Instant::now();
        }
        self.mic_drain_until = None;
        self.tx_fifo.clear();
        self.tx_out.clear();
        self.pace.rekey();
        self.tci_last_audio = Instant::now();
        // Every LO and filter move before RF goes on (DATV: the LO onto
        // the signal). The backend sequences the PTT line and the TX LO.
        self.retune(matches!(source, TxSource::Datv(_)));
        self.datv_tx_bw();
        if let Err(e) = self.radio.set_tx_rf(true) {
            warn!("TX on: {e}");
        }
        self.rf_live = true;
        info!(freq = self.tx_vfo(), mode = mode_name(self.mode), ?source, "TX");
    }

    // ---- Level meter (power.rs, calib.rs, docs/DBM.md) ----

    /// Feed the level meters with this block; a reading when one is due.
    fn feed_meter(&mut self, iq: &[Complex32]) {
        for z in iq {
            let m = z.re.abs().max(z.im.abs());
            if m > self.stream_peak || m.is_nan() {
                self.stream_peak = if m.is_nan() { 1.0 } else { m };
            }
        }
        let at = (self.rx_eff(), self.center, self.rx_port());
        if at != self.meter_at {
            self.meter_at = at;
            self.chan_meter.reset();
            self.stream_meter.reset();
            self.chan_spec = None;
            self.stream_spec = None;
        }
        let mut fresh = false;
        // (DATV mode with a wide scope leaves the channel silent.)
        if self.datv_mode && self.web_span > NARROW_SPAN_MAX {
            self.chan_spec = None;
        } else if let Some(sp) = self.chan_meter.feed(&self.chan) {
            self.chan_spec = Some(sp);
            fresh = true;
        }
        if self.datv_rx.is_some() || Instant::now() < self.stream_meter_until {
            if let Some(sp) = self.stream_meter.feed(iq) {
                self.stream_spec = Some(sp);
                fresh = true;
            }
        } else {
            self.stream_spec = None;
        }
        if fresh {
            self.update_meter();
        }
    }

    /// The band the meter measures, offsets from the dial (Hz): the
    /// receive filter, or a DATV signal's whole channel.
    fn meter_band(&self) -> (f64, f64) {
        if let Some(r) = &self.datv_rx {
            let half = match r.t2_bw {
                Some(bw) => bw / 2.0,
                None => r.sr * 1.35 / 2.0,
            };
            return (-half, half);
        }
        let (a, b) = (self.filter.0 as f64, self.filter.1 as f64);
        (a.min(b), a.max(b))
    }

    /// Raw levels of `[lo, hi]` (offsets from the dial) at the channel, the
    /// stream and the Maia spectrometer, each where it covers the band, in
    /// that source's own dBFS.
    fn raw_levels(&mut self, lo: f64, hi: f64) -> [Option<RawLevel>; 3] {
        use crate::power::db;
        let level = |sp: &crate::power::Spectrum, a: f64, b: f64| {
            sp.covers(a, b).then(|| {
                let span = (b - a).max(4_000.0);
                RawLevel {
                    dbfs: db(sp.band_power(a, b)),
                    peak_dbfs: db(sp.peak_power(a, b)),
                    noise_dbfs_hz: sp.noise_density(a, b, span).map(db),
                    seq: sp.seq,
                }
            })
        };
        let chan = self.chan_spec.as_ref().and_then(|sp| level(sp, lo, hi));
        let o = self.rx_eff() - self.center;
        let stream = self.stream_spec.as_ref().and_then(|sp| level(sp, o + lo, o + hi));
        // Wide bands: the spectrometer, its rows taken here unless the
        // scope is reading them (then its last one).
        let mut maia = None;
        if chan.is_none() && stream.is_none() {
            let scope_reads = self.web_span > STREAM_SPAN_MAX && self.web.as_ref().is_some_and(|w| w.clients() > 0);
            if !scope_reads {
                if let Some(r) = self.maia.as_ref().and_then(|m| m.try_iter().last()) {
                    self.maia_last = Some(r);
                }
            }
            if let Some(row) = &self.maia_last {
                // The row is the AD936x's own spectrum, centred on its LO.
                let inv = self.xvtr.as_ref().is_some_and(|t| t.inverted);
                let (a, b) = if inv { (-(o + hi), -(o + lo)) } else { (o + lo, o + hi) };
                maia = crate::power::maia_band(row, self.cfg.radio.adc_rate as f64, a, b).map(|(p, d)| RawLevel {
                    dbfs: db(p),
                    peak_dbfs: db(p),
                    noise_dbfs_hz: d.map(db),
                    seq: 0,
                });
            }
        }
        [chan, stream, maia]
    }

    /// The RX socket pair in use (1 or 2).
    fn rx_port(&self) -> u8 {
        self.radio.port().unwrap_or(1).clamp(1, 2)
    }

    /// The AD936x's own frequency for the dial (the IF through a transverter).
    fn hw_freq(&self) -> f64 {
        let dial = self.rx_eff();
        self.xvtr.as_ref().map_or(dial, |t| t.to_if(dial))
    }

    /// A new reading from the latest spectra.
    fn update_meter(&mut self) {
        use crate::calib::Quality;
        let (lo, hi) = self.meter_band();
        let lv = self.raw_levels(lo, hi);
        let table = self.calib[self.rx_port() as usize - 1].as_ref();
        let (sdb, mdb) = table.map_or((0.0, 0.0), |c| (c.stream_db, c.maia_db));
        let pick = [(0, 0.0, "channel"), (1, sdb, "stream"), (2, mdb, "maia")]
            .into_iter()
            .find_map(|(i, add, name)| lv[i].as_ref().map(|r| (r.dbfs + add, r.noise_dbfs_hz.map(|d| d + add), name)));
        let Some((dbfs, dens, src)) = pick else { return };
        let g = self.hw_gain_db;
        let temp = self.temps.lock().ok().and_then(|t| t.ad936x);
        let (off, quality) = match table {
            Some(c) => c.offset(g, self.hw_freq(), temp, self.xvtr.as_ref().map(|t| t.name.as_str())),
            None => {
                let band = self.cal_band();
                if self.settings.smeter.get(&band).is_some_and(|v| !v.is_empty()) {
                    (self.settings.dbm(&band, self.reading_db) - dbfs, Quality::Legacy)
                } else {
                    (-g + crate::calib::K_DEFAULT_DB, Quality::None)
                }
            }
        };
        let bw = (hi - lo).max(1.0);
        let noise = dens.is_some_and(|d| 10f64.powf(dbfs / 10.0) < 2.0 * 10f64.powf(d / 10.0) * bw);
        // The scope shows the source its span reads (scope.rs: a full-scale
        // tone at 0 dBFS, as here).
        let scope_off = table.map(|_| {
            if self.web_span <= NARROW_SPAN_MAX {
                off
            } else if self.web_span > STREAM_SPAN_MAX && self.maia.is_some() {
                off + mdb
            } else {
                off + sdb
            }
        });
        self.meter = Meter {
            dbm: dbfs + off,
            dbm_hz: dens.map(|d| d + off),
            dbfs,
            quality,
            clip: self.stream_peak >= crate::power::CLIP,
            noise,
            src,
            scope_off,
        };
        self.stream_peak = 0.0;
    }

    /// The meter's raw measurement of `[lo_hz, hi_hz]` (absolute, on the
    /// air; else the meter's band), for the calibration tool. Keeps the
    /// stream meter running for 10 s.
    fn meter_raw(&mut self, lo_hz: Option<f64>, hi_hz: Option<f64>) -> serde_json::Value {
        self.stream_meter_until = Instant::now() + Duration::from_secs(10);
        let dial = self.rx_eff();
        let (lo, hi) = match (lo_hz, hi_hz) {
            (Some(a), Some(b)) if b > a && b - a <= 4e6 => (a - dial, b - dial),
            _ => self.meter_band(),
        };
        let lv = self.raw_levels(lo, hi);
        let j = |r: &Option<RawLevel>| {
            r.as_ref().map(|r| serde_json::json!({"dbfs": r.dbfs, "peak_dbfs": r.peak_dbfs, "noise_dbfs_hz": r.noise_dbfs_hz, "seq": r.seq}))
        };
        serde_json::json!({"type": "meter_raw", "port": self.rx_port(), "dial_hz": dial, "center_hz": self.center,
            "hw_freq_hz": self.hw_freq(), "lo_hz": dial + lo, "hi_hz": dial + hi,
            "hw_gain_db": self.hw_gain_db, "gain_mode": format!("{:?}", self.rx_gain_mode),
            "temp_c": self.temps.lock().ok().and_then(|t| t.ad936x), "clip": self.meter.clip,
            "chan": j(&lv[0]), "stream": j(&lv[1]), "maia": j(&lv[2]),
            "dbm": self.meter.dbm, "cal": self.meter.quality, "xvtr": self.xvtr.as_ref().map(|t| t.name.clone()),
            "datv": self.datv_mode, "tx": self.tx_on.is_some()})
    }

    /// Both sockets' tables and per-band status, for SET and the tool.
    fn calib_json(&self) -> serde_json::Value {
        let ports: Vec<serde_json::Value> = (1..=2u8)
            .map(|p| {
                let c = &self.calib[p as usize - 1];
                serde_json::json!({"port": p, "table": c, "status": c.as_ref().map(|c| c.status()),
                    "note": c.as_ref().map(|c| c.note.clone())})
            })
            .collect();
        serde_json::json!({"type": "calib", "port": self.rx_port(), "ports": ports,
            "bands": crate::calib::BANDS.iter().map(|b| b.0).collect::<Vec<_>>(),
            "legacy_bands": self.settings.smeter.keys().collect::<Vec<_>>()})
    }

    /// MUTE AT TX and transmitting (or just stopped): the receiver is muted.
    fn rx_quiet(&self) -> bool {
        self.settings.mute_at_tx
            && (self.tx_on.is_some() || self.rx_quiet_until.is_some_and(|t| Instant::now() < t))
    }

    fn unkey(&mut self) {
        if self.tx_on.take().is_none() {
            return;
        }
        self.mic_drain_until = None;
        self.tx_since = None;
        self.rx_quiet_until = Some(Instant::now() + RX_RECOVER);
        let datv = self.datv.take();
        if datv.is_none() {
            // The DAC ends on silence rather than holding the last sample.
            let _ = self.tx_sink.try_send(crate::stream::TxBlock::Iq(vec![Complex32::default(); self.block]));
        }
        if datv.is_some() {
            info!("DATV off");
            if let Some((sr, rate)) = self.datv_rx_req.clone().filter(|_| self.datv_mode && self.datv_rx.is_none()) {
                self.datv_rx_start(sr, &rate);
            }
        }
        if self.tx_bw != self.cfg.radio.rf_bandwidth {
            // Back from DVB-T2's width.
            let _ = self.radio.set_tx_bandwidth(self.cfg.radio.rf_bandwidth);
            self.tx_bw = self.cfg.radio.rf_bandwidth;
        }
        self.keyer.abort();
        if let Some(w) = &self.web {
            w.set_mic_owner(None);
        }
        if let Some(t) = &mut self.tci {
            t.drain_tx_audio();
            t.deny_tx();
        }
        // Let the queued RF (a few blocks) drain before RF goes off; the
        // backend then waits ptt_delay before the PTT line drops.
        let queued = self.tx_sink.len() as f64 * self.block as f64 / self.rate;
        std::thread::sleep(Duration::from_secs_f64(0.03 + queued.min(0.5)));
        if let Err(e) = self.radio.set_tx_rf(false) {
            warn!("TX off: {e}");
        }
        self.rf_live = false;
        info!("RX");
        // Back to where receiving wants the LO (after a cross-band split).
        self.retune(false);
    }

    // ---- Command handling (TCI / rigctld / MQTT all end up here) ----

    fn apply(&mut self, cmd: Command) {
        match cmd {
            Command::SetVfo { vfo, hz } => self.set_vfo(vfo, hz),
            Command::SelectVfo(v) => self.select_vfo(v),
            Command::SwapVfos => {
                std::mem::swap(&mut self.vfo_a, &mut self.vfo_b);
                let (a, b) = (self.vfo_mode[0], self.vfo_mode[1]);
                self.vfo_mode = [b, a];
                let (m, f) = self.vfo_mode[self.active as usize];
                self.mode = m;
                self.filter = f;
                self.rebuild_demod();
                self.cwlive.restart();
                self.retune(false);
            }
            Command::CopyAtoB => {
                self.vfo_b = self.vfo_a;
                self.vfo_mode[1] = self.vfo_mode[0];
            }
            Command::SetSplit(s) => {
                self.split = s;
                self.retune(false);
            }
            Command::SetMode { mode, .. } => self.set_mode(mode),
            Command::SetFilter { lo, hi, .. } => self.set_filter(lo, hi),
            Command::SetPtt(on) => {
                if on {
                    let src = if self.mode == Mode::Cw { TxSource::CwPtt } else { TxSource::Ptt };
                    self.key(src);
                } else {
                    self.unkey();
                }
            }
            Command::SetTune(on) => {
                if on {
                    self.key(TxSource::Tune)
                } else {
                    self.unkey()
                }
            }
            Command::SetTxDrive(d) => self.drive = d.clamp(0.0, 1.0),
            Command::SetVolume { v, .. } => self.volume = v.clamp(0.0, 1.0),
            Command::SetMute { muted, .. } => self.muted = muted,
            Command::SetAgc { agc, .. } => {
                self.agc_mode = agc;
                self.agc.set_mode(agc);
            }
            other => debug!(?other, "command not supported here"),
        }
    }

    /// A named setting from the web UI (drive, txatt, cw_wpm, rxgain), its
    /// value as text.
    fn apply_setting(&mut self, name: &str, payload: &str) {
        let num = payload.parse::<f64>();
        match (name, num) {
            ("cw_wpm", Ok(w)) => self.keyer.set_wpm(w as f32),
            ("drive", Ok(p)) => self.drive = (p / 100.0).clamp(0.0, 1.0) as f32,
            ("txatt", Ok(db)) => {
                self.tx_att_db = db;
                if let Err(e) = self.radio.set_tx_attenuation(db) {
                    warn!("{e}");
                }
            }
            ("rxgain", _) => {
                let (mode, db) = match payload {
                    "auto" | "slow" => (GainMode::SlowAttack, self.rx_gain_db),
                    "fast" => (GainMode::FastAttack, self.rx_gain_db),
                    p => match p.parse::<f64>() {
                        Ok(db) => (GainMode::Manual, db),
                        Err(_) => return warn!("rxgain '{p}'"),
                    },
                };
                self.rx_gain_mode = mode;
                self.rx_gain_db = db;
                if let Err(e) = self.radio.set_rx_gain(mode, db) {
                    warn!("{e}");
                }
            }
            _ => warn!("setting '{name}' = '{payload}' not understood"),
        }
    }

    fn send_cw(&mut self, text: &str) {
        if self.mode != Mode::Cw {
            self.set_mode(Mode::Cw);
        }
        self.keyer.send(text);
        self.cw_idle_since = None;
        // Under a held PTT the text just joins the queue.
        if self.tx_on != Some(TxSource::CwPtt) {
            self.key(TxSource::Cw);
        }
    }

    fn poll_controls(&mut self) {
        let mut cmds = Vec::new();
        let mut keys = Vec::new();
        if let Some(t) = &self.tci {
            for r in t.poll() {
                match r {
                    TciRequest::Cmd(c) => cmds.push(c),
                    TciRequest::Key(on) => keys.push(on),
                    TciRequest::Clients(n) => info!(clients = n, "TCI"),
                }
            }
        }
        if let Some(r) = &self.rig {
            for q in r.poll() {
                match q {
                    RigRequest::Cmd(c) => cmds.push(c),
                    RigRequest::Clients(n) => info!(clients = n, "rigctld"),
                }
            }
        }
        for c in cmds {
            self.apply(c);
        }
        for on in keys {
            if on {
                self.key(TxSource::Tci);
            } else {
                self.unkey();
            }
        }
    }

    // ---- State out ----

    fn publish_state(&mut self) {
        let can_tx = self.cfg.trx.allow_tx;
        let range = (self.cfg.radio.freq_min_hz, self.cfg.radio.freq_max_hz);
        let strength = self.meter.dbm.round() as i32;
        let rig = RigState {
            vfo_a_hz: self.vfo_a,
            vfo_b_hz: self.vfo_b,
            active_vfo: self.active,
            split: self.split,
            mode: self.mode,
            filter_lo: self.filter.0,
            filter_hi: self.filter.1,
            ptt: self.tx_on.is_some(),
            tune: self.tx_on == Some(TxSource::Tune),
            drive: self.drive,
            volume: self.volume,
            band: Band::containing(self.rx_vfo()),
            muted: self.muted,
            strength_dbm: strength,
            can_tx,
            rx_ranges: vec![range],
            tx_ranges: if can_tx { vec![range] } else { Vec::new() },
            ..RigState::default()
        };
        if self.last_rig.as_ref() != Some(&rig) {
            if let Some(r) = &self.rig {
                r.publish_state(rig.clone());
            }
            self.last_rig = Some(rig);
        }

        let iq_rate = self.iq_tap.as_ref().map_or(0, |(r, _)| *r);
        let tci = TciStateSnapshot {
            vfo_a_hz: self.vfo_a,
            vfo_b_hz: self.vfo_b,
            center_hz: self.center,
            if_span_hz: self.rate * 0.4,
            mode: self.mode,
            split: self.split,
            ptt: self.tx_on.is_some(),
            tune: self.tx_on == Some(TxSource::Tune),
            drive_pct: (self.drive * 100.0).round() as u32,
            tune_drive_pct: (self.drive * 100.0).round() as u32,
            muted: self.muted,
            volume_db: TciStateSnapshot::volume_db_from(self.volume),
            iq_rate: if iq_rate == 0 { 48_000 } else { iq_rate },
            vfo_lo_hz: range.0,
            vfo_hi_hz: range.1,
            can_tx,
        };
        if self.last_tci.as_ref() != Some(&tci) {
            if let Some(t) = &self.tci {
                t.broadcast_state(tci.clone());
            }
            self.last_tci = Some(tci);
        }
    }

    // ---- The per-block work ----

    fn receive(&mut self, b: &RxBlock) {
        // Through an inverting transverter the IF spectrum is mirrored.
        let inverted = self.xvtr.as_ref().is_some_and(|t| t.inverted);
        let mirrored: Vec<Complex32>;
        let iq: &[Complex32] = if inverted {
            mirrored = b.iq.iter().map(|z| z.conj()).collect();
            &mirrored
        } else {
            &b.iq
        };
        let mut mark = Instant::now();
        // TCI wideband IQ, at whichever rate the clients asked for.
        let want = self.tci.as_ref().and_then(|t| t.wants_iq()).filter(|r| (*r as f64) <= self.rate);
        match (want, &self.iq_tap) {
            (Some(r), Some((cur, _))) if *cur == r => {}
            (Some(r), _) => self.iq_tap = Some((r, Ddc::new(self.rate, r as f64))),
            (None, Some(_)) => self.iq_tap = None,
            (None, None) => {}
        }
        if let Some((r, ddc)) = &mut self.iq_tap {
            self.iq_buf.clear();
            ddc.process(iq, &mut self.iq_buf);
            if let Some(t) = &self.tci {
                t.on_rx_iq(sdroxide_dsp::as_interleaved(&self.iq_buf), *r);
            }
        }

        if let Some(r) = &self.datv_rx {
            r.set_center(self.rx_eff() - self.center);
        }
        // DVB-T2 reception: the RX filter opens to 1.3 x the channel (its
        // default, about 1 MHz, cut the outer carriers by up to 15 dB and
        // left no P1 to find), and closes again after.
        // DVB-S2 beside the LO (datv_lo_offset): open it over both.
        let s2 = self.datv_rx_half().zip(self.datv_lo_offset()).map(|(half, off)| 2.0 * (half + off));
        let want = self.datv_rx.as_ref().and_then(|r| r.t2_bw).or(s2).map_or(self.cfg.radio.rf_bandwidth, |bw| {
            self.cfg.radio.rf_bandwidth.max((bw * 1.3) as u32)
        });
        if want != self.rx_bw {
            match self.radio.set_rx_bandwidth(want) {
                Ok(()) => info!(hz = want, "RX bandwidth"),
                Err(e) => warn!("RX bandwidth: {e}"),
            }
            self.rx_bw = want;
        }
        if let Some(sc) = &self.datv_scan {
            sc.set_center(self.rx_eff() - self.center);
        }
        self.lap(0, &mut mark);
        // The channel. DATV mode has no audio to make and its own receiver
        // has the IQ: the channel DDC (3.072 MS/s to 48 kHz, a fifth of an
        // A9 core) runs only if the narrow waterfall shows it; otherwise
        // silence of the same length keeps the rest in step.
        self.chan.clear();
        if self.datv_mode && self.web_span > NARROW_SPAN_MAX {
            self.chan_frac += iq.len() as f64 * CH_RATE / self.rate;
            let n = self.chan_frac.floor();
            self.chan_frac -= n;
            self.chan.resize(n as usize, Complex32::default());
        } else if let Some(fe) = &mut self.chan_fpga {
            // (lapped: the samples lost are lost; the ring goes on)
            fe.read_channel(&mut self.chan_words, &mut self.chan);
            if inverted {
                for z in &mut self.chan {
                    *z = z.conj();
                }
            }
        } else {
            self.ddc.process(iq, &mut self.chan);
        }
        self.lap(1, &mut mark);
        if let Some(nb) = &mut self.nb {
            nb.process(&mut self.chan);
        }
        self.audio.clear();
        if self.datv_mode {
            // DATV mode: nothing to demodulate (the DVB-S2 receiver has the IQ);
            // silence of the same length keeps the rest of the chain in step.
            self.audio.resize(if self.narrow { self.chan.len() / 4 } else { self.chan.len() }, 0.0);
        } else if self.narrow {
            self.chan12.clear();
            self.dec4.process(&self.chan, &mut self.chan12);
            self.demod.process(&self.chan12, &mut self.audio);
        } else {
            self.demod.process(&self.chan, &mut self.audio);
        }
        self.s_dbfs = self.demod.power_dbfs();
        if self.mode == Mode::Wfm {
            self.rds_poll();
        }
        // Gain-compensated level for the S-meter, smoothed over ~0.3 s.
        // The AD936x AGC moves the gain on its own (read off the sample path).
        self.hw_gain_db = match &self.hw_gain {
            Some(g) => f64::from_bits(g.load(std::sync::atomic::Ordering::Relaxed)),
            None => self.rx_gain_db,
        };
        // MUTE AT TX: our own signal is not received; the S-meter and the
        // AGC hold what they had, so the other station comes back at once.
        let quiet = self.rx_quiet();
        if quiet {
            self.audio.fill(0.0);
        } else {
            let reading = self.s_dbfs as f64 - self.hw_gain_db;
            self.reading_db += (reading - self.reading_db) * 0.03;
            self.feed_meter(iq);
        }
        if let Some(n) = self.notch.as_mut().filter(|_| !self.datv_mode) {
            n.process(&mut self.audio);
        }
        if let Some(n) = self.nr.as_mut().filter(|_| !self.datv_mode) {
            n.process(&mut self.audio);
        }
        if self.mode == Mode::Cw && self.tx_on.is_none() && !self.datv_mode {
            self.cw_audio.clear();
            self.cw_audio.extend_from_slice(&self.audio);
            self.cw_agc.process(&mut self.cw_audio);
            self.cwlive.set_band(self.filter.0, self.filter.1);
            self.cwlive.audio(&self.cw_audio);
        }
        if !quiet && !self.datv_mode {
            self.agc.process(&mut self.audio);
        }
        // Squelch on the channel power, 2 dB of hysteresis.
        self.squelch_open = match self.squelch_db {
            None => true,
            Some(t) if self.squelch_open => self.s_dbfs >= t - 2.0,
            Some(t) => self.s_dbfs >= t,
        };

        self.lap(2, &mut mark);
        // Decoders (and the web UI) get the AGC'd audio, before volume and mute.
        let web_clients = self.web.as_ref().map_or(0, |w| w.clients());
        self.audio12.clear();
        if self.narrow {
            self.audio12.extend_from_slice(&self.audio);
        } else {
            self.dec12.process(&self.audio, &mut self.audio12);
        }
        self.lap(3, &mut mark);
        // (The live CW copy is fed above, before the listening AGC, in CW mode
        // only as on the IC-705: its speed fit costs a third of a Cortex-A9
        // core and has nothing to read in the other modes.)
        if web_clients > 0 {
            if self.mode == Mode::Wfm && !self.datv_mode {
                // Wide FM: 24 kHz to the page (the 12 kHz audio still feeds
                // the meter, TCI and the decoders).
                let n = self.web_audio24.len();
                self.dec24.process(&self.audio, &mut self.web_audio24);
                if !self.squelch_open {
                    self.web_audio24[n..].fill(0.0);
                }
                if self.web_audio24.len() >= 960 {
                    if let Some(w) = &self.web {
                        w.send_audio24(&self.web_audio24);
                    }
                    self.web_audio24.clear();
                }
            } else if self.squelch_open {
                self.web_audio.extend_from_slice(&self.audio12);
            } else {
                self.web_audio.resize(self.web_audio.len() + self.audio12.len(), 0.0);
            }
            if self.web_audio.len() >= 480 {
                if let Some(w) = self.web.as_ref().filter(|_| !self.datv_mode) {
                    w.send_audio(&self.web_audio);
                }
                self.web_audio.clear();
            }
            // The channel is centred where the receiver listens (RIT included).
            let vfo = self.rx_eff();
            let view = self.web_view_center();
            let span = self.web_span;
            // The view moved (or a browser came): take the LO out of it.
            if let Some((a, b)) = self.dc_keepout() {
                if self.center >= a && self.center <= b {
                    self.retune(false);
                }
            }
            let lo = self.center;
            let row = if span <= NARROW_SPAN_MAX {
                self.scope_narrow
                    .process(&self.chan)
                    .map(|r| scope::render(&r, vfo, self.chan_rate, view, span))
            } else if let (true, Some(m)) = (span > STREAM_SPAN_MAX, &self.maia) {
                let adc = self.cfg.radio.adc_rate as f64;
                let got = m.try_iter().last();
                // (The level meter measures wide bands on it too.)
                if let Some(r) = &got {
                    self.maia_last = Some(r.clone());
                }
                got.map(|mut r| {
                    if inverted {
                        r.reverse();
                    }
                    let mut cols = scope::render(&r, lo, adc, view, span);
                    scope::blank_dc(&mut cols, lo, view, span, 2.5 * adc / crate::maia::BINS as f64);
                    cols
                })
            } else {
                let rate = self.rate;
                self.scope_wide.process(iq).map(|r| {
                    let mut cols = scope::render(&r, lo, rate, view, span);
                    scope::blank_dc(&mut cols, lo, view, span, 1_000.0);
                    cols
                })
            };
            if let (Some(cols), Some(w)) = (row, &self.web) {
                w.send_spectrum(view, span, &cols);
            }
        }
        self.lap(4, &mut mark);
        // Q65 / PI4 listen in DATA mode only.
        if !self.slots.is_empty() && self.mode == Mode::Digu && !self.datv_mode {
            let dial = self.rx_eff();
            for (kind, rec) in &mut self.slots {
                for slot in rec.push(b.t0, &self.audio12) {
                    let job = match kind {
                        DecoderKind::Q65 => Job::Q65 {
                            audio: slot.audio,
                            letter: Q65Letter::parse(&self.cfg.trx.q65_submode).unwrap_or(Q65Letter::D),
                            slot_utc: slot.utc,
                            dial_hz: dial,
                            range: (200.0, 3_000.0),
                        },
                        DecoderKind::Pi4 => Job::Pi4 {
                            audio: slot.audio,
                            boundary: slot.boundary,
                            slot_utc: slot.utc,
                            dial_hz: dial,
                        },
                        DecoderKind::Cw => continue,
                    };
                    // Nobody transmits into their own receive slot.
                    if self.tx_on.is_none() {
                        self.decoder.submit(job);
                    }
                }
            }
        }

        self.lap(5, &mut mark);
        if let Some(t) = &self.tci {
            if t.wants_audio() {
                let g = if self.muted || !self.squelch_open { 0.0 } else { self.volume * 2.0 };
                let out: Vec<f32> = if self.narrow {
                    // x4 to TCI's 48 kHz: zero-stuff and low-pass (4x makes up the energy).
                    let stuffed: Vec<f32> = self.audio.iter().flat_map(|&a| [a * g * 4.0, 0.0, 0.0, 0.0]).collect();
                    let mut up = Vec::with_capacity(stuffed.len());
                    self.tci_up.process(&stuffed, &mut up);
                    up
                } else {
                    self.audio.iter().map(|s| s * g).collect()
                };
                t.on_rx_audio(&out);
            }
        }
        self.lap(6, &mut mark);
    }

    fn lap(&mut self, stage: usize, mark: &mut Instant) {
        let now = Instant::now();
        self.prof[stage] += now - *mark;
        *mark = now;
    }

    fn transmit(&mut self) {
        let n48 = (self.block as f64 * CH_RATE / self.rate).round() as usize;
        self.tx_bb.clear();

        // Safety rails first.
        if let Some(since) = self.tx_since {
            // DATV runs long by nature; it stops when its browser goes quiet instead.
            let datv = matches!(self.tx_on, Some(TxSource::Datv(_)));
            if !datv && since.elapsed() > Duration::from_secs(self.cfg.trx.max_tx_seconds as u64) {
                warn!("TX time-out ({} s), unkeying", self.cfg.trx.max_tx_seconds);
                self.unkey();
            }
        }

        match self.tx_on {
            None => {
                // Anything a TCI client sends while we are not keyed is stale.
                if let Some(t) = &mut self.tci {
                    t.drain_tx_audio();
                }
            }
            Some(src @ (TxSource::Cw | TxSource::CwPtt)) => {
                self.keyer.set_offset_hz(CW_PITCH_HZ);
                self.keyer.render(n48, &mut self.tx_bb);
                if src == TxSource::CwPtt || self.keyer.busy() {
                    self.cw_idle_since = None;
                } else {
                    let idle = *self.cw_idle_since.get_or_insert_with(Instant::now);
                    if idle.elapsed() > self.cw_hang {
                        self.cw_idle_since = None;
                        self.unkey();
                    }
                }
            }
            Some(TxSource::Key) => {
                // 5 ms raised-cosine edges: no key clicks. Released and faded
                // out, the transmitter drops.
                let step = std::f64::consts::TAU * CW_PITCH_HZ / CH_RATE;
                let ramp = 1.0 / (0.005 * CH_RATE as f32);
                for _ in 0..n48 {
                    let target = if self.key_down { 1.0 } else { 0.0 };
                    self.key_env = if self.key_env < target { (self.key_env + ramp).min(1.0) } else { (self.key_env - ramp).max(0.0) };
                    let a = 0.5 - 0.5 * (std::f32::consts::PI * self.key_env).cos();
                    self.tune_phase = (self.tune_phase + step) % std::f64::consts::TAU;
                    self.tx_bb.push(Complex32::new(self.tune_phase.cos() as f32, self.tune_phase.sin() as f32) * a);
                }
                if !self.key_down && self.key_env == 0.0 {
                    self.unkey();
                }
            }
            Some(TxSource::Datv(client)) => {
                let mut starved = self.datv.is_none();
                if let Some(d) = &mut self.datv {
                    if let Some(w) = &self.web {
                        for b in w.take_media() {
                            if let Some(m) = crate::dvbs2::ts::Media::from_ws(&b) {
                                d.mux.push(m);
                                d.last_media = Instant::now();
                            }
                        }
                        if d.mux.take_key_request() {
                            w.send_json_to(client, &serde_json::json!({"type": "datv_key"}));
                        }
                    }
                    if let Some(t2) = &d.t2 {
                        // The T2 thread takes packets at the TS rate (paced
                        // by the DAC) and writes the TX buffer itself.
                        t2.feed(&mut d.mux);
                    } else if let Some(fx) = &mut d.fpga {
                        // Encoder bytes as fast as the DMA takes them: the
                        // writer blocks on it, so a few blocks queued is all
                        // the pacing needed (the encoder runs at the symbol rate).
                        let bytes = self.block * 4;
                        while self.tx_sink.len() < 3 {
                            while d.pending.len() < bytes {
                                let x = &mut d.mux;
                                fx.frame(0, &mut || x.next(), &mut d.pending);
                            }
                            let blk: Vec<u8> = d.pending.drain(..bytes).collect();
                            if self.tx_sink.try_send(crate::stream::TxBlock::Raw(blk)).is_err() {
                                break;
                            }
                        }
                    }
                    if d.last_media.elapsed() > DATV_STARVE {
                        warn!("DATV: nothing from the browser for {} s; unkeying", DATV_STARVE.as_secs());
                        starved = true;
                    }
                }
                if starved {
                    self.unkey();
                }
            }
            Some(TxSource::Tune) => {
                let hz = if self.mode == Mode::Cw { CW_PITCH_HZ } else { TUNE_TONE_HZ };
                let step = std::f64::consts::TAU * hz / CH_RATE;
                for _ in 0..n48 {
                    self.tune_phase = (self.tune_phase + step) % std::f64::consts::TAU;
                    self.tx_bb.push(Complex32::new(self.tune_phase.cos() as f32, self.tune_phase.sin() as f32));
                }
            }
            Some(TxSource::Web(_)) => {
                let n12 = n48 / 4;
                let mut audio12 = Vec::with_capacity(n12);
                let draining = self.mic_drain_until.is_some();
                let mut drained = self.mic_drain_until.is_some_and(|t| Instant::now() >= t);
                if let Some(w) = &self.web {
                    if !self.mic_started && (draining || w.mic_queued() >= MIC_PREROLL) {
                        self.mic_started = true;
                    }
                    if self.mic_started && w.take_mic(n12, &mut audio12) > 0 {
                        self.mic_last = Instant::now();
                    }
                    drained |= draining && audio12.len() < n12;
                }
                audio12.resize(n12, 0.0);
                // x4: zero-stuff, low-pass, and make up the 4x energy loss.
                self.mic_buf.clear();
                let g = 4.0 * self.mic_gain;
                for a in &audio12 {
                    self.mic_buf.extend_from_slice(&[a * g, 0.0, 0.0, 0.0]);
                }
                let mut audio = Vec::with_capacity(n48);
                self.mic_up.process(&self.mic_buf, &mut audio);
                audio.resize(n48, 0.0);
                let comp = self.ssb_comp();
                self.speech.process(&mut audio, comp);
                match &mut self.modulator {
                    Some(m) => m.process(&audio, &mut self.tx_bb),
                    None => self.tx_bb.extend(audio.iter().map(|&a| Complex32::new(a, 0.0))),
                }
                if drained {
                    self.unkey();
                } else if self.mic_last.elapsed() > MIC_STARVE {
                    warn!("web microphone stopped; unkeying");
                    self.unkey();
                }
            }
            Some(src @ (TxSource::Tci | TxSource::Ptt)) => {
                let mut got = 0;
                if let Some(t) = &mut self.tci {
                    let mut buf = [0.0f32; 1024];
                    loop {
                        let k = t.read_tx_audio(&mut buf);
                        self.tx_fifo.extend(&buf[..k]);
                        got += k;
                        if k < buf.len() {
                            break;
                        }
                    }
                    let playing = self.tx_fifo.len() >= n48;
                    if let Some(frames) = self.pace.request(n48, self.tx_fifo.len(), got, playing) {
                        t.request_chrono(frames);
                    }
                }
                if got > 0 {
                    self.tci_last_audio = Instant::now();
                }
                // Queue bound: nobody transmits faster than real time.
                while self.tx_fifo.len() > 24_000 {
                    self.tx_fifo.pop_front();
                }
                let mut audio: Vec<f32> = (0..n48).map(|_| self.tx_fifo.pop_front().unwrap_or(0.0)).collect();
                let comp = self.ssb_comp();
                self.speech.process(&mut audio, comp);
                match &mut self.modulator {
                    Some(m) => m.process(&audio, &mut self.tx_bb),
                    None => self.tx_bb.extend(audio.iter().map(|&a| Complex32::new(a, 0.0))),
                }
                if src == TxSource::Tci && self.tci_last_audio.elapsed() > TCI_STARVE {
                    warn!("TCI client stopped sending TX audio; unkeying");
                    self.unkey();
                }
            }
        }

        // The FPGA transmits DATV from the encoder bytes queued above: no IQ.
        if matches!(self.tx_on, Some(TxSource::Datv(_))) && self.datv.as_ref().is_some_and(|d| d.fpga.is_some() || d.t2.is_some()) {
            return;
        }

        // CESSB for SSB voice only: never on data (it would distort the
        // digital signal), CW, AM/FM or the tune carrier.
        let ssb = matches!(self.mode, Mode::Usb | Mode::Lsb) && !self.datv_mode;
        if let (true, Some(c), Some(TxSource::Web(_) | TxSource::Tci | TxSource::Ptt)) = (ssb, &mut self.comp, self.tx_on) {
            if !self.tx_bb.is_empty() {
                c.process(&mut self.tx_bb);
            }
        }

        // Idle: the transmit thread writes its own zeros.
        if self.tx_on.is_none() && self.tx_bb.is_empty() && self.tx_out.is_empty() {
            if self.tx_sink.try_send(crate::stream::TxBlock::Silence).is_err() {
                debug!("TX queue full");
            }
            return;
        }
        // Up to the stream rate and out to the VFO.
        let mut block = vec![Complex32::default(); self.block];
        if !self.tx_bb.is_empty() {
            let g = self.drive;
            for z in &mut self.tx_bb {
                *z *= g;
            }
            self.tx_up.clear();
            self.duc.process(&self.tx_bb, &mut self.tx_up);
            let mut mixed = Vec::with_capacity(self.tx_up.len());
            self.tx_nco.mix(&self.tx_up, &mut mixed);
            self.tx_out.extend(mixed);
        }
        for z in block.iter_mut() {
            match self.tx_out.pop_front() {
                Some(s) => *z = s,
                None => break,
            }
        }
        if self.tx_on.is_none() {
            self.tx_out.clear();
        } else {
            self.tx_peak = block.iter().fold(self.tx_peak, |m, z| m.max(z.norm()));
        }
        // An inverting transverter mirrors what it sends as well.
        if self.xvtr.as_ref().is_some_and(|t| t.inverted) {
            for z in &mut block {
                *z = z.conj();
            }
        }
        if self.tx_sink.try_send(crate::stream::TxBlock::Iq(block)).is_err() {
            debug!("TX queue full");
        }
    }

    // ---- Web UI ----

    fn web_cw_json(&self) -> serde_json::Value {
        let (text, pending) = self.cwlive.snapshot();
        serde_json::json!({"type": "cw", "text": text, "pending": pending})
    }

    /// Live CW text out to the web UI when it changed (the whole, capped, text).
    fn publish_cwlive(&mut self) {
        let Some((text, pending, _fresh)) = self.cwlive.changed() else { return };
        if let Some(w) = &self.web {
            if w.clients() > 0 {
                w.send_json(&serde_json::json!({"type": "cw", "text": text, "pending": pending}));
            }
        }
    }

    fn web_decode(&self, d: &Decode) {
        if let Some(w) = &self.web {
            let mut v = serde_json::to_value(d).unwrap_or_default();
            v["type"] = "decode".into();
            w.send_json(&v);
        }
    }

    fn web_state_json(&self) -> serde_json::Value {
        let temps = *self.temps.lock().unwrap();
        let agc = match self.agc_mode {
            AgcMode::Off => "off",
            AgcMode::Slow => "slow",
            AgcMode::Med => "med",
            AgcMode::Fast => "fast",
        };
        let mut v = serde_json::json!({
            "type": "state",
            "call": self.callsign(),
            "vfo_a": self.vfo_a,
            "vfo_b": self.vfo_b,
            "active": if self.active == Vfo::A { "A" } else { "B" },
            "split": self.split,
            "mode": mode_name(self.mode),
            "filter": [self.filter.0, self.filter.1],
            "ptt": self.tx_on.is_some(),
            "tune": self.tx_on == Some(TxSource::Tune),
            "tx_source": self.tx_on.map(|s| match s {
                TxSource::Tci => "tci",
                TxSource::Ptt => "ptt",
                TxSource::Cw | TxSource::CwPtt => "cw",
                TxSource::Tune => "tune",
                TxSource::Key => "key",
                TxSource::Web(_) => "web",
                TxSource::Datv(_) => "datv",
            }),
            "drive": (self.drive * 100.0).round(),
            "tx_att_db": self.tx_att_db,
            "rx_gain_db": self.rx_gain_db,
            "rx_gain_mode": match self.rx_gain_mode {
                GainMode::Manual => "manual",
                GainMode::SlowAttack => "slow",
                GainMode::FastAttack => "fast",
                GainMode::Hybrid => "hybrid",
            },
            "agc": agc,
            "cw_wpm": self.keyer.wpm(),
            "center": self.center,
            "rate": self.rate,
            "span": self.web_span,
            "span_max": if self.maia.is_some() { self.cfg.radio.adc_rate as f64 } else { self.rate },
            "allow_tx": self.cfg.trx.allow_tx,
            "rade": self.rade,
            "tx_ok": self.tx_check(self.tx_eff(), self.tx_half_bw()).is_ok(),
            "decoders": self.slots.iter().map(|(k, _)| match k {
                DecoderKind::Q65 => "q65",
                DecoderKind::Pi4 => "pi4",
                DecoderKind::Cw => "cw",
            }).collect::<Vec<_>>(),
            "freq_min": self.freq_range().0,
            "freq_max": self.freq_range().1,
            "time_synced": time_synced(),
            "ref": crate::refclock::summary(),
            "temp_fpga": temps.fpga.map(|c| c.round()),
            "temp_ad936x": temps.ad936x.map(|c| c.round()),
            "tci_clients": self.tci.as_ref().map_or(0, |t| t.clients()),
            "rigctl_clients": self.rig.as_ref().map_or(0, |r| r.clients()),
        });
        let more = serde_json::json!({
            "rit": [self.rit.0, self.rit.1],
            "xit": [self.xit.0, self.xit.1],
            "nb": self.nb.is_some(),
            "nr": self.nr.is_some(),
            "notch": self.notch.is_some(),
            "squelch": self.squelch_db,
            "comp": self.comp.as_ref().map(|_| self.comp_db),
            "mic_gain": self.mic_gain,
            "cw_hang_ms": self.cw_hang.as_millis() as u64,
            "scope_center": self.scope_center,
            "mute_at_tx": self.settings.mute_at_tx,
            "port": self.radio.port(),
            "ports": self.settings.ports,
            "datv": self.datv_json(),
            "datv_mode": self.datv_mode,
            "datv_rx": match (&self.datv_rx, self.datv_auto) {
                (Some(r), auto) => Some(serde_json::json!({"sr": r.sr, "rate": r.label, "pilots": r.spec.pilots, "fpga": true, "auto": auto})),
                (None, true) => Some(serde_json::json!({"sr": 0, "rate": "auto", "pilots": true, "fpga": true, "auto": true})),
                (None, false) => None,
            },
            "xvtr": self.xvtr.as_ref().map(|t| t.name.clone()),
            "xvtrs": self.settings.transverters,
            "cal_band": self.cal_band(),
            // What this firmware can do (docs/FLAVOURS.md): the page hides
            // what is not there.
            "features": {"datv": crate::fpgamode::datv_available(), "flavour": crate::fpgamode::flavour()},
        });
        if let (Some(v), serde_json::Value::Object(m)) = (v.as_object_mut(), more) {
            v.extend(m);
        }
        v
    }

    fn poll_web(&mut self) {
        let Some(w) = &self.web else { return };
        let joined = w.poll_joined();
        let cmds = w.poll_cmds();
        if !joined.is_empty() {
            let st = self.web_state_json();
            let cw = self.web_cw_json();
            if let Some(w) = &self.web {
                for c in joined {
                    w.send_json_to(c, &st);
                    w.send_json_to(c, &cw);
                }
            }
        }
        for c in cmds {
            self.apply_web(c.client, &c.msg);
        }
        self.datv_auto_step();
        self.datv_rx_watchdog();
        let Some(w) = &self.web else { return };
        if let Some(r) = &self.datv_rx {
            let (stats, msgs) = r.take();
            self.datv_rx_stats = stats;
            // A trace in the log every 10 s: how a reception went, afterwards.
            if self.datv_rx_log.elapsed() >= Duration::from_secs(10) {
                self.datv_rx_log = Instant::now();
                let s = &stats;
                info!(
                    locked = s.locked,
                    esn0 = format!("{:.1}", s.esn0_db),
                    mer = format!("{:.1}", s.data_esn0_db),
                    carrier_hz = s.freq_hz.round(),
                    frames = s.frames,
                    bad = s.frames_bad,
                    skipped = s.frames_skipped,
                    fec_busy = s.frames_fec_busy,
                    iq_dropped = s.blocks_dropped,
                    demod_pct = (100.0 * s.other_s / s.wall_s.max(1e-9)).round(),
                    fec_pct = (100.0 * s.fec_cpu_s / s.wall_s.max(1e-9)).round(),
                    fec_wait_pct = (100.0 * s.ldpc_s / s.wall_s.max(1e-9)).round(),
                    blocks_fpga_made = ?(s.trk_hw, s.trk_model),
                    "DATV receive"
                );
            }
            if w.clients() > 0 {
                for m in msgs {
                    w.send_bin(m);
                }
            }
        }
        if w.clients() == 0 {
            return;
        }
        if self.web_meter_at.elapsed() >= Duration::from_millis(100) {
            self.web_meter_at = Instant::now();
            let tx = self.tx_on.is_some();
            // TX meters: mic peak (before the compressor), gain reduction, and
            // the envelope actually sent relative to DAC full scale.
            let comp_on = self.ssb_comp().is_some();
            let txm = tx.then(|| {
                let po = std::mem::take(&mut self.tx_peak);
                serde_json::json!({"mic_db": (self.speech.take_peak_db() * 10.0).round() / 10.0,
                    "gr_db": comp_on.then(|| (self.speech.take_gr_db() * 10.0).round() / 10.0),
                    "po": (po * 1000.0).round() / 1000.0})
            });
            let cw = self.cwlive.readout().map(|r| {
                serde_json::json!({"tone_hz": r.tone_hz.round(), "wpm": r.wpm.round(), "snr_db": r.snr_db.round(), "locked": r.locked})
            });
            let mt = &self.meter;
            let dbm = mt.dbm;
            let datv = self.datv.as_ref().map(|d| serde_json::json!({"backlog": (d.mux.backlog_s() * 10.0).round() / 10.0, "dropped": d.mux.dropped_frames, "rx_paused": self.datv_rx_paused()}));
            let s = &self.datv_rx_stats;
            let datv_rx = (self.datv_rx.is_some() || self.datv_auto).then(|| serde_json::json!({"auto": self.datv_auto.then(|| self.datv_auto_note.clone()), "rx": self.datv_rx.is_some(), "locked": s.locked, "esn0": (s.esn0_db * 10.0).round() / 10.0,
                "mer": (s.data_esn0_db * 10.0).round() / 10.0,
                "freq": s.freq_hz.round(), "frames": s.frames, "bad": s.frames_bad, "packets": s.packets,
                "dropped": s.blocks_dropped, "skipped": s.frames_skipped, "busy": s.frames_fec_busy,
                "demod_pct": (100.0 * s.other_s / s.wall_s.max(1e-9)).round(), "fec_pct": (100.0 * s.fec_cpu_s / s.wall_s.max(1e-9)).round(),
                "si": self.datv_rx.as_ref().map(|r| r.si().json())}));
            w.send_json(&serde_json::json!({"type": "meter", "s_dbfs": self.s_dbfs, "tx": tx, "rx_gain_db": self.hw_gain_db, "cw": cw, "datv": datv, "datv_rx": datv_rx, "txm": txm,
                "dbm": (dbm * 10.0).round() / 10.0, "s": crate::settings::s_units(self.rx_eff(), dbm),
                "dbm_hz": mt.dbm_hz.map(|d| (d * 10.0).round() / 10.0), "cal": mt.quality, "clip": mt.clip, "noise": mt.noise,
                "msrc": mt.src, "dbfs": (mt.dbfs * 10.0).round() / 10.0, "scope_dbm_off": mt.scope_off.map(|d| (d * 10.0).round() / 10.0),
                "reading": (self.reading_db * 10.0).round() / 10.0, "sq": self.squelch_open}));
        }
        if self.web_state_at.elapsed() >= Duration::from_millis(100) {
            let st = self.web_state_json();
            let s = st.to_string();
            if self.web_state.as_deref() != Some(&s) {
                w.send_json(&st);
                self.web_state = Some(s);
            }
            self.web_state_at = Instant::now();
        }
    }

    /// The state saved before an FPGA bitstream switch ([`crate::fpgamode`]):
    /// VFOs, split, mode, filter, then the command that asked for it.
    fn resume(&mut self, r: &serde_json::Value) {
        use serde_json::json;
        info!(fpga = %crate::fpgamode::loaded(), "resuming after the FPGA bitstream switch");
        if let (Some(a), Some(b)) = (r["vfo_a"].as_f64(), r["vfo_b"].as_f64()) {
            self.apply_web(0, &json!({"cmd": "vfo", "sel": "A"}));
            self.apply_web(0, &json!({"cmd": "freq", "hz": a}));
            self.apply_web(0, &json!({"cmd": "vfo", "sel": "B"}));
            self.apply_web(0, &json!({"cmd": "freq", "hz": b}));
            let sel = if r["active_b"].as_bool() == Some(true) { "B" } else { "A" };
            self.apply_web(0, &json!({"cmd": "vfo", "sel": sel}));
        }
        if r["split"].as_bool() == Some(true) {
            self.apply_web(0, &json!({"cmd": "split", "on": true}));
        }
        if let Some(md) = r["mode"].as_str() {
            self.apply_web(0, &json!({"cmd": "mode", "mode": md}));
        }
        if let (Some(lo), Some(hi)) = (r["filter"][0].as_f64(), r["filter"][1].as_f64()) {
            self.apply_web(0, &json!({"cmd": "filter", "lo": lo, "hi": hi}));
        }
        for k in ["txatt", "drive"] {
            if let Some(v) = r[k].as_f64() {
                self.apply_web(0, &json!({"cmd": k, "value": v}));
            }
        }
        if let Some(g) = r["rxgain"].as_object() {
            let mut c = serde_json::Value::Object(g.clone());
            c["cmd"] = json!("rxgain");
            self.apply_web(0, &c);
        }
        if let Some(a) = r["agc"].as_str() {
            self.apply_web(0, &json!({"cmd": "agc", "mode": a}));
        }
        for c in r["cmds"].as_array().into_iter().flatten() {
            self.apply_web(0, c);
        }
    }

    fn apply_web(&mut self, client: u64, m: &serde_json::Value) {
        let cmd = m["cmd"].as_str().unwrap_or("");
        let num = |k: &str| m[k].as_f64();
        let on = m["on"].as_bool().unwrap_or(false);
        // A feature on a part of the FPGA the loaded bitstream lacks: load
        // one that has it (trxd restarts and takes this command up again).
        let part = match cmd {
            "datv_mode" | "datv_rx" | "datv" if on => Some(crate::fpgamode::datv_part(m["rate"].as_str())),
            _ => None,
        };
        // No bitstream with DATV in this firmware (BASIC): refused, not run
        // on hardware that is not there.
        if matches!(part, Some(crate::fpgamode::Part::S2 | crate::fpgamode::Part::T2)) && !crate::fpgamode::datv_available() {
            warn!(cmd, "DATV refused: this firmware has no DATV bitstream (BASIC flavour)");
            if let Some(w) = &self.web {
                w.send_json_to(client, &serde_json::json!({"type": "datv_error", "msg": "this firmware (BASIC) has no DATV: install the BASIC+ image"}));
            }
            return;
        }
        if let Some(want) = part.and_then(crate::fpgamode::switch_for) {
            if self.tx_on.is_none() {
                // every page shows it (the switch restarts trxd: all of them
                // lose the connection for a few seconds)
                if let Some(w) = &self.web {
                    w.send_json(&serde_json::json!({"type": "fpga_switch", "mode": want}));
                }
                let resume = serde_json::json!({
                    "vfo_a": self.vfo_a, "vfo_b": self.vfo_b, "active_b": self.active == Vfo::B,
                    "split": self.split, "mode": mode_name(self.mode),
                    "filter": [self.filter.0, self.filter.1], "cmds": [m],
                    // the levels too: a TX attenuation set before entering
                    // DATV mode was lost and DATV went out 20 dB down
                    "txatt": self.tx_att_db, "drive": (self.drive * 100.0).round(),
                    "rxgain": match self.rx_gain_mode {
                        GainMode::Manual => serde_json::json!({"mode": "manual", "db": self.rx_gain_db}),
                        GainMode::FastAttack => serde_json::json!({"mode": "fast"}),
                        _ => serde_json::json!({"mode": "slow"}),
                    },
                    "agc": match self.agc_mode { AgcMode::Off => "off", AgcMode::Slow => "slow", AgcMode::Fast => "fast", _ => "med" },
                });
                std::thread::sleep(std::time::Duration::from_millis(300));
                crate::fpgamode::request(want, &resume);
            }
        }
        match cmd {
            "freq" => {
                if let Some(hz) = num("hz") {
                    self.set_vfo(self.active, hz.round());
                }
            }
            "mode" => {
                // Any voice mode leaves DATV mode.
                self.datv_leave();
                if let Some(md) = m["mode"].as_str().and_then(sdroxide_rigctld::from_hamlib_mode) {
                    self.set_mode(md);
                }
            }
            "filter" => {
                if let (Some(lo), Some(hi)) = (num("lo"), num("hi")) {
                    self.apply(Command::SetFilter { rx: sdroxide_types::RxId::Main, lo: lo as f32, hi: hi as f32 });
                }
            }
            // DATV has its own start/stop (and stops when its page goes
            // quiet); voice PTT, pressed or released, leaves it alone. In DATV
            // mode there is no voice to send.
            "ptt" if matches!(self.tx_on, Some(TxSource::Datv(_))) || self.datv_mode => {
                debug!("PTT ignored: DATV is transmitting");
            }
            "ptt" => {
                if self.mode == Mode::Cw {
                    // In CW the web PTT is a straight key; release lets the
                    // carrier fade out first (see TxSource::Key).
                    self.key_down = on;
                    if on && self.tx_on.is_none() {
                        self.key_env = 0.0;
                        self.key(TxSource::Key);
                    } else if !on && self.tx_on != Some(TxSource::Key) {
                        self.unkey();
                    }
                } else if on {
                    self.key(TxSource::Web(client));
                } else if m["drain"].as_bool() == Some(true) && self.tx_on == Some(TxSource::Web(client)) {
                    // The rest of the over (RADE's EOO) still in the queue.
                    self.mic_drain_until = Some(Instant::now() + MIC_DRAIN_MAX);
                } else {
                    self.unkey();
                }
            }
            "tune" => self.apply(Command::SetTune(on)),
            "datv_mode" => {
                if on {
                    self.datv_mode = true;
                    self.datv_rx_start(num("sr").unwrap_or(64_000.0), m["rate"].as_str().unwrap_or("L-QPSK-1/2"));
                } else {
                    self.datv_leave();
                }
            }
            "datv_rx" => {
                self.datv_rx = None;
                self.datv_scan = None;
                self.datv_rx_stats = Default::default();
                self.datv_rx_req = on.then(|| (num("sr").unwrap_or(64_000.0), m["rate"].as_str().unwrap_or("L-QPSK-1/2").to_string()));
                // Sending DATV the A9 cannot carry as well (DVB-T2, the
                // software modulator): the receiver starts when that ends.
                if let Some((sr, rate)) = self.datv_rx_req.clone().filter(|_| self.datv_rx_alongside_tx()) {
                    self.datv_rx_start(sr, &rate);
                }
                // The LO back beside the signal if no DVB-T2 receiver needs it on.
                self.retune(false);
            }
            "datv" => {
                if on {
                    if self.tx_on.is_some() {
                        warn!("DATV refused: already transmitting");
                    } else {
                        let sr = num("sr").unwrap_or(64_000.0);
                        let rate = m["rate"].as_str().unwrap_or("1/2").to_string();
                        self.datv_start(client, sr, &rate);
                        // The FPGA's DVB-S2 transmitter (or DVB-T2 with the
                        // FPGA's IFFT) leaves the A9 room: the receiver goes
                        // on (the board hears itself, or another station on
                        // the RX VFO: split, cross band too). The software
                        // modulator needs the cores: receiving waits.
                        if !self.datv_rx_alongside_tx() && (self.datv_rx.is_some() || self.datv_scan.is_some()) {
                            info!(why = self.datv_rx_paused(), "DATV receive paused while sending");
                            self.datv_rx = None;
                            self.datv_scan = None;
                        }
                    }
                } else if matches!(self.tx_on, Some(TxSource::Datv(_))) {
                    self.unkey();
                }
            }
            "vfo" => {
                let v = if m["sel"].as_str() == Some("B") { Vfo::B } else { Vfo::A };
                self.apply(Command::SelectVfo(v));
            }
            "swap" => self.apply(Command::SwapVfos),
            "a_to_b" => self.apply(Command::CopyAtoB),
            "split" => self.apply(Command::SetSplit(on)),
            "agc" => {
                let a = match m["mode"].as_str() {
                    Some("off") => AgcMode::Off,
                    Some("slow") => AgcMode::Slow,
                    Some("fast") => AgcMode::Fast,
                    _ => AgcMode::Med,
                };
                self.agc_mode = a;
                self.agc.set_mode(a);
            }
            "span" => {
                if let Some(sp) = num("hz") {
                    let max = if self.maia.is_some() { self.cfg.radio.adc_rate as f64 } else { self.rate };
                    self.web_span = sp.clamp(2_000.0, max);
                    self.web_center = 0.0;
                    self.scope_wide.reset();
                    self.scope_narrow.reset();
                }
            }
            "drive" | "txatt" | "cw_wpm" => {
                if let Some(v) = num("value") {
                    self.apply_setting(cmd, &v.to_string());
                }
            }
            "rxgain" => {
                let payload = match m["mode"].as_str() {
                    Some("manual") => num("db").unwrap_or(self.rx_gain_db).to_string(),
                    Some("fast") => "fast".into(),
                    _ => "slow".into(),
                };
                self.apply_setting("rxgain", &payload);
            }
            "cw" => {
                if let Some(t) = m["text"].as_str() {
                    self.send_cw(t);
                }
            }
            "cw_stop" => {
                self.keyer.abort();
            }
            "cw_clear" => self.cwlive.clear(),
            "rit" | "xit" => {
                let hz = num("hz").unwrap_or(0.0).clamp(-9_999.0, 9_999.0).round();
                let v = (on, hz);
                if cmd == "rit" { self.rit = v } else { self.xit = v }
                self.cwlive.restart();
                self.retune(false);
            }
            "nb" => self.nb = on.then(NoiseBlanker::new),
            "nr" => self.nr = on.then(SpectralNr::new),
            "notch" => self.notch = on.then(AutoNotch::new),
            "squelch" => {
                self.squelch_db = num("db").map(|d| d.clamp(-160.0, 0.0) as f32);
                self.squelch_open = true;
            }
            "comp" => {
                if let Some(d) = num("db") {
                    self.comp_db = d.clamp(0.0, 20.0) as f32;
                }
                self.comp = on.then(|| self.make_comp());
            }
            "mic_gain" => {
                if let Some(v) = num("value") {
                    self.mic_gain = v.clamp(0.0, 4.0) as f32;
                }
            }
            "cw_hang" => {
                if let Some(ms) = num("ms") {
                    self.cw_hang = Duration::from_millis(ms.clamp(50.0, 3_000.0) as u64);
                }
            }
            "mute_at_tx" => {
                self.settings.mute_at_tx = on;
                self.settings.save(&self.settings_dir);
                info!(on, "mute the receiver at TX");
            }
            "port" => {
                if let Some(p) = num("port") {
                    self.set_port(p as u8);
                }
            }
            "port_map" => {
                // A band (label or transverter name) to sockets 1 or 2; 0 forgets it.
                let band = m["band"].as_str().unwrap_or("").trim().to_string();
                if let (false, Some(p)) = (band.is_empty(), num("port")) {
                    match p as u8 {
                        1 | 2 => {
                            self.settings.ports.insert(band.clone(), p as u8);
                        }
                        _ => {
                            self.settings.ports.remove(&band);
                        }
                    }
                    self.settings.save(&self.settings_dir);
                    info!(band, port = p, "band sockets");
                    // Apply at once if it is the band we are in.
                    self.port_band = None;
                    self.follow_port();
                }
            }
            "scope_center" => {
                self.scope_center = on;
                self.web_center = 0.0;
            }
            "callsign" => {
                // An empty call goes back to trxd.toml's.
                let raw = m["call"].as_str().unwrap_or("");
                match Settings::clean_call(raw) {
                    Some(c) => self.settings.callsign = Some(c),
                    None if raw.trim().is_empty() => self.settings.callsign = None,
                    None => {
                        warn!(call = raw, "callsign refused");
                        return;
                    }
                }
                self.settings.save(&self.settings_dir);
                info!(call = %self.callsign(), "callsign set");
            }
            "xvtr_set" => {
                if let Ok(list) = serde_json::from_value::<Vec<Transverter>>(m["list"].clone()) {
                    // Names go into the page and the S-meter tables: plain ones only.
                    let name_ok = |n: &str| {
                        let n = n.trim();
                        (1..=24).contains(&n.len()) && n.chars().all(|c| c.is_ascii_alphanumeric() || " ._-".contains(c))
                    };
                    let ok = list.iter().all(|t| t.rf_max > t.rf_min && name_ok(&t.name));
                    if ok {
                        self.settings.transverters = list;
                        self.settings.save(&self.settings_dir);
                        info!(n = self.settings.transverters.len(), "transverters saved");
                        let f = self.reachable(self.rx_vfo());
                        self.set_vfo(self.active, f);
                        self.retune(true);
                    } else {
                        warn!("transverter list refused: a name is empty or not plain (letters, digits, space . _ -, 24 at most), or a range is backwards");
                    }
                }
            }
            "meter_raw" => {
                let r = self.meter_raw(num("lo_hz"), num("hi_hz"));
                if let Some(w) = &self.web {
                    w.send_json_to(client, &r);
                }
            }
            "calib_get" => {
                let r = self.calib_json();
                if let Some(w) = &self.web {
                    w.send_json_to(client, &r);
                }
            }
            "calib_set" | "calib_clear" => {
                let port = num("port").map_or(0, |p| p as u8);
                let res = if !(1..=2).contains(&port) {
                    Err("port must be 1 or 2".to_string())
                } else if cmd == "calib_clear" {
                    crate::calib::Calib::remove(&self.settings_dir, port).map(|_| None)
                } else {
                    serde_json::from_value::<crate::calib::Calib>(m["table"].clone())
                        .map_err(|e| e.to_string())
                        .and_then(|mut c| {
                            c.validate()?;
                            c.normalise();
                            c.save(&self.settings_dir, port)?;
                            Ok(Some(c))
                        })
                };
                let reply = match res {
                    Ok(c) => {
                        info!(port, points = c.as_ref().map_or(0, |c| c.k.len()), "{cmd}");
                        self.calib[port as usize - 1] = c;
                        serde_json::json!({"type": "calib_ack", "cmd": cmd, "port": port, "ok": true})
                    }
                    Err(e) => {
                        warn!(port, "{cmd}: {e}");
                        serde_json::json!({"type": "calib_ack", "cmd": cmd, "port": port, "ok": false, "error": e})
                    }
                };
                if let Some(w) = &self.web {
                    w.send_json_to(client, &reply);
                    w.send_json_to(client, &self.calib_json());
                }
            }
            // The page has set DATA mode and its filter first.
            "rade" => self.rade = on && self.mode == Mode::Digu,
            "decoder" => {
                let kind = match m["kind"].as_str() {
                    Some("q65") => Some(DecoderKind::Q65),
                    Some("pi4") => Some(DecoderKind::Pi4),
                    _ => None,
                };
                if let Some(k) = kind {
                    self.set_decoder(k, on);
                }
            }
            "disconnected" => {
                // A straight key whose browser went away comes up.
                if self.tx_on == Some(TxSource::Key) {
                    warn!("web client holding the key disconnected; key up");
                    self.key_down = false;
                }
                if self.tx_on == Some(TxSource::Web(client)) {
                    warn!("web client holding PTT disconnected; unkeying");
                    self.unkey();
                }
                if self.tx_on == Some(TxSource::Datv(client)) {
                    warn!("web client sending DATV disconnected; unkeying");
                    self.unkey();
                }
            }
            other => debug!(cmd = other, "web command not understood"),
        }
    }

    /// Run until the RX stream ends.
    pub fn run(mut self, rx: Receiver<RxBlock>) {
        crate::stream::realtime_thread();
        info!(freq = self.rx_vfo(), mode = mode_name(self.mode), rate = self.rate, "transceiver running");
        if let Some(r) = crate::fpgamode::take_resume() {
            self.resume(&r);
        }
        // Sockets: the band's own pair (SET), else the last one used.
        let band = self.cal_band();
        let p = self.settings.ports.get(&band).copied().unwrap_or(self.settings.port);
        self.set_port(p);
        self.port_band = Some(band);
        let mut loads = LoadMeter::new();
        for b in rx {
            let t = Instant::now();
            crate::safety::tick();
            if crate::safety::take_tripped() {
                warn!("the TX watchdog switched RF off: unkeying");
                self.unkey();
            }
            if self.tx_on.is_none() {
                if let Some(hz) = crate::refclock::take_pending() {
                    if let Err(e) = self.radio.apply_xo(hz) {
                        warn!("xo_correction: {e}");
                    }
                }
            }
            self.poll_controls();
            self.receive(&b);
            let mut mark = Instant::now();
            self.transmit();
            self.lap(7, &mut mark);
            for d in self.decoder.poll() {
                info!(mode = %d.mode, snr = d.snr_db, freq = d.freq_hz, "{}", d.message);
                self.web_decode(&d);
            }
            self.poll_web();
            self.publish_cwlive();
            self.publish_state();
            if let Some(t) = &self.tci {
                if self.tx_on.is_some() {
                    t.push_telemetry(TxTelemetry::default());
                }
            }
            self.lap(8, &mut mark);
            if loads.add(t.elapsed(), b.iq.len() as f64 / self.rate) {
                let span = 60.0;
                let parts: Vec<String> = STAGES
                    .iter()
                    .zip(&self.prof)
                    .map(|(n, d)| format!("{n}={:.1}", 100.0 * d.as_secs_f64() / span))
                    .collect();
                info!("engine profile (% of one core): {}", parts.join(" "));
                self.prof = [Duration::ZERO; STAGES.len()];
            }
        }
        self.unkey();
    }
}

/// The slot recorder a decoder reads from; `None` for CW (the live CW box reads it).
fn slot_recorder(kind: DecoderKind) -> Option<SlotRecorder> {
    match kind {
        DecoderKind::Q65 => Some(SlotRecorder::new(12_000.0, 60, 0.0, 59.0)),
        DecoderKind::Pi4 => Some(SlotRecorder::new(12_000.0, 60, 2.5, 30.0)),
        DecoderKind::Cw => None,
    }
}


/// Engine CPU use, logged once a minute: the one number that says whether
/// this board keeps up.
struct LoadMeter {
    busy: f64,
    span: f64,
    since: Instant,
}

impl LoadMeter {
    fn new() -> Self {
        LoadMeter { busy: 0.0, span: 0.0, since: Instant::now() }
    }
    /// True when a report went out (once a minute).
    fn add(&mut self, busy: Duration, span_s: f64) -> bool {
        self.busy += busy.as_secs_f64();
        self.span += span_s;
        if self.since.elapsed() >= Duration::from_secs(60) {
            info!(load_pct = (100.0 * self.busy / self.span.max(1e-9)).round(), "engine load");
            *self = LoadMeter::new();
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_duplex_only_across_bands() {
        assert!(!cross_band(2330e6, 2330e6));
        assert!(!cross_band(2330e6, 2400e6));
        assert!(cross_band(2330e6, 1255e6));
        assert!(cross_band(436e6, 1290e6));
        assert!(!cross_band(10368e6, 10370e6));
        assert!(cross_band(10368e6, 5760e6));
    }
    use std::sync::{Arc, Mutex};

    /// A radio that only writes down what it is told.
    struct Rec {
        log: Arc<Mutex<Vec<String>>>,
    }

    impl Rec {
        fn note(&self, s: String) -> Result<(), String> {
            self.log.lock().unwrap().push(s);
            Ok(())
        }
    }

    impl RadioControl for Rec {
        fn stream_rate(&self) -> f64 {
            384_000.0
        }
        fn set_lo(&mut self, hz: f64) -> Result<(), String> {
            self.note(format!("lo {hz:.0}"))
        }
        fn set_los(&mut self, rx: f64, tx: f64) -> Result<(), String> {
            self.note(format!("los {rx:.0} {tx:.0}"))
        }
        fn set_rx_lo(&mut self, hz: f64) -> Result<(), String> {
            self.note(format!("rxlo {hz:.0}"))
        }
        fn apply_xo(&mut self, hz: f64) -> Result<(), String> {
            self.note(format!("xo {hz:.0}"))
        }
        fn set_rx_gain(&mut self, _: GainMode, _: f64) -> Result<(), String> {
            Ok(())
        }
        fn set_tx_attenuation(&mut self, _: f64) -> Result<(), String> {
            Ok(())
        }
        fn set_tx_rf(&mut self, on: bool) -> Result<(), String> {
            self.note(format!("rf {}", if on { "on" } else { "off" }))
        }
        fn rx_gain_db(&mut self) -> f64 {
            0.0
        }
    }

    fn trx(freq: f64) -> (Trx, Arc<Mutex<Vec<String>>>) {
        let dir = std::env::temp_dir().join(format!("trxd-trx-test-{}-{}", std::process::id(), freq as u64));
        let _ = std::fs::create_dir_all(&dir);
        let mut cfg = Config::default();
        cfg.trx.freq_hz = freq;
        cfg.trx.tci_bind = "127.0.0.1".into();
        cfg.trx.tci_port = 0;
        cfg.trx.rigctl_bind = "127.0.0.1".into();
        cfg.trx.rigctl_port = 0;
        cfg.web.state_dir = dir.to_string_lossy().into();
        let log = Arc::new(Mutex::new(Vec::new()));
        let (tx, _rx) = crossbeam_channel::bounded(64);
        let t = Trx::new(cfg, Box::new(Rec { log: log.clone() }), tx, None);
        log.lock().unwrap().clear();
        (t, log)
    }

    fn after_rf_on(log: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        let l = log.lock().unwrap();
        let i = l.iter().position(|s| s == "rf on").expect("keyed");
        l[i + 1..].to_vec()
    }

    #[test]
    fn wide_fm_takes_a_192k_channel_beside_the_lo() {
        let (mut t, _log) = trx(100_000_000.0);
        t.set_mode(Mode::Wfm);
        assert_eq!(t.chan_rate, WFM_RATE);
        // The whole channel inside the stream.
        let off = (t.rx_eff() - t.center).abs();
        assert!(off + WFM_HALF_HZ < t.rate / 2.0, "offset {off}");
        // Broadcast FM is receive only.
        assert!(t.tx_check(t.tx_eff(), t.tx_half_bw()).is_err());
        // A 1 kHz tone at +/-75 kHz deviation comes out as 48 kHz audio at 1 kHz.
        let n = 192_000;
        let mut ph = 0.0f64;
        let iq: Vec<Complex32> = (0..n)
            .map(|i| {
                ph += std::f64::consts::TAU * 75_000.0 * (std::f64::consts::TAU * 1_000.0 * i as f64 / WFM_RATE).sin() / WFM_RATE;
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut audio = Vec::new();
        t.demod.process(&iq, &mut audio);
        // (less the start-up of the 383-tap decimating filters)
        assert!((audio.len() as i64 - (n / 4) as i64).abs() < 256, "{} samples", audio.len());
        let tail = &audio[audio.len() / 2..];
        let crossings = tail.windows(2).filter(|w| w[0] < 0.0 && w[1] >= 0.0).count() as f64;
        let hz = crossings * CH_RATE / tail.len() as f64;
        assert!((hz - 1_000.0).abs() < 20.0, "{hz} Hz");
        t.set_mode(Mode::Usb);
        assert_eq!(t.chan_rate, CH_RATE);
    }

    #[test]
    fn transmit_refused_outside_the_tx_ranges() {
        let (mut t, log) = trx(147_000_000.0);
        t.apply(Command::SetPtt(true));
        assert!(t.tx_on.is_none());
        assert!(!log.lock().unwrap().iter().any(|s| s == "rf on"));
        t.cfg.trx.tx_ranges = vec![[146e6, 148e6]];
        t.apply(Command::SetPtt(true));
        assert_eq!(t.tx_on, Some(TxSource::Ptt));
    }

    #[test]
    fn small_moves_while_keyed_keep_the_tx_lo() {
        let (mut t, log) = trx(144_300_000.0);
        t.apply(Command::SetPtt(true));
        t.set_vfo(Vfo::A, 144_310_000.0);
        assert_eq!(t.tx_on, Some(TxSource::Ptt));
        assert!(after_rf_on(&log).is_empty(), "{:?}", log.lock().unwrap());
    }

    #[test]
    fn a_move_that_needs_the_lo_ends_the_transmission_first() {
        let (mut t, log) = trx(144_300_000.0);
        t.apply(Command::SetPtt(true));
        t.set_vfo(Vfo::A, 145_500_000.0);
        assert!(t.tx_on.is_none());
        let after = after_rf_on(&log);
        assert_eq!(after.first().map(String::as_str), Some("rf off"), "{after:?}");
        assert!(after.iter().any(|s| s.starts_with("lo ")), "{after:?}");
    }

    #[test]
    fn leaving_the_band_while_keyed_unkeys() {
        let (mut t, log) = trx(145_990_000.0);
        t.apply(Command::SetPtt(true));
        assert!(t.tx_on.is_some());
        t.set_vfo(Vfo::A, 146_010_000.0);
        assert!(t.tx_on.is_none());
        assert_eq!(after_rf_on(&log).first().map(String::as_str), Some("rf off"));
    }

    /// Feed `secs` of stream with a tone of amplitude `amp` at `off_hz` from
    /// the dial, scaled by `gain_db` (the front end), through the engine.
    fn feed_tone(t: &mut Trx, amp: f64, off_hz: f64, gain_db: f64, secs: f64) {
        let rate = t.rate;
        let f = t.rx_eff() + off_hz - t.center;
        let a = amp * 10f64.powf(gain_db / 20.0);
        let n = (secs * rate) as usize;
        let mut i = 0usize;
        while i < n {
            let iq: Vec<Complex32> = (i..i + t.block)
                .map(|k| {
                    let p = 2.0 * std::f64::consts::PI * f * k as f64 / rate;
                    Complex32::new((a * p.cos()) as f32, (a * p.sin()) as f32)
                })
                .collect();
            t.receive(&RxBlock { t0: 0.0, iq });
            i += t.block;
        }
    }

    #[test]
    fn the_level_meter_reads_the_same_in_every_mode_filter_and_gain() {
        let (mut t, _log) = trx(144_300_000.0);
        // -60 dBFS at the converter with 0 dB of front-end gain.
        let amp = 1e-3;
        let mut seen = Vec::new();
        for (mode, gain) in [(Mode::Usb, 40.0), (Mode::Cw, 40.0), (Mode::Am, 40.0), (Mode::Nfm, 40.0), (Mode::Usb, 10.0), (Mode::Cw, 65.0)] {
            t.set_mode(mode);
            t.rx_gain_db = gain;
            t.hw_gain_db = gain;
            feed_tone(&mut t, amp, 700.0, gain, 1.5);
            assert_eq!(t.meter.src, "channel");
            // Uncalibrated: dBm = dBFS - G + K_DEFAULT, so dBFS - G is what
            // must not move.
            seen.push((mode, gain, t.meter.dbfs - gain, t.meter.dbm));
        }
        let first = seen[0].2;
        for (mode, gain, v, _) in &seen {
            assert!((v - first).abs() < 0.1, "{mode:?} at {gain} dB: {v} vs {first}");
        }
        // The shared channel filter passes the tone at (nearly) unity gain.
        assert!((first - -60.0).abs() < 0.5, "{first}");
        let dbm: Vec<f64> = seen.iter().map(|x| x.3).collect();
        assert!(dbm.iter().all(|d| (d - dbm[0]).abs() < 0.1), "{dbm:?}");
    }

    #[test]
    fn a_calibration_table_turns_the_reading_into_dbm() {
        use crate::calib::{Calib, KPoint, Quality};
        let (mut t, _log) = trx(144_300_000.0);
        // K such that this tone is -80 dBm: dBm = dBFS - G + K.
        let k = -80.0 - (-60.0);
        let c = Calib { version: 1, k: vec![KPoint { f: 144.3e6, k, src: "test".into(), date: String::new(), t: None }], ..Default::default() };
        t.calib[0] = Some(c);
        for gain in [20.0, 50.0] {
            t.rx_gain_db = gain;
            t.hw_gain_db = gain;
            feed_tone(&mut t, 1e-3, 700.0, gain, 1.5);
            assert!((t.meter.dbm - -80.0).abs() < 0.5, "{gain}: {}", t.meter.dbm);
            assert_eq!(t.meter.quality, Quality::Calibrated);
            assert!(!t.meter.noise);
        }
        // Another socket pair has no table.
        assert!(t.calib[1].is_none());
        let j = t.calib_json();
        assert_eq!(j["ports"][0]["status"][2]["band"], "2m");
        assert_eq!(j["ports"][0]["status"][2]["status"], "calibrated");
        let raw = t.meter_raw(Some(144.3e6 + 200.0), Some(144.3e6 + 1200.0));
        assert!((raw["chan"]["dbfs"].as_f64().unwrap() - (-60.0 + 50.0)).abs() < 0.5, "{raw}");
    }

    #[test]
    fn old_smeter_points_apply_only_without_a_table() {
        use crate::calib::{Calib, KPoint, Quality};
        let (mut t, _log) = trx(144_300_000.0);
        let band = t.cal_band();
        t.settings.add_cal(&band, -100.0, -93.0);
        feed_tone(&mut t, 1e-3, 700.0, 40.0, 1.5);
        assert_eq!(t.meter.quality, Quality::Legacy);
        t.calib[0] = Some(Calib { version: 1, k: vec![KPoint { f: 144.3e6, k: 0.0, src: String::new(), date: String::new(), t: None }], ..Default::default() });
        feed_tone(&mut t, 1e-3, 700.0, 40.0, 1.5);
        assert_eq!(t.meter.quality, Quality::Calibrated);
    }

    #[test]
    fn a_second_source_does_not_take_over() {
        let (mut t, _log) = trx(144_300_000.0);
        t.apply(Command::SetPtt(true));
        t.apply(Command::SetTune(true));
        assert_eq!(t.tx_on, Some(TxSource::Ptt));
        t.apply(Command::SetPtt(false));
        assert!(t.tx_on.is_none());
    }
}
