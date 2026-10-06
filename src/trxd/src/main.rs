//! trxd — headless AD936x transceiver / MGM beacon daemon for tezuka_fw_simple.
//!
//! ```text
//! trxd [--config /etc/trxd.toml] [--sim] [--check]
//! ```
//!
//! `--sim` swaps the AD936x for the simulated radio (run it on a PC);
//! `--check` parses the config, prints it, and exits.

mod fpgamode;
#[cfg(target_os = "linux")]
mod beacon;
mod config;
mod calib;
mod cwlive;
mod settings;
mod temps;
mod decode;
mod dvbs2;
mod dvbt2;
mod keyer;
mod maia;
mod morse;
mod pace;
mod pi4;
mod rade;
mod power;
mod radio;
mod refclock;
mod safety;
mod scope;
mod scopeplan;
mod slots;
mod speech;
mod stream;
mod trx;
mod web;
mod wfm;

use std::path::PathBuf;
use std::process::ExitCode;

use tracing::{error, info};

use config::{Backend, Config, Role};

fn usage() -> ! {
    eprintln!("usage: trxd [--config FILE] [--sim] [--check] | --datv-mux MEDIA OUT.ts SR [MODE] | [--config FILE] --capture-iq OUT.cf32 FREQ_HZ SECONDS [BW_HZ] (stop trxd first)");
    std::process::exit(2);
}

/// Raw receive at the full converter rate (the FPGA decimator off, the RX
/// filter opened to `bw`) into complex f32 LE: for checking wide signals
/// such as DVB-T2 offline. trxd must not be running.
fn capture_iq(cfg: &mut Config, out: &str, freq: f64, secs: f64, bw: u32) -> Result<(), String> {
    use std::io::Write;
    cfg.radio.fpga_decimation = false;
    cfg.radio.rf_bandwidth = bw;
    let mut radio = radio::open(&cfg.radio, freq)?;
    let rate = radio.control.stream_rate();
    let mut f = std::io::BufWriter::new(std::fs::File::create(out).map_err(|e| format!("{out}: {e}"))?);
    let mut buf = vec![num_complex::Complex32::default(); cfg.radio.buffer_samples];
    // Let the AGC and the stream settle.
    for _ in 0..8 {
        radio.rx.read(&mut buf)?;
    }
    let total = (secs * rate) as usize;
    let mut n = 0;
    while n < total {
        radio.rx.read(&mut buf)?;
        for z in &buf {
            f.write_all(&z.re.to_le_bytes()).and_then(|_| f.write_all(&z.im.to_le_bytes())).map_err(|e| e.to_string())?;
        }
        n += buf.len();
    }
    info!(samples = n, rate, freq, "captured to {out}");
    Ok(())
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_target(false)
        .with_writer(std::io::stderr)
        .init();

    let mut path = PathBuf::from("/etc/trxd.toml");
    let mut sim = false;
    let mut check = false;
    let mut capture: Option<(String, f64, f64, u32)> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" | "-c" => path = args.next().map(PathBuf::from).unwrap_or_else(|| usage()),
            "--sim" => sim = true,
            "--check" => check = true,
            "--fit-data" => {
                let (Some(f), Some(p)) = (args.next(), args.next()) else { usage() };
                let prop = args.next().unwrap_or_else(|| "data".into());
                return match fpgamode::fit_data_cli(&f, &p, &prop) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        eprintln!("trxd --fit-data: {e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--bit2bin" => {
                return match fpgamode::bit2bin_cli() {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        eprintln!("trxd --bit2bin: {e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--datv-mux" => {
                let (Some(i), Some(o)) = (args.next(), args.next()) else { usage() };
                let rest: Vec<String> = args.by_ref().collect();
                return match dvbs2::ts::mux_cli(&i, &o, &rest) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--capture-iq" => {
                let (Some(o), Some(f), Some(t)) = (args.next(), args.next(), args.next()) else { usage() };
                let (Ok(f), Ok(t)) = (f.parse::<f64>(), t.parse::<f64>()) else { usage() };
                let bw = args.next().and_then(|b| b.parse::<u32>().ok()).unwrap_or(2_400_000);
                capture = Some((o, f, t, bw));
            }
            "--ring-bench" => {
                return match dvbs2::fpga::ring_bench() {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--version" | "-V" => {
                println!("trxd {}", env!("CARGO_PKG_VERSION"));
                return ExitCode::SUCCESS;
            }
            _ => usage(),
        }
    }

    let mut cfg = if path.exists() {
        match Config::load(&path) {
            Ok(c) => c,
            Err(e) => {
                error!("{e}");
                return ExitCode::FAILURE;
            }
        }
    } else {
        info!("{} not found, using defaults", path.display());
        Config::default()
    };
    if sim {
        cfg.radio.backend = Backend::Sim;
    }
    // The converter rate the loaded FPGA image is built for (the wide trx
    // image: 24.576 MS/s, x64 to the same 384 kS/s stream).
    if cfg.radio.backend == Backend::Iio {
        if let Some((rate, decim)) = fpgamode::rate(&fpgamode::loaded()) {
            info!(rate, decim, "converter rate from the FPGA image");
            cfg.radio.adc_rate = rate;
            cfg.radio.fpga_decim = decim;
        }
    }
    dvbs2::fpga::set_fs_in(cfg.radio.adc_rate as f64);
    if check {
        println!("{cfg:#?}");
        return ExitCode::SUCCESS;
    }
    if let Some((out, freq, secs, bw)) = capture {
        return match capture_iq(&mut cfg, &out, freq, secs, bw) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                error!("{e}");
                ExitCode::FAILURE
            }
        };
    }

    // RF off on a panic, a signal or a stalled engine (before any thread).
    safety::install();
    let initial_lo = match cfg.role {
        Role::Trx => cfg.trx.freq_hz,
        Role::BeaconTx => cfg.beacon.freq_hz,
        Role::BeaconRx => cfg.beacon_rx.freq_hz - cfg.beacon_rx.audio_offset_hz,
    } - cfg.radio.lo_offset_hz;
    let radio = match radio::open(&cfg.radio, initial_lo) {
        Ok(r) => r,
        Err(e) => {
            error!("radio: {e}");
            return ExitCode::FAILURE;
        }
    };
    let rate = radio.control.stream_rate();
    let rx = stream::spawn_rx(radio.rx, rate, cfg.radio.buffer_samples);
    let tx = stream::spawn_tx(radio.tx, cfg.radio.buffer_samples);
    if cfg.mqtt.is_some() {
        tracing::warn!("the [mqtt] section is ignored: MQTT was removed (decodes and reports go to the log)");
    }
    info!(role = ?cfg.role, call = %cfg.callsign, "trxd {}", env!("CARGO_PKG_VERSION"));

    // A reference correction retunes the synthesizers for a few ms. A beacon
    // takes it only in its carrier tail (seconds 56..58), never mid-symbol.
    fn always() -> bool {
        true
    }
    fn beacon_tail() -> bool {
        let s = stream::unix_now() % 60.0;
        (56.0..58.5).contains(&s)
    }
    if cfg.radio.backend == Backend::Iio {
        let apply = match cfg.role {
            Role::Trx => refclock::Apply::Engine,
            Role::BeaconTx => refclock::Apply::Direct(beacon_tail),
            Role::BeaconRx => refclock::Apply::Direct(always),
        };
        refclock::spawn(cfg.reference.clone(), apply);
    }

    match cfg.role {
        Role::Trx => {
            let web = web::start(&cfg.web);
            trx::Trx::new(cfg, radio.control, tx, web).run(rx)
        }
        Role::BeaconTx => beacon::run_tx(cfg, radio.control, rx, tx),
        Role::BeaconRx => beacon::run_rx(cfg, radio.control, rx, tx),
    }
    ExitCode::SUCCESS
}
