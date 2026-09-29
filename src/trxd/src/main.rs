//! trxd — headless AD936x transceiver / MGM beacon daemon for tezuka_fw_simple.
//!
//! ```text
//! trxd [--config /etc/trxd.toml] [--sim] [--check]
//! ```
//!
//! `--sim` swaps the AD936x for the simulated radio (run it on a PC);
//! `--check` parses the config, prints it, and exits.

mod rscw;
mod rsnn;
#[cfg(target_os = "linux")]
mod rsnn_fpga;
mod beacon;
mod config;
mod cwlive;
mod settings;
mod temps;
mod decode;
mod dvbs2;
mod dvbt2;
mod keyer;
mod maia;
mod morse;
mod model;
mod mqtt;
mod pace;
mod pi4;
mod radio;
mod refclock;
mod scope;
mod slots;
mod speech;
mod stream;
mod trx;
mod web;

use std::path::PathBuf;
use std::process::ExitCode;

use tracing::{error, info};

use config::{Backend, Config, Role};

fn usage() -> ! {
    eprintln!("usage: trxd [--config FILE] [--sim] [--check] | --pack-model IN.onnx OUT.bin | --bench-deepcw MODEL [N] | --dvbs2-mod IN.ts OUT.cf32 [RATE] [SPS] [pilots] | --datv-mux MEDIA OUT.ts SR [RATE] [pilots] | [--config FILE] --capture-iq OUT.cf32 FREQ_HZ SECONDS [BW_HZ] (stop trxd first)");
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
            "--pack-model" => {
                let (Some(i), Some(o)) = (args.next(), args.next()) else { usage() };
                return match model::pack_cli(&i, &o) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--bench-deepcw" => {
                let Some(m) = args.next() else { usage() };
                let n = args.next().and_then(|v| v.parse().ok()).unwrap_or(5);
                return match model::bench_cli(&m, n) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--dvbs2-mod" => {
                let (Some(i), Some(o)) = (args.next(), args.next()) else { usage() };
                let rest: Vec<String> = args.by_ref().collect();
                return match dvbs2::mod_cli(&i, &o, &rest) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--dvbs2-demod" => {
                let (Some(i), Some(o)) = (args.next(), args.next()) else { usage() };
                let rest: Vec<String> = args.by_ref().collect();
                return match dvbs2::rx::demod_cli(&i, &o, &rest) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--ldpc-file" => {
                let rest: Vec<String> = args.by_ref().collect();
                return match dvbs2::ldpc::file_cli(&rest) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
                        ExitCode::FAILURE
                    }
                };
            }
            "--ldpc-helper" => {
                let rest: Vec<String> = args.by_ref().collect();
                return match dvbs2::ldpc::helper_cli(&rest) {
                    Ok(()) => ExitCode::SUCCESS,
                    Err(e) => {
                        error!("{e}");
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
    // The transceiver always reads CW; a beacon receiver when asked to.
    let wants_cw = cfg.role == Role::Trx || cfg.beacon_rx.decoders.contains(&config::DecoderKind::Cw);
    if wants_cw && cfg.role != Role::BeaconTx {
        model::install(&cfg.cw_model);
    }
    let rate = radio.control.stream_rate();
    let rx = stream::spawn_rx(radio.rx, rate, cfg.radio.buffer_samples);
    let tx = stream::spawn_tx(radio.tx, cfg.radio.buffer_samples);
    let mqtt = mqtt::Mqtt::start(&cfg);
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
        let quiet = if cfg.role == Role::BeaconTx { beacon_tail } else { always };
        refclock::spawn(cfg.reference.clone(), mqtt.publisher(), quiet);
    }

    match cfg.role {
        Role::Trx => {
            let web = web::start(&cfg.web);
            trx::Trx::new(cfg, radio.control, tx, mqtt, web).run(rx)
        }
        Role::BeaconTx => beacon::run_tx(cfg, radio.control, rx, tx, mqtt),
        Role::BeaconRx => beacon::run_rx(cfg, radio.control, rx, tx, mqtt),
    }
    ExitCode::SUCCESS
}
