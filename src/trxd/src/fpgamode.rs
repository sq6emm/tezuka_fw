//! FPGA bitstream per mode (docs/FPGA-MODES.md). The boot bitstream (the
//! FIT's) and the others in /lib/firmware/fpga-<mode>.bin each hold only
//! part of what trxd can use; when a feature needs a part the loaded one
//! lacks, trxd saves what to resume, names the mode it wants and exits: its
//! S80trxd loop runs `fpga-mode <mode>` (the PL reloaded, the drivers bound
//! again) and starts trxd, which takes up the saved state.
//!
//! Modes: "all" (everything; also what a board without mode bitstreams
//! has), "trx" (radio, wide scope, CW-RS network front end), "datv" (radio,
//! wide scope, DVB-S2/T2 receive and transmit, LDPC).

use std::path::Path;

const LOADED: &str = "/run/fpga-mode";
const BOOT: &str = "/etc/fpga-boot-mode";
const FIRMWARE: &str = "/lib/firmware";
const WANT: &str = "/run/fpga-mode.want";
const RESUME: &str = "/run/trxd-resume.json";

/// A part of the FPGA a feature runs on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Part {
    /// DVB-S2/T2 receive and transmit (Maia DDC ring, encoder, IFFT, LDPC).
    Datv,
    /// The CW-RS network's front end (rsnn_front).
    Rsnn,
}

/// The mode of the bitstream loaded now ("boot": the boot one's, from
/// /etc/fpga-boot-mode; "all" when the firmware has no modes).
pub fn loaded() -> String {
    let read = |p: &str| std::fs::read_to_string(p).ok().map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    match read(LOADED) {
        Some(m) if m != "boot" => m,
        _ => read(BOOT).unwrap_or_else(|| "all".into()),
    }
}

fn has(mode: &str, part: Part) -> bool {
    match mode {
        "trx" => part == Part::Rsnn,
        "datv" => part == Part::Datv,
        // "all" and anything unknown: never switch away
        _ => true,
    }
}

/// The mode to load for `part`, if the loaded bitstream lacks it and one
/// that has it is installed.
pub fn switch_for(part: Part) -> Option<&'static str> {
    if has(&loaded(), part) {
        return None;
    }
    let order: &[&'static str] = match part {
        Part::Datv => &["datv", "all"],
        Part::Rsnn => &["trx", "all"],
    };
    order.iter().copied().find(|m| Path::new(FIRMWARE).join(format!("fpga-{m}.bin")).exists())
}

/// Ask for `mode` and exit (the S80trxd loop loads it and starts trxd
/// again, which takes `resume` back with [`take_resume`]).
pub fn request(mode: &str, resume: &serde_json::Value) -> ! {
    let _ = std::fs::write(RESUME, resume.to_string());
    let _ = std::fs::write(WANT, mode);
    tracing::info!(from = %loaded(), to = mode, "FPGA bitstream switch: restarting");
    std::process::exit(0);
}

/// The state saved before a switch (once).
pub fn take_resume() -> Option<serde_json::Value> {
    let s = std::fs::read_to_string(RESUME).ok()?;
    let _ = std::fs::remove_file(RESUME);
    serde_json::from_str(&s).ok()
}
