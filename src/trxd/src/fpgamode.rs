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
    // the boot mode's comes out of the running slot's FIT (fpga-mode)
    let boot = std::fs::read_to_string(BOOT).ok().map(|s| s.trim().to_string());
    order.iter().copied().find(|m| boot.as_deref() == Some(*m) || Path::new(FIRMWARE).join(format!("fpga-{m}.bin")).exists())
}

/// Ask for `mode` and exit (the S80trxd loop loads it and starts trxd
/// again, which takes `resume` back with [`take_resume`]).
pub fn request(mode: &str, resume: &serde_json::Value) -> ! {
    let _ = std::fs::write(RESUME, resume.to_string());
    let _ = std::fs::write(WANT, mode);
    tracing::info!(from = %loaded(), to = mode, "FPGA bitstream switch: restarting");
    // The bus masters in the PL to rest before it is reconfigured (fpga-mode
    // does it again, and the rest: the spectrometer, the T2 router, the
    // CW-RS network). exit() runs no destructors: the front end's would
    // not stop the ring.
    if loaded() != "trx" {
        crate::dvbs2::fpga_tx::quiesce();
        crate::dvbs2::fpga::quiesce();
        crate::dvbs2::fpga_ldpc::quiesce();
    }
    std::process::exit(0);
}

/// The state saved before a switch (once).
pub fn take_resume() -> Option<serde_json::Value> {
    let s = std::fs::read_to_string(RESUME).ok()?;
    let _ = std::fs::remove_file(RESUME);
    serde_json::from_str(&s).ok()
}

/// A property of a node in a flattened device tree (a FIT image is one):
/// `path` like "/images/fpga@1".
pub fn fdt_prop<'a>(fdt: &'a [u8], path: &str, prop: &str) -> Result<&'a [u8], String> {
    let be = |at: usize| -> Result<usize, String> {
        fdt.get(at..at + 4).map(|b| u32::from_be_bytes(b.try_into().unwrap()) as usize).ok_or_else(|| "truncated".to_string())
    };
    if be(0)? != 0xd00d_feed {
        return Err("not a device tree / FIT".into());
    }
    let (off_struct, off_strings) = (be(8)?, be(12)?);
    let want: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
    let mut stack: Vec<String> = Vec::new();
    let mut at = off_struct;
    loop {
        match be(at)? {
            1 => {
                let name_end = fdt[at + 4..].iter().position(|&c| c == 0).ok_or("bad node name")? + at + 4;
                let name = String::from_utf8_lossy(&fdt[at + 4..name_end]).into_owned();
                if at != off_struct {
                    stack.push(name);
                }
                at = (name_end + 1).div_ceil(4) * 4;
            }
            2 => {
                stack.pop();
                at += 4;
            }
            3 => {
                let (len, nameoff) = (be(at + 4)?, be(at + 8)?);
                let data = at + 12;
                let s = off_strings + nameoff;
                let end = fdt[s..].iter().position(|&c| c == 0).ok_or("bad string")? + s;
                if stack.len() == want.len() && stack.iter().zip(&want).all(|(a, b)| a == b) && &fdt[s..end] == prop.as_bytes() {
                    return fdt.get(data..data + len).ok_or_else(|| "truncated property".to_string());
                }
                at = (data + len).div_ceil(4) * 4;
            }
            4 => at += 4,
            9 => return Err(format!("{path} {prop}: not found")),
            t => return Err(format!("bad token {t}")),
        }
    }
}

/// `trxd --fit-data FILE PATH [PROP]`: a FIT property (default "data") to
/// stdout (fpga-mode: the boot bitstream from the running slot's FIT, gzip).
pub fn fit_data_cli(file: &str, path: &str, prop: &str) -> Result<(), String> {
    use std::io::{Read, Write};
    let mut f = std::fs::File::open(file).map_err(|e| format!("{file}: {e}"))?;
    let mut head = [0u8; 8];
    f.read_exact(&mut head).map_err(|e| format!("{file}: {e}"))?;
    let total = u32::from_be_bytes(head[4..8].try_into().unwrap()) as usize;
    if total > 64 << 20 {
        return Err(format!("{file}: {total} bytes?"));
    }
    let mut fdt = head.to_vec();
    fdt.resize(total, 0);
    f.read_exact(&mut fdt[8..]).map_err(|e| format!("{file}: {e}"))?;
    let d = fdt_prop(&fdt, path, prop)?;
    std::io::stdout().lock().write_all(d).map_err(|e| e.to_string())
}

/// A Xilinx .bit to the byte-swapped .bin the zynq fpga manager takes (from
/// the first dummy word; board/tezuka/common/bit2bin.py likewise).
pub fn bit2bin(bit: &[u8]) -> Result<Vec<u8>, String> {
    let find = |pat: &[u8]| bit.windows(4).position(|w| w == pat);
    let (Some(i), Some(sync)) = (find(&[0xff; 4]), find(&[0xaa, 0x99, 0x55, 0x66])) else {
        return Err("no sync word".into());
    };
    if i >= sync {
        return Err("no sync word".into());
    }
    let mut d = bit[i..].to_vec();
    d.resize(d.len().div_ceil(4) * 4, 0);
    for w in d.chunks_exact_mut(4) {
        w.reverse();
    }
    Ok(d)
}

/// `trxd --bit2bin`: stdin (.bit) to stdout (.bin).
pub fn bit2bin_cli() -> Result<(), String> {
    use std::io::{Read, Write};
    let mut bit = Vec::new();
    std::io::stdin().lock().read_to_end(&mut bit).map_err(|e| e.to_string())?;
    let bin = bit2bin(&bit)?;
    std::io::stdout().lock().write_all(&bin).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tiny FIT-like tree: / { images { fpga@1 { data = <..>; }; }; };
    fn tree() -> Vec<u8> {
        let mut st: Vec<u8> = Vec::new();
        let tok = |v: &mut Vec<u8>, t: u32| v.extend(t.to_be_bytes());
        let name = |v: &mut Vec<u8>, n: &str| {
            v.extend(n.as_bytes());
            v.push(0);
            while v.len() % 4 != 0 {
                v.push(0);
            }
        };
        let strings = b"description\0data\0".to_vec();
        tok(&mut st, 1);
        name(&mut st, "");
        tok(&mut st, 1);
        name(&mut st, "images");
        tok(&mut st, 1);
        name(&mut st, "fpga@1");
        for (off, val) in [(0u32, &b"x\0"[..]), (12, &[1, 2, 3, 4, 5][..])] {
            tok(&mut st, 3);
            tok(&mut st, val.len() as u32);
            tok(&mut st, off);
            st.extend(val);
            while st.len() % 4 != 0 {
                st.push(0);
            }
        }
        tok(&mut st, 2);
        tok(&mut st, 2);
        tok(&mut st, 2);
        tok(&mut st, 9);
        let off_struct = 40usize;
        let off_strings = off_struct + st.len();
        let total = off_strings + strings.len();
        let mut f = Vec::new();
        for v in [0xd00d_feedu32, total as u32, off_struct as u32, off_strings as u32, 0, 17, 16, 0, strings.len() as u32, st.len() as u32] {
            f.extend(v.to_be_bytes());
        }
        f.extend(st);
        f.extend(strings);
        f
    }

    #[test]
    fn fit_property() {
        let t = tree();
        assert_eq!(fdt_prop(&t, "/images/fpga@1", "data").unwrap(), &[1, 2, 3, 4, 5]);
        assert!(fdt_prop(&t, "/images/fpga@2", "data").is_err());
    }

    #[test]
    fn bit_to_bin() {
        let mut bit = b"header junk".to_vec();
        bit.extend([0xff, 0xff, 0xff, 0xff, 0xaa, 0x99, 0x55, 0x66, 1, 2, 3, 4]);
        assert_eq!(bit2bin(&bit).unwrap(), vec![0xff, 0xff, 0xff, 0xff, 0x66, 0x55, 0x99, 0xaa, 4, 3, 2, 1]);
    }
}
