//! RADE V2 (FreeDV's neural voice mode) runs in the browser, not here: the
//! page loads `rade.wasm` (src/rade-web) into a Worker and decodes the USB
//! audio it already gets, and transmits through the web microphone. trxd
//! only serves the module, keeps the shared RADE flag and drains the
//! microphone queue at the end of an over (the EOO frame).
//!
//! The module is 3.3 MB gzipped, too big for the firmware slots, so it lives
//! in the raw flash partition labelled `model` (docs/FLASH.md), shared by both
//! A/B slots. Blob format (little endian):
//!
//! ```text
//! 0   4   magic "RAD1"
//! 4   4   length of the gzip stream that follows the header
//! 8   32  SHA-256 of the gzip stream
//! 40  ..  rade.wasm, gzip
//! ```
//!
//! On a PC, `[web] rade_wasm` names the blob or a plain `rade.wasm.gz`.

use std::io::Read;

use sha2::{Digest, Sha256};
use tracing::{info, warn};

pub const MAGIC: &[u8; 4] = b"RAD1";
const HEADER: usize = 40;
/// The `model` partition is 5.5 MB.
const MAX_GZ: usize = 0x580000 - HEADER;

/// The gzipped module and a short hash of it (the page's cache key).
pub struct Wasm {
    pub gz: Vec<u8>,
    pub tag: String,
}

impl Wasm {
    fn new(gz: Vec<u8>) -> Self {
        let tag = Sha256::digest(&gz)[..6].iter().map(|b| format!("{b:02x}")).collect();
        Wasm { gz, tag }
    }
}

/// Parse and verify a blob (from its start; trailing erased flash is fine).
pub fn unpack(blob: &[u8]) -> Result<Vec<u8>, String> {
    if blob.len() < HEADER || &blob[..4] != MAGIC {
        return Err("no RADE blob (bad magic)".into());
    }
    let len = u32::from_le_bytes(blob[4..8].try_into().unwrap()) as usize;
    if len > MAX_GZ {
        return Err("RADE blob length out of range".into());
    }
    let gz = blob.get(HEADER..HEADER + len).ok_or("RADE blob truncated")?;
    if Sha256::digest(gz).as_slice() != &blob[8..40] {
        return Err("RADE blob checksum mismatch".into());
    }
    Ok(gz.to_vec())
}

/// Build a blob from `rade.wasm.gz` (the firmware build does the same in
/// board/tezuka/common/pack-rade.sh).
pub fn pack(gz: &[u8]) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER + gz.len());
    b.extend_from_slice(MAGIC);
    b.extend_from_slice(&(gz.len() as u32).to_le_bytes());
    b.extend_from_slice(&Sha256::digest(gz));
    b.extend_from_slice(gz);
    b
}

/// The `model` partition's device, from /proc/mtd.
fn model_mtd() -> Option<String> {
    let t = std::fs::read_to_string("/proc/mtd").ok()?;
    t.lines().find(|l| l.ends_with("\"model\"")).and_then(|l| l.split(':').next()).map(|d| format!("/dev/{d}"))
}

fn from_bytes(b: &[u8]) -> Result<Vec<u8>, String> {
    if b.starts_with(MAGIC) {
        unpack(b)
    } else if b.starts_with(&[0x1f, 0x8b]) {
        Ok(b.to_vec())
    } else {
        Err("neither a RADE blob nor gzip".into())
    }
}

/// Find the module: the configured file, else the `model` partition.
pub fn load(path: &str) -> Option<Wasm> {
    let r = if !path.is_empty() {
        std::fs::read(path).map_err(|e| format!("{path}: {e}")).and_then(|b| from_bytes(&b))
    } else if let Some(dev) = model_mtd() {
        // Read the header first: an erased (0xff) partition costs nothing.
        let mut f = match std::fs::File::open(&dev) {
            Ok(f) => f,
            Err(e) => {
                warn!("RADE: {dev}: {e}");
                return None;
            }
        };
        let mut head = [0u8; HEADER];
        if f.read_exact(&mut head).is_err() || &head[..4] != MAGIC {
            info!("RADE: no module in the model partition; the page has no RADE mode");
            return None;
        }
        let len = (u32::from_le_bytes(head[4..8].try_into().unwrap()) as usize).min(MAX_GZ);
        let mut blob = head.to_vec();
        blob.resize(HEADER + len, 0);
        f.read_exact(&mut blob[HEADER..]).map_err(|e| format!("{dev}: {e}")).and_then(|_| unpack(&blob))
    } else {
        return None;
    };
    match r {
        Ok(gz) => {
            let w = Wasm::new(gz);
            info!(bytes = w.gz.len(), tag = %w.tag, "RADE module for the web page");
            Some(w)
        }
        Err(e) => {
            warn!("RADE: {e}");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blob_round_trip() {
        let gz = vec![0x1f, 0x8b, 8, 0, 1, 2, 3, 4, 5];
        let mut b = pack(&gz);
        assert_eq!(unpack(&b).unwrap(), gz);
        // Erased flash after the blob is ignored.
        b.extend_from_slice(&[0xff; 64]);
        assert_eq!(from_bytes(&b).unwrap(), gz);
        assert_eq!(from_bytes(&gz).unwrap(), gz);
        b[HEADER] ^= 1;
        assert!(unpack(&b).is_err());
        assert!(unpack(&[0xff; 64]).is_err());
    }
}
