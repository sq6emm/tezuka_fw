//! The DeepCW model lives outside the firmware image: a raw flash partition
//! labelled `model` (see docs/FLASH.md), shared by both A/B slots, because it
//! is 15 MB of float32 and the slots are 12 MB each.
//!
//! Blob format (little endian):
//!
//! ```text
//! 0   4   magic "DCW1"
//! 4   4   length of the xz stream that follows the header
//! 8   4   length of the decompressed ONNX model
//! 12  32  SHA-256 of the decompressed ONNX model
//! 44  ..  xz stream
//! ```
//!
//! The model's weights are rounded to bfloat16 precision (still stored as
//! float32) before packing: the low 16 bits of every weight become zero and
//! xz squeezes 15 MB down to 5.3 MB, and DeepCW's accuracy matrix (WPM x SNR)
//! is unchanged within the test's noise.

use std::io::Read;
use std::path::Path;

use sha2::{Digest, Sha256};
use tracing::{info, warn};

pub const MAGIC: &[u8; 4] = b"DCW1";
const HEADER: usize = 44;

/// Parse and verify a blob, returning the ONNX bytes.
pub fn unpack(blob: &[u8]) -> Result<Vec<u8>, String> {
    if blob.len() < HEADER || &blob[..4] != MAGIC {
        return Err("no DeepCW model blob (bad magic)".into());
    }
    let u32_at = |o: usize| u32::from_le_bytes(blob[o..o + 4].try_into().unwrap()) as usize;
    let (xz_len, onnx_len) = (u32_at(4), u32_at(8));
    let want: [u8; 32] = blob[12..44].try_into().unwrap();
    let xz = blob.get(HEADER..HEADER + xz_len).ok_or("model blob truncated")?;
    let mut onnx = Vec::with_capacity(onnx_len);
    lzma_rs::xz_decompress(&mut std::io::BufReader::new(xz), &mut onnx).map_err(|e| format!("model xz: {e}"))?;
    if onnx.len() != onnx_len || Sha256::digest(&onnx).as_slice() != want {
        return Err("model checksum mismatch".into());
    }
    Ok(onnx)
}

/// Build a blob from ONNX bytes and their xz compression (made by the build,
/// with `xz -9e`, which beats what we could do here).
pub fn pack(onnx: &[u8], xz: &[u8]) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER + xz.len());
    b.extend_from_slice(MAGIC);
    b.extend_from_slice(&(xz.len() as u32).to_le_bytes());
    b.extend_from_slice(&(onnx.len() as u32).to_le_bytes());
    b.extend_from_slice(&Sha256::digest(onnx));
    b.extend_from_slice(xz);
    b
}

/// Round every float32 in the ONNX model's initializers to bfloat16 precision.
///
/// Walks the protobuf just far enough to find `GraphProto.initializer`
/// (field 5 of `ModelProto.graph`, field 7) tensors whose `data_type` is FLOAT
/// and rewrites their `raw_data` (field 9) in place. Lengths never change, so
/// the rest of the file is copied untouched.
pub fn round_bf16(onnx: &mut [u8]) -> Result<usize, String> {
    fn varint(b: &[u8], p: &mut usize) -> Result<u64, String> {
        let mut v = 0u64;
        for shift in (0..64).step_by(7) {
            let byte = *b.get(*p).ok_or("truncated varint")?;
            *p += 1;
            v |= ((byte & 0x7f) as u64) << shift;
            if byte & 0x80 == 0 {
                return Ok(v);
            }
        }
        Err("bad varint".into())
    }
    /// Visit length-delimited field `want` of the message in `range`.
    fn fields(b: &[u8], range: (usize, usize), want: u64, out: &mut Vec<(usize, usize)>) -> Result<(), String> {
        let mut p = range.0;
        while p < range.1 {
            let key = varint(b, &mut p)?;
            let (field, wire) = (key >> 3, key & 7);
            match wire {
                0 => {
                    varint(b, &mut p)?;
                }
                1 => p += 8,
                5 => p += 4,
                2 => {
                    let len = varint(b, &mut p)? as usize;
                    if field == want {
                        out.push((p, p + len));
                    }
                    p += len;
                }
                _ => return Err(format!("unsupported wire type {wire}")),
            }
        }
        Ok(())
    }
    fn data_type(b: &[u8], range: (usize, usize)) -> Result<u64, String> {
        let mut p = range.0;
        while p < range.1 {
            let key = varint(b, &mut p)?;
            match (key >> 3, key & 7) {
                (2, 0) => return varint(b, &mut p),
                (_, 0) => {
                    varint(b, &mut p)?;
                }
                (_, 1) => p += 8,
                (_, 5) => p += 4,
                (_, 2) => {
                    let len = varint(b, &mut p)? as usize;
                    p += len;
                }
                (_, w) => return Err(format!("unsupported wire type {w}")),
            }
        }
        Ok(0)
    }
    let mut graphs = Vec::new();
    fields(onnx, (0, onnx.len()), 7, &mut graphs)?;
    let mut inits = Vec::new();
    for g in graphs {
        fields(onnx, g, 5, &mut inits)?;
    }
    let mut n = 0;
    for t in inits {
        if data_type(onnx, t)? != 1 {
            continue; // not FLOAT
        }
        let mut raws = Vec::new();
        fields(onnx, t, 9, &mut raws)?;
        for (a, z) in raws {
            for w in onnx[a..z].chunks_exact_mut(4) {
                let u = u32::from_le_bytes(w.try_into().unwrap()) as u64;
                let r = ((u + 0x7fff + ((u >> 16) & 1)) & 0xffff_0000) as u32;
                w.copy_from_slice(&r.to_le_bytes());
                n += 1;
            }
        }
    }
    Ok(n)
}

/// `/dev/mtdN` for the partition labelled `label` in /proc/mtd.
fn mtd_by_label(label: &str) -> Option<String> {
    let t = std::fs::read_to_string("/proc/mtd").ok()?;
    t.lines().find_map(|l| {
        let (dev, rest) = l.split_once(':')?;
        rest.contains(&format!("\"{label}\"")).then(|| format!("/dev/{dev}"))
    })
}

/// Find, verify and install the model for DeepCW. `source` is "auto" (the
/// `model` flash partition), or a path to a blob or a plain `.onnx` file.
/// Failure only disables neural CW decoding; everything else runs on.
pub fn install(source: &str) -> bool {
    let path = match source {
        "" | "auto" => match mtd_by_label("model") {
            Some(p) => p,
            None => {
                warn!("no 'model' flash partition: neural CW decoding disabled");
                return false;
            }
        },
        p => p.to_string(),
    };
    let started = std::time::Instant::now();
    let onnx = match read_model(Path::new(&path)) {
        Ok(o) => o,
        Err(e) => {
            warn!("DeepCW model from {path}: {e}; neural CW decoding disabled");
            return false;
        }
    };
    let len = onnx.len();
    // The model is needed for the life of the process; rten borrows it 'static.
    let ok = sdroxide_deepcw::set_weights(Box::leak(onnx.into_boxed_slice()));
    info!(%path, bytes = len, ms = started.elapsed().as_millis() as u64, "DeepCW model loaded");
    ok
}

fn read_model(path: &Path) -> Result<Vec<u8>, String> {
    let mut f = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut head = [0u8; HEADER];
    f.read_exact(&mut head).map_err(|e| e.to_string())?;
    if &head[..4] == MAGIC {
        let xz_len = u32::from_le_bytes(head[4..8].try_into().unwrap()) as usize;
        let mut blob = head.to_vec();
        blob.resize(HEADER + xz_len, 0);
        f.read_exact(&mut blob[HEADER..]).map_err(|e| format!("reading {xz_len} bytes: {e}"))?;
        unpack(&blob)
    } else {
        let mut all = head.to_vec();
        f.read_to_end(&mut all).map_err(|e| e.to_string())?;
        Ok(all)
    }
}

/// `trxd --pack-model IN.onnx OUT.bin`: round to bf16, xz, add the header.
pub fn pack_cli(input: &str, output: &str) -> Result<(), String> {
    let mut onnx = std::fs::read(input).map_err(|e| format!("{input}: {e}"))?;
    let n = round_bf16(&mut onnx)?;
    let mut child = std::process::Command::new("xz")
        .args(["-9e", "-T0", "-c"])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("xz: {e}"))?;
    let mut stdin = child.stdin.take().unwrap();
    let data = onnx.clone();
    let writer = std::thread::spawn(move || std::io::Write::write_all(&mut stdin, &data));
    let out = child.wait_with_output().map_err(|e| e.to_string())?;
    writer.join().unwrap().map_err(|e| e.to_string())?;
    if !out.status.success() {
        return Err("xz failed".into());
    }
    let blob = pack(&onnx, &out.stdout);
    unpack(&blob)?; // self-check: this is exactly what the board will do
    std::fs::write(output, &blob).map_err(|e| format!("{output}: {e}"))?;
    eprintln!("rounded {n} weights to bf16; {} -> {} bytes", onnx.len(), blob.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_round_trip_and_corruption() {
        let onnx = b"not really onnx, but bytes".to_vec();
        let mut xz = Vec::new();
        lzma_rs::xz_compress(&mut std::io::Cursor::new(&onnx), &mut xz).unwrap();
        let blob = pack(&onnx, &xz);
        assert_eq!(unpack(&blob).unwrap(), onnx);
        let mut bad = blob.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert!(unpack(&bad).is_err());
        assert!(unpack(b"XXXX").is_err());
    }

    #[test]
    fn bf16_rounding_finds_float_initializers() {
        // ModelProto{ graph(7){ initializer(5){ data_type(2)=1, raw_data(9)=[1.1f32] } } }
        let raw = 1.1f32.to_le_bytes();
        let tensor = [&[0x10, 0x01][..], &[0x4a, 4], &raw].concat();
        let graph = [&[0x2a, tensor.len() as u8][..], &tensor].concat();
        let mut model = [&[0x3a, graph.len() as u8][..], &graph].concat();
        assert_eq!(round_bf16(&mut model).unwrap(), 1);
        let got = f32::from_le_bytes(model[model.len() - 4..].try_into().unwrap());
        assert_eq!(got.to_bits() & 0xffff, 0);
        assert!((got - 1.1).abs() < 0.01);
    }
}

/// `trxd --bench-deepcw MODEL [N]`: load a model (blob or .onnx), decode a
/// 6 s synthetic CW window N times and print the time per decode and the text,
/// to compare models (e.g. an int8 quantised one) on the board itself.
pub fn bench_cli(source: &str, n: usize) -> Result<(), String> {
    if !install(source) {
        return Err(format!("{source}: model not loaded"));
    }
    let rate = sdroxide_deepcw::SAMPLE_RATE;
    let text = "CQ CQ DE SQ6EMM K";
    let mut tx = sdroxide_dsp::CwTx::new(rate, sdroxide_deepcw::CENTER_FREQ_HZ as f64, 18.0);
    tx.push_text(text);
    let mut audio = Vec::new();
    while !tx.drained() {
        let mut blk = [0.0f32; 256];
        tx.next_block(&mut blk);
        audio.extend_from_slice(&blk);
    }
    audio.resize((6.0 * rate) as usize, 0.0);
    let mut seed = 0x1234_5678u32;
    for a in &mut audio {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *a = *a * 0.5 + ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.05;
    }
    let mut dec = sdroxide_deepcw::Decoder::new().map_err(|e| e.to_string())?;
    let mut times = Vec::new();
    let mut out = String::new();
    for _ in 0..n.max(1) {
        let t = std::time::Instant::now();
        let d = dec.decode(&audio).map_err(|e| e.to_string())?;
        times.push(t.elapsed().as_secs_f64() * 1e3);
        out = d.normalized();
    }
    times.sort_by(|a, b| a.total_cmp(b));
    println!(
        "model {source}: 6 s window, {} decodes, median {:.0} ms, min {:.0} ms; sent {text:?}, read {out:?}",
        times.len(),
        times[times.len() / 2],
        times[0]
    );
    Ok(())
}
