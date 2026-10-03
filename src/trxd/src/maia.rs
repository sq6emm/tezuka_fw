//! Maia SDR's FPGA spectrometer as the web UI's wide scope.
//!
//! The simple bitstream keeps only Maia's spectrometer (see
//! maia-hdl/projects/simple/maia_scope.tcl): it windows, FFTs (4096 points)
//! and averages the full-rate ADC stream in the fabric and DMAs each finished
//! spectrum into a ring of 8 buffers at 0x16000000, raising an interrupt per
//! buffer. The ARM only converts 4096 numbers per row — the whole 3 MHz the
//! AD936x delivers, for free.
//!
//! Registers (maia-sdr.svd): 0x00 product id "maia", 0x08 control
//! (bit 0 sdr_reset), 0x0C interrupts (bit 0 spectrometer, read clears),
//! 0x20 spectrometer (bit 0 use_ddc_out, 10:1 num_integrations, 11 abort,
//! 14:12 last_buffer, 15 peak_detect).
//!
//! Bins arrive FFT-shifted (DC in the middle). Each is a u64 "float": a
//! 47-bit mantissa and a base-4 exponent in bits 57:56; bin 0 carries a
//! fastlock flag instead of a power and is blanked.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use crossbeam_channel::{Receiver, bounded};
use tracing::{info, warn};

const RING_PHYS: u64 = 0x1600_0000;
const BUFFERS: usize = 8;
pub const BINS: usize = 4096;
const RING_BYTES: usize = BUFFERS * BINS * 8;

const REG_CONTROL: usize = 0x08;
const REG_INTERRUPTS: usize = 0x0C;
const REG_SPECTROMETER: usize = 0x20;

struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: accessed only by the owning reader thread, with volatile reads.
unsafe impl Send for Mapping {}

impl Mapping {
    fn new(f: &File, len: usize, offset: u64) -> Result<Mapping, String> {
        // SAFETY: a MAP_SHARED mapping of a device / reserved-memory region
        // we own for the life of the process; accesses stay within `len`.
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                f.as_raw_fd(),
                offset as libc::off_t,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error().to_string());
        }
        Ok(Mapping { ptr: p.cast(), len })
    }
    fn rd32(&self, off: usize) -> u32 {
        debug_assert!(off + 4 <= self.len);
        // SAFETY: aligned, in bounds (see above).
        unsafe { std::ptr::read_volatile(self.ptr.add(off).cast::<u32>()) }
    }
    fn wr32(&self, off: usize, v: u32) {
        // SAFETY: as rd32.
        unsafe { std::ptr::write_volatile(self.ptr.add(off).cast::<u32>(), v) }
    }
    fn rd64(&self, off: usize) -> u64 {
        // SAFETY: as rd32; the ring is 8-byte aligned.
        unsafe { std::ptr::read_volatile(self.ptr.add(off).cast::<u64>()) }
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: unmapping exactly what new() mapped.
        unsafe {
            libc::munmap(self.ptr.cast(), self.len);
        }
    }
}

fn find_uio(name: &str) -> Option<String> {
    for e in std::fs::read_dir("/sys/class/uio").ok()?.flatten() {
        let n = std::fs::read_to_string(e.path().join("name")).unwrap_or_default();
        if n.trim() == name {
            return Some(format!("/dev/{}", e.file_name().to_string_lossy()));
        }
    }
    None
}

/// Decode one bin of Maia's u64 "floating point" power.
pub fn decode_bin(x: u64) -> f32 {
    let exponent = ((x >> 56) & 3) as u32;
    let value = x & ((1u64 << 47) - 1);
    (value << (2 * exponent)) as f32
}

/// Start the spectrometer and a thread reading it. Rows come out as linear
/// powers, FFT-shifted, normalised so a full-scale tone reads about 0 dBFS
/// (approximately: the IP's internal scaling is fixed-point).
pub fn start(adc_rate: f64, rows_per_s: f64) -> Option<Receiver<Vec<f32>>> {
    let uio_path = find_uio("maia-sdr")?;
    let run = || -> Result<(File, Mapping, Mapping), String> {
        let uio = OpenOptions::new().read(true).write(true).open(&uio_path).map_err(|e| format!("{uio_path}: {e}"))?;
        let regs = Mapping::new(&uio, 4096, 0).map_err(|e| format!("map registers: {e}"))?;
        let mem = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC) // uncached: the DMA writes behind the CPU's back
            .open("/dev/mem")
            .map_err(|e| format!("/dev/mem: {e}"))?;
        let ring = Mapping::new(&mem, RING_BYTES, RING_PHYS).map_err(|e| format!("map spectrum ring: {e}"))?;
        Ok((uio, regs, ring))
    };
    let (mut uio, regs, ring) = match run() {
        Ok(v) => v,
        Err(e) => {
            warn!("Maia spectrometer unavailable: {e}");
            return None;
        }
    };
    // The core's registers live in its sampling clock domain (the AD936x's
    // data clock): read right after the radio was set up, or with the core
    // left in reset by the trxd before (fpga-mode stop-dma), the ID came
    // back as garbage and the scope and DATV were off until the next
    // bitstream reload. Out of reset first, then a few tries.
    regs.wr32(REG_CONTROL, 0);
    let mut id = regs.rd32(0).to_le_bytes();
    for _ in 0..20 {
        if &id == b"maia" {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
        id = regs.rd32(0).to_le_bytes();
    }
    if &id != b"maia" {
        warn!("no Maia IP core at the maia-sdr UIO (id {id:?})");
        return None;
    }
    let nint = ((adc_rate / (BINS as f64 * rows_per_s)).round() as u32).clamp(1, 1023);
    regs.wr32(REG_CONTROL, 0); // out of reset
    // Full-rate ADC input, average mode, nint integrations; abort any
    // integration in progress so the new count takes effect now.
    regs.wr32(REG_SPECTROMETER, (nint << 1) | (1 << 11));
    // Full-scale 12-bit tone through a 4096-point FFT, averaged nint times.
    let norm = 1.0 / (nint as f32 * (2048.0f32 * BINS as f32).powi(2));
    info!(nint, rows_per_s = adc_rate / (BINS as f64 * nint as f64), "Maia FPGA spectrometer running");

    let (tx, rx) = bounded::<Vec<f32>>(2);
    std::thread::Builder::new()
        .name("maia-scope".into())
        .spawn(move || {
            let mut last: Option<usize> = None;
            let mut quiet_logged = false;
            loop {
                if uio.write_all(&1u32.to_ne_bytes()).is_err() {
                    break;
                }
                // The interrupt with a timeout: a PL that stopped (reloaded,
                // held in reset) is said once instead of a silent hang.
                let mut pfd = libc::pollfd { fd: uio.as_raw_fd(), events: libc::POLLIN, revents: 0 };
                // SAFETY: one valid pollfd for the duration of the call.
                let r = unsafe { libc::poll(&mut pfd, 1, 5000) };
                if r == 0 {
                    if !quiet_logged {
                        warn!(sdr_reset = regs.rd32(REG_CONTROL) & 1, "Maia spectrometer: no row for 5 s");
                        quiet_logged = true;
                    }
                    continue;
                }
                quiet_logged = false;
                let mut n = [0u8; 4];
                if uio.read_exact(&mut n).is_err() {
                    break;
                }
                if regs.rd32(REG_INTERRUPTS) & 1 == 0 {
                    continue;
                }
                let newest = ((regs.rd32(REG_SPECTROMETER) >> 12) & 7) as usize;
                // Only the newest buffer matters for a display; older ones are
                // skipped rather than queued.
                if last == Some(newest) {
                    continue;
                }
                last = Some(newest);
                let base = newest * BINS * 8;
                let mut row = Vec::with_capacity(BINS);
                for k in 0..BINS {
                    row.push(if k == 0 { 0.0 } else { decode_bin(ring.rd64(base + k * 8)) * norm });
                }
                if let Err(crossbeam_channel::TrySendError::Disconnected(_)) = tx.try_send(row) {
                    break; // engine gone
                }
            }
            warn!("Maia spectrometer reader stopped");
        })
        .ok()?;
    Some(rx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_maia_floating_point() {
        assert_eq!(decode_bin(1000), 1000.0);
        // exponent 2 (base 4): x16
        assert_eq!(decode_bin((2u64 << 56) | 1000), 16_000.0);
        // fastlock bits above 60 are ignored
        assert_eq!(decode_bin((5u64 << 61) | 7), 7.0);
    }
}
