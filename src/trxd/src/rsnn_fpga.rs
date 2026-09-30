//! The CW-RS keying detector in the FPGA (maia-sdr `rsnn_front.py`,
//! `rsnn_temporal.py`, `rsnn_axi.py`; 0x43C50000): a feature row in, the
//! front end's outputs for the frame three rows back ([`crate::rsnn::Net::
//! front_q`] bit for bit) and, with "RSF2" (the trx bitstream), the last
//! temporal layer's outputs whenever a frame is through them
//! ([`crate::rsnn::Stream`]'s fixed point, bit for bit): the ARM does only
//! the features and the last 1x1 then.
//!
//! Window: 0x0000.. weights (two a word, the even one low), 0x9000..
//! biases, 0x9400.. the next row (two bins a word), 0x9500.. the front's
//! outputs (i32), 0x9600.. the temporal outputs, 0x9800.. temporal biases
//! (64 a layer), 0xFF00 control (bit 0 go, bit 1 reset; 12:8 c2, 22:16
//! c1), 0xFF04 status (bit 0 busy, bit 1 front valid, bit 2 temporal
//! valid), 0xFF08 id "RSF1"/"RSF2", 0xFF10 temporal layers, 0xFF14 their
//! weights' DDR address, 0xFF18 its 64-bit words, 0xFF20 + 4 l layer l
//! (6:0 dilation, 16:7 first ring frame, 26:17 ring frames). The temporal
//! weights live in the reserved memory at 0x16200000 (device tree
//! rsnn_weights).
//!
//! One engine: the first detector to open it has it (the others run on the
//! CPU). TRXD_NO_RSNN_FPGA=1: never.

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::rsnn::{Net, NB};

const PHYS: u64 = 0x43C5_0000;
const ID: u32 = 0x3146_5352;
const ID2: u32 = 0x3246_5352;
const W_PHYS: u64 = 0x1620_0000;
const W_BYTES: usize = 0x10_0000;
const DT_WEIGHTS: &str = "/proc/device-tree/reserved-memory/rsnn_weights@16200000";
const LMAX: usize = 8;
const RING_FRAMES: usize = 520;
const C2MAX: usize = 24;
const C1MAX: usize = 64;
const WMAX: usize = 17408;
const BMAX: usize = 256;
/// The engine's clock (the CPU interconnect's, 100 MHz): cycles to us.
const CYCLES_PER_US: usize = 100;

static TAKEN: AtomicBool = AtomicBool::new(false);

pub struct FpgaFront {
    _mem: File,
    ptr: *mut u32,
    cfg: u32,
    c1: usize,
    /// About how long a frame takes (us): sleep that before polling.
    frame_us: u64,
    row: [u32; 32],
    out: Vec<u32>,
    /// The temporal layers run here too.
    temporal: bool,
}

/// What a row gave: the front's outputs (frame n - 3), or with the
/// temporal layers in the FPGA the last one's (a frame 2 x the dilations
/// further back), or nothing yet.
pub enum Out<'a> {
    Front(&'a [u32]),
    Temporal(&'a [u32]),
}

// SAFETY: owned by the detector's thread alone.
unsafe impl Send for FpgaFront {}

impl FpgaFront {
    /// The engine loaded with `net`'s front, if the bitstream has one, it is
    /// free and the network fits.
    pub fn open(net: &Net, temporal: Option<(Vec<i16>, Vec<i32>, Vec<usize>)>) -> Option<FpgaFront> {
        if std::env::var_os("TRXD_NO_RSNN_FPGA").is_some() {
            return None;
        }
        let (c2, c1) = net.channels();
        let (w, b) = net.fpga_image();
        if c2 > C2MAX || c1 > C1MAX || w.len() > WMAX || b.len() > BMAX {
            tracing::info!("rsnn: network too large for the FPGA front ({c2}/{c1} channels)");
            return None;
        }
        if TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        match FpgaFront::map(c2, c1) {
            Ok(mut f) => {
                f.load(&w, &b);
                if let Some(t) = temporal {
                    f.load_temporal(c1, &t);
                }
                tracing::info!(temporal = f.temporal, "rsnn: in the FPGA ({c2}/{c1} channels, ~{} us a frame)", f.frame_us);
                Some(f)
            }
            Err(e) => {
                tracing::debug!("rsnn: no FPGA front: {e}");
                TAKEN.store(false, Ordering::Release);
                None
            }
        }
    }

    fn map(c2: usize, c1: usize) -> Result<FpgaFront, String> {
        let mem = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open("/dev/mem")
            .map_err(|e| format!("/dev/mem: {e}"))?;
        // SAFETY: MAP_SHARED of the engine's 64 KiB AXI window; accesses
        // below are aligned 32-bit words inside it.
        let p = unsafe {
            libc::mmap(std::ptr::null_mut(), 0x1_0000, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), PHYS as libc::off_t)
        };
        if p == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error().to_string());
        }
        let cycles = c2 * NB * 15 + c2 * c2 * 31 * 24 + c2 * 31 * 3 + 31 * 3 + 32 + c1 * 2 * c2;
        let f = FpgaFront {
            _mem: mem,
            ptr: p.cast(),
            cfg: ((c2 as u32) << 8) | ((c1 as u32) << 16),
            c1,
            frame_us: (cycles / CYCLES_PER_US) as u64,
            row: [0; 32],
            out: vec![0; c1],
            temporal: false,
        };
        // A bitstream without the engine has nothing at this address: the
        // read raises a bus error. Look from a child process first.
        static THERE: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if !*THERE.get_or_init(|| f.probe()) {
            return Err("no rsnn front in this bitstream".into());
        }
        Ok(f)
    }

    fn probe(&self) -> bool {
        // SAFETY: the child only reads the mapping and _exits (both
        // async-signal-safe); the parent waits for it.
        unsafe {
            match libc::fork() {
                0 => libc::_exit(match self.rd(0xFF08) {
                    ID => 1,
                    ID2 => 2,
                    _ => 0,
                }),
                -1 => false,
                pid => {
                    let mut st = 0;
                    libc::waitpid(pid, &mut st, 0) == pid && libc::WIFEXITED(st) && libc::WEXITSTATUS(st) != 0
                }
            }
        }
    }

    fn rd(&self, off: usize) -> u32 {
        // SAFETY: see map().
        unsafe { std::ptr::read_volatile(self.ptr.add(off / 4)) }
    }

    fn wr(&self, off: usize, v: u32) {
        // SAFETY: see map().
        unsafe { std::ptr::write_volatile(self.ptr.add(off / 4), v) }
    }

    fn write_words(&self, off: usize, v: &[u32]) {
        assert!(off % 4 == 0 && off + 4 * v.len() <= 0x1_0000);
        // SAFETY: see map(); both sides word-aligned, inside the window.
        unsafe { std::ptr::copy_nonoverlapping(v.as_ptr(), self.ptr.add(off / 4), v.len()) }
    }

    fn read_words(&self, off: usize, v: &mut [u32]) {
        assert!(off % 4 == 0 && off + 4 * v.len() <= 0x1_0000);
        // SAFETY: see map().
        unsafe { std::ptr::copy_nonoverlapping(self.ptr.add(off / 4), v.as_mut_ptr(), v.len()) }
    }

    fn wait(&self) {
        while self.rd(0xFF04) & 1 != 0 {
            std::thread::sleep(Duration::from_micros(100));
        }
    }

    fn load(&mut self, w: &[i16], b: &[i32]) {
        self.wait();
        self.wr(0xFF00, self.cfg);
        let words: Vec<u32> = w.chunks(2).map(|p| (p[0] as u16 as u32) | (p.get(1).map_or(0, |&v| v as u16 as u32) << 16)).collect();
        self.write_words(0, &words);
        let bw: Vec<u32> = b.iter().map(|&v| v as u32).collect();
        self.write_words(0x9000, &bw);
        self.reset();
    }

    /// The temporal layers into the engine ("RSF2" and the reserved memory
    /// there, the network within its limits): weights to DDR, the rest to
    /// its registers.
    fn load_temporal(&mut self, c1: usize, (w, b, dils): &(Vec<i16>, Vec<i32>, Vec<usize>)) {
        let frames: usize = dils.iter().map(|d| 4 * d + 1).sum();
        if self.rd(0xFF08) != ID2 || !std::path::Path::new(DT_WEIGHTS).exists() {
            return;
        }
        if dils.is_empty() || dils.len() > LMAX || c1 % 2 != 0 || frames > RING_FRAMES || 2 * w.len() > W_BYTES || dils.iter().any(|&d| d > 127) {
            tracing::info!(layers = dils.len(), frames, "rsnn: temporal layers too large for the FPGA");
            return;
        }
        let mem = match OpenOptions::new().read(true).write(true).custom_flags(libc::O_SYNC).open("/dev/mem") {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!("rsnn: /dev/mem: {e}");
                return;
            }
        };
        // SAFETY: MAP_SHARED of the reserved (no-map) weight memory; written
        // below as aligned words inside it, then unmapped.
        let p = unsafe { libc::mmap(std::ptr::null_mut(), W_BYTES, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), W_PHYS as libc::off_t) };
        if p == libc::MAP_FAILED {
            tracing::warn!("rsnn: map the weight memory: {}", std::io::Error::last_os_error());
            return;
        }
        let words: Vec<u32> = w.chunks(2).map(|p| (p[0] as u16 as u32) | (p.get(1).map_or(0, |&v| v as u16 as u32) << 16)).collect();
        // SAFETY: inside the mapping (2 w.len() <= W_BYTES), word-aligned.
        unsafe {
            std::ptr::copy_nonoverlapping(words.as_ptr(), p.cast::<u32>(), words.len());
            libc::munmap(p, W_BYTES);
        }
        self.wait();
        let mut base = 0;
        for (l, &d) in dils.iter().enumerate() {
            let cfg = d as u32 | (base as u32) << 7 | ((4 * d + 1) as u32) << 17;
            self.wr(0xFF20 + 4 * l, cfg);
            // the layer registers read back (a bitstream that decodes fewer
            // of them would run a different network)
            if self.rd(0xFF20 + 4 * l) != cfg {
                tracing::warn!(layer = l, "rsnn: temporal layer registers do not read back: temporal layers on the CPU");
                self.wr(0xFF10, 0);
                return;
            }
            let bw: Vec<u32> = b[l * c1..(l + 1) * c1].iter().map(|&v| v as u32).collect();
            self.write_words(0x9800 + 4 * 64 * l, &bw);
            base += 4 * d + 1;
        }
        self.wr(0xFF14, W_PHYS as u32);
        self.wr(0xFF18, w.len().div_ceil(4) as u32);
        self.wr(0xFF10, dils.len() as u32);
        self.temporal = true;
        let macs: usize = dils.len() * (c1 * (5 * c1 + 8) + c1);
        self.frame_us += (macs / CYCLES_PER_US) as u64;
        self.reset();
    }

    /// The temporal layers run in the FPGA.
    pub fn temporal(&self) -> bool {
        self.temporal
    }

    /// A fresh stream: the rows before the next are zeros.
    pub fn reset(&mut self) {
        self.wait();
        self.wr(0xFF00, self.cfg | 2);
        self.wait();
    }

    /// Row n in (as [`Net::front_q`] quantizes it); what came out (x AQ).
    pub fn push(&mut self, row: &[f32; NB]) -> Option<Out<'_>> {
        let q = |v: f32| (v * crate::rsnn::AQ).round() as i16 as u16 as u32;
        for (k, w) in self.row.iter_mut().enumerate() {
            let a = row.get(2 * k).map_or(0, |&v| q(v));
            let b = row.get(2 * k + 1).map_or(0, |&v| q(v));
            *w = a | b << 16;
        }
        self.write_words(0x9400, &self.row);
        self.wr(0xFF00, self.cfg | 1);
        std::thread::sleep(Duration::from_micros(self.frame_us));
        self.wait();
        let st = self.rd(0xFF04);
        let at = if self.temporal {
            if st & 4 == 0 {
                return None;
            }
            0x9600
        } else {
            if st & 2 == 0 {
                return None;
            }
            0x9500
        };
        let mut out = std::mem::take(&mut self.out);
        self.read_words(at, &mut out[..self.c1]);
        self.out = out;
        Some(if self.temporal { Out::Temporal(&self.out) } else { Out::Front(&self.out) })
    }
}

impl Drop for FpgaFront {
    fn drop(&mut self) {
        // SAFETY: unmapping what map() mapped.
        unsafe {
            libc::munmap(self.ptr.cast(), 0x1_0000);
        }
        TAKEN.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    /// On a board: how fast the CPU writes / reads the reserved DDR through
    /// /dev/mem (O_SYNC mapping, as trxd uses it), against the AXI-Lite
    /// LLR writes into the LDPC decoder (3.5 ms for 16200 words).
    /// `trxd-test ddr_bench --ignored --nocapture` (CW-RS off).
    #[test]
    #[ignore]
    fn ddr_bench() {
        use std::os::unix::fs::OpenOptionsExt;
        use std::os::unix::io::AsRawFd;
        let mem = std::fs::OpenOptions::new().read(true).write(true).custom_flags(libc::O_SYNC).open("/dev/mem").unwrap();
        // SAFETY: the reserved rsnn_weights memory (1 MB), mapped and unmapped here.
        let p = unsafe { libc::mmap(std::ptr::null_mut(), super::W_BYTES, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), super::W_PHYS as libc::off_t) };
        assert!(p != libc::MAP_FAILED);
        let src: Vec<u32> = (0..16200u32).map(|i| i.wrapping_mul(2654435761)).collect();
        let mut back = vec![0u32; 16200];
        for _ in 0..3 {
            let t = std::time::Instant::now();
            // SAFETY: inside the mapping.
            unsafe { std::ptr::copy_nonoverlapping(src.as_ptr(), p.cast::<u32>(), src.len()) };
            let w = t.elapsed();
            let t = std::time::Instant::now();
            // SAFETY: inside the mapping.
            unsafe { std::ptr::copy_nonoverlapping(p.cast::<u32>(), back.as_mut_ptr(), 2025) };
            let r = t.elapsed();
            eprintln!("write 16200 words {:.3} ms, read 2025 words {:.3} ms", w.as_secs_f64() * 1e3, r.as_secs_f64() * 1e3);
        }
        // SAFETY: inside the mapping.
        unsafe { std::ptr::copy_nonoverlapping(p.cast::<u32>(), back.as_mut_ptr(), 16200) };
        assert_eq!(back, src);
        // SAFETY: unmapping.
        unsafe { libc::munmap(p, super::W_BYTES) };
    }
}
