//! LDPC decoding for the receiver: the float decoder for short frames; for
//! normal frames the FPGA's decoder (maia-sdr `ldpc_axi.py`, 0x43C40000) on
//! a board, or its bit-exact model ([`super::ldpc_fpga`]) elsewhere.
//!
//! FPGA window: 0x0000.. posterior RAM (byte v % 4 of word v / 4 is variable
//! v: LLRs in, signs out), 0xFF00 control (bit 0 start, bit 1 rate 3/4,
//! 13:8 iterations), 0xFF04 status (bit 0 busy, bit 1 converged, 13:8
//! iterations), 0xFF08 id "LDP1".

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use super::FrameSpec;
use super::ldpc::Decoder;
use super::ldpc_fpga::{FpgaDecoder, LongRate, N, quantize_llr};

const PHYS: u64 = 0x43C4_0000;
const ID_LDP1: u32 = 0x3150_444C;
const MAX_ITER: u32 = 50;
/// Channel LLR -> the decoder's 6-bit input (as the model was tuned).
const LLR_SCALE: f32 = 2.0;

pub struct Window {
    _mem: File,
    ptr: *mut u32,
}

// SAFETY: owned by the decoding thread alone.
unsafe impl Send for Window {}

impl Window {
    fn open() -> Result<Window, String> {
        let mem = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open("/dev/mem")
            .map_err(|e| format!("/dev/mem: {e}"))?;
        // SAFETY: MAP_SHARED of the decoder's 64 KiB AXI window; accesses
        // below are aligned 32-bit words inside it.
        let p = unsafe {
            libc::mmap(std::ptr::null_mut(), 0x1_0000, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), PHYS as libc::off_t)
        };
        if p == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error().to_string());
        }
        let w = Window { _mem: mem, ptr: p.cast() };
        // A bitstream without the decoder has nothing at this address: the
        // read raises a bus error. Look from a child process first.
        static PRESENT: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if !*PRESENT.get_or_init(|| w.probe()) {
            return Err("no LDPC decoder in this bitstream".into());
        }
        Ok(w)
    }
    fn probe(&self) -> bool {
        // SAFETY: the child only reads the mapping and _exits (both
        // async-signal-safe); the parent waits for it.
        unsafe {
            match libc::fork() {
                0 => libc::_exit(if self.rd(0xFF08) == ID_LDP1 { 0 } else { 1 }),
                -1 => false,
                pid => {
                    let mut st = 0;
                    libc::waitpid(pid, &mut st, 0) == pid && libc::WIFEXITED(st) && libc::WEXITSTATUS(st) == 0
                }
            }
        }
    }
    fn rd(&self, off: usize) -> u32 {
        // SAFETY: see open().
        unsafe { std::ptr::read_volatile(self.ptr.add(off / 4)) }
    }
    fn wr(&self, off: usize, v: u32) {
        // SAFETY: see open().
        unsafe { std::ptr::write_volatile(self.ptr.add(off / 4), v) }
    }
}

impl Drop for Window {
    fn drop(&mut self) {
        // SAFETY: unmapping what open() mapped.
        unsafe {
            libc::munmap(self.ptr.cast(), 0x1_0000);
        }
    }
}

/// Is the FPGA decoder there?
pub fn available() -> bool {
    Window::open().is_ok()
}

pub enum Ldpc {
    Short(Decoder),
    Model(FpgaDecoder),
    Fpga { win: Window, rate: LongRate, q: Vec<i8>, max_iter: u32 },
}

impl Ldpc {
    pub fn for_spec(spec: &FrameSpec) -> Ldpc {
        if let Some(rate) = spec.short_rate {
            return Ldpc::Short(Decoder::new(rate));
        }
        let rate = spec.long_rate.expect("a long-frame rate");
        match Window::open() {
            Ok(win) => {
                tracing::info!(?rate, "LDPC: the FPGA decoder");
                Ldpc::Fpga { win, rate, q: vec![0; N], max_iter: MAX_ITER }
            }
            Err(e) => {
                tracing::info!(?rate, "LDPC: FPGA decoder unavailable ({e}); its model in software");
                Ldpc::Model(FpgaDecoder::new(rate))
            }
        }
    }

    /// At most `n` iterations (fewer when frames queue up behind this one).
    pub fn set_max_iter(&mut self, n: usize) {
        match self {
            Ldpc::Short(d) => d.max_iter = n,
            Ldpc::Model(d) => d.max_iter = n,
            Ldpc::Fpga { max_iter, .. } => *max_iter = (n as u32).clamp(1, 63),
        }
    }

    /// Decode `llr` (positive = 0) into `bits`; the iterations when every
    /// check is satisfied.
    pub fn decode(&mut self, llr: &[f32], bits: &mut [u8]) -> Option<usize> {
        match self {
            Ldpc::Short(d) => d.decode(llr, bits),
            Ldpc::Model(d) => {
                let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, LLR_SCALE)).collect();
                d.decode(&q, bits)
            }
            Ldpc::Fpga { win, rate, q, max_iter } => {
                for (d, &l) in q.iter_mut().zip(llr) {
                    *d = quantize_llr(l, LLR_SCALE);
                }
                for (w, c) in q.chunks_exact(4).enumerate() {
                    let v = u32::from_le_bytes([c[0] as u8, c[1] as u8, c[2] as u8, c[3] as u8]);
                    win.wr(4 * w, v);
                }
                win.wr(0xFF00, 1 | ((*rate == LongRate::R3_4) as u32) << 1 | *max_iter << 8);
                // 2.5 ms an iteration: sleep in small steps until done.
                let t0 = std::time::Instant::now();
                while win.rd(0xFF04) & 1 == 1 {
                    if t0.elapsed() > std::time::Duration::from_millis(500) {
                        tracing::warn!("LDPC: the FPGA decoder did not finish");
                        return None;
                    }
                    std::thread::sleep(std::time::Duration::from_micros(500));
                }
                let st = win.rd(0xFF04);
                // Decisions: the sign bits (all of them: the caller reads
                // the BBFRAME part).
                for w in 0..N / 4 {
                    let v = win.rd(4 * w);
                    for i in 0..4 {
                        bits[4 * w + i] = ((v >> (8 * i + 7)) & 1) as u8;
                    }
                }
                ((st >> 1) & 1 == 1).then_some(((st >> 8) & 0x3F) as usize)
            }
        }
    }
}
