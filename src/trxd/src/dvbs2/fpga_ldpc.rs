//! LDPC decoding for the receiver: the float decoder for short frames; for
//! normal frames the FPGA's decoder (maia-sdr `ldpc_axi.py`, 0x43C40000) on
//! a board, or its bit-exact model ([`super::ldpc_fpga`]) elsewhere.
//!
//! FPGA window: 0x0000.. posterior RAM (byte v % 4 of word v / 4 is variable
//! v: LLRs in, signs out), 0xFF00 control (bit 0 start, bit 1 rate 3/4,
//! 13:8 iterations), 0xFF04 status (bit 0 busy, bit 1 converged, 13:8
//! iterations), 0xFF08 id "LDP1".
//!
//! "LDP4" (maia-sdr `ldpc_dec4.py`, four checks at a time): the same, but the
//! parity part of the RAM in banks: parity bit p = c q + r at word
//! k / 4 + r 90 + c / 4, byte c % 4 (the info part as above).

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use super::FrameSpec;
use super::ldpc::Decoder;
use super::ldpc_fpga::{FpgaDecoder, LongRate, N, quantize_llr};

const PHYS: u64 = 0x43C4_0000;
const ID_LDP1: u32 = 0x3150_444C;
const ID_LDP4: u32 = 0x3450_444C;
const MAX_ITER: u32 = 50;
/// Time in the FPGA decoder's stages (ns): LLRs in, waiting, decisions out.
pub static PROF_NS: [std::sync::atomic::AtomicU64; 3] = [const { std::sync::atomic::AtomicU64::new(0) }; 3];

/// Channel LLR -> the decoder's 6-bit input (as the model was tuned).
pub const LLR_SCALE: f32 = 2.0;

pub struct Window {
    _mem: File,
    ptr: *mut u32,
    /// 1 (LDP1) or 4 (LDP4).
    lanes: u8,
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
        let mut w = Window { _mem: mem, ptr: p.cast(), lanes: 0 };
        // A bitstream without the decoder has nothing at this address: the
        // read raises a bus error. Look from a child process first.
        static LANES: std::sync::OnceLock<u8> = std::sync::OnceLock::new();
        let lanes = *LANES.get_or_init(|| w.probe());
        if lanes == 0 {
            return Err("no LDPC decoder in this bitstream".into());
        }
        w.lanes = lanes;
        Ok(w)
    }
    /// The decoder's lanes (0: none there).
    fn probe(&self) -> u8 {
        // SAFETY: the child only reads the mapping and _exits (both
        // async-signal-safe); the parent waits for it.
        unsafe {
            match libc::fork() {
                0 => libc::_exit(match self.rd(0xFF08) {
                    ID_LDP1 => 1,
                    ID_LDP4 => 4,
                    _ => 0,
                }),
                -1 => 0,
                pid => {
                    let mut st = 0;
                    if libc::waitpid(pid, &mut st, 0) == pid && libc::WIFEXITED(st) { libc::WEXITSTATUS(st) as u8 } else { 0 }
                }
            }
        }
    }
    fn rd(&self, off: usize) -> u32 {
        // SAFETY: see open().
        unsafe { std::ptr::read_volatile(self.ptr.add(off / 4)) }
    }
    /// Aligned words to the window from `off` in one copy.
    fn write_words(&self, off: usize, v: &[u32]) {
        assert!(off % 4 == 0 && off + 4 * v.len() <= 0x1_0000);
        // SAFETY: see open(); both sides word-aligned, inside the window.
        unsafe { std::ptr::copy_nonoverlapping(v.as_ptr(), self.ptr.add(off / 4), v.len()) }
    }
    /// Aligned words from the window at `off` in one copy.
    fn read_words(&self, off: usize, v: &mut [u32]) {
        assert!(off % 4 == 0 && off + 4 * v.len() <= 0x1_0000);
        // SAFETY: see open().
        unsafe { std::ptr::copy_nonoverlapping(self.ptr.add(off / 4), v.as_mut_ptr(), v.len()) }
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

/// The LLR words of the window: `v(i)` the 6-bit LLR of variable i, the
/// parity in the four-lane decoder's layout when `perm` is there.
fn pack(q: &mut [u32], perm: &[u32], k: usize, v: impl Fn(usize) -> u32) {
    if perm.is_empty() {
        for (w, o) in q.iter_mut().enumerate() {
            let i = 4 * w;
            *o = v(i) | v(i + 1) << 8 | v(i + 2) << 16 | v(i + 3) << 24;
        }
    } else {
        for (w, o) in q[..k / 4].iter_mut().enumerate() {
            let i = 4 * w;
            *o = v(i) | v(i + 1) << 8 | v(i + 2) << 16 | v(i + 3) << 24;
        }
        for (o, p) in q[k / 4..].iter_mut().zip(perm.chunks_exact(4)) {
            let x = |j: usize| v(k + p[j] as usize);
            *o = x(0) | x(1) << 8 | x(2) << 16 | x(3) << 24;
        }
    }
}

/// The LLRs into the window, a decode, the decisions back.
fn run_fpga(win: &Window, rate: LongRate, q: &mut [u32], max_iter: u32, need: usize, bits: &mut [u8], t_q: std::time::Instant) -> Option<usize> {
    win.write_words(0, q);
    let t_in = std::time::Instant::now();
    win.wr(0xFF00, 1 | ((rate == LongRate::R3_4) as u32) << 1 | max_iter << 8);
    // 2.5 ms an iteration: sleep in small steps until done.
    let t0 = std::time::Instant::now();
    while win.rd(0xFF04) & 1 == 1 {
        if t0.elapsed() > std::time::Duration::from_millis(500) {
            tracing::warn!("LDPC: the FPGA decoder did not finish");
            return None;
        }
        std::thread::sleep(std::time::Duration::from_micros(500));
    }
    let t_out = std::time::Instant::now();
    let st = win.rd(0xFF04);
    // Decisions: the sign bits of the BCH codeword part (the
    // parity bits after it are never read).
    let words = need.div_ceil(4);
    win.read_words(0, &mut q[..words]);
    for (w, &v) in q[..words].iter().enumerate() {
        for i in 0..4 {
            bits[4 * w + i] = ((v >> (8 * i + 7)) & 1) as u8;
        }
    }
    use std::sync::atomic::Ordering::Relaxed;
    PROF_NS[0].fetch_add((t_in - t_q).as_nanos() as u64, Relaxed);
    PROF_NS[1].fetch_add((t_out - t0).as_nanos() as u64, Relaxed);
    PROF_NS[2].fetch_add(t_out.elapsed().as_nanos() as u64, Relaxed);
    ((st >> 1) & 1 == 1).then_some(((st >> 8) & 0x3F) as usize)
}

/// The four-lane decoder's parity layout: for each parity byte of its RAM
/// (word k / 4 + r 90 + c / 4, byte c % 4), the parity bit c q + r.
pub fn parity_layout(rate: LongRate) -> Vec<u32> {
    let q = rate.q();
    let mut perm = vec![0u32; N - rate.k()];
    for (i, v) in perm.iter_mut().enumerate() {
        let (w, byte) = (i / 4, i % 4);
        let (r, c) = (w / 90, 4 * (w % 90) + byte);
        *v = (c * q + r) as u32;
    }
    perm
}

/// Is the FPGA decoder there?
pub fn available() -> bool {
    Window::open().is_ok()
}

pub enum Ldpc {
    Short(Decoder),
    Model(FpgaDecoder),
    /// `need`: decisions read back (the BCH codeword's Kbch + 192 bits).
    /// `perm` (four-lane decoder): for each parity byte of the RAM in
    /// order, its parity bit.
    Fpga { win: Window, rate: LongRate, q: Vec<u32>, max_iter: u32, need: usize, perm: Vec<u32> },
}

impl Ldpc {
    pub fn for_spec(spec: &FrameSpec) -> Ldpc {
        if let Some(rate) = spec.short_rate {
            return Ldpc::Short(Decoder::new(rate));
        }
        let rate = spec.long_rate.expect("a long-frame rate");
        match Window::open() {
            Ok(win) => {
                tracing::info!(?rate, lanes = win.lanes, "LDPC: the FPGA decoder");
                let perm = if win.lanes == 4 { parity_layout(rate) } else { Vec::new() };
                Ldpc::Fpga { win, rate, q: vec![0; N / 4], max_iter: MAX_ITER, need: (spec.kbch + 192).min(N), perm }
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

    /// [`Self::decode`] from LLRs already in the decoder's 6 bits
    /// ([`quantize_llr`] at [`LLR_SCALE`]): the DVB-T2 receiver makes them
    /// so, without a float vector between.
    pub fn decode_q(&mut self, llr: &[i8], bits: &mut [u8]) -> Option<usize> {
        match self {
            Ldpc::Short(_) => {
                let f: Vec<f32> = llr.iter().map(|&v| v as f32 / LLR_SCALE).collect();
                self.decode(&f, bits)
            }
            Ldpc::Model(d) => d.decode(llr, bits),
            Ldpc::Fpga { win, rate, q, max_iter, need, perm } => {
                let t_q = std::time::Instant::now();
                pack(q, perm, rate.k(), |i| llr[i] as u8 as u32);
                run_fpga(win, *rate, q, *max_iter, *need, bits, t_q)
            }
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
            Ldpc::Fpga { win, rate, q, max_iter, need, perm } => {
                let t_q = std::time::Instant::now();
                // Four 6-bit LLRs a word, then one bulk copy into the
                // window (word writes one at a time cost 10 ms a frame).
                let b = |l: f32| quantize_llr(l, LLR_SCALE) as u8 as u32;
                pack(q, perm, rate.k(), |i| b(llr[i]));
                run_fpga(win, *rate, q, *max_iter, *need, bits, t_q)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parity_layout_is_a_permutation() {
        for rate in [LongRate::R1_2, LongRate::R3_4] {
            let mut p = parity_layout(rate);
            p.sort_unstable();
            assert!(p.iter().enumerate().all(|(i, &v)| v as usize == i), "{rate:?}");
        }
    }
}
