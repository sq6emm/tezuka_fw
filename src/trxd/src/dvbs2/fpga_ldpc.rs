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
//!
//! "LDP5" (the same with `ldpc_dma.py`): the LLR words go to DDR (reserved
//! 1 MB at 0x16300000, device tree ldpc_buffers: 0.2 ms, against 3.5 ms
//! through the window), control bit 2 has the decoder load them, decode and
//! write the decisions back packed (bit j of word k: RAM variable 32 k + j)
//! at +64 KiB; 0xFF10 / 0xFF14 the addresses, 0xFF18 words in, 0xFF1C words
//! out (a multiple of 32).

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use super::FrameSpec;
use super::ldpc::Decoder;
use super::ldpc_fpga::{FpgaDecoder, LongRate, N, quantize_llr};

const PHYS: u64 = 0x43C4_0000;
const ID_LDP1: u32 = 0x3150_444C;
const ID_LDP4: u32 = 0x3450_444C;
const ID_LDP5: u32 = 0x3550_444C;
/// LDP5 whose cells path also takes 16QAM (0xFF20 bit 3, a14 at 0xFF2C).
const ID_LDP6: u32 = 0x3650_444C;
const DMA_PHYS: u64 = 0x1630_0000;
const DMA_BYTES: usize = 0x10_0000;
const DMA_OUT: usize = 0x1_0000;
const DT_DMA: &str = "/proc/device-tree/reserved-memory/ldpc_buffers@16300000";
/// One decoder in the FPGA (and one pair of DDR buffers): one frame at a time.
static DECODER: std::sync::Mutex<()> = std::sync::Mutex::new(());
const MAX_ITER: u32 = 50;
/// Time in the FPGA decoder's stages (ns): LLRs in, waiting, decisions out.
pub static PROF_NS: [std::sync::atomic::AtomicU64; 3] = [const { std::sync::atomic::AtomicU64::new(0) }; 3];

/// Channel LLR -> the decoder's 6-bit input (as the model was tuned).
pub const LLR_SCALE: f32 = 2.0;

pub struct Window {
    _mem: File,
    ptr: *mut u32,
    /// 1 (LDP1) or 4 (LDP4, LDP5).
    lanes: u8,
    /// LDP5 and its DDR buffers mapped: the LLRs in and decisions out there.
    dma: Option<*mut u32>,
    /// LDP6: the cells path takes 16QAM too.
    qam16: bool,
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
        let mut w = Window { _mem: mem, ptr: p.cast(), lanes: 0, dma: None, qam16: false };
        // A bitstream without the decoder has nothing at this address: the
        // read raises a bus error. Look from a child process first.
        static LANES: std::sync::OnceLock<u8> = std::sync::OnceLock::new();
        let lanes = *LANES.get_or_init(|| w.probe());
        if lanes == 0 {
            return Err("no LDPC decoder in this bitstream".into());
        }
        w.lanes = lanes.min(4);
        w.qam16 = lanes == 6;
        if lanes >= 5 && std::path::Path::new(DT_DMA).exists() && std::env::var_os("TRXD_NO_LDPC_DMA").is_none() {
            // SAFETY: MAP_SHARED of the reserved (no-map) buffer memory;
            // accessed below as aligned words inside it, unmapped on drop.
            let d = unsafe {
                libc::mmap(std::ptr::null_mut(), DMA_BYTES, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, w._mem.as_raw_fd(), DMA_PHYS as libc::off_t)
            };
            if d != libc::MAP_FAILED {
                w.dma = Some(d.cast());
            }
        }
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
                    ID_LDP5 => 5,
                    ID_LDP6 => 6,
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
            if let Some(d) = self.dma {
                libc::munmap(d.cast(), DMA_BYTES);
            }
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
/// `cells`: the words in `q` are DVB-T2 QPSK cells (two a word) and the
/// decoder's DDR engine makes the LLRs (LDP5 only).
/// `ddr_in`: the words are in DDR there already (the cell router's frame
/// buffer; `q` only gives their count).
#[allow(clippy::too_many_arguments)]
fn run_fpga(win: &Window, rate: LongRate, q: &mut [u32], max_iter: u32, need: usize, bits: &mut [u8], t_q: std::time::Instant, cells: Option<&crate::dvbt2::stream::CellParams>, ddr_in: Option<u32>) -> Option<usize> {
    let _one = DECODER.lock().unwrap_or_else(|e| e.into_inner());
    let out_words = need.div_ceil(32).div_ceil(32) * 32;
    let mut ctl = 1 | ((rate == LongRate::R3_4) as u32) << 1 | max_iter << 8;
    if let Some(d) = win.dma {
        match ddr_in {
            Some(a) => win.wr(0xFF10, a),
            None => {
                // SAFETY: the mapped buffers: at most N / 4 words in at 0 (< DMA_OUT).
                unsafe { std::ptr::copy_nonoverlapping(q.as_ptr(), d, q.len()) };
                win.wr(0xFF10, DMA_PHYS as u32);
            }
        }
        win.wr(0xFF14, (DMA_PHYS as usize + DMA_OUT) as u32);
        win.wr(0xFF18, q.len() as u32);
        win.wr(0xFF1C, out_words as u32);
        match cells {
            Some(p) => {
                win.wr(0xFF20, 1 | (p.rot as u32) << 1 | (p.qam16.is_some() as u32) << 3);
                win.wr(0xFF24, p.kq as u32);
                win.wr(0xFF28, (p.c14 as u32 & 0xFFFF) | (p.s14 as u32) << 16);
                if p.qam16.is_some() {
                    win.wr(0xFF2C, p.a14 as u32);
                }
            }
            None => win.wr(0xFF20, 0),
        }
        ctl |= 4;
    } else {
        win.write_words(0, q);
    }
    let t_in = std::time::Instant::now();
    win.wr(0xFF00, ctl);
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
    if let Some(d) = win.dma {
        // packed: bit j of word k is variable 32 k + j (the info part is in
        // natural order in the RAM)
        let mut w = vec![0u32; out_words];
        // SAFETY: the mapped buffers: out_words (<= 2048) words at DMA_OUT.
        unsafe { std::ptr::copy_nonoverlapping(d.add(DMA_OUT / 4), w.as_mut_ptr(), out_words) };
        for (i, b) in bits[..need].iter_mut().enumerate() {
            *b = ((w[i / 32] >> (i % 32)) & 1) as u8;
        }
    } else {
        let words = need.div_ceil(4);
        win.read_words(0, &mut q[..words]);
        for (w, &v) in q[..words].iter().enumerate() {
            for i in 0..4 {
                bits[4 * w + i] = ((v >> (8 * i + 7)) & 1) as u8;
            }
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

/// Does the FPGA decoder take DVB-T2 QPSK cells (it makes the LLRs)?
pub fn cells_available() -> bool {
    static CELLS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *CELLS.get_or_init(|| Window::open().is_ok_and(|w| w.dma.is_some()) && std::env::var_os("TRXD_NO_LDPC_CELLS").is_none())
}

/// And 16QAM cells (four LLRs a cell and the bit deinterleaver there too)?
pub fn cells16_available() -> bool {
    static CELLS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *CELLS.get_or_init(|| cells_available() && Window::open().is_ok_and(|w| w.qam16))
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
                tracing::info!(?rate, lanes = win.lanes, dma = win.dma.is_some(), "LDPC: the FPGA decoder");
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
    /// A DVB-T2 block's cells (QPSK, or 16QAM on LDP6; after the cell
    /// deinterleavers): the FPGA's DDR engine makes the LLRs; anywhere else
    /// they are made here, the same ([`crate::dvbt2::stream::cell_llrs`]).
    pub fn decode_cells(&mut self, cells: &[[i8; 2]], p: &crate::dvbt2::stream::CellParams, bits: &mut [u8]) -> Option<usize> {
        if let Ldpc::Fpga { win, rate, q, max_iter, need, .. } = self {
            let per_cell = if p.qam16.is_some() { 4 } else { 2 };
            if win.dma.is_some() && cells.len() * per_cell == N && (p.qam16.is_none() || win.qam16) {
                let t_q = std::time::Instant::now();
                for (w, c) in q.iter_mut().zip(cells.chunks_exact(2)) {
                    *w = (c[0][0] as u8 as u32) | (c[0][1] as u8 as u32) << 8 | (c[1][0] as u8 as u32) << 16 | (c[1][1] as u8 as u32) << 24;
                }
                let words = cells.len() / 2;
                return run_fpga(win, *rate, &mut q[..words], *max_iter, *need, bits, t_q, Some(p), None);
            }
        }
        let llr = crate::dvbt2::stream::cell_llrs(cells, p);
        self.decode_q(&llr, bits)
    }

    /// A DVB-T2 QPSK block whose cells the FPGA's cell router put in DDR at
    /// `addr` (only with the FPGA decoder's DDR engine: None otherwise).
    pub fn decode_ddr(&mut self, addr: u32, p: &crate::dvbt2::stream::CellParams, bits: &mut [u8]) -> Option<usize> {
        if let Ldpc::Fpga { win, rate, q, max_iter, need, .. } = self {
            if win.dma.is_some() && (p.qam16.is_none() || win.qam16) {
                let t_q = std::time::Instant::now();
                let words = if p.qam16.is_some() { N / 8 } else { N / 4 };
                return run_fpga(win, *rate, &mut q[..words], *max_iter, *need, bits, t_q, Some(p), Some(addr));
            }
        }
        None
    }

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
                run_fpga(win, *rate, q, *max_iter, *need, bits, t_q, None, None)
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
                run_fpga(win, *rate, q, *max_iter, *need, bits, t_q, None, None)
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

#[cfg(test)]
mod board_tests {
    use super::*;

    /// On a board with "LDP5": the same noisy rotated-QPSK block decoded
    /// from cells (the FPGA makes the LLRs) and from the LLRs made here
    /// (the word path): the decisions and iterations must agree.
    /// `trxd-test cells_vs_llr --ignored --nocapture`
    #[test]
    #[ignore]
    fn cells_vs_llr() {
        use crate::dvbt2::stream::{CellParams, qpsk_llrs};
        let spec = FrameSpec::long(crate::dvbs2::fpga_tx::LongMode::Qpsk12);
        let mut dec = Ldpc::for_spec(&spec);
        let rate = LongRate::R1_2;
        let mut x = 12345u32;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        let (c, s) = (29f64.to_radians().cos(), 29f64.to_radians().sin());
        for (trial, sigma) in [8.0f64, 14.0, 18.0, 22.0].into_iter().enumerate() {
            let info: Vec<u8> = (0..rate.k()).map(|_| (rnd() & 1) as u8).collect();
            let cw = super::super::ldpc_fpga::encode(rate, &info);
            let n = N / 2;
            // word j: (I, Q) from bits 2j, 2j+1, rotated +29 degrees; cell
            // j carries word j's I and word j - 1's Q (cyclic Q delay)
            let amp = 40.0;
            let words: Vec<(f64, f64)> = (0..n)
                .map(|j| {
                    let (i, q) = (amp * (1.0 - 2.0 * cw[2 * j] as f64), amp * (1.0 - 2.0 * cw[2 * j + 1] as f64));
                    (i * c - q * s, i * s + q * c)
                })
                .collect();
            let mut g = || {
                let (u1, u2) = ((rnd() as f64 + 1.0) / 4294967297.0, rnd() as f64 / 4294967296.0);
                (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
            };
            let cells: Vec<[i8; 2]> = (0..n)
                .map(|j| {
                    let q = words[(j + n - 1) % n].1;
                    [(words[j].0 + sigma * g()).round().clamp(-127.0, 127.0) as i8, (q + sigma * g()).round().clamp(-127.0, 127.0) as i8]
                })
                .collect();
            let p = CellParams { rot: true, kq: (2048.0 * 1024.0 / (sigma * sigma)).min(65536.0) as i32, c14: (c * 16384.0).round() as i32, s14: (-s * 16384.0).round() as i32, qam16: None, a14: 0 };
            let mut b1 = vec![0u8; N];
            let mut b2 = vec![0u8; N];
            let t = std::time::Instant::now();
            let r1 = dec.decode_cells(&cells, &p, &mut b1);
            let t1 = t.elapsed();
            let mut llr = vec![0i8; N];
            qpsk_llrs(&cells, &p, &mut llr);
            let t = std::time::Instant::now();
            let r2 = dec.decode_q(&llr, &mut b2);
            let t2 = t.elapsed();
            let need = (spec.kbch + 192).min(N);
            let diff = (0..need).filter(|&i| b1[i] != b2[i]).count();
            let errs = (0..need).filter(|&i| b1[i] != cw[i]).count();
            eprintln!("trial {trial} sigma {sigma}: cells {r1:?} in {:.2} ms, llr {r2:?} in {:.2} ms, {diff} decisions differ, {errs} bit errors left (cells)", t1.as_secs_f64() * 1e3, t2.as_secs_f64() * 1e3);
            assert_eq!(r1, r2);
            assert_eq!(diff, 0);
        }
    }

    /// On a board with "LDP6": 16QAM cells (both rates, rotated) decoded
    /// from cells (the FPGA makes the LLRs and deinterleaves the bits) and
    /// from [`crate::dvbt2::stream::cell_llrs`]: the same decisions. The
    /// cells are random (a failing decode: all 50 iterations compared).
    /// `trxd-test cells16_vs_llr --ignored --nocapture`
    #[test]
    #[ignore]
    fn cells16_vs_llr() {
        use crate::dvbt2::stream::{CellParams, cell_llrs};
        assert!(cells16_available(), "no LDP6 decoder");
        let mut x = 777u32;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        for (mode, rate) in [(crate::dvbs2::fpga_tx::LongMode::Qpsk12, LongRate::R1_2), (crate::dvbs2::fpga_tx::LongMode::Qpsk34, LongRate::R3_4)] {
            let spec = FrameSpec::long(mode);
            let mut dec = Ldpc::for_spec(&spec);
            let cells: Vec<[i8; 2]> = (0..N / 4).map(|_| [(rnd() >> 8) as i8, (rnd() >> 8) as i8]).collect();
            let p = CellParams { rot: true, kq: 1811, c14: 15685, s14: -4739, qam16: Some(rate), a14: (2.0 * 40.0 / 10f64.sqrt() * 16384.0).round() as i32 };
            let mut b1 = vec![0u8; N];
            let mut b2 = vec![0u8; N];
            let t = std::time::Instant::now();
            let r1 = dec.decode_cells(&cells, &p, &mut b1);
            let t1 = t.elapsed();
            let t = std::time::Instant::now();
            let llr = cell_llrs(&cells, &p);
            let r2 = dec.decode_q(&llr, &mut b2);
            let t2 = t.elapsed();
            let need = (spec.kbch + 192).min(N);
            let diff = (0..need).filter(|&i| b1[i] != b2[i]).count();
            eprintln!("{rate:?}: cells {r1:?} in {:.2} ms, llr {r2:?} in {:.2} ms (LLRs here), {diff} decisions differ", t1.as_secs_f64() * 1e3, t2.as_secs_f64() * 1e3);
            assert_eq!(r1, r2);
            assert_eq!(diff, 0);
        }
    }
}
