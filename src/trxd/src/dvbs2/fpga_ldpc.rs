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
//!
//! Newer cores (0xFF30 features, 0 on older ones): bit 0 status 31:24
//! counts finished decodes, bit 1 status bit 2 is a sticky AXI error of the
//! DDR engine (a burst answered SLVERR/DECERR), bit 2 configuration writes
//! are ignored while busy. Whatever the core, a decode is never started (nor
//! its registers or buffers written) while the engine is busy: a decode
//! that times out keeps the decoder until it is idle again, and an engine
//! that stays busy is left alone ([`STUCK`]) with the frames decoded by the
//! model in software, slowly, until it comes back.
//!
//! BBFRAME out (features bit 5, 0xFF20 bit 6; ldpc_dma.py `bb`): the
//! decisions come back packed MSB first a byte at a time, the first Kbch
//! descrambled, so the buffer starts with the BBFRAME's bytes; the BCH
//! remainder of the first Nbch is in 0xFF4C..0xFF60 (status bit 3: zero, a
//! valid codeword). The receiver ([`Ldpc::want_bb`]) then neither unpacks
//! bits nor divides by g(x) nor descrambles: [`Ldpc::take_bb`].
//!

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use super::FrameSpec;
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
/// The engine stayed busy past every deadline: no start until it is idle.
static STUCK: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// Waiting for a decode (2.5 ms an iteration, 63 at most: 160 ms).
const DECODE_WAIT: std::time::Duration = std::time::Duration::from_millis(500);
/// After that, how long a late decode may take to end before the engine
/// counts as stuck.
const LATE_WAIT: std::time::Duration = std::time::Duration::from_secs(3);
/// Time in the FPGA decoder's stages (ns): LLRs in, waiting, decisions out.
pub static PROF_NS: [std::sync::atomic::AtomicU64; 3] = [const { std::sync::atomic::AtomicU64::new(0) }; 3];
/// Decodes run, iterations they took, and how many ran to the limit
/// without converging (for the FEC thread's log: what `fpga_ms` is made of).
pub static PROF_ITER: [std::sync::atomic::AtomicU64; 3] = [const { std::sync::atomic::AtomicU64::new(0) }; 3];

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
    /// 0xFF30 (0 on older cores): bit 0 finished counter, bit 1 AXI error.
    feat: u32,
    /// BBFRAME out and the BCH remainder in the fabric (features bit 5).
    bb: bool,
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
        let mut w = Window { _mem: mem, ptr: p.cast(), lanes: 0, dma: None, qam16: false, feat: 0, bb: false };
        // A bitstream without the decoder has nothing at this address: the
        // read raises a bus error. Look from a child process first.
        static LANES: std::sync::OnceLock<u8> = std::sync::OnceLock::new();
        let lanes = *LANES.get_or_init(|| w.probe());
        if lanes == 0 {
            return Err("no LDPC decoder in this bitstream".into());
        }
        w.lanes = lanes.min(4);
        w.qam16 = lanes == 6;
        w.feat = w.rd(0xFF30) & 63;
        w.bb = w.feat & 32 != 0;
        if lanes >= 5 && std::path::Path::new(DT_DMA).exists() {
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
    /// Busy (0xFF04 bit 0) low within `limit`?
    fn wait_idle(&self, limit: std::time::Duration) -> bool {
        let t0 = std::time::Instant::now();
        while self.rd(0xFF04) & 1 == 1 {
            if t0.elapsed() > limit {
                return false;
            }
            std::thread::sleep(std::time::Duration::from_micros(500));
        }
        true
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
fn run_fpga(win: &Window, rate: LongRate, q: &mut [u32], max_iter: u32, need: usize, bits: &mut [u8], bb: &mut BbOut, t_q: std::time::Instant, cells: Option<&crate::dvbt2::stream::CellParams>, ddr_in: Option<u32>) -> Result<Option<usize>, Stuck> {
    run_fpga_ring(win, rate, q, max_iter, need, bits, bb, t_q, cells, ddr_in, None)
}

/// What the engine needs to read a DVB-S2 frame from the receive ring
/// (s2ring): its first word (128-aligned) and how many to read with the
/// lead, the ring, the gain and lead word, the segment table.
struct RingRegs<'a> {
    in_addr: u32,
    in_words: u32,
    gain_lead: u32,
    segs: &'a [(u32, u32)],
}

#[allow(clippy::too_many_arguments)]
fn run_fpga_ring(win: &Window, rate: LongRate, q: &mut [u32], max_iter: u32, need: usize, bits: &mut [u8], bb: &mut BbOut, t_q: std::time::Instant, cells: Option<&crate::dvbt2::stream::CellParams>, ddr_in: Option<u32>, ring: Option<&RingRegs>) -> Result<Option<usize>, Stuck> {
    use std::sync::atomic::Ordering::Relaxed;
    bb.valid = false;
    bb.tried = false;
    // BBFRAME out: only for a caller that takes it, through the DDR engine.
    let bb_on = bb.want && win.bb && win.dma.is_some();
    let _one = DECODER.lock().unwrap_or_else(|e| e.into_inner());
    // Never touch the registers or the buffers of a decode still running.
    let stuck = STUCK.load(Relaxed);
    if !win.wait_idle(if stuck { std::time::Duration::from_millis(1) } else { LATE_WAIT }) {
        if !STUCK.swap(true, Relaxed) {
            tracing::error!("LDPC: the FPGA decoder stays busy: decoding in software until it is idle");
        }
        return Err(Stuck);
    }
    if stuck {
        STUCK.store(false, Relaxed);
        tracing::warn!("LDPC: the FPGA decoder is idle again");
    }
    let done0 = win.rd(0xFF04) >> 24;
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
        if let Some(r) = ring {
            win.wr(0xFF10, r.in_addr);
            win.wr(0xFF18, r.in_words);
            win.wr(0xFF34, super::fpga::RING_START);
            win.wr(0xFF38, super::fpga::RING_END);
            win.wr(0xFF3C, r.gain_lead);
            for (i, &(a, b)) in r.segs.iter().enumerate() {
                win.wr(0xFF40, a);
                win.wr(0xFF44, b);
                win.wr(0xFF48, i as u32);
            }
        }
        match cells {
            Some(p) => {
                win.wr(0xFF20, 1 | (p.rot as u32) << 1 | (p.qam16.is_some() as u32) << 3 | (p.psk8 as u32) << 4 | (ring.is_some() as u32) << 5 | (bb_on as u32) << 6);
                win.wr(0xFF24, p.kq as u32);
                win.wr(0xFF28, (p.c14 as u32 & 0xFFFF) | (p.s14 as u32) << 16);
                if p.qam16.is_some() {
                    win.wr(0xFF2C, p.a14 as u32);
                }
            }
            None => win.wr(0xFF20, (bb_on as u32) << 6),
        }
        ctl |= 4;
    } else {
        win.write_words(0, q);
    }
    let t_in = std::time::Instant::now();
    bb.tried = bb_on;
    win.wr(0xFF00, ctl);
    // 2.5 ms an iteration: sleep in small steps until done.
    let t0 = std::time::Instant::now();
    if !win.wait_idle(DECODE_WAIT) {
        // Keep the decoder (the lock) until this decode is over: the next
        // frame's registers and buffers must not go in under it.
        let late = win.wait_idle(LATE_WAIT);
        tracing::warn!(late_end = late, "LDPC: the FPGA decoder did not finish in time (frame dropped)");
        if !late {
            STUCK.store(true, Relaxed);
            tracing::error!("LDPC: the FPGA decoder stays busy: decoding in software until it is idle");
        }
        return Ok(None);
    }
    let t_out = std::time::Instant::now();
    let st = win.rd(0xFF04);
    // This start really ran (the busy edge counted once).
    if win.feat & 1 != 0 && (st >> 24) != (done0 + 1) & 0xFF {
        warn_every("LDPC: the FPGA decoder did not run this frame (start not taken)");
        return Ok(None);
    }
    if win.feat & 2 != 0 && st & 4 != 0 {
        warn_every("LDPC: AXI error in the FPGA decoder's DDR engine (frame dropped)");
        return Ok(None);
    }
    // Decisions: the sign bits of the BCH codeword part (the
    // parity bits after it are never read).
    if let Some(d) = win.dma {
        // packed: bit j of word k is variable 32 k + j (the info part is in
        // natural order in the RAM)
        if bb_on {
            // the BBFRAME's bytes (and the parity after them), as they are
            let nbytes = need / 8;
            bb.bytes.resize(nbytes, 0);
            // SAFETY: the mapped buffers: nbytes <= out_words * 4 bytes at DMA_OUT.
            unsafe { std::ptr::copy_nonoverlapping(d.add(DMA_OUT / 4).cast::<u8>(), bb.bytes.as_mut_ptr(), nbytes) };
            bb.rem = if st & 8 != 0 {
                [0; 3]
            } else {
                let r: Vec<u64> = (0..6).map(|n| win.rd(0xFF4C + 4 * n) as u64).collect();
                [r[0] | r[1] << 32, r[2] | r[3] << 32, r[4] | r[5] << 32]
            };
            bb.valid = true;
        } else {
            let mut w = vec![0u32; out_words];
            // SAFETY: the mapped buffers: out_words (<= 2048) words at DMA_OUT.
            unsafe { std::ptr::copy_nonoverlapping(d.add(DMA_OUT / 4), w.as_mut_ptr(), out_words) };
            for (i, b) in bits[..need].iter_mut().enumerate() {
                *b = ((w[i / 32] >> (i % 32)) & 1) as u8;
            }
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
    PROF_NS[0].fetch_add((t_in - t_q).as_nanos() as u64, Relaxed);
    PROF_NS[1].fetch_add((t_out - t0).as_nanos() as u64, Relaxed);
    PROF_ITER[0].fetch_add(1, Relaxed);
    PROF_ITER[1].fetch_add(((st >> 8) & 0x3F) as u64, Relaxed);
    if (st >> 1) & 1 == 0 {
        PROF_ITER[2].fetch_add(1, Relaxed);
    }
    PROF_NS[2].fetch_add(t_out.elapsed().as_nanos() as u64, Relaxed);
    Ok(((st >> 1) & 1 == 1).then_some(((st >> 8) & 0x3F) as usize))
}

/// The FPGA engine is stuck busy: nothing was started.
pub struct Stuck;

/// A warning at most every 10 s (the rest counted).
fn warn_every(msg: &'static str) {
    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
    static LAST: AtomicU64 = AtomicU64::new(0);
    static SKIPPED: AtomicU64 = AtomicU64::new(0);
    let now = crate::stream::unix_now() as u64;
    if now >= LAST.load(Relaxed) + 10 {
        LAST.store(now, Relaxed);
        tracing::warn!(more = SKIPPED.swap(0, Relaxed), "{msg}");
    } else {
        SKIPPED.fetch_add(1, Relaxed);
    }
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

/// Before the PL is reloaded (fpgamode): no decode running and none to
/// start (the decoder kept until the process exits).
pub fn quiesce() {
    if let Ok(w) = Window::open() {
        let one = DECODER.lock().unwrap_or_else(|e| e.into_inner());
        if !w.wait_idle(std::time::Duration::from_secs(1)) {
            tracing::warn!("LDPC: the FPGA decoder still busy before the reload");
        }
        std::mem::forget(one);
    }
}

/// Is the FPGA decoder there?
pub fn available() -> bool {
    Window::open().is_ok()
}

/// Does the FPGA decoder take DVB-T2 QPSK cells (it makes the LLRs)?
pub fn cells_available() -> bool {
    static CELLS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *CELLS.get_or_init(|| Window::open().is_ok_and(|w| w.dma.is_some()))
}

/// DVB-S2 8PSK cells (0xFF30 bit 3: the max-log demapper and the 3-column
/// deinterleaver in the engine)?
pub fn psk8_cells_available() -> bool {
    static CELLS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *CELLS.get_or_init(|| cells_available() && Window::open().is_ok_and(|w| w.feat & 8 != 0))
}

/// DVB-S2 long frames straight from the receive ring (0xFF30 bit 4:
/// s2front.py, the CPU no longer touches the data symbols)?
pub fn ring_available() -> bool {
    static RING: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *RING.get_or_init(|| psk8_cells_available() && Window::open().is_ok_and(|w| w.feat & 16 != 0))
}

/// And 16QAM cells (four LLRs a cell and the bit deinterleaver there too)?
pub fn cells16_available() -> bool {
    static CELLS: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *CELLS.get_or_init(|| cells_available() && Window::open().is_ok_and(|w| w.qam16))
}

/// A decode's BBFRAME from the fabric ([`Ldpc::take_bb`]).
#[derive(Default)]
pub struct BbOut {
    /// The decisions as bytes, MSB first: the BBFRAME (descrambled), then
    /// the BCH and LDPC parity.
    pub bytes: Vec<u8>,
    /// The BCH remainder of the first Nbch decisions (zero: valid).
    pub rem: super::bch::Reg,
    /// Filled by the last decode and not taken yet.
    valid: bool,
    /// The last decode ran in this mode (its bits were not written, even
    /// when it produced nothing: a time-out).
    tried: bool,
    /// The caller takes BBFRAMEs (and not bits) from FPGA decodes.
    want: bool,
}

pub enum Ldpc {
    Model(FpgaDecoder),
    /// `need`: decisions read back (the BCH codeword's Kbch + 192 bits).
    /// `perm` (four-lane decoder): for each parity byte of the RAM in
    /// order, its parity bit.
    /// `model`: the software decoder, made when the engine is stuck.
    Fpga { win: Window, rate: LongRate, q: Vec<u32>, max_iter: u32, need: usize, perm: Vec<u32>, model: Option<Box<FpgaDecoder>>, bb: BbOut },
}

impl Ldpc {
    pub fn for_spec(spec: &FrameSpec) -> Ldpc {
        let rate = spec.long_rate;
        match Window::open() {
            Ok(win) => {
                tracing::info!(?rate, lanes = win.lanes, dma = win.dma.is_some(), "LDPC: the FPGA decoder");
                let perm = if win.lanes == 4 { parity_layout(rate) } else { Vec::new() };
                Ldpc::Fpga { win, rate, q: vec![0; N / 4], max_iter: MAX_ITER, need: (spec.kbch + 192).min(N), perm, model: None, bb: BbOut::default() }
            }
            Err(e) => {
                tracing::info!(?rate, "LDPC: FPGA decoder unavailable ({e}); its model in software");
                Ldpc::Model(FpgaDecoder::new(rate))
            }
        }
    }

    /// The caller takes the BBFRAME bytes and BCH remainder of FPGA
    /// decodes ([`Self::take_bb`]) where the engine makes them; decodes it
    /// does not (no engine, an older core, the model) still fill the bits.
    pub fn want_bb(&mut self) {
        if let Ldpc::Fpga { bb, .. } = self {
            bb.want = true;
        }
    }

    /// The last decode's BBFRAME from the fabric (its bits were not
    /// unpacked then); give the buffer back with [`Self::return_bb`].
    pub fn take_bb(&mut self) -> Option<(Vec<u8>, super::bch::Reg)> {
        match self {
            Ldpc::Fpga { bb, .. } if bb.valid => {
                bb.valid = false;
                bb.tried = false;
                Some((std::mem::take(&mut bb.bytes), bb.rem))
            }
            _ => None,
        }
    }

    /// The last decode ran with the BBFRAME out but gave nothing (its bits
    /// are not this frame's either): lost.
    pub fn bb_lost(&mut self) -> bool {
        match self {
            Ldpc::Fpga { bb, .. } => std::mem::take(&mut bb.tried) && !bb.valid,
            _ => false,
        }
    }

    pub fn return_bb(&mut self, v: Vec<u8>) {
        if let Ldpc::Fpga { bb, .. } = self {
            bb.bytes = v;
        }
    }

    /// The FPGA decoder (fast enough for the full budget with a queue).
    pub fn is_fpga(&self) -> bool {
        matches!(self, Ldpc::Fpga { .. })
    }

    /// At most `n` iterations (fewer when frames queue up behind this one).
    pub fn set_max_iter(&mut self, n: usize) {
        match self {
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
        if let Ldpc::Fpga { win, rate, q, max_iter, need, bb, .. } = self {
            let per_cell = if p.psk8 { 3 } else if p.qam16.is_some() { 4 } else { 2 };
            if win.dma.is_some() && cells.len() * per_cell == N && (p.qam16.is_none() || win.qam16) && (!p.psk8 || win.feat & 8 != 0) {
                let t_q = std::time::Instant::now();
                for (w, c) in q.iter_mut().zip(cells.chunks_exact(2)) {
                    *w = (c[0][0] as u8 as u32) | (c[0][1] as u8 as u32) << 8 | (c[1][0] as u8 as u32) << 16 | (c[1][1] as u8 as u32) << 24;
                }
                let words = cells.len() / 2;
                if let Ok(r) = run_fpga(win, *rate, &mut q[..words], *max_iter, *need, bits, bb, t_q, Some(p), None) {
                    return r;
                }
            }
        }
        let llr = crate::dvbt2::stream::cell_llrs(cells, p);
        self.decode_q(&llr, bits)
    }

    /// A DVB-S2 long frame the engine reads from the receive ring itself
    /// (s2ring). Ok(None) when it did not decode; Err(()) when the frame is
    /// no longer (or not wholly) in the ring, or there is no engine for it.
    pub fn decode_ring(&mut self, job: &super::s2ring::Job, watch: &super::fpga::RingWatch, bits: &mut [u8]) -> Result<Option<usize>, ()> {
        use super::fpga::{RING_WORDS, ring_addr};
        let Ldpc::Fpga { win, rate, q, max_iter, need, bb, .. } = self else { return Err(()) };
        if win.dma.is_none() || win.feat & 16 == 0 {
            return Err(());
        }
        let nsym = super::s2ring::frame_symbols(job.n_cells, job.pilots) as u64;
        let lead = job.at % 32;
        // The frame's start must stay in the ring until the engine has read
        // it all (a few ms): margin for that at any rate here.
        match watch.ahead_of(job.at) {
            Some(a) if super::s2ring::in_ring(a, nsym) => {}
            _ => return Err(()),
        }
        let p = super::s2cells::params(job.bps as usize, job.kq);
        let r = RingRegs {
            in_addr: ring_addr(job.at - lead),
            in_words: (lead + nsym) as u32,
            gain_lead: job.gain | (lead as u32) << 24 | (job.pilots as u32) << 31,
            segs: &job.segs,
        };
        let t_q = std::time::Instant::now();
        let res = run_fpga_ring(win, *rate, &mut q[..1], *max_iter, *need, bits, bb, t_q, Some(&p), Some(r.in_addr), Some(&r)).map_err(|_| ())?;
        // read while the DMA still had the start? (lapped during the decode)
        match watch.ahead_of(job.at) {
            Some(a) if a < RING_WORDS => Ok(res),
            _ => Err(()),
        }
    }

    /// A DVB-T2 QPSK block whose cells the FPGA's cell router put in DDR at
    /// `addr` (only with the FPGA decoder's DDR engine: None otherwise).
    pub fn decode_ddr(&mut self, addr: u32, p: &crate::dvbt2::stream::CellParams, bits: &mut [u8]) -> Option<usize> {
        if let Ldpc::Fpga { win, rate, q, max_iter, need, bb, .. } = self {
            if win.dma.is_some() && (p.qam16.is_none() || win.qam16) {
                let t_q = std::time::Instant::now();
                let words = if p.qam16.is_some() { N / 8 } else { N / 4 };
                // (stuck: the cells are only in DDR, the block is lost)
                return run_fpga(win, *rate, &mut q[..words], *max_iter, *need, bits, bb, t_q, Some(p), Some(addr)).ok().flatten();
            }
        }
        None
    }

    pub fn decode_q(&mut self, llr: &[i8], bits: &mut [u8]) -> Option<usize> {
        match self {
            Ldpc::Model(d) => d.decode(llr, bits),
            Ldpc::Fpga { win, rate, q, max_iter, need, perm, model, bb } => {
                let t_q = std::time::Instant::now();
                pack(q, perm, rate.k(), |i| llr[i] as u8 as u32);
                match run_fpga(win, *rate, q, *max_iter, *need, bits, bb, t_q, None, None) {
                    Ok(r) => r,
                    Err(Stuck) => soft(model, *rate, *max_iter, llr, bits),
                }
            }
        }
    }

    /// Decode `llr` (positive = 0) into `bits`; the iterations when every
    /// check is satisfied.
    pub fn decode(&mut self, llr: &[f32], bits: &mut [u8]) -> Option<usize> {
        match self {
            Ldpc::Model(d) => {
                let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, LLR_SCALE)).collect();
                d.decode(&q, bits)
            }
            Ldpc::Fpga { win, rate, q, max_iter, need, perm, model, bb } => {
                let t_q = std::time::Instant::now();
                // Four 6-bit LLRs a word, then one bulk copy into the
                // window (word writes one at a time cost 10 ms a frame).
                let b = |l: f32| quantize_llr(l, LLR_SCALE) as u8 as u32;
                pack(q, perm, rate.k(), |i| b(llr[i]));
                match run_fpga(win, *rate, q, *max_iter, *need, bits, bb, t_q, None, None) {
                    Ok(r) => r,
                    Err(Stuck) => {
                        let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, LLR_SCALE)).collect();
                        soft(model, *rate, *max_iter, &q, bits)
                    }
                }
            }
        }
    }
}

/// The model decodes while the FPGA engine is stuck (the same decisions,
/// tens of times slower).
fn soft(model: &mut Option<Box<FpgaDecoder>>, rate: LongRate, max_iter: u32, llr: &[i8], bits: &mut [u8]) -> Option<usize> {
    let d = model.get_or_insert_with(|| Box::new(FpgaDecoder::new(rate)));
    d.max_iter = max_iter as usize;
    d.decode(llr, bits)
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
            let p = CellParams { rot: true, kq: (2048.0 * 1024.0 / (sigma * sigma)).min(65536.0) as i32, c14: (c * 16384.0).round() as i32, s14: (-s * 16384.0).round() as i32, qam16: None, a14: 0, psk8: false };
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
            let p = CellParams { rot: true, kq: 1811, c14: 15685, s14: -4739, qam16: Some(rate), a14: (2.0 * 40.0 / 10f64.sqrt() * 16384.0).round() as i32, psk8: false };
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
