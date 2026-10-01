//! The FPGA's DVB-T2 cell router (maia-sdr `t2router.py`, 0x43C60000, id
//! "T2R1", datv bitstream): every data cell of an equalized symbol goes
//! from the equalizer straight to its place in a FEC block of a frame buffer
//! in DDR, through a table made here once (the time and cell deinterleavers
//! in one: entry (symbol j, carrier k) at table + 8192 j + 4 k, the cell's
//! r cells + q, or all ones). The LDPC decoder's DDR engine takes the blocks
//! from there (cells: it makes the LLRs), so the A9 never touches a data
//! cell of an equalized symbol; the P2 symbols' few it writes itself.
//!
//! Reserved memory (device tree t2router_buffers): 4 MB at 0x16400000, the
//! table at +0 (up to 224 symbols), four frame buffers from +0x1C0000,
//! 0x8F000 apart. Registers: 0x00 control (bit 0 enable), 0x04 table, 0x08
//! frame buffers, 0x0C their stride, 0x10 symbols a frame, 0x14 the first
//! data symbol; 0x20 + 4 b buffer b (bit 31 complete, 20:0 its frame's
//! start F); 0x30 frames, 0x34 symbols, 0x38 words lost, 0x3C id.

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

const REGS: u64 = 0x43C6_0000;
const ID: u32 = 0x3152_3254;
const DDR: u64 = 0x1640_0000;
const DDR_BYTES: usize = 0x40_0000;
const TABLE: usize = 0;
const TABLE_MAX: usize = 0x1C_0000;
const FB: usize = 0x1C_0000;
const STRIDE: usize = 0x8_F000;
const DT: &str = "/proc/device-tree/reserved-memory/t2router_buffers@16400000";
pub const SKIP: u32 = u32::MAX;
/// Table entries a symbol (8 KB).
pub const ROW: usize = 2048;

/// FEC block `r` starts at cell `r * block_stride(cells)`: a multiple of 64
/// cells (128 bytes), so the LDPC engine's 128-byte bursts are aligned and
/// never cross a 4 KB boundary (AXI3). 9 QPSK blocks (32448 cells) and 18
/// 16QAM (16256) still fit a buffer (STRIDE).
pub fn block_stride(cells: usize) -> usize {
    (cells + 63) & !63
}

/// The buffers' tags, readable from another thread: the decoding thread
/// checks a block's buffer still holds its frame (the router has four and
/// reuses them about a second later).
pub struct Tags {
    _mem: File,
    regs: *const u32,
}

// SAFETY: a read-only mapping of the router's registers, read with volatile
// word loads only.
unsafe impl Send for Tags {}
unsafe impl Sync for Tags {}

impl Tags {
    /// Buffer `b` complete with frame `f21` (its start, low 21 bits).
    pub fn holds(&self, b: usize, f21: u32) -> bool {
        // SAFETY: word 0x20 + 4 b (b < 4) inside the 64 KiB mapping.
        let v = unsafe { std::ptr::read_volatile(self.regs.add((0x20 + 4 * (b & 3)) / 4)) };
        v & 0x1F_FFFF == f21 & 0x1F_FFFF && v >> 31 == 1
    }
}

impl Drop for Tags {
    fn drop(&mut self) {
        // SAFETY: unmapping what Router::tags mapped.
        unsafe { libc::munmap(self.regs as *mut libc::c_void, 0x1_0000) };
    }
}

pub struct Router {
    _mem: File,
    regs: *mut u32,
    ddr: *mut u8,
}

// SAFETY: owned by the DVB-T2 receive thread alone.
unsafe impl Send for Router {}

impl Router {
    /// The router, if the bitstream has it and the device tree reserves its
    /// memory (TRXD_NO_T2ROUTER=1: never).
    pub fn open() -> Option<Router> {
        if std::env::var_os("TRXD_NO_T2ROUTER").is_some() || !std::path::Path::new(DT).exists() {
            return None;
        }
        let mem = OpenOptions::new().read(true).write(true).custom_flags(libc::O_SYNC).open("/dev/mem").ok()?;
        // SAFETY: MAP_SHARED of the router's 64 KiB window and of its
        // reserved memory; accessed as aligned words / halfwords inside.
        let regs = unsafe { libc::mmap(std::ptr::null_mut(), 0x1_0000, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), REGS as libc::off_t) };
        if regs == libc::MAP_FAILED {
            return None;
        }
        let ddr = unsafe { libc::mmap(std::ptr::null_mut(), DDR_BYTES, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, mem.as_raw_fd(), DDR as libc::off_t) };
        if ddr == libc::MAP_FAILED {
            // SAFETY: unmapping what was mapped above.
            unsafe { libc::munmap(regs, 0x1_0000) };
            return None;
        }
        let r = Router { _mem: mem, regs: regs.cast(), ddr: ddr.cast() };
        // A bitstream without the router: nothing at the address (a bus
        // error): look from a child first.
        static THERE: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if !*THERE.get_or_init(|| r.probe()) {
            return None;
        }
        Some(r)
    }

    fn probe(&self) -> bool {
        // SAFETY: the child only reads the mapping and _exits; the parent waits.
        unsafe {
            match libc::fork() {
                0 => libc::_exit((self.rd(0x3C) == ID) as i32),
                -1 => false,
                pid => {
                    let mut st = 0;
                    libc::waitpid(pid, &mut st, 0) == pid && libc::WIFEXITED(st) && libc::WEXITSTATUS(st) == 1
                }
            }
        }
    }

    fn rd(&self, off: usize) -> u32 {
        // SAFETY: see open().
        unsafe { std::ptr::read_volatile(self.regs.add(off / 4)) }
    }

    fn wr(&self, off: usize, v: u32) {
        // SAFETY: see open().
        unsafe { std::ptr::write_volatile(self.regs.add(off / 4), v) }
    }

    /// The table (symbols x [`ROW`] entries) in, the registers set, on.
    /// False if it does not fit.
    pub fn start(&mut self, table: &[u32], nsym: usize, j0: usize, frame_bytes: usize) -> bool {
        if table.len() * 4 > TABLE_MAX || frame_bytes > STRIDE || nsym > 255 {
            return false;
        }
        self.wr(0x00, 0);
        // SAFETY: inside the mapping (checked above), word-aligned.
        unsafe { std::ptr::copy_nonoverlapping(table.as_ptr(), self.ddr.add(TABLE).cast::<u32>(), table.len()) };
        self.wr(0x04, (DDR as usize + TABLE) as u32);
        self.wr(0x08, (DDR as usize + FB) as u32);
        self.wr(0x0C, STRIDE as u32);
        self.wr(0x10, nsym as u32);
        self.wr(0x14, j0 as u32);
        self.wr(0x00, 1);
        true
    }

    /// The buffer holding frame `f21` (its start, low 21 bits), once complete
    /// (waits up to `wait`).
    pub fn frame_buffer(&self, f21: u32, wait: std::time::Duration) -> Option<usize> {
        let t0 = std::time::Instant::now();
        loop {
            for b in 0..4 {
                let v = self.rd(0x20 + 4 * b);
                if v & 0x1F_FFFF == f21 & 0x1F_FFFF && v >> 31 == 1 {
                    return Some(b);
                }
            }
            if t0.elapsed() >= wait {
                return None;
            }
            std::thread::sleep(std::time::Duration::from_micros(500));
        }
    }

    /// A second, read-only mapping of the registers for [`Tags`].
    pub fn tags(&self) -> Option<Tags> {
        let mem = OpenOptions::new().read(true).custom_flags(libc::O_SYNC).open("/dev/mem").ok()?;
        // SAFETY: MAP_SHARED, read-only, of the router's 64 KiB window (the
        // router was probed in open()).
        let regs = unsafe { libc::mmap(std::ptr::null_mut(), 0x1_0000, libc::PROT_READ, libc::MAP_SHARED, mem.as_raw_fd(), REGS as libc::off_t) };
        if regs == libc::MAP_FAILED {
            return None;
        }
        Some(Tags { _mem: mem, regs: regs.cast() })
    }

    /// A cell the A9 made (a P2 symbol's) into buffer `b` at `dest`.
    pub fn write_cell(&self, b: usize, dest: usize, c: [i8; 2]) {
        let off = FB + b * STRIDE + 2 * dest;
        // SAFETY: inside the mapping (dest < the frame's cells, frame_bytes
        // <= STRIDE checked in start), halfword-aligned.
        unsafe { std::ptr::write_volatile(self.ddr.add(off).cast::<u16>(), c[0] as u8 as u16 | (c[1] as u8 as u16) << 8) };
    }

    /// A cell of buffer `b` back (checks: uncached, slow).
    pub fn read_cell(&self, b: usize, dest: usize) -> [i8; 2] {
        let off = FB + b * STRIDE + 2 * dest;
        // SAFETY: as write_cell.
        let v = unsafe { std::ptr::read_volatile(self.ddr.add(off).cast::<u16>()) };
        [v as u8 as i8, (v >> 8) as u8 as i8]
    }

    /// The physical address of cell `dest` of buffer `b`.
    pub fn addr(&self, b: usize, dest: usize) -> u32 {
        (DDR as usize + FB + b * STRIDE + 2 * dest) as u32
    }

    /// (frames started, symbols, words lost)
    pub fn counters(&self) -> (u32, u32, u32) {
        (self.rd(0x30), self.rd(0x34), self.rd(0x38))
    }
}

impl Drop for Router {
    fn drop(&mut self) {
        self.wr(0x00, 0);
        // SAFETY: unmapping what open() mapped.
        unsafe {
            libc::munmap(self.regs.cast(), 0x1_0000);
            libc::munmap(self.ddr.cast(), DDR_BYTES);
        }
    }
}
