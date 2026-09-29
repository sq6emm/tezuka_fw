//! The CW-RS keying detector's front end in the FPGA (maia-sdr
//! `rsnn_front.py`, 0x43C50000): [`crate::rsnn::Net::front_q`] bit for bit,
//! a feature row in, the 1x1's outputs for the frame three rows back out.
//! The temporal layers stay on the ARM.
//!
//! Window: 0x0000.. weights (two a word, the even one low), 0x9000..
//! biases, 0x9400.. the next row (two bins a word), 0x9500.. the outputs
//! (i32), 0xFF00 control (bit 0 go, bit 1 reset; 12:8 c2, 22:16 c1),
//! 0xFF04 status (bit 0 busy, bit 1 valid), 0xFF08 id "RSF1".
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
}

// SAFETY: owned by the detector's thread alone.
unsafe impl Send for FpgaFront {}

impl FpgaFront {
    /// The engine loaded with `net`'s front, if the bitstream has one, it is
    /// free and the network fits.
    pub fn open(net: &Net) -> Option<FpgaFront> {
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
                tracing::info!("rsnn: front end in the FPGA ({c2}/{c1} channels, ~{} us a frame)", f.frame_us);
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
                0 => libc::_exit((self.rd(0xFF08) == ID) as i32),
                -1 => false,
                pid => {
                    let mut st = 0;
                    libc::waitpid(pid, &mut st, 0) == pid && libc::WIFEXITED(st) && libc::WEXITSTATUS(st) == 1
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

    /// A fresh stream: the rows before the next are zeros.
    pub fn reset(&mut self) {
        self.wait();
        self.wr(0xFF00, self.cfg | 2);
        self.wait();
    }

    /// Row n in (as [`Net::front_q`] quantizes it); the outputs for frame
    /// n - 3 (x AQ), from the fourth row on.
    pub fn push(&mut self, row: &[f32; NB]) -> Option<&[u32]> {
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
        if self.rd(0xFF04) & 2 == 0 {
            return None;
        }
        let mut out = std::mem::take(&mut self.out);
        self.read_words(0x9500, &mut out[..self.c1]);
        self.out = out;
        Some(&self.out)
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
