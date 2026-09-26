//! The DATV receive front end in the FPGA (see [`super::ddc`] and
//! docs/DATV-FPGA.md): Maia's DDC channel- and matched-filters the ADC
//! samples to 2 per symbol, and the Maia recorder, in ring mode, writes them
//! into 1 MiB of reserved memory that this module follows.
//!
//! Registers (Maia core at 0x7C460000, maia-sdr.svd of `maia_iio_lite_datv`):
//! 0x00 product id "maia"; 0x04 version, bits 31:24 platform = 0xD5 for
//! this core; 0x10 recorder_control (0 start, 1 stop, 3:2 mode, 4
//! dropped_samples); 0x18 recorder_committed_address; 0x24 ddc_coeff_addr;
//! 0x28 ddc_coeff (0 wren, 18:1 data); 0x2C ddc_decimation (6:0, 12:7,
//! 19:13); 0x30 ddc_frequency (27:0); 0x34 ddc_control (6:0, 12:7, 19:13
//! operations - 1, 20/21 odd operations 1/3, 22/23 bypass 2/3, 24 enable).
//!
//! Only a core that says it is the ring one (platform 0xD5) is touched, and
//! only when the device tree reserves the ring: any other Maia core's
//! recorder writes 128 MB of RAM Linux owns.

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use num_complex::Complex32;

use super::ddc::{Design, design, frequency_word};

const REGS_PHYS: u64 = 0x7C46_0000;
pub const RING_START: u32 = 0x1610_0000;
pub const RING_END: u32 = 0x1620_0000;
const RING_BYTES: usize = (RING_END - RING_START) as usize;
/// The DDC's input: the AD936x rate before the x8 decimator.
pub const FS_IN: f64 = 3_072_000.0;
const PLATFORM_DATV: u32 = 0xD5;
const DT_RING: &str = "/proc/device-tree/reserved-memory/maia_sdr_datv_ring@16100000";

const REG_ID: usize = 0x00;
const REG_VERSION: usize = 0x04;
const REG_REC_CONTROL: usize = 0x10;
const REG_REC_COMMITTED: usize = 0x18;
const REG_COEFF_ADDR: usize = 0x24;
const REG_COEFF: usize = 0x28;
const REG_DECIMATION: usize = 0x2C;
const REG_FREQUENCY: usize = 0x30;
const REG_DDC_CONTROL: usize = 0x34;

struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: used by one thread at a time (the owner moves it into its thread).
unsafe impl Send for Mapping {}

impl Mapping {
    fn new(f: &File, len: usize, phys: u64) -> Result<Mapping, String> {
        // SAFETY: MAP_SHARED of a register window / reserved region; every
        // access below stays inside `len` and is 4-byte aligned.
        let p = unsafe {
            libc::mmap(std::ptr::null_mut(), len, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, f.as_raw_fd(), phys as libc::off_t)
        };
        if p == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error().to_string());
        }
        Ok(Mapping { ptr: p.cast(), len })
    }
    fn rd32(&self, off: usize) -> u32 {
        assert!(off + 4 <= self.len);
        // SAFETY: see new().
        unsafe { std::ptr::read_volatile(self.ptr.add(off).cast::<u32>()) }
    }
    fn wr32(&self, off: usize, v: u32) {
        assert!(off + 4 <= self.len);
        // SAFETY: see new().
        unsafe { std::ptr::write_volatile(self.ptr.add(off).cast::<u32>(), v) }
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

/// Is the FPGA front end there (right core, ring reserved)? Cheap; no side effects.
pub fn available() -> bool {
    std::path::Path::new(DT_RING).exists() && FrontEnd::open_regs().is_ok_and(|(_, r)| is_datv_core(&r))
}

fn is_datv_core(regs: &Mapping) -> bool {
    regs.rd32(REG_ID).to_le_bytes() == *b"maia" && regs.rd32(REG_VERSION) >> 24 == PLATFORM_DATV
}

/// The running front end: DDC set up for one symbol rate, recorder running.
pub struct FrontEnd {
    _mem: File,
    regs: Mapping,
    ring: Mapping,
    design: Design,
    /// Next physical address to read.
    rd: u32,
    center_hz: f64,
}

impl FrontEnd {
    fn open_regs() -> Result<(File, Mapping), String> {
        let mem = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC) // uncached: the DMA writes behind the CPU's back
            .open("/dev/mem")
            .map_err(|e| format!("/dev/mem: {e}"))?;
        let regs = Mapping::new(&mem, 4096, REGS_PHYS).map_err(|e| format!("map Maia registers: {e}"))?;
        Ok((mem, regs))
    }

    /// Set the DDC up for `rs` (whole even ADC samples per symbol, >= 8)
    /// with the signal `center_hz` from the LO, and start the ring.
    pub fn start(rs: f64, rolloff: f32, center_hz: f64) -> Result<FrontEnd, String> {
        if !std::path::Path::new(DT_RING).exists() {
            return Err("no DATV ring reserved in the device tree".into());
        }
        let (mem, regs) = Self::open_regs()?;
        if !is_datv_core(&regs) {
            return Err(format!("the FPGA has no DATV ring recorder (version {:#010x})", regs.rd32(REG_VERSION)));
        }
        let ring = Mapping::new(&mem, RING_BYTES, RING_START as u64).map_err(|e| format!("map DATV ring: {e}"))?;
        let design = design(FS_IN, rs, rolloff)?;
        let r = design.registers();
        // Stop a ring left running by a previous instance before rewriting
        // the filters under it (a stop while stopped is harmless here: the
        // DMA just stays idle).
        regs.wr32(REG_REC_CONTROL, 1 << 1);
        std::thread::sleep(std::time::Duration::from_millis(2));
        for &(addr, c) in &r.coeffs {
            regs.wr32(REG_COEFF_ADDR, addr as u32);
            regs.wr32(REG_COEFF, 1 | (((c as u32) & 0x3_FFFF) << 1));
        }
        regs.wr32(REG_DECIMATION, r.decimation[0] as u32 | (r.decimation[1] as u32) << 7 | (r.decimation[2] as u32) << 13);
        regs.wr32(REG_FREQUENCY, frequency_word(center_hz, FS_IN) & 0x0FFF_FFFF);
        regs.wr32(
            REG_DDC_CONTROL,
            r.operations_minus_one[0] as u32
                | (r.operations_minus_one[1] as u32) << 7
                | (r.operations_minus_one[2] as u32) << 13
                | (r.odd_operations[0] as u32) << 20
                | (r.odd_operations[1] as u32) << 21
                | (r.bypass2 as u32) << 22
                | (r.bypass3 as u32) << 23
                | 1 << 24,
        );
        // 16-bit mode (0), start: the ring fills from RING_START.
        regs.wr32(REG_REC_CONTROL, 1);
        Ok(FrontEnd { _mem: mem, regs, ring, design, rd: RING_START, center_hz })
    }

    /// Output rate (2 samples per symbol).
    pub fn fs_out(&self) -> f64 {
        self.design.fs_out()
    }

    /// The signal moved (LO retuned): move the DDC's NCO with it.
    pub fn set_center(&mut self, hz: f64) {
        if (hz - self.center_hz).abs() > 0.1 {
            self.center_hz = hz;
            self.regs.wr32(REG_FREQUENCY, frequency_word(hz, FS_IN) & 0x0FFF_FFFF);
        }
    }

    /// The recorder's FIFO overflowed at some point (samples lost).
    pub fn dropped(&self) -> bool {
        self.regs.rd32(REG_REC_CONTROL) & (1 << 4) != 0
    }

    /// Everything the DMA has committed since the last call, as complex
    /// samples (full scale 1.0). Call often: the ring holds 0.5 s at 512 kS/s.
    pub fn read(&mut self, out: &mut Vec<Complex32>) {
        let c = self.regs.rd32(REG_REC_COMMITTED);
        if !(RING_START..RING_END).contains(&c) {
            return;
        }
        let (a, b) = (self.rd, c);
        if b >= a {
            self.copy(a, b, out);
        } else {
            self.copy(a, RING_END, out);
            self.copy(RING_START, b, out);
        }
        self.rd = c;
    }

    fn copy(&self, from: u32, to: u32, out: &mut Vec<Complex32>) {
        let (s, e) = ((from - RING_START) as usize, (to - RING_START) as usize);
        out.reserve((e - s) / 4);
        for off in (s..e).step_by(4) {
            let w = self.ring.rd32(off);
            // Recorder16IQ: re in the low half, im in the high half.
            let (re, im) = (w as u16 as i16, (w >> 16) as u16 as i16);
            out.push(Complex32::new(re as f32 / 32768.0, im as f32 / 32768.0));
        }
    }
}

impl Drop for FrontEnd {
    fn drop(&mut self) {
        self.regs.wr32(REG_REC_CONTROL, 1 << 1);
        let ctl = self.regs.rd32(REG_DDC_CONTROL);
        self.regs.wr32(REG_DDC_CONTROL, ctl & !(1 << 24));
    }
}
