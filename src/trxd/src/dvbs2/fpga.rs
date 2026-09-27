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
//! operations - 1, 20/21 odd operations 1/3, 22/23 bypass 2/3, 24 enable);
//! 0x38 datv_symsync (0 enable, 5:1 kp shift, 10:6 ki shift, 11 header
//! detector [`super::hdrdet`]: flags in the ring word's bit 16) and 0x3C
//! datv_omega (samples per symbol, Q8.24): the timing recovery between DDC
//! and recorder ([`super::symsync`]), in bitstreams that have it (0x3C
//! reads back what was written; 0 otherwise). With it on, the ring holds
//! one sample a symbol.
//!
//! DVB-T2 ([`FrontEnd::start_t2`]): datv_symsync bit 12 gives the recorder
//! the T2 resampler's output instead (ADC samples straight to the T2
//! elementary rate, [`crate::dvbt2::resamp`]; step in datv_omega, Q2.30),
//! and with bit 13 set the ddc_coeff writes (address 11:0) load its table
//! instead of the DDC's.
//!
//! Only a core that says it is the ring one (platform 0xD5) is touched, and
//! only when the device tree reserves the ring: any other Maia core's
//! recorder writes 128 MB of RAM Linux owns.

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use num_complex::Complex32;

use super::ddc::{design, frequency_word};

const REGS_PHYS: u64 = 0x7C46_0000;
pub const RING_START: u32 = 0x1610_0000;
pub const RING_END: u32 = 0x1620_0000;
const RING_BYTES: usize = (RING_END - RING_START) as usize;
/// The DDC's input: the AD936x rate before the x8 decimator.
pub const FS_IN: f64 = 3_072_000.0;
const PLATFORM_DATV: u32 = 0xD5;
const DT_RING: &str = "/proc/device-tree/reserved-memory/maia_sdr_datv_ring@16100000";

/// The newest front end started. A receiver being replaced lets go of the
/// hardware after its successor has set it up (the web UI restarts the
/// receiver on every setting): only the newest may stop the ring.
static GENERATION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

const REG_ID: usize = 0x00;
const REG_VERSION: usize = 0x04;
const REG_REC_CONTROL: usize = 0x10;
const REG_REC_COMMITTED: usize = 0x18;
const REG_COEFF_ADDR: usize = 0x24;
const REG_COEFF: usize = 0x28;
const REG_DECIMATION: usize = 0x2C;
const REG_FREQUENCY: usize = 0x30;
const REG_DDC_CONTROL: usize = 0x34;
const REG_SYMSYNC: usize = 0x38;
const REG_OMEGA: usize = 0x3C;
const T2: u32 = 1 << 12;
const T2_COEFF: u32 = 1 << 13;

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
    fs_out: f64,
    /// Next physical address to read.
    rd: u32,
    center_hz: f64,
    generation: u64,
    /// The ring holds symbols (the FPGA's timing recovery is on).
    symbols: bool,
    /// ... with header candidate flags in bit 16 (bit 0 of im).
    flagged: bool,
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
        // Claim the hardware first: from here on an older front end being
        // dropped leaves it alone.
        let generation = GENERATION.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
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
        // Timing recovery in the FPGA when the bitstream has it (and it is
        // not turned off for comparison: TRXD_NO_SYMSYNC=1). Always written:
        // disabled, it also resets the loop.
        let ss = super::symsync::Params::new(design.fs_out(), rs);
        regs.wr32(REG_SYMSYNC, 0);
        regs.wr32(REG_OMEGA, ss.omega);
        let symbols = regs.rd32(REG_OMEGA) == ss.omega && std::env::var_os("TRXD_NO_SYMSYNC").is_none();
        let mut flagged = false;
        if symbols {
            // The header detector too, if this core has it (its bit reads
            // back) and it is not turned off (TRXD_NO_HDRDET=1).
            let want = std::env::var_os("TRXD_NO_HDRDET").is_none();
            regs.wr32(REG_SYMSYNC, ss.kp_shift << 1 | ss.ki_shift << 6 | (want as u32) << 11);
            flagged = want && regs.rd32(REG_SYMSYNC) & (1 << 11) != 0;
            regs.wr32(REG_SYMSYNC, 1 | ss.kp_shift << 1 | ss.ki_shift << 6 | (flagged as u32) << 11);
        }
        // 16-bit mode (0), start: the ring fills from RING_START.
        regs.wr32(REG_REC_CONTROL, 1);
        Ok(FrontEnd { _mem: mem, regs, ring, fs_out: design.fs_out(), rd: RING_START, center_hz, generation, symbols, flagged })
    }

    /// DVB-T2: the recorder takes the T2 resampler's samples at (about)
    /// `fs` (the exact rate the step gives is [`Self::fs_out`]). The DDC is
    /// left off; the signal is expected on the LO (T2 fills the channel).
    pub fn start_t2(fs: f64) -> Result<FrontEnd, String> {
        use crate::dvbt2::resamp;
        if !std::path::Path::new(DT_RING).exists() {
            return Err("no DATV ring reserved in the device tree".into());
        }
        let generation = GENERATION.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
        let (mem, regs) = Self::open_regs()?;
        if !is_datv_core(&regs) {
            return Err(format!("the FPGA has no DATV ring recorder (version {:#010x})", regs.rd32(REG_VERSION)));
        }
        let ring = Mapping::new(&mem, RING_BYTES, RING_START as u64).map_err(|e| format!("map DATV ring: {e}"))?;
        regs.wr32(REG_REC_CONTROL, 1 << 1);
        std::thread::sleep(std::time::Duration::from_millis(2));
        // The T2 bits must read back: older bitstreams have no resampler.
        regs.wr32(REG_SYMSYNC, T2_COEFF);
        if regs.rd32(REG_SYMSYNC) & T2_COEFF == 0 {
            regs.wr32(REG_SYMSYNC, 0);
            return Err("the bitstream has no DVB-T2 resampler".into());
        }
        let ctl = regs.rd32(REG_DDC_CONTROL);
        regs.wr32(REG_DDC_CONTROL, ctl & !(1 << 24));
        for (addr, &c) in resamp::t2_table(FS_IN, fs).iter().enumerate() {
            regs.wr32(REG_COEFF_ADDR, addr as u32);
            regs.wr32(REG_COEFF, 1 | (((c as u32) & 0x3_FFFF) << 1));
        }
        let step = resamp::step(FS_IN, fs);
        regs.wr32(REG_OMEGA, step);
        regs.wr32(REG_SYMSYNC, 0); // resets the resampler
        regs.wr32(REG_SYMSYNC, T2);
        regs.wr32(REG_REC_CONTROL, 1);
        Ok(FrontEnd {
            _mem: mem,
            regs,
            ring,
            fs_out: resamp::rate_out(FS_IN, step),
            rd: RING_START,
            center_hz: 0.0,
            generation,
            symbols: false,
            flagged: false,
        })
    }

    /// Output rate (2 samples per symbol; T2: the elementary rate).
    pub fn fs_out(&self) -> f64 {
        self.fs_out
    }

    /// One ring sample a symbol (the FPGA recovers the timing).
    pub fn symbols(&self) -> bool {
        self.symbols
    }

    /// Symbols carry header candidate flags (read with [`Self::read_flagged`]).
    pub fn flagged(&self) -> bool {
        self.flagged
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
        self.read_inner(out, None);
    }

    /// [`Self::read`], and each word's bit 16 (the header detector's flag).
    pub fn read_flagged(&mut self, out: &mut Vec<Complex32>, flags: &mut Vec<bool>) {
        self.read_inner(out, Some(flags));
    }

    fn read_inner(&mut self, out: &mut Vec<Complex32>, mut flags: Option<&mut Vec<bool>>) {
        let c = self.regs.rd32(REG_REC_COMMITTED);
        if !(RING_START..RING_END).contains(&c) {
            return;
        }
        let (a, b) = (self.rd, c);
        if b >= a {
            self.copy(a, b, out, flags.as_deref_mut());
        } else {
            self.copy(a, RING_END, out, flags.as_deref_mut());
            self.copy(RING_START, b, out, flags.as_deref_mut());
        }
        self.rd = c;
    }

    fn copy(&self, from: u32, to: u32, out: &mut Vec<Complex32>, mut flags: Option<&mut Vec<bool>>) {
        let (s, e) = ((from - RING_START) as usize, (to - RING_START) as usize);
        assert!(s <= e && e <= self.ring.len);
        // One bulk copy out of the uncached ring (memcpy's wide loads: about
        // 185 MB/s on the A9, against 32 MB/s a word at a time), then the
        // conversion from cached memory.
        let mut words = vec![0u32; (e - s) / 4];
        // SAFETY: [s, e) is inside the mapping; `words` holds (e - s) bytes.
        unsafe { std::ptr::copy_nonoverlapping(self.ring.ptr.add(s), words.as_mut_ptr().cast::<u8>(), words.len() * 4) };
        out.reserve(words.len());
        for &w in &words {
            // Recorder16IQ: re in the low half, im in the high half.
            let (re, im) = (w as u16 as i16, (w >> 16) as u16 as i16);
            out.push(Complex32::new(re as f32 / 32768.0, im as f32 / 32768.0));
            if let Some(f) = flags.as_deref_mut() {
                f.push(w & (1 << 16) != 0);
            }
        }
    }
}

impl Drop for FrontEnd {
    fn drop(&mut self) {
        // A newer front end owns the hardware now: leave it running.
        if GENERATION.load(std::sync::atomic::Ordering::SeqCst) != self.generation {
            return;
        }
        self.regs.wr32(REG_REC_CONTROL, 1 << 1);
        let ctl = self.regs.rd32(REG_DDC_CONTROL);
        self.regs.wr32(REG_DDC_CONTROL, ctl & !(1 << 24));
    }
}

/// `trxd --ring-bench`: how fast the CPU reads the (uncached) DATV ring,
/// word by word and in wider loads. Reads only.
pub fn ring_bench() -> Result<(), String> {
    let (mem, _regs) = FrontEnd::open_regs()?;
    let ring = Mapping::new(&mem, RING_BYTES, RING_START as u64).map_err(|e| format!("map DATV ring: {e}"))?;
    let mut dst = vec![0u32; RING_BYTES / 4];
    let report = |name: &str, t: std::time::Duration, sum: u64| {
        println!("{name:>12}: {:6.1} MB/s ({sum:x})", RING_BYTES as f64 / t.as_secs_f64() / 1e6);
    };
    for _ in 0..2 {
        let t = std::time::Instant::now();
        let mut sum = 0u64;
        for off in (0..RING_BYTES).step_by(4) {
            sum = sum.wrapping_add(ring.rd32(off) as u64);
        }
        report("u32", t.elapsed(), sum);
        let t = std::time::Instant::now();
        let mut sum = 0u64;
        for off in (0..RING_BYTES).step_by(8) {
            // SAFETY: inside the mapping, 8-byte aligned.
            sum = sum.wrapping_add(unsafe { std::ptr::read_volatile(ring.ptr.add(off).cast::<u64>()) });
        }
        report("u64", t.elapsed(), sum);
        let t = std::time::Instant::now();
        // SAFETY: both RING_BYTES long.
        unsafe { std::ptr::copy_nonoverlapping(ring.ptr, dst.as_mut_ptr().cast::<u8>(), RING_BYTES) };
        report("memcpy", t.elapsed(), dst[1000] as u64);
        #[cfg(target_arch = "arm")]
        {
            let t = std::time::Instant::now();
            let (mut s, mut d) = (ring.ptr as *const u8, dst.as_mut_ptr() as *mut u8);
            for _ in 0..RING_BYTES / 32 {
                // SAFETY: 32 bytes a step inside both buffers.
                unsafe {
                    core::arch::asm!(
                        "vld1.32 {{d16-d19}}, [{s}]!",
                        "vst1.32 {{d16-d19}}, [{d}]!",
                        s = inout(reg) s, d = inout(reg) d,
                        out("d16") _, out("d17") _, out("d18") _, out("d19") _,
                    );
                }
            }
            report("neon 32 B", t.elapsed(), dst[1000] as u64);
        }
    }
    Ok(())
}

