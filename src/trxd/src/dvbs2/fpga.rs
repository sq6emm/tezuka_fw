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
//! Its OFDM front end ([`crate::dvbt2::fe`], maia-hdl `t2ofdm.py`), in
//! bitstreams that have it, at 0x40.. (t2_layout reads back): 0x40 control
//! (0 enable, 1 scheduled, 2 load, 5:3 shift), 0x44 frame_len, 0x48 layout
//! (7:0 symbols, 17:8 guard interval, 25:18 early), 0x4C track, 0x50 NCO
//! step, 0x54 next frame start, 0x58 counter, 0x5C status (21:0 frames, 22
//! word FIFO overflow, 23 resampler input FIFO overflow). With it on the ring carries its tagged words.
//!
//! Newer cores: 0x1C recorder_wraps (bit 31 set: present; 15:0 the times
//! committed_address wrapped since the start), so a reader the DMA lapped
//! is seen for what it is (older cores: from the time between reads and
//! the ring's rate); a stop completes a burst the stream left half full
//! and always ends in `finished`; 0x74 t2eq_status bit 16, the G bank the
//! equalizer took at the last symbol header (with gshift).
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
/// Ring words (one symbol each with the FPGA's timing recovery).
pub const RING_WORDS: u64 = RING_BYTES as u64 / 4;
/// Absolute ring positions as the wrap counter gives them: modulo 65536 rings.
pub const RING_SPAN_WORDS: u64 = 65536 * RING_WORDS;
/// The DDC's input: the AD936x rate before the x8 decimator.
pub const FS_IN: f64 = 3_072_000.0;
const PLATFORM_DATV: u32 = 0xD5;
/// The trx bitstream's core: the DDC feeds the ring with the radio's
/// channel (maia_iio_lite_trx, platform 0xD7).
const PLATFORM_CHAN: u32 = 0xD7;
const DT_RING: &str = "/proc/device-tree/reserved-memory/maia_sdr_datv_ring@16100000";

/// The newest front end started. A receiver being replaced lets go of the
/// hardware after its successor has set it up (the web UI restarts the
/// receiver on every setting): only the newest may stop the ring.
static GENERATION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

const REG_ID: usize = 0x00;
const REG_VERSION: usize = 0x04;
const REG_REC_CONTROL: usize = 0x10;
const REG_CONTROL: usize = 0x08;
const REG_REC_COMMITTED: usize = 0x18;
const REG_REC_WRAPS: usize = 0x1C;
const WRAPS_PRESENT: u32 = 1 << 31;
const REG_COEFF_ADDR: usize = 0x24;
const REG_COEFF: usize = 0x28;
const REG_DECIMATION: usize = 0x2C;
const REG_FREQUENCY: usize = 0x30;
const REG_DDC_CONTROL: usize = 0x34;
const REG_SYMSYNC: usize = 0x38;
const REG_OMEGA: usize = 0x3C;
const T2: u32 = 1 << 12;
const REG_T2_CONTROL: usize = 0x40;
/// The S2 known-symbol accumulator (maia-hdl s2trk.py; the s2 mode
/// bitstream, in the window T2 has elsewhere): control (enable 0, load 1,
/// pilots 2, pilot blocks 7:3), the frame's base and length, the mixer's
/// step, the header table (addr 6:0, q 8:7, we 9), the FIFO's entry (6
/// words), status (level 9:0, overflow 10, synced 11), pop, features (bit 16).
const REG_S2TRK_CONTROL: usize = 0x40;
const REG_S2TRK_BASE: usize = 0x44;
const REG_S2TRK_LEN: usize = 0x48;
const REG_S2TRK_DTH: usize = 0x4C;
const REG_S2TRK_HDR: usize = 0x50;
const REG_S2TRK_ENTRY: usize = 0x54;
const REG_S2TRK_STATUS: usize = 0x6C;
const REG_S2TRK_POP: usize = 0x70;
const REG_S2TRK_FEATURES: usize = 0x7C;
const S2TRK_FEATURE: u32 = 1 << 16;
const REG_T2_FRAME_LEN: usize = 0x44;
const REG_T2_LAYOUT: usize = 0x48;
const REG_T2_TRACK: usize = 0x4C;
const REG_T2_FREQ: usize = 0x50;
const REG_T2_NEXT_START: usize = 0x54;
const REG_T2_STATUS: usize = 0x5C;
/// The P1 / GI / MER reports (maia-hdl t2p1.py; registers 14 and 15 of the
/// T2 window): control, and the features a bitstream has (0: none).
const REG_T2_EXT: usize = 0x78;
const REG_T2_FEATURES: usize = 0x7C;
const T2_EXT_P1: u32 = 1;
const T2_EXT_ACQ_RAW_OFF: u32 = 1 << 9;
const T2_EXT_GI_RAW_OFF: u32 = 1 << 10;
const T2_EXT_MER: u32 = 1 << 11;
const T2_EXT_RING_J_SHIFT: u32 = 12;
const T2_EXT_REF8_SHIFT: u32 = 20;
/// P1 threshold factor (Q8) for the detector's best-window score.
const T2_P1_K_Q8: u32 = 64;
// The T2 equalizer (maia-hdl t2eq.py), bitstreams that have it: control (0
// enable, 1 gbank, 9:2 p2, 14:10 gshift, 22:15 fc_j), pilots (5:0 dx, 8:6
// dy), 1/D (15:0 data symbols, 31:16 the frame closing one, x 65536), the
// G write address (10:0, 11 bank, 12 write), G data, status (symbols).
const REG_T2EQ_CONTROL: usize = 0x60;
const REG_T2EQ_PILOTS: usize = 0x64;
const REG_T2EQ_REC: usize = 0x68;
const REG_T2EQ_GADDR: usize = 0x6C;
const REG_T2EQ_GDATA: usize = 0x70;
const REG_T2EQ_STATUS: usize = 0x74;
const GBANK_USED: u32 = 1 << 16;
const T2_ENABLE: u32 = 1;
const T2_SCHEDULED: u32 = 1 << 1;
const T2_LOAD: u32 = 1 << 2;
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

/// Before the PL is reloaded (fpgamode): the ring recorder stopped and its
/// last burst in memory (a DMA cut off mid-burst can wedge the HP port).
pub fn quiesce() {
    if let Ok((_mem, regs)) = FrontEnd::open_regs() {
        if is_datv_core(&regs) {
            stop_recorder(&regs);
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

/// The core's platform byte, read a few times over: its registers live in
/// the sampling clock domain and right after the radio was set up the
/// first reads came back as garbage (maia.rs).
fn platform(regs: &Mapping) -> Option<u32> {
    for _ in 0..20 {
        if regs.rd32(REG_ID).to_le_bytes() == *b"maia" {
            return Some(regs.rd32(REG_VERSION) >> 24);
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    None
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
    /// DVB-T2 with the FPGA's OFDM front end: the ring holds its words.
    t2_fe: bool,
    /// The T2 equalizer's control word (p2, fc_j) when the bitstream has
    /// one, and the G bank in use.
    t2_eq: Option<u32>,
    eq_bank: std::cell::Cell<u32>,
    /// The equalizer reports the bank it took (newer cores; cleared after a
    /// wait for it timed out: an older core reads 0 there).
    eq_bank_seen: std::cell::Cell<bool>,
    /// The T2 report control word when the reports are on.
    t2_ext: std::cell::Cell<Option<u32>>,
    /// DVB-S2: the known-symbol accumulator is on (s2trk).
    s2trk: bool,
    /// Ring bytes a second (lap detection without the wrap counter).
    ring_rate: f64,
    /// The channel's words scaled to unity passband gain (start_channel).
    chan_gain: f32,
    /// The recorder counts its wraps (0x1C).
    wraps_hw: bool,
    /// Bytes read since the start (with the wrap counter: the DMA's bytes
    /// committed when last read).
    total: u64,
    last_read: std::time::Instant,
    overruns: u64,
    overrun_log: Option<std::time::Instant>,
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
        stop_recorder(&regs);
        out_of_reset(&regs);
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
        // Timing recovery in the FPGA when the bitstream has it. Always
        // written: disabled, it also resets the loop.
        let ss = super::symsync::Params::new(design.fs_out(), rs);
        regs.wr32(REG_SYMSYNC, 0);
        regs.wr32(REG_OMEGA, ss.omega);
        let symbols = regs.rd32(REG_OMEGA) == ss.omega;
        let mut flagged = false;
        if symbols {
            // The header detector too, if this core has it (its bit reads back).
            regs.wr32(REG_SYMSYNC, ss.kp_shift << 1 | ss.ki_shift << 6 | 1 << 11);
            flagged = regs.rd32(REG_SYMSYNC) & (1 << 11) != 0;
            regs.wr32(REG_SYMSYNC, 1 | ss.kp_shift << 1 | ss.ki_shift << 6 | (flagged as u32) << 11);
        }
        // The known-symbol accumulator (a bitstream that has it; with the
        // header detector's symbols; without it the receiver makes the same
        // sums from the ring's words).
        let s2trk = flagged && regs.rd32(REG_S2TRK_FEATURES) & S2TRK_FEATURE != 0;
        if s2trk {
            regs.wr32(REG_S2TRK_CONTROL, 1);
        }
        // 16-bit mode (0), start: the ring fills from RING_START.
        regs.wr32(REG_REC_CONTROL, 1);
        let ring_rate = 4.0 * if symbols { rs } else { design.fs_out() };
        let mut fe = FrontEnd::new(mem, regs, ring, design.fs_out(), center_hz, generation, symbols, flagged, false, None, ring_rate);
        fe.s2trk = s2trk;
        tracing::info!(s2trk, "DVB-S2 front end: known-symbol accumulator");
        Ok(fe)
    }

    /// The radio's channel out of the trx bitstream's DDC: `fs_out` (48
    /// kHz) with `pass_hz` flat each side, the channel `center_hz` from the
    /// LO, into the ring. None on a core without it (the software DDC then).
    pub fn start_channel(fs_out: f64, pass_hz: f64, center_hz: f64) -> Result<FrontEnd, String> {
        if !std::path::Path::new(DT_RING).exists() {
            return Err("no ring reserved in the device tree".into());
        }
        let generation = GENERATION.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
        let (mem, regs) = Self::open_regs()?;
        if platform(&regs) != Some(PLATFORM_CHAN) {
            return Err(format!("the FPGA has no channel ring (version {:#010x})", regs.rd32(REG_VERSION)));
        }
        let ring = Mapping::new(&mem, RING_BYTES, RING_START as u64).map_err(|e| format!("map channel ring: {e}"))?;
        let design = super::ddc::design_channel(FS_IN, fs_out, pass_hz)?;
        let r = design.registers();
        stop_recorder(&regs);
        out_of_reset(&regs);
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
        let mut fe = FrontEnd::new(mem, regs, ring, design.fs_out(), center_hz, generation, false, false, false, None, 4.0 * design.fs_out());
        // Scaled to the stream path's units (the x8 decimator's output at
        // 1/2048, which the S-meter calibration is in): the DDC's quantised
        // passband gain undone, and a half (measured: the same noise read
        // 6.0 dB higher through the DDC, the meter on the filter band).
        fe.chan_gain = 0.5 / super::ddc::passband_gain(&design);
        tracing::info!(fs_out = design.fs_out(), taps = r.coeffs.len(), gain = fe.chan_gain, "radio channel from the FPGA DDC");
        Ok(fe)
    }

    /// The ring's words as the channel's samples (16-bit I low, Q high),
    /// appended to `out`; None when the reader was lapped (samples lost).
    pub fn read_channel(&mut self, words: &mut Vec<u32>, out: &mut Vec<Complex32>) -> Option<u64> {
        words.clear();
        let at = self.read_raw(words);
        let g = self.chan_gain / 32768.0;
        out.extend(words.iter().map(|&w| Complex32::new((w as u16 as i16) as f32 * g, ((w >> 16) as u16 as i16) as f32 * g)));
        at
    }

    /// DVB-T2: the recorder takes the T2 resampler's samples at (about)
    /// `fs` (the exact rate the step gives is [`Self::fs_out`]). The DDC is
    /// left off; the signal is expected on the LO (T2 fills the channel).
    pub fn start_t2(fs: f64, p: &crate::dvbt2::Params) -> Result<FrontEnd, String> {
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
        stop_recorder(&regs);
        out_of_reset(&regs);
        // The T2 bits must read back: older bitstreams have no resampler.
        regs.wr32(REG_SYMSYNC, T2_COEFF);
        if regs.rd32(REG_SYMSYNC) & T2_COEFF == 0 {
            regs.wr32(REG_SYMSYNC, 0);
            return Err("the bitstream has no DVB-T2 resampler".into());
        }
        let ctl = regs.rd32(REG_DDC_CONTROL);
        regs.wr32(REG_DDC_CONTROL, ctl & !(1 << 24));
        // Debug: `touch /tmp/t2-passthrough`: the resampler passes the ADC
        // samples through (one tap, step 1.0): the ring holds what the FPGA
        // gets from the ADC, at 3.072 MS/s.
        let passthrough = std::path::Path::new("/tmp/t2-passthrough").exists();
        let table: Vec<i32> = if passthrough {
            (0..resamp::SPAN * resamp::PHASES).map(|a| if a / resamp::PHASES == resamp::SPAN / 2 { 1 << 14 } else { 0 }).collect()
        } else {
            resamp::t2_table(FS_IN, fs)
        };
        for (addr, &c) in table.iter().enumerate() {
            regs.wr32(REG_COEFF_ADDR, addr as u32);
            regs.wr32(REG_COEFF, 1 | (((c as u32) & 0x3_FFFF) << 1));
        }
        let step = if passthrough { 1 << resamp::FRAC } else { resamp::step(FS_IN, fs) };
        regs.wr32(REG_OMEGA, step);
        regs.wr32(REG_SYMSYNC, 0); // resets the resampler
        // The OFDM front end, if there (its layout reads back), off until
        // the resampler runs, then sending every sample (searching).
        use crate::dvbt2::fe::{TRACK, early};
        let gi = p.guard.samples();
        let layout = p.symbols() as u32 | (gi as u32) << 8 | early(gi) << 18;
        regs.wr32(REG_T2_CONTROL, 0);
        regs.wr32(REG_T2_LAYOUT, layout);
        let t2_fe = regs.rd32(REG_T2_LAYOUT) == layout;
        if t2_fe {
            regs.wr32(REG_T2_FRAME_LEN, p.frame_samples() as u32);
            regs.wr32(REG_T2_TRACK, TRACK);
            regs.wr32(REG_T2_FREQ, 0);
        }
        // The equalizer, if there (its pilot layout reads back); on once the
        // receiver sends the first channel inverse.
        let (dx, dy) = p.pilots.dxdy();
        let pilots = dx as u32 | (dy as u32) << 6;
        let mut t2_eq = None;
        if t2_fe {
            regs.wr32(REG_T2EQ_CONTROL, 0);
            regs.wr32(REG_T2EQ_PILOTS, pilots);
            if regs.rd32(REG_T2EQ_PILOTS) == pilots {
                let (_, n_fc, _) = p.data_cells();
                let fc_j = if n_fc != 0 { p.symbols() as u32 - 1 } else { 255 };
                let rec = |d: usize| ((65536.0 / d as f64).round() as u32).min(0xFFFF);
                regs.wr32(REG_T2EQ_REC, rec(dx * dy) | rec(dx) << 16);
                let base = (crate::dvbt2::N_P2 as u32) << 2 | fc_j << 15;
                regs.wr32(REG_T2EQ_CONTROL, base);
                t2_eq = Some(base);
            }
        }
        // The P1, GI and MER reports (a bitstream that has them, with the
        // equalizer): searching and frequency tracking without raw samples
        // (stream.rs).
        let mut t2_ext = None;
        if t2_fe && t2_eq.is_some() {
            let feats = regs.rd32(REG_T2_FEATURES) & 0xFF;
            if feats & crate::dvbt2::fe::FEATURES_REPORTS == crate::dvbt2::fe::FEATURES_REPORTS {
                let boost = match p.pilots {
                    crate::dvbt2::Pilots::PP1 | crate::dvbt2::Pilots::PP2 => 4.0 / 3.0,
                    crate::dvbt2::Pilots::PP3 | crate::dvbt2::Pilots::PP4 => 7.0 / 4.0,
                    _ => 7.0 / 3.0,
                };
                let ref8 = ((8.0 * crate::dvbt2::fe::EQ_UNIT as f64 * boost).round() as u32).min(1023);
                let ext = T2_EXT_P1 | T2_P1_K_Q8 << 1 | T2_EXT_ACQ_RAW_OFF | T2_EXT_GI_RAW_OFF | T2_EXT_MER | ref8 << T2_EXT_REF8_SHIFT;
                regs.wr32(REG_T2_EXT, ext);
                if regs.rd32(REG_T2_EXT) == ext {
                    t2_ext = Some(ext);
                }
            }
        }
        regs.wr32(REG_SYMSYNC, T2);
        if t2_fe {
            regs.wr32(REG_T2_CONTROL, T2_ENABLE);
        }
        regs.wr32(REG_REC_CONTROL, 1);
        let fs_out = resamp::rate_out(FS_IN, step);
        let fe = FrontEnd::new(mem, regs, ring, fs_out, 0.0, generation, false, false, t2_fe, t2_eq, 4.0 * fs_out);
        fe.t2_ext.set(t2_ext);
        tracing::info!(reports = t2_ext.is_some(), "DVB-T2 front end: P1 / GI / MER reports");
        Ok(fe)
    }

    #[allow(clippy::too_many_arguments)]
    fn new(mem: File, regs: Mapping, ring: Mapping, fs_out: f64, center_hz: f64, generation: u64, symbols: bool, flagged: bool, t2_fe: bool, t2_eq: Option<u32>, ring_rate: f64) -> FrontEnd {
        let wraps_hw = regs.rd32(REG_REC_WRAPS) & WRAPS_PRESENT != 0;
        FrontEnd {
            _mem: mem,
            regs,
            ring,
            fs_out,
            rd: RING_START,
            center_hz,
            generation,
            symbols,
            flagged,
            t2_fe,
            t2_eq,
            eq_bank: std::cell::Cell::new(0),
            eq_bank_seen: std::cell::Cell::new(true),
            t2_ext: std::cell::Cell::new(None),
            s2trk: false,
            chan_gain: 1.0,
            ring_rate,
            wraps_hw,
            total: 0,
            last_read: std::time::Instant::now(),
            overruns: 0,
            overrun_log: None,
        }
    }

    /// The DMA's position: bytes committed since the start (wrap counter
    /// cores) and the committed address, read consistently.
    fn position(&self) -> Option<(u64, u32)> {
        for _ in 0..4 {
            let w0 = self.regs.rd32(REG_REC_WRAPS);
            let c = self.regs.rd32(REG_REC_COMMITTED);
            let w1 = self.regs.rd32(REG_REC_WRAPS);
            if !(RING_START..RING_END).contains(&c) {
                return None;
            }
            if w0 == w1 {
                return Some(((w0 & 0xFFFF) as u64 * RING_BYTES as u64 + (c - RING_START) as u64, c));
            }
        }
        None
    }

    /// The bytes the DMA committed since the reader's last position, and
    /// whether it lapped the reader (the ring holds no more than its size:
    /// what lay between is lost or overwritten). Without the wrap counter
    /// the time since the last read tells (a lap at the ring's rate).
    fn check_lap(&mut self, c: u32) -> bool {
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_read).as_secs_f64();
        self.last_read = now;
        let lapped = if self.wraps_hw {
            match self.position() {
                Some((pos, _)) => {
                    // 16-bit wrap counter: positions modulo 65536 rings
                    let span = 65536 * RING_BYTES as u64;
                    let ahead = (pos + span - self.total % span) % span;
                    self.total += ahead;
                    ahead >= RING_BYTES as u64
                }
                None => false,
            }
        } else {
            let _ = c;
            dt * self.ring_rate > 0.9 * RING_BYTES as f64
        };
        if lapped {
            self.overruns += 1;
            let log = self.overrun_log.is_none_or(|t| now.duration_since(t).as_secs() >= 10);
            if log {
                tracing::warn!(overruns = self.overruns, gap_ms = (dt * 1e3) as u64, ring_ms = (RING_BYTES as f64 / self.ring_rate * 1e3) as u64, "DATV ring: the reader was lapped (samples lost)");
                self.overrun_log = Some(now);
            }
        }
        lapped
    }

    /// Times the DMA lapped the reader (data lost) since the start.
    pub fn overruns(&self) -> u64 {
        self.overruns
    }

    /// DVB-T2: the ring carries the OFDM front end's words ([`Self::read_words`]).
    pub fn t2_fe(&self) -> bool {
        self.t2_fe
    }

    /// DVB-S2: the known-symbol accumulator runs ([`Self::read_trk`]).
    pub fn s2trk(&self) -> bool {
        self.s2trk
    }

    /// The accumulator's entries so far (its FIFO emptied).
    pub fn read_trk(&mut self, out: &mut Vec<[u32; super::s2trk::ENTRY_WORDS]>) {
        if !self.s2trk {
            return;
        }
        let n = self.regs.rd32(REG_S2TRK_STATUS) & 0x3FF;
        for _ in 0..n {
            let mut e = [0u32; super::s2trk::ENTRY_WORDS];
            for (i, w) in e.iter_mut().enumerate() {
                *w = self.regs.rd32(REG_S2TRK_ENTRY + 4 * i);
            }
            self.regs.wr32(REG_S2TRK_POP, 1);
            out.push(e);
        }
    }

    /// A command from the receiver for the accumulator.
    pub fn trk_ctl(&mut self, c: &super::s2trk::Ctl) {
        if !self.s2trk {
            return;
        }
        use super::s2trk::Ctl;
        match c {
            Ctl::Header(q) => {
                for (a, &q) in q.iter().enumerate() {
                    self.regs.wr32(REG_S2TRK_HDR, a as u32 | (q as u32 & 3) << 7 | 1 << 9);
                }
            }
            Ctl::Dth(d) => self.regs.wr32(REG_S2TRK_DTH, *d),
            Ctl::Load { base, len, pilots, npil } => {
                self.regs.wr32(REG_S2TRK_BASE, *base as u32);
                self.regs.wr32(REG_S2TRK_LEN, *len);
                self.regs.wr32(REG_S2TRK_CONTROL, 1 | 1 << 1 | (*pilots as u32) << 2 | (*npil as u32 & 31) << 3);
            }
        }
    }

    /// DVB-T2: the front end sends the P1 / GI / MER reports.
    pub fn t2_hw(&self) -> bool {
        self.t2_ext.get().is_some()
    }

    /// DVB-T2 front end: do what the receiver asks.
    pub fn t2_ctl(&self, c: crate::dvbt2::fe::Ctl) {
        use crate::dvbt2::fe::Ctl;
        // Debug: `touch /tmp/t2-rawall`: every sample raw as well (the FFTs
        // can be checked against them offline).
        let dbg = if std::path::Path::new("/tmp/t2-rawall").exists() { 1 << 6 } else { 0 };
        let en = T2_ENABLE | dbg;
        match c {
            Ctl::RawAll => self.regs.wr32(REG_T2_CONTROL, en),
            Ctl::Schedule { start, freq } => {
                self.regs.wr32(REG_T2_FREQ, freq);
                self.regs.wr32(REG_T2_NEXT_START, start as u32);
                self.regs.wr32(REG_T2_CONTROL, en | T2_SCHEDULED | T2_LOAD);
            }
            Ctl::Freq(f) => self.regs.wr32(REG_T2_FREQ, f),
            Ctl::EqRing(j) => {
                if let Some(ext) = self.t2_ext.get() {
                    let ext = ext & !(0xFF << T2_EXT_RING_J_SHIFT) | (j as u32) << T2_EXT_RING_J_SHIFT;
                    self.regs.wr32(REG_T2_EXT, ext);
                    self.t2_ext.set(Some(ext));
                }
            }
            Ctl::EqTable { g, gshift } => {
                let Some(base) = self.t2_eq else { return };
                // into the bank not in use, then flip (taken, with gshift,
                // at the next symbol's start). The bank written now was in
                // use until the last flip was taken: wait for that (newer
                // cores report it), else a second table within a symbol
                // overwrites the bank the equalizer still reads.
                let bank = self.eq_bank.get() ^ 1;
                if self.eq_bank_seen.get() {
                    let t0 = std::time::Instant::now();
                    let in_use = || (self.regs.rd32(REG_T2EQ_STATUS) & GBANK_USED != 0) as u32;
                    while in_use() == bank {
                        if t0.elapsed() > std::time::Duration::from_millis(20) {
                            // no symbols coming, or an older core (bit 16 is 0)
                            self.eq_bank_seen.set(false);
                            break;
                        }
                        std::thread::sleep(std::time::Duration::from_micros(200));
                    }
                }
                for (k, &v) in g.iter().enumerate() {
                    self.regs.wr32(REG_T2EQ_GDATA, v);
                    self.regs.wr32(REG_T2EQ_GADDR, k as u32 | bank << 11 | 1 << 12);
                }
                self.regs.wr32(REG_T2EQ_CONTROL, base | 1 | bank << 1 | (gshift & 31) << 10);
                self.eq_bank.set(bank);
            }
        }
    }

    /// DVB-T2 front end: its word FIFOs overflowed (words lost).
    pub fn t2_overflow(&self) -> bool {
        // 22: the front end's word FIFOs; 23: the resampler's input FIFO.
        self.t2_fe && self.regs.rd32(REG_T2_STATUS) & (3 << 22) != 0
    }

    /// Everything the DMA has committed since the last call, as ring words.
    pub fn read_words(&mut self, out: &mut Vec<u32>) {
        let c = self.regs.rd32(REG_REC_COMMITTED);
        if !(RING_START..RING_END).contains(&c) {
            return;
        }
        if self.check_lap(c) {
            // the words between are a mix of laps: start again at the DMA
            self.rd = c;
            return;
        }
        let mut copy = |from: u32, to: u32| {
            let (s, e) = ((from - RING_START) as usize, (to - RING_START) as usize);
            let at = out.len();
            out.resize(at + (e - s) / 4, 0);
            // SAFETY: [s, e) is inside the mapping; `out` has room.
            unsafe { std::ptr::copy_nonoverlapping(self.ring.ptr.add(s), out[at..].as_mut_ptr().cast::<u8>(), e - s) };
        };
        if c >= self.rd {
            copy(self.rd, c);
        } else {
            copy(self.rd, RING_END);
            copy(RING_START, c);
        }
        self.rd = c;
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

    /// The recorder counts its wraps (needed to follow absolute positions).
    pub fn wraps_hw(&self) -> bool {
        self.wraps_hw
    }

    /// Everything the DMA has committed since the last call, as the ring's
    /// raw words (one bulk copy, nothing converted): `Some(at)`, the absolute
    /// word (since the start) of the first one, or None when the DMA lapped
    /// the reader (the words between are gone; reading resumes at the DMA).
    pub fn read_raw(&mut self, out: &mut Vec<u32>) -> Option<u64> {
        let mut cur = RingCursor { rd: self.rd, total: self.total };
        let mut hw = HwRing { fe: self };
        let r = cur.read(&mut hw, out);
        self.rd = cur.rd;
        self.total = cur.total;
        r
    }

    /// A watch on the recorder's position for another thread (the decoder:
    /// is a frame still in the ring?).
    pub fn watch(&self) -> Option<RingWatch> {
        let (mem, regs) = Self::open_regs().ok()?;
        Some(RingWatch { _mem: mem, regs })
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
        if self.check_lap(c) {
            self.rd = c;
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

/// What [`RingCursor`] reads: the recorder's registers and the ring.
pub trait RingHw {
    /// Absolute bytes committed (wrap counter) and the committed address, consistently.
    fn position(&mut self) -> Option<(u64, u32)>;
    /// Ring bytes [from, to) (both inside the ring, from <= to) to `out` as words.
    fn copy(&mut self, from: u32, to: u32, out: &mut Vec<u32>);
    fn lapped(&mut self, _gap_bytes: u64) {}
}

struct HwRing<'a> {
    fe: &'a mut FrontEnd,
}

impl RingHw for HwRing<'_> {
    fn position(&mut self) -> Option<(u64, u32)> {
        self.fe.position()
    }
    fn copy(&mut self, from: u32, to: u32, out: &mut Vec<u32>) {
        let (s, e) = ((from - RING_START) as usize, (to - RING_START) as usize);
        let k = out.len();
        out.resize(k + (e - s) / 4, 0);
        // SAFETY: [s, e) is inside the mapping; `out` has room.
        unsafe { std::ptr::copy_nonoverlapping(self.fe.ring.ptr.add(s), out[k..].as_mut_ptr().cast::<u8>(), e - s) };
    }
    fn lapped(&mut self, gap: u64) {
        self.fe.overruns += 1;
        let now = std::time::Instant::now();
        if self.fe.overrun_log.is_none_or(|t| now.duration_since(t).as_secs() >= 10) {
            tracing::warn!(overruns = self.fe.overruns, gap_bytes = gap, "DATV ring: the reader was lapped (samples lost)");
            self.fe.overrun_log = Some(now);
        }
    }
}

/// The reader's place in the ring: `rd` the next address, `total` its
/// absolute byte position (the wrap counter's scale, modulo 65536 rings).
pub struct RingCursor {
    pub rd: u32,
    pub total: u64,
}

impl RingCursor {
    /// Everything committed since the last call to `out`: Some(the absolute
    /// word of the first), None when the DMA lapped the reader.
    ///
    /// The absolute position and the address to copy up to come from one
    /// consistent read of the recorder (wrap counter, committed address,
    /// wrap counter): `total` is always the absolute position of `rd`. (The
    /// committed address read apart from the wrap counter, the DMA having
    /// moved between the two, put every block a few words off: the
    /// receiver saw a gap each time, lost lock and half the frames.)
    pub fn read(&mut self, hw: &mut impl RingHw, out: &mut Vec<u32>) -> Option<u64> {
        let Some((pos, c)) = hw.position() else { return Some(self.total / 4) };
        let span = 65536 * RING_BYTES as u64;
        let ahead = (pos + span - self.total % span) % span;
        let at = self.total / 4;
        if ahead >= RING_BYTES as u64 {
            hw.lapped(ahead);
            self.rd = c;
            self.total += ahead;
            return None;
        }
        // (ahead == the bytes from rd to c)
        if c >= self.rd {
            hw.copy(self.rd, c, out);
        } else {
            hw.copy(self.rd, RING_END, out);
            hw.copy(RING_START, c, out);
        }
        self.rd = c;
        self.total += ahead;
        Some(at)
    }
}

/// The recorder's position, read from any thread.
pub struct RingWatch {
    _mem: File,
    regs: Mapping,
}

// SAFETY: only reads two registers.
unsafe impl Send for RingWatch {}
unsafe impl Sync for RingWatch {}

impl RingWatch {
    /// Words the DMA has committed past the absolute word `at` (modulo the
    /// wrap counter's span); None when the registers do not read sanely.
    pub fn ahead_of(&self, at: u64) -> Option<u64> {
        for _ in 0..4 {
            let w0 = self.regs.rd32(REG_REC_WRAPS);
            let c = self.regs.rd32(REG_REC_COMMITTED);
            let w1 = self.regs.rd32(REG_REC_WRAPS);
            if !(RING_START..RING_END).contains(&c) {
                return None;
            }
            if w0 == w1 {
                let pos = ((w0 & 0xFFFF) as u64 * RING_BYTES as u64 + (c - RING_START) as u64) / 4;
                return Some((pos + RING_SPAN_WORDS - at % RING_SPAN_WORDS) % RING_SPAN_WORDS);
            }
        }
        None
    }
}

/// The ring's physical address of absolute word `at`.
pub fn ring_addr(at: u64) -> u32 {
    RING_START + ((at % RING_WORDS) * 4) as u32
}

/// Stop the ring recorder and wait until its last burst is in memory: the
/// next start resets the address counters, and a write response still to
/// come would land in the new run's (committed one burst ahead of the data,
/// for good). Newer cores complete a half-filled burst on stop; with older
/// ones it waits for data, so the input is left running here.
fn stop_recorder(regs: &Mapping) {
    regs.wr32(REG_REC_CONTROL, 1 << 1);
    let t0 = std::time::Instant::now();
    let mut last = regs.rd32(REG_REC_COMMITTED);
    let mut still = 0;
    while still < 3 && t0.elapsed() < std::time::Duration::from_millis(50) {
        std::thread::sleep(std::time::Duration::from_millis(1));
        let c = regs.rd32(REG_REC_COMMITTED);
        still = if c == last { still + 1 } else { 0 };
        last = c;
    }
    if still < 3 {
        tracing::warn!("DATV ring: the recorder did not settle after stop");
    }
}

/// The Maia core's sdr_reset (control bit 0) holds the DDC, the recorder
/// and the T2 front end; the spectrometer's start (maia.rs) clears it, but
/// the DATV front end must not depend on that having run.
fn out_of_reset(regs: &Mapping) {
    if regs.rd32(REG_CONTROL) & 1 != 0 {
        tracing::info!("Maia core held in sdr_reset: releasing it for the DATV front end");
        regs.wr32(REG_CONTROL, 0);
    }
}

impl Drop for FrontEnd {
    fn drop(&mut self) {
        // A newer front end owns the hardware now: leave it running.
        if GENERATION.load(std::sync::atomic::Ordering::SeqCst) != self.generation {
            return;
        }
        self.regs.wr32(REG_REC_CONTROL, 1 << 1);
        if self.s2trk {
            self.regs.wr32(REG_S2TRK_CONTROL, 0);
        }
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

