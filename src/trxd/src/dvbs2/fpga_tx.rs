//! DATV transmit in the FPGA (maia-sdr `projects/simple/datv_tx.tcl`): the
//! DAC DMA carries framed BBFRAMEs to the DVB-S2 encoder (ORI dvb_fpga, as in
//! F5OEO's tezuka), whose symbols go through an arbitrary-rate RRC
//! interpolator (`maia_hdl/arb_interp.py`) straight to the DAC at 3.072 MS/s:
//! any symbol rate, long frames, pilots, QPSK and 8PSK.
//!
//! trxd's part: build BBFRAMEs (BBHEADER + TS, [`super::Framer`]), put each
//! behind the in-band header the encoder synchronises on (0xB8, then the
//! config byte: bit 6 = 0 normal frame, bit 5 pilots, bits 4:0 MODCOD), and
//! write the bytes into the IIO TX buffer in place of IQ; set the
//! interpolator's step and pulse; flip DAC GPIO bit 1 (DMA to the encoder,
//! encoder out of reset).
//!
//! Registers: datv_tx at 0x43C20000 (0x0 step, 0x4 coefficient address,
//! 0x8 bit 0 write strobe / bits 18:1 coefficient, 0xC id "DTX1").

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use super::{Framer, TS_LEN};

const DATV_TX_PHYS: u64 = 0x43C2_0000;
const AD9361_PHYS: u64 = 0x7902_0000;
const DAC_GPIO_OUT: usize = 0x40BC;
const DATV_BIT: u32 = 1 << 1;
/// With DATV_BIT: DMA words go to the interpolator as samples (datv_raw.v),
/// not to the DVB-S2 encoder (DVB-T2 from trxd).
const RAW_BIT: u32 = 1 << 2;
/// With RAW_BIT: 8-bit I/Q pairs, four a DMA word (half the traffic).
const RAW8_BIT: u32 = 1 << 3;
/// With RAW_BIT (16-bit words): the DVB-T2 transmit IFFT in front of the
/// interpolator (maia-hdl t2ifft.py): the words are, per T2 frame, a sync
/// word, P1's samples and each symbol's carriers.
const IFFT_BIT: u32 = 1 << 4;
const ID_DTX1: u32 = 0x3158_5444;
/// The same transmitter with the DVB-T2 IFFT in front of it.
const ID_DTX2: u32 = 0x3258_5444;

fn tx_id(id: u32) -> bool {
    id == ID_DTX1 || id == ID_DTX2
}

/// The bitstream does the DVB-T2 transmit IFFT (see [`RawMode::T2Ifft`]).
pub fn has_t2ifft() -> bool {
    let Ok(mem) = open_mem() else { return false };
    Mapping::new(&mem, 4096, DATV_TX_PHYS).is_ok_and(|r| r.rd32(0xC) == ID_DTX2)
}

/// What the raw DMA words are.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RawMode {
    /// 8-bit I/Q samples, four a word.
    Samples8,
    /// DVB-T2 frames for the FPGA's IFFT: 16-bit I/Q, two a word.
    T2Ifft,
}
pub const FS_DAC: f64 = 3_072_000.0;
const SPAN: usize = 16;
const PHASES_LOG2: u32 = 8;
const COEFF_BITS: u32 = 18;

/// The long-frame modes offered (pilots always on).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LongMode {
    Qpsk12,
    Qpsk34,
    Psk8_34,
}

impl LongMode {
    pub fn parse(s: &str) -> Option<LongMode> {
        Some(match s {
            "L-QPSK-1/2" => LongMode::Qpsk12,
            "L-QPSK-3/4" => LongMode::Qpsk34,
            "L-8PSK-3/4" => LongMode::Psk8_34,
            _ => return None,
        })
    }
    pub fn label(self) -> &'static str {
        match self {
            LongMode::Qpsk12 => "L-QPSK-1/2",
            LongMode::Qpsk34 => "L-QPSK-3/4",
            LongMode::Psk8_34 => "L-8PSK-3/4",
        }
    }
    pub fn modcod(self) -> u8 {
        match self {
            LongMode::Qpsk12 => 4,
            LongMode::Qpsk34 => 7,
            LongMode::Psk8_34 => 14,
        }
    }
    /// BCH message = BBFRAME, bits (normal frames, Table 5a).
    pub fn kbch(self) -> usize {
        match self {
            LongMode::Qpsk12 => 32_208,
            LongMode::Qpsk34 | LongMode::Psk8_34 => 48_408,
        }
    }
    fn bits_per_symbol(self) -> usize {
        match self {
            LongMode::Psk8_34 => 3,
            _ => 2,
        }
    }
    /// PLFRAME symbols with pilots: header, slots, a pilot block every 16 slots.
    pub fn frame_symbols(self) -> usize {
        let slots = 64_800 / self.bits_per_symbol() / 90;
        90 + slots * 90 + (slots - 1) / 16 * 36
    }
    /// TS bit rate at `sr`.
    pub fn ts_rate(self, sr: f64) -> f64 {
        sr * (self.kbch() - 80) as f64 / self.frame_symbols() as f64
    }
    /// The encoder's config byte: normal frame, pilots, MODCOD.
    pub fn config_byte(self) -> u8 {
        0x20 | self.modcod()
    }
}

struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: owned by one thread at a time.
unsafe impl Send for Mapping {}

impl Mapping {
    fn new(f: &File, len: usize, phys: u64) -> Result<Mapping, String> {
        // SAFETY: MAP_SHARED of a register window; accesses stay in `len`.
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
        // SAFETY: unmapping what new() mapped.
        unsafe {
            libc::munmap(self.ptr.cast(), self.len);
        }
    }
}

fn open_mem() -> Result<File, String> {
    OpenOptions::new().read(true).write(true).custom_flags(libc::O_SYNC).open("/dev/mem").map_err(|e| format!("/dev/mem: {e}"))
}

/// Does the bitstream have the DATV transmitter?
pub fn available() -> bool {
    let Ok(mem) = open_mem() else { return false };
    Mapping::new(&mem, 4096, DATV_TX_PHYS).is_ok_and(|r| tx_id(r.rd32(0xC)))
}

/// The RRC table for the interpolator (as `rrc_table` in arb_interp.py):
/// h at t = k + p/P symbols after each symbol, centred, Kaiser(3) taper,
/// scaled so the worst phase's sum of |h| is full scale.
pub fn rrc_table(rolloff: f64) -> Vec<i32> {
    let p = 1usize << PHASES_LOG2;
    let n = SPAN * p;
    let b = rolloff;
    let pi = std::f64::consts::PI;
    let bessel_i0 = |x: f64| {
        let (mut s, mut t) = (1.0, 1.0);
        for k in 1..40 {
            t *= (x / 2.0) / k as f64;
            s += t * t;
        }
        s
    };
    let h: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 / p as f64 - SPAN as f64 / 2.0;
            let v = if x.abs() < 1e-12 {
                1.0 - b + 4.0 * b / pi
            } else if (x.abs() - 1.0 / (4.0 * b)).abs() < 1e-9 {
                b / 2f64.sqrt() * ((1.0 + 2.0 / pi) * (pi / (4.0 * b)).sin() + (1.0 - 2.0 / pi) * (pi / (4.0 * b)).cos())
            } else {
                ((pi * x * (1.0 - b)).sin() + 4.0 * b * x * (pi * x * (1.0 + b)).cos()) / (pi * x * (1.0 - (4.0 * b * x).powi(2)))
            };
            // numpy.kaiser(n, 3) at index i
            let r = 2.0 * i as f64 / (n - 1) as f64 - 1.0;
            v * bessel_i0(3.0 * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(3.0)
        })
        .collect();
    let worst = (0..p).map(|ph| (0..SPAN).map(|k| h[k * p + ph].abs()).sum::<f64>()).fold(0.0, f64::max);
    let scale = ((1u64 << (COEFF_BITS - 1)) - 1) as f64 / worst;
    h.iter().map(|v| (v * scale).round() as i32).collect()
}

/// A low-pass (interpolating) pulse for the interpolator used as a
/// resampler: sinc at the input rate's Nyquist, Kaiser(6) over the 16
/// inputs (pass band flat to about 0.37 of the input rate, stop band from
/// about 0.63, better than 60 dB down), unity gain.
pub fn lowpass_table() -> Vec<i32> {
    let p = 1usize << PHASES_LOG2;
    let n = SPAN * p;
    let bessel_i0 = |x: f64| {
        let (mut s, mut t) = (1.0, 1.0);
        for k in 1..40 {
            t *= (x / 2.0) / k as f64;
            s += t * t;
        }
        s
    };
    let beta = 6.0;
    let h: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 / p as f64 - SPAN as f64 / 2.0;
            let v = if x.abs() < 1e-12 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
            let r = 2.0 * i as f64 / (n - 1) as f64 - 1.0;
            v * bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(beta)
        })
        .collect();
    // Unity gain at DC (the phase-0 taps sum to one: its centre tap is
    // then the largest coefficient that fits). Scaling for the worst case
    // instead, as for the RRC, cost 6 dB; the interpolator saturates the
    // rare peaks that overflow.
    let dc: f64 = (0..SPAN).map(|k| h[k * p]).sum();
    let peak = h.iter().fold(0f64, |m, v| m.max(v.abs()));
    let scale = (((1u64 << (COEFF_BITS - 1)) - 1) as f64 / dc).min(((1u64 << (COEFF_BITS - 1)) - 1) as f64 / peak);
    h.iter().map(|v| (v * scale).round() as i32).collect()
}

/// The interpolator's phase step for symbol rate `sr`.
pub fn step(sr: f64) -> u32 {
    (sr / FS_DAC * 4_294_967_296.0).round() as u32
}

/// DATV TX on: the pulse and rate loaded, the DMA switched to the encoder.
/// Dropping it switches back to IQ.
pub struct Transmitter {
    _mem: File,
    regs: Mapping,
    ad9361: Mapping,
    pub mode: LongMode,
    framer: Framer,
}

impl Transmitter {
    pub fn start(mode: LongMode, sr: f64, rolloff: f64) -> Result<Transmitter, String> {
        let mem = open_mem()?;
        let regs = Mapping::new(&mem, 4096, DATV_TX_PHYS).map_err(|e| format!("map datv_tx: {e}"))?;
        if !tx_id(regs.rd32(0xC)) {
            return Err("this bitstream has no DATV transmitter".into());
        }
        let ad9361 = Mapping::new(&mem, 0x1_0000, AD9361_PHYS).map_err(|e| format!("map axi_ad9361: {e}"))?;
        for (a, c) in rrc_table(rolloff).into_iter().enumerate() {
            regs.wr32(0x4, a as u32);
            regs.wr32(0x8, 1 | (((c as u32) & ((1 << COEFF_BITS) - 1)) << 1));
        }
        regs.wr32(0x0, step(sr));
        let g = ad9361.rd32(DAC_GPIO_OUT);
        ad9361.wr32(DAC_GPIO_OUT, g | DATV_BIT);
        Ok(Transmitter { _mem: mem, regs, ad9361, mode, framer: Framer::new() })
    }

    /// One frame for the encoder: 0xB8, the config byte, the BBFRAME.
    pub fn frame(&mut self, rolloff_code: u8, next: &mut dyn FnMut() -> [u8; TS_LEN], out: &mut Vec<u8>) {
        out.push(0xB8);
        out.push(self.mode.config_byte());
        out.extend(self.framer.frame_bytes(self.mode.kbch() / 8, rolloff_code, next));
    }
}

/// Raw IQ through the FPGA's interpolator (DVB-T2): the pulse is a
/// low-pass, the step the input rate over the DAC's. trxd writes 16-bit I,
/// Q pairs (two a 64-bit DMA word) or, with `eight`, 8-bit pairs (four a
/// word; the IIO path tops out near 7 MB/s, below 2.3 MS/s of 16-bit IQ)
/// into the TX buffer. Dropping it switches back to IQ.
pub struct RawTransmitter {
    _mem: File,
    ad9361: Mapping,
}

impl RawTransmitter {
    pub fn start(fs_in: f64, mode: RawMode) -> Result<RawTransmitter, String> {
        if fs_in >= FS_DAC {
            return Err(format!("{fs_in} S/s is not below the DAC rate"));
        }
        let mem = open_mem()?;
        let regs = Mapping::new(&mem, 4096, DATV_TX_PHYS).map_err(|e| format!("map datv_tx: {e}"))?;
        if !tx_id(regs.rd32(0xC)) {
            return Err("this bitstream has no DATV transmitter".into());
        }
        let ad9361 = Mapping::new(&mem, 0x1_0000, AD9361_PHYS).map_err(|e| format!("map axi_ad9361: {e}"))?;
        for (a, c) in lowpass_table().into_iter().enumerate() {
            regs.wr32(0x4, a as u32);
            regs.wr32(0x8, 1 | (((c as u32) & ((1 << COEFF_BITS) - 1)) << 1));
        }
        regs.wr32(0x0, step(fs_in));
        let g = ad9361.rd32(DAC_GPIO_OUT);
        let bits = DATV_BIT | RAW_BIT | if mode == RawMode::Samples8 { RAW8_BIT } else { IFFT_BIT };
        ad9361.wr32(DAC_GPIO_OUT, (g & !(RAW8_BIT | IFFT_BIT)) | bits);
        Ok(RawTransmitter { _mem: mem, ad9361 })
    }
}

impl Drop for RawTransmitter {
    fn drop(&mut self) {
        let g = self.ad9361.rd32(DAC_GPIO_OUT);
        self.ad9361.wr32(DAC_GPIO_OUT, g & !(DATV_BIT | RAW_BIT | RAW8_BIT | IFFT_BIT));
    }
}

impl Drop for Transmitter {
    fn drop(&mut self) {
        let g = self.ad9361.rd32(DAC_GPIO_OUT);
        self.ad9361.wr32(DAC_GPIO_OUT, g & !DATV_BIT);
        let _ = &self.regs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_sizes_and_rates() {
        assert_eq!(LongMode::Qpsk12.frame_symbols(), 33_282);
        assert_eq!(LongMode::Psk8_34.frame_symbols(), 22_194);
        // 8PSK 3/4 at 250 kS/s: about 545 kbit/s of TS.
        let r = LongMode::Psk8_34.ts_rate(250e3);
        assert!((r - 544_400.0).abs() < 1_000.0, "{r}");
        assert_eq!(LongMode::Psk8_34.config_byte(), 0x2E);
        assert_eq!(step(250e3), 349_525_333);
    }

    #[test]
    fn rrc_table_fits_the_coefficients() {
        let t = rrc_table(0.35);
        assert_eq!(t.len(), 16 * 256);
        let max = t.iter().map(|v| v.abs()).max().unwrap();
        assert!(max < (1 << 17) && max > 1 << 15, "{max}");
    }
}
