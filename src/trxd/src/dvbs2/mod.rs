//! DVB-S2 transmitter (EN 302 307-1), the low-rate subset trxd needs: MPEG-TS
//! in, QPSK short FECFRAMEs (rates 1/4 .. 2/3), CCM, optional pilots, PL
//! scrambling code 0, root-raised-cosine shaping at an integer number of
//! samples per symbol. Checked bit-exact against leandvbtx and decoded by
//! leandvb (see the tests and `--dvbs2-mod`).
//!
//! ```text
//! TS packets -> BBFRAME (BBHEADER, sync byte -> CRC-8 of the previous packet)
//!            -> BB scrambling -> BCH (t = 12) -> LDPC -> QPSK
//!            -> PLFRAME (SOF + PLS code, pilots, PL scrambling) -> RRC
//! ```

// 3/4 is generated with the others but not offered (leansdr encodes it wrong).
#[allow(dead_code)]
pub mod ddc;
pub mod fpga;
pub mod fpga_tx;
pub mod bch;
pub mod fpga_ldpc;
pub mod hdrdet;
pub mod pls;
pub mod scan;
pub mod ldpc_fpga;
pub mod symsync;
mod tables;
pub mod ldpc;
pub mod rx;
pub mod ts;

use num_complex::Complex32;

/// Short FECFRAME, bits.
pub const NLDPC: usize = 16_200;
/// A TS packet.
pub const TS_LEN: usize = 188;
/// PL slot, symbols; the PLHEADER is one slot.
const SLOT: usize = 90;
/// Pilot block, symbols (every 16 slots when pilots are on).
const PILOT: usize = 36;
/// BBHEADER, bytes.
const BBHEADER: usize = 10;

/// QPSK code rates offered (short frames).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rate {
    R1_4,
    R1_3,
    R1_2,
    R2_3,
    R3_4,
}

impl Rate {
    pub fn parse(s: &str) -> Option<Rate> {
        Some(match s.trim() {
            "1/4" => Rate::R1_4,
            "1/3" => Rate::R1_3,
            "1/2" => Rate::R1_2,
            "2/3" => Rate::R2_3,
            "3/4" => Rate::R3_4,
            _ => return None,
        })
    }
    pub fn label(self) -> &'static str {
        match self {
            Rate::R1_4 => "1/4",
            Rate::R1_3 => "1/3",
            Rate::R1_2 => "1/2",
            Rate::R2_3 => "2/3",
            Rate::R3_4 => "3/4",
        }
    }
    /// MODCOD number (QPSK).
    fn modcod(self) -> u8 {
        match self {
            Rate::R1_4 => 1,
            Rate::R1_3 => 2,
            Rate::R1_2 => 4,
            Rate::R2_3 => 6,
            Rate::R3_4 => 7,
        }
    }
    /// BCH message (= BBFRAME) size, bits (Table 5b).
    fn kbch(self) -> usize {
        match self {
            Rate::R1_4 => 3_072,
            Rate::R1_3 => 5_232,
            Rate::R1_2 => 7_032,
            Rate::R2_3 => 10_632,
            Rate::R3_4 => 11_712,
        }
    }
    /// LDPC message (= BCH codeword) size, bits.
    fn kldpc(self) -> usize {
        self.kbch() + BCH_PARITY
    }
    fn table(self) -> &'static [&'static [u16]] {
        match self {
            Rate::R1_4 => tables::SF_1_4,
            Rate::R1_3 => tables::SF_1_3,
            Rate::R1_2 => tables::SF_1_2,
            Rate::R2_3 => tables::SF_2_3,
            Rate::R3_4 => tables::SF_3_4,
        }
    }
}

/// What the receiver needs to know about the frames it receives: short
/// QPSK (the software chain) or normal frames (QPSK or 8PSK, decoded by the
/// FPGA's LDPC decoder).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameSpec {
    /// LDPC codeword bits: 16200 or 64800.
    pub n: usize,
    /// Bits per data symbol: 2 (QPSK) or 3 (8PSK).
    pub bps: usize,
    pub modcod: u8,
    pub pilots: bool,
    /// BBFRAME bits.
    pub kbch: usize,
    /// Short-frame rate (software LDPC), or the long-frame rate.
    pub short_rate: Option<Rate>,
    pub long_rate: Option<ldpc_fpga::LongRate>,
    pub rolloff: f32,
    /// Es/N0 (dB) below which a frame is not worth decoding.
    pub hopeless_db: f32,
}

impl FrameSpec {
    pub fn short(p: Params) -> Self {
        let hopeless_db = match p.rate {
            Rate::R1_4 => -5.0,
            Rate::R1_3 => -3.5,
            Rate::R1_2 => -2.0,
            Rate::R2_3 => 0.5,
            Rate::R3_4 => 1.5,
        };
        FrameSpec { n: NLDPC, bps: 2, modcod: p.rate.modcod(), pilots: p.pilots, kbch: p.rate.kbch(), short_rate: Some(p.rate), long_rate: None, rolloff: p.rolloff, hopeless_db }
    }
    pub fn long(mode: fpga_tx::LongMode) -> Self {
        use fpga_tx::LongMode::*;
        let (bps, rate, hopeless_db) = match mode {
            Qpsk12 => (2, ldpc_fpga::LongRate::R1_2, -2.0),
            Qpsk34 => (2, ldpc_fpga::LongRate::R3_4, 1.0),
            Psk8_34 => (3, ldpc_fpga::LongRate::R3_4, 5.0),
        };
        FrameSpec { n: 64_800, bps, modcod: mode.modcod(), pilots: true, kbch: mode.kbch(), short_rate: None, long_rate: Some(rate), rolloff: 0.35, hopeless_db }
    }
    pub fn is_short(&self) -> bool {
        self.n == NLDPC
    }
    /// 90-symbol slots of data.
    pub fn slots(&self) -> usize {
        self.n / self.bps / SLOT
    }
    /// PLFRAME length, symbols.
    pub fn frame_symbols(&self) -> usize {
        let s = self.slots();
        SLOT + s * SLOT + if self.pilots { (s - 1) / 16 * PILOT } else { 0 }
    }
    pub fn header(&self) -> Vec<Complex32> {
        plheader_typed(self.modcod, self.pilots, self.is_short())
    }
}

/// 8PSK (5.4.2, Figure 10): bits y0 y1 y2 as a number -> phase / (pi/4).
pub const PSK8_PHASE: [u8; 8] = [1, 0, 4, 5, 2, 7, 3, 6];

/// Transmission settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub rate: Rate,
    pub pilots: bool,
    /// Roll-off: 0.35, 0.25 or 0.20.
    pub rolloff: f32,
}

impl Params {
    /// PLFRAME length, symbols.
    pub fn frame_symbols(&self) -> usize {
        let slots = NLDPC / 2 / SLOT;
        SLOT + slots * SLOT + if self.pilots { (slots - 1) / 16 * PILOT } else { 0 }
    }
    /// TS payload bits carried per PLFRAME.
    pub fn payload_bits(&self) -> usize {
        self.rate.kbch() - BBHEADER * 8
    }
    /// TS bit rate at `symbol_rate`.
    pub fn ts_rate(&self, symbol_rate: f64) -> f64 {
        symbol_rate * self.payload_bits() as f64 / self.frame_symbols() as f64
    }
    pub(crate) fn rolloff_code(&self) -> u8 {
        if self.rolloff > 0.3 {
            0
        } else if self.rolloff > 0.225 {
            1
        } else {
            2
        }
    }
}

// ---------------------------------------------------------------- mode adaptation

/// CRC-8 of the DVB-S2 BBHEADER and user packets (5.1.4): g = 0xD5, MSB first.
fn crc8(data: &[u8]) -> u8 {
    let mut c = 0u8;
    for &b in data {
        c ^= b;
        for _ in 0..8 {
            c = if c & 0x80 != 0 { (c << 1) ^ 0xD5 } else { c << 1 };
        }
    }
    c
}

/// Packs TS packets into BBFRAMEs (5.1): packetized TS, single stream, CCM.
/// A packet may straddle two frames (SYNCD says where the first whole one
/// starts); each sync byte is replaced by the CRC-8 of the packet before it.
pub(crate) struct Framer {
    /// The unsent tail of the last packet (its first byte already replaced).
    rest: Vec<u8>,
    /// CRC-8 of the last packet taken, for the next one's sync byte.
    crc: u8,
}

impl Framer {
    pub(crate) fn new() -> Self {
        Framer { rest: Vec::new(), crc: 0 }
    }

    /// One BBFRAME of `p.rate.kbch() / 8` bytes.
    fn frame(&mut self, p: &Params, next: &mut dyn FnMut() -> [u8; TS_LEN]) -> Vec<u8> {
        self.frame_bytes(p.rate.kbch() / 8, p.rolloff_code(), next)
    }

    /// One BBFRAME of `len` bytes (Kbch / 8, any frame size and rate).
    pub(crate) fn frame_bytes(&mut self, len: usize, rolloff_code: u8, next: &mut dyn FnMut() -> [u8; TS_LEN]) -> Vec<u8> {
        let mut f = Vec::with_capacity(len);
        let dfl = (len - BBHEADER) * 8;
        let syncd = self.rest.len() * 8;
        f.push(0xC0 | 0x20 | 0x10 | rolloff_code); // TS, SIS, CCM, roll-off
        f.push(0); // MATYPE-2
        f.extend_from_slice(&((TS_LEN * 8) as u16).to_be_bytes()); // UPL
        f.extend_from_slice(&(dfl as u16).to_be_bytes());
        f.push(0x47); // SYNC
        f.extend_from_slice(&(syncd as u16).to_be_bytes());
        let c = crc8(&f[..9]);
        f.push(c);
        let take = self.rest.len().min(len - f.len());
        f.extend(self.rest.drain(..take));
        while f.len() < len {
            let pkt = next();
            let mut up = [0u8; TS_LEN];
            up[0] = self.crc;
            up[1..].copy_from_slice(&pkt[1..]);
            self.crc = crc8(&pkt[1..]);
            let n = (len - f.len()).min(TS_LEN);
            f.extend_from_slice(&up[..n]);
            self.rest.extend_from_slice(&up[n..]);
        }
        f
    }
}

// ---------------------------------------------------------------- FEC

/// BCH parity bits for short frames: 12 polynomials of degree 14.
const BCH_PARITY: usize = 168;

/// BB scrambler sequence (5.2.2): 1 + X^14 + X^15, loaded with 100101010000000.
pub(crate) fn bb_scrambling(len: usize) -> Vec<u8> {
    let mut st: u16 = 0x00A9;
    (0..len)
        .map(|_| {
            let mut out = 0u8;
            for _ in 0..8 {
                let bit = ((st >> 13) ^ (st >> 14)) & 1;
                out = (out << 1) | bit as u8;
                st = (st << 1) | bit;
            }
            out
        })
        .collect()
}

/// The short-frame BCH generator (5.3.1, Table 6b): the product of g1 .. g12,
/// as 169 coefficients, index = degree.
fn bch_generator() -> Vec<u8> {
    const POLYS: [u32; 12] =
        [0x402B, 0x4941, 0x4647, 0x5591, 0x6B55, 0x6389, 0x6CE5, 0x4F21, 0x460F, 0x5A49, 0x5811, 0x65EF];
    let mut g = vec![1u8];
    for p in POLYS {
        let mut out = vec![0u8; g.len() + 14];
        for (i, &gi) in g.iter().enumerate() {
            if gi == 0 {
                continue;
            }
            for d in 0..=14 {
                if (p >> d) & 1 == 1 {
                    out[i + d] ^= 1;
                }
            }
        }
        g = out;
    }
    debug_assert_eq!(g.len(), BCH_PARITY + 1);
    g
}

/// 168-bit shift register in three words (bit 167 = most significant).
#[derive(Clone, Copy, Default)]
struct Reg168([u64; 3]);

impl Reg168 {
    fn msb(&self) -> bool {
        (self.0[2] >> (BCH_PARITY - 128 - 1)) & 1 == 1
    }
    fn shl1(&mut self) {
        self.0[2] = ((self.0[2] << 1) | (self.0[1] >> 63)) & ((1u64 << (BCH_PARITY - 128)) - 1);
        self.0[1] = (self.0[1] << 1) | (self.0[0] >> 63);
        self.0[0] <<= 1;
    }
    fn xor(&mut self, o: &Reg168) {
        for i in 0..3 {
            self.0[i] ^= o.0[i];
        }
    }
    fn bit(&self, i: usize) -> bool {
        (self.0[i / 64] >> (i % 64)) & 1 == 1
    }
}

struct Fec {
    rate: Rate,
    scramble: Vec<u8>,
    /// The generator without its X^168 term.
    g: Reg168,
}

impl Fec {
    fn new(rate: Rate) -> Self {
        let poly = bch_generator();
        let mut g = Reg168::default();
        for (d, &c) in poly.iter().enumerate().take(BCH_PARITY) {
            if c == 1 {
                g.0[d / 64] |= 1 << (d % 64);
            }
        }
        Fec { rate, scramble: bb_scrambling(rate.kbch() / 8), g }
    }

    /// BBFRAME (Kbch/8 bytes) -> FECFRAME bits (16200, one per byte, 0/1).
    fn encode(&self, bb: &[u8]) -> Vec<u8> {
        let kbch = self.rate.kbch();
        let kldpc = self.rate.kldpc();
        let mut bits = Vec::with_capacity(NLDPC);
        for (b, s) in bb.iter().zip(&self.scramble) {
            let v = b ^ s;
            bits.extend((0..8).rev().map(|i| (v >> i) & 1));
        }
        debug_assert_eq!(bits.len(), kbch);
        // BCH: remainder of m(x) * x^168 divided by g(x), highest degree first.
        let mut r = Reg168::default();
        for &m in &bits {
            let fb = r.msb() ^ (m == 1);
            r.shl1();
            if fb {
                r.xor(&self.g);
            }
        }
        bits.extend((0..BCH_PARITY).rev().map(|i| r.bit(i) as u8));
        // LDPC (5.3.2): accumulate each information bit into the parity
        // addresses of its row, then p[i] ^= p[i-1].
        let nk = NLDPC - kldpc;
        let q = nk / 360;
        let mut p = vec![0u8; nk];
        for (row, addrs) in self.rate.table().iter().enumerate() {
            for m in 0..360 {
                if bits[row * 360 + m] == 0 {
                    continue;
                }
                for &x in addrs.iter() {
                    p[(x as usize + m * q) % nk] ^= 1;
                }
            }
        }
        for i in 1..nk {
            p[i] ^= p[i - 1];
        }
        bits.extend_from_slice(&p);
        bits
    }
}

// ---------------------------------------------------------------- physical layer

/// The 90 PLHEADER symbols (5.5.2): SOF and the PLS code, pi/2-BPSK.
fn plheader(modcod: u8, pilots: bool) -> Vec<Complex32> {
    plheader_typed(modcod, pilots, true)
}

/// PLHEADER for either frame size (TYPE bit 1: short).
pub(crate) fn plheader_typed(modcod: u8, pilots: bool, short: bool) -> Vec<Complex32> {
    const SOF: u32 = 0x18D_2E82;
    const PLS_SCRAMBLE: u64 = 0x719D_83C9_5342_2DFA;
    const G: [u32; 6] = [0x5555_5555, 0x3333_3333, 0x0F0F_0F0F, 0x00FF_00FF, 0x0000_FFFF, 0xFFFF_FFFF];
    let index = ((modcod as u32) << 2) | ((short as u32) << 1) | pilots as u32;
    let mut y = 0u32;
    for (row, g) in G.iter().enumerate() {
        if (index >> (6 - row)) & 1 == 1 {
            y ^= g;
        }
    }
    let mut code = 0u64;
    for bit in (0..32).rev() {
        let yi = ((y >> bit) & 1) as u64;
        let second = if index & 1 == 1 { yi ^ 1 } else { yi };
        code = (code << 2) | (yi << 1) | second;
    }
    code ^= PLS_SCRAMBLE;
    let mut bits: Vec<u8> = (0..26).rev().map(|i| ((SOF >> i) & 1) as u8).collect();
    bits.extend((0..64).rev().map(|i| ((code >> i) & 1) as u8));
    // pi/2-BPSK: bit 0 -> 45 deg (even symbols) or 135 deg (odd), bit 1 opposite.
    bits.iter()
        .enumerate()
        .map(|(s, &b)| {
            let q = (b as usize * 2 + (s & 1)) as f32;
            let a = std::f32::consts::FRAC_PI_4 + std::f32::consts::FRAC_PI_2 * q;
            Complex32::new(a.cos(), a.sin())
        })
        .collect()
}

/// PL scrambling rotations (5.5.4), code 0: R(i) = 2 z(i + 131072) + z(i).
fn pl_scrambling(len: usize) -> Vec<u8> {
    let (mut x, mut y) = (1u32, 0x3FFFFu32);
    let mut z = Vec::with_capacity(131_072 + len);
    for _ in 0..131_072 + len {
        z.push(((x ^ y) & 1) as u8);
        let bx = ((x >> 7) ^ x) & 1;
        x = ((bx << 18) | x) >> 1;
        let by = ((y >> 10) ^ (y >> 7) ^ (y >> 5) ^ y) & 1;
        y = ((by << 18) | y) >> 1;
    }
    (0..len).map(|i| z[i] | (z[i + 131_072] << 1)).collect()
}

/// Rotate by `r` quarter turns.
fn rotate(s: Complex32, r: u8) -> Complex32 {
    match r & 3 {
        0 => s,
        1 => Complex32::new(-s.im, s.re),
        2 => -s,
        _ => Complex32::new(s.im, -s.re),
    }
}

/// Everything from TS packets to unit-power PLFRAME symbols.
pub struct Encoder {
    p: Params,
    framer: Framer,
    fec: Fec,
    header: Vec<Complex32>,
    scramble: Vec<u8>,
}

impl Encoder {
    pub fn new(p: Params) -> Self {
        let n = p.frame_symbols() - SLOT;
        Encoder {
            p,
            framer: Framer::new(),
            fec: Fec::new(p.rate),
            header: plheader(p.rate.modcod(), p.pilots),
            scramble: pl_scrambling(n),
        }
    }

    pub fn params(&self) -> Params {
        self.p
    }

    /// One PLFRAME; `next` hands over TS packets as they are needed.
    pub fn frame(&mut self, next: &mut dyn FnMut() -> [u8; TS_LEN], out: &mut Vec<Complex32>) {
        let bb = self.framer.frame(&self.p, next);
        let bits = self.fec.encode(&bb);
        out.extend_from_slice(&self.header);
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let pilot = Complex32::new(a, a);
        let mut k = 0; // PL scrambling index: data and pilot symbols
        for (n, pair) in bits.chunks_exact(2).enumerate() {
            if self.p.pilots && n > 0 && n % (16 * SLOT) == 0 {
                for _ in 0..PILOT {
                    out.push(rotate(pilot, self.scramble[k]));
                    k += 1;
                }
            }
            // QPSK (5.4.1): first bit -> sign of I, second -> sign of Q.
            let s = Complex32::new(if pair[0] == 0 { a } else { -a }, if pair[1] == 0 { a } else { -a });
            out.push(rotate(s, self.scramble[k]));
            k += 1;
        }
    }
}

// ---------------------------------------------------------------- pulse shaping

/// Root-raised-cosine pulse at `t` symbols from its centre (peak
/// 1 - b + 4b/pi).
pub(crate) fn rrc_at(t: f64, b: f64) -> f64 {
    let pi = std::f64::consts::PI;
    if t.abs() < 1e-9 {
        1.0 - b + 4.0 * b / pi
    } else if (t.abs() - 1.0 / (4.0 * b)).abs() < 1e-9 {
        b / 2f64.sqrt() * ((1.0 + 2.0 / pi) * (pi / (4.0 * b)).sin() + (1.0 - 2.0 / pi) * (pi / (4.0 * b)).cos())
    } else {
        ((pi * t * (1.0 - b)).sin() + 4.0 * b * t * (pi * t * (1.0 + b)).cos()) / (pi * t * (1.0 - (4.0 * b * t).powi(2)))
    }
}

/// Root-raised-cosine taps, `sps` samples per symbol over `span` symbols,
/// scaled so that unit-power symbols come out at unit power.
fn rrc_taps(sps: usize, rolloff: f32, span: usize) -> Vec<f32> {
    rrc_taps_frac(sps as f64, rolloff, span)
}

/// [`rrc_taps`] at a fractional `sps` (an odd number of taps, centred).
pub(crate) fn rrc_taps_frac(sps: f64, rolloff: f32, span: usize) -> Vec<f32> {
    let n = 2 * (span as f64 * sps / 2.0).round() as usize + 1;
    let mut h: Vec<f64> = (0..n).map(|i| rrc_at((i as f64 - (n - 1) as f64 / 2.0) / sps, rolloff as f64)).collect();
    let e: f64 = h.iter().map(|x| x * x).sum();
    let k = (sps / e).sqrt();
    h.iter_mut().for_each(|x| *x *= k);
    h.into_iter().map(|x| x as f32).collect()
}

/// Continuous DVB-S2 baseband at `sps` samples per symbol: PLFRAMEs back to
/// back (CCM), RRC-shaped, unit mean power.
pub struct Modulator {
    enc: Encoder,
    /// Polyphase RRC: `phases[p][j]` weights symbol `k - j` for output `k*sps + p`.
    phases: Vec<Vec<f32>>,
    hist: Vec<Complex32>,
    symbols: Vec<Complex32>,
    pos: usize,
    out: Vec<Complex32>,
    out_pos: usize,
}

impl Modulator {
    pub fn new(p: Params, sps: usize) -> Self {
        let taps = rrc_taps(sps, p.rolloff, 12);
        let per = taps.len().div_ceil(sps);
        let phases = (0..sps).map(|ph| (0..per).map(|j| taps.get(j * sps + ph).copied().unwrap_or(0.0)).collect()).collect();
        Modulator {
            enc: Encoder::new(p),
            phases,
            hist: vec![Complex32::default(); per],
            symbols: Vec::new(),
            pos: 0,
            out: Vec::new(),
            out_pos: 0,
        }
    }

    pub fn params(&self) -> Params {
        self.enc.params()
    }

    /// Fill `out` with baseband; TS packets are pulled from `next` as frames
    /// are built (one frame's worth at a time).
    pub fn fill(&mut self, out: &mut [Complex32], next: &mut dyn FnMut() -> [u8; TS_LEN]) {
        let mut i = 0;
        while i < out.len() {
            if self.out_pos == self.out.len() {
                self.out.clear();
                self.out_pos = 0;
                if self.pos == self.symbols.len() {
                    self.symbols.clear();
                    self.pos = 0;
                    self.enc.frame(next, &mut self.symbols);
                }
                // One symbol in, `sps` samples out.
                self.hist.rotate_right(1);
                self.hist[0] = self.symbols[self.pos];
                self.pos += 1;
                for ph in &self.phases {
                    let mut acc = Complex32::default();
                    for (h, s) in ph.iter().zip(&self.hist) {
                        acc += s * *h;
                    }
                    self.out.push(acc);
                }
            }
            let n = (self.out.len() - self.out_pos).min(out.len() - i);
            out[i..i + n].copy_from_slice(&self.out[self.out_pos..self.out_pos + n]);
            self.out_pos += n;
            i += n;
        }
    }
}

// ---------------------------------------------------------------- CLI

/// `trxd --dvbs2-mod IN.ts OUT.cf32 [RATE] [SPS] [pilots]`: modulate a TS
/// file to complex float32 IQ (SPS 1 = the raw PLFRAME symbols, for comparing
/// with leandvbtx -f 1). The file is played once, then the last frame is
/// completed with null packets.
pub fn mod_cli(input: &str, output: &str, rest: &[String]) -> Result<(), String> {
    let rate = rest.first().map_or(Some(Rate::R1_2), |s| Rate::parse(s)).ok_or("rate: 1/4, 1/3, 1/2 or 2/3")?;
    let sps: usize = rest.get(1).map_or(Ok(1), |s| s.parse()).map_err(|_| "sps: an integer")?;
    let pilots = rest.iter().any(|s| s == "pilots");
    let ts = std::fs::read(input).map_err(|e| format!("{input}: {e}"))?;
    if ts.len() % TS_LEN != 0 || ts.chunks(TS_LEN).any(|p| p[0] != 0x47) {
        return Err(format!("{input}: not a whole number of 188-byte TS packets"));
    }
    let p = Params { rate, pilots, rolloff: 0.35 };
    let mut packets = ts.chunks(TS_LEN);
    let left = std::cell::Cell::new(ts.len() / TS_LEN);
    let mut next = || -> [u8; TS_LEN] {
        let mut pkt = null_packet();
        if let Some(c) = packets.next() {
            pkt.copy_from_slice(c);
            left.set(left.get() - 1);
        }
        pkt
    };
    let mut iq = Vec::new();
    if sps == 1 {
        let mut enc = Encoder::new(p);
        let mut done = false;
        while !done {
            enc.frame(&mut next, &mut iq);
            done = left.get() == 0;
        }
    } else {
        let mut m = Modulator::new(p, sps);
        let frame = p.frame_symbols() * sps;
        let mut buf = vec![Complex32::default(); frame];
        loop {
            m.fill(&mut buf, &mut next);
            iq.extend_from_slice(&buf);
            if left.get() == 0 {
                // Flush the filter and the frame in flight.
                m.fill(&mut buf, &mut next);
                iq.extend_from_slice(&buf);
                break;
            }
        }
    }
    let bytes: Vec<u8> = iq.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()]).flatten().collect();
    std::fs::write(output, bytes).map_err(|e| format!("{output}: {e}"))?;
    eprintln!(
        "DVB-S2 QPSK {} short{}, {} symbols ({} per frame, TS {:.0} bit/s at 64 kS/s)",
        rate.label(),
        if pilots { " pilots" } else { "" },
        iq.len() / sps,
        p.frame_symbols(),
        p.ts_rate(64_000.0)
    );
    Ok(())
}

/// An MPEG-TS null packet (PID 0x1FFF).
pub fn null_packet() -> [u8; TS_LEN] {
    let mut p = [0xFFu8; TS_LEN];
    p[..4].copy_from_slice(&[0x47, 0x1F, 0xFF, 0x10]);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_matches_the_dvb_s2_polynomial() {
        // g = X^8 + X^7 + X^6 + X^4 + X^2 + 1; CRC of 0x80 is g's low byte shifted once.
        assert_eq!(crc8(&[0x01]), 0xD5);
        assert_eq!(crc8(&[]), 0);
    }

    #[test]
    fn bch_codewords_are_multiples_of_the_generator() {
        // A codeword (message then parity) divided by g leaves no remainder.
        let fec = Fec::new(Rate::R1_2);
        let bb: Vec<u8> = (0..Rate::R1_2.kbch() / 8).map(|i| (i * 37 + 11) as u8).collect();
        let bits = fec.encode(&bb);
        let cw = &bits[..Rate::R1_2.kldpc()];
        let mut r = Reg168::default();
        for &b in cw {
            let fb = r.msb();
            r.shl1();
            r.0[0] |= b as u64;
            if fb {
                r.xor(&fec.g);
            }
        }
        assert_eq!(r.0, [0; 3]);
    }

    #[test]
    fn ldpc_codewords_satisfy_every_parity_check() {
        for rate in [Rate::R1_4, Rate::R1_3, Rate::R1_2, Rate::R2_3, Rate::R3_4] {
            let fec = Fec::new(rate);
            let bb: Vec<u8> = (0..rate.kbch() / 8).map(|i| (i * 101 + 7) as u8).collect();
            let bits = fec.encode(&bb);
            let (k, nk) = (rate.kldpc(), NLDPC - rate.kldpc());
            let q = nk / 360;
            // Check j: sum of the information bits addressed to it, and p[j] ^ p[j-1].
            let mut chk = vec![0u8; nk];
            for (row, addrs) in rate.table().iter().enumerate() {
                for m in 0..360 {
                    if bits[row * 360 + m] == 1 {
                        for &x in addrs.iter() {
                            chk[(x as usize + m * q) % nk] ^= 1;
                        }
                    }
                }
            }
            for j in 0..nk {
                let prev = if j > 0 { bits[k + j - 1] } else { 0 };
                assert_eq!(chk[j] ^ bits[k + j] ^ prev, 0, "rate {} check {j}", rate.label());
            }
        }
    }

    #[test]
    fn frames_have_the_expected_length_and_rate() {
        let p = Params { rate: Rate::R1_2, pilots: false, rolloff: 0.35 };
        assert_eq!(p.frame_symbols(), 8_190);
        assert_eq!(Params { pilots: true, ..p }.frame_symbols(), 8_370);
        // 64 kS/s, 1/2: 6952 payload bits per 8190 symbols.
        assert!((p.ts_rate(64_000.0) - 54_325.0).abs() < 1.0, "{}", p.ts_rate(64_000.0));
        let mut enc = Encoder::new(p);
        let mut out = Vec::new();
        let mut next = null_packet;
        enc.frame(&mut next, &mut out);
        assert_eq!(out.len(), 8_190);
        assert!(out.iter().all(|s| (s.norm() - 1.0).abs() < 1e-5));
    }

    #[test]
    fn packets_straddle_frames_and_come_back_whole() {
        // Mode adaptation alone: undo the CRC-for-sync swap and re-join.
        let p = Params { rate: Rate::R1_4, pilots: false, rolloff: 0.35 };
        let mut f = Framer::new();
        let mut n = 0u8;
        let mut next = || {
            let mut pkt = [n; TS_LEN];
            pkt[0] = 0x47;
            n = n.wrapping_add(1);
            pkt
        };
        let mut stream = Vec::new();
        for _ in 0..5 {
            let bb = f.frame(&p, &mut next);
            assert_eq!(crc8(&bb[..9]), bb[9]);
            let syncd = u16::from_be_bytes([bb[7], bb[8]]) as usize / 8;
            if stream.is_empty() {
                assert_eq!(syncd, 0);
            }
            stream.extend_from_slice(&bb[BBHEADER..]);
        }
        for (i, up) in stream.chunks_exact(TS_LEN).enumerate() {
            assert!(up[1..].iter().all(|&b| b == i as u8), "packet {i}");
            if i > 0 {
                assert_eq!(up[0], crc8(&[i as u8 - 1; TS_LEN - 1]));
            }
        }
    }

    #[test]
    fn modulator_output_has_unit_power() {
        let p = Params { rate: Rate::R1_2, pilots: true, rolloff: 0.35 };
        let mut m = Modulator::new(p, 6);
        let mut buf = vec![Complex32::default(); 8_370 * 6];
        let mut next = null_packet;
        m.fill(&mut buf, &mut next);
        let pw: f32 = buf[600..].iter().map(|z| z.norm_sqr()).sum::<f32>() / (buf.len() - 600) as f32;
        assert!((pw - 1.0).abs() < 0.05, "{pw}");
    }
}
