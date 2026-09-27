//! DVB-T2 transmitter (EN 302 755) for amateur DATV: the narrow channels
//! Portsdown 4 / Ryde / Knucker / Lynx use (1.7 MHz = 131/71 MS/s, and the
//! 8/7 x bandwidth family), 2K FFT, one PLP, SISO, normal FEC frames.
//!
//! A port of GNU Radio gr-dtv's DVB-T2 blocks, stage for stage, checked
//! against them bit for bit / cell for cell (`t2_matches_gr_dtv`, with the
//! stage dumps of datv-ref/t2/ref/t2ref.py OUTDIR 190 9):
//!
//! ```text
//! TS -> BBFRAME (dvbs2::Framer, roll-off bits 0) -> BB scrambling -> BCH
//!    -> LDPC 64800 (the DVB-S2 codes) -> cells (QPSK: two bits a cell)
//!    -> mapping (optionally rotated, Q delayed a cell) -> cell interleaver
//!    -> time interleaver -> frame (L1-pre, L1-post in the P2 symbols, data,
//!    dummy cells) -> frequency interleaver -> pilots, IFFT -> guard
//!    interval -> P1
//! ```

pub mod frame;
pub mod l1;
pub mod ofdm;
pub mod tables;
pub mod tx;
#[cfg(test)]
pub mod rx;

use num_complex::Complex32;

use crate::dvbs2::ldpc_fpga::LongRate;

pub const FFT: usize = 2048;
/// P2 symbols and their cells (2K, SISO).
pub const N_P2: usize = 8;
pub const C_P2: usize = 1118;
/// L1-pre cells (BPSK).
pub const L1_PRE_CELLS: usize = 1840;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Constellation {
    Qpsk,
}

impl Constellation {
    /// L1 PLP_MOD / gr-dtv dvb_constellation_t.
    pub fn code(self) -> u32 {
        0
    }
    pub fn bits(self) -> usize {
        2
    }
}

/// Guard interval (L1 GUARD_INTERVAL codes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Guard {
    G1_32 = 0,
    G1_16 = 1,
    G1_8 = 2,
    G1_4 = 3,
    G1_128 = 4,
    G19_128 = 5,
    G19_256 = 6,
}

impl Guard {
    pub fn samples(self) -> usize {
        match self {
            Guard::G1_32 => FFT / 32,
            Guard::G1_16 => FFT / 16,
            Guard::G1_8 => FFT / 8,
            Guard::G1_4 => FFT / 4,
            Guard::G1_128 => FFT / 128,
            Guard::G19_128 => FFT * 19 / 128,
            Guard::G19_256 => FFT * 19 / 256,
        }
    }
}

/// Scattered pilot pattern (L1 PILOT_PATTERN codes: PP1 = 0).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pilots {
    PP1 = 0,
    PP2 = 1,
    PP3 = 2,
    PP4 = 3,
    PP5 = 4,
    PP7 = 6,
}

impl Pilots {
    /// (Dx, Dy): scattered pilot carrier and symbol spacing.
    pub fn dxdy(self) -> (usize, usize) {
        match self {
            Pilots::PP1 => (3, 4),
            Pilots::PP2 => (6, 2),
            Pilots::PP3 => (6, 4),
            Pilots::PP4 => (12, 2),
            Pilots::PP5 => (12, 4),
            Pilots::PP7 => (24, 4),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub constellation: Constellation,
    pub rate: LongRate,
    pub rotation: bool,
    pub guard: Guard,
    pub pilots: Pilots,
    /// Data symbols per T2 frame (after the P2 symbols).
    pub data_symbols: usize,
    /// FEC blocks per T2 frame (one TI block).
    pub fec_blocks: usize,
    /// T2 frames per super-frame.
    pub t2_frames: usize,
}

impl Params {
    /// The amateur default (Portsdown DVB-T2): QPSK 1/2, GI 1/8, PP2.
    pub fn amateur() -> Params {
        Params {
            constellation: Constellation::Qpsk,
            rate: LongRate::R1_2,
            rotation: false,
            guard: Guard::G1_8,
            pilots: Pilots::PP2,
            // 250 ms frames less a symbol; 9 FEC blocks fill them (5609
            // dummy cells): TS 1.164 Mbit/s at 1.7 MHz.
            data_symbols: 190,
            fec_blocks: 9,
            t2_frames: 2,
        }
    }

    pub fn kbch(&self) -> usize {
        match self.rate {
            LongRate::R1_2 => 32_208,
            LongRate::R3_4 => 48_408,
        }
    }

    /// L1 PLP_COD.
    pub fn rate_code(&self) -> u32 {
        match self.rate {
            LongRate::R1_2 => 0,
            LongRate::R3_4 => 3,
        }
    }

    /// Cells of one FEC block.
    pub fn cells(&self) -> usize {
        64_800 / self.constellation.bits()
    }

    /// (C_DATA, N_FC, C_FC): active cells of a data symbol and of the frame
    /// closing symbol (N_FC = 0: no frame closing symbol), 2K.
    pub fn data_cells(&self) -> (usize, usize, usize) {
        let (c, n, cfc) = match self.pilots {
            Pilots::PP1 => (1522, 1136, 804),
            Pilots::PP2 => (1532, 1420, 1309),
            Pilots::PP3 => (1596, 1420, 980),
            Pilots::PP4 => (1602, 1562, 1415),
            Pilots::PP5 => (1632, 1562, 1088),
            Pilots::PP7 => (1646, 1632, 1396),
        };
        let no_fc = matches!(
            (self.guard, self.pilots),
            (Guard::G1_128, Pilots::PP7) | (Guard::G1_32, Pilots::PP4) | (Guard::G1_16, Pilots::PP2) | (Guard::G19_256, Pilots::PP2)
        );
        if no_fc { (c, 0, 0) } else { (c, n, cfc) }
    }

    /// Cells a T2 frame carries after the frame builder.
    pub fn frame_cells(&self) -> usize {
        let (c_data, n_fc, _) = self.data_cells();
        if n_fc == 0 {
            N_P2 * C_P2 + self.data_symbols * c_data
        } else {
            N_P2 * C_P2 + (self.data_symbols - 1) * c_data + n_fc
        }
    }

    /// OFDM symbols in a T2 frame after P1 (P2 and data).
    pub fn symbols(&self) -> usize {
        N_P2 + self.data_symbols
    }

    /// Samples of a T2 frame at the elementary rate (P1 is 2048).
    pub fn frame_samples(&self) -> usize {
        2048 + self.symbols() * (FFT + self.guard.samples())
    }
}

/// Cells travel as one-byte codes through the interleavers and the frame
/// builder (a T2 frame of complex cells is 2.4 MB: the A9's caches cannot
/// hold it, and moving it around cost more than the IFFTs), and become
/// complex values only in the IFFT input:
/// - 0..16: data, (QPSK word << 2) | the previous cell's word (rotation
///   takes Q from the cell before);
/// - 16, 17: BPSK +1 / -1 (L1-pre, dummy cells);
/// - 18..22: L1-post QPSK words;
/// - 22: nothing (unmodulated).
pub type Cell = u8;
pub const BPSK0: Cell = 16;
pub const L1_QPSK: Cell = 18;
pub const ZERO: Cell = 22;

/// Code -> complex value.
pub struct Palette(pub [Complex32; 23]);

impl Palette {
    pub fn new(p: &Params) -> Palette {
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let plain = [Complex32::new(a, a), Complex32::new(a, -a), Complex32::new(-a, a), Complex32::new(-a, -a)];
        let mut pts = plain;
        if p.rotation {
            let r = 2.0 * std::f64::consts::PI * 29.0 / 360.0;
            for z in pts.iter_mut() {
                let zr = num_complex::Complex64::new(z.re as f64, z.im as f64) * num_complex::Complex64::from_polar(1.0, r);
                *z = Complex32::new(zr.re as f32, zr.im as f32);
            }
        }
        let mut v = [Complex32::default(); 23];
        for cur in 0..4 {
            for prev in 0..4 {
                v[cur << 2 | prev] = if p.rotation { Complex32::new(pts[cur].re, pts[prev].im) } else { pts[cur] };
            }
        }
        v[16] = Complex32::new(1.0, 0.0);
        v[17] = Complex32::new(-1.0, 0.0);
        for w in 0..4 {
            v[18 + w] = plain[w];
        }
        Palette(v)
    }

    pub fn get(&self, c: Cell) -> Complex32 {
        self.0[c as usize]
    }
}

/// Cell codes of one FEC block: two bits a QPSK cell (no bit interleaving
/// for QPSK at these rates), with the previous cell's word (cyclically in
/// the block) for the rotated constellation's delayed Q.
pub fn cell_codes(codeword: &[u8]) -> Vec<Cell> {
    let n = codeword.len() / 2;
    let word = |j: usize| (codeword[2 * j] << 1) | codeword[2 * j + 1];
    (0..n).map(|j| (word(j) << 2) | word((j + n - 1) % n)).collect()
}

/// Cells of one FEC block as complex values (for comparisons).
pub fn map_cells(p: &Params, codeword: &[u8]) -> Vec<Complex32> {
    let pal = Palette::new(p);
    cell_codes(codeword).iter().map(|&c| pal.get(c)).collect()
}

/// Cell interleaver permutation (a 15-bit LFSR for 32400 cells).
fn cell_permutation(cells: usize) -> (Vec<usize>, u32) {
    // QPSK normal: degree 15, taps 0, 1, 2, 12.
    let (pn_degree, pn_mask, max_states, logic): (u32, u32, u32, &[u32]) = match cells {
        32_400 => (15, 0x3fff, 32768, &[0, 1, 2, 12]),
        _ => unimplemented!("cell interleaver for {cells} cells"),
    };
    let mut perm = Vec::with_capacity(cells);
    let mut lfsr: u32 = 0;
    for i in 0..max_states {
        if i == 0 || i == 1 {
            lfsr = 0;
        } else if i == 2 {
            lfsr = 1;
        } else {
            let mut r = 0;
            for &k in logic {
                r ^= (lfsr >> k) & 1;
            }
            lfsr &= pn_mask;
            lfsr >>= 1;
            lfsr |= r << (pn_degree - 2);
        }
        lfsr |= (i % 2) << (pn_degree - 1);
        if (lfsr as usize) < cells {
            perm.push(lfsr as usize);
        }
    }
    (perm, pn_degree)
}

/// Cell interleaving of each FEC block and time interleaving of the frame's
/// blocks (one TI block, 5 columns a FEC block).
pub fn interleave<T: Copy + Default>(p: &Params, blocks: &[Vec<T>]) -> Vec<T> {
    CellInterleaver::new(p).frame(blocks)
}

/// [`interleave`] with the cell permutation worked out once.
pub struct CellInterleaver {
    cells: usize,
    perm: Vec<usize>,
    shifts: Vec<usize>,
}

impl CellInterleaver {
    pub fn new(p: &Params) -> CellInterleaver {
        let cells = p.cells();
        let (perm, pn_degree) = cell_permutation(cells);
        // Each FEC block's cyclic shift: the bit-reversed counter (doubled),
        // skipping values of cells or more.
        let mut n = 0u32;
        let shifts = (0..p.fec_blocks)
            .map(|_| {
                let mut shift = cells;
                while shift >= cells {
                    let mut temp = n;
                    shift = 0;
                    for _ in 0..pn_degree {
                        shift |= (temp & 1) as usize;
                        shift <<= 1;
                        temp >>= 1;
                    }
                    n += 1;
                }
                shift
            })
            .collect();
        CellInterleaver { cells, perm, shifts }
    }

    pub fn frame<T: Copy + Default>(&self, blocks: &[Vec<T>]) -> Vec<T> {
        let cells = self.cells;
        let mut ti = vec![T::default(); cells * blocks.len()];
        for (r, blk) in blocks.iter().enumerate() {
            let shift = self.shifts[r];
            let base = r * cells;
            for (w, &c) in blk.iter().enumerate() {
                let mut x = self.perm[w] + shift;
                if x >= cells {
                    x -= cells;
                }
                ti[x + base] = c;
            }
        }
        let cols = 5 * blocks.len();
        let rows = cells / 5;
        let mut out = Vec::with_capacity(ti.len());
        for k in 0..rows {
            for w in 0..cols {
                out.push(ti[rows * w + k]);
            }
        }
        out
    }

    /// The reverse (receive): a frame's data cells back to FEC blocks.
    pub fn deinterleave<T: Copy + Default>(&self, data: &[T], blocks: usize) -> Vec<Vec<T>> {
        let cells = self.cells;
        let cols = 5 * blocks;
        let rows = cells / 5;
        let mut ti = vec![T::default(); cells * blocks];
        for k in 0..rows {
            for w in 0..cols {
                ti[rows * w + k] = data[k * cols + w];
            }
        }
        (0..blocks)
            .map(|r| {
                let shift = self.shifts[r];
                (0..cells)
                    .map(|w| {
                        let mut x = self.perm[w] + shift;
                        if x >= cells {
                            x -= cells;
                        }
                        ti[x + r * cells]
                    })
                    .collect()
            })
            .collect()
    }
}

/// TS in, T2 frames out (complex baseband at the elementary rate, P1 to
/// the last symbol's end).
pub struct Modulator {
    pub p: Params,
    framer: crate::dvbs2::Framer,
    bbscr: Vec<u8>,
    bch: crate::dvbs2::bch::Bch,
    ci: CellInterleaver,
    mapper: frame::FrameMapper,
    fi: frame::FreqInterleaver,
    ofdm: ofdm::Ofdm,
}

impl Modulator {
    pub fn new(p: Params) -> Modulator {
        Self::oversampled(p, 4)
    }

    /// Output at `os4 / 4` times the elementary rate (see [`ofdm`]).
    pub fn oversampled(p: Params, os4: usize) -> Modulator {
        Modulator {
            ci: CellInterleaver::new(&p),
            p,
            framer: crate::dvbs2::Framer::new(),
            bbscr: crate::dvbs2::bb_scrambling(p.kbch() / 8),
            bch: crate::dvbs2::bch::Bch::new(),
            mapper: frame::FrameMapper::new(p),
            fi: frame::FreqInterleaver::new(&p),
            ofdm: ofdm::Ofdm::oversampled(p, os4),
        }
    }

    /// Samples a frame makes.
    pub fn frame_samples(&self) -> usize {
        self.ofdm.frame_samples()
    }

    /// One FEC block's codeword bits from the next TS packets.
    pub fn codeword(&mut self, next: &mut dyn FnMut() -> [u8; crate::dvbs2::TS_LEN]) -> Vec<u8> {
        let kbch = self.p.kbch();
        let bb = self.framer.frame_bytes(kbch / 8, 0, next);
        let mut info = vec![0u8; self.p.rate.k()];
        for (i, byte) in bb.iter().enumerate() {
            for b in 0..8 {
                info[i * 8 + b] = ((byte ^ self.bbscr[i]) >> (7 - b)) & 1;
            }
        }
        self.bch.encode(&mut info);
        crate::dvbs2::ldpc_fpga::encode(self.p.rate, &info)
    }

    /// One T2 frame of samples (appended to `out`).
    pub fn frame(&mut self, next: &mut dyn FnMut() -> [u8; crate::dvbs2::TS_LEN], out: &mut Vec<Complex32>) {
        let blocks: Vec<Vec<Cell>> = (0..self.p.fec_blocks).map(|_| cell_codes(&self.codeword(next))).collect();
        let data = self.ci.frame(&blocks);
        let cells = self.mapper.frame(&data);
        let syms = self.fi.frame(&cells);
        self.ofdm.frame(&syms, out);
    }
}

#[cfg(test)]
mod tests;
