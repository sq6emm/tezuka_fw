//! L1 signalling in the P2 symbols (EN 302 755 7.2), as gr-dtv's frame
//! mapper builds it: L1-pre (BPSK, 16K LDPC 1/4, shortened and punctured to
//! 1840 cells) and L1-post (configurable + dynamic, QPSK, 16K LDPC 1/2).

use super::{Cell, Params, BPSK0, L1_PRE_CELLS, L1_QPSK, N_P2};

const N_SHORT: usize = 16_200;
const KBCH_1_4: usize = 3072;
const NBCH_1_4: usize = 3240;
const KBCH_1_2: usize = 7032;
const NBCH_1_2: usize = 7200;
const KSIG_PRE: usize = 200;
const KSIG_POST: usize = 350;
const NBCH_PARITY: usize = 168;

const LDPC_1_4S: [&[u16]; 9] = [
    &[6295, 9626, 304, 7695, 4839, 4936, 1660, 144, 11203, 5567, 6347, 12557],
    &[10691, 4988, 3859, 3734, 3071, 3494, 7687, 10313, 5964, 8069, 8296, 11090],
    &[10774, 3613, 5208, 11177, 7676, 3549, 8746, 6583, 7239, 12265, 2674, 4292],
    &[11869, 3708, 5981, 8718, 4908, 10650, 6805, 3334, 2627, 10461, 9285, 11120],
    &[7844, 3079, 10773],
    &[3385, 10854, 5747],
    &[1360, 12010, 12202],
    &[6189, 4241, 2343],
    &[9840, 12726, 4977],
];

const LDPC_1_2S: [&[u16]; 20] = [
    &[20, 712, 2386, 6354, 4061, 1062, 5045, 5158],
    &[21, 2543, 5748, 4822, 2348, 3089, 6328, 5876],
    &[22, 926, 5701, 269, 3693, 2438, 3190, 3507],
    &[23, 2802, 4520, 3577, 5324, 1091, 4667, 4449],
    &[24, 5140, 2003, 1263, 4742, 6497, 1185, 6202],
    &[0, 4046, 6934],
    &[1, 2855, 66],
    &[2, 6694, 212],
    &[3, 3439, 1158],
    &[4, 3850, 4422],
    &[5, 5924, 290],
    &[6, 1467, 4049],
    &[7, 7820, 2242],
    &[8, 4606, 3080],
    &[9, 4633, 7877],
    &[10, 3884, 6868],
    &[11, 8935, 4996],
    &[12, 3028, 764],
    &[13, 5988, 1057],
    &[14, 7411, 3450],
];

const PRE_PUNCTURE: [usize; 36] =
    [27, 13, 29, 32, 5, 0, 11, 21, 33, 20, 25, 28, 18, 35, 8, 3, 9, 31, 22, 24, 7, 14, 17, 4, 2, 26, 16, 34, 19, 10, 12, 23, 1, 6, 30, 15];
const POST_PADDING_BQPSK: [usize; 20] = [18, 17, 16, 15, 14, 13, 12, 11, 4, 10, 9, 8, 3, 2, 7, 6, 5, 1, 19, 0];
const POST_PUNCTURE_BQPSK: [usize; 25] = [6, 4, 18, 9, 13, 8, 15, 20, 5, 17, 2, 24, 10, 22, 12, 3, 16, 23, 1, 14, 0, 21, 19, 7, 11];

struct Bits(Vec<u8>);

impl Bits {
    fn put(&mut self, v: u64, n: usize) {
        for i in (0..n).rev() {
            self.0.push(((v >> i) & 1) as u8);
        }
    }
    fn crc32(&mut self) {
        let mut crc: u32 = 0xffff_ffff;
        for &b in &self.0 {
            let x = b as u32 ^ (crc >> 31);
            crc <<= 1;
            if x != 0 {
                crc ^= 0x04C1_1DB7;
            }
        }
        self.put(crc as u64, 32);
    }
}

/// The short-frame BCH generator (12 polynomials of degree 14), bit i =
/// coefficient of x^i.
fn bch_short_generator() -> Vec<u8> {
    const POLYS: [u32; 12] =
        [0x402B, 0x4941, 0x4647, 0x5591, 0x6B55, 0x6389, 0x6CE5, 0x4F21, 0x460F, 0x5A49, 0x5811, 0x65EF];
    let mut g = vec![1u8];
    for p in POLYS {
        let mut out = vec![0u8; g.len() + 14];
        for (i, &gi) in g.iter().enumerate() {
            if gi != 0 {
                for d in 0..=14 {
                    out[i + d] ^= ((p >> d) & 1) as u8;
                }
            }
        }
        g = out;
    }
    g
}

/// Systematic BCH parity (168 bits) of `msg` (first bit = highest power).
fn bch_parity(msg: &[u8]) -> Vec<u8> {
    let g = bch_short_generator();
    let n = NBCH_PARITY;
    // Long division: remainder of msg(x) x^168 by g(x).
    let mut r = vec![0u8; n];
    for &b in msg {
        let fb = b ^ r[n - 1];
        for i in (1..n).rev() {
            r[i] = r[i - 1] ^ (fb & g[i]);
        }
        r[0] = fb & g[0];
    }
    r.iter().rev().copied().collect()
}

/// IRA LDPC parity of a 16K codeword (tables as gr-dtv: rows of
/// accumulator addresses, `q` the step per column).
fn ldpc16k(info: &[u8], table: &[&[u16]], q: usize) -> Vec<u8> {
    let pbits = N_SHORT - info.len();
    let mut p = vec![0u8; pbits];
    let mut im = 0;
    for row in table {
        for n in 0..360 {
            for &a in row.iter() {
                p[(a as usize + n * q) % pbits] ^= info[im];
            }
            im += 1;
        }
    }
    for j in 1..pbits {
        p[j] ^= p[j - 1];
    }
    p
}

fn bpsk(b: u8) -> Cell {
    BPSK0 + b as Cell
}

fn qpsk(b0: u8, b1: u8) -> Cell {
    L1_QPSK + ((b0 << 1) | b1) as Cell
}

/// L1-post length: (N_post bits, N_punc).
pub fn post_sizes() -> (usize, usize) {
    let eta = 2; // QPSK
    let n_punc_temp = (6 * (KBCH_1_2 - KSIG_POST)) / 5;
    let n_post_temp = KSIG_POST + NBCH_PARITY + 9000 - n_punc_temp;
    let n_post = n_post_temp.div_ceil(eta * N_P2) * eta * N_P2;
    (n_post, n_punc_temp - (n_post - n_post_temp))
}

/// L1-post cells (QPSK).
pub fn post_cells() -> usize {
    post_sizes().0 / 2
}

/// The 200 L1-pre signalling bits (CRC-32 included) for `p`.
pub fn pre_bits(p: &Params) -> Vec<u8> {
    let mut b = Bits(Vec::with_capacity(KBCH_1_4));
    b.put(0, 8); // TYPE: TS
    b.put(0, 1); // BWT_EXT
    b.put(0, 3); // S1: T2 SISO
    b.put(0, 3); // S2: 2K (fft_size & 7)
    b.put(0, 1);
    b.put(0, 1); // L1_REPETITION_FLAG
    b.put(p.guard as u64, 3);
    b.put(0, 4); // PAPR off
    b.put(1, 4); // L1_MOD: QPSK
    b.put(0, 2); // L1_COD
    b.put(0, 2); // L1_FEC_TYPE
    b.put((post_sizes().0 / 2) as u64, 18); // L1_POST_SIZE
    b.put((KSIG_POST - 32) as u64, 18); // L1_POST_INFO_SIZE
    b.put(p.pilots as u64, 4);
    b.put(0, 8); // TX_ID_AVAILABILITY
    b.put(0, 16); // CELL_ID
    b.put(p.network_id as u64, 16); // NETWORK_ID
    b.put(p.t2_system_id as u64, 16); // T2_SYSTEM_ID
    b.put(p.t2_frames as u64, 8);
    b.put(p.data_symbols as u64, 12);
    b.put(0, 3); // REGEN_FLAG
    b.put(0, 1); // L1_POST_EXTENSION
    b.put(1, 3); // NUM_RF
    b.put(0, 3); // CURRENT_RF_IDX
    b.put(0, 4); // T2_VERSION 1.1.1
    b.put(0, 1); // L1_POST_SCRAMBLED
    b.put(0, 1); // T2_BASE_LITE
    b.put(0, 4); // RESERVED
    b.crc32();
    assert_eq!(b.0.len(), KSIG_PRE);
    b.0
}

/// L1-pre puncturing of the LDPC parity bits: 31 whole groups and 328 of
/// the 32nd.
fn pre_punctured() -> Vec<bool> {
    let mut punct = vec![false; N_SHORT - NBCH_1_4];
    for (c, &g) in PRE_PUNCTURE.iter().enumerate().take(32) {
        let n = if c < 31 { 360 } else { 328 };
        for c2 in 0..n {
            punct[c2 * 36 + g] = true;
        }
    }
    punct
}

/// The 1840 L1-pre cells.
pub fn pre(p: &Params) -> Vec<Cell> {
    let mut msg = pre_bits(p);
    msg.resize(KBCH_1_4, 0);
    let parity = bch_parity(&msg);
    let mut info = msg.clone();
    info.extend_from_slice(&parity);
    let ldpc = ldpc16k(&info, &LDPC_1_4S, 36);
    let punct = pre_punctured();
    let mut out = Vec::with_capacity(L1_PRE_CELLS);
    out.extend(msg[..KSIG_PRE].iter().map(|&x| bpsk(x)));
    out.extend(parity.iter().map(|&x| bpsk(x)));
    out.extend(ldpc.iter().zip(&punct).filter(|(_, pu)| !**pu).map(|(&x, _)| bpsk(x)));
    assert_eq!(out.len(), L1_PRE_CELLS);
    out
}

/// The 350 L1-post bits (configurable, dynamic, CRC-32) of T2 frame
/// `frame_idx`.
pub fn post_bits(p: &Params, frame_idx: usize) -> Vec<u8> {
    let mut b = Bits(Vec::with_capacity(KSIG_POST));
    // Configurable.
    b.put(1, 15); // SUB_SLICES_PER_FRAME
    b.put(1, 8); // NUM_PLP
    b.put(0, 4); // NUM_AUX
    b.put(0, 8); // AUX_CONFIG_RFU
    b.put(0, 3); // RF_IDX
    b.put(p.frequency_hz as u64, 32); // FREQUENCY
    b.put(0, 8); // PLP_ID
    b.put(1, 3); // PLP_TYPE: data type 1
    b.put(3, 5); // PLP_PAYLOAD_TYPE: TS
    b.put(0, 1); // FF_FLAG
    b.put(0, 3); // FIRST_RF_IDX
    b.put(0, 8); // FIRST_FRAME_IDX
    b.put(1, 8); // PLP_GROUP_ID
    b.put(p.rate_code() as u64, 3);
    b.put(p.constellation.code() as u64, 3);
    b.put(p.rotation as u64, 1);
    b.put(1, 2); // PLP_FEC_TYPE: 64K
    b.put(p.fec_blocks as u64, 10); // PLP_NUM_BLOCKS_MAX
    b.put(1, 8); // FRAME_INTERVAL
    b.put(1, 8); // TIME_IL_LENGTH
    b.put(0, 1); // TIME_IL_TYPE
    b.put(0, 1); // IN_BAND_A_FLAG
    b.put(0, 1); // IN_BAND_B_FLAG
    b.put(0, 11); // RESERVED_1
    b.put(0, 2); // PLP_MODE (1.1.1)
    b.put(0, 1); // STATIC_FLAG
    b.put(0, 1); // STATIC_PADDING_FLAG
    b.put(0, 2); // FEF_LENGTH_MSB
    b.put(0, 30); // RESERVED_2
    // Dynamic.
    b.put(frame_idx as u64, 8);
    b.put(0, 22); // SUB_SLICE_INTERVAL
    b.put(0, 22); // TYPE_2_START
    b.put(0, 8); // L1_CHANGE_COUNTER
    b.put(0, 3); // START_RF_IDX
    b.put(0, 8); // RESERVED_1
    b.put(0, 8); // PLP_ID
    b.put(0, 22); // PLP_START
    b.put(p.fec_blocks as u64, 10); // PLP_NUM_BLOCKS
    b.put(0, 8); // RESERVED_2
    b.put(0, 8); // RESERVED_3
    b.crc32();
    assert_eq!(b.0.len(), KSIG_POST);
    b.0
}

/// L1-post shortening: which of the 7032 BCH information bits are padding.
fn post_padded() -> Vec<bool> {
    // Shortening: whole groups of 360 (group 19 has 192) padded, in the
    // standard's order, then `last` bits of the next.
    let mut pad = vec![false; KBCH_1_2];
    let (m, last) = if KSIG_POST <= 360 {
        (19, 360 - KSIG_POST)
    } else {
        let m = (KBCH_1_2 - KSIG_POST) / 360;
        (m, KBCH_1_2 - KSIG_POST - 360 * m)
    };
    for &g in &POST_PADDING_BQPSK[..m] {
        let n = if g == 19 { 192 } else { 360 };
        pad[g * 360..g * 360 + n].fill(true);
    }
    let g = POST_PADDING_BQPSK[m];
    let start = if g == 19 { g * 360 + 192 - last } else { g * 360 + 360 - last };
    pad[start..start + last].fill(true);
    pad
}

/// L1-post puncturing of the 9000 LDPC parity bits.
fn post_punctured() -> Vec<bool> {
    let (_, n_punc) = post_sizes();
    let mut punct = vec![false; N_SHORT - NBCH_1_2];
    for c in 0..n_punc / 360 {
        let g = POST_PUNCTURE_BQPSK[c];
        for c2 in 0..360 {
            punct[c2 * 25 + g] = true;
        }
    }
    let g = POST_PUNCTURE_BQPSK[n_punc / 360];
    for c2 in 0..n_punc % 360 {
        punct[c2 * 25 + g] = true;
    }
    punct
}

/// The L1-post cells of T2 frame `frame_idx`.
pub fn post(p: &Params, frame_idx: usize) -> Vec<Cell> {
    let bits = post_bits(p, frame_idx);
    let pad = post_padded();
    let mut msg = vec![0u8; KBCH_1_2];
    let mut it = bits.iter();
    for (n, m) in msg.iter_mut().enumerate() {
        if !pad[n] {
            *m = *it.next().unwrap();
        }
    }
    let parity = bch_parity(&msg);
    let mut info = msg.clone();
    info.extend_from_slice(&parity);
    let ldpc = ldpc16k(&info, &LDPC_1_2S, 25);
    let (n_post, _) = post_sizes();
    let punct = post_punctured();
    let mut out_bits = Vec::with_capacity(n_post);
    out_bits.extend(msg.iter().zip(&pad).filter(|(_, pd)| !**pd).map(|(&x, _)| x));
    out_bits.extend_from_slice(&parity);
    out_bits.extend(ldpc.iter().zip(&punct).filter(|(_, pu)| !**pu).map(|(&x, _)| x));
    assert_eq!(out_bits.len(), n_post);
    out_bits.chunks_exact(2).map(|c| qpsk(c[0], c[1])).collect()
}

/// What the receiver makes of a frame's L1-pre.
#[derive(Debug, Clone, PartialEq)]
pub enum PreOutcome {
    /// Decoded (CRC right), as expected for these parameters.
    Ok,
    /// Decoded, but another configuration: the differing fields.
    Mismatch(Vec<&'static str>),
    /// Not decoded (BCH or CRC failed).
    Failed,
}

/// The L1-pre fields the receiver's fixed layout depends on, (name, first
/// bit, width). (NETWORK_ID, T2_SYSTEM_ID and the like are another
/// station's own: not a reason to refuse it.)
const PRE_FIELDS: [(&str, usize, usize); 10] = [
    ("TYPE", 0, 8),
    ("S1", 9, 3),
    ("S2", 12, 4),
    ("GUARD_INTERVAL", 17, 3),
    ("PAPR", 20, 4),
    ("L1_MOD/COD/FEC", 24, 8),
    ("L1_POST_SIZE", 32, 18),
    ("PILOT_PATTERN", 68, 4),
    ("NUM_DATA_SYMBOLS", 136, 12),
    ("T2_VERSION", 158, 4),
];

/// The same for L1-post's configurable part (one PLP, its modulation,
/// code, rotation, blocks, time interleaving).
const POST_FIELDS: [(&str, usize, usize); 11] = [
    ("NUM_PLP", 15, 8),
    ("NUM_AUX", 23, 4),
    ("PLP_TYPE", 78, 3),
    ("PLP_PAYLOAD_TYPE", 81, 5),
    ("PLP_COD", 106, 3),
    ("PLP_MOD", 109, 3),
    ("PLP_ROTATION", 112, 1),
    ("PLP_FEC_TYPE", 113, 2),
    ("PLP_NUM_BLOCKS_MAX", 115, 10),
    ("FRAME_INTERVAL/TIME_IL_LENGTH", 125, 16),
    ("TIME_IL_TYPE", 141, 1),
];

/// Decodes an L1 block (EN 302 755 7.3): BCH on the hard decisions first
/// (enough at any usable MER), the 16K LDPC (the DVB-S2 short code of the
/// same rate) when that fails; then the CRC-32, and the fields the
/// receiver's layout depends on against what `p` sends.
struct L1Decoder {
    want: Vec<u8>,
    fields: &'static [(&'static str, usize, usize)],
    ksig: usize,
    kbch: usize,
    /// Shortened information bits (known zeros) and punctured parity bits.
    pad: Vec<bool>,
    punct: Vec<bool>,
    ldpc: crate::dvbs2::ldpc::Decoder,
    bch: crate::dvbs2::bch::Bch,
    llr: Vec<f32>,
    bits: Vec<u8>,
}

impl L1Decoder {
    /// `llr`: the block's bits as sent (positive = 0).
    fn decode(&mut self, llr: &[f32]) -> PreOutcome {
        let nbch = self.kbch + NBCH_PARITY;
        let big = 64.0;
        let mut it = llr.iter();
        let mut next = || *it.next().unwrap_or(&0.0);
        for n in 0..self.kbch {
            self.llr[n] = if self.pad[n] { big } else { next() };
        }
        for n in self.kbch..nbch {
            self.llr[n] = next();
        }
        for (j, &pu) in self.punct.iter().enumerate() {
            self.llr[nbch + j] = if pu { 0.0 } else { next() };
        }
        for (b, &l) in self.bits.iter_mut().zip(&self.llr) {
            *b = (l < 0.0) as u8;
        }
        if !self.check() {
            let llr = std::mem::take(&mut self.llr);
            let conv = self.ldpc.decode(&llr, &mut self.bits);
            self.llr = llr;
            if conv.is_none() || !self.check() {
                return PreOutcome::Failed;
            }
        }
        let sig: Vec<u8> = self.bits[..self.kbch].iter().zip(&self.pad).filter(|(_, p)| !**p).map(|(&b, _)| b).take(self.ksig).collect();
        let differ: Vec<&'static str> = self.fields.iter().filter(|(_, at, n)| sig[*at..at + n] != self.want[*at..at + n]).map(|(name, ..)| *name).collect();
        if differ.is_empty() { PreOutcome::Ok } else { PreOutcome::Mismatch(differ) }
    }

    /// BCH corrects (or finds clean), and the signalling's CRC-32 holds.
    fn check(&mut self) -> bool {
        let nbch = self.kbch + NBCH_PARITY;
        if matches!(self.bch.decode(&mut self.bits[..nbch]), crate::dvbs2::bch::Outcome::Failed) {
            return false;
        }
        let sig: Vec<u8> = self.bits[..self.kbch].iter().zip(&self.pad).filter(|(_, p)| !**p).map(|(&b, _)| b).take(self.ksig).collect();
        let mut b = Bits(sig[..self.ksig - 32].to_vec());
        b.crc32();
        b.0[self.ksig - 32..] == sig[self.ksig - 32..]
    }
}

/// L1-pre from its 1840 BPSK cells' LLRs.
pub struct PreDecoder(L1Decoder);

impl PreDecoder {
    pub fn new(p: &Params) -> Self {
        let mut pad = vec![false; KBCH_1_4];
        pad[KSIG_PRE..].fill(true);
        PreDecoder(L1Decoder {
            want: pre_bits(p),
            fields: &PRE_FIELDS,
            ksig: KSIG_PRE,
            kbch: KBCH_1_4,
            pad,
            punct: pre_punctured(),
            ldpc: crate::dvbs2::ldpc::Decoder::from_table(&LDPC_1_4S, N_SHORT, NBCH_1_4),
            bch: crate::dvbs2::bch::Bch::short(),
            llr: vec![0.0; N_SHORT],
            bits: vec![0; N_SHORT],
        })
    }

    /// `llr`: the 1840 cells' LLRs in order (positive = 0).
    pub fn decode(&mut self, llr: &[f32]) -> PreOutcome {
        assert_eq!(llr.len(), super::L1_PRE_CELLS);
        self.0.decode(llr)
    }
}

/// L1-post from its QPSK cells' bit LLRs (two a cell: I, then Q).
pub struct PostDecoder(L1Decoder);

impl PostDecoder {
    pub fn new(p: &Params) -> Self {
        PostDecoder(L1Decoder {
            want: post_bits(p, 0),
            fields: &POST_FIELDS,
            ksig: KSIG_POST,
            kbch: KBCH_1_2,
            pad: post_padded(),
            punct: post_punctured(),
            ldpc: crate::dvbs2::ldpc::Decoder::from_table(&LDPC_1_2S, N_SHORT, NBCH_1_2),
            bch: crate::dvbs2::bch::Bch::short(),
            llr: vec![0.0; N_SHORT],
            bits: vec![0; N_SHORT],
        })
    }

    pub fn decode(&mut self, llr: &[f32]) -> PreOutcome {
        assert_eq!(llr.len(), post_sizes().0);
        self.0.decode(llr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn llrs(cells: &[Cell], flip: &[usize], noise: f32) -> Vec<f32> {
        let mut seed = 9u64;
        cells
            .iter()
            .enumerate()
            .map(|(i, &c)| {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                let n = ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 2.0 * noise;
                let v = if c == super::super::BPSK0 { 1.0 } else { -1.0 };
                4.0 * (if flip.contains(&i) { -v } else { v } + n)
            })
            .collect()
    }

    #[test]
    fn pre_decodes_and_tells_a_different_configuration() {
        let p = Params::amateur();
        let mut d = PreDecoder::new(&p);
        let cells = pre(&p);
        assert_eq!(d.decode(&llrs(&cells, &[], 0.0)), PreOutcome::Ok);
        // hard-decision errors beyond BCH's 12: the LDPC puts them right
        let flips: Vec<usize> = (0..40).map(|i| i * 41 + 3).collect();
        assert_eq!(d.decode(&llrs(&cells, &flips, 0.3)), PreOutcome::Ok);
        // another transmitter: PP4 (its own T2_SYSTEM_ID is no matter)
        let mut q = p;
        q.pilots = super::super::Pilots::PP4;
        q.t2_system_id = 7;
        assert_eq!(d.decode(&llrs(&pre(&q), &[], 0.0)), PreOutcome::Mismatch(vec!["PILOT_PATTERN"]));
        q.pilots = p.pilots;
        assert_eq!(d.decode(&llrs(&pre(&q), &[], 0.0)), PreOutcome::Ok);
        // noise only
        let noise: Vec<f32> = llrs(&cells, &[], 0.0).iter().enumerate().map(|(i, _)| if (i * 7919) % 3 == 0 { 1.0 } else { -1.0 }).collect();
        assert_eq!(d.decode(&noise), PreOutcome::Failed);
    }

    /// L1-post from its QPSK cells: any frame index; another modulation,
    /// rotation or the real FREQUENCY told apart from layout changes.
    #[test]
    fn post_decodes_and_tells_a_different_configuration() {
        let p = Params::amateur();
        let mut d = PostDecoder::new(&p);
        let bits = |q: &Params, f: usize, noise: f32| -> Vec<f32> {
            let mut seed = 5u64;
            post(q, f)
                .iter()
                .flat_map(|&c| {
                    let w = c - L1_QPSK;
                    [(w >> 1) & 1, w & 1]
                })
                .map(|b| {
                    seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                    let n = ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 2.0 * noise;
                    4.0 * (if b == 0 { 1.0 } else { -1.0 } + n)
                })
                .collect()
        };
        assert_eq!(d.decode(&bits(&p, 0, 0.0)), PreOutcome::Ok);
        assert_eq!(d.decode(&bits(&p, 1, 0.9)), PreOutcome::Ok);
        let mut q = p;
        q.frequency_hz = 437_000_000;
        assert_eq!(d.decode(&bits(&q, 0, 0.0)), PreOutcome::Ok);
        q.constellation = super::super::Constellation::Qam16;
        q.rotation = !p.rotation;
        assert_eq!(d.decode(&bits(&q, 0, 0.0)), PreOutcome::Mismatch(vec!["PLP_MOD", "PLP_ROTATION"]));
    }
}
