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

/// The 1840 L1-pre cells.
pub fn pre(p: &Params) -> Vec<Cell> {
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
    b.put(0x3085, 16); // NETWORK_ID
    b.put(0x8001, 16); // T2_SYSTEM_ID
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
    let mut msg = b.0;
    msg.resize(KBCH_1_4, 0);
    let parity = bch_parity(&msg);
    let mut info = msg.clone();
    info.extend_from_slice(&parity);
    let ldpc = ldpc16k(&info, &LDPC_1_4S, 36);
    // Puncturing: 31 whole groups and 328 of the 32nd.
    let mut punct = vec![false; ldpc.len()];
    for (c, &g) in PRE_PUNCTURE.iter().enumerate().take(32) {
        let n = if c < 31 { 360 } else { 328 };
        for c2 in 0..n {
            punct[c2 * 36 + g] = true;
        }
    }
    let mut out = Vec::with_capacity(L1_PRE_CELLS);
    out.extend(msg[..KSIG_PRE].iter().map(|&x| bpsk(x)));
    out.extend(parity.iter().map(|&x| bpsk(x)));
    out.extend(ldpc.iter().zip(&punct).filter(|(_, pu)| !**pu).map(|(&x, _)| bpsk(x)));
    assert_eq!(out.len(), L1_PRE_CELLS);
    out
}

/// The L1-post cells of T2 frame `frame_idx`.
pub fn post(p: &Params, frame_idx: usize) -> Vec<Cell> {
    let mut b = Bits(Vec::with_capacity(KSIG_POST));
    // Configurable.
    b.put(1, 15); // SUB_SLICES_PER_FRAME
    b.put(1, 8); // NUM_PLP
    b.put(0, 4); // NUM_AUX
    b.put(0, 8); // AUX_CONFIG_RFU
    b.put(0, 3); // RF_IDX
    b.put(729_833_333, 32); // FREQUENCY
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
    let bits = b.0;
    assert_eq!(bits.len(), KSIG_POST);
    // Shortening: whole groups of 360 (group 19 has 192) padded, in the
    // standard's order, then `last` bits of the next.
    let mut pad = vec![false; KBCH_1_2];
    let (m, last) = if bits.len() <= 360 {
        (19, 360 - bits.len())
    } else {
        let m = (KBCH_1_2 - bits.len()) / 360;
        (m, KBCH_1_2 - bits.len() - 360 * m)
    };
    for &g in &POST_PADDING_BQPSK[..m] {
        let n = if g == 19 { 192 } else { 360 };
        pad[g * 360..g * 360 + n].fill(true);
    }
    let g = POST_PADDING_BQPSK[m];
    let start = if g == 19 { g * 360 + 192 - last } else { g * 360 + 360 - last };
    pad[start..start + last].fill(true);
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
    let (n_post, n_punc) = post_sizes();
    let mut punct = vec![false; ldpc.len()];
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
    let mut out_bits = Vec::with_capacity(n_post);
    out_bits.extend(msg.iter().zip(&pad).filter(|(_, pd)| !**pd).map(|(&x, _)| x));
    out_bits.extend_from_slice(&parity);
    out_bits.extend(ldpc.iter().zip(&punct).filter(|(_, pu)| !**pu).map(|(&x, _)| x));
    assert_eq!(out_bits.len(), n_post);
    out_bits.chunks_exact(2).map(|c| qpsk(c[0], c[1])).collect()
}
