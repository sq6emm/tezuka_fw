//! Reference model of the FPGA LDPC decoder for DVB-S2 normal frames
//! (64800 bits; rates 1/2 and 3/4 here), bit-exact with the HDL to come.
//!
//! Layered normalized min-sum in fixed point, one edge per clock:
//!
//! * Checks are taken in DVB-S2's natural groups: j, j + q, j + 2q, ...
//!   (j = 0..q). Checks of a group share no variable, so the hardware
//!   pipelines through a group without hazards; between groups the parity
//!   chain (check m holds p(m-1) and p(m)) needs the previous group done.
//! * A check's info edges come from a per-group list of (table row g,
//!   shift s): variable g * 360 + ((k - s) mod 360) for check j + k q.
//! * Per check the state is compressed: min1, min2 (magnitudes after
//!   normalization), the edge index of min1 and one sign bit per edge.
//! * Normalization 15/16, P 8 bits, messages 5 bits, up to 50 iterations:
//!   about 0.1-0.2 dB from float BP (tests fx_sweep, fer_long).
//! * Pass 1, per edge: R = stored message, Q = sat(P - R); track the two
//!   smallest |Q| and the sign product. Pass 2, per edge: R' = normalized
//!   min (min2 on the min1 edge) with the sign product of the other edges;
//!   P = sat(Q + R'). The check's hard-decision parity after pass 2 feeds
//!   the early stop (a whole iteration with every check satisfied).

use super::tables;

pub const N: usize = 64_800;

/// Bit widths (the HDL takes the same).
pub const LLR_BITS: u32 = 6;
pub const P_BITS: u32 = 8;
pub const M_BITS: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LongRate {
    R1_2,
    R3_4,
}

impl LongRate {
    pub fn k(self) -> usize {
        match self {
            LongRate::R1_2 => 32_400,
            LongRate::R3_4 => 48_600,
        }
    }
    pub fn table(self) -> &'static [&'static [u16]] {
        match self {
            LongRate::R1_2 => tables::NF_1_2,
            LongRate::R3_4 => tables::NF_3_4,
        }
    }
    pub fn q(self) -> usize {
        (N - self.k()) / 360
    }
}

/// Systematic IRA encoding: the parity bits of `info` (k bits, 0/1).
pub fn encode(rate: LongRate, info: &[u8]) -> Vec<u8> {
    let (k, q) = (rate.k(), rate.q());
    let nk = N - k;
    let mut p = vec![0u8; nk];
    // (a + m q) mod (N - K) by a running index: a + 359 q < 2 (N - K), so
    // one subtraction wraps it (no division: slow on the A9).
    for (g, addrs) in rate.table().iter().enumerate() {
        let bits = &info[g * 360..g * 360 + 360];
        for &a in addrs.iter() {
            let mut x = a as usize;
            for &b in bits {
                p[x] ^= b;
                x += q;
                if x >= nk {
                    x -= nk;
                }
            }
        }
    }
    for i in 1..nk {
        p[i] ^= p[i - 1];
    }
    let mut cw = info.to_vec();
    cw.extend(p);
    cw
}

/// The per-group edge lists: for group j, (row g, shift s) pairs.
pub fn group_edges(rate: LongRate) -> Vec<Vec<(u16, u16)>> {
    let q = rate.q();
    let mut groups = vec![Vec::new(); q];
    for (g, addrs) in rate.table().iter().enumerate() {
        for &a in addrs.iter() {
            let a = a as usize;
            groups[a % q].push((g as u16, (a / q) as u16));
        }
    }
    groups
}

/// Fixed-point choices (the HDL's generics).
#[derive(Debug, Clone, Copy)]
pub struct Fixed {
    pub p_bits: u32,
    pub m_bits: u32,
    /// Normalization factor numerator over 16 (12 = 0.75).
    pub norm16: i32,
}

impl Default for Fixed {
    fn default() -> Self {
        // Swept against float BP on long frames (fx_sweep): 15/16 with up
        // to 50 iterations loses about 0.1-0.2 dB; wider words gain nothing.
        Fixed { p_bits: P_BITS, m_bits: M_BITS, norm16: 15 }
    }
}

fn sat(x: i32, bits: u32) -> i32 {
    let m = (1 << (bits - 1)) - 1;
    x.clamp(-m, m)
}

/// Normalization of a min-sum magnitude: x * norm16 / 16, rounded down.
fn norm(x: i32, norm16: i32) -> i32 {
    (x * norm16) >> 4
}

#[derive(Clone, Copy, Default)]
struct CheckState {
    min1: u8,
    min2: u8,
    idx: u8,
    signs: u32,
}

pub struct FpgaDecoder {
    pub rate: LongRate,
    groups: Vec<Vec<(u16, u16)>>,
    state: Vec<CheckState>,
    post: Vec<i32>,
    pub max_iter: usize,
    pub fx: Fixed,
}

impl FpgaDecoder {
    pub fn new(rate: LongRate) -> Self {
        FpgaDecoder { rate, groups: group_edges(rate), state: vec![CheckState::default(); N - rate.k()], post: vec![0; N], max_iter: 50, fx: Fixed::default() }
    }

    /// Variables of check j + k q, in the order the hardware visits them:
    /// info edges (group list order), then p(m - 1) (if m > 0), then p(m).
    fn check_vars(&self, j: usize, kk: usize, out: &mut Vec<usize>) {
        out.clear();
        let (k, q) = (self.rate.k(), self.rate.q());
        for &(g, s) in &self.groups[j] {
            out.push(g as usize * 360 + (kk + 360 - s as usize) % 360);
        }
        let m = j + kk * q;
        if m > 0 {
            out.push(k + m - 1);
        }
        out.push(k + m);
    }

    /// The posteriors after the last decode (the HDL test compares them all).
    pub fn posteriors(&self) -> &[i32] {
        &self.post
    }

    /// Decode channel LLRs (`LLR_BITS`-bit, positive = bit 0) into hard
    /// bits. Returns the iterations used when every check is satisfied.
    pub fn decode(&mut self, llr: &[i8], bits: &mut [u8]) -> Option<usize> {
        assert_eq!(llr.len(), N);
        for (p, &l) in self.post.iter_mut().zip(llr) {
            *p = sat(l as i32, LLR_BITS);
        }
        self.state.iter_mut().for_each(|s| *s = CheckState::default());
        let fx = self.fx;
        let (q, mmax) = (self.rate.q(), (1 << fx.m_bits) - 1);
        let mut vars = Vec::with_capacity(16);
        let mut qv = Vec::with_capacity(16);
        let mut result = None;
        for it in 1..=self.max_iter {
            let mut unsatisfied = 0;
            for j in 0..q {
                for kk in 0..360 {
                    self.check_vars(j, kk, &mut vars);
                    let m = j + kk * q;
                    let st = self.state[m];
                    // Pass 1.
                    let (mut m1, mut m2, mut idx, mut sp) = (mmax, mmax, 0u8, 0u32);
                    qv.clear();
                    for (e, &v) in vars.iter().enumerate() {
                        let mag = if e as u8 == st.idx { st.min2 } else { st.min1 } as i32;
                        let r = if (st.signs >> e) & 1 == 1 { -mag } else { mag };
                        let qq = sat(self.post[v] - r, fx.p_bits);
                        qv.push(qq);
                        let a = qq.abs().min(mmax);
                        if a < m1 {
                            m2 = m1;
                            m1 = a;
                            idx = e as u8;
                        } else if a < m2 {
                            m2 = a;
                        }
                        sp ^= (qq < 0) as u32;
                    }
                    let (n1, n2) = (norm(m1, fx.norm16), norm(m2, fx.norm16));
                    // Pass 2.
                    let mut signs = 0u32;
                    let mut parity = 0u32;
                    for (e, &v) in vars.iter().enumerate() {
                        let sq = (qv[e] < 0) as u32;
                        let sr = sp ^ sq;
                        let mag = if e as u8 == idx { n2 } else { n1 };
                        let r = if sr == 1 { -mag } else { mag };
                        let p = sat(qv[e] + r, fx.p_bits);
                        self.post[v] = p;
                        signs |= sr << e;
                        parity ^= (p < 0) as u32;
                    }
                    self.state[m] = CheckState { min1: n1 as u8, min2: n2 as u8, idx, signs };
                    unsatisfied += parity as usize;
                }
            }
            if unsatisfied == 0 {
                result = Some(it);
                break;
            }
        }
        for (b, &p) in bits.iter_mut().zip(&self.post) {
            *b = (p < 0) as u8;
        }
        result
    }
}

/// Channel LLR (float, positive = 0) to the decoder's input.
pub fn quantize_llr(l: f32, scale: f32) -> i8 {
    let m = ((1 << (LLR_BITS - 1)) - 1) as f32;
    (l * scale).round().clamp(-m, m) as i8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rng(seed: &mut u64) -> u64 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        *seed
    }
    fn gauss(seed: &mut u64) -> f32 {
        let u = ((rng(seed) >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
        let v = (rng(seed) >> 11) as f64 / (1u64 << 53) as f64;
        ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
    }

    /// A codeword over QPSK at Es/N0: channel LLRs (float).
    fn channel(rate: LongRate, esn0_db: f32, seed: &mut u64) -> (Vec<u8>, Vec<f32>) {
        let info: Vec<u8> = (0..rate.k()).map(|_| (rng(seed) & 1) as u8).collect();
        let cw = encode(rate, &info);
        // QPSK: each bit on one axis at amplitude 1/sqrt2; noise per axis N0/2.
        let n0 = 10f32.powf(-esn0_db / 10.0);
        let sigma = (n0 / 2.0).sqrt();
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let llr = cw.iter().map(|&b| {
            let y = if b == 0 { a } else { -a } + sigma * gauss(seed);
            2.0 * a * y / (sigma * sigma)
        }).collect();
        (cw, llr)
    }

    #[test]
    fn encoder_output_satisfies_every_check() {
        for rate in [LongRate::R1_2, LongRate::R3_4] {
            let mut seed = 9;
            let info: Vec<u8> = (0..rate.k()).map(|_| (rng(&mut seed) & 1) as u8).collect();
            let cw = encode(rate, &info);
            // The float decoder's parity structure agrees: zero iterations needed.
            let mut dec = super::super::ldpc::Decoder::from_table(rate.table(), N, rate.k());
            let llr: Vec<f32> = cw.iter().map(|&b| if b == 0 { 8.0 } else { -8.0 }).collect();
            let mut bits = vec![0u8; N];
            assert_eq!(dec.decode(&llr, &mut bits), Some(1), "{rate:?}");
            assert_eq!(bits, cw);
        }
    }

    #[test]
    fn fixed_point_decoder_corrects_noise() {
        for (rate, esn0) in [(LongRate::R1_2, 2.0f32), (LongRate::R3_4, 5.0)] {
            let mut seed = 21;
            let (cw, llr) = channel(rate, esn0, &mut seed);
            let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, 2.0)).collect();
            let hard_errors = q.iter().zip(&cw).filter(|(l, b)| (**l < 0) as u8 != **b).count();
            let mut dec = FpgaDecoder::new(rate);
            let mut bits = vec![0u8; N];
            let it = dec.decode(&q, &mut bits);
            assert!(hard_errors > 100, "{rate:?}: the channel should need correcting");
            assert!(it.is_some() && bits == cw, "{rate:?} at {esn0} dB: {it:?}, {hard_errors} channel errors");
        }
    }

    /// Vectors for maia-hdl's test_ldpc_dec.py: `LDPC_VECTORS=<file> cargo
    /// test ldpc_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn ldpc_vectors() {
        let path = std::env::var("LDPC_VECTORS").expect("LDPC_VECTORS=<file>");
        let mut cases = Vec::new();
        // (rate, Es/N0, max iterations): one stopped early at a low SNR (every
        // posterior compared after 2 iterations), two decoding.
        for (rate, esn0, max_iter) in [(LongRate::R1_2, 0.5f32, 2usize), (LongRate::R3_4, 6.0, 50), (LongRate::R1_2, 3.0, 50)] {
            let mut seed = 101;
            let (cw, llr) = channel(rate, esn0, &mut seed);
            let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, 2.0)).collect();
            let mut dec = FpgaDecoder::new(rate);
            dec.max_iter = max_iter;
            let mut bits = vec![0u8; N];
            let r = dec.decode(&q, &mut bits);
            eprintln!("{rate:?} {esn0} dB max {max_iter}: {r:?}, correct {}", bits == cw);
            let it = r.unwrap_or(max_iter);
            cases.push(serde_json::json!({
                "rate": if rate == LongRate::R1_2 { 0 } else { 1 },
                "max_iter": max_iter, "iterations": it, "converged": r.is_some(),
                "llr": q, "post": dec.posteriors(),
            }));
        }
        std::fs::write(path, serde_json::to_vec(&cases).unwrap()).unwrap();
    }

    /// Fixed-point choices near the thresholds: `-- --ignored fx_sweep --nocapture`.
    #[test]
    #[ignore]
    fn fx_sweep() {
        let frames = 30;
        let variants: Vec<(u32, u32, i32, usize)> = vec![
            (8, 5, 14, 30), (8, 5, 15, 30), (8, 5, 14, 50), (8, 6, 14, 30), (8, 6, 15, 30), (9, 6, 15, 50),
        ];
        for (rate, esn0, scale) in [(LongRate::R1_2, 0.9f32, 2.0f32), (LongRate::R1_2, 1.0, 2.0), (LongRate::R3_4, 3.9, 2.0), (LongRate::R3_4, 4.0, 2.0)] {
            let mut seed = 5 + (esn0 * 10.0) as u64;
            let chans: Vec<_> = (0..frames).map(|_| channel(rate, esn0, &mut seed)).collect();
            let mut line = format!("{rate:?} {esn0:.1} dB:");
            for &(p, m, n, it) in &variants {
                let mut bad = 0;
                let mut its = 0;
                for (cw, llr) in &chans {
                    // Inputs scaled to the message width: LLR_BITS stays 6.
                    let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, scale)).collect();
                    let mut dec = FpgaDecoder::new(rate);
                    dec.fx = Fixed { p_bits: p, m_bits: m, norm16: n };
                    dec.max_iter = it;
                    let mut bits = vec![0u8; N];
                    match dec.decode(&q, &mut bits) {
                        Some(i) if &bits == cw => its += i,
                        _ => bad += 1,
                    }
                }
                line += &format!("  P{p}M{m}n{n}i{it}:{bad}/{frames}({:.0})", its as f64 / (frames - bad).max(1) as f64);
            }
            eprintln!("{line}");
        }
    }

    /// Frame error rate, float BP against the fixed-point model, near the
    /// thresholds: `-- --ignored fer_long --nocapture`.
    #[test]
    #[ignore]
    fn fer_long() {
        let frames = 40;
        for (rate, points) in [(LongRate::R1_2, [0.8f32, 1.0, 1.2, 1.5]), (LongRate::R3_4, [3.8, 4.0, 4.3, 4.6])] {
            for esn0 in points {
                let mut seed = 77 + (esn0 * 10.0) as u64;
                let (mut bp_bad, mut fx_bad) = ([0usize; 1], [0usize; 3]);
                let (mut fx_it, mut fx_ok) = (0usize, 0usize);
                let scales = [1.0f32, 2.0, 4.0];
                for _ in 0..frames {
                    let (cw, llr) = channel(rate, esn0, &mut seed);
                    let mut bits = vec![0u8; N];
                    let mut bp = super::super::ldpc::Decoder::from_table(rate.table(), N, rate.k());
                    bp.max_iter = 50;
                    if bp.decode(&llr, &mut bits).is_none() || bits != cw {
                        bp_bad[0] += 1;
                    }
                    for (si, &sc) in scales.iter().enumerate() {
                        let q: Vec<i8> = llr.iter().map(|&l| quantize_llr(l, sc)).collect();
                        let mut dec = FpgaDecoder::new(rate);
                        let r = dec.decode(&q, &mut bits);
                        if r.is_none() || bits != cw {
                            fx_bad[si] += 1;
                        } else if si == 1 {
                            fx_it += r.unwrap();
                            fx_ok += 1;
                        }
                    }
                }
                eprintln!(
                    "{rate:?} Es/N0 {esn0:.1} dB: BP {}/{frames} bad; fixed (LLR{LLR_BITS} P{P_BITS} M{M_BITS}) scale 1/2/4: {:?} bad, mean {:.1} it",
                    bp_bad[0], fx_bad, fx_it as f64 / fx_ok.max(1) as f64
                );
            }
        }
    }
}
