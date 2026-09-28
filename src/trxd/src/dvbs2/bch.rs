//! BCH decoding (EN 302 307-1 5.3.1): t = 12, normal frames over GF(2^16)
//! (192 parity bits), short frames over GF(2^14) (168). After the LDPC
//! decoder it corrects the few bits a (falsely) converged or nearly
//! converged LDPC decode leaves wrong.
//!
//! The codeword's first bit is the coefficient of x^(n-1); the parity is the
//! last bits. A short code's generator sits 24 bits up in the 192-bit
//! register (g x^24): the remainder of c x^24, shifted back down. Fast path: the remainder mod g(x) (a 192-bit LFSR, byte at
//! a time); only when it is non-zero the syndromes, Berlekamp-Massey and a
//! Chien search over the n positions.

const T: usize = 12;

/// Table 6a: g1 .. g12 of normal frames (g1 is also the field's primitive
/// polynomial).
const POLYS16: [u32; T] =
    [0x1002D, 0x10173, 0x10FBD, 0x15A55, 0x11F2F, 0x1F7B5, 0x1AF65, 0x17367, 0x10EA1, 0x175A7, 0x13A2D, 0x11AE3];
/// Table 6b: short frames.
const POLYS14: [u32; T] = [0x402B, 0x4941, 0x4647, 0x5591, 0x6B55, 0x6389, 0x6CE5, 0x4F21, 0x460F, 0x5A49, 0x5811, 0x65EF];

/// 192-bit register, bit i = coefficient of x^i.
type Reg = [u64; 3];

pub struct Bch {
    /// Field GF(2^m), q = 2^m - 1; parity bits m t; the generator's shift
    /// up in the register (192 - parity).
    m: usize,
    q: usize,
    parity: usize,
    shift: usize,
    exp: Vec<u16>,
    log: Vec<u16>,
    /// g(x) without its x^192 term.
    g: Reg,
    /// Remainder update for the top byte: (b(x) x^192) mod g(x) for each b.
    byte_tab: Vec<Reg>,
}

pub enum Outcome {
    Clean,
    Fixed(usize),
    Failed,
}

impl Bch {
    /// Normal frames.
    pub fn new() -> Bch {
        Bch::with(16, &POLYS16)
    }

    /// Short frames.
    pub fn short() -> Bch {
        Bch::with(14, &POLYS14)
    }

    /// Parity bits (the codeword is Kbch + this).
    pub fn parity(&self) -> usize {
        self.parity
    }

    fn with(m: usize, polys: &[u32; T]) -> Bch {
        let q = (1usize << m) - 1;
        let parity = m * T;
        let shift = 192 - parity;
        let mut exp = vec![0u16; 2 * q];
        let mut log = vec![0u16; q + 1];
        let mut x: u32 = 1;
        for i in 0..q {
            exp[i] = x as u16;
            exp[i + q] = x as u16;
            log[x as usize] = i as u16;
            x <<= 1;
            if x >> m != 0 {
                x ^= polys[0];
            }
        }
        // g = product of the 12 polynomials (degree m t).
        let mut gp = vec![1u8];
        for &p in polys {
            let mut out = vec![0u8; gp.len() + m];
            for (i, &gi) in gp.iter().enumerate() {
                if gi != 0 {
                    for d in 0..=m {
                        out[i + d] ^= ((p >> d) & 1) as u8;
                    }
                }
            }
            gp = out;
        }
        debug_assert_eq!(gp.len(), parity + 1);
        let mut g: Reg = [0; 3];
        for i in 0..parity {
            if gp[i] != 0 {
                let j = i + shift;
                g[j / 64] |= 1 << (j % 64);
            }
        }
        let mut bch = Bch { m, q, parity, shift, exp, log, g, byte_tab: Vec::new() };
        bch.byte_tab = (0..256u32)
            .map(|b| {
                let mut r: Reg = [0; 3];
                r[2] = (b as u64) << 56;
                for _ in 0..8 {
                    bch.step(&mut r, 0);
                }
                r
            })
            .collect();
        bch
    }

    /// r = r x + bit (mod g), bitwise.
    fn step(&self, r: &mut Reg, bit: u64) {
        let top = r[2] >> 63;
        r[2] = (r[2] << 1) | (r[1] >> 63);
        r[1] = (r[1] << 1) | (r[0] >> 63);
        r[0] = (r[0] << 1) | bit;
        if top != 0 {
            for i in 0..3 {
                r[i] ^= self.g[i];
            }
        }
    }

    fn mul(&self, a: u16, b: u16) -> u16 {
        if a == 0 || b == 0 {
            0
        } else {
            self.exp[self.log[a as usize] as usize + self.log[b as usize] as usize]
        }
    }

    /// Remainder of the codeword (one bit a byte, 0/1) mod g(x), bit i =
    /// coefficient of x^i.
    fn remainder(&self, bits: &[u8]) -> Reg {
        let mut r = self.remainder_up(bits);
        for _ in 0..self.shift {
            self.step(&mut r, 0);
        }
        // (c x^shift mod g x^shift) = (c mod g) x^shift: shift it back
        let s = self.shift;
        if s > 0 {
            r = [r[0] >> s | r[1] << (64 - s), r[1] >> s | r[2] << (64 - s), r[2] >> s];
        }
        r
    }

    fn remainder_up(&self, bits: &[u8]) -> Reg {
        let mut r: Reg = [0; 3];
        let whole = bits.len() / 8 * 8;
        for c in bits[..whole].chunks_exact(8) {
            let b = c.iter().fold(0u8, |a, &v| (a << 1) | v);
            let top = (r[2] >> 56) as usize;
            r[2] = (r[2] << 8) | (r[1] >> 56);
            r[1] = (r[1] << 8) | (r[0] >> 56);
            r[0] = (r[0] << 8) | b as u64;
            let t = &self.byte_tab[top];
            for i in 0..3 {
                r[i] ^= t[i];
            }
        }
        for &v in &bits[whole..] {
            self.step(&mut r, v as u64);
        }
        r
    }

    /// Does `lam` (degree l = len - 1, lam[l] != 0) have l distinct roots
    /// in the field, i.e. x^(2^16) = x mod lam? A frame too damaged for
    /// BCH almost always gives a locator that does not, and this costs 16
    /// squarings of a degree < 12 polynomial instead of a Chien search over
    /// every position (tens of ms on the A9).
    fn splits(&self, lam: &[u16]) -> bool {
        let l = lam.len() - 1;
        if l == 0 {
            return true;
        }
        // Monic: x^l = sum m[k] x^k (characteristic 2: minus is plus).
        let inv = self.exp[(self.q - self.log[lam[l] as usize] as usize) % self.q];
        let m: Vec<u16> = lam[..l].iter().map(|&c| self.mul(c, inv)).collect();
        let x: Vec<u16> = if l == 1 { vec![m[0]] } else { (0..l).map(|i| (i == 1) as u16).collect() };
        let mut r = x.clone();
        let mut sq = vec![0u16; 2 * l - 1];
        for _ in 0..self.m {
            sq.iter_mut().for_each(|v| *v = 0);
            for (i, &c) in r.iter().enumerate() {
                sq[2 * i] = self.mul(c, c);
            }
            for d in (l..sq.len()).rev() {
                let c = sq[d];
                if c != 0 {
                    for (k, &mk) in m.iter().enumerate() {
                        sq[d - l + k] ^= self.mul(c, mk);
                    }
                }
            }
            r.copy_from_slice(&sq[..l]);
        }
        r == x
    }

    /// Systematic encoding: fills the last 192 of `cw` (bits, 0/1) with the
    /// parity of the rest.
    pub fn encode(&self, cw: &mut [u8]) {
        let par = self.parity;
        let k = cw.len() - par;
        cw[k..].fill(0);
        let r = self.remainder(cw);
        for i in 0..par {
            cw[k + par - 1 - i] = ((r[i / 64] >> (i % 64)) & 1) as u8;
        }
    }

    /// Correct `bits` (n = Kbch + 192 of them, 0/1) in place.
    pub fn decode(&self, bits: &mut [u8]) -> Outcome {
        let n = bits.len();
        let r = self.remainder(bits);
        if r == [0; 3] {
            return Outcome::Clean;
        }
        // S_j = r(alpha^j), j = 1 .. 2t (g(alpha^j) = 0).
        let mut s = [0u16; 2 * T + 1];
        for (j, sj) in s.iter_mut().enumerate().skip(1) {
            let mut acc = 0u16;
            for i in 0..self.parity {
                if (r[i / 64] >> (i % 64)) & 1 == 1 {
                    acc ^= self.exp[(i * j) % self.q];
                }
            }
            *sj = acc;
        }
        // Berlekamp-Massey: the error locator.
        let mut lam = [0u16; 2 * T + 2];
        let mut b = [0u16; 2 * T + 2];
        lam[0] = 1;
        b[0] = 1;
        let mut l = 0usize;
        let mut mshift = 1usize;
        let mut bd = 1u16;
        for k in 0..2 * T {
            let mut d = s[k + 1];
            for i in 1..=l {
                d ^= self.mul(lam[i], s[k + 1 - i]);
            }
            if d == 0 {
                mshift += 1;
                continue;
            }
            let coef = self.mul(d, self.exp[(self.q - self.log[bd as usize] as usize) % self.q]);
            let prev = lam;
            for i in 0..lam.len() - mshift {
                lam[i + mshift] ^= self.mul(coef, b[i]);
            }
            if 2 * l <= k {
                l = k + 1 - l;
                b = prev;
                bd = d;
                mshift = 1;
            } else {
                mshift += 1;
            }
        }
        if l > T || !self.splits(&lam[..=l]) {
            return Outcome::Failed;
        }
        // Chien search: bit at degree p is wrong when lam(alpha^-p) = 0.
        // Each term in the log domain: log(lam_i) - i p (mod Q), one table
        // read and a conditional subtract a step (the A9 has no divide: a %
        // here made a failed frame cost 67 ms).
        let mut terms: Vec<(usize, usize)> =
            (1..=l).filter(|&i| lam[i] != 0).map(|i| (self.log[lam[i] as usize] as usize, i)).collect();
        let mut found = Vec::with_capacity(l);
        for p in 0..n {
            let mut sum = 1u16;
            for &(e, _) in &terms {
                sum ^= self.exp[e];
            }
            if sum == 0 {
                found.push(p);
                if found.len() == l {
                    break;
                }
            }
            // next p: term i times alpha^-i
            for (e, i) in terms.iter_mut() {
                *e = if *e >= *i { *e - *i } else { *e + self.q - *i };
            }
        }
        if found.len() != l {
            return Outcome::Failed;
        }
        for p in &found {
            bits[n - 1 - p] ^= 1;
        }
        Outcome::Fixed(l)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bch_rejects_far_beyond_t() {
        // 40 errors: the locator (almost always) does not split; the frame
        // fails without a Chien search.
        let bch = Bch::new();
        let mut cw = vec![0u8; 32208 + 192];
        let mut x = 99u32;
        for v in cw.iter_mut().take(32208) {
            x = x.wrapping_mul(1103515245).wrapping_add(12345);
            *v = ((x >> 16) & 1) as u8;
        }
        bch.encode(&mut cw);
        let mut failed = 0;
        for _ in 0..20 {
            let mut c = cw.clone();
            for _ in 0..40 {
                x = x.wrapping_mul(1103515245).wrapping_add(12345);
                let n = c.len();
                c[(x as usize >> 3) % n] ^= 1;
            }
            if matches!(bch.decode(&mut c), Outcome::Failed) {
                failed += 1;
            }
        }
        assert!(failed >= 19, "{failed} of 20 failed");
    }

    #[test]
    fn bch_corrects_up_to_t() {
        let bch = Bch::new();
        for (kbch, seed) in [(32208usize, 1u32), (48408, 7)] {
            let mut x = seed;
            let msg: Vec<u8> = (0..kbch)
                .map(|_| {
                    x = x.wrapping_mul(1103515245).wrapping_add(12345);
                    ((x >> 16) & 1) as u8
                })
                .collect();
            let mut cw = msg.clone();
            cw.extend(std::iter::repeat_n(0u8, bch.parity()));
            bch.encode(&mut cw);
            let mut c = cw.clone();
            assert!(matches!(bch.decode(&mut c), Outcome::Clean));
            for ne in [1usize, 5, 12] {
                let mut c = cw.clone();
                for e in 0..ne {
                    x = x.wrapping_mul(1103515245).wrapping_add(12345);
                    let p = (x as usize >> 3) % c.len();
                    let p = if e == 0 { c.len() - 1 } else if e == 1 { 0 } else { p };
                    c[p] ^= 1;
                }
                let want = c.iter().zip(&cw).filter(|(a, b)| a != b).count();
                match bch.decode(&mut c) {
                    Outcome::Fixed(k) => assert_eq!(k, want),
                    _ => panic!("{ne} errors not fixed"),
                }
                assert!(c == cw, "{ne} errors: wrong correction");
            }
            let mut c = cw.clone();
            for p in (0..40).map(|i| i * 797) {
                c[p] ^= 1;
            }
            assert!(!matches!(bch.decode(&mut c), Outcome::Fixed(_)) || c != cw);
        }
    }
}
