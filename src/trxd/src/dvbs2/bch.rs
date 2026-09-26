//! BCH decoding for normal frames (EN 302 307-1 5.3.1): t = 12 over GF(2^16),
//! 192 parity bits. After the LDPC decoder it corrects the few bits a
//! (falsely) converged or nearly converged LDPC decode leaves wrong.
//!
//! The codeword's first bit is the coefficient of x^(n-1); the parity is the
//! last 192 bits. Fast path: the remainder mod g(x) (a 192-bit LFSR, byte at
//! a time); only when it is non-zero the syndromes, Berlekamp-Massey and a
//! Chien search over the n positions.

const M: usize = 16;
const T: usize = 12;
const PARITY: usize = M * T;
const Q: usize = (1 << M) - 1;

/// Table 6a: g1 .. g12 (g1 is also the field's primitive polynomial).
const POLYS: [u32; T] =
    [0x1002D, 0x10173, 0x10FBD, 0x15A55, 0x11F2F, 0x1F7B5, 0x1AF65, 0x17367, 0x10EA1, 0x175A7, 0x13A2D, 0x11AE3];

/// 192-bit register, bit i = coefficient of x^i.
type Reg = [u64; 3];

pub struct Bch {
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
    pub fn new() -> Bch {
        let mut exp = vec![0u16; 2 * Q];
        let mut log = vec![0u16; Q + 1];
        let mut x: u32 = 1;
        for i in 0..Q {
            exp[i] = x as u16;
            exp[i + Q] = x as u16;
            log[x as usize] = i as u16;
            x <<= 1;
            if x >> M != 0 {
                x ^= POLYS[0];
            }
        }
        // g = product of the 12 polynomials (degree 192).
        let mut gp = vec![1u8];
        for p in POLYS {
            let mut out = vec![0u8; gp.len() + M];
            for (i, &gi) in gp.iter().enumerate() {
                if gi != 0 {
                    for d in 0..=M {
                        out[i + d] ^= ((p >> d) & 1) as u8;
                    }
                }
            }
            gp = out;
        }
        debug_assert_eq!(gp.len(), PARITY + 1);
        let mut g: Reg = [0; 3];
        for i in 0..PARITY {
            if gp[i] != 0 {
                g[i / 64] |= 1 << (i % 64);
            }
        }
        let mut bch = Bch { exp, log, g, byte_tab: Vec::new() };
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

    /// Remainder of the codeword (one bit a byte, 0/1) mod g(x).
    fn remainder(&self, bits: &[u8]) -> Reg {
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

    /// Systematic encoding: fills the last 192 of `cw` (bits, 0/1) with the
    /// parity of the rest.
    pub fn encode(&self, cw: &mut [u8]) {
        let k = cw.len() - PARITY;
        cw[k..].fill(0);
        let r = self.remainder(cw);
        for i in 0..PARITY {
            cw[k + PARITY - 1 - i] = ((r[i / 64] >> (i % 64)) & 1) as u8;
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
            for i in 0..PARITY {
                if (r[i / 64] >> (i % 64)) & 1 == 1 {
                    acc ^= self.exp[(i * j) % Q];
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
            let coef = self.mul(d, self.exp[(Q - self.log[bd as usize] as usize) % Q]);
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
        if l > T {
            return Outcome::Failed;
        }
        // Chien search: bit at degree p is wrong when lam(alpha^-p) = 0.
        let mut terms: Vec<(u16, usize)> =
            (1..=l).filter(|&i| lam[i] != 0).map(|i| (lam[i], i)).collect();
        let mut found = Vec::with_capacity(l);
        for p in 0..n {
            let mut sum = 1u16;
            for (v, _) in &terms {
                sum ^= *v;
            }
            if sum == 0 {
                found.push(p);
                if found.len() == l {
                    break;
                }
            }
            // next p: term i times alpha^-i
            for (v, i) in terms.iter_mut() {
                *v = self.exp[self.log[*v as usize] as usize + Q - *i % Q];
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
            cw.extend(std::iter::repeat_n(0u8, PARITY));
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
