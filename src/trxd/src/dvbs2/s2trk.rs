//! The FPGA's known-symbol accumulator (maia-hdl `s2trk.py`) and its model:
//! per header or pilot block of a frame, the sum of its symbols times the
//! conjugate references (less e^(j pi/4): every reference is e^(j pi/4)
//! j^q), the power and the AFC mixer's phase it used. The ring-mode
//! receiver (`rx.rs` frame()) fits the carrier and measures amplitude and
//! noise from these entries instead of from every known symbol; where the
//! FPGA has none (no unit, a frame it did not follow) [`block`] makes the
//! same entry from the ring's words.
//!
//! Also the soft PLS decoder ([`pls_decode`]): the 64 PLS symbols of a
//! header against all 128 codes through a 32-point Hadamard transform, in
//! place of correlating the 115 other headers one by one.

use num_complex::Complex32;

pub const SLOT: usize = 90;
pub const PILOT: usize = 36;
pub const GROUP: usize = 16 * SLOT;
pub const ENTRY_WORDS: usize = 6;

/// The receiver's commands for the FPGA's unit (fpga.rs trk_ctl).
#[derive(Clone, Debug, PartialEq)]
pub enum Ctl {
    /// Our header's 90 quarter turns.
    Header(Vec<u8>),
    /// The mixer's step (2^32 a turn), taken at the next frame start.
    Dth(u32),
    /// Follow frames of `len` symbols from the header at absolute `base`.
    Load { base: u64, len: u32, pilots: bool, npil: u8 },
}

/// An angle (radians) as the unit's phase (2^32 a turn).
pub fn turns(a: f64) -> u32 {
    ((a / std::f64::consts::TAU).rem_euclid(1.0) * 4_294_967_296.0).round() as u64 as u32
}

/// A step a symbol (radians) as the unit's dth.
pub fn step(w: f64) -> u32 {
    (w / std::f64::consts::TAU * 4_294_967_296.0).round() as i64 as u32
}

/// One block's entry (as the FPGA's FIFO gives it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    /// Absolute ring index of the block's first symbol (low 32 bits).
    pub k0: u32,
    /// Sum of y j^(-q) (y: the symbol times the mixer, Q15 table, 16 bits).
    pub s: [i32; 2],
    /// Sum of |x|^2 (the ring's raw words) >> 8.
    pub p: u32,
    /// The mixer's phase at the first symbol and its step (2^32 a turn).
    pub phi: u32,
    pub dth: u32,
}

impl Entry {
    pub fn from_words(w: &[u32; ENTRY_WORDS]) -> Entry {
        Entry { k0: w[0], s: [w[1] as i32, w[2] as i32], p: w[3], phi: w[4], dth: w[5] }
    }

    pub fn words(&self) -> [u32; ENTRY_WORDS] {
        [self.k0, self.s[0] as u32, self.s[1] as u32, self.p, self.phi, self.dth]
    }

    /// The block's correlation with its references per symbol (the ring's
    /// unit: full scale 1), as if mixed with phase `phi_mid` (radians) at
    /// its centre instead of the entry's own.
    pub fn corr(&self, len: usize, phi_mid: f64) -> Complex32 {
        let own = (self.phi as f64 + self.dth as i32 as f64 * (len - 1) as f64 / 2.0) * std::f64::consts::TAU / 4_294_967_296.0;
        let a = phi_mid - own - std::f64::consts::FRAC_PI_4;
        Complex32::new(self.s[0] as f32, self.s[1] as f32) * Complex32::new(a.cos() as f32, a.sin() as f32) / (32768.0 * len as f32)
    }

    /// The block's power (sum of |x|^2, full scale 1).
    pub fn power(&self) -> f64 {
        self.p as f64 * 256.0 / (32768.0 * 32768.0)
    }
}

/// The 1024-entry Q15 table (maia-hdl t2p1.nco_tables).
pub struct Tables {
    cos: Vec<i32>,
    sin: Vec<i32>,
}

impl Default for Tables {
    fn default() -> Self {
        let ph = |i: usize| std::f64::consts::TAU * i as f64 / 1024.0;
        Tables {
            cos: (0..1024).map(|i| (32767.0 * ph(i).cos()).round() as i32).collect(),
            sin: (0..1024).map(|i| (32767.0 * ph(i).sin()).round() as i32).collect(),
        }
    }
}

fn sat16(v: i32) -> i32 {
    v.clamp(-32768, 32767)
}

/// A block's entry from its words (the first at absolute index `k0`), the
/// references' quarter turns and the mixer (s2trk.py block_entry: bit for
/// bit what the FPGA gives).
pub fn block(t: &Tables, words: &[u32], k0: u64, refs: impl Iterator<Item = u8>, phi0: u32, dth: u32) -> Entry {
    let (mut sr, mut si, mut p) = (0i64, 0i64, 0u64);
    let mut phi = phi0;
    for (&w, q) in words.iter().zip(refs) {
        let (xr, xi) = (w as u16 as i16 as i32, (w >> 16) as u16 as i16 as i32);
        let i = (phi.wrapping_add(1 << 21) >> 22) as usize & 1023;
        let (c, s) = (t.cos[i], t.sin[i]);
        let yr = sat16((xr * c - xi * s + (1 << 14)) >> 15);
        let yi = sat16((xr * s + xi * c + (1 << 14)) >> 15);
        let (zr, zi) = match q & 3 {
            0 => (yr, yi),
            1 => (yi, -yr),
            2 => (-yr, -yi),
            _ => (-yi, yr),
        };
        sr += zr as i64;
        si += zi as i64;
        p += (xr * xr + xi * xi) as u64;
        phi = phi.wrapping_add(dth);
    }
    Entry { k0: k0 as u32, s: [sr as i32, si as i32], p: (p >> 8) as u32, phi: phi0, dth }
}

/// The quarter turns q of references e^(j pi/4) j^q (a header's symbols).
pub fn quarters(refs: &[Complex32]) -> Vec<u8> {
    refs.iter()
        .map(|r| ((((r.arg() - std::f32::consts::FRAC_PI_4) / std::f32::consts::FRAC_PI_2).round() as i32).rem_euclid(4)) as u8)
        .collect()
}

/// The PLS index (MODCOD << 2 | short << 1 | pilots) a header's 90 symbols
/// (derotated, roughly in phase) carry, and how well it fits (the
/// correlation over the sum of magnitudes, 0..1). EN 302 307 5.5.2.4: 7
/// bits, a (32, 6) first-order Reed-Muller code, each bit sent twice (the
/// second inverted when the last bit is 1), scrambled, pi/2-BPSK after the
/// 26 SOF symbols.
pub fn pls_decode(h: &[Complex32]) -> (u8, f32) {
    const PLS_SCRAMBLE: u64 = 0x719D_83C9_5342_2DFA;
    // soft bit of symbol s (positive: 0): pi/2-BPSK, 45 deg (even) or 135
    // deg (odd) for a 0
    let a = std::f32::consts::FRAC_1_SQRT_2;
    let soft = |s: usize| -> f32 {
        let r = if s & 1 == 0 { Complex32::new(a, a) } else { Complex32::new(-a, a) };
        (h[s] * r.conj()).re
    };
    let mut u = [0f32; 32];
    let mut v = [0f32; 32];
    let mut mag = 0f32;
    for m in 0..32 {
        let s0 = 26 + 2 * m;
        let sc = |i: usize| if (PLS_SCRAMBLE >> (63 - i)) & 1 == 1 { -1.0 } else { 1.0 };
        let (x, y) = (soft(s0) * sc(2 * m), soft(s0 + 1) * sc(2 * m + 1));
        mag += h[s0].norm() + h[s0 + 1].norm();
        u[m] = x + y;
        v[m] = x - y;
    }
    // Hadamard: H[c] = sum_m u[m] (-1)^popcount(c & m)
    let fht = |x: &mut [f32; 32]| {
        let mut h = 1;
        while h < 32 {
            for i in (0..32).step_by(2 * h) {
                for j in i..i + h {
                    let (p, q) = (x[j], x[j + h]);
                    x[j] = p + q;
                    x[j + h] = p - q;
                }
            }
            h *= 2;
        }
    };
    fht(&mut u);
    fht(&mut v);
    let (mut best, mut idx) = (f32::MIN, 0u8);
    for (pilots, x) in [(0u8, &u), (1u8, &v)] {
        for (c, &val) in x.iter().enumerate() {
            if val.abs() > best {
                best = val.abs();
                // c's bit r is index bit 6 - r; the sign the all-ones row
                // (index bit 1)
                let mut index = pilots;
                for r in 0..5 {
                    index |= (((c >> r) & 1) as u8) << (6 - r);
                }
                if val < 0.0 {
                    index |= 2;
                }
                idx = index;
            }
        }
    }
    (idx, best / mag.max(1e-20))
}

/// The FPGA's unit word by word (maia-hdl s2trk.Model), for the
/// receiver's tests: what the FIFO would give.
#[cfg(test)]
pub struct Unit {
    hdr_q: Vec<u8>,
    pub dth: u32,
    tabs: Tables,
    k: u64,
    phi: u32,
    dth_cur: u32,
    synced: bool,
    started: bool,
    pending: Option<(u64, u32, bool, u8)>,
    base: u64,
    frame_len: u64,
    pilots: bool,
    npil: u8,
    scr: Vec<u8>,
    acc: (u64, i64, i64, u64, u32, u32),
    pub entries: Vec<[u32; ENTRY_WORDS]>,
}

#[cfg(test)]
impl Unit {
    pub fn new(k0: u64) -> Unit {
        Unit {
            hdr_q: vec![0; SLOT],
            dth: 0,
            tabs: Tables::default(),
            k: k0,
            phi: 0,
            dth_cur: 0,
            synced: false,
            started: false,
            pending: None,
            base: 0,
            frame_len: 0,
            pilots: false,
            npil: 0,
            scr: Vec::new(),
            acc: (0, 0, 0, 0, 0, 0),
            entries: Vec::new(),
        }
    }

    pub fn ctl(&mut self, c: &Ctl) {
        match c {
            Ctl::Header(q) => self.hdr_q = q.clone(),
            Ctl::Dth(d) => self.dth = *d,
            Ctl::Load { base, len, pilots, npil } => self.pending = Some((*base, *len, *pilots, *npil)),
        }
    }

    pub fn push(&mut self, words: &[u32]) {
        for &w in words {
            self.word(w);
        }
    }

    fn word(&mut self, w: u32) {
        let k = self.k;
        if let Some((mut base, len, pilots, npil)) = self.pending.take() {
            self.frame_len = len as u64;
            if base <= k {
                while k - base >= self.frame_len {
                    base += self.frame_len;
                }
            }
            self.base = base;
            self.pilots = pilots;
            self.npil = npil;
            self.synced = true;
            self.started = false;
            if self.scr.len() < len as usize {
                self.scr = super::pl_scrambling(len as usize);
            }
        }
        let (mut known, mut q, mut first, mut last) = (false, 0u8, false, false);
        if self.synced && k >= self.base {
            let mut pos = k - self.base;
            if pos == self.frame_len {
                self.base += self.frame_len;
                pos = 0;
            }
            if pos == 0 {
                self.started = true;
                self.dth_cur = self.dth;
            }
            if self.started {
                let pos = pos as usize;
                if pos < SLOT {
                    (known, q, first, last) = (true, self.hdr_q[pos], pos == 0, pos == SLOT - 1);
                } else {
                    let d = pos - SLOT;
                    q = self.scr[d];
                    let (g, r) = (d / (GROUP + PILOT), d % (GROUP + PILOT));
                    if self.pilots && r >= GROUP && g < self.npil as usize {
                        (known, first, last) = (true, r == GROUP, r == GROUP + PILOT - 1);
                    }
                }
            }
        }
        if !self.started {
            self.dth_cur = self.dth;
        }
        if known {
            let e = block(&self.tabs, &[w], k, std::iter::once(q), self.phi, 0);
            // (one symbol: its sum is the term; its power is p << 8 + the
            // low bits: made again here)
            let (xr, xi) = (w as u16 as i16 as i64, (w >> 16) as u16 as i16 as i64);
            if first {
                self.acc = (k, 0, 0, 0, self.phi, self.dth_cur);
            }
            self.acc.1 += e.s[0] as i64;
            self.acc.2 += e.s[1] as i64;
            self.acc.3 += (xr * xr + xi * xi) as u64;
            if last {
                let a = self.acc;
                self.entries.push([a.0 as u32, a.1 as u32, a.2 as u32, (a.3 >> 8) as u32, a.4, a.5]);
            }
        }
        self.phi = self.phi.wrapping_add(self.dth_cur);
        self.k += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pls_decode_finds_every_header() {
        for modcod in 0..=28u8 {
            for short in [false, true] {
                for pilots in [false, true] {
                    let h = super::super::plheader_typed(modcod, pilots, short);
                    // turned a little and noisy
                    let mut seed = modcod as u64 * 7 + 1;
                    let mut g = || {
                        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                        ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.8
                    };
                    let rx: Vec<Complex32> = h.iter().map(|z| z * Complex32::from_polar(0.9, 0.2) + Complex32::new(g(), g())).collect();
                    let (idx, score) = pls_decode(&rx);
                    assert_eq!(idx, modcod << 2 | (short as u8) << 1 | pilots as u8, "modcod {modcod} short {short} pilots {pilots}");
                    assert!(score > 0.5, "{score}");
                }
            }
        }
    }

    #[test]
    fn quarters_of_a_header() {
        let h = super::super::plheader_typed(4, true, false);
        let q = quarters(&h);
        for (z, &q) in h.iter().zip(&q) {
            let r = Complex32::from_polar(1.0, std::f32::consts::FRAC_PI_4 + std::f32::consts::FRAC_PI_2 * q as f32);
            assert!((z - r).norm() < 1e-5);
        }
    }

    /// block() against maia-hdl s2trk.block_entry (the same input, the
    /// same entry; the Python: words from an LCG, refs (k * 7) % 4).
    #[test]
    fn block_matches_the_python_one() {
        let mut st: u64 = 99;
        let mut rnd = || {
            st = (st * 1103515245 + 12345) & 0x7FFF_FFFF;
            ((st >> 8) % 40001) as i64 - 20000
        };
        let words: Vec<u32> = (0..90).map(|_| (rnd() as u32 & 0xFFFF) | ((rnd() as u32 & 0xFFFF) << 16)).collect();
        let e = block(&Tables::default(), &words, 123_456_789, (0..90u32).map(|k| ((k * 7) % 4) as u8), 0x89AB_CDEF, 0xFFF0_1234);
        assert_eq!(e.words(), WANT);
    }

    const WANT: [u32; ENTRY_WORDS] = [0x075BCD15, 0xFFFF76B7, 0xFFFE5CB2, 0x06442036, 0x89ABCDEF, 0xFFF01234];
}
