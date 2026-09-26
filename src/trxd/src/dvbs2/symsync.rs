//! Symbol timing recovery in the FPGA (maia-hdl `symsync.py`), after the
//! DDC: Gardner timing error, cubic (Catmull-Rom) interpolation, a PI loop.
//! This is its bit-exact model; the receiver takes the symbols it makes
//! (one ring word a symbol) instead of doing timing itself.
//!
//! Integer arithmetic throughout (every shift an arithmetic, flooring one):
//! - positions in samples, Q.24 (`t`: the next strobe; `omega`: samples per
//!   symbol, clamped to the nominal +- 1/128);
//! - interpolation at `mu` Q0.16 from four samples a, b, c, d (b at the
//!   integer part), doubled coefficients `C1 = c - a`,
//!   `C2 = 2a - 5b + 4c - d`, `C3 = (d - a) + 3 (b - c)`, Horner
//!   `v = ((C3 mu >> 16) + C2) mu >> 16 ...`, `y = (v + 2b ...) >> 1`;
//! - error `e = Re((prev - y) conj(mid))` with `mid` half a symbol earlier,
//!   normalized by the AGC's power of two: `en = (e << 16) >> msb(agc)`,
//!   clamped to +-1.0 (Q.16); `agc += (|y|^2 - agc) >> 10`;
//! - `omega += (en << 8) >> ki_shift`, `t += omega + ((en << 8) >> kp_shift)`.
//!
//! A strobe happens once the sample after next of the strobe's integer
//! position has come in (n - floor(t) >= 2); omega >= 2 means at most one a
//! sample.

pub const FRAC: u32 = 24;
const MU_BITS: u32 = 16;
/// Default loop gains (shifts): about the software receiver's 0.01 / 1e-4.
pub const KP_SHIFT: u32 = 7;
pub const KI_SHIFT: u32 = 13;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    /// Samples per symbol, Q8.24.
    pub omega: u32,
    pub kp_shift: u32,
    pub ki_shift: u32,
}

impl Params {
    pub fn new(fs: f64, rs: f64) -> Params {
        Params { omega: (fs / rs * (1u64 << FRAC) as f64).round() as u32, kp_shift: KP_SHIFT, ki_shift: KI_SHIFT }
    }
}

pub struct SymSync {
    p: Params,
    hist: [[i32; 2]; 8],
    /// Samples received (the newest is n - 1).
    n: u64,
    t: u64,
    omega: i64,
    prev: [i32; 2],
    agc: i64,
}

fn sat16(x: i32) -> i32 {
    x.clamp(-32768, 32767)
}

impl SymSync {
    pub fn new(p: Params) -> SymSync {
        SymSync { p, hist: [[0; 2]; 8], n: 0, t: 0, omega: p.omega as i64, prev: [0; 2], agc: 0 }
    }

    fn interp(&self, pos: u64) -> [i32; 2] {
        let i = pos >> FRAC;
        let mu = ((pos >> (FRAC - MU_BITS)) & 0xFFFF) as i64;
        let h = |k: u64| self.hist[(k & 7) as usize];
        let (a, b, c, d) = (h(i.wrapping_sub(1)), h(i), h(i + 1), h(i + 2));
        let mut y = [0; 2];
        for q in 0..2 {
            let (a, b, c, d) = (a[q] as i64, b[q] as i64, c[q] as i64, d[q] as i64);
            let c1 = c - a;
            let c2 = 2 * a - 5 * b + 4 * c - d;
            let c3 = (d - a) + 3 * (b - c);
            let mut v = c3;
            v = ((v * mu) >> MU_BITS) + c2;
            v = ((v * mu) >> MU_BITS) + c1;
            v = ((v * mu) >> MU_BITS) + 2 * b;
            y[q] = sat16((v >> 1) as i32);
        }
        y
    }

    /// One DDC sample in; a symbol out when a strobe falls on it.
    pub fn push(&mut self, x: [i16; 2]) -> Option<[i16; 2]> {
        self.hist[(self.n & 7) as usize] = [x[0] as i32, x[1] as i32];
        self.n += 1;
        // The newest sample is n - 1: strobe when floor(t) + 2 <= n - 1.
        if (self.n - 1) < (self.t >> FRAC) + 2 {
            return None;
        }
        let y = self.interp(self.t);
        let mid = self.interp(self.t - (self.omega as u64 >> 1));
        let e = (self.prev[0] - y[0]) as i64 * mid[0] as i64 + (self.prev[1] - y[1]) as i64 * mid[1] as i64;
        let p = y[0] as i64 * y[0] as i64 + y[1] as i64 * y[1] as i64;
        self.agc += (p - self.agc) >> 10;
        let s = 63 - (self.agc.max(1) as u64).leading_zeros();
        let one = 1i64 << 16;
        let en = ((e << 16) >> s).clamp(-one, one);
        let nom = self.p.omega as i64;
        self.omega = (self.omega + ((en << 8) >> self.p.ki_shift)).clamp(nom - (nom >> 7), nom + (nom >> 7));
        self.t = (self.t as i64 + self.omega + ((en << 8) >> self.p.kp_shift)) as u64;
        self.prev = y;
        Some([y[0] as i16, y[1] as i16])
    }

    /// Samples per symbol the loop has settled on.
    pub fn omega(&self) -> f64 {
        self.omega as f64 / (1u64 << FRAC) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test vectors for maia-hdl (`test/test_symsync.py`): parameters, DDC
    /// samples in, symbols out. From a board recording (`DATV_CAP`, cf32 at
    /// 512 kS/s, 250 kS/s DVB-S2) and a synthetic weak signal at 333 kS/s.
    /// `SYMSYNC_VECTORS=<file> DATV_CAP=<cf32> cargo test symsync_vectors -- --ignored`
    #[test]
    #[ignore]
    fn symsync_vectors() {
        let path = std::env::var("SYMSYNC_VECTORS").expect("SYMSYNC_VECTORS=<file>");
        let cap = std::fs::read(std::env::var("DATV_CAP").expect("DATV_CAP=<cf32>")).unwrap();
        let q = |v: f32| (v * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
        let real: Vec<[i16; 2]> = cap
            .chunks_exact(8)
            .skip(100_000)
            .take(12_000)
            .map(|c| [q(f32::from_le_bytes(c[..4].try_into().unwrap())), q(f32::from_le_bytes(c[4..].try_into().unwrap()))])
            .collect();
        // Weak QPSK-ish signal: random symbols, linear pulses, noise.
        let mut seed = 9u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            (seed >> 33) as i64
        };
        let sps = 768_000.0 / 333_000.0;
        let syms: Vec<[i64; 2]> = (0..6000).map(|_| [if rnd() & 1 == 0 { 300 } else { -300 }, if rnd() & 1 == 0 { 300 } else { -300 }]).collect();
        let synth: Vec<[i16; 2]> = (0..12_000)
            .map(|k| {
                let tk = k as f64 / sps;
                let i = tk.floor() as usize;
                let f = tk - i as f64;
                let s0 = syms[i.min(5999)];
                let s1 = syms[(i + 1).min(5999)];
                let mut z = [0i16; 2];
                for c in 0..2 {
                    let v = s0[c] as f64 * (1.0 - f) + s1[c] as f64 * f + ((rnd() % 121) - 60) as f64;
                    z[c] = v as i16;
                }
                z
            })
            .collect();
        let mut cases = Vec::new();
        for (name, fs, rs, input) in [("capture_250k", 512e3, 250e3, real), ("synthetic_333k", 768e3, 333e3, synth)] {
            let p = Params::new(fs, rs);
            let mut ss = SymSync::new(p);
            let out: Vec<[i16; 2]> = input.iter().filter_map(|&x| ss.push(x)).collect();
            eprintln!("{name}: {} samples, {} symbols, omega {:.6}", input.len(), out.len(), ss.omega());
            cases.push(serde_json::json!({"name": name, "omega": p.omega, "kp_shift": p.kp_shift, "ki_shift": p.ki_shift,
                "input": input, "output": out, "omega_end": ss.omega}));
        }
        std::fs::write(path, serde_json::to_vec(&cases).unwrap()).unwrap();
    }
}
