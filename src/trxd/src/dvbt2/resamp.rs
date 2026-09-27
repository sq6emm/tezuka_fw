//! The FPGA's T2 receive resampler (maia-hdl `t2resamp.py`): ADC samples
//! (3.072 MS/s, 12 bits) to the T2 elementary rate (131/71 MS/s at 1.7 MHz)
//! into the DDC ring. This is its bit-exact model.
//!
//! Output m falls at input time `T_m = m R` (R = input / output rate, Q2.30
//! in the datv_omega register). `d` is the time of the next output less the
//! newest input's, Q.30: each input subtracts 1.0; when that leaves d <= 0 an
//! output is due, `phase = -d` (in [0, 1)) and d += R. The output is
//!
//! ```text
//! y = sat16((sum_k h[k P + p] x[n - k] + 2^13) >> 14),   p = phase >> 23
//! ```
//!
//! over the last `SPAN` inputs (x[n] the newest), `h` a low-pass sampled at
//! t = k - p / P - (SPAN - 1) / 2 input samples and loaded by the CPU.

use num_complex::Complex32;

pub const SPAN: usize = 32;
pub const PHASES_LOG2: u32 = 7;
pub const PHASES: usize = 1 << PHASES_LOG2;
pub const COEFF_WIDTH: u32 = 18;
/// Coefficients are fractions of 2^17 (unity gain at full scale); the
/// output takes 3 bits more (12-bit ADC samples to 15 bits).
const SHIFT: u32 = COEFF_WIDTH - 1;
const OUT_SHIFT: u32 = SHIFT - 3;
pub const FRAC: u32 = 30;

/// R, the input samples per output sample, Q2.30.
pub fn step(fs_in: f64, fs_out: f64) -> u32 {
    (fs_in / fs_out * (1u64 << FRAC) as f64).round() as u32
}

/// The output rate a step gives exactly.
pub fn rate_out(fs_in: f64, step: u32) -> f64 {
    fs_in * (1u64 << FRAC) as f64 / step as f64
}

/// The low-pass table (address k * PHASES + p): a Kaiser-windowed sinc with
/// its cut-off at `cutoff_hz`, a DC gain of `gain` at every phase (times 8
/// through the output shift).
pub fn table(fs_in: f64, cutoff_hz: f64, gain: f64) -> Vec<i32> {
    let fc = cutoff_hz / fs_in;
    let half = SPAN as f64 / 2.0;
    let beta = 5.0;
    let i0 = |x: f64| {
        let (mut s, mut t) = (1.0, 1.0);
        for k in 1..40 {
            t *= (x / 2.0) / k as f64;
            s += t * t;
        }
        s
    };
    let g = |t: f64| {
        let sinc = if t.abs() < 1e-12 { 1.0 } else { (std::f64::consts::PI * 2.0 * fc * t).sin() / (std::f64::consts::PI * 2.0 * fc * t) };
        let r = t / half;
        let w = if r.abs() >= 1.0 { 0.0 } else { i0(beta * (1.0 - r * r).sqrt()) / i0(beta) };
        2.0 * fc * sinc * w
    };
    let c = (SPAN as f64 - 1.0) / 2.0;
    let mut h = vec![0i32; SPAN * PHASES];
    for p in 0..PHASES {
        let taps: Vec<f64> = (0..SPAN).map(|k| g(k as f64 - p as f64 / PHASES as f64 - c)).collect();
        let sum: f64 = taps.iter().sum();
        for (k, v) in taps.iter().enumerate() {
            h[k * PHASES + p] = (v / sum * gain * (1 << SHIFT) as f64).round() as i32;
        }
    }
    h
}

/// The table trxd loads for a T2 channel: flat over the occupied band,
/// down before the first alias at `fs_out - band / 2`.
pub fn t2_table(fs_in: f64, fs_out: f64) -> Vec<i32> {
    // Occupied: 1705 carriers of fs_out / 2048.
    let edge = 1705.0 / 2048.0 * fs_out / 2.0;
    table(fs_in, (edge + (fs_out - edge)) / 2.0, 1.0)
}

pub struct Resampler {
    h: Vec<i32>,
    step: i64,
    hist: [[i32; 2]; SPAN],
    d: i64,
}

impl Resampler {
    pub fn new(h: Vec<i32>, step: u32) -> Resampler {
        assert_eq!(h.len(), SPAN * PHASES);
        assert!(h.iter().all(|&c| c.unsigned_abs() < 1 << SHIFT), "coefficients over 18 bits");
        Resampler { h, step: step as i64, hist: [[0; 2]; SPAN], d: step as i64 }
    }

    /// One ADC sample in; an output sample when one is due.
    pub fn push(&mut self, x: [i16; 2]) -> Option<[i16; 2]> {
        self.hist.copy_within(0..SPAN - 1, 1);
        self.hist[0] = [x[0] as i32, x[1] as i32];
        self.d -= 1 << FRAC;
        if self.d > 0 {
            return None;
        }
        let p = ((-self.d) >> (FRAC - PHASES_LOG2)) as usize;
        self.d += self.step;
        let mut s = [1i64 << (OUT_SHIFT - 1); 2];
        for k in 0..SPAN {
            let c = self.h[k * PHASES + p] as i64;
            s[0] += c * self.hist[k][0] as i64;
            s[1] += c * self.hist[k][1] as i64;
        }
        let q = |v: i64| (v >> OUT_SHIFT).clamp(-32768, 32767) as i16;
        Some([q(s[0]), q(s[1])])
    }

    pub fn process(&mut self, x: &[[i16; 2]]) -> Vec<[i16; 2]> {
        x.iter().filter_map(|&v| self.push(v)).collect()
    }
}

/// Float samples (full scale 1.0) to the 12-bit ADC's integers.
pub fn adc12(x: &[Complex32], scale: f32) -> Vec<[i16; 2]> {
    let q = |v: f32| (v * scale * 2048.0).round().clamp(-2048.0, 2047.0) as i16;
    x.iter().map(|z| [q(z.re), q(z.im)]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS_IN: f64 = 3_072_000.0;
    const FS_T2: f64 = 131e6 / 71.0;

    #[test]
    fn rate_is_exact_enough() {
        let s = step(FS_IN, FS_T2);
        let err = (rate_out(FS_IN, s) - FS_T2) / FS_T2;
        assert!(err.abs() < 1e-9, "{err}");
    }

    /// A tone in the pass band comes out at the right frequency with little
    /// distortion; one where an alias would land is gone.
    #[test]
    fn pass_and_stop() {
        let s = step(FS_IN, FS_T2);
        let h = t2_table(FS_IN, FS_T2);
        let tone = |f: f64, n: usize| -> Vec<[i16; 2]> {
            (0..n)
                .map(|i| {
                    let t = std::f64::consts::TAU * f * i as f64 / FS_IN;
                    [(1000.0 * t.cos()).round() as i16, (1000.0 * t.sin()).round() as i16]
                })
                .collect()
        };
        for (f, want_db, pass) in [(760e3, 0.0, true), (-500e3, 0.0, true), (1.1e6, -45.0, false), (-1.2e6, -45.0, false)] {
            let mut r = Resampler::new(h.clone(), s);
            let y = r.process(&tone(f, 40_000));
            let y = &y[100..];
            // Project on the expected output tone (aliased into fs_out).
            let fo = f - (f / FS_T2).round() * FS_T2;
            let (mut acc, mut pw) = (Complex32::default(), 0f64);
            for (i, v) in y.iter().enumerate() {
                let t = -std::f64::consts::TAU * fo * i as f64 / FS_T2;
                acc += Complex32::new(v[0] as f32, v[1] as f32) * Complex32::new(t.cos() as f32, t.sin() as f32);
                pw += (v[0] as f64).powi(2) + (v[1] as f64).powi(2);
            }
            let amp = acc.norm() as f64 / y.len() as f64;
            let db = 20.0 * (amp / 8000.0).log10();
            let total = (pw / y.len() as f64).sqrt();
            eprintln!("{f} Hz: tone {db:.2} dB, total {:.1} dB", 20.0 * (total / 8000.0).log10());
            if pass {
                assert!(db.abs() < 0.5, "{f}: {db}");
                // Everything else (images, phase quantization) 40 dB down.
                let rest = (total * total - amp * amp).max(0.0).sqrt();
                eprintln!("  rest {:.1} dB", 20.0 * (rest / amp).log10());
                assert!(20.0 * (rest / amp).log10() < -40.0, "{f}: rest {rest}");
            } else {
                assert!(20.0 * (total / 8000.0).log10() < want_db, "{f}: {total}");
            }
        }
    }

    /// An ADC recording (cf32, `T2RSIN`, full scale `T2RSSCALE`) through the
    /// model, written as cf32 at the T2 rate (`T2RSOUT`, 1.0 = 32768).
    #[test]
    #[ignore]
    fn t2resamp_file() {
        let raw = std::fs::read(std::env::var("T2RSIN").unwrap()).unwrap();
        let scale: f32 = std::env::var("T2RSSCALE").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0);
        let x: Vec<Complex32> = raw.chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
        let adc = adc12(&x, scale);
        eprintln!("ADC peak {}", adc.iter().map(|v| v[0].abs().max(v[1].abs())).max().unwrap());
        let mut r = Resampler::new(t2_table(FS_IN, FS_T2), step(FS_IN, FS_T2));
        let y = r.process(&adc);
        let b: Vec<u8> = y.iter().flat_map(|v| [(v[0] as f32 / 32768.0).to_le_bytes(), (v[1] as f32 / 32768.0).to_le_bytes()]).flatten().collect();
        std::fs::write(std::env::var("T2RSOUT").unwrap(), b).unwrap();
    }

    /// Writes vectors for maia-hdl test_t2resamp.py.
    #[test]
    #[ignore]
    fn t2resamp_vectors() {
        let path = std::env::var("T2RESAMP_VECTORS").expect("T2RESAMP_VECTORS=<file>");
        let s = step(FS_IN, FS_T2);
        let h = t2_table(FS_IN, FS_T2);
        let mut seed = 5u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            (seed >> 33) as i64
        };
        // Full-scale noise and a strong tone: the sums and saturation.
        let input: Vec<[i16; 2]> = (0..6000)
            .map(|i| {
                if i < 3000 {
                    [(rnd() % 4096 - 2048) as i16, (rnd() % 4096 - 2048) as i16]
                } else {
                    let t = std::f64::consts::TAU * 0.13 * i as f64;
                    [(2047.0 * t.cos()) as i16, (-2048.0 * t.sin()).max(-2048.0) as i16]
                }
            })
            .collect();
        let mut r = Resampler::new(h.clone(), s);
        let out = r.process(&input);
        let v = serde_json::json!([{"name": "t2_1m7", "step": s, "coeffs": h, "input": input, "output": out}]);
        std::fs::write(path, v.to_string()).unwrap();
    }
}
