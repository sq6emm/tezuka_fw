//! DATV receive front end in the FPGA: Maia SDR's DDC (maia-hdl `ddc.py`),
//! fed with the AD936x samples before the x8 decimator, set up here as a
//! channel filter plus the RRC matched filter.
//!
//! ```text
//! ADC 12-bit IQ (fs_in, 3.072 MS/s) -> mixer (28-bit NCO, 1024-entry table)
//!   -> FIR1 low-pass /d1 -> [FIR2 low-pass /d2] -> FIR3 RRC /2
//!   -> 16-bit IQ at exactly 2 samples per symbol -> DMA ring -> trxd
//! ```
//!
//! The CPU is left with timing, frame sync, carrier and LDPC at 2 samples per
//! symbol; the matched filter at the input rate (the costly part at high
//! symbol rates) and the 384 kS/s stream's bandwidth limit are gone.
//!
//! This module designs the three FIR stages for a symbol rate, lays the
//! coefficients out the way the FPGA's coefficient RAM wants them (the same
//! rules as maia-httpd's `fpga.rs`), and models the DDC bit-exactly (maia-hdl's
//! `Mixer.model`, `FIR4DSP.model`, `FIR2DSP.model`) so the receiver can be
//! tested on exactly what the hardware will deliver.

use num_complex::Complex32;

/// FPGA DDC constants (maia-httpd `ddc/constants.rs`).
pub const COEFF_BITS: u32 = 18;
pub const MAX_DECIMATION: usize = 127;
pub const MAX_OPERATIONS: usize = 128;
/// Coefficient RAM per stage: FIR1 and FIR3 have 4 DSPs (folded), FIR2 has 2.
pub const NUM_ADDR: [usize; 3] = [256, 128, 256];
/// Coefficient address of each stage (the 2 MSBs of the 10-bit address).
pub const ADDR_OFFSET: [usize; 3] = [0, 256, 512];
/// The DDC's fast clock (3x).
pub const CLOCK_HZ: f64 = 187.5e6;
pub const MACC_TRUNC: [u32; 3] = [17, 18, 18];
pub const WIDTH_GROWTH: [u32; 3] = [4, 0, 0];
const IN_WIDTH: u32 = 12;
const OUT_WIDTH: u32 = 16;
const NCO_WIDTH: u32 = 28;
const EXP_WIDTH: u32 = 18;
const PHASE_BITS: u32 = 10;
/// Output samples per symbol.
pub const SPS_OUT: usize = 2;
/// RRC length in symbols (FIR3 runs at 4 samples per symbol).
const RRC_SPAN: usize = 24;
/// Stop-band attenuation of the channel filters, dB.
const STOP_DB: f64 = 60.0;

#[derive(Debug, Clone, PartialEq)]
pub struct Stage {
    pub taps: Vec<i32>,
    pub decimation: usize,
}

/// A DDC set-up: FIR2 may be bypassed; FIR3 is always the matched filter.
#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    pub fs_in: f64,
    pub rs: f64,
    pub fir1: Stage,
    pub fir2: Option<Stage>,
    /// None: bypassed (fast rates, where FIR1 is the matched filter).
    pub fir3: Option<Stage>,
}

impl Design {
    pub fn decimation(&self) -> usize {
        self.fir1.decimation * self.fir2.as_ref().map_or(1, |s| s.decimation) * self.fir3.as_ref().map_or(1, |s| s.decimation)
    }
    pub fn fs_out(&self) -> f64 {
        self.fs_in / self.decimation() as f64
    }
}

/// Symbol rates the DDC can take at `fs_in`: an even decimation D to 2 or
/// more samples per symbol (below 2.7; fractional is fine, the receiver's
/// timing loop interpolates), at least 2 in the first stage; above fs / 8
/// (500 kS/s at 3.072 MS/s) FIR1 at /2 is the matched filter itself, FIR2
/// and FIR3 bypassed (2 to 4 samples per symbol).
pub fn symbol_rate_ok(fs_in: f64, rs: f64) -> bool {
    // Or, faster (above fs / 8), FIR1 alone at /2 as the matched filter.
    rs > 0.0 && (half_decimation(fs_in, rs) >= 2 || fs_in / (2.0 * rs) >= SPS_OUT as f64)
}

/// D / 2: FIR1 x FIR2's share (FIR3 decimates by 2).
fn half_decimation(fs_in: f64, rs: f64) -> usize {
    (fs_in / (2.0 * SPS_OUT as f64 * rs) + 1e-9).floor() as usize
}

/// Kaiser-window low-pass, pass band to `fp`, stop band from `fstop` (Hz at `fs`).
fn lowpass(fs: f64, fp: f64, fstop: f64, max_taps: usize) -> Vec<f64> {
    let dw = std::f64::consts::TAU * (fstop - fp) / fs;
    let beta = 0.1102 * (STOP_DB - 8.7);
    let n = (((STOP_DB - 8.0) / (2.285 * dw)).ceil() as usize + 1).clamp(3, max_taps) | 1;
    let fc = (fp + fstop) / 2.0 / fs;
    let i0 = |x: f64| {
        let (mut s, mut t) = (1.0, 1.0);
        for k in 1..40 {
            t *= (x / 2.0) / k as f64;
            s += t * t;
        }
        s
    };
    let m = (n - 1) as f64 / 2.0;
    (0..n)
        .map(|i| {
            let x = i as f64 - m;
            let sinc = if x == 0.0 { 2.0 * fc } else { (std::f64::consts::TAU * fc * x).sin() / (std::f64::consts::PI * x) };
            sinc * i0(beta * (1.0 - (x / m).powi(2)).max(0.0).sqrt()) / i0(beta)
        })
        .collect()
}

/// Operations a stage needs per output (folded stages do two taps per DSP pair).
fn operations(n_taps: usize, decimation: usize, folded: bool) -> usize {
    let ops = n_taps.div_ceil(decimation);
    if folded { ops.div_ceil(2) } else { ops }
}

/// Design the DDC for symbol rate `rs` (see [`symbol_rate_ok`]) and roll-off.
pub fn design(fs_in: f64, rs: f64, rolloff: f32) -> Result<Design, String> {
    if !symbol_rate_ok(fs_in, rs) {
        return Err(format!("{rs} S/s: too fast for the DDC at {fs_in} S/s"));
    }
    let rem = half_decimation(fs_in, rs);
    let edge = rs * (1.0 + rolloff as f64) / 2.0;
    let budget = |fs: f64, d: usize, folded: bool, max_addr: usize| {
        // Longest filter the clock and the coefficient RAM allow.
        let ops = ((CLOCK_HZ / fs).floor() as usize).min(MAX_OPERATIONS).min(max_addr / d);
        (if folded { 2 * ops } else { ops }) * d
    };
    if rem < 2 {
        // FIR1 = the RRC at the input rate, /2; the rest bypassed.
        let sps1 = fs_in / rs;
        let span = ((budget(fs_in, 2, true, NUM_ADDR[0]) - 1) as f64 / sps1) as usize;
        let h: Vec<f64> = super::rrc_taps_frac(sps1, rolloff, span).into_iter().map(|x| x as f64).collect();
        let (fir1, _, _) = quantize((h, 2), None, (vec![1.0], 1));
        let d = Design { fs_in, rs, fir1, fir2: None, fir3: None };
        check(&d)?;
        return Ok(d);
    }
    // FIR1 (and FIR2 if the rest is large): the split with the larger factor first.
    let (d1, d2) = if rem <= 8 {
        (rem.max(2), 1)
    } else {
        let d2 = (2..=rem).rev().filter(|d| rem % d == 0 && d * d <= rem).next().unwrap_or(1);
        (rem / d2, d2)
    };
    if d1 * d2 != rem || d1 < 2 {
        return Err(format!("no FIR split for decimation {rem}"));
    }
    let mut fs = fs_in;
    let fir1 = {
        let fs_next = fs / d1 as f64;
        let h = lowpass(fs, edge, fs_next - edge, budget(fs, d1, true, NUM_ADDR[0]));
        fs = fs_next;
        (h, d1)
    };
    let fir2 = (d2 > 1).then(|| {
        let fs_next = fs / d2 as f64;
        let h = lowpass(fs, edge, fs_next - edge, budget(fs, d2, false, NUM_ADDR[1]));
        fs = fs_next;
        (h, d2)
    });
    // 4 samples per symbol into FIR3, or a little more when fs_in / rs is
    // not a multiple of 4.
    let sps3 = fs / rs;
    let span = RRC_SPAN.min(((budget(fs, 2, true, NUM_ADDR[2]) - 1) as f64 / sps3) as usize);
    let fir3: Vec<f64> = super::rrc_taps_frac(sps3, rolloff, span).into_iter().map(|x| x as f64).collect();
    let (fir1, fir2, fir3) = quantize(fir1, fir2, (fir3, 2));
    let d = Design { fs_in, rs, fir1, fir2, fir3: Some(fir3) };
    check(&d)?;
    Ok(d)
}

/// maia-httpd's scaling: each stage's integer taps sum (in absolute value,
/// through the cascade) to its output growth, capped by the coefficient range.
fn quantize(h1: (Vec<f64>, usize), h2: Option<(Vec<f64>, usize)>, h3: (Vec<f64>, usize)) -> (Stage, Option<Stage>, Stage) {
    let max_coeff = ((1i64 << (COEFF_BITS - 1)) - 1) as f64;
    let growth = |i: usize| MACC_TRUNC[i] + WIDTH_GROWTH[i];
    let max_abs = |h: &[f64]| h.iter().fold(0f64, |m, x| m.max(x.abs()));
    let sum_abs = |h: &[f64]| h.iter().map(|x| x.abs()).sum::<f64>();
    let scale = |h: &[f64], s: f64| h.iter().map(|x| (x * s).round()).collect::<Vec<f64>>();
    let zero_pack = |h: &[f64], d: usize| {
        let mut v = vec![0.0; (h.len() - 1) * d + 1];
        for (i, x) in h.iter().enumerate() {
            v[i * d] = *x;
        }
        v
    };
    let conv = |a: &[f64], b: &[f64]| {
        let mut v = vec![0.0; a.len() + b.len() - 1];
        for (i, x) in a.iter().enumerate() {
            for (j, y) in b.iter().enumerate() {
                v[i + j] += x * y;
            }
        }
        v
    };
    let to_stage = |h: &[f64], d: usize| Stage { taps: h.iter().map(|&x| x as i32).collect(), decimation: d };

    let (h1, d1) = h1;
    let s1 = ((1u64 << growth(0)) as f64 / sum_abs(&h1)).min(max_coeff / max_abs(&h1));
    let q1 = scale(&h1, s1);
    let (q2, d2, st2) = match h2 {
        Some((h2, d2)) => {
            let s = ((1u64 << (growth(0) + growth(1))) as f64 / sum_abs(&conv(&q1, &zero_pack(&h2, d1)))).min(max_coeff / max_abs(&h2));
            let q2 = scale(&h2, s);
            let st = to_stage(&q2, d2);
            (q2, d2, Some(st))
        }
        None => (vec![(1u64 << growth(1)) as f64], 1, None),
    };
    let (h3, d3) = h3;
    let cascade = conv(&conv(&q1, &zero_pack(&q2, d1)), &zero_pack(&h3, d1 * d2));
    let s3 = ((1u64 << (growth(0) + growth(1) + growth(2))) as f64 / sum_abs(&cascade)).min(max_coeff / max_abs(&h3));
    let q3 = scale(&h3, s3);
    (to_stage(&q1, d1), st2, to_stage(&q3, d3))
}

/// The FPGA limits maia-httpd checks before writing a stage.
fn check(d: &Design) -> Result<(), String> {
    let mut fs = d.fs_in;
    for (i, st) in [Some(&d.fir1), d.fir2.as_ref(), d.fir3.as_ref()].into_iter().enumerate() {
        let Some(st) = st else { continue };
        let folded = i != 1;
        let ops = operations(st.taps.len(), st.decimation, folded);
        let lim = (1i32 << (COEFF_BITS - 1)) - 1;
        if st.taps.iter().any(|&c| c > lim || c < -lim - 1) {
            return Err(format!("FIR{}: coefficient out of range", i + 1));
        }
        if !(2..=MAX_DECIMATION).contains(&st.decimation) {
            return Err(format!("FIR{}: decimation {} out of range", i + 1, st.decimation));
        }
        if ops > MAX_OPERATIONS || ops as f64 * fs > CLOCK_HZ || ops * st.decimation > NUM_ADDR[i] {
            return Err(format!("FIR{}: {} taps do not fit", i + 1, st.taps.len()));
        }
        fs /= st.decimation as f64;
    }
    Ok(())
}

/// What to write to the DDC: coefficient RAM (address, value) and the
/// decimation / control register fields.
#[derive(Debug, Clone, PartialEq)]
pub struct Registers {
    pub coeffs: Vec<(u16, i32)>,
    pub decimation: [u8; 3],
    pub operations_minus_one: [u8; 3],
    pub odd_operations: [bool; 2],
    pub bypass2: bool,
    pub bypass3: bool,
}

/// Coefficient RAM image of a stage (maia-httpd `impl_set_ddc_fir`).
fn stage_ram(st: &Stage, i: usize, out: &mut Vec<(u16, i32)>) -> (u8, u8, bool) {
    let folded = i != 1;
    let d = st.decimation;
    let raw_ops = st.taps.len().div_ceil(d);
    let ops = operations(st.taps.len(), d, folded);
    let n_addr = NUM_ADDR[i];
    for addr in 0..n_addr {
        let (off, fold) = if folded && addr >= n_addr / 2 { (1, n_addr / 2) } else { (0, 0) };
        let k = (addr - fold) / ops;
        let c = if k >= d {
            0
        } else {
            let j = (addr - fold) % ops;
            let mult = if folded { 2 } else { 1 };
            let n = (mult * j + off) * d + (d - 1 - k);
            st.taps.get(n).copied().unwrap_or(0)
        };
        out.push(((addr + ADDR_OFFSET[i]) as u16, c));
    }
    (d as u8, (ops - 1) as u8, raw_ops % 2 == 1)
}

impl Design {
    pub fn registers(&self) -> Registers {
        let mut coeffs = Vec::with_capacity(640);
        let (d1, o1, odd1) = stage_ram(&self.fir1, 0, &mut coeffs);
        let (d2, o2, _) = match &self.fir2 {
            Some(st) => stage_ram(st, 1, &mut coeffs),
            None => (1, 0, false),
        };
        let (d3, o3, odd3) = match &self.fir3 {
            Some(st) => stage_ram(st, 2, &mut coeffs),
            None => (2, 0, false),
        };
        Registers {
            coeffs,
            decimation: [d1, d2, d3],
            operations_minus_one: [o1, o2, o3],
            odd_operations: [odd1, odd3],
            bypass2: self.fir2.is_none(),
            bypass3: self.fir3.is_none(),
        }
    }
}

/// NCO word that moves `hz` (relative to the LO) to baseband.
pub fn frequency_word(hz: f64, fs_in: f64) -> u32 {
    let w = (hz / fs_in * (1u64 << NCO_WIDTH) as f64).round() as i64;
    (w.rem_euclid(1i64 << NCO_WIDTH)) as u32
}

fn clamp_bits(x: i64, bits: u32) -> i64 {
    let m = 1i64 << (bits - 1);
    x.clamp(-m, m - 1)
}

/// One FIR stage as maia-hdl models it (history in, rounding, saturation).
struct FirModel {
    taps: Vec<i64>,
    d: usize,
    trunc: u32,
    /// FIR4DSP: two accumulators (even / odd tap rows), each rounded.
    split: bool,
    hist: Vec<[i64; 2]>,
    phase: usize,
}

impl FirModel {
    fn new(st: &Stage, trunc: u32, split: bool) -> Self {
        let mut taps: Vec<i64> = st.taps.iter().map(|&c| c as i64).collect();
        taps.resize(taps.len().div_ceil(st.decimation) * st.decimation, 0);
        // maia-hdl emits the first output after the first input.
        FirModel { hist: vec![[0, 0]; taps.len()], taps, d: st.decimation, trunc, split, phase: st.decimation - 1 }
    }

    fn push(&mut self, x: [i64; 2], out: &mut Vec<[i64; 2]>) {
        self.hist.rotate_left(1);
        *self.hist.last_mut().unwrap() = x;
        self.phase += 1;
        if self.phase < self.d {
            return;
        }
        self.phase = 0;
        let init = if self.trunc >= 1 { 1i64 << (self.trunc - 1) } else { 0 };
        let mut acc = [[init; 2]; 2];
        let n = self.taps.len();
        for (t, h) in self.taps.iter().enumerate() {
            // Tap n multiplies the sample n behind the newest; tap row k = n / d.
            let s = self.hist[n - 1 - t];
            let row = (t / self.d) % 2;
            let a = if self.split { row } else { 0 };
            acc[a][0] += h * s[0];
            acc[a][1] += h * s[1];
        }
        let y = if self.split {
            let f = |v: i64| clamp_bits(v >> self.trunc, OUT_WIDTH);
            [clamp_bits(f(acc[0][0]) + f(acc[1][0]), OUT_WIDTH), clamp_bits(f(acc[0][1]) + f(acc[1][1]), OUT_WIDTH)]
        } else {
            [clamp_bits(acc[0][0] >> self.trunc, OUT_WIDTH), clamp_bits(acc[0][1] >> self.trunc, OUT_WIDTH)]
        };
        out.push(y);
    }
}

/// The DDC, bit for bit: 12-bit IQ in, 16-bit IQ out.
pub struct DdcModel {
    cexp: Vec<(i64, i64)>,
    freq: u32,
    phase: u32,
    stages: Vec<FirModel>,
}

impl DdcModel {
    pub fn new(d: &Design, freq_word: u32) -> Self {
        let n = 1usize << PHASE_BITS;
        let scale = ((1i64 << (EXP_WIDTH - 1)) - 1) as f64;
        let cexp = (0..n)
            .map(|k| {
                let a = -std::f64::consts::TAU * k as f64 / n as f64;
                ((a.cos() * scale).round() as i64, (a.sin() * scale).round() as i64)
            })
            .collect();
        let mut stages = vec![FirModel::new(&d.fir1, MACC_TRUNC[0], true)];
        if let Some(st) = &d.fir2 {
            stages.push(FirModel::new(st, MACC_TRUNC[1], false));
        }
        if let Some(st) = &d.fir3 {
            stages.push(FirModel::new(st, MACC_TRUNC[2], true));
        }
        DdcModel { cexp, freq: freq_word, phase: 0, stages }
    }

    pub fn process(&mut self, iq: &[[i16; 2]], out: &mut Vec<[i16; 2]>) {
        let a = self.mix(iq);
        out.extend(self.filter(a).into_iter().map(|[re, im]| [re as i16, im as i16]));
    }

    /// The mixer alone (12-bit out).
    fn mix(&mut self, iq: &[[i16; 2]]) -> Vec<[i64; 2]> {
        let trunc = EXP_WIDTH - 1;
        let round = 1i64 << (trunc - 1);
        let mask = (1u32 << NCO_WIDTH) - 1;
        let mut a = Vec::with_capacity(iq.len());
        for &[re, im] in iq {
            let (c, s) = self.cexp[(self.phase >> (NCO_WIDTH - PHASE_BITS)) as usize];
            self.phase = (self.phase + self.freq) & mask;
            let (re, im) = (re as i64, im as i64);
            a.push([
                clamp_bits((re * c - im * s + round) >> trunc, IN_WIDTH),
                clamp_bits((re * s + im * c + round) >> trunc, IN_WIDTH),
            ]);
        }
        a
    }

    /// The FIR stages alone.
    fn filter(&mut self, mut a: Vec<[i64; 2]>) -> Vec<[i64; 2]> {
        for st in &mut self.stages {
            let mut b = Vec::with_capacity(a.len() / st.d + 1);
            for x in a {
                st.push(x, &mut b);
            }
            a = b;
        }
        a
    }
}

/// 16-bit DDC output as the receiver's complex samples (full scale 1.0).
pub fn to_complex(iq: &[[i16; 2]], out: &mut Vec<Complex32>) {
    out.extend(iq.iter().map(|&[re, im]| Complex32::new(re as f32 / 32768.0, im as f32 / 32768.0)));
}

#[cfg(test)]
mod tests {
    use super::super::{Modulator, Params, Rate, TS_LEN, rx::Receiver, symsync::SymSync, hdrdet::HdrDet};
    use super::*;

    const FS: f64 = 3_072_000.0;

    #[test]
    fn designs_fit_the_fpga_for_every_offered_rate() {
        for rs in [32e3, 33e3, 48e3, 64e3, 66e3, 96e3, 125e3, 128e3, 192e3, 250e3, 256e3, 333e3, 384e3, 500e3] {
            let d = design(FS, rs, 0.35).unwrap_or_else(|e| panic!("{rs}: {e}"));
            let sps = d.fs_out() / rs;
            assert!((2.0..4.0).contains(&sps), "{rs}: {sps}");
            let r = d.registers();
            assert_eq!(r.coeffs.len(), 256 + d.fir2.as_ref().map_or(0, |_| 128) + d.fir3.as_ref().map_or(0, |_| 256), "{rs}");
        }
        assert!(design(FS, 1e6, 0.35).is_err(), "1 MS/s: under 2 samples a symbol");
    }

    /// Test vectors for maia-hdl (`test/test_datv_ddc.py`): the designs,
    /// the coefficient RAM image and registers, ADC input, and this model's
    /// mixer and filter outputs. `DDC_VECTORS=<file> cargo test dump -- --ignored`
    #[test]
    #[ignore]
    fn dump_vectors_for_the_hdl() {
        let path = std::env::var("DDC_VECTORS").expect("DDC_VECTORS=<file>");
        let mut cases = Vec::new();
        // The third case is full scale (the ADC clipping a strong signal):
        // I and Q both near +-2047 rotate past 12 bits in the mixer, where
        // the HDL saturates as this model does (it used to wrap).
        for (rs, center, full) in [(256e3, 100e3, false), (64e3, -37_500.0, false), (256e3, 100e3, true)] {
            let d = design(FS, rs, 0.35).unwrap();
            let r = d.registers();
            let fw = frequency_word(center, FS);
            // Noise plus a tone in the channel, near full scale on peaks.
            let mut seed = 3u64;
            let input: Vec<[i16; 2]> = (0..20_000)
                .map(|k| {
                    seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                    let n = ((seed >> 40) as i64 & 0x3FF) - 512;
                    let p = std::f64::consts::TAU * (center + rs / 5.0) * k as f64 / FS;
                    if full {
                        // a square-ish clipped tone: both rails most of the time
                        let c = |v: f64| (3000.0 * v + n as f64).clamp(-2048.0, 2047.0) as i16;
                        [c(p.cos()), c(p.sin())]
                    } else {
                        [(1200.0 * p.cos()) as i16 + n as i16, (1200.0 * p.sin()) as i16 - (n / 2) as i16]
                    }
                })
                .collect();
            let mut m = DdcModel::new(&d, fw);
            let mixed = m.mix(&input);
            let output = m.filter(mixed.clone());
            cases.push(serde_json::json!({
                "rs": rs, "freq_word": fw,
                "decimation": r.decimation, "operations_minus_one": r.operations_minus_one,
                "odd_operations": r.odd_operations, "bypass2": r.bypass2, "bypass3": r.bypass3,
                "coeffs": r.coeffs, "input": input, "mixed": mixed, "output": output,
                "taps": [&d.fir1.taps, &d.fir2.as_ref().map(|s| s.taps.clone()).unwrap_or_default(), &d.fir3.as_ref().map(|s| s.taps.clone()).unwrap_or_default()],
            }));
        }
        std::fs::write(path, serde_json::to_vec(&cases).unwrap()).unwrap();
    }

    /// Full-scale I and Q rotate beyond 12 bits: the mixer (and the HDL)
    /// saturate instead of wrapping to the other rail.
    #[test]
    fn full_scale_input_saturates() {
        let d = design(FS, 256e3, 0.35).unwrap();
        let mut m = DdcModel::new(&d, frequency_word(FS / 8.0, FS));
        let x = vec![[2047i16, 2047i16]; 64];
        let mixed = m.mix(&x);
        let lim = (1i64 << (IN_WIDTH - 1)) - 1;
        assert!(mixed.iter().any(|v| v[0] == lim || v[1] == lim || v[0] == -lim - 1 || v[1] == -lim - 1));
        // No sign flip: at 45 degrees the rotated sample is (0, 2895) ->
        // (0, 2047), never a large negative value.
        for (k, v) in mixed.iter().enumerate() {
            let a = -std::f64::consts::TAU * k as f64 / 8.0;
            let (re, im) = (2047.0 * (a.cos() - a.sin()), 2047.0 * (a.sin() + a.cos()));
            for (got, want) in [(v[0], re), (v[1], im)] {
                assert!((got as f64 - want.clamp(-2048.0, 2047.0)).abs() <= 2.0, "{k}: {got} vs {want}");
            }
        }
    }

    #[test]
    fn coefficient_ram_follows_maia_httpd() {
        // 5 taps, decimation 2, folded: 3 raw operations -> 2, odd.
        let st = Stage { taps: vec![1, 2, 3, 4, 5], decimation: 2 };
        let mut ram = Vec::new();
        let (d, opm1, odd) = stage_ram(&st, 0, &mut ram);
        assert_eq!((d, opm1, odd), (2, 1, true));
        let at = |a: usize| ram[a].1;
        // First half: rows j = 0, 1 of even tap pairs; k = 0 is the later phase.
        assert_eq!([at(0), at(1), at(2), at(3)], [2, 0, 1, 5]);
        // Second half (off = 1): taps 3, 4 and nothing.
        assert_eq!([at(128), at(129), at(130), at(131)], [4, 0, 3, 0]);
    }

    #[test]
    fn nco_and_filters_pass_a_tone_and_stop_its_alias() {
        let d = design(FS, 256e3, 0.35).unwrap();
        let run = |hz: f64| {
            let mut m = DdcModel::new(&d, frequency_word(100e3, FS));
            let x: Vec<[i16; 2]> = (0..30_000)
                .map(|k| {
                    let p = std::f64::consts::TAU * hz * k as f64 / FS;
                    [(1000.0 * p.cos()).round() as i16, (1000.0 * p.sin()).round() as i16]
                })
                .collect();
            let mut y = Vec::new();
            m.process(&x, &mut y);
            let tail = &y[y.len() / 2..];
            (tail.iter().map(|v| (v[0] as f64).powi(2) + (v[1] as f64).powi(2)).sum::<f64>() / tail.len() as f64).sqrt()
        };
        let pass = run(100e3 + 50e3); // in band
        let stop = run(100e3 + 700e3); // far out, would alias without the filters
        assert!(pass > 1000.0, "pass-band level {pass}");
        assert!(20.0 * (pass / stop.max(1e-9)).log10() > 50.0, "pass {pass} stop {stop}");
    }

    /// Modulate at the ADC rate, add noise and an offset, quantize to the
    /// ADC's 12 bits, run the DDC model, receive at 2 samples per symbol.
    fn link(rs: f64, rate: Rate, esn0_db: f32, frames: usize, fpga: bool) -> (super::super::rx::Stats, usize) {
        let p = Params { rate, pilots: true, rolloff: 0.35 };
        let sps = (FS / rs).round() as usize;
        let mut m = Modulator::new(p, sps);
        let mut n = 0u32;
        let mut next = || {
            let mut pkt = [0u8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1..5].copy_from_slice(&n.to_be_bytes());
            n += 1;
            pkt
        };
        let mut iq = vec![Complex32::default(); frames * p.frame_symbols() * sps];
        m.fill(&mut iq, &mut next);
        let esn0 = 10f32.powf(esn0_db / 10.0);
        let sigma = (sps as f32 / esn0 / 2.0).sqrt();
        let mut seed = 11u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        // 100 kHz above the LO, the transmitter 300 Hz off; a weak signal
        // (about -30 dBFS) in the ADC.
        let (center, err) = (100e3, 300.0);
        let amp = 60.0;
        let adc: Vec<[i16; 2]> = iq
            .iter()
            .enumerate()
            .map(|(k, z)| {
                let ph = std::f64::consts::TAU * (center + err) * k as f64 / FS;
                let v = (*z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g())) * amp;
                [v.re.round().clamp(-2048.0, 2047.0) as i16, v.im.round().clamp(-2048.0, 2047.0) as i16]
            })
            .collect();
        let d = design(FS, rs, 0.35).unwrap();
        let mut ddc = DdcModel::new(&d, frequency_word(center, FS));
        let mut rx = if fpga { Receiver::new_prefiltered(p, d.fs_out(), rs, 0.0) } else { Receiver::new(p, FS, rs, center) };
        let mut out = Vec::new();
        for c in adc.chunks(30_720) {
            let (mut y, mut z) = (Vec::new(), Vec::new());
            if fpga {
                ddc.process(c, &mut y);
                to_complex(&y, &mut z);
            } else {
                z.extend(c.iter().map(|&[re, im]| Complex32::new(re as f32 / 2048.0, im as f32 / 2048.0)));
            }
            rx.process(&z, &mut out);
        }
        let data = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).count();
        (rx.stats, data)
    }

    /// The FPGA path is as good as the all-software front end on the same
    /// ADC samples: the same Es/N0 after it (no implementation loss from the
    /// fixed point), and every frame after acquisition decoded.
    fn same_as_software(rs: f64, rate: Rate, esn0_db: f32, frames: usize) {
        let (h, hd) = link(rs, rate, esn0_db, frames, true);
        let (s, sd) = link(rs, rate, esn0_db, frames, false);
        eprintln!("{rs} {rate:?} {esn0_db} dB  DDC: {hd} packets {h:?}\n  software: {sd} packets {s:?}");
        assert!(h.locked && (h.freq_hz - 300.0).abs() < 10.0, "{h:?}");
        assert!(h.frames as usize >= frames - 3, "{h:?}");
        assert!((h.data_esn0_db - s.data_esn0_db).abs() < 0.3, "DDC {h:?}\nsoftware {s:?}");
        assert!(h.frames_bad <= 2, "only acquisition may cost frames: {h:?}");
        assert!(hd + 2 * rate_packets(rate) >= sd, "DDC {hd} vs software {sd} packets");
    }
    fn rate_packets(rate: Rate) -> usize {
        Params { rate, pilots: true, rolloff: 0.35 }.payload_bits() / (8 * TS_LEN)
    }

    /// CPU per second of signal with the DDC doing the filtering, for the
    /// A9 (run the test binary on the board):
    /// `trxd-<hash> ddc_path_cpu --ignored --nocapture`
    #[test]
    #[ignore]
    fn ddc_path_cpu() {
        for (rs, rate, esn0) in [(64e3, Rate::R1_2, 5.0), (128e3, Rate::R1_2, 5.0), (256e3, Rate::R1_2, 5.0), (256e3, Rate::R3_4, 9.0)] {
            let frames = 60;
            let (s, _) = link(rs, rate, esn0, frames, true);
            let secs = frames as f64 * Params { rate, pilots: true, rolloff: 0.35 }.frame_symbols() as f64 / rs;
            let (sw, _) = link(rs, rate, esn0, frames, false);
            eprintln!(
                "{rs:>6} S/s {rate:?} {esn0} dB: per second of signal: demod {:.0} % of a core with the DDC ({:.0} % without), LDPC {:.0} %; {} frames, {} bad",
                100.0 * s.other_s / secs, 100.0 * sw.other_s / secs, 100.0 * s.ldpc_s / secs, s.frames, s.frames_bad
            );
        }
    }

    #[test]
    fn qpsk_3_4_at_256_ksps_through_the_ddc() {
        same_as_software(256e3, Rate::R3_4, 7.5, 40);
    }

    #[test]
    fn qpsk_1_2_at_64_ksps_through_the_ddc() {
        same_as_software(64e3, Rate::R1_2, 3.5, 30);
    }

    #[test]
    fn qpsk_1_2_at_128_ksps_through_the_ddc() {
        same_as_software(128e3, Rate::R1_2, 3.5, 30);
    }

    /// Long frames (pilots) at a symbol rate that does not divide the ADC
    /// rate: pulse-shaped at 3.072 MS/s from the continuous RRC, then the
    /// DDC (fractional samples per symbol out) and the long-frame receiver.
    fn long_link_ddc(rs: f64, mode: super::super::fpga_tx::LongMode, esn0_db: f32, frames: usize, symsync: bool) -> (super::super::rx::Stats, usize) {
        long_link_ddc_at(rs, mode, esn0_db, frames, symsync, 300.0, 0.0)
    }

    /// [`long_link_ddc`] with the carrier `err` Hz off, moving `drift` Hz/s.
    fn long_link_ddc_at(rs: f64, mode: super::super::fpga_tx::LongMode, esn0_db: f32, frames: usize, symsync: bool, err: f64, drift: f64) -> (super::super::rx::Stats, usize) {
        use super::super::{FrameSpec, rrc_at, rx::tests::{counter_packets, long_symbols}};
        let spec = FrameSpec::long(mode);
        let mut next = counter_packets();
        let syms = long_symbols(mode, frames, &mut next);
        // RRC table, 256 points a symbol over 12 symbols (nearest point).
        const RES: usize = 256;
        const SPAN: usize = 12;
        let tab: Vec<f32> = (0..=SPAN * RES).map(|i| rrc_at(i as f64 / RES as f64 - (SPAN / 2) as f64, spec.rolloff as f64) as f32).collect();
        let sps = FS / rs;
        let n_out = ((syms.len() - SPAN) as f64 * sps) as usize;
        let esn0 = 10f32.powf(esn0_db / 10.0);
        // Pulse energy per symbol: sum of h^2 at sps samples a symbol.
        let e: f32 = tab.iter().step_by(1).map(|h| h * h).sum::<f32>() / RES as f32 * sps as f32;
        let sigma = (e / esn0 / 2.0).sqrt();
        let mut seed = 23u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        let center = 100e3;
        let amp = 1200.0 / (e.sqrt() + 1.0);
        let mut rx = None;
        let mut ss: Option<SymSync> = None;
        let mut hd = HdrDet::default();
        let d = design(FS, rs, spec.rolloff).unwrap();
        let mut ddc = DdcModel::new(&d, frequency_word(center, FS));
        let mut out = Vec::new();
        let mut adc = Vec::with_capacity(30_720);
        for k in 0..n_out {
            // Symbols whose pulse covers sample k (time in symbols).
            let t = k as f64 / sps + (SPAN / 2) as f64;
            let mut z = Complex32::default();
            let i0 = t.ceil() as usize - SPAN / 2;
            for i in i0..=(t.floor() as usize + SPAN / 2).min(syms.len() - 1) {
                let x = ((t - i as f64 + (SPAN / 2) as f64) * RES as f64).round() as usize;
                if x <= SPAN * RES {
                    z += syms[i] * tab[x];
                }
            }
            let tk = k as f64 / FS;
            let ph = std::f64::consts::TAU * ((center + err) * tk + drift * tk * tk / 2.0);
            let v = (z * Complex32::new(ph.cos() as f32, ph.sin() as f32) + Complex32::new(sigma * g(), sigma * g())) * amp;
            adc.push([v.re.round().clamp(-2048.0, 2047.0) as i16, v.im.round().clamp(-2048.0, 2047.0) as i16]);
            if adc.len() == 30_720 || k + 1 == n_out {
                let (mut y, mut c) = (Vec::new(), Vec::new());
                ddc.process(&adc, &mut y);
                if symsync {
                    let ss = ss.get_or_insert_with(|| SymSync::new(super::super::symsync::Params::new(d.fs_out(), rs)));
                    let mut sy: Vec<[i16; 2]> = y.iter().filter_map(|&x| ss.push(x)).collect();
                    // And the header detector, flags as the ring carries them.
                    let fl: Vec<bool> = sy
                        .iter_mut()
                        .map(|y| {
                            let f = hd.push(*y);
                            *y = HdrDet::mark(*y, f);
                            f
                        })
                        .collect();
                    to_complex(&sy, &mut c);
                    rx.get_or_insert_with(|| Receiver::new_symbols_spec(spec, rs, 0.0)).process_flagged(&c, Some(&fl), &mut out);
                } else {
                    to_complex(&y, &mut c);
                    rx.get_or_insert_with(|| Receiver::new_prefiltered_spec(spec, d.fs_out(), rs, 0.0)).process(&c, &mut out);
                }
                adc.clear();
            }
        }
        let rx = rx.unwrap();
        let data: Vec<_> = out.iter().filter(|p| p[1..3] != [0x1F, 0xFF]).collect();
        let f0 = data.first().map_or(0, |p| u32::from_be_bytes(p[1..5].try_into().unwrap()));
        // (In order, with no gap: lost frames show in the stats. Under drift
        // the probe below wants the stats.)
        for (i, p) in data.iter().enumerate().filter(|_| drift == 0.0) {
            assert_eq!(u32::from_be_bytes(p[1..5].try_into().unwrap()), f0 + i as u32, "{mode:?} packet {i}; {:?}", rx.stats);
        }
        (rx.stats, data.len())
    }

    #[test]
    fn long_frames_at_250_ksps_through_the_ddc() {
        use super::super::fpga_tx::LongMode;
        for (mode, esn0, per_frame) in [(LongMode::Qpsk12, 3.0, 21), (LongMode::Psk8_34, 10.0, 32)] {
            let (s, n) = long_link_ddc(250e3, mode, esn0, 22, false);
            eprintln!("{mode:?}: {n} packets {s:?}");
            assert!(s.locked && (s.freq_hz - 300.0).abs() < 10.0, "{s:?}");
            // At 3 dB the header alone gives the frequency to about 100 Hz
            // at 250 kS/s, more than half the pilots' alias spacing (169 Hz):
            // the first frames of the acquisition average may fail. From
            // there on every frame.
            assert!(n >= 12 * per_frame && s.ldpc_fail <= 8, "{mode:?}: {n} packets, {s:?}");
        }
    }

    /// The same through the FPGA's timing recovery and header detector
    /// (their models) and the receiver on flagged symbols.
    #[test]
    fn long_frames_at_250_ksps_through_the_ddc_and_symsync() {
        use super::super::fpga_tx::LongMode;
        for (mode, esn0, per_frame) in [(LongMode::Qpsk12, 3.0, 21), (LongMode::Psk8_34, 10.0, 32)] {
            let (s, n) = long_link_ddc(250e3, mode, esn0, 22, true);
            eprintln!("{mode:?} symsync: {n} packets {s:?}");
            assert!(s.locked && (s.freq_hz - 300.0).abs() < 10.0, "{s:?}");
            assert!(n >= 12 * per_frame && s.ldpc_fail <= 8, "{mode:?}: {n} packets, {s:?}");
        }
    }

    /// A DDC recording from a board (`touch /tmp/datv-iq`) through the
    /// receiver: `DATV_CAP=<file.cf32> DATV_SR=250000 DATV_MODE=L-QPSK-1/2
    /// cargo test --release replay_capture -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn replay_capture() {
        use super::super::{FrameSpec, fpga_tx::LongMode};
        let path = std::env::var("DATV_CAP").expect("DATV_CAP=<file.cf32>");
        let rs: f64 = std::env::var("DATV_SR").map_or(250e3, |v| v.parse().unwrap());
        let mode = LongMode::parse(&std::env::var("DATV_MODE").unwrap_or("L-QPSK-1/2".into())).unwrap();
        let d = design(FS, rs, 0.35).unwrap();
        let raw = if path == "noise" {
            let mut seed = 5u64;
            (0..6_000_000 * 2).flat_map(|_| {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                (((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.01).to_le_bytes()
            }).collect()
        } else {
            std::fs::read(path).unwrap()
        };
        let iq: Vec<Complex32> = raw.chunks_exact(8).map(|c| Complex32::new(f32::from_le_bytes(c[..4].try_into().unwrap()), f32::from_le_bytes(c[4..].try_into().unwrap()))).collect();
        let p = iq.iter().map(|z| z.norm_sqr()).sum::<f32>() / iq.len() as f32;
        eprintln!("{} samples at {} S/s ({:.1} s), mean power {p:.3e}", iq.len(), d.fs_out(), iq.len() as f64 / d.fs_out());
        // DATV_SYMSYNC=1: through the timing recovery model first.
        // DATV_HDRDET=1: and the header detector's model, flags to the receiver.
        let hdr = std::env::var_os("DATV_HDRDET").is_some();
        let symsync = hdr || std::env::var_os("DATV_SYMSYNC").is_some();
        let mut hd = HdrDet::default();
        let (mut nflags, mut nsyms) = (0usize, 0usize);
        let mut ss = SymSync::new(super::super::symsync::Params::new(d.fs_out(), rs));
        let mut rx = if symsync { Receiver::new_symbols_spec(FrameSpec::long(mode), rs, 0.0) } else { Receiver::new_prefiltered_spec(FrameSpec::long(mode), d.fs_out(), rs, 0.0) };
        let mut out = Vec::new();
        let t0 = std::time::Instant::now();
        let chunk: usize = std::env::var("DATV_CHUNK").map_or(16384, |v| v.parse().unwrap());
        for (k, c) in iq.chunks(chunk).enumerate() {
            if symsync {
                let q = |v: f32| (v * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
                let mut sy: Vec<[i16; 2]> = c.iter().filter_map(|z| ss.push([q(z.re), q(z.im)])).collect();
                let mut cs = Vec::new();
                if hdr {
                    // As the ring delivers them: flag in bit 0 of im.
                    for y in sy.iter_mut() {
                        let f = hd.push(*y);
                        *y = HdrDet::mark(*y, f);
                    }
                    let fl: Vec<bool> = sy.iter().map(|y| y[1] & 1 == 1).collect();
                    nflags += fl.iter().filter(|&&f| f).count();
                    nsyms += fl.len();
                    to_complex(&sy, &mut cs);
                    rx.process_flagged(&cs, Some(&fl), &mut out);
                } else {
                    to_complex(&sy, &mut cs);
                    rx.process(&cs, &mut out);
                }
            } else {
                rx.process(c, &mut out);
            }
            if k % (655_360 / chunk) == 0 {
                eprintln!("{:.1} s: {:?}", (k * chunk) as f64 / d.fs_out(), rx.stats);
            }
        }
        eprintln!("done in {:.2} s: {} packets {:?}", t0.elapsed().as_secs_f64(), out.len(), rx.stats);
        if symsync {
            eprintln!("symsync omega {:.6} (nominal {:.6})", ss.omega(), d.fs_out() / rs);
        }
        if hdr {
            eprintln!("hdrdet: {nflags} flags in {nsyms} symbols ({:.3} %)", 100.0 * nflags as f64 / nsyms.max(1) as f64);
        }
    }

    /// A carrier offset of a few percent of the symbol rate (a 0.2 ppm
    /// crystal at 3.4 GHz is 650 Hz, 2 % of 33 kS/s): the 90-symbol header
    /// turns through more than a cycle while acquisition holds the NCO.
    /// Measured as it came, its phase flipped and the receiver settled an
    /// alias (rs / 1476, 22 Hz) off, locked with nothing decoding (seen on
    /// air, 8PSK 3/4 33 kS/s at 2330 and 3410 MHz).
    #[test]
    fn psk8_slow_rate_large_offset() {
        use super::super::fpga_tx::LongMode;
        for err in [650.0, 1000.0] {
            let (s, n) = long_link_ddc_at(33e3, LongMode::Psk8_34, 20.0, 14, true, err, 0.0);
            eprintln!("{err} Hz: {n} packets, {} bad, freq {:.1}, Es/N0 {:.1}", s.frames_bad, s.freq_hz, s.esn0_db);
            assert!(s.locked && (s.freq_hz as f64 - err).abs() < 3.0 && n >= 10 * 32, "{err} Hz: {n} packets, {s:?}");
        }
        // And drifting as a transmitter just keyed does (5 Hz/s seen at
        // 3.4 GHz): acquisition ends early on a strong signal, tracking
        // learns the drift.
        let (s, n) = long_link_ddc_at(33e3, LongMode::Psk8_34, 20.0, 30, true, 700.0, -5.0);
        eprintln!("-5 Hz/s: {n} packets, {} bad of {}", s.frames_bad, s.frames);
        assert!(s.locked && s.frames_bad <= 2 && n >= 26 * 32, "-5 Hz/s: {n} packets, {s:?}");
    }

    /// Probe: 8PSK 3/4 at 33 kS/s, 700 Hz off, drifting DATV_DRIFT Hz/s
    /// (default -5). Before the fixes -5 Hz/s lost 17 of 29 frames; now
    /// -10 Hz/s loses 5 (acquisition), -20 Hz/s still fails.
    #[test]
    #[ignore]
    fn psk8_slow_drift() {
        use super::super::fpga_tx::LongMode;
        let drift: f64 = std::env::var("DATV_DRIFT").map_or(-5.0, |v| v.parse().unwrap());
        let (s, n) = long_link_ddc_at(33e3, LongMode::Psk8_34, 20.0, 30, true, 700.0, drift);
        eprintln!("drift {drift} Hz/s: {n} packets, {} bad of {}, freq {:.1}, Es/N0 {:.1}, data {:.1}", s.frames_bad, s.frames, s.freq_hz, s.esn0_db, s.data_esn0_db);
    }

    /// The amateur standard symbol rates through the DDC and the FPGA's
    /// timing recovery and header detector (models), long frames QPSK 1/2.
    #[test]
    fn standard_rates_through_the_ddc() {
        use super::super::fpga_tx::LongMode;
        for rs in [33e3, 66e3, 125e3, 250e3, 333e3, 500e3] {
            let (s, n) = long_link_ddc(rs, LongMode::Qpsk12, 5.0, 20, true);
            eprintln!("{rs}: {n} packets, {} bad, freq {:.0}", s.frames_bad, s.freq_hz);
            assert!(s.locked && n >= 10 * 21, "{rs}: {n} packets, {s:?}");
        }
    }
}
