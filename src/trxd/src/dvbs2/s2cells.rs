//! DVB-S2 long frames as cells for the FPGA's LDPC engine (maia-sdr
//! `ldpc_dma.py`): the receiver hands over each frame's data symbols,
//! derotated and descrambled, as 8-bit I/Q cells plus one noise scale `kq`;
//! the engine makes the LLRs (QPSK: the DVB-T2 QPSK path unrotated; 8PSK:
//! max-log over the eight points and the 3-column bit deinterleaver) and
//! writes them straight into the decoder. This is its bit-exact model (the
//! decoder's input where there is no engine, and the tests).
//!
//! Scaling: a cell is the symbol times [`gain`] (the constellation's radius
//! at [`RADIUS`]), so `kq` carries the LLR scale of the frame:
//! LLR (6 bits) = round(z kq / 2^17) with z the axis (QPSK) or max-log
//! difference (8PSK) in Q14 of the cell, z >> 7 first, as the engine does.

use num_complex::Complex32;

use super::fpga_ldpc::LLR_SCALE;

/// The constellation's radius in cell units (QPSK points at +-45, 8PSK on
/// the same circle): room up to 127 for noise twice the signal.
pub const RADIUS: f32 = 64.0;

/// Cell units per symbol unit for a frame whose points have amplitude `amp`.
pub fn gain(amp: f32) -> f32 {
    RADIUS / amp.max(1e-9)
}

/// One symbol as a cell (rounded, saturated).
#[inline]
pub fn cell(d: Complex32, g: f32) -> [i8; 2] {
    // Plain compares and the saturating cast (no libm round on the A9):
    // +-0.5 toward the sign, then truncation.
    let q = |v: f32| {
        let x = v * g;
        let x = if x > 127.0 { 127.0 } else if x < -128.0 { -128.0 } else { x };
        (x + if x < 0.0 { -0.5 } else { 0.5 }) as i8
    };
    [q(d.re), q(d.im)]
}

/// The frame's `kq`: the receiver's LLR is `scale_qpsk * d.re` (QPSK, with
/// scale 2 sqrt2 amp / sigma2) or `2 amp (c0 - c1) / sigma2` (8PSK, c the
/// correlations with unit points); in cells, times LLR_SCALE, over the
/// engine's 1024 (= 2^17 / 2^7).
pub fn kq(bps: usize, amp: f32, sigma2: f32) -> i32 {
    kq_at(bps, amp, sigma2, gain(amp))
}

/// [`kq`] for cells made at gain `g` (cell units per symbol unit) rather
/// than [`gain`]'s (the ring path's engine gain saturates for very weak
/// signals; s2ring).
pub fn kq_at(bps: usize, amp: f32, sigma2: f32, g: f32) -> i32 {
    let llr_per_cell = if bps == 2 {
        2.0 * std::f32::consts::SQRT_2 * amp / sigma2
    } else {
        2.0 * amp / sigma2
    } / g;
    let k = 1024.0 * LLR_SCALE * llr_per_cell;
    if k.is_nan() { 0 } else { k.round().clamp(0.0, 131_071.0) as i32 }
}

#[inline]
fn q6(z: i32, kq: i32) -> i8 {
    let v = (((z >> 7) as i64 * kq as i64 + (1 << 16)) >> 17) as i32;
    v.clamp(-31, 31) as i8
}

/// 8PSK cells -> the LLRs in codeword order (bit m of cell j: variable
/// m rows + j), bit for bit as the engine.
pub fn psk8_llrs(cells: &[[i8; 2]], kq: i32) -> Vec<i8> {
    let rows = cells.len();
    let mut out = vec![0i8; 3 * rows];
    let (k, h) = (16384i32, 11585i32);
    // the points with bit m 0 / 1, as angle indices (labels via PSK8_PHASE)
    let groups: [([usize; 4], [usize; 4]); 3] = std::array::from_fn(|b| {
        let mask = 4 >> b;
        let mut g0 = [0usize; 4];
        let mut g1 = [0usize; 4];
        let (mut i0, mut i1) = (0, 0);
        for v in 0..8 {
            let a = super::PSK8_PHASE[v] as usize;
            if v & mask == 0 {
                g0[i0] = a;
                i0 += 1;
            } else {
                g1[i1] = a;
                i1 += 1;
            }
        }
        (g0, g1)
    });
    for (j, c) in cells.iter().enumerate() {
        let (i, q) = (c[0] as i32, c[1] as i32);
        let corr = [i * k, (i + q) * h, q * k, (q - i) * h, -i * k, -(i + q) * h, -q * k, (i - q) * h];
        for (b, (g0, g1)) in groups.iter().enumerate() {
            let z0 = g0.iter().map(|&a| corr[a]).max().unwrap_or(0);
            let z1 = g1.iter().map(|&a| corr[a]).max().unwrap_or(0);
            out[b * rows + j] = q6(z0 - z1, kq);
        }
    }
    out
}

/// QPSK cells -> LLRs in codeword order (2 j from I, 2 j + 1 from Q).
pub fn qpsk_llrs(cells: &[[i8; 2]], kq: i32) -> Vec<i8> {
    let mut out = vec![0i8; 2 * cells.len()];
    for (j, c) in cells.iter().enumerate() {
        out[2 * j] = q6(c[0] as i32 * 16384, kq);
        out[2 * j + 1] = q6(c[1] as i32 * 16384, kq);
    }
    out
}

/// The engine's parameters for a frame of `bps` (2 or 3) cells.
pub fn params(bps: usize, kq: i32) -> crate::dvbt2::stream::CellParams {
    crate::dvbt2::stream::CellParams { rot: false, kq, c14: 16384, s14: 0, qam16: None, a14: 0, psk8: bps == 3 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbs2::ldpc_fpga::quantize_llr;

    /// The cell path against the float LLRs it replaces: the same signs, and
    /// within quantization of each other where neither saturates.
    #[test]
    fn cells_follow_the_float_llrs() {
        let mut seed = 7u32;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f32 / u32::MAX as f32 - 0.5
        };
        let amp = 0.37f32;
        let sigma2 = 0.02f32;
        let a = std::f32::consts::FRAC_1_SQRT_2;
        let g = gain(amp);
        // QPSK
        let scale = 2.0 * std::f32::consts::SQRT_2 * amp / sigma2;
        let k2 = kq(2, amp, sigma2);
        let (mut worst, mut n) = (0i32, 0);
        for _ in 0..4000 {
            let d = Complex32::new(amp * a * if rnd() > 0.0 { 1.0 } else { -1.0 } + 0.3 * amp * rnd(), 0.3 * amp * rnd());
            let c = cell(d, g);
            let l = qpsk_llrs(&[c], k2)[0] as i32;
            let f = quantize_llr(scale * d.re, LLR_SCALE) as i32;
            if f.abs() < 31 && l.abs() < 31 {
                worst = worst.max((l - f).abs());
                n += 1;
            }
        }
        assert!(n > 100 && worst <= 1, "QPSK: worst {worst} over {n}");
        // 8PSK
        let k3 = kq(3, amp, sigma2);
        let pts: Vec<Complex32> = (0..8).map(|v| {
            let t = std::f32::consts::FRAC_PI_4 * crate::dvbs2::PSK8_PHASE[v] as f32;
            Complex32::new(t.cos(), t.sin())
        }).collect();
        let (mut worst, mut n) = (0i32, 0);
        for s in 0..4000 {
            let d = pts[s % 8] * amp + Complex32::new(0.2 * amp * rnd(), 0.2 * amp * rnd());
            let c = cell(d, g);
            let l = psk8_llrs(&[c], k3);
            for bit in 0..3 {
                let mask = 4 >> bit;
                let (mut d0, mut d1) = (f32::MAX, f32::MAX);
                for (v, p) in pts.iter().enumerate() {
                    let dv = (d - p * amp).norm_sqr();
                    if v & mask == 0 { d0 = d0.min(dv) } else { d1 = d1.min(dv) }
                }
                let f = quantize_llr((d1 - d0) / sigma2, LLR_SCALE) as i32;
                let q = l[bit] as i32;
                if f.abs() < 31 && q.abs() < 31 {
                    worst = worst.max((q - f).abs());
                    n += 1;
                }
            }
        }
        assert!(n > 100 && worst <= 1, "8PSK: worst {worst} over {n}");
    }

    #[test]
    fn psk8_full_scale_corners() {
        // (-128, -128) and (127, 127): no overflow, opposite signs.
        let l = psk8_llrs(&[[127, 127], [-128, -128]], 131_071);
        assert_eq!(l.len(), 6);
        for b in 0..3 {
            assert!(l[b * 2].abs() == 31 || l[b * 2] == 0, "{l:?}");
        }
    }
}
