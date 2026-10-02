//! DVB-S2 long frames straight from the receive ring (maia-sdr `s2front.py`
//! in front of the LDPC engine): the receiver fits each frame's carrier
//! from its known blocks and hands over, per data group (the data between
//! two known blocks), the angle of its first symbol and the step per
//! symbol; the engine's DMA reads the frame's symbols where the recorder
//! left them in the ring, turns them, descrambles them and makes the cells
//! ([`super::s2cells`]) itself. The CPU never touches the data symbols.
//! This is its bit-exact model (the decoder's input where there is no
//! engine, and the tests).
//!
//! Fixed point: data symbol t of group s is turned by A_s + t B_s (32 bits
//! a turn) rounded to 16 bits, plus (4 - R_j) mod 4 quarter turns (R the PL
//! scrambling sequence at the symbol's position j after the header), by a
//! 12-step CORDIC (after a +-90 degree pre-rotation) on the raw i16 I/Q;
//! the cell is (x G + 2^19) >> 20 saturated to i8, G the frame's gain with
//! the CORDIC's 1.64676 in it.

use super::{PILOT, SLOT};

/// Data symbols between two pilot blocks.
pub const GROUP: usize = 16 * SLOT;
const STAGES: usize = 12;
pub const GAIN_SHIFT: u32 = 20;
/// The CORDIC's gain after [`STAGES`] steps.
pub const CORDIC_GAIN: f64 = 1.646_760_258_121_065_6;
/// Largest gain the engine takes (17 bits).
pub const GAIN_MAX: u32 = (1 << 17) - 1;

fn atan_table() -> [i32; STAGES] {
    std::array::from_fn(|i| ((2f64).powi(-(i as i32)).atan() * 32768.0 / std::f64::consts::PI).round() as i32)
}

/// Rotate (x, y) by `th` (16 bits a turn, signed): the engine's CORDIC.
pub fn cordic(mut x: i32, mut y: i32, mut th: i32) -> (i32, i32) {
    if th >= 16384 {
        (x, y, th) = (-y, x, th - 16384);
    } else if th < -16384 {
        (x, y, th) = (y, -x, th + 16384);
    }
    let at = atan_table();
    for (i, &a) in at.iter().enumerate() {
        if th >= 0 {
            (x, y, th) = (x - (y >> i), y + (x >> i), th - a);
        } else {
            (x, y, th) = (x + (y >> i), y - (x >> i), th + a);
        }
    }
    (x, y)
}

/// The engine's gain for `g` cell units per symbol unit (symbol units: the
/// ring's i16 over 32768), and the `g` it really applies.
pub fn gain(g: f32) -> (u32, f32) {
    let k = (g as f64 * (1u64 << GAIN_SHIFT) as f64 / (32768.0 * CORDIC_GAIN)).round();
    let q = if k.is_nan() { 1 } else { k.clamp(1.0, GAIN_MAX as f64) as u32 };
    (q, (q as f64 * 32768.0 * CORDIC_GAIN / (1u64 << GAIN_SHIFT) as f64) as f32)
}

/// A turn as 32 bits (any phase in radians).
pub fn angle(rad: f64) -> u32 {
    let t = (rad / std::f64::consts::TAU).rem_euclid(1.0);
    ((t * 4_294_967_296.0) as u64 & 0xFFFF_FFFF) as u32
}

/// A small phase step in radians as 32 bits a turn (signed).
pub fn step(rad: f64) -> u32 {
    ((rad / std::f64::consts::TAU * 4_294_967_296.0).round() as i64) as u32
}

/// Symbols the frame occupies after its header: data and pilots.
pub fn frame_symbols(n_cells: usize, pilots: bool) -> usize {
    if pilots { n_cells + (n_cells.div_ceil(GROUP) - 1) * PILOT } else { n_cells }
}

/// The cells of a frame from its ring words after the header (`scramble`:
/// the PL scrambling sequence from there).
pub fn cells(words: &[u32], n_cells: usize, pilots: bool, segs: &[(u32, u32)], g: u32, scramble: &[u8]) -> Vec<[i8; 2]> {
    let sat = |v: i32| -> i8 {
        let q = ((v as i64 * g as i64 + (1 << (GAIN_SHIFT - 1))) >> GAIN_SHIFT) as i32;
        q.clamp(-128, 127) as i8
    };
    let mut out = Vec::with_capacity(n_cells);
    let (mut s, mut t, mut pos) = (0usize, 0u32, 0usize);
    for (j, &w) in words.iter().enumerate() {
        if out.len() == n_cells {
            break;
        }
        if !pilots || pos < GROUP {
            let (a, b) = segs[s];
            let acc = a.wrapping_add(t.wrapping_mul(b));
            let q = (4 - scramble[j] as u32) & 3;
            let th = ((acc.wrapping_add(0x8000) >> 16).wrapping_add(q << 14) & 0xFFFF) as u16 as i16 as i32;
            let (x, y) = cordic(w as u16 as i16 as i32, (w >> 16) as u16 as i16 as i32, th);
            out.push([sat(x), sat(y)]);
            t += 1;
        }
        pos += 1;
        if pilots && pos == GROUP + PILOT {
            pos = 0;
            s += 1;
            t = 0;
        }
    }
    out
}

/// Words of margin before the recorder comes round to a frame's start: the
/// engine reads a frame in a few ms (65 ms of margin at 500 kS/s).
pub const MARGIN: u64 = 32_768;

/// Is a frame of `nsym` symbols wholly in the ring, its start not about to
/// be overwritten, when the recorder is `ahead` words past its start?
pub fn in_ring(ahead: u64, nsym: u64) -> bool {
    ahead >= nsym && ahead + MARGIN < super::fpga::RING_WORDS
}

/// One frame for the engine: where it lies in the ring, how to turn it.
#[derive(Clone)]
pub struct Job {
    /// The ring word (absolute, since the recorder started) of the first
    /// symbol after the header.
    pub at: u64,
    pub n_cells: usize,
    pub pilots: bool,
    pub segs: Vec<(u32, u32)>,
    pub gain: u32,
    pub kq: i32,
    pub bps: u8,
    /// The frame's words when there is no engine (the model makes the
    /// cells from them): tests, or a ring the engine cannot read.
    pub words: Option<Vec<u32>>,
    /// The recorder's position (is the frame still in the ring?).
    pub watch: Option<std::sync::Arc<super::fpga::RingWatch>>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex32;

    /// The CORDIC turns like exp(j th) times its gain, to a fraction of a cell.
    #[test]
    fn cordic_turns_by_the_angle() {
        let mut seed = 5u32;
        let mut r = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            seed
        };
        for _ in 0..2000 {
            let (x, y) = ((r() >> 16) as u16 as i16 as i32, (r() >> 16) as u16 as i16 as i32);
            let th = (r() >> 16) as u16 as i16 as i32;
            let (cx, cy) = cordic(x, y, th);
            let a = th as f64 * std::f64::consts::PI / 32768.0;
            let z = Complex32::new(x as f32, y as f32) * Complex32::new(a.cos() as f32, a.sin() as f32) * CORDIC_GAIN as f32;
            let err = (Complex32::new(cx as f32, cy as f32) - z).norm();
            // 12 steps: about 5e-4 rad, plus the shifts' truncation
            assert!(err < 2e-3 * z.norm().max(1.0) + 12.0, "({x},{y}) by {th}: ({cx},{cy}) vs {z}");
        }
    }

    /// The model's cells against the float path's (derotate, descramble,
    /// times g): within a cell.
    #[test]
    fn cells_follow_the_float_path() {
        let scramble = super::super::pl_scrambling(4000);
        let mut seed = 9u32;
        let mut r = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / 16_777_216.0
        };
        let (n_cells, pilots) = (2 * GROUP + 100, true);
        let nsym = frame_symbols(n_cells, pilots);
        let words: Vec<u32> = (0..nsym)
            .map(|_| {
                let (re, im) = ((r() - 0.5) * 9000.0, (r() - 0.5) * 9000.0);
                (re as i16 as u16 as u32) | (im as i16 as u16 as u32) << 16
            })
            .collect();
        let segs: Vec<(f64, f64)> = vec![(0.3, 1e-4), (-2.0, -3e-4), (3.1, 2e-5)];
        let (g, g_eff) = gain(64.0 / 0.12);
        let got = cells(&words, n_cells, pilots, &segs.iter().map(|&(a, b)| (angle(a), step(b))).collect::<Vec<_>>(), g, &scramble);
        let (mut j, mut s, mut t, mut n) = (0usize, 0usize, 0usize, 0usize);
        let mut worst = 0f32;
        while n < n_cells {
            let pos = j % (GROUP + PILOT);
            if pos < GROUP {
                let w = words[j];
                let z = Complex32::new((w as u16 as i16) as f32 / 32768.0, ((w >> 16) as u16 as i16) as f32 / 32768.0);
                let a = segs[s].0 + t as f64 * segs[s].1;
                let d = z * Complex32::new(a.cos() as f32, a.sin() as f32);
                let d = super::super::rotate(d, (4 - scramble[j]) & 3);
                let want = super::super::s2cells::cell(d, g_eff);
                for k in 0..2 {
                    worst = worst.max((got[n][k] as f32 - want[k] as f32).abs());
                }
                n += 1;
                t += 1;
            }
            j += 1;
            if j % (GROUP + PILOT) == 0 {
                s += 1;
                t = 0;
            }
        }
        assert!(worst <= 1.0, "worst {worst}");
    }

    /// Vectors for the HDL test (maia-hdl test/vectors/s2ring.json):
    /// S2RING_VECTORS=<file> cargo test --release s2ring_vectors -- --ignored
    #[test]
    #[ignore]
    fn s2ring_vectors() {
        let Some(path) = std::env::var_os("S2RING_VECTORS") else { return };
        let scramble = super::super::pl_scrambling(3000);
        let words: Vec<u32> = (0..3000u32).map(|k| k.wrapping_mul(2_654_435_761) ^ 0x5A5A_1234).collect();
        let segs = vec![(0x1234_5678u32, 0x0000_9ABCu32), (0xFEDC_BA98, 0xFFFF_0123)];
        let c = cells(&words, 2000, true, &segs, 10170, &scramble);
        let j = serde_json::json!({"words": words, "segs": segs, "gain": 10170, "n_cells": 2000, "pilots": true,
            "scramble": &scramble[..64], "cells": c});
        std::fs::write(path, j.to_string()).unwrap();
    }
}
