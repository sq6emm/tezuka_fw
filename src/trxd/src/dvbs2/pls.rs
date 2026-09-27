//! Which DVB-S2 mode is on the air: the PL header's signalling (PLS: MODCOD,
//! frame size, pilots) read from the 90 header symbols. Every valid MODCOD
//! / type pair's header is matched coherently over all 90 symbols at every
//! carrier offset at once: the header times the candidate's conjugate,
//! zero-padded to 256, FFT, the peak (any offset up to half the symbol
//! rate; at low symbol rates a ppm is several percent). The best wins.

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

use super::fpga_tx::LongMode;
use super::plheader_typed;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pls {
    pub modcod: u8,
    pub short: bool,
    pub pilots: bool,
}

impl Pls {
    /// The long-frame mode trxd receives, if it is one of them.
    pub fn long_mode(&self) -> Option<LongMode> {
        if self.short || !self.pilots {
            return None;
        }
        [LongMode::Qpsk12, LongMode::Qpsk34, LongMode::Psk8_34].into_iter().find(|m| m.modcod() == self.modcod)
    }

    pub fn describe(&self) -> String {
        const NAMES: [&str; 29] = [
            "dummy", "QPSK 1/4", "QPSK 1/3", "QPSK 2/5", "QPSK 1/2", "QPSK 3/5", "QPSK 2/3", "QPSK 3/4", "QPSK 4/5", "QPSK 5/6", "QPSK 8/9",
            "QPSK 9/10", "8PSK 3/5", "8PSK 2/3", "8PSK 3/4", "8PSK 5/6", "8PSK 8/9", "8PSK 9/10", "16APSK 2/3", "16APSK 3/4", "16APSK 4/5",
            "16APSK 5/6", "16APSK 8/9", "16APSK 9/10", "32APSK 3/4", "32APSK 4/5", "32APSK 5/6", "32APSK 8/9", "32APSK 9/10",
        ];
        format!(
            "{} {}{}",
            NAMES.get(self.modcod as usize).unwrap_or(&"?"),
            if self.short { "short" } else { "long" },
            if self.pilots { ", pilots" } else { "" }
        )
    }
}

/// Every valid header (90 symbols each).
pub struct PlsDecoder {
    refs: Vec<(Pls, Vec<Complex32>)>,
    fft: Arc<dyn Fft<f32>>,
}

const NFFT: usize = 256;
/// A decode above this is a header (noise stays under about 0.45; a header
/// at 0 dB Es/N0 scores over 0.6).
pub const ACCEPT: f32 = 0.52;

impl Default for PlsDecoder {
    fn default() -> Self {
        let mut refs = Vec::new();
        for modcod in 1..=28u8 {
            for (short, pilots) in [(false, false), (false, true), (true, false), (true, true)] {
                refs.push((Pls { modcod, short, pilots }, plheader_typed(modcod, pilots, short)));
            }
        }
        PlsDecoder { refs, fft: FftPlanner::new().plan_fft_forward(NFFT) }
    }
}

impl PlsDecoder {
    /// The best-matching PLS for 90 header symbols, its score (1 for a
    /// perfect match, about 0.3 for noise), the second best's, and the
    /// carrier offset found (radians a symbol).
    pub fn decode(&self, hdr: &[Complex32]) -> (Pls, f32, f32, f32) {
        assert!(hdr.len() >= 90);
        let norm: f32 = hdr[..90].iter().map(|z| z.norm()).sum::<f32>().max(1e-20);
        let (mut best, mut s1, mut s2, mut w1) = (self.refs[0].0, f32::MIN, f32::MIN, 0f32);
        let mut buf = vec![Complex32::default(); NFFT];
        let mut scratch = vec![Complex32::default(); self.fft.get_inplace_scratch_len()];
        for (p, r) in &self.refs {
            buf.fill(Complex32::default());
            for i in 0..90 {
                buf[i] = hdr[i] * r[i].conj();
            }
            self.fft.process_with_scratch(&mut buf, &mut scratch);
            let (k, m) = buf.iter().enumerate().map(|(k, z)| (k, z.norm_sqr())).fold((0, 0f32), |a, b| if b.1 > a.1 { b } else { a });
            let s = m.sqrt() / norm;
            if s > s1 {
                s2 = s1;
                s1 = s;
                best = *p;
                // bin k: the header turns by -2 pi k / N a symbol
                let kk = if k > NFFT / 2 { k as f32 - NFFT as f32 } else { k as f32 };
                w1 = -std::f32::consts::TAU * kk / NFFT as f32;
            } else if s > s2 {
                s2 = s;
            }
        }
        (best, s1, s2, w1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_every_mode_with_noise_and_offset() {
        let dec = PlsDecoder::default();
        let mut seed = 3u64;
        let mut g = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            ((-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        // Es/N0 0 dB, a carrier 2 % of the symbol rate off, a random phase.
        let sigma = (0.5f32).sqrt();
        let (mut worst, mut lowest) = (1f32, 1f32);
        for modcod in 1..=28u8 {
            for (short, pilots) in [(false, true), (false, false), (true, true)] {
                for trial in 0..4 {
                    let h = plheader_typed(modcod, pilots, short);
                    let ph0 = trial as f32 * 1.3;
                    let x: Vec<Complex32> = h
                        .iter()
                        .enumerate()
                        .map(|(i, z)| z * Complex32::from_polar(1.0, ph0 + 0.02 * std::f32::consts::TAU * i as f32) + Complex32::new(sigma * g(), sigma * g()))
                        .collect();
                    let (p, s1, s2, _) = dec.decode(&x);
                    assert_eq!(p, Pls { modcod, short, pilots }, "scores {s1} {s2}");
                    worst = worst.min(s1 - s2);
                    lowest = lowest.min(s1);
                }
            }
        }
        eprintln!("smallest margin to the second best: {worst:.2}, lowest score {lowest:.2}");
        // Noise alone scores well below a header at 0 dB (the best of 112
        // candidates at 256 offsets: about 0.4).
        let mut noise = 0f32;
        for _ in 0..20 {
            let x: Vec<Complex32> = (0..90).map(|_| Complex32::new(g(), g())).collect();
            noise = noise.max(dec.decode(&x).1);
        }
        eprintln!("noise scores up to {noise:.2}");
        assert!(noise < ACCEPT && lowest > ACCEPT, "noise {noise}, lowest header {lowest}");
    }
}
