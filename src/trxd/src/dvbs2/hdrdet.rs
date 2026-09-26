//! DVB-S2 header screening in the FPGA (maia-hdl `hdrdet.py`), after the
//! timing recovery: a bit-exact model.
//!
//! For every symbol the detector correlates the last 26 with the SOF (the
//! same for every MODCOD) in two coherent chunks of 13 and flags the symbol
//! when the chunk magnitudes add up to at least 12/16 of the window summed
//! magnitude. SOF symbols are (a + jb)/sqrt2 with a, b = +-1, so the
//! correlation needs adds only: re += a re_s + b im_s, im += a im_s - b re_s
//! (sqrt2 times the true value). Magnitudes: |z| ~ max + (3 min >> 3) on
//! the components absolute values. The threshold is then about 0.53 of the
//! true normalized metric (noise gives about 0.25 +- 0.09, a header 0.7 at
//! 0 dB): under 1 % of the symbols flagged on noise (0.7 % measured).
//!
//! The flag goes to the CPU in bit 0 of the symbol's imaginary part (the
//! ring word's bit 16); it marks the SOF's last symbol, so the header starts
//! 25 symbols earlier. The receiver computes its full 90-symbol metric only
//! at flagged positions.

pub const SOF_LEN: usize = 26;
const CHUNK: usize = 13;
const THR16: i64 = 12;

/// SOF symbols as (a, b), h = (a + jb)/sqrt2.
pub fn sof_signs() -> [(i64, i64); SOF_LEN] {
    const SOF: u32 = 0x18D_2E82;
    std::array::from_fn(|s| {
        let bit = (SOF >> (SOF_LEN - 1 - s)) & 1;
        // pi/2-BPSK: angle pi/4 + q pi/2, q = 2 bit + (s & 1).
        match 2 * bit as usize + (s & 1) {
            0 => (1, 1),
            1 => (-1, 1),
            2 => (-1, -1),
            _ => (1, -1),
        }
    })
}

fn mag(re: i64, im: i64) -> i64 {
    let (a, b) = (re.abs(), im.abs());
    let (hi, lo) = if a > b { (a, b) } else { (b, a) };
    hi + ((3 * lo) >> 3)
}

pub struct HdrDet {
    buf: [[i64; 2]; 32],
    n: usize,
    sof: [(i64, i64); SOF_LEN],
}

impl Default for HdrDet {
    fn default() -> Self {
        HdrDet { buf: [[0; 2]; 32], n: 0, sof: sof_signs() }
    }
}

impl HdrDet {
    /// One symbol in; true when it may end a SOF.
    pub fn push(&mut self, y: [i16; 2]) -> bool {
        self.buf[self.n % 32] = [y[0] as i64, y[1] as i64];
        self.n += 1;
        if self.n < SOF_LEN {
            return false;
        }
        let (mut num, mut den) = (0i64, 0i64);
        let (mut ar, mut ai) = (0i64, 0i64);
        for i in 0..SOF_LEN {
            let [re, im] = self.buf[(self.n - SOF_LEN + i) % 32];
            let (a, b) = self.sof[i];
            ar += a * re + b * im;
            ai += a * im - b * re;
            den += mag(re, im);
            if i % CHUNK == CHUNK - 1 {
                num += mag(ar, ai);
                ar = 0;
                ai = 0;
            }
        }
        16 * num >= THR16 * den && den > 0
    }

    /// The ring word's symbol with the flag in bit 0 of its imaginary part.
    pub fn mark(y: [i16; 2], flag: bool) -> [i16; 2] {
        [y[0], (y[1] & !1) | flag as i16]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sof_signs_match_the_header() {
        let h = super::super::FrameSpec::long(super::super::fpga_tx::LongMode::Qpsk12).header();
        let r = std::f32::consts::FRAC_1_SQRT_2;
        for (i, (a, b)) in sof_signs().iter().enumerate() {
            assert!((h[i].re - *a as f32 * r).abs() < 1e-6 && (h[i].im - *b as f32 * r).abs() < 1e-6, "{i}");
        }
    }
}

#[cfg(test)]
mod vectors {
    use super::*;

    /// Test vectors for maia-hdl (`test/test_hdrdet.py`): symbols from the
    /// timing recovery model on a board recording (`DATV_CAP`, cf32 at
    /// 512 kS/s, 250 kS/s DVB-S2: several headers) and weak noise; the
    /// flags. `HDRDET_VECTORS=<file> DATV_CAP=<cf32> cargo test hdrdet_vectors -- --ignored`
    #[test]
    #[ignore]
    fn hdrdet_vectors() {
        use super::super::symsync::{Params, SymSync};
        let path = std::env::var("HDRDET_VECTORS").expect("HDRDET_VECTORS=<file>");
        let cap = std::fs::read(std::env::var("DATV_CAP").expect("DATV_CAP=<cf32>")).unwrap();
        let q = |v: f32| (v * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
        let mut ss = SymSync::new(Params::new(512e3, 250e3));
        let syms: Vec<[i16; 2]> = cap
            .chunks_exact(8)
            .take(150_000)
            .filter_map(|c| ss.push([q(f32::from_le_bytes(c[..4].try_into().unwrap())), q(f32::from_le_bytes(c[4..].try_into().unwrap()))]))
            .collect();
        let mut seed = 4u64;
        let noise: Vec<[i16; 2]> = (0..8000)
            .map(|_| {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
                [((seed >> 40) as i64 % 400 - 200) as i16, ((seed >> 20) as i64 % 400 - 200) as i16]
            })
            .collect();
        let mut cases = Vec::new();
        for (name, input) in [("capture_250k", syms), ("noise", noise)] {
            let mut hd = HdrDet::default();
            let flags: Vec<usize> = input.iter().enumerate().filter(|(_, y)| hd.push(**y)).map(|(k, _)| k).collect();
            eprintln!("{name}: {} symbols, {} flags", input.len(), flags.len());
            cases.push(serde_json::json!({"name": name, "input": input, "flags": flags}));
        }
        std::fs::write(path, serde_json::to_vec(&cases).unwrap()).unwrap();
    }
}
