//! The FPGA LDPC engine's BBFRAME out (maia-sdr `ldpc_dma.py` `bb`, 0xFF20
//! bit 6), bit for bit: the decisions packed MSB first a byte at a time
//! (variable 32 k + 8 c + r at bit 8 c + 7 - r of word k), the first Kbch
//! = Nbch - 192 descrambled (the BB scrambler), and the BCH remainder of
//! the first Nbch ([`super::bch::Bch`]'s, bit i: x^i). The receiver takes
//! the bytes as the BBFRAME and corrects from the remainder alone
//! ([`super::bch::Bch::correct_bytes`]); see [`super::fpga_ldpc`].

use super::bch::{Bch, Reg};

/// The engine's `out_words` words and remainder for decisions `dec` (0/1,
/// RAM variable order: the info part is codeword order).
#[cfg_attr(not(test), allow(dead_code))]
pub fn pack(dec: &[u8], nbch: usize, out_words: usize) -> (Vec<u32>, Reg) {
    let kbch = nbch - 192;
    let scr = super::bb_scrambling(kbch.div_ceil(8));
    let mut words = vec![0u32; out_words];
    for (k, w) in words.iter_mut().enumerate() {
        for c in 0..4 {
            for r in 0..8 {
                let v = 32 * k + 8 * c + r;
                let s = if v < kbch { (scr[v / 8] >> (7 - v % 8)) & 1 } else { 0 };
                *w |= ((dec[v] ^ s) as u32) << (8 * c + 7 - r);
            }
        }
    }
    (words, Bch::new().remainder(&dec[..nbch]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbs2::bch::Outcome;

    fn rng(seed: &mut u64) -> u64 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        *seed
    }

    fn bytes(words: &[u32], n: usize) -> Vec<u8> {
        words.iter().flat_map(|w| w.to_le_bytes()).take(n).collect()
    }

    /// A valid codeword comes out as its descrambled BBFRAME with a zero
    /// remainder; up to t = 12 wrong bits are corrected from the remainder
    /// alone, in the bytes; 13 are not taken as a codeword.
    #[test]
    fn bytes_and_remainder_correct_as_bits_do() {
        let bch = Bch::new();
        let mut seed = 0x1234_5678_9abc_def1u64;
        for nbch in [32_400usize, 48_600] {
            let kbch = nbch - 192;
            let out_words = nbch.div_ceil(1024) * 32;
            let mut dec = vec![0u8; out_words * 32];
            for b in dec.iter_mut() {
                *b = (rng(&mut seed) & 1) as u8;
            }
            bch.encode(&mut dec[..nbch]);
            let scr = crate::dvbs2::bb_scrambling(kbch / 8);
            let want: Vec<u8> = (0..kbch / 8)
                .map(|i| (0..8).fold(0u8, |a, b| (a << 1) | dec[8 * i + b]) ^ scr[i])
                .collect();
            let (w, rem) = pack(&dec, nbch, out_words);
            assert_eq!(rem, [0; 3], "{nbch}: a codeword");
            assert_eq!(bytes(&w, kbch / 8), want, "{nbch}: the BBFRAME");
            for errs in [1usize, 5, 12, 13] {
                let mut d = dec.clone();
                for _ in 0..errs {
                    // distinct positions, some in the BCH parity
                    loop {
                        let p = (rng(&mut seed) % nbch as u64) as usize;
                        if d[p] == dec[p] {
                            d[p] ^= 1;
                            break;
                        }
                    }
                }
                let (w, rem) = pack(&d, nbch, out_words);
                assert_ne!(rem, [0; 3]);
                let mut b = bytes(&w, kbch / 8);
                match bch.correct_bytes(&mut b, rem, nbch, kbch) {
                    Outcome::Fixed(k) => {
                        assert!(errs <= 12, "{nbch}: {errs} errors taken as {k}");
                        assert_eq!(k, errs);
                        assert_eq!(b, want, "{nbch}: corrected");
                    }
                    Outcome::Failed => assert!(errs > 12, "{nbch}: {errs} errors not corrected"),
                    Outcome::Clean => panic!("non-zero remainder taken as clean"),
                }
            }
        }
    }

    /// What the ARM does per 3/4 frame, before (unpack the 48600 decisions
    /// a byte each, divide by g(x), pack and descramble the BBFRAME) and
    /// with the fabric's BBFRAME (copy the bytes; the remainder is zero):
    /// cargo test --release bb_arm_time -- --ignored --nocapture
    #[test]
    #[ignore]
    fn bb_arm_time() {
        let bch = Bch::new();
        let (nbch, kbch) = (48_600usize, 48_408usize);
        let out_words = 1536;
        let mut seed = 99u64;
        let mut dec = vec![0u8; out_words * 32];
        for b in dec.iter_mut() {
            *b = (rng(&mut seed) & 1) as u8;
        }
        bch.encode(&mut dec[..nbch]);
        let words: Vec<u32> = (0..out_words).map(|k| (0..32).fold(0u32, |a, j| a | (dec[32 * k + j] as u32) << j)).collect();
        let (fw, rem) = pack(&dec, nbch, out_words);
        let scr = crate::dvbs2::bb_scrambling(kbch / 8);
        let n = 200;
        let t = std::time::Instant::now();
        let mut bits = vec![0u8; nbch];
        let mut sink = 0u64;
        for _ in 0..n {
            for (i, b) in bits.iter_mut().enumerate() {
                *b = ((words[i / 32] >> (i % 32)) & 1) as u8;
            }
            assert!(matches!(bch.decode(&mut bits), Outcome::Clean));
            let bb: Vec<u8> = (0..kbch / 8).map(|i| (0..8).fold(0u8, |a, b| (a << 1) | bits[i * 8 + b]) ^ scr[i]).collect();
            sink += bb[17] as u64;
        }
        let old = t.elapsed().as_secs_f64() / n as f64;
        let t = std::time::Instant::now();
        let mut buf = vec![0u8; nbch / 8];
        for _ in 0..n {
            for (d, w) in buf.chunks_exact_mut(4).zip(&fw) {
                d.copy_from_slice(&w.to_le_bytes());
            }
            assert!(matches!(bch.correct_bytes(&mut buf, rem, nbch, kbch), Outcome::Clean));
            sink += buf[17] as u64;
        }
        let new = t.elapsed().as_secs_f64() / n as f64;
        println!("per 3/4 frame: bits path {:.3} ms, fabric BBFRAME {:.4} ms ({sink})", old * 1e3, new * 1e3);
    }

    /// The HDL test's model (maia-hdl ldpc_dma.py bb_model) against this
    /// one: BBOUT_VECTORS=<maia-hdl/test/vectors/bbout.json> cargo test
    /// --release bbout_vectors -- --ignored
    #[test]
    #[ignore]
    fn bbout_vectors() {
        let Some(path) = std::env::var_os("BBOUT_VECTORS") else { return };
        let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        for case in v.as_array().unwrap() {
            let nbch = case["nbch"].as_u64().unwrap() as usize;
            let out_words = case["out_words"].as_u64().unwrap() as usize;
            let hex = case["dec"].as_str().unwrap();
            let dec: Vec<u8> = hex
                .chars()
                .flat_map(|c| {
                    let x = c.to_digit(16).unwrap() as u8;
                    (0..4).map(move |i| (x >> (3 - i)) & 1)
                })
                .collect();
            let (w, rem) = pack(&dec, nbch, out_words);
            let want: Vec<u32> = case["words"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap() as u32).collect();
            assert_eq!(w, want, "{nbch}: words");
            let r = case["rem"].as_str().unwrap();
            let r = u128::from_str_radix(&r[r.len() - 32..], 16).unwrap();
            let hi = u64::from_str_radix(&case["rem"].as_str().unwrap()[..16], 16).unwrap();
            assert_eq!(rem, [r as u64, (r >> 64) as u64, hi], "{nbch}: remainder");
            println!("{nbch}: {out_words} words and the remainder match");
        }
    }
}
