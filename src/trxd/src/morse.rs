//! Morse keying timelines, one entry per dot period: `true` while key-down.
//! Shared by the beacon's CW identification, the rigctl `send_morse` keyer and
//! the simulated radio's test signal.

/// Standard timing: dot = 1, dash = 3, element gap = 1, character gap = 3,
/// word gap = 7. Characters without a Morse code are skipped.
pub fn timeline(text: &str) -> Vec<bool> {
    let mut out = Vec::new();
    for c in text.chars() {
        if c == ' ' {
            // The previous character already left 3 units of gap.
            out.extend([false; 4]);
            continue;
        }
        let Some(code) = sdroxide_dsp::morse_encode(c) else {
            continue;
        };
        for (i, sym) in code.chars().filter(|c| *c == '.' || *c == '-').enumerate() {
            if i > 0 {
                out.push(false);
            }
            let n = if sym == '-' { 3 } else { 1 };
            out.extend(std::iter::repeat_n(true, n));
        }
        out.extend([false; 3]);
    }
    out
}

/// Dot length in seconds at `wpm` (PARIS, 50 dots per word).
pub fn dot_seconds(wpm: f32) -> f64 {
    1.2 / wpm.max(1.0) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeline_shape() {
        assert_eq!(timeline("E"), vec![true, false, false, false]);
        assert_eq!(timeline("T"), vec![true, true, true, false, false, false]);
        assert_eq!(timeline("EE").len(), 8);
        // PARIS + word gap is the 50-dot standard word.
        assert_eq!(timeline("PARIS ").len(), 50);
    }
}
