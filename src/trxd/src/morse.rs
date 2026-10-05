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

/// Gap stretches tried (slow fists, Farnsworth).
const SPACE_RATIOS: [f32; 14] = [1.0, 1.15, 1.35, 1.6, 1.9, 2.2, 2.6, 3.0, 3.5, 4.1, 4.8, 5.6, 6.5, 8.0];

/// How Morse-like a keying is, 0..1 (after the rain-scatter decoder project's
/// `_quant_fit`): Morse run lengths are quantised against one time unit,
/// elements 1 and 3, gaps 1, 3 and 7 (gaps one-sided: only a gap shorter
/// than its target counts against, with a fitted stretch); speech run
/// lengths are not. Units of 28-130 ms (about 9-43 WPM). The runs cut
/// by the window's edges are left out. `dt`: seconds a frame.
pub fn morse_fit(keyed: &[bool], dt: f32) -> f32 {
    let mut runs: Vec<(bool, f32)> = Vec::new();
    for &k in keyed {
        match runs.last_mut() {
            Some((v, n)) if *v == k => *n += dt,
            _ => runs.push((k, dt)),
        }
    }
    if runs.len() < 3 {
        return 0.0;
    }
    let runs = &runs[1..runs.len() - 1];
    let on: Vec<f32> = runs.iter().filter(|r| r.0).map(|r| r.1).collect();
    let off: Vec<f32> = runs.iter().filter(|r| !r.0).map(|r| r.1).collect();
    if runs.len() < 8 || on.len() < 4 || off.len() < 3 {
        return 0.0;
    }
    let n = (on.len() + off.len()) as f32;
    let (umin, umax) = (0.028f32, 0.130f32);
    let mut best = f32::MAX;
    for i in 0..80 {
        let u = (umin.ln() + (umax.ln() - umin.ln()) * i as f32 / 79.0).exp();
        let c_on: f32 = on.iter().map(|&l| {
            let k = l / u;
            let t = if k < 2.0 { 1.0 } else { 3.0 };
            (k - t).abs() / t
        }).sum();
        let mut c_gap = f32::MAX;
        for sr in SPACE_RATIOS {
            let c: f32 = off.iter().map(|&l| {
                let k = l / u;
                let (t, two) = [(1.0, true), (3.0 * sr, false), (7.0 * sr, false)]
                    .into_iter()
                    .min_by(|a, b| (k - a.0).abs().total_cmp(&(k - b.0).abs()))
                    .unwrap();
                (if two { (k - t).abs() } else { (t - k).max(0.0) }) / t
            }).sum();
            c_gap = c_gap.min(c);
        }
        best = best.min((c_on + c_gap) / n);
    }
    (1.0 - best / 0.45).max(0.0)
}

#[cfg(test)]
mod tests {
    /// The Morse rhythm test: Morse keying (with timing jitter) scores
    /// clearly above speech-like runs of random length, a steady carrier 0.
    #[test]
    fn morse_fit_tells_morse_from_speech() {
        let dt = 128.0 / 12000.0;
        let mut seed = 7u64;
        let mut rnd = move || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            (seed >> 11) as f32 / (1u64 << 53) as f32
        };
        let push = |v: &mut Vec<bool>, k: bool, secs: f32| v.extend(std::iter::repeat(k).take((secs / dt).round() as usize));
        // "CQ DE SP6GWB K" at 15 WPM, 10 % jitter, 4 s of it
        let u = 1.2 / 15.0;
        let mut m = Vec::new();
        for w in "CQ DE SP6GWB K CQ".split(' ') {
            for c in w.chars() {
                let code = sdroxide_dsp::morse_encode(c).unwrap();
                for e in code.chars() {
                    push(&mut m, true, u * if e == '.' { 1.0 } else { 3.0 } * (0.9 + 0.2 * rnd()));
                    push(&mut m, false, u * (0.9 + 0.2 * rnd()));
                }
                push(&mut m, false, 2.0 * u);
            }
            push(&mut m, false, 4.0 * u);
        }
        m.truncate((4.0 / dt) as usize);
        let mut sp = Vec::new();
        while sp.len() < (4.0 / dt) as usize {
            let (a, b) = (0.1 + 0.4 * rnd(), 0.05 + 0.35 * rnd());
            push(&mut sp, true, a);
            push(&mut sp, false, b);
        }
        let (fm, fs, fc) = (morse_fit(&m, dt), morse_fit(&sp, dt), morse_fit(&vec![true; 400], dt));
        eprintln!("morse {fm:.2}, speech {fs:.2}, carrier {fc:.2}");
        // (synthetic speech scores about 0.5: the threshold comes from
        // labelled recordings, not from this)
        assert!(fm > 0.75 && fs < fm - 0.25 && fc == 0.0);
    }

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
