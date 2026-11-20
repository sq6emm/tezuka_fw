//! Two-antenna noise canceller on the 384 kS/s stream (AD936x in 2R2T).
//!
//! RX1 carries the wanted signal plus the local noise, RX2 a noise
//! antenna that hears mostly the noise. Both receivers share one LO and one
//! sample clock, so the noise reaches them as copies that differ only by a
//! complex gain and a small delay. An adaptive FIR on RX2 (complex
//! normalised LMS) learns that difference and its output is subtracted from
//! RX1:
//!
//! ```text
//!   e[n] = d[n - D] - sum_k w[k] * x[n - k]     d = RX1, x = RX2
//!   w[k] += mu * e[n] * conj(x[n - k]) / (|x|^2 + eps)
//! ```
//!
//! RX1 is delayed by half the filter (`D`), so the noise may reach either
//! antenna first. One tap is the classic amplitude-and-phase knob of an
//! analog canceller; more taps follow a delay or a dispersion between the
//! two paths across the whole stream.
//!
//! Everything after this point (demodulator, scope, TCI IQ, decoders) sees
//! the cleaned stream. Maia's wide scope taps the ADC and stays raw.
//!
//! The weights only make sense for one RF frequency: a retune resets them.
//! They freeze on request (a strong wanted station that also reaches the
//! noise antenna would otherwise be learnt and cancelled) and while
//! transmitting.

use num_complex::Complex32;

pub const TAPS_MAX: usize = 32;
pub const TAPS_DEFAULT: usize = 8;
/// Step size range (normalised LMS: stable below 2, fast near 1).
pub const MU_MIN: f32 = 1e-4;
pub const MU_MAX: f32 = 0.5;
pub const MU_DEFAULT: f32 = 0.01;

pub struct Canceller {
    taps: usize,
    mu: f32,
    /// Weights and the RX2 window as separate real / imaginary arrays,
    /// a multiple of 4 long: plain f32 loops the compiler maps onto
    /// NEON (the A9 has no complex arithmetic of its own).
    wr: Vec<f32>,
    wi: Vec<f32>,
    /// RX2 history, doubled so the window is one slice; the window is
    /// `[xi .. xi + lp]`, oldest first.
    xr: Vec<f32>,
    xim: Vec<f32>,
    xi: usize,
    /// Window length (= taps, a multiple of 4).
    lp: usize,
    /// RX1 delay line (`taps / 2 + 1` samples).
    d: Vec<Complex32>,
    di: usize,
    /// Running |x|^2 over the window.
    xp: f32,
    /// Smoothed powers in / out of the subtraction, for the depth.
    p_in: f32,
    p_out: f32,
    p_ref: f32,
    pub freeze: bool,
}

impl Canceller {
    pub fn new(taps: usize, mu: f32) -> Self {
        // Whole groups of four (the loops below run four lanes).
        let taps = taps.clamp(1, TAPS_MAX).div_ceil(4) * 4;
        let lp = taps;
        let mut c = Canceller {
            taps,
            mu: mu.clamp(MU_MIN, MU_MAX),
            wr: vec![0.0; lp],
            wi: vec![0.0; lp],
            xr: vec![0.0; 2 * lp],
            xim: vec![0.0; 2 * lp],
            xi: 0,
            lp,
            d: vec![Complex32::default(); taps / 2 + 1],
            di: 0,
            xp: 0.0,
            p_in: 0.0,
            p_out: 0.0,
            p_ref: 0.0,
            freeze: false,
        };
        c.reset();
        c
    }

    pub fn taps(&self) -> usize {
        self.taps
    }

    pub fn mu(&self) -> f32 {
        self.mu
    }

    pub fn set_mu(&mut self, mu: f32) {
        self.mu = mu.clamp(MU_MIN, MU_MAX);
    }

    /// Forget the weights (a retune: the path difference is per frequency).
    pub fn reset(&mut self) {
        for v in [&mut self.wr, &mut self.wi, &mut self.xr, &mut self.xim] {
            v.iter_mut().for_each(|x| *x = 0.0);
        }
        self.d.iter_mut().for_each(|v| *v = Complex32::default());
        self.xi = 0;
        self.di = 0;
        self.xp = 0.0;
        self.p_in = 0.0;
        self.p_out = 0.0;
        self.p_ref = 0.0;
    }

    /// Clean `main` (RX1) in place with `aux` (RX2, same length). `adapt`
    /// false holds the weights (transmitting); `freeze` does the same.
    pub fn process(&mut self, main: &mut [Complex32], aux: &[Complex32], adapt: bool) {
        let n = main.len().min(aux.len());
        let lp = self.lp;
        let dl = self.d.len();
        let adapt = adapt && !self.freeze;
        let (mut pi, mut po, mut pr) = (0.0f32, 0.0f32, 0.0f32);
        for i in 0..n {
            let xn = aux[i];
            let (or, oi) = (self.xr[self.xi], self.xim[self.xi]);
            self.xp += xn.norm_sqr() - (or * or + oi * oi);
            self.xr[self.xi] = xn.re;
            self.xr[self.xi + lp] = xn.re;
            self.xim[self.xi] = xn.im;
            self.xim[self.xi + lp] = xn.im;
            self.xi += 1;
            if self.xi == lp {
                self.xi = 0;
            }
            let xr = &self.xr[self.xi..self.xi + lp];
            let xim = &self.xim[self.xi..self.xi + lp];

            let dn = self.d[self.di];
            self.d[self.di] = main[i];
            self.di += 1;
            if self.di == dl {
                self.di = 0;
            }

            // y = sum w * x, four lanes at a time.
            let mut ar = [0.0f32; 4];
            let mut ai = [0.0f32; 4];
            for (((wr, wi), xr), xi) in self.wr.chunks_exact(4).zip(self.wi.chunks_exact(4)).zip(xr.chunks_exact(4)).zip(xim.chunks_exact(4)) {
                for j in 0..4 {
                    ar[j] += wr[j] * xr[j] - wi[j] * xi[j];
                    ai[j] += wr[j] * xi[j] + wi[j] * xr[j];
                }
            }
            let y = Complex32::new(ar[0] + ar[1] + ar[2] + ar[3], ai[0] + ai[1] + ai[2] + ai[3]);
            let e = dn - y;
            if adapt {
                // w += mu e conj(x) / |x|^2
                let g = self.mu / (self.xp.max(0.0) + 1e-9);
                let (gr, gi) = (e.re * g, e.im * g);
                for (((wr, wi), xr), xi) in self.wr.iter_mut().zip(self.wi.iter_mut()).zip(xr).zip(xim) {
                    *wr += gr * xr + gi * xi;
                    *wi += gi * xr - gr * xi;
                }
            }
            main[i] = e;
            pi += dn.norm_sqr();
            po += e.norm_sqr();
            pr += xn.norm_sqr();
        }
        if n > 0 {
            // ~0.1 s smoothing at 384 kS/s with 1024-sample blocks.
            let a = 0.03;
            let k = 1.0 / n as f32;
            self.p_in += a * (pi * k - self.p_in);
            self.p_out += a * (po * k - self.p_out);
            self.p_ref += a * (pr * k - self.p_ref);
        }
        // Keep the running power honest against float drift.
        if self.xi == 0 {
            self.xp = self.xr[..lp].iter().zip(&self.xim[..lp]).map(|(a, b)| a * a + b * b).sum();
        }
    }

    /// Power removed by the subtraction, dB (positive: quieter after).
    pub fn depth_db(&self) -> f32 {
        if self.p_out <= 0.0 || self.p_in <= 0.0 { 0.0 } else { 10.0 * (self.p_in / self.p_out).log10() }
    }

    /// RX2 relative to RX1, dB (the noise antenna's level).
    pub fn ref_db(&self) -> f32 {
        if self.p_ref <= 0.0 || self.p_in <= 0.0 { 0.0 } else { 10.0 * (self.p_ref / self.p_in).log10() }
    }

    /// Magnitude of the strongest weight, dB (how hard RX2 is leant on).
    pub fn weight_db(&self) -> f32 {
        let m = self.wr.iter().zip(&self.wi).map(|(a, b)| a * a + b * b).fold(0.0f32, f32::max);
        if m <= 0.0 { -99.0 } else { 10.0 * m.log10() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    /// A small LCG: reproducible noise without a crate.
    struct Rng(u64);
    impl Rng {
        fn u(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 / (1u64 << 24) as f32) - 0.5
        }
        fn c(&mut self) -> Complex32 {
            Complex32::new(self.u(), self.u())
        }
    }

    fn pow(v: &[Complex32]) -> f32 {
        v.iter().map(|z| z.norm_sqr()).sum::<f32>() / v.len() as f32
    }

    /// Wanted tone + noise via a 2-tap path on RX1, the same noise on RX2
    /// with its own small receiver noise.
    fn scene(n: usize, path: &[Complex32], rng: &mut Rng) -> (Vec<Complex32>, Vec<Complex32>, Vec<Complex32>) {
        let noise: Vec<Complex32> = (0..n).map(|_| rng.c() * 4.0).collect();
        let tone: Vec<Complex32> = (0..n).map(|i| Complex32::from_polar(0.05, TAU * 0.0123 * i as f32)).collect();
        let mut main = vec![Complex32::default(); n];
        let mut aux = vec![Complex32::default(); n];
        for i in 0..n {
            let mut v = Complex32::default();
            for (k, p) in path.iter().enumerate() {
                if i >= k {
                    v += p * noise[i - k];
                }
            }
            main[i] = tone[i] + v + rng.c() * 0.002;
            aux[i] = noise[i] + rng.c() * 0.002;
        }
        (main, aux, tone)
    }

    fn run(c: &mut Canceller, main: &mut [Complex32], aux: &[Complex32]) {
        for (m, a) in main.chunks_mut(1024).zip(aux.chunks(1024)) {
            c.process(m, a, true);
        }
    }

    #[test]
    fn cancels_a_coherent_noise_and_keeps_the_wanted_tone() {
        let mut rng = Rng(7);
        let path = [Complex32::new(0.6, -0.5), Complex32::new(0.2, 0.1)];
        let n = 384_000;
        let (mut main, aux, tone) = scene(n, &path, &mut rng);
        let before = pow(&main[n - 38_400..]);
        let mut c = Canceller::new(8, 0.02);
        run(&mut c, &mut main, &aux);
        // The last 0.1 s: the residual is the tone (delayed by taps/2).
        let d = c.taps() / 2;
        let tail = &main[n - 38_400..];
        let resid: Vec<Complex32> = tail.iter().zip(&tone[n - 38_400 - d..n - d]).map(|(m, t)| m - t).collect();
        let tone_p = pow(&tone);
        let after = pow(&resid);
        let depth = 10.0 * (before / after).log10();
        assert!(depth > 30.0, "only {depth:.1} dB of noise removed");
        // The tone itself survives: what is left is mostly tone.
        assert!(10.0 * (tone_p / after).log10() > 10.0, "tone buried: {:.1} dB", 10.0 * (tone_p / after).log10());
        assert!(c.depth_db() > 20.0, "depth meter {:.1}", c.depth_db());
    }

    #[test]
    fn the_noise_may_reach_the_noise_antenna_later() {
        // RX1 hears the noise 2 samples before RX2: the RX1 delay covers it.
        let mut rng = Rng(11);
        let n = 200_000;
        let noise: Vec<Complex32> = (0..n).map(|_| rng.c()).collect();
        let main: Vec<Complex32> = noise.iter().map(|z| z * Complex32::new(0.0, 0.8)).collect();
        let aux: Vec<Complex32> = (0..n).map(|i| if i >= 2 { noise[i - 2] } else { Complex32::default() }).collect();
        let mut m = main.clone();
        let mut c = Canceller::new(8, 0.05);
        run(&mut c, &mut m, &aux);
        let depth = 10.0 * (pow(&main[n - 20_000..]) / pow(&m[n - 20_000..])).log10();
        assert!(depth > 30.0, "{depth:.1} dB");
    }

    #[test]
    fn frozen_weights_do_not_move_and_reset_clears_them() {
        let mut rng = Rng(3);
        let (main, aux, _) = scene(50_000, &[Complex32::new(1.0, 0.0)], &mut rng);
        let mut c = Canceller::new(4, 0.05);
        c.freeze = true;
        run(&mut c, &mut main.clone(), &aux);
        assert_eq!(c.weight_db(), -99.0);
        c.freeze = false;
        run(&mut c, &mut main.clone(), &aux);
        assert!(c.weight_db() > -3.0);
        c.reset();
        assert_eq!(c.weight_db(), -99.0);
    }

    #[test]
    fn uncorrelated_inputs_leave_the_main_channel_alone() {
        let mut rng = Rng(5);
        let n = 200_000;
        let main: Vec<Complex32> = (0..n).map(|_| rng.c()).collect();
        let aux: Vec<Complex32> = (0..n).map(|_| rng.c()).collect();
        let mut m = main.clone();
        let mut c = Canceller::new(8, 0.01);
        run(&mut c, &mut m, &aux);
        // Misadjustment only: well under 0.5 dB of added noise.
        let change = 10.0 * (pow(&m[n - 20_000..]) / pow(&main[n - 20_000..])).log10();
        assert!(change.abs() < 0.5, "{change:.2} dB");
    }
}
