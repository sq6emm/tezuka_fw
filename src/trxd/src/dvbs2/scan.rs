//! Finding a DVB-S2 signal's symbol rate and mode, the way a tuner's blind
//! scan does: the FPGA front end (DDC, timing recovery, header screening)
//! set to each standard amateur symbol rate in turn, the PL headers it
//! flags read ([`super::pls`]), until one decodes. The receiver proper then
//! starts with what was found; trx comes back here when it loses the signal.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use num_complex::Complex32;

use super::fpga::FrontEnd;
use super::pls::{Pls, PlsDecoder, ACCEPT};

/// The amateur DATV symbol rates (BATC / QO-100 practice).
pub const RATES: [f64; 6] = [250e3, 333e3, 500e3, 125e3, 66e3, 33e3];

#[derive(Clone, Debug)]
pub enum State {
    /// Looking at this symbol rate now.
    Scanning(f64),
    /// A header decoded: rate, mode, carrier offset from the tuned
    /// frequency (Hz).
    Found { sr: f64, pls: Pls, offset_hz: f64 },
    /// Cannot scan (no FPGA front end with timing recovery).
    Failed(String),
}

pub struct Scanner {
    state: Arc<Mutex<State>>,
    stop: Arc<AtomicBool>,
    center: Arc<AtomicU64>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Scanner {
    /// Scan around `center_hz` (the signal's offset from the LO), the rate
    /// found last time first.
    pub fn start(center_hz: f64, first: Option<f64>) -> Scanner {
        let state = Arc::new(Mutex::new(State::Scanning(first.unwrap_or(RATES[0]))));
        let stop = Arc::new(AtomicBool::new(false));
        let center = Arc::new(AtomicU64::new(center_hz.to_bits()));
        let (s, st, c) = (state.clone(), stop.clone(), center.clone());
        let thread = std::thread::Builder::new()
            .name("datv-scan".into())
            .spawn(move || run(s, st, c, first))
            .expect("spawn datv-scan");
        Scanner { state, stop, center, thread: Some(thread) }
    }

    pub fn state(&self) -> State {
        self.state.lock().unwrap().clone()
    }

    /// The signal moved (LO retuned).
    pub fn set_center(&self, hz: f64) {
        self.center.store(hz.to_bits(), Ordering::Relaxed);
    }
}

impl Drop for Scanner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// How long to look at a rate: two long QPSK frames with pilots (the
/// longest), at least 0.4 s.
fn dwell(sr: f64) -> std::time::Duration {
    std::time::Duration::from_secs_f64((2.2 * 33_282.0 / sr).max(0.4))
}

fn run(state: Arc<Mutex<State>>, stop: Arc<AtomicBool>, center: Arc<AtomicU64>, first: Option<f64>) {
    crate::stream::thread_nice(-5);
    let dec = PlsDecoder::default();
    let mut order: Vec<f64> = RATES.to_vec();
    if let Some(f) = first {
        order.retain(|&r| r != f);
        order.insert(0, f);
    }
    'scan: loop {
        for &sr in &order {
            if stop.load(Ordering::Relaxed) {
                return;
            }
            *state.lock().unwrap() = State::Scanning(sr);
            let c = f64::from_bits(center.load(Ordering::Relaxed));
            let mut fe = match FrontEnd::start(sr, 0.35, c) {
                Ok(fe) => fe,
                Err(e) => {
                    *state.lock().unwrap() = State::Failed(e);
                    return;
                }
            };
            if !fe.flagged() {
                *state.lock().unwrap() = State::Failed("the bitstream has no FPGA timing recovery / header detector".into());
                return;
            }
            let (mut syms, mut flags): (Vec<Complex32>, Vec<bool>) = (Vec::new(), Vec::new());
            let mut next = 0usize; // first symbol not yet looked at for flags
            let (mut decodes, mut hits): (usize, Vec<(Pls, f32, f32)>) = (0, Vec::new());
            let t0 = std::time::Instant::now();
            while t0.elapsed() < dwell(sr) && !stop.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(5));
                fe.set_center(f64::from_bits(center.load(Ordering::Relaxed)));
                fe.read_flagged(&mut syms, &mut flags);
                // A flag marks the SOF's last symbol: the header starts 25
                // before and runs 90.
                while next < flags.len() && next + 65 <= syms.len() && decodes < 40 {
                    if flags[next] && next >= 25 {
                        decodes += 1;
                        let (p, s1, s2, w) = dec.decode(&syms[next - 25..next + 65]);
                        if s1 > ACCEPT && s1 - s2 > 0.03 {
                            hits.push((p, s1, w));
                            let same = hits.iter().filter(|h| h.0 == p).count();
                            if same >= 2 || s1 > 0.8 {
                                let offset_hz = w as f64 / std::f64::consts::TAU * sr;
                                tracing::info!(sr, mode = %p.describe(), score = s1, offset_hz = offset_hz.round(), "DATV scan: found");
                                *state.lock().unwrap() = State::Found { sr, pls: p, offset_hz };
                                break 'scan;
                            }
                        }
                    }
                    next += 1;
                }
                // Keep the buffers short: only the symbols a header at the
                // edge still needs.
                if next > 200 {
                    let k = next - 100;
                    syms.drain(..k);
                    flags.drain(..k);
                    next -= k;
                }
            }
        }
    }
    // Leave the front end to the receiver (it restarts it).
}
