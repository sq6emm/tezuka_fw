//! The two sample-moving threads, and the UTC clock the receive side stamps.
//!
//! RX: blocking reads, each block stamped with the UTC time of its first
//! sample, handed to the engine over a short channel. TX: the engine's blocks,
//! written in order. Neither thread does DSP.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crossbeam_channel::{Receiver, Sender, bounded};
use num_complex::Complex32;
use tracing::{error, info, warn};

use crate::radio::{RxStream, TxStream};

/// Raise the calling thread above the decoders (nice -10). The sample path
/// (RX, engine, TX) must never wait behind a neural net: a late block is a
/// dropped block, while a late decode is only late. Needs root; elsewhere
/// (the simulator on a PC) it quietly stays at the default.
pub fn realtime_thread() {
    thread_nice(-10);
}

/// Set the calling thread's nice value (lower runs first).
pub fn thread_nice(nice: i32) {
    // SAFETY: setpriority on our own thread id, no pointers involved.
    unsafe {
        let tid = libc::syscall(libc::SYS_gettid) as libc::id_t;
        libc::setpriority(libc::PRIO_PROCESS, tid, nice);
    }
}

pub fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or(Duration::ZERO).as_secs_f64()
}

/// Whether the kernel considers the system clock synchronised (chrony / NTP
/// / GPS has disciplined it). Beacon timing depends on this.
pub fn time_synced() -> bool {
    // SAFETY: adjtimex with modes = 0 only reads the kernel's clock state
    // into the zero-initialised struct we own.
    unsafe {
        let mut tx: libc::timex = std::mem::zeroed();
        let state = libc::adjtimex(&mut tx);
        state >= 0 && state != libc::TIME_ERROR && tx.status & libc::STA_UNSYNC == 0
    }
}

pub struct RxBlock {
    /// UTC (Unix seconds) of `iq[0]`.
    pub t0: f64,
    pub iq: Vec<Complex32>,
}

/// Maps the converter's sample count onto UTC. The sample clock is the
/// reference (it is what the signal is timed by); the system clock anchors it
/// and pulls it back gently, and a jump beyond `RESYNC_S` (an overrun, or
/// chrony stepping the clock) re-anchors outright.
pub struct SampleClock {
    rate: f64,
    anchor: Option<f64>,
    count: u64,
}

const RESYNC_S: f64 = 0.05;

impl SampleClock {
    pub fn new(rate: f64) -> Self {
        SampleClock { rate, anchor: None, count: 0 }
    }

    /// Stamp a block of `n` samples that finished arriving at `now`.
    pub fn stamp(&mut self, n: usize, now: f64) -> f64 {
        let measured = now - n as f64 / self.rate;
        let t = match self.anchor {
            None => {
                self.anchor = Some(measured);
                measured
            }
            Some(a) => {
                let predicted = a + self.count as f64 / self.rate;
                let err = measured - predicted;
                if err.abs() > RESYNC_S {
                    warn!(err_ms = err * 1e3, "RX timeline re-anchored");
                    self.anchor = Some(measured - self.count as f64 / self.rate);
                    measured
                } else {
                    // Read wake-ups are late, never early, so only chase a
                    // clock that is running ahead of us, and slowly.
                    self.anchor = Some(a + err.min(0.0) * 0.01 + err.max(0.0) * 0.0005);
                    predicted
                }
            }
        };
        self.count += n as u64;
        t
    }
}

pub fn spawn_rx(mut rx: Box<dyn RxStream>, rate: f64, block: usize) -> Receiver<RxBlock> {
    let (tx, out) = bounded::<RxBlock>(32);
    std::thread::Builder::new()
        .name("rx".into())
        .spawn(move || {
            realtime_thread();
            let mut clock = SampleClock::new(rate);
            let mut dropped = 0u64;
            loop {
                let mut iq = vec![Complex32::default(); block];
                if let Err(e) = rx.read(&mut iq) {
                    error!("{e}");
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
                let t0 = clock.stamp(block, unix_now());
                match tx.try_send(RxBlock { t0, iq }) {
                    Ok(()) => {}
                    Err(crossbeam_channel::TrySendError::Full(_)) => {
                        dropped += 1;
                        if dropped.is_power_of_two() {
                            warn!(dropped, "engine behind, RX blocks dropped");
                        }
                    }
                    Err(crossbeam_channel::TrySendError::Disconnected(_)) => break,
                }
            }
            info!("RX thread done");
        })
        .expect("spawn rx thread");
    out
}

/// Blocks queued ahead of the DAC before the engine is let loose: the
/// transmit latency, in blocks, and the jitter margin it buys.
pub const TX_PREFILL_BLOCKS: usize = 2;

/// What goes to the DAC DMA: IQ samples, or (DATV in the FPGA) bytes for
/// the DVB-S2 encoder, written as they are.
pub enum TxBlock {
    Iq(Vec<Complex32>),
    Raw(Vec<u8>),
}

pub fn spawn_tx(mut tx: Box<dyn TxStream>, block: usize) -> Sender<TxBlock> {
    let (send, blocks) = bounded::<TxBlock>(8);
    std::thread::Builder::new()
        .name("tx".into())
        .spawn(move || {
            realtime_thread();
            let silence = vec![Complex32::default(); block];
            for _ in 0..TX_PREFILL_BLOCKS {
                if let Err(e) = tx.write(&silence) {
                    error!("{e}");
                }
            }
            for b in blocks {
                let r = match b {
                    TxBlock::Iq(b) => tx.write(&b),
                    TxBlock::Raw(b) => tx.write_raw(&b),
                };
                if let Err(e) = r {
                    error!("{e}");
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        })
        .expect("spawn tx thread");
    send
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_follows_samples_and_resyncs_on_jumps() {
        let mut c = SampleClock::new(1000.0);
        let t0 = c.stamp(100, 10.1);
        assert!((t0 - 10.0).abs() < 1e-9);
        // A late wake-up does not move the timeline by the lateness.
        let t1 = c.stamp(100, 10.21);
        assert!((t1 - 10.1).abs() < 1e-3, "{t1}");
        // A one-second gap (overrun / clock step) re-anchors.
        let t2 = c.stamp(100, 11.3);
        assert!((t2 - 11.2).abs() < 1e-9, "{t2}");
    }
}
