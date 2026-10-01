//! Transmitter interlocks that do not depend on the engine thread.
//!
//! The engine keys and unkeys the radio, and its own safety rails (TX
//! time-out, starved audio) run once per received block. When that thread
//! stops (a stalled RX DMA, a panic, a kill) the hardware would stay keyed:
//! the TX LO powered and the PTT line high. Three things catch that here:
//!
//! * a watchdog thread: RF on and no engine tick for [`ENGINE_TIMEOUT`]
//!   switches RF off (and tells the engine when it comes back);
//! * a panic hook (the release build aborts on panic): RF off first;
//! * SIGTERM / SIGINT / SIGHUP: RF off, then exit.
//!
//! "RF off" is the same two sysfs writes the backend makes, without its
//! locks or state: the TX LO powered down, then the PTT line low.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

/// No engine tick for this long while RF is on: the watchdog switches RF off.
pub const ENGINE_TIMEOUT: Duration = Duration::from_secs(2);

struct Hw {
    phy: PathBuf,
    ptt: Option<PathBuf>,
}

static HW: OnceLock<Hw> = OnceLock::new();
static EPOCH: OnceLock<Instant> = OnceLock::new();
static RF_ON: AtomicBool = AtomicBool::new(false);
static TRIPPED: AtomicBool = AtomicBool::new(false);
/// Milliseconds since [`EPOCH`] of the last engine tick; 0 = never (disarmed).
static TICK_MS: AtomicU64 = AtomicU64::new(0);

fn now_ms() -> u64 {
    EPOCH.get_or_init(Instant::now).elapsed().as_millis() as u64 + 1
}

/// The hardware to silence (the IIO backend, once opened).
pub fn register(phy: PathBuf, ptt: Option<PathBuf>) {
    let _ = HW.set(Hw { phy, ptt });
}

/// The backend's view of the transmitter: set before keying, cleared after.
pub fn set_rf(on: bool) {
    RF_ON.store(on, Ordering::SeqCst);
}

pub fn rf_on() -> bool {
    RF_ON.load(Ordering::SeqCst)
}

/// The engine is alive (once per block). Arms the watchdog.
pub fn tick() {
    TICK_MS.store(now_ms(), Ordering::Relaxed);
}

/// Did the watchdog switch RF off behind the engine's back (since the last call)?
pub fn take_tripped() -> bool {
    TRIPPED.swap(false, Ordering::SeqCst)
}

/// Switch RF off now: TX LO down, then the PTT line low. Safe to call from
/// any thread, any number of times; no locks, no allocation beyond the paths.
pub fn rf_off() {
    if let Some(hw) = HW.get() {
        silence(hw);
    }
    RF_ON.store(false, Ordering::SeqCst);
}

fn silence(hw: &Hw) {
    let _ = std::fs::write(hw.phy.join("out_altvoltage1_TX_LO_powerdown"), "1");
    if let Some(p) = &hw.ptt {
        // The RF is gone before the relay moves.
        std::thread::sleep(Duration::from_millis(10));
        let _ = std::fs::write(p, "0");
    }
}

/// Install the panic hook, the signal thread and the watchdog. Call first
/// thing in main, before any other thread exists (the signal mask is
/// inherited by threads created later; child processes get it reset).
pub fn install() {
    EPOCH.get_or_init(Instant::now);

    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let keyed = rf_on();
        rf_off();
        if keyed {
            eprintln!("trxd: panic while transmitting: RF switched off");
        }
        prev(info);
    }));

    // SAFETY: plain libc signal-set calls on a local sigset_t; the mask is
    // set for this (the main) thread, so every thread spawned later blocks
    // these signals and only `sigwait` below receives them.
    let set = unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        for s in [libc::SIGTERM, libc::SIGINT, libc::SIGHUP] {
            libc::sigaddset(&mut set, s);
        }
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
        set
    };
    std::thread::Builder::new()
        .name("signals".into())
        .spawn(move || loop {
            let mut sig: libc::c_int = 0;
            // SAFETY: `set` is a valid, initialised sigset_t owned by this closure.
            if unsafe { libc::sigwait(&set, &mut sig) } == 0 {
                let keyed = rf_on();
                rf_off();
                tracing::info!(signal = sig, keyed, "stopping");
                std::process::exit(128 + sig);
            }
        })
        .expect("spawn signal thread");

    std::thread::Builder::new()
        .name("tx-watchdog".into())
        .spawn(|| loop {
            std::thread::sleep(Duration::from_millis(100));
            let tick = TICK_MS.load(Ordering::Relaxed);
            if tick == 0 || !rf_on() {
                continue;
            }
            let quiet = now_ms().saturating_sub(tick);
            if quiet > ENGINE_TIMEOUT.as_millis() as u64 {
                rf_off();
                TRIPPED.store(true, Ordering::SeqCst);
                tracing::error!(ms = quiet, "engine stalled while transmitting: RF switched off");
            }
        })
        .expect("spawn TX watchdog");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rf_off_writes_powerdown_then_ptt() {
        let dir = std::env::temp_dir().join(format!("trxd-safety-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let ptt = dir.join("ptt");
        std::fs::write(&ptt, "1").unwrap();
        std::fs::write(dir.join("out_altvoltage1_TX_LO_powerdown"), "0").unwrap();
        silence(&Hw { phy: dir.clone(), ptt: Some(ptt.clone()) });
        assert_eq!(std::fs::read_to_string(dir.join("out_altvoltage1_TX_LO_powerdown")).unwrap(), "1");
        assert_eq!(std::fs::read_to_string(&ptt).unwrap(), "0");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
