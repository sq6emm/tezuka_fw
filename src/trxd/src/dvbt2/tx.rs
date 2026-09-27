//! DVB-T2 transmission: the modulator on a thread of its own, its samples
//! (oversampled x1.25) as raw 16-bit IQ into the TX buffer, and the FPGA
//! interpolator resampling them to the DAC rate ([`crate::dvbs2::fpga_tx::RawTransmitter`]).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};

use super::{Cell, FecStage, OfdmStage, Params};
use crate::dvbs2::TS_LEN;
use crate::dvbs2::ts::Mux;
use crate::stream::TxBlock;

/// Oversampling (in quarters) of the modulator's output: none. At x1.25
/// the A9 had 8 % to spare and the DAC ran dry now and then; at x1 the
/// FPGA's 16-tap resampler still puts the images about 44 dB down at the
/// channel edge (more further out, and the AD936x's analog filter adds).
const OS4: usize = 4;
/// Unit-RMS samples to 8 bits (the FPGA scales them by 256): RMS 48 (about
/// -9 dBFS), peaks above 2.65 sigma clip (clipping noise near -25 dB,
/// quantization near -41 dB: both far below what QPSK/16QAM need; power is
/// what counts over the air).
const SCALE: f32 = 48.0;

/// A T2 mode as the UI names it: "T2-<MHz>-<QPSK|16QAM>-<rate>".
#[derive(Clone, Copy, Debug)]
pub struct Mode {
    pub bw_hz: f64,
    pub p: Params,
}

impl Mode {
    pub fn parse(s: &str) -> Option<Mode> {
        let mut it = s.split('-');
        if it.next()? != "T2" {
            return None;
        }
        let bw_hz = match it.next()? {
            "1.7" => 1_700_000.0,
            "2.0" => 2_000_000.0,
            "1.35" => 1_350_000.0,
            _ => return None,
        };
        let constellation = match it.next()? {
            "QPSK" => super::Constellation::Qpsk,
            "16QAM" => super::Constellation::Qam16,
            _ => return None,
        };
        let rate = match it.next()? {
            "1/2" => crate::dvbs2::ldpc_fpga::LongRate::R1_2,
            "3/4" => crate::dvbs2::ldpc_fpga::LongRate::R3_4,
            _ => return None,
        };
        let mut p = Params::amateur();
        p.rate = rate;
        p.constellation = constellation;
        // 190 data symbols hold 9 QPSK FEC blocks (32400 cells) or 18 16QAM.
        p.fec_blocks = 9 * constellation.bits() / 2;
        // Rotated QPSK (29 degrees, Q a cell later): free robustness against
        // fading, as T2 intends; receivers read it from L1-post.
        p.rotation = true;
        Some(Mode { bw_hz, p })
    }

    /// Elementary sample rate: 131/71 MHz for the standard 1.7 MHz channel,
    /// 8/7 x bandwidth otherwise (the amateur convention).
    pub fn fs(&self) -> f64 {
        if self.bw_hz == 1_700_000.0 { 131e6 / 71.0 } else { self.bw_hz * 8.0 / 7.0 }
    }

    /// TS bit rate the frames carry.
    pub fn ts_rate(&self) -> f64 {
        let frame_s = self.p.frame_samples() as f64 / self.fs();
        self.p.fec_blocks as f64 * (self.p.kbch() - 80) as f64 / frame_s
    }
}

pub struct T2Tx {
    pub mode: Mode,
    packets: Sender<[u8; TS_LEN]>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    _fpga: crate::dvbs2::fpga_tx::RawTransmitter,
}

impl T2Tx {
    /// Start: the FPGA path set up, the modulator thread writing `block`
    /// bytes at a time to `sink`.
    pub fn start(mode: Mode, sink: Sender<TxBlock>, block: usize) -> Result<T2Tx, String> {
        let fs = mode.fs() * OS4 as f64 / 4.0;
        let fpga = crate::dvbs2::fpga_tx::RawTransmitter::start(fs, true)?;
        // About a T2 frame of packets queued ahead of the modulator.
        let per_frame = mode.p.fec_blocks * (mode.p.kbch() - 80) / (8 * TS_LEN) + 2;
        let (tx, rx) = crossbeam_channel::bounded::<[u8; TS_LEN]>(per_frame);
        let stop = Arc::new(AtomicBool::new(false));
        let st = stop.clone();
        let p = mode.p;
        // Frames of bytes to a writer thread of their own, two deep: the
        // next frame is modulated while this one drains into the DMA (which
        // takes it at the DAC's pace; the TX queue behind it holds only
        // tens of ms).
        let (ftx, frx) = crossbeam_channel::bounded::<Vec<u8>>(2);
        let st2 = stop.clone();
        std::thread::Builder::new()
            .name("dvbt2-write".into())
            .spawn(move || write(frx, sink, block, st2))
            .map_err(|e| e.to_string())?;
        // The FEC half (codewords to frame cells) on a thread of its own,
        // one frame ahead of the OFDM half: the two A9 cores in parallel.
        let (ctx, crx) = crossbeam_channel::bounded::<Vec<Cell>>(1);
        let st3 = stop.clone();
        std::thread::Builder::new()
            .name("dvbt2-fec".into())
            .spawn(move || fec(p, rx, ctx, st3))
            .map_err(|e| e.to_string())?;
        let thread = std::thread::Builder::new()
            .name("dvbt2-tx".into())
            .spawn(move || run(p, crx, ftx, st))
            .map_err(|e| e.to_string())?;
        Ok(T2Tx { mode, packets: tx, stop, thread: Some(thread), _fpga: fpga })
    }

    /// Keep the modulator's packet queue full from the mux (the modulator
    /// takes them at the TS rate, paced by the DAC).
    pub fn feed(&self, mux: &mut Mux) {
        while !self.packets.is_full() {
            if self.packets.try_send(mux.next()).is_err() {
                break;
            }
        }
    }
}

impl Drop for T2Tx {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Frames of bytes into `block`-sized TX writes (blocking on the DMA).
fn write(frames: Receiver<Vec<u8>>, sink: Sender<TxBlock>, block: usize, stop: Arc<AtomicBool>) {
    crate::stream::thread_nice(-5);
    let mut rest: Vec<u8> = Vec::new();
    // Time spent waiting for the modulator after the first frames: the DAC
    // runs dry once the TX queue (tens of ms) empties meanwhile.
    let (mut n, mut starved) = (0u64, 0f64);
    loop {
        let t0 = std::time::Instant::now();
        let Ok(f) = frames.recv() else { return };
        n += 1;
        if n > 4 {
            starved += t0.elapsed().as_secs_f64();
        }
        if n % 40 == 0 {
            tracing::info!(frames = n, starved_ms = (starved * 1e3).round(), "DVB-T2 writer");
            starved = 0.0;
        }
        rest.extend_from_slice(&f);
        let whole = rest.len() / block * block;
        for blk in rest[..whole].chunks_exact(block) {
            if sink.send(TxBlock::Raw(blk.to_vec())).is_err() || stop.load(Ordering::Relaxed) {
                return;
            }
        }
        rest.copy_within(whole.., 0);
        rest.truncate(rest.len() - whole);
    }
}

/// TS packets to frame cells (FEC, interleaving, frame builder).
fn fec(p: Params, packets: Receiver<[u8; TS_LEN]>, cells_out: Sender<Vec<Cell>>, stop: Arc<AtomicBool>) {
    crate::stream::thread_nice(-5);
    let mut f = FecStage::new(p);
    let null = {
        let mut n = [0u8; TS_LEN];
        n[0] = 0x47;
        n[1] = 0x1F;
        n[2] = 0xFF;
        n[3] = 0x10;
        n
    };
    let (mut frames, mut nulls, mut fec_s) = (0u64, 0u64, 0f64);
    while !stop.load(Ordering::Relaxed) {
        let t0 = std::time::Instant::now();
        // A packet late from the mux (engine busy) becomes a null packet:
        // the frame must go out on time. At most 20 ms of waiting a frame (a
        // wait a packet stalled whole frames and ran the DAC dry, which
        // moves the frames on air and loses receivers).
        let mut budget = std::time::Duration::from_millis(20);
        let mut next = || {
            if let Ok(p) = packets.try_recv() {
                return p;
            }
            let t = std::time::Instant::now();
            let got = packets.recv_timeout(budget);
            budget = budget.saturating_sub(t.elapsed());
            got.unwrap_or_else(|_| {
                nulls += 1;
                null
            })
        };
        let cells = f.frame(&mut next);
        fec_s += t0.elapsed().as_secs_f64();
        frames += 1;
        if frames % 40 == 0 {
            tracing::info!(frames, nulls, fec_ms = (fec_s / 40.0 * 1e3).round(), "DVB-T2 FEC");
            fec_s = 0.0;
        }
        if cells_out.send(cells).is_err() {
            return;
        }
    }
}

/// Frame cells to 8-bit I/Q bytes (OFDM, conversion) for the writer.
fn run(p: Params, cells_in: Receiver<Vec<Cell>>, frames_out: Sender<Vec<u8>>, stop: Arc<AtomicBool>) {
    crate::stream::thread_nice(-5);
    let o = OfdmStage::new(p, OS4);
    let mut iq = Vec::with_capacity(o.frame_samples());
    // TRXD_T2_TONE=1: a tone 200 kHz above the LO instead (checks the raw
    // FPGA path on an analyser).
    let tone = std::env::var_os("TRXD_T2_TONE").is_some();
    let (mut frames, mut ofdm_s, mut conv_s) = (0u64, 0f64, 0f64);
    let mut ph = 0f64;
    for cells in cells_in.iter() {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        iq.clear();
        let t0 = std::time::Instant::now();
        if tone {
            // A complex exponential by recursion (cheap on the A9).
            let w = std::f64::consts::TAU * 200e3 / (131e6 / 71.0 * OS4 as f64 / 4.0);
            let step = num_complex::Complex32::new(w.cos() as f32, w.sin() as f32);
            let mut z = num_complex::Complex32::from_polar(1.0, ph as f32);
            for _ in 0..o.frame_samples() {
                iq.push(z);
                z *= step;
            }
            ph = z.arg() as f64;
        } else {
            o.frame(&cells, &mut iq);
        }
        ofdm_s += t0.elapsed().as_secs_f64();
        let t1 = std::time::Instant::now();
        let mut bytes = vec![0u8; 2 * iq.len()];
        for (b, z) in bytes.chunks_exact_mut(2).zip(&iq) {
            b[0] = ((z.re * SCALE) as i32).clamp(-127, 127) as i8 as u8;
            b[1] = ((z.im * SCALE) as i32).clamp(-127, 127) as i8 as u8;
        }
        conv_s += t1.elapsed().as_secs_f64();
        frames += 1;
        if frames % 40 == 0 {
            let ms = |s: f64| (s / 40.0 * 1e3).round();
            tracing::info!(frames, ofdm_ms = ms(ofdm_s), conv_ms = ms(conv_s), tone, "DVB-T2 OFDM");
            (ofdm_s, conv_s) = (0.0, 0.0);
        }
        // To the writer (blocks while two frames wait: that is the pacing).
        if frames_out.send(bytes).is_err() {
            return;
        }
    }
}
