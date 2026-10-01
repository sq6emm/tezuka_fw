//! DVB-T2 transmission: the modulator on a thread of its own, its samples
//! (oversampled x1.25) as raw 16-bit IQ into the TX buffer, and the FPGA
//! interpolator resampling them to the DAC rate ([`crate::dvbs2::fpga_tx::RawTransmitter`]).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};

use super::{Cell, FastFrame, FecStage, OfdmStage, Params};
use crate::dvbs2::TS_LEN;
use crate::dvbs2::ts::Mux;
use crate::stream::TxBlock;

/// Oversampling (in quarters) of the modulator's output: none. At x1.25
/// the A9 had 8 % to spare and the DAC ran dry now and then; at x1 the
/// FPGA's 16-tap resampler still puts the images about 44 dB down at the
/// channel edge (more further out, and the AD936x's analog filter adds).
const OS4: usize = 4;
/// The transmitter's threads above the receive decoders (nice -5) and below
/// the sample path (-10): with both on the A9's cores at once (full duplex),
/// a receiver that falls behind (failing blocks run the LDPC to its last
/// iteration) must lose frames of its own, not starve the DAC: a gap in
/// the sent frames broke the receivers of this station and its own.
const TX_NICE: i32 = -8;
/// Engine blocks per TX write (see [`T2Tx::start`]).
const TX_BLOCKS: usize = 16;
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
        if bw_hz == 1_350_000.0 {
            // At 1.35 MHz (1.543 MS/s) 190 data symbols make a 297 ms frame,
            // over T2's 250 ms: 145 hold 7 QPSK blocks (14 16QAM) in 230 ms,
            // with fewer dummy cells (978 kbit/s instead of 974).
            p.data_symbols = 145;
            p.fec_blocks = 7 * constellation.bits() / 2;
        }
        // Rotated QPSK (29 degrees, Q a cell later): free robustness against
        // fading, as T2 intends; receivers read it from L1-post.
        p.rotation = true;
        Some(Mode { bw_hz, p })
    }

    /// L1-post FREQUENCY: the centre frequency on the air, Hz (saturated at
    /// 2^32 - 1: the field has 32 bits, about 4.29 GHz).
    pub fn with_frequency(mut self, hz: f64) -> Mode {
        self.p.frequency_hz = hz.round().clamp(0.0, u32::MAX as f64) as u32;
        self
    }

    /// The standard's channel: 1.7 MHz has its own elementary period
    /// (EN 302 755 9.5, 71/131 us); 2.0 and 1.35 MHz are not T2 channels
    /// (8/7 x bandwidth, the 5-8 MHz rule scaled down: receivers with a free
    /// elementary clock only, not Sony-based TV demodulators).
    pub fn is_standard(&self) -> bool {
        self.bw_hz == 1_700_000.0
    }

    /// Elementary sample rate: 131/71 MHz for the standard 1.7 MHz channel,
    /// 8/7 x bandwidth otherwise (the amateur convention, not in EN 302 755).
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
    /// The FPGA's IFFT does the OFDM (the A9 is left about half a core:
    /// a receiver can run beside it).
    pub fpga_ifft: bool,
    packets: Sender<[u8; TS_LEN]>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    _fpga: crate::dvbs2::fpga_tx::RawTransmitter,
}

impl T2Tx {
    /// Start: the FPGA path set up, the modulator thread writing `block`
    /// bytes at a time to `sink`.
    pub fn start(mode: Mode, sink: Sender<TxBlock>, block: usize, drive_db: f32) -> Result<T2Tx, String> {
        // TX writes of 16 engine blocks: the TX queue (8 writes deep) then
        // holds about 350 ms of T2 instead of 30. A receiver beside the
        // transmitter (full duplex) delays its threads now and then by more
        // than 30 ms; each time the DAC ran dry between frames and every
        // receiver (this station's too) lost the frames around it. The
        // latency added is nothing for video.
        let block = block * TX_BLOCKS;
        let fs = mode.fs() * OS4 as f64 / 4.0;
        // The bitstream's transmit IFFT when there is one (it takes the
        // OFDM half off the A9: about 60 % of a core); TRXD_NO_T2IFFT=1 keeps
        // it on the CPU.
        use crate::dvbs2::fpga_tx::{has_t2ifft, RawMode};
        // TRXD_T2_TONE=1: a tone 200 kHz above the LO instead (checks the
        // raw FPGA path on an analyser; samples, so no IFFT).
        let tone = std::env::var_os("TRXD_T2_TONE").is_some();
        let ifft = OS4 == 4 && !tone && std::env::var_os("TRXD_NO_T2IFFT").is_none() && has_t2ifft();
        let fpga = crate::dvbs2::fpga_tx::RawTransmitter::start(fs, if ifft { RawMode::T2Ifft } else { RawMode::Samples8 })?;
        tracing::info!(fpga_ifft = ifft, "DVB-T2 transmitter");
        // About a T2 frame of packets queued ahead of the modulator.
        let per_frame = mode.p.fec_blocks * (mode.p.kbch() - 80) / (8 * TS_LEN) + 2;
        let (tx, rx) = crossbeam_channel::bounded::<[u8; TS_LEN]>(per_frame);
        let stop = Arc::new(AtomicBool::new(false));
        let st = stop.clone();
        let p = mode.p;
        // The FEC half (codewords to frame cells, or with the FPGA's IFFT
        // the interleaved data cells) on a thread of its own, one frame
        // ahead of the OFDM half: the two A9 cores in parallel.
        let (ctx, crx) = crossbeam_channel::bounded::<Vec<Cell>>(1);
        let st3 = stop.clone();
        std::thread::Builder::new()
            .name("dvbt2-fec".into())
            .spawn(move || fec(p, rx, ctx, st3, ifft))
            .map_err(|e| e.to_string())?;
        let scale = SCALE * 10f32.powf(drive_db.clamp(-20.0, 6.0) / 20.0);
        let thread = if ifft {
            // The FPGA's IFFT: frames gathered straight into the TX blocks
            // (FastFrame), the blocking send paces it.
            std::thread::Builder::new()
                .name("dvbt2-tx".into())
                .spawn(move || run_fpga(p, crx, sink, block, st, scale))
                .map_err(|e| e.to_string())?
        } else {
            // Frames of bytes to a writer thread of their own, two deep:
            // the next frame is modulated while this one drains into the DMA
            // (which takes it at the DAC's pace; the TX queue behind it
            // holds only tens of ms).
            let (ftx, frx) = crossbeam_channel::bounded::<Vec<u8>>(2);
            let st2 = stop.clone();
            std::thread::Builder::new()
                .name("dvbt2-write".into())
                .spawn(move || write(frx, sink, block, st2))
                .map_err(|e| e.to_string())?;
            std::thread::Builder::new()
                .name("dvbt2-tx".into())
                .spawn(move || run(p, crx, ftx, st, scale))
                .map_err(|e| e.to_string())?
        };
        Ok(T2Tx { mode, fpga_ifft: ifft, packets: tx, stop, thread: Some(thread), _fpga: fpga })
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
    crate::stream::thread_nice(TX_NICE);
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

/// TS packets to frame cells (FEC, interleaving, frame builder), or with
/// `codes` a frame's data cells for [`FastFrame`].
fn fec(p: Params, packets: Receiver<[u8; TS_LEN]>, cells_out: Sender<Vec<Cell>>, stop: Arc<AtomicBool>, codes: bool) {
    crate::stream::thread_nice(TX_NICE);
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
        let cells = if codes { f.frame_codes(&mut next) } else { f.frame(&mut next) };
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

/// Frames' data cells to the FPGA IFFT's words, straight into `block`-byte
/// TX writes (blocking on the DMA).
fn run_fpga(p: Params, codes_in: Receiver<Vec<Cell>>, sink: Sender<TxBlock>, block: usize, stop: Arc<AtomicBool>, drive: f32) {
    crate::stream::thread_nice(TX_NICE);
    assert!(block % 4 == 0);
    let o = OfdmStage::new(p, OS4);
    // Debug: `echo 68 > /tmp/t2-scale` drives harder (RMS in 8-bit units).
    let scale: f32 = std::fs::read_to_string("/tmp/t2-scale").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(drive);
    if scale != SCALE {
        tracing::info!(scale, "DVB-T2 TX scale (trx.t2_drive_db, or /tmp/t2-scale)");
    }
    let ff = FastFrame::new(p, o.ofdm(), scale);
    drop(o);
    let fb = ff.frame_bytes();
    let (mut frames, mut fill_s, mut starved) = (0u64, 0f64, 0f64);
    // the TX queue's lowest depth (0: the DAC may have run dry)
    let mut min_q = usize::MAX;
    let mut cur: Option<Vec<Cell>> = None;
    let mut at = 0;
    loop {
        let mut blk = vec![0u8; block];
        let mut filled = 0;
        while filled < block {
            let codes = match &cur {
                Some(c) => c,
                None => {
                    let t = std::time::Instant::now();
                    let Ok(c) = codes_in.recv() else { return };
                    if frames > 4 {
                        starved += t.elapsed().as_secs_f64();
                    }
                    cur.insert(c)
                }
            };
            let n = (block - filled).min(fb - at);
            let t = std::time::Instant::now();
            ff.fill(codes, frames as usize, at, &mut blk[filled..filled + n]);
            fill_s += t.elapsed().as_secs_f64();
            (at, filled) = (at + n, filled + n);
            if at == fb {
                (cur, at) = (None, 0);
                frames += 1;
                if frames % 40 == 0 {
                    let ms = |s: f64| (s / 40.0 * 1e3).round();
                    tracing::info!(frames, fill_ms = ms(fill_s), starved_ms = ms(starved), min_queue = min_q, fpga_ifft = true, "DVB-T2 OFDM");
                    (fill_s, starved, min_q) = (0.0, 0.0, usize::MAX);
                }
            }
        }
        if frames > 4 {
            min_q = min_q.min(sink.len());
        }
        if stop.load(Ordering::Relaxed) || sink.send(TxBlock::Raw(blk)).is_err() {
            return;
        }
    }
}

/// Frame cells to 8-bit I/Q bytes (OFDM on the CPU, conversion) for the
/// writer.
fn run(p: Params, cells_in: Receiver<Vec<Cell>>, frames_out: Sender<Vec<u8>>, stop: Arc<AtomicBool>, drive: f32) {
    crate::stream::thread_nice(TX_NICE);
    let o = OfdmStage::new(p, OS4);
    let mut iq = Vec::with_capacity(o.frame_samples());
    let tone = std::env::var_os("TRXD_T2_TONE").is_some();
    // Debug: `echo 68 > /tmp/t2-scale` drives harder (RMS in 8-bit units).
    let scale: f32 = std::fs::read_to_string("/tmp/t2-scale").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(drive);
    if scale != SCALE {
        tracing::info!(scale, "DVB-T2 TX scale (trx.t2_drive_db, or /tmp/t2-scale)");
    }
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
            b[0] = ((z.re * scale) as i32).clamp(-127, 127) as i8 as u8;
            b[1] = ((z.im * scale) as i32).clamp(-127, 127) as i8 as u8;
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
