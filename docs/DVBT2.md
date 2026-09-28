# DVB-T2 (amateur narrow channels)

trxd sends DVB-T2 (EN 302 755) the way amateur DATV uses it: the Portsdown 4
DVB-T2 option's profile, received by the Ryde, the Knucker and the Lynx
(Raspberry Pi TV HAT, Sony CXD2880) receivers and by T2 TV tuners at 1.7 MHz.

| | |
|---|---|
| Channel | 1.7 MHz (standard, 131/71 MS/s), or 2.0 / 1.35 MHz (sample rate 8/7 x bandwidth) |
| OFDM | 2K, normal carriers, guard 1/8, pilot pattern PP2, SISO |
| Frame | P1, 8 P2 symbols, 190 data symbols (about 248 ms at 1.7 MHz), 2 frames a super-frame |
| PLP | one, TS, normal FEC frames (64800), QPSK or 16QAM, 1/2 or 3/4, rotated constellation (29 / 16.8 degrees), one TI block, 9 (QPSK) or 18 (16QAM) FEC blocks a frame |
| L1 | L1-pre BPSK, L1-post QPSK 1/2 (16K LDPC), version 1.1.1 |
| TS rate at 1.7 MHz | QPSK 1.164 (1/2) / 1.751 (3/4) Mbit/s, 16QAM 2.328 / 3.502 Mbit/s |

UI: DATV panel, code rate list: "DVB-T2 1.7 MHz QPSK 1/2" and friends
(`T2-<MHz>-QPSK-<rate>`; the symbol rate setting is ignored). The same
browser capture and TS mux as DVB-S2 feed it.

## How it is built

```text
browser H.264/Opus -> dvbs2::ts::Mux (TS at the T2 rate)
  -> dvbt2-tx thread: Modulator (src/dvbt2): BBFRAME (dvbs2::Framer), BB
     scrambling, BCH, LDPC (the DVB-S2 normal codes), QPSK cells, cell and
     time interleaving, frame builder with L1-pre/post, frequency
     interleaving, pilots + 2048 IFFT, guard interval, P1
  -> 8-bit I/Q, four samples a 64-bit DMA word -> dvbt2-write thread -> TX DMA
  -> FPGA: datv_split -> FIFO -> datv_raw (DAC GPIO bit 2, 8-bit: bit 3)
     -> datv_tx, the arbitrary-rate interpolator of DVB-S2, loaded with a
        low-pass (sinc, Kaiser 6, 16 taps) instead of the RRC: 131/71 MS/s
        -> 3.072 MS/s -> datv_merge -> DAC
```

- The modulator is a port of GNU Radio gr-dtv's DVB-T2 blocks, checked
  against them stage by stage (`dvbt2::tests::t2_matches_gr_dtv`, reference
  script `datv-ref/t2/ref/t2ref.py` in the gr-dtv Docker image `datv-t2:1`):
  every LDPC codeword and cell bit-exact, output within 1e-6, for QPSK 1/2
  and 3/4 and 16QAM 1/2, plain and rotated (16QAM: parity interleaving,
  column twist, demux, 16200-cell interleaver).
- Cells travel as one-byte codes through the interleavers and the frame
  builder (complex cells made the A9 wait on memory more than on the IFFTs).
- Three threads: FEC (codewords to frame cells), OFDM (frequency
  interleaving, pilots, IFFTs, 8-bit conversion) and the DMA writer, each a
  frame ahead of the next. A9 per 248 ms frame: 16QAM FEC about 210 ms
  (including waits for TS packets) alongside OFDM 127 + conversion 42 ms;
  QPSK about half the FEC.
- The IIO TX path tops out near 7 MB/s: 16-bit I/Q at 2.3 MS/s did not fit,
  8-bit at 1.845 MS/s (3.7 MB/s) does. Level: RMS 48 of 127 (peaks above 2.65
  sigma clip), low-pass at unity gain.
- The AD936x TX analog filter opens to 1.3 x the channel (2.21 MHz) while T2
  transmits and returns to the configured 1 MHz afterwards.
- Images of the 1.845 MS/s stream: the 16-tap resampler puts them about
  44 dB down at the nearest edge (more further out), the analog filter adds.

## Checked

- `dvbt2::rx` (tests only): a receiver for this profile (P1 in coherent
  chunks, frequency from the guard intervals, channel from the P2 pilots,
  the transmitter's permutations undone, the DVB-S2 LDPC/BCH/BBFRAME chain).
  - Loopback with 3 kHz offset: QPSK at 15 dB and 16QAM at 22 dB SNR,
    plain and rotated, every packet.
  - Through the FPGA path's integer model (8-bit, the low-pass resampler):
    MER 39 dB, every packet.
- Over the air (2026-09-27, Libre 1 -> Libre 2, 1255 MHz, T2 1.7 MHz QPSK
  1/2, browser camera): Libre 2 recorded 2 s raw at 3.072 MS/s (`trxd
  --capture-iq`, trxd stopped), decoded offline (`t2_capture`): 7 frames,
  1150 TS packets (video PID 256, audio 257, nulls), 54 of 63 FEC blocks, L1
  MER 4-5 dB: the indoor path was this weak (the analyser sees the channel
  at -60 dBm in 30 kHz; the S2 signal at 250 kS/s is about 10 dB stronger
  in total, OFDM's peaks need the headroom).
- Rotated (the default since): 3 s recording, 11 frames, 1533 TS packets
  (PAT, SDT, PMT, video, audio), 72 of 99 FEC blocks at L1 MER about 5 dB
  (the test receiver's channel estimate uses the P2 pilots only).
- With the receiver's channel interpolation fixed (it counted the pilot
  carriers twice: harmless to QPSK, fatal to 16QAM): 3 s at L1 MER 2-3 dB,
  1919 TS packets, 90 of 99 FEC blocks.
- Analyser: flat 1.54 MHz block, sharp edges (QPSK and 16QAM).

## Receiving (live, FPGA front end)

trxd receives the same profile live (DATV panel, receive, a T2 rate), with
the FPGA doing the heavy signal processing (maia-sdr `datv-ddc`:
t2resamp.py, t2ofdm.py) and the A9 the rest (`src/dvbt2/stream.rs`,
`fe.rs`):

- FPGA: the ADC stream resampled to 131/71 MS/s; a 32-bit sample counter;
  once trxd has given it a frame start and a frequency, an NCO, each
  symbol's 2048-point FFT (window timed from the frame start, 64 samples
  into the guard interval) with only the 1705 active carriers sent, and raw
  samples only around P1 and the guard intervals; every sample raw while
  searching. Ring words tagged (header / carrier stream).
- A9: P1 search in the raw samples, the frequency from the guard
  intervals, the front end scheduled a few frames ahead; then per frame the
  P2 channel estimate, per symbol equalization (common phase and timing
  slope from the pilots), one gather for all deinterleavers, LLRs; the
  FPGA's LDPC decoder, BCH, the TS demultiplexer. The frequency is tracked
  frame to frame (a jump only when three frames agree); a missed P1 six
  times running goes back to searching.
- The MER shown is the pilots' after the equalizer, in data-cell units.

Fixes that made it work (2026-09-27): Maia's ADC input dropped the FIFO's
valid (about 1 % of samples duplicated); the resampler needed an input
FIFO; the RX analog filter must open to 1.3 x the channel; the FFT and its
window labels must restart whenever no schedule runs, and a schedule that
starts in the past must skip that frame (its symbol counters started at 0
mid-frame); BCH after LDPC rescues most blocks the FPGA's decoder leaves a
few bits short, and a splitting test before the Chien search keeps its cost
at 2-3 ms a block on the A9 (it was 67).

Over the air between the Libres (QPSK 1/2, 1.7 MHz, indoor, 2026-09-27):

| Band | Libre 1 -> Libre 2 | Libre 2 -> Libre 1 |
|---|---|---|
| 437 MHz | MER up to 8 dB, some video | MER up to 3 dB |
| 1255 MHz | Libre 2 has local interference on 23 cm | MER about 0 dB |
| 2330 MHz | MER 12-16 dB, 90 % of FEC blocks, 596 video frames in 90 s | MER up to 6 dB |
| 2400 MHz | WiFi | MER up to 6 dB |
| 3405, 5760 MHz | nothing (the antennas) | nothing |

With a few dB of margin every block decodes (36 a second); what is lost is
lost to fading on the indoor path. A9: the demodulator about 70 % of a
core, the FEC about 31 ms a block (FPGA LDPC 22 ms of it).

## Not done

- 64QAM/256QAM, other FFT sizes, PAPR reduction.
- 16QAM live: decoded over the air at 2330 MHz (MER about 10 dB, video
  shown), but its 72 FEC blocks a second are twice what the A9 and the
  FPGA's LDPC decoder get through: about half are dropped.
- The receiver's A9 load: equalization and LLRs could move into the FPGA,
  and a parallel (layered) LDPC decoder would cut the 22 ms a block; the
  deinterleavers need a frame of cells (DDR, not block RAM).
- A check with an independent T2 receiver (TV HAT / Ryde).
