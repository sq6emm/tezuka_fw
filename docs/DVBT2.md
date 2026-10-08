# DVB-T2 (amateur narrow channels)

trxd sends DVB-T2 (EN 302 755) the way amateur DATV uses it: the Portsdown 4
DVB-T2 option's profile, received by the Ryde, the Knucker and the Lynx
(Raspberry Pi TV HAT, Sony CXD2880) receivers and by T2 TV tuners at 1.7 MHz.

| | |
|---|---|
| Channel | 1.7 MHz (standard, 131/71 MS/s), or 2.0 / 1.35 MHz (non-standard: not in EN 302 755, sample rate 8/7 x bandwidth, nothing in P1 or L1 says so; only receivers whose elementary clock can be set freely decode them, not Sony CXD2880-based ones (Raspberry Pi TV HAT and similar) or TV tuners: use 1.7 MHz for third-party receivers) |
| OFDM | 2K, normal carriers, guard 1/8, pilot pattern PP2, SISO |
| Frame | P1, 8 P2 symbols, 190 data symbols (248 ms at 1.7 MHz, 200 ms at 2.0 MHz); 1.35 MHz: 145 data symbols, 7 QPSK / 14 16QAM blocks (230 ms: 190 made 297 ms, over T2's 250 ms); 2 frames a super-frame |
| PLP | one, TS, normal FEC frames (64800), QPSK or 16QAM, 1/2 or 3/4, rotated constellation (29 / 16.8 degrees), one TI block, 9 (QPSK) or 18 (16QAM) FEC blocks a frame (1.35 MHz: 7 / 14) |
| L1 | L1-pre BPSK, L1-post QPSK 1/2 (16K LDPC), version 1.1.1; L1-post FREQUENCY the real centre frequency (`Mode::with_frequency`; 32 bits, saturated above 4.29 GHz, so 5.76 GHz cannot be signalled) |
| TS rate at 1.7 MHz | QPSK 1.164 (1/2) / 1.751 (3/4) Mbit/s, 16QAM 2.328 / 3.502 Mbit/s |

UI: DATV panel, code rate list: "DVB-T2 1.7 MHz QPSK 1/2" and friends
(`T2-<MHz>-QPSK-<rate>`; the symbol rate setting is ignored). The same
browser capture and TS mux as DVB-S2 feed it.

## How it is built

```text
browser H.264/Opus -> AAC-LC on the board -> dvbs2::ts::Mux (TS at the T2 rate)
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

## FPGA IFFT, lighter A9 work and full duplex (2026-09-28)

- Transmit: with the bitstream's `t2ifft` (maia-hdl `t2ifft.py`, DAC GPIO
  bit 4, id "DTX2") the A9 sends each frame's carriers (16-bit words: sync
  word, P1, per symbol 1705 carriers in bin order) and the FPGA does the
  IFFTs and guard intervals. `TRXD_NO_T2IFFT=1` keeps the old path.
- FEC on packed bits: LDPC as 360-bit row rotate-XORs with the
  accumulation done by prefix XORs (`ldpc_fpga::parity_rows`), BCH a byte at
  a time, the 16QAM bit interleaver as 8 x 8 bit transposes from the
  parity-interleaved codeword. `FastFrame` (src/dvbt2/mod.rs): everything
  after the cell interleaver (time and frequency interleavers, frame
  builder, pilots, bin order) is the same in every frame, so it is run once
  on labels at start-up; a frame is then one gather straight into the TX
  blocks (no writer thread, no frame-sized copies). Bit-exact with the
  stage-by-stage path (`t2_fec_fast_matches_ref`, `t2_fast_frame_matches`)
  and with gr-dtv. TX board trxd: 81 % -> about 40 % of a core.
- Receive: the data cells equalized in fixed point from the front end's
  16-bit carriers (the A9's VFP takes about 40 ns a complex multiply), the
  early-window rotation folded into the channel inverse, fast paths for
  runs of carrier and raw words: demodulator 155 -> about 117 ms a frame.
- The TX queue now holds about 350 ms of T2 (TX writes of 16 engine blocks)
  and the T2 threads run above the receive decoders (nice -8): with only
  30 ms queued, a receiver beside the transmitter delayed it now and then
  and the DAC ran dry between frames (P1s 458240 + 4k..56k samples apart:
  every receiver, the board's own too, lost frames).
- Full duplex: the receiver keeps running while T2 transmits with the FPGA
  IFFT (as with the FPGA's DVB-S2); split (SPLIT, VFO A receive, VFO B
  send) across bands tunes the AD936x's RX and TX synthesizers apart
  (`RadioControl::set_los`, log "LOs apart"). Checked on the Libres: each
  receives its own T2 at MER about 31 dB with video; DVB-S2 cross band
  437/2330 MHz on both boards at once. Cross-band T2 between the boards is
  limited by the link here (Libre 2 -> Libre 1 at 2330 MHz arrives 13 dB
  weaker than the other way) and by the own transmitter at full power
  desensing the receiver (fine at 40 dB attenuation), and the FEC below.
- FEC throughput: T2 hands the decoder 9 blocks (QPSK) or 18 (16QAM) at
  each frame end; the serial FPGA LDPC decoder (2.5 ms an iteration) plus
  loading the LLRs (8-10 ms a block over AXI-Lite) could not keep up. The
  four-lane decoder (maia-hdl `ldpc_dec4.py`, "LDP4", 0.66 ms an
  iteration, bit-exact with the model) is the next step.
- Harness: `datv-ref/harness/run_fdx.sh` (both boards full duplex at once,
  CPU per thread), `cdp_fdx.py` (one board; the UI's script is private, so
  it drives the buttons).

- Equalizer in the FPGA (maia-hdl `t2eq.py`, 2026-09-28): between the
  front end's FFTs and the ring. Per data symbol z = c G (G the channel
  inverse the ARM loads after each frame's P2 symbols: two banks, flipped
  at a symbol's start), the timing slope from the scattered pilots (angle
  of the neighbours' products / D, a CORDIC), the common phase, then 7-bit
  cells (unit 20) two to a word; a flag in the carrier header marks those
  symbols (the frame start is 21 bits now). Two symbol banks: a symbol's
  carriers follow the last one's without a gap (the FFT's latency). The P2
  symbols and every symbol before the first table go through as before
  (the A9's equalizer stays for them). Bit-exact with its model
  (`test_t2eq.py`, at the front end's pace); the model's cells MER 37 dB.
  Receive board: datv-rx 57-60 -> about 40 % of a core, the demodulator
  busy 41 % of the time instead of all of it; decoded over the air.
- Cross-band T2 full duplex between the two boards (2330 / 437 MHz): each
  direction decodes with the right attenuation (the board sending 437
  desenses its own 2330 receiver at full power), both at once not with
  these antennas: link, not firmware.

- Receive, later the same day: cells go only through the time
  deinterleaver as they come (about 45 short runs a symbol; stores straight
  to FEC block order hit the whole 580 KB frame at random, 30 ms a frame),
  the cell deinterleaver runs per block with the LLRs; the LLRs are
  computed in fixed point straight into the LDPC decoder's 6 bits (the FEC
  thread no longer quantizes a float vector: 6.4 -> 3.5 ms a block).
  Beware on the A9: f32::max/min/clamp are libm calls (fmaxf/fminf) in
  armv7 code, about 40 ns each. Demodulator 155 -> 63 ms a frame over the
  day; the receive board's trxd about 90 % of a core while receiving T2.

## LLRs and the LDPC decoder's input in the FPGA (2026-09-29)

The FEC thread spent 3.5 ms a block writing LLR words into the decoder
through its AXI4-Lite window and 1.3 ms reading the decisions back. The
decoder (maia-sdr `ldpc_dma.py`, id "LDP5") now takes them from DDR: trxd
writes into a reserved 1 MB (device tree `ldpc_buffers`, 0x16300000) in
0.2 ms, the decoder loads them over an AXI3 master (HP0), decodes and
writes the decisions back packed 32 to a word. For QPSK the demodulator
does not make LLRs at all any more: it hands over the block's cells (after
the deinterleavers, `T2Block::Cells`) and the decoder's engine makes the
LLRs itself, bit for bit trxd's fixed-point formula (rotation back by
c14/s14, the cyclic Q delay, x kq, rounding, +-31; `qpsk_llrs` is the one
copy in trxd, for the software fallback), writing each into the decoder's
RAM layout (the parity banked). 16QAM too (id "LDP6", 0xFF20 bit 3, the
level a14 at 0xFF2C): four LLRs a cell (I, Q, |I| - a, |Q| - a, max-log)
and the bit deinterleaver on the way into the RAM: a row pair's 8 LLRs
through the demux to the 8 columns, each column a counter with the column
twist's start (i = row - twist mod 8100), the parity columns as t, s (the
parity interleaver and the decoder's banked layout cancel: word K/4 + 90 t
+ s/4). trxd's model is `stream::cell_llrs` (with `BitInterleaver`).

Checked: HDL against the model (`test_ldpc_dma.py`: DMA decode, cells
decode, every RAM word of rotated cells, QPSK and 16QAM at both rates;
`t2_loopback_cells` for trxd's side); on Libre 2 `cells_vs_llr`
(trxd-test) decodes the same noisy block both ways on the FPGA: the same
decisions and iterations. Over the air Libre 1 -> Libre 2 at 2330 MHz,
QPSK 1/2: decoded with video; A/B runs with TRXD_NO_LDPC_CELLS=1 are within
the indoor link's swing (no skipped or FEC-busy blocks either way).

| FEC thread, a block | before | DMA | + LLRs in the FPGA |
|---|---|---|---|
| LLRs in | 3.46 ms | 1.24 ms | 0.38 ms |
| decisions out | 1.30 ms | 0.41 ms | 0.39 ms |
| FEC thread (web UI) | 33 % | 23 % | 14-30 % (with the MER) |

Knobs: TRXD_NO_LDPC_DMA=1 (the window as before), TRXD_NO_LDPC_CELLS=1
(LLRs on the A9, into DDR).

Libre 2 -> Libre 1 at 2330 MHz does not lock (the known weak direction,
about 13 dB down); each board receives its own T2 at MER 29 dB.

## Cell router: the data cells never reach the A9 (2026-09-29 night)

With the LLRs in the FPGA the demodulator's heaviest work was moving cells:
the time-deinterleaver scatter as each symbol came (17.6 ms a frame) and
the cell-deinterleaver gather per block (10 ms), both random access over a
580 KB frame (47 ms a frame in all, `t2_fe_speed` T2EQ=1 on Libre 2). The
datv bitstream's cell router (maia-sdr `t2router.py`, 0x43C60000, "T2R1")
takes the equalizer's words (tapped out of the Maia core, t2_eq_data)
across into the CPU clock and writes every data cell of an equalized
symbol straight to its place in its FEC block in DDR, from a table trxd
makes once (`Demod::enable_router`: entry (symbol, carrier) = the time and
cell deinterleavers in one, `CellInterleaver::deinterleaved_index`,
checked against `block_gather` in `deinterleaved_index_matches_gather`).
Four frame buffers (device tree `t2router_buffers`, 4 MB at 0x16400000:
the table, then the buffers), a new one at each frame start; a buffer is
complete once every data symbol went out and was answered. The A9 writes
only the P2 symbols' data cells (its own equalizer's) and hands the FEC
thread `T2Block::Ddr` (the block's address); the LDPC engine reads the
cells from there and makes the LLRs.

With the router the A9 needs only the pilots (the MER) and a few cells
for the browser: it reads one data symbol in four (`MER_EVERY`) and skips
the other symbols' words unread (demodulator 17 -> 10-16 % of a core with
16QAM). The demodulator's log line carries `prof_ms` (per frame: p1, fft,
equalize, deinterleave, llr, input).

TRXD_NO_T2ROUTER=1: the A9 path; TRXD_T2ROUTER_CHECK=1: the A9 scatters and
gathers too and compares the first frames' blocks with the router's.

Checked on the Libres: the check found no cell different in any frame
(291 600 a frame, P2's included); Libre 2 receiving itself 1773/1773
frames; Libre 1 -> Libre 2 at 2330 MHz 1458/1584 frames (the link), one
frame missed by the router at the start (acquisition). The receive
board's demodulator: about 13 % of a core (30-35 % with the A9's
deinterleavers), the FEC thread about 10 %.

| DVB-T2 receive (QPSK 1/2, 1.7 MHz) | demodulator | FEC thread |
|---|---|---|
| 2026-09-28 (equalizer in the FPGA) | 30-35 % | 33 % |
| + LLRs and the decoder's input in the FPGA | 30 % | 14-25 % |
| + cell router | 13 % | ~10 % |

16QAM with the router and the LDPC engine's 16QAM cells (LDP6, both
Libres flashed 2026-09-29 evening): on Libre 2 `cells16_vs_llr` gives the
same decisions from cells as from trxd's LLRs at 1/2 and 3/4. Over the air
Libre 1 -> Libre 2 at 2330 MHz, 16QAM 1/2 rotated: 72 FEC blocks a second,
none skipped or FEC-busy (before: about half dropped for time); the LDPC
decoder 6 ms a block (budget 13.8), the demodulator 17-18 % of a core;
the blocks lost are the indoor link's (MER swinging 4-16 dB: 95 % of the
blocks at MER 12, fewer below). 20 minutes through the FPGA switch path
(the receiver started from the voice bitstream): 71 355 of 81 684 blocks,
memory flat.

Those losses (bursts of 8 frames every 10-40 s) turned out to be two
bugs, not the link:

- Receiver: the front end's P1 tracking moved the frame timing to false
  peaks in a fade (42 or 63 samples, the edge of its window), and every
  move was made twice (the check after a move still measured a frame on
  the old timing). Now a large offset (over 8 samples) must be seen by two
  checks in a row, and the check after a move is skipped; the clocks'
  drift (3 samples every 8-10 s between the Libres) is followed as before.
  Each move is logged ("frame moved to its P1"), the demodulator line
  counts `resched` and `retunes`.
- Sender: with a receiver beside the transmitter (full duplex, the page's
  own receiver) both cores ran about 92 % busy and the thread feeding the
  DAC (nice -10 only) waited long enough for the kernel's two TX buffers
  to run dry: Libre 1 receiving itself at MER 29 lost the same bursts as
  Libre 2. The feeder now runs SCHED_FIFO (`stream::fifo_thread`; it
  sleeps in its blocking write). The writer's log line carries the TX
  queue's lowest depth (`min_queue`, always full: the loss was below it).

| Libre 1 -> Libre 2, 2330 MHz, 16QAM 1/2, 3 min | blocks decoded |
|---|---|
| before | 85-89 % |
| P1 tracking fixed | 92.8 % |
| Libre 1 sending alone (TRXD_TX_ALONE=1) | 99.0 % |
| both fixes, receiver beside the sender | 98.7 % (Libre 1's own: 36 bad of 12 204, at the start) |

QPSK 1/2 after both: 2718 of 2718 blocks in 90 s.

## Every mode on air (2026-09-30)

Libre 1 -> Libre 2 at 2330 MHz, 90 s each, all through the FPGA path:

| Mode | Blocks decoded |
|---|---|
| 1.7 MHz QPSK 1/2 | 2718/2718 |
| 1.7 MHz QPSK 3/4 | 2234/2376 (one 15 s dropout) |
| 1.7 MHz 16QAM 1/2 | 98.7 % (3 min) |
| 1.7 MHz 16QAM 3/4 | 5552/5796 |
| 2.0 MHz QPSK 1/2 | 2895/3258 |
| 2.0 MHz 16QAM 1/2 | 6298/6534 |
| 1.35 MHz QPSK 1/2 | 2465/2502 |

The dropouts were on the path or Libre 2's receive side: Libre 1's own
receiver beside the transmitter lost nothing at those moments.

## Compliance with EN 302 755 (reviewed 2026-09-30)

The 1.7 MHz modes follow the standard in every field and table checked:
L1-pre (200 bits) and L1-post, mode adaptation (normal mode, TS, CCM), BCH
and LDPC, the 16QAM bit interleaver, rotation and Q delay, cell, time and
frequency interleavers, frame builder, pilots (PP2, continual, edge, P2),
P1, the frame closing symbol GI 1/8 + PP2 requires; gr-dtv agrees bit for
bit (`t2_matches_gr_dtv`: QPSK 1/2 and 3/4, rotated, 16QAM 1/2, and
rotated 16QAM 3/4 since 2026-09-30, reference o1634r). L1-pre NETWORK_ID is
the transport stream's network (0xFF01) and T2_SYSTEM_ID 0x0001 (Params;
the gr-dtv comparison sets gr-dtv's fixed 0x3085 / 0x8001).

Outside the standard: the 2.0 and 1.35 MHz channels (amateur: the clock
scaled, not signalled in L1). The 1.35 MHz frame was 297 ms, over the
250 ms limit; it now has 145 data symbols (230 ms). The FPGA's transmit
IFFT takes frames of any length up to 198 symbols (a sync word where the
next symbol would start ends the frame; `test_short_frames`), and
`t2_modes_fit_and_short_frames_decode` checks every mode's frame length and
capacity and decodes a 1.35 MHz frame. Not yet confirmed by an independent
receiver: the spectrum's orientation on air (a Libre-to-Libre link would
not notice an inversion).

## Receiver's L1 check (2026-10-01)

Every frame the receiver decodes L1-pre and L1-post (`l1::PreDecoder`,
`PostDecoder`: BCH on the hard decisions, the 16K LDPC when that fails,
then the CRC-32) and compares the fields its fixed layout depends on (S1,
S2, guard, PAPR, L1 modulation and size, pilot pattern, data symbols, T2
version; PLP count, type, code rate, modulation, rotation, FEC type,
blocks, time interleaving) with what it is set for. A frame that signals
something else is not taken (`l1_mismatch`, logged once per change):
another transmitter's frames would otherwise be read with the wrong
layout. NETWORK_ID, T2_SYSTEM_ID and FREQUENCY are the other station's
own and do not count. `t2_loopback` and `t2_through_the_front_end`
require every frame's L1 to decode.

## Receiver limits (review 2026-10-01)

The channel comes from the P2 pilots once a frame (about 248 ms); the
scattered and continual pilots of the data symbols give each symbol's
common phase and timing slope only. Equalization is zero-forcing, the LLRs
carry one noise level for the frame (no per-cell channel state) and
rotated cells are de-rotated and sliced per axis. Fine on a static path;
fragile on a moving one (aircraft scatter, portable). Improving it is FPGA
work: the equalizer (`t2eq`) runs in the fabric and the cells go from it to
DDR and the LDPC engine without the A9 (the cell router), so time
interpolation over the scattered pilots, a per-cell CSI word next to each
cell and a CSI-weighted (per-component for rotated QPSK/16QAM) demapper in
the LDPC engine's LLR stage all need HDL (t2eq, t2router, ldpc_dma), with
the software model (`fe.rs`, `stream.rs`) changed to match.

## Capture range (2026-10-08)

P1's coarse frequency (`find_p1_in`: the phase step between 16 coherent
chunks of 128 samples) wraps at fs/128 = 14.4 kHz, so a carrier more than
7.2 kHz off locked 8 carriers wrong: P1, GI and the frequency all looked
fine, but no L1 ever decoded (R2 to Libre 2 at about -9 kHz, 2026-10-08;
the simulation sweep `t2_offset_sweep` showed it at +/-7.5 kHz). The
demodulator now treats L1 failing twice in a row, with no L1 decoded since
the lock, as the wrong multiple and acquires again with the next one
(0, +1, -1, +2, -2 x 14.4 kHz); a hypothesis that decoded stays until the
lock is lost. Range about +/-12 kHz (P1 itself fades beyond that: one
128-sample chunk then turns by a full cycle); the lock takes a few frames
longer there (`t2_through_the_front_end_9_khz_off`, with and without the
FPGA reports).

## Searching and tracking in the FPGA: P1, GI and MER reports (2026-10-03)

Profile of the locked receiver before (Libre, per frame): P1 work 245 ms,
word loop 71 ms, equalized symbols 10 ms; demodulator 61 % of a core.
Searching cost 99.8 % of a core (raw samples, every one through the
ring and the A9's P1 correlator), so a gap meant more gaps (re-acquisition
overloaded the A9) and in full duplex the frame end came too late for the
router (751 ms finish, software fallback).

The front end (maia-hdl `t2ofdm.py`) now does that work itself and puts
short records in the carrier stream:

- **P1 detector** (`t2p1.py`): trxd's structure correlation, streaming
  from one 2048-sample delay line (taps 482, 964, 1024, 1506, 2048): C
  over A's last 542 samples against C, B over its last 482 against B, the
  1/1024 shift from the NCO table, E the energy of 2048 samples; every
  sum adds a term and later takes the same one away, so they stay exact
  (48 bits). Score (|C| + |B|) >> 10 - k (E >> 10) >> 8 (k = 64), the
  best window of each frame-length block reported (record 253: start,
  |C| + |B|, E). About 16 cycles a sample, four multipliers.
- **Guard-interval correlation** (same unit): the sum of x[t] conj(x[t -
  2048]) over the scheduled symbols' last guard-interval samples, raw
  (before the NCO), >> 16 at the frame's end (record 252: re, im, the
  frame's start).
- **Pilot MER** (`t2eq.py`): (8 I - s ref8)^2 + (8 Q)^2 of every scattered
  pilot after the equalizer (ref8 = 8 x 20 x boost: 213 for PP1/PP2),
  summed over the frame's symbols, reported after the last (record 251:
  sum >> 4, count, the frame's start). The A9's MER is the same definition
  as before: -10 log10(sum 16 / (n 25600)); the 7-bit cells cap it near 35
  dB.
- **Less in the ring**: no raw samples while searching
  (`acq_raw_off`), none around the guard intervals (`gi_raw_off`; only
  the 2176-sample window around each scheduled P1 is left), and with the
  router running only one equalized data symbol a frame for the browser's
  constellation (`eq_ring_j`; the router still takes every one).

A record is a carrier-stream header with symbol number 251 to 253 and six
words of 16 bits; its words go out together, possibly inside a symbol's
carriers. Control: T2 register 14 (0x78) `t2_ext`: p1_en 0, p1_k 8:1,
acq_raw_off 9, gi_raw_off 10, mer_en 11, eq_ring_j 19:12, ref8 29:20;
register 15 (0x7C) `t2_features` (read only): bits 2:0 = P1/GI, MER,
eq_ring; bit 8 P1 overflow. An older bitstream reads 0 there.

trxd (`dvbs2/fpga.rs` start_t2): with all three feature bits and the
equalizer it writes t2_ext and reads it back; `TRXD_T2_HW=0` keeps the
raw-sample path (A/B). `stream.rs` then:
- acquires from a P1 record whose (|C| + |B|) / E is 0.15 or more: the
  schedule starts two frames on; the first scheduled P1 window gives the
  coarse frequency (`find_p1_in`, as before), the first GI record the fine
  one outright; that first frame is not decoded (it came in partly at the
  search frequency);
- ends a frame on its MER record (its P2 symbols all in): `finish()` (L1,
  the router's buffer), the MER from the record, the frequency from the
  frame's GI record (before or after the MER record), then the next P1
  window as before.

Models: `t2p1.Model` (Python) and `fe::model::P1Model` (Rust) bit-exact
(`p1_model_matches_the_python_one`: the same input, the same 28 words);
`T2P1` against its model with the front end's pacing, bursts and a
consumer that stalls (`test_t2p1.py`); t2eq's MER against `mer_terms`
(`test_t2eq.py`); the front end's records and raw gating against the model
(`test_t2ofdm.py TestT2OfdmReports`); the ring filter alone
(`test_t2ringfilter.py`). Receiver: `t2_through_the_front_end_with_reports`
(the front-end model with reports, 3 of 3 frames after acquisition, no
LDPC failure, MER 27.1 dB against 27.4 with raw samples, 10 885 raw words
for 3.57 M samples; `t2_through_the_front_end` unchanged) and
`t2_reports_reacquire` (ignored, about 25 s: 50 000 samples cut out
mid-stream, the lock lost after six missed P1s, found again from the P1
records, 575 packets before the cut and 960 after).

On the boards (2026-10-03) one-way T2 took 4-6 % of a core; in full
duplex Libre 1 (MER 7.8 dB) took 52-76 %. The cause was not the frame moves
(P1 off by 3-4 samples about once a second, cheap) but L1: at that MER
L1-post's hard decisions rarely check, and every frame ran the float 16K
LDPC on the A9 (once or twice, each longer than a frame there). Now L1 is
verified with the LDPC at most once every 32 frames (`L1_LDPC_EVERY`, about
8 s); between, a frame whose hard decisions do not check is taken as
`l1_unchecked` (log field `l1_ok_unchecked_failed`); a mismatch on the hard
decisions still refuses it. `t2_reports_low_mer_cost` (ignored; T2NOISE,
T2PPM): MER 7.9 dB, 3 ppm, the frame moved 3 times in 8 frames; steady
state on the PC 1.06 ms a frame (1.07 at 20.8 dB, 1.36 at 5.4 dB; before:
about 3.5 and 9 ms; the test has no router, so the A9 also handles all the
data symbols there).

Bitstream `t2` (Libre, Vivado 2023.1): WNS +0.176 ns, WHS +0.040 ns; LUTs
70 % (64.8 % without the reports), BRAM 117.5 tiles (113: the P1 delay
line), DSPs 208 of 220 (196).

Expected on the board (to be measured): no P1 correlation on the A9 at
all while searching or locked (it was 245 ms a frame locked and the whole
core searching), raw-word handling down from every guard interval to one
2176-sample window a frame, equalized-symbol handling from 1 in 4 to 1
symbol a frame. What is left per frame: the P2 symbols (channel estimate,
equalizer table, L1), one P1 window, the records.

## Not done

- 64QAM/256QAM, other FFT sizes, PAPR reduction.
- A check with an independent T2 receiver (TV HAT / Ryde).

The `TRXD_*` A/B environment switches mentioned above were removed on 2026-10-03: on a board the FPGA paths are the only ones, and the software equivalents live on as the models the tests run. The mentions are history.
