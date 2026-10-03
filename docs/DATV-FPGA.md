# DATV in the FPGA: receive front end (step 1 of 2)

Goal: DVB-S2 above the 384 kS/s stream's limits, e.g. 256 kS/s QPSK 3/4
(about 355 kbit/s of TS), without running out of A9. See `DATV.md` for the
software chain this builds on.

Plan, in order:
1. **DDC front end in the FPGA** (this note): Maia's DDC channel-filters and
   matched-filters the ADC samples down to 2 samples per symbol, and a DMA
   ring hands them to trxd. trxd keeps timing, frame sync, carrier and LDPC.
2. **LDPC decoder in the FPGA**, if the A9 cannot keep up near threshold
   (see the numbers below), or a NEON fixed-point decoder first.

Transmit at 256 kS/s is a separate question (see the end).

## Chain

```text
AD936x 12-bit IQ at 3.072 MS/s (before the x8 FIR decimator)
  -> Maia DDC (maia-hdl ddc.py, unchanged): mixer (28-bit NCO) -> FIR1 low-pass /d1
       -> [FIR2 low-pass /d2] -> FIR3 RRC /2 -> 16-bit IQ at 2 samples/symbol
  -> Recorder16IQ in ring mode (new) -> DDR ring -> trxd reads it (/dev/mem, like
       the spectrometer ring) behind committed_address
  -> dvbs2::rx::Receiver::new_prefiltered (AFC mixer, Gardner, frames, LDPC)
```

The stream trxd already uses (384 kS/s, x8 FIR) is untouched: voice,
scopes and TCI keep working, and DATV no longer depends on its bandwidth.

## Symbol rates

The DDC decimates by whole numbers, D = 2 x floor(3.072 MS/s / (4 rs)),
at least 4: the output then carries 2 to 2.7 samples per symbol, fractional
when 3.072 MS/s / rs is not a multiple of 4 (the RRC in FIR3 is designed at
that fractional rate; the receiver's Gardner loop interpolates anyway). Every
rate up to 384 kS/s works this way, 250 and 333 kS/s included; 500 kS/s
would need FIR1 at /1 and is transmit only. Occupied bandwidth at 256 kS/s,
roll-off 0.35: 346 kHz.

| Symbol rate | FIR1 | FIR2 | FIR3 (RRC) | Output |
|---|---|---|---|---|
| 333 kS/s | /2 | bypass | /2 | 768 kS/s (2.30 per symbol) |
| 256 kS/s | /3 | bypass | /2 | 512 kS/s |
| 250 kS/s | /3 | bypass | /2 | 512 kS/s (2.048 per symbol) |
| 128 kS/s | /6 | bypass | /2 | 256 kS/s |
| 64 kS/s | /4 | /3 | /2 | 128 kS/s |

The FIR3 RRC spans 24 symbols at 4 samples per symbol (97 taps).

## What is done and verified (2026-09-26, on power, no Vivado)

trxd (`src/trxd/src/dvbs2/ddc.rs`):
- Filter design for a symbol rate: Kaiser low-pass stages plus the RRC,
  quantized and scaled exactly as maia-httpd does (`ddc.rs`).
- The coefficient RAM image and register fields, with maia-httpd's
  `impl_set_ddc_fir` addressing and FPGA limits: operations, RAM size, clock
  budget.
- A bit-exact model of the DDC: maia-hdl's `Mixer.model` plus the FIR4DSP
  (two rounded accumulators) and FIR2DSP models, with the same decimation
  phase.
- `Receiver::new_prefiltered`: the receiver on the DDC output, at any rate
  near 2 samples/symbol.
- Tests: 256 kS/s 3/4, and 64 and 128 kS/s 1/2.
  - Each test modulates at 3.072 MS/s, adds noise and a 300 Hz transmitter
    error, and quantizes to a weak signal (about -30 dBFS) in the 12-bit
    ADC.
  - The samples then go through the DDC model into the receiver, and the
    result is compared with the all-software front end on the same samples.
  - Es/N0 after the DDC agrees within 0.1-0.3 dB, so the fixed point adds
    no loss, and every frame after acquisition decodes.

maia-hdl (fork, branch `datv-ddc` in `/data/claude/maia-sdr` on power):
- `DmaStreamWrite(ring=True)`: wraps at the end address instead of stopping.
  New `committed_address` output: the address after the last burst whose
  write response came back, so everything before it is in DDR.
- `Recorder16IQ(ring=...)` passes the option through and exposes
  `committed_address`.
- `test/test_dma_ring.py`: a stream into a 5-burst ring for 3.5 laps against
  an AXI memory model with random `wready` and delayed B responses. It checks
  that bursts go round in order, that the newest lap is in memory, and that
  `committed_address` steps one burst at a time and never runs ahead of the
  data.
- `test/test_datv_ddc.py` (vectors from trxd:
  `DDC_VECTORS=... cargo test dump_vectors -- --ignored`), three bit-exact
  checks:
  - trxd's mixer equals `Mixer.model`.
  - trxd's FIR cascade equals `FIR4DSP.model` / `FIR2DSP.model`.
  - `FIRDecimator3Stage`, simulated with the exact coefficient RAM image and
    registers trxd writes, equals those models.
- Two things the HDL simulation showed:
  - The stages do not backpressure each other. Input faster than the clock
    budget (61 cycles per sample at 3.072 MS/s and 187.5 MHz) drops
    samples between stages, so the budget check in `ddc.rs` and
    maia-httpd is a real limit.
  - After reset the HDL decimates at its own phase, and its sample buffers
    are not cleared, so the first outputs (one filter length) are
    transient. Neither matters to the receiver.

### CPU on the A9

Measured on Libre 2 with trxd running alongside, per second of signal, as
a percentage of one core:

| | Demod, with the DDC | Demod, software filtering at 3.072 MS/s | LDPC |
|---|---|---|---|
| 64 kS/s, 1/2, 5 dB | 10 % | 94 % | 39 % |
| 128 kS/s, 1/2, 5 dB | 20 % | 102 % | 81 % |
| 256 kS/s, 1/2, 5 dB | 39 % | 119 % | 166 % |
| 256 kS/s, 3/4, 9 dB | 39 % | 119 % | 82 % |

With the DDC, 256 kS/s at 3/4 fits the two cores: demodulation on one,
LDPC on the other (they already run on separate threads). Near threshold,
LDPC needs more iterations and will exceed a core; that is step 2's reason.
Rate 1/2 at 256 kS/s needs step 2 regardless.

## Integration (done 2026-09-26, on power)

The `simple` bitstream already carried the whole Maia core
(`maia_sdr_maia_iio_lite`), DDC and recorder included: only the
spectrometer was used, and the recorder was never started. That was just
as well: its range, 0x06000000-0x0E000000, is not reserved in the device
tree.

Changes (maia-sdr checkout `/data/claude/maia-sdr-simple`, branch `datv-ddc`):
- `configs.py`: `maia_iio_lite_datv`:
  - recorder in ring mode over 0x16100000-0x16200000 (1 MiB, 0.5 s at
    512 kS/s), fed from the DDC output directly, so the wide scope keeps the
    full ADC stream;
  - `platform` = 0xD5 in the version register, so software can tell this
    core from others.
- `maia_sdr.py`: the recorder block gains `recorder_committed_address` at
  0x18; nothing else moves.
- `projects/simple/maia_scope.tcl`: instantiates `maia_sdr_maia_iio_lite_datv`.
- Device trees (both boards): `maia_sdr_datv_ring@16100000`, reserved, `no-map`.

Build on power: `/data/claude/fwbuild/fpga-build.sh libre|plutoskyr2`
(Vivado 2023.1 in `/data/claude/Xilinx`, Docker image `vivado:2`, log in
`/data/claude/fwbuild/fpga-<board>.log`).

trxd (`src/dvbs2/fpga.rs`):
- The DDC and ring are used only when the core says 0xD5 and the device
  tree reserves the ring.
- It writes the design's coefficient RAM and fields, sets the NCO to the
  channel's offset from the LO, and starts the recorder in 16-bit mode.
- It then follows `committed_address` every 10 ms into
  `Receiver::new_prefiltered`.
- Otherwise it uses the 384 kS/s path as before.

Symbol rates:
- 192 kS/s, both directions: TX from the 384 kS/s stream at 2 samples per
  symbol, RX through the DDC.
- 256 kS/s, receive only, needs the DDC.

## Transmit in the FPGA (done 2026-09-26)

Long frames with pilots, QPSK 1/2 (MODCOD 4), QPSK 3/4 (7), 8PSK 3/4 (14),
any symbol rate from 8 kS/s to 1.2 MS/s (UI: 250, 256, 333, 500 kS/s):

```text
DAC DMA -> datv_split --(DAC GPIO bit 1)--> async FIFO -> ORI dvb_fpga encoder
  (F5OEO's in-band config: 0xB8, config byte = 0x20 | MODCOD, BBFRAME)
  -> datv_tx (maia_hdl/arb_interp.py: 16-symbol RRC, 256 phases, any rate)
  -> async FIFO -> datv_merge (in place of the x8 interpolator) -> XO NCO -> DAC
```

- trxd (`src/dvbs2/fpga_tx.rs`) builds the BBFRAMEs and writes them into
  the IIO TX buffer as raw bytes (`TxBlock::Raw`). The encoder's pace sets
  the rate, and the LO sits on the signal.
- Registers: datv_tx at 0x43C20000 (step, RRC table, id "DTX1"); encoder at
  0x43C30000.
- The encoder packs Q in bits 31:16 and I in 15:0. Reading it the other way
  round inverted every second header symbol and garbled all the data.
- Interpolator: bit-exact against its model (test_arb_interp.py). At
  250 kS/s from 3.072 MS/s: MER 38.6 dB, out-of-band -48 dB.
- Over the air, verified with an independent checker
  (`/data/claude/datv-ref/harness/verify_s2_long.py`, first validated on
  leandvbtx: 0 parity errors):
  - Libre 1 sending and recording itself through the DDC, 256 kS/s, QPSK
    1/2 long: 32/32 frames with every LDPC parity bit and the BBHEADER
    CRC right, MER 36-37 dB.
  - The TS carries service SQ6EMM / SQTRX with H.264 and Opus.
- On air 2026-09-30 (Libre 1 -> Libre 2, 2330 MHz, 250 kS/s): QPSK 3/4
  563/597 frames (one 10 s dropout on the path, MER 23 dB otherwise),
  8PSK 3/4 970/971.
- Libre 2 to Libre 1 at 256 kS/s arrives at about 2 dB MER: 6 dB below the
  64 kS/s link, around the QPSK 1/2 threshold.

## Long-frame receive (2026-09-26)

QPSK 1/2, QPSK 3/4, 8PSK 3/4, normal frames, pilots, through the DDC at any
rate it takes (250 kS/s included):

```text
DDC -> Receiver (FrameSpec::long: Gardner, header sync, pilot-aided carrier,
  QPSK or max-log 8PSK LLRs, bit deinterleaving)
  -> LDPC in the FPGA (0x43C40000, maia-hdl ldpc_axi.py; its bit-exact model
     ldpc_fpga.rs off the board) -> BCH (bch.rs, t = 12) -> BBFRAME -> TS
```

- The decoder's early stop (every check satisfied in one pass) can stop on
  a word with a few bits wrong, since checks are evaluated while later
  layers still change their variables. BCH (software, t = 12, remainder
  first, Berlekamp-Massey and Chien only when it is not zero) removes them
  and also rescues frames LDPC nearly decoded.
- Acquisition to tracking: the NCO's step at the end of acquisition also
  applies to the symbols already buffered (up to an input block beyond the
  next header); they are derotated to match, and the NCO phase is turned so
  the following samples continue them. Before that fix the first frame after
  acquisition always failed.
- Tests: long_frames_* (rx.rs, 64 kS/s: every frame, every packet in
  order at 3.0 / 6.5 / 10 dB), long_frames_at_250_ksps_through_the_ddc
  (ddc.rs: the DDC model at 2.048 samples per symbol; at 3 dB the first
  frames of the acquisition average may fail, since one header gives the
  frequency to about 100 Hz, more than half the pilots' alias spacing of
  169 Hz; every frame after that).
- A bitstream without the decoder: trxd probes its ID from a child process
  (the read is a bus error there) and falls back to the model in software.
- Unlocked, the header search runs at every symbol; it now screens each
  position on the SOF (30 symbols, prefix-summed normalization) before the
  full 90-symbol metric: 3x cheaper, about the cost of the locked receiver.
  Before, at 250 kS/s the A9 needed more than a core while searching, fell
  behind and never locked.
- The DDC ring holds 0.5 s. A thread of its own (datv-ring) drains it every
  5 ms into a queue of about 4 s for the demodulator; reading it between
  demodulator bursts let the DMA lap the reader unnoticed (bursts of 12 bad
  frames over the air).

Over the air (2026-09-26, Libre 2 -> Libre 1, 1255.000 MHz, 250 kS/s, Libre
2 at 0 dB attenuation, Es/N0 about 21 dB, carrier +137 Hz):

| Mode | Frames | Libre 1 CPU (one core = 100 %) |
|---|---|---|
| QPSK 1/2 long | 422/422 in 60 s, 532 pictures | demod 48 %, FEC 14 % |
| QPSK 3/4 long | 309/309 in 45 s | demod 49 %, FEC 13 % |
| 8PSK 3/4 long | 463/465 in 45 s (acquisition) | demod 60 %, FEC 21 % |

FEC there is the thread handing frames to the FPGA and waiting (LLR
quantization and the AXI copy in and out), BCH and the TS demux.


## Demodulator in the FPGA, step by step (2026-09-26 night)

Each block has a bit-exact Rust model in trxd, used by the receiver tests,
and an Amaranth implementation checked against vectors the model writes.

```text
DDC (2 to 2.7 samples/symbol)
  -> symsync.py (timing recovery; trxd src/dvbs2/symsync.rs)
  -> hdrdet.py  (SOF screening; trxd src/dvbs2/hdrdet.rs)
  -> recorder ring: one word a symbol, header candidates in bit 16
  -> Receiver::new_symbols_spec + process_flagged (AFC, AGC, frames,
     carrier, LLRs) -> FPGA LDPC -> BCH -> TS
```

1. **Timing recovery (symsync).** Gardner error, Catmull-Rom cubic
   interpolation, PI loop; integers only: positions Q.24, mu Q0.16, the
   error normalized by the AGC's power of two, gains as shifts (kp 7, ki
   13: about the software receiver's 0.01 / 1e-4). Sequential on one
   multiplier, about 45 clocks a symbol at 62.5 MHz. Registers: Maia 0x38
   datv_symsync (0 enable, 5:1 kp, 10:6 ki), 0x3C datv_omega (Q8.24). The
   model decodes the same frames as the float receiver (3 dB QPSK 1/2 and
   10 dB 8PSK 3/4 at 250 kS/s through the DDC model; both board
   recordings, 90/90 frames), and it cuts the locked demodulator's CPU by
   more than half (PC: 0.206 -> 0.087 s per 12 s of signal).
2. **Header screening (hdrdet).** The last 26 symbols against the SOF in
   two coherent chunks of 13, adds only (SOF symbols are (+-1 +-j)/sqrt2),
   magnitudes as max + 3/8 min, flag when 16 num >= 12 den. 0.3-0.7 % of
   symbols flagged on data and noise, every header of the recordings
   flagged. Enable: datv_symsync bit 11. The receiver's unlocked search
   evaluates its 90-symbol metric only where flagged.

trxd turns them on when the bitstream has them (the omega register and the
hdrdet bit read back) unless TRXD_NO_SYMSYNC=1 / TRXD_NO_HDRDET=1, and
always writes datv_symsync, so an older trxd on a newer bitstream gets
samples as before.

On the boards (both flashed, firmware v0.3.21-25-g7d23 plus these; Libre 2
-> Libre 1 over the air, 250 kS/s QPSK 1/2 long, 60 s): 422/422 frames,
534 pictures, Libre 1 demodulator 18 % of a core and FEC 12 % (51 % and 13 %
with TRXD_NO_SYMSYNC=1 on the same link, same MER of about 15 dB).

## Standard symbol rates and automatic receive (2026-09-27)

The DATV panel offers the amateur standard rates only, 33, 66, 125, 250,
333 and 500 kS/s (BATC / QO-100 practice; 256k and the other whole
fractions of 3.072 MS/s are gone), and the long-frame modes QPSK 1/2,
QPSK 3/4 and 8PSK 3/4, always with pilots (no tick: the FPGA sends them,
the receiver needs them). Both directions go through the FPGA at every
rate: 500 kS/s receive uses FIR1 at /2 as the matched filter (3.07 samples
per symbol) with FIR2 and FIR3 bypassed.

"Auto (receive)" as the symbol rate: a blind scan like a tuner's
(`dvbs2/scan.rs`). The FPGA front end is set to each standard rate in turn
(the last one found first; about two long frames at each, 0.4 s at least,
2.2 s at 33 kS/s); every header the FPGA flags has its PLS decoded
(`dvbs2/pls.rs`: all 112 MODCOD/type headers matched coherently over 90
symbols at every carrier offset through a 256-point FFT; 0 dB Es/N0 with
2 % of the symbol rate offset still decodes, noise scores under 0.45, a
header over 0.6). Two matching decodes (or one over 0.8) start the
receiver for that rate and mode; 5 s (or 2.5 long frames) without a good
frame and it scans again. Other MODCODs are named in the status ("not
receivable here").

Over the air (Libre 2 -> Libre 1, Libre 1 on Auto): 33, 66, 125, 250, 333
and 500 kS/s, QPSK 1/2, QPSK 3/4 and 8PSK 3/4 all found and locked, about
10-20 s after the transmitter started (a scan cycle is about 5 s); a
change from 250 kS/s QPSK 1/2 to 125 kS/s QPSK 3/4 followed by itself.

## Four-lane LDPC decoder (2026-09-28)

`maia-hdl/maia_hdl/ldpc_dec4.py` (id "LDP4", `ldpc_axi.py --lanes 4`):
the serial decoder's schedule and arithmetic, four checks of a group at a
time. Checks of a group share no variable (apart from pairs k, k + d:
d >= 11, except one group of rate 3/4 with d = 2, which runs two at a time;
batches that far apart wait for each other), so the result is the same,
bit for bit (the model's vectors, `test_ldpc_dec4.py`, all three cases).
Four consecutive checks read four consecutive positions of a table row
(info) or four consecutive columns of the parity seen as q rows of 360:
one byte from each of four banks, rotated. The CPU's window is unchanged
for the info bits; the parity LLRs go in the banks' order (trxd
`fpga_ldpc::parity_layout`, chosen by the id). Rate 1/2: about 66 000
cycles an iteration (0.66 ms at 100 MHz) against 250 000; the same block
RAM (4 posterior banks 16200 x 8, 4 state banks 8100 x 30). The first
build missed 100 MHz by 0.1 ns on four decoder paths (ROM -> address ->
RAM, pass 1 -> result FIFO, pass 2 -> unsat, pass 2 -> rotated write):
each has a register now.

## DVB-S2 soft bits in the FPGA (stage 1 of 3, 2026-10-02)

The A9 no longer makes, quantizes or lays out the 64 800 LLRs of a long
frame. It hands the LDPC engine (`ldpc_dma.py`, "LDP6") the frame's data
symbols as cells instead: derotated (phase still tracked on the A9),
descrambled, times `s2cells::gain` (the constellation on radius 64), as
8-bit I/Q, two a word, plus one per-frame noise scale `kq` (0xFF24). The
engine makes the LLRs and writes them where the decoder wants them:

- QPSK (1/2, 3/4): the DVB-T2 QPSK cells path, unrotated (0xFF20 = 1):
  bit 2 j from I, 2 j + 1 from Q.
- 8PSK 3/4 (0xFF20 bit 4, features 0xFF30 bit 3): three LLRs a cell,
  max-log over the eight points (correlations in Q14, 1/sqrt 2 as 11585;
  labels via PSK8_PHASE), and the 3-column bit interleaver undone (bit m of
  cell j is variable m 21600 + j; column 2 crosses K, the parity part as
  p = c q + r, no parity interleaving in DVB-S2). One extra cycle a cell
  (C_PREP) for the max trees.

Bit for bit with trxd `dvbs2/s2cells.rs` (`test_ldpc_dma.py
test_cells_psk8`, full-scale and -128 cells included). An older bitstream
(no feature bit 3) gets its 8PSK LLRs from the model on the A9, QPSK
through the old cells path; `TRXD_S2_FLOAT_LLR=1` brings back the float
LLRs of before. Decoding margin unchanged (`rx.rs cells_margin`: the same
noisy frames at 0.4 dB steps around each threshold decode alike, within a
frame of 39). Per frame the A9 now packs 10 800 (8PSK) or 16 200 (QPSK)
cell words and copies them to DDR, instead of 64 800 float-to-6-bit
conversions and the parity permutation (4.6 ms a frame on the A9 before).
The demodulator also steps a rotator between known blocks instead of a
sine and cosine a symbol, and computes the 8PSK decision (only for the
MER shown) on every fourth symbol.

## DVB-S2 frames straight from the ring (stage 2 of 3, 2026-10-02)

The CPU no longer touches the data symbols of a long frame. The LDPC
engine's DMA reads them from the receive ring where the recorder left them,
and a new block in front of the demapper (maia-hdl `s2front.py`, model
`src/trxd/src/dvbs2/s2ring.rs`, bit-exact) turns, descrambles and scales
them into the stage-1 cells.

Per frame the receiver (`rx.rs`, ring mode) still:
- finds and tracks the headers (the FPGA's flags, then its header
  correlation at those few positions; other PLS codes stepped over);
- fits the carrier from the known blocks (header, pilots, next header),
  as before;
- measures the amplitude and noise on the known symbols, and the MER and
  the constellation on every 32nd data symbol;
- hands the engine a job: where the frame's first data symbol is (an
  absolute ring position, from the recorder's wrap counter), and per data
  group (1440 symbols between pilot blocks) the angle of its first symbol
  and the step per symbol (the AFC's mixer less the carrier fit: both are
  linear inside a group), the gain and kq.

The ring reader takes a block's absolute position and the address it
copies up to from one consistent read of the recorder (wrap counter,
committed address, wrap counter). The first stage-2 image read the
committed address apart from the wrap counter: the DMA moved between the
two, every block came out a few words off, the receiver saw a gap each
time, lost lock and about half the frames, and the TS broke (over the air
2026-10-02: 430 of 1012 frames at 8PSK 500k). `ring_stream_at_real_rates`
(recorder model with time passing between register reads, reads every 5 ms
with late ones, decoder queue and latency) reproduced it (28 of 60 frames)
and passes since (59 of 60, every packet in order).

Symbols are made from the ring's raw words only where the receiver looks
(headers, pilots, the sampled data), with the AFC's mixer as a function of
the absolute symbol position (a retune changes it from the next header on).

Fixed point (s2front.py): the angle A + t B (32 bits a turn) rounded to 16
bits, plus (4 - R_j) quarter turns of descrambling (R the PL scrambling
sequence, generated in the block), a +-90 degree pre-rotation and a 12-step
CORDIC (no multipliers), then x G >> 20 saturated to i8 (two DSPs; G has
the CORDIC gain in it). The 32-entry segment table is LUT RAM.

Registers (ldpc_axi, besides stage 1): 0xFF20 bit 5 ring mode; 0xFF34 ring
start, 0xFF38 ring end; 0xFF3C gain (16:0), lead (28:24, words before the
frame's first symbol in its 128-byte aligned first beat), bit 31 pilots;
0xFF40 / 0xFF44 segment angle / step, 0xFF48 write them at entry (4:0).
0xFF30 bit 4 says the core has it. The DMA's read address wraps at the ring
end; 0xFF18 counts the words with the lead (16 bits now).

A frame is decoded only if it is still wholly in the ring, with 32768 words
(65 ms at 500 kS/s) of margin, before and after the engine read it; one the
recorder overwrote first (the decoder far behind) is counted in the
receiver stats as `ring_lapped` and lost. Without the wrap counter, the
feature bit, or with `TRXD_S2_NO_RING=1` (an A/B on the board) the stage-1
path runs.

PC model, 250 kS/s, the receiver's own time per frame (the ARM's share is
the same ratio): QPSK 1/2 0.28 ms (stage 1: 1.05 ms), 8PSK 3/4 0.22 ms
(0.87 ms); `cargo test --release ring_demod_time -- --ignored --nocapture`.
Decoding margin unchanged against the symbol path on the same noisy words
(`ring_matches_the_symbol_path`: equal frames at 8PSK 8.2 dB, QPSK 1/2 1.0
and 1.4 dB; one frame of 29 apart at 8PSK 7.7 dB and QPSK 3/4 4 dB, either
way within the noise of 29 frames).

Libre DATV bitstream: LUT 78.7 % (+2.1 points), registers 36.5 %, BRAM
92.5 % (unchanged), DSP 202 of 220 (+2); WNS +0.013 ns after post-route
phys_opt (-0.069 ns routed): met, but there is no slack left. Stage 3 (BCH,
descrambling in the fabric) will need room freed first.

## BCH and descrambling in the FPGA (stage 3 of 3, 2026-10-02)

The A9 no longer unpacks the decisions, divides them by the BCH generator
or descrambles the BBFRAME. With 0xFF20 bit 6 (features 0xFF30 bit 5) the
LDPC engine's store stage (`ldpc_dma.py`, model `src/trxd/src/dvbs2/bbout.rs`,
bit-exact) does it while the decisions go out, four a cycle:

- the BCH division: the first Nbch decisions (the info part, natural order
  in the decoder's RAM; 32400 / 48600) through a 192-bit LFSR, r = r x + b
  mod g(x), g the product of Table 6a's g1..g12 (the same code for DVB-S2
  and DVB-T2 normal frames at 1/2 and 3/4); the remainder in 0xFF4C (bits
  31:0) .. 0xFF60 (191:160), status bit 3 when it is zero;
- the BB descrambler (1 + x^14 + x^15) over the first Kbch = Nbch - 192;
- the packing MSB first a byte at a time (variable 32 k + 8 c + r at bit
  8 c + 7 - r of word k): the buffer is the BBFRAME's bytes in order.

The receiver (`rx.rs` `Fec`, normal frames) takes the Kbch / 8 bytes as the
BBFRAME; only a non-zero remainder (rare after LDPC) costs anything: the
syndromes from the 192-bit remainder, Berlekamp-Massey and the Chien search
on the A9 (`bch.rs correct_bytes`), the wrong bits flipped in the bytes.
A decode that runs in this mode and gives nothing (a time-out) is a lost
frame (its bits were not written). The T2 receiver goes the same way (same
code, same Kbch). Without the feature bit, on the model, or with
`TRXD_S2_SW_BCH=1` (an A/B on the board) the bits path as before.

One more register stage in the store pipeline (the four decisions of a RAM
word, then the LFSR, the scrambler and the packing): no BRAM, no DSP.

Tests: maia-hdl `test_ldpc_dma.py test_bb` (the three vectors of
ldpc_long.json decoded, words, remainder and zero flag against the model);
`vectors/bbout.json` ties the Python model to the Rust one
(`BBOUT_VECTORS=... cargo test --release bbout_vectors -- --ignored`);
`bbout.rs bytes_and_remainder_correct_as_bits_do` (a codeword's BBFRAME,
1..12 errors corrected in the bytes, 13 refused). The A9's share before:
`bits_out_ms` 0.4-0.64 and `bch_ms` 0.37-0.6 a block (board log); PC
`cargo test --release bb_arm_time -- --ignored --nocapture`: 0.036 ms
against nothing measurable.

## What the A9 still does for DVB-S2 (measured 2026-10-03)

The ring-mode receiver's own time per long frame on the PC
(`ring_demod_time`, 250 kS/s, timers around each step): `frame()` 78-93 us
(70 %: the known blocks' phases, the carrier fit, amplitude and noise, the
MER and constellation from every 32nd data symbol, the engine's job),
`other_at` 24 us (20 %: the other PLS codes' headers around the expected
one), the header scan 3 us, appending the ring words 4-5 us, the buffer
drain 0.2 us. On the board that is the 12 % of a core at 500 kS/s.

## Known blocks in the FPGA, PLS decoder, cheaper sampling (2026-10-03)

The tracking receiver (ring mode, after acquisition) no longer makes the
known symbols one by one:

- **Known-symbol accumulator** (maia-hdl `s2trk.py`, s2 mode bitstream):
  on the words the recorder writes into the ring (so its index is the
  ring's absolute position), it follows the frames from a loaded header
  position (whole frames on; a frame of another length or a slip: trxd
  loads again), mixes each header and pilot symbol with its own AFC phase
  accumulator (the step trxd sends, taken at each frame start; the
  1024-entry Q15 table), turns it by the reference's quarter turns (every
  header and pilot reference is e^(j pi/4) j^q; trxd applies the e^(j
  pi/4)) and sums them; per block a FIFO entry: first symbol, sum re/im,
  power, its phase at the first symbol and its step. trxd corrects each
  entry to its own mixer at the block's centre (`s2trk::Entry::corr`) and
  takes it when the two steps differ by under 0.05 rad over the block
  (the AFC retunes a little every frame; the unit takes a step a frame
  later). Three pipeline stages, a word a cycle at most. Registers in the
  window T2 has in the other bitstreams (0x40-0x7F): control (enable,
  load, pilots, pilot blocks), base, length, step, header table, the entry
  (6 words), status (level, overflow, synced), pop, counter, features 0x7C
  bit 16 (T2's features are bits 7:0, so a t2 or datv bitstream reads 0
  there).
- **Without the unit** (another bitstream, a frame it did not follow,
  `TRXD_S2_TRK=0`) the receiver makes the same entry from the ring's words
  (`s2trk::block`, bit for bit the unit's: `block_matches_the_python_one`;
  the unit against its model: `test_s2trk.py`).
- From the entries: the carrier fit's phases, the amplitude (mean of
  Re(c e^(-j phase)) over the blocks) and the noise (mean power less the
  amplitude squared), the frequency left inside a block ignored (a few Hz
  when tracking; acquisition keeps the symbol-by-symbol path).
- **PLS decoder** (`s2trk::pls_decode`): the 64 PLS symbols' soft bits
  through a 32-point Hadamard transform (the (32, 6) Reed-Muller code, the
  repeated or inverted bit), phase from the SOF: `other_at` decodes the PLS
  at the best SOF position and scores that one header instead of
  correlating all 115 (`pls_decode_finds_every_header`).
- The MER and the constellation from every 128th data symbol (169 to 253
  a long frame), a rotator per data group stepped 128 symbols at a time,
  instead of every 32nd with a sine and cosine each.

`TRXD_S2_OLDFRAME=1` brings back the previous path (A/B). The receiver's
log line has `blocks_fpga_made` (blocks from the unit / made on the A9).

PC (`ring_demod_time`, 250 kS/s, no unit: the model makes the blocks;
the first frames' acquisition included): QPSK 1/2 0.130 -> 0.034 ms a
frame, 8PSK 3/4 0.112 -> 0.029 ms; `other_at` 24 -> 2.3 us. With the unit
the block sums go too (about 8 us of the rest; its FIFO costs the reader
thread 7 register accesses a block). `ring_with_the_fpga_accumulator`:
the unit's model fed the words as they come, the receiver's commands a
chunk late: every frame decoded (QPSK 1/2 and 8PSK 3/4, 8 dB, 900 Hz off),
all blocks from the unit after the first two tracked frames.

Bitstream `s2` with the unit (Libre, Vivado 2023.1): WNS +0.144 ns, WHS
+0.015 ns; LUTs 53.9 % (52.1 % before), BRAM 111.5 tiles (108.5: the
FIFO and the table), DSPs 121 (115); 830 KiB xz in the image (805).

The `TRXD_*` A/B environment switches mentioned above were removed on 2026-10-03: on a board the FPGA paths are the only ones, and the software equivalents live on as the models the tests run. The mentions are history.
