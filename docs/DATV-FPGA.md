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

The DDC decimates by whole numbers, so the symbol rate must give an even
whole number of ADC samples per symbol, at least 8: at 3.072 MS/s that is
32, 48, 64, 96, 128, 192, 256 or 384 kS/s. 250 kS/s is not possible; 256 kS/s
takes its place (both boards are ours, so nothing standard is lost).
Occupied bandwidth at 256 kS/s, roll-off 0.35: 346 kHz.

| Symbol rate | FIR1 | FIR2 | FIR3 (RRC) | Output |
|---|---|---|---|---|
| 256 kS/s | /3 | bypass | /2 | 512 kS/s |
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
- Not checked on air yet: QPSK 3/4 and 8PSK 3/4 (same encoder path), and
  exactly 250 kS/s (the DDC needs whole ratios; the interpolator is
  verified in simulation).
- Libre 2 to Libre 1 at 256 kS/s arrives at about 2 dB MER: 6 dB below the
  64 kS/s link, around the QPSK 1/2 threshold.

Receive of long frames needs the FPGA LDPC decoder (next).
