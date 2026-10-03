# Performance on the LibreSDR (2x Cortex-A9, xc7z020)

Measured on 2026-09-26 on the board itself, trxd `role = "trx"`, 1296 MHz,
USB, one web client connected (scope + audio), nothing else switched on.
"% of one core" comes from trxd's own engine profile (logged once a minute,
`engine profile (% of one core): ...`); "total" is `/proc/stat` busy time over
both cores.

## Where the CPU went, and what was done

| Stage (engine thread) | before | after | what changed |
|---|---|---|---|
| DDC 384 -> 48 kHz | 13-14 % | 11 % | - |
| receive DSP (SSB demod + AGC) | 57 % | 10 % | narrow modes (SSB/CW/data) demodulate at 12 kHz: the 331-tap complex passband FIR runs at a quarter of the rate |
| 48 -> 12 kHz audio decimation | 1 % | 0 % | the 12 kHz demod output *is* the 12 kHz audio |
| web scope | 9-10 % | 6 % | - |
| engine load (wall time) | 82-85 % | 28 % | |
| live CW decoder thread | 33 % | 0 % outside CW mode | the timing decoder refits a 6 s window 10 times a second; it now runs in CW mode only (as on the IC-705) |
| **total, both cores** | **~95 %** | **40 %** | |

Two more things that were cheap to fix and expensive to leave:

- The AD9361 `in_voltage0_hardwaregain` attribute costs **74 ms per read**
  (SPI round trip in the driver). It is read once a second on its own thread
  (`rx-gain`), never on the sample path.
- DeepCW streaming live (NEURAL in the CW box) needs more than both cores even
  with a 6 s window (`Worker::with_window`, added to sdroxide-deepcw): it
  falls behind and copies fragments. TIMING is the default; NEURAL stays
  available for weak signals when a late copy is acceptable.
- The CW skimmer (a second DeepCW) was removed.

## Remaining budget

About 60 % of the two cores is free with the web UI open. The biggest
remaining consumers are the DDC (11 %), the scope (6 %), the live CW decoder
in CW mode (~33 %), and Q65/PI4 when switched on (each a burst every
60 s).

## What could move into the FPGA

The Libre bitstream (`simple` project, Vivado 2023.1) uses 27 % of the LUTs,
26 % of the BRAM (37 of 140 tiles), 44 % of the DSP48 (96 of 220) and meets
timing with 0.63 ns to spare. It already does the heavy fixed-rate work:
the x8 FIR decimator/interpolator (3.072 MS/s <-> 384 kS/s) and Maia's wide
spectrometer (every span above 300 kHz).

| Candidate | CPU saved | FPGA cost | Verdict |
|---|---|---|---|
| Channel DDC (NCO + decimation 384 k -> 48 k) | ~11 % of one core | NCO + two CIC/FIR stages, a second DMA stream (the 384 k stream must stay for TCI IQ and the scope) | Worth it only if CPU becomes short again; about a week with simulation and two board builds. |
| Narrow scope (spans <= 300 kHz) | ~6 % | the ARM FFTs a decimated stream Maia does not see | Not worth it. |
| SSB demod filter | ~5 % now | a fixed FIR with loadable taps | Not worth it after the 12 kHz change. |
| Timing CW decoder | 33 % in CW mode | it is not DSP: a search over a speed/threshold grid | Better fixed in software (see below). |
| **DeepCW (neural)** | enables NEURAL live | see below | Feasible, large project. |

### DeepCW in the FPGA

The model is a Conformer-CTC with about 3.6 M parameters, reading a 65-bin
log spectrogram at 66.7 frames/s. One 6 s window is about 0.4-1.4 GMAC
(parameters x frames after subsampling); one decode a second is more than
the two A9 cores' realistic NEON throughput, which is what was measured.

The fabric left over has 124 DSP48s: about 18 GMAC/s at 150 MHz in int8.
Weights quantised to int8 are 3.6 MB, too much for BRAM (about 450 kB free)
but trivial to stream from DDR once per window through an HP port. So the
shape that fits is a **matrix-multiply engine** (systolic int8 array + weight
and activation DMA) with the ARM keeping the small non-matmul operations
(layer norm, softmax, activations, the depthwise convolutions).

What that takes, in order:

1. Quantise the ONNX model to int8 and check on PC recordings (the SR6LB
   beacon, the loopback test) that it still copies.
2. The GEMM engine in HDL with cocotb tests (maia-hdl already has the
   infrastructure), timed at 150 MHz in the `simple` project.
3. A UIO driver and an rten operator (or a small hand-written executor for
   this one graph) that sends the matmuls to the engine.
4. Measure: the goal is one decode a second of a 6 s window at < 20 % CPU.

That is weeks of work and several bitstream builds, not a night. It was not
started without agreeing on it first.

### Cheaper first

- **Timing decoder**: `CwDecoder` re-analyses its 6 s window every 100 ms
  (`HOP_S` in sdroxide-dsp `cw.rs`). Every 250 ms would cut it to ~40 % of
  the cost for a quarter second more latency; skipping analysis while the
  signal is a steady carrier or silence would cut most of the rest.
- **DeepCW in software**: decode every 2 s instead of 1 s, and only while
  there is keying at the pitch (the timing front end already knows).

## 2026-10-03: where the time goes now, what moved, what is left

Measured on Libre 1 (v0.3.21-56, the S2/T2 offload in): trxd's engine
profile and `top`.

| Situation | engine, % of one core | the rest |
|---|---|---|
| USB, no web client | 2 (rx_dsp 1.9) | trxd 4 % of a core in all |
| USB, one page open (scope + audio) | ~30: DDC 7, rx_dsp 12, scope 6-7, transmit 3, rest 1 | web + audio threads a few % |
| DATV mode, page open, S2 receiver running | 17: rx_dsp 6, scope 6.6, transmit 2.9 (DDC off in that mode) | demod 0-3 %, FEC 0-5 % (FPGA decoder; the thread's own CPU, `fec_cpu_s`) |
| DVB-T2 1.7 MHz receive | | demod 5 %, FEC 1.5 % CPU (the page showed 21 % wall time before: waiting for the FPGA) |

The DATV modems are off the A9 (docs/DATV-FPGA.md, docs/DVBT2.md): the
demodulators, LDPC, BCH and descrambling, the T2 cell router and reports
run in the fabric; the ARM keeps the carrier fit from per-block sums,
the TS demux and the page. The CW-RS network's front end and temporal
layers are in the trx bitstream (docs/RSCW.md). What the engine still
spends with a page open is the radio itself: the channel DDC, the SSB
chain at 12 kHz, the scope FFTs, and a transmit stage that kept making
zero blocks while idle.

Done on 2026-10-03:

- **Channel DDC in the FPGA** (`radio.fpga_ddc`, default on): the trx
  bitstream's Maia DDC (NCO + three FIR stages, 3.072 MS/s to 48 kHz,
  flat to 12 kHz, 60 dB stop; `ddc::design_channel`) feeds the 1 MiB ring
  in DDR that the DVB-S2 front end uses in the s2 bitstream; the engine
  reads the channel from the ring (`FrontEnd::start_channel`,
  `read_channel`; platform byte 0xD7 marks the core) instead of running
  sdroxide's DDC on the 384 kS/s stream. An inverting transverter gets the
  negated offset and a conjugated channel. The software DDC stays as the
  fallback (a bitstream without the ring, `fpga_ddc = false`, the PC).
  Measured on Libre 1, USB, one page open (span 12.5 kHz): engine 21 %
  of a core with the FPGA channel against 27-28 % with the ARM DDC (ddc
  7.1 % to 0.3 %, the ring read); the channel's level the same as the
  stream path's within a dB after the scaling (the S-meter calibration is
  in those units). The trx bitstream's resources are unchanged (the DDC
  was in Maia's core already): LUTs 34.6 %, 86 BRAM tiles, 101 DSPs,
  timing met with 0.33 ns.
- **Idle transmit**: a `TxBlock::Silence` marker; the transmit thread
  writes its own zero buffer (no block made, scaled and queued each cycle).
- **DeepCW model**: loaded only when the neural engine is the configured
  one or is selected on the page (it was 15 MB of RAM and 5 s of flash
  reading at every start, for an engine the A9 cannot run live).
- **A/B switches gone**: the `TRXD_*` environment knobs that kept the
  software twins of the FPGA stages selectable on a board (symsync,
  hdrdet, s2trk, float LLRs, software BCH, LDPC DMA and cells, the S2
  ring, the T2 front end, equalizer, reports, router and its check, the
  IFFT, the T2 test tone, the RSNN front, the RX/TX bandwidth) were
  removed with the code behind them where it was only theirs. The models
  stay as test code.
- **LDPC iterations with the FPGA decoder**: 50 up to 12 queued blocks
  (a T2 frame's 9 arrive at once), 30 to 24, 16 beyond; the software
  decoders keep the old 50/20/12/8 table. 276 of 730 T2 blocks had hit
  the 12 cap before.

Still on the ARM, and why:

| What | Cost | Verdict |
|---|---|---|
| SSB chain at 12 kHz (filter, AGC, demod, NB, NR, notch) | ~6-12 % with a page | Fixed-point filters in the fabric would save most of it; the chain is sdroxide's and changes with the mode, filter and DSP settings. Not now. |
| Scope at mid spans (20-300 kHz: a 4096-point FFT of the 384 kS/s stream) | ~6 % | Maia's spectrometer can take the DDC output (`use_ddc_out`), but the channel DDC is fixed at 48 kHz: only spans up to 48 kHz could move. A second, span-wide DDC or a decimating FFT path is a bigger change; the wide spans (> 300 kHz) are already Maia's. |
| Live CW timing decoder in CW mode | ~33 % | A grid search, not DSP; cheaper fixed in software (analyse every 250 ms, skip steady carrier / silence). |
| DeepCW neural CW | not live | An int8 GEMM engine in the fabric (see above): weeks. |
| Web server, WebSocket, TS demux, page audio | a few % | Not DSP. |

Candidates for removal that need a decision (they are features, not
dead code):

- The software DVB-S2 modem (short frames, the software modulator and
  receiver, software LDPC): no board uses it (BASIC+ sends and receives
  long frames through the FPGA, BASIC has no DATV); it serves `--sim` on a
  PC and the tests. About 3000 lines.
- The DeepCW neural engine (`cw_engine = "neural"`): too heavy to run live
  on the A9; its model is 15 MB of the flash (`model` partition).
- The PlutoSky R2 / ADALM-Pluto paths that have no FPGA mode bitstreams.
