# DVB-T2 transmit (amateur narrow channels)

trxd sends DVB-T2 (EN 302 755) the way amateur DATV uses it: the Portsdown 4
DVB-T2 option's profile, received by the Ryde, the Knucker and the Lynx
(Raspberry Pi TV HAT, Sony CXD2880) receivers and by T2 TV tuners at 1.7 MHz.

| | |
|---|---|
| Channel | 1.7 MHz (standard, 131/71 MS/s), or 2.0 / 1.35 MHz (sample rate 8/7 x bandwidth) |
| OFDM | 2K, normal carriers, guard 1/8, pilot pattern PP2, SISO |
| Frame | P1, 8 P2 symbols, 190 data symbols (about 248 ms at 1.7 MHz), 2 frames a super-frame |
| PLP | one, TS, normal FEC frames (64800), QPSK 1/2 or 3/4, no rotation, one TI block, 9 FEC blocks a frame |
| L1 | L1-pre BPSK, L1-post QPSK 1/2 (16K LDPC), version 1.1.1 |
| TS rate | 1.164 Mbit/s (1/2), 1.751 Mbit/s (3/4) at 1.7 MHz |

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
  every LDPC codeword and cell bit-exact, output within 8e-7, for QPSK 1/2
  and 3/4.
- Cells travel as one-byte codes through the interleavers and the frame
  builder (complex cells made the A9 wait on memory more than on the IFFTs).
- Generation and writing overlap (two frames queued between the threads):
  the A9 needs about 154 ms (modulator, IFFTs on both cores) + 33 ms (8-bit
  conversion) per 248 ms frame.
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
  - Loopback with 3 kHz offset and 15 dB SNR: every packet, MER 15 dB.
  - Through the FPGA path's integer model (8-bit, the low-pass resampler):
    MER 39 dB, every packet.
- Over the air (2026-09-27, Libre 1 -> Libre 2, 1255 MHz, T2 1.7 MHz QPSK
  1/2, browser camera): Libre 2 recorded 2 s raw at 3.072 MS/s (`trxd
  --capture-iq`, trxd stopped), decoded offline (`t2_capture`): 7 frames,
  1150 TS packets (video PID 256, audio 257, nulls), 54 of 63 FEC blocks, L1
  MER 4-5 dB: the indoor path was this weak (the analyser sees the channel
  at -60 dBm in 30 kHz; the S2 signal at 250 kS/s is about 10 dB stronger
  in total, OFDM's peaks need the headroom).
- Analyser: flat 1.54 MHz block, sharp edges.

## Not done

- A T2 receiver in trxd (the offline one is for this profile only and not
  real time; the 3.072 MS/s stream would need the FPGA's help).
- 16QAM/64QAM (bit interleaver and demux), rotated constellations on air
  (the code has rotation, unchecked OTA), other FFT sizes, PAPR reduction.
- A check with an independent T2 receiver (TV HAT / Ryde).
