# DATV transmitter quality (analyser measurements)

Libre 1 (192.168.12.178, firmware v0.3.21-30-g4f77) transmitting on
1255 MHz, TX attenuation 0 dB, the FPGA encoder and pulse shaper (DVB-S2,
long frames, pilots) or the DVB-T2 modulator. A Siglent SVA1032X with a
small antenna picked the signal up from about 1 m away (input attenuator
0 dB). 2026-09-27; captured with `SCDP` over SCPI while
`datv-ref/harness/run_tx.sh` kept the transmitter on.

## DVB-S2: digital demodulation (EVM / MER)

Analyser: MA mode, PSK demodulation (QPSK or 8PSK), root raised cosine
alpha 0.35, symbol rate as sent. Figures are the median of five
`:READ:DDEMod?` readings. The frequency error (about -560 Hz) is the
analyser's own clock (-0.41 ppm), not the Libre's.

| Mode | Rate | EVM rms | MER | Phase error | IQ offset | Screenshot |
|---|---|---|---|---|---|---|
| QPSK 1/2 | 33 kS/s | 1.71 % | 35.4 dB | 0.72 deg | -54.5 dB | [png](images/siglent/dvbs2-qpsk-12-33k.png) |
| QPSK 1/2 | 66 kS/s | 0.63 % | 44.0 dB | 0.28 deg | -59.3 dB | [png](images/siglent/dvbs2-qpsk-12-66k.png) |
| QPSK 1/2 | 125 kS/s | 0.71 % | 42.9 dB | 0.35 deg | -59.8 dB | [png](images/siglent/dvbs2-qpsk-12-125k.png) |
| QPSK 1/2 | 250 kS/s | 0.86 % | 41.3 dB | 0.42 deg | -59.2 dB | [png](images/siglent/dvbs2-qpsk-12-250k.png) |
| QPSK 3/4 | 250 kS/s | 0.95 % | 40.5 dB | 0.47 deg | -55.8 dB | [png](images/siglent/dvbs2-qpsk-34-250k.png) |
| 8PSK 3/4 | 250 kS/s | 1.19 % | 38.5 dB | 0.61 deg | -63.0 dB | [png](images/siglent/dvbs2-8psk-34-250k.png) |
| 8PSK 3/4 | 333 kS/s | 1.24 % | 38.1 dB | 0.64 deg | -56.5 dB | [png](images/siglent/dvbs2-8psk-34-333k.png) |
| QPSK 1/2 | 500 kS/s | 1.34 % | 37.5 dB | 0.63 deg | -60.1 dB | [png](images/siglent/dvbs2-qpsk-12-500k.png) |

- All modes and rates are near 40 dB MER, far above what any DVB-S2 mode
  needs (8PSK 3/4 decodes at about 8 dB). The receive path (small antenna,
  analyser noise floor) limits some of these figures, so the transmitter is
  at least this good.
- 33 kS/s reads a little worse: the analyser's capture holds fewer symbols
  at the lowest rate, so the Libre's residual phase noise shows more.
- The analyser shows a gain imbalance of about 2.6 dB for 8PSK and about
  0 dB for QPSK from the same transmitter. It fits its I/Q gain model to
  the 8PSK points, which does not work well, and the constellation is round.
  Read the QPSK figure (-0.005 dB).
- The screenshots show the constellation, the spectrum (flat top, steep
  RRC skirts), the error table with the decided symbols, and the I eye
  diagram (open, single crossing points).

## DVB-T2: spectrum only

The SVA1032X demodulates single carriers only (ASK/FSK/MSK/PSK/QAM). It
has no OFDM, DVB-T or DVB-T2 personality, so it shows the T2 signal as a
spectrum only: SA mode, span 4 MHz, RBW 30 kHz, VBW 300 Hz, positive peak.
The decode proof is the offline receiver (docs/DVBT2.md, Checked).

| Mode | Screenshot |
|---|---|
| T2 1.7 MHz QPSK 1/2 | [png](images/siglent/dvbt2-qpsk-12-1m7.png) |
| T2 1.7 MHz 16QAM 1/2 | [png](images/siglent/dvbt2-16qam-12-1m7.png) |

- Flat 1.54 MHz block (2K, 1.7 MHz channel), within about 1 dB across the
  band. The shoulders are 34-35 dB down at the band edges and fall to the
  analyser's floor within about 100 kHz. Two small spurs sit about 0.6 MHz
  outside the edges, about 35 dB below the channel at this RBW.
- The 16QAM capture has one narrow notch near the lower edge that the QPSK
  capture does not have. The sweep takes 1.25 s, so the notch is a short
  gap in the signal during that sweep (one capture only, cause not checked),
  not a feature of the spectrum.
