# RADE V2 digital voice

[RADE](https://github.com/drowe67/radae) (Radio Autoencoder) is FreeDV's
neural voice mode: a neural encoder turns speech features into an OFDM
waveform, and at the receiver a neural decoder and the FARGAN vocoder turn
it back into speech. V2 uses 14 carriers and no pilots, occupies about
860 Hz (1060-1940 Hz in the SSB passband), and works down to about -4 dB
SNR in AWGN.

**Status upstream: V2 is pre-release.** The waveform, the weights and the
API may change without notice, and later versions will not be compatible
with this one. The FreeDV team does not recommend V2 for regular on-air use
yet. The C port used here is [freedv/rade_c](https://github.com/freedv/rade_c)
at `cc17222` (src/rade-web/build.sh).

## Where it runs: in the browser

The SQTRX page runs RADE itself, in a Web Worker with a WebAssembly build of
rade_c. The board does no RADE work at all. On the Libre's A9, RADE costs
about 140 % of one core to receive and 40 % to transmit (measured
2026-10-04). In a browser it costs a few per cent of one desktop core:

| | x86 native (AVX2) | WASM in Node (SIMD) |
|---|---|---|
| RX (decoder + FARGAN) | 12 % of a core | 4-7 % |
| TX (features + encoder) | 2 % | 2 % |

Board CPU with a page open (Libre 2): 21 % in USB-D, 22 % in RADE.

Signal path:

- **RX:** trxd's usual 12 kHz mu-law USB audio goes to the worker, which
  resamples it to 8 kHz and runs the V2 receiver (CP timing and frequency
  acquisition, ML frame sync, the neural decoder), then FARGAN at 16 kHz,
  and resamples the speech back to 12 kHz for the page's player. While the
  receiver searches, the page plays the band itself 12 dB down, so that
  tuning works by ear.
- **TX:** the microphone at 48 kHz (the DATV capture worklet), down to
  16 kHz, LPCNet features, the V2 encoder and OFDM, the real part of the
  8 kHz modem signal, up to 12 kHz, and out as the web microphone (mu-law).
  Modem level: RMS 0.24, peaks 0.5 of full scale.
- **End of over:** PTT up sends the last (zero-padded) frame and the EOO
  frame, then `{"cmd":"ptt","on":false,"drain":true}`. trxd keeps sending
  until the microphone queue is empty (at most 1.5 s, `MIC_DRAIN_MAX`), then
  unkeys. Without `drain` it would clear the queue and lose the EOO.

The RADE button sets USB-D (PKTUSB: no compressor or CESSB on TX), a
250-2750 Hz filter (wide enough for the frequency search below), slow AGC,
and `{"cmd":"rade","on":true}`. trxd keeps
the `rade` flag in the state, so every open page shows RADE and decodes;
any mode change clears it. The tag next to the TX/RX tag shows `RADE
search`, `RADE <snr> dB <offset> Hz` in sync, or `RADE TX`.

## Frequency offset: the coarse search

RADE V2 estimates the frequency offset from its cyclic prefix, which
resolves only +/-31 Hz (half the 62.5 Hz carrier spacing). Further off, it
locks whole carriers away, still reports sync, and decodes nonsense. Two
Libres on free-running VCTCXOs (no 10 MHz or PPS: gpsdo_boot.sh holds the
DAC) were 285 Hz apart at 1296 MHz, and at 144 MHz the offset moved from
+158 to -95 Hz in five hours (1.75 ppm of drift).

So the worker searches first. `rw_rx_set_shift()` in the wrapper mixes the
input by a complex exponential before the receiver. While the receiver
searches, `Coarse` (rade-worker.js) finds the 870 Hz block of 14 carriers
within +/-600 Hz in a spectrum averaged over about 1 s. It matches a
template on log power: the block against 120 Hz guard bands beside it,
refined with a parabola. It needs 1.5 dB of contrast to count, and the
shift only moves at 2.5 dB or more. In sync, the receiver is moved, and
restarted, when it is two or more carriers off (the spectral estimate, at
6 dB contrast or more), or one carrier off (`rw_rx_slip`: the power of the
carriers on exact DFT bins of the symbols before the receiver's band-pass,
twice running). The tag shows the total offset: the shift plus RADE's own
estimate.

## Over the air (2026-10-04, Libre 1 -> Libre 2)

The speech score is the mean squared cepstral distance between the decoded
speech and the input, both limited to 6 kHz (the page plays 12 kHz audio).
A clean loop scores about 1.6.

| Band, TX att | Receiver | Speech score |
|---|---|---|
| 1296.300, 30 dB | 13-17 dB SNR, +285 Hz | 2.18 |
| 1296.300, 40 dB | 7-9 dB | 2.65 (one run of three locked a carrier off: about 18) |
| 1296.300, 50 dB | about -4 dB, RADE's threshold | occasional sync only |
| 144.300, 20 dB | 6-11 dB, about +160 Hz | locks one or two carriers off: no |

Open: over the air the carriers leak into each other (block inside to
outside power about 9, against 800-3000 in the simulator and 20000 on the
stream the page sends). The cause is not the TX feed (timing checked), the
FPGA DDC (the software DDC is the same), the audio or RF AGC, the TX drive,
lost audio frames, or phase noise (a TUNE carrier is 47 dB clean). It is
what makes the one-carrier decision unreliable, and it fails at 144 MHz.
With both stations on a common reference (10 MHz or GPS), the search is
not needed.

Not done yet: the V2 auxiliary data channel (one BPSK bit per 40 ms frame).
The C port has no text protocol on it yet, so the page sends no callsign
and decodes none.

## The module on the board

`rade.wasm` is 4.1 MB, or 3.3 MB gzipped, with int8 weights only
(`-DDISABLE_DEBUG_FLOAT`). That is too big for the firmware slots (about
990 KB free), so it lives in the 5.5 MB `model` flash partition
(docs/FLASH.md), shared by both slots:

| Offset | Size | Contents |
|---|---|---|
| 0 | 4 | magic `RAD1` |
| 4 | 4 | length of the gzip stream, little endian |
| 8 | 32 | SHA-256 of the gzip stream |
| 40 | ... | `rade.wasm`, gzip |

- `package/trxd/rade.wasm.gz` is committed (built by src/rade-web/build.sh).
- `board/tezuka/common/pack-rade.sh` packs it into `flash/model.bin` in the
  build zip.
- `fw-update` writes the partition only when the 40-byte header differs.
- trxd (`src/trxd/src/rade.rs`) reads and verifies it at start, in its own
  thread, and serves it.

The web server (all routes need a login):

| Route | What |
|---|---|
| `GET /rade-info` | `{"tag": "<hash>", "bytes": n}`, or `{"tag": null}` with no module (the page then hides the RADE button) |
| `GET /rade-worker.js` | the worker (`src/trxd/web/rade-worker.js`, compiled in) |
| `GET /rade.wasm?v=<tag>` | the module, served as stored with `Content-Encoding: gzip`, cached for good |

The page's CSP allows `'wasm-unsafe-eval'`. On a PC, `[web] rade_wasm =
"<path>"` names a `RAD1` blob or a plain `rade.wasm.gz`.

## Weights: int8 only

The weight tables carry both float and int8 copies; opus's linear layers
use the float copy when it is present. The int8-only build loses nothing
measurable. Mean squared cepstral distance between the input speech and the
decoded speech (input_sample.wav, no channel noise, LPCNet features):

| Build | Distance |
|---|---|
| x86 native, float weights | 2.15 |
| x86 native, int8 weights | 2.12 |
| WASM, int8, through the page's resamplers | 2.01 |

A first resampler with short filters made this 2.51 on its own (passband
droop). The resamplers are now 40 lobes long.

## Building and testing

```bash
src/rade-web/build.sh                 # Docker (emscripten 4.0.15): src/rade-web/out/rade.wasm{,.gz}
cp src/rade-web/out/rade.wasm.gz package/trxd/
node src/rade-web/test.js speech16k.wav [snr_db] [out16k.s16]   # in the rade-web:1 image
```

`test.js` runs a round trip through the page's worker code: the speech is
resampled to 48 kHz, sent through TX, has noise added at the given SNR (in
3 kHz), goes through RX, and the result is checked. On input_sample.wav it
decodes 241 frames, the same as rade_c's own tools, with SNR estimates of
17.6 dB clean and 1.9 dB at 0 dB. It also decodes at -3 dB.

In the simulator: `trxd --sim` with `rade_wasm` set, and two headless Chrome
pages. The TX page holds PTT with a speech file as its fake microphone; the
RX page has MUTE AT TX off, so it hears the loopback. The RX page syncs
within 1 s (16-23 dB in the sim) and is back to search right after the
EOO; PTT release unkeys within 0.5 s. Harness: /data/claude/rade-test
(run.sh; board_rx.sh for a page against a real board).
