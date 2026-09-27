# CW through rain scatter

Rain scatter at 10 GHz and up spreads a CW carrier over a few hundred Hz
to more than a kHz (the rain's fall and the wind) and makes it fade fast.
A narrow filter catches a sliver of that, and a tone-tracking decoder has
no tone to follow. trxd's live CW box has a third engine for it, "rs"
(the RS button in the CW panel, or `cw_engine = "rs"` in trxd.toml);
`src/trxd/src/rscw.rs`.

Open the CW filter wide (2 to 3 kHz) so the whole spread reaches it.

## How it works

1. Short-time spectrum of the audio: 256 points at 12 kHz (47 Hz bins), a
   frame every 5.3 ms.
2. Noise per bin: a slow quantile tracker.
3. The signal's spread: each bin's average excess SNR over a few seconds.
   The bins, weighted by S / (1 + S), give the matched energy detector for a
   spread signal in noise: one score a frame.
4. A log-likelihood ratio a frame (keyed against not): noise statistics
   from the score's low quantiles, the signal's level from a rolling upper
   quantile (it fades), a keyed frame modelled with a wide spread.
5. Morse by Viterbi: dots, dashes and the three gaps as chains of states
   with flexible durations (dot 0.6-1.6 dots, dash 2.2-4.2, gaps 0.5-1.8,
   1.8-4.5, 4.5 on), Morse's grammar between them. A bank of seven speeds
   (10-28 WPM) runs side by side; every half second the best-fitting
   speed's path is turned into text up to two seconds back.
6. Squelch: no text while the signal's level is under 6 noise sigmas (noise
   alone would decode as a stream of E and T).

A Cortex-A9 core: about 7 %. No FPGA help needed (the FFT is 256 points
187 times a second; the Viterbi bank about 2000 states).

## Results

52 recordings of 10 GHz rain scatter, tropo and beacons (SP6GWB's
collection, phone recordings of receiver audio among them); 55 tokens
(callsigns, locators, words) known to be in them:

| Decoder | Tokens found |
|---|---|
| sdroxide CwRx (the timing engine), on each recording's strongest tone | 8 |
| Spread detector + threshold keying | 12 |
| Spread detector + Viterbi, whole recording | 18 |
| Streaming (as trxd runs it) | 18-19 |

Examples (streaming): "CQ CQ CQ DE DL6NCI DL6NCI", "HA8MV/P DE SP6GWB FB
TNK 73 73", "CQ DE 9A2SB CQ DE 9A2SB", "DB0ANU IN ANSBACH JN59GG".
Synthetic rain scatter (Rayleigh fading, 300 or 800 Hz spread, 0 dB SNR in
2.5 kHz, 18 WPM) copies without error once the trackers have settled (a
few seconds).

Tests: `synthetic_rain_scatter` (always), and on a directory of 12 kHz
WAVs `RSCW_DIR=... cargo test --release rscw_stream -- --ignored
--nocapture` (also `rscw_viterbi`, `rscw_score`, `rscw_baseline`).

## Not done

- Speed follows the best of seven fixed speeds; a QSO's two stations at
  different speeds decode through whichever fits better at the moment.
- Several of the phone recordings pass through a receiver AGC that pumps
  within characters; a per-frame gain normalization did not help on them.
