# CW through rain scatter

Rain scatter at 10 GHz and up spreads a CW carrier over a few hundred Hz
to more than a kHz (the rain's fall and the wind) and makes it fade fast.
A narrow filter catches a sliver of that, and a tone-tracking decoder has
no tone to follow. trxd has a mode for it: **CW-RS** in the mode row (CW
with the rain-scatter decoder, the filter open to 200-3000 Hz so the whole
spread gets in, AGC slow). The CW panel's RS button switches the decoder
alone; `cw_engine = "rs"` in trxd.toml makes it the default.

## How it works

1. Short-time spectrum of the audio: 256 points at 12 kHz, a frame every
   128 samples (10.7 ms), bins 200-3000 Hz; log power over a per-bin noise
   floor (its 25 % point over the first 64 frames, then a slow quantile
   tracker).
2. A small neural network (`src/rsnn.rs`, weights `rsnn.bin` in the binary)
   turns that into each frame's log-likelihood that the key is down: three
   2-D convolutions over time and frequency (8 channels), attention and max
   pooling over frequency (the signal may sit anywhere in the band, spread
   or not), four dilated temporal convolutions (32 channels, 0.7 s of
   context), about 23 k weights. It runs in 16-bit fixed point.
3. Morse by Viterbi at the character level (`src/rscw.rs`, `CharModel`):
   chains of duration states for marks and gaps (dot 0.7-1.5 dots, dash
   2.4-4.0, element gap 0.6-1.7, character gap to 4.0, word gap from 4.0),
   and a trie of the Morse codes so that the elements between character
   gaps must spell a character. Nine speeds (9.5-27 WPM) side by side;
   every half second the best-fitting speed's text is committed up to two
   seconds back. (Six speeds lost two tokens against the whole-recording
   decoder; the speed grid matters more than the beam or the lag.)
4. Squelch: no text while the network has not been sure of a key-down for
   four seconds; the Morse banks rest meanwhile (on noise they cost most)
   and start afresh at the next key-down.

A Cortex-A9 core: about 17 % with a signal, 15 % on noise (the network
about 6 %). The network's float layers ran at 50 % of a core: the A9's
NEON takes integers but not floats in compiled code, so the heavy layers
are 16-bit fixed point; a smaller network at half the frame rate did the
rest (and scored better).

### Training

The network is trained on synthetic signals only (`tools/rscw-nn/`:
rsgen.py makes them, train.py trains, export.py writes `rsnn.bin` and a
test vector that `rsnn_matches_torch` checks trxd against): random amateur
text (callsigns, locators, QSO words) keyed at 8-30 WPM with jittered
timing, as rain scatter (Gaussian Doppler spectra 20-450 Hz wide, drifting),
tropo (narrow fading tones) or steady beacons, in receiver-shaped noise,
with static crashes, a weaker second signal, receiver AGC and quantisation.
The recordings below were never used for training.

FSK beacons (key down on one tone, up on another) were tried in the
training data and dropped: a tone can then mean either state, and on-off
keying suffered.

## Results

52 recordings of 10 GHz rain scatter, tropo and beacons (SP6GWB's
collection, phone recordings of receiver audio among them); 55 tokens
(callsigns, locators, words) known to be in them:

| Decoder | Tokens found |
|---|---|
| sdroxide CwRx (the timing engine), on each recording's strongest tone | 8 |
| Previous RS engine (spread detector + element Viterbi), streaming | 18 |
| Network + character Viterbi, whole recording | 30-31 |
| Network + character Viterbi, streaming (as trxd runs it) | 30 |

The shipped weights are the average of three checkpoints (two of a
network trained from scratch, one of it fine-tuned with fluctuating noise
and weaker signals); single checkpoints scored 26-31, training longer on
the synthetic signals made it worse on the recordings. With the best
speed picked afterwards for each recording the same network reaches 32:
the rest is in the network, not the Morse decoder.

Examples (streaming): "CQ DE DL6NCI DL6NCI", "DB0ANU IN ANSBACH JN59GG",
"CQDE9A2SB CQDE9A2SB", "QRZ?DE9A1CAL 9A1CAL", "JO80HK DE SP6GWB 73".
What is left: weak recordings that give no text, and one element wrong in
a callsign (SP6GW6 for SP6GWB, OK1TES for OK1TEH).

Tests: `rsnn_matches_torch` (RSNN_VEC=testvec.bin), and on a directory of
12 kHz WAVs `RSCW_DIR=... cargo test --release rscw_nnstream -- --ignored
--nocapture` (also `rscw_viterbi`, `rscw_oracle` with RSCW_LLRDIR for
LLRs from the training side, `rscw_stream` for the previous engine).

### Larger networks (2026-09-28)

train.py takes the temporal layers' width and dilations (C1D, DIL); the
weight format RSN3 carries them and rsnn.rs runs any size, now frame by
frame (`Stream`: every frame through every layer once, bit for bit the
batch pass; the chunked batch redid 2 x reach frames of context each time).
Tried: C2D 16, C1D 64, DIL 1..32 (132 k weights) and C2D 24, C1D 64,
DIL 1..64 (161 k), 8000 steps each. Midway they scored no better than the
shipped network on the recordings; at the end (streaming, as trxd runs):
the 161 k one 34 of 55 (characters 33, oracle 35), the 132 k one 33,
against the shipped network's 30. The cost on the A9 (streaming): about
23.5 % of a core for 132 k (6 % shipped): the case for running it in the
FPGA.

### The front end in the FPGA

Most of the larger networks' work is the front (the three 2-D layers:
450 k of the 161 k network's 600 k multiplies a frame). maia-sdr
`rsnn_front.py` (0x43C50000, id RSF1) does it: `Net::front_q` bit for bit
(fixed point all through, softmax from an exp table with one reciprocal a
frame), one multiply a cycle at the CPU clock, 4.6 ms a frame for the 161 k
network (a frame is 10.7 ms), up to c2 24 / c1 64. A feature row in, the
1x1's outputs for the frame three rows back out; the rings (the frames the
time kernels take) and the weights (in BRAM, loaded once) stay in the FPGA.
rsnn_fpga.rs drives it: `Stream` takes it when the bitstream has one and
nobody else holds it, else the CPU (TRXD_NO_RSNN_FPGA=1: always the CPU).
The temporal layers stay on the ARM.

Tests: maia-hdl `test_rsnn_front.py` against `rsnn_front_vectors`
(`cargo test rsnn_front_vectors -- --ignored`, RSNN_FRONT_VEC=<json>,
RSNN_FRONT_SMALL=1 for the quick small network; RSNN_FRONT_VEC and
RSNN_FRONT_ROWS on the test side for the full-size one); on a board
`stream_matches_batch` runs the FPGA front against the CPU's batch pass.

## Not done

- A language model (callsign structure, repeated calls combined): the
  near misses above would mostly go.
- FSK beacons (see Training).
