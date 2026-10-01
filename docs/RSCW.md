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

(Since 2026-09-29 `rsnn.bin` is the 161 k network b2 below: 34 of 55
streaming; with the whole network in the FPGA it costs the A9 0.5 % of a
core. Where the FPGA has no engine it runs on the CPU at about 48 %.)

The first shipped weights were the average of three checkpoints (two of a
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

The temporal layers followed (2026-09-29), in the `trx` bitstream only
(docs/FPGA-MODES.md; the `all` one has no room): maia-sdr
`rsnn_temporal.py`, `rsnn_axi.py` ("RSF2"). Their weights stay in DDR
(1 MB reserved at 0x16200000, device tree `rsnn_weights`, written by
trxd once) and stream in over HP0 each frame, one multiply a cycle; each
layer keeps its last 4d + 1 input frames in block RAM (up to 520 frames of
64 channels, 8 layers). The engine starts on its own when the front is
done with a frame; the ARM keeps the features, the last 1x1 and the
character decoder. trxd reads the layer registers back and runs the
temporal layers itself if they do not match.

On Libre 1 (`stream_matches_batch` bit for bit on the FPGA path):

| Network | ARM, all on the CPU | front in the FPGA | front + temporal in the FPGA |
|---|---|---|---|
| shipped (23 k weights) | 6.9 % of a core | 1.7 % | 0.4 % |
| b2 (161 k) | 47.7 % | 8.8 % | 0.5 % (6.3 ms a 10.7 ms frame) |

trxd as a whole in CW-RS mode: about 18 % of a core more than idle USB
(31.5 -> 49.8 %), almost all of it the features and the character
Viterbi banks now.

The banks (nine speeds, 12 749 states, 94 frames a second) then took 11 %
of a core on the A9 (`rscw_banks_speed`, trxd-test on a board): memory
bound, the f64 scores (200 KB) do not stay in its caches. Now 5.7 %, with
the same text on every recording (`rscw_nnstream`, compared line by
line): f32 scores, the best score taken off as each is read instead of a
pass over all of them, chains wholly outside the beam skipped (from each
chain's maximum), successors and log counts precomputed per chain, the
oldest frame's back pointers reused. `rscw_nnstream` also prints the time
in the network and in the banks.

Tests: maia-hdl `test_rsnn_front.py` against `rsnn_front_vectors`
(`cargo test rsnn_front_vectors -- --ignored`, RSNN_FRONT_VEC=<json>,
RSNN_FRONT_SMALL=1 for the quick small network; RSNN_FRONT_VEC and
RSNN_FRONT_ROWS on the test side for the full-size one); on a board
`stream_matches_batch` runs the FPGA front against the CPU's batch pass.

### Tried 2026-09-30, no gain

- Repeat combining (`rscw_fold`): the repetition period from the LLRs'
  autocorrelation (8-150 s), the repetitions summed, the folded message
  decoded. The period only stands out (r 0.5-0.9) on recordings that
  decode anyway; on the weak beacons r is 0.1-0.2 and the folded text is
  noise. The score stays 34 (plain or folded, either counted).
- Weight averages of b2 and its fine-tunes (c2, c3, d1, the 6000-step
  checkpoint): 32-35. The best (b2 + c2, 35) wins two tokens and loses
  one, with the text changing on most recordings: noise at 55 tokens,
  not shipped.

The misses left: weak beacons with no keying the network finds
(SR6NCI, OE5XBM, S51ZO, S56BD, SR6KBL, IW5DHN), single-element slips
(SP6GW6, OK1TES, E0FGB for DB0FGB), and SSB-only recordings.

## Against operators' labels (2026-10-01)

62 fragments of the same recordings that operators labelled (SP5BIN,
SQ6EMM; from the separate rain-scatter-decoder project): 45 with CW and
the text heard, 17 voice, weak or empty. Scored as that project does:
the longest run of the operator's text found in the output, and whether
anything is shown on a fragment without CW. The decoder runs over the
whole recording and the characters inside the fragment's time window
count (`rscw_nntimed`, characters with times). Fragments are split in
two halves: gates tuned on one, the other looked at once.

Found and fixed:
- The last 2.7 s of a recording were never decoded: the network gives a
  frame's output once it has its context after it (`RsNn::latency`).
  `finish()` now pushes that much silence through. Fragments decoded
  alone: mean 20 -> 32 %.
- Text on noise and voice (E, T, TE ... on 88 % of the fragments without
  CW). Two gates (defaults; `RSCW_CHARCONF`, `RSCW_JUNK`): a character is
  shown only if the network's LLRs agree with the path's key states over
  it by at least 5 per frame on average, and a word of at most two of the
  shortest codes (E T I A N M) is dropped (only such a word's start waits).

Results (b2 network), shown text / at least half / at least 80 % / mean:

| | invents | half+ | 80%+ | mean |
|---|---|---|---|---|
| tuning half, gated | 0 % | 35 % | 26 % | 36 % |
| other half, gated | 38 % | 23 % | 14 % | 23 % |
| other half, rain-scatter-decoder | 0 % | 23 % | 9 % | 23 % |

On the other half three voice recordings still showed a letter or three
(O TK, S AJ, K).

Then a Morse rhythm test (`morse_fit`, a port of rain-scatter-decoder's
`_quant_fit`): over the last 4 s of the network's key decisions, how well
the run lengths fit one time unit (elements 1 and 3, gaps 1, 3, 7 with a
fitted stretch, gaps only penalised when short), 0..1. Text is shown only
at 0.45 or above (`RSCW_MORSE`): just above every fragment without Morse
in the tuning half (their highest 0.42; Morse mostly 0.65-0.96). With it
the character gate eased to 4. Chosen on the tuning half before the other
half was looked at again:

| | invents | half+ | 80%+ | mean |
|---|---|---|---|---|
| other half, gate 5 (before) | 38 % | 23 % | 14 % | 23 % |
| other half, gate 4 + rhythm | 12 % | 23 % | 14 % | 22 % |
| all 62, gate 4 + rhythm | 6 % | 27 % | 20 % | 28 % |
| all 62, rain-scatter-decoder | 0 % | 27 % | 16 % | 26 % |

The one left: a "T" in an FM voice fragment. The 55-token score with
the defaults: 34 (as ungated).

## Training on the GPU, scored on the fragments (2026-10-01)

With the batch made on the GPU (rscw/nn/rsgen_gpu.py) a b2-sized network
trains in 3-6 minutes, so recipes were compared by four seeds each on the
tuning half of the 62 fragments, the other half kept for the end. No
synthetic recipe beat b2: 16000 steps looked +6 on the tuning half and
was +1 on the other (selection among six recipes on 23 fragments);
weaker signals, realism, phone codecs and 32000 steps did not help.

Self-training on real recordings did. An ensemble (b2 and eight 16000-step
networks) labels 57 real recordings (none of the other half's): frames
where its mean LLR is beyond 2 are labels, and 4 s windows whose keying
fails the Morse rhythm test (0.45) are left unlabelled (labelling them key
up instead taught the network to drop weak CW). Each training step adds
16 such real crops to the 32 synthetic ones (16000 steps, from scratch).
On the other half (recordings the training never heard), four seeds:
mean 28.0 +- 1.1 % with the gates at 6 and 0.55 (b2 22 %); ungated 41 %
(b2 31 %). These networks are surer, so the gates were retuned for them
on the tuning half, inside the region where no seed showed text without
Morse.

Shipped: rsnn.bin = st3-s5 (the best seed on the tuning half) with the
gates at 6 / 0.55 / 3 (`RSCW_CHARCONF`, `RSCW_MORSE`, `RSCW_JUNK`), after
an independent review of method and code (2026-10-01):

- No leakage: no other-half recording is in the self-training pool, by
  name and by audio fingerprint (two other-half recordings share 16 s of
  audio, so that half holds about 21 independent CW fragments, not 22).
- The gain is modest and rests on few fragments: per fragment against b2
  on the other half, +6 to 9 points mean (95 % interval roughly +1 to
  +17), better on 4 to 6 fragments and worse on none; every seed is at
  least as good as b2. Text on fragments without Morse is unchanged
  within noise (one or none of 8): the gates do that, not the network.
- The other half has been looked at several times during this work
  (the rhythm test, the choice of st3 and the retuned gates followed
  looks at it): treat it as half-tuning now. A clean verdict needs new
  labelled fragments the pipeline has never touched.

The code review found that the rhythm gate scored the newest 4 s while
the characters it let through were 2-2.5 s older, so a transmission's
last characters were dropped as its window filled with silence. Now
each half second's window score is kept with its frame, and characters
pass on the best score of the windows that hold them. Also: the junk
filter's state is cleared (and the word closed) at the squelch reset;
the timed character and score logs are kept only on request (`log`; a
live decoder runs for days) and the text is capped; the word filter
covers three letters (an EEE came through at two).

With these, other half: st3-s5 at 6 / 0.55 / 3 mean 33 %, half+ 32 %,
80 %+ 27 %; b2 at its old gates 4 / 0.45 / 2 mean 25 %, 23 %, 18 %; each
shows text on one of 8 fragments without Morse (st3-s5: RRDE on one an
operator called too weak to read; b2: T TEM O on voice). The 55-token
score: 33 (b2 34). On the board the network runs on the FPGA front and
temporal layers, bit-exact with the batch forward pass.

## Not done

- A language model (callsign structure, repeated calls combined): the
  near misses above would mostly go.
- FSK beacons (see Training).
