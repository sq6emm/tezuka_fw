# DATV: DVB-S2 between two boards (trxd)

Low-rate digital amateur television from the web UI: a browser's camera and
microphone go out as DVB-S2, and a board receives DVB-S2 and shows it in the
browser. Everything runs in trxd software on the 384 kS/s stream; the FPGA is
unchanged. Target: reliable, narrow links (terrestrial, 23 cm), not picture quality.

## Chain

```text
TX  browser: camera -> OffscreenCanvas (4:3 crop) -> VideoEncoder H.264 (Constrained Baseline, Annex B)
             mic -> mono -> AudioEncoder Opus (20 ms)
             -> WebSocket [4][key][i64 us][H.264] / [5][i64 us][Opus]
    trxd:    dvbs2::ts::Mux (MPEG-TS, constant rate, pulled by the modulator)
             -> dvbs2::Modulator (BBFRAME, BB scrambling, BCH, LDPC, QPSK, PLFRAME,
                pilots, PL scrambling, RRC 0.35) -> TX NCO -> DAC

RX  trxd:    stream IQ -> dvbs2::rx::RxThread (own thread): NCO + RRC matched filter
             (2-3 samples/symbol) -> Gardner timing -> PLHEADER sync -> carrier (averaged
             acquisition, pilot tracking) -> LLRs -> LDPC (layered BP) -> BBFRAME -> TS
             -> dvbs2::ts::Demux (PAT/PMT/PES) -> WebSocket [6][key][i64 us][H.264] / [7][i64 us][Opus]
    browser: VideoDecoder -> canvas, AudioDecoder -> AudioContext (short jitter buffer)
```

Web UI: header button **DATV**. START sends on the TX frequency (the VFO, or the
split TX VFO); RECEIVE decodes on the RX frequency. Both use the panel's symbol
rate, code rate and pilots settings, which must match the other station.

## Settings and what they carry

QPSK, short FECFRAMEs, CCM, roll-off 0.35, pilots on (the receiver needs them).
Symbol rates are those with whole samples per symbol at 384 kS/s: 32, 48, 64,
96 and 128 kS/s. Occupied bandwidth is 1.35 x the symbol rate.

| Symbol rate, code rate | TS kbit/s | Profile (board picks it from the TS rate) |
|---|---|---|
| 64 kS/s, 1/4 | 22.9 | 160x120, 2 fps, video ~6 k, Opus 6 k in 800 ms PES, PSI every 1 s |
| 32 kS/s, 1/2 | 26.6 | same |
| 64 kS/s, 1/2 | 53.2 | 320x240, 5 fps, video ~24.5 k, Opus 12 k in 200 ms PES |
| 128 kS/s, 2/3 | 161.4 | 320x240, 10 fps, video ~104 k, Opus 16 k |

At these rates the 188-byte TS packet is the unit of cost: one 20 ms Opus frame
per PES would cost 50 packets a second. The mux packs audio into long PES and
stamps PCRs exactly when the modulator pulls a packet.

Repetition follows ISO/IEC 13818-1 and ETSI TR 101 290 (since 2026-09-30):
PAT and PMT every 0.5 s, the SDT within 2 s, a PCR within 100 ms (90 ms
below 200 kbit/s) and every 40 ms from 200 kbit/s up. Every video packet
reserves an 8-byte adaptation field that takes the PCR when one is due; a
PCR-only packet goes first when audio or tables hold the queue. PCR
intervals are whole packet slots, never under 3 (so data moves between
them): from 45.1 kbit/s (30 packets a second) up 3 slots are within
100 ms. The lean profile below that is a deliberate exception (tables
every second, a PCR every 0.5 s, PTS up to 1.4 s after the data where
13818-1's T-STD allows 1 s): the standard rates would leave it no
picture, and only MiniTiouner-class receivers work there anyway. (Until
2026-10-01 the boundary was 40 kbit/s: from 40 to about 50 kbit/s PCRs
every 2 slots and 12 kbit/s audio took nearly every packet and almost all
video was dropped. The 45-80 kbit/s profile now has 8 kbit/s audio, 300 ms
to a PES; the mux's lateness estimate counts the table, PCR-only and audio
slots.) EIT sections are at least 25 ms apart (TS 101 211). When the
modulator says (`Mux::with_delivery`), the NIT carries the
satellite_delivery_system_descriptor (S2: frequency, symbol rate,
roll-off, modulation, FEC) or the T2_delivery_system_descriptor (T2: at
1.7 MHz with bandwidth, guard, FFT and centre frequency; the other channel
widths have no code and get the short form). The mux adds an access unit
delimiter only when the access unit has none (after a 3- or 4-byte start
code). `repetition_meets_tr101290` checks the intervals at nine rates
with video at its budget; TSDuck (`tsp -P continuity -P pcrverify -P analyze`, image
tsduck:1) finds no continuity errors, every PCR within 27 us of the constant
rate, service type 0x16 and network id 0xFF01.

Each table's version_number changes when its content does (the EIT's at the
hour, as its present event moves): receivers cache by version.
T-STD timing: every PES is in whole before its PTS, at most 1 s after its
first byte (lean profile 1.4 s), and a stream's PTS never steps back. Audio has its own queue and goes before video (its PES
are few and small). A video frame that would come in after its PTS is
dropped (a keyframe asked for); a late keyframe drops the video queued
before it, and if still late takes a later PTS (the timeline moves on).
Keyframes: the browser sends one every 2 s; the mux asks for one if none
came for 2 s, and puts the last SPS/PPS in front of a keyframe that comes
without them. `repetition_meets_tr101290` checks PES arrival against PTS
and PTS order at five rates, `versions_follow_the_content` and
`keyframes_carry_parameter_sets_and_come_often` the rest. With over 1 s of video queued
it drops non-key frames and asks the browser for a keyframe; over 3 s it
flushes.

The TS carries an SDT: service name = the callsign from SET, provider SQTRX,
service type 0x16 (H.264 SD television), original network id 0xFF01 (the
private range; 0x0001 is a registered satellite network). The DVB SI
tables EN 300 468 asks for go out too: NIT actual (network "SQTRX DATV",
this TS and its service) every 9.5 s, EIT present/following actual (the
present event "DATV <call>", "Amateur television", the current hour,
running; no following event) every 1.7 s, TDT (UTC from the board's clock)
every 25 s; the PAT names the NIT's PID and the SDT flags the EIT. trxd's
own receiver reads the SDT, NIT, EIT and TDT back (`ts::Si`) and the DATV
panel shows them: service and provider, network, the event on air, and the
transmitter's clock against the browser's. TSDuck's `tstables` decodes all
of them.

The receiver (`rx.rs`, `ts::Demux`):
- PLFRAMEs it does not decode are stepped over by their own length, told
  apart by the PLS code (all share the SOF, which alone passes the header
  metric): dummy frames (EN 302 307-1 5.5.1; the TS continues across them)
  and frames of another MODCOD or size (VCM: counted, the packet
  straddling one is lost). Only the configured MODCOD is decoded.
- BBFRAMEs whose MATYPE is not TS / single stream / no ISSY / no
  null-packet deletion, whose UPL is not 188 bytes, or whose DFL or SYNCD
  is impossible are not taken (counted, logged once); SYNCD 0xFFFF (no
  packet starts in the frame) continues the packet.
- Each user packet's CRC-8 (in the next packet's first byte) is checked: a
  mismatch sets the transport_error_indicator and the demux drops the
  packet (its PES then too).
- Sections may span packets and follow each other in one; the PMT PID (from
  the PAT) and the streams (from the PMT) follow changes; one duplicate
  packet is ignored as 13818-1 allows; PTS are unwrapped over the 33-bit
  wrap.

Opus is not a
DVB broadcast codec (TS 101 154): TVs and set-top boxes show the picture
without sound; players and DATV receivers decode it.
Opus in TS follows ffmpeg (registration descriptor "Opus", control header per
access unit); ffprobe, ffmpeg and VLC read it.

## Verified (2026-09-26, on power)

- Symbols bit-exact against leandvbtx (leansdr work branch) for QPSK 1/4, 1/3,
  1/2, 2/3 short, with and without pilots, over 320-1131 frames each.
- LDPC tables for those rates checked equal between leansdr and GNU Radio
  gr-dtv (leansdr's short 2/5 and 3/4 tables have rows with a wrong entry
  count; not used here).
- Short 3/4 (2026-09-30): our table equals gr-dtv's ldpc_tab_3_4S, and a
  frame from `--dvbs2-mod` checks against it independently
  (datv-ref/harness/check_short34.py: PL descrambling, demapping, parity
  recomputed from the information bits): 0 of 4320 parity bits wrong.
  leandvbtx's short 3/4 gets 2198 wrong, from its broken table, so it is
  no reference for that rate.
- leandvb decodes the modulator's output back to the identical TS.
- Browser path (Chrome 153, fake camera and microphone, headless) through
  trxd --sim: DVB-S2 frames 319/319, about 5 fps shown, about 0.85 s end to end.
- Receiver against impairments (20 s recordings of a real browser stream):
  +121 Hz and 30 ppm, +1 kHz and 50 ppm, -1.4 kHz and -50 ppm all acquired;
  QPSK 1/2 decodes every frame after acquisition down to Es/N0 1.5 dB
  (DVB-S2 nominal about 1 dB). Acquisition takes 16 frames (about 2 s at 64 kS/s).
- Decoder alone (ideal LLRs): frames clean from Es/N0 -2.5 dB (1/4),
  0 dB (1/3), +0.5 dB (1/2), about 4 dB (2/3).

## Every mode on every band

DATV-OTA.md: all DVB-S2 rates and modes and all DVB-T2 modes, both
directions, 145 MHz to 5.7 GHz (2026-09-30), and what it found: the LO
now goes beside a DVB-S2 signal being received (the AD936x's image below
1 GHz), slow 8PSK with a large carrier offset or drift, a receive thread
crash; DVB-T2 below 1 GHz is still open.

## Over the air (2026-09-26, Libre 2 -> Libre 1, 1255.000 MHz, indoors)

64 kS/s, QPSK 1/2, pilots, Libre 2 at 0 dB TX attenuation (DATV runs 6 dB
under the TUNE carrier to keep RRC peaks off the DAC ceiling):
- Libre 1 locked in about 10 s at +130 Hz (Libre 2's crystal); Es/N0
  wandered 0-5 dB without losing lock; 376 of 418 frames decoded, the losses
  almost all during acquisition; about 4 pictures a second shown.
- CPU on Libre 1 (Cortex-A9, 2 cores): demodulator 32-52 % of a core, LDPC
  and demux 55-66 % of the other (they run on separate threads), 72-82 % of
  both cores in all. Libre 2 while sending: 16-36 %.
- The receiver has to keep up in real time: with demodulation and decoding
  on one thread, the A9 dropped IQ blocks and never held lock. Decoding on its
  own thread, behind a 4-frame queue, drops whole frames instead when busy.

## Not yet done

- Higher rates (256 kS/s QPSK 3/4) with the FPGA doing the front end: see
  `DATV-FPGA.md` (designed and simulated, not yet in a bitstream).

- More than 64 kS/s QPSK 1/2 on the A9: 128 kS/s or rate 1/4 cost more
  decoding per second; measure before relying on them. Ideas if needed:
  skip the SSB demodulator while DATV receives, min-sum for rates >= 1/2,
  fixed-point NEON LDPC.
- Two-way at once: each board would send on its own frequency and receive the
  other's; the other signal has to fall inside the same 384 kS/s stream (about
  +-120 kHz of the TX signal), and the board's own transmitter leaks into its
  receiver.
- No BCH decoding on receive for short frames (LDPC convergence and the
  BBHEADER CRC stand in); long frames have it (DATV-FPGA.md).

## Test tools

`trxd --dvbs2-mod`, `--datv-mux`, `--dvbs2-demod` and `--ldpc-helper` (leandvb's
external LDPC decoder protocol) are command-line entry points for offline
tests; `cargo test` covers the encoder, mux/demux round trip and an end-to-end
modulate-impair-receive test. With `TRXD_SIM_TX_DUMP=<file>`, `--sim` writes
everything it transmits as complex f32 at the stream rate.
