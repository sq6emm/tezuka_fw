# DATV over the air: every mode on every band

Libre 1 (192.168.12.178) and Libre 2 (192.168.12.162), 4 m apart indoors
with line of sight, TX attenuation 0 dB, 2026-09-30 night. Each run: the
receiving board's web UI starts DATV receive, 8 s later the other board's
web UI sends for 45 s (headless Chrome, fake camera and microphone), then
both pages close. 25 modes (DVB-S2 QPSK 1/2, QPSK 3/4, 8PSK 3/4 long
frames with pilots at 33, 66, 125, 250, 333 and 500 kS/s; DVB-T2 1.7 MHz
QPSK 1/2, 3/4, 16QAM 1/2, 3/4, 2.0 MHz QPSK 1/2, 16QAM 1/2, 1.35 MHz QPSK
1/2) on 145, 436, 1255, 2330, 3410 and 5700 MHz, both directions.

Frames count from the receiver's first lock to its last 5 s report while
the transmitter was on (the receiver holds lock a few seconds after the
signal goes: an earlier count that included that window showed losses
that never happened, e.g. 93 % and 90.8 % for DVB-T2 at 2330 MHz). MER is
the receiver's own figure, median and minimum of the locked reports.

Harness (outside the repo): `datv-ref/harness/matrix/` on power, with
every run's logs in `runs/` and the results page. `ota.sh TXIP RXIP
FREQ MODE SR RXSR SECS TAG` around `datv-ref/harness/cdp_ota.py`,
`matrix.sh DIR BANDS...` (resumable, skips runs already in
results.jsonl, waits while a PAUSE file exists), `parse3.py` (one run's
logs to JSON), `table.py` and `doc_tables.py` (the tables below),
`spec.py`, `evm.py`, `image.py` (capture analysis used below). The
scripts carry the scratch paths they ran from; adjust T= to reuse them.

## Found and fixed: DVB-S2 receive LO placement (low bands)

500 kS/s on 145 MHz never locked and on 436 MHz stopped at MER 14-15 dB,
while 333 kS/s on the same bands, and 500 kS/s on 1255 and 2330 MHz, gave
27-32 dB. The input was clean: raw DDC captures (`TRXD_NO_SYMSYNC=1`,
`touch /tmp/datv-iq`) at 436 MHz showed a flat signal about 28 dB above
the noise, and the software receiver, the FPGA symsync model and a
home-made demodulator all read 15 dB from it (2330 MHz: 27.6 dB). It was
not the analog filter (opening it to 2 MHz changed nothing), not
compression (10 and 20 dB less TX power made it worse, never better: the
error scales with the signal), not a static I/Q image or ISI (linear and
widely linear equalizers, even refitted every 1000 symbols, gained
nothing). The error sat at the band edges (-30 dB relative to the signal
in the middle, -10 dB at +-220 kHz).

What decided it was the RX LO's place. trxd left the LO wherever it
was for DVB-S2 receive (any offset that fits the 384 kS/s stream, here
100.8 kHz below the signal). At 436 MHz, 500 kS/s QPSK 1/2:

| LO below the signal | 0 | 25 kHz | 100 kHz | -100 kHz | 150 kHz | 347.5 kHz (fix) |
|---|---|---|---|---|---|---|
| MER dB | 6.2 | 4.9-6.6 | 11-16 | 14.8 | 27.1 | 29.8 |

The damage depends on how symmetric the signal is around the LO: parts
at +f and -f overlay each other's image. The AD936x's image rejection
falls off away from the LO below 1 GHz, with a frequency-dependent I/Q
mismatch its quadrature tracking cannot remove (turning tracking off
changed nothing; the driver has no RX quadrature calibration to rerun).
Self-reception (a board receiving its own transmission) at 436 MHz is
equally bad at every rate, at 2330 MHz fine.

Fix (trx.rs `datv_lo_offset`, `datv_rx_half`): receiving DVB-S2 through
the FPGA DDC and not sending, the LO goes half the occupied width plus
10 kHz below the signal (what the software transmitter already did), in
automatic mode at the widest scan rate's offset so the LO stays put, and
the RX filter opens to 1.3 x twice (offset + half width) when that is more
than radio.rf_bandwidth. The DC spike leaves the signal as a side effect.
500 kS/s after the fix: 436 MHz QPSK 1/2 29.8 dB, 8PSK 3/4 29.5 dB; 145
MHz QPSK 1/2 99.8 % at 15.1 dB (the 2 m path's limit); 2330 MHz 8PSK 3/4
27.3 dB (unchanged); 33 and 250 kS/s unchanged.

## Found and fixed: DVB-S2 8PSK at 33 kS/s on 13 cm and up

8PSK 3/4 at 33 kS/s locked at 2330 and 3410 MHz but decoded nothing for
12 s, or for the whole run: frame headers matched, the pilots' Es/N0 read
-0.3 dB, the data 6.6 dB, on a 26 dB signal. The simulated link
(`long_link_ddc_at`) reproduced it exactly from a carrier offset of
rs / 90 up (367 Hz at 33 kS/s; a 0.2 ppm crystal is 650 Hz at 3.4 GHz),
with or without the timing-recovery models.

1. Known-block phases (rx.rs `block`): during acquisition the NCO is held,
   and the 90-symbol header turned through more than a cycle; averaged as
   it came its phase flipped by pi, the frame's phase fit landed an alias
   (rs / 1476, 22 Hz at 33 kS/s) off, and tracking (which deliberately does
   not hop aliases) kept it there; every restart did the same. Each block
   is now measured with the frequency so far taken out inside it, about
   its centre. Simulated: 650 Hz 0 -> 384 packets.
2. Acquisition and drift: Libre 1's carrier moves about 5 Hz/s for tens of
   seconds after keying (727 -> 652 Hz at 3410 MHz). Acquisition averaged
   16 frames, 11 s at 33 kS/s 8PSK, and the average fell an alias behind.
   It now ends after 3 frames once two of them had known-block Es/N0 of 10
   dB or more (weak signals still average 16), and tracking is second
   order (learns the drift per frame, clamped to a quarter alias).
   Simulated 8PSK 3/4 33 kS/s, 700 Hz: -5 Hz/s 17 of 29 frames bad -> 1;
   -10 Hz/s 5 bad; -20 Hz/s still fails (acquisition).
3. Watchdog (trx.rs `datv_rx_watchdog`): a fixed-rate S2 receiver locked
   for 8 frames and 3 s with nothing decoded and known-block Es/N0 3 dB
   below the data's restarts (not DVB-T2, not automatic mode).

On air afterwards: 3410 MHz 100 % and 96.6 %, 5700 MHz (1082 Hz off) 100 %,
2330 MHz 100 %. Tests: `psk8_slow_rate_large_offset` (650 and 1000 Hz, and
-5 Hz/s), probe `psk8_slow_drift` (DATV_DRIFT=Hz/s, ignored).

## Found and fixed: receive thread crash

On 2 m, 8PSK 3/4 at 500 kS/s in an interference burst, the datv-rx
thread panicked (index 4294967295, twice): confirming lock, the receiver
takes the frame a frame length before the second header; with the first
header in the buffer's first TRACK_SLACK symbols and symbols lost between,
`best - l` went below zero and wrapped (32-bit). The panel showed nothing
until DATV was restarted. The second header is now a new candidate
instead (`early_second_header_at_the_buffer_start` builds that case; it
panics on the old code).

## Automatic receive

Symbol rate "Auto", Libre 1 sending, Libre 2 scanning (the scan uses the
new LO placement): every run found the mode and decoded every frame.

| MHz | Sent | Found and locked after | Decoded | MER dB |
|---|---|---|---|---|
| 436 | 8PSK 3/4 500 kS/s | 12 s | 100 % | 30.9 |
| 436 | QPSK 1/2 33 kS/s | 12 s | 100 % | 29.7 |
| 436 | QPSK 3/4 125 kS/s | 12 s | 100 % | 29.8 |
| 2330 | 8PSK 3/4 500 kS/s | 7 s | 100 % | 27.4 |
| 2330 | QPSK 1/2 33 kS/s | 17 s | 100 % | 26.6 |
| 2330 | QPSK 3/4 125 kS/s | 2 s | 100 % | 27.7 |

## 9 and 6 cm

Where the path limits MER, DVB-T2 reads 10-15 dB below DVB-S2 at 500
kS/s (1.5 MHz instead of 0.675 MHz of noise, 7-10 dB OFDM back-off). 6 cm
Libre 1 -> Libre 2: S2 17 dB, T2 1-5 dB; Libre 2 -> Libre 1: T2 14 dB
and every block. Moving Libre 1's TX 1080 Hz down (residual carrier
offset -18 Hz instead of +1067) changed nothing (MER 2.8 vs 1.6 dB): not
a frequency problem. On 9 cm, during the second pass, MER fell 5 dB in one
step and T2 with it; 3410 MHz is the lower edge of 5G n78 (3.4-3.8 GHz).
Libre 1's carrier settles after keying (about 5 Hz/s at 3.4 GHz, 12 Hz/s
at 5.7 GHz, for half a minute): the slowest rates on 6 cm can lose a frame
or two then.

## Open: DVB-T2 on 2 m and 70 cm

The same image hurts DVB-T2 below 1 GHz, and it cannot move: the FPGA T2
resampler is centred on the LO, and a 1.5 MHz channel beside the LO does
not fit 3.072 MS/s. On 145 and 436 MHz T2 reads MER 1-6 dB where DVB-S2
gets 20-30; QPSK 1/2 still decodes, 16QAM does not. 1255 MHz and up: 26-28
dB, every block. Possible fixes: an NCO in front of the T2 resampler (the
channel beside the LO, the RX filter opened), or a 2x2 equalizer per
carrier pair (k, -k) in the FPGA equalizer (t2eq), trained on the pilots:
with the LO in the channel's centre each carrier's image lands exactly
on its mirror carrier.

## The 2 m path

The antennas are not for 2 m. At 145 MHz Libre 2's noise floor is 17 dB
higher than at 2330 MHz (local noise and a steady carrier at about 144.9
MHz), a 500 kS/s signal arrives only about 10-20 dB above it with a 5 dB
tilt across its 675 kHz. DVB-S2 MER falls from 20 dB at 33 kS/s to 15 dB
at 500 kS/s.

## Results

Counts start at the receiver's first lock, so at 33 kS/s (frames of 0.7-1
s) the 2-4 frames of acquisition count too. Both boards ran test binaries
bind-mounted over /usr/bin/trxd. Every DVB-S2 run is with the LO,
known-block, acquisition and tracking fixes (the crash fix came last; the
two 2 m 8PSK runs it showed up in were repeated with it); DVB-T2 runs,
which these fixes do not touch, partly with earlier builds. Before the fixes, Libre 1 -> Libre 2:
145 MHz 500 kS/s no lock (every mode), 436 MHz 500 kS/s 13.5-15.0 dB; 8PSK
3/4 33 kS/s 2330 MHz 74.6 %, 3410 MHz 0 %; 5700 MHz QPSK 1/2 33 kS/s 67.6 %.

### Libre 1 -> Libre 2

MER dB (median) / % of frames decoded after lock.

| Mode | 2 m 145 | 70 cm 436 | 23 cm 1255 | 13 cm 2330 | 9 cm 3410 | 6 cm 5700 |
|---|---|---|---|---|---|---|
| S2 QPSK 1/2 33k | 22.6 / 92.3 | 29.7 / 100 | 29.4 / 100 | 26.8 / 100 | 25.5 / 100 | 20.9 / 100 |
| S2 QPSK 1/2 66k | 21.1 / 100 | 30.5 / 100 | 31.0 / 100 | 28.7 / 100 | 27.4 / 100 | 21.1 / 98.7 |
| S2 QPSK 1/2 125k | 18.4 / 100 | 29.5 / 100 | 30.4 / 100 | 28.0 / 100 | 27.6 / 100 | 20.2 / 100 |
| S2 QPSK 1/2 250k | 17.2 / 100 | 28.9 / 100 | 29.5 / 100 | 27.2 / 100 | 26.7 / 100 | 18.2 / 100 |
| S2 QPSK 1/2 333k | 15.4 / 97.8 | 30.1 / 100 | 31.1 / 100 | 27.7 / 100 | 26.8 / 100 | 18.0 / 100 |
| S2 QPSK 1/2 500k | 13.6 / 99.7 | 30.9 / 100 | 32.5 / 100 | 28.2 / 100 | 26.7 / 100 | 17.2 / 100 |
| S2 QPSK 3/4 33k | 22.8 / 100 | 29.4 / 100 | 29.2 / 100 | 26.9 / 100 | 25.3 / 100 | 21.4 / 100 |
| S2 QPSK 3/4 66k | 21.1 / 100 | 30.3 / 100 | 30.9 / 100 | 28.6 / 100 | 27.5 / 100 | 22.6 / 95.0 |
| S2 QPSK 3/4 125k | 17.8 / 100 | 29.1 / 100 | 30.2 / 100 | 28.2 / 100 | 27.7 / 100 | 20.8 / 100 |
| S2 QPSK 3/4 250k | 18.8 / 100 | 28.5 / 100 | 29.5 / 100 | 27.1 / 100 | 27.2 / 100 | 18.6 / 100 |
| S2 QPSK 3/4 333k | 15.3 / 95.0 | 29.4 / 100 | 30.9 / 100 | 27.8 / 100 | 27.9 / 100 | 17.9 / 100 |
| S2 QPSK 3/4 500k | 17.9 / 94.7 | 29.1 / 100 | 32.5 / 100 | 28.0 / 100 | 27.9 / 100 | 17.2 / 100 |
| S2 8PSK 3/4 33k | 21.1 / 100 | 29.0 / 100 | 29.4 / 100 | 26.5 / 100 | 25.5 / 100 | 21.1 / 100 |
| S2 8PSK 3/4 66k | 19.3 / 100 | 29.8 / 100 | 30.9 / 100 | 27.8 / 100 | 26.9 / 100 | 22.3 / 100 |
| S2 8PSK 3/4 125k | 17.6 / 100 | 27.9 / 100 | 30.2 / 100 | 27.5 / 100 | 27.4 / 100 | 20.6 / 100 |
| S2 8PSK 3/4 250k | 16.8 / 99.6 | 28.3 / 100 | 29.4 / 100 | 26.7 / 100 | 27.2 / 100 | 18.0 / 100 |
| S2 8PSK 3/4 333k | 15.2 / 92.2 | 28.8 / 100 | 30.8 / 100 | 27.2 / 100 | 28.0 / 100 | 18.0 / 100 |
| S2 8PSK 3/4 500k | 16.1 / 92.2 | 29.8 / 100 | 32.2 / 100 | 27.6 / 100 | 28.2 / 100 | 17.1 / 100 |
| T2 1.7 QPSK 1/2 | 4.9 / 98.1 | 4.3 / 100 | 26.8 / 100 | 27.6 / 100 | 21.2 / 100 | 4.8 / 75.1 |
| T2 1.7 QPSK 3/4 | 6.0 / 86.1 | 2.9 / 85.3 | 27.6 / 100 | 27.7 / 100 | 21.1 / 100 | 1.1 / 24.3 |
| T2 1.7 16QAM 1/2 | 4.7 / 54.6 | 3.2 / 1.0 | 27.6 / 100 | 27.5 / 100 | 21.1 / 100 | 4.4 / 1.3 |
| T2 1.7 16QAM 3/4 | 0.2 / 0.9 | 2.7 / 0.8 | 27.2 / 100 | 27.9 / 100 | 21.4 / 100 | 4.1 / 0.5 |
| T2 2.0 QPSK 1/2 | 4.5 / 91.6 | 1.6 / 100 | 26.9 / 100 | 27.4 / 100 | 20.6 / 100 | 2.6 / 94.8 |
| T2 2.0 16QAM 1/2 | 5.8 / 51.1 | 2.0 / 0.9 | 26.8 / 100 | 27.5 / 100 | 20.3 / 100 | 3.1 / 0.5 |
| T2 1.35 QPSK 1/2 | 6.7 / 100 | 0.9 / 90.4 | 28.2 / 100 | 28.4 / 100 | 21.9 / 100 | 1.1 / 50.2 |

### Libre 2 -> Libre 1

MER dB (median) / % of frames decoded after lock.

| Mode | 2 m 145 | 70 cm 436 | 23 cm 1255 | 13 cm 2330 | 9 cm 3410 | 6 cm 5700 |
|---|---|---|---|---|---|---|
| S2 QPSK 1/2 33k | 22.2 / 100 | 29.6 / 100 | 29.5 / 100 | 26.9 / 100 | 24.9 / 92.3 | 22.3 / 100 |
| S2 QPSK 1/2 66k | 20.5 / 100 | 30.6 / 100 | 30.9 / 100 | 28.6 / 100 | 26.1 / 100 | 24.0 / 96.2 |
| S2 QPSK 1/2 125k | 17.1 / 100 | 29.6 / 100 | 30.2 / 100 | 28.3 / 100 | 25.3 / 100 | 22.5 / 100 |
| S2 QPSK 1/2 250k | 16.7 / 100 | 28.6 / 100 | 29.4 / 100 | 27.3 / 100 | 23.6 / 100 | 20.3 / 100 |
| S2 QPSK 1/2 333k | 13.8 / 99.5 | 29.6 / 100 | 31.0 / 100 | 27.9 / 100 | 23.2 / 100 | 19.9 / 100 |
| S2 QPSK 1/2 500k | 13.3 / 99.5 | 30.0 / 100 | 32.6 / 100 | 28.4 / 100 | 22.3 / 100 | 19.4 / 100 |
| S2 QPSK 3/4 33k | 22.1 / 100 | 29.4 / 100 | 29.5 / 100 | 27.1 / 100 | 24.7 / 100 | 22.2 / 100 |
| S2 QPSK 3/4 66k | 20.6 / 100 | 30.2 / 100 | 31.0 / 100 | 28.8 / 100 | 26.1 / 100 | 24.1 / 100 |
| S2 QPSK 3/4 125k | 17.3 / 100 | 29.1 / 100 | 30.3 / 100 | 28.5 / 100 | 25.3 / 100 | 22.8 / 100 |
| S2 QPSK 3/4 250k | 16.5 / 100 | 28.4 / 100 | 29.4 / 100 | 27.3 / 100 | 18.1 / 98.3 | 20.3 / 100 |
| S2 QPSK 3/4 333k | 13.8 / 99.5 | 29.4 / 100 | 31.0 / 100 | 27.9 / 100 | 18.1 / 100 | 19.9 / 100 |
| S2 QPSK 3/4 500k | 13.4 / 99.2 | 29.5 / 100 | 32.6 / 100 | 28.4 / 100 | 16.7 / 100 | 19.2 / 100 |
| S2 8PSK 3/4 33k | 22.1 / 100 | 29.2 / 100 | 29.2 / 100 | 26.5 / 100 | 23.4 / 100 | 21.6 / 100 |
| S2 8PSK 3/4 66k | 20.5 / 100 | 30.0 / 100 | 30.6 / 100 | 28.0 / 100 | 22.8 / 100 | 24.2 / 98.3 |
| S2 8PSK 3/4 125k | 18.0 / 100 | 29.2 / 100 | 30.1 / 100 | 27.7 / 100 | 21.2 / 100 | 22.6 / 100 |
| S2 8PSK 3/4 250k | 17.3 / 98.9 | 28.3 / 100 | 29.4 / 100 | 26.8 / 100 | 18.6 / 94.0 | 20.1 / 100 |
| S2 8PSK 3/4 333k | 14.6 / 96.0 | 29.3 / 100 | 30.8 / 100 | 27.4 / 100 | 17.9 / 100 | 19.7 / 100 |
| S2 8PSK 3/4 500k | 15.1 / 97.2 | 29.9 / 100 | 32.3 / 100 | 27.8 / 100 | 16.5 / 100 | 19.2 / 100 |
| T2 1.7 QPSK 1/2 | 5.0 / 99.2 | -10.8 / 0.8 | 29.3 / 100 | 27.8 / 100 | 1.0 / 74.9 | 14.5 / 100 |
| T2 1.7 QPSK 3/4 | 5.6 / 94.9 | -11.2 / 0.4 | 29.0 / 100 | 28.1 / 100 | 0.3 / 17.2 | 14.1 / 100 |
| T2 1.7 16QAM 1/2 | 4.8 / 24.7 | -13.8 / 1.1 | 29.2 / 100 | 28.1 / 100 | 2.8 / 0.8 | 14.3 / 100 |
| T2 1.7 16QAM 3/4 | 4.1 / 0.6 | -14.4 / 0.7 | 29.4 / 100 | 28.2 / 100 | 2.1 / 0.9 | 14.6 / 100 |
| T2 2.0 QPSK 1/2 | 5.1 / 100 | -22.4 / 0.5 | 28.3 / 100 | 28.0 / 100 | 1.9 / 66.8 | 13.8 / 100 |
| T2 2.0 16QAM 1/2 | 5.6 / 61.5 | -22.4 / 0.8 | 28.0 / 100 | 27.8 / 100 | 1.4 / 0.5 | 14.1 / 100 |
| T2 1.35 QPSK 1/2 | 4.7 / 99.2 | -3.9 / 0.5 | 29.9 / 100 | 28.7 / 100 | 1.7 / 94.4 | 14.8 / 100 |
