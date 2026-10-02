# Firmware flavours

One source tree builds two flavours of the firmware for three kinds of
board, five images in all.

| | LibreSDR | PlutoSky R2 | ADALM-Pluto |
|---|---|---|---|
| FPGA | xc7z020 | xc7z020 | xc7z010 |
| **BASIC** | `libre-basic` | `plutoskyr2-basic` | `pluto-basic` |
| **BASIC+** | `libre-plus` | `plutoskyr2-plus` | not available |

Images: `build/<board>-<flavour>.zip`, e.g. `build/libre-plus.zip`.

## BASIC

SQTRX as a narrowband transceiver, nothing else:

* SSB, CW, data modes (PKTUSB), AM, FM; TCI and Hamlib rigctld for
  WSJT-X, JTDX, MSHV and others; the web UI with microphone and speaker
  in the browser.
* Decoders on the board: Q65 and PI4 slot decoders, the live CW box
  (timing decoder; CW-RS rain-scatter decoder; DeepCW where the model is
  installed).
* Receive level in dBm, the same in every mode and filter, with the
  per-board calibration (docs/DBM.md).
* Transverters, antenna socket mapping, GPS / reference disciplining
  where the board has the inputs (docs/REFERENCE.md), beacon transmitter
  and beacon receiver roles.
* One FPGA bitstream (the transceiver one); no DATV.

## BASIC+

Everything in BASIC, plus amateur television (docs/DATV.md):

* DVB-S2 transmit and receive (long frames with pilots: QPSK 1/2,
  QPSK 3/4, 8PSK 3/4 at 33 to 500 kS/s, automatic receive), and DVB-T2
  (1.7 MHz standard profile; 2.0 and 1.35 MHz non-standard), from and to
  the browser's camera and microphone (H.264 video, Opus audio).
* The receiver runs in the FPGA (DDC, timing, frame handling, LDPC):
  the ARM stays free for the rest.
* A second FPGA bitstream for DATV: the board switches to it when DATV
  mode is selected and back afterwards (docs/FPGA-MODES.md), a few
  seconds each way.

BASIC+ needs the larger xc7z020: the DATV receiver's LDPC decoder alone
takes most of the xc7z010's block RAM, so there is no BASIC+ for the
ADALM-Pluto.

## Which one

* Narrowband operation (SSB/CW/digital, beacons): **BASIC**. Smaller,
  one bitstream, nothing switching underneath.
* DATV as well: **BASIC+** (LibreSDR or PlutoSky R2).

A board can change flavour with a normal update (`tools/fw-push.sh`,
docs/FLASH.md): the updater checks that the image is for this board and
says when the flavour changes. Settings (callsign, transverters, sockets,
calibration) are kept.

## Board notes

* **LibreSDR**: Ethernet; two firmware slots in the QSPI flash (A/B
  update with automatic rollback, docs/FLASH.md); 10 MHz / GPS
  reference.
* **PlutoSky R2**: Ethernet; A/B slots; ADF4001 reference and GPS PPS
  on EXT_IO0.
* **ADALM-Pluto**: USB networking only (the board appears as a USB
  network adapter; a USB Ethernet adapter on its OTG port also works);
  AD9363 run in AD9364 mode for 70 MHz to 6 GHz (outside 325 MHz to
  3.8 GHz the chip is out of its specification); single Cortex-A9 core;
  no reference or PPS input (the bitstream has no refmeter: reference
  disciplining is off). Its 32 MB flash takes the same layout as the
  other boards: two firmware slots (A/B update with rollback) and the CW
  model; the last resort is the Pluto's own DFU mode (docs/FLASH.md,
  "ADALM-Pluto").

Building, flashing, changing flavour and recovering a Pluto: docs/FLASH.md,
"Images and flavours".
