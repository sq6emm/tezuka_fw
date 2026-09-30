# FPGA bitstream per mode

One bitstream with everything filled the xc7z020 (block RAM 139.5 of 140
tiles, LUTs 75 %, DSPs 89 %): nothing more could go in. Now each mode has
its own bitstream, and trxd loads the one a feature needs while Linux runs.

| Mode | What is in it | LUTs | BRAM tiles | DSPs |
|---|---|---|---|---|
| `all` | everything (the single bitstream as before) | 75 % | 139.5 | 195 |
| `trx` | radio, wide scope, CW-RS network (front end and temporal layers, weights in DDR over HP0) | 34 % | 86 | 101 |
| `datv` | radio, wide scope, DVB-S2/T2 receive and transmit, LDPC | 71 % | 124.5 | 191 |

"Radio" is the AD936x interface, the DMAs, the x8 decimator/interpolator,
refmeter and on Libre `vctcxo_lock` and the XO corrector: in every mode, at
the same addresses, so the device tree is the same for all.

## How a switch works

1. A web command needs a part the loaded bitstream lacks (`datv_mode`,
   `datv_rx`, `datv` on: DATV; `cw_engine` rs: the network front end).
   trxd (`src/fpgamode.rs`) saves the VFOs, split, mode, filter and the
   command in `/run/trxd-resume.json`, writes the mode it wants to
   `/run/fpga-mode.want`, tells the browser, and exits.
2. The S80trxd loop runs `fpga-mode <mode>` (board overlay, /usr/sbin):
   - on LibreSDR the GPSDO script is stopped through `S22gpsdo stop` (it
     polls vctcxo_lock; the reload resets that core anyway);
   - the drivers of the PL devices are unbound (cf_axi_adc, cf_axi_dds,
     the ad9361 phy, both axi-dmac, the Maia uio);
   - `/lib/firmware/fpga-<mode>.bin` goes to
     `/sys/class/fpga_manager/fpga0/firmware` (full reconfiguration);
   - the drivers are bound again in probe order (the ad9361 phy
     initializes the chip and tunes the digital interface into the new
     fabric; the IIO device numbers come back the same) and `S22gpsdo
     start` re-acquires as at boot.
   About 2.8 s. `/run/fpga-mode` holds the mode loaded.
3. trxd starts again (no 5 s pause), takes the saved state and replays the
   command. Browser sessions live in `/run/trxd-sessions`, so the page
   reconnects without a new login. From the click to the feature running:
   about 4 s (the DeepCW model now loads in the background).

While it switches every open page shows a banner ("Loading the FPGA image
for DATV (DVB-S2/T2): the radio restarts, back in a few seconds", with the
seconds counting), turning to "loaded" once trxd is back (web message
`fpga_switch`). A setting changed while trxd restarts (a DATV code rate
picked right after entering DATV mode) never reaches it: once the switch
is done the page compares the receiver trxd resumed with its own choice
and sends its settings again if they differ (`DATV.resync`). Before that
a DVB-T2 receive started from the voice bitstream could run with the
default DVB-S2 settings and never lock. START pressed while the board
restarts waits for it (up to 30 s) instead of the command being lost and
a "did not start" alert. The saved state carries the levels too (TX
attenuation, drive, RX gain mode, AGC): a TX attenuation set before
entering DATV mode used to be lost with the restart, and the first DATV
transmission went out 20 dB down (the receiver across the room saw MER
-6 dB, the board's own receiver beside it could not lock).

The vctcxo_lock registers (control, set point / manual DAC code, reference
source) are saved before the reload and written back after it: with the
reset values the GPSDO script's warm-start check (error 0: "already
locked") took the reset DAC code, 12 ppm off, and DVB-T2 between the boards
never found a P1 (DVB-S2 tolerated it).

Nothing touches the PL while it is reloaded: trxd is not running, the
drivers are unbound, the GPSDO script is stopped. A hang would stop the CPU
and the 5 s hardware watchdog would reset the board into its slot.

Partial reconfiguration was not used: the mode logic sits in the data paths
(DMA/DAC split and merge, the Maia DDC and recorder ring), so a common
partition interface would mean redesigning them.

## Image

`board/tezuka/<board>/bitstream/boot-mode` names the boot mode (Libre:
`trx`). U-Boot loads `simple-<boot>.xsa` from the FIT; every other
`simple-<mode>.xsa` goes into the rootfs as `/lib/firmware/fpga-<mode>.bin`;
the boot one fpga-mode takes out of the running slot's FIT the first time
it switches back (`trxd --fit-data /dev/mtdN /images/fpga@1 | zcat | trxd
--bit2bin`: the flash has no room for it twice)
(package/board-fpga; `board/tezuka/common/bit2bin.py`), and the boot mode's
name into `/etc/fpga-boot-mode`. Without a boot-mode file a board builds as
before (`simple.xsa`, no modes). The flash is full (two 12.25 MB slots and
the DeepCW model), so the mode bitstreams must fit in the slot: `trx`
compresses to 0.6 MB (the `all` one took 1.3 MB).

## Building

```bash
/data/claude/fwbuild/fpga-build.sh libre trx    # -> bitstream/simple-trx.xsa
/data/claude/fwbuild/fpga-build.sh libre datv   # -> bitstream/simple-datv.xsa
/data/claude/fwbuild/fpga-build.sh libre        # all -> bitstream/simple.xsa
```

maia-hdl: `FPGA_MODE` (projects/simple system_project.tcl) picks the parts
(system_bd.tcl, maia_scope.tcl); Maia cores `maia_iio_lite_trx`
(spectrometer only, platform 0xD6: trxd leaves its recorder alone) and
`maia_iio_lite_s2` (DATV without the T2 front end, `datv_t2 = False`, for
a later s2/t2 split).

## Checked (Libre 1, 2026-09-29)

- `all` reloaded over itself at runtime: DVB-S2 from Libre 2 decoded after
  it (QPSK 1/2 250 kS/s found automatically, 180/180 frames).
- `all` -> `trx` by hand: trxd runs on it (Maia platform 0xD6, network front
  end there).
- Automatic: on `trx`, opening DATV receive in the web UI made trxd switch
  to a DATV bitstream and restart; receive resumed and decoded (422/422
  frames, 533 pictures), the browser session kept.
- Both Libres flashed with it (boot `trx`): from boot, DATV receive ->
  `datv` (decoded), CW-RS -> back to `trx` (network front end in the
  FPGA), about 4 s each way. Image: FIT 12.49 of 12.85 MB (351 KB left;
  the flash has no other free space).
