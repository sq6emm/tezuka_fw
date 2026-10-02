# sqtrx-cal: receive level calibration for SQTRX

Measures a board's conversion from its level meter to dBm at the antenna
socket, for each RX socket pair (RX1, RX2), and uploads the table to the
board (it keeps it in jffs2, across firmware updates). After that the
S-meter, the dBm readout and the scope scale show dBm, the same in every
mode, filter, bandwidth and AGC state. Design: docs/DBM.md.

References:

* **Siglent SVA1032X** over the LAN (SCPI, port 5025): its tracking
  generator is the carrier (up to 3.2 GHz), its analyser input measures the
  path first.
* **HP 8642B signal generator** (HP-IB; 0.1-2115 MHz, -140 to +16 dBm)
  through the xyphro **UsbGpib v2** adapter (USBTMC). Its level is trusted;
  the cable loss is either given (`--cable-loss`) or measured with the
  Siglent.

Above the highest measured frequency (3.2 GHz) the board extrapolates from
the highest points and says so (SET and the meter: `dBm*`).

## Install (on the notebook)

Python 3.9 or later.

Linux:

    cd tools/sqtrx-cal
    python3 -m venv .venv && . .venv/bin/activate
    pip install -r requirements.txt
    # UsbGpib v2: either the kernel's usbtmc driver (--hp usbtmc:/dev/usbtmc0;
    # add yourself to the group owning /dev/usbtmc*), or libusb through
    # pyvisa-py (--hp USB0::0x03EB::0x2065::<serial>::INSTR; a udev rule
    # giving you access to the device, and `sudo rmmod usbtmc` if the kernel
    # driver holds it). `python -m pyvisa info` and
    # `python -c "import pyvisa; print(pyvisa.ResourceManager('@py').list_resources())"`
    # show what is found.

Windows:

    cd tools\sqtrx-cal
    py -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt
    # UsbGpib v2: with NI-VISA installed it is a USB0::...::INSTR resource
    # (NI MAX lists it). Without NI-VISA: pyvisa-py needs libusb (install
    # libusb-1.0.dll, e.g. with Zadig give the adapter the WinUSB driver).

## Check first

Identifies the board and the instruments and changes nothing (the 8642B has
no ID query: the check sets its output to the minimum level, so look at its
display to see it took the command):

    python -m sqtrx_cal check --board 192.168.12.162 --siglent 192.168.12.196 --hp usbtmc:/dev/usbtmc0
    python -m sqtrx_cal run --source hp --hp usbtmc:/dev/usbtmc0 --cable-loss 1 --dry-run --yes   # 8642B codes

A dry run prints every instrument command a run would send, against a
simulated board (no instrument or board is touched):

    python -m sqtrx_cal run --source siglent --siglent 192.168.12.196 --dry-run --yes

### HP 8642B codes

`--hp-model 8642b` is the default. What the tool sends, and how sure it is:

| Function | Code sent | Sure? |
|---|---|---|
| Frequency | `FR 145.000000 MZ` | Yes: the HP-IB frequency code and MHz unit of this generator family (8642A/B, 8656B, 8657A/B) |
| Amplitude | `AP -60.0 DM` (dBm, 0.1 dB) | Yes, as above |
| Level range | -140.0 to +16.0 dBm; anything outside is refused by the tool | Yes (8642B spec); the runs use -10 dBm down in 10 dB steps |
| RF off | `AP -140.0 DM` (minimum level) | Not a real RF-off code: the 8642B's RF on/off code is not known for sure, so the tool never sends one |
| RF on | nothing (the next `AP` sets the level) | as above |
| Identify | none (no ID query before IEEE 488.2) | Yes |

Each code goes as its own line, LF-terminated. Before every frequency
change the level goes to -140 dBm first, so the board never sees a high
level at a passing frequency.

If your manual gives the RF on/off codes, pass them:
`--hp-on <code> --hp-off <code>`. Check what is sent with `--dry-run`
first. Other command sets: `8642a` (to 1057 MHz), `8657`, `8656`
(`R3`/`R2` for RF on/off), `scpi` (8648 and later).

The tool's reachable range with the 8642B is -140 dBm. Your note of
-150 dBm may count the vernier or an external pad: with a pad, add its loss
to `--cable-loss`.

## Run

Siglent (both sockets, all bands to 3.2 GHz and a general grid, gain sweeps
at one frequency per band):

    python -m sqtrx_cal run --board 192.168.12.162 --source siglent --siglent 192.168.12.196

1. Put a 30-40 dB attenuator at the end of the tracking generator cable
   (the board must not see more than about -30 dBm, and the gain sweep
   needs low levels at high gain).
2. Prompted: connect the cable end (with the attenuator) to the Siglent's
   RF input. The path is measured at every frequency and level.
3. Prompted: connect it to the board's RX1 input. Then RX2.

HP (to 2.1 GHz), cable loss measured with the Siglent at the start:

    python -m sqtrx_cal run --board 192.168.12.162 --source hp --hp usbtmc:/dev/usbtmc0 --siglent 192.168.12.196

or with a known cable loss: `--cable-loss 1.2`.

Options: `--ports 1` (one socket pair), `--bands 2m,70cm`, `--gain-step 3`,
`--no-gain`, `--replace` (start a new table instead of merging into the
board's), `--save FILE`, `--no-upload`, `--yes` (no prompts), `--password`
(asked otherwise), `--fingerprint` (pin the board's certificate).

The board's web password is asked for. During a run the board is tuned,
switched to manual gain and between socket pairs; afterwards it goes back
to AGC, its frequency and socket pair. A band mapped to a socket pair in
SET > ANTENNA SOCKETS would switch pairs behind the tool's back: set those
to "manual" for the run. No transverter may be active.

A run takes about 15 minutes per board with the Siglent.

## Other commands

    python -m sqtrx_cal show   --board HOST            # tables and per-band status
    python -m sqtrx_cal upload --board HOST --port 1 calib-....json
    python -m sqtrx_cal clear  --board HOST --port 1

## Tests

    python -m unittest discover -s tests

(mock Siglent over TCP, mock GPIB, simulated board; no hardware).
