"""sqtrx-cal: level calibration of an SQTRX board (LibreSDR, PlutoSky R2).

    python -m sqtrx_cal check  --board HOST --password PW [--siglent HOST] [--hp RES]
    python -m sqtrx_cal run    --board HOST --password PW --source siglent|hp [...]
    python -m sqtrx_cal show   --board HOST --password PW
    python -m sqtrx_cal upload --board HOST --password PW --port 1 FILE.json
    python -m sqtrx_cal clear  --board HOST --password PW --port 1

See README.md.
"""

import argparse
import datetime
import json
import sys

from . import sweep
from .board import Board, BoardError
from .instruments import InstrumentError, open_hp, open_siglent

VERSION = "1.0"


def log(msg):
    print(msg, flush=True)


def ask_user(yes):
    def ask(msg):
        if yes:
            log(f">>> {msg} (--yes: going on)")
            return
        input(f">>> {msg} ")
    return ask


def board_of(a):
    if not a.board:
        sys.exit("--board HOST is needed")
    if a.password is None:
        import getpass
        a.password = getpass.getpass(f"web password of {a.board}: ")
    return Board(a.board, a.password, a.fingerprint, log=log).connect()


def cmd_check(a):
    ok = True
    if a.board:
        try:
            b = board_of(a)
            st = b.state or {}
            raw = b.raw()
            c = b.calib_get()
            log(f"board {a.board}: {st.get('call', '')} at {raw['dial_hz'] / 1e6:.4f} MHz, RX{raw['port']}, "
                f"gain {raw['hw_gain_db']} dB, level meter source "
                f"{'channel' if raw.get('chan') else 'stream' if raw.get('stream') else 'none'}; tables: "
                + ", ".join(f"RX{p['port']} {'yes' if p.get('table') else 'none'}" for p in c["ports"]))
            b.close()
        except (BoardError, OSError) as e:
            ok = False
            log(f"board {a.board}: {e}")
    if a.siglent:
        try:
            s = open_siglent(a.siglent, dry_run=a.dry_run, log=log)
            log(f"Siglent {a.siglent}: {s.identify()}")
            s.setup()
            s.off()
            log(f"  error queue: {s.errors()}")
            s.i.close()
        except (InstrumentError, OSError) as e:
            ok = False
            log(f"Siglent {a.siglent}: {e}")
    if a.hp:
        try:
            h = open_hp(a.hp, a.hp_model, dry_run=a.dry_run, log=log, on=a.hp_on, off=a.hp_off)
            log(f"HP {a.hp}: {h.identify()}")
            h.i.close()
        except (InstrumentError, OSError, ImportError) as e:
            ok = False
            log(f"HP {a.hp}: {e}")
    return 0 if ok else 1


def cmd_run(a):
    ports = [int(p) for p in a.ports.split(",")]
    bands = a.bands.split(",") if a.bands else None
    if a.dry_run:
        log("DRY RUN: instrument commands are printed, not sent; the board is simulated (no connection).")
    # Instruments.
    sa = None
    if a.source == "siglent":
        if not a.siglent:
            sys.exit("--siglent HOST is needed")
        src = open_siglent(a.siglent, dry_run=a.dry_run, log=log, rbw_hz=a.rbw)
        log(f"Siglent: {src.identify()}")
        src.setup()
        sa = src
        max_hz = min(a.max_mhz * 1e6, src.max_hz)
        levels = [float(x) for x in a.levels.split(",")]
    else:
        if not a.hp:
            sys.exit("--hp RESOURCE is needed (usbtmc:/dev/usbtmc0 or a VISA resource)")
        src = open_hp(a.hp, a.hp_model, dry_run=a.dry_run, log=log, max_hz=a.hp_max_mhz * 1e6, on=a.hp_on, off=a.hp_off)
        log(f"HP: {src.identify()}")
        src.setup()
        if a.siglent and not a.cable_loss:
            sa = open_siglent(a.siglent, dry_run=a.dry_run, log=log, rbw_hz=a.rbw)
            log(f"Siglent (cable loss): {sa.identify()}")
            sa.setup()
        max_hz = min(a.max_mhz * 1e6, src.max_hz)
        levels = [lv for lv in (a.hp_max_level - 10 * i for i in range(16)) if lv >= src.min_level]
    # Board.
    if a.dry_run:
        from .sim import SimAnalyser, SimBoard
        board = SimBoard(src, log=log)
        if sa is not None:
            sa = SimAnalyser(src, board.loss)
        old = {}
    else:
        board = board_of(a)
        old = {p["port"]: p.get("table") for p in board.calib_get()["ports"]}
        start = board.raw()
    run = sweep.Run(board, src, a.source, analyser=sa, ask=ask_user(a.yes), log=log,
                    levels=levels, cable_loss_db=a.cable_loss or 0.0, gain_step=a.gain_step,
                    gains=not a.no_gain, bands=bands)
    try:
        tables = run.run(ports, max_hz)
    finally:
        src.off()
        if not a.dry_run:
            board.gain_auto()
            board.tune(start["dial_hz"])
            board.port(start["port"])
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M")
    for port, t in tables.items():
        t["note"] = f"sqtrx-cal {VERSION} {a.source} {stamp}"
        final = sweep.merge(old.get(port), t, replace=a.replace)
        name = a.save or f"calib-{a.board or 'dry'}-rx{port}-{stamp}.json"
        if len(tables) > 1 and a.save:
            name = a.save.replace(".json", f"-rx{port}.json")
        with open(name, "w") as f:
            json.dump(final, f, indent=1)
        log(f"RX{port}: {len(final['k'])} K points, {len(final['gain'])} gain tables, stream {final['stream_db']:+.2f} dB, "
            f"spectrometer {final['maia_db']:+.2f} dB -> {name}")
        if not a.dry_run and not a.no_upload:
            board.calib_set(port, final)
            log(f"RX{port}: uploaded to {a.board}")
    if not a.dry_run:
        board.close()
    return 0


def cmd_show(a):
    b = board_of(a)
    c = b.calib_get()
    for p in c["ports"]:
        log(f"RX{p['port']}: " + ("no table" if not p.get("table") else p["table"].get("note", "")))
        for s in p.get("status") or []:
            log(f"  {s['band']:5} {s['status']:12} points {s['points']:2}  {s['date']} {s['src']}  gain table: {'yes' if s['gain_table'] else 'no'}")
    b.close()
    return 0


def cmd_upload(a):
    with open(a.file) as f:
        t = json.load(f)
    b = board_of(a)
    b.calib_set(a.port, t)
    log(f"RX{a.port}: {a.file} uploaded")
    b.close()
    return 0


def cmd_clear(a):
    b = board_of(a)
    b.send(cmd="calib_clear", port=a.port)
    ack = b.wait("calib_ack")
    log(f"RX{a.port}: {'cleared' if ack.get('ok') else ack.get('error')}")
    b.close()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="sqtrx_cal", description="SQTRX receive level calibration")
    p.add_argument("--version", action="version", version=VERSION)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--board", help="the board's address (host or host:port)")
        sp.add_argument("--password", help="its web password (asked when left out)")
        sp.add_argument("--fingerprint", help="SHA-256 of the board's certificate to pin (optional)")

    def instruments(sp):
        sp.add_argument("--siglent", help="Siglent SVA1032X address (host or host:port, SCPI 5025)")
        sp.add_argument("--hp", help="HP generator: usbtmc:/dev/usbtmc0, or a VISA resource (visa:USB0::...::INSTR)")
        sp.add_argument("--hp-model", default="8642b", help="HP command set: 8642b (default), 8642a, 8657, 8656, scpi")
        sp.add_argument("--hp-on", help="HP-IB code for RF on (default: none for the 8642: the level is set instead)")
        sp.add_argument("--hp-off", help="HP-IB code for RF off (default for the 8642: AP -140.0 DM)")
        sp.add_argument("--rbw", type=float, default=1000.0, help="Siglent RBW, Hz")
        sp.add_argument("--dry-run", action="store_true", help="print instrument commands instead of sending them")

    sp = sub.add_parser("check", help="identify the board and the instruments, change nothing")
    common(sp)
    instruments(sp)
    sp.set_defaults(fn=cmd_check)

    sp = sub.add_parser("run", help="measure and upload the calibration")
    common(sp)
    instruments(sp)
    sp.add_argument("--source", choices=["siglent", "hp"], required=True, help="the reference signal")
    sp.add_argument("--ports", default="1,2", help="RX socket pairs to calibrate (default 1,2)")
    sp.add_argument("--bands", help="only these bands (6m,4m,2m,70cm,23cm,13cm,9cm,6cm); default all plus a general grid")
    sp.add_argument("--max-mhz", type=float, default=3200.0, help="highest frequency (the source's own limit applies too)")
    sp.add_argument("--levels", default="0,-10,-20,-30,-40", help="Siglent TG levels to use, dBm")
    sp.add_argument("--hp-max-level", type=float, default=-10.0, help="highest HP level used, dBm (then 10 dB steps down)")
    sp.add_argument("--hp-max-mhz", type=float, default=2100.0, help="the HP generator's top frequency, MHz")
    sp.add_argument("--cable-loss", type=float, help="HP: constant cable loss, dB (else measured with --siglent)")
    sp.add_argument("--gain-step", type=float, default=5.0, help="gain sweep step, dB")
    sp.add_argument("--no-gain", action="store_true", help="skip the gain sweeps")
    sp.add_argument("--replace", action="store_true", help="replace the board's table instead of merging into it")
    sp.add_argument("--save", help="where to save the table (JSON); default calib-<board>-rx<N>-<time>.json")
    sp.add_argument("--no-upload", action="store_true", help="save the table only")
    sp.add_argument("--yes", action="store_true", help="do not wait for Enter at the cable prompts")
    sp.set_defaults(fn=cmd_run)

    sp = sub.add_parser("show", help="the board's tables and band status")
    common(sp)
    sp.set_defaults(fn=cmd_show)

    sp = sub.add_parser("upload", help="upload a saved table")
    common(sp)
    sp.add_argument("--port", type=int, required=True, choices=[1, 2])
    sp.add_argument("file")
    sp.set_defaults(fn=cmd_upload)

    sp = sub.add_parser("clear", help="remove a socket pair's table from the board")
    common(sp)
    sp.add_argument("--port", type=int, required=True, choices=[1, 2])
    sp.set_defaults(fn=cmd_clear)

    a = p.parse_args(argv)
    try:
        return a.fn(a)
    except (BoardError, InstrumentError, RuntimeError) as e:
        log(f"error: {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
