"""Tests against mock instruments and a simulated board:
    python -m unittest discover -s tests   (from tools/sqtrx-cal)
"""

import io
import json
import os
import re
import socketserver
import sys
import tempfile
import threading
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sqtrx_cal import sweep  # noqa: E402
from sqtrx_cal.__main__ import main  # noqa: E402
from sqtrx_cal.instruments import HpGenerator, Instrument, InstrumentError, Siglent, ScpiSocket  # noqa: E402
from sqtrx_cal.sim import SimAnalyser, SimBoard, true_err, true_k  # noqa: E402


def loss(f):
    """The test path: 35 dB pad, cable loss rising with frequency."""
    return 35.0 + 1.5 * f / 1e9


class MockSiglent(socketserver.StreamRequestHandler):
    """Just enough of an SVA1032X: TG on/off, level, centre; the marker reads
    the TG level less the path loss (normalisation: the path ends at its input)."""

    state = {}

    def handle(self):
        st = self.state
        for line in self.rfile:
            cmd = line.decode().strip()
            st.setdefault("log", []).append(cmd)
            if cmd == "*IDN?":
                self.wfile.write(b"Siglent Technologies,SVA1032X,SVA1XEXX0000,1.3.0\n")
            elif cmd == ":SYSTem:ERRor?":
                self.wfile.write(b'0,"No error"\n')
            elif cmd.startswith(":FREQuency:CENTer"):
                st["f"] = float(re.findall(r"[-\d.]+", cmd)[0])
            elif cmd.startswith(":SOURce:POWer"):
                st["lv"] = float(re.findall(r"[-\d.]+", cmd)[0])
            elif cmd == ":OUTPut:STATe ON":
                st["on"] = True
            elif cmd == ":OUTPut:STATe OFF":
                st["on"] = False
            elif cmd == ":CALCulate:MARKer1:Y?":
                v = st["lv"] - loss(st["f"]) if st.get("on") else -120.0
                self.wfile.write(f"{v:.3f}\n".encode())


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def quiet(*_):
    pass


class FakeGpib(Instrument):
    def __init__(self):
        super().__init__("HP", False, quiet)
        self.sent = []

    def _write(self, cmd):
        self.sent.append(cmd)

    def _query(self, cmd):
        return "HP8657B"


class Sweeps(unittest.TestCase):
    def setUp(self):
        MockSiglent.state = {}
        self.srv = Server(("127.0.0.1", 0), MockSiglent)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.addr = self.srv.server_address

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def siglent(self):
        s = Siglent(ScpiSocket("Siglent", *self.addr, log=quiet))
        s.measure_settle = 0
        return s

    def check_tables(self, tables, max_hz, ports=(1, 2), tol=0.05, bands=None):
        for port in ports:
            t = tables[port]
            self.assertEqual(len(t["k"]), len(sweep.plan(max_hz, bands)))
            for p in t["k"]:
                self.assertLessEqual(p["f"], max_hz + 1)
                # K at the reference gain: the board's K less its gain error there (0 by design).
                self.assertAlmostEqual(p["k"], true_k(p["f"], port) - true_err(40.0, p["f"]), delta=tol, msg=f"{port} {p['f']}")
            # Gain tables: the error relative to the reference gain.
            for g in t["gain"]:
                f = 0.5 * (g["f_min"] + g["f_max"]) if g["f_max"] < 7e9 else g["f_min"] * 1.1
                for gain, e in g["pts"]:
                    fm = next(f for f in sweep.gain_plan(max_hz, bands) if g["f_min"] <= f <= g["f_max"])
                    want = true_err(gain, fm) - true_err(40.0, fm)
                    self.assertAlmostEqual(e, want, delta=tol, msg=f"{gain} at {fm}")
            self.assertAlmostEqual(t["stream_db"], -0.7, delta=0.01)
            self.assertAlmostEqual(t["maia_db"], 3.0, delta=0.3)

    def test_siglent_run_recovers_the_board(self):
        import time as _t
        sg = self.siglent()
        sg.measure = lambda settle_s=0, reads=1: Siglent.measure(sg, settle_s=0, reads=1)
        board = SimBoard(sg, path_loss=loss)
        prompts = []

        def ask(msg):
            prompts.append(msg)
            # Moving the cable: the path ends at RX1, then RX2.
            m = re.search(r"RX(\d)", msg)
            if m:
                board.connected_port = int(m.group(1))

        run = sweep.Run(board, sg, "siglent", analyser=sg, ask=ask, log=quiet, gain_step=10.0)
        t0 = _t.monotonic()
        tables = run.run([1, 2], 3.2e9)
        self.assertLess(_t.monotonic() - t0, 60)
        self.assertEqual(len(prompts), 3)  # path to the analyser, RX1, RX2
        self.check_tables(tables, 3.2e9)
        self.assertIn(":OUTPut:STATe OFF", MockSiglent.state["log"])
        # Every K point is valid for trxd's validation ranges.
        for p in tables[1]["k"]:
            self.assertTrue(-100 < p["k"] < 100 and p["src"] == "siglent" and p["date"])

    def test_hp_run_with_constant_cable_loss(self):
        gp = FakeGpib()
        hp = HpGenerator(gp, "8642b")
        board = SimBoard(hp, path_loss=lambda f: 1.0)
        run = sweep.Run(board, hp, "hp", analyser=None, ask=quiet, log=quiet, cable_loss_db=1.0,
                        levels=[-10.0 - 10 * i for i in range(12)], gain_step=10.0, bands=["2m", "70cm", "23cm"])
        tables = run.run([1], 2.1e9)
        self.check_tables(tables, 2.1e9, ports=(1,), bands=["2m", "70cm", "23cm"])
        self.assertTrue(any(c.startswith("FR 145.000000 MZ") for c in gp.sent))
        self.assertTrue(any(c.startswith("AP ") and c.endswith(" DM") for c in gp.sent))
        # 8642B: off is the minimum level; never a level above it while the
        # frequency moves.
        self.assertEqual(gp.sent[-1], "AP -140.0 DM")
        for i, c in enumerate(gp.sent):
            if c.startswith("FR ") and i > 0:
                self.assertEqual(gp.sent[i - 1], "AP -140.0 DM", gp.sent[i - 3:i + 1])
        self.assertFalse(any(c in ("R2", "R3", "RO", "RF") for c in gp.sent))

    def test_hp_refuses_beyond_its_range(self):
        hp = HpGenerator(FakeGpib(), "8642b")
        with self.assertRaises(InstrumentError):
            hp.source(2.4e9, -60)
        with self.assertRaises(InstrumentError):
            hp.source(145e6, -141)
        hp.source(145e6, -140)
        # Real on/off codes, once checked, replace the defaults.
        gp = FakeGpib()
        hp = HpGenerator(gp, "8642b", on="R3", off="R2")
        hp.source(145e6, -60)
        hp.off()
        self.assertEqual(gp.sent, ["FR 145.000000 MZ", "AP -60.0 DM", "R3", "R2"])

    def test_merge_keeps_old_points_elsewhere(self):
        old = {"k": [{"f": 145e6, "k": 1.0}, {"f": 2400e6, "k": 2.0}], "gain": [{"f_min": 1e6, "f_max": 7e9, "pts": [[0, 0]]}], "note": "a"}
        new = {"k": [{"f": 145.5e6, "k": 1.5}], "gain": [], "stream_db": 0.0, "maia_db": 0.0, "note": "b"}
        m = sweep.merge(old, new)
        self.assertEqual([p["f"] for p in m["k"]], [145.5e6, 2400e6])
        self.assertEqual(len(m["gain"]), 1)
        self.assertEqual(sweep.merge(old, new, replace=True), new)

    def test_plan(self):
        f = sweep.plan(3.2e9)
        self.assertTrue(all(x <= 3.2e9 for x in f))
        self.assertIn(145e6, f)
        self.assertNotIn(3410e6, f)
        self.assertEqual(sweep.gain_plan(2.1e9), [52e6, 145e6, 435e6, 1270e6])
        r = sweep.gain_ranges([52e6, 145e6])
        self.assertEqual(r, [(1e6, 98.5e6), (98.5e6, 7e9)])

    def test_cli_dry_run_and_check(self):
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as d, redirect_stdout(out):
            save = os.path.join(d, "t.json")
            rc = main(["run", "--source", "siglent", "--siglent", "192.0.2.1", "--dry-run", "--yes",
                       "--ports", "1", "--bands", "2m", "--save", save])
            self.assertEqual(rc, 0, out.getvalue())
            t = json.load(open(save))
            self.assertEqual(len(t["k"]), 3)
        text = out.getvalue()
        self.assertIn("DRY RUN", text)
        self.assertIn("[Siglent] > :OUTPut:STATe ON", text)
        out = io.StringIO()
        with redirect_stdout(out):
            rc = main(["check", "--siglent", f"{self.addr[0]}:{self.addr[1]}"])
        self.assertEqual(rc, 0, out.getvalue())
        self.assertIn("SVA1032X", out.getvalue())


if __name__ == "__main__":
    unittest.main()
