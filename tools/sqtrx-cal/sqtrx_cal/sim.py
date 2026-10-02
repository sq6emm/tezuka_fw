"""A simulated board (and reference path) for the dry run and the tests.

It answers like trxd's web API: the channel level of the carrier the source
sends through the path into the selected socket, with the board's "true"
K(f) and gain error, plus noise and clipping.
"""

import math


def true_k(f, port=1):
    """A plausible board: K rises with frequency, RX2 1.5 dB worse."""
    return 8.0 + 4.0 * math.log10(f / 50e6) + (1.5 if port == 2 else 0.0)


def true_err(g, f):
    """The chip's gain error against its report: a few tenths, more at the ends."""
    return 0.02 * (g - 40.0) - 0.0006 * (g - 40.0) ** 2 + (0.3 if f > 2e9 else 0.0)


class SimBoard:
    def __init__(self, source, path_loss=lambda f: 30.0 + f / 1e9, noise_dbfs_hz=-150.0, log=None,
                 k=true_k, err=true_err):
        self.src = source
        self.loss = path_loss
        self.noise = noise_dbfs_hz
        self.log = log
        self.k, self.err = k, err
        self.dial = 145e6
        self.gain = 40.0
        self.port_n = 1
        self.connected_port = 1
        self.seq = 0
        self.tables = {}

    def _say(self, msg):
        if self.log:
            self.log(f"  [board] {msg}")

    def connect(self):
        return self

    def close(self):
        pass

    def tune(self, hz, mode="USB"):
        self._say(f"mode {mode}, freq {hz:.0f}")
        self.dial = hz

    def gain_manual(self, db):
        self._say(f"rxgain manual {db}")
        # The chip takes whole dB, up to 71.
        self.gain = float(min(71, max(0, round(db))))

    def gain_auto(self):
        self._say("rxgain auto")

    def port(self, n):
        self.port_n = n

    def _level(self, lo, hi):
        """Channel dBFS in [lo, hi]: the carrier if it is inside and reaches
        the selected socket, plus noise."""
        noise = 10 ** ((self.noise + 10 * math.log10(hi - lo)) / 10)
        p = noise
        out = self.src.out
        if out and lo <= out[0] <= hi and self.port_n == self.connected_port:
            f, lv = out
            dbm = lv - self.loss(f)
            dbfs = dbm + self.gain + self.err(self.gain, f) - self.k(f, self.port_n)
            p += 10 ** (dbfs / 10)
        return 10 * math.log10(p)

    def raw(self, lo_hz=None, hi_hz=None):
        self.seq += 1
        lo, hi = lo_hz, hi_hz
        db = self._level(lo, hi)
        chan = {"dbfs": db, "peak_dbfs": db, "noise_dbfs_hz": self.noise, "seq": self.seq} if hi - lo <= 40e3 else None
        maia = {"dbfs": db - 3.0, "peak_dbfs": db - 3.0, "noise_dbfs_hz": self.noise - 3.0, "seq": 0} if hi - lo > 300e3 else None
        return {"type": "meter_raw", "port": self.port_n, "dial_hz": self.dial, "hw_freq_hz": 0.5 * (lo + hi),
                "hw_gain_db": self.gain, "temp_c": 45.0, "clip": db > -1.0,
                "chan": chan, "stream": ({"dbfs": db + 0.7, "seq": self.seq} if hi - lo <= 300e3 else None), "maia": maia,
                "xvtr": None}

    def fresh_raw(self, lo_hz, hi_hz, **_):
        return self.raw(lo_hz, hi_hz)

    def calib_get(self):
        return {"type": "calib", "port": self.port_n,
                "ports": [{"port": p, "table": self.tables.get(p)} for p in (1, 2)]}

    def calib_set(self, port, table):
        self._say(f"calib_set RX{port}: {len(table['k'])} K points, {len(table['gain'])} gain tables")
        self.tables[port] = table
        return {"ok": True}


class SimAnalyser:
    """The analyser end of the path in the simulation: what the source sends,
    less the path loss."""

    def __init__(self, source, path_loss):
        self.src = source
        self.loss = path_loss

    def measure(self, **_):
        f, lv = self.src.out
        return lv - self.loss(f)
