"""The calibration procedure and its fitting (pure, testable).

Per RX socket pair the board gets a table (trxd calib.rs):

    dBm = dBFS - G - E(G, f) + K(f)

K(f) is measured at a reference gain G0 at every frequency of the plan:
the reference (Siglent tracking generator, or the HP generator) sends a
carrier of known level P into the socket, and K = P + G - dBFS. E(G, f) is
measured at one frequency per band by stepping the gain with the carrier
level adjusted to keep the converter in range: E = dBFS(G) - P - (dBFS(G0)
- P0) - (G - G0).

Known levels: the Siglent's tracking generator path (cable, attenuator)
is first measured into the Siglent's own input (normalisation), at every
frequency and level the run uses; then the same path goes to the board.
The HP generator's level is taken as set, less a cable loss (constant, or
measured with the Siglent).
"""

import datetime
import math

# The plan: amateur bands (edges and middle) and a general-coverage grid, MHz.
BAND_POINTS = {
    "6m": [50.2, 52.0, 53.8],
    "4m": [70.1, 70.4],
    "2m": [144.1, 145.0, 145.9],
    "70cm": [430.5, 435.0, 439.5],
    "23cm": [1240.5, 1270.0, 1299.5],
    "13cm": [2300.5, 2375.0, 2449.5],
    "9cm": [3401.0, 3410.0, 3474.0],
    "6cm": [5651.0, 5760.0, 5849.0],
}
GENERAL = [60.0, 100.0, 200.0, 300.0, 500.0, 700.0, 1000.0, 1500.0, 2000.0, 2700.0, 3000.0, 3200.0]
# One gain sweep per band (MHz).
GAIN_POINTS = {"6m": 52.0, "2m": 145.0, "70cm": 435.0, "23cm": 1270.0, "13cm": 2375.0, "9cm": 3410.0}

G0 = 40.0
# Keep the channel level here (dBFS): far above the noise, below clipping.
TARGET_DBFS = -30.0
DBFS_MAX = -12.0
MIN_SNR_DB = 25.0
# Measurement window around the carrier (Hz either side).
HALF_WINDOW = 500.0


def plan(max_hz, bands=None, general=True):
    """Frequencies (Hz) the run measures K at, up to `max_hz`."""
    f = []
    for b, pts in BAND_POINTS.items():
        if bands and b not in bands:
            continue
        f += [p * 1e6 for p in pts]
    if general and not bands:
        f += [p * 1e6 for p in GENERAL]
    return sorted({x for x in f if x <= max_hz + 1})


def gain_plan(max_hz, bands=None):
    return sorted(p * 1e6 for b, p in GAIN_POINTS.items() if p * 1e6 <= max_hz + 1 and (not bands or b in bands))


def gain_ranges(freqs):
    """Each gain sweep frequency's table covers up to halfway to the next."""
    out = []
    for i, f in enumerate(freqs):
        lo = 1e6 if i == 0 else 0.5 * (freqs[i - 1] + f)
        hi = 7e9 if i == len(freqs) - 1 else 0.5 * (f + freqs[i + 1])
        out.append((lo, hi))
    return out


def snr_db(raw_chan):
    """Carrier over the noise in the measurement window."""
    if not raw_chan or raw_chan.get("noise_dbfs_hz") is None:
        return None
    noise = raw_chan["noise_dbfs_hz"] + 10 * math.log10(2 * HALF_WINDOW)
    return raw_chan["dbfs"] - noise


def median(v):
    v = sorted(v)
    if not v:
        return None
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


class Run:
    """One calibration run. `source`: an instruments.Siglent or HpGenerator;
    `analyser`: the Siglent used to normalise the path (None for the HP with
    a constant cable loss); `ask(msg)`: wait for the operator."""

    def __init__(self, board, source, kind, analyser=None, ask=input, log=print,
                 levels=(0.0, -10.0, -20.0, -30.0, -40.0), cable_loss_db=0.0, gain_step=5.0, gains=True, bands=None):
        self.b = board
        self.src = source
        self.kind = kind
        self.sa = analyser
        self.ask = ask
        self.log = log
        self.levels = list(levels)
        self.cable = cable_loss_db
        self.gain_step = gain_step
        self.do_gains = gains
        self.bands = bands
        self.path = {}  # (f, source level) -> dBm at the socket
        self.date = datetime.date.today().isoformat()

    # ---- known input levels ----------------------------------------

    def source_levels(self):
        """The source settings the run may use, highest first."""
        return sorted(set(self.levels), reverse=True)

    def normalise(self, freqs):
        """Measure the reference path into the analyser (Siglent TG path, or
        the HP's cable)."""
        if self.kind == "hp" and self.sa is None:
            for f in freqs:
                for lv in self.source_levels():
                    self.path[(f, lv)] = lv - self.cable
            return
        self.ask("Connect the reference path (cable, attenuator) to the ANALYSER input, then press Enter.")
        for f in freqs:
            for lv in self.source_levels():
                if self.kind == "hp" and lv < -60:
                    # Below the analyser's reach: the generator's attenuator
                    # is trusted, the cable loss measured at -60 dBm reused.
                    continue
                self.src.source(f, lv)
                self.path[(f, lv)] = self.sa.measure()
            if self.kind == "hp":
                ref = max(lv for lv in self.source_levels() if (f, lv) in self.path)
                loss = ref - self.path[(f, ref)]
                for lv in self.source_levels():
                    self.path.setdefault((f, lv), lv - loss)
            self.log(f"  path {f / 1e6:9.3f} MHz: " + ", ".join(f"{lv:+.0f} -> {self.path[(f, lv)]:.2f} dBm" for lv in self.source_levels() if (f, lv) in self.path))
        self.src.off()

    def p_in(self, f, lv):
        return self.path[(f, lv)]

    # ---- the board -------------------------------------------------

    def read(self, f, port):
        self.b.port(port)
        r = self.b.fresh_raw(f - HALF_WINDOW, f + HALF_WINDOW)
        if int(r.get("port", port)) != port:
            raise RuntimeError(f"the board is on RX{r.get('port')}, not RX{port} (a band-to-socket mapping in SET overrides it?)")
        if r.get("xvtr"):
            raise RuntimeError(f"transverter {r['xvtr']} is active at {f / 1e6} MHz: calibrate without transverters")
        return r

    def measure_k(self, f, port):
        """K at f, from the level that puts the channel nearest TARGET_DBFS."""
        self.b.tune(f - 1000.0)
        self.b.port(port)
        self.b.gain_manual(G0)
        k_guess = 12.0
        lv = self.pick_level(f, G0, k_guess)
        for _ in range(3):
            self.src.source(f, lv)
            r = self.read(f, port)
            ch = r.get("chan")
            if not ch:
                raise RuntimeError(f"no channel measurement at {f / 1e6} MHz: {r}")
            g = r["hw_gain_db"]
            k = self.p_in(f, lv) + g - ch["dbfs"]
            better = self.pick_level(f, g, k)
            if r.get("clip") or ch["dbfs"] > DBFS_MAX or (snr_db(ch) or 99) < MIN_SNR_DB:
                if better == lv:
                    raise RuntimeError(f"{f / 1e6} MHz: no source level gives a clean reading (dBFS {ch['dbfs']:.1f}, SNR {snr_db(ch)})")
                lv = better
                continue
            break
        # The stream (wide bands) and the spectrometer (wider) on the same carrier.
        st = r.get("stream")
        wide = self.b.raw(f - 300e3, f + 300e3).get("maia") if hasattr(self.b, "raw") else None
        return {"f": r["hw_freq_hz"], "k": k, "src": self.kind, "date": self.date, "t": r.get("temp_c"),
                "_stream": (ch["dbfs"] - st["dbfs"]) if st else None,
                "_maia": (ch["dbfs"] - wide["dbfs"]) if wide else None, "_g": g}

    def pick_level(self, f, g, k):
        """The source level whose predicted channel level is nearest the target."""
        best = None
        for lv in self.source_levels():
            if (f, lv) not in self.path:
                continue
            pred = self.p_in(f, lv) + g - k
            if pred > DBFS_MAX:
                continue
            if best is None or abs(pred - TARGET_DBFS) < abs(best[1] - TARGET_DBFS):
                best = (lv, pred)
        if best is None:
            raise RuntimeError(f"{f / 1e6} MHz: every source level would overload the board at {g} dB gain")
        return best[0]

    def gain_table(self, f, port, k):
        """E(G) at f: (reported gain, error) pairs, 0 at the reference."""
        self.b.tune(f - 1000.0)
        pts = []
        ref = None
        g = 0.0
        while g <= 71.0:
            self.b.gain_manual(g)
            try:
                lv = self.pick_level(f, g, k)
            except RuntimeError as e:
                self.log(f"    {g:.0f} dB: skipped ({e}; a bigger attenuator reaches it)")
                g += self.gain_step
                continue
            self.src.source(f, lv)
            r = self.read(f, port)
            ch = r.get("chan")
            gr = r["hw_gain_db"]
            ok = ch and not r.get("clip") and ch["dbfs"] <= DBFS_MAX and (snr_db(ch) or 0) >= MIN_SNR_DB
            if ok:
                # Gain of the whole chain as measured, against the report.
                meas = ch["dbfs"] - self.p_in(f, lv)
                pts.append((gr, meas - gr))
                if abs(gr - G0) < 0.6:
                    ref = meas - gr
            else:
                self.log(f"    {gr:.0f} dB: skipped (dBFS {ch and ch['dbfs']}, SNR {ch and snr_db(ch)})")
            g += self.gain_step
        if ref is None:
            raise RuntimeError(f"{f / 1e6} MHz: no clean reading at the reference gain {G0} dB")
        return [[round(gr, 2), round(e - ref, 3)] for gr, e in pts]

    # ---- a whole run -----------------------------------------------

    def run(self, ports, max_hz):
        freqs = plan(max_hz, self.bands)
        gfreqs = gain_plan(max_hz, self.bands) if self.do_gains else []
        self.log(f"{len(freqs)} frequencies up to {max_hz / 1e6:.0f} MHz; gain sweeps at "
                 + (", ".join(f"{g / 1e6:.0f}" for g in gfreqs) or "none") + " MHz; sockets " + ", ".join(f"RX{p}" for p in ports))
        self.normalise(sorted(set(freqs) | set(gfreqs)))
        tables = {}
        for port in ports:
            self.ask(f"Connect the reference path to the board's RX{port} input (and nothing to the other), then press Enter.")
            pts = []
            for f in freqs:
                p = self.measure_k(f, port)
                self.log(f"  RX{port} {f / 1e6:9.3f} MHz: K {p['k']:+.2f} dB at {p['_g']:.0f} dB gain")
                pts.append(p)
            gains = []
            kmap = {round(p["f"]): p["k"] for p in pts}
            for (lo, hi), f in zip(gain_ranges(gfreqs), gfreqs):
                k = kmap.get(round(f))
                if k is None:
                    k = self.measure_k(f, port)["k"]
                self.log(f"  RX{port} gain sweep at {f / 1e6:.0f} MHz")
                gains.append({"f_min": lo, "f_max": hi, "pts": self.gain_table(f, port, k)})
            self.src.off()
            tables[port] = build_table(pts, gains, f"sqtrx-cal {self.kind} {self.date}")
        return tables


def build_table(points, gains, note):
    s = [p["_stream"] for p in points if p.get("_stream") is not None]
    m = [p["_maia"] for p in points if p.get("_maia") is not None]
    k = [{key: (round(v, 3) if isinstance(v, float) else v) for key, v in p.items() if not key.startswith("_") and v is not None} for p in points]
    return {"version": 1, "note": note, "k": k, "gain": gains,
            "stream_db": round(median(s), 3) if s else 0.0, "maia_db": round(median(m), 3) if m else 0.0,
            "temp_coef": 0.0, "xvtr": {}}


def merge(old, new, replace=False):
    """New points over an existing table: points within 1 MHz of a new one
    and gain tables overlapping a new one are dropped."""
    if replace or not old:
        return new
    out = dict(old)
    out["k"] = [p for p in old.get("k", []) if all(abs(p["f"] - q["f"]) > 1e6 for q in new["k"])] + new["k"]
    out["k"].sort(key=lambda p: p["f"])
    out["gain"] = [g for g in old.get("gain", []) if all(g["f_max"] < n["f_min"] or g["f_min"] > n["f_max"] for n in new["gain"])] + new["gain"]
    for key in ("stream_db", "maia_db"):
        if new.get(key):
            out[key] = new[key]
    out["note"] = (old.get("note", "") + " + " + new["note"])[-500:]
    return out
