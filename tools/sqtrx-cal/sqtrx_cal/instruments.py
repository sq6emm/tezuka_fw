"""Reference instruments: the Siglent SVA1032X (LAN SCPI, tracking
generator to 3.2 GHz) and an HP signal generator (to 2.1 GHz, down to
-150 dBm) on the xyphro UsbGpib v2 adapter (USBTMC).

Every command goes through `Instrument.write`/`query`, which in a dry run
only prints them. Command strings are in tables so they can be fixed
without touching the logic (check them against your instrument's manual
before the first real run: `python -m sqtrx_cal check`).
"""

import socket
import time


class InstrumentError(Exception):
    pass


class Instrument:
    def __init__(self, name, dry_run=False, log=print):
        self.name = name
        self.dry_run = dry_run
        self.log = log

    def _write(self, cmd):
        raise NotImplementedError

    def _query(self, cmd):
        raise NotImplementedError

    def write(self, cmd):
        if self.dry_run:
            self.log(f"  [{self.name}] > {cmd}")
            return
        self._write(cmd)

    def query(self, cmd, dry_answer=""):
        if self.dry_run:
            self.log(f"  [{self.name}] ? {cmd}")
            return dry_answer
        return self._query(cmd).strip()

    def close(self):
        pass


# ---------------------------------------------------------------- Siglent

class ScpiSocket(Instrument):
    """Raw SCPI over TCP (port 5025), newline terminated."""

    def __init__(self, name, host, port=5025, dry_run=False, log=print, timeout=10.0):
        super().__init__(name, dry_run, log)
        self.addr = (host, port)
        self.sock = None
        self.timeout = timeout
        if not dry_run:
            self.sock = socket.create_connection(self.addr, timeout=timeout)
            self.buf = b""

    def _write(self, cmd):
        self.sock.sendall(cmd.encode() + b"\n")

    def _query(self, cmd):
        self._write(cmd)
        end = time.monotonic() + self.timeout
        while b"\n" not in self.buf:
            if time.monotonic() > end:
                raise InstrumentError(f"{self.name}: no answer to {cmd}")
            chunk = self.sock.recv(65536)
            if not chunk:
                raise InstrumentError(f"{self.name}: connection closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace")

    def close(self):
        if self.sock:
            self.sock.close()


# SVA1000X series (SA mode). Tracking generator in zero span: the TG sends a
# CW carrier at the centre frequency, and the trace is that level.
SIGLENT_CMDS = {
    "idn": "*IDN?",
    "err": ":SYSTem:ERRor?",
    "sa_mode": ":INSTrument:SELect SA",
    "center": ":FREQuency:CENTer {hz:.0f} Hz",
    "zero_span": ":FREQuency:SPAN 0 Hz",
    "rbw": ":BWIDth {hz:.0f} Hz",
    "ref_level": ":DISPlay:WINDow:TRACe:Y:RLEVel {dbm:.1f} dBm",
    "tg_level": ":SOURce:POWer {dbm:.1f} dBm",
    "tg_on": ":OUTPut:STATe ON",
    "tg_off": ":OUTPut:STATe OFF",
    "marker_on": ":CALCulate:MARKer1:STATe ON",
    "marker_y": ":CALCulate:MARKer1:Y?",
}


class Siglent:
    """Tracking generator as a CW source, the analyser as a level meter."""

    max_hz = 3.2e9
    min_level, max_level = -40.0, 0.0

    def __init__(self, inst, rbw_hz=1000.0, cmds=None):
        self.i = inst
        self.cmds = dict(SIGLENT_CMDS, **(cmds or {}))
        self.rbw = rbw_hz
        self.level = None
        # What the source sends now: (Hz, dBm) or None (the simulation reads it).
        self.out = None

    def c(self, key, **kw):
        return self.cmds[key].format(**kw)

    def identify(self):
        idn = self.i.query(self.c("idn"), dry_answer="Siglent Technologies,SVA1032X,(dry run),1.0")
        if "siglent" not in idn.lower():
            raise InstrumentError(f"not a Siglent: {idn!r}")
        return idn

    def setup(self):
        self.i.write(self.c("sa_mode"))
        self.i.write(self.c("zero_span"))
        self.i.write(self.c("rbw", hz=self.rbw))
        self.i.write(self.c("ref_level", dbm=0.0))
        self.i.write(self.c("marker_on"))

    def errors(self):
        e = self.i.query(self.c("err"), dry_answer='0,"No error"')
        return e

    def source(self, hz, dbm):
        if hz > self.max_hz:
            raise InstrumentError(f"the tracking generator stops at {self.max_hz / 1e9} GHz")
        if not (self.min_level <= dbm <= self.max_level):
            raise InstrumentError(f"tracking generator level {dbm} dBm out of {self.min_level}..{self.max_level}")
        self.i.write(self.c("center", hz=hz))
        if dbm != self.level:
            self.i.write(self.c("tg_level", dbm=dbm))
            self.level = dbm
        self.i.write(self.c("tg_on"))
        self.out = (hz, dbm)

    def off(self):
        self.i.write(self.c("tg_off"))
        self.out = None

    def measure(self, settle_s=0.6, reads=3):
        """The level at the analyser input, dBm (mean of a few marker reads)."""
        time.sleep(0 if self.i.dry_run else settle_s)
        vals = []
        for _ in range(reads):
            v = self.i.query(self.c("marker_y"), dry_answer="-30.0")
            vals.append(float(v.split(",")[0]))
            if not self.i.dry_run:
                time.sleep(0.15)
        return sum(vals) / len(vals)


# ---------------------------------------------------------------- HP

class Usbtmc(Instrument):
    """Linux kernel usbtmc device (/dev/usbtmcN): the UsbGpib v2 adapter
    shows each GPIB instrument as one."""

    def __init__(self, name, path, dry_run=False, log=print):
        super().__init__(name, dry_run, log)
        self.path = path
        self.f = None if dry_run else open(path, "r+b", buffering=0)

    def _write(self, cmd):
        self.f.write(cmd.encode() + b"\n")

    def _query(self, cmd):
        self._write(cmd)
        return self.f.read(4096).decode(errors="replace")

    def close(self):
        if self.f:
            self.f.close()


class Visa(Instrument):
    """Any VISA resource (pyvisa with pyvisa-py/pyusb, or NI-VISA on Windows),
    e.g. USB0::0x03EB::0x2065::<serial>::INSTR for the UsbGpib v2."""

    def __init__(self, name, resource, dry_run=False, log=print, backend=""):
        super().__init__(name, dry_run, log)
        self.res = None
        if not dry_run:
            import pyvisa  # noqa: local import, only needed for real runs

            rm = pyvisa.ResourceManager(backend) if backend else pyvisa.ResourceManager()
            self.res = rm.open_resource(resource)
            self.res.timeout = 10000
            self.res.write_termination = "\n"
            self.res.read_termination = "\n"

    def _write(self, cmd):
        self.res.write(cmd)

    def _query(self, cmd):
        return self.res.query(cmd)

    def close(self):
        if self.res:
            self.res.close()


# HP-IB command sets (pre-488.2: no *IDN?, `idn` None). Each: frequency,
# amplitude in dBm, RF on and off ("" = no code sent), level range, top
# frequency. 8642B (the user's): FR <f> MZ and AP <l> DM are the HP-IB
# codes of this generator family (8642A/B, 8656B, 8657A/B); the RF on/off
# codes are NOT known for sure for the 8642B, so "off" is the lowest
# amplitude, -140 dBm (always valid), and "on" sends nothing (the next AP
# sets the level). --hp-on / --hp-off give real codes once checked.
HP_CMDS = {
    "8642b": {"idn": None, "freq": "FR {mhz:.6f} MZ", "level": "AP {dbm:.1f} DM", "on": "", "off": "AP -140.0 DM",
              "min": -140.0, "max": 16.0, "max_hz": 2.1e9},
    "8642a": {"idn": None, "freq": "FR {mhz:.6f} MZ", "level": "AP {dbm:.1f} DM", "on": "", "off": "AP -140.0 DM",
              "min": -140.0, "max": 16.0, "max_hz": 1.057e9},
    "8657": {"idn": None, "freq": "FR {mhz:.6f} MZ", "level": "AP {dbm:.1f} DM", "on": "R3", "off": "R2",
             "min": -143.5, "max": 13.0, "max_hz": 1.04e9},
    "8656": {"idn": None, "freq": "FR {mhz:.6f} MZ", "level": "AP {dbm:.1f} DM", "on": "R3", "off": "R2",
             "min": -127.0, "max": 13.0, "max_hz": 0.99e9},
    "scpi": {"idn": "*IDN?", "freq": "FREQ {hz:.0f} HZ", "level": "POW {dbm:.1f} DBM", "on": "OUTP ON", "off": "OUTP OFF",
             "min": -136.0, "max": 13.0, "max_hz": 4e9},
}


class HpGenerator:
    def __init__(self, inst, model="8642b", max_hz=None, on=None, off=None):
        if model not in HP_CMDS:
            raise InstrumentError(f"unknown HP command set {model!r}: one of {', '.join(HP_CMDS)}")
        self.i = inst
        self.model = model
        self.cmds = dict(HP_CMDS[model])
        if on is not None:
            self.cmds["on"] = on
        if off is not None:
            self.cmds["off"] = off
        self.min_level, self.max_level = self.cmds["min"], self.cmds["max"]
        self.max_hz = min(max_hz, self.cmds["max_hz"]) if max_hz else self.cmds["max_hz"]
        self.out = None

    def _send(self, key, **kw):
        c = self.cmds[key]
        if c:
            self.i.write(c.format(**kw))

    def identify(self):
        if self.cmds["idn"]:
            return self.i.query(self.cmds["idn"], dry_answer="HP (dry run)")
        # No ID query: a harmless command, and the adapter must accept it.
        self._send("off")
        return f"HP {self.model} command set (no ID query; output set to off/minimum as a check: look at the generator's display)"

    def setup(self):
        self._send("off")

    def errors(self):
        return ""

    def source(self, hz, dbm):
        if hz > self.max_hz:
            raise InstrumentError(f"the HP generator stops at {self.max_hz / 1e9} GHz")
        if not (self.min_level <= dbm <= self.max_level):
            raise InstrumentError(f"HP level {dbm} dBm out of {self.min_level}..{self.max_level}")
        # Level to the minimum first, so a frequency change never passes a
        # high level into the board, then frequency, level, RF on.
        if self.out is not None:
            self.i.write(self.cmds["level"].format(dbm=self.min_level))
        self.i.write(self.cmds["freq"].format(mhz=hz / 1e6, hz=hz))
        self.i.write(self.cmds["level"].format(dbm=dbm))
        self._send("on")
        self.out = (hz, dbm)
        if not self.i.dry_run:
            time.sleep(0.3)

    def off(self):
        self._send("off")
        self.out = None


def open_hp(spec, model="8642b", dry_run=False, log=print, max_hz=None, on=None, off=None):
    """`spec`: usbtmc:/dev/usbtmc0, or visa:<resource>, or a bare VISA resource."""
    if spec.startswith("usbtmc:"):
        inst = Usbtmc("HP", spec[7:], dry_run, log)
    else:
        inst = Visa("HP", spec[5:] if spec.startswith("visa:") else spec, dry_run, log)
    return HpGenerator(inst, model, max_hz, on, off)


def open_siglent(addr, dry_run=False, log=print, rbw_hz=1000.0):
    host, _, port = addr.partition(":")
    return Siglent(ScpiSocket("Siglent", host, int(port or 5025), dry_run, log), rbw_hz)
