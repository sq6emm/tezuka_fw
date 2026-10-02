"""SQTRX (trxd) web API client: log in, open the WebSocket, send commands,
read the level meter's raw measurements, read and upload calibration tables.

The board serves HTTPS with a self-signed certificate; it is not verified
(pass --fingerprint to pin it).
"""

import hashlib
import http.client
import json
import ssl
import time
import urllib.parse

try:
    import websocket  # websocket-client
except ImportError:  # the tests use a fake board, the dry run none
    websocket = None


class BoardError(Exception):
    pass


class Board:
    """One SQTRX board. `host` may carry a port (host:8443)."""

    def __init__(self, host, password, fingerprint=None, log=print, timeout=10.0):
        self.host = host
        self.password = password
        self.fingerprint = (fingerprint or "").replace(":", "").lower() or None
        self.log = log
        self.timeout = timeout
        self.ws = None
        self.state = None

    # ---- connection -------------------------------------------------

    def _ctx(self):
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        return ctx

    def _check_fp(self, sock):
        if not self.fingerprint:
            return
        der = sock.getpeercert(binary_form=True)
        fp = hashlib.sha256(der).hexdigest()
        if fp != self.fingerprint:
            raise BoardError(f"certificate fingerprint {fp} is not the expected {self.fingerprint}")

    def login(self):
        conn = http.client.HTTPSConnection(self.host, timeout=self.timeout, context=self._ctx())
        body = urllib.parse.urlencode({"password": self.password})
        conn.request("POST", "/login", body, {"Content-Type": "application/x-www-form-urlencoded"})
        self._check_fp(conn.sock)
        r = conn.getresponse()
        r.read()
        cookie = None
        for k, v in r.getheaders():
            if k.lower() == "set-cookie" and v.startswith("trxd="):
                cookie = v.split(";", 1)[0]
        if not cookie or cookie == "trxd=":
            raise BoardError(f"login refused by {self.host} (status {r.status}): wrong password?")
        return cookie

    def connect(self):
        if websocket is None:
            raise BoardError("the websocket-client package is missing: pip install -r requirements.txt")
        cookie = self.login()
        self.ws = websocket.create_connection(
            f"wss://{self.host}/ws",
            cookie=cookie,
            sslopt={"cert_reqs": ssl.CERT_NONE, "check_hostname": False},
            timeout=self.timeout,
        )
        self.wait("state")
        return self

    def close(self):
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass
            self.ws = None

    # ---- messages ---------------------------------------------------

    def send(self, **cmd):
        self.ws.send(json.dumps(cmd))

    def wait(self, typ, pred=None, timeout=None):
        """The next JSON message of type `typ` (and `pred(msg)`)."""
        end = time.monotonic() + (timeout or self.timeout)
        while time.monotonic() < end:
            self.ws.settimeout(max(0.05, end - time.monotonic()))
            try:
                raw = self.ws.recv()
            except Exception as e:  # timeout and friends
                if "timed out" in str(e).lower():
                    continue
                raise
            if isinstance(raw, (bytes, bytearray)):
                continue  # spectrum rows, audio
            try:
                m = json.loads(raw)
            except ValueError:
                continue
            if m.get("type") == "state":
                self.state = m
            if m.get("type") == typ and (pred is None or pred(m)):
                return m
        raise BoardError(f"no '{typ}' message from the board within {timeout or self.timeout:.0f} s")

    # ---- radio ------------------------------------------------------

    def tune(self, hz, mode="USB"):
        self.send(cmd="mode", mode=mode)
        self.send(cmd="freq", hz=float(hz))

    def gain_manual(self, db):
        self.send(cmd="rxgain", mode="manual", db=float(db))

    def gain_auto(self):
        self.send(cmd="rxgain", mode="slow")

    def port(self, n):
        self.send(cmd="port", port=int(n))

    def raw(self, lo_hz=None, hi_hz=None):
        kw = {}
        if lo_hz is not None:
            kw = {"lo_hz": float(lo_hz), "hi_hz": float(hi_hz)}
        self.send(cmd="meter_raw", **kw)
        return self.wait("meter_raw")

    def fresh_raw(self, lo_hz, hi_hz, settle_s=1.5, spectra=2, timeout=15.0):
        """A measurement made wholly after the last change: waits `settle_s`
        (the gain readback is refreshed once a second), then until the
        channel meter has finished `spectra` new averages."""
        time.sleep(settle_s)
        first = self.raw(lo_hz, hi_hz)
        seq0 = (first.get("chan") or {}).get("seq")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            time.sleep(0.2)
            r = self.raw(lo_hz, hi_hz)
            seq = (r.get("chan") or {}).get("seq")
            if seq0 is None or (seq is not None and seq >= seq0 + spectra):
                return r
        raise BoardError("the level meter gave no fresh reading (is the board in DATV mode or transmitting?)")

    def calib_get(self):
        self.send(cmd="calib_get")
        return self.wait("calib")

    def calib_set(self, port, table):
        self.send(cmd="calib_set", port=int(port), table=table)
        ack = self.wait("calib_ack")
        if not ack.get("ok"):
            raise BoardError(f"the board refused the RX{port} table: {ack.get('error')}")
        return ack
