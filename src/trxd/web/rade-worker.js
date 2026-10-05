// RADE V2 in the SQTRX page: a Worker around rade.wasm (rade_web.c).
// trxd serves it as /rade-worker.js (compiled in) and the module as
// /rade.wasm (from the model flash partition, src/trxd/src/rade.rs).
//
// Page -> worker:
//   { init: url }                   load the module (then { ready } or { error })
//   { rx: Float32Array }            12 kHz receive audio (the radio's USB audio)
//   { rxReset: true }               receiver from scratch (retune, mode change)
//   { tx: Float32Array }            48 kHz microphone audio
//   { txStart: true }               a new over
//   { txEnd: true }                 end of over: the rest and the EOO frame
// Worker -> page:
//   { pcm: Float32Array, sync, snr, foff, frames, eoo }   12 kHz decoded speech
//                                   (the band 12 dB down while searching)
//   { mod: Float32Array, end }      12 kHz modem audio for the web microphone
"use strict";

// Rational resampler: up by L, low-pass, down by M (polyphase FIR,
// Blackman-windowed sinc at 0.92 x the lower Nyquist, 40 lobes long).
class Resampler {
  constructor(L, M) {
    this.L = L; this.M = M;
    const tapsPerPhase = Math.ceil(40 * Math.max(L, M) / L);
    const n = L * tapsPerPhase, fc = 0.46 / Math.max(L, M), mid = (n - 1) / 2;
    const h = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const t = i - mid, w = 0.42 - 0.5 * Math.cos(2 * Math.PI * i / (n - 1)) + 0.08 * Math.cos(4 * Math.PI * i / (n - 1));
      h[i] = (t === 0 ? 2 * fc : Math.sin(2 * Math.PI * fc * t) / (Math.PI * t)) * w * L;
    }
    // phase p holds h[p], h[p+L], ... (newest input first in the dot product)
    this.T = tapsPerPhase;
    this.ph = [];
    for (let p = 0; p < L; p++) { const a = new Float32Array(this.T); for (let k = 0; k < this.T; k++) a[k] = h[p + k * L]; this.ph.push(a); }
    this.hist = new Float32Array(2 * this.T);   // the last T inputs, twice: hist[hi .. hi+T) newest last
    this.hi = 0; this.phase = 0;                // output position in the up-sampled stream, mod L
  }
  process(x) {
    const T = this.T, L = this.L, M = this.M, h = this.hist;
    const out = new Float32Array(Math.ceil((x.length * L + L) / M) + 1);
    let n = 0;
    for (let i = 0; i < x.length; i++) {
      h[this.hi] = h[this.hi + T] = x[i];
      this.hi = this.hi + 1 === T ? 0 : this.hi + 1;
      // newest sample at h[hi + T - 1], oldest at h[hi]
      const top = this.hi + T - 1;
      while (this.phase < L) {
        const c = this.ph[this.phase]; let s = 0;
        for (let k = 0; k < T; k++) s += c[k] * h[top - k];
        out[n++] = s;
        this.phase += M;
      }
      this.phase -= L;
    }
    return out.subarray(0, n);
  }
}

// Coarse frequency search: RADE V2 resolves only +/-31 Hz of offset (half
// its 62.5 Hz carrier spacing) and, further off, locks whole carriers away
// and decodes nonsense. On VHF/UHF two radios are often 100s of Hz apart.
// While searching, find the 875 Hz wide block of 14 carriers (1062-1875 Hz
// when on frequency) in the averaged 8 kHz spectrum, +/-600 Hz.
class Coarse {
  constructor() {
    this.N = 512; this.buf = new Float32Array(this.N); this.n = 0;
    this.P = new Float64Array(this.N / 2); this.blocks = 0;
    this.win = Float32Array.from({ length: this.N }, (_, i) => 0.5 - 0.5 * Math.cos(2 * Math.PI * i / this.N));
    this.re = new Float64Array(this.N); this.im = new Float64Array(this.N);
  }
  reset() { this.P.fill(0); this.blocks = 0; this.n = 0; }
  // Halve the history: the average reaches about a second back.
  decay() { for (let k = 0; k < this.P.length; k++) this.P[k] *= 0.5; this.blocks /= 2; }
  fft() {   // in place, radix 2
    const N = this.N, re = this.re, im = this.im;
    for (let i = 1, j = 0; i < N; i++) { let b = N >> 1; for (; j & b; b >>= 1) j ^= b; j ^= b; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; } }
    for (let len = 2; len <= N; len <<= 1) {
      const a = -2 * Math.PI / len;
      for (let i = 0; i < N; i += len) for (let k = 0; k < len / 2; k++) {
        const c = Math.cos(a * k), s = Math.sin(a * k), p = i + k, q = p + len / 2;
        const tr = re[q] * c - im[q] * s, ti = re[q] * s + im[q] * c;
        re[q] = re[p] - tr; im[q] = im[p] - ti; re[p] += tr; im[p] += ti;
      }
    }
  }
  push(a8) {
    for (let i = 0; i < a8.length; i++) {
      this.buf[this.n++] = a8[i];
      if (this.n < this.N) continue;
      this.n = 0;
      for (let k = 0; k < this.N; k++) { this.re[k] = this.buf[k] * this.win[k]; this.im[k] = 0; }
      this.fft();
      for (let k = 0; k < this.N / 2; k++) this.P[k] += this.re[k] * this.re[k] + this.im[k] * this.im[k];
      this.blocks++;
    }
  }
  // Offset in Hz of the 875 Hz block against its nominal place, or null
  // without a clear one (needs ~0.5 s of signal). Template: the block's
  // power minus the power just outside its edges (edges, not the content of
  // the carriers, which varies with the speech), peak refined by a parabola.
  estimate() {
    if (this.blocks < 7) return null;
    const bin = 8000 / this.N, w = Math.round(BLOCK_HZ / bin), g = Math.round(120 / bin);
    // In dB: a carrier (a CW signal, a birdie) in one bin hardly moves the
    // block's average, while the block's edges stay sharp.
    const P = this.P.map((v) => 10 * Math.log10(v + 1e-30));
    const lo0 = Math.round(BLOCK_LO / bin), from = Math.round(-600 / bin), to = Math.round(600 / bin);
    const c = new Float64Array(P.length + 1); for (let k = 0; k < P.length; k++) c[k + 1] = c[k] + P[k];
    const sum = (a, b) => c[Math.min(Math.max(b, 0), P.length)] - c[Math.min(Math.max(a, 0), P.length)];
    const score = (d) => { const a = lo0 + d; return sum(a, a + w) / w - (sum(a - g, a) + sum(a + w, a + w + g)) / (2 * g); };
    let best = -Infinity, at = 0;
    for (let d = from; d <= to; d++) { const e = score(d); if (e > best) { best = e; at = d; } }
    // Clear only if the block stands out from the band beside it by 1.5 dB
    // (noise alone, averaged over 0.5 s, reaches about 1 dB somewhere).
    if (best < 1.5) return null;
    const l = score(at - 1), r = score(at + 1), den = l - 2 * best + r;
    const frac = den < 0 ? Math.max(-0.5, Math.min(0.5, 0.5 * (l - r) / den)) : 0;
    return { hz: (at + frac) * bin - BLOCK_CAL, db: best };
  }
}
// The block's edges and the estimator's offset on a clean signal
// (calibrated with src/rade-web/test.js output: 0 Hz in, 0 Hz out).
const BLOCK_LO = 1040, BLOCK_HZ = 870, BLOCK_CAL = -9, CARRIER_HZ = 62.5;

class Rade {
  constructor(instance) {
    this.x = instance.exports;
    this.x._initialize();
    if (this.x.rw_rx_open() !== 0 || this.x.rw_tx_open() !== 0) throw new Error("RADE open failed");
    this.monitor = true;
    this.rxReset(); this.txStart();
  }
  f32(ptr, n) { return new Float32Array(this.x.memory.buffer, ptr, n); }
  rxReset() {
    this.x.rw_rx_open();
    this.coarse = new Coarse(); this.slip = 0; this.shift = 0; this.x.rw_rx_set_shift(0); this.sinceEst = 0;
    this.rxIn = new Resampler(2, 3);     // 12 -> 8 kHz
    this.rxOut = new Resampler(3, 4);    // 16 -> 12 kHz
  }
  // 12 kHz receive audio -> 12 kHz speech (empty while not in sync)
  rx(a12) {
    const a8 = this.rxIn.process(a12), x = this.x, parts = [];
    this.afc(a8);
    for (let p = 0; p < a8.length; p += 4096) {
      const n = Math.min(4096, a8.length - p);
      this.f32(x.rw_rx_in(), n).set(a8.subarray(p, p + n));
      const k = x.rw_rx_push(n);
      if (k > 0) parts.push(this.f32(x.rw_rx_out(), k).slice());
    }
    let pcm16 = new Float32Array(parts.reduce((s, a) => s + a.length, 0)), o = 0;
    for (const a of parts) { pcm16.set(a, o); o += a.length; }
    this.last16 = pcm16;
    const speech = this.rxOut.process(pcm16);
    // Searching: the band itself, 12 dB down, so that tuning is by ear.
    if (!speech.length && !x.rw_rx_sync() && this.monitor) return a12.map((v) => v * 0.25);
    return speech;
  }
  // Every 0.5 s. Searching: move the receiver onto the block the coarse
  // search finds. In sync: a lock whole carriers away also looks like sync
  // (and decodes nonsense): move by that many carriers and start the
  // receiver afresh.
  afc(a8) {
    this.coarse.push(a8);
    this.sinceEst += a8.length;
    if (this.sinceEst < 4000) return;
    this.sinceEst = 0;
    const e = this.coarse.estimate(), x = this.x;
    this.coarse.decay();
    if (!x.rw_rx_sync()) {
      this.slip = 0;
      // (Near the threshold the estimate wanders: leave the shift there.)
      if (e && e.db >= 2.5 && Math.abs(e.hz - this.shift) > 15) { this.shift = Math.round(e.hz); x.rw_rx_set_shift(this.shift); }
      return;
    }
    // Two or more carriers off: the coarse search sees that plainly on a
    // strong signal (the receiver's own view is narrowed by its band-pass
    // around where it thinks the signal is). One carrier: rw_rx_slip.
    const big = e && e.db >= 6 ? Math.round((e.hz - (this.shift + x.rw_rx_foff())) / CARRIER_HZ) : 0;
    let j;
    if (Math.abs(big) >= 2) { j = big; this.slip = 0; } else {
      j = x.rw_rx_slip();
      if (this.debug && x.rw_rx_slip_ratio) this.debug.push({ f: x.rw_rx_frames(), j, r: [-2, -1, 0, 1, 2].map((k) => +x.rw_rx_slip_ratio(k).toFixed(1)) });
      const again = j !== 0 && j === this.slip;
      this.slip = j;
      if (!again) return;
    }
    if (this.debug) this.debug.push({ frames: x.rw_rx_frames(), shift: this.shift, foff: x.rw_rx_foff(), j, big });
    this.shift = Math.round(this.shift + j * CARRIER_HZ);
    this.slip = 0;
    x.rw_rx_open();
    x.rw_rx_set_shift(this.shift);
  }
  status() {
    const x = this.x;
    return { sync: !!x.rw_rx_sync(), snr: x.rw_rx_snr(), foff: this.shift + x.rw_rx_foff(), frames: x.rw_rx_frames(), eoo: x.rw_rx_eoo_count() };
  }
  txStart() {
    this.x.rw_tx_open();
    this.txIn = new Resampler(1, 3);     // 48 -> 16 kHz
    this.txOut = new Resampler(3, 2);    // 8 -> 12 kHz
  }
  txFrom(k) { return this.txOut.process(this.f32(this.x.rw_tx_out(), k).slice()); }
  // 48 kHz microphone -> 12 kHz modem audio
  tx(a48) {
    const a16 = this.txIn.process(a48), x = this.x, parts = [];
    for (let p = 0; p < a16.length; p += 4096) {
      const n = Math.min(4096, a16.length - p);
      this.f32(x.rw_tx_in(), n).set(a16.subarray(p, p + n));
      const k = x.rw_tx_push(n);
      if (k > 0) parts.push(this.txFrom(k));
    }
    let m = new Float32Array(parts.reduce((s, a) => s + a.length, 0)), o = 0;
    for (const a of parts) { m.set(a, o); o += a.length; }
    return m;
  }
  txEnd() {
    const k = this.x.rw_tx_eoo();
    // The resampler's delay line flushed with zeros: the EOO goes out whole.
    const a = this.txFrom(k), z = this.txOut.process(new Float32Array(48));
    const m = new Float32Array(a.length + z.length); m.set(a); m.set(z, a.length);
    return m;
  }
}

async function radeLoad(bytesOrResponse) {
  const imports = {
    env: { emscripten_notify_memory_growth: () => {} },
    wasi_snapshot_preview1: { fd_close: () => 0, fd_write: () => 0, fd_seek: () => 0 },
  };
  const r = bytesOrResponse instanceof Response
    ? await WebAssembly.instantiateStreaming(bytesOrResponse, imports).catch(async () => WebAssembly.instantiate(await bytesOrResponse.arrayBuffer(), imports))
    : await WebAssembly.instantiate(bytesOrResponse, imports);
  return new Rade(r.instance);
}

if (typeof module !== "undefined") module.exports = { Resampler, Coarse, Rade, radeLoad };

if (typeof WorkerGlobalScope !== "undefined" && self instanceof WorkerGlobalScope) {
  let rade = null;
  self.onmessage = async (e) => {
    const d = e.data;
    try {
      if (d.init) {
        const res = await fetch(d.init);
        if (!res.ok) throw new Error("rade.wasm: HTTP " + res.status);
        rade = await radeLoad(res);
        self.postMessage({ ready: true });
        return;
      }
      if (!rade) return;
      if (d.rx) {
        const pcm = rade.rx(d.rx);
        self.postMessage({ pcm, ...rade.status() }, [pcm.buffer]);
      } else if (d.rxReset) rade.rxReset();
      else if (d.txStart) rade.txStart();
      else if (d.tx) { const mod = rade.tx(d.tx); if (mod.length) self.postMessage({ mod }, [mod.buffer]); }
      else if (d.txEnd) { const mod = rade.txEnd(); self.postMessage({ mod, end: true }, [mod.buffer]); }
    } catch (err) { self.postMessage({ error: String(err && err.message || err) }); }
  };
}
