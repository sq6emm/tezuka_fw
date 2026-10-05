// Round trip of rade.wasm through src/trxd/web/rade-worker.js, as the page uses it:
//   node test.js speech16k.wav [snr_db] [out16k.s16]
// 16 kHz speech -> 48 kHz "microphone" -> TX -> 12 kHz modem audio
// (+ noise in 3 kHz) -> RX -> 16 kHz speech. Prints sync, frames, SNR, speed.
"use strict";
const fs = require("fs");
const { Resampler, radeLoad } = require("../trxd/web/rade-worker.js");

function readWav16(path) {
  const b = fs.readFileSync(path);
  let p = 12;
  while (p < b.length) {
    const id = b.toString("ascii", p, p + 4), n = b.readUInt32LE(p + 4);
    if (id === "data") {
      const s = new Float32Array(n / 2);
      for (let i = 0; i < s.length; i++) s[i] = b.readInt16LE(p + 8 + 2 * i) / 32768;
      return s;
    }
    p += 8 + n + (n & 1);
  }
  throw new Error("no data chunk");
}

function gauss() { let u = 0, v = 0; while (!u) u = Math.random(); while (!v) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

(async () => {
  const [wav, snrArg, outPath] = process.argv.slice(2);
  const rade = await radeLoad(fs.readFileSync(__dirname + "/out/rade.wasm"));
  rade.monitor = false; rade.debug = [];
  const s16 = readWav16(wav), s48 = new Resampler(3, 1).process(s16);
  let t0 = process.hrtime.bigint();
  const parts = [];
  for (let p = 0; p < s48.length; p += 960) parts.push(rade.tx(s48.subarray(p, Math.min(p + 960, s48.length))));
  parts.push(rade.txEnd());
  const txSec = Number(process.hrtime.bigint() - t0) / 1e9;
  let mod = new Float32Array(parts.reduce((s, a) => s + a.length, 0)), o = 0;
  for (const a of parts) { mod.set(a, o); o += a.length; }
  // 0.5 s of silence either side, as on the air.
  const air = new Float32Array(mod.length + 12000); air.set(mod, 6000);
  const rms = Math.sqrt(mod.reduce((s, x) => s + x * x, 0) / mod.length);
  let peak = 0; for (const x of mod) peak = Math.max(peak, Math.abs(x));
  if (snrArg !== undefined && snrArg !== "") {
    // SNR in 3 kHz: noise power density so that S / (N0 * 3000) = snr.
    const ps = rms * rms, n0 = ps / Math.pow(10, +snrArg / 10) / 3000, sigma = Math.sqrt(n0 * 6000);
    for (let i = 0; i < air.length; i++) air[i] += sigma * gauss();
  }
  t0 = process.hrtime.bigint();
  const out = [], syncs = [], raw16 = [], snrs = [];
  for (let p = 0; p < air.length; p += 240) {
    out.push(rade.rx(air.subarray(p, Math.min(p + 240, air.length))));
    raw16.push(rade.last16);
    const s = rade.status();
    syncs.push(s.sync ? 1 : 0);
    if (s.sync) snrs.push(s.snr);
  }
  const rxSec = Number(process.hrtime.bigint() - t0) / 1e9;
  const pcm = new Float32Array(out.reduce((s, a) => s + a.length, 0)); o = 0;
  for (const a of out) { pcm.set(a, o); o += a.length; }
  for (const d of rade.debug) console.log("afc", JSON.stringify(d));
  const st = rade.status(), dur = s16.length / 16000;
  const firstSync = syncs.indexOf(1) * 0.02;
  console.log(JSON.stringify({
    speech_s: +dur.toFixed(2), modem_rms: +rms.toFixed(3), modem_peak: +peak.toFixed(3),
    out_s: +(pcm.length / 12000).toFixed(2), frames: st.frames, eoo: st.eoo, snr_est: snrs.length ? +(snrs.reduce((a, b) => a + b, 0) / snrs.length).toFixed(1) : null,
    acquire_s: +firstSync.toFixed(2), tx_rt: +(txSec / dur).toFixed(3), rx_rt: +(rxSec / dur).toFixed(3),
  }));
  if (outPath) {
    // 16 kHz s16 (before the 12 kHz resampler), for feature comparisons.
    const n = raw16.reduce((s, a) => s + a.length, 0), i16 = Buffer.alloc(n * 2);
    let k = 0;
    for (const a of raw16) for (const x of a) i16.writeInt16LE(Math.max(-32767, Math.min(32767, Math.round(x * 32768))), 2 * k++);
    fs.writeFileSync(outPath, i16);
  }
})();
