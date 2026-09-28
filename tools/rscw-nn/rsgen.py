"""Synthetic rain-scatter / tropo CW at 12 kHz with frame labels, and the
features trxd computes (rscw.rs): STFT 256, hop 64, sin^2 window, bins
200..3000 Hz, log power minus a per-bin log-domain quantile noise tracker."""
import numpy as np

import os
FS = 12000
N = 256
HOP = int(os.environ.get('HOP', '64'))
LO, HI = 4, 64            # bins, as rscw.rs: round(200/12000*256), round(3000/12000*256)
NB = HI - LO + 1
Q = 0.25
WARM = 64      # frames for the noise floor's start
FSK = float(os.environ.get('FSK', '0'))   # share of FSK beacons (0: they confuse on-off keying)
SNR_LO = float(os.environ.get('SNR_LO', '-14'))
SNR_HI = float(os.environ.get('SNR_HI', '14'))
NOISEMOD = float(os.environ.get('NOISEMOD', '0'))   # share with a fluctuating noise level
ETA = 0.01 * HOP / 64     # tracker step (log domain), the same per second at any hop

MORSE = {
    'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 'F': '..-.', 'G': '--.', 'H': '....', 'I': '..',
    'J': '.---', 'K': '-.-', 'L': '.-..', 'M': '--', 'N': '-.', 'O': '---', 'P': '.--.', 'Q': '--.-', 'R': '.-.',
    'S': '...', 'T': '-', 'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-', 'Y': '-.--', 'Z': '--..',
    '1': '.----', '2': '..---', '3': '...--', '4': '....-', '5': '.....', '6': '-....', '7': '--...',
    '8': '---..', '9': '----.', '0': '-----', '/': '-..-.', '?': '..--..', '=': '-...-', '+': '.-.-.',
}
L = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
D = '0123456789'
WORDS = ['CQ', 'DE', 'TNX', 'TKS', '73', 'QSO', 'K', 'BK', 'FB', 'UR', 'GM', 'GA', 'GE', 'TEST', 'QRZ', 'RST',
         '599', '55S', '59S', '5NN', 'R', 'RR', 'OK', 'ES', 'HR', 'QTH', 'NAME', 'OP', 'PSE', 'AGN', 'SK', 'AR', 'BEACON']


def callsign(r):
    p = r.choice([1, 2, 2, 2])
    s = ''.join(r.choice(list(L + D if i else L)) for i in range(p))
    if r.random() < 0.3:
        s = r.choice(list(D)) + r.choice(list(L)) + r.choice(list(L))[:r.integers(0, 2)]
    s += r.choice(list(D)) + ''.join(r.choice(list(L)) for _ in range(r.integers(1, 4)))
    if r.random() < 0.15:
        s += '/' + r.choice(['P', 'B', 'M', 'QRP'])
    return s


def locator(r):
    return (r.choice(list('ABCDEFGHIJKLMNOPQR')) + r.choice(list('ABCDEFGHIJKLMNOPQR')) + r.choice(list(D))
            + r.choice(list(D)) + r.choice(list('ABCDEFGHIJKLMNOPQRSTUVWX')).lower().upper()
            + r.choice(list('ABCDEFGHIJKLMNOPQRSTUVWX')))


def text(r):
    out = []
    while sum(len(w) + 1 for w in out) < 60:
        k = r.random()
        if k < 0.35:
            out.append(callsign(r))
        elif k < 0.5:
            out.append(locator(r)[:r.choice([4, 6, 6])])
        elif k < 0.85:
            out.append(r.choice(WORDS))
        else:
            out.append(''.join(r.choice(list(L + D)) for _ in range(r.integers(1, 6))))
    return ' '.join(out)


def keying(r, t_len, wpm):
    """Key-down envelope (0/1 per sample) of random text, jittered timing."""
    dot = 1.2 / wpm * FS
    dash_w = r.uniform(2.6, 3.6)
    el_gap = r.uniform(0.8, 1.2)
    ch_gap = r.uniform(2.6, 3.8)
    wd_gap = r.uniform(5.5, 9.0)
    jit = r.uniform(0.03, 0.18)
    key = np.zeros(t_len, np.float32)
    pos = int(r.uniform(-4, 1.0) * FS)      # may start mid-message
    for w in text(r).split(' '):
        for c in w:
            code = MORSE[c]
            for i, e in enumerate(code):
                n = dot * (1 if e == '.' else dash_w) * (1 + jit * r.standard_normal())
                n = int(max(0.4 * dot, n))
                a, b = max(pos, 0), min(pos + n, t_len)
                if b > a:
                    key[a:b] = 1
                pos += n
                if i + 1 < len(code):
                    pos += int(max(0.3, el_gap * (1 + jit * r.standard_normal())) * dot)
            pos += int(max(1.5, ch_gap * (1 + jit * r.standard_normal())) * dot)
            if pos >= t_len:
                return key
        pos += int(max(3.0, wd_gap * (1 + 0.1 * r.standard_normal())) * dot) - int(ch_gap * dot)
        if r.random() < 0.08:            # a pause between overs
            pos += int(r.uniform(1, 4) * FS)
        if pos >= t_len:
            return key
    return key


def colored(r, n, f0, sigma_f):
    """Complex Gaussian process with a Gaussian spectrum (std sigma_f Hz) at f0."""
    m = 1 << int(np.ceil(np.log2(n)))
    f = np.fft.fftfreq(m, 1 / FS)
    h = np.exp(-0.5 * ((f - 0.0) / max(sigma_f, 0.3)) ** 2)
    x = np.fft.ifft(np.fft.fft(r.standard_normal(m) + 1j * r.standard_normal(m)) * h)[:n]
    x /= np.sqrt(np.mean(np.abs(x) ** 2)) + 1e-12
    drift = np.cumsum(r.standard_normal(n)) * r.uniform(0, 0.02)     # slow random walk (Hz)
    ph = 2 * np.pi * np.cumsum(f0 + drift) / FS
    return x * np.exp(1j * ph)


def example(r, secs=8.0):
    n = int(secs * FS)
    wpm = r.uniform(8, 30)
    key = keying(r, n, wpm)
    # soft edges
    ramp = int(r.uniform(0.002, 0.008) * FS)
    k = np.convolve(key, np.hanning(2 * ramp + 1) / (ramp + 1), 'same').clip(0, 1)
    kind = r.random()
    f0 = r.uniform(350, 2300)
    if kind < 0.6:                                  # rain scatter: spread signal
        sig = colored(r, n, f0, r.uniform(20, 450))
    elif kind < 0.85:                               # tropo / aircraft: narrow fading
        sig = colored(r, n, f0, r.uniform(0.3, 8))
        sig = 0.6 * sig + r.uniform(0, 1) * np.exp(2j * np.pi * f0 * np.arange(n) / FS)
    elif kind < 1.0 - FSK:                          # steady tone (beacon)
        sig = np.exp(2j * np.pi * (f0 + np.cumsum(r.standard_normal(n)) * 0.002) * np.arange(n) / FS)
    else:                                           # FSK beacon: the higher tone is key down
        shift = r.uniform(80, 900)
        f0 = max(f0, 300 + shift)
        fading = r.random() < 0.5
        a = colored(r, n, f0, r.uniform(0.3, 5)) if fading else np.exp(2j * np.pi * f0 * np.arange(n) / FS)
        b = colored(r, n, f0 - shift, r.uniform(0.3, 5)) if fading else np.exp(2j * np.pi * (f0 - shift) * np.arange(n) / FS)
        sig = a * k + b * (1 - k)
        k = np.ones_like(k)                        # the power is always there
    s = np.real(sig) * k
    # noise: white shaped like an SSB receiver's passband, a random tilt
    m = 1 << int(np.ceil(np.log2(n)))
    f = np.abs(np.fft.rfftfreq(m, 1 / FS))
    lo, hi = r.uniform(150, 400), r.uniform(2400, 3200)
    shape = 1 / (1 + ((lo / np.maximum(f, 1)) ** 8)) / (1 + (f / hi) ** 12)
    shape *= 10 ** (r.uniform(-0.4, 0.4) * (f - 1500) / 1500)
    noise = np.fft.irfft(np.fft.rfft(r.standard_normal(m)) * shape)[:n]
    pn = np.mean(noise ** 2) * (2500 / (hi - lo))
    if r.random() < NOISEMOD:                       # noise bursts / AGC pumping on noise
        tc = r.uniform(0.02, 0.2) * FS                # correlation time
        m = int(n / tc) + 3
        env = np.exp(r.normal(0, r.uniform(0.3, 1.0), m))
        env = np.interp(np.arange(n) / tc, np.arange(m), env)
        noise = noise * env / np.sqrt(np.mean(env ** 2))
    snr_db = r.uniform(SNR_LO, SNR_HI)             # key-down power vs noise in 2.5 kHz
    s *= np.sqrt(pn * 10 ** (snr_db / 10) / (np.mean(np.real(sig) ** 2) + 1e-12))
    x = s + noise
    if r.random() < 0.4:                            # clicks / static crashes
        for _ in range(r.poisson(r.uniform(1, 20) * secs)):
            i = r.integers(0, n - 200)
            ln = r.integers(5, 150)
            x[i:i + ln] += r.standard_normal(ln) * np.sqrt(pn) * r.uniform(2, 30)
    if r.random() < 0.25:                           # another, weaker signal (QRM)
        k2 = keying(r, n, r.uniform(10, 30))
        q = np.real(colored(r, n, r.uniform(350, 2300), r.uniform(0.3, 200))) * k2
        x += q * np.sqrt(pn * 10 ** ((snr_db - r.uniform(6, 20)) / 10))
    if r.random() < 0.4:                            # receiver AGC (fast attack, slow decay)
        env = np.abs(x)
        g = np.empty(n)
        a, dcy = 0.0, np.exp(-1 / (r.uniform(0.05, 0.6) * FS))
        att = np.exp(-1 / (0.003 * FS))
        # vectorised enough: block of 64
        e = env.reshape(-1, 64).max(1) if n % 64 == 0 else np.pad(env, (0, 64 - n % 64)).reshape(-1, 64).max(1)
        out = np.empty_like(e)
        for i, v in enumerate(e):
            a = v if v > a else a * dcy ** 64
            out[i] = a
        g = np.repeat(1 / (out + np.sqrt(pn) * r.uniform(0.5, 3)), 64)[:n]
        x = x * g
    x = x / (np.max(np.abs(x)) + 1e-9) * 0.5
    if r.random() < 0.3:                            # silence / fade-in at the start
        m0 = int(r.uniform(0, 0.05) * FS)
        x[:m0] = 0
        fi = int(r.uniform(0, 0.05) * FS)
        x[m0:m0 + fi] *= np.linspace(0, 1, len(x[m0:m0 + fi]))
    if r.random() < 0.3:                            # 16-bit / codec-ish quantisation
        x = np.round(x * 2 ** r.integers(8, 15)) / 2 ** r.integers(8, 15)
    return x.astype(np.float32), key


def features(x):
    """(frames, NB) log power minus the per-bin noise tracker, as rscw.rs."""
    nf = (len(x) - N) // HOP + 1
    w = np.sin(np.pi * np.arange(N) / N) ** 2
    idx = np.arange(N)[None, :] + HOP * np.arange(nf)[:, None]
    X = np.fft.fft(x[idx] * w, axis=1)[:, LO:HI + 1]
    lp = np.log(np.maximum(np.abs(X) ** 2, 1e-20)).astype(np.float32)
    out = np.empty_like(lp)
    # start: each bin's Q quantile over the first WARM frames (a partly
    # silent first frame must not set the floor), then the tracker
    nl = np.quantile(lp[:WARM], Q, axis=0).astype(np.float32)
    for t in range(nf):
        if t >= WARM:
            below = lp[t] < nl
            nl = np.where(below, nl - ETA * (1 - Q), nl + ETA * Q)
        out[t] = lp[t] - nl
    return np.clip(out, -6, 12)


def labels(key, nf):
    c = HOP * np.arange(nf) + N // 2
    return key[np.minimum(c, len(key) - 1)]
