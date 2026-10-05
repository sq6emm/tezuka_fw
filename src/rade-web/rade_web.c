/* RADE V2 for the SQTRX page: a thin streaming wrapper over the rade_c V2
 * transmitter/receiver, opus's LPCNet feature extractor and FARGAN, built to
 * a standalone WebAssembly module (build.sh) that the page runs in a Worker.
 *
 * Everything is real-valued audio, as an SSB radio sees it:
 *   RX: 8 kHz modem audio in  -> 16 kHz speech out
 *   TX: 16 kHz speech in      -> 8 kHz modem audio out
 * The caller writes its samples into the buffer returned by rw_*_in() and
 * calls rw_*_push(n); the result is in rw_*_out(), its length the return
 * value. Levels: +/-1.0 full scale on both sides.
 */
#include <math.h>
#include <string.h>

#include "rade_tx_v2.h"
#include "rade_rx_v2.h"
#include "fargan.h"
#include "lpcnet.h"
#include "lpcnet_private.h"
#include "cpu_support.h"

#define EXPORT __attribute__((visibility("default"), used))

#define MAX_IN 4096                       /* samples per push */
#define NFRAME LPCNET_FRAME_SIZE          /* 160 speech samples = 10 ms at 16 kHz */
#define NTOT RADE_V2_NB_TOTAL_FEATURES    /* 36 */
#define FRAMES RADE_V2_FRAMES_PER_STEP    /* 4 feature frames per modem frame */

/* The real modem signal at nominal amplitude: Re{tx} * 0.5 (rade_tx_wav's
 * 16384/32768, 6 dB below full scale for the peaks); the receiver takes the
 * real signal x2 (rade_rx_wav). Its AGC corrects the rest within +/-20 dB. */
#define TX_GAIN 0.5f
#define RX_GAIN 2.0f

/* ------------------------------------------------------------------ RX */
static rade_rx_v2_state rx;
static FARGANState fargan;
static int fargan_ready, cont_frames;
static float cont_buf[5 * NTOT];
static RADE_COMP rx_q[MAX_IN + 2 * RADE_V2_SYM_LEN];
static int rx_qn;
static float rx_in[MAX_IN];
static float rx_out[(MAX_IN / RADE_V2_SYM_LEN + 2) * FRAMES * NFRAME];
static int rx_eoo_count, rx_frames;
/* Coarse frequency correction set by the page (Hz, subtracted): RADE's own
 * estimate only covers +/-31 Hz (half the carrier spacing). A real input
 * mixed by a complex exponential moves its negative-frequency image away
 * too, and the receiver ignores negative frequencies. */
static float rx_shift_hz;
static double rx_ph;

/* Carrier slip: a receiver locked whole carriers away (its fine estimate
 * covers only +/-31 Hz) still "syncs" and decodes nonsense. In sync, each
 * symbol after CP removal and fine correction has the 14 carriers exactly
 * on DFT bins 17..30 of 128; average the power of bins 17-SLIP_MAX-2 ..
 * 30+SLIP_MAX+2 and find where the 14-bin block really is. The symbols are
 * taken from a copy of the input before the receiver's band-pass (which
 * is centred where the receiver thinks the signal is, and would cut a
 * block lying beside it): same buffer layout, the filter's delay back. */
#define SLIP_MAX 8
#define SLIP_LO (17 - SLIP_MAX - 2)
#define SLIP_N (14 + 2 * SLIP_MAX + 4)
static float slip_e[SLIP_N];
static int slip_syms;
static RADE_COMP pre_buf[RADE_V2_RX_BUF_SIZE];
#define BPF_DELAY ((RADE_BPF_NTAP - 1) / 2)

static void pre_append(const RADE_COMP *in, int nin) {
    memmove(pre_buf, &pre_buf[nin], sizeof(RADE_COMP) * (RADE_V2_RX_BUF_SIZE - nin));
    memcpy(&pre_buf[RADE_V2_RX_BUF_SIZE - nin], in, sizeof(RADE_COMP) * nin);
}

static void slip_update(void) {
    /* As extract_symbol(): the symbol's M samples after its CP. */
    int st = RADE_V2_SYM_LEN + (int)rx.delta_hat - RADE_V2_NCP + RADE_V2_NCP - BPF_DELAY;
    if (st < 0 || st + RADE_V2_M > RADE_V2_RX_BUF_SIZE) return;
    RADE_COMP y[RADE_V2_M];
    double om = -2.0 * M_PI * rx.freq_offset / RADE_FS;
    for (int n = 0; n < RADE_V2_M; n++) {
        double c = cos(om * n), s = sin(om * n);
        y[n].real = (float)(pre_buf[st + n].real * c - pre_buf[st + n].imag * s);
        y[n].imag = (float)(pre_buf[st + n].real * s + pre_buf[st + n].imag * c);
    }
    for (int b = 0; b < SLIP_N; b++) {
        double w = -2.0 * M_PI * (SLIP_LO + b) / RADE_V2_M, re = 0.0, im = 0.0;
        for (int n = 0; n < RADE_V2_M; n++) {
            double c = cos(w * n), s = sin(w * n);
            re += y[n].real * c - y[n].imag * s;
            im += y[n].real * s + y[n].imag * c;
        }
        slip_e[b] = 0.95f * slip_e[b] + 0.05f * (float)(re * re + im * im);
    }
    slip_syms++;
}

/* Whole carriers the signal sits away from where the receiver thinks it
 * is (0: aligned or not known yet). The block's inside-to-outside power
 * ratio is about 1 + SNR per carrier (+14 dB at -3 dB SNR in 3 kHz) when
 * aligned, and falls to about 4 one carrier off. */
static float slip_dbg[2 * SLIP_MAX + 1];
EXPORT float rw_rx_slip_ratio(int j) { return slip_dbg[j + SLIP_MAX]; }
EXPORT int rw_rx_slip(void) {
    if (slip_syms < 40) return 0;
    int best = 0;
    float bestr = 0.0f, r0 = 0.0f;
    for (int j = -SLIP_MAX; j <= SLIP_MAX; j++) {
        int a = 17 + j - SLIP_LO;
        float in = 0.0f;
        for (int c = 0; c < 14; c++) in += slip_e[a + c];
        float out = slip_e[a - 2] + slip_e[a - 1] + slip_e[a + 14] + slip_e[a + 15];
        float r = (in / 14.0f) / (out / 4.0f + 1e-20f);
        slip_dbg[j + SLIP_MAX] = r;
        if (j == 0) r0 = r;
        if (r > bestr) { bestr = r; best = j; }
    }
    return (best != 0 && bestr > 2.0f * r0 && bestr > 6.0f) ? best : 0;
}

EXPORT int rw_rx_open(void) {
    if (rade_rx_v2_init(&rx, 1) != 0) return -1;
    rx.verbose = 0;
    fargan_init(&fargan);
    fargan_ready = cont_frames = 0;
    rx_qn = 0;
    rx_eoo_count = rx_frames = 0;
    rx_ph = 0.0;
    memset(slip_e, 0, sizeof(slip_e));
    memset(pre_buf, 0, sizeof(pre_buf));
    slip_syms = 0;
    return 0;
}
EXPORT void rw_rx_set_shift(float hz) { rx_shift_hz = hz; }
EXPORT float *rw_rx_in(void) { return rx_in; }
EXPORT float *rw_rx_out(void) { return rx_out; }

/* One decoded modem frame (4 feature frames) -> speech; returns samples. */
static int synth(const float *feat, float *out) {
    int n = 0;
    for (int f = 0; f < FRAMES; f++) {
        const float *ft = &feat[f * NTOT];
        if (!fargan_ready) {
            memcpy(&cont_buf[cont_frames * NTOT], ft, sizeof(float) * NTOT);
            if (++cont_frames >= 5) {
                float packed[5 * NB_FEATURES], zeros[FARGAN_CONT_SAMPLES];
                for (int i = 0; i < 5; i++)
                    memcpy(&packed[i * NB_FEATURES], &cont_buf[i * NTOT], sizeof(float) * NB_FEATURES);
                memset(zeros, 0, sizeof(zeros));
                fargan_cont(&fargan, zeros, packed);
                fargan_ready = 1;
            }
            continue;
        }
        fargan_synthesize(&fargan, &out[n], ft);
        n += NFRAME;
    }
    return n;
}

EXPORT int rw_rx_push(int n) {
    if (n < 0 || n > MAX_IN) return -1;
    double dph = -2.0 * M_PI * rx_shift_hz / RADE_FS;
    for (int i = 0; i < n; i++) {
        float x = rx_in[i] * RX_GAIN;
        rx_q[rx_qn + i].real = x * (float)cos(rx_ph);
        rx_q[rx_qn + i].imag = x * (float)sin(rx_ph);
        rx_ph += dph;
        if (rx_ph > M_PI) rx_ph -= 2.0 * M_PI;
        if (rx_ph < -M_PI) rx_ph += 2.0 * M_PI;
    }
    rx_qn += n;
    int out = 0, pos = 0;
    float feat[RADE_V2_FEATURES_OUT];
    for (;;) {
        int nin = rade_rx_v2_nin(&rx);
        if (rx_qn - pos < nin) break;
        int was_sync = rx.state == RADE_RX_V2_SYNC;
        pre_append(&rx_q[pos], nin);
        int r = rade_rx_v2_process(&rx, feat, &rx_q[pos]);
        pos += nin;
        if (r & 0x2) rx_eoo_count++;
        if (rx.state == RADE_RX_V2_SYNC) slip_update();
        else if (slip_syms) { memset(slip_e, 0, sizeof(slip_e)); slip_syms = 0; }
        if (r & 0x1) {
            rx_frames++;
            out += synth(feat, &rx_out[out]);
        }
        if (was_sync && rx.state != RADE_RX_V2_SYNC) {
            /* Lost: the next over warms the vocoder up afresh. */
            fargan_init(&fargan);
            fargan_ready = cont_frames = 0;
        }
    }
    memmove(rx_q, &rx_q[pos], sizeof(RADE_COMP) * (rx_qn - pos));
    rx_qn -= pos;
    return out;
}
EXPORT int rw_rx_sync(void) { return rx.state == RADE_RX_V2_SYNC; }
EXPORT float rw_rx_snr(void) { return rx.snr_est_dB; }
EXPORT float rw_rx_foff(void) { return rx.freq_offset; }
EXPORT int rw_rx_eoo_count(void) { return rx_eoo_count; }
EXPORT int rw_rx_frames(void) { return rx_frames; }
/* FrameSyncNet's smoothed outputs for the two frame alignments: how much the
 * demodulated latents look like RADE frames (a lock whole carriers away
 * gives nonsense latents). */
EXPORT float rw_rx_timing(void) { return rx.delta_hat; }
EXPORT float rw_rx_fsync_even(void) { return rx.frame_sync_even; }
EXPORT float rw_rx_fsync_odd(void) { return rx.frame_sync_odd; }
EXPORT float rw_rx_data_symbol(void) { return rade_rx_v2_get_data_symbol(&rx); }

/* ------------------------------------------------------------------ TX */
static rade_tx_v2_state tx;
static LPCNetEncState *enc;
static float tx_in[MAX_IN];
static float tx_out[(MAX_IN / (FRAMES * NFRAME) + 2) * RADE_V2_NMF + RADE_V2_NEOO];
static float tx_pcm[NFRAME];
static int tx_pcm_n;
static float tx_feat[FRAMES * NTOT];
static int tx_feat_n;

EXPORT int rw_tx_open(void) {
    if (rade_tx_v2_init(&tx, 1) != 0) return -1;
    if (!enc) enc = lpcnet_encoder_create();
    else lpcnet_encoder_init(enc);
    if (!enc) return -1;
    tx_pcm_n = tx_feat_n = 0;
    return 0;
}
EXPORT float *rw_tx_in(void) { return tx_in; }
EXPORT float *rw_tx_out(void) { return tx_out; }
EXPORT void rw_tx_set_data_symbol(float s) { rade_tx_v2_set_data_symbol(&tx, s); }

static int modulate(float *out) {
    RADE_COMP iq[RADE_V2_NEOO > RADE_V2_NMF ? RADE_V2_NEOO : RADE_V2_NMF];
    int k = rade_tx_v2_process(&tx, iq, tx_feat);
    for (int i = 0; i < k; i++) out[i] = iq[i].real * TX_GAIN;
    tx_feat_n = 0;
    return k;
}

EXPORT int rw_tx_push(int n) {
    if (n < 0 || n > MAX_IN) return -1;
    int out = 0;
    for (int i = 0; i < n; i++) {
        tx_pcm[tx_pcm_n++] = tx_in[i];
        if (tx_pcm_n < NFRAME) continue;
        tx_pcm_n = 0;
        opus_int16 pcm[NFRAME];
        for (int j = 0; j < NFRAME; j++) {
            float v = tx_pcm[j] * 32768.0f;
            if (v > 32767.0f) v = 32767.0f;
            if (v < -32767.0f) v = -32767.0f;
            pcm[j] = (opus_int16)floorf(0.5f + v);
        }
        lpcnet_compute_single_frame_features(enc, pcm, &tx_feat[tx_feat_n * NTOT], opus_select_arch());
        if (++tx_feat_n == FRAMES) out += modulate(&tx_out[out]);
    }
    return out;
}

/* End of the over: the partial modem frame (zero-padded) and the EOO frame. */
EXPORT int rw_tx_eoo(void) {
    int out = 0;
    if (tx_feat_n > 0) {
        memset(&tx_feat[tx_feat_n * NTOT], 0, sizeof(float) * (FRAMES - tx_feat_n) * NTOT);
        out += modulate(&tx_out[out]);
    }
    RADE_COMP iq[RADE_V2_NEOO];
    int k = rade_tx_v2_eoo(&tx, iq);
    for (int i = 0; i < k; i++) tx_out[out + i] = iq[i].real * TX_GAIN;
    out += k;
    tx_pcm_n = 0;
    return out;
}
