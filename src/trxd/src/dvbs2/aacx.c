/* DATV audio: the browser's Opus packets to AAC-LC frames, through a
 * minimal static libavcodec (package/ffmpeg-aac: the Opus decoder and the
 * AAC encoder only, and libswresample, which the Opus decoder needs).
 * Called from src/dvbs2/aac.rs; mono throughout.
 *
 * aacx_open(rate, bitrate)            the encoder at `rate` Hz (8000-24000)
 * aacx_opus(x, pkt, len, out, max)    one Opus packet -> 48 kHz samples
 * aacx_encode(x, pcm, pts, ...)       1024 samples at `rate` -> raw AAC (no
 *                                     ADTS: aac.rs adds the header), with the
 *                                     packet's PTS in samples (the encoder's
 *                                     priming delay already taken off)
 */
#include <stdint.h>
#include <string.h>
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/log.h>

typedef struct aacx {
    AVCodecContext *dec, *enc;
    AVFrame *df, *ef;
    AVPacket *pkt;
} aacx;

void aacx_close(aacx *x);

aacx *aacx_open(int rate, int bitrate) {
    av_log_set_level(AV_LOG_ERROR);
    aacx *x = av_mallocz(sizeof *x);
    if (!x) return NULL;
    const AVCodec *d = avcodec_find_decoder(AV_CODEC_ID_OPUS);
    const AVCodec *e = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!d || !e) goto fail;
    x->dec = avcodec_alloc_context3(d);
    x->enc = avcodec_alloc_context3(e);
    x->df = av_frame_alloc();
    x->ef = av_frame_alloc();
    x->pkt = av_packet_alloc();
    if (!x->dec || !x->enc || !x->df || !x->ef || !x->pkt) goto fail;
    x->dec->sample_rate = 48000;
    av_channel_layout_default(&x->dec->ch_layout, 1);
    if (avcodec_open2(x->dec, d, NULL) < 0) goto fail;
    x->enc->sample_rate = rate;
    x->enc->bit_rate = bitrate;
    x->enc->sample_fmt = AV_SAMPLE_FMT_FLTP;
    x->enc->profile = AV_PROFILE_AAC_LOW;
    x->enc->time_base = (AVRational){1, rate};
    av_channel_layout_default(&x->enc->ch_layout, 1);
    if (avcodec_open2(x->enc, e, NULL) < 0) goto fail;
    x->ef->nb_samples = x->enc->frame_size; /* 1024 */
    x->ef->format = AV_SAMPLE_FMT_FLTP;
    x->ef->sample_rate = rate;
    av_channel_layout_default(&x->ef->ch_layout, 1);
    if (av_frame_get_buffer(x->ef, 0) < 0) goto fail;
    return x;
fail:
    aacx_close(x);
    return NULL;
}

void aacx_close(aacx *x) {
    if (!x) return;
    avcodec_free_context(&x->dec);
    avcodec_free_context(&x->enc);
    av_frame_free(&x->df);
    av_frame_free(&x->ef);
    av_packet_free(&x->pkt);
    av_free(x);
}

/* Samples per encoder frame (1024 for AAC-LC). */
int aacx_frame_size(const aacx *x) { return x->enc->frame_size; }

/* Decoded samples written to out (48 kHz), or < 0 on an error (a bad packet
 * is skipped by the caller). */
int aacx_opus(aacx *x, const uint8_t *data, int len, float *out, int max) {
    x->pkt->data = (uint8_t *)data;
    x->pkt->size = len;
    int r = avcodec_send_packet(x->dec, x->pkt);
    x->pkt->data = NULL;
    x->pkt->size = 0;
    if (r < 0) return r;
    int n = 0;
    while ((r = avcodec_receive_frame(x->dec, x->df)) >= 0) {
        const float *s = (const float *)x->df->extended_data[0];
        int k = x->df->nb_samples;
        if (x->df->format == AV_SAMPLE_FMT_FLTP || x->df->format == AV_SAMPLE_FMT_FLT) {
            if (k > max - n) k = max - n;
            memcpy(out + n, s, (size_t)k * sizeof(float));
            n += k;
        }
        av_frame_unref(x->df);
    }
    return n;
}

/* One frame of `frame_size` samples in (pts in samples); raw AAC out. Returns
 * the bytes written (0: the encoder is still filling its delay line), or < 0. */
int aacx_encode(aacx *x, const float *pcm, int64_t pts, uint8_t *out, int max, int64_t *out_pts) {
    int r = av_frame_make_writable(x->ef);
    if (r < 0) return r;
    memcpy(x->ef->data[0], pcm, (size_t)x->enc->frame_size * sizeof(float));
    x->ef->pts = pts;
    r = avcodec_send_frame(x->enc, x->ef);
    if (r < 0) return r;
    r = avcodec_receive_packet(x->enc, x->pkt);
    if (r == AVERROR(EAGAIN)) return 0;
    if (r < 0) return r;
    int n = x->pkt->size;
    if (n > max) n = max;
    memcpy(out, x->pkt->data, (size_t)n);
    *out_pts = x->pkt->pts;
    av_packet_unref(x->pkt);
    return n;
}
