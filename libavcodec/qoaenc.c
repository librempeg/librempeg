/*
 * This file is part of Librempeg
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Librempeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"

#define QOA_SLICE_LEN 20
#define QOA_LMS_LEN 4

typedef struct QOAChannel {
    int16_t history[QOA_LMS_LEN];
    int weights[QOA_LMS_LEN];
} QOAChannel;

typedef struct QOAContext {
    QOAChannel ch[255];
    int init_history;
} QOAContext;

static const uint8_t qoa_quant_tab[17] = {
    7, 7, 7, 5, 5, 3, 3, 1, 0, 0, 2, 2, 4, 4, 6, 6, 6,
};

static const int16_t qoa_dequant_tab[16][8] = {
    {   1,    -1,    3,    -3,    5,    -5,     7,     -7},
    {   5,    -5,   18,   -18,   32,   -32,    49,    -49},
    {  16,   -16,   53,   -53,   95,   -95,   147,   -147},
    {  34,   -34,  113,  -113,  203,  -203,   315,   -315},
    {  63,   -63,  210,  -210,  378,  -378,   588,   -588},
    { 104,  -104,  345,  -345,  621,  -621,   966,   -966},
    { 158,  -158,  528,  -528,  950,  -950,  1477,  -1477},
    { 228,  -228,  760,  -760, 1368, -1368,  2128,  -2128},
    { 316,  -316, 1053, -1053, 1895, -1895,  2947,  -2947},
    { 422,  -422, 1405, -1405, 2529, -2529,  3934,  -3934},
    { 548,  -548, 1828, -1828, 3290, -3290,  5117,  -5117},
    { 696,  -696, 2320, -2320, 4176, -4176,  6496,  -6496},
    { 868,  -868, 2893, -2893, 5207, -5207,  8099,  -8099},
    {1064, -1064, 3548, -3548, 6386, -6386,  9933,  -9933},
    {1286, -1286, 4288, -4288, 7718, -7718, 12005, -12005},
    {1536, -1536, 5120, -5120, 9216, -9216, 14336, -14336},
};

static const int32_t qoa_reciprocal_tab[16] = {
    65536, 9363, 3121, 1457, 781, 475, 311, 216, 156, 117, 90, 71, 57, 47, 39, 32
};

static av_cold int qoa_encode_init(AVCodecContext *avctx)
{
    const int nb_channels = avctx->ch_layout.nb_channels;

    if (nb_channels > 255) {
        av_log(avctx, AV_LOG_ERROR, "Invalid number of channels\n");
        return AVERROR(EINVAL);
    }
    avctx->frame_size = FFMIN((((1<<16)-1)-8-QOA_LMS_LEN*4*nb_channels) / (QOA_SLICE_LEN*8), 256) * QOA_SLICE_LEN;

    return 0;
}

static ptrdiff_t qoa_encode_header(AVCodecContext *avctx, QOAContext *s,
                                   int nb_samples, uint32_t fsize,
                                   uint8_t *buf, int bufsize)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    uint8_t *orig = buf;

    bytestream_put_byte(&buf, avctx->ch_layout.nb_channels);
    bytestream_put_be24(&buf, avctx->sample_rate);
    bytestream_put_be16(&buf, nb_samples);
    bytestream_put_be16(&buf, fsize);

    for (int ch = 0; ch < nb_channels; ch++) {
        QOAChannel *qch = &s->ch[ch];

        for (int n = 0; n < QOA_LMS_LEN; n++)
            bytestream_put_be16(&buf, qch->history[n]);
        for (int n = 0; n < QOA_LMS_LEN; n++)
            bytestream_put_be16(&buf, qch->weights[n]);
    }

    return buf - orig;
}

static int qoa_lms_predict(QOAChannel *lms)
{
    int prediction = 0;
    for (int i = 0; i < QOA_LMS_LEN; i++)
        prediction += (int32_t)lms->weights[i] * lms->history[i];
    prediction >>= 13;
    return prediction;
}

static void qoa_lms_update(QOAChannel *lms, int sample, int residual)
{
    int delta = residual >> 4;
    for (int i = 0; i < QOA_LMS_LEN; i++) {
        lms->weights[i] += lms->history[i] < 0 ? -delta : delta;
        av_assert2(lms->weights[i] <= INT16_MAX);
        av_assert2(lms->weights[i] >= INT16_MIN);
    }
    memmove(lms->history, lms->history+1, (QOA_LMS_LEN-1) * sizeof(*lms->history));
    lms->history[QOA_LMS_LEN-1] = av_clip_int16(sample);
}

static inline int qoa_div(int v, int scalefactor)
{
    int64_t reciprocal = qoa_reciprocal_tab[scalefactor];
    int n = (v * reciprocal + (1 << 15)) / (1 << 16);
    n = n + ((v > 0) - (v < 0)) - ((n > 0) - (n < 0)); /* round away from 0 */
    return n;
}

static int qoa_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                            const AVFrame *frame, int *got_packet_ptr)
{
    QOAContext *s = avctx->priv_data;
    const int nb_channels = avctx->ch_layout.nb_channels;
    int prev_scalefactor[255] = {0};
    int64_t out_size;
    uint8_t *dst;
    int ret;

    out_size = 8 + QOA_LMS_LEN * 4 * nb_channels + nb_channels *
        8LL * ((frame->nb_samples + QOA_SLICE_LEN - 1) / QOA_SLICE_LEN);
    if (out_size > UINT16_MAX)
        return AVERROR(EINVAL);

    if ((ret = ff_get_encode_buffer(avctx, avpkt, out_size, 0)) < 0)
        return ret;
    dst = avpkt->data;

    if (!s->init_history) {
        for (int ch = 0; ch < nb_channels; ch++) {
            const int16_t *samples = (const int16_t *)frame->extended_data[ch];
            QOAChannel *qch = &s->ch[ch];

            for (int n = 0; n < QOA_LMS_LEN; n++) {
                qch->history[QOA_LMS_LEN-1-n] = samples[n];
                qch->weights[0] = qch->weights[1] = 0;
                qch->weights[2] = -(1<<13);
                qch->weights[3] =  (1<<14);
            }
        }
        s->init_history = 1;
    }

    dst += qoa_encode_header(avctx, s, frame->nb_samples,
                             out_size, dst, avpkt->size);

    for (int sample_index = 0; sample_index < frame->nb_samples;
         sample_index += QOA_SLICE_LEN) {
        for (int ch = 0; ch < nb_channels; ch++) {
            const int16_t *samples = (const int16_t *)frame->extended_data[ch];
            QOAChannel *lms = &s->ch[ch];
            int slice_len = FFMIN(QOA_SLICE_LEN, frame->nb_samples - sample_index);
            int slice_start = sample_index;
            int slice_end = sample_index + slice_len;
            uint64_t best_error = UINT64_MAX, best_slice = 0;
            QOAChannel best_lms = {0};
            int best_scalefactor = 0;

            for (int sfi = 0; sfi < 16; sfi++) {
                int scalefactor = (sfi + prev_scalefactor[ch]) & 15;
                uint64_t slice = scalefactor;
                uint64_t current_error = 0;
                QOAChannel new_lms = *lms;

                for (int si = slice_start; si < slice_end; si++) {
                    int sample = samples[si];
                    int predicted = qoa_lms_predict(&new_lms);
                    int residual = sample - predicted;
                    int scaled = qoa_div(residual, scalefactor);
                    int clamped = av_clip(scaled, -8, 8);
                    int quantized = qoa_quant_tab[clamped + 8];
                    int dequantized = qoa_dequant_tab[scalefactor][quantized];
                    int reconstructed = predicted + dequantized;
                    int error = sample - reconstructed;

                    current_error += (int64_t)error * error;
                    if (current_error > best_error)
                        break;

                    qoa_lms_update(&new_lms, reconstructed, dequantized);
                    slice = (slice << 3) | quantized;
                }

                if (current_error < best_error) {
                    best_error = current_error;
                    best_slice = slice;
                    best_lms = new_lms;
                    best_scalefactor = scalefactor;
                }
            }

            prev_scalefactor[ch] = best_scalefactor;

            memcpy(lms, &best_lms, sizeof(*lms));

            best_slice <<= (QOA_SLICE_LEN - slice_len) * 3;
            bytestream_put_be64(&dst, best_slice);
        }
    }

    *got_packet_ptr = 1;

    return 0;
}

const FFCodec ff_qoa_encoder = {
    .p.name         = "qoa",
    CODEC_LONG_NAME("QOA (Quite OK Audio)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_QOA,
    .p.capabilities = AV_CODEC_CAP_DR1 |
                      AV_CODEC_CAP_SMALL_LAST_FRAME,
    .priv_data_size = sizeof(QOAContext),
    .init           = qoa_encode_init,
    FF_CODEC_ENCODE_CB(qoa_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};
