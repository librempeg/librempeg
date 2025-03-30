/*
 * PCA decoder
 * Copyright (c) 2025 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "bytestream.h"
#include "decode.h"
#include "get_bits.h"
#include "unary.h"

typedef struct PCAChannel {
    int32_t coef[16384+4];
    int32_t lpc[4];
    uint8_t rice_code;
    uint8_t filter_order;
} PCAChannel;

typedef struct PCAContext {
    AVClass *class;

    int bps;
    int ms;
    int lpc;
    int crc;

    PCAChannel ch[2];
} PCAContext;

static av_cold int pca_decode_init(AVCodecContext *avctx)
{
    PCAContext *s = avctx->priv_data;

    if (avctx->extradata_size >= 5) {
        s->ms  = avctx->extradata[4] & 1;
        s->lpc = avctx->extradata[4] & 2;
        s->crc = avctx->extradata[4] & 4;
    }

    s->bps = avctx->bits_per_coded_sample;
    switch (s->bps) {
    case 16:
        avctx->sample_fmt = AV_SAMPLE_FMT_S16P;
        break;
    case 24:
        avctx->sample_fmt = AV_SAMPLE_FMT_S32P;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (avctx->ch_layout.nb_channels < 1 || avctx->ch_layout.nb_channels > 2)
        return AVERROR_INVALIDDATA;

    return 0;
}

static uint32_t get_urice(GetBitContext *gb, int k)
{
    uint32_t x = get_unary(gb, 1, get_bits_left(gb));
    uint32_t y = get_bits_long(gb, k);
    uint32_t z = (x << k) | y;

    return z;
}

static int32_t get_srice(GetBitContext *gb, int k)
{
    uint32_t z = get_urice(gb, k);
    int sign = z & 1;

    z = sign ? z+1 : z;
    z /= 2;

    return sign ? -z : z;
}

static int pca_decode_channel(AVCodecContext *avctx, const int ch,
                              const int nb_samples, AVFrame *frame,
                              GetBitContext *gb)
{
    PCAContext *s = avctx->priv_data;
    PCAChannel *pc = &s->ch[ch];
    const int rice_code = pc->rice_code;
    const int32_t *lpc = pc->lpc;
    int32_t *coef = pc->coef + 4;

    switch (pc->filter_order) {
    case 5:
        for (int n = 0; n < nb_samples; n++)
            coef[n] = 0;
        break;
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
        for (int n = 0; n < nb_samples; n++)
            coef[n] = get_srice(gb, rice_code);
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (s->lpc) {
        for (int n = 4; n < nb_samples; n++)
            coef[n] -= (coef[n-1] * lpc[0] + coef[n-2] * lpc[1] + coef[n-3] * lpc[2] + coef[n-4] * lpc[3]) >> 5;
    }

    switch (pc->filter_order) {
    case 1:
        for (int n = 0; n < nb_samples; n++)
            coef[n] += coef[n-1];
        break;
    case 2:
        for (int n = 0; n < nb_samples; n++)
            coef[n] += coef[n-1] * 2 - coef[n-2];
        break;
    case 3:
        for (int n = 0; n < nb_samples; n++)
            coef[n] += coef[n-1] * 3 - coef[n-2] * 3 + coef[n-3];
        break;
    case 4:
        for (int n = 0; n < nb_samples; n++)
            coef[n] += coef[n-1] * 4 - coef[n-2] * 6 + coef[n-3] * 4 - coef[n-4];
        break;
    default:
        break;
    }

    if (!s->ms || avctx->ch_layout.nb_channels == 1) {
        switch (avctx->sample_fmt) {
        case AV_SAMPLE_FMT_S16P:
            {
                int16_t *dst = (int16_t *)frame->extended_data[ch];

                for (int n = 0; n < nb_samples; n++)
                    dst[n] = coef[n] & 0xffff;
            }
            break;
        case AV_SAMPLE_FMT_S32P:
            {
                int32_t *dst = (int32_t *)frame->extended_data[ch];

                for (int n = 0; n < nb_samples; n++)
                    dst[n] = av_clip_intp2(coef[n], 23);
            }
            break;
        default:
            return AVERROR_BUG;
        }
    }

    memcpy(pc->coef, coef + nb_samples - 4, sizeof(int32_t) * 4);

    return 0;
}

static void decorrelate(int32_t M, int32_t S, uint32_t *L, uint32_t *R)
{
    uint32_t l, r;

    r = M * 2 - S;
    *L = M * 2 + S;
    *R = r;
    l = *L;
    if ((((int)l < 1) || (-1 < (int)r)) && ((-1 < (int)l || ((int)r < 1)))) {
        *L = (int)l / 2;
        l = *R;
    } else {
        if (((M < 1) || (S < 1)) && ((-1 < M || (-1 < S)))) {
            *R = (int)r / 2;
            l = *L;
            if ((l & 1) == 0) {
                *L = (int)l / 2;
                return;
            }
            if ((int)l < 1) {
                *L = (int)l / 2 - 1;
                return;
            }
            *L = (int)l / 2 + 1;
            return;
        }
        *L = (int)l / 2;
        l = *R;
        if ((l & 1) != 0) {
            if ((int)l < 1) {
                *R = (int)l / 2 - 1;
                return;
            }
            *R = (int)l / 2 + 1;
            return;
        }
    }
    *R = (int)l / 2;
}

static int pca_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *avpkt)
{
    PCAContext *s = avctx->priv_data;
    GetBitContext gbit;
    GetByteContext gbc;
    GetByteContext *gb = &gbc;
    int ret, frame_size;
    int16_t nb_samples;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    if (bytestream2_peek_le32(gb) == 0xffffffff) {
        bytestream2_skip(gb, 4);

        for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
            PCAChannel *pc = &s->ch[ch];

            for (int n = 0; n < 4; n++)
                pc->coef[4-n-1] = bytestream2_get_le32(gb);
        }
    }

    nb_samples = bytestream2_get_le16(gb);
    if (nb_samples > FF_ARRAY_ELEMS(s->ch[0].coef)-4)
        return AVERROR_INVALIDDATA;

    frame_size = bytestream2_get_le16(gb);

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        PCAChannel *pc = &s->ch[ch];

        pc->rice_code = bytestream2_get_byte(gb);
    }

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        PCAChannel *pc = &s->ch[ch];

        pc->filter_order = bytestream2_get_byte(gb);
    }

    for (int ch = 0; ch < avctx->ch_layout.nb_channels && s->lpc; ch++) {
        PCAChannel *pc = &s->ch[ch];

        for (int n = 0; n < 4; n++) {
            pc->lpc[n] = bytestream2_get_byte(gb);
            if (pc->lpc[n] & 0x80)
                pc->lpc[n] = -(pc->lpc[n] & 0x7f);
        }
    }

    frame->nb_samples = FFABS(nb_samples);
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if (bytestream2_get_bytes_left(gb) < frame_size)
        return AVERROR_INVALIDDATA;

    ret = init_get_bits8(&gbit, gb->buffer, frame_size);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        ret = pca_decode_channel(avctx, ch, nb_samples, frame, &gbit);
        if (ret < 0)
            return ret;
    }

    if (s->ms && avctx->ch_layout.nb_channels == 2) {
        switch (avctx->sample_fmt) {
        case AV_SAMPLE_FMT_S16P:
            {
                int16_t *l = (int16_t *)frame->extended_data[0];
                int16_t *r = (int16_t *)frame->extended_data[1];
                int32_t *M = s->ch[0].coef + 4;
                int32_t *S = s->ch[1].coef + 4;

                for (int n = 0; n < nb_samples; n++) {
                    int32_t L, R;

                    decorrelate(M[n], S[n], &L, &R);

                    l[n] = L;
                    r[n] = R;
                }
            }
            break;
        case AV_SAMPLE_FMT_S32P:
            {
                int32_t *l = (int32_t *)frame->extended_data[0];
                int32_t *r = (int32_t *)frame->extended_data[1];
                int32_t *M = s->ch[0].coef + 4;
                int32_t *S = s->ch[1].coef + 4;

                for (int n = 0; n < nb_samples; n++)
                    decorrelate(M[n], S[n], &l[n], &r[n]);
            }
            break;
        default:
            return AVERROR_BUG;
        }
    }

    bytestream2_skip(gb, frame_size);

    *got_frame_ptr = 1;

    return bytestream2_tell(gb);
}

const FFCodec ff_pca_decoder = {
    .p.name         = "pca",
    CODEC_LONG_NAME("PCA (Perfect Clarity Audio)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_PCA,
    .priv_data_size = sizeof(PCAContext),
    .init           = pca_decode_init,
    FF_CODEC_DECODE_CB(pca_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P),
};
