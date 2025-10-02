/*
 * NWA decoder
 * Copyright (c) 2025 Paul B Mahol
 *
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
#include "codec_internal.h"
#include "decode.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"

typedef struct NWAContext {
    AVClass *class;

    int compression;
    int block_size;
    int use_rle;
    int bps;
} NWAContext;

static int use_rle(const int channels, const int compression, const int bps)
{
    if (compression == 5 && channels == 1)
        return 1;

    return 0;
}

static av_cold int nwa_decode_init(AVCodecContext *avctx)
{
    NWAContext *s = avctx->priv_data;

    if (avctx->extradata_size >= 8) {
        s->compression = AV_RL32(avctx->extradata);
        s->block_size = AV_RL32(avctx->extradata+4);
    } else {
        return AVERROR_INVALIDDATA;
    }

    s->bps = avctx->bits_per_coded_sample;
    switch (s->bps) {
    case 8:
        avctx->sample_fmt = AV_SAMPLE_FMT_U8;
        break;
    case 16:
        avctx->sample_fmt = AV_SAMPLE_FMT_S16;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (avctx->ch_layout.nb_channels < 1 || avctx->ch_layout.nb_channels > 2)
        return AVERROR_INVALIDDATA;

    s->use_rle = use_rle(avctx->ch_layout.nb_channels, s->compression, s->bps);

    return 0;
}

static int nwa_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *avpkt)
{
    int ret, nb_samples, bits0, bits1, shift0, shift1;
    const int channels = avctx->ch_layout.nb_channels;
    NWAContext *s = avctx->priv_data;
    const int compression = s->compression;
    GetBitContext gbit, *gb = &gbit;
    const int use_rle = s->use_rle;
    int runlen = 0, ch_idx = 0;
    const int bps = s->bps;
    int16_t *dst16;
    uint8_t *dst8;
    int16_t d[2];

    ret = init_get_bits8(gb, avpkt->data, avpkt->size);
    if (ret < 0)
        return ret;

    frame->nb_samples = nb_samples = s->block_size / (bps/8);
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int ch = 0; ch < channels; ch++)
        d[ch] = get_sbits(gb, bps);

    dst16 = (int16_t *)frame->data[0];
    dst8  = frame->data[0];

    if (compression >= 3) {
        bits0 = 8;
        bits1 = compression + 3;
        shift0 = 9;
        shift1 = 1;
    } else {
        bits0 = 8 - compression;
        bits1 = 5 - compression;
        shift0 = 9 + compression;
        shift1 = 2 + compression;
    }

    for (int n = 0; n < nb_samples * channels; n++) {
        if (get_bits_left(gb) < 0)
            return AVERROR_INVALIDDATA;

        if (channels == 2)
            ch_idx = n & 1;

        if (runlen <= 0) {
            const int type = get_bits(gb, 3);

            if (type == 7) {
                if (get_bits1(gb)) {
                    d[ch_idx] = 0;
                } else {
                    const int mask1 = 1 << (bits0 - 1);
                    const int mask2 = mask1 - 1;
                    const int b = get_bits(gb, bits0);
                    const int sgn = (b & mask1) ? -1 : 1;

                    d[ch_idx] += sgn * (b & mask2) * (1 << shift0);
                }
            } else if (type != 0) {
                const int shift = shift1 + type;
                const int mask1 = 1 << (bits1 - 1);
                const int mask2 = mask1 - 1;
                const int b = get_bits(gb, bits1);
                const int sgn = (b & mask1) ? -1 : 1;

                d[ch_idx] += sgn * (b & mask2) * (1 << shift);
            } else if (use_rle) {
                if (get_bits1(gb)) {
                    runlen = get_bits(gb, 2);
                    if (runlen == 3)
                        runlen = get_bits(gb, 8);
                }
            }
        } else {
            runlen--;
        }

        if (bps == 16)
            dst16[n] = d[ch_idx];
        else
            dst8[n] = av_clip_uint8(d[ch_idx] + 128);
    }

    *got_frame_ptr = 1;

    return avpkt->size;
}

const FFCodec ff_nwa_decoder = {
    .p.name         = "nwa",
    CODEC_LONG_NAME("Visual Arts NWA"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_NWA,
    .priv_data_size = sizeof(NWAContext),
    .init           = nwa_decode_init,
    FF_CODEC_DECODE_CB(nwa_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_U8, AV_SAMPLE_FMT_S16),
};
