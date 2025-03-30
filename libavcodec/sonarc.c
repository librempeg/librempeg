/*
 * SONARC audio decoder
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

#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/reverse.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"
#include "bytestream.h"
#include "mathops.h"
#include "sonarc_data.h"

typedef struct SonarcContext {
    AVClass *av_class;

    int flags;
    int channels;

    uint8_t input[65536];

    int16_t residuals16[2][4096+256];
    int16_t samples16[4096];
    int16_t history16[2][256];

    int8_t  residuals[2][4096+256];
    int8_t  samples[4096];
    int8_t  history[2][256];

    int16_t factors[256];
} SonarcContext;

static av_cold int sonarc_init(AVCodecContext *avctx)
{
    SonarcContext *s = avctx->priv_data;

    if (avctx->extradata_size < 2)
        return AVERROR_INVALIDDATA;

    s->channels = avctx->ch_layout.nb_channels;
    if (s->channels < 1 || s->channels > 2)
        return AVERROR_INVALIDDATA;

    avctx->sample_fmt = avctx->bits_per_coded_sample >= 16 ? AV_SAMPLE_FMT_S16P : AV_SAMPLE_FMT_U8P;
    s->flags = AV_RL16(avctx->extradata);

    for (int n = 0; n < 256; n++)
        s->history[0][n] = s->history[1][n] = -128;

    return 0;
}

static int decode_residue(AVCodecContext *avctx,
                          int mode, int nb_samples, int flags,
                          GetBitContext *gb, int8_t *dst)
{
    int bits, value, idx;

    if (mode & 0x80)
        mode &= 0x7f;

    for (int n = 0; n < nb_samples; n++) {
        if (get_bits_left(gb) < 0) {
            for (int i = n; i < nb_samples; i++)
                dst[n] = 0x0;
            break;
        }

        switch (mode) {
        case 0:
            idx = show_bits(gb, 8);
            bits = cb_bits0[idx];

            if (bits == 9) {
                skip_bits(gb, 8);
                idx = show_bits(gb, 8);
                bits = cb_bits0_long0[idx];
                value = cb_codes0_long0[idx];
            } else if (bits == 10) {
                skip_bits(gb, 8);
                idx = show_bits(gb, 8);
                bits = cb_bits0_long1[idx];
                value = cb_codes0_long1[idx];
            } else if (bits == 11) {
                skip_bits(gb, 8);
                idx = show_bits(gb, 8);
                bits = cb_bits0_long2[idx];
                value = cb_codes0_long2[idx];
            } else if (bits == 12) {
                skip_bits(gb, 8);
                idx = show_bits(gb, 8);
                bits = cb_bits0_long3[idx];
                value = cb_codes0_long3[idx];
            } else {
                value = cb_codes0[idx];
            }

            skip_bits(gb, bits);
            dst[n] = value;
            break;
        case 1:
            idx = show_bits(gb, 8);
            bits = cb_bits1[idx];

            if (bits == 9) {
                bits = cb_bits1_long0[idx];
                value = cb_codes1_long0[idx];
            } else if (bits == 10) {
                bits = cb_bits1_long1[idx];
                value = cb_codes1_long1[idx];
            } else {
                value = cb_codes1[idx];
            }

            skip_bits(gb, bits);
            dst[n] = value;
            break;
        case 2:
            idx = show_bits(gb, 8);
            bits = cb_bits2[idx];

            if (bits == 9) {
                bits = cb_bits2_long0[idx];
                value = cb_codes2_long0[idx];
            } else if (bits == 10) {
                bits = cb_bits2_long1[idx];
                value = cb_codes2_long1[idx];
            } else if (bits == 11) {
                bits = cb_bits2_long2[idx];
                value = cb_codes2_long2[idx];
            } else if (bits == 12) {
                bits = cb_bits2_long3[idx];
                value = cb_codes2_long3[idx];
            } else {
                value = cb_codes2[idx];
            }

            skip_bits(gb, bits);
            dst[n] = value;
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        case 7:
            break;
        case 8:
            break;
        case 15:
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "Invalid mode: %d\n", mode);
            return AVERROR_INVALIDDATA;
        }
    }

    return 0;
}

static void decode_lpc(int order, int nb_samples, int8_t *dest,
                       const int16_t *factors, uint8_t *dst)
{
    if (order <= 0)
        return;

    dest -= order;
    for (int n = 0; n < nb_samples; n++) {
        int8_t *loopdest = dest++;
        int accum = 0;

        for (int j = order - 1; j >= 0; j--) {
            int value = sign_extend((uint8_t)((loopdest[0]) ^ 0x80) & 0xff, 8);

            accum += value * factors[j];
            loopdest++;
        }

        accum += 0x00000800;
        accum = loopdest[0] - (int)((accum >> 12) & 0xFF);
        loopdest[0] = dst[n] = accum & 0xFF;
    }
}

static int decode_residue16(AVCodecContext *avctx,
                            int mode, int nb_samples, int flags,
                            GetBitContext *gb, int16_t *dst)
{
    int lut_bits, bits, xor, sample, idx, lidx;
    const uint8_t *tab0, *tab1, *tab2;

    if (mode & 0x80)
        mode &= 0x7f;

    if (mode >= FF_ARRAY_ELEMS(cb_luts))
        return AVERROR_INVALIDDATA;

    tab0 = cb_luts[mode][0];
    tab1 = cb_luts[mode][1];
    tab2 = cb_luts[mode][2];

    for (int n = 0; n < nb_samples; n++) {
        if (get_bits_left(gb) < 0) {
            for (int i = n; i < nb_samples; i++)
                dst[n] = 0;
            break;
        }

        idx = show_bits(gb, 8);
        lidx = cb_tabs[idx];
        if (lidx > 7) {
            idx = show_bits(gb, 16) >> 8;
            lidx = 8 + cb_tabs[idx];
        }

        lut_bits = tab0[lidx];
        bits = tab1[lidx];
        xor = tab2[lidx];
        skip_bits_long(gb, lut_bits);
        sample = get_sbits(gb, bits);
        if (xor)
            sample ^= -1 * (1 << bits);

        dst[n] = sample;
    }

    return 0;
}

static void decode_lpc16(int order, int nb_samples, int16_t *dest,
                         const int16_t *factors, int16_t *dst)
{
    if (order <= 0)
        return;

    dest -= order;
    for (int n = 0; n < nb_samples; n++) {
        int16_t *loopdest = dest++;
        int64_t accum = 0;

        for (int j = order - 1; j >= 0; j--) {
            int value = loopdest[0];

            accum += value * factors[j];
            loopdest++;
        }

        dst[n] = loopdest[0] = loopdest[0] - ((accum + (1 << 11)) >> 12);
    }
}

static int uncompress_extra(AVCodecContext *avctx, GetByteContext *gb)
{
    SonarcContext *s = avctx->priv_data;
    PutByteContext pb;

    bytestream2_init_writer(&pb, s->input, sizeof(s->input));

    while (bytestream2_get_bytes_left(gb) > 0) {
        uint8_t byte = bytestream2_get_byte(gb);

        if (byte != 0x81) {
            bytestream2_put_byte(&pb, byte);
        } else {
            int len = bytestream2_get_byte(gb);

            if (len == 0) {
                bytestream2_put_byte(&pb, byte);
            } else if (len < 3) {
                int len = bytestream2_get_byte(gb);
                uint8_t byte = bytestream2_get_byte(gb);

                for (int i = 0; i < len; i++)
                    bytestream2_put_byte(&pb, byte);
            } else {
                for (int i = 0; i < len; i++)
                    bytestream2_put_byte(&pb, 0xff);
            }
        }
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    int nb_samples, size, ret, order, mode, ch = 0, offset = 0;
    SonarcContext *s = avctx->priv_data;
    uint16_t csum = 0, *dst16;
    GetBitContext gbit;
    GetByteContext gb;
    uint8_t *dst;

    *got_frame_ptr = 0;
    frame->nb_samples = 0;

next_channel:
    if (avpkt->size <= offset + 6)
        return AVERROR_INVALIDDATA;

    bytestream2_init(&gb, avpkt->data + offset, FFMIN(AV_RL16(avpkt->data + offset), avpkt->size - offset));

    size = bytestream2_peek_le16u(&gb);
    for (int n = 0; n < size / 2; n++) {
        csum ^= bytestream2_get_le16(&gb);
        if (csum == 0xACED)
            break;
    }

    if (csum != 0xACED && !(avpkt->data[offset+6] & 0x80) && ch == 0)
        return AVERROR_INVALIDDATA;

    bytestream2_seek(&gb, 0, SEEK_SET);

    size = bytestream2_get_le16u(&gb);
    if ((size <= 4) || size > avpkt->size)
        return AVERROR_INVALIDDATA;
    nb_samples = bytestream2_get_le16u(&gb);
    if (nb_samples < 1 || nb_samples > 4096)
        return AVERROR_INVALIDDATA;

    bytestream2_skip(&gb, 2);

    mode = bytestream2_get_byte(&gb);
    order = bytestream2_get_byte(&gb);

    if (frame->nb_samples == 0) {
        frame->nb_samples = nb_samples;
        if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    } else if (nb_samples != frame->nb_samples) {
        return AVERROR_INVALIDDATA;
    }

    for (int n = 0; n < order; n++)
        s->factors[n] = sign_extend(bytestream2_get_le16(&gb), 16);

    if (mode & 0x80) {
        ret = uncompress_extra(avctx, &gb);
        if (ret < 0)
            return ret;

        if ((ret = init_get_bits8(&gbit, s->input, sizeof(s->input))) < 0)
            return ret;
    } else {
        if ((ret = init_get_bits8(&gbit, gb.buffer, bytestream2_get_bytes_left(&gb))) < 0)
            return ret;
    }

    switch (avctx->sample_fmt) {
    case AV_SAMPLE_FMT_U8P:
        memcpy(s->residuals[ch], s->history[ch], sizeof(s->history[0]));

        ret = decode_residue(avctx, mode, nb_samples, s->flags, &gbit, s->residuals[ch] + 256);
        if (ret < 0)
            return ret;

        if (s->channels == 1) {
            dst = frame->extended_data[0];
        } else if (s->channels == 2 && ch == 1) {
            dst = frame->extended_data[1];
            memcpy(frame->extended_data[0], s->samples, nb_samples);
        } else {
            dst = s->samples;
        }
        decode_lpc(order, nb_samples, s->residuals[ch] + 256, s->factors, dst);
        memcpy(s->history[ch] + 256 - order, s->residuals[ch] + nb_samples + 256 - order, order);
        break;
    case AV_SAMPLE_FMT_S16P:
        memcpy(s->residuals16[ch], s->history16[ch], sizeof(s->history16[0]));

        ret = decode_residue16(avctx, mode, nb_samples, s->flags, &gbit, s->residuals16[ch] + 256);
        if (ret < 0)
            return ret;

        if (s->channels == 1) {
            dst16 = (int16_t *)frame->extended_data[0];
        } else if (s->channels == 2 && ch == 1) {
            dst16 = (int16_t *)frame->extended_data[1];
            memcpy(frame->extended_data[0], s->samples16, nb_samples * 2);
        } else {
            dst16 = s->samples16;
        }
        decode_lpc16(order, nb_samples, s->residuals16[ch] + 256, s->factors, dst16);
        memcpy(s->history16[ch] + 256 - order, s->residuals16[ch] + nb_samples + 256 - order, order * 2);
        break;
    default:
        return AVERROR_BUG;
    }

    offset += size;
    ch++;
    if (ch < s->channels)
        goto next_channel;

    *got_frame_ptr = 1;

    return avpkt->size;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    SonarcContext *s = avctx->priv_data;

    memset(s->residuals16, 0, sizeof(s->residuals16));
    memset(s->samples16, 0, sizeof(s->samples16));
    memset(s->history16, 0, sizeof(s->history16));

    memset(s->residuals, 0, sizeof(s->residuals));
    memset(s->samples, 0, sizeof(s->samples));
    memset(s->history, 0, sizeof(s->history));
}

const FFCodec ff_sonarc_decoder = {
    .p.name           = "sonarc",
    CODEC_LONG_NAME("Sonarc (Speech Compression)"),
    .p.type           = AVMEDIA_TYPE_AUDIO,
    .p.id             = AV_CODEC_ID_SONARC,
    .priv_data_size   = sizeof(SonarcContext),
    .init             = sonarc_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush            = decode_flush,
    .p.capabilities   = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_U8P, AV_SAMPLE_FMT_S16P),
};
