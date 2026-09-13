/*
 * ECM Video decoder
 * Copyright (c) 2026 Paul B Mahol
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

#include "libavutil/attributes.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "internal.h"
#include "decode.h"
#include "bytestream.h"
#define CACHED_BITSTREAM_READER 1
#define BITSTREAM_READER_LE
#include "get_bits.h"

#define FRAME_STRIDE 0x2580
#define FRAME_HEADER 0x204
#define SIM_TABLE_DIM 256
#define MAX_CODES 512
#define MAX_CODE_LEN 15
#define CANVAS_W 320
#define CANVAS_H 240
#define FRAME_W 80
#define FRAME_H 60
#define JITTER_CNT 47

static const uint8_t len_extra_bits[33] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
    2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 0, 0, 0
};

static const uint8_t len_base[32] = {
    0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12,
    14, 16, 20, 24, 28, 32, 40, 48, 56, 64,
    80, 96, 112, 128, 160, 192, 224, 255, 0, 0
};

static const uint8_t dist_extra_bits[32] = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
    7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 0, 0
};

static const uint16_t dist_base[32] = {
    0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48,
    64, 96, 128, 192, 256, 384, 512, 768,
    1024, 1536, 2048, 3072, 4096, 6144, 8192,
    12288, 16384, 24576
};

typedef struct ECMContext {
    GetBitContext gb;

    VLC litlen;
    VLC dist;

    int64_t last_pts;
    int frame_index;
    AVPacket *pkt;

    uint8_t cursor;
    int32_t jitter[47];
    int32_t dist2[32][32];
    uint8_t sim_tab[256 * 256];

    uint8_t *uncompressed_data;
    unsigned int uncompressed_data_size;
} ECMContext;

static int build_vlc(AVCodecContext *avctx, GetBitContext *gb, VLC *vlc, int table_bits)
{
    uint32_t len_count[MAX_CODE_LEN + 1] = { 0 };
    uint32_t count = get_bits(gb, 9);

    ff_vlc_free(vlc);

    if (count == 0)
        return AVERROR_INVALIDDATA;

    uint8_t lens[MAX_CODES] = { 0 };
    int prev_len = 0;

    for (int i = 0; i < count; i++) {
        if (get_bits1(gb))
            prev_len = get_bits(gb, 4);
        lens[i] = prev_len;
        len_count[prev_len]++;
    }
    len_count[0] = 0;

    uint16_t sym_by_len[MAX_CODES] = { 0 };
    uint8_t len_by_len[MAX_CODES + 1] = { 0 };
    int num_used = 0;

    for (int len = 1; len <= MAX_CODE_LEN; len++)
        for (int sym = 0; sym < count; sym++)
            if (lens[sym] == len)
                sym_by_len[num_used++] = sym;

    num_used = 0;
    for (int len = 1; len <= MAX_CODE_LEN; len++)
        for (int k = 0; k < len_count[len]; k++)
            len_by_len[num_used++] = len;
    len_by_len[num_used] = 0;

    return ff_vlc_init_from_lengths(vlc, table_bits, num_used, len_by_len, 1,
                                    sym_by_len, 2, 2, 0, VLC_INIT_OUTPUT_LE, avctx);
}

static int uncompress_lz77(AVCodecContext *avctx, const unsigned size)
{
    ECMContext *s = avctx->priv_data;
    uint8_t *out = s->uncompressed_data;
    GetBitContext *gb = &s->gb;
    int offset = 0, ret;

    while (get_bits_left(gb) > 0) {
        if (get_bits1(gb)) {
            ret = build_vlc(avctx, gb, &s->litlen, 10);
            if (ret < 0)
                return ret;

            ret = build_vlc(avctx, gb, &s->dist, 8);
            if (ret < 0)
                return ret;

            for (;;) {
                if (get_bits_left(gb) < 0)
                    return AVERROR_INVALIDDATA;

                int sym = get_vlc2(gb, s->litlen.table, 10, 2);
                if (sym < 0)
                    return AVERROR_INVALIDDATA;

                if (sym <= 0xFF) {
                    if (offset >= size)
                        return AVERROR_INVALIDDATA;

                    out[offset++] = sym;
                    continue;
                }

                if (sym == 0x100)
                    break;

                uint32_t len_extra = get_bitsz(gb, len_extra_bits[sym - 256]);
                uint32_t match_len = len_base[sym - 256] + 3 + len_extra;
                int dist_sym = get_vlc2(gb, s->dist.table, 8, 2);
                if (dist_sym < 0)
                    return AVERROR_INVALIDDATA;
                uint32_t dist_extra = get_bitsz(gb, dist_extra_bits[dist_sym]);
                uint32_t distance = dist_base[dist_sym] + dist_extra;

                if (offset <= distance)
                    return AVERROR_INVALIDDATA;

                uint8_t *src = out + offset - 1 - distance;
                for (int i = 0; i < match_len; i++) {
                    if (offset >= size)
                        return AVERROR_INVALIDDATA;

                    out[offset++] = *src++;
                }
            }
        } else {
            int len = get_bits(gb, 15);

            while (len > 0) {
                if (get_bits_left(gb) < 8)
                    return AVERROR_INVALIDDATA;

                if (offset >= size)
                    return AVERROR_INVALIDDATA;

                out[offset++] = get_bits(gb, 8);
                len--;
            }
        }

        if (get_bits1(gb))
            break;
    }

    align_get_bits(gb);
    if (get_bits_left(gb) > 0)
        av_log(avctx, AV_LOG_DEBUG, "underread: %d\n", get_bits_left(gb));

    return 0;
}

static inline int32_t spread5(int32_t k)
{
    return (k << 3) | (k & 7);
}

static int decode(AVCodecContext *avctx, int frame_index, AVFrame *frame)
{
    const ptrdiff_t linesize = frame->linesize[0];
    GetByteContext gbyte, *gb = &gbyte;
    ECMContext *s = avctx->priv_data;

    bytestream2_init(gb, s->uncompressed_data, s->uncompressed_data_size);

    if (frame_index == 0) {
        for (int a = 0; a < 32; a++) {
            int32_t wa = spread5(a);

            for (int b = 0; b < 32; b++) {
                int32_t d  = spread5(b) - wa;
                int32_t sq = d * d;

                s->dist2[a][b] = sq;
                s->dist2[b][a] = sq;
            }
        }

        GetByteContext cur_row = *gb;
        bytestream2_seek(&cur_row, 4, SEEK_SET);
        GetByteContext prev_row = cur_row;
        uint8_t *out_row = s->sim_tab;

        for (int row = 0; row < SIM_TABLE_DIM; row++) {
            for (int col = 0; col < SIM_TABLE_DIM; col++) {
                unsigned cur  = bytestream2_get_le16(&cur_row);
                unsigned prev = bytestream2_get_le16(&prev_row);

                unsigned r = s->dist2[(cur >> 10) & 0x1F][(prev >> 10) & 0x1F];
                unsigned g = s->dist2[(cur >>  5) & 0x1F][(prev >>  5) & 0x1F];
                unsigned b = s->dist2[ cur        & 0x1F][ prev        & 0x1F];

                out_row[col] = (r + g + b) <= 1000;
            }

            bytestream2_seek(&prev_row, -SIM_TABLE_DIM, SEEK_CUR);
            out_row += SIM_TABLE_DIM;
        }

        for (int i = 0; i < JITTER_CNT; i++)
            s->jitter[i] = (random() % 5) * CANVAS_W + (random() % 5) - 0x282;
    }

    GetByteContext pgb = *gb;
    bytestream2_seek(&pgb, frame_index * FRAME_STRIDE + FRAME_HEADER, SEEK_SET);

    for (int row = 0; row < FRAME_H; row++) {
        for (int col = 0; col < FRAME_W; col++) {
            uint16_t pixel = bytestream2_get_le16(&pgb);
            uint32_t *block = (uint32_t *)(frame->data[0] + (row * 4) * linesize);

            block += col * 4;
            if (pixel < 65000) {
                if (pixel >= 13500)
                    return AVERROR_INVALIDDATA;

                bytestream2_seek(gb, (pixel * 16) + 15*FRAME_STRIDE+FRAME_HEADER, SEEK_SET);
                for (int r = 0; r < 4; r++) {
                    for (int c = 0; c < 4; c++) {
                        unsigned shade = bytestream2_get_byte(gb);
                        uint32_t value = shade | (shade << 8) | (shade << 16);

                        block[r * (linesize/4) + c] = value;
                    }
                }
            } else {
                unsigned shade = (pixel - 65000) & 0xff;
                uint32_t value = shade | (shade << 8) | (shade << 16);

                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 4; c++)
                        block[r * (linesize/4) + c] = value;
            }
        }
    }

    return 0;
}

static int receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    ECMContext *s = avctx->priv_data;
    int ret;

    if (!s->pkt->data) {
        s->frame_index = 0;
        ret = ff_decode_get_packet(avctx, s->pkt);
        if (ret < 0)
            return ret;

        if (s->last_pts == AV_NOPTS_VALUE)
            s->last_pts = s->pkt->pts;
        if (s->pkt->size <= 4) {
            av_packet_unref(s->pkt);
            return AVERROR_INVALIDDATA;
        }
    }

    if (s->pkt->size >= 4 && s->frame_index == 0) {
        int uncompressed_size = AV_RL32(s->pkt->data);

        if (uncompressed_size <= 4)
            return AVERROR_INVALIDDATA;

        av_fast_padded_malloc(&s->uncompressed_data, &s->uncompressed_data_size, uncompressed_size);
        if (!s->uncompressed_data)
            return AVERROR(ENOMEM);

        ret = init_get_bits8(&s->gb, s->pkt->data + 4, s->pkt->size - 4);
        if (ret < 0)
            return ret;

        ret = uncompress_lz77(avctx, uncompressed_size);
        if (ret < 0) {
            av_packet_unref(s->pkt);
            return ret;
        }
    }

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    ret = decode(avctx, s->frame_index, frame);
    if (ret < 0)
        return ret;

    s->frame_index++;
    if (s->frame_index >= 15) {
        s->frame_index = 0;
        av_packet_unref(s->pkt);
    }

    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags |= AV_FRAME_FLAG_KEY;
    frame->duration = 1;
    frame->pts = s->last_pts;
    if (s->last_pts != AV_NOPTS_VALUE)
        s->last_pts += frame->duration;

    return ret;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    ECMContext *s = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_RGB0;
    avctx->width = CANVAS_W;
    avctx->height = CANVAS_H;

    s->last_pts = AV_NOPTS_VALUE;
    s->pkt = avctx->internal->in_pkt;
    s->frame_index = 0;

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    ECMContext *s = avctx->priv_data;

    s->frame_index = 0;
    s->last_pts = AV_NOPTS_VALUE;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    ECMContext *s = avctx->priv_data;

    av_freep(&s->uncompressed_data);

    ff_vlc_free(&s->litlen);
    ff_vlc_free(&s->dist);

    return 0;
}

const FFCodec ff_ecm_decoder = {
    .p.name         = "ecm",
    CODEC_LONG_NAME("Eurocom Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_ECM,
    .priv_data_size = sizeof(ECMContext),
    .init           = decode_init,
    FF_CODEC_RECEIVE_FRAME_CB(receive_frame),
    .flush          = decode_flush,
    .close          = decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
