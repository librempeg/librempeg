/*
 * Sony PlayStation MDEC (Motion DECoder)
 * Copyright (c) 2003 Michael Niedermayer
 *
 * based upon code from Sebastian Jedruszkiewicz <elf@frogger.rules.pl>
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

/**
 * @file
 * Sony PlayStation MDEC (Motion DECoder)
 * This is very similar to intra-only MPEG-1.
 */

#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"

#include "avcodec.h"
#include "blockdsp.h"
#include "bswapdsp.h"
#include "codec_internal.h"
#include "idctdsp.h"
#include "mpeg12data.h"
#include "mpeg12dec.h"
#include "thread.h"

typedef struct MDECContext {
    AVCodecContext *avctx;
    BlockDSPContext bdsp;
    BswapDSPContext bbdsp;
    IDCTDSPContext idsp;
    GetBitContext gb;
    uint8_t permutated_scantable[64];
    int version;
    int qscale;
    int last_dc[3];
    int mb_width;
    int mb_height;
    int mb_x, mb_y;
    DECLARE_ALIGNED(32, int16_t, block)[6][64];
    DECLARE_ALIGNED(16, uint16_t, quant_matrix)[64];
    uint8_t *bitstream_buffer;
    unsigned int bitstream_buffer_size;
} MDECContext;

//very similar to MPEG-1
static inline int mdec_decode_block_intra(MDECContext *a, int16_t *block, int n)
{
    GetBitContext *gb = &a->gb;
    int level, diff, i, j, run;
    int component;
    const uint8_t *const scantable = a->permutated_scantable;
    const uint16_t *quant_matrix = a->quant_matrix;
    const int qscale = a->qscale;

    /* DC coefficient */
    if (a->version <= 2) {
        block[0] = 2 * get_sbits(gb, 10) + 1024;
    } else {
        component = (n <= 3 ? 0 : n - 4 + 1);
        diff = decode_dc(gb, component);
        a->last_dc[component] += diff;
        block[0] = a->last_dc[component] * (1 << 3);
    }

    i = 0;
    {
        OPEN_READER(re, gb);
        /* now quantify & encode AC coefficients */
        for (;;) {
            UPDATE_CACHE(re, gb);
            GET_RL_VLC(level, run, re, gb, ff_mpeg1_rl_vlc, TEX_VLC_BITS, 2, 0);

            if (level == 127) {
                break;
            } else if (level != 0) {
                i += run;
                if (i > 63) {
                    av_log(a->avctx, AV_LOG_ERROR,
                           "ac-tex damaged at %d %d\n", a->mb_x, a->mb_y);
                    return AVERROR_INVALIDDATA;
                }
                j     = scantable[i];
                level = (level * qscale * quant_matrix[j]) >> 3;
                level = (level ^ SHOW_SBITS(re, gb, 1)) - SHOW_SBITS(re, gb, 1);
                LAST_SKIP_BITS(re, gb, 1);
            } else {
                /* escape */
                run = SHOW_UBITS(re, gb, 6) + 1;
                SKIP_BITS(re, gb, 6);
                level = SHOW_SBITS(re, gb, 10);
                LAST_SKIP_BITS(re, gb, 10);
                i += run;
                if (i > 63) {
                    av_log(a->avctx, AV_LOG_ERROR,
                           "ac-tex damaged at %d %d\n", a->mb_x, a->mb_y);
                    return AVERROR_INVALIDDATA;
                }
                j = scantable[i];
                if (level < 0) {
                    level = -level;
                    level = (level * (unsigned)qscale * quant_matrix[j]) >> 3;
                    level = (level - 1) | 1;
                    level = -level;
                } else {
                    level = (level * (unsigned)qscale * quant_matrix[j]) >> 3;
                    level = (level - 1) | 1;
                }
            }

            block[j] = level;
        }
        CLOSE_READER(re, gb);
    }
    return 0;
}

static inline int decode_mb(MDECContext *a, int16_t block[6][64])
{
    int i, ret;
    static const int block_index[6] = { 5, 4, 0, 1, 2, 3 };

    a->bdsp.clear_blocks(block[0]);

    for (i = 0; i < 6; i++) {
        if ((ret = mdec_decode_block_intra(a, block[block_index[i]],
                                           block_index[i])) < 0)
            return ret;
        if (get_bits_left(&a->gb) < 0)
            return AVERROR_INVALIDDATA;
    }
    return 0;
}

static inline void idct_put(MDECContext *a, AVFrame *frame, int mb_x, int mb_y)
{
    int16_t (*block)[64] = a->block;
    int linesize = frame->linesize[0];

    uint8_t *dest_y  = frame->data[0] + (mb_y * 16* linesize              ) + mb_x * 16;
    uint8_t *dest_cb = frame->data[1] + (mb_y * 8 * frame->linesize[1]) + mb_x * 8;
    uint8_t *dest_cr = frame->data[2] + (mb_y * 8 * frame->linesize[2]) + mb_x * 8;

    a->idsp.idct_put(dest_y,                    linesize, block[0]);
    a->idsp.idct_put(dest_y + 8,                linesize, block[1]);
    a->idsp.idct_put(dest_y + 8 * linesize,     linesize, block[2]);
    a->idsp.idct_put(dest_y + 8 * linesize + 8, linesize, block[3]);

    if (!(a->avctx->flags & AV_CODEC_FLAG_GRAY)) {
        a->idsp.idct_put(dest_cb, frame->linesize[1], block[4]);
        a->idsp.idct_put(dest_cr, frame->linesize[2], block[5]);
    }
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    MDECContext * const a = avctx->priv_data;
    const uint8_t *buf    = avpkt->data;
    int buf_size          = avpkt->size;
    int ret;

    if ((ret = ff_thread_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    av_fast_padded_malloc(&a->bitstream_buffer, &a->bitstream_buffer_size, buf_size);
    if (!a->bitstream_buffer)
        return AVERROR(ENOMEM);
    a->bbdsp.bswap16_buf((uint16_t *)a->bitstream_buffer, (uint16_t *)buf, (buf_size + 1) / 2);
    if ((ret = init_get_bits8(&a->gb, a->bitstream_buffer, buf_size)) < 0)
        return ret;

    /* skip over 4 preamble bytes in stream (typically 0xXX 0xXX 0x00 0x38) */
    if ((show_bits_long(&a->gb, 32) & 0xFFFF) == 0x3800) {
        skip_bits_long(&a->gb, 32);
    } else {
        skip_bits_long(&a->gb, 64);
    }

    a->qscale  = get_bits(&a->gb, 16);
    a->version = get_bits(&a->gb, 16);

    a->last_dc[0] = a->last_dc[1] = a->last_dc[2] = 128;

    for (a->mb_x = 0; a->mb_x < a->mb_width; a->mb_x++) {
        for (a->mb_y = 0; a->mb_y < a->mb_height; a->mb_y++) {
            if ((ret = decode_mb(a, a->block)) < 0)
                return ret;

            idct_put(a, frame, a->mb_x, a->mb_y);
        }
    }

    *got_frame = 1;

    return (get_bits_count(&a->gb) + 31) / 32 * 4;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    MDECContext * const a = avctx->priv_data;
    int i;

    a->mb_width  = (avctx->coded_width  + 15) / 16;
    a->mb_height = (avctx->coded_height + 15) / 16;

    a->avctx           = avctx;

    ff_blockdsp_init(&a->bdsp);
    ff_bswapdsp_init(&a->bbdsp);
    ff_idctdsp_init(&a->idsp, avctx);
    ff_mpeg12_init_vlcs();
    ff_permute_scantable(a->permutated_scantable, ff_zigzag_direct,
                         a->idsp.idct_permutation);

    avctx->pix_fmt  = AV_PIX_FMT_YUVJ420P;
    avctx->color_range = AVCOL_RANGE_JPEG;

    /* init q matrix */
    for (i = 0; i < 64; i++) {
        int j = a->idsp.idct_permutation[i];

        a->quant_matrix[j] = ff_mpeg1_default_intra_matrix[i];
    }

    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    MDECContext * const a = avctx->priv_data;

    av_freep(&a->bitstream_buffer);
    a->bitstream_buffer_size = 0;

    return 0;
}

const FFCodec ff_mdec_decoder = {
    .p.name           = "mdec",
    CODEC_LONG_NAME("Sony PlayStation MDEC (Motion DECoder)"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_MDEC,
    .priv_data_size   = sizeof(MDECContext),
    .init             = decode_init,
    .close            = decode_end,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS,
};
