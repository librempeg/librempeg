/*
 * Bink video 2 decoder
 * Copyright (c) 2014 Konstantin Shishkov
 * Copyright (c) 2019 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/attributes.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"

#include "avcodec.h"
#include "blockdsp.h"
#include "codec_internal.h"
#include "copy_block.h"
#include "decode.h"
#include "idctdsp.h"
#include "internal.h"
#include "mathops.h"

#define BITSTREAM_READER_LE
#include "get_bits.h"
#include "unary.h"
#include "bink2.h"

#include "bink2f.h"
#include "bink2g.h"

static void bink2_set_flags(uint8_t *dst, int pos, int n, int flag)
{
    for (int i = pos; i < pos + n; i++) {
        if (flag)
            dst[i >> 3] |=   1 << (i & 7);
        else
            dst[i >> 3] &= ~(1 << (i & 7));
    }
}

/* Read a list of size flags, one bit each from bit 0 of dst on. */
static void bink2_get_block_flags(GetBitContext *gb, int size, uint8_t *dst)
{
    int pos = 0, left = size, toggle = 0, flag = 0;

    if (get_bits1(gb) == 0) {
        for (int j = 0; j < size; j++)
            bink2_set_flags(dst, j, 1, get_bits1(gb));
        return;
    }

    while (left > 0) {
        if (get_bits1(gb) == 0) {
            const int n = FFMIN(left, 5);

            flag = toggle ? !flag : get_bits1(gb);
            toggle = 0;
            bink2_set_flags(dst, pos, 1, flag);
            for (int j = 1; j < n; j++)
                bink2_set_flags(dst, pos + j, 1, get_bits1(gb));
            pos  += n;
            left -= n;
        } else {
            const int bits = left < 4 ? 2 : left < 16 ? 4 : 5;
            int run;

            if (toggle) {
                flag = !flag;
                run  = bits + 1;
            } else {
                flag = get_bits1(gb);
                run  = bits + 2;
            }
            run    = FFMIN(run, left);
            left  -= run;
            toggle = 0;
            if (left > 0) {
                const int extra = get_bits(gb, bits);

                toggle = extra != (1 << bits) - 1;
                run   += FFMIN(extra, left);
                left  -= FFMIN(extra, left);
            }
            bink2_set_flags(dst, pos, run, flag);
            pos += run;
        }
    }
}

static int bink2_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                              int *got_frame, AVPacket *pkt)
{
    Bink2Context * const c = avctx->priv_data;
    GetBitContext *gb = &c->gb;
    uint8_t *dst[4];
    uint8_t *src[4];
    int stride[4];
    int sstride[4];
    uint32_t off = 0;
    int is_kf = !!(pkt->flags & AV_PKT_FLAG_KEY);
    int height_a;
    int ret;

    if ((ret = ff_get_buffer(avctx, frame, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    for (int i = 0; i < 4; i++) {
        src[i]     = c->last->data[i];
        dst[i]     = frame->data[i];
        stride[i]  = frame->linesize[i];
        sstride[i] = c->last->linesize[i];
    }

    if (!is_kf && (!src[0] || !src[1] || !src[2]))
        return AVERROR_INVALIDDATA;

    c->frame_flags = AV_RL32(pkt->data);
    av_log(avctx, AV_LOG_DEBUG, "frame flags %X\n", c->frame_flags);

    if ((ret = init_get_bits8(gb, pkt->data, pkt->size)) < 0)
        return ret;

    height_a = (avctx->height + 31) & 0xFFFFFFE0;
    if (c->version <= 'f') {
        c->num_slices = 2;
        c->slice_height[0] = (avctx->height / 2 + 16) & 0xFFFFFFE0;
    } else if (c->version == 'g') {
        if (height_a < 128) {
            c->num_slices = 1;
        } else {
            c->num_slices = 2;
            c->slice_height[0] = (avctx->height / 2 + 16) & 0xFFFFFFE0;
        }
    } else {
        int start, end;

        c->num_slices = kb2h_num_slices[c->flags & 3];
        start = 0;
        end = height_a + 32 * c->num_slices - 1;
        for (int i = 0; i < c->num_slices - 1; i++) {
            start += ((end - start) / (c->num_slices - i)) & 0xFFFFFFE0;
            end -= 32;
            c->slice_height[i] = start;
        }
    }
    c->slice_height[c->num_slices - 1] = height_a;

    skip_bits_long(gb, 32 + 32 * (c->num_slices - 1));

    if (c->frame_flags & 0x10000) {
        if (!(c->frame_flags & 0x8000))
            bink2_get_block_flags(gb, (((avctx->height + 15) & ~15) >> 3) - 1, c->row_cbp);
        else
            memset(c->row_cbp, 0, c->row_cbp_size);

        if (!(c->frame_flags & 0x4000))
            bink2_get_block_flags(gb, (avctx->coded_width >> 3) - 1, c->col_cbp);
        else
            memset(c->col_cbp, 0, c->col_cbp_size);
    } else {
        memset(c->row_cbp, 0, c->row_cbp_size);
        memset(c->col_cbp, 0, c->col_cbp_size);
    }

    for (int i = 0; i < c->num_slices; i++) {
        if (i == c->num_slices - 1)
            off = pkt->size;
        else
            off = AV_RL32(pkt->data + 4 + i * 4);

        if (c->version <= 'f')
            ret = bink2f_decode_slice(c, dst, stride, src, sstride, is_kf, i ? c->slice_height[i-1] : 0, c->slice_height[i]);
        else
            ret = bink2g_decode_slice(c, dst, stride, src, sstride, is_kf, i ? c->slice_height[i-1] : 0, c->slice_height[i]);
        if (ret < 0)
            return ret;

        align_get_bits(gb);
        if (get_bits_left(gb) < 0)
            av_log(avctx, AV_LOG_WARNING, "slice %d: overread\n", i);
        if (8 * (off - (get_bits_count(gb) >> 3)) > 24)
            av_log(avctx, AV_LOG_WARNING, "slice %d: underread %d\n", i, 8 * (off - (get_bits_count(gb) >> 3)));
        skip_bits_long(gb, 8 * (off - (get_bits_count(gb) >> 3)));

        dst[0] = frame->data[0] + c->slice_height[i]   * stride[0];
        dst[1] = frame->data[1] + c->slice_height[i]/2 * stride[1];
        dst[2] = frame->data[2] + c->slice_height[i]/2 * stride[2];
        dst[3] = frame->data[3] + c->slice_height[i]   * stride[3];
    }

    if (c->version > 'f')
        bink2g_loop_filter(c, frame, c->has_alpha && !(c->frame_flags & 0x80000));

    if (is_kf)
        frame->flags |= AV_FRAME_FLAG_KEY;
    else
        frame->flags &= ~AV_FRAME_FLAG_KEY;
    frame->pict_type = is_kf ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if ((ret = av_frame_replace(c->last, frame)) < 0)
        return ret;

    *got_frame = 1;

    /* always report that the buffer was completely consumed */
    return pkt->size;
}

#define INIT_VLC_STATIC_LE(vlc, nb_bits, nb_codes,                 \
                           bits, bits_wrap, bits_size,             \
                           codes, codes_wrap, codes_size,          \
                           symbols, symbols_wrap, symbols_size,    \
                           static_size)                            \
    do {                                                           \
        static VLCElem table[static_size];                         \
        (vlc)->table           = table;                            \
        (vlc)->table_allocated = static_size;                      \
        ff_vlc_init_sparse(vlc, nb_bits, nb_codes,                 \
                           bits, bits_wrap, bits_size,             \
                           codes, codes_wrap, codes_size,          \
                           symbols, symbols_wrap, symbols_size,    \
                           VLC_INIT_LE | VLC_INIT_USE_STATIC);     \
    } while (0)

static av_cold void bink2_init_vlcs(void)
{
    INIT_VLC_STATIC_LE(&bink2f_quant_vlc, 9, FF_ARRAY_ELEMS(bink2f_quant_codes),
                       bink2f_quant_bits, 1, 1, bink2f_quant_codes, 1, 1, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2f_ac_val0_vlc, 9, FF_ARRAY_ELEMS(bink2f_ac_val_bits[0]),
                       bink2f_ac_val_bits[0], 1, 1, bink2f_ac_val_codes[0], 2, 2, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2f_ac_val1_vlc, 9, FF_ARRAY_ELEMS(bink2f_ac_val_bits[1]),
                       bink2f_ac_val_bits[1], 1, 1, bink2f_ac_val_codes[1], 2, 2, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2f_ac_skip0_vlc, 9, FF_ARRAY_ELEMS(bink2f_ac_skip_bits[0]),
                       bink2f_ac_skip_bits[0], 1, 1, bink2f_ac_skip_codes[0], 2, 2, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2f_ac_skip1_vlc, 9, FF_ARRAY_ELEMS(bink2f_ac_skip_bits[1]),
                       bink2f_ac_skip_bits[1], 1, 1, bink2f_ac_skip_codes[1], 2, 2, NULL, 0, 0, 512);

    INIT_VLC_STATIC_LE(&bink2g_ac_skip0_vlc, 9, FF_ARRAY_ELEMS(bink2g_ac_skip_bits[0]),
                       bink2g_ac_skip_bits[0], 1, 1, bink2g_ac_skip_codes[0], 2, 2, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2g_ac_skip1_vlc, 9, FF_ARRAY_ELEMS(bink2g_ac_skip_bits[1]),
                       bink2g_ac_skip_bits[1], 1, 1, bink2g_ac_skip_codes[1], 2, 2, NULL, 0, 0, 512);
    INIT_VLC_STATIC_LE(&bink2g_mv_vlc, 9, FF_ARRAY_ELEMS(bink2g_mv_bits),
                       bink2g_mv_bits, 1, 1, bink2g_mv_codes, 1, 1, NULL, 0, 0, 512);
}

static av_cold int bink2_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    Bink2Context * const c = avctx->priv_data;
    int ret, w, h;

    w = avctx->width;
    h = avctx->height;
    ret = ff_set_dimensions(avctx, FFALIGN(w, 32), FFALIGN(h, 32));
    if (ret < 0)
        return ret;
    avctx->width  = w;
    avctx->height = h;

    c->version = avctx->codec_tag >> 24;
    if (avctx->extradata_size < 4) {
        av_log(avctx, AV_LOG_ERROR, "Extradata missing or too short\n");
        return AVERROR_INVALIDDATA;
    }
    c->flags = AV_RL32(avctx->extradata);
    av_log(avctx, AV_LOG_DEBUG, "flags: 0x%X\n", c->flags);
    c->has_alpha = c->flags & BINK_FLAG_ALPHA;
    c->avctx = avctx;

    c->last = av_frame_alloc();
    if (!c->last)
        return AVERROR(ENOMEM);

    if ((ret = av_image_check_size(avctx->width, avctx->height, 0, avctx)) < 0)
        return ret;

    avctx->pix_fmt = c->has_alpha ? AV_PIX_FMT_YUVA420P : AV_PIX_FMT_YUV420P;

    ff_blockdsp_init(&c->dsp);

    c->current_q = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->current_q));
    if (!c->current_q)
        return AVERROR(ENOMEM);

    c->prev_q = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->prev_q));
    if (!c->prev_q)
        return AVERROR(ENOMEM);

    c->current_dc = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->current_dc));
    if (!c->current_dc)
        return AVERROR(ENOMEM);

    c->prev_dc = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->prev_dc));
    if (!c->prev_dc)
        return AVERROR(ENOMEM);

    c->current_idc = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->current_idc));
    if (!c->current_idc)
        return AVERROR(ENOMEM);

    c->prev_idc = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->prev_idc));
    if (!c->prev_q)
        return AVERROR(ENOMEM);

    c->current_mv = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->current_mv));
    if (!c->current_mv)
        return AVERROR(ENOMEM);

    c->prev_mv = av_malloc_array((avctx->width + 31) / 32, sizeof(*c->prev_mv));
    if (!c->prev_mv)
        return AVERROR(ENOMEM);

    c->col_cbp_size = (((avctx->width + 31) >> 3) + 7) >> 3;
    c->col_cbp = av_calloc(c->col_cbp_size, sizeof(*c->col_cbp));
    if (!c->col_cbp)
        return AVERROR(ENOMEM);

    c->row_cbp_size = (((avctx->height + 31) >> 3) + 7) >> 3;
    c->row_cbp = av_calloc(c->row_cbp_size, sizeof(*c->row_cbp));
    if (!c->row_cbp)
        return AVERROR(ENOMEM);

    for (int i = 0; i < 4; i++) {
        const int shift = (i == 1 || i == 2) ? 4 : 3;

        c->block_info_stride[i] = avctx->coded_width >> shift;
        c->block_info[i] = av_calloc(c->block_info_stride[i] * (avctx->coded_height >> shift),
                                     sizeof(*c->block_info[i]));
        if (!c->block_info[i])
            return AVERROR(ENOMEM);
    }

    c->quad_mv_stride = avctx->coded_width >> 4;
    c->quad_mv = av_calloc(c->quad_mv_stride * (avctx->coded_height >> 4), sizeof(*c->quad_mv));
    if (!c->quad_mv)
        return AVERROR(ENOMEM);

    ff_thread_once(&init_static_once, bink2_init_vlcs);

    return 0;
}

static void bink2_flush(AVCodecContext *avctx)
{
    Bink2Context *c = avctx->priv_data;

    av_frame_unref(c->last);
}

static av_cold int bink2_decode_end(AVCodecContext *avctx)
{
    Bink2Context * const c = avctx->priv_data;

    av_frame_free(&c->last);
    av_freep(&c->current_q);
    av_freep(&c->prev_q);
    av_freep(&c->current_dc);
    av_freep(&c->prev_dc);
    av_freep(&c->current_idc);
    av_freep(&c->prev_idc);
    av_freep(&c->current_mv);
    av_freep(&c->prev_mv);
    av_freep(&c->col_cbp);
    av_freep(&c->row_cbp);
    for (int i = 0; i < 4; i++)
        av_freep(&c->block_info[i]);
    av_freep(&c->quad_mv);

    return 0;
}

const FFCodec ff_bink2_decoder = {
    .p.name         = "binkvideo2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Bink video 2"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_BINKVIDEO2,
    .priv_data_size = sizeof(Bink2Context),
    .init           = bink2_decode_init,
    .close          = bink2_decode_end,
    FF_CODEC_DECODE_CB(bink2_decode_frame),
    .flush          = bink2_flush,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
