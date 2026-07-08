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

static void bink2_get_block_flags(GetBitContext *gb, int offset, int size, uint8_t *dst)
{
    int j, v = 0, flags_left, mode = 0, nv;
    unsigned cache, flag = 0;

    if (get_bits1(gb) == 0) {
        for (j = 0; j < size >> 3; j++)
            dst[j] = get_bits(gb, 8);
        dst[j] = get_bitsz(gb, size & 7);

        return;
    }

    flags_left = size;
    while (flags_left > 0) {
        cache = offset;
        if (get_bits1(gb) == 0) {
            if (mode == 3) {
                flag ^= 1;
            } else {
                flag = get_bits1(gb);
            }
            mode = 2;
            if (flags_left < 5) {
                nv = get_bitsz(gb, flags_left - 1);
                nv <<= (offset + 1) & 0x1f;
                offset += flags_left;
                flags_left = 0;
            } else {
                nv = get_bits(gb, 4) << ((offset + 1) & 0x1f);
                offset += 5;
                flags_left -= 5;
            }
            v |= flag << (cache & 0x1f) | nv;
            if (offset >= 8) {
                *dst++ = v & 0xff;
                v >>= 8;
                offset -= 8;
            }
        } else {
            int temp, bits, nb_coded;

            bits = flags_left < 4 ? 2 : flags_left < 16 ? 4 : 5;
            nb_coded = bits + 1;
            if (mode == 3) {
                flag ^= 1;
            } else {
                nb_coded++;
                flag = get_bits1(gb);
            }
            nb_coded = FFMIN(nb_coded, flags_left);
            flags_left -= nb_coded;
            if (flags_left > 0) {
                temp = get_bits(gb, bits);
                flags_left -= temp;
                nb_coded += temp;
                mode = temp == (1 << bits) - 1U ? 1 : 3;
            }

            temp = (flag << 0x1f) >> 0x1f & 0xff;
            while (nb_coded > 8) {
                v |= temp << (cache & 0x1f);
                *dst++ = v & 0xff;
                v >>= 8;
                nb_coded -= 8;
            }
            if (nb_coded > 0) {
                offset += nb_coded;
                v |= ((1 << (nb_coded & 0x1f)) - 1U & temp) << (cache & 0x1f);
                if (offset >= 8) {
                    *dst++ = v & 0xff;
                    v >>= 8;
                    offset -= 8;
                }
            }
        }
    }

    if (offset != 0)
        *dst = v;
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
    int ret, w, h;
    int height_a;

    w = avctx->width;
    h = avctx->height;
    ret = ff_set_dimensions(avctx, FFALIGN(w, 32), FFALIGN(h, 32));
    if (ret < 0)
        return ret;
    avctx->width  = w;
    avctx->height = h;

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
    ff_dlog(avctx, "frame flags %X\n", c->frame_flags);

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
            bink2_get_block_flags(gb, 1, (((avctx->height + 15) & ~15) >> 3) - 1, c->row_cbp);
        if (!(c->frame_flags & 0x4000))
            bink2_get_block_flags(gb, 1, (((avctx->width + 15) & ~15) >> 3) - 1, c->col_cbp);
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

    if (is_kf)
        frame->flags |= AV_FRAME_FLAG_KEY;
    frame->pict_type = is_kf ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    av_frame_unref(c->last);
    if ((ret = av_frame_ref(c->last, frame)) < 0)
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

static av_cold int bink2_decode_init(AVCodecContext *avctx)
{
    Bink2Context * const c = avctx->priv_data;
    int ret;

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

    c->col_cbp = av_calloc((((avctx->width + 31) >> 3) + 7) >> 3, sizeof(*c->col_cbp));
    if (!c->col_cbp)
        return AVERROR(ENOMEM);

    c->row_cbp = av_calloc((((avctx->height + 31) >> 3) + 7) >> 3, sizeof(*c->row_cbp));
    if (!c->row_cbp)
        return AVERROR(ENOMEM);

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
