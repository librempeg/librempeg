/*
 * Actimagine VX Video decoder
 * Copyright (c) 2021 Florian Nouwt
 *
 * This file is part of Librempeg.
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

#include <inttypes.h>

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "avcodec.h"
#include "bytestream.h"
#include "bswapdsp.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "unary.h"
#include "golomb.h"
#include "internal.h"
#include "libavutil/mem_internal.h"
#include "h264dsp.h"
#include "h264pred.h"
#include "actimagine_vx_vlc.h"

static const uint8_t old_mb_mode_remap_tab[24] = {
     1,  2,  0,  4,  7,  3, 11, 15,  9,  5, 14,  6,
    12, 13,  8, 16, 23, 10, 22, 19, 20, 17, 21, 18
};

static const uint8_t residu_mask_old_tab[32] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x03, 0x05,
    0x0A, 0x0C, 0x0F, 0x1F, 0x07, 0x0B, 0x0D, 0x0E,
    0x06, 0x09, 0x13, 0x15, 0x1A, 0x1C, 0x11, 0x12,
    0x14, 0x18, 0x17, 0x1B, 0x1D, 0x1E, 0x16, 0x19
};

static const uint8_t residu_mask_new_tab[32] = {
    0x00, 0x08, 0x04, 0x02, 0x01, 0x1F, 0x0F, 0x0A,
    0x05, 0x0C, 0x03, 0x10, 0x0E, 0x0D, 0x0B, 0x07,
    0x09, 0x06, 0x1E, 0x1B, 0x1A, 0x1D, 0x17, 0x15,
    0x18, 0x12, 0x11, 0x1C, 0x14, 0x13, 0x16, 0x19
};

static const uint8_t quant4x4_tab[][8] = {
    { 0x0A, 0x0D, 0x0A, 0x0D, 0x0D, 0x10, 0x0D, 0x10 },
    { 0x0B, 0x0E, 0x0B, 0x0E, 0x0E, 0x12, 0x0E, 0x12 },
    { 0x0D, 0x10, 0x0D, 0x10, 0x10, 0x14, 0x10, 0x14 },
    { 0x0E, 0x12, 0x0E, 0x12, 0x12, 0x17, 0x12, 0x17 },
    { 0x10, 0x14, 0x10, 0x14, 0x14, 0x19, 0x14, 0x19 },
    { 0x12, 0x17, 0x12, 0x17, 0x17, 0x1D, 0x17, 0x1D }
};

typedef struct MVec {
    int x, y;
} MVec;

typedef struct VxContext {
    AVFrame *out_frame;
    AVFrame *cur_frame;
    AVFrame *ref_frames[3];
    int      ref_frame_count;

    int version;
    int quantizer;

    GetBitContext gb;

    uint8_t *bitstream;
    int bitstream_size;

    uint8_t zigzag_scan[16];

    int qtab[2][4];

    uint8_t pred4_cache[5][5];

    MVec *vectors;
    int   vectors_stride;

    uint8_t *total_coeff_y;
    int      total_coeff_y_stride;

    uint8_t *total_coeff_uv;
    int      total_coeff_uv_stride;

    BswapDSPContext bdsp;
    H264DSPContext  h264dsp;
    H264PredContext h264pred;
} VxContext;

#define PIXEL_REF(s, ref, plane, x, y)\
    ((s)->ref_frames[(ref)]->data[(plane)]\
        [(y) * (s)->ref_frames[(ref)]->linesize[(plane)] + (x)])

#define PIXEL_CUR(s, plane, x, y)\
    ((s)->cur_frame->data[(plane)]\
        [(y) * (s)->cur_frame->linesize[(plane)] + (x)])

#define VX_VERSION_INVALID  -1
#define VX_VERSION_OLD       0
#define VX_VERSION_NEW       1

static int setup_qtables(AVCodecContext *avctx, int quantizer)
{
    int qx, qy;
    VxContext *s = avctx->priv_data;

    if (quantizer < 12 || quantizer > 161)
        return AVERROR_INVALIDDATA;

    s->quantizer = quantizer;

    qx = quantizer % 6;
    qy = quantizer / 6;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++)
            s->qtab[i][j] = quant4x4_tab[qx][4 * i + j] << qy;

    return 0;
}

static av_cold int vx_init(AVCodecContext *avctx)
{
    int vectors_size;
    int total_coeff_y_size;
    int total_coeff_uv_size;
    VxContext *s = avctx->priv_data;
    int ret;

    if (avctx->width & 15 || avctx->height & 15) {
        av_log(avctx, AV_LOG_ERROR, "width/height not multiple of 16\n");
        return AVERROR_INVALIDDATA;
    }

    ff_bswapdsp_init(&s->bdsp);
    ff_h264dsp_init(&s->h264dsp, 8, 1);
    ff_h264_pred_init(&s->h264pred, AV_CODEC_ID_ACTIMAGINE_VX, 8, 1);

    avctx->pix_fmt = AV_PIX_FMT_GBRP;

    // zigzag_scan is a transposed version of ff_zigzag_scan, like in h264
    for (int i = 0; i < 16; i++)
        s->zigzag_scan[i] = (ff_zigzag_scan[i] >> 2) | ((ff_zigzag_scan[i] << 2) & 0xF);

    // predict4 cache
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            s->pred4_cache[i][j] = 9;

    // motion vector cache
    s->vectors_stride = (avctx->width >> 4) + 2;
    vectors_size = ((avctx->height >> 4) + 1) * s->vectors_stride;
    s->vectors = av_calloc(vectors_size, sizeof(MVec));
    if (!s->vectors)
        return AVERROR(ENOMEM);

    // total dct coefficient cache for luma
    s->total_coeff_y_stride = (avctx->width >> 2) + 1;
    total_coeff_y_size = ((avctx->height >> 2) + 1) * s->total_coeff_y_stride;
    s->total_coeff_y = av_mallocz(total_coeff_y_size);
    if (!s->total_coeff_y)
        return AVERROR(ENOMEM);

    // total dct coefficient cache for chroma
    s->total_coeff_uv_stride = (avctx->width >> 3) + 1;
    total_coeff_uv_size = ((avctx->height >> 3) + 1) * s->total_coeff_uv_stride;
    s->total_coeff_uv = av_mallocz(total_coeff_uv_size);
    if (!s->total_coeff_uv)
        return AVERROR(ENOMEM);

    s->ref_frame_count = 0;
    for (int i = 0; i < 3; i++) {
        s->ref_frames[i] = av_frame_alloc();
        if (!s->ref_frames[i])
            return AVERROR(ENOMEM);
    }
    s->cur_frame = av_frame_alloc();
    if (!s->cur_frame)
        return AVERROR(ENOMEM);
    s->out_frame = av_frame_alloc();
    if (!s->out_frame)
        return AVERROR(ENOMEM);

    s->version = VX_VERSION_INVALID;
    s->quantizer = -1;

    if (avctx->extradata_size >= 4)
        s->quantizer = AV_RL32(avctx->extradata);
    ret = setup_qtables(avctx, s->quantizer);

    ff_h264_cavlc_data_init_vlc();

    return ret;
}

static void clear_total_coeff(AVCodecContext *avctx, int x, int y, int w, int h)
{
    VxContext *s = avctx->priv_data;

    // luma
    uint8_t *total_coeff = &s->total_coeff_y[
        ((y >> 2) + 1) * s->total_coeff_y_stride + (x >> 2) + 1];

    for (int y2 = 0; y2 < (h >> 2); y2++)
        for (int x2 = 0; x2 < (w >> 2); x2++)
            total_coeff[y2 * s->total_coeff_y_stride + x2] = 0;

    // chroma
    total_coeff = &s->total_coeff_uv[
        ((y >> 3) + 1) * s->total_coeff_uv_stride + (x >> 3) + 1];

    for (int y2 = 0; y2 < (h >> 3); y2++)
        for (int x2 = 0; x2 < (w >> 3); x2++)
            total_coeff[y2 * s->total_coeff_uv_stride + x2] = 0;
}

static void predict_plane_intern(AVCodecContext *avctx, int x, int y,
                                 int w, int h, int plane)
{
    VxContext *s = avctx->priv_data;
    if (w == 1 && h == 1)
        return;
    if (w == 1 && h != 1) {
        uint8_t top    = PIXEL_CUR(s, plane, x, y - 1);
        uint8_t bottom = PIXEL_CUR(s, plane, x, y + h - 1);
        PIXEL_CUR(s, plane, x, y + (h >> 1) - 1) = (top + bottom) >> 1;
        predict_plane_intern(avctx, x, y, 1, h >> 1, plane);
        predict_plane_intern(avctx, x, y + (h >> 1), 1, h >> 1, plane);
    } else if (w != 1 && h == 1) {
        uint8_t left  = PIXEL_CUR(s, plane, x - 1, y);
        uint8_t right = PIXEL_CUR(s, plane, x + w - 1, y);
        PIXEL_CUR(s, plane, x + (w >> 1) - 1, y) = (left + right) >> 1;
        predict_plane_intern(avctx, x, y, w >> 1, 1, plane);
        predict_plane_intern(avctx, x + (w >> 1), y, w >> 1, 1, plane);
    } else {
        uint8_t bottom_left  = PIXEL_CUR(s, plane, x - 1, y + h - 1);
        uint8_t top_right    = PIXEL_CUR(s, plane, x + w - 1, y - 1);
        uint8_t bottom_right = PIXEL_CUR(s, plane, x + w - 1, y + h - 1);
        uint8_t bottom_center = (bottom_left + bottom_right) >> 1;
        uint8_t center_right = (top_right + bottom_right) >> 1;
        PIXEL_CUR(s, plane, x + (w >> 1) - 1, y + h - 1) = bottom_center;
        PIXEL_CUR(s, plane, x + w - 1, y + (h >> 1) - 1) = center_right;
        if ((w == 4 || w == 16) ^ (h == 4 || h == 16)) {
            uint8_t center_left = PIXEL_CUR(s, plane, x - 1, y + (h >> 1) - 1);
            PIXEL_CUR(s, plane, x + (w >> 1) - 1, y + (h >> 1) - 1)
                = (center_left + center_right) >> 1;
        } else {
            uint8_t top_center = PIXEL_CUR(s, plane, x + (w >> 1) - 1, y - 1);
            PIXEL_CUR(s, plane, x + (w >> 1) - 1, y + (h >> 1) - 1)
                = (top_center + bottom_center) >> 1;
        }
        predict_plane_intern(avctx, x, y, w >> 1, h >> 1, plane);
        predict_plane_intern(avctx, x + (w >> 1), y, w >> 1, h >> 1, plane);
        predict_plane_intern(avctx, x, y + (h >> 1), w >> 1, h >> 1, plane);
        predict_plane_intern(avctx, x + (w >> 1), y + (h >> 1), w >> 1, h >> 1,
                             plane);
    }
}

static void predict_plane(AVCodecContext *avctx, int x, int y, int w, int h,
                          int plane, int param)
{
    VxContext *s = avctx->priv_data;
    uint8_t bottom_left = PIXEL_CUR(s, plane, x - 1, y + h - 1);
    uint8_t top_right   = PIXEL_CUR(s, plane, x + w - 1, y - 1);
    PIXEL_CUR(s, plane, x + w - 1, y + h - 1)
        = ((bottom_left + top_right + 1) >> 1) + param;
    predict_plane_intern(avctx, x, y, w, h, plane);
}

static int predict_mb_plane(AVCodecContext *avctx, int x, int y, int w, int h)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    // y
    int param = get_se_golomb(gb);
    if (param < -(1 << 16) || param >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid plane param\n");
        return AVERROR_INVALIDDATA;
    }
    predict_plane(avctx, x, y, w, h, 0, param << 1);

    // u
    param = get_se_golomb(gb);
    if (param < -(1 << 16) || param >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid plane param\n");
        return AVERROR_INVALIDDATA;
    }
    predict_plane(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 1, param << 1);

    // v
    param = get_se_golomb(gb);
    if (param < -(1 << 16) || param >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid plane param\n");
        return AVERROR_INVALIDDATA;
    }
    predict_plane(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 2, param << 1);

    return 0;
}

static void predict_horizontal(AVCodecContext *avctx, int x, int y,
                               int w, int h, int plane)
{
    VxContext *s = avctx->priv_data;
    for (int y2 = 0; y2 < h; y2++) {
        uint8_t pixel = PIXEL_CUR(s, plane, x - 1, y + y2);
        for (int x2 = 0; x2 < w; x2++)
            PIXEL_CUR(s, plane, x + x2, y + y2) = pixel;
    }
}

static void predict_vertical(AVCodecContext *avctx, int x, int y,
                             int w, int h, int plane)
{
    VxContext *s = avctx->priv_data;
    for (int y2 = 0; y2 < h; y2++)
        for (int x2 = 0; x2 < w; x2++)
            PIXEL_CUR(s, plane, x + x2, y + y2)
                = PIXEL_CUR(s, plane, x + x2, y - 1);
}

static void predict_dc(AVCodecContext *avctx, int x, int y, int w, int h,
                       int plane)
{
    static const uint8_t shift_tab[] = { 1, 2, 3, 0, 4 };
    uint8_t dc;
    VxContext *s = avctx->priv_data;
    if (x != 0 && y != 0) {
        int sum_h, sum_v;
        sum_h = w >> 1;
        for (int x2 = 0; x2 < w; x2++)
            sum_h += PIXEL_CUR(s, plane, x + x2, y - 1);
        sum_h >>= shift_tab[w >> 2];

        sum_v = h >> 1;
        for (int y2 = 0; y2 < h; y2++)
            sum_v += PIXEL_CUR(s, plane, x - 1, y + y2);
        sum_v >>= shift_tab[h >> 2];

        dc = (sum_h + sum_v + 1) >> 1;
    } else if (x == 0 && y != 0) {
        int sum = w >> 1;
        for (int x2 = 0; x2 < w; x2++)
            sum += PIXEL_CUR(s, plane, x + x2, y - 1);
        dc = sum >> shift_tab[w >> 2];
    } else if (x != 0 && y == 0) {
        int sum = h >> 1;
        for (int y2 = 0; y2 < h; y2++)
            sum += PIXEL_CUR(s, plane, x - 1, y + y2);
        dc = sum >> shift_tab[h >> 2];
    } else
        dc = 128;

    for (int y2 = 0; y2 < h; y2++)
        for (int x2 = 0; x2 < w; x2++)
            PIXEL_CUR(s, plane, x + x2, y + y2) = dc;
}

static int predict_notile_uv(AVCodecContext *avctx, int x, int y, int w, int h)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int mode = get_ue_golomb_31(gb);
    switch (mode) {
    case 0:// dc
        predict_dc(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 1);
        predict_dc(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 2);
        break;
    case 1:// horizontal
        predict_horizontal(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 1);
        predict_horizontal(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 2);
        break;
    case 2:// vertical
        predict_vertical(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 1);
        predict_vertical(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 2);
        break;
    case 3:// plane
        predict_plane(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 1, 0);
        predict_plane(avctx, x >> 1, y >> 1, w >> 1, h >> 1, 2, 0);
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "invalid predict notile uv mode\n");
        return AVERROR_INVALIDDATA;
    }
    return 0;
}

static int predict_notile(AVCodecContext *avctx, int x, int y, int w, int h)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    switch (get_ue_golomb_31(gb)) {
    case 0:// vertical
        predict_vertical(avctx, x, y, w, h, 0);
        break;
    case 1:// horizontal
        predict_horizontal(avctx, x, y, w, h, 0);
        break;
    case 2:// dc
        predict_dc(avctx, x, y, w, h, 0);
        break;
    case 3:// plane
        predict_plane(avctx, x, y, w, h, 0, 0);
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "invalid predict notile y mode\n");
        return AVERROR_INVALIDDATA;
    }
    return predict_notile_uv(avctx, x, y, w, h);
}

static int predict4(AVCodecContext *avctx, int x, int y, int w, int h)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int stride = s->cur_frame->linesize[0];
    for (int y2 = 0; y2 < h >> 2; y2++) {
        for (int x2 = 0; x2 < w >> 2; x2++) {
            uint8_t *dst, *top_right;
            uint8_t mode = FFMIN(s->pred4_cache[1 + y2 - 1][1 + x2],
                                 s->pred4_cache[1 + y2][1 + x2 - 1]);
            if (mode == 9)// if invalid predict dc
                mode = 2;

            if (!get_bits1(gb)) {
                uint8_t val = get_bits(gb, 3);
                mode = val + (val >= mode);
            }

            s->pred4_cache[1 + y2][1 + x2] = mode;

            dst = &PIXEL_CUR(s, 0, x + x2 * 4, y + y2 * 4);
            top_right = &PIXEL_CUR(s, 0, x + x2 * 4 + 4, y + y2 * 4 - 1);

            switch (mode) {
            case 0:// vertical
                s->h264pred.pred4x4[VERT_PRED](dst, NULL, stride);
                break;
            case 1:// horizontal
                s->h264pred.pred4x4[HOR_PRED](dst, NULL, stride);
                break;
            case 2:// dc
                if (x + x2 * 4 == 0 && y + y2 * 4 == 0)
                    s->h264pred.pred4x4[DC_128_PRED](dst, NULL, stride);
                else if (x + x2 * 4 == 0 && y + y2 * 4 != 0)
                    s->h264pred.pred4x4[TOP_DC_PRED](dst, NULL, stride);
                else if (x + x2 * 4 != 0 && y + y2 * 4 == 0)
                    s->h264pred.pred4x4[LEFT_DC_PRED](dst, NULL, stride);
                else
                    s->h264pred.pred4x4[DC_PRED](dst, NULL, stride);
                break;
            case 3:// diagonal-down-left
                s->h264pred.pred4x4[DIAG_DOWN_LEFT_PRED](dst, top_right, stride);
                break;
            case 4:// diagonal-down-right
                s->h264pred.pred4x4[DIAG_DOWN_RIGHT_PRED](dst, NULL, stride);
                break;
            case 5:// vertical-right
                s->h264pred.pred4x4[VERT_RIGHT_PRED](dst, NULL, stride);
                break;
            case 6:// horizontal-down
                s->h264pred.pred4x4[HOR_DOWN_PRED](dst, NULL, stride);
                break;
            case 7:// vertical-left
                s->h264pred.pred4x4[VERT_LEFT_PRED](dst, top_right, stride);
                break;
            case 8:// horizontal-up
                s->h264pred.pred4x4[HOR_UP_PRED](dst, NULL, stride);
                break;
            default:
                av_log(avctx, AV_LOG_ERROR, "invalid predict4 mode\n");
                return AVERROR_INVALIDDATA;
            }
        }
    }
    return predict_notile_uv(avctx, x, y, w, h);
}

static void decode_dct(AVCodecContext *avctx, int x, int y, int plane,
                       const int *level)
{
    LOCAL_ALIGNED_16(int16_t, dct, [16]);
    VxContext *s = avctx->priv_data;

    // dezigzag
    for (int i = 0; i < 16; i++)
        dct[s->zigzag_scan[i]] = level[i];

    // dequantize
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            dct[4 * j + i]     *= s->qtab[i][j];
            dct[4 * j + i + 2] *= s->qtab[i][j];
        }
    }

    s->h264dsp.idct_add(&PIXEL_CUR(s, plane, x, y), dct,
                        s->cur_frame->linesize[plane]);
}

static int decode_residu_cavlc(AVCodecContext *avctx, int x, int y, int plane,
                                int nc, uint8_t *out_total_coeff)
{
    int level[16];
    int coeff_token, total_coeff, trailing_ones, i, zeros_left, suffix_length;
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    coeff_token = get_vlc2(gb, ff_h264_cavlc_coeff_token_vlc[ff_h264_cavlc_coeff_token_table_index[nc]].table,
                           FF_H264_CAVLC_COEFF_TOKEN_VLC_BITS, 2);
    if (coeff_token < 0)
        return AVERROR_INVALIDDATA;

    trailing_ones = coeff_token & 3;
    total_coeff   = coeff_token >> 2;

    *out_total_coeff = total_coeff;
    if (total_coeff == 0)
        return 0;

    av_assert2(total_coeff <= 16);

    i = 15;
    if (total_coeff != 16) {
        int trailing_zeros;
        zeros_left = get_vlc2(gb, ff_h264_cavlc_total_zeros_vlc[total_coeff-1].table,
                              FF_H264_CAVLC_TOTAL_ZEROS_VLC_BITS, 2);
        if (zeros_left < 0)
            return AVERROR_INVALIDDATA;

        trailing_zeros = 16 - (total_coeff + zeros_left);
        while (trailing_zeros-- > 0)
            level[i--] = 0;
    } else {
        zeros_left = 0;
    }

    suffix_length = 0;
    while (1) {
        int level_suffix, level_code, run_before;
        if (trailing_ones > 0) {
            trailing_ones--;
            level[i--] = get_bits1(gb) ? -1 : 1;
        } else {
            int level_prefix = 0;
            while (!get_bits1(gb))
                level_prefix++;

            if (level_prefix == 15)
                level_suffix = get_bits(gb, 11);
            else
                level_suffix = suffix_length == 0 ? 0 : get_bits(gb, suffix_length);

            level_code = level_suffix + (level_prefix << suffix_length) + 1;

            suffix_length += level_code > ff_h264_cavlc_suffix_limit[suffix_length + 1];

            if (get_bits1(gb))
                level_code = -level_code;
            level[i--] = level_code;
        }

        if (--total_coeff == 0)
            break;

        if (zeros_left < 0)
            return AVERROR_INVALIDDATA;

        if (zeros_left == 0)
            continue;

        if (zeros_left < 7)
            run_before = get_vlc2(gb, ff_h264_cavlc_run_vlc[zeros_left-1].table,
                                  FF_H264_CAVLC_RUN_VLC_BITS, 2);
        else
            run_before = get_vlc2(gb, ff_h264_cavlc_run7_vlc.table,
                                  FF_H264_CAVLC_RUN7_VLC_BITS, 2);
        zeros_left -= run_before;
        while (run_before-- > 0)
            level[i--] = 0;
    }

    while (zeros_left-- > 0)
        level[i--] = 0;

    decode_dct(avctx, x, y, plane, level);

    return 0;
}

static int decode_residu_blocks(AVCodecContext *avctx, int x, int y,
                                int w, int h)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    uint8_t *total_coeff_y = &s->total_coeff_y[
        ((y >> 2) + 1) * s->total_coeff_y_stride + (x >> 2) + 1];
    uint8_t *total_coeff_uv = &s->total_coeff_uv[
        ((y >> 3) + 1) * s->total_coeff_uv_stride + (x >> 3) + 1];
    int ret;

    for (int y2 = 0; y2 < h >> 3; y2++) {
        for (int x2 = 0; x2 < w >> 3; x2++) {
            unsigned code = get_ue_golomb_31(gb);
            uint8_t residu_mask;

            if (code > 0x1F) {
                av_log(avctx, AV_LOG_ERROR, "invalid residu mask code\n");
                return AVERROR_INVALIDDATA;
            }

            if (s->version == VX_VERSION_OLD)
                residu_mask = residu_mask_old_tab[code];
            else
                residu_mask = residu_mask_new_tab[code];

            if (residu_mask & 1) {
                int nc = (total_coeff_y[-1] +
                          total_coeff_y[-s->total_coeff_y_stride] + 1) >> 1;
                ret  = decode_residu_cavlc(avctx, x + x2 * 8, y + y2 * 8, 0, nc,
                                           &total_coeff_y[0]);
                if (ret < 0)
                    return ret;
            } else {
                total_coeff_y[0] = 0;
            }

            if (residu_mask & 2) {
                int nc = (total_coeff_y[0] +
                          total_coeff_y[-s->total_coeff_y_stride + 1] + 1) >> 1;
                ret = decode_residu_cavlc(avctx, x + x2 * 8 + 4, y + y2 * 8, 0, nc,
                                          &total_coeff_y[1]);
                if (ret < 0)
                    return ret;
            } else {
                total_coeff_y[1] = 0;
            }

            if (residu_mask & 4) {
                int nc = (total_coeff_y[s->total_coeff_y_stride - 1] +
                          total_coeff_y[0] + 1) >> 1;
                ret = decode_residu_cavlc(avctx, x + x2 * 8, y + y2 * 8 + 4, 0, nc,
                                          &total_coeff_y[s->total_coeff_y_stride]);
                if (ret < 0)
                    return ret;
            } else {
                total_coeff_y[s->total_coeff_y_stride] = 0;
            }

            if (residu_mask & 8) {
                int nc = (total_coeff_y[s->total_coeff_y_stride] +
                          total_coeff_y[1] + 1) >> 1;
                ret = decode_residu_cavlc(avctx, x + x2 * 8 + 4, y + y2 * 8 + 4, 0, nc,
                                          &total_coeff_y[s->total_coeff_y_stride + 1]);
                if (ret < 0)
                    return ret;
            } else {
                total_coeff_y[s->total_coeff_y_stride + 1] = 0;
            }

            if (residu_mask & 16) {
                uint8_t total_coeff_u, total_coeff_v;
                int nc = (total_coeff_uv[-1] +
                          total_coeff_uv[-s->total_coeff_uv_stride] + 1) >> 1;
                ret = decode_residu_cavlc(avctx, (x + x2 * 8) >> 1, (y + y2 * 8) >> 1,
                                          1, nc, &total_coeff_u);
                if (ret < 0)
                    return ret;
                ret = decode_residu_cavlc(avctx, (x + x2 * 8) >> 1, (y + y2 * 8) >> 1,
                                          2, nc, &total_coeff_v);
                if (ret < 0)
                    return ret;
                total_coeff_uv[0] = (total_coeff_u + total_coeff_v + 1) >> 1;
            } else {
                total_coeff_uv[0] = 0;
            }

            total_coeff_y += 2;
            total_coeff_uv++;
        }
        total_coeff_y  += (s->total_coeff_y_stride << 1) - (w >> 2);
        total_coeff_uv += s->total_coeff_uv_stride - (w >> 3);
    }
    return 0;
}

static int predict_inter(AVCodecContext *avctx, int x, int y, int w, int h,
                          const MVec *predVec, int has_delta, int ref_frame)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    MVec vec = *predVec;

    if (ref_frame >= s->ref_frame_count) {
        av_log(avctx, AV_LOG_ERROR, "reference to unavailable frame %d/%d\n", ref_frame, s->ref_frame_count);
        return AVERROR_INVALIDDATA;
    }

    s->cur_frame->pict_type = AV_PICTURE_TYPE_P;

    if (has_delta) {
        vec.x += (unsigned)get_se_golomb(gb);
        vec.y += (unsigned)get_se_golomb(gb);
    }

    if (vec.x >= INT_MAX || vec.y >= INT_MAX)
        return AVERROR_INVALIDDATA;

    s->vectors[(1 + (y >> 4)) * s->vectors_stride + 1 + (x >> 4)] = vec;

    if (x + vec.x < 0 || x + vec.x + w > avctx->width ||
        y + vec.y < 0 || y + vec.y + h > avctx->height) {
        av_log(avctx, AV_LOG_ERROR, "motion vector out of bounds\n");
        return AVERROR_INVALIDDATA;
    }

    // luma
    for (int y2 = 0; y2 < h; y2++)
        for (int x2 = 0; x2 < w; x2++)
            PIXEL_CUR(s, 0, x + x2, y + y2)
                = PIXEL_REF(s, ref_frame, 0, x + x2 + vec.x, y + y2 + vec.y);

    // chroma
    for (int y2 = 0; y2 < (h >> 1); y2++) {
        for (int x2 = 0; x2 < (w >> 1); x2++) {
            // u
            PIXEL_CUR(s, 1, (x >> 1) + x2, (y >> 1) + y2)
                = PIXEL_REF(s, ref_frame, 1, (x >> 1) + x2 + (vec.x >> 1),
                            (y >> 1) + y2 + (vec.y >> 1));
            // v
            PIXEL_CUR(s, 2, (x >> 1) + x2, (y >> 1) + y2)
                = PIXEL_REF(s, ref_frame, 2, (x >> 1) + x2 + (vec.x >> 1),
                            (y >> 1) + y2 + (vec.y >> 1));
        }
    }

    return 0;
}

static int predict_inter_dc(AVCodecContext *avctx, int x, int y, int w, int h)
{
    int dx, dy, dc_y, dc_u, dc_v;
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    if (s->ref_frame_count == 0) {
        av_log(avctx, AV_LOG_ERROR, "reference to unavailable frame\n");
        return AVERROR_INVALIDDATA;
    }

    dx = get_se_golomb(gb);
    dy = get_se_golomb(gb);

    if (x + dx < 0 || x + dx + w > avctx->width ||
        y + dy < 0 || y + dy + h > avctx->height) {
        av_log(avctx, AV_LOG_ERROR, "motion vector out of bounds\n");
        return AVERROR_INVALIDDATA;
    }

    dc_y = get_se_golomb(gb);
    if (dc_y < -(1<<16) || dc_y >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid dc offset\n");
        return AVERROR_INVALIDDATA;
    }
    dc_y <<= 1;

    dc_u = get_se_golomb(gb);
    if (dc_u < -(1<<16) || dc_u >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid dc offset\n");
        return AVERROR_INVALIDDATA;
    }
    dc_u <<= 1;

    dc_v = get_se_golomb(gb);
    if (dc_v < -(1<<16) || dc_v >= (1 << 16)) {
        av_log(avctx, AV_LOG_ERROR, "invalid dc offset\n");
        return AVERROR_INVALIDDATA;
    }
    dc_v <<= 1;

    s->cur_frame->pict_type = AV_PICTURE_TYPE_P;

    // luma
    for (int y2 = 0; y2 < h; y2++)
        for (int x2 = 0; x2 < w; x2++)
            PIXEL_CUR(s, 0, x + x2, y + y2) = av_clip_uint8(
                PIXEL_REF(s, 0, 0, x + x2 + dx, y + y2 + dy) + dc_y);

    // chroma
    for (int y2 = 0; y2 < (h >> 1); y2++) {
        for (int x2 = 0; x2 < (w >> 1); x2++) {
            // u
            PIXEL_CUR(s, 1, (x >> 1) + x2, (y >> 1) + y2) = av_clip_uint8(
                PIXEL_REF(s, 0, 1, (x >> 1) + x2 + (dx >> 1),
                          (y >> 1) + y2 + (dy >> 1)) + dc_u);
            // v
            PIXEL_CUR(s, 2, (x >> 1) + x2, (y >> 1) + y2) = av_clip_uint8(
                PIXEL_REF(s, 0, 2, (x >> 1) + x2 + (dx >> 1),
                          (y >> 1) + y2 + (dy >> 1)) + dc_v);
        }
    }

    return 0;
}

static int decode_mb(AVCodecContext *avctx, int x, int y, int w, int h,
                     const MVec *predVec)
{
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int ret = 0;

    int mode = get_ue_golomb_31(gb);
    if (s->version == VX_VERSION_OLD)
        mode = old_mb_mode_remap_tab[mode];

    switch (mode) {
    case 0:// v-split, no residu
        if (w == 2) {
            av_log(avctx, AV_LOG_ERROR, "invalid macroblock mode\n");
            return AVERROR_INVALIDDATA;
        }
        if ((ret = decode_mb(avctx, x, y, w >> 1, h, predVec)) < 0)
            return ret;
        if ((ret = decode_mb(avctx, x + (w >> 1), y, w >> 1, h, predVec)) < 0)
            return ret;
        if (w == 8 && h >= 8)
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 1:// no delta, no residu, ref 0
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 0, 0)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 2:// h-split, no residu
        if (h == 2) {
            av_log(avctx, AV_LOG_ERROR, "invalid macroblock mode\n");
            return AVERROR_INVALIDDATA;
        }
        if ((ret = decode_mb(avctx, x, y, w, h >> 1, predVec)) < 0)
            return ret;
        if ((ret = decode_mb(avctx, x, y + (h >> 1), w, h >> 1, predVec)) < 0)
            return ret;
        if ((w == 8 || w == 16) && h == 8)
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 3:// unpredicted delta ref0 + dc offset, no residu
        if ((ret = predict_inter_dc(avctx, x, y, w, h)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 4:// delta, no residu, ref 0
    case 5:// delta, no residu, ref 1
    case 6:// delta, no residu, ref 2
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 1, mode - 4)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 7:// plane, no residu
        if ((ret = predict_mb_plane(avctx, x, y, w, h)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 8:// v-split, residu
        if (w == 2) {
            av_log(avctx, AV_LOG_ERROR, "invalid macroblock mode\n");
            return AVERROR_INVALIDDATA;
        }
        if ((ret = decode_mb(avctx, x, y, w >> 1, h, predVec)) < 0)
            return ret;
        if ((ret = decode_mb(avctx, x + (w >> 1), y, w >> 1, h, predVec)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 9:// no delta, no residu, ref 1
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 0, 1)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 10:// unpredicted delta ref0 + dc offset, residu
        if ((ret = predict_inter_dc(avctx, x, y, w, h)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 11:// predict notile, no residu
        if ((ret = predict_notile(avctx, x, y, w, h)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 12:// no delta, residu, ref 0
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 0, 0)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 13:// h-split, residu
        if (h == 2) {
            av_log(avctx, AV_LOG_ERROR, "invalid macroblock mode\n");
            return AVERROR_INVALIDDATA;
        }
        if ((ret = decode_mb(avctx, x, y, w, h >> 1, predVec)) < 0)
            return ret;
        if ((ret = decode_mb(avctx, x, y + (h >> 1), w, h >> 1, predVec)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 14:// no delta, no residu, ref 2
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 0, 2)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 15:// predict4, no residu
        if ((ret = predict4(avctx, x, y, w, h)) < 0)
            return ret;
        if ((w == 8 || w == 16) && (h == 8 || h == 16))
            clear_total_coeff(avctx, x, y, w, h);
        break;
    case 16:// delta, residu, ref 0
    case 17:// delta, residu, ref 1
    case 18:// delta, residu, ref 2
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 1, mode - 16)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 19:// predict4, residu
        if ((ret = predict4(avctx, x, y, w, h)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 20:// no delta, residu, ref 1
    case 21:// no delta, residu, ref 2
        if ((ret = predict_inter(avctx, x, y, w, h, predVec, 0, mode - 20 + 1)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 22:// predict notile, residu
        if ((ret = predict_notile(avctx, x, y, w, h)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    case 23:// plane, residu
        if ((ret = predict_mb_plane(avctx, x, y, w, h)) < 0)
            return ret;
        if ((ret = decode_residu_blocks(avctx, x, y, w, h)) < 0)
            return ret;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "invalid macroblock mode\n");
        return AVERROR_INVALIDDATA;
    }
    return 0;
}

static int detect_format(AVCodecContext *avctx)
{
    // assume the new format, if any incorrect decisions are made for the
    // first macroblock of a keyframe (ref, non-dc prediction) then it must be
    // the old format
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int w = 16;
    int h = 16;
    while (1) {
        int mode = get_ue_golomb_31(gb);
        if (mode == 0 || mode == 8) { // v-split
            if (w == 2) // too many splits
                return VX_VERSION_OLD;
            w >>= 1;
            continue;
        } else if (mode == 2 || mode == 13) { // h-split
            if (h == 2) // too many splits
                return VX_VERSION_OLD;
            h >>= 1;
            continue;
        } else if (mode == 11 || mode == 22) { // predict notile
            if (get_ue_golomb_31(gb) != 2) // mode_y != dc
                return VX_VERSION_OLD;
            if (get_ue_golomb_31(gb) != 0) // mode_uv != dc
                return VX_VERSION_OLD;
            break; //we should have enough evidence now
        } else if (mode == 15 || mode == 19) { // predict4
            // initial prediction is always dc
            // we don't expect that prediction to be wrong
            if (!get_bits1(gb))
                return VX_VERSION_OLD;
            break; //we should have enough evidence now
        } else // inter prediction, plane or any other value
            return VX_VERSION_OLD;
    }
    return VX_VERSION_NEW;
}

static int convert_colorspace(AVCodecContext *avctx)
{
    VxContext *s = avctx->priv_data;
    int ret;

    if ((ret = ff_reget_buffer(avctx, s->out_frame, 0)) < 0)
        return ret;

    for (int y = 0; y < avctx->height; y++) {
        for (int x = 0; x < avctx->width; x++) {
            int Y = PIXEL_CUR(s, 0, x, y);
            int u = PIXEL_CUR(s, 1, x>>1, y>>1) - 128;
            int v = PIXEL_CUR(s, 2, x>>1, y>>1) - 128;
            int r, g, b;

            r = av_clip_uint8(Y + 2 * v);
            g = av_clip_uint8(Y - u/2 - v);
            b = av_clip_uint8(Y + 2 * u);

            s->out_frame->data[0][y * s->out_frame->linesize[0] + x] = g;
            s->out_frame->data[1][y * s->out_frame->linesize[1] + x] = b;
            s->out_frame->data[2][y * s->out_frame->linesize[2] + x] = r;
        }
    }

    return 0;
}

static int vx_decode(AVCodecContext *avctx, AVFrame *avframe,
                     int *got_frame_ptr, AVPacket *pkt)
{
    MVec *vectors;
    VxContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    AVFrame *frame = s->cur_frame;
    int ret, offset = 4;

    av_fast_padded_malloc(&s->bitstream, &s->bitstream_size,
                          pkt->size);

    if ((ret = ff_reget_buffer(avctx, frame, 0)) < 0)
        return ret;

    s->bdsp.bswap16_buf((uint16_t *)s->bitstream, (uint16_t *)pkt->data,
                        (pkt->size + 1) >> 1);

    ret = init_get_bits8(gb, s->bitstream + offset,
                         FFALIGN(pkt->size - offset, 2));
    if (ret < 0)
        return ret;

    // determine the bitstream version if this was not done yet
    if (s->version == VX_VERSION_INVALID) {
        if (s->ref_frame_count != 0) {
            av_log(avctx, AV_LOG_ERROR, "can't determine version on P frame\n");
            return AVERROR_INVALIDDATA;
        }
        s->version = detect_format(avctx);

        // reinit bitreader
        ret = init_get_bits8(gb, s->bitstream + offset,
                             FFALIGN(pkt->size - offset, 2));
        if (ret < 0)
            return ret;
    }

    vectors = s->vectors + s->vectors_stride + 1;

    // initially we assume this is an i frame
    // this is corrected when a p block is found
    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags |= AV_FRAME_FLAG_KEY;

    if (s->quantizer == -1) {
        av_log(avctx, AV_LOG_ERROR, "no quantizer configured\n");
        return AVERROR_INVALIDDATA;
    }

    for (int y = 0; y < avctx->height; y += 16) {
        MVec *vec_cur = vectors;
        for (int x = 0; x < avctx->width; x += 16) {
            MVec predVec;
            vec_cur[0].x = 0;
            vec_cur[0].y = 0;
            predVec.x = mid_pred(vec_cur[-1].x, vec_cur[-s->vectors_stride].x,
                                 vec_cur[-s->vectors_stride + 1].x);
            predVec.y = mid_pred(vec_cur[-1].y, vec_cur[-s->vectors_stride].y,
                                 vec_cur[-s->vectors_stride + 1].y);
            if ((ret = decode_mb(avctx, x, y, 16, 16, &predVec)) < 0)
                return ret;
            vec_cur++;
        }
        vectors += s->vectors_stride;
    }

    if ((ret = convert_colorspace(avctx)) < 0)
        return ret;

    s->cur_frame = s->ref_frames[2];
    s->ref_frames[2] = s->ref_frames[1];
    s->ref_frames[1] = s->ref_frames[0];
    s->ref_frames[0] = frame;

    if (s->ref_frame_count < 3)
        s->ref_frame_count++;

    if ((ret = av_frame_ref(avframe, s->out_frame)) < 0)
        return ret;

    *got_frame_ptr = 1;

    return 0;
}

static void vx_flush(AVCodecContext *avctx)
{
    VxContext *s = avctx->priv_data;

    for (int i = 0; i < 3; i++)
        av_frame_unref(s->ref_frames[i]);

    av_frame_unref(s->cur_frame);
    av_frame_unref(s->out_frame);
    s->ref_frame_count = 0;
}

static av_cold int vx_close(AVCodecContext *avctx)
{
    VxContext *s = avctx->priv_data;

    av_freep(&s->vectors);
    s->vectors_stride = 0;
    av_freep(&s->total_coeff_y);
    s->total_coeff_y_stride = 0;
    av_freep(&s->total_coeff_uv);
    s->total_coeff_uv_stride = 0;

    av_freep(&s->bitstream);
    s->bitstream_size = 0;

    for (int i = 0; i < 3; i++)
        av_frame_free(&s->ref_frames[i]);
    av_frame_free(&s->cur_frame);
    av_frame_free(&s->out_frame);

    return 0;
}

const FFCodec ff_actimagine_vx_decoder = {
    .p.name         = "actimagine_vx",
    CODEC_LONG_NAME("Actimagine VX Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_ACTIMAGINE_VX,
    .priv_data_size = sizeof(VxContext),
    .init           = vx_init,
    FF_CODEC_DECODE_CB(vx_decode),
    .flush          = vx_flush,
    .close          = vx_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
};
