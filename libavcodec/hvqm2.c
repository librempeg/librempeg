/*
 * HVQM2 Video decoder
 * Copyright (c) 2018-2020 Tillmann Karras
 * Copyright (c) 2024 Paul B Mahol
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

#include <inttypes.h>

#include "libavutil/mem.h"
#include "libavutil/thread.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

#define CACHED_BITSTREAM_READER !ARCH_X86_32

#include "get_bits.h"
#include "internal.h"

#define HVQM2_NESTSIZE_L 70
#define HVQM2_NESTSIZE_S 38
#define HVQM2_NESTSIZE (HVQM2_NESTSIZE_L * HVQM2_NESTSIZE_S)
#define HVQM2_VIDEO_KEYFRAME 0
#define HVQM2_VIDEO_PREDICT 1
#define HVQM2_VIDEO_HOLD 2
#define MAXWIDTH 640
#define MAXHEIGHT 480

#define PLANE_COUNT 3
#define LUMA_CHROMA 2
#define LUMA_IDX 0
#define CHROMA_IDX 1

typedef struct HVQM2KeyFrame {
    uint32_t dcrun_offset[PLANE_COUNT];
    uint16_t nest_start_x;
    uint16_t nest_start_y;
} HVQM2KeyFrame;

typedef struct HVQM2PredictFrame {
    uint32_t mv_offset;
    uint32_t mb_offset;
} HVQM2PredictFrame;

typedef struct HVQM2Frame {
    uint32_t basisnum_offset[2];
    uint32_t basnumrn_offset[2];
    uint32_t scale_offset[PLANE_COUNT];
    uint32_t fixvl_offset[PLANE_COUNT];
    uint32_t dcval_offset[PLANE_COUNT];
} HVQM2Frame;

typedef struct HVQM2Context {
    uint8_t nestbuf[HVQM2_NESTSIZE_L * HVQM2_NESTSIZE_S];
    GetBitContext basisnum_buf[2];
    GetBitContext basisnumrun_buf[2];
    GetBitContext scale_buf[PLANE_COUNT];
    GetBitContext dcval_buf[PLANE_COUNT];
    GetBitContext dcrun_buf[PLANE_COUNT];
    GetBitContext mb_buf;
    GetBitContext mv_buf;
    uint8_t const *fixvl[PLANE_COUNT];
    VLC dcval_vlc;
    VLC basisnum_vlc;
    VLC basisnumrun_vlc;
    VLC scale_vlc;
    VLC mv_vlc;
    int16_t dc_max;
    int16_t dc_min;
    uint8_t *basis[PLANE_COUNT];
    uint8_t *dcbuf[PLANE_COUNT];
    struct wcode
    {
        uint8_t *basis_prev_line;
        uint8_t *dcbuf_prev_line;
        uint8_t *basis_curr_line;
        uint8_t *dcbuf_curr_line;
        uint8_t *basis_next_line;
        uint8_t *dcbuf_next_line;
        uint8_t basis_next;
        uint8_t dcbuf_next;
        uint8_t basis_curr;
        uint8_t dcbuf_curr;
        uint8_t dcbuf_prev;
    } u_wcode, v_wcode, y0wcode, y1wcode;
    uint32_t divT[0x200];
    uint8_t clipT[0x300];
    int fb_width;
    uint32_t h_sampling_rate;
    uint32_t v_sampling_rate;
    uint32_t y_shiftnum;
    uint32_t video_quantize_shift;
    uint32_t mcu_h_pix;
    uint32_t mcu_v_pix;
    uint32_t mcu411;
    uint32_t next_macroblk_line;
    uint32_t im_width;
    uint32_t im_height;
    uint32_t lum_hblocks;
    uint32_t lum_vblocks;
    int lum_totalblocks;
    uint32_t col_hblocks;
    uint32_t col_vblocks;
    int col_totalblocks;
    uint8_t *nest;
    uint32_t nest_w;
    uint32_t nest_h;
    uint8_t yshift;
    uint8_t pix_alpha;
    int16_t pix_y[4][16];
    int16_t pix_u[16];
    int16_t pix_v[16];
    uint8_t workbuf[(MAXWIDTH/8)*(MAXHEIGHT/4)*4];

    AVFrame *frame[2];

    GetBitContext gb;
} HVQM2Context;

static void init_table(HVQM2Context *s, uint8_t alpha)
{
    s->pix_alpha = alpha;

    for (int i = -0x100; i < 0x200; i++) {
        int j = i < 0 ? 0 : (i < 0x100 ? i : 255);

        s->clipT[0x100 + i] = j;
    }

    s->divT[0] = 0;
    for (int i = 1; i < 0x200; i++)
        s->divT[i] = 0x1000 / i;
}

static void hvqm2_setup(AVCodecContext *avctx, HVQM2Context *s)
{
    s->im_width = avctx->width;
    s->im_height = avctx->height;
    s->fb_width = avctx->width;
    s->mcu_h_pix = s->h_sampling_rate * 4;
    s->mcu_v_pix = s->v_sampling_rate * 4 * s->fb_width;
    s->lum_hblocks = (int)avctx->width >> 2;
    s->lum_vblocks = (int)avctx->height >> 2;
    s->lum_totalblocks = s->lum_hblocks * s->lum_vblocks;
    s->mcu411 = s->v_sampling_rate == 2;
    s->col_hblocks = (int)avctx->width >> 3;
    s->col_vblocks = (int)avctx->height >> (s->mcu411 ? 3 : 2);
    s->col_totalblocks = s->col_hblocks * s->col_vblocks;
    s->next_macroblk_line = s->fb_width * 8;
    s->yshift = s->y_shiftnum;

    if (s->yshift == 8) {
        s->nest_w = HVQM2_NESTSIZE_L;
        s->nest_h = HVQM2_NESTSIZE_S;
    } else {
        s->nest_w = HVQM2_NESTSIZE_S;
        s->nest_h = HVQM2_NESTSIZE_L;
    }

    s->dc_min = 0xFF80 << s->video_quantize_shift;
    s->dc_max = 0x007F << s->video_quantize_shift;
}

static av_cold int hvqm2_init(AVCodecContext *avctx)
{
    HVQM2Context *s = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;

    for (int i = 0; i < 2; i++) {
        s->frame[i] = av_frame_alloc();
        if (!s->frame[i])
            return AVERROR(ENOMEM);
    }

    if (avctx->extradata_size < 4)
        return AVERROR_INVALIDDATA;

    s->h_sampling_rate = avctx->extradata[0];
    s->v_sampling_rate = avctx->extradata[1];
    s->y_shiftnum = avctx->extradata[2];
    s->video_quantize_shift = avctx->extradata[3];

    init_table(s, 0xFF);
    hvqm2_setup(avctx, s);

    return 0;
}

static int set_bit_reader(GetBitContext *dst, GetBitContext src, int offset)
{
    GetBitContext tmp;
    uint32_t size;
    int ret;

    ret = init_get_bits(&tmp, src.buffer, src.size_in_bits);
    if (ret < 0)
        return ret;
    if (get_bits_left(&tmp) < offset * 8)
        return AVERROR_INVALIDDATA;

    skip_bits_long(&tmp, offset*8);
    size = get_bits_long(&tmp, 32);
    size = FFMIN(get_bits_left(&tmp) >> 3, size);

    return init_get_bits8(dst, src.buffer+offset+4, size);
}

typedef struct HuffTree {
    int nb_codes;
    int max_bits;
    uint8_t bits[256];
    uint32_t codes[256];
    int16_t symbols[256];
} HuffTree;

static int read_trees(uint32_t length,
                      uint16_t code,
                      int scale,
                      HuffTree *huff,
                      GetBitContext *gb)
{
    int ret;

    if (get_bits_left(gb) < 1 ||
        get_bits1(gb) == 0) { // leaf node
        uint8_t byte = get_bits(gb, 8);
        int16_t symbol = byte;

        if (get_bits_left(gb) < 0)
            return AVERROR_INVALIDDATA;

        if (scale > 1)
            symbol = (int8_t)symbol * scale;
        else
            symbol *= scale;
        if (huff->nb_codes >= FF_ARRAY_ELEMS(huff->bits))
            return AVERROR_INVALIDDATA;
        huff->bits[huff->nb_codes] = FFMAX(length, 1);
        huff->codes[huff->nb_codes] = code;
        huff->symbols[huff->nb_codes] = symbol;
        huff->max_bits = FFMAX3(huff->max_bits, length, 1);
        huff->nb_codes++;
        return 0;
    } else { // recurse
        if (length >= 16)
            return AVERROR_INVALIDDATA;
        code <<= 1;
        ret = read_trees(length + 1, code|0, scale, huff, gb);
        if (ret < 0)
            return ret;
        return read_trees(length + 1, code|1, scale, huff, gb);
    }
}

static int read_vlc(AVCodecContext *avctx, GetBitContext *gb, VLC *vlc, int scale)
{
    HuffTree huff;
    int ret;

    huff.nb_codes = 0;
    huff.max_bits = 0;

    ret = read_trees(0, 0, scale, &huff, gb);
    if (ret < 0)
        return ret;

    if (huff.nb_codes > 0) {
        ff_vlc_free(vlc);
        return ff_vlc_init_sparse(vlc, FFMIN(huff.max_bits, 12),
                                  huff.nb_codes,
                                  huff.bits, 1, 1, huff.codes, 4, 4,
                                  huff.symbols, 2, 2, 0);
    }

    return 0;
}

static int16_t decode_huff(GetBitContext *gb, VLC *vlc)
{
    return get_vlc2(gb, vlc->table, vlc->bits, 3);
}

static void iframe_basis_num_dec(AVCodecContext *avctx)
{
    HVQM2Context *s = avctx->priv_data;
    uint8_t rle = 0;

    for (int i = 0; i < s->lum_totalblocks; i++) {
        if (rle) {
            s->basis[0][i] = 0;
            rle--;
        } else {
            uint8_t value = decode_huff(&s->basisnum_buf[0], &s->basisnum_vlc);

            if (value == 0)
                rle = decode_huff(&s->basisnumrun_buf[0], &s->basisnumrun_vlc);
            s->basis[0][i] = value;
        }
    }
    rle = 0;
    for (int i = 0; i < s->col_totalblocks; i++) {
        if (rle) {
            s->basis[1][i] = 0;
            s->basis[2][i] = 0;
            rle--;
        } else {
            uint8_t value = decode_huff(&s->basisnum_buf[1], &s->basisnum_vlc);

            if (value == 0)
                rle = decode_huff(&s->basisnumrun_buf[1], &s->basisnumrun_vlc);
            s->basis[1][i] = value & 0xF;
            s->basis[2][i] = value >> 4;
        }
    }
}

static int16_t decode_dc(AVCodecContext *avctx, GetBitContext *gb)
{
    HVQM2Context *s = avctx->priv_data;
    int16_t value = decode_huff(gb, &s->dcval_vlc);
    int16_t sum = value;

    if (value == s->dc_min || value == s->dc_max) {
        do {
            if (get_bits_left(gb) <= 0)
                break;

            value = decode_huff(gb, &s->dcval_vlc);
            sum += value;
        } while (value <= s->dc_min || value >= s->dc_max);
    }
    return sum;
}

static int16_t get_delta_dc(AVCodecContext *avctx, int plane_idx, int *rle)
{
    HVQM2Context *s = avctx->priv_data;

    if (*rle == 0) {
        int16_t delta = decode_dc(avctx, &s->dcval_buf[plane_idx]);

        if (delta == 0)
            rle[0] = decode_huff(&s->dcrun_buf[plane_idx], &s->basisnumrun_vlc);
        return delta;
    } else {
        rle[0]--;
        return 0;
    }
}

static uint32_t mean(uint32_t a, uint32_t b)
{
    return (a + b) / 2;
}

static void iframe_dc_dec(AVCodecContext *avctx)
{
    HVQM2Context *s = avctx->priv_data;
    uint8_t dcvalY = 0;
    uint8_t dcvalU = 0;
    uint8_t dcvalV = 0;
    int rleY = 0;
    int rleU = 0;
    int rleV = 0;
    uint8_t *dcbufY = s->dcbuf[0];
    uint8_t *dcbufU = s->dcbuf[1];
    uint8_t *dcbufV = s->dcbuf[2];
    uint8_t *dcbufY_prev = s->dcbuf[0];
    uint8_t *dcbufU_prev = s->dcbuf[1];
    uint8_t *dcbufV_prev = s->dcbuf[2];
    for (uint32_t h = 0; h < s->col_hblocks; h++) {
        dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY;
        dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY;
        dcvalU += get_delta_dc(avctx, 1, &rleU); *dcbufU++ = dcvalU;
        dcvalV += get_delta_dc(avctx, 2, &rleV); *dcbufV++ = dcvalV;
    }
    dcvalY = *dcbufY_prev++;
    dcvalU = *dcbufU_prev++;
    dcvalV = *dcbufV_prev++;
    if (s->mcu411) {
        for (uint32_t h = 0; h < s->col_hblocks; h++) {
            dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
            dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
        }
    }
    for (uint32_t v = 0; v < s->col_vblocks - 1; v++) {
        dcvalY = *(dcbufY_prev - 1);
        dcvalU = *(dcbufU_prev - 1);
        dcvalV = *(dcbufV_prev - 1);
        for (uint32_t h = 0; h < s->col_hblocks; h++) {
            dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
            dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
            dcvalU += get_delta_dc(avctx, 1, &rleU); *dcbufU++ = dcvalU; dcvalU = mean(dcvalU, *dcbufU_prev++);
            dcvalV += get_delta_dc(avctx, 2, &rleV); *dcbufV++ = dcvalV; dcvalV = mean(dcvalV, *dcbufV_prev++);
        }
        dcvalY = *(dcbufY_prev - 1);
        if (s->mcu411) {
            for (uint32_t h = 0; h < s->col_hblocks; h++) {
                dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
                dcvalY += get_delta_dc(avctx, 0, &rleY); *dcbufY++ = dcvalY; dcvalY = mean(dcvalY, *dcbufY_prev++);
            }
        }
    }
}

static void make_nest(AVCodecContext *avctx, uint16_t nest_start_x, uint16_t nest_start_y)
{
    HVQM2Context *s = avctx->priv_data;
    uint32_t h_nest_blocks, v_nest_blocks, h_mirror, v_mirror, h_empty, v_empty;
    uint8_t *nest, *ptr, *nest2;

    if (s->lum_hblocks < s->nest_w) {
        h_nest_blocks = s->lum_hblocks;
        h_mirror = s->lum_hblocks < s->nest_w - s->lum_hblocks ? s->lum_hblocks : s->nest_w - s->lum_hblocks;
        h_empty = s->nest_w - (h_nest_blocks + h_mirror);
    } else {
        h_nest_blocks = s->nest_w;
        h_mirror = 0;
        h_empty = 0;
    }

    if (s->lum_vblocks < s->nest_h) {
        v_nest_blocks = s->lum_vblocks;
        v_mirror = s->lum_vblocks < s->nest_h - s->lum_vblocks ? s->lum_vblocks : s->nest_h - s->lum_vblocks;
        v_empty = s->nest_w - (v_nest_blocks + v_mirror);
    } else {
        v_nest_blocks = s->nest_h;
        v_mirror = 0;
        v_empty = 0;
    }

    nest = s->nest;
    ptr = s->dcbuf[0] + s->lum_hblocks * nest_start_y + nest_start_x;
    for (uint32_t i = 0; i < v_nest_blocks; i++) {
        uint8_t const *p = ptr;
        for (uint32_t j = 0; j < h_nest_blocks; j++)
            *nest++ = *p++;
        for (uint32_t j = 0; j < h_mirror; j++)
            *nest++ = *--p;
        for (uint32_t j = 0; j < h_empty; j++)
            *nest++ = 0;
        ptr += s->lum_hblocks;
    }

    nest2 = nest - s->nest_w;
    for (uint32_t i = 0; i < v_mirror; i++) {
        for (uint32_t j = 0; j < s->nest_w; j++)
            *nest++ = nest2[j];
        nest2 -= s->nest_w;
    }

    for (uint32_t i = 0; i < v_empty; i++)
        for (uint32_t j = 0; j < s->nest_w; j++)
            *nest++ = 0;
}

static int decode_iframe(AVCodecContext *avctx, const HVQM2KeyFrame *keyframe)
{
    HVQM2Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int ret;

    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = set_bit_reader(&s->dcrun_buf[i], *gb, keyframe->dcrun_offset[i]);
        if (ret < 0)
            return ret;
    }
    iframe_basis_num_dec(avctx);
    iframe_dc_dec(avctx);
    make_nest(avctx, keyframe->nest_start_x, keyframe->nest_start_y);

    return 0;
}

static void advance_wcode(struct wcode *w, uint32_t amount)
{
    w->basis_prev_line += amount;
    w->dcbuf_prev_line += amount;
    w->basis_curr_line += amount;
    w->dcbuf_curr_line += amount;
    w->basis_next_line += amount;
    w->dcbuf_next_line += amount;
}

static void update_wcode(struct wcode *w)
{
    w->basis_curr = w->basis_next;
    w->dcbuf_curr = w->dcbuf_next;
    w->basis_curr_line++;
    w->dcbuf_curr_line++;
    w->basis_next = *w->basis_curr_line;
    w->dcbuf_next = *w->dcbuf_curr_line;
}

static void weight_block(int16_t *pix, uint8_t curr, uint8_t top, uint8_t bottom, uint8_t left, uint8_t right)
{
    int32_t const tmb = top - bottom;
    int32_t const lmr = left - right;
    int32_t const vph = tmb + lmr;
    int32_t const vmh = tmb - lmr;

    int32_t const c2 = curr * 2;
    int32_t const c8 = (curr * 8) + 4;

    int32_t const tpl = top    + left  - c2;
    int32_t const tpr = top    + right - c2;
    int32_t const bpr = bottom + right - c2;
    int32_t const bpl = bottom + left  - c2;

    int32_t const tml = top    - left;
    int32_t const tmr = top    - right;
    int32_t const bmr = bottom - right;
    int32_t const bml = bottom - left;

    pix[0] = (c8 + vph + tpl) / 8;
    pix[1] = (c8 + vph + tml) / 8;
    pix[2] = (c8 + vmh + tmr) / 8;
    pix[3] = (c8 + vmh + tpr) / 8;

    pix[4] = (c8 + vph - tml) / 8;
    pix[5] = (c8 - bpr      ) / 8;
    pix[6] = (c8 - bpl      ) / 8;
    pix[7] = (c8 + vmh - tmr) / 8;

    pix[8]  = (c8 - vmh - bml) / 8;
    pix[9]  = (c8 - tpr      ) / 8;
    pix[10] = (c8 - tpl      ) / 8;
    pix[11] = (c8 - vph - bmr) / 8;

    pix[12] = (c8 - vmh + bpl) / 8;
    pix[13] = (c8 - vmh + bml) / 8;
    pix[14] = (c8 - vph + bmr) / 8;
    pix[15] = (c8 - vph + bpr) / 8;
}

static void decode_block(AVCodecContext *avctx,
                        int16_t *pix, struct wcode *wcode, uint32_t plane_idx)
{
    HVQM2Context *s = avctx->priv_data;

    if (wcode->basis_curr == 0) {
        uint8_t curr   =  wcode->dcbuf_curr;
        uint8_t right  =  wcode->basis_next      == 0 ?  wcode->dcbuf_next      : curr;
        uint8_t top    = *wcode->basis_prev_line == 0 ? *wcode->dcbuf_prev_line : curr;
        uint8_t bottom = *wcode->basis_next_line == 0 ? *wcode->dcbuf_next_line : curr;
        uint8_t left   =  wcode->dcbuf_prev;

        weight_block(pix, curr, top, bottom, left, right);
        wcode->dcbuf_prev = wcode->dcbuf_curr;
    } else {
        if (wcode->basis_curr == 8) {
            uint8_t const *src = s->fixvl[plane_idx];

            for (uint32_t i = 0; i < 16; i++)
                pix[i] = src[i];
            s->fixvl[plane_idx] += 16;
        } else {
            for (uint32_t i = 0; i < 16; i++)
                pix[i] = wcode->dcbuf_curr;
            for (uint32_t i = 0; i < wcode->basis_curr; i++) {
                uint16_t bits = AV_RB16(s->fixvl[plane_idx]);
                uint32_t x_stride = ((bits >> 0) & 1) + 1;
                uint32_t y_stride = ((bits >> 1) & 1) + 1;
                int sum, mean, beta_q, scale, div, bar;
                uint32_t nest_pos_x, nest_pos_y;
                int16_t tmp[4][4], max;
                const uint8_t *nest;

                s->fixvl[plane_idx] += 2;
                if (s->yshift == 8) {
                    // landscape nest
                    nest_pos_y = ((bits >> 8) & 0x1F);
                    nest_pos_x = ((bits >> 2) & 0x3F);
                } else {
                    // portrait nest
                    nest_pos_y = ((bits >> 7) & 0x3F);
                    nest_pos_x = ((bits >> 2) & 0x1F);
                }
                nest = s->nest + nest_pos_y * s->nest_w + nest_pos_x;
                sum = 0;
                for (int y = 0; y < 4; y++) {
                    for (int x = 0; x < 4; x++) {
                        uint8_t nest_value = nest[y * y_stride * s->nest_w + x * x_stride];
                        tmp[y][x] = nest_value;
                        sum += nest_value;
                    }
                }
                mean = (sum + 8) >> 4;
                max = -1;
                for (int y = 0; y < 4; y++) {
                    for (int x = 0; x < 4; x++) {
                        int16_t value;

                        tmp[y][x] -= mean;
                        value = tmp[y][x];
                        if (value < 0)
                            value = -value;
                        if (max < value)
                            max = value;
                    }
                }
                beta_q = (bits >> 13) & 7;
                scale = decode_huff(&s->scale_buf[plane_idx], &s->scale_vlc);
                div = s->divT[max];
                bar = (beta_q + scale) * div;
                for (int y = 0; y < 4; y++)
                    for (int x = 0; x < 4; x++)
                        pix[y*4 + x] += (tmp[y][x] * bar + 512) / 1024;
            }
        }
        wcode->dcbuf_prev = wcode->dcbuf_next;
    }
    wcode->basis_prev_line++;
    wcode->dcbuf_prev_line++;
    wcode->basis_next_line++;
    wcode->dcbuf_next_line++;
}

static void copy_pix(uint8_t *dst, const int16_t *src)
{
    for (int i = 0; i < 4; i++)
        dst[i] = av_clip_uint8(src[i]);
}

static void copy_block(AVFrame *f,
                       int16_t pix_y[4][16],
                       int16_t pix_u[16],
                       int16_t pix_v[16],
                       int x, int y)
{
    uint8_t *dst_y0 = f->data[0] + f->linesize[0] * y*8 + x*8;
    uint8_t *dst_y1 = f->data[0] + f->linesize[0] * y*8 + x*8+4;
    uint8_t *dst_y2 = f->data[0] + f->linesize[0] * (y*8+4) + x*8;
    uint8_t *dst_y3 = f->data[0] + f->linesize[0] * (y*8+4) + x*8+4;
    uint8_t *dst_u = f->data[1] + f->linesize[1] * y*4 + x*4;
    uint8_t *dst_v = f->data[2] + f->linesize[2] * y*4 + x*4;

    for (int i = 0; i < 4; i++) {
        copy_pix(dst_y0, pix_y[0]+i*4);
        copy_pix(dst_y1, pix_y[1]+i*4);
        copy_pix(dst_y2, pix_y[2]+i*4);
        copy_pix(dst_y3, pix_y[3]+i*4);
        copy_pix(dst_u, pix_u+i*4);
        copy_pix(dst_v, pix_v+i*4);

        dst_y0 += f->linesize[0];
        dst_y1 += f->linesize[0];
        dst_y2 += f->linesize[0];
        dst_y3 += f->linesize[0];
        dst_u += f->linesize[1];
        dst_v += f->linesize[2];
    }
}

static void decode_line(AVCodecContext *avctx, AVFrame *frame, int line)
{
    HVQM2Context *s = avctx->priv_data;

    s->u_wcode.basis_next = *s->u_wcode.basis_curr_line;
    s->u_wcode.dcbuf_prev = *s->u_wcode.dcbuf_curr_line;
    s->u_wcode.dcbuf_next = *s->u_wcode.dcbuf_curr_line;
    s->v_wcode.basis_next = *s->v_wcode.basis_curr_line;
    s->v_wcode.dcbuf_prev = *s->v_wcode.dcbuf_curr_line;
    s->v_wcode.dcbuf_next = *s->v_wcode.dcbuf_curr_line;
    s->y0wcode.basis_next = *s->y0wcode.basis_curr_line;
    s->y0wcode.dcbuf_prev = *s->y0wcode.dcbuf_curr_line;
    s->y0wcode.dcbuf_next = *s->y0wcode.dcbuf_curr_line;
    if (s->mcu411) {
        s->y1wcode.basis_next = *s->y1wcode.basis_curr_line;
        s->y1wcode.dcbuf_prev = *s->y1wcode.dcbuf_curr_line;
        s->y1wcode.dcbuf_next = *s->y1wcode.dcbuf_curr_line;
    }
    for (uint32_t i = 0; i < s->col_hblocks-1; i++) {
        if (s->y0wcode.basis_next == 0x80) {
            advance_wcode(&s->y0wcode, 2);
            s->y0wcode.basis_next = *s->y0wcode.basis_curr_line;
            s->y0wcode.dcbuf_prev = *s->y0wcode.dcbuf_curr_line;
            s->y0wcode.dcbuf_next = *s->y0wcode.dcbuf_curr_line;
            if (s->mcu411) {
                advance_wcode(&s->y1wcode, 2);
                s->y1wcode.basis_next = *s->y1wcode.basis_curr_line;
                s->y1wcode.dcbuf_prev = *s->y1wcode.dcbuf_curr_line;
                s->y1wcode.dcbuf_next = *s->y1wcode.dcbuf_curr_line;
            }
            advance_wcode(&s->u_wcode, 1);
            s->u_wcode.basis_next = *s->u_wcode.basis_curr_line;
            s->u_wcode.dcbuf_prev = *s->u_wcode.dcbuf_curr_line;
            s->u_wcode.dcbuf_next = *s->u_wcode.dcbuf_curr_line;
            advance_wcode(&s->v_wcode, 1);
            s->v_wcode.basis_next = *s->v_wcode.basis_curr_line;
            s->v_wcode.dcbuf_prev = *s->v_wcode.dcbuf_curr_line;
            s->v_wcode.dcbuf_next = *s->v_wcode.dcbuf_curr_line;
        } else {
            update_wcode(&s->y0wcode);
            decode_block(avctx, s->pix_y[0], &s->y0wcode, 0);
            update_wcode(&s->y0wcode);
            decode_block(avctx, s->pix_y[1], &s->y0wcode, 0);
            if (s->mcu411) {
                update_wcode(&s->y1wcode);
                decode_block(avctx, s->pix_y[2], &s->y1wcode, 0);
                update_wcode(&s->y1wcode);
                decode_block(avctx, s->pix_y[3], &s->y1wcode, 0);
            }
            update_wcode(&s->u_wcode);
            decode_block(avctx, s->pix_u, &s->u_wcode, 1);
            update_wcode(&s->v_wcode);
            decode_block(avctx, s->pix_v, &s->v_wcode, 2);
            copy_block(frame, s->pix_y, s->pix_u, s->pix_v, i, line);
        }
    }
    if (s->y0wcode.basis_next == 0x80) {
        advance_wcode(&s->y0wcode, 2);
        if (s->mcu411)
            advance_wcode(&s->y1wcode, 2);
        advance_wcode(&s->u_wcode, 1);
        advance_wcode(&s->v_wcode, 1);
    } else {
        update_wcode(&s->y0wcode);
        decode_block(avctx, s->pix_y[0], &s->y0wcode, 0);
        s->y0wcode.basis_curr = s->y0wcode.basis_next;
        s->y0wcode.dcbuf_curr = s->y0wcode.dcbuf_next;
        s->y0wcode.basis_curr_line++;
        s->y0wcode.dcbuf_curr_line++;
        decode_block(avctx, s->pix_y[1], &s->y0wcode, 0);
        if (s->mcu411) {
            update_wcode(&s->y1wcode);
            decode_block(avctx, s->pix_y[2], &s->y1wcode, 0);
            s->y1wcode.basis_curr = s->y1wcode.basis_next;
            s->y1wcode.dcbuf_curr = s->y1wcode.dcbuf_next;
            s->y1wcode.basis_curr_line++;
            s->y1wcode.dcbuf_curr_line++;
            decode_block(avctx, s->pix_y[3], &s->y1wcode, 0);
        }
        s->u_wcode.basis_curr = s->u_wcode.basis_next;
        s->u_wcode.dcbuf_curr = s->u_wcode.dcbuf_next;
        s->u_wcode.basis_curr_line++;
        s->u_wcode.dcbuf_curr_line++;
        decode_block(avctx, s->pix_u, &s->u_wcode, 1);
        s->v_wcode.basis_curr = s->v_wcode.basis_next;
        s->v_wcode.dcbuf_curr = s->v_wcode.dcbuf_next;
        s->v_wcode.basis_curr_line++;
        s->v_wcode.dcbuf_curr_line++;
        decode_block(avctx, s->pix_v, &s->v_wcode, 2);
        copy_block(frame, s->pix_y, s->pix_u, s->pix_v, s->col_hblocks-1, line);
    }
}

static void decode(AVCodecContext *avctx, AVFrame *frame)
{
    HVQM2Context *s = avctx->priv_data;

    s->u_wcode.basis_prev_line = s->basis[1];
    s->u_wcode.dcbuf_prev_line = s->dcbuf[1];
    s->u_wcode.basis_curr_line = s->basis[1];
    s->u_wcode.dcbuf_curr_line = s->dcbuf[1];
    s->u_wcode.basis_next_line = s->basis[1] + s->col_hblocks;
    s->u_wcode.dcbuf_next_line = s->dcbuf[1] + s->col_hblocks;
    s->v_wcode.basis_prev_line = s->basis[2];
    s->v_wcode.dcbuf_prev_line = s->dcbuf[2];
    s->v_wcode.basis_curr_line = s->basis[2];
    s->v_wcode.dcbuf_curr_line = s->dcbuf[2];
    s->v_wcode.basis_next_line = s->basis[2] + s->col_hblocks;
    s->v_wcode.dcbuf_next_line = s->dcbuf[2] + s->col_hblocks;
    s->y0wcode.basis_prev_line = s->basis[0];
    s->y0wcode.dcbuf_prev_line = s->dcbuf[0];
    s->y0wcode.basis_curr_line = s->basis[0];
    s->y0wcode.dcbuf_curr_line = s->dcbuf[0];
    s->y0wcode.basis_next_line = s->basis[0] + s->lum_hblocks;
    s->y0wcode.dcbuf_next_line = s->dcbuf[0] + s->lum_hblocks;
    if (s->mcu411) {
        s->y1wcode.basis_prev_line = s->basis[0];
        s->y1wcode.dcbuf_prev_line = s->dcbuf[0];
        s->y1wcode.basis_curr_line = s->basis[0] + s->lum_hblocks;
        s->y1wcode.dcbuf_curr_line = s->dcbuf[0] + s->lum_hblocks;
        s->y1wcode.basis_next_line = s->basis[0] + s->lum_hblocks + s->lum_hblocks;
        s->y1wcode.dcbuf_next_line = s->dcbuf[0] + s->lum_hblocks + s->lum_hblocks;
    }
    decode_line(avctx, frame, 0);

    s->u_wcode.basis_prev_line = s->basis[1];
    s->u_wcode.dcbuf_prev_line = s->dcbuf[1];
    s->v_wcode.basis_prev_line = s->basis[2];
    s->v_wcode.dcbuf_prev_line = s->dcbuf[2];
    if (s->mcu411) {
        s->y0wcode.basis_curr_line += s->lum_hblocks;
        s->y0wcode.dcbuf_curr_line += s->lum_hblocks;
        s->y0wcode.basis_next_line += s->lum_hblocks;
        s->y0wcode.dcbuf_next_line += s->lum_hblocks;
        advance_wcode(&s->y1wcode, s->lum_hblocks);
    } else {
        s->y0wcode.basis_prev_line = s->basis[0];
        s->y0wcode.dcbuf_prev_line = s->dcbuf[0];
    }

    for (uint32_t line = 1; line < s->col_vblocks-1; line++) {
        decode_line(avctx, frame, line);
        if (s->mcu411) {
            advance_wcode(&s->y0wcode, s->lum_hblocks);
            advance_wcode(&s->y1wcode, s->lum_hblocks);
        }
    }

    s->u_wcode.basis_next_line = s->u_wcode.basis_curr_line;
    s->u_wcode.dcbuf_next_line = s->u_wcode.dcbuf_curr_line;
    s->v_wcode.basis_next_line = s->v_wcode.basis_curr_line;
    s->v_wcode.dcbuf_next_line = s->v_wcode.dcbuf_curr_line;
    if (s->mcu411) {
        s->y1wcode.basis_next_line = s->y1wcode.basis_curr_line;
        s->y1wcode.dcbuf_next_line = s->y1wcode.dcbuf_curr_line;
    } else {
        s->y0wcode.basis_next_line = s->y0wcode.basis_curr_line;
        s->y0wcode.dcbuf_next_line = s->y0wcode.dcbuf_curr_line;
    }
    decode_line(avctx, frame, s->col_vblocks-1);
}

static uint8_t get_delta_bn(HVQM2Context *s, uint8_t *rle, GetBitContext *val_gb, GetBitContext *rle_gb)
{
    uint8_t value;

    if (rle[0]) {
        rle[0]--;
        return 0;
    }

    value = decode_huff(val_gb, &s->basisnum_vlc);
    if (value == 0)
        rle[0] = decode_huff(rle_gb, &s->basisnumrun_vlc);

    return value;
}

static int decode_pframe(AVCodecContext *avctx, const HVQM2PredictFrame *predict)
{
    HVQM2Context *s = avctx->priv_data;
    uint8_t dc_y = 0, dc_u = 0, dc_v = 0;
    AVFrame *prev_frame = s->frame[1];
    AVFrame *frame = s->frame[0];
    GetBitContext *gb = &s->gb;
    int8_t mv_x = 0, mv_y = 0;
    uint8_t rle_lum = 0;
    uint8_t rle_col = 0;
    int ret;

    if (!prev_frame->data[0])
        return AVERROR_INVALIDDATA;

    ret = set_bit_reader(&s->mv_buf, *gb, predict->mv_offset);
    if (ret < 0)
        return ret;

    ret = read_vlc(avctx, &s->mv_buf, &s->mv_vlc, 1);
    if (ret < 0)
        return ret;

    ret = set_bit_reader(&s->mb_buf, *gb, predict->mb_offset);
    if (ret < 0)
        return ret;

    s->y0wcode.basis_curr_line = s->basis[0];
    s->y0wcode.dcbuf_curr_line = s->dcbuf[0];
    s->u_wcode.basis_curr_line = s->basis[1];
    s->u_wcode.dcbuf_curr_line = s->dcbuf[1];
    s->v_wcode.basis_curr_line = s->basis[2];
    s->v_wcode.dcbuf_curr_line = s->dcbuf[2];

    s->y0wcode.basis_next_line = s->basis[0] + s->lum_hblocks;
    s->y0wcode.dcbuf_next_line = s->dcbuf[0] + s->lum_hblocks;
    if (!s->mcu411) {
        s->u_wcode.basis_next_line = s->basis[1] + s->col_hblocks;
        s->u_wcode.dcbuf_next_line = s->dcbuf[1] + s->col_hblocks;
        s->v_wcode.basis_next_line = s->basis[2] + s->col_hblocks;
        s->v_wcode.dcbuf_next_line = s->dcbuf[2] + s->col_hblocks;
    }

    for (uint32_t y = 0; y < s->im_height; y += 8) {
        for (uint32_t x = 0; x < s->im_width; x += 8) {
            if (get_bits1(&s->mb_buf)) {
                if (get_bits1(&s->mb_buf)) {
                    dc_y += decode_dc(avctx, &s->dcval_buf[0]);
                    dc_u += decode_dc(avctx, &s->dcval_buf[1]);
                    dc_v += decode_dc(avctx, &s->dcval_buf[2]);
                    *s->y0wcode.basis_curr_line++ = 0;
                    *s->y0wcode.basis_curr_line++ = 0;
                    *s->y0wcode.dcbuf_curr_line++ = dc_y;
                    *s->y0wcode.dcbuf_curr_line++ = dc_y;
                    *s->y0wcode.basis_next_line++ = 0;
                    *s->y0wcode.basis_next_line++ = 0;
                    *s->y0wcode.dcbuf_next_line++ = dc_y;
                    *s->y0wcode.dcbuf_next_line++ = dc_y;
                    *s->u_wcode.basis_curr_line++ = 0;
                    *s->u_wcode.dcbuf_curr_line++ = dc_u;
                    *s->v_wcode.basis_curr_line++ = 0;
                    *s->v_wcode.dcbuf_curr_line++ = dc_v;
                    if (!s->mcu411) {
                        *s->u_wcode.basis_next_line++ = 0;
                        *s->u_wcode.dcbuf_next_line++ = dc_u;
                        *s->v_wcode.basis_next_line++ = 0;
                        *s->v_wcode.dcbuf_next_line++ = dc_v;
                    }
                } else {
                    uint8_t delta_bn;

                    *s->y0wcode.basis_curr_line++ = get_delta_bn(s, &rle_lum, &s->basisnum_buf[0], &s->basisnumrun_buf[0]);
                    *s->y0wcode.basis_curr_line++ = get_delta_bn(s, &rle_lum, &s->basisnum_buf[0], &s->basisnumrun_buf[0]);
                    *s->y0wcode.dcbuf_curr_line++ = (dc_y += decode_dc(avctx, &s->dcval_buf[0]));
                    *s->y0wcode.dcbuf_curr_line++ = (dc_y += decode_dc(avctx, &s->dcval_buf[0]));
                    *s->y0wcode.basis_next_line++ = get_delta_bn(s, &rle_lum, &s->basisnum_buf[0], &s->basisnumrun_buf[0]);
                    *s->y0wcode.basis_next_line++ = get_delta_bn(s, &rle_lum, &s->basisnum_buf[0], &s->basisnumrun_buf[0]);
                    *s->y0wcode.dcbuf_next_line++ = (dc_y += decode_dc(avctx, &s->dcval_buf[0]));
                    *s->y0wcode.dcbuf_next_line++ = (dc_y += decode_dc(avctx, &s->dcval_buf[0]));
                    delta_bn = get_delta_bn(s, &rle_col, &s->basisnum_buf[1], &s->basisnumrun_buf[1]);
                    *s->u_wcode.basis_curr_line++ = (delta_bn >> 0) & 0xF;
                    *s->v_wcode.basis_curr_line++ = (delta_bn >> 4) & 0xF;
                    *s->u_wcode.dcbuf_curr_line++ = (dc_u += decode_dc(avctx, &s->dcval_buf[1]));
                    *s->v_wcode.dcbuf_curr_line++ = (dc_v += decode_dc(avctx, &s->dcval_buf[2]));
                    if (!s->mcu411) {
                        delta_bn = get_delta_bn(s, &rle_col, &s->basisnum_buf[1], &s->basisnumrun_buf[1]);
                        *s->u_wcode.basis_next_line++ = (delta_bn >> 0) & 0xF;
                        *s->v_wcode.basis_next_line++ = (delta_bn >> 4) & 0xF;
                        *s->u_wcode.dcbuf_next_line++ = (dc_u += decode_dc(avctx, &s->dcval_buf[1]));
                        *s->v_wcode.dcbuf_next_line++ = (dc_v += decode_dc(avctx, &s->dcval_buf[2]));
                    }
                }
            } else {
                uint32_t src_y, src_x;
                uint8_t const *src;
                int8_t mx, my;
                uint8_t *dst;

                mx = decode_huff(&s->mv_buf, &s->mv_vlc);
                my = decode_huff(&s->mv_buf, &s->mv_vlc);

                mv_x += mx;
                mv_y += my;

                src_y = y + mv_y;
                src_x = x + mv_x;

                if (src_x > frame->width-8 || src_y > frame->height-8)
                    return AVERROR_INVALIDDATA;

                src = prev_frame->data[0] + src_x + src_y * prev_frame->linesize[0];
                dst = frame->data[0] + x + y * frame->linesize[0];
                for (int i = 0; i < 8; i++) {
                    memcpy(dst, src, 8);

                    src += prev_frame->linesize[0];
                    dst += frame->linesize[0];
                }

                src_x >>= 1;
                src_y >>= 1;

                for (int j = 1; j < 3; j++) {
                    src = prev_frame->data[j] + src_x + src_y * prev_frame->linesize[j];
                    dst = frame->data[j] + (x>>1) + (y>>1) * frame->linesize[j];
                    for (int i = 0; i < 4; i++) {
                        memcpy(dst, src, 4);

                        src += prev_frame->linesize[j];
                        dst += frame->linesize[j];
                    }
                }

                *s->y0wcode.basis_curr_line++ = 0x80;
                *s->y0wcode.basis_curr_line++ = 0x80;
                s->y0wcode.dcbuf_curr_line += 2;
                *s->y0wcode.basis_next_line++ = 0x80;
                *s->y0wcode.basis_next_line++ = 0x80;
                s->y0wcode.dcbuf_next_line += 2;

                *s->u_wcode.basis_curr_line++ = 0x80;
                s->u_wcode.dcbuf_curr_line++;
                *s->v_wcode.basis_curr_line++ = 0x80;
                s->v_wcode.dcbuf_curr_line++;
                if (!s->mcu411) {
                    *s->u_wcode.basis_next_line++ = 0x80;
                    s->u_wcode.dcbuf_next_line++;
                    *s->v_wcode.basis_next_line++ = 0x80;
                    s->v_wcode.dcbuf_next_line++;
                }
            }
        }

        s->y0wcode.basis_curr_line += s->lum_hblocks;
        s->y0wcode.dcbuf_curr_line += s->lum_hblocks;
        s->y0wcode.basis_next_line += s->lum_hblocks;
        s->y0wcode.dcbuf_next_line += s->lum_hblocks;
        if (!s->mcu411) {
            s->u_wcode.basis_curr_line += s->col_hblocks;
            s->u_wcode.dcbuf_curr_line += s->col_hblocks;
            s->u_wcode.basis_next_line += s->col_hblocks;
            s->u_wcode.dcbuf_next_line += s->col_hblocks;
            s->v_wcode.basis_curr_line += s->col_hblocks;
            s->v_wcode.dcbuf_curr_line += s->col_hblocks;
            s->v_wcode.basis_next_line += s->col_hblocks;
            s->v_wcode.dcbuf_next_line += s->col_hblocks;
        }
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVPacket *avpkt, int format,
                        AVFrame *frame, AVFrame *prev_frame, uint8_t *workbuf)
{
    HVQM2Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    HVQM2Frame header;
    int ret;

    if (format == HVQM2_VIDEO_HOLD) {
        av_frame_copy(frame, prev_frame);

        frame->pict_type = AV_PICTURE_TYPE_P;

        return 0;
    }

    if ((ret = init_get_bits8(gb, avpkt->data+2, avpkt->size-2)) < 0)
        return ret;

    header.basisnum_offset[0] = get_bits_long(gb, 32);
    header.basisnum_offset[1] = get_bits_long(gb, 32);
    header.basnumrn_offset[0] = get_bits_long(gb, 32);
    header.basnumrn_offset[1] = get_bits_long(gb, 32);

    for (int i = 0; i < PLANE_COUNT; i++)
        header.scale_offset[i] = get_bits_long(gb, 32);
    for (int i = 0; i < PLANE_COUNT; i++)
        header.fixvl_offset[i] = get_bits_long(gb, 32);
    for (int i = 0; i < PLANE_COUNT; i++)
        header.dcval_offset[i] = get_bits_long(gb, 32);

    s->basis[0] = workbuf; workbuf += s->lum_totalblocks;
    s->dcbuf[0] = workbuf; workbuf += s->lum_totalblocks;
    s->basis[1] = workbuf; workbuf += s->col_totalblocks;
    s->dcbuf[1] = workbuf; workbuf += s->col_totalblocks;
    s->basis[2] = workbuf; workbuf += s->col_totalblocks;
    s->dcbuf[2] = workbuf;
    for (int i = 0; i < 2; i++) {
        ret = set_bit_reader(&s->basisnum_buf[i], *gb, header.basisnum_offset[i]);
        if (ret < 0)
            return ret;

        ret = set_bit_reader(&s->basisnumrun_buf[i], *gb, header.basnumrn_offset[i]);
        if (ret < 0)
            return ret;
    }
    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = set_bit_reader(&s->scale_buf[i], *gb, header.scale_offset[i]);
        if (ret < 0)
            return ret;

        ret = set_bit_reader(&s->dcval_buf[i], *gb, header.dcval_offset[i]);
        if (ret < 0)
            return ret;

        s->fixvl[i] = gb->buffer + header.fixvl_offset[i] + 4;
    }

    ret = read_vlc(avctx, &s->basisnum_buf[0], &s->basisnum_vlc, 1);
    if (ret < 0)
        return ret;

    ret = read_vlc(avctx, &s->basisnumrun_buf[0], &s->basisnumrun_vlc, 1);
    if (ret < 0)
        return ret;

    ret = read_vlc(avctx, &s->scale_buf[0], &s->scale_vlc, 1 << 3);
    if (ret < 0)
        return ret;

    ret = read_vlc(avctx, &s->dcval_buf[0], &s->dcval_vlc, 1 << s->video_quantize_shift);
    if (ret < 0)
        return ret;

    s->nest = s->nestbuf;

    if (format == HVQM2_VIDEO_KEYFRAME) {
        HVQM2KeyFrame keyframe;

        for (int i = 0; i < PLANE_COUNT; i++)
            keyframe.dcrun_offset[i] = get_bits_long(gb, 32);
        keyframe.nest_start_x = get_bits(gb, 16);
        keyframe.nest_start_y = get_bits(gb, 16);

        ret = decode_iframe(avctx, &keyframe);
        if (ret < 0)
            return ret;

        decode(avctx, frame);

        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->flags |= AV_FRAME_FLAG_KEY;
    } else {
        HVQM2PredictFrame predict;

        predict.mv_offset = get_bits_long(gb, 32);
        predict.mb_offset = get_bits_long(gb, 32);

        ret = decode_pframe(avctx, &predict);
        if (ret < 0)
            return ret;

        decode(avctx, frame);

        frame->pict_type = AV_PICTURE_TYPE_P;
    }

    return 0;
}

static int hvqm2_decode(AVCodecContext *avctx, AVFrame *avframe,
                        int *got_frame, AVPacket *avpkt)
{
    HVQM2Context *s = avctx->priv_data;
    AVFrame *frame = s->frame[0];
    int frame_type, ret;

    if (avpkt->size < 2)
        return AVERROR_INVALIDDATA;

    frame_type = AV_RB16(avpkt->data);

    if ((ret = ff_reget_buffer(avctx, frame, 0)) < 0)
        return ret;

    ret = decode_frame(avctx, avpkt, frame_type, frame, s->frame[1], s->workbuf);
    if (ret < 0)
        return ret;

    ret = av_frame_ref(avframe, frame);
    if (ret < 0)
        return ret;

    FFSWAP(AVFrame *, s->frame[0], s->frame[1]);

    *got_frame = 1;

    return 0;
}

static void hvqm2_flush(AVCodecContext *avctx)
{
    HVQM2Context *s = avctx->priv_data;

    for (int i = 0; i < 2; i++)
        av_frame_unref(s->frame[i]);
}

static av_cold int hvqm2_close(AVCodecContext *avctx)
{
    HVQM2Context *s = avctx->priv_data;

    ff_vlc_free(&s->dcval_vlc);
    ff_vlc_free(&s->basisnum_vlc);
    ff_vlc_free(&s->basisnumrun_vlc);
    ff_vlc_free(&s->scale_vlc);
    ff_vlc_free(&s->mv_vlc);

    for (int i = 0; i < 2; i++)
        av_frame_free(&s->frame[i]);

    return 0;
}

const FFCodec ff_hvqm2_decoder = {
    .p.name         = "hvqm2",
    CODEC_LONG_NAME("HVQM2 Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_HVQM2,
    .priv_data_size = sizeof(HVQM2Context),
    .init           = hvqm2_init,
    FF_CODEC_DECODE_CB(hvqm2_decode),
    .flush          = hvqm2_flush,
    .close          = hvqm2_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
};
