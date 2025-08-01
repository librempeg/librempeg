/*
 * Copyright (C) 2009 David Conrad
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

#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "diracdsp.h"

#define FILTER(src, stride)                                     \
    ((21*((src)[ 0*stride] + (src)[1*stride])                   \
      -7*((src)[-1*stride] + (src)[2*stride])                   \
      +3*((src)[-2*stride] + (src)[3*stride])                   \
      -1*((src)[-3*stride] + (src)[4*stride]) + 16) >> 5)

static void dirac_hpel_filter(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc, const uint8_t *src,
                              int stride, int width, int height)
{
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = -3; x < width+5; x++)
            dstv[x] = av_clip_uint8(FILTER(src+x, stride));

        for (x = 0; x < width; x++)
            dstc[x] = av_clip_uint8(FILTER(dstv+x, 1));

        for (x = 0; x < width; x++)
            dsth[x] = av_clip_uint8(FILTER(src+x, 1));

        src  += stride;
        dsth += stride;
        dstv += stride;
        dstc += stride;
    }
}

#define PIXOP_BILINEAR(PFX, OP, WIDTH)                                  \
    static void ff_ ## PFX ## _dirac_pixels ## WIDTH ## _bilinear_c(uint8_t *dst, const uint8_t *src[5], int stride, int h) \
    {                                                                   \
        int x;                                                          \
        const uint8_t *s0 = src[0];                                     \
        const uint8_t *s1 = src[1];                                     \
        const uint8_t *s2 = src[2];                                     \
        const uint8_t *s3 = src[3];                                     \
        const uint8_t *w  = src[4];                                     \
                                                                        \
        while (h--) {                                                   \
            for (x = 0; x < WIDTH; x++) {                               \
                OP(dst[x], (s0[x]*w[0] + s1[x]*w[1] + s2[x]*w[2] + s3[x]*w[3] + 8) >> 4); \
            }                                                           \
                                                                        \
            dst += stride;                                              \
            s0 += stride;                                               \
            s1 += stride;                                               \
            s2 += stride;                                               \
            s3 += stride;                                               \
        }                                                               \
    }

#define OP_PUT(dst, val) (dst) = (val)
#define OP_AVG(dst, val) (dst) = (((dst) + (val) + 1)>>1)

PIXOP_BILINEAR(put, OP_PUT, 8)
PIXOP_BILINEAR(put, OP_PUT, 16)
PIXOP_BILINEAR(put, OP_PUT, 32)
PIXOP_BILINEAR(avg, OP_AVG, 8)
PIXOP_BILINEAR(avg, OP_AVG, 16)
PIXOP_BILINEAR(avg, OP_AVG, 32)

#define op_scale1(x)  block[x] = av_clip_uint8( (block[x]*weight + (1<<(log2_denom-1))) >> log2_denom)
#define op_scale2(x)  dst[x] = av_clip_uint8( (src[x]*weights + dst[x]*weightd + (1<<(log2_denom-1))) >> log2_denom)

#define DIRAC_WEIGHT(W)                                                 \
    static void weight_dirac_pixels ## W ## _c(uint8_t *block, int stride, int log2_denom, \
                                               int weight, int h) {     \
        int x;                                                          \
        while (h--) {                                                   \
            for (x = 0; x < W; x++) {                                   \
                op_scale1(x);                                           \
                op_scale1(x+1);                                         \
            }                                                           \
            block += stride;                                            \
        }                                                               \
    }                                                                   \
    static void biweight_dirac_pixels ## W ## _c(uint8_t *dst, const uint8_t *src, int stride, int log2_denom, \
                                                 int weightd, int weights, int h) { \
        int x;                                                          \
        while (h--) {                                                   \
            for (x = 0; x < W; x++) {                                   \
                op_scale2(x);                                           \
                op_scale2(x+1);                                         \
            }                                                           \
            dst += stride;                                              \
            src += stride;                                              \
        }                                                               \
    }

DIRAC_WEIGHT(8)
DIRAC_WEIGHT(16)
DIRAC_WEIGHT(32)

#define ADD_OBMC(xblen)                                                 \
    static void add_obmc ## xblen ## _c(uint16_t *dst, const uint8_t *src, int stride, \
                                        const uint8_t *obmc_weight, int yblen) \
    {                                                                   \
        int x;                                                          \
        while (yblen--) {                                               \
            for (x = 0; x < xblen; x += 2) {                            \
                dst[x  ] += src[x  ] * obmc_weight[x  ];                \
                dst[x+1] += src[x+1] * obmc_weight[x+1];                \
            }                                                           \
            dst += stride;                                              \
            src += stride;                                              \
            obmc_weight += 32;                                          \
        }                                                               \
    }

ADD_OBMC(8)
ADD_OBMC(16)
ADD_OBMC(32)

static void put_signed_rect_clamped_8bit_c(uint8_t *dst, int dst_stride, const uint8_t *_src, int src_stride, int width, int height)
{
    int x, y;
    const int16_t *src = (const int16_t *)_src;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x+=4) {
            dst[x  ] = av_clip_uint8(src[x  ] + 128);
            dst[x+1] = av_clip_uint8(src[x+1] + 128);
            dst[x+2] = av_clip_uint8(src[x+2] + 128);
            dst[x+3] = av_clip_uint8(src[x+3] + 128);
        }
        dst += dst_stride;
        src += src_stride >> 1;
    }
}

#define PUT_SIGNED_RECT_CLAMPED(PX)                                                                     \
static void put_signed_rect_clamped_ ## PX ## bit_c(uint8_t *_dst, int dst_stride, const uint8_t *_src, \
                                                  int src_stride, int width, int height)                \
{                                                                                                       \
    int x, y;                                                                                           \
    uint16_t *dst = (uint16_t *)_dst;                                                                   \
    const int32_t *src = (const int32_t *)_src;                                                         \
    for (y = 0; y < height; y++) {                                                                      \
        for (x = 0; x < width; x+=4) {                                                                  \
            dst[x  ] = av_clip_uintp2(src[x  ] + (1U << (PX - 1)), PX);                                  \
            dst[x+1] = av_clip_uintp2(src[x+1] + (1U << (PX - 1)), PX);                                  \
            dst[x+2] = av_clip_uintp2(src[x+2] + (1U << (PX - 1)), PX);                                  \
            dst[x+3] = av_clip_uintp2(src[x+3] + (1U << (PX - 1)), PX);                                  \
        }                                                                                               \
        dst += dst_stride >> 1;                                                                         \
        src += src_stride >> 2;                                                                         \
    }                                                                                                   \
}

PUT_SIGNED_RECT_CLAMPED(10)
PUT_SIGNED_RECT_CLAMPED(12)

static void add_rect_clamped_c(uint8_t *dst, const uint16_t *src, int stride,
                               const int16_t *idwt, int idwt_stride,
                               int width, int height)
{
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x+=2) {
            dst[x  ] = av_clip_uint8(((src[x  ]+32)>>6) + idwt[x  ]);
            dst[x+1] = av_clip_uint8(((src[x+1]+32)>>6) + idwt[x+1]);
        }
        dst += stride;
        src += stride;
        idwt += idwt_stride;
    }
}

#define DEQUANT_SUBBAND(PX)                                                                \
static void dequant_subband_ ## PX ## _c(uint8_t *src, uint8_t *dst, ptrdiff_t stride,     \
                                         const int qf, const int qs, int tot_v, int tot_h) \
{                                                                                          \
    int i, y;                                                                              \
    for (y = 0; y < tot_v; y++) {                                                          \
        PX c, *src_r = (PX *)src, *dst_r = (PX *)dst;                                      \
        for (i = 0; i < tot_h; i++) {                                                      \
            c = *src_r++;                                                                  \
            if     (c < 0) c = -((-(unsigned)c*qf + qs) >> 2);                             \
            else if(c > 0) c =  (( (unsigned)c*qf + qs) >> 2);                             \
            *dst_r++ = c;                                                                  \
        }                                                                                  \
        src += tot_h << (sizeof(PX) >> 1);                                                 \
        dst += stride;                                                                     \
    }                                                                                      \
}

DEQUANT_SUBBAND(int16_t)
DEQUANT_SUBBAND(int32_t)

#define PIXFUNC(PFX, WIDTH)                                             \
    c->PFX ## _dirac_pixels_tab[WIDTH>>4][0] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _c; \
    c->PFX ## _dirac_pixels_tab[WIDTH>>4][1] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _l2_c; \
    c->PFX ## _dirac_pixels_tab[WIDTH>>4][2] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _l4_c; \
    c->PFX ## _dirac_pixels_tab[WIDTH>>4][3] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _bilinear_c

av_cold void ff_diracdsp_init(DiracDSPContext *c)
{
    c->dirac_hpel_filter = dirac_hpel_filter;
    c->add_rect_clamped = add_rect_clamped_c;
    c->put_signed_rect_clamped[0] = put_signed_rect_clamped_8bit_c;
    c->put_signed_rect_clamped[1] = put_signed_rect_clamped_10bit_c;
    c->put_signed_rect_clamped[2] = put_signed_rect_clamped_12bit_c;

    c->add_dirac_obmc[0] = add_obmc8_c;
    c->add_dirac_obmc[1] = add_obmc16_c;
    c->add_dirac_obmc[2] = add_obmc32_c;

    c->weight_dirac_pixels_tab[0] = weight_dirac_pixels8_c;
    c->weight_dirac_pixels_tab[1] = weight_dirac_pixels16_c;
    c->weight_dirac_pixels_tab[2] = weight_dirac_pixels32_c;
    c->biweight_dirac_pixels_tab[0] = biweight_dirac_pixels8_c;
    c->biweight_dirac_pixels_tab[1] = biweight_dirac_pixels16_c;
    c->biweight_dirac_pixels_tab[2] = biweight_dirac_pixels32_c;

    c->dequant_subband[0] = c->dequant_subband[2] = dequant_subband_int16_t_c;
    c->dequant_subband[1] = c->dequant_subband[3] = dequant_subband_int32_t_c;

    PIXFUNC(put, 8);
    PIXFUNC(put, 16);
    PIXFUNC(put, 32);
    PIXFUNC(avg, 8);
    PIXFUNC(avg, 16);
    PIXFUNC(avg, 32);

#if ARCH_X86
    ff_diracdsp_init_x86(c);
#endif
}
