/*
 * Copyright (C) 2010 David Conrad
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

#include "libavutil/x86/cpu.h"
#include "libavcodec/diracdsp.h"
#include "fpel.h"

void ff_add_rect_clamped_sse2(uint8_t *, const uint16_t *, int, const int16_t *, int, int, int);

void ff_add_dirac_obmc8_sse2(uint16_t *dst, const uint8_t *src, int stride, const uint8_t *obmc_weight, int yblen);
void ff_add_dirac_obmc16_sse2(uint16_t *dst, const uint8_t *src, int stride, const uint8_t *obmc_weight, int yblen);
void ff_add_dirac_obmc32_sse2(uint16_t *dst, const uint8_t *src, int stride, const uint8_t *obmc_weight, int yblen);

void ff_put_rect_clamped_sse2(uint8_t *dst, int dst_stride, const int16_t *src, int src_stride, int width, int height);
void ff_put_signed_rect_clamped_sse2(uint8_t *dst, int dst_stride, const int16_t *src, int src_stride, int width, int height);
void ff_put_signed_rect_clamped_10_sse4(uint8_t *dst, int dst_stride, const uint8_t *src, int src_stride, int width, int height);

void ff_dequant_subband_32_sse4(uint8_t *src, uint8_t *dst, ptrdiff_t stride, const int qf, const int qs, int tot_v, int tot_h);

#if HAVE_X86ASM

#define HPEL_FILTER(MMSIZE, EXT)                                                             \
    void ff_dirac_hpel_filter_v_ ## EXT(uint8_t *, const uint8_t *, int, int);               \
    void ff_dirac_hpel_filter_h_ ## EXT(uint8_t *, const uint8_t *, int);                    \
                                                                                             \
    static void dirac_hpel_filter_ ## EXT(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc,       \
                                          const uint8_t *src, int stride, int width, int height)   \
    {                                                                                        \
        while( height-- )                                                                    \
        {                                                                                    \
            ff_dirac_hpel_filter_v_ ## EXT(dstv-MMSIZE, src-MMSIZE, stride, width+MMSIZE+5); \
            ff_dirac_hpel_filter_h_ ## EXT(dsth, src, width);                                \
            ff_dirac_hpel_filter_h_ ## EXT(dstc, dstv, width);                               \
                                                                                             \
            dsth += stride;                                                                  \
            dstv += stride;                                                                  \
            dstc += stride;                                                                  \
            src  += stride;                                                                  \
        }                                                                                    \
    }

#define DIRAC_PIXOP(OPNAME, EXT)\
static void OPNAME ## _dirac_pixels16_ ## EXT(uint8_t *dst, const uint8_t *src[5], \
                                              int stride, int h) \
{\
    if (h&3)\
        ff_ ## OPNAME ## _dirac_pixels16_c(dst, src, stride, h);\
    else\
        ff_ ## OPNAME ## _pixels16_ ## EXT(dst, src[0], stride, h);\
}\
static void OPNAME ## _dirac_pixels32_ ## EXT(uint8_t *dst, const uint8_t *src[5], \
                                              int stride, int h) \
{\
    if (h&3) {\
        ff_ ## OPNAME ## _dirac_pixels32_c(dst, src, stride, h);\
    } else {\
        ff_ ## OPNAME ## _pixels16_ ## EXT(dst   , src[0]   , stride, h);\
        ff_ ## OPNAME ## _pixels16_ ## EXT(dst+16, src[0]+16, stride, h);\
    }\
}

DIRAC_PIXOP(put, sse2)
DIRAC_PIXOP(avg, sse2)

HPEL_FILTER(16, sse2)

#endif // HAVE_X86ASM

void ff_diracdsp_init_x86(DiracDSPContext* c)
{
#if HAVE_X86ASM
    int mm_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE2(mm_flags)) {
        c->dirac_hpel_filter = dirac_hpel_filter_sse2;
        c->add_rect_clamped = ff_add_rect_clamped_sse2;
        c->put_signed_rect_clamped[0] = (void *)ff_put_signed_rect_clamped_sse2;

        c->add_dirac_obmc[0] = ff_add_dirac_obmc8_sse2;
        c->add_dirac_obmc[1] = ff_add_dirac_obmc16_sse2;
        c->add_dirac_obmc[2] = ff_add_dirac_obmc32_sse2;

        c->put_dirac_pixels_tab[1][0] = put_dirac_pixels16_sse2;
        c->avg_dirac_pixels_tab[1][0] = avg_dirac_pixels16_sse2;
        c->put_dirac_pixels_tab[2][0] = put_dirac_pixels32_sse2;
        c->avg_dirac_pixels_tab[2][0] = avg_dirac_pixels32_sse2;
    }

    if (EXTERNAL_SSE4(mm_flags)) {
        c->dequant_subband[1]         = ff_dequant_subband_32_sse4;
        c->put_signed_rect_clamped[1] = ff_put_signed_rect_clamped_10_sse4;
    }
#endif // HAVE_X86ASM
}
