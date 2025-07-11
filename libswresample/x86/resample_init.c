/*
 * audio resampling
 * Copyright (c) 2004-2012 Michael Niedermayer <michaelni@gmx.at>
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
 * audio resampling
 * @author Michael Niedermayer <michaelni@gmx.at>
 */

#include "libavutil/attributes.h"
#include "libavutil/x86/cpu.h"
#include "libswresample/resample.h"

#define RESAMPLE_FUNCS(type, opt) \
int ff_resample_common_##type##_##opt(ResampleContext *c, void *dst, \
                                      const void *src, int sz, int upd); \
int ff_resample_linear_##type##_##opt(ResampleContext *c, void *dst, \
                                      const void *src, int sz, int upd)

RESAMPLE_FUNCS(int16,  sse2);
RESAMPLE_FUNCS(int16,  xop);
RESAMPLE_FUNCS(float,  sse);
RESAMPLE_FUNCS(float,  avx);
RESAMPLE_FUNCS(float,  fma3);
RESAMPLE_FUNCS(float,  fma4);
RESAMPLE_FUNCS(double, sse2);
RESAMPLE_FUNCS(double, avx);
RESAMPLE_FUNCS(double, fma3);

av_cold void swri_resample_dsp_x86_init(ResampleContext *c)
{
    int av_unused mm_flags = av_get_cpu_flags();

    switch(c->format){
    case AV_SAMPLE_FMT_S16P:
        if (EXTERNAL_SSE2(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_int16_sse2;
            c->dsp.resample_common = ff_resample_common_int16_sse2;
        }
        if (EXTERNAL_XOP(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_int16_xop;
            c->dsp.resample_common = ff_resample_common_int16_xop;
        }
        break;
    case AV_SAMPLE_FMT_FLTP:
        if (EXTERNAL_SSE(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_float_sse;
            c->dsp.resample_common = ff_resample_common_float_sse;
        }
        if (EXTERNAL_AVX_FAST(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_float_avx;
            c->dsp.resample_common = ff_resample_common_float_avx;
        }
        if (EXTERNAL_FMA3_FAST(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_float_fma3;
            c->dsp.resample_common = ff_resample_common_float_fma3;
        }
        if (EXTERNAL_FMA4(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_float_fma4;
            c->dsp.resample_common = ff_resample_common_float_fma4;
        }
        break;
    case AV_SAMPLE_FMT_DBLP:
        if (EXTERNAL_SSE2(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_double_sse2;
            c->dsp.resample_common = ff_resample_common_double_sse2;
        }
        if (EXTERNAL_AVX_FAST(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_double_avx;
            c->dsp.resample_common = ff_resample_common_double_avx;
        }
        if (EXTERNAL_FMA3_FAST(mm_flags)) {
            c->dsp.resample_linear = ff_resample_linear_double_fma3;
            c->dsp.resample_common = ff_resample_common_double_fma3;
        }
        break;
    }
}
