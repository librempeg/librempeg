/*
 * AAC encoder assembly optimizations
 * Copyright (C) 2016 Rostislav Pehlivanov <atomnuker@gmail.com>
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
#include "libavutil/x86/cpu.h"
#include "libavcodec/aacencdsp.h"

void ff_abs_pow34_sse(float *out, const float *in, const int size);

void ff_aac_quantize_bands_sse2(int *out, const float *in, const float *scaled,
                                int size, int is_signed, int maxval, const float Q34,
                                const float rounding);
void ff_aac_quantize_bands_avx(int *out, const float *in, const float *scaled,
                               int size, int is_signed, int maxval, const float Q34,
                               const float rounding);

av_cold void ff_aacenc_dsp_init_x86(AACEncDSPContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE(cpu_flags))
        s->abs_pow34   = ff_abs_pow34_sse;

    if (EXTERNAL_SSE2(cpu_flags))
        s->quant_bands = ff_aac_quantize_bands_sse2;

    if (EXTERNAL_AVX_FAST(cpu_flags))
        s->quant_bands = ff_aac_quantize_bands_avx;
}
