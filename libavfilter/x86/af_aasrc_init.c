/*
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
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/af_aasrcdsp.h"

float ff_vector_fmul_real_fma3(const complex_float *cur, const complex_float *h,
                               const int N);

double ff_vector_dmul_real_fma3(const complex_double *cur, const complex_double *h,
                                const int N);

void ff_vector_fmul_complex_fma3(complex_float *x,
                                 const complex_float *a, const complex_float *b,
                                 const int N);

void ff_vector_dmul_complex_fma3(complex_double *x,
                                 const complex_double *a, const complex_double *b,
                                 const int N);

void ff_vector_fmul_complex_add_fma3(const float src,
                                     const complex_float *fixed,
                                     const complex_float *in,
                                     complex_float *out,
                                     const int N);

void ff_vector_dmul_complex_add_fma3(const double src,
                                     const complex_double *fixed,
                                     const complex_double *in,
                                     complex_double *out,
                                     const int N);

av_cold void ff_aasrc_init_x86(AudioASRCDSPContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_FMA3_FAST(cpu_flags)) {
        s->vector_fmul_real = ff_vector_fmul_real_fma3;
        s->vector_dmul_real = ff_vector_dmul_real_fma3;
        s->vector_fmul_complex = ff_vector_fmul_complex_fma3;
        s->vector_dmul_complex = ff_vector_dmul_complex_fma3;
        s->vector_fmul_complex_add = ff_vector_fmul_complex_add_fma3;
        s->vector_dmul_complex_add = ff_vector_dmul_complex_add_fma3;
    }
}
