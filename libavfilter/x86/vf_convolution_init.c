/*
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
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/convolution.h"

void ff_filter_3x3_sse4(uint8_t *dst, int width,
                        float rdiv, float bias, const int *const matrix,
                        const uint8_t *c[], int peak, int radius,
                        int dstride, int stride, int size);

void ff_filter_sobel_avx512icl(uint8_t *dst, int width,
                         float scale, float delta, const int *const matrix,
                         const uint8_t *c[], int peak, int radius,
                         int dstride, int stride, int size);

av_cold void ff_convolution_init_x86(ConvolutionContext *s)
{
#if ARCH_X86_64
    int cpu_flags = av_get_cpu_flags();
    for (int i = 0; i < 4; i++) {
        int matrix_length;

        switch (i) {
        case 0:
            matrix_length = s->matrix_length0;
            break;
        case 1:
            matrix_length = s->matrix_length1;
            break;
        case 2:
            matrix_length = s->matrix_length2;
            break;
        case 3:
            matrix_length = s->matrix_length3;
            break;
        }
        if (s->mode[i] == MATRIX_SQUARE) {
            if (matrix_length == 9 && s->depth == 8) {
                if (EXTERNAL_SSE4(cpu_flags))
                    s->filter[i] = ff_filter_3x3_sse4;
            }
        }
    }
#endif
}

av_cold void ff_sobel_init_x86(ConvolutionContext *s, int depth, int nb_planes)
{
#if ARCH_X86_64
    int cpu_flags = av_get_cpu_flags();
    for (int i = 0; i < nb_planes; i++) {
        if (depth == 8) {
            if (EXTERNAL_AVX512ICL(cpu_flags))
                s->filter[i] = ff_filter_sobel_avx512icl;
        }
    }
#endif
}
