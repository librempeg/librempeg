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

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/dcadsp.h"

#define LFE_FIR_FLOAT_FUNC(opt)                                               \
void ff_lfe_fir0_float_##opt(float *pcm_samples, int32_t *lfe_samples,         \
                             const float *filter_coeff, ptrdiff_t npcmblocks); \
void ff_lfe_fir1_float_##opt(float *pcm_samples, int32_t *lfe_samples,         \
                             const float *filter_coeff, ptrdiff_t npcmblocks);

LFE_FIR_FLOAT_FUNC(sse2)
LFE_FIR_FLOAT_FUNC(sse3)
LFE_FIR_FLOAT_FUNC(avx)
LFE_FIR_FLOAT_FUNC(fma3)

av_cold void ff_dcadsp_init_x86(DCADSPContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE2(cpu_flags))
        s->lfe_fir_float[0] = ff_lfe_fir0_float_sse2;
    if (EXTERNAL_SSE3(cpu_flags))
        s->lfe_fir_float[1] = ff_lfe_fir1_float_sse3;
    if (EXTERNAL_AVX(cpu_flags)) {
        s->lfe_fir_float[0] = ff_lfe_fir0_float_avx;
        s->lfe_fir_float[1] = ff_lfe_fir1_float_avx;
    }
    if (EXTERNAL_FMA3(cpu_flags))
        s->lfe_fir_float[0] = ff_lfe_fir0_float_fma3;
}
