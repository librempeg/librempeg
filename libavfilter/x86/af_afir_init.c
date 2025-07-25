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
#include "libavfilter/af_afirdsp.h"

void ff_fcmul_add_sse3(float *sum, const float *t, const float *c,
                       ptrdiff_t len);
void ff_fcmul_add_avx(float *sum, const float *t, const float *c,
                      ptrdiff_t len);
void ff_fcmul_add_fma3(float *sum, const float *t, const float *c,
                       ptrdiff_t len);

av_cold void ff_afir_init_x86(AudioFIRDSPContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE3(cpu_flags)) {
        s->fcmul_add = ff_fcmul_add_sse3;
    }
    if (EXTERNAL_AVX_FAST(cpu_flags)) {
        s->fcmul_add = ff_fcmul_add_avx;
    }
    if (EXTERNAL_FMA3_FAST(cpu_flags)) {
        s->fcmul_add = ff_fcmul_add_fma3;
    }
}
