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
#include "libavutil/x86/cpu.h"
#include "libavcodec/v210dec.h"

extern void ff_v210_planar_unpack_unaligned_ssse3(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);
extern void ff_v210_planar_unpack_unaligned_avx(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);
extern void ff_v210_planar_unpack_unaligned_avx2(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);

extern void ff_v210_planar_unpack_aligned_ssse3(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);
extern void ff_v210_planar_unpack_aligned_avx(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);
extern void ff_v210_planar_unpack_aligned_avx2(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);

extern void ff_v210_planar_unpack_avx512icl(const uint32_t *src, uint16_t *y, uint16_t *u, uint16_t *v, int width);

av_cold void ff_v210_x86_init(V210DecContext *s)
{
#if HAVE_X86ASM
    int cpu_flags = av_get_cpu_flags();

    if (s->aligned_input) {
        if (cpu_flags & AV_CPU_FLAG_SSSE3)
            s->unpack_frame = ff_v210_planar_unpack_aligned_ssse3;

        if (HAVE_AVX_EXTERNAL && cpu_flags & AV_CPU_FLAG_AVX)
            s->unpack_frame = ff_v210_planar_unpack_aligned_avx;

        if (HAVE_AVX2_EXTERNAL && cpu_flags & AV_CPU_FLAG_AVX2)
            s->unpack_frame = ff_v210_planar_unpack_aligned_avx2;

        if (EXTERNAL_AVX512ICL(cpu_flags))
            s->unpack_frame = ff_v210_planar_unpack_avx512icl;
    }
    else {
        if (cpu_flags & AV_CPU_FLAG_SSSE3)
            s->unpack_frame = ff_v210_planar_unpack_unaligned_ssse3;

        if (HAVE_AVX_EXTERNAL && cpu_flags & AV_CPU_FLAG_AVX)
            s->unpack_frame = ff_v210_planar_unpack_unaligned_avx;

        if (HAVE_AVX2_EXTERNAL && cpu_flags & AV_CPU_FLAG_AVX2)
            s->unpack_frame = ff_v210_planar_unpack_unaligned_avx2;

        if (EXTERNAL_AVX512ICL(cpu_flags))
            s->unpack_frame = ff_v210_planar_unpack_avx512icl;
    }
#endif
}
