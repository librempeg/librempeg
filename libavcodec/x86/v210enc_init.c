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
#include "libavcodec/v210enc.h"

void ff_v210_planar_pack_8_ssse3(const uint8_t *y, const uint8_t *u,
                                 const uint8_t *v, uint8_t *dst,
                                 ptrdiff_t width);
void ff_v210_planar_pack_8_avx(const uint8_t *y, const uint8_t *u,
                               const uint8_t *v, uint8_t *dst, ptrdiff_t width);
void ff_v210_planar_pack_8_avx2(const uint8_t *y, const uint8_t *u,
                                const uint8_t *v, uint8_t *dst, ptrdiff_t width);
void ff_v210_planar_pack_8_avx512(const uint8_t *y, const uint8_t *u,
                                const uint8_t *v, uint8_t *dst, ptrdiff_t width);
void ff_v210_planar_pack_8_avx512icl(const uint8_t *y, const uint8_t *u,
                                const uint8_t *v, uint8_t *dst, ptrdiff_t width);
void ff_v210_planar_pack_10_ssse3(const uint16_t *y, const uint16_t *u,
                                  const uint16_t *v, uint8_t *dst,
                                  ptrdiff_t width);
void ff_v210_planar_pack_10_avx2(const uint16_t *y, const uint16_t *u,
                                 const uint16_t *v, uint8_t *dst,
                                 ptrdiff_t width);
void ff_v210_planar_pack_10_avx512(const uint16_t *y, const uint16_t *u,
                                   const uint16_t *v, uint8_t *dst,
                                   ptrdiff_t width);
void ff_v210_planar_pack_10_avx512icl(const uint16_t *y, const uint16_t *u,
                                      const uint16_t *v, uint8_t *dst,
                                      ptrdiff_t width);

av_cold void ff_v210enc_init_x86(V210EncContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSSE3(cpu_flags)) {
        s->pack_line_8 = ff_v210_planar_pack_8_ssse3;
        s->pack_line_10 = ff_v210_planar_pack_10_ssse3;
    }

    if (EXTERNAL_AVX(cpu_flags))
        s->pack_line_8 = ff_v210_planar_pack_8_avx;

    if (EXTERNAL_AVX2(cpu_flags)) {
        s->sample_factor_8  = 2;
        s->pack_line_8      = ff_v210_planar_pack_8_avx2;
        s->sample_factor_10 = 2;
        s->pack_line_10     = ff_v210_planar_pack_10_avx2;
    }

    if (EXTERNAL_AVX512(cpu_flags)) {
        s->sample_factor_8  = 2;
        s->pack_line_8      = ff_v210_planar_pack_8_avx512;
#if ARCH_X86_64
        s->sample_factor_10  = 2;
        s->pack_line_10      = ff_v210_planar_pack_10_avx512;
#endif
    }

    if (EXTERNAL_AVX512ICL(cpu_flags)) {
        s->sample_factor_8  = 4;
        s->pack_line_8      = ff_v210_planar_pack_8_avx512icl;
        s->sample_factor_10 = 4;
        s->pack_line_10     = ff_v210_planar_pack_10_avx512icl;
    }
}
