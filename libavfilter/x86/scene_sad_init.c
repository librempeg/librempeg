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

#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/scene_sad.h"

#define SCENE_SAD_FUNC(FUNC_NAME, ASM_FUNC_NAME, MMSIZE)                      \
void ASM_FUNC_NAME(SCENE_SAD_PARAMS);                                         \
                                                                              \
static void FUNC_NAME(SCENE_SAD_PARAMS) {                                     \
    uint64_t sad[MMSIZE / 8] = {0};                                           \
    ptrdiff_t awidth = width & ~(MMSIZE - 1);                                 \
    *sum = 0;                                                                 \
    ASM_FUNC_NAME(src1, stride1, src2, stride2, awidth, height, sad);         \
    for (int i = 0; i < MMSIZE / 8; i++)                                      \
        *sum += sad[i];                                                       \
    ff_scene_sad_c(src1 + awidth, stride1,                                    \
                   src2 + awidth, stride2,                                    \
                   width - awidth, height, sad);                              \
    *sum += sad[0];                                                           \
}

#define SCENE_SAD16_FUNC(FUNC_NAME, ASM_FUNC_NAME, MMSIZE)                    \
void ASM_FUNC_NAME(SCENE_SAD_PARAMS);                                         \
                                                                              \
static void FUNC_NAME(SCENE_SAD_PARAMS) {                                     \
    uint64_t sad[MMSIZE / 8] = {0};                                           \
    ptrdiff_t bytes = (width << 1) & ~(MMSIZE - 1);                           \
    *sum = 0;                                                                 \
    ASM_FUNC_NAME(src1, stride1, src2, stride2, bytes, height, sad);          \
    for (int i = 0; i < MMSIZE / 8; i++)                                      \
        *sum += sad[i];                                                       \
    ff_scene_sad16_c(src1 + bytes, stride1,                                   \
                     src2 + bytes, stride2,                                   \
                     width - (bytes >> 1), height, sad);                      \
    *sum += sad[0];                                                           \
}

#if HAVE_X86ASM
SCENE_SAD_FUNC(scene_sad_sse2, ff_scene_sad8_sse2, 16)
#if HAVE_AVX2_EXTERNAL
SCENE_SAD_FUNC(scene_sad_avx2,     ff_scene_sad8_avx2,  32)
SCENE_SAD16_FUNC(scene_sad16_avx2, ff_scene_sad16_avx2, 32)
#endif
#if HAVE_AVX512_EXTERNAL
SCENE_SAD_FUNC(scene_sad_avx512,     ff_scene_sad8_avx512,  64)
SCENE_SAD16_FUNC(scene_sad16_avx512, ff_scene_sad16_avx512, 64)
#endif
#endif

ff_scene_sad_fn ff_scene_sad_get_fn_x86(int depth)
{
#if HAVE_X86ASM
    int cpu_flags = av_get_cpu_flags();
    if (depth <= 8) {
#if HAVE_AVX512_EXTERNAL
        if (EXTERNAL_AVX512(cpu_flags))
            return scene_sad_avx512;
#endif
#if HAVE_AVX2_EXTERNAL
        if (EXTERNAL_AVX2_FAST(cpu_flags))
            return scene_sad_avx2;
#endif
        if (EXTERNAL_SSE2(cpu_flags))
            return scene_sad_sse2;
    } else if (depth < 16) { /* this routine is only safe up to 15 bits */
#if HAVE_AVX512_EXTERNAL
        if (EXTERNAL_AVX512(cpu_flags))
            return scene_sad16_avx512;
#endif
#if HAVE_AVX2_EXTERNAL
        if (EXTERNAL_AVX2_FAST(cpu_flags))
            return scene_sad16_avx2;
#endif
    }
#endif
    return NULL;
}
