/*
 * SIMD-optimized halfpel functions
 * Copyright (c) 2000, 2001 Fabrice Bellard
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
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
 *
 * MMX optimization by Nick Kurshev <nickols_k@mail.ru>
 */

#include <stddef.h>
#include <stdint.h>

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/hpeldsp.h"
#include "fpel.h"
#include "hpeldsp.h"
#include "inline_asm.h"

void ff_put_pixels8_x2_mmxext(uint8_t *block, const uint8_t *pixels,
                              ptrdiff_t line_size, int h);
void ff_put_pixels16_x2_sse2(uint8_t *block, const uint8_t *pixels,
                             ptrdiff_t line_size, int h);
void ff_avg_pixels16_x2_sse2(uint8_t *block, const uint8_t *pixels,
                             ptrdiff_t line_size, int h);
void ff_put_pixels16_y2_sse2(uint8_t *block, const uint8_t *pixels,
                             ptrdiff_t line_size, int h);
void ff_avg_pixels16_y2_sse2(uint8_t *block, const uint8_t *pixels,
                             ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels8_x2_mmxext(uint8_t *block, const uint8_t *pixels,
                                     ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels8_x2_exact_mmxext(uint8_t *block,
                                           const uint8_t *pixels,
                                           ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels16_x2_sse2(uint8_t *block, const uint8_t *pixels,
                                    ptrdiff_t line_size, int h);
void ff_avg_no_rnd_pixels16_x2_sse2(uint8_t *block, const uint8_t *pixels,
                                    ptrdiff_t line_size, int h);
void ff_put_pixels8_y2_mmxext(uint8_t *block, const uint8_t *pixels,
                              ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels8_y2_mmxext(uint8_t *block, const uint8_t *pixels,
                                     ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels8_y2_exact_mmxext(uint8_t *block,
                                           const uint8_t *pixels,
                                           ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels16_y2_sse2(uint8_t *block, const uint8_t *pixels,
                                    ptrdiff_t line_size, int h);
void ff_avg_no_rnd_pixels16_y2_sse2(uint8_t *block, const uint8_t *pixels,
                                    ptrdiff_t line_size, int h);
void ff_put_no_rnd_pixels16_xy2_sse2(uint8_t *block, const uint8_t *pixels,
                                     ptrdiff_t line_size, int h);
void ff_avg_no_rnd_pixels16_xy2_sse2(uint8_t *block, const uint8_t *pixels,
                                     ptrdiff_t line_size, int h);
void ff_avg_pixels8_x2_mmxext(uint8_t *block, const uint8_t *pixels,
                              ptrdiff_t line_size, int h);
void ff_avg_pixels8_y2_mmxext(uint8_t *block, const uint8_t *pixels,
                              ptrdiff_t line_size, int h);

#if HAVE_INLINE_ASM

/***********************************/
/* MMX no rounding */

// put_pixels
static void put_no_rnd_pixels8_xy2_mmx(uint8_t *block, const uint8_t *pixels,
                                       ptrdiff_t line_size, int h)
{
    MOVQ_ZERO(mm7);
    MOVQ_WONE(mm6); // =1 for no_rnd version
    __asm__ volatile(
        "movq   (%1), %%mm0             \n\t"
        "movq   1(%1), %%mm4            \n\t"
        "movq   %%mm0, %%mm1            \n\t"
        "movq   %%mm4, %%mm5            \n\t"
        "punpcklbw %%mm7, %%mm0         \n\t"
        "punpcklbw %%mm7, %%mm4         \n\t"
        "punpckhbw %%mm7, %%mm1         \n\t"
        "punpckhbw %%mm7, %%mm5         \n\t"
        "paddusw %%mm0, %%mm4           \n\t"
        "paddusw %%mm1, %%mm5           \n\t"
        "xor    %%"FF_REG_a", %%"FF_REG_a" \n\t"
        "add    %3, %1                  \n\t"
        ".p2align 3                     \n\t"
        "1:                             \n\t"
        "movq   (%1, %%"FF_REG_a"), %%mm0  \n\t"
        "movq   1(%1, %%"FF_REG_a"), %%mm2 \n\t"
        "movq   %%mm0, %%mm1            \n\t"
        "movq   %%mm2, %%mm3            \n\t"
        "punpcklbw %%mm7, %%mm0         \n\t"
        "punpcklbw %%mm7, %%mm2         \n\t"
        "punpckhbw %%mm7, %%mm1         \n\t"
        "punpckhbw %%mm7, %%mm3         \n\t"
        "paddusw %%mm2, %%mm0           \n\t"
        "paddusw %%mm3, %%mm1           \n\t"
        "paddusw %%mm6, %%mm4           \n\t"
        "paddusw %%mm6, %%mm5           \n\t"
        "paddusw %%mm0, %%mm4           \n\t"
        "paddusw %%mm1, %%mm5           \n\t"
        "psrlw  $2, %%mm4               \n\t"
        "psrlw  $2, %%mm5               \n\t"
        "packuswb  %%mm5, %%mm4         \n\t"
        "movq   %%mm4, (%2, %%"FF_REG_a")  \n\t"
        "add    %3, %%"FF_REG_a"           \n\t"

        "movq   (%1, %%"FF_REG_a"), %%mm2  \n\t" // 0 <-> 2   1 <-> 3
        "movq   1(%1, %%"FF_REG_a"), %%mm4 \n\t"
        "movq   %%mm2, %%mm3            \n\t"
        "movq   %%mm4, %%mm5            \n\t"
        "punpcklbw %%mm7, %%mm2         \n\t"
        "punpcklbw %%mm7, %%mm4         \n\t"
        "punpckhbw %%mm7, %%mm3         \n\t"
        "punpckhbw %%mm7, %%mm5         \n\t"
        "paddusw %%mm2, %%mm4           \n\t"
        "paddusw %%mm3, %%mm5           \n\t"
        "paddusw %%mm6, %%mm0           \n\t"
        "paddusw %%mm6, %%mm1           \n\t"
        "paddusw %%mm4, %%mm0           \n\t"
        "paddusw %%mm5, %%mm1           \n\t"
        "psrlw  $2, %%mm0               \n\t"
        "psrlw  $2, %%mm1               \n\t"
        "packuswb  %%mm1, %%mm0         \n\t"
        "movq   %%mm0, (%2, %%"FF_REG_a")  \n\t"
        "add    %3, %%"FF_REG_a"        \n\t"

        "subl   $2, %0                  \n\t"
        "jnz    1b                      \n\t"
        :"+g"(h), "+S"(pixels)
        :"D"(block), "r"((x86_reg)line_size)
        :FF_REG_a, "memory");
}

#endif /* HAVE_INLINE_ASM */

static void hpeldsp_init_mmx(HpelDSPContext *c, int flags)
{
#if HAVE_MMX_INLINE
    c->put_no_rnd_pixels_tab[1][3] = put_no_rnd_pixels8_xy2_mmx;
#endif
#if HAVE_MMX_EXTERNAL
    c->put_no_rnd_pixels_tab[1][0] =
    c->put_pixels_tab[1][0] = ff_put_pixels8_mmx;
#endif
}

static void hpeldsp_init_mmxext(HpelDSPContext *c, int flags)
{
#if HAVE_MMXEXT_EXTERNAL
    c->put_pixels_tab[1][1] = ff_put_pixels8_x2_mmxext;
    c->put_pixels_tab[1][2] = ff_put_pixels8_y2_mmxext;

    c->avg_pixels_tab[1][0] = ff_avg_pixels8_mmxext;
    c->avg_pixels_tab[1][1] = ff_avg_pixels8_x2_mmxext;
    c->avg_pixels_tab[1][2] = ff_avg_pixels8_y2_mmxext;

    c->put_no_rnd_pixels_tab[1][1] = ff_put_no_rnd_pixels8_x2_exact_mmxext;
    c->put_no_rnd_pixels_tab[1][2] = ff_put_no_rnd_pixels8_y2_exact_mmxext;

    if (!(flags & AV_CODEC_FLAG_BITEXACT)) {
        c->put_no_rnd_pixels_tab[1][1] = ff_put_no_rnd_pixels8_x2_mmxext;
        c->put_no_rnd_pixels_tab[1][2] = ff_put_no_rnd_pixels8_y2_mmxext;
    }
#endif /* HAVE_MMXEXT_EXTERNAL */
}

static void hpeldsp_init_sse2(HpelDSPContext *c, int flags)
{
#if HAVE_SSE2_EXTERNAL
    c->put_pixels_tab[0][0]        = ff_put_pixels16_sse2;
    c->put_pixels_tab[0][1]        = ff_put_pixels16_x2_sse2;
    c->put_pixels_tab[0][2]        = ff_put_pixels16_y2_sse2;
    c->put_pixels_tab[0][3]        = ff_put_pixels16_xy2_sse2;

    c->put_no_rnd_pixels_tab[0][0] = ff_put_pixels16_sse2;
    c->put_no_rnd_pixels_tab[0][1] = ff_put_no_rnd_pixels16_x2_sse2;
    c->put_no_rnd_pixels_tab[0][2] = ff_put_no_rnd_pixels16_y2_sse2;
    c->put_no_rnd_pixels_tab[0][3] = ff_put_no_rnd_pixels16_xy2_sse2;

    c->avg_pixels_tab[0][0]        = ff_avg_pixels16_sse2;
    c->avg_pixels_tab[0][1]        = ff_avg_pixels16_x2_sse2;
    c->avg_pixels_tab[0][2]        = ff_avg_pixels16_y2_sse2;
    c->avg_pixels_tab[0][3]        = ff_avg_pixels16_xy2_sse2;

    c->avg_no_rnd_pixels_tab[0]    = ff_avg_pixels16_sse2;
    c->avg_no_rnd_pixels_tab[1]    = ff_avg_no_rnd_pixels16_x2_sse2;
    c->avg_no_rnd_pixels_tab[2]    = ff_avg_no_rnd_pixels16_y2_sse2;
    c->avg_no_rnd_pixels_tab[3]    = ff_avg_no_rnd_pixels16_xy2_sse2;
#endif /* HAVE_SSE2_EXTERNAL */
}

static void hpeldsp_init_ssse3(HpelDSPContext *c, int flags)
{
#if HAVE_SSSE3_EXTERNAL
    c->put_pixels_tab[0][3]            = ff_put_pixels16_xy2_ssse3;
    c->avg_pixels_tab[0][3]            = ff_avg_pixels16_xy2_ssse3;
    c->put_pixels_tab[1][3]            = ff_put_pixels8_xy2_ssse3;
    c->avg_pixels_tab[1][3]            = ff_avg_pixels8_xy2_ssse3;
#endif
}

av_cold void ff_hpeldsp_init_x86(HpelDSPContext *c, int flags)
{
    int cpu_flags = av_get_cpu_flags();

    if (INLINE_MMX(cpu_flags))
        hpeldsp_init_mmx(c, flags);

    if (EXTERNAL_MMXEXT(cpu_flags))
        hpeldsp_init_mmxext(c, flags);

    if (EXTERNAL_SSE2(cpu_flags))
        hpeldsp_init_sse2(c, flags);

    if (EXTERNAL_SSSE3(cpu_flags))
        hpeldsp_init_ssse3(c, flags);
}
