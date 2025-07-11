/*
 * GMC (Global Motion Compensation), AltiVec-enabled
 *
 * Copyright (c) 2003 Romain Dolbeau <romain@dolbeau.org>
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

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/mem_internal.h"
#include "libavutil/ppc/cpu.h"
#include "libavutil/ppc/util_altivec.h"

#include "libavcodec/mpeg4videodsp.h"

#if HAVE_ALTIVEC
/* AltiVec-enhanced gmc1. ATM this code assumes stride is a multiple of 8
 * to preserve proper dst alignment. */
static void gmc1_altivec(uint8_t *dst /* align 8 */, const uint8_t *src /* align1 */,
                         int stride, int h, int x16, int y16, int rounder)
{
    int i;
    const DECLARE_ALIGNED(16, unsigned short, rounder_a) = rounder;
    const DECLARE_ALIGNED(16, unsigned short, ABCD)[8] = {
        (16 - x16) * (16 - y16), /* A */
             (x16) * (16 - y16), /* B */
        (16 - x16) * (y16),      /* C */
             (x16) * (y16),      /* D */
        0, 0, 0, 0               /* padding */
    };
    register const vector unsigned char vczero =
        (const vector unsigned char) vec_splat_u8(0);
    register const vector unsigned short vcsr8 =
        (const vector unsigned short) vec_splat_u16(8);
    register vector unsigned char dstv, dstv2, srcvB, srcvC, srcvD;
    register vector unsigned short tempB, tempC, tempD;
    unsigned long dst_odd        = (unsigned long) dst & 0x0000000F;
    unsigned long src_really_odd = (unsigned long) src & 0x0000000F;
    register vector unsigned short tempA =
        vec_ld(0, (const unsigned short *) ABCD);
    register vector unsigned short Av = vec_splat(tempA, 0);
    register vector unsigned short Bv = vec_splat(tempA, 1);
    register vector unsigned short Cv = vec_splat(tempA, 2);
    register vector unsigned short Dv = vec_splat(tempA, 3);
    register vector unsigned short rounderV =
        vec_splat((vec_u16) vec_lde(0, &rounder_a), 0);

    /* we'll be able to pick-up our 9 char elements at src from those
     * 32 bytes we load the first batch here, as inside the loop we can
     * reuse 'src + stride' from one iteration as the 'src' of the next. */
    register vector unsigned char src_0 = vec_ld(0, src);
    register vector unsigned char src_1 = vec_ld(16, src);
    register vector unsigned char srcvA = vec_perm(src_0, src_1,
                                                   vec_lvsl(0, src));

    if (src_really_odd != 0x0000000F)
        /* If (src & 0xF) == 0xF, then (src + 1) is properly aligned
         * on the second vector. */
        srcvB = vec_perm(src_0, src_1, vec_lvsl(1, src));
    else
        srcvB = src_1;
    srcvA = vec_mergeh(vczero, srcvA);
    srcvB = vec_mergeh(vczero, srcvB);

    for (i = 0; i < h; i++) {
        dst_odd        =   (unsigned long) dst            & 0x0000000F;
        src_really_odd = (((unsigned long) src) + stride) & 0x0000000F;

        dstv = vec_ld(0, dst);

        /* We'll be able to pick-up our 9 char elements at src + stride from
         * those 32 bytes then reuse the resulting 2 vectors srvcC and srcvD
         * as the next srcvA and srcvB. */
        src_0 = vec_ld(stride +  0, src);
        src_1 = vec_ld(stride + 16, src);
        srcvC = vec_perm(src_0, src_1, vec_lvsl(stride + 0, src));

        if (src_really_odd != 0x0000000F)
            /* If (src & 0xF) == 0xF, then (src + 1) is properly aligned
             * on the second vector. */
            srcvD = vec_perm(src_0, src_1, vec_lvsl(stride + 1, src));
        else
            srcvD = src_1;

        srcvC = vec_mergeh(vczero, srcvC);
        srcvD = vec_mergeh(vczero, srcvD);

        /* OK, now we (finally) do the math :-)
         * Those four instructions replace 32 int muls & 32 int adds.
         * Isn't AltiVec nice? */
        tempA = vec_mladd((vector unsigned short) srcvA, Av, rounderV);
        tempB = vec_mladd((vector unsigned short) srcvB, Bv, tempA);
        tempC = vec_mladd((vector unsigned short) srcvC, Cv, tempB);
        tempD = vec_mladd((vector unsigned short) srcvD, Dv, tempC);

        srcvA = srcvC;
        srcvB = srcvD;

        tempD = vec_sr(tempD, vcsr8);

        dstv2 = vec_pack(tempD, (vector unsigned short) vczero);

        if (dst_odd)
            dstv2 = vec_perm(dstv, dstv2, vcprm(0, 1, s0, s1));
        else
            dstv2 = vec_perm(dstv, dstv2, vcprm(s0, s1, 2, 3));

        vec_st(dstv2, 0, dst);

        dst += stride;
        src += stride;
    }
}
#endif /* HAVE_ALTIVEC */

av_cold void ff_mpeg4videodsp_init_ppc(Mpeg4VideoDSPContext *c)
{
#if HAVE_ALTIVEC
    if (!PPC_ALTIVEC(av_get_cpu_flags()))
        return;

    c->gmc1 = gmc1_altivec;
#endif /* HAVE_ALTIVEC */
}
