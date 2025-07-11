/*
 * Copyright (c) 2002 Brian Foley
 * Copyright (c) 2002 Dieter Shirley
 * Copyright (c) 2003-2004 Romain Dolbeau <romain@dolbeau.org>
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
#include "libavutil/ppc/cpu.h"
#include "libavutil/ppc/util_altivec.h"

#include "libavcodec/hpeldsp.h"

#include "hpeldsp_altivec.h"

#if HAVE_ALTIVEC
/* next one assumes that ((line_size % 16) == 0) */
void ff_put_pixels16_altivec(uint8_t *block, const uint8_t *pixels, ptrdiff_t line_size, int h)
{
    register vector unsigned char pixelsv1;
    register vector unsigned char pixelsv1B;
    register vector unsigned char pixelsv1C;
    register vector unsigned char pixelsv1D;

    int i;
    register ptrdiff_t line_size_2 = line_size * (1 << 1);
    register ptrdiff_t line_size_3 = line_size + line_size_2;
    register ptrdiff_t line_size_4 = line_size * (1 << 2);

// hand-unrolling the loop by 4 gains about 15%
// mininum execution time goes from 74 to 60 cycles
// it's faster than -funroll-loops, but using
// -funroll-loops w/ this is bad - 74 cycles again.
// all this is on a 7450, tuning for the 7450
    for (i = 0; i < h; i += 4) {
        pixelsv1  = unaligned_load( 0, pixels);
        pixelsv1B = unaligned_load(line_size, pixels);
        pixelsv1C = unaligned_load(line_size_2, pixels);
        pixelsv1D = unaligned_load(line_size_3, pixels);
        VEC_ST(pixelsv1, 0, (unsigned char*)block);
        VEC_ST(pixelsv1B, line_size, (unsigned char*)block);
        VEC_ST(pixelsv1C, line_size_2, (unsigned char*)block);
        VEC_ST(pixelsv1D, line_size_3, (unsigned char*)block);
        pixels+=line_size_4;
        block +=line_size_4;
    }
}

/* next one assumes that ((line_size % 16) == 0) */
#define op_avg(a,b)  a = ( ((a)|(b)) - ((((a)^(b))&0xFEFEFEFEUL)>>1) )
void ff_avg_pixels16_altivec(uint8_t *block, const uint8_t *pixels, ptrdiff_t line_size, int h)
{
    register vector unsigned char pixelsv, blockv;

    int i;
    for (i = 0; i < h; i++) {
        blockv = vec_ld(0, block);
        pixelsv = VEC_LD( 0, pixels);
        blockv = vec_avg(blockv,pixelsv);
        vec_st(blockv, 0, (unsigned char*)block);
        pixels+=line_size;
        block +=line_size;
    }
}

/* next one assumes that ((line_size % 8) == 0) */
static void avg_pixels8_altivec(uint8_t * block, const uint8_t * pixels, ptrdiff_t line_size, int h)
{
    register vector unsigned char pixelsv, blockv;
    int i;

   for (i = 0; i < h; i++) {
       /* block is 8 bytes-aligned, so we're either in the
          left block (16 bytes-aligned) or in the right block (not) */
       int rightside = ((unsigned long)block & 0x0000000F);

       blockv = vec_ld(0, block);
       pixelsv = VEC_LD( 0, pixels);

       if (rightside) {
           pixelsv = vec_perm(blockv, pixelsv, vcprm(0,1,s0,s1));
       } else {
           pixelsv = vec_perm(blockv, pixelsv, vcprm(s0,s1,2,3));
       }

       blockv = vec_avg(blockv, pixelsv);

       vec_st(blockv, 0, block);

       pixels += line_size;
       block += line_size;
   }
}

/* next one assumes that ((line_size % 8) == 0) */
static void put_pixels8_xy2_altivec(uint8_t *block, const uint8_t *pixels, ptrdiff_t line_size, int h)
{
    register int i;
    register vector unsigned char pixelsv1, pixelsv2, pixelsavg;
    register vector unsigned char blockv;
    register vector unsigned short pixelssum1, pixelssum2, temp3;
    register const vector unsigned char vczero = (const vector unsigned char)vec_splat_u8(0);
    register const vector unsigned short vctwo = (const vector unsigned short)vec_splat_u16(2);

    pixelsv1 = VEC_LD(0, pixels);
    pixelsv2 = VEC_LD(1, pixels);
    pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
    pixelsv2 = VEC_MERGEH(vczero, pixelsv2);

    pixelssum1 = vec_add((vector unsigned short)pixelsv1,
                         (vector unsigned short)pixelsv2);
    pixelssum1 = vec_add(pixelssum1, vctwo);

    for (i = 0; i < h ; i++) {
        int rightside = ((unsigned long)block & 0x0000000F);
        blockv = vec_ld(0, block);

        pixelsv1 = unaligned_load(line_size, pixels);
        pixelsv2 = unaligned_load(line_size+1, pixels);
        pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
        pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
        pixelssum2 = vec_add((vector unsigned short)pixelsv1,
                             (vector unsigned short)pixelsv2);
        temp3 = vec_add(pixelssum1, pixelssum2);
        temp3 = vec_sra(temp3, vctwo);
        pixelssum1 = vec_add(pixelssum2, vctwo);
        pixelsavg = vec_packsu(temp3, (vector unsigned short) vczero);

        if (rightside) {
            blockv = vec_perm(blockv, pixelsavg, vcprm(0, 1, s0, s1));
        } else {
            blockv = vec_perm(blockv, pixelsavg, vcprm(s0, s1, 2, 3));
        }

        vec_st(blockv, 0, block);

        block += line_size;
        pixels += line_size;
    }
}

/* next one assumes that ((line_size % 8) == 0) */
static void put_no_rnd_pixels8_xy2_altivec(uint8_t *block, const uint8_t *pixels, ptrdiff_t line_size, int h)
{
    register int i;
    register vector unsigned char pixelsv1, pixelsv2, pixelsavg;
    register vector unsigned char blockv;
    register vector unsigned short pixelssum1, pixelssum2, temp3;
    register const vector unsigned char vczero = (const vector unsigned char)vec_splat_u8(0);
    register const vector unsigned short vcone = (const vector unsigned short)vec_splat_u16(1);
    register const vector unsigned short vctwo = (const vector unsigned short)vec_splat_u16(2);

    pixelsv1 = VEC_LD(0, pixels);
    pixelsv2 = VEC_LD(1, pixels);
    pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
    pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
    pixelssum1 = vec_add((vector unsigned short)pixelsv1,
                         (vector unsigned short)pixelsv2);
    pixelssum1 = vec_add(pixelssum1, vcone);

    for (i = 0; i < h ; i++) {
        int rightside = ((unsigned long)block & 0x0000000F);
        blockv = vec_ld(0, block);

        pixelsv1 = unaligned_load(line_size, pixels);
        pixelsv2 = unaligned_load(line_size+1, pixels);
        pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
        pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
        pixelssum2 = vec_add((vector unsigned short)pixelsv1,
                             (vector unsigned short)pixelsv2);
        temp3 = vec_add(pixelssum1, pixelssum2);
        temp3 = vec_sra(temp3, vctwo);
        pixelssum1 = vec_add(pixelssum2, vcone);
        pixelsavg = vec_packsu(temp3, (vector unsigned short) vczero);

        if (rightside) {
            blockv = vec_perm(blockv, pixelsavg, vcprm(0, 1, s0, s1));
        } else {
            blockv = vec_perm(blockv, pixelsavg, vcprm(s0, s1, 2, 3));
        }

        vec_st(blockv, 0, block);

        block += line_size;
        pixels += line_size;
    }
}

/* next one assumes that ((line_size % 16) == 0) */
static void put_pixels16_xy2_altivec(uint8_t * block, const uint8_t * pixels, ptrdiff_t line_size, int h)
{
    register int i;
    register vector unsigned char pixelsv1, pixelsv2, pixelsv3, pixelsv4;
    register vector unsigned char blockv;
    register vector unsigned short temp3, temp4,
        pixelssum1, pixelssum2, pixelssum3, pixelssum4;
    register const vector unsigned char vczero = (const vector unsigned char)vec_splat_u8(0);
    register const vector unsigned short vctwo = (const vector unsigned short)vec_splat_u16(2);

    pixelsv1 = VEC_LD(0, pixels);
    pixelsv2 = VEC_LD(1, pixels);
    pixelsv3 = VEC_MERGEL(vczero, pixelsv1);
    pixelsv4 = VEC_MERGEL(vczero, pixelsv2);
    pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
    pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
    pixelssum3 = vec_add((vector unsigned short)pixelsv3,
                         (vector unsigned short)pixelsv4);
    pixelssum3 = vec_add(pixelssum3, vctwo);
    pixelssum1 = vec_add((vector unsigned short)pixelsv1,
                         (vector unsigned short)pixelsv2);
    pixelssum1 = vec_add(pixelssum1, vctwo);

    for (i = 0; i < h ; i++) {
        blockv = vec_ld(0, block);

        pixelsv1 = unaligned_load(line_size, pixels);
        pixelsv2 = unaligned_load(line_size+1, pixels);

        pixelsv3 = VEC_MERGEL(vczero, pixelsv1);
        pixelsv4 = VEC_MERGEL(vczero, pixelsv2);
        pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
        pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
        pixelssum4 = vec_add((vector unsigned short)pixelsv3,
                             (vector unsigned short)pixelsv4);
        pixelssum2 = vec_add((vector unsigned short)pixelsv1,
                             (vector unsigned short)pixelsv2);
        temp4 = vec_add(pixelssum3, pixelssum4);
        temp4 = vec_sra(temp4, vctwo);
        temp3 = vec_add(pixelssum1, pixelssum2);
        temp3 = vec_sra(temp3, vctwo);

        pixelssum3 = vec_add(pixelssum4, vctwo);
        pixelssum1 = vec_add(pixelssum2, vctwo);

        blockv = vec_packsu(temp3, temp4);

        vec_st(blockv, 0, block);

        block += line_size;
        pixels += line_size;
    }
}

/* next one assumes that ((line_size % 16) == 0) */
static void put_no_rnd_pixels16_xy2_altivec(uint8_t * block, const uint8_t * pixels, ptrdiff_t line_size, int h)
{
    register int i;
    register vector unsigned char pixelsv1, pixelsv2, pixelsv3, pixelsv4;
    register vector unsigned char blockv;
    register vector unsigned short temp3, temp4,
        pixelssum1, pixelssum2, pixelssum3, pixelssum4;
    register const vector unsigned char vczero = (const vector unsigned char)vec_splat_u8(0);
    register const vector unsigned short vcone = (const vector unsigned short)vec_splat_u16(1);
    register const vector unsigned short vctwo = (const vector unsigned short)vec_splat_u16(2);

    pixelsv1 = VEC_LD(0, pixels);
    pixelsv2 = VEC_LD(1, pixels);
    pixelsv3 = VEC_MERGEL(vczero, pixelsv1);
    pixelsv4 = VEC_MERGEL(vczero, pixelsv2);
    pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
    pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
    pixelssum3 = vec_add((vector unsigned short)pixelsv3,
                         (vector unsigned short)pixelsv4);
    pixelssum3 = vec_add(pixelssum3, vcone);
    pixelssum1 = vec_add((vector unsigned short)pixelsv1,
                         (vector unsigned short)pixelsv2);
    pixelssum1 = vec_add(pixelssum1, vcone);

    for (i = 0; i < h ; i++) {
        pixelsv1 = unaligned_load(line_size, pixels);
        pixelsv2 = unaligned_load(line_size+1, pixels);

        pixelsv3 = VEC_MERGEL(vczero, pixelsv1);
        pixelsv4 = VEC_MERGEL(vczero, pixelsv2);
        pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
        pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
        pixelssum4 = vec_add((vector unsigned short)pixelsv3,
                             (vector unsigned short)pixelsv4);
        pixelssum2 = vec_add((vector unsigned short)pixelsv1,
                             (vector unsigned short)pixelsv2);
        temp4 = vec_add(pixelssum3, pixelssum4);
        temp4 = vec_sra(temp4, vctwo);
        temp3 = vec_add(pixelssum1, pixelssum2);
        temp3 = vec_sra(temp3, vctwo);

        pixelssum3 = vec_add(pixelssum4, vcone);
        pixelssum1 = vec_add(pixelssum2, vcone);

        blockv = vec_packsu(temp3, temp4);

        VEC_ST(blockv, 0, block);

        block += line_size;
        pixels += line_size;
    }
}

/* next one assumes that ((line_size % 8) == 0) */
static void avg_pixels8_xy2_altivec(uint8_t *block, const uint8_t *pixels, ptrdiff_t line_size, int h)
{
    register int i;
    register vector unsigned char pixelsv1, pixelsv2, pixelsavg;
    register vector unsigned char blockv, blocktemp;
    register vector unsigned short pixelssum1, pixelssum2, temp3;

    register const vector unsigned char vczero = (const vector unsigned char)
                                        vec_splat_u8(0);
    register const vector unsigned short vctwo = (const vector unsigned short)
                                        vec_splat_u16(2);

    pixelsv1 = VEC_LD(0, pixels);
    pixelsv2 = VEC_LD(1, pixels);
    pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
    pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
    pixelssum1 = vec_add((vector unsigned short)pixelsv1,
                         (vector unsigned short)pixelsv2);
    pixelssum1 = vec_add(pixelssum1, vctwo);

    for (i = 0; i < h ; i++) {
        int rightside = ((unsigned long)block & 0x0000000F);
        blockv = vec_ld(0, block);

        pixelsv1 = unaligned_load(line_size, pixels);
        pixelsv2 = unaligned_load(line_size+1, pixels);

        pixelsv1 = VEC_MERGEH(vczero, pixelsv1);
        pixelsv2 = VEC_MERGEH(vczero, pixelsv2);
        pixelssum2 = vec_add((vector unsigned short)pixelsv1,
                             (vector unsigned short)pixelsv2);
        temp3 = vec_add(pixelssum1, pixelssum2);
        temp3 = vec_sra(temp3, vctwo);
        pixelssum1 = vec_add(pixelssum2, vctwo);
        pixelsavg = vec_packsu(temp3, (vector unsigned short) vczero);

        if (rightside) {
            blocktemp = vec_perm(blockv, pixelsavg, vcprm(0, 1, s0, s1));
        } else {
            blocktemp = vec_perm(blockv, pixelsavg, vcprm(s0, s1, 2, 3));
        }

        blockv = vec_avg(blocktemp, blockv);
        vec_st(blockv, 0, block);

        block += line_size;
        pixels += line_size;
    }
}
#endif /* HAVE_ALTIVEC */

av_cold void ff_hpeldsp_init_ppc(HpelDSPContext *c, int flags)
{
#if HAVE_ALTIVEC
    if (!PPC_ALTIVEC(av_get_cpu_flags()))
        return;

    c->avg_pixels_tab[0][0]        = ff_avg_pixels16_altivec;
    c->avg_pixels_tab[1][0]        = avg_pixels8_altivec;
    c->avg_pixels_tab[1][3]        = avg_pixels8_xy2_altivec;

    c->put_pixels_tab[0][0]        = ff_put_pixels16_altivec;
    c->put_pixels_tab[1][3]        = put_pixels8_xy2_altivec;
    c->put_pixels_tab[0][3]        = put_pixels16_xy2_altivec;

    c->put_no_rnd_pixels_tab[0][0] = ff_put_pixels16_altivec;
    c->put_no_rnd_pixels_tab[1][3] = put_no_rnd_pixels8_xy2_altivec;
    c->put_no_rnd_pixels_tab[0][3] = put_no_rnd_pixels16_xy2_altivec;
#endif /* HAVE_ALTIVEC */
}
