/*
 * Copyright (c) 2015 Parag Salasakar (Parag.Salasakar@imgtec.com)
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

#ifndef AVCODEC_MIPS_ME_CMP_MIPS_H
#define AVCODEC_MIPS_ME_CMP_MIPS_H

#include "../mpegvideoenc.h"
#include "libavcodec/bit_depth_template.c"

int ff_hadamard8_diff8x8_msa(MPVEncContext *s, const uint8_t *dst, const uint8_t *src,
                             ptrdiff_t stride, int h);
int ff_hadamard8_intra8x8_msa(MPVEncContext *s, const uint8_t *dst, const uint8_t *src,
                              ptrdiff_t stride, int h);
int ff_hadamard8_diff16_msa(MPVEncContext *s, const uint8_t *dst, const uint8_t *src,
                            ptrdiff_t stride, int h);
int ff_hadamard8_intra16_msa(MPVEncContext *s, const uint8_t *dst, const uint8_t *src,
                             ptrdiff_t stride, int h);
int ff_pix_abs16_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                     ptrdiff_t stride, int h);
int ff_pix_abs16_x2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                        ptrdiff_t stride, int h);
int ff_pix_abs16_y2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                        ptrdiff_t stride, int h);
int ff_pix_abs16_xy2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                         ptrdiff_t stride, int h);
int ff_pix_abs8_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                    ptrdiff_t stride, int h);
int ff_pix_abs8_x2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                       ptrdiff_t stride, int h);
int ff_pix_abs8_y2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                       ptrdiff_t stride, int h);
int ff_pix_abs8_xy2_msa(MPVEncContext *v, const uint8_t *pix1, const uint8_t *pix2,
                        ptrdiff_t stride, int h);
int ff_sse16_msa(MPVEncContext *v, const uint8_t *pu8Src, const uint8_t *pu8Ref,
                 ptrdiff_t stride, int i32Height);
int ff_sse8_msa(MPVEncContext *v, const uint8_t *pu8Src, const uint8_t *pu8Ref,
                ptrdiff_t stride, int i32Height);
int ff_sse4_msa(MPVEncContext *v, const uint8_t *pu8Src, const uint8_t *pu8Ref,
                ptrdiff_t stride, int i32Height);
void ff_add_pixels8_msa(const uint8_t *restrict pixels, int16_t *block,
                        ptrdiff_t stride);

#endif  // #ifndef AVCODEC_MIPS_ME_CMP_MIPS_H
