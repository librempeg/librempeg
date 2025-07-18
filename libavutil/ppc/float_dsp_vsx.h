/*
 * Copyright (c) 2015 Luca Barbato <lu_zero@gentoo.org>
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

#ifndef AVUTIL_PPC_FLOAT_DSP_VSX_H
#define AVUTIL_PPC_FLOAT_DSP_VSX_H

void ff_vector_fmul_vsx(float *dst, const float *src0,
                        const float *src1, int len);

void ff_vector_fmul_window_vsx(float *dst, const float *src0,
                               const float *src1, const float *win,
                               int len);

void ff_vector_fmul_add_vsx(float *dst, const float *src0,
                            const float *src1, const float *src2,
                            int len);

void ff_vector_fmul_reverse_vsx(float *dst, const float *src0,
                                const float *src1, int len);

#endif /* AVUTIL_PPC_FLOAT_DSP_VSX_H */
