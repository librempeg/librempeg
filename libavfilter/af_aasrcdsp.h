/*
 * Copyright (c) 2025 Paul B Mahol
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

#ifndef AVFILTER_AASRCDSP_H
#define AVFILTER_AASRCDSP_H

#include "libavutil/common.h"

#include "audio.h"
#include "avfilter.h"
#include "formats.h"

typedef struct complex_double {
    double re, im;
} complex_double;

typedef struct complex_float {
    float re, im;
} complex_float;

typedef struct AudioASRCDSPContext {
    void (*vector_fmul_complex)(complex_float *x,
                                const complex_float *a,
                                const complex_float *b, const int N);

    void (*vector_dmul_complex)(complex_double *x,
                                const complex_double *a,
                                const complex_double *b, const int N);

    float (*vector_fmul_real)(const complex_float *cur,
                              const complex_float *h,
                              const int N);

    double (*vector_dmul_real)(const complex_double *cur,
                               const complex_double *h,
                               const int N);

    void (*vector_fmul_complex_add)(const float src,
                                    const complex_float *fixed,
                                    const complex_float *in,
                                    complex_float *out,
                                    const int N);

    void (*vector_dmul_complex_add)(const double src,
                                    const complex_double *fixed,
                                    const complex_double *in,
                                    complex_double *out,
                                    const int N);
} AudioASRCDSPContext;

void ff_aasrc_init(AudioASRCDSPContext *s);
void ff_aasrc_init_x86(AudioASRCDSPContext *s);

#endif /* AVFILTER_AASRCDSP_H */
