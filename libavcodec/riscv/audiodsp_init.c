/*
 * Copyright © 2022 Rémi Denis-Courmont.
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
#include "libavcodec/audiodsp.h"

int32_t ff_scalarproduct_int16_rvv(const int16_t *v1, const int16_t *v2, int len);
void ff_vector_clip_int32_rvv(int32_t *dst, const int32_t *src, int32_t min,
                              int32_t max, unsigned int len);
void ff_vector_clipf_rvv(float *dst, const float *src, int len, float min, float max);

av_cold void ff_audiodsp_init_riscv(AudioDSPContext *c)
{
#if HAVE_RVV
    int flags = av_get_cpu_flags();

    if ((flags & AV_CPU_FLAG_RVV_I32) && (flags & AV_CPU_FLAG_RVB)) {
        c->scalarproduct_int16 = ff_scalarproduct_int16_rvv;
        c->vector_clip_int32 = ff_vector_clip_int32_rvv;

        if (flags & AV_CPU_FLAG_RVV_F32)
            c->vector_clipf = ff_vector_clipf_rvv;
    }
#endif
}
