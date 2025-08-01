/*
 * Copyright (c) 2019 Paul B Mahol
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
#include "libavutil/x86/asm.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/maskedclamp.h"

void ff_maskedclamp8_sse2(const uint8_t *bsrc, uint8_t *dst,
                          const uint8_t *darksrc, const uint8_t *brightsrc,
                          int w, const void *undershoot, const void *overshoot);

void ff_maskedclamp16_sse4(const uint8_t *bsrc, uint8_t *dst,
                           const uint8_t *darksrc, const uint8_t *brightsrc,
                           int w, const void *undershoot, const void *overshoot);

av_cold void ff_maskedclamp_init_x86(MaskedClampDSPContext *dsp, int depth)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE2(cpu_flags) && depth <= 8) {
        dsp->maskedclamp = ff_maskedclamp8_sse2;
    }

    if (EXTERNAL_SSE4(cpu_flags) && depth > 8 && depth <= 16) {
        dsp->maskedclamp = ff_maskedclamp16_sse4;
    }
}
