/*
 * Copyright (c) 2010 Mans Rullgard <mans@mansr.com>
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

#include <stdint.h>

#include "libavutil/attributes.h"
#include "libavutil/arm/cpu.h"

#include "libavcodec/vp56dsp.h"

void ff_vp6_edge_filter_hor_neon(uint8_t *yuv, ptrdiff_t stride, int t);
void ff_vp6_edge_filter_ver_neon(uint8_t *yuv, ptrdiff_t stride, int t);

av_cold void ff_vp6dsp_init_arm(VP56DSPContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_neon(cpu_flags)) {
        s->edge_filter_hor = ff_vp6_edge_filter_hor_neon;
        s->edge_filter_ver = ff_vp6_edge_filter_ver_neon;
    }
}
