/*
 * ARM NEON optimised H.264 chroma functions
 * Copyright (c) 2008 Mans Rullgard <mans@mansr.com>
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
#include "libavutil/cpu.h"
#include "libavutil/aarch64/cpu.h"
#include "libavcodec/h264chroma.h"

#include "config.h"

void ff_put_h264_chroma_mc8_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);
void ff_put_h264_chroma_mc4_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);
void ff_put_h264_chroma_mc2_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);

void ff_avg_h264_chroma_mc8_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);
void ff_avg_h264_chroma_mc4_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);
void ff_avg_h264_chroma_mc2_neon(uint8_t *dst, const uint8_t *src, ptrdiff_t stride,
                                 int h, int x, int y);

av_cold void ff_h264chroma_init_aarch64(H264ChromaContext *c, int bit_depth)
{
    const int high_bit_depth = bit_depth > 8;
    int cpu_flags = av_get_cpu_flags();

    if (have_neon(cpu_flags) && !high_bit_depth) {
        c->put_h264_chroma_pixels_tab[0] = ff_put_h264_chroma_mc8_neon;
        c->put_h264_chroma_pixels_tab[1] = ff_put_h264_chroma_mc4_neon;
        c->put_h264_chroma_pixels_tab[2] = ff_put_h264_chroma_mc2_neon;

        c->avg_h264_chroma_pixels_tab[0] = ff_avg_h264_chroma_mc8_neon;
        c->avg_h264_chroma_pixels_tab[1] = ff_avg_h264_chroma_mc4_neon;
        c->avg_h264_chroma_pixels_tab[2] = ff_avg_h264_chroma_mc2_neon;
    }
}
