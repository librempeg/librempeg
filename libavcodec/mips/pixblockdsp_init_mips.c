/*
 * Copyright (c) 2015 Shivraj Patil (Shivraj.Patil@imgtec.com)
 *                    Zhou Xiaoyong <zhouxiaoyong@loongson.cn>
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

#include "libavutil/mips/cpu.h"
#include "libavcodec/pixblockdsp.h"
#include "pixblockdsp_mips.h"

void ff_pixblockdsp_init_mips(PixblockDSPContext *c,
                              unsigned high_bit_depth)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_mmi(cpu_flags)) {
        c->diff_pixels = ff_diff_pixels_mmi;

        if (!high_bit_depth)
            c->get_pixels = ff_get_pixels_8_mmi;
    }

    if (have_msa(cpu_flags)) {
        c->diff_pixels = ff_diff_pixels_msa;

        c->get_pixels = high_bit_depth ? ff_get_pixels_16_msa : ff_get_pixels_8_msa;
    }
}
