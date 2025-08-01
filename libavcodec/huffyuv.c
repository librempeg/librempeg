/*
 * huffyuv codec for libavcodec
 *
 * Copyright (c) 2002-2014 Michael Niedermayer <michaelni@gmx.at>
 *
 * see https://multimedia.cx/huffyuv.txt for a description of
 * the algorithm used
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

/**
 * @file
 * huffyuv codec for libavcodec.
 */

#include <stddef.h>
#include <stdint.h>

#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/macros.h"

#include "huffyuv.h"

int ff_huffyuv_generate_bits_table(uint32_t *dst, const uint8_t *len_table, int n)
{
    int lens[33] = { 0 };
    uint32_t codes[33];

    for (int i = 0; i < n; i++)
        lens[len_table[i]]++;

    codes[32] = 0;
    for (int i = FF_ARRAY_ELEMS(lens) - 1; i > 0; i--) {
        if ((lens[i] + codes[i]) & 1) {
            av_log(NULL, AV_LOG_ERROR, "Error generating huffman table\n");
            return AVERROR_INVALIDDATA;
        }
        codes[i - 1] = (lens[i] + codes[i]) >> 1;
    }
    for (int i = 0; i < n; i++) {
        if (len_table[i])
            dst[i] = codes[len_table[i]]++;
    }
    return 0;
}
