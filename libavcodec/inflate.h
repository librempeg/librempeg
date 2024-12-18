/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_INFLATE_H
#define AVCODEC_INFLATE_H

#include <stdint.h>

#define BITSTREAM_READER_LE
#include "get_bits.h"

typedef struct InflateTree {
    uint16_t counts[16];
    uint16_t symbols[288];
    int max_sym;
} InflateTree;

typedef struct InflateContext {
    GetBitContext gb;

    int x, y;
    int height, width;
    int fixed_cb_initialized;
    uint8_t *dst;
    ptrdiff_t linesize;

    InflateTree fixed_ltree, fixed_dtree;
    InflateTree dynamic_ltree, dynamic_dtree;
} InflateContext;

int ff_inflate(InflateContext *s, const uint8_t *src, int src_len,
               uint8_t *dst, int height, int width, ptrdiff_t linesize);

#endif /* AVCODEC_INFLATE_H */
