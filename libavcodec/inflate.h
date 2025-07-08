/*
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

#ifndef AVCODEC_INFLATE_H
#define AVCODEC_INFLATE_H

#include <stdint.h>

#define CACHED_BITSTREAM_READER !ARCH_X86_32
#define BITSTREAM_READER_LE
#include "get_bits.h"

typedef struct InflateTree {
    VLC vlc;
    int max_sym;
} InflateTree;

typedef struct InflateContext {
    GetBitContext gb;

    uint8_t history[32768];
    unsigned history_pos;

    int x, y;
    int height, width;
    int fixed_cb_initialized;
    uint8_t *dst;
    ptrdiff_t linesize;

    void *priv_data;
    uint8_t *tmp;
    void (*row_fun)(void *priv_data, uint8_t *row, ptrdiff_t linesize,
                    uint8_t *tmp, int *y, int *w, const int height);

    InflateTree fixed_ltree, fixed_dtree;
    InflateTree dynamic_ltree, dynamic_dtree;
} InflateContext;

int ff_inflate(InflateContext *s, const uint8_t *src, int src_len,
               uint8_t *dst, int height, int width, ptrdiff_t linesize);

int ff_inflatex(InflateContext *s, const uint8_t *src, int src_len,
                uint8_t *dst, int height, int width, ptrdiff_t linesize,
                void *priv_data, uint8_t *tmp, void (*row_fun)(void *priv_data, uint8_t *row,
                                                               ptrdiff_t linesize,
                                                               uint8_t *tmp,
                                                               int *y, int *w, const int h));

#endif /* AVCODEC_INFLATE_H */
