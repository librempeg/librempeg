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

#ifndef AVCODEC_DEFLATE_H
#define AVCODEC_DEFLATE_H

#include <stdint.h>

#define BITSTREAM_WRITER_LE
#include "put_bits.h"

typedef struct DeflateContext {
    PutBitContext pb;

    int x, y;
    int nb_literals;
    int height, width;
    int fixed_cb_initialized;
    const uint8_t *src;
    ptrdiff_t linesize;

    uint16_t lit[65536];
    uint16_t dst[65536];
    uint8_t  run[65536];
    uint8_t  dsc[65536];
    uint8_t  val[65536];

    uint16_t lit_count[288];
    uint16_t dst_count[32];
    uint16_t huf_count[19];

    uint8_t  lit_code_sizes[2][288];
    uint16_t lit_codes[2][288];

    uint8_t  dst_code_sizes[2][32];
    uint16_t dst_codes[2][32];

    uint8_t  huf_code_sizes[2][19];
    uint16_t huf_codes[2][19];
} DeflateContext;

int ff_deflate(DeflateContext *s, uint8_t *dst, int dst_len,
               const uint8_t *src, int height, int width, ptrdiff_t linesize);

size_t ff_deflate_bound(const size_t in_size);

#endif /* AVCODEC_DEFLATE_H */
