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

#ifndef AVFILTER_LIMITER_H
#define AVFILTER_LIMITER_H

#include <stddef.h>
#include <stdint.h>

typedef struct LimiterDSPContext {
    void (*limiter)(const uint8_t *src, uint8_t *dst,
                    ptrdiff_t slinesize, ptrdiff_t dlinesize,
                    int w, int h, int min, int max);
} LimiterDSPContext;

void ff_limiter_init_x86(LimiterDSPContext *dsp, int bpp);

#endif /* AVFILTER_LIMITER_H */
