/*
 * check XMM registers for clobbers on Win64
 * Copyright (c) 2012 Ronald S. Bultje <rsbultje@gmail.com>
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

#include "libavutil/x86/w64xmmtest.h"
#include "libswscale/swscale.h"

wrap(sws_scale(SwsContext *c, const uint8_t *const srcSlice[],
               const int srcStride[], int srcSliceY, int srcSliceH,
               uint8_t *const dst[], const int dstStride[]))
{
    testxmmclobbers(sws_scale, c, srcSlice, srcStride, srcSliceY,
                    srcSliceH, dst, dstStride);
}
