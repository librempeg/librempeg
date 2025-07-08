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

#include "avfilter.h"
#include "audio.h"

#undef dtype
#undef DST_F
#if DST_DEPTH == 8
#define dtype uint8_t
#define DST_F u8
#elif DST_DEPTH == 16
#define dtype int16_t
#define DST_F s16
#elif DST_DEPTH == 31
#define dtype int32_t
#define DST_F s32
#elif DST_DEPTH == 32
#define dtype float
#define DST_F flt
#elif DST_DEPTH == 63
#define dtype int64_t
#define DST_F s64
#else
#define dtype double
#define DST_F dbl
#endif

#undef SRC_DEPTH
#define SRC_DEPTH 8
#include "asf2sf_template2.c"

#undef SRC_DEPTH
#define SRC_DEPTH 16
#include "asf2sf_template2.c"

#undef SRC_DEPTH
#define SRC_DEPTH 31
#include "asf2sf_template2.c"

#undef SRC_DEPTH
#define SRC_DEPTH 32
#include "asf2sf_template2.c"

#undef SRC_DEPTH
#define SRC_DEPTH 63
#include "asf2sf_template2.c"

#undef SRC_DEPTH
#define SRC_DEPTH 64
#include "asf2sf_template2.c"
