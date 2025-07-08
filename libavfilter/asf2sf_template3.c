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

#undef dpidx
#undef DST_P
#undef DST_p
#undef fn_dst_ptr
#define dpidx m
#define DST_P packed
#define DST_p 0
#define fn_dst_ptr(x, ch, n) (((dtype *restrict)(x)->data[0]) + ((n)*nb_channels) + (ch))
#if !((SRC_DEPTH == DST_DEPTH) && (SRC_p == DST_p))
#include "asf2sf_template4.c"
#endif

#undef dpidx
#undef DST_P
#undef DST_p
#undef fn_dst_ptr
#define dpidx n
#define DST_P planar
#define DST_p 1
#define fn_dst_ptr(x, ch, n) ((dtype *restrict)(x)->extended_data[(ch)] + (n))
#if !((SRC_DEPTH == DST_DEPTH) && (SRC_p == DST_p))
#include "asf2sf_template4.c"
#endif
