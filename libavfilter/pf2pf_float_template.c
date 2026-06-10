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
#include "video.h"

#undef dtype
#undef DRCONV
#undef DWCONV
#if DST_FL == 0
#define dtype unsigned
#define DRCONV(x) (x)
#define DWCONV(x) (x)
#else
#define dtype float
#define DRCONV(x) av_int2float(x)
#define DWCONV(x) av_float2int(x)
#endif

#undef SRC_FL
#define SRC_FL 0

#include "pf2pf_template.c"

#undef SRC_FL
#define SRC_FL 1

#include "pf2pf_template.c"
