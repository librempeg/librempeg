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

#undef DST_F
#undef DWP
#undef DRP
#if DST_DEPTH == 8
#define DST_F 0
#define DWP(x, y) ((x)[0] = (y))
#define DRP(x) ((x)[0])
#elif DST_DEPTH == 16
#define DST_F 1
#if DST_E == 0
#define DWP(x, y) AV_WL16((x), (y))
#define DRP(x) AV_RL16(x)
#elif DST_E == 1
#define DWP(x, y) AV_WB16((x), (y))
#define DRP(x) AV_RB16(x)
#endif
#elif DST_DEPTH == 32
#define DST_F 3
#if DST_E == 0
#define DWP(x, y) AV_WL32((x), (y))
#define DRP(x) AV_RL32(x)
#elif DST_E == 1
#define DWP(x, y) AV_WB32((x), (y))
#define DRP(x) AV_RB32(x)
#endif
#endif

#undef SRC_DEPTH
#define SRC_DEPTH 8
#include "pf2pf_src_depth_template.c"

#undef SRC_DEPTH
#define SRC_DEPTH 16
#include "pf2pf_src_depth_template.c"

#undef SRC_DEPTH
#define SRC_DEPTH 32
#include "pf2pf_src_depth_template.c"
