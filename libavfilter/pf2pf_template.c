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

#if SRC_DEPTH > 8 || SRC_E == 0
#define fn3(a,b,c,d,e) a##_##b##_##c##_to_##d##_##e
#define fn2(a,b,c,d,e) fn3(a,b,c,d,e)
#define fn(a)          fn2(a, SRC_F, SRC_E, DST_F, DST_E)

static void fn(pf2pf_generic_loop)(uint8_t **dstp,
                                   const uint8_t **srcp,
                                   const int *dst_linesizep,
                                   const int *src_linesizep,
                                   const int w, const int h,
                                   const int dst_plane,
                                   const int dst_step, const int dst_shift,
                                   const int dst_offset, const int dst_depth,
                                   const int src_plane,
                                   const int src_step, const int src_shift,
                                   const int src_offset, const int src_depth)
{
    const ptrdiff_t dst_linesize = dst_linesizep[dst_plane];
    const ptrdiff_t src_linesize = src_linesizep[src_plane];
    const unsigned dst_mask = (1ULL << dst_depth) - 1;
    const unsigned src_mask = (1ULL << src_depth) - 1;
    const unsigned dst_rshift = FFMAX(src_depth - dst_depth, 0);
    const unsigned dst_lshift = dst_shift + FFMAX(dst_depth-src_depth, 0);
    const uint8_t *src = srcp[src_plane];
    uint8_t *dst = dstp[dst_plane];

    for (int y = 0; y < h; y++) {
        for (int x = 0, i = 0, j = 0; x < w; x++) {
            unsigned odst = DRP(dst + i + dst_offset);
            DWP(dst + i + dst_offset, odst | (((((SRP(src + j + src_offset) >> src_shift) & src_mask) >> dst_rshift) & dst_mask) << dst_lshift));
            i += dst_step;
            j += src_step;
        }

        dst += dst_linesize;
        src += src_linesize;
    }
}
#endif

#if SRC_DEPTH < 32 && DST_DEPTH < 32

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 0
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 1
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 2
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 3
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 4
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 5
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 6
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 7
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 8
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 9
#include "pf2pf_step_offset_template.c"

#undef SRC_OFFSET_NAME
#define SRC_OFFSET_NAME 10
#include "pf2pf_step_offset_template.c"

#endif
