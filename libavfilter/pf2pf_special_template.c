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

#if (SRC_DEPTH > 8 || SRC_E == 0) && (DST_DEPTH > 8 || DST_E == 0)

#define SRC_STEP        step_offset_tab[SRC_OFFSET_NAME][0]
#define SRC_OFFSET      step_offset_tab[SRC_OFFSET_NAME][1]
#define DST_STEP        step_offset_tab[DST_OFFSET_NAME][0]
#define DST_OFFSET      step_offset_tab[DST_OFFSET_NAME][1]

#define fns3(a,b,c,d,e,f,g) a##_##b##_##c##_##d##_to_##e##_##f##_##g
#define fns2(a,b,c,d,e,f,g) fns3(a,b,c,d,e,f,g)
#define fns(a)              fns2(a, SRC_F, SRC_E, SRC_OFFSET_NAME, DST_F, DST_E, DST_OFFSET_NAME)

static void fns(pf2pf_special_loop)(uint8_t **dstp,
                                    const uint8_t **srcp,
                                    const int *dst_linesizep,
                                    const int *src_linesizep,
                                    const int w, const int h,
                                    const int dst_plane,
                                    const int dst_shift,
                                    const int dst_depth,
                                    const int src_plane,
                                    const int src_shift,
                                    const int src_depth)
{
    const ptrdiff_t dst_linesize = dst_linesizep[dst_plane];
    const ptrdiff_t src_linesize = src_linesizep[src_plane];
    const unsigned dst_mask = (1U << dst_depth) - 1;
    const unsigned src_mask = (1U << src_depth) - 1;
    const unsigned dst_rshift = FFMAX(src_depth - dst_depth, 0);
    const unsigned dst_lshift = dst_shift + FFMAX(dst_depth-src_depth, 0);
    const uint8_t *src = srcp[src_plane];
    uint8_t *dst = dstp[dst_plane];

    for (int y = 0; y < h; y++) {
        for (int x = 0, i = 0, j = 0; x < w; x++) {
            unsigned odst = DRP(dst + i + DST_OFFSET);
            DWP(dst + i + DST_OFFSET, odst | (((((SRP(src + j + SRC_OFFSET) >> src_shift) & src_mask) >> dst_rshift) & dst_mask) << dst_lshift));
            i += DST_STEP;
            j += SRC_STEP;
        }

        dst += dst_linesize;
        src += src_linesize;
    }
}
#endif
