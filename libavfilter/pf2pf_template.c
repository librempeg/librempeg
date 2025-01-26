/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avfilter.h"
#include "video.h"

#if SRC_DEPTH > 8 || SRC_E == 0
#define fn3(a,b,c,d,e) a##_##b##_##c##_to_##d##_##e
#define fn2(a,b,c,d,e) fn3(a,b,c,d,e)
#define fn(a)          fn2(a, SRC_F, SRC_E, DST_F, DST_E)

static void fn(pf2pf_loop)(uint8_t **dstp,
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
    const unsigned dst_mask = (1U << dst_depth) - 1;
    const unsigned src_mask = (1U << src_depth) - 1;
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

static int fn(pf2pf)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    PF2PFContext *s = ctx->priv;
    const int nb_components = s->dst_desc->nb_components;
    const int *linesize = s->linesize;
    ThreadData *td = arg;
    AVFrame *restrict out = td->out;
    AVFrame *restrict in = td->in;
    const int w = in->width;
    const int h = in->height;
    const int start = (h * jobnr) / nb_jobs;
    const int end = (h * (jobnr+1)) / nb_jobs;

    for (int comp = 0; comp < nb_components; comp++) {
        if (out->data[comp]) {
            const int dst_sh = (comp > 0) ? s->dst_desc->log2_chroma_h : 0;
            const int cstart = start >> dst_sh;
            const int cend = end >> dst_sh;
            uint8_t *dst_data = out->data[comp] + cstart * out->linesize[comp];

            for (int y = cstart; y < cend; y++) {
                memset(dst_data, 0, linesize[comp]);
                dst_data += out->linesize[comp];
            }
        }
    }

    for (int comp = 0; comp < nb_components; comp++) {
        const int dst_plane = s->dst_desc->comp[comp].plane;
        const int dst_step  = s->dst_desc->comp[comp].step;
        const int dst_shift = s->dst_desc->comp[comp].shift;
        const int dst_offset= s->dst_desc->comp[comp].offset;
        const int dst_depth = s->dst_desc->comp[comp].depth;
        const int src_plane = s->src_desc->comp[comp].plane;
        const int src_step  = s->src_desc->comp[comp].step;
        const int src_shift = s->src_desc->comp[comp].shift;
        const int src_offset= s->src_desc->comp[comp].offset;
        const int src_depth = s->src_desc->comp[comp].depth;
        const int dst_sw = (comp > 0) ? s->dst_desc->log2_chroma_w : 0;
        const int dst_sh = (comp > 0) ? s->dst_desc->log2_chroma_h : 0;
        uint8_t *dst_data[4] = {NULL}, *src_data[4] = {NULL};
        const int cstart = start >> dst_sh;
        const int cend = end >> dst_sh;
        const int cw = w >> dst_sw;

        for (int i = 0; i < 4; i++) {
            if (out->data[i])
                dst_data[i] = out->data[i] + cstart * out->linesize[i];
            if (in->data[i])
                src_data[i] = in->data[i] + cstart * in->linesize[i];
        }

        if (s->fun[comp]) {
            s->fun[comp]((uint8_t **)dst_data, (const uint8_t **)src_data, out->linesize, in->linesize,
                         cw, cend - cstart,
                         dst_plane, dst_shift, dst_depth,
                         src_plane, src_shift, src_depth);
        } else {
            fn(pf2pf_loop)((uint8_t **)dst_data, (const uint8_t **)src_data, out->linesize, in->linesize,
                           cw, cend - cstart,
                           dst_plane, dst_step, dst_shift, dst_offset, dst_depth,
                           src_plane, src_step, src_shift, src_offset, src_depth);
        }
    }

    return 0;
}
#endif

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
