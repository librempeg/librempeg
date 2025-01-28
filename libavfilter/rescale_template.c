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

#undef pixel_type
#if DEPTH == 8
#define pixel_type uint8_t
#elif DEPTH == 16
#define pixel_type uint16_t
#elif DEPTH == 32
#define pixel_type uint32_t
#else
#define pixel_type float
#endif

#define fn3(a,b) a##_##b
#define fn2(a,b) fn3(a,b)
#define fn(a)    fn2(a, DEPTH)

static int fn(rescale_slice)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ReScaleContext *s = ctx->priv;
    const int nb_components = s->dst_desc->nb_components;
    ThreadData *td = arg;
    AVFrame *restrict out = td->out;
    AVFrame *restrict in = td->in;
    const int dst_w = out->width;
    const int dst_h = out->height;
    const int src_w = in->width;
    const int src_h = in->height;
    const int start = (dst_h * jobnr) / nb_jobs;
    const int end = (dst_h * (jobnr+1)) / nb_jobs;

    for (int comp = 0; comp < nb_components; comp++) {
        const int dst_sw = (comp > 0) ? s->dst_desc->log2_chroma_w : 0;
        const int dst_sh = (comp > 0) ? s->dst_desc->log2_chroma_h : 0;
        const int src_sw = (comp > 0) ? s->src_desc->log2_chroma_w : 0;
        const int src_sh = (comp > 0) ? s->src_desc->log2_chroma_h : 0;
        const int cstart = start >> dst_sh;
        const int dst_ch = AV_CEIL_RSHIFT(dst_h, dst_sh);
        const int src_ch = AV_CEIL_RSHIFT(src_h, src_sh);
        const int cend = (end == dst_h) ? dst_ch : (end >> dst_sh);
        const int dst_cw = AV_CEIL_RSHIFT(dst_w, dst_sw);
        const int src_cw = AV_CEIL_RSHIFT(src_w, src_sw);
        const int inc = (((int64_t)src_cw) << 16) / dst_cw;
        int psy = -1;

        for (int y = cstart; y < cend; y++) {
            pixel_type *dst_data = (pixel_type *)(out->data[comp] + y * out->linesize[comp]);
            const int sy = FFMIN(lrintf(y * (src_ch  / ((float)dst_ch))), src_ch-1);
            const pixel_type *src_data = (const pixel_type *)(in->data[comp] + sy * in->linesize[comp]);
            int isx = 0;

            if (sy == psy) {
                memcpy(dst_data, dst_data - out->linesize[comp], dst_cw * (DEPTH/8));
            } else {
                for (int x = 0; x < dst_cw; x++) {
                    const int sx = FFMIN(isx >> 16, src_cw-1);

                    dst_data[x] = src_data[sx];
                    isx += inc;
                }
            }

            psy = sy;
        }
    }

    return 0;
}
