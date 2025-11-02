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

#define SHIFT 11

#undef SH
#undef AND
#undef FDIV
#undef FACTOR
#undef IFACTOR
#undef im_type
#undef inc_type
#undef pixel_type
#if DEPTH == 8
#define FACTOR 2048
#define IFACTOR (FACTOR-1)
#define im_type int
#define inc_type int64_t
#define pixel_type uint8_t
#define SH(x) ((x) >> SHIFT)
#define AND(x) ((x) & IFACTOR)
#define FDIV(x, y) ((((inc_type)(x)) << SHIFT) / (y))
#elif DEPTH == 16
#define FACTOR 2048
#define IFACTOR (FACTOR-1)
#define im_type int64_t
#define inc_type int64_t
#define pixel_type uint16_t
#define SH(x) ((x) >> SHIFT)
#define AND(x) ((x) & IFACTOR)
#define FDIV(x, y) ((((inc_type)(x)) << SHIFT) / (y))
#elif DEPTH == 32
#define FACTOR 2048
#define IFACTOR (FACTOR-1)
#define im_type int64_t
#define inc_type int64_t
#define pixel_type uint32_t
#define SH(x) ((x) >> SHIFT)
#define AND(x) ((x) & IFACTOR)
#define FDIV(x, y) ((((inc_type)(x)) << SHIFT) / (y))
#else
#define FACTOR 0.f
#define IFACTOR 1.f
#define im_type float
#define inc_type float
#define pixel_type float
#define SH(x) (x)
#define AND(x) (fmodf(x, 1.f))
#define FDIV(x, y) ((x) / ((float)y))
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
        const ptrdiff_t out_linesize = out->linesize[comp];
        const ptrdiff_t in_linesize = in->linesize[comp];
        const int dst_sw = (comp > 0 && comp < 3) ? s->dst_desc->log2_chroma_w : 0;
        const int dst_sh = (comp > 0 && comp < 3) ? s->dst_desc->log2_chroma_h : 0;
        const int src_sw = (comp > 0 && comp < 3) ? s->src_desc->log2_chroma_w : 0;
        const int src_sh = (comp > 0 && comp < 3) ? s->src_desc->log2_chroma_h : 0;
        const int cstart = start >> dst_sh;
        const int dst_ch = AV_CEIL_RSHIFT(dst_h, dst_sh);
        const int src_ch = AV_CEIL_RSHIFT(src_h, src_sh);
        const int cend = (end == dst_h) ? dst_ch : (end >> dst_sh);
        const int dst_cw = AV_CEIL_RSHIFT(dst_w, dst_sw);
        const int src_cw = AV_CEIL_RSHIFT(src_w, src_sw);
        const int inc = (((int64_t)src_cw) << 16) / dst_cw;
        int psy = -1;

        for (int y = cstart; y < cend; y++) {
            pixel_type *dst_data = (pixel_type *)(out->data[comp] + y * out_linesize);
            const int sy = FFMIN(lrintf(y * (src_ch  / ((float)dst_ch))), src_ch-1);
            const pixel_type *src_data = (const pixel_type *)(in->data[comp] + sy * in_linesize);
            int isx = 0;

            if (sy == psy) {
                memcpy(dst_data, dst_data - out_linesize, dst_cw * (DEPTH/8));
            } else {
                for (int x = 0; x < dst_cw; x++) {
                    const int sx = isx >> 16;

                    dst_data[x] = src_data[sx];
                    isx += inc;
                }
            }

            psy = sy;
        }
    }

    return 0;
}

static int fn(rescale_slice_linear)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
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
        const ptrdiff_t out_linesize = out->linesize[comp];
        const ptrdiff_t in_linesize = in->linesize[comp];
        const int dst_sw = (comp > 0 && comp < 3) ? s->dst_desc->log2_chroma_w : 0;
        const int dst_sh = (comp > 0 && comp < 3) ? s->dst_desc->log2_chroma_h : 0;
        const int src_sw = (comp > 0 && comp < 3) ? s->src_desc->log2_chroma_w : 0;
        const int src_sh = (comp > 0 && comp < 3) ? s->src_desc->log2_chroma_h : 0;
        const int cstart = start >> dst_sh;
        const int dst_ch = AV_CEIL_RSHIFT(dst_h, dst_sh);
        const int src_ch = AV_CEIL_RSHIFT(src_h, src_sh);
        const int cend = (end == dst_h) ? dst_ch : (end >> dst_sh);
        const int dst_cw = AV_CEIL_RSHIFT(dst_w, dst_sw);
        const int src_cw = AV_CEIL_RSHIFT(src_w, src_sw);
        const inc_type w_inc = FDIV(src_cw, dst_cw);
        const inc_type h_inc = FDIV(src_ch, dst_ch);
        inc_type isy = cstart * h_inc;

        for (int y = cstart; y < cend; y++) {
            pixel_type *dst_data = (pixel_type *)(out->data[comp] + y * out_linesize);
            const int sy = SH(isy);
            const pixel_type *src_data = (const pixel_type *)(in->data[comp] + sy * in_linesize);
            const pixel_type *src_data2 = (const pixel_type *)(in->data[comp] + (sy+(sy+1<src_ch)) * in_linesize);
            const im_type fracy = AND(isy);
            const im_type ffracy = IFACTOR-fracy;
            inc_type isx = 0;

            if (dst_cw == src_cw && dst_ch == src_ch) {
                memcpy(dst_data, src_data, dst_cw * sizeof(*dst_data));
                isy += h_inc;
                continue;
            }

            for (int x = 0; x < dst_cw; x++) {
                const int sx = SH(isx);
                const int o = (sx+1)<src_cw;
                const im_type fracx = AND(isx);
                const im_type ffracx = IFACTOR-fracx;

                dst_data[x] = (src_data[sx+0] * ffracx * ffracy +
                               src_data[sx+o] * fracx * ffracy +
                               src_data2[sx+0] * ffracx * fracy +
                               src_data2[sx+o] * fracx * fracy + (FACTOR * FACTOR / 2))
#if DEPTH == 33
                    ;
#else
                    >> (2 * SHIFT);
#endif
                isx += w_inc;
            }

            isy += h_inc;
        }
    }

    return 0;
}
