/*
 * Copyright (c) 2021 Paul B Mahol
 *
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

#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "drawutils.h"
#include "filters.h"
#include "video.h"

#include <float.h>

typedef struct ShearContext {
    const AVClass *class;

    float shx, shy;
    int interp;

    uint8_t fillcolor[4];   ///< color expressed either in YUVA or RGBA colorspace for the padding area
    int fillcolor_enable;
    int nb_planes;
    int depth;
    FFDrawContext draw;
    FFDrawColor color;

    int hsub, vsub;
    int planewidth[4];
    int planeheight[4];

    int (*filter_slice[2])(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} ShearContext;

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define OFFSET(x) offsetof(ShearContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption shear_options[] = {
    { "shx",       "set x shear factor",        OFFSET(shx),           AV_OPT_TYPE_FLOAT,  {.dbl=0.},     -2, 2, .flags=FLAGS },
    { "shy",       "set y shear factor",        OFFSET(shy),           AV_OPT_TYPE_FLOAT,  {.dbl=0.},     -2, 2, .flags=FLAGS },
    { "fillcolor", "set background fill color", OFFSET(fillcolor),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, .flags=FLAGS },
    { "c",         "set background fill color", OFFSET(fillcolor),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, .flags=FLAGS },
    { "dofill",    "enable background fill",    OFFSET(fillcolor_enable),AV_OPT_TYPE_BOOL, {.i64=1},       0, 1, .flags=FLAGS },
    { "interp",    "set interpolation",         OFFSET(interp),        AV_OPT_TYPE_INT,    {.i64=1},       0, 1, .flags=FLAGS, .unit = "interp" },
    {  "nearest",  "nearest neighbour",         0,                     AV_OPT_TYPE_CONST,  {.i64=0},       0, 0, .flags=FLAGS, .unit = "interp" },
    {  "bilinear", "bilinear",                  0,                     AV_OPT_TYPE_CONST,  {.i64=1},       0, 0, .flags=FLAGS, .unit = "interp" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(shear);

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9,
    AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14,
    AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_YUVJ411P,
    AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV440P10,
    AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV420P12,
    AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV420P14,
    AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
    AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
    AV_PIX_FMT_YUVA420P,  AV_PIX_FMT_YUVA422P,   AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA444P9, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA444P12, AV_PIX_FMT_YUVA444P16,
    AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA422P16,
    AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA420P16,
    AV_PIX_FMT_GBRAP,     AV_PIX_FMT_GBRAP10,    AV_PIX_FMT_GBRAP12,    AV_PIX_FMT_GBRAP16,
    AV_PIX_FMT_NONE
};

#define NN(type, name)                                                       \
static int filter_slice_nn##name(AVFilterContext *ctx, void *arg, int jobnr, \
                                 int nb_jobs)                                \
{                                                                            \
    ThreadData *td = arg;                                                    \
    AVFrame *in = td->in;                                                    \
    AVFrame *out = td->out;                                                  \
    ShearContext *s = ctx->priv;                                             \
    const float shx = s->shx;                                                \
    const float shy = s->shy;                                                \
                                                                             \
    for (int p = 0; p < s->nb_planes; p++) {                                 \
        const int hsub = (p == 1 || p == 2) ? s->hsub: 1;                    \
        const int vsub = (p == 1 || p == 2) ? s->vsub: 1;                    \
        const int width = s->planewidth[p];                                  \
        const int height = s->planeheight[p];                                \
        const int wx = vsub * shx * height * 0.5f / hsub;                    \
        const int wy = hsub * shy * width  * 0.5f / vsub;                    \
        const int slice_start = (height * jobnr) / nb_jobs;                  \
        const int slice_end = (height * (jobnr+1)) / nb_jobs;                \
        const int src_linesize = in->linesize[p] / sizeof(type);             \
        const int dst_linesize = out->linesize[p] / sizeof(type);            \
        const type *src = (const type *)in->data[p];                         \
        type *dst = (type *)out->data[p] + slice_start * dst_linesize;       \
                                                                             \
        for (int y = slice_start; y < slice_end; y++) {                      \
            for (int x = 0; x < width; x++) {                                \
                int sx = x + vsub * shx * y / hsub - wx;                     \
                int sy = y + hsub * shy * x / vsub - wy;                     \
                                                                             \
                if (sx >= 0 && sx < width - 1 &&                             \
                    sy >= 0 && sy < height - 1) {                            \
                    dst[x] = src[sy * src_linesize + sx];                    \
                }                                                            \
            }                                                                \
                                                                             \
            dst += dst_linesize;                                             \
        }                                                                    \
    }                                                                        \
                                                                             \
    return 0;                                                                \
}

NN(uint8_t, 8)
NN(uint16_t, 16)

#define BL(type, name)                                                       \
static int filter_slice_bl##name(AVFilterContext *ctx, void *arg, int jobnr, \
                                 int nb_jobs)                                \
{                                                                            \
    ThreadData *td = arg;                                                    \
    AVFrame *in = td->in;                                                    \
    AVFrame *out = td->out;                                                  \
    ShearContext *s = ctx->priv;                                             \
    const int depth = s->depth;                                              \
    const float shx = s->shx;                                                \
    const float shy = s->shy;                                                \
                                                                             \
    for (int p = 0; p < s->nb_planes; p++) {                                 \
        const int hsub = (p == 1 || p == 2) ? s->hsub: 1;                    \
        const int vsub = (p == 1 || p == 2) ? s->vsub: 1;                    \
        const int width = s->planewidth[p];                                  \
        const int height = s->planeheight[p];                                \
        const float wx = vsub * shx * height * 0.5f / hsub;                  \
        const float wy = hsub * shy * width  * 0.5f / vsub;                  \
        const int slice_start = (height * jobnr) / nb_jobs;                  \
        const int slice_end = (height * (jobnr+1)) / nb_jobs;                \
        const int src_linesize = in->linesize[p] / sizeof(type);             \
        const int dst_linesize = out->linesize[p] / sizeof(type);            \
        const type *src = (const type *)in->data[p];                         \
        type *dst = (type *)out->data[p] + slice_start * dst_linesize;       \
                                                                             \
        for (int y = slice_start; y < slice_end; y++) {                      \
            for (int x = 0; x < width; x++) {                                \
                const float sx = x + vsub * shx * y / hsub - wx;             \
                const float sy = y + hsub * shy * x / vsub - wy;             \
                                                                             \
                if (sx >= 0 && sx < width - 1 &&                             \
                    sy >= 0 && sy < height - 1) {                            \
                    float sum = 0.f;                                         \
                    int ax = floorf(sx);                                     \
                    int ay = floorf(sy);                                     \
                    float du = sx - ax;                                      \
                    float dv = sy - ay;                                      \
                    int bx = FFMIN(ax + 1, width - 1);                       \
                    int by = FFMIN(ay + 1, height - 1);                      \
                                                                             \
                    sum += (1.f - du) * (1.f - dv) * src[ay * src_linesize + ax];\
                    sum += (      du) * (1.f - dv) * src[ay * src_linesize + bx];\
                    sum += (1.f - du) * (      dv) * src[by * src_linesize + ax];\
                    sum += (      du) * (      dv) * src[by * src_linesize + bx];\
                    dst[x] = av_clip_uintp2_c(lrintf(sum), depth);           \
                }                                                            \
            }                                                                \
                                                                             \
            dst += dst_linesize;                                             \
        }                                                                    \
    }                                                                        \
                                                                             \
    return 0;                                                                \
}

BL(uint8_t, 8)
BL(uint16_t, 16)

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    ShearContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;
    int ret;

    out = av_frame_alloc();
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        return ret;
    }

    av_frame_copy_props(out, in);

    /* fill background */
    if (s->fillcolor_enable)
        ff_fill_rectangle(&s->draw, &s->color, out->data, out->linesize,
                          0, 0, outlink->w, outlink->h);

    td.in = in, td.out = out;
    ff_filter_execute(ctx, s->filter_slice[s->interp], &td, NULL,
                      FFMIN(s->planeheight[1], ff_filter_get_nb_threads(ctx)));

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ShearContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    int ret;

    s->nb_planes = av_pix_fmt_count_planes(outlink->format);
    s->depth = desc->comp[0].depth;
    s->hsub = 1 << desc->log2_chroma_w;
    s->vsub = 1 << desc->log2_chroma_h;
    s->planewidth[1]  = s->planewidth[2] = AV_CEIL_RSHIFT(ctx->inputs[0]->w, desc->log2_chroma_w);
    s->planewidth[0]  = s->planewidth[3] = ctx->inputs[0]->w;
    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(ctx->inputs[0]->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = ctx->inputs[0]->h;

    ret = ff_draw_init2(&s->draw, outlink->format, outlink->colorspace, outlink->color_range, 0);
    if (ret < 0)
        return ret;
    ff_draw_color(&s->draw, &s->color, s->fillcolor);

    s->filter_slice[0] = s->depth <= 8 ? filter_slice_nn8 : filter_slice_nn16;
    s->filter_slice[1] = s->depth <= 8 ? filter_slice_bl8 : filter_slice_bl16;

    return 0;
}

static int process_command(AVFilterContext *ctx,
                           const char *cmd,
                           const char *arg,
                           char *res,
                           int res_len,
                           int flags)
{
    ShearContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg, res, res_len, flags);
    if (ret < 0)
        return ret;

    ff_draw_color(&s->draw, &s->color, s->fillcolor);

    return 0;
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const ShearContext *s_src = src->priv;
    ShearContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    if (memcmp(s_dst->fillcolor, s_src->fillcolor, sizeof(s_src->fillcolor))) {
        memcpy(s_dst->fillcolor, s_src->fillcolor, sizeof(s_src->fillcolor));
        ff_draw_color(&s_dst->draw, &s_dst->color, s_dst->fillcolor);
    }
    s_dst->fillcolor_enable = s_src->fillcolor_enable;
    s_dst->interp = s_src->interp;
    s_dst->shx = s_src->shx;
    s_dst->shy = s_src->shy;

    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_shear = {
    .p.name          = "shear",
    .p.description   = NULL_IF_CONFIG_SMALL("Shear transform the input video stream."),
    .p.priv_class    = &shear_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS |
                       AVFILTER_FLAG_FRAME_THREADS,
    .priv_size       = sizeof(ShearContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .process_command = process_command,
};
