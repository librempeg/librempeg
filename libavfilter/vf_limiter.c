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

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"
#include "limiter.h"
#include "video.h"

typedef struct ThreadData {
    AVFrame *in;
    AVFrame *out;
} ThreadData;

typedef struct LimiterContext {
    const AVClass *class;
    int min;
    int max;
    int planes;
    int nb_planes;
    int linesize[4];
    int width[4];
    int height[4];

    LimiterDSPContext dsp;
} LimiterContext;

#define OFFSET(x) offsetof(LimiterContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption limiter_options[] = {
    { "min",    "set min value", OFFSET(min),    AV_OPT_TYPE_INT, {.i64=0},     0, 65535, .flags = FLAGS },
    { "max",    "set max value", OFFSET(max),    AV_OPT_TYPE_INT, {.i64=65535}, 0, 65535, .flags = FLAGS },
    { "planes", "set planes",    OFFSET(planes), AV_OPT_TYPE_INT, {.i64=15},    0,    15, .flags = FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(limiter);

static av_cold int init(AVFilterContext *ctx)
{
    LimiterContext *s = ctx->priv;

    if (s->min > s->max)
        return AVERROR(EINVAL);
    return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV420P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV444P14,
    AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA444P9,
    AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA444P10,
    AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA444P12,
    AV_PIX_FMT_YUVA420P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA444P16,
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
    AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
    AV_PIX_FMT_GBRAP, AV_PIX_FMT_GBRAP10, AV_PIX_FMT_GBRAP12, AV_PIX_FMT_GBRAP16,
    AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9, AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_NONE
};

#define LIMITER(n, type)                                        \
static void limiter##n(const uint8_t *ssrc, uint8_t *ddst,      \
                       ptrdiff_t slinesize, ptrdiff_t dlinesize,\
                       int w, int h, int min, int max)          \
{                                                               \
    const type *src = (const type *)ssrc;                       \
    type *dst = (type *)ddst;                                   \
                                                                \
    dlinesize /= sizeof(type);                                  \
    slinesize /= sizeof(type);                                  \
                                                                \
    for (int y = 0; y < h; y++) {                               \
        for (int x = 0; x < w; x++) {                           \
            dst[x] = av_clip(src[x], min, max);                 \
        }                                                       \
                                                                \
        dst += dlinesize;                                       \
        src += slinesize;                                       \
    }                                                           \
}

LIMITER(8,  uint8_t)
LIMITER(16, uint16_t)

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    LimiterContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int depth, vsub, hsub, ret;

    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    if ((ret = av_image_fill_linesizes(s->linesize, inlink->format, inlink->w)) < 0)
        return ret;

    depth = desc->comp[0].depth;
    hsub = desc->log2_chroma_w;
    vsub = desc->log2_chroma_h;
    s->height[1] = s->height[2] = AV_CEIL_RSHIFT(inlink->h, vsub);
    s->height[0] = s->height[3] = inlink->h;
    s->width[1]  = s->width[2]  = AV_CEIL_RSHIFT(inlink->w, hsub);
    s->width[0]  = s->width[3]  = inlink->w;

    s->max = FFMIN(s->max, (1 << depth) - 1);
    s->min = FFMIN(s->min, (1 << depth) - 1);

    if (depth == 8) {
        s->dsp.limiter = limiter8;
    } else {
        s->dsp.limiter = limiter16;
    }

#if ARCH_X86
    ff_limiter_init_x86(&s->dsp, desc->comp[0].depth);
#endif

    return 0;
}

static int filter_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    LimiterContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    int p;

    for (p = 0; p < s->nb_planes; p++) {
        const int h = s->height[p];
        const int slice_start = (h * jobnr) / nb_jobs;
        const int slice_end = (h * (jobnr+1)) / nb_jobs;

        if (!((1 << p) & s->planes)) {
            if (out != in)
                av_image_copy_plane(out->data[p] + slice_start * out->linesize[p],
                                    out->linesize[p],
                                    in->data[p] + slice_start * in->linesize[p],
                                    in->linesize[p],
                                    s->linesize[p], slice_end - slice_start);
            continue;
        }

        s->dsp.limiter(in->data[p] + slice_start * in->linesize[p],
                       out->data[p] + slice_start * out->linesize[p],
                       in->linesize[p], out->linesize[p],
                       s->width[p], slice_end - slice_start,
                       s->min, s->max);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    LimiterContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;
    int ret;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = av_frame_alloc();
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        ret = ff_filter_get_buffer(ctx, out);
        if (ret < 0) {
            av_frame_free(&in);
            av_frame_free(&out);
            return ret;
        }

        av_frame_copy_props(out, in);
    }

    td.out = out;
    td.in = in;
    ff_filter_execute(ctx, filter_slice, &td, NULL,
                      FFMIN(s->height[2], ff_filter_get_nb_threads(ctx)));
    if (out != in)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_input(ctx->inputs[0]);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const LimiterContext *s_src = src->priv;
    LimiterContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    s_dst->min    = s_src->min;
    s_dst->max    = s_src->max;
    s_dst->planes = s_src->planes;

    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_limiter = {
    .p.name        = "limiter",
    .p.description = NULL_IF_CONFIG_SMALL("Limit pixels components to the specified range."),
    .p.priv_class  = &limiter_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_FRAME_THREADS |
                     AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(LimiterContext),
    .init          = init,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .process_command = process_command,
};
