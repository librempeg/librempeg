/*
 * Copyright (c) 2016 Paul B Mahol
 *
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct SliceStats {
    uint64_t stats[4];
} SliceStats;

typedef struct BPNContext {
    const AVClass *class;

    int bitplane;
    int filter;

    SliceStats *stats;

    int nb_planes;
    int nb_threads;
    int planeheight[4];
    int planewidth[4];
    int depth;
} BPNContext;

#define OFFSET(x) offsetof(BPNContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
static const AVOption bitplanenoise_options[] = {
    { "bitplane", "set bit plane to use for measuring noise",  OFFSET(bitplane), AV_OPT_TYPE_INT,  {.i64=1}, 1, 16, FLAGS},
    { "filter",   "show noisy pixels",                         OFFSET(filter),   AV_OPT_TYPE_BOOL, {.i64=0}, 0,  1, FLAGS},
    { NULL }
};

AVFILTER_DEFINE_CLASS(bitplanenoise);

static const enum AVPixelFormat pixfmts[] = {
    AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ411P,
    AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUV444P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV420P9,
    AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV420P10,
    AV_PIX_FMT_YUV440P10,
    AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV420P12,
    AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV420P14,
    AV_PIX_FMT_YUV444P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV420P16,
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
    AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
    AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9, AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_NONE
};

static int config_input(AVFilterLink *inlink)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    AVFilterContext *ctx = inlink->dst;
    BPNContext *s = ctx->priv;

    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = inlink->h;
    s->planewidth[1]  = s->planewidth[2]  = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    s->planewidth[0]  = s->planewidth[3]  = inlink->w;

    s->nb_threads = FFMAX(1, FFMIN(s->planeheight[1], ff_filter_get_nb_threads(ctx)));
    s->stats = av_calloc(s->nb_threads, sizeof(*s->stats));
    if (!s->stats)
        return AVERROR(ENOMEM);

    s->depth = desc->comp[0].depth;

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define CHECK_BIT(x, a, b, c) { \
    int bit = (((val[(x)] & mask) == (val[(x) + (a)] & mask)) + \
               ((val[(x)] & mask) == (val[(x) + (b)] & mask)) + \
               ((val[(x)] & mask) == (val[(x) + (c)] & mask))) > 1; \
    if (dst) \
        dst[(x)] = bit ? factor : 0; \
    stats->stats[plane] += bit; }

static int filter_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    BPNContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    const int mask = (1 << (s->bitplane - 1));
    const int factor = (1 << s->depth) - 1;
    SliceStats *stats = &s->stats[jobnr];

    if (s->depth <= 8) {
        for (int plane = 0; plane < s->nb_planes; plane++) {
            const int h = s->planeheight[plane];
            const int w = s->planewidth[plane];
            const int slice_h_start = ((h-1) * jobnr) / nb_jobs;
            const int slice_h_end = ((h-1) * (jobnr+1)) / nb_jobs;
            const int linesize = s->planeheight[plane] > 1 ? in->linesize[plane] : 0;
            const int dlinesize = out->linesize[plane];
            uint8_t *val = in->data[plane] + slice_h_start * linesize;
            uint8_t *dst = s->filter ? out->data[plane] + slice_h_start * dlinesize : NULL;

            for (int y = slice_h_start; y < slice_h_end; y++) {
                CHECK_BIT(0, 1, 1 + linesize, linesize)

                for (int x = 1; x < w - 1; x++) {
                    CHECK_BIT(x, -1, 1, linesize)
                }

                CHECK_BIT(w, -1, -1 + linesize, linesize)

                val += linesize;
                if (dst)
                    dst += dlinesize;
            }

            if (jobnr < nb_jobs-1)
                continue;

            CHECK_BIT(0, 1, 1 - linesize, -linesize)

            for (int x = 1; x < w - 1; x++) {
                CHECK_BIT(x, -1, 1, -linesize)
            }

            CHECK_BIT(w, -1, -1 - linesize, -linesize)
        }
    } else {
        for (int plane = 0; plane < s->nb_planes; plane++) {
            const int h = s->planeheight[plane];
            const int w = s->planewidth[plane];
            const int slice_h_start = ((h-1) * jobnr) / nb_jobs;
            const int slice_h_end = ((h-1) * (jobnr+1)) / nb_jobs;
            const int linesize = s->planeheight[plane] > 1 ? in->linesize[plane] / 2 : 0;
            const int dlinesize = out->linesize[plane] / 2;
            uint16_t *val = (uint16_t *)in->data[plane] + slice_h_start * linesize;
            uint16_t *dst = s->filter ? (uint16_t *)out->data[plane] + slice_h_start * dlinesize : NULL;

            val = (uint16_t *)in->data[plane];
            for (int y = slice_h_start; y < slice_h_end; y++) {
                CHECK_BIT(0, 1, 1 + linesize, linesize)

                for (int x = 1; x < w - 1; x++) {
                    CHECK_BIT(x, -1, 1, linesize)
                }

                CHECK_BIT(w, -1, -1 + linesize, linesize)

                val += linesize;
                if (dst)
                    dst += dlinesize;
            }

            if (jobnr < nb_jobs-1)
                continue;

            CHECK_BIT(0, 1, 1 - linesize, -linesize)

            for (int x = 1; x < w - 1; x++) {
                CHECK_BIT(x, -1, 1, -linesize)
            }

            CHECK_BIT(w, -1, -1 -linesize, -linesize)
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    BPNContext *s = ctx->priv;
    uint64_t stats[4] = { 0 };
    char metabuf[128];
    ThreadData td;
    AVFrame *out = s->filter ? NULL : in;

    if (!out) {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    memset(s->stats, 0, sizeof(*s->stats) * s->nb_threads);
    td.in = in; td.out = out;
    ff_filter_execute(ctx, filter_slice, &td, NULL, s->nb_threads);

    for (int n = 0; n < s->nb_threads; n++) {
        SliceStats *slice_stats = &s->stats[n];

        for (int plane = 0; plane < s->nb_planes; plane++)
            stats[plane] += slice_stats->stats[plane];
    }

    for (int plane = 0; plane < s->nb_planes; plane++) {
        char key[32];
        double value;

        value = stats[plane] / ((double)s->planewidth[plane] * s->planeheight[plane]);
        snprintf(key, sizeof(key), "lavfi.bitplanenoise.%d.%d", plane, s->bitplane);
        snprintf(metabuf, sizeof(metabuf), "%f", 1. - 2.* fabs(value - 0.5));
        av_dict_set(&out->metadata, key, metabuf, 0);
    }

    if (out != in)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    BPNContext *s = ctx->priv;

    av_freep(&s->stats);
}

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .filter_frame   = filter_frame,
        .config_props   = config_input,
    },
};

const FFFilter ff_vf_bitplanenoise = {
    .p.name         = "bitplanenoise",
    .p.description  = NULL_IF_CONFIG_SMALL("Measure bit plane noise."),
    .p.priv_class   = &bitplanenoise_class,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS,
    .priv_size      = sizeof(BPNContext),
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pixfmts),
    .process_command = ff_filter_process_command,
};
