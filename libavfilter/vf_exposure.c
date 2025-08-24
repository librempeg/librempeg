/*
 * Copyright (c) 2021 Paul B Mahol
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

#include <float.h>

#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct ExposureContext {
    const AVClass *class;

    float exposure;
    float black;

    float scale;
    int (*do_slice)(AVFilterContext *s, void *arg,
                    int jobnr, int nb_jobs);
} ExposureContext;

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

static int exposure_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ExposureContext *s = ctx->priv;
    ThreadData *td = arg;
    const int width = td->out->width;
    const int height = td->out->height;
    const int slice_start = (height * jobnr) / nb_jobs;
    const int slice_end = (height * (jobnr + 1)) / nb_jobs;
    const float black = s->black;
    const float scale = s->scale;

    for (int p = 0; p < 3; p++) {
        const int slinesize = td->in->linesize[p] / 4;
        const int dlinesize = td->out->linesize[p] / 4;
        const float *src = (const float *)td->in->data[p] + slice_start * slinesize;
        float *ptr = (float *)td->out->data[p] + slice_start * dlinesize;
        for (int y = slice_start; y < slice_end; y++) {
            for (int x = 0; x < width; x++)
                ptr[x] = (src[x] - black) * scale;

            ptr += dlinesize;
            src += slinesize;
        }
    }

    if (td->in->data[3] && td->in->linesize[3] && td->in != td->out) {
        const int slinesize = td->in->linesize[3] / 4;
        const int dlinesize = td->out->linesize[3] / 4;
        const float *src = (const float *)td->in->data[3] + slice_start * slinesize;
        float *ptr = (float *)td->out->data[3] + slice_start * dlinesize;
        for (int y = slice_start; y < slice_end; y++) {
            memcpy(ptr, src, width * sizeof(*ptr));
            ptr += dlinesize;
            src += slinesize;
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ExposureContext *s = ctx->priv;
    float diff = fabsf(exp2f(-s->exposure) - s->black);
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        int ret;

        out = ff_graph_frame_alloc(ctx);
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
    }

    diff = diff > 0.f ? diff : 1.f / 1024.f;
    s->scale = 1.f / diff;
    td.out = out;
    td.in = in;
    ff_filter_execute(ctx, s->do_slice, &td, NULL,
                      FFMIN(out->height, ff_filter_get_nb_threads(ctx)));

    if (out != in)
        ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ExposureContext *s = ctx->priv;

    s->do_slice = exposure_slice;

    return 0;
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const ExposureContext *s_src = src->priv;
    ExposureContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    s_dst->exposure = s_src->exposure;
    s_dst->black = s_src->black;

    return 0;
}
#endif

static const AVFilterPad exposure_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .filter_frame   = filter_frame,
        .config_props   = config_input,
    },
};

#define OFFSET(x) offsetof(ExposureContext, x)
#define VF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption exposure_options[] = {
    { "exposure", "set the exposure correction",    OFFSET(exposure), AV_OPT_TYPE_FLOAT, {.dbl=0}, -3, 3, VF },
    { "black",    "set the black level correction", OFFSET(black),    AV_OPT_TYPE_FLOAT, {.dbl=0}, -1, 1, VF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(exposure);

const FFFilter ff_vf_exposure = {
    .p.name        = "exposure",
    .p.description = NULL_IF_CONFIG_SMALL("Adjust exposure of the video stream."),
    .p.priv_class  = &exposure_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_FRAME_THREADS,
    .priv_size     = sizeof(ExposureContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(exposure_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS(AV_PIX_FMT_GBRPF32, AV_PIX_FMT_GBRAPF32),
    .process_command = ff_filter_process_command,
};
