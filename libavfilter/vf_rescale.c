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

#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "avfilter.h"
#include "video.h"
#include "filters.h"
#include "formats.h"

enum Interpolation {
    NEAREST,
    LINEAR,
    NB_INTERP
};

typedef struct ReScaleContext {
    const AVClass *class;
    int w, h;
    int interpolation;

    const AVPixFmtDescriptor *dst_desc, *src_desc;

    int pass;

    int (*rescale_slice[NB_INTERP])(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} ReScaleContext;

#define OFFSET(x) offsetof(ReScaleContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption rescale_options[] = {
    { "size", "output video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, .flags = FLAGS },
    { "interpolation", "set interpolation", OFFSET(interpolation), AV_OPT_TYPE_INT, {.i64=LINEAR}, 0, NB_INTERP-1, .flags=TFLAGS, .unit="interp" },
    { "nearest", "nearest neighbor", 0, AV_OPT_TYPE_CONST, {.i64=NEAREST}, 0, 0, TFLAGS, .unit = "interp" },
    { "linear",  "bilinear", 0, AV_OPT_TYPE_CONST, {.i64=LINEAR},  0, 0, TFLAGS, .unit = "interp" },
    {NULL}
};

AVFILTER_DEFINE_CLASS(rescale);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AVPixFmtDescriptor *desc;
    AVFilterFormats *formats;
    int ret;

    formats = NULL;
    for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
        if (!(desc->flags & AV_PIX_FMT_FLAG_PAL ||
              desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
              desc->flags & AV_PIX_FMT_FLAG_BITSTREAM) &&
            ((desc->flags & AV_PIX_FMT_FLAG_PLANAR) || (desc->nb_components == 1)) &&
            (ret = ff_add_format(&formats, fmt)) < 0)
            return ret;
    }

    formats->flags = FILTER_SAME_BITDEPTH | FILTER_SAME_ENDIANNESS | FILTER_SAME_RGB_FLAG | FILTER_SAME_PLANAR_FLAG;
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = NULL;
    for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
        if (!(desc->flags & AV_PIX_FMT_FLAG_PAL ||
              desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
              desc->flags & AV_PIX_FMT_FLAG_BITSTREAM) &&
            ((desc->flags & AV_PIX_FMT_FLAG_PLANAR) || (desc->nb_components == 1)) &&
            (ret = ff_add_format(&formats, fmt)) < 0)
            return ret;
    }

    formats->flags = FILTER_SAME_BITDEPTH | FILTER_SAME_ENDIANNESS | FILTER_SAME_RGB_FLAG | FILTER_SAME_PLANAR_FLAG;
    return ff_formats_ref(formats, &cfg_out[0]->formats);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 8
#include "rescale_template.c"

#undef DEPTH
#define DEPTH 16
#include "rescale_template.c"

#undef DEPTH
#define DEPTH 32
#include "rescale_template.c"

#undef DEPTH
#define DEPTH 33
#include "rescale_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ReScaleContext *s = ctx->priv;

    if (!s->w || !s->h) {
        outlink->w = inlink->w;
        outlink->h = inlink->h;
    } else {
        outlink->w = s->w;
        outlink->h = s->h;
    }

    if (outlink->w == inlink->w && outlink->h == inlink->h &&
        outlink->format == inlink->format) {
        s->pass = 1;
        return 0;
    }

    s->dst_desc = av_pix_fmt_desc_get(outlink->format);
    s->src_desc = av_pix_fmt_desc_get(inlink->format);

    if (s->dst_desc->comp[0].depth <= 8) {
        s->rescale_slice[NEAREST] = rescale_slice_8;
        s->rescale_slice[LINEAR] = rescale_slice_linear_8;
    } else if (s->dst_desc->comp[0].depth <= 16) {
        s->rescale_slice[NEAREST] = rescale_slice_16;
        s->rescale_slice[LINEAR] = rescale_slice_linear_16;
    } else if (s->dst_desc->comp[0].depth <= 32 && !(s->dst_desc->flags & AV_PIX_FMT_FLAG_FLOAT)) {
        s->rescale_slice[NEAREST] = rescale_slice_32;
        s->rescale_slice[LINEAR] = rescale_slice_linear_32;
    } else if (s->dst_desc->comp[0].depth <= 32 && (s->dst_desc->flags & AV_PIX_FMT_FLAG_FLOAT)) {
        s->rescale_slice[NEAREST] = rescale_slice_33;
        s->rescale_slice[LINEAR] = rescale_slice_linear_33;
    } else {
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ReScaleContext *s = ctx->priv;
    AVFrame *out;

    if (s->pass) {
        out = in;
    } else {
        ThreadData td;
        int ret, nb_jobs;

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

        td.in = in;
        td.out = out;

        nb_jobs = out->height;

        ff_filter_execute(ctx, s->rescale_slice[s->interpolation], &td, NULL,
                          FFMIN(nb_jobs, ff_filter_get_nb_threads(ctx)));

        av_frame_copy_props(out, in);
        ff_graph_frame_free(ctx, &in);
    }

    return ff_filter_frame(outlink, out);
}

static AVFrame *get_in_video_buffer(AVFilterLink *inlink, int w, int h)
{
    AVFilterContext *ctx = inlink->dst;
    ReScaleContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_video_buffer   (inlink, w, h) :
        ff_default_get_video_buffer(inlink, w, h);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const ReScaleContext *s_src = src->priv;
    ReScaleContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    s_dst->interpolation  = s_src->interpolation;

    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
        .get_buffer.video = get_in_video_buffer,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_rescale = {
    .p.name        = "rescale",
    .p.description = NULL_IF_CONFIG_SMALL("Rescale Video stream."),
    .p.priv_class  = &rescale_class,
    .priv_size     = sizeof(ReScaleContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_QUERY_FUNC2(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_FRAME_THREADS,
    .process_command = ff_filter_process_command,
};
