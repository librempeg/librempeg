/*
 * Copyright (c) 2015 Kyle Swanson <k@ylo.ph>.
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "audio.h"

typedef struct TremoloContext {
    const AVClass *class;
    double freq;
    double depth;

    int nb_channels;
    AVFrame *in;
    void *st;

    int (*init_state)(AVFilterContext *ctx);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} TremoloContext;

#define OFFSET(x) offsetof(TremoloContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption tremolo_options[] = {
    { "f", "set frequency in hertz",    OFFSET(freq),    AV_OPT_TYPE_DOUBLE,   {.dbl = 5.0},   0.1,   20000.0, FLAGS },
    { "d", "set depth as percentage",   OFFSET(depth),   AV_OPT_TYPE_DOUBLE,   {.dbl = 0.5},   0.0,   1.0,     FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(tremolo);

#define DEPTH 32
#include "tremolo_template.c"

#undef DEPTH
#define DEPTH 64
#include "tremolo_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    TremoloContext *s = ctx->priv;

    s->nb_channels = inlink->ch_layout.nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        s->init_state = init_state_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        s->init_state = init_state_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    TremoloContext *s = ctx->priv;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    s->in = in;
    ff_filter_execute(ctx, s->filter_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->in = NULL;
    if (in != out)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TremoloContext *s = ctx->priv;

    av_freep(&s->st);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    TremoloContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    return s->init_state(ctx);
}

static const AVFilterPad tremolo_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_tremolo = {
    .p.name        = "tremolo",
    .p.description = NULL_IF_CONFIG_SMALL("Apply tremolo effect."),
    .p.priv_class  = &tremolo_class,
    .priv_size     = sizeof(TremoloContext),
    .uninit        = uninit,
    FILTER_INPUTS(tremolo_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
