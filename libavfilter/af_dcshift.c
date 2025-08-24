/*
 * Copyright (c) 2000 Chris Ausbrooks <weed@bucket.pp.ualr.edu>
 * Copyright (c) 2000 Fabien COELHO <fabien@coelho.net>
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

#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct DCShiftContext {
    const AVClass *class;

    double dcshift;
    double limitergain;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} DCShiftContext;

#define OFFSET(x) offsetof(DCShiftContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption dcshift_options[] = {
    { "shift",       "set DC shift",     OFFSET(dcshift),     AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1, 1, A },
    { "limitergain", "set limiter gain", OFFSET(limitergain), AV_OPT_TYPE_DOUBLE, {.dbl=0},  0, 1, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(dcshift);

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 16
#include "dcshift_template.c"

#undef DEPTH
#define DEPTH 32
#include "dcshift_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    DCShiftContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S32P:
        s->filter_channels = filter_channels_s32p;
        break;
    case AV_SAMPLE_FMT_S16P:
        s->filter_channels = filter_channels_s16p;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    DCShiftContext *s = ctx->priv;
    ThreadData td;
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

    td.in = in;
    td.out = out;

    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));
     if (out != in)
        ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static const AVFilterPad dcshift_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_dcshift = {
    .p.name         = "dcshift",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply a DC shift to the audio."),
    .p.priv_class   = &dcshift_class,
    .priv_size      = sizeof(DCShiftContext),
    FILTER_INPUTS(dcshift_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P),
    .process_command = ff_filter_process_command,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                      AVFILTER_FLAG_SLICE_THREADS,
};
