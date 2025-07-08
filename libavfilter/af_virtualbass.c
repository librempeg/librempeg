/*
 * Copyright (c) 2022 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#include <float.h>

typedef struct AudioVirtualBassContext {
    const AVClass *class;

    double cutoff;
    double strength;

    double a[3], m[3], cf[2];

    void (*vb_stereo)(AVFilterContext *ctx, AVFrame *out, const AVFrame *in);
} AudioVirtualBassContext;

#define OFFSET(x) offsetof(AudioVirtualBassContext, x)
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption virtualbass_options[] = {
    { "cutoff",   "set virtual bass cutoff",   OFFSET(cutoff),   AV_OPT_TYPE_DOUBLE, {.dbl=250},100,500, FLAGS },
    { "strength", "set virtual bass strength", OFFSET(strength), AV_OPT_TYPE_DOUBLE, {.dbl=3},  0.5,  3, TFLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(virtualbass);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    AVFilterChannelLayouts *in_layout = NULL, *out_layout = NULL;
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    if ((ret = ff_add_channel_layout         (&in_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO)) < 0 ||
        (ret = ff_channel_layouts_ref(in_layout, &cfg_in[0]->channel_layouts)) < 0 ||
        (ret = ff_add_channel_layout         (&out_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_2POINT1)) < 0 ||
        (ret = ff_channel_layouts_ref(out_layout, &cfg_out[0]->channel_layouts)) < 0)
        return ret;

     return 0;
}

#define DEPTH 32
#include "virtualbass_template.c"

#undef DEPTH
#define DEPTH 64
#include "virtualbass_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioVirtualBassContext *s = ctx->priv;
    const double Q = 0.707;
    double g, k;

    g = tan(M_PI * s->cutoff / inlink->sample_rate);
    k = 1. / Q;
    s->a[0] = 1. / (1. + g * (g + k));
    s->a[1] = g * s->a[0];
    s->a[2] = g * s->a[1];
    s->m[0] = 0.;
    s->m[1] = 0.;
    s->m[2] = 1.;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->vb_stereo = vb_stereo_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->vb_stereo = vb_stereo_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioVirtualBassContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->vb_stereo(ctx, out, in);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_virtualbass = {
    .p.name          = "virtualbass",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Virtual Bass."),
    .p.priv_class    = &virtualbass_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size       = sizeof(AudioVirtualBassContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
