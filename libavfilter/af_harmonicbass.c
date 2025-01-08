/*
 * Copyright (c) 2025 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#include <float.h>

typedef struct AudioHarmonicBassContext {
    const AVClass *class;

    double scutoff;
    double sqfactor;
    double slevel;
    double hcutoff;
    double hqfactor;
    double hlevel;
    double hdrive;
    double hbias;

    double sa[3], sm[3], scf[2];
    double ha[3], hm[3], hcf[2];

    void (*hb_stereo)(AVFilterContext *ctx, AVFrame *out, AVFrame *in);
} AudioHarmonicBassContext;

#define OFFSET(x) offsetof(AudioHarmonicBassContext, x)
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption harmonicbass_options[] = {
    { "scutoff",  "set source bass cutoff",     OFFSET(scutoff),   AV_OPT_TYPE_DOUBLE, {.dbl=250},100,500,  FLAGS  },
    { "sqfactor", "set source bass Q-factor",   OFFSET(sqfactor),  AV_OPT_TYPE_DOUBLE, {.dbl=2.0},0.5,4.0,  FLAGS  },
    { "slevel",   "set source bass level",      OFFSET(slevel),    AV_OPT_TYPE_DOUBLE, {.dbl=0.0},0.0,2.0,  TFLAGS },
    { "hcutoff",  "set harmonic bass cutoff",   OFFSET(hcutoff),   AV_OPT_TYPE_DOUBLE, {.dbl=500},100,800,  FLAGS  },
    { "hqfactor", "set harmonic bass Q-factor", OFFSET(hqfactor),  AV_OPT_TYPE_DOUBLE, {.dbl=1.0},0.25,2.0, FLAGS  },
    { "hlevel",   "set harmonic bass level",    OFFSET(hlevel),    AV_OPT_TYPE_DOUBLE, {.dbl=0.8},0.0,2.0,  TFLAGS },
    { "hdrive",   "set harmonic drive level",   OFFSET(hdrive),    AV_OPT_TYPE_DOUBLE, {.dbl=10},0.0,20.0,  TFLAGS },
    { "hbias",    "set harmonic bias level",    OFFSET(hbias),     AV_OPT_TYPE_DOUBLE, {.dbl=0.0},-0.5,0.5, TFLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(harmonicbass);

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
#include "harmonicbass_template.c"

#undef DEPTH
#define DEPTH 64
#include "harmonicbass_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioHarmonicBassContext *s = ctx->priv;
    double g, k;

    g = tan(M_PI * s->scutoff / inlink->sample_rate);
    k = 1. / s->sqfactor;
    s->sa[0] = 1. / (1. + g * (g + k));
    s->sa[1] = g * s->sa[0];
    s->sa[2] = g * s->sa[1];
    s->sm[0] = 0.;
    s->sm[1] = 0.;
    s->sm[2] = 1.;

    g = tan(M_PI * s->hcutoff / inlink->sample_rate);
    k = 1. / s->hqfactor;
    s->ha[0] = 1. / (1. + g * (g + k));
    s->ha[1] = g * s->ha[0];
    s->ha[2] = g * s->ha[1];
    s->hm[0] = 0.;
    s->hm[1] = 0.;
    s->hm[2] = 1.;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->hb_stereo = hb_stereo_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->hb_stereo = hb_stereo_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioHarmonicBassContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->hb_stereo(ctx, out, in);

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

const FFFilter ff_af_harmonicbass = {
    .p.name          = "harmonicbass",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Harmonic Bass."),
    .p.priv_class    = &harmonicbass_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size       = sizeof(AudioHarmonicBassContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};