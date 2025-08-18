/*
 * Copyright (c) 2025 Paul B Mahol
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
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

typedef struct AmbiophonicContext {
    const AVClass *class;

    double lcut, hcut;
    double delay_us;
    double att;
    double delay[2];
    double tmp[2];
    int filter[2];
    int max_delay;
    int nb_channels;

    void *st;
    void *biquad_st;

    int (*init_filter)(AVFilterContext *ctx);
    void (*uninit_filter)(AVFilterContext *ctx);
    void (*do_filter)(AVFilterContext *ctx, AVFrame *in, AVFrame *out);

    int (*init_biquad)(AVFilterContext *ctx, void **st,
                       const int nb_channels, const int block_samples, const int reset,
                       const double a[3], const double b[3], const double mix);

    void (*biquad_filter)(void *st, const void *ibuf, void *obuf, int len,
                          int ch, int disabled);
} AmbiophonicContext;

#define OFFSET(x) offsetof(AmbiophonicContext, x)
#define A  AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption ambiophonic_options[] = {
    { "lcut",  "set the low-cut frequency",  OFFSET(lcut),     AV_OPT_TYPE_DOUBLE, {.dbl=250},  1, INT_MAX, A },
    { "hcut",  "set the high-cut frequency", OFFSET(hcut),     AV_OPT_TYPE_DOUBLE, {.dbl=5000}, 1, INT_MAX, A },
    { "att",   "set the attenuation",        OFFSET(att),      AV_OPT_TYPE_DOUBLE, {.dbl=0.707},0,       1, AT},
    { "delay", "set the delay time in us",   OFFSET(delay_us), AV_OPT_TYPE_DOUBLE, {.dbl=80},   0.01, 1000, A },
//  { "lc",    "enable low-cut",             OFFSET(filter[0]),AV_OPT_TYPE_BOOL,   {.i64=1},    0,       1, A },
//  { "hc",    "enable high-cut",            OFFSET(filter[1]),AV_OPT_TYPE_BOOL,   {.i64=1},    0,       1, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(ambiophonic);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    static const AVChannelLayout layouts[] = {
        AV_CHANNEL_LAYOUT_STEREO,
        { .nb_channels = 0 },
    };
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    return ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
}

#define CLIP_RESET 0
#define BIQUAD_DI 0
#define BIQUAD_DII 0
#define BIQUAD_TDI 0
#define BIQUAD_TDII 0
#define BIQUAD_LATT 0
#define BIQUAD_SVF 0
#define BIQUAD_WDF 0
#define BIQUAD_ZDF 1

#define DEPTH 32
#include "ambiophonic_template.c"
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 64
#include "ambiophonic_template.c"
#include "biquads_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AmbiophonicContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_filter = ambiophonic_fltp;
        s->init_filter = init_ambiophonic_fltp;
        s->uninit_filter = uninit_ambiophonic_fltp;
        s->biquad_filter = biquad_zdf_fltp;
        s->init_biquad = init_biquad_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_filter = ambiophonic_dblp;
        s->init_filter = init_ambiophonic_dblp;
        s->uninit_filter = uninit_ambiophonic_dblp;
        s->biquad_filter = biquad_zdf_dblp;
        s->init_biquad = init_biquad_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_filter(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AmbiophonicContext *s = ctx->priv;
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->do_filter(ctx, in, out);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AmbiophonicContext *s = ctx->priv;

    if (s->uninit_filter)
        s->uninit_filter(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_ambiophonic = {
    .p.name         = "ambiophonic",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply ambiophonic crosstalk elimination."),
    .p.priv_class   = &ambiophonic_class,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size      = sizeof(AmbiophonicContext),
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
