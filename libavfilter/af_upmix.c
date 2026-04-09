/*
 * Copyright (c) 2026 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct ThreadData {
    AVFrame *frame;
    int idx;
} ThreadData;

typedef struct AUpmixContext {
    const AVClass *class;

    AVChannelLayout out_ch_layout;

    int channels;
    int n;

    int aswift_size;
    float gain;
    float smooth;

    void *state;
    void *x_pos;
    void *y_pos;

    int (*do_upmix_in)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*do_angle)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*do_upmix)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*do_upmix_out)(AVFilterContext *ctx, AVFrame *out, const int ch, const int n);
    int (*upmix_init)(AVFilterContext *ctx);
    void (*upmix_uninit)(AVFilterContext *ctx);
} AUpmixContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    const AUpmixContext *s = ctx->priv;
    AVFilterChannelLayouts *layouts;
    int ret;

    ret = ff_set_sample_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    layouts = NULL;
    ret = ff_add_channel_layout(&layouts, &s->out_ch_layout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    layouts = NULL;
    ret = ff_add_channel_layout(&layouts, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO);
    if (ret)
        return ret;

    return ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts);
}

enum SurroundChannel {
    SC_FL = 1, SC_FR, SC_FC, SC_LF, SC_BL, SC_BR, SC_BC, SC_SL, SC_SR,
    SC_TC, SC_TFC, SC_TFL, SC_TFR, SC_TBC, SC_TBL, SC_TBR,
    SC_LF2, SC_TSL, SC_TSR, SC_BFC, SC_BFL, SC_BFR, SC_FLC, SC_FRC,
    SC_NB,
};

static const float sc_ch_pos[SC_NB][5] = {
    [SC_FL]  = { -1.f,  1.f, 0.f, 1.f, 0.f },
    [SC_FR]  = {  1.f,  1.f, 0.f, 0.f, 1.f },
    [SC_FC]  = {  0.f,  1.f, 0.f, .5f, .5f },
    [SC_SL]  = { -1.f,  0.f, 0.f, 1.f, 0.f },
    [SC_SR]  = {  1.f,  0.f, 0.f, 0.f, 1.f },
    [SC_BL]  = { -1.f, -1.f, 0.f, 1.f, 0.f },
    [SC_BR]  = {  1.f, -1.f, 0.f, 0.f, 1.f },
    [SC_BC]  = {  0.f, -1.f, 0.f, .5f, .5f },
    [SC_LF]  = {  0.f,  0.f, 0.f, .5f, .5f },
    [SC_LF2] = {  0.f,  0.f, 0.f, .5f, .5f },
    [SC_FLC] = {-0.4f,  1.f, 0.f, 1.f, 0.f },
    [SC_FRC] = { 0.4f,  1.f, 0.f, 0.f, 1.f },
    [SC_TFL] = {-1.0f,  1.f, 1.f, 1.f, 0.f },
    [SC_TFR] = { 1.0f,  1.f, 1.f, 0.f, 1.f },
    [SC_TSL] = {-1.0f,  0.f, 1.f, 1.f, 0.f },
    [SC_TSR] = { 1.0f,  0.f, 1.f, 0.f, 1.f },
    [SC_TBL] = {-1.0f, -1.f, 1.f, 1.f, 0.f },
    [SC_TBR] = { 1.0f, -1.f, 1.f, 0.f, 1.f },
};

static const int sc_map[64] = {
    [AV_CHAN_FRONT_LEFT      ] = SC_FL,
    [AV_CHAN_FRONT_RIGHT     ] = SC_FR,
    [AV_CHAN_FRONT_CENTER    ] = SC_FC,
    [AV_CHAN_LOW_FREQUENCY   ] = SC_LF,
    [AV_CHAN_BACK_LEFT       ] = SC_BL,
    [AV_CHAN_BACK_RIGHT      ] = SC_BR,
    [AV_CHAN_BACK_CENTER     ] = SC_BC,
    [AV_CHAN_SIDE_LEFT       ] = SC_SL,
    [AV_CHAN_SIDE_RIGHT      ] = SC_SR,
    [AV_CHAN_TOP_CENTER      ] = SC_TC,
    [AV_CHAN_TOP_FRONT_CENTER] = SC_TFC,
    [AV_CHAN_TOP_FRONT_LEFT  ] = SC_TFL,
    [AV_CHAN_TOP_FRONT_RIGHT ] = SC_TFR,
    [AV_CHAN_TOP_BACK_CENTER ] = SC_TBC,
    [AV_CHAN_TOP_BACK_LEFT   ] = SC_TBL,
    [AV_CHAN_TOP_BACK_RIGHT  ] = SC_TBR,
    [AV_CHAN_LOW_FREQUENCY_2 ] = SC_LF2,
    [AV_CHAN_TOP_SIDE_LEFT   ] = SC_TSL,
    [AV_CHAN_TOP_SIDE_RIGHT  ] = SC_TSR,
    [AV_CHAN_BOTTOM_FRONT_CENTER] = SC_BFC,
    [AV_CHAN_BOTTOM_FRONT_LEFT  ] = SC_BFL,
    [AV_CHAN_BOTTOM_FRONT_RIGHT ] = SC_BFR,
    [AV_CHAN_FRONT_LEFT_OF_CENTER] = SC_FLC,
    [AV_CHAN_FRONT_RIGHT_OF_CENTER] = SC_FRC,
};

#define DEPTH 32
#include "upmix_template.c"

#undef DEPTH
#define DEPTH 64
#include "upmix_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AUpmixContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_angle = angle_fltp;
        s->do_upmix = upmix_fltp;
        s->do_upmix_in = upmix_in_fltp;
        s->do_upmix_out = upmix_out_fltp;
        s->upmix_init = upmix_init_fltp;
        s->upmix_uninit = upmix_uninit_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_angle = angle_dblp;
        s->do_upmix = upmix_dblp;
        s->do_upmix_in = upmix_in_dblp;
        s->do_upmix_out = upmix_out_dblp;
        s->upmix_init = upmix_init_dblp;
        s->upmix_uninit = upmix_uninit_dblp;
        break;
    default:
        AVERROR_BUG;
    }

    return s->upmix_init(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_out_channels = outlink->ch_layout.nb_channels;
    AUpmixContext *s = ctx->priv;
    const int aswift_size = s->aswift_size;
    const int nb_threads = FFMIN(aswift_size, ff_filter_get_nb_threads(ctx));
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    for (int n = 0; n < in->nb_samples; n++) {
        ThreadData td;

        td.frame = in;
        td.idx = n;

        ff_filter_execute(ctx, s->do_upmix_in, &td, NULL, nb_threads);
        ff_filter_execute(ctx, s->do_angle, NULL, NULL, nb_threads);
        ff_filter_execute(ctx, s->do_upmix, NULL, NULL, nb_threads);

        for (int ch = 0; ch < nb_out_channels; ch++)
            s->do_upmix_out(ctx, out, ch, n);
    }

    ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AUpmixContext *s = ctx->priv;

    if (s->upmix_uninit)
        s->upmix_uninit(ctx);
}

static const AVFilterPad upmix_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad upmix_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(AUpmixContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption upmix_options[] = {
    { "chl_out", "set output channel layout", OFFSET(out_ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="5.1"}, 0, 0, FLAGS },
    { "size", "set swift transform size", OFFSET(aswift_size), AV_OPT_TYPE_INT, {.i64=128}, 16, 4096, FLAGS },
    { "smooth", "set swift smooth value", OFFSET(smooth), AV_OPT_TYPE_FLOAT, {.dbl=0.001}, 0, 1, TFLAGS },
    { "gain", "set output gain value", OFFSET(gain), AV_OPT_TYPE_FLOAT, {.dbl=0.001}, 0, 32, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(upmix);

const FFFilter ff_af_upmix = {
    .p.name          = "upmix",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Upmixer."),
    .p.priv_class    = &upmix_class,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(AUpmixContext),
    .uninit          = uninit,
    FILTER_INPUTS(upmix_inputs),
    FILTER_OUTPUTS(upmix_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
