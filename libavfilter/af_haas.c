/*
 * Copyright (c) 2001-2010 Vladimir Sadovnikov
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

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

#define MAX_HAAS_DELAY 40

typedef struct HaasContext {
    const AVClass *class;

    int par_m_source;
    double par_delay[2];
    int par_phase[2];
    int par_middle_phase;
    double par_side_gain;
    double par_gain[2];
    double par_balance[2];
    double level_in;
    double level_out;

    void *buffer;
    size_t buffer_size;
    size_t write_ptr;
    size_t delay[2];
    double balance_l[2];
    double balance_r[2];
    double phase[2];

    void (*do_haas)(AVFilterContext *ctx, AVFrame *out, AVFrame *in);
    int (*do_update)(AVFilterContext *ctx);
} HaasContext;

#define OFFSET(x) offsetof(HaasContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption haas_options[] = {
    { "level_in",      "set level in",      OFFSET(level_in),         AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0.015625,  64, A },
    { "level_out",     "set level out",     OFFSET(level_out),        AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0.015625,  64, A },
    { "side_gain",     "set side gain",     OFFSET(par_side_gain),    AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0.015625,  64, A },
    { "middle_source", "set middle source", OFFSET(par_m_source),     AV_OPT_TYPE_INT,     {.i64=2},        0,   3, A, .unit = "source" },
    {   "left",        0,                   0,                        AV_OPT_TYPE_CONST,   {.i64=0},        0,   0, A, .unit = "source" },
    {   "right",       0,                   0,                        AV_OPT_TYPE_CONST,   {.i64=1},        0,   0, A, .unit = "source" },
    {   "mid",         "L+R",               0,                        AV_OPT_TYPE_CONST,   {.i64=2},        0,   0, A, .unit = "source" },
    {   "side",        "L-R",               0,                        AV_OPT_TYPE_CONST,   {.i64=3},        0,   0, A, .unit = "source" },
    { "middle_phase",  "set middle phase",  OFFSET(par_middle_phase), AV_OPT_TYPE_BOOL,    {.i64=0},        0,   1, A },
    { "left_delay",    "set left delay",    OFFSET(par_delay[0]),     AV_OPT_TYPE_DOUBLE,  {.dbl=2.05},     0,  MAX_HAAS_DELAY, A },
    { "left_balance",  "set left balance",  OFFSET(par_balance[0]),   AV_OPT_TYPE_DOUBLE,  {.dbl=-1.0},    -1,   1, A },
    { "left_gain",     "set left gain",     OFFSET(par_gain[0]),      AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0.015625,  64, A },
    { "left_phase",    "set left phase",    OFFSET(par_phase[0]),     AV_OPT_TYPE_BOOL,    {.i64=0},        0,   1, A },
    { "right_delay",   "set right delay",   OFFSET(par_delay[1]),     AV_OPT_TYPE_DOUBLE,  {.dbl=2.12},     0,  MAX_HAAS_DELAY, A },
    { "right_balance", "set right balance", OFFSET(par_balance[1]),   AV_OPT_TYPE_DOUBLE,  {.dbl=1},       -1,   1, A },
    { "right_gain",    "set right gain",    OFFSET(par_gain[1]),      AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0.015625,  64, A },
    { "right_phase",   "set right phase",   OFFSET(par_phase[1]),     AV_OPT_TYPE_BOOL,    {.i64=1},        0,   1, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(haas);

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

#define DEPTH 32
#include "haas_template.c"

#undef DEPTH
#define DEPTH 64
#include "haas_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    const size_t min_buf_size = FFMAX(1, lrint(inlink->sample_rate * MAX_HAAS_DELAY * 0.001));
    const size_t new_buf_size = 1LL << av_ceil_log2(min_buf_size);
    HaasContext *s = ctx->priv;

    if (!s->buffer)
        s->buffer = av_calloc(new_buf_size, av_get_bytes_per_sample(inlink->format));
    if (!s->buffer)
        return AVERROR(ENOMEM);

    s->buffer_size = new_buf_size;
    s->write_ptr = 0;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_haas = do_haas_fltp;
        s->do_update = do_update_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_haas = do_haas_dblp;
        s->do_update = do_update_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->do_update(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    HaasContext *s = ctx->priv;
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

    s->do_haas(ctx, out, in);

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    HaasContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->do_update(ctx);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    HaasContext *s = ctx->priv;

    av_freep(&s->buffer);
    s->buffer_size = 0;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_haas = {
    .p.name         = "haas",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply Haas Stereo Enhancer."),
    .p.priv_class   = &haas_class,
    .priv_size      = sizeof(HaasContext),
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .process_command = process_command,
};
