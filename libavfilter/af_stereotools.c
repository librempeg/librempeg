/*
 * Copyright (C) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen
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

typedef struct StereoToolsContext {
    const AVClass *class;

    int softclip;
    int mute_l;
    int mute_r;
    int phase_l;
    int phase_r;
    int mode;
    int bmode_in;
    int bmode_out;
    double slev;
    double sbal;
    double mlev;
    double mpan;
    double phase;
    double base;
    double delay;
    double balance_in;
    double balance_out;
    double phase_sin_coef;
    double phase_cos_coef;
    double sc_level;
    double inv_atan_shape;
    double level_in;
    double level_out;

    void *buffer;
    int length;
    int pos;

    void (*do_filter)(AVFilterContext *ctx, AVFrame *out, AVFrame *in);
} StereoToolsContext;

#define OFFSET(x) offsetof(StereoToolsContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption stereotools_options[] = {
    { "level_in",    "set level in",     OFFSET(level_in),    AV_OPT_TYPE_DOUBLE, {.dbl=1},   0.015625,  64, A },
    { "level_out",   "set level out",    OFFSET(level_out),   AV_OPT_TYPE_DOUBLE, {.dbl=1},   0.015625,  64, A },
    { "balance_in",  "set balance in",   OFFSET(balance_in),  AV_OPT_TYPE_DOUBLE, {.dbl=0},  -1,          1, A },
    { "balance_out", "set balance out",  OFFSET(balance_out), AV_OPT_TYPE_DOUBLE, {.dbl=0},  -1,          1, A },
    { "softclip",    "enable softclip",  OFFSET(softclip),    AV_OPT_TYPE_BOOL,   {.i64=0},   0,          1, A },
    { "mutel",       "mute L",           OFFSET(mute_l),      AV_OPT_TYPE_BOOL,   {.i64=0},   0,          1, A },
    { "muter",       "mute R",           OFFSET(mute_r),      AV_OPT_TYPE_BOOL,   {.i64=0},   0,          1, A },
    { "phasel",      "phase L",          OFFSET(phase_l),     AV_OPT_TYPE_BOOL,   {.i64=0},   0,          1, A },
    { "phaser",      "phase R",          OFFSET(phase_r),     AV_OPT_TYPE_BOOL,   {.i64=0},   0,          1, A },
    { "mode",        "set stereo mode",  OFFSET(mode),        AV_OPT_TYPE_INT,    {.i64=0},   0,         10, A, .unit = "mode" },
    {     "lr>lr",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=0},   0,          0, A, .unit = "mode" },
    {     "lr>ms",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=1},   0,          0, A, .unit = "mode" },
    {     "ms>lr",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=2},   0,          0, A, .unit = "mode" },
    {     "lr>ll",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=3},   0,          0, A, .unit = "mode" },
    {     "lr>rr",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=4},   0,          0, A, .unit = "mode" },
    {     "lr>l+r",  0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=5},   0,          0, A, .unit = "mode" },
    {     "lr>rl",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=6},   0,          0, A, .unit = "mode" },
    {     "ms>ll",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=7},   0,          0, A, .unit = "mode" },
    {     "ms>rr",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=8},   0,          0, A, .unit = "mode" },
    {     "ms>rl",   0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=9},   0,          0, A, .unit = "mode" },
    {     "lr>l-r",  0,                  0,                   AV_OPT_TYPE_CONST,  {.i64=10},  0,          0, A, .unit = "mode" },
    { "slev",        "set side level",   OFFSET(slev),        AV_OPT_TYPE_DOUBLE, {.dbl=1},   0.015625,  64, A },
    { "sbal",        "set side balance", OFFSET(sbal),        AV_OPT_TYPE_DOUBLE, {.dbl=0},  -1,          1, A },
    { "mlev",        "set middle level", OFFSET(mlev),        AV_OPT_TYPE_DOUBLE, {.dbl=1},   0.015625,  64, A },
    { "mpan",        "set middle pan",   OFFSET(mpan),        AV_OPT_TYPE_DOUBLE, {.dbl=0},  -1,          1, A },
    { "base",        "set stereo base",  OFFSET(base),        AV_OPT_TYPE_DOUBLE, {.dbl=0},  -1,          1, A },
    { "delay",       "set delay",        OFFSET(delay),       AV_OPT_TYPE_DOUBLE, {.dbl=0}, -20,         20, A },
    { "sclevel",     "set S/C level",    OFFSET(sc_level),    AV_OPT_TYPE_DOUBLE, {.dbl=1},   1,        100, A },
    { "phase",       "set stereo phase", OFFSET(phase),       AV_OPT_TYPE_DOUBLE, {.dbl=0},   0,        360, A },
    { "bmode_in",    "set balance in mode", OFFSET(bmode_in), AV_OPT_TYPE_INT,    {.i64=0},   0,          2, A, .unit = "bmode" },
    {     "balance",   0,                0,                   AV_OPT_TYPE_CONST,  {.i64=0},   0,          0, A, .unit = "bmode" },
    {     "amplitude", 0,                0,                   AV_OPT_TYPE_CONST,  {.i64=1},   0,          0, A, .unit = "bmode" },
    {     "power",     0,                0,                   AV_OPT_TYPE_CONST,  {.i64=2},   0,          0, A, .unit = "bmode" },
    { "bmode_out", "set balance out mode", OFFSET(bmode_out), AV_OPT_TYPE_INT,    {.i64=0},   0,          2, A, .unit = "bmode" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(stereotools);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLT,
        AV_SAMPLE_FMT_DBL,
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
#include "stereotools_template.c"

#undef DEPTH
#define DEPTH 64
#include "stereotools_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    StereoToolsContext *s = ctx->priv;

    s->length = 1 << av_ceil_log2((inlink->sample_rate + 9) / 10);
    if (!s->buffer)
        s->buffer = av_calloc(s->length, av_get_bytes_per_sample(inlink->format));
    if (!s->buffer)
        return AVERROR(ENOMEM);

    s->inv_atan_shape = 1.0 / atan(s->sc_level);
    s->phase_cos_coef = cos(s->phase / 180.0 * M_PI);
    s->phase_sin_coef = sin(s->phase / 180.0 * M_PI);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLT:
        s->do_filter = do_filter_flt;
        break;
    case AV_SAMPLE_FMT_DBL:
        s->do_filter = do_filter_dbl;
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
    StereoToolsContext *s = ctx->priv;
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

    s->do_filter(ctx, out, in);

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_input(ctx->inputs[0]);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    StereoToolsContext *s = ctx->priv;

    av_freep(&s->buffer);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_stereotools = {
    .p.name         = "stereotools",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply various stereo tools."),
    .p.priv_class   = &stereotools_class,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size      = sizeof(StereoToolsContext),
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = process_command,
};
