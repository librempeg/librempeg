/*
 * Copyright (c) 2013 Nicolas George
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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct OscContext {
    int64_t u, v;
    int64_t k1, k2;
} OscContext;

typedef struct SineContext {
    const AVClass *class;
    double frequency;
    double beep_factor;
    char *samples_per_frame;
    AVExpr *samples_per_frame_expr;
    int sample_rate;
    int64_t duration;
    int64_t pts;
    OscContext osc;
    unsigned beep_period;
    unsigned beep_index;
    unsigned beep_length;
    OscContext beep_osc;

    int (*init_state)(AVFilterContext *ctx);
    void (*output_samples)(AVFilterContext *ctx, AVFrame *frame);
} SineContext;

#define CONTEXT SineContext
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

#define OPT_GENERIC(name, field, def, min, max, descr, type, deffield, ...) \
    { name, descr, offsetof(CONTEXT, field), AV_OPT_TYPE_ ## type,          \
      { .deffield = def }, min, max, FLAGS, __VA_ARGS__ }

#define OPT_INT(name, field, def, min, max, descr, ...) \
    OPT_GENERIC(name, field, def, min, max, descr, INT, i64, __VA_ARGS__)

#define OPT_DBL(name, field, def, min, max, descr, ...) \
    OPT_GENERIC(name, field, def, min, max, descr, DOUBLE, dbl, __VA_ARGS__)

#define OPT_DUR(name, field, def, min, max, descr, ...) \
    OPT_GENERIC(name, field, def, min, max, descr, DURATION, str, __VA_ARGS__)

#define OPT_STR(name, field, def, min, max, descr, ...) \
    OPT_GENERIC(name, field, def, min, max, descr, STRING, str, __VA_ARGS__)

static const AVOption sine_options[] = {
    OPT_DBL("frequency",         frequency,            440, 0, DBL_MAX,   "set the sine frequency",),
    OPT_DBL("f",                 frequency,            440, 0, DBL_MAX,   "set the sine frequency",),
    OPT_DBL("beep_factor",       beep_factor,            0, 0, DBL_MAX,   "set the beep frequency factor",),
    OPT_DBL("b",                 beep_factor,            0, 0, DBL_MAX,   "set the beep frequency factor",),
    OPT_INT("sample_rate",       sample_rate,        44100, 1, INT_MAX,   "set the sample rate",),
    OPT_INT("r",                 sample_rate,        44100, 1, INT_MAX,   "set the sample rate",),
    OPT_DUR("duration",          duration,               0, 0, INT64_MAX, "set the audio duration",),
    OPT_DUR("d",                 duration,               0, 0, INT64_MAX, "set the audio duration",),
    OPT_STR("samples_per_frame", samples_per_frame, "1024", 0, 0,         "set the number of samples per frame",),
    {NULL}
};

AVFILTER_DEFINE_CLASS(sine);

static const char *const var_names[] = {
    "n",
    "pts",
    "t",
    "TB",
    NULL
};

enum {
    VAR_N,
    VAR_PTS,
    VAR_T,
    VAR_TB,
    VAR_VARS_NB
};

#define DEPTH 16
#include "sine_template.c"

#undef DEPTH
#define DEPTH 32
#include "sine_template.c"

static av_cold void uninit(AVFilterContext *ctx)
{
    SineContext *s = ctx->priv;

    av_expr_free(s->samples_per_frame_expr);
    s->samples_per_frame_expr = NULL;
}

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const SineContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S32,
        AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_NONE };
    int ret = ff_set_sample_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static av_cold int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    SineContext *s = ctx->priv;

    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);

    switch (outlink->format) {
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S16P:
        s->init_state = init_state_s16;
        s->output_samples = output_samples_s16;
        break;
    case AV_SAMPLE_FMT_S32:
    case AV_SAMPLE_FMT_S32P:
        s->init_state = init_state_s32;
        s->output_samples = output_samples_s32;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *outl = ff_filter_link(outlink);
    SineContext *s = ctx->priv;
    AVFrame *frame;
    double values[VAR_VARS_NB] = {
        [VAR_N]   = outl->frame_count_in,
        [VAR_PTS] = s->pts,
        [VAR_T]   = s->pts * av_q2d(outlink->time_base),
        [VAR_TB]  = av_q2d(outlink->time_base),
    };
    int nb_samples = lrint(av_expr_eval(s->samples_per_frame_expr, values, s));

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;
    if (nb_samples <= 0) {
        av_log(ctx, AV_LOG_WARNING, "nb samples expression evaluated to %d, "
               "defaulting to 1024\n", nb_samples);
        nb_samples = 1024;
    }

    if (s->duration) {
        nb_samples = FFMIN(nb_samples, s->duration - s->pts);
        av_assert1(nb_samples >= 0);
        if (!nb_samples) {
            ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
            return 0;
        }
    }

    if (!(frame = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);

    s->output_samples(ctx, frame);

    frame->pts = s->pts;
    s->pts += nb_samples;
    return ff_filter_frame(outlink, frame);
}

static const AVFilterPad sine_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_props,
    },
};

const FFFilter ff_asrc_sine = {
    .p.name        = "sine",
    .p.description = NULL_IF_CONFIG_SMALL("Generate sine wave audio signal."),
    .p.priv_class  = &sine_class,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(SineContext),
    FILTER_OUTPUTS(sine_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
