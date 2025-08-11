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
    int32_t u, v;
    int32_t k1, k2;
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
#define FIXED(x) (lrint((x) * (1 << (DEPTH-1))))
#define MULT(x, y) (((x) * (y)) >> (DEPTH-1))

static av_cold int init(AVFilterContext *ctx)
{
    SineContext *sine = ctx->priv;
    double w0 = fmin(sine->frequency / (double)sine->sample_rate, 0.49999) * 2.0*M_PI;
    OscContext *osc = &sine->osc;
    int ret;

    osc->k1 = FIXED(tan(w0*0.5));
    osc->k2 = FIXED(sin(w0));
    osc->u  = FIXED(cos(0));
    osc->v  = FIXED(sin(0));

    if (sine->beep_factor) {
        OscContext *beep_osc = &sine->beep_osc;

        w0 = fmin(sine->frequency * sine->beep_factor / (double)sine->sample_rate, 0.49999) * 2.0*M_PI;
        beep_osc->k1 = FIXED(tan(w0*0.5));
        beep_osc->k2 = FIXED(sin(w0));
        beep_osc->u  = FIXED(cos(0));
        beep_osc->v  = FIXED(sin(0));

        sine->beep_period = sine->sample_rate;
        sine->beep_length = sine->beep_period / 25;
    }

    ret = av_expr_parse(&sine->samples_per_frame_expr,
                        sine->samples_per_frame, var_names,
                        NULL, NULL, NULL, NULL, 0, sine);
    if (ret < 0)
        return ret;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SineContext *sine = ctx->priv;

    av_expr_free(sine->samples_per_frame_expr);
    sine->samples_per_frame_expr = NULL;
}

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const SineContext *sine = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { sine->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16,
                                                       AV_SAMPLE_FMT_NONE };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static av_cold int config_props(AVFilterLink *outlink)
{
    SineContext *sine = outlink->src->priv;
    sine->duration = av_rescale(sine->duration, sine->sample_rate, AV_TIME_BASE);
    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *outl = ff_filter_link(outlink);
    SineContext *sine = ctx->priv;
    AVFrame *frame;
    double values[VAR_VARS_NB] = {
        [VAR_N]   = outl->frame_count_in,
        [VAR_PTS] = sine->pts,
        [VAR_T]   = sine->pts * av_q2d(outlink->time_base),
        [VAR_TB]  = av_q2d(outlink->time_base),
    };
    int nb_samples = lrint(av_expr_eval(sine->samples_per_frame_expr, values, sine));
    int16_t *samples;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;
    if (nb_samples <= 0) {
        av_log(ctx, AV_LOG_WARNING, "nb samples expression evaluated to %d, "
               "defaulting to 1024\n", nb_samples);
        nb_samples = 1024;
    }

    if (sine->duration) {
        nb_samples = FFMIN(nb_samples, sine->duration - sine->pts);
        av_assert1(nb_samples >= 0);
        if (!nb_samples) {
            ff_outlink_set_status(outlink, AVERROR_EOF, sine->pts);
            return 0;
        }
    }
    if (!(frame = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);
    samples = (int16_t *)frame->data[0];

    {
        const int32_t k1 = sine->osc.k1;
        const int32_t k2 = sine->osc.k2;
        int32_t u = sine->osc.u;
        int32_t v = sine->osc.v;
        int32_t w;

        for (int i = 0; i < nb_samples; i++) {
            samples[i] = u >> 3;
            w = u - MULT(k1, v);
            v += MULT(k2, w);
            u = w - MULT(k1, v);
        }

        sine->osc.u = u;
        sine->osc.v = v;

        if (sine->beep_length > 0) {
            OscContext *beep_osc = &sine->beep_osc;
            const int32_t k1 = beep_osc->k1;
            const int32_t k2 = beep_osc->k2;
            int32_t u = beep_osc->u;
            int32_t v = beep_osc->v;

            for (int i = 0; i < nb_samples; i++) {
                if (sine->beep_index < sine->beep_length) {
                    samples[i] += u >> 2;
                    w = u - MULT(k1, v);
                    v += MULT(k2, w);
                    u = w - MULT(k1, v);
                }

                if (++sine->beep_index == sine->beep_period)
                    sine->beep_index = 0;
            }

            beep_osc->u = u;
            beep_osc->v = v;
        }
    }

    frame->pts = sine->pts;
    sine->pts += nb_samples;
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
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(SineContext),
    FILTER_OUTPUTS(sine_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
