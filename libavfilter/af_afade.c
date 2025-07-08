/*
 * Copyright (c) 2013-2015 Paul B Mahol
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

/**
 * @file
 * fade audio filter
 */

#include "config_components.h"

#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioFadeContext {
    const AVClass *class;
    int type;
    int curve, curve2;
    int64_t nb_samples;
    int64_t start_sample;
    int64_t duration;
    int64_t start_time;
    double silence;
    double unity;
    int overlap;
    int status[2];
    int passthrough;
    int64_t pts;

    void (*fade_samples)(uint8_t **dst, uint8_t * const *src,
                         int nb_samples, int channels, int direction,
                         int64_t start, int64_t range, int curve,
                         double silence, double unity);
    void (*scale_samples)(uint8_t **dst, uint8_t * const *src,
                          int nb_samples, int channels, double unity);
    void (*crossfade_samples)(uint8_t **dst, uint8_t * const *cf0,
                              uint8_t * const *cf1,
                              int nb_samples, int channels,
                              int curve0, int curve1);
} AudioFadeContext;

enum CurveType { NONE = -1, TRI, QSIN, ESIN, HSIN, LOG, IPAR, QUA, CUB, SQU, CBR, PAR, EXP, IQSIN, IHSIN, DESE, DESI, LOSI, SINC, ISINC, QUAT, QUATR, QSIN2, HSIN2, NB_CURVES };

#define OFFSET(x) offsetof(AudioFadeContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };

#define DEPTH 16
#include "afade_template.c"

#undef DEPTH
#define DEPTH 31
#include "afade_template.c"

#undef DEPTH
#define DEPTH 32
#include "afade_template.c"

#undef DEPTH
#define DEPTH 64
#include "afade_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFadeContext *s  = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBL:  s->fade_samples = fade_samples_dbl;
                             s->scale_samples = scale_samples_dbl;
                             break;
    case AV_SAMPLE_FMT_DBLP: s->fade_samples = fade_samplesp_dbl;
                             s->scale_samples = scale_samplesp_dbl;
                             break;
    case AV_SAMPLE_FMT_FLT:  s->fade_samples = fade_samples_flt;
                             s->scale_samples = scale_samples_flt;
                             break;
    case AV_SAMPLE_FMT_FLTP: s->fade_samples = fade_samplesp_flt;
                             s->scale_samples = scale_samplesp_flt;
                             break;
    case AV_SAMPLE_FMT_S16:  s->fade_samples = fade_samples_s16;
                             s->scale_samples = scale_samples_s16;
                             break;
    case AV_SAMPLE_FMT_S16P: s->fade_samples = fade_samplesp_s16;
                             s->scale_samples = scale_samplesp_s16;
                             break;
    case AV_SAMPLE_FMT_S32:  s->fade_samples = fade_samples_s32;
                             s->scale_samples = scale_samples_s32;
                             break;
    case AV_SAMPLE_FMT_S32P: s->fade_samples = fade_samplesp_s32;
                             s->scale_samples = scale_samplesp_s32;
                             break;
    default: return AVERROR_BUG;
    }

    if (s->duration)
        s->nb_samples = av_rescale(s->duration, outlink->sample_rate, AV_TIME_BASE);
    s->duration = 0;
    if (s->start_time)
        s->start_sample = av_rescale(s->start_time, outlink->sample_rate, AV_TIME_BASE);
    s->start_time = 0;

    return 0;
}

#if CONFIG_AFADE_FILTER

static const AVOption afade_options[] = {
    { "type",         "set the fade direction",                      OFFSET(type),         AV_OPT_TYPE_INT,    {.i64 = 0    }, 0, 1, TFLAGS, .unit = "type" },
    { "t",            "set the fade direction",                      OFFSET(type),         AV_OPT_TYPE_INT,    {.i64 = 0    }, 0, 1, TFLAGS, .unit = "type" },
    { "in",           "fade-in",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = 0    }, 0, 0, TFLAGS, .unit = "type" },
    { "out",          "fade-out",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = 1    }, 0, 0, TFLAGS, .unit = "type" },
    { "start_sample", "set number of first sample to start fading",  OFFSET(start_sample), AV_OPT_TYPE_INT64,  {.i64 = 0    }, 0, INT64_MAX, TFLAGS },
    { "ss",           "set number of first sample to start fading",  OFFSET(start_sample), AV_OPT_TYPE_INT64,  {.i64 = 0    }, 0, INT64_MAX, TFLAGS },
    { "nb_samples",   "set number of samples for fade duration",     OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX, TFLAGS },
    { "ns",           "set number of samples for fade duration",     OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT64_MAX, TFLAGS },
    { "start_time",   "set time to start fading",                    OFFSET(start_time),   AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "st",           "set time to start fading",                    OFFSET(start_time),   AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "duration",     "set fade duration",                           OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "d",            "set fade duration",                           OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, INT64_MAX, TFLAGS },
    { "curve",        "set fade curve type",                         OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, TFLAGS, .unit = "curve" },
    { "c",            "set fade curve type",                         OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, TFLAGS, .unit = "curve" },
    { "nofade",       "no fade; keep audio as-is",                   0,                    AV_OPT_TYPE_CONST,  {.i64 = NONE }, 0, 0, TFLAGS, .unit = "curve" },
    { "tri",          "linear slope",                                0,                    AV_OPT_TYPE_CONST,  {.i64 = TRI  }, 0, 0, TFLAGS, .unit = "curve" },
    { "qsin",         "quarter of sine wave",                        0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "esin",         "exponential sine wave",                       0,                    AV_OPT_TYPE_CONST,  {.i64 = ESIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "hsin",         "half of sine wave",                           0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN }, 0, 0, TFLAGS, .unit = "curve" },
    { "log",          "logarithmic",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = LOG  }, 0, 0, TFLAGS, .unit = "curve" },
    { "ipar",         "inverted parabola",                           0,                    AV_OPT_TYPE_CONST,  {.i64 = IPAR }, 0, 0, TFLAGS, .unit = "curve" },
    { "qua",          "quadratic",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = QUA  }, 0, 0, TFLAGS, .unit = "curve" },
    { "cub",          "cubic",                                       0,                    AV_OPT_TYPE_CONST,  {.i64 = CUB  }, 0, 0, TFLAGS, .unit = "curve" },
    { "squ",          "square root",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = SQU  }, 0, 0, TFLAGS, .unit = "curve" },
    { "cbr",          "cubic root",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = CBR  }, 0, 0, TFLAGS, .unit = "curve" },
    { "par",          "parabola",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = PAR  }, 0, 0, TFLAGS, .unit = "curve" },
    { "exp",          "exponential",                                 0,                    AV_OPT_TYPE_CONST,  {.i64 = EXP  }, 0, 0, TFLAGS, .unit = "curve" },
    { "iqsin",        "inverted quarter of sine wave",               0,                    AV_OPT_TYPE_CONST,  {.i64 = IQSIN}, 0, 0, TFLAGS, .unit = "curve" },
    { "ihsin",        "inverted half of sine wave",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = IHSIN}, 0, 0, TFLAGS, .unit = "curve" },
    { "dese",         "double-exponential seat",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = DESE }, 0, 0, TFLAGS, .unit = "curve" },
    { "desi",         "double-exponential sigmoid",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = DESI }, 0, 0, TFLAGS, .unit = "curve" },
    { "losi",         "logistic sigmoid",                            0,                    AV_OPT_TYPE_CONST,  {.i64 = LOSI }, 0, 0, TFLAGS, .unit = "curve" },
    { "sinc",         "sine cardinal function",                      0,                    AV_OPT_TYPE_CONST,  {.i64 = SINC }, 0, 0, TFLAGS, .unit = "curve" },
    { "isinc",        "inverted sine cardinal function",             0,                    AV_OPT_TYPE_CONST,  {.i64 = ISINC}, 0, 0, TFLAGS, .unit = "curve" },
    { "quat",         "quartic",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = QUAT }, 0, 0, TFLAGS, .unit = "curve" },
    { "quatr",        "quartic root",                                0,                    AV_OPT_TYPE_CONST,  {.i64 = QUATR}, 0, 0, TFLAGS, .unit = "curve" },
    { "qsin2",        "squared quarter of sine wave",                0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN2}, 0, 0, TFLAGS, .unit = "curve" },
    { "hsin2",        "squared half of sine wave",                   0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN2}, 0, 0, TFLAGS, .unit = "curve" },
    { "silence",      "set the silence gain",                        OFFSET(silence),      AV_OPT_TYPE_DOUBLE, {.dbl = 0 },    0, 1, TFLAGS },
    { "unity",        "set the unity gain",                          OFFSET(unity),        AV_OPT_TYPE_DOUBLE, {.dbl = 1 },    0, 1, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(afade);

static av_cold int init(AVFilterContext *ctx)
{
    AudioFadeContext *s = ctx->priv;

    if (INT64_MAX - s->nb_samples < s->start_sample)
        return AVERROR(EINVAL);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AudioFadeContext *s     = inlink->dst->priv;
    AVFilterLink *outlink   = inlink->dst->outputs[0];
    int nb_samples          = buf->nb_samples;
    AVFrame *out_buf;
    int64_t cur_sample = av_rescale_q(buf->pts, inlink->time_base, (AVRational){1, inlink->sample_rate});

    if (s->unity == 1.0 &&
        ((!s->type && (s->start_sample + s->nb_samples < cur_sample)) ||
         ( s->type && (cur_sample + nb_samples < s->start_sample))))
        return ff_filter_frame(outlink, buf);

    if (av_frame_is_writable(buf)) {
        out_buf = buf;
    } else {
        out_buf = ff_get_audio_buffer(outlink, nb_samples);
        if (!out_buf)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out_buf, buf);
    }

    if ((!s->type && (cur_sample + nb_samples < s->start_sample)) ||
        ( s->type && (s->start_sample + s->nb_samples < cur_sample))) {
        if (s->silence == 0.) {
            av_samples_set_silence(out_buf->extended_data, 0, nb_samples,
                                   out_buf->ch_layout.nb_channels, out_buf->format);
        } else {
            s->scale_samples(out_buf->extended_data, buf->extended_data,
                             nb_samples, buf->ch_layout.nb_channels,
                             s->silence);
        }
    } else if (( s->type && (cur_sample + nb_samples < s->start_sample)) ||
               (!s->type && (s->start_sample + s->nb_samples < cur_sample))) {
        s->scale_samples(out_buf->extended_data, buf->extended_data,
                         nb_samples, buf->ch_layout.nb_channels,
                         s->unity);
    } else {
        int64_t start;

        if (!s->type)
            start = cur_sample - s->start_sample;
        else
            start = s->start_sample + s->nb_samples - cur_sample;

        s->fade_samples(out_buf->extended_data, buf->extended_data,
                        nb_samples, buf->ch_layout.nb_channels,
                        s->type ? -1 : 1, start,
                        s->nb_samples, s->curve, s->silence, s->unity);
    }

    if (buf != out_buf)
        av_frame_free(&buf);

    return ff_filter_frame(outlink, out_buf);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret = ff_filter_process_command(ctx, cmd, arg);

    if (ret < 0)
        return ret;

    return config_output(ctx->outputs[0]);
}

static const AVFilterPad afade_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad afade_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_afade = {
    .p.name        = "afade",
    .p.description = NULL_IF_CONFIG_SMALL("Fade in/out input audio."),
    .p.priv_class  = &afade_class,
    .priv_size     = sizeof(AudioFadeContext),
    .init          = init,
    FILTER_INPUTS(afade_inputs),
    FILTER_OUTPUTS(afade_outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};

#endif /* CONFIG_AFADE_FILTER */

#if CONFIG_ACROSSFADE_FILTER

static const AVOption acrossfade_options[] = {
    { "nb_samples",   "set number of samples for cross fade duration", OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT32_MAX/10, FLAGS },
    { "ns",           "set number of samples for cross fade duration", OFFSET(nb_samples),   AV_OPT_TYPE_INT64,  {.i64 = 44100}, 1, INT32_MAX/10, FLAGS },
    { "duration",     "set cross fade duration",                       OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, 60000000, FLAGS },
    { "d",            "set cross fade duration",                       OFFSET(duration),     AV_OPT_TYPE_DURATION, {.i64 = 0 },  0, 60000000, FLAGS },
    { "overlap",      "overlap 1st stream end with 2nd stream start",  OFFSET(overlap),      AV_OPT_TYPE_BOOL,   {.i64 = 1    }, 0,  1, FLAGS },
    { "o",            "overlap 1st stream end with 2nd stream start",  OFFSET(overlap),      AV_OPT_TYPE_BOOL,   {.i64 = 1    }, 0,  1, FLAGS },
    { "curve1",       "set fade curve type for 1st stream",            OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { "c1",           "set fade curve type for 1st stream",            OFFSET(curve),        AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    {     "nofade",   "no fade; keep audio as-is",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = NONE }, 0, 0, FLAGS, .unit = "curve" },
    {     "tri",      "linear slope",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = TRI  }, 0, 0, FLAGS, .unit = "curve" },
    {     "qsin",     "quarter of sine wave",                          0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "esin",     "exponential sine wave",                         0,                    AV_OPT_TYPE_CONST,  {.i64 = ESIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "hsin",     "half of sine wave",                             0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN }, 0, 0, FLAGS, .unit = "curve" },
    {     "log",      "logarithmic",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = LOG  }, 0, 0, FLAGS, .unit = "curve" },
    {     "ipar",     "inverted parabola",                             0,                    AV_OPT_TYPE_CONST,  {.i64 = IPAR }, 0, 0, FLAGS, .unit = "curve" },
    {     "qua",      "quadratic",                                     0,                    AV_OPT_TYPE_CONST,  {.i64 = QUA  }, 0, 0, FLAGS, .unit = "curve" },
    {     "cub",      "cubic",                                         0,                    AV_OPT_TYPE_CONST,  {.i64 = CUB  }, 0, 0, FLAGS, .unit = "curve" },
    {     "squ",      "square root",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = SQU  }, 0, 0, FLAGS, .unit = "curve" },
    {     "cbr",      "cubic root",                                    0,                    AV_OPT_TYPE_CONST,  {.i64 = CBR  }, 0, 0, FLAGS, .unit = "curve" },
    {     "par",      "parabola",                                      0,                    AV_OPT_TYPE_CONST,  {.i64 = PAR  }, 0, 0, FLAGS, .unit = "curve" },
    {     "exp",      "exponential",                                   0,                    AV_OPT_TYPE_CONST,  {.i64 = EXP  }, 0, 0, FLAGS, .unit = "curve" },
    {     "iqsin",    "inverted quarter of sine wave",                 0,                    AV_OPT_TYPE_CONST,  {.i64 = IQSIN}, 0, 0, FLAGS, .unit = "curve" },
    {     "ihsin",    "inverted half of sine wave",                    0,                    AV_OPT_TYPE_CONST,  {.i64 = IHSIN}, 0, 0, FLAGS, .unit = "curve" },
    {     "dese",     "double-exponential seat",                       0,                    AV_OPT_TYPE_CONST,  {.i64 = DESE }, 0, 0, FLAGS, .unit = "curve" },
    {     "desi",     "double-exponential sigmoid",                    0,                    AV_OPT_TYPE_CONST,  {.i64 = DESI }, 0, 0, FLAGS, .unit = "curve" },
    {     "losi",     "logistic sigmoid",                              0,                    AV_OPT_TYPE_CONST,  {.i64 = LOSI }, 0, 0, FLAGS, .unit = "curve" },
    {     "sinc",     "sine cardinal function",                        0,                    AV_OPT_TYPE_CONST,  {.i64 = SINC }, 0, 0, FLAGS, .unit = "curve" },
    {     "isinc",    "inverted sine cardinal function",               0,                    AV_OPT_TYPE_CONST,  {.i64 = ISINC}, 0, 0, FLAGS, .unit = "curve" },
    {     "quat",     "quartic",                                       0,                    AV_OPT_TYPE_CONST,  {.i64 = QUAT }, 0, 0, FLAGS, .unit = "curve" },
    {     "quatr",    "quartic root",                                  0,                    AV_OPT_TYPE_CONST,  {.i64 = QUATR}, 0, 0, FLAGS, .unit = "curve" },
    {     "qsin2",    "squared quarter of sine wave",                  0,                    AV_OPT_TYPE_CONST,  {.i64 = QSIN2}, 0, 0, FLAGS, .unit = "curve" },
    {     "hsin2",    "squared half of sine wave",                     0,                    AV_OPT_TYPE_CONST,  {.i64 = HSIN2}, 0, 0, FLAGS, .unit = "curve" },
    { "curve2",       "set fade curve type for 2nd stream",            OFFSET(curve2),       AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { "c2",           "set fade curve type for 2nd stream",            OFFSET(curve2),       AV_OPT_TYPE_INT,    {.i64 = TRI  }, NONE, NB_CURVES - 1, FLAGS, .unit = "curve" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acrossfade);

static int check_input(AVFilterLink *inlink)
{
    const int queued_samples = ff_inlink_queued_samples(inlink);

    return ff_inlink_check_available_samples(inlink, queued_samples + 1) == 1;
}

static int activate(AVFilterContext *ctx)
{
    AudioFadeContext *s   = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in = NULL, *out, *cf[2] = { NULL };
    int ret = 0, nb_samples, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (s->passthrough && s->status[0]) {
        ret = ff_inlink_consume_frame(ctx->inputs[1], &in);
        if (ret > 0) {
            in->pts = s->pts;
            s->pts += av_rescale_q(in->nb_samples,
                      (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
            return ff_filter_frame(outlink, in);
        } else if (ret < 0) {
            return ret;
        } else if (ff_inlink_acknowledge_status(ctx->inputs[1], &status, &pts)) {
            ff_outlink_set_status(outlink, status, pts);
            return 0;
        } else if (!ret) {
            if (ff_outlink_frame_wanted(outlink)) {
                ff_inlink_request_frame(ctx->inputs[1]);
                return 0;
            }
        }
    }

    nb_samples = ff_inlink_queued_samples(ctx->inputs[0]);
    if (nb_samples  > s->nb_samples) {
        nb_samples -= s->nb_samples;
        s->passthrough = 1;
        ret = ff_inlink_consume_samples(ctx->inputs[0], nb_samples, nb_samples, &in);
        if (ret < 0)
            return ret;
        in->pts = s->pts;
        s->pts += av_rescale_q(in->nb_samples,
            (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
        return ff_filter_frame(outlink, in);
    } else if (s->status[0] && nb_samples >= s->nb_samples &&
               ff_inlink_queued_samples(ctx->inputs[1]) >= s->nb_samples) {
        if (s->overlap) {
            out = ff_get_audio_buffer(outlink, s->nb_samples);
            if (!out)
                return AVERROR(ENOMEM);

            ret = ff_inlink_consume_samples(ctx->inputs[0], s->nb_samples, s->nb_samples, &cf[0]);
            if (ret < 0) {
                av_frame_free(&out);
                return ret;
            }

            ret = ff_inlink_consume_samples(ctx->inputs[1], s->nb_samples, s->nb_samples, &cf[1]);
            if (ret < 0) {
                av_frame_free(&out);
                return ret;
            }

            s->crossfade_samples(out->extended_data, cf[0]->extended_data,
                                 cf[1]->extended_data,
                                 s->nb_samples, out->ch_layout.nb_channels,
                                 s->curve, s->curve2);
            out->pts = s->pts;
            s->pts += av_rescale_q(s->nb_samples,
                (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
            s->passthrough = 1;
            av_frame_free(&cf[0]);
            av_frame_free(&cf[1]);
            return ff_filter_frame(outlink, out);
        } else {
            out = ff_get_audio_buffer(outlink, s->nb_samples);
            if (!out)
                return AVERROR(ENOMEM);

            ret = ff_inlink_consume_samples(ctx->inputs[0], s->nb_samples, s->nb_samples, &cf[0]);
            if (ret < 0) {
                av_frame_free(&out);
                return ret;
            }

            s->fade_samples(out->extended_data, cf[0]->extended_data, s->nb_samples,
                            outlink->ch_layout.nb_channels, -1, s->nb_samples - 1, s->nb_samples, s->curve, 0., 1.);
            out->pts = s->pts;
            s->pts += av_rescale_q(s->nb_samples,
                (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
            av_frame_free(&cf[0]);
            ret = ff_filter_frame(outlink, out);
            if (ret < 0)
                return ret;

            out = ff_get_audio_buffer(outlink, s->nb_samples);
            if (!out)
                return AVERROR(ENOMEM);

            ret = ff_inlink_consume_samples(ctx->inputs[1], s->nb_samples, s->nb_samples, &cf[1]);
            if (ret < 0) {
                av_frame_free(&out);
                return ret;
            }

            s->fade_samples(out->extended_data, cf[1]->extended_data, s->nb_samples,
                            outlink->ch_layout.nb_channels, 1, 0, s->nb_samples, s->curve2, 0., 1.);
            out->pts = s->pts;
            s->pts += av_rescale_q(s->nb_samples,
                (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
            s->passthrough = 1;
            av_frame_free(&cf[1]);
            return ff_filter_frame(outlink, out);
        }
    } else if (ff_outlink_frame_wanted(outlink)) {
        if (!s->status[0] && check_input(ctx->inputs[0]))
            s->status[0] = AVERROR_EOF;
        s->passthrough = !s->status[0];
        if (check_input(ctx->inputs[1])) {
            s->status[1] = AVERROR_EOF;
            ff_outlink_set_status(outlink, AVERROR_EOF, AV_NOPTS_VALUE);
            return 0;
        }
        if (!s->status[0])
            ff_inlink_request_frame(ctx->inputs[0]);
        else
            ff_inlink_request_frame(ctx->inputs[1]);
        return 0;
    }

    return ret;
}

static int acrossfade_config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFadeContext *s  = ctx->priv;

    outlink->time_base   = ctx->inputs[0]->time_base;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBL:  s->crossfade_samples = crossfade_samples_dbl;  break;
    case AV_SAMPLE_FMT_DBLP: s->crossfade_samples = crossfade_samplesp_dbl; break;
    case AV_SAMPLE_FMT_FLT:  s->crossfade_samples = crossfade_samples_flt;  break;
    case AV_SAMPLE_FMT_FLTP: s->crossfade_samples = crossfade_samplesp_flt; break;
    case AV_SAMPLE_FMT_S16:  s->crossfade_samples = crossfade_samples_s16;  break;
    case AV_SAMPLE_FMT_S16P: s->crossfade_samples = crossfade_samplesp_s16; break;
    case AV_SAMPLE_FMT_S32:  s->crossfade_samples = crossfade_samples_s32;  break;
    case AV_SAMPLE_FMT_S32P: s->crossfade_samples = crossfade_samplesp_s32; break;
    default: return AVERROR_BUG;
    }

    config_output(outlink);

    return 0;
}

static AVFrame *get_audio_buffer(AVFilterLink *inlink, int nb_samples)
{
    AVFilterContext *ctx = inlink->dst;
    AudioFadeContext *s = ctx->priv;

    return s->passthrough ?
        ff_null_get_audio_buffer   (inlink, nb_samples) :
        ff_default_get_audio_buffer(inlink, nb_samples);
}

static const AVFilterPad acrossfade_inputs[] = {
    {
        .name         = "crossfade0",
        .type         = AVMEDIA_TYPE_AUDIO,
        .get_buffer.audio = get_audio_buffer,
    },
    {
        .name         = "crossfade1",
        .type         = AVMEDIA_TYPE_AUDIO,
        .get_buffer.audio = get_audio_buffer,
    },
};

static const AVFilterPad acrossfade_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = acrossfade_config_output,
    },
};

const FFFilter ff_af_acrossfade = {
    .p.name        = "acrossfade",
    .p.description = NULL_IF_CONFIG_SMALL("Cross fade two input audio streams."),
    .p.priv_class  = &acrossfade_class,
    .priv_size     = sizeof(AudioFadeContext),
    .activate      = activate,
    FILTER_INPUTS(acrossfade_inputs),
    FILTER_OUTPUTS(acrossfade_outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
};

#endif /* CONFIG_ACROSSFADE_FILTER */
