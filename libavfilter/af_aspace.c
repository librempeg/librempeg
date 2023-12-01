/*
 * Copyright (c) 2023 Paul B Mahol
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

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/opt.h"
#include "libavutil/qsort.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"

enum PrecisionType {
    P_AUTO = -1,
    P_SINGLE,
    P_DOUBLE,
    NB_PTYPES,
};

typedef struct Speaker {
    double position[3];
    double distance;
    double b;
    double gain;
} Speaker;

typedef struct AudioSpaceContext {
    const AVClass *class;

    double polar[3];
    int precision;
    int set;
    AVChannelLayout outlayout;
    AVFrame *w;

    double a, k, dmax, rs;
    double blur;
    double radius;
    double rolloff;
    double source[3];
    double prev_source[3];
    double reference[3];
    Speaker *speakers;
    Speaker *sorted_speakers;

    void (*process)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, AVFrame *w);
    AVFloatDSPContext *fdsp;
} AudioSpaceContext;

#define OFFSET(x) offsetof(AudioSpaceContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aspace_options[] = {
    { "layout", "set the layout of output audio", OFFSET(outlayout), AV_OPT_TYPE_CHLAYOUT, {.str="stereo"}, 0, 0, AF},
    { "a", "set the azimuth of source audio", OFFSET(polar[0]), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -180, 180.0, AFT },
    { "e", "set the elevation of source audio", OFFSET(polar[1]), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -90.0, 90.0, AFT },
    { "r", "set the distance of source audio", OFFSET(polar[2]), AV_OPT_TYPE_DOUBLE, {.dbl=2.}, 0.0, DBL_MAX, AFT },
    { "b", "set the spatial blur factor", OFFSET(blur), AV_OPT_TYPE_DOUBLE, {.dbl=0.2}, 0.001, 1.0, AFT },
    { "o", "set the rolloff", OFFSET(rolloff), AV_OPT_TYPE_DOUBLE, {.dbl=18}, 0, 90, AFT },
    { "R", "set the distance of each speaker in regular layout", OFFSET(radius), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.1, 100, AF },
    { "precision", "processing precision", OFFSET(precision), AV_OPT_TYPE_INT, {.i64=P_AUTO}, P_AUTO, NB_PTYPES-1, AF, "pre"},
    {   "auto",   "auto",                             0, AV_OPT_TYPE_CONST, {.i64=P_AUTO}, 0, 0, AF, "pre"},
    {   "float",  "single floating-point precision",  0, AV_OPT_TYPE_CONST, {.i64=P_SINGLE}, 0, 0, AF, "pre"},
    {   "double", "double floating-point precision" , 0, AV_OPT_TYPE_CONST, {.i64=P_DOUBLE}, 0, 0, AF, "pre"},
    {NULL}
};

static int query_formats(AVFilterContext *ctx)
{
    AudioSpaceContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *outlayouts = NULL;
    AVFilterChannelLayouts *inlayouts = NULL;
    AVChannelLayout inlayout = AV_CHANNEL_LAYOUT_MONO;
    int ret = 0;

    if (s->precision == P_AUTO) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
        if (ret)
            return ret;
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    } else if (s->precision == P_SINGLE) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    } else if (s->precision == P_DOUBLE) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    }
    if (ret)
        return ret;
    ret = ff_set_common_formats(ctx, formats);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&outlayouts, &s->outlayout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(outlayouts, &ctx->outputs[0]->incfg.channel_layouts);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&inlayouts, &inlayout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(inlayouts, &ctx->inputs[0]->outcfg.channel_layouts);
    if (ret)
        return ret;

    return ff_set_common_all_samplerates(ctx);
}

#define DEPTH 32
#include "aspace_template.c"

#undef DEPTH
#define DEPTH 64
#include "aspace_template.c"

static double sqr(double x)
{
    return x * x;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioSpaceContext *s = ctx->priv;

    switch (s->precision) {
    case P_AUTO:
        s->process = outlink->format == AV_SAMPLE_FMT_FLTP ? process_float : process_double;
        break;
    case P_SINGLE:
        s->process = process_float;
        break;
    case P_DOUBLE:
        s->process = process_double;
        break;
    default: av_assert0(0);
    }

    if (!s->speakers) {
        s->speakers = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->speakers));
        if (!s->speakers)
            return AVERROR(ENOMEM);
    }

    if (!s->sorted_speakers) {
        s->sorted_speakers = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->sorted_speakers));
        if (!s->sorted_speakers)
            return AVERROR(ENOMEM);
    }

    s->source[0] = s->polar[2] * cos(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[1] = s->polar[2] * sin(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[2] = s->polar[2] * sin(s->polar[1] * M_PI / 180.0);

    memcpy(s->prev_source, s->source, sizeof(s->prev_source));

    s->reference[0] = 0.0;
    s->reference[1] = 0.0;
    s->reference[2] = 0.0;

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        Speaker *speaker = &s->speakers[ch];
        int chan = av_channel_layout_channel_from_index(&outlink->ch_layout, ch);
        double azim = 0, elev = 0;

        switch (chan) {
        case AV_CHAN_FRONT_LEFT:          azim =  30;      break;
        case AV_CHAN_FRONT_RIGHT:         azim = 330;      break;
        case AV_CHAN_FRONT_CENTER:        azim =   0;      break;
        case AV_CHAN_LOW_FREQUENCY:
        case AV_CHAN_LOW_FREQUENCY_2:     azim = 0;        break;
        case AV_CHAN_BACK_LEFT:           azim = 150;      break;
        case AV_CHAN_BACK_RIGHT:          azim = 210;      break;
        case AV_CHAN_BACK_CENTER:         azim = 180;      break;
        case AV_CHAN_SIDE_LEFT:           azim =  90;      break;
        case AV_CHAN_SIDE_RIGHT:          azim = 270;      break;
        case AV_CHAN_FRONT_LEFT_OF_CENTER:  azim =  15;    break;
        case AV_CHAN_FRONT_RIGHT_OF_CENTER: azim = 345;    break;
        case AV_CHAN_TOP_CENTER:          azim =   0;
                                          elev =  90;      break;
        case AV_CHAN_TOP_FRONT_LEFT:      azim =  30;
                                          elev =  45;      break;
        case AV_CHAN_TOP_FRONT_CENTER:    azim =   0;
                                          elev =  45;      break;
        case AV_CHAN_TOP_FRONT_RIGHT:     azim = 330;
                                          elev =  45;      break;
        case AV_CHAN_TOP_BACK_LEFT:       azim = 150;
                                          elev =  45;      break;
        case AV_CHAN_TOP_BACK_RIGHT:      azim = 210;
                                          elev =  45;      break;
        case AV_CHAN_TOP_BACK_CENTER:     azim = 180;
                                          elev =  45;      break;
        case AV_CHAN_WIDE_LEFT:           azim =  90;      break;
        case AV_CHAN_WIDE_RIGHT:          azim = 270;      break;
        case AV_CHAN_SURROUND_DIRECT_LEFT:  azim =  90;    break;
        case AV_CHAN_SURROUND_DIRECT_RIGHT: azim = 270;    break;
        case AV_CHAN_STEREO_LEFT:         azim =  90;      break;
        case AV_CHAN_STEREO_RIGHT:        azim = 270;      break;
        default:
            return AVERROR(EINVAL);
        }

        speaker->position[0] = s->radius * cos(azim * M_PI / 180.0) * cos(elev * M_PI / 180.0);
        speaker->position[1] = s->radius * sin(azim * M_PI / 180.0) * cos(elev * M_PI / 180.0);
        speaker->position[2] = s->radius * sin(elev * M_PI / 180.0);

        s->reference[0] += speaker->position[0];
        s->reference[1] += speaker->position[1];
        s->reference[2] += speaker->position[2];
    }

    s->reference[0] /= outlink->ch_layout.nb_channels;
    s->reference[1] /= outlink->ch_layout.nb_channels;
    s->reference[2] /= outlink->ch_layout.nb_channels;

    s->dmax = 0.0;
    s->rs = 0.0;
    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        Speaker *speaker = &s->speakers[ch];
        double distance;

        distance = sqrt(sqr(speaker->position[0] - s->reference[0]) +
                        sqr(speaker->position[1] - s->reference[1]) +
                        sqr(speaker->position[2] - s->reference[2]));

        speaker->b = 1.0;

        s->dmax = fmax(s->dmax, distance);
        s->rs += distance;
    }

    s->rs /= outlink->ch_layout.nb_channels;

    return 0;
}

static inline void iposition(double *out, const double *a,
                             const double *b, double f)
{
    out[0] = a[0] + (b[0] - a[0]) * f;
    out[1] = a[1] + (b[1] - a[1]) * f;
    out[2] = a[2] + (b[2] - a[2]) * f;
}

static int comp_distance(const Speaker *a, const Speaker *b)
{
    return FFDIFFSIGN(a->distance, b->distance);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioSpaceContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_speakers = outlink->ch_layout.nb_channels;
    double scale;
    AVFrame *out;

    if (!s->w || s->w->nb_samples < in->nb_samples) {
        av_frame_free(&s->w);
        s->w = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!s->w) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
    }

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->source[0] = s->polar[2] * cos(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[1] = s->polar[2] * sin(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[2] = s->polar[2] * sin(s->polar[1] * M_PI / 180.0);

    scale = 1.0 / out->nb_samples;
    s->a = s->rolloff / (20.0 * log10(2.0));
    for (int n = 0; n < out->nb_samples; n++) {
        double p, mediand, drs, maxd, um, source[3];

        if (s->set && !memcmp(s->prev_source, s->source, sizeof(s->source))) {
            switch (outlink->format) {
            case AV_SAMPLE_FMT_FLTP:
                for (int ch = 0; ch < nb_speakers; ch++) {
                    Speaker *speaker = &s->speakers[ch];
                    const float gain = speaker->gain;

                    for (int n = 0; n < out->nb_samples; n++) {
                        float *w = (float *)s->w->extended_data[ch];

                        w[n] = gain;
                    }
                }
                break;
            case AV_SAMPLE_FMT_DBLP:
                for (int ch = 0; ch < nb_speakers; ch++) {
                    Speaker *speaker = &s->speakers[ch];
                    const double gain = speaker->gain;

                    for (int n = 0; n < out->nb_samples; n++) {
                        double *w = (double *)s->w->extended_data[ch];

                        w[n] = gain;
                    }
                }
                break;
            default:
                av_assert0(0);
            }

            break;
        }

        iposition(source, s->prev_source, s->source, n * scale);

        maxd = 0.0;
        for (int ch = 0; ch < nb_speakers; ch++) {
            Speaker *speaker = &s->speakers[ch];

            speaker->distance = sqrt(sqr(speaker->position[0] - source[0]) +
                                     sqr(speaker->position[1] - source[1]) +
                                     sqr(speaker->position[2] - source[2]) + sqr(s->rs * s->blur));

            maxd = fmax(maxd, speaker->distance);
        }

        memcpy(s->sorted_speakers, s->speakers, sizeof(*s->speakers) * nb_speakers);
        AV_QSORT(s->sorted_speakers, nb_speakers, Speaker, comp_distance);
        if (nb_speakers & 1) {
            mediand = s->sorted_speakers[(nb_speakers+1)/2].distance;
        } else {
            mediand = (s->sorted_speakers[nb_speakers/2].distance +
                       s->sorted_speakers[nb_speakers/2-1].distance) * 0.5;
        }

        um = sqr(mediand - maxd) + 0.0000001;

        drs = sqrt(sqr(s->reference[0] - source[0]) +
                   sqr(s->reference[1] - source[1]) +
                   sqr(s->reference[2] - source[2]));

        p = fmin(s->dmax / drs, 1.0);
        s->k = 0.0;
        for (int ch = 0; ch < nb_speakers; ch++) {
            Speaker *speaker = &s->speakers[ch];
            double ui = sqr(speaker->distance - maxd) + 0.0000001;

            speaker->b = sqr((ui / um) * (1.0 / p - 1.0)) + 1.0;
            s->k += sqr(speaker->b) / pow(speaker->distance, 2.0 * s->a);
        }

        s->k = pow(p, 2.0 * s->a) / sqrt(s->k);

        switch (outlink->format) {
        case AV_SAMPLE_FMT_FLTP:
            for (int ch = 0; ch < nb_speakers; ch++) {
                float *w = (float *)s->w->extended_data[ch];
                Speaker *speaker = &s->speakers[ch];

                w[n] = speaker->b * s->k / pow(speaker->distance, s->a);
                speaker->gain = w[n];
            }
            break;
        case AV_SAMPLE_FMT_DBLP:
            for (int ch = 0; ch < nb_speakers; ch++) {
                double *w = (double *)s->w->extended_data[ch];
                Speaker *speaker = &s->speakers[ch];

                w[n] = speaker->b * s->k / pow(speaker->distance, s->a);
                speaker->gain = w[n];
            }
            break;
        default:
            av_assert0(0);
        }

        s->set = 1;
    }

    memcpy(s->prev_source, s->source, sizeof(s->prev_source));

    s->process(ctx, in, out, s->w);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioSpaceContext *s = ctx->priv;

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioSpaceContext *s = ctx->priv;

    av_freep(&s->fdsp);
    av_freep(&s->speakers);
    av_frame_free(&s->w);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

AVFILTER_DEFINE_CLASS(aspace);

const AVFilter ff_af_aspace = {
    .name            = "aspace",
    .description     = NULL_IF_CONFIG_SMALL("Arbitrary Distance Amplitude Panning"),
    .priv_size       = sizeof(AudioSpaceContext),
    .priv_class      = &aspace_class,
    .init            = init,
    .uninit          = uninit,
    FILTER_QUERY_FUNC(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    .process_command = ff_filter_process_command,
};
