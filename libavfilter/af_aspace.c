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

#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/qsort.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"

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

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);

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

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioSpaceContext *s = ctx->priv;
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
    ret = ff_set_common_formats2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&outlayouts, &s->outlayout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(outlayouts, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&inlayouts, &inlayout);
    if (ret)
        return ret;

    return ff_channel_layouts_ref(inlayouts, &cfg_in[0]->channel_layouts);
}

typedef struct ThreadData {
    AVFrame *in, *out, *w;
} ThreadData;

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
        s->filter_channels = outlink->format == AV_SAMPLE_FMT_FLTP ? filter_channels_float : filter_channels_double;
        break;
    case P_SINGLE:
        s->filter_channels = filter_channels_float;
        break;
    case P_DOUBLE:
        s->filter_channels = filter_channels_double;
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
    int ret, nb_samples = in->nb_samples;
    ThreadData td;
    double scale;
    AVFrame *out;

    if (!s->w || s->w->nb_samples < nb_samples) {
        if (!s->w) {
            s->w = av_frame_alloc();
            if (!s->w) {
                av_frame_free(&in);
                return AVERROR(ENOMEM);
            }
        } else {
            av_frame_unref(s->w);
        }
        s->w->nb_samples = nb_samples;
        ret = ff_filter_get_buffer(ctx, s->w);
        if (ret < 0) {
            av_frame_free(&in);
            return ret;
        }
    }

    out = av_frame_alloc();
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    out->nb_samples = nb_samples;
    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        return ret;
    }

    av_frame_copy_props(out, in);

    s->source[0] = s->polar[2] * cos(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[1] = s->polar[2] * sin(s->polar[0] * M_PI / 180.0) * cos(s->polar[1] * M_PI / 180.0);
    s->source[2] = s->polar[2] * sin(s->polar[1] * M_PI / 180.0);

    scale = 1.0 / nb_samples;
    s->a = s->rolloff / (20.0 * log10(2.0));
    for (int n = 0; n < nb_samples; n++) {
        double p, mediand, drs, maxd, um, source[3];

        if (s->set && !memcmp(s->prev_source, s->source, sizeof(s->source))) {
            switch (outlink->format) {
            case AV_SAMPLE_FMT_FLTP:
                for (int ch = 0; ch < nb_speakers; ch++) {
                    Speaker *speaker = &s->speakers[ch];
                    const float gain = speaker->gain;

                    for (int m = 0; m < nb_samples; m++) {
                        float *w = (float *)s->w->extended_data[ch];

                        w[m] = gain;
                    }
                }
                break;
            case AV_SAMPLE_FMT_DBLP:
                for (int ch = 0; ch < nb_speakers; ch++) {
                    Speaker *speaker = &s->speakers[ch];
                    const double gain = speaker->gain;

                    for (int m = 0; m < nb_samples; m++) {
                        double *w = (double *)s->w->extended_data[ch];

                        w[m] = gain;
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

    td.out = out;
    td.in = in;
    td.w = s->w;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

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
    av_freep(&s->sorted_speakers);
    av_frame_free(&s->w);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const AudioSpaceContext *s_src = src->priv;
    AudioSpaceContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    memcpy(s_dst->polar, s_src->polar, sizeof(s_src->polar));
    s_dst->rolloff = s_src->rolloff;
    s_dst->blur = s_src->blur;

    memcpy(s_dst->prev_source, s_src->prev_source, sizeof(s_src->prev_source));
    memcpy(s_dst->reference, s_src->reference, sizeof(s_src->reference));
    memcpy(s_dst->source, s_src->source, sizeof(s_src->source));

    return 0;
}
#endif

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

const FFFilter ff_af_aspace = {
    .p.name          = "aspace",
    .p.description   = NULL_IF_CONFIG_SMALL("Arbitrary Distance Amplitude Panning."),
    .p.priv_class    = &aspace_class,
    .priv_size       = sizeof(AudioSpaceContext),
    .init            = init,
    .uninit          = uninit,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state  = transfer_state,
#endif
    FILTER_QUERY_FUNC2(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    .process_command = ff_filter_process_command,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_FRAME_THREADS,
};
