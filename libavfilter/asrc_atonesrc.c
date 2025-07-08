/*
 * Copyright (c) 2022 Paul B Mahol
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
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioToneContext {
    const AVClass *class;

    double frequency;
    double amplitude;
    double phase;
    double fmf;
    double fmp;
    double fma;
    double amf;
    double amp;
    double ama;
    double pmf;
    double pmp;
    double pma;

    int samples_per_frame;
    int sample_rate;
    int64_t duration;

    int64_t pts;
} AudioToneContext;

#define OFFSET(x) offsetof(AudioToneContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption atonesrc_options[] = {
    { "frequency", "set the tone frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=440.0}, 0.1, INT_MAX, AFT },
    { "phase", "set the tone phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1, 1, AFT },
    { "amplitude", "set the tone amplitude", OFFSET(amplitude), AV_OPT_TYPE_DOUBLE, {.dbl=0.2}, 0, 1, AFT },
    { "fmf", "set the tone frequency modulation frequency", OFFSET(fmf), AV_OPT_TYPE_DOUBLE, {.dbl=0.3}, 0.001, INT_MAX, AFT },
    { "fmp", "set the tone frequency modulation phase", OFFSET(fmp), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1, 1, AFT },
    { "fma", "set the tone frequency modulation amplitude", OFFSET(fma), AV_OPT_TYPE_DOUBLE, {.dbl=200.0}, 0, INT_MAX, AFT },
    { "amf", "set the tone amplitude modulation frequency", OFFSET(amf), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.001, INT_MAX, AFT },
    { "amp", "set the tone amplitude modulation phase", OFFSET(amp), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1, 1, AFT },
    { "ama", "set the tone amplitude modulation amplitude", OFFSET(ama), AV_OPT_TYPE_DOUBLE, {.dbl=0.1}, 0, 1, AFT },
    { "pmf", "set the tone phase modulation frequency", OFFSET(pmf), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.001, INT_MAX, AFT },
    { "pmp", "set the tone phase modulation phase", OFFSET(pmp), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1, 1, AFT },
    { "pma", "set the tone phase modulation amplitude", OFFSET(pma), AV_OPT_TYPE_DOUBLE, {.dbl=0.1}, 0, 1, AFT },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, AF },
    { "duration", "set the duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, AF },
    { "samples_per_frame", "set the samples per frame", OFFSET(samples_per_frame), AV_OPT_TYPE_INT,  {.i64=1024}, 64, 65536, AFT },
    {NULL}
};

AVFILTER_DEFINE_CLASS(atonesrc);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AudioToneContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_DBLP,
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
    AudioToneContext *s = outlink->src->priv;

    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioToneContext *s = ctx->priv;
    int nb_samples = s->samples_per_frame;
    const double tfactor = av_q2d(outlink->time_base);
    double *samples;
    AVFrame *frame;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

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
    samples = (double *)frame->data[0];

    {
        double amplitude = s->amplitude;
        double frequency = s->frequency;
        double phase = s->phase*2.*M_PI;
        double fma = s->fma;
        double fmf = s->fmf;
        double fmp = s->fmp*2.*M_PI;
        double ama = s->ama;
        double amf = s->amf;
        double amp = s->amp*2.*M_PI;
        double pma = s->pma;
        double pmf = s->pmf;
        double pmp = s->pmp*2.*M_PI;

        for (int i = 0; i < nb_samples; i++) {
            double t = (s->pts + i) * tfactor;
            double a = amplitude * (1. + ama * sin(2. * M_PI * amf * t + amp));
            double p =     phase * (1. + pma * sin(2. * M_PI * pmf * t + pmp));
            double f = frequency *  t  + fma * sin(2. * M_PI * fmf * t + fmp);

            samples[i] = a * sin(2. * M_PI * f + p);
        }
    }

    frame->pts = s->pts;
    s->pts += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_props,
    },
};

const FFFilter ff_asrc_atonesrc = {
    .p.name          = "atonesrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Generate a tone audio signal."),
    .p.priv_class    = &atonesrc_class,
    .p.inputs        = NULL,
    .activate        = activate,
    .priv_size       = sizeof(AudioToneContext),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
