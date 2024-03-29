/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "internal.h"

typedef struct AQuadOscContext {
    const AVClass *class;

    double frequency;
    double old_frequency;
    double amplitude;
    double phase;
    double old_phase;
    double u, v;
    double k1, k2;

    int samples_per_frame;
    int sample_rate;
    int64_t duration;

    int64_t pts;
} AQuadOscContext;

#define OFFSET(x) offsetof(AQuadOscContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aquadosc_options[] = {
    { "frequency", "set the oscillator frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0.01}, 0.0, 0.5, AFT },
    { "amplitude", "set the oscillator amplitude", OFFSET(amplitude), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.0, 1.0, AFT },
    { "phase", "set the oscillator phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1.0, 1.0, AFT },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, AF },
    { "duration", "set the duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, AF },
    { "samples_per_frame", "set the samples per frame", OFFSET(samples_per_frame), AV_OPT_TYPE_INT,  {.i64=1024}, 64, 65536, AFT },
    {NULL}
};

AVFILTER_DEFINE_CLASS(aquadosc);

static av_cold int query_formats(AVFilterContext *ctx)
{
    AQuadOscContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_STEREO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP,
                                                       AV_SAMPLE_FMT_DBLP,
                                                       AV_SAMPLE_FMT_NONE };
    int ret = ff_set_common_formats_from_list(ctx, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list(ctx, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list(ctx, sample_rates);
}

static av_cold void init_state(AQuadOscContext *s)
{
    if (s->frequency != s->old_frequency) {
        double w0 = fmin(s->frequency, 0.49999) * 2.0*M_PI;

        s->k1 = tan(w0*0.5);
        s->k2 = sin(w0);
        s->old_frequency = s->frequency;
    }

    if (s->phase != s->old_phase) {
        double p0 = M_PI*s->phase;
        s->u = cos(p0);
        s->v = sin(p0);
        s->old_phase = s->phase;
    }
}

static av_cold int config_props(AVFilterLink *outlink)
{
    AQuadOscContext *s = outlink->src->priv;

    s->old_frequency = -1.0;
    s->old_phase = -2.0;

    init_state(s);

    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AQuadOscContext *s = ctx->priv;
    int nb_samples = s->samples_per_frame;
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

    switch (frame->format) {
    case AV_SAMPLE_FMT_DBLP:
        {
            double *real = (double *)frame->extended_data[0];
            double *imag = (double *)frame->extended_data[1];
            const double a = s->amplitude;
            const double k1 = s->k1;
            const double k2 = s->k2;
            double u = s->u;
            double v = s->v;
            double w;

            for (int i = 0; i < nb_samples; i++) {
                real[i] = u * a;
                imag[i] = v * a;
                w = u - k1 * v;
                v += k2 * w;
                u = w - k1 * v;
            }

            s->u = u;
            s->v = v;
        }
        break;
    case AV_SAMPLE_FMT_FLTP:
        {
            float *real = (float *)frame->extended_data[0];
            float *imag = (float *)frame->extended_data[1];
            const float a = s->amplitude;
            const float k1 = s->k1;
            const float k2 = s->k2;
            float u = s->u;
            float v = s->v;
            float w;

            for (int i = 0; i < nb_samples; i++) {
                real[i] = u * a;
                imag[i] = v * a;
                w = u - k1 * v;
                v += k2 * w;
                u = w - k1 * v;
            }

            s->u = u;
            s->v = v;
        }
        break;
    }

    frame->pts = s->pts;
    frame->duration = nb_samples;
    s->pts += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    AQuadOscContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    init_state(s);

    return 0;
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_props,
    },
};

const AVFilter ff_asrc_aquadosc = {
    .name            = "aquadosc",
    .description     = NULL_IF_CONFIG_SMALL("Generate Quadrature Oscillator samples."),
    .activate        = activate,
    .priv_size       = sizeof(AQuadOscContext),
    .inputs          = NULL,
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
    .priv_class      = &aquadosc_class,
    .process_command = process_command,
};
