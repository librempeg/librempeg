/*
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

#include "libavutil/ffmath.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct AudioDynamicSmoothContext {
    const AVClass *class;

    double sensitivity;
    double basefreq;

    AVFrame *coeffs;
} AudioDynamicSmoothContext;

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioDynamicSmoothContext *s = ctx->priv;

    s->coeffs = ff_get_audio_buffer(inlink, 3);
    if (!s->coeffs)
        return AVERROR(ENOMEM);

    return 0;
}

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioDynamicSmoothContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int is_disabled = ff_filter_disabled(ctx);
    const double sensitivity = s->sensitivity;
    const double wc = s->basefreq / in->sample_rate;
    const int nb_samples = out->nb_samples;

    for (int ch = start; ch < end; ch++) {
        const double *src = (const double *)in->extended_data[ch];
        double *dst = (double *)out->extended_data[ch];
        double *coeffs = (double *)s->coeffs->extended_data[ch];
        double low1 = coeffs[0];
        double low2 = coeffs[1];
        double inz  = coeffs[2];

        for (int n = 0; n < nb_samples; n++) {
            double low1z = low1;
            double low2z = low2;
            double bandz = low2z - low1z;
            double wd = wc + sensitivity * fabs(bandz);
            double g = fmin(1., wd * (5.9948827 + wd * (-11.969296 + wd * 15.959062)));

            low1 = low1z + g * (0.5 * (src[n] + inz)   - low1z);
            low2 = low2z + g * (0.5 * (low1   + low1z) - low2z);
            inz = src[n];
            dst[n] = is_disabled ? src[n] : low2;
        }

        coeffs[0] = low1;
        coeffs[1] = low2;
        coeffs[2] = inz;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
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

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (out != in)
        ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioDynamicSmoothContext *s = ctx->priv;

    av_frame_free(&s->coeffs);
}

#define OFFSET(x) offsetof(AudioDynamicSmoothContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption adynamicsmooth_options[] = {
    { "sensitivity",  "set smooth sensitivity",  OFFSET(sensitivity),  AV_OPT_TYPE_DOUBLE, {.dbl=2},     0, 1000000, FLAGS },
    { "basefreq",     "set base frequency",      OFFSET(basefreq),     AV_OPT_TYPE_DOUBLE, {.dbl=22050}, 2, 1000000, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adynamicsmooth);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_adynamicsmooth = {
    .p.name          = "adynamicsmooth",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Dynamic Smoothing of input audio."),
    .p.priv_class    = &adynamicsmooth_class,
    .priv_size       = sizeof(AudioDynamicSmoothContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
