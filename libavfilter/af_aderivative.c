/*
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

#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct ADerivativeContext {
    const AVClass *class;
    AVFrame *prev;
    void (*filter)(void **dst, void **prv, const void **src,
                   const int nb_samples, const int ch);
} ADerivativeContext;

#define DERIVATIVE(name, type)                                          \
static void aderivative_## name ##p(void **d, void **p, const void **s, \
                                    const int nb_samples, const int ch) \
{                                                                       \
    const type *src = s[ch];                                            \
    type *dst = d[ch];                                                  \
    type *prvptr = p[ch];                                               \
    type prv = prvptr[0];                                               \
                                                                        \
    for (int n = 0; n < nb_samples; n++) {                              \
        const type current = src[n];                                    \
                                                                        \
        dst[n] = current - prv;                                         \
        prv = current;                                                  \
    }                                                                   \
                                                                        \
    prvptr[0] = prv;                                                    \
}

DERIVATIVE(flt, float)
DERIVATIVE(dbl, double)
DERIVATIVE(s16, int16_t)
DERIVATIVE(s32, int32_t)

#define INTEGRAL(name, type)                                          \
static void aintegral_## name ##p(void **d, void **p, const void **s, \
                                  const int nb_samples, const int ch) \
{                                                                     \
    const type *src = s[ch];                                          \
    type *dst = d[ch];                                                \
    type *prvptr = p[ch];                                             \
    type prv = prvptr[0];                                             \
                                                                      \
    for (int n = 0; n < nb_samples; n++) {                            \
        const type current = src[n];                                  \
                                                                      \
        dst[n] = current + prv;                                       \
        prv = dst[n];                                                 \
    }                                                                 \
                                                                      \
    prvptr[0] = prv;                                                  \
}

INTEGRAL(flt, float)
INTEGRAL(dbl, double)

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ADerivativeContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP: s->filter = aderivative_fltp; break;
    case AV_SAMPLE_FMT_DBLP: s->filter = aderivative_dblp; break;
    case AV_SAMPLE_FMT_S32P: s->filter = aderivative_s32p; break;
    case AV_SAMPLE_FMT_S16P: s->filter = aderivative_s16p; break;
    default: return AVERROR_BUG;
    }

    if (strcmp(ctx->filter->name, "aintegral"))
        return 0;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP: s->filter = aintegral_fltp; break;
    case AV_SAMPLE_FMT_DBLP: s->filter = aintegral_dblp; break;
    }

    return 0;
}

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ADerivativeContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->filter((void **)out->extended_data, (void **)s->prev->extended_data, (const void **)in->extended_data,
                  in->nb_samples, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    ADerivativeContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;

    if (ctx->is_disabled) {
        if (s->prev)
            av_samples_set_silence(s->prev->extended_data, 0, 1,
                                   s->prev->ch_layout.nb_channels,
                                   s->prev->format);

        return ff_filter_frame(outlink, in);
    }

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    if (!s->prev) {
        s->prev = ff_get_audio_buffer(inlink, 1);
        if (!s->prev) {
            av_frame_free(&in);
            av_frame_free(&out);
            return AVERROR(ENOMEM);
        }
    }

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ADerivativeContext *s = ctx->priv;

    av_frame_free(&s->prev);
}

static const AVFilterPad aderivative_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

static const AVOption aderivative_options[] = {
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(aderivative, "aderivative/aintegral", aderivative_options);

const FFFilter ff_af_aderivative = {
    .p.name        = "aderivative",
    .p.description = NULL_IF_CONFIG_SMALL("Compute derivative of input audio."),
    .p.priv_class  = &aderivative_class,
    .priv_size     = sizeof(ADerivativeContext),
    .uninit        = uninit,
    FILTER_INPUTS(aderivative_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_FLTP,
                      AV_SAMPLE_FMT_S32P, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};

const FFFilter ff_af_aintegral = {
    .p.name        = "aintegral",
    .p.description = NULL_IF_CONFIG_SMALL("Compute integral of input audio."),
    .p.priv_class  = &aderivative_class,
    .priv_size     = sizeof(ADerivativeContext),
    .uninit        = uninit,
    FILTER_INPUTS(aderivative_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
