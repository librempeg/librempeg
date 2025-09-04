/*
 * Copyright (c) 2021 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"

enum FilterType {
    ASDR,
    APSNR,
    ASISDR,
    ANRMSE,
    AMAE,
    AMDA,
    AIDENTITY,
    AMAPE,
    AMSE,
};

typedef struct ChanStats {
    double u;
    double v;
    double uv;
    uint64_t cnt;
} ChanStats;

typedef struct AudioSDRContext {
    const AVClass *class;

    double *export;
    unsigned nb_export;

    int filter_type;
    int channels;
    uint64_t nb_samples;

    ChanStats *chs;

    AVFrame *cache[2];

    int (*filter)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioSDRContext;

#define DEPTH 32
#include "asdr_template.c"

#undef DEPTH
#define DEPTH 64
#include "asdr_template.c"

static void get_score(AVFilterContext *ctx)
{
    AudioSDRContext *s = ctx->priv;

    for (int ch = 0; ch < s->channels; ch++) {
        switch (s->filter_type) {
        case ASDR:
            s->export[ch] = 10. * log10(s->chs[ch].u / s->chs[ch].uv);
            break;
        case ASISDR:
            {
                double scale = s->chs[ch].uv / s->chs[ch].v;
                double sisdr = scale * scale * s->chs[ch].v / fmax(0., s->chs[ch].u + scale*scale*s->chs[ch].v - 2.0*scale*s->chs[ch].uv);

                s->export[ch] = 10. * log10(sisdr);
            }
            break;
        case ANRMSE:
            s->export[ch] = -10. * log10(sqrt(s->chs[ch].uv / s->chs[ch].u));
            break;
        case AMAE:
        case AMAPE:
        case AMSE:
            s->export[ch] = -10. * log10(s->chs[ch].uv / s->nb_samples);
            break;
        case AMDA:
        case AIDENTITY:
            s->export[ch] = 10. * log10((double)s->nb_samples / (s->nb_samples - s->chs[ch].cnt));
            break;
        case APSNR:
            s->export[ch] = s->chs[ch].uv > 0.0 ? 10. * log10(s->chs[ch].u / sqrt(s->chs[ch].uv / s->nb_samples)) : INFINITY;
            break;
        default:
            break;
        }
    }
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSDRContext *s = ctx->priv;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->cache[0]) {
        int ret = ff_inlink_consume_frame(ctx->inputs[0], &s->cache[0]);
        if (ret < 0)
            return ret;
    }

    if (s->cache[0] && !s->cache[1]) {
        int ret = ff_inlink_consume_samples(ctx->inputs[1],
                                            s->cache[0]->nb_samples,
                                            s->cache[0]->nb_samples,
                                            &s->cache[1]);
        if (ret < 0)
            return ret;
    }

    if (s->cache[0] && s->cache[1]) {
        AVFrame *out;

        if (!ff_filter_disabled(ctx))
            ff_filter_execute(ctx, s->filter, NULL, NULL,
                              FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        ff_graph_frame_free(ctx, &s->cache[1]);
        out = s->cache[0];

        s->nb_samples += s->cache[0]->nb_samples;
        s->cache[0] = NULL;

        get_score(ctx);

        return ff_filter_frame(outlink, out);
    }

    for (int i = 0; i < 2; i++) {
        int64_t pts;
        int status;

        if (ff_inlink_queued_samples(ctx->inputs[i]) || s->cache[i])
            continue;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            ff_outlink_set_status(outlink, status, pts);
            return 0;
        }
    }

    if (ff_outlink_frame_wanted(outlink)) {
        for (int i = 0; i < 2; i++) {
            if (s->cache[i])
                continue;
            ff_inlink_request_frame(ctx->inputs[i]);
            return 0;
        }
    }

    return FFERROR_NOT_READY;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioSDRContext *s = ctx->priv;

    s->channels = inlink->ch_layout.nb_channels;

    switch (s->filter_type) {
    case ASDR:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? sdr_fltp : sdr_dblp;
        break;
    case ASISDR:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? sisdr_fltp : sisdr_dblp;
        break;
    case ANRMSE:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? nrmse_fltp : nrmse_dblp;
        break;
    case AMAE:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? mae_fltp : mae_dblp;
        break;
    case AMDA:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? mda_fltp : mda_dblp;
        break;
    case AIDENTITY:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? identity_fltp : identity_dblp;
        break;
    case APSNR:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? psnr_fltp : psnr_dblp;
        break;
    case AMAPE:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? mape_fltp : mape_dblp;
        break;
    case AMSE:
        s->filter = inlink->format == AV_SAMPLE_FMT_FLTP ? mse_fltp : mse_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->chs  = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->chs));
    if (!s->chs)
        return AVERROR(ENOMEM);

    s->export = av_calloc(s->channels, sizeof(*s->export));
    if (!s->export)
        return AVERROR(ENOMEM);
    s->nb_export = s->channels;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioSDRContext *s = ctx->priv;

    for (int ch = 0; ch < s->channels; ch++)
        av_log(ctx, AV_LOG_INFO, "%s ch%d: %g dB\n", ctx->filter->name, ch, s->export[ch]);

    av_frame_free(&s->cache[0]);
    av_frame_free(&s->cache[1]);

    av_freep(&s->export);
    av_freep(&s->chs);
}

static const AVFilterPad inputs[] = {
    {
        .name = "input0",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "input1",
        .type = AVMEDIA_TYPE_AUDIO,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(AudioSDRContext, x)
#define AR AV_OPT_TYPE_FLAG_ARRAY
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_EXPORT|AV_OPT_FLAG_READONLY
static const AVOptionArrayDef def_export  = {.def="0",.size_min=1,.sep=' '};

#define DEFINE_AAA_FILTER_2(name_, short_name_, description_, priv_class_) \
static const AVOption short_name_##_options[] = {               \
    {""#short_name_"", NULL, OFFSET(export), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_export}, -INFINITY, INFINITY, FLAGS},\
    {NULL}                                                      \
};                                                              \
                                                                \
AVFILTER_DEFINE_CLASS(short_name_);                             \
static av_cold int short_name_##_init(AVFilterContext *ctx)     \
{                                                               \
    AudioSDRContext *s = ctx->priv;                             \
    s->filter_type = name_;                                     \
    return 0;                                                   \
}                                                               \
                                                         \
const FFFilter ff_af_##short_name_ = {                   \
    .p.name        = ""#short_name_"",                   \
    .p.description = NULL_IF_CONFIG_SMALL(description_), \
    .p.priv_class  = &priv_class_##_class,               \
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY |       \
                     AVFILTER_FLAG_SLICE_THREADS |       \
                     AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL, \
    .priv_size     = sizeof(AudioSDRContext),            \
    .init          = short_name_##_init,                 \
    .activate      = activate,                           \
    .uninit        = uninit,                             \
    FILTER_INPUTS(inputs),                               \
    FILTER_OUTPUTS(outputs),                             \
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP), \
}

#define DEFINE_AAA_FILTER(name, short_name, description) \
    DEFINE_AAA_FILTER_2(name, short_name, description, short_name)

DEFINE_AAA_FILTER(ASDR,      asdr,      "Measure Audio Signal-to-Distortion Ratio.");
DEFINE_AAA_FILTER(APSNR,     apsnr,     "Measure Audio Peak Signal-to-Noise Ratio.");
DEFINE_AAA_FILTER(ASISDR,    asisdr,    "Measure Audio Scale-Invariant Signal-to-Distortion Ratio.");
DEFINE_AAA_FILTER(ANRMSE,    anrmse,    "Measure Audio Normalized Root Mean Square Error.");
DEFINE_AAA_FILTER(AMAE,      amae,      "Measure Audio Mean Absolute Error.");
DEFINE_AAA_FILTER(AMDA,      amda,      "Measure Audio Mean Directional Accuracy.");
DEFINE_AAA_FILTER(AMSE,      amse,      "Measure Audio Mean Squared Error.");
DEFINE_AAA_FILTER(AIDENTITY, aidentity, "Measure Identity between two audio streams.");
DEFINE_AAA_FILTER(AMAPE,     amape,     "Measure Audio Mean Absolute Percentage Error.");
