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

#include <float.h> /* DBL_MAX */

#include "libavutil/ffmath.h"
#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "filters.h"
#include "avfilter.h"

typedef struct PeakContext {
    void *iqueue;
    void *dqueue;
    void *queue;
    int nb_samples;
    int position;
    int front;
    int back;
} PeakContext;

typedef struct AudioTransientContext {
    const AVClass *class;
    int nb_channels;
    int sidechain;
    double arange;
    double rrange;
    double aratio;
    double rratio;
    double attack;
    double decay;
    double release;

    AVFrame *in, *sc;

    int a_size;
    int d_size;
    int r_size;

    PeakContext *ac;
    PeakContext *dc;
    PeakContext *rc;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioTransientContext;

#define OFFSET(x) offsetof(AudioTransientContext, x)
#define AF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
#define AFR AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption atransient_options[] = {
    { "attack",  "set transient attack duration",  OFFSET(attack),   AV_OPT_TYPE_DOUBLE, {.dbl=1},  0, 1000, AF },
    { "decay",   "set transient decay duration",   OFFSET(decay),    AV_OPT_TYPE_DOUBLE, {.dbl=2},  0, 1000, AF },
    { "release", "set transient release duration", OFFSET(release),  AV_OPT_TYPE_DOUBLE, {.dbl=5},  0, 1000, AF },
    { "aratio",  "set transient attack ratio",     OFFSET(aratio),   AV_OPT_TYPE_DOUBLE, {.dbl=0}, -9,    9, AFR},
    { "rratio",  "set transient release ratio",    OFFSET(rratio),   AV_OPT_TYPE_DOUBLE, {.dbl=0}, -9,    9, AFR},
    { "arange",  "set transient attack range",     OFFSET(arange),   AV_OPT_TYPE_DOUBLE, {.dbl=30}, 0, 1000, AFR},
    { "rrange",  "set transient release range",    OFFSET(rrange),   AV_OPT_TYPE_DOUBLE, {.dbl=30}, 0, 1000, AFR},
    { "sidechain","enable sidechain input",        OFFSET(sidechain),AV_OPT_TYPE_BOOL,   {.i64=0},  0,    1, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(atransient);

#define DEPTH 32
#include "atransient_template.c"

#undef DEPTH
#define DEPTH 64
#include "atransient_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioTransientContext *s = ctx->priv;
    size_t sample_size;

    s->ac = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->ac));
    s->dc = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->dc));
    s->rc = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->rc));
    if (!s->ac || !s->dc || !s->rc)
        return AVERROR(ENOMEM);
    s->nb_channels = inlink->ch_layout.nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        sample_size = sizeof(double);
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        sample_size = sizeof(float);
        break;
    default:
        return AVERROR_BUG;
    }

    s->a_size = FFMAX(lrint(s->attack  * inlink->sample_rate / 1000.), 1);
    s->d_size = FFMAX(lrint(s->decay   * inlink->sample_rate / 1000.), 1);
    s->r_size = FFMAX(lrint(s->release * inlink->sample_rate / 1000.), 1);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        PeakContext *ac = &s->ac[ch], *rc = &s->rc[ch], *dc = &s->dc[ch];

        ac->queue  = av_calloc(inlink->sample_rate, sample_size);
        ac->iqueue = av_calloc(inlink->sample_rate, sample_size);
        ac->dqueue = av_calloc(inlink->sample_rate, sample_size);
        dc->queue  = av_calloc(inlink->sample_rate, sample_size);
        dc->iqueue = av_calloc(inlink->sample_rate, sample_size);
        dc->dqueue = av_calloc(inlink->sample_rate, sample_size);
        rc->queue  = av_calloc(inlink->sample_rate, sample_size);
        rc->iqueue = av_calloc(inlink->sample_rate, sample_size);
        rc->dqueue = av_calloc(inlink->sample_rate, sample_size);
        if (!rc->queue || !rc->dqueue || !rc->iqueue ||
            !dc->queue || !dc->dqueue || !dc->iqueue ||
            !ac->queue || !ac->dqueue || !ac->iqueue)
            return AVERROR(ENOMEM);

        ac->nb_samples = s->a_size;
        dc->nb_samples = s->d_size;
        rc->nb_samples = s->r_size;
    }

    return 0;
}

static int filter_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioTransientContext *s = ctx->priv;
    AVFrame *out;

    if (av_frame_is_writable(s->in)) {
        out = s->in;
    } else {
        out = ff_get_audio_buffer(outlink, s->in->nb_samples);
        if (!out) {
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, s->in);
    }

    ff_filter_execute(ctx, s->filter_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (out != s->in)
        av_frame_free(&s->in);
    s->in = NULL;
    av_frame_free(&s->sc);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioTransientContext *s = ctx->priv;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->in) {
        int ret = ff_inlink_consume_frame(inlink, &s->in);
        if (ret < 0)
            return ret;
    }

    if (s->in) {
        if (s->sidechain && !s->sc) {
            AVFilterLink *sclink = ctx->inputs[1];
            int ret = ff_inlink_consume_samples(sclink, s->in->nb_samples,
                                                s->in->nb_samples, &s->sc);
            if (ret < 0)
                return ret;

            if (!ret) {
                FF_FILTER_FORWARD_STATUS(sclink, outlink);
                FF_FILTER_FORWARD_WANTED(outlink, sclink);
                return 0;
            }
        }

        return filter_frame(outlink);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioTransientContext *s = ctx->priv;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        PeakContext *ac = &s->ac[ch], *dc = &s->dc[ch], *rc = &s->rc[ch];

        av_freep(&ac->iqueue);
        av_freep(&ac->queue);
        av_freep(&ac->dqueue);
        av_freep(&dc->iqueue);
        av_freep(&dc->queue);
        av_freep(&dc->dqueue);
        av_freep(&rc->iqueue);
        av_freep(&rc->queue);
        av_freep(&rc->dqueue);
    }

    av_freep(&s->ac);
    av_freep(&s->dc);
    av_freep(&s->rc);

    av_frame_free(&s->in);
    av_frame_free(&s->sc);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioTransientContext *s = ctx->priv;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = "sidechain";
        return ff_append_inpad(ctx, &pad);
    }

    return 0;
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_af_atransient = {
    .p.name        = "atransient",
    .p.description = NULL_IF_CONFIG_SMALL("Audio Transient Shaper."),
    .p.priv_class  = &atransient_class,
    .priv_size     = sizeof(AudioTransientContext),
    .activate      = activate,
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_DYNAMIC_INPUTS |
                     AVFILTER_FLAG_SLICE_THREADS,
};
