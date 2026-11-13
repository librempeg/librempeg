/*
 * Music Normalizer
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
 * Music Normalizer
 */

#include <float.h>

#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "filters.h"
#include "avfilter.h"

typedef struct MusicNormalizerContext {
    const AVClass *class;

    double D;
    double Lc;
    double sigma;
    double det_ms;
    double attack_ms;
    double release_ms;

    int nb_channels;

    void *st;

    int (*musicnorm_init)(AVFilterContext *ctx);
    void (*musicnorm_uninit)(AVFilterContext *ctx);
    int (*do_musicnorm)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
} MusicNormalizerContext;

#define OFFSET(x) offsetof(MusicNormalizerContext, x)
#define AF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
#define AFR AF|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption musicnorm_options[] = {
    { "max",  "set the max gain, dB", OFFSET(D),          AV_OPT_TYPE_DOUBLE, {.dbl=2},   0.0,  20.0, AFR },
    { "Lc",   "set the center level", OFFSET(Lc),         AV_OPT_TYPE_DOUBLE, {.dbl=0.1}, 0.0,   1.0, AFR },
    { "w",    "set the width",        OFFSET(sigma),      AV_OPT_TYPE_DOUBLE, {.dbl=2},   0.1,  10.0, AFR },
    { "detect",  "set the detection", OFFSET(det_ms),     AV_OPT_TYPE_DOUBLE, {.dbl=50},  0.1, 100.0, AFR },
    { "attack",  "set the attack",    OFFSET(attack_ms),  AV_OPT_TYPE_DOUBLE, {.dbl=10},  0.1, 100.0, AFR },
    { "release", "set the release",   OFFSET(release_ms), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 0.1, 200.0, AFR },
    { NULL }
};

AVFILTER_DEFINE_CLASS(musicnorm);

#define DEPTH 32
#include "musicnorm_template.c"

#undef DEPTH
#define DEPTH 64
#include "musicnorm_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MusicNormalizerContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_musicnorm = do_musicnorm_fltp;
        s->musicnorm_uninit = musicnorm_uninit_fltp;
        s->musicnorm_init = musicnorm_init_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_musicnorm = do_musicnorm_dblp;
        s->musicnorm_uninit = musicnorm_uninit_dblp;
        s->musicnorm_init = musicnorm_init_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->nb_channels = outlink->ch_layout.nb_channels;

    return s->musicnorm_init(ctx);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int musicnorm_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    MusicNormalizerContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_musicnorm(ctx, in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *outlink, AVFrame *in)
{
    AVFilterContext *ctx = outlink->src;
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
    ff_filter_execute(ctx, musicnorm_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (in != out)
        ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    int ret, status;
    int64_t pts;
    AVFrame *in;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(outlink, in);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);

        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MusicNormalizerContext *s = ctx->priv;

    if (s->musicnorm_uninit)
        s->musicnorm_uninit(ctx);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    MusicNormalizerContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->musicnorm_init(ctx);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_musicnorm = {
    .p.name        = "musicnorm",
    .p.description = NULL_IF_CONFIG_SMALL("Music Normalizer."),
    .p.priv_class  = &musicnorm_class,
    .priv_size     = sizeof(MusicNormalizerContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
