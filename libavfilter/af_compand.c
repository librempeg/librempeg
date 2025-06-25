/*
 * Copyright (c) 1999 Chris Bagwell
 * Copyright (c) 1999 Nick Bailey
 * Copyright (c) 2007 Rob Sykes <robs@users.sourceforge.net>
 * Copyright (c) 2013 Paul B Mahol
 * Copyright (c) 2014 Andrew Kelley
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

/**
 * @file
 * audio compand filter
 */

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct CompandContext {
    const AVClass *class;

    int nb_segments;
    float *attacks;
    unsigned nb_attacks;
    float *decays;
    unsigned nb_decays;
    float *in_points;
    unsigned nb_in_points;
    float *out_points;
    unsigned nb_out_points;
    int sidechain;

    void *segments;
    void *channels;

    double in_min_log;
    double out_min_lin;
    double curve_dB;
    double gain_dB;
    double delay;
    AVFrame *delay_frame, *in_frame, *sort_frame, *in, *sc;
    int delay_samples;
    int64_t pts;

    int (*prepare)(AVFilterContext *ctx, AVFilterLink *outlink, const int reset);
    int (*compand)(AVFilterContext *ctx);
    int (*compand_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*delay_count)(AVFilterContext *ctx);
    int (*out_samples)(AVFilterContext *ctx);
    void (*drain)(AVFilterContext *ctx, AVFrame *frame);
} CompandContext;

#define OFFSET(x) offsetof(CompandContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_attacks = {.def="0.01",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_decays  = {.def="0.8",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_in_points  = {.def="-70|-60|1",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_out_points = {.def="-70|-20|0",.size_min=1,.sep='|'};

static const AVOption compand_options[] = {
    { "attacks", "set time over which increase of volume is determined", OFFSET(attacks), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_attacks }, 0, 10, AT },
    { "decays", "set time over which decrease of volume is determined", OFFSET(decays), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_decays }, 0, 10, AT },
    { "ipoints", "set input points of transfer function", OFFSET(in_points), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_in_points }, -900, 900, AT },
    { "opoints", "set output points of transfer function", OFFSET(out_points), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_out_points }, -900, 900, AT },
    { "soft-knee", "set soft-knee", OFFSET(curve_dB), AV_OPT_TYPE_DOUBLE, { .dbl = 0.01 }, 0.01, 900, AT },
    { "gain", "set output gain", OFFSET(gain_dB), AV_OPT_TYPE_DOUBLE, { .dbl = 0 }, -900, 900, AT },
    { "delay", "set delay for samples before sending them to volume adjuster", OFFSET(delay), AV_OPT_TYPE_DOUBLE, { .dbl = 0 }, 0, 2, A },
    { "sidechain", "enable sidechain input", OFFSET(sidechain), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(compand);

static av_cold int init(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = "sidechain";
        return ff_append_inpad(ctx, &pad);
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;

    av_freep(&s->channels);
    av_freep(&s->segments);
    av_frame_free(&s->in);
    av_frame_free(&s->sc);
    av_frame_free(&s->in_frame);
    av_frame_free(&s->sort_frame);
    av_frame_free(&s->delay_frame);
}

#define DEPTH 32
#include "compand_template.c"

#undef DEPTH
#define DEPTH 64
#include "compand_template.c"

static int compand_drain(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    CompandContext *s = ctx->priv;
    AVFrame *out;

    /* 2048 is to limit output frame size during drain */
    out = ff_get_audio_buffer(outlink, FFMIN(2048, s->delay_count(ctx)));
    if (!out)
        return AVERROR(ENOMEM);
    out->pts = s->pts;
    s->pts += out->nb_samples;

    s->drain(ctx, out);

    return ff_filter_frame(outlink, out);
}

static int compand_nodelay(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_samples = s->in->nb_samples;
    AVFrame *out;
    int ret;

    if (av_frame_is_writable(s->in)) {
        out = s->in;
    } else {
        out = ff_get_audio_buffer(outlink, nb_samples);
        if (!out) {
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return AVERROR(ENOMEM);
        }
        ret = av_frame_copy_props(out, s->in);
        if (ret < 0) {
            av_frame_free(&out);
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return ret;
        }
    }

    ff_filter_execute(ctx, s->compand_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (s->in != out)
        av_frame_free(&s->in);

    s->in = NULL;
    av_frame_free(&s->sc);

    return ff_filter_frame(outlink, out);
}

static int compand_delay(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    const int nb_samples = s->in->nb_samples;
    AVFrame *out;
    int ret;

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out) {
        av_frame_free(&s->in);
        av_frame_free(&s->sc);
        return AVERROR(ENOMEM);
    }
    ret = av_frame_copy_props(out, s->in);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&s->in);
        av_frame_free(&s->sc);
        return ret;
    }

    ff_filter_execute(ctx, s->compand_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->pts = out->pts + out->nb_samples;
    out->pts -= s->delay_samples;

    av_frame_free(&s->in);
    av_frame_free(&s->sc);

    if (s->delay_count(ctx) < s->delay_samples) {
        av_frame_free(&out);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    out->nb_samples = s->out_samples(ctx);

    return ff_filter_frame(outlink, out);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx  = outlink->src;
    CompandContext *s     = ctx->priv;
    const int sample_rate = outlink->sample_rate;
    const int channels    = outlink->ch_layout.nb_channels;

    if (s->nb_attacks > channels || s->nb_decays > channels)
        av_log(ctx, AV_LOG_WARNING,
                "Number of attacks/decays bigger than number of channels. Ignoring rest of entries.\n");

    uninit(ctx);

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->drain = drain_fltp;
        s->prepare = prepare_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->drain = drain_dblp;
        s->prepare = prepare_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->delay_samples = lrint(s->delay * sample_rate);
    if (s->delay_samples <= 0) {
        s->compand = compand_nodelay;
        switch (outlink->format) {
        case AV_SAMPLE_FMT_FLTP:
            s->compand_channels = compand_nodelay_channels_fltp;
            s->delay_count = delay_count_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->compand_channels = compand_nodelay_channels_dblp;
            s->delay_count = delay_count_dblp;
            break;
        default:
            return AVERROR_BUG;
        }
    } else {
        s->in_frame = ff_get_audio_buffer(outlink, s->delay_samples);
        s->sort_frame = ff_get_audio_buffer(outlink, s->delay_samples);
        s->delay_frame = ff_get_audio_buffer(outlink, s->delay_samples);
        if (!s->in_frame || !s->delay_frame || !s->sort_frame)
            return AVERROR(ENOMEM);

        s->compand = compand_delay;
        switch (outlink->format) {
        case AV_SAMPLE_FMT_FLTP:
            s->compand_channels = compand_delay_channels_fltp;
            s->delay_count = delay_count_fltp;
            s->out_samples = out_samples_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->compand_channels = compand_delay_channels_dblp;
            s->delay_count = delay_count_dblp;
            s->out_samples = out_samples_dblp;
            break;
        default:
            return AVERROR_BUG;
        }
    }

    return s->prepare(ctx, outlink, 1);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    CompandContext *s = ctx->priv;
    int64_t pts;
    int status;

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

        return s->compand(ctx);
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        while (s->delay_count(ctx) > 0) {
            int ret = compand_drain(outlink);
            if (ret < 0)
                return ret;
        }

        ff_outlink_set_status(outlink, status, pts);

        return 0;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    CompandContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->prepare(ctx, ctx->outputs[0], 0);
}

static const AVFilterPad compand_outputs[] = {
    {
        .name         = "default",
        .config_props = config_output,
        .type         = AVMEDIA_TYPE_AUDIO,
    },
};

const FFFilter ff_af_compand = {
    .p.name         = "compand",
    .p.description  = NULL_IF_CONFIG_SMALL(
            "Compress or expand audio dynamic range."),
    .p.priv_class   = &compand_class,
    .priv_size      = sizeof(CompandContext),
    .init           = init,
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(compand_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags        = AVFILTER_FLAG_DYNAMIC_INPUTS |
                      AVFILTER_FLAG_SLICE_THREADS |
                      AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .process_command = process_command,
};
