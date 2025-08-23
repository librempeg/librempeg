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

/**
 * @file
 * Lookahead limiter filter
 */

#include <float.h>

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioLimiterContext {
    const AVClass *class;

    double limit;
    double look;
    double releaset;
    double attack;
    double release;
    int level;
    int link;
    int sidechain;
    int nb_channels;

    int l_size;
    int eof;
    int64_t eof_pts;

    void *st;

    AVFrame *in, *sc;

    int (*init_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);

    int (*filter_channels_link)(AVFilterContext *ctx, AVFrame *out);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioLimiterContext;

#define OFFSET(x) offsetof(AudioLimiterContext, x)
#define AFR AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AF AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption alimiter_options[] = {
    { "limit",     "set the limit",           OFFSET(limit),     AV_OPT_TYPE_DOUBLE, {.dbl=1},     0.0625, 1,    AFR},
    { "look",      "set the look-ahead time", OFFSET(look),      AV_OPT_TYPE_DOUBLE, {.dbl=0.001}, 0.001,  0.01, AF },
    { "release",   "set the release time",    OFFSET(releaset),  AV_OPT_TYPE_DOUBLE, {.dbl=0.1},   0.01,   1,    AFR},
    { "level",     "enable leveling",         OFFSET(level),     AV_OPT_TYPE_BOOL,   {.i64=1},     0,      1,    AFR},
    { "link",      "enable channels linking", OFFSET(link),      AV_OPT_TYPE_BOOL,   {.i64=1},     0,      1,    AFR},
    { "sidechain", "enable sidechain input",  OFFSET(sidechain), AV_OPT_TYPE_BOOL,   {.i64=0},     0,      1,    AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(alimiter);

static int filter_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioLimiterContext *s = ctx->priv;
    AVFrame *out;
    int ret;

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

    if (s->link) {
        s->filter_channels_link(ctx, out);
    } else {
        ff_filter_execute(ctx, s->filter_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels,
                                ff_filter_get_nb_threads(ctx)));
    }

    if (out != s->in)
        ff_graph_frame_free(ctx, &s->in);
    s->in = NULL;
    ff_graph_frame_free(ctx, &s->sc);
    out->pts -= av_rescale_q(s->l_size, av_make_q(1, outlink->sample_rate), outlink->time_base);
    ret = ff_filter_frame(outlink, out);
    if (s->eof)
        ff_outlink_set_status(outlink, AVERROR_EOF, s->eof_pts);
    return ret;
}

#define DEPTH 32
#include "alimiter_template.c"

#undef DEPTH
#define DEPTH 64
#include "alimiter_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioLimiterContext *s = ctx->priv;

    s->nb_channels = outlink->ch_layout.nb_channels;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels_link = filter_channels_link_dblp;
        s->filter_channels = filter_channels_dblp;
        s->init_state = init_state_dblp;
        s->uninit_state = uninit_state_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels_link = filter_channels_link_fltp;
        s->filter_channels = filter_channels_fltp;
        s->init_state = init_state_fltp;
        s->uninit_state = uninit_state_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->l_size = FFMAX(lrint(s->look * outlink->sample_rate), 1);
    s->attack = 1.0 / s->l_size;

    return s->init_state(ctx);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioLimiterContext *s = ctx->priv;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = "sidechain";
        return ff_append_inpad(ctx, &pad);
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioLimiterContext *s = ctx->priv;
    int status;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->in) {
        int ret = ff_inlink_consume_frame(inlink, &s->in);
        if (ret < 0)
            return ret;

        if (s->eof && ret == 0) {
            s->in = ff_get_audio_buffer(outlink, s->l_size);
            if (!s->in)
                return AVERROR(ENOMEM);

            s->in->pts = s->eof_pts;
            s->in->pts += av_rescale_q(s->l_size, av_make_q(1, inlink->sample_rate), inlink->time_base);
        }
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

        s->release = 1.0 / FFMAX(lrint(s->releaset * inlink->sample_rate), 1);

        return filter_frame(outlink);
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &s->eof_pts)) {
        if (status == AVERROR_EOF) {
            s->eof = 1;
            ff_filter_set_ready(ctx, 100);
            return 0;
        }
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioLimiterContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);

    av_frame_free(&s->in);
    av_frame_free(&s->sc);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_alimiter = {
    .p.name         = "alimiter",
    .p.description  = NULL_IF_CONFIG_SMALL("Audio lookahead limiter."),
    .p.priv_class   = &alimiter_class,
    .priv_size      = sizeof(AudioLimiterContext),
    .init           = init,
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_SLICE_THREADS |
                      AVFILTER_FLAG_DYNAMIC_INPUTS,
};
