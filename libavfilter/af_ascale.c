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

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

#define MAX_STATES 4

#define IN 0
#define OUT 1
#define FIRST 0
#define LAST 1

typedef struct ChannelContext {
    AVTXContext *r2c[MAX_STATES], *c2r[MAX_STATES];
    av_tx_fn r2c_fn[MAX_STATES], c2r_fn[MAX_STATES];

    double state[2];

    int mode;
    int keep[2];
    int best_period;
    int best_max_period;
    double best_score;

    void *r_data[2];
    void *c_data[2];

    void *data[2];

    AVAudioFifo *in_fifo;
    AVAudioFifo *out_fifo;
} ChannelContext;

typedef struct AScaleContext {
    const AVClass *class;

    double tempo;
    int hz;
    int max_period;
    int max_size;

    int eof;
    int64_t pts[2];
    int flush[2];
    int nb_channels;
    ChannelContext *c;

    int (*init_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);
    void (*filter_channel)(AVFilterContext *ctx, const int ch);
    void (*correlate_stereo)(AVFilterContext *ctx, AVFrame *out);
    void (*decorrelate_stereo)(AVFilterContext *ctx, AVFrame *out);
} AScaleContext;

#define OFFSET(x) offsetof(AScaleContext, x)
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ascale_options[] = {
    { "tempo", "set the tempo", OFFSET(tempo), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.01, 100.0, TFLAGS },
    { NULL }
};

enum ScaleMode {
    COPY,
    EXPAND,
    COMPRESS,
};

AVFILTER_DEFINE_CLASS(ascale);

static int min_input_fifo_samples(AVFilterContext *ctx)
{
    AScaleContext *s = ctx->priv;
    int nb_samples = INT_MAX;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];

        nb_samples = FFMIN(av_audio_fifo_size(c->in_fifo), nb_samples);
    }

    return nb_samples;
}

static int min_output_fifo_samples(AVFilterContext *ctx)
{
    AScaleContext *s = ctx->priv;
    int nb_samples = INT_MAX;
    const int have_input = !s->eof || (min_input_fifo_samples(ctx) > 0);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        const int size = FFMAX(av_audio_fifo_size(c->out_fifo)-c->keep[OUT] * have_input, 0);

        nb_samples = FFMIN(size, nb_samples);
    }

    return nb_samples;
}

static void read_output_samples(AVFilterContext *ctx, AVFrame *out)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AScaleContext *s = ctx->priv;
    const int nb_samples = out->nb_samples;
    const int max_period = s->max_period;
    const int have_input = !s->eof || (min_input_fifo_samples(ctx) > 0);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        void *data[1] = { (void *)out->extended_data[ch] };
        int size;

        size = av_audio_fifo_peek_at(c->out_fifo, data, nb_samples, c->keep[OUT] * have_input);
        if (size > 0) {
            c->keep[OUT] = FFMIN(c->keep[OUT]+size, max_period);
            av_audio_fifo_drain(c->out_fifo, size);
        }
    }

    if (outlink->ch_layout.nb_channels == 2)
        s->decorrelate_stereo(ctx, out);
}

#define DEPTH 32
#include "ascale_template.c"

#undef DEPTH
#define DEPTH 64
#include "ascale_template.c"

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AScaleContext *s = ctx->priv;
    const int nb_channels = s->nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int stop = (nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < stop; ch++)
        s->filter_channel(ctx, ch);

    return 0;
}

static void filter_frame(AVFilterContext *ctx)
{
    AScaleContext *s = ctx->priv;

    ff_filter_execute(ctx, filter_channels, NULL, NULL,
                      FFMIN(s->nb_channels, ff_filter_get_nb_threads(ctx)));
}

static void peek_input_samples(AVFilterContext *ctx, AVFrame *in)
{
    AScaleContext *s = ctx->priv;
    const int nb_samples = in->nb_samples;

    if (s->pts[IN] == AV_NOPTS_VALUE)
        s->pts[IN] = s->pts[OUT] = in->pts;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        const float *dptr = (const float *)in->extended_data[ch];
        void *data[1] = { (void *)dptr };

        av_audio_fifo_peek_at(c->in_fifo, data, nb_samples, 0);
    }

    if (s->nb_channels == 2)
        s->decorrelate_stereo(ctx, in);
}

static int write_input_samples(AVFilterContext *ctx, AVFrame *in)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AScaleContext *s = ctx->priv;
    const double fs = F(1.0)/in->sample_rate;
    const int nb_samples = in->nb_samples;

    if (s->pts[IN] == AV_NOPTS_VALUE)
        s->pts[IN] = s->pts[OUT] = in->pts;

    if (outlink->ch_layout.nb_channels == 2) {
        int ret = ff_inlink_make_frame_writable(ctx->inputs[0], &in);

        if (ret < 0) {
            ff_graph_frame_free(ctx, &in);
            return ret;
        }

        s->correlate_stereo(ctx, in);
    }

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        const float *dptr = (const float *)in->extended_data[ch];
        void *data[1] = { (void *)dptr };

        av_audio_fifo_write(c->in_fifo, data, nb_samples);
        c->state[IN] += nb_samples * fs;
    }

    ff_graph_frame_free(ctx, &in);

    return 0;
}

static int output_frame(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AScaleContext *s = ctx->priv;
    const int nb_samples = min_output_fifo_samples(ctx);
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out)
        return AVERROR(ENOMEM);

    out->pts = s->pts[OUT];
    s->pts[OUT] += out->nb_samples;
    read_output_samples(ctx, out);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AScaleContext *s = ctx->priv;
    int status, ret;
    int64_t pts;
    AVFrame *in;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        av_log(ctx, AV_LOG_DEBUG, "[%d]: out: %d | in: %d\n", ch,
               av_audio_fifo_size(c->out_fifo),
               av_audio_fifo_size(c->in_fifo));
    }

    if (min_input_fifo_samples(ctx) < s->max_period*2) {
        ret = ff_inlink_consume_frame(inlink, &in);
        if (ret < 0)
            return ret;
        if (ret > 0) {
            ret = write_input_samples(ctx, in);
            if (ret < 0)
                return ret;
        }
    }

    if (min_input_fifo_samples(ctx) >= s->max_period && !s->flush[FIRST]) {
        s->flush[FIRST] = 1;

        if (s->tempo != 1.0) {
            AVFrame *out = ff_get_audio_buffer(outlink, s->max_period);
            if (!out)
                return AVERROR(ENOMEM);

            out->pts = s->pts[OUT];
            s->pts[OUT] += out->nb_samples;
            peek_input_samples(ctx, out);
            return ff_filter_frame(outlink, out);
        }
    }

    if (min_output_fifo_samples(ctx) > 0)
        return output_frame(ctx);

    if (min_input_fifo_samples(ctx) >= s->max_period*2)
        filter_frame(ctx);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts))
        s->eof = 1;

    if (s->eof) {
        while (min_input_fifo_samples(ctx) > 0) {
            filter_frame(ctx);
            if (min_output_fifo_samples(ctx) > 0)
                return output_frame(ctx);
            else
                ff_filter_set_ready(ctx, 100);
        }

        if (min_input_fifo_samples(ctx) <= 0) {
            while (min_output_fifo_samples(ctx) > 0)
                return output_frame(ctx);
        }

        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts[OUT]);
        return 0;
    } else if (min_input_fifo_samples(ctx) >= s->max_period*2) {
        ff_filter_set_ready(ctx, 100);
        return 0;
    } else {
        FF_FILTER_FORWARD_WANTED(outlink, inlink);
    }

    return FFERROR_NOT_READY;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AScaleContext *s = ctx->priv;

    s->hz = 5;
    s->pts[IN] = AV_NOPTS_VALUE;
    s->max_period = (inlink->sample_rate + s->hz-1) / s->hz;
    s->max_size = 1 << av_ceil_log2(s->max_period*2);
    s->max_period = s->max_size >> 1;
    s->nb_channels = inlink->ch_layout.nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channel = filter_channel_dblp;
        s->init_state = init_state_dblp;
        s->uninit_state = uninit_state_dblp;
        s->correlate_stereo = correlate_stereo_dblp;
        s->decorrelate_stereo = decorrelate_stereo_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channel = filter_channel_fltp;
        s->init_state = init_state_fltp;
        s->uninit_state = uninit_state_fltp;
        s->correlate_stereo = correlate_stereo_fltp;
        s->decorrelate_stereo = decorrelate_stereo_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AScaleContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_ascale = {
    .p.name          = "ascale",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Scale."),
    .p.priv_class    = &ascale_class,
    .priv_size       = sizeof(AScaleContext),
    .activate        = activate,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
