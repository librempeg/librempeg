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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

#define MAX_DELAY 33

typedef struct AudioFDelayContext {
    const AVClass *class;
    double *delays_opt;
    unsigned nb_delays;
    size_t max_delay;
    int64_t next_pts;
    int nb_channels;
    int eof;

    void *st;

    int (*init_state)(AVFilterContext *ctx, void **state,
                      const double *delays, const int nb_delays,
                      const int max_delay, const int nb_channels);
    int (*update_state)(AVFilterContext *ctx, void *state,
                        const double *delays, const int nb_delays,
                        const int max_delay,
                        const int nb_channels, const int reset);
    void (*uninit_state)(AVFilterContext *ctx, void **state, const int nb_channels);

    void (*filter_channel)(AVFilterContext *ctx, void *state, const int nb_samples,
                           const uint8_t *src, uint8_t *dst, const int ch);
} AudioFDelayContext;

#define OFFSET(x) offsetof(AudioFDelayContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_delays = {.def="0.5",.size_min=1,.sep=' '};

static const AVOption afdelay_options[] = {
    { "delays", "set the list of delays for each channel", OFFSET(delays_opt), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_delays}, 0.0, MAX_DELAY, A},
    { NULL }
};

AVFILTER_DEFINE_CLASS(afdelay);

#define DEPTH 32
#include "afdelay_template.c"

#undef DEPTH
#define DEPTH 64
#include "afdelay_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioFDelayContext *s = ctx->priv;

    s->max_delay = 0;
    for (int ch = 0; ch < s->nb_delays; ch++) {
        size_t delay = lrint(ceil(s->delays_opt[ch]));

        s->max_delay = FFMAX(s->max_delay, delay);
    }
    s->max_delay++;

    s->nb_channels = inlink->ch_layout.nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channel = afdelay_channel_dblp;
        s->init_state = init_afdelay_dblp;
        s->update_state = update_afdelay_dblp;
        s->uninit_state = uninit_afdelay_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channel = afdelay_channel_fltp;
        s->init_state = init_afdelay_fltp;
        s->update_state = update_afdelay_fltp;
        s->uninit_state = uninit_afdelay_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx, &s->st, s->delays_opt, s->nb_delays,
                         s->max_delay, s->nb_channels);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioFDelayContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        const uint8_t *src = in->extended_data[ch];
        uint8_t *dst = out->extended_data[ch];

        s->filter_channel(ctx, s->st, in->nb_samples, src, dst, ch);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFDelayContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (ff_filter_disabled(ctx) || !s->nb_delays)
        return ff_filter_frame(outlink, in);

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
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->next_pts = in->pts;
    s->next_pts += in->duration;

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFDelayContext *s = ctx->priv;
    AVFrame *frame = NULL;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF)
            s->eof = 1;
    }

    ret = ff_inlink_consume_frame(inlink, &frame);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, frame);

    if (s->eof && s->max_delay) {
        int nb_samples = FFMIN(s->max_delay, 2048);

        frame = ff_get_audio_buffer(outlink, nb_samples);
        if (!frame)
            return AVERROR(ENOMEM);
        s->max_delay -= nb_samples;

        av_samples_set_silence(frame->extended_data, 0,
                               frame->nb_samples,
                               outlink->ch_layout.nb_channels,
                               frame->format);

        frame->duration = av_rescale_q(frame->nb_samples,
                                       (AVRational){1, outlink->sample_rate},
                                       outlink->time_base);
        frame->pts = s->next_pts;
        return filter_frame(inlink, frame);
    }

    if (s->eof && s->max_delay == 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->next_pts);
        return 0;
    }

    if (!s->eof)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AudioFDelayContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    s->update_state(ctx, s->st, s->delays_opt, s->nb_delays,
                    s->max_delay, s->nb_channels, 0);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx, &s->st, s->nb_channels);
}

static const AVFilterPad afdelay_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_afdelay = {
    .p.name        = "afdelay",
    .p.description = NULL_IF_CONFIG_SMALL("Fractional delay one or more audio channels."),
    .p.priv_class  = &afdelay_class,
    .priv_size     = sizeof(AudioFDelayContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(afdelay_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
