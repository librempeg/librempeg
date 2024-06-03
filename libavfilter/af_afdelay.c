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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "internal.h"

typedef struct AudioFDelayContext {
    const AVClass *class;
    double *delays_opt;
    unsigned nb_delays;
    size_t max_delay;
    int64_t next_pts;
    int nb_channels;
    int eof;

    void *st;

    int (*init_state)(AVFilterContext *ctx);
    void (*update_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);

    void (*filter_channel)(AVFilterContext *ctx, const int nb_samples,
                           const uint8_t *src, uint8_t *dst, const int ch);
} AudioFDelayContext;

#define OFFSET(x) offsetof(AudioFDelayContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM//|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_delays = {.def="0.0",.size_min=1,.sep=' '};

static const AVOption afdelay_options[] = {
    { "delays", "set the list of delays for each channel", OFFSET(delays_opt), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_delays}, 0, 1<<21, A},
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
        s->filter_channel = filter_channel_dblp;
        s->init_state = init_state_dblp;
        s->update_state = update_state_dblp;
        s->uninit_state = uninit_state_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channel = filter_channel_fltp;
        s->init_state = init_state_fltp;
        s->update_state = update_state_fltp;
        s->uninit_state = uninit_state_fltp;
        break;
    }

    return s->init_state(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFDelayContext *s = ctx->priv;
    AVFrame *out;

    if (ctx->is_disabled || !s->nb_delays)
        return ff_filter_frame(outlink, in);

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        const uint8_t *src = in->extended_data[ch];
        uint8_t *dst = out->extended_data[ch];

        s->filter_channel(ctx, in->nb_samples, src, dst, ch);
    }

    out->pts--;
    s->next_pts = in->pts;
    s->next_pts += in->duration;

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

static int process_command(AVFilterContext *ctx,
                           const char *cmd,
                           const char *args,
                           char *res,
                           int res_len,
                           int flags)
{
    AudioFDelayContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    s->update_state(ctx);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);
}

static const AVFilterPad afdelay_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_afdelay = {
    .name          = "afdelay",
    .description   = NULL_IF_CONFIG_SMALL("Fractional delay one or more audio channels."),
    .priv_size     = sizeof(AudioFDelayContext),
    .priv_class    = &afdelay_class,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(afdelay_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .process_command = process_command,
};
