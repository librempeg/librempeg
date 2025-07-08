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

typedef struct AudioExtraContext {
    const AVClass *class;

    double extra_s;
    int nb_samples;
    int order;
    int pre, post;

    int64_t next_pts;
    int nb_channels;
    int eof;

    void *st;

    int (*init_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);

    void (*filter_channel)(AVFilterContext *ctx, void *state, const int nb_samples,
                           const uint8_t *src, uint8_t *dst, const int ch);
} AudioExtraContext;

#define OFFSET(x) offsetof(AudioExtraContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption aextra_options[] = {
    { "extra", "set the duration of extrapolation", OFFSET(extra_s), AV_OPT_TYPE_DOUBLE, {.dbl=0.1}, 0,  1, A},
    { "order", "set the extrapolation order",       OFFSET(order),   AV_OPT_TYPE_INT,    {.i64=32},  1, 64, A},
    { NULL }
};

AVFILTER_DEFINE_CLASS(aextra);

#define DEPTH 32
#include "aextra_template.c"

#undef DEPTH
#define DEPTH 64
#include "aextra_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioExtraContext *s = ctx->priv;

    s->nb_channels = inlink->ch_layout.nb_channels;
    s->nb_samples = FFMAX(lrint(s->extra_s * inlink->sample_rate), s->order*2+1);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channel = aextra_channel_dblp;
        s->init_state = init_aextra_dblp;
        s->uninit_state = uninit_aextra_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channel = aextra_channel_fltp;
        s->init_state = init_aextra_fltp;
        s->uninit_state = uninit_aextra_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioExtraContext *s = ctx->priv;
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
    AudioExtraContext *s = ctx->priv;
    int extra = (s->pre && !s->eof) ? 0 : s->nb_samples;
    ThreadData td;
    AVFrame *out;

    if (!s->nb_samples)
        return ff_filter_frame(outlink, in);

    out = ff_get_audio_buffer(outlink, in->nb_samples + extra);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->next_pts = in->pts;
    s->next_pts += in->duration;

    s->pre = 1;
    s->post = s->eof;
    if (s->post)
        out->nb_samples = s->nb_samples;

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioExtraContext *s = ctx->priv;
    int ret, status;
    AVFrame *frame;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF)
            s->eof = 1;
    }

    ret = ff_inlink_consume_samples(inlink, s->nb_samples, s->nb_samples, &frame);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, frame);

    if (s->eof && !s->post) {
        frame = ff_get_audio_buffer(outlink, s->nb_samples);
        if (!frame)
            return AVERROR(ENOMEM);

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

    if (s->eof && s->post == 1) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->next_pts);
        return 0;
    }

    if (!s->eof)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioExtraContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);
}

static const AVFilterPad aextra_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_aextra = {
    .p.name        = "aextra",
    .p.description = NULL_IF_CONFIG_SMALL("Extrapolate start and end samples in the audio stream."),
    .p.priv_class  = &aextra_class,
    .priv_size     = sizeof(AudioExtraContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(aextra_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
};
