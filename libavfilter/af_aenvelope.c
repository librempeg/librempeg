/*
 * Copyright (c) 2024 Paul B Mahol
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

#include <float.h> /* DBL_MAX */

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "filters.h"
#include "avfilter.h"

typedef struct AudioEnvelopeContext {
    const AVClass *class;

    double *attack;
    unsigned nb_attack;

    double *release;
    unsigned nb_release;

    double *hold;
    unsigned nb_hold;

    double look;
    int link;
    int hlook;
    int trim_size;
    int flush_size;
    int64_t last_pts;

    int nb_channels;
    AVFrame *in;

    void *st;

    int (*envelope_init)(AVFilterContext *ctx);
    void (*envelope_uninit)(AVFilterContext *ctx);
    int (*do_envelope)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
} AudioEnvelopeContext;

#define OFFSET(x) offsetof(AudioEnvelopeContext, x)
#define AF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
#define AFR AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_attack = {.def="0.01",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_release = {.def="0.8",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_hold = {.def="0.0",.size_min=1,.sep=' '};

static const AVOption aenvelope_options[] = {
    { "attack", "set the attack time", OFFSET(attack), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_attack}, 0, 10, AFR },
    { "release", "set the release time", OFFSET(release), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_release}, 0, 10, AFR },
    { "hold", "set the hold time", OFFSET(hold), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_hold}, 0, 10, AFR },
    { "look", "set the look-ahead time", OFFSET(look), AV_OPT_TYPE_DOUBLE, {.dbl=0.01}, 0.0005, 0.5, AF },
    { "link", "enable channels linking", OFFSET(link), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aenvelope);

#define DEPTH 32
#include "aenvelope_template.c"

#undef DEPTH
#define DEPTH 64
#include "aenvelope_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioEnvelopeContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_envelope = s->link ? do_envelope_link_fltp : do_envelope_fltp;
        s->envelope_uninit = envelope_uninit_fltp;
        s->envelope_init = envelope_init_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_envelope = s->link ? do_envelope_link_dblp :  do_envelope_dblp;
        s->envelope_uninit = envelope_uninit_dblp;
        s->envelope_init = envelope_init_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->envelope_init(ctx);
}

static int envelope_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioEnvelopeContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_envelope(ctx, s->in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *outlink, AVFrame *in)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioEnvelopeContext *s = ctx->priv;
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);
    out->pts -= av_rescale_q(s->hlook - s->trim_size, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->nb_samples -= s->trim_size;
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    s->last_pts = out->pts + out->duration;

    s->in = in;
    ff_filter_execute(ctx, envelope_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (s->trim_size > 0) {
        if (s->trim_size < in->nb_samples) {
            const size_t bps = av_get_bytes_per_sample(out->format);

            for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
                out->extended_data[ch] += s->trim_size * bps;

            s->trim_size = 0;
        } else {
            s->trim_size = FFMAX(s->trim_size - in->nb_samples, 0);
        }
    }

    if (s->trim_size > 0) {
        ff_inlink_request_frame(inlink);
        av_frame_free(&out);
    }

    av_frame_free(&in);
    s->in = NULL;
    if (out)
        return ff_filter_frame(outlink, out);
    return 0;
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioEnvelopeContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = s->flush_size;
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);
        AVFrame *in = ff_get_audio_buffer(outlink, nb_samples);

        if (!out || !in) {
            av_frame_free(&out);
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        s->flush_size -= nb_samples;

        s->in = in;
        ff_filter_execute(ctx, envelope_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        out->pts = s->last_pts;
        out->duration = av_rescale_q(out->nb_samples,
                                     (AVRational){1, outlink->sample_rate},
                                     outlink->time_base);
        s->last_pts += out->duration;

        av_frame_free(&in);
        s->in = NULL;
        ret = ff_filter_frame(outlink, out);
        if (ret < 0)
            break;
    }

    return ret;
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
        AudioEnvelopeContext *s = ctx->priv;

        if (s->flush_size > 0)
            ret = flush_frame(outlink);

        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioEnvelopeContext *s = ctx->priv;

    if (s->envelope_uninit)
        s->envelope_uninit(ctx);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AudioEnvelopeContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->envelope_init(ctx);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_aenvelope = {
    .p.name        = "aenvelope",
    .p.description = NULL_IF_CONFIG_SMALL("Audio envelope."),
    .p.priv_class  = &aenvelope_class,
    .priv_size     = sizeof(AudioEnvelopeContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
