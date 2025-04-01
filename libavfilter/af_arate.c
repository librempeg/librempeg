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

typedef struct AudioRateContext {
    const AVClass *class;

    double tolerance;
    int nb_samples;

    uint64_t in;
    uint64_t out;
    uint64_t drop;
    uint64_t add;

    int64_t last_in_pts;
    int64_t last_out_pts;
    int64_t eof_pts;
    int eof;
} AudioRateContext;

#define OFFSET(x) offsetof(AudioRateContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define RT AV_OPT_FLAG_RUNTIME_PARAM
#define RO AV_OPT_FLAG_READONLY
#define X AV_OPT_FLAG_EXPORT

static const AVOption arate_options[] = {
    { "tolerance", "set the tolerance for dropped/added samples", OFFSET(tolerance),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, UINT64_MAX, A|F|RT},
    { "nb_samples","set the max number of samples per frame",     OFFSET(nb_samples), AV_OPT_TYPE_INT,    {.i64=1024}, 1, INT32_MAX, A|F|RT},
    { "in",        "number of input samples",                     OFFSET(in),         AV_OPT_TYPE_UINT64, {.i64=0}, 0, UINT64_MAX, A|F|RO|X},
    { "out",       "number of output samples",                    OFFSET(out),        AV_OPT_TYPE_UINT64, {.i64=0}, 0, UINT64_MAX, A|F|RO|X},
    { "drop",      "number of dropped samples",                   OFFSET(drop),       AV_OPT_TYPE_UINT64, {.i64=0}, 0, UINT64_MAX, A|F|RO|X},
    { "add",       "number of added samples",                     OFFSET(add),        AV_OPT_TYPE_UINT64, {.i64=0}, 0, UINT64_MAX, A|F|RO|X},
    { NULL }
};

AVFILTER_DEFINE_CLASS(arate);

static int64_t get_frame_duration(const AVFrame *frame, const AVFilterLink *inlink)
{
    int64_t frame_duration;

    if (frame->duration > 0) {
        frame_duration = frame->duration;
    } else {
        frame_duration = av_rescale_q(frame->nb_samples, inlink->time_base, av_make_q(1, inlink->sample_rate));
    }

    return frame_duration;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRateContext *s = ctx->priv;

    if (ctx->is_disabled)
        return ff_filter_frame(outlink, in);

    if (s->last_in_pts == AV_NOPTS_VALUE) {
        s->last_in_pts = in->pts + get_frame_duration(in, inlink);
        s->last_out_pts = s->last_in_pts;

        s->in += in->nb_samples;
        s->out += in->nb_samples;

        return ff_filter_frame(outlink, in);
    }

    s->in += in->nb_samples;

    if (FFABS(in->pts - s->last_in_pts) <= s->tolerance) {
        in->pts = s->last_out_pts;

        s->last_in_pts = in->pts + get_frame_duration(in, inlink);
        s->last_out_pts += get_frame_duration(in, inlink);

        s->out += in->nb_samples;
    } else if (in->pts - s->last_in_pts > s->tolerance) {
        int64_t last_pts = s->last_in_pts;
        int64_t nb_samples;
        AVFrame *out;
        int ret;

        do {
            int64_t frame_duration;

            nb_samples = FFMIN((in->pts - last_pts) - s->tolerance, s->nb_samples);
            if (nb_samples <= 0)
                break;

            out = ff_get_audio_buffer(outlink, nb_samples);
            frame_duration = av_rescale_q(nb_samples, inlink->time_base, av_make_q(1, inlink->sample_rate));
            out->pts = s->last_out_pts;
            out->duration = frame_duration;
            last_pts += out->duration;
            s->last_out_pts += out->duration;

            s->out += out->nb_samples;
            s->add += out->nb_samples;

            ret = ff_filter_frame(outlink, out);
        } while (ret >= 0);

        s->last_in_pts = in->pts + get_frame_duration(in, inlink);
        in->pts = s->last_out_pts;
        s->last_out_pts += get_frame_duration(in, inlink);
    } else {
        int64_t nb_samples = s->tolerance - (in->pts - s->last_in_pts);

        s->last_in_pts = in->pts + get_frame_duration(in, inlink);

        if (nb_samples >= in->nb_samples) {
            ff_filter_set_ready(ctx, 100);
            s->drop += in->nb_samples;
            av_frame_free(&in);
            return 0;
        } else {
            s->drop += nb_samples;
            in->nb_samples -= nb_samples;
            s->out += in->nb_samples;
        }

        in->pts = s->last_out_pts;
        s->last_out_pts += get_frame_duration(in, inlink);
    }

    return ff_filter_frame(outlink, in);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioRateContext *s = ctx->priv;

    s->last_in_pts = AV_NOPTS_VALUE;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioRateContext *s = ctx->priv;
    AVFrame *frame = NULL;
    int ret, status;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_acknowledge_status(inlink, &status, &s->eof_pts)) {
        if (status == AVERROR_EOF)
            s->eof = 1;
    }

    ret = ff_inlink_consume_frame(inlink, &frame);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, frame);

    if (s->eof) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->eof_pts);
        return 0;
    }

    if (!s->eof)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioRateContext *s = ctx->priv;

    av_log(ctx, AV_LOG_VERBOSE, "in: %"PRIu64", out: %"PRIu64", add: %"PRIu64", drop: %"PRIu64"\n", s->in, s->out, s->add, s->drop);
}

const FFFilter ff_af_arate = {
    .p.name        = "arate",
    .p.description = NULL_IF_CONFIG_SMALL("Make audio stream continuous."),
    .p.priv_class  = &arate_class,
    .priv_size     = sizeof(AudioRateContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
