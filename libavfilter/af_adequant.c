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

#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "audio.h"

typedef struct ADequantContext {
    const AVClass *class;

    unsigned cur_size;
    unsigned max_size;
    int nb_channels;
    int blocksize;
    double window;
    double gain;

    void *state;

    int64_t pts;
    int nb_samples;

    int (*init_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} ADequantContext;

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 32
#include "adequant_template.c"

#undef DEPTH
#define DEPTH 64
#include "adequant_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ADequantContext *s = ctx->priv;

    s->pts = AV_NOPTS_VALUE;
    s->max_size = inlink->sample_rate;
    s->cur_size = lrint(s->window * s->max_size);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->init_state = init_state_fltp;
        s->uninit_state = uninit_state_fltp;
        s->filter_channels = (s->blocksize > 0) ? filter_block_channels_fltp : filter_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_state = init_state_dblp;
        s->uninit_state = uninit_state_dblp;
        s->filter_channels = (s->blocksize > 0) ? filter_block_channels_dblp : filter_channels_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ADequantContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;
    int drop = 0;

    if (av_frame_is_writable(in) && s->blocksize == 0) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, s->blocksize > 0 ? s->blocksize : in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    if (s->blocksize > 0 && s->pts == AV_NOPTS_VALUE)
        drop = 1;

    td.in = in; td.out = out;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(s->nb_channels, ff_filter_get_nb_threads(ctx)));

    if (s->blocksize > 0) {
        int nb_samples = in->nb_samples;
        int64_t pts = in->pts;

        out->pts = s->pts;
        out->nb_samples = s->nb_samples;
        s->pts = pts;
        s->nb_samples = nb_samples;
    }

    if (in != out)
        av_frame_free(&in);
    if (!drop) {
        return ff_filter_frame(outlink, out);
    } else {
        av_frame_free(&out);
        ff_filter_set_ready(ctx, 10);
        return 0;
    }
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    ADequantContext *s = ctx->priv;
    AVFrame *in = NULL;
    int64_t pts;
    int status;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (s->blocksize > 0) {
        ret = ff_inlink_consume_samples(inlink, s->blocksize, s->blocksize, &in);
    } else {
        ret = ff_inlink_consume_frame(inlink, &in);
    }
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    if (s->blocksize > 0 && ff_inlink_queued_samples(inlink) >= s->blocksize) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->blocksize > 0) {
            in = ff_get_audio_buffer(outlink, s->blocksize);
            if (!in)
                return AVERROR(ENOMEM);

            ret = filter_frame(inlink, in);
        }

        ff_outlink_set_status(outlink, status, pts);

        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);
}

#define OFFSET(x) offsetof(ADequantContext, x)
#define TF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption adequant_options[] = {
    { "window",   "set the window size", OFFSET(window),    AV_OPT_TYPE_DOUBLE, {.dbl=1.0},  0.1, 1.0, TF },
    { "gain",     "set the gain",        OFFSET(gain),      AV_OPT_TYPE_DOUBLE, {.dbl=0.01}, 0.0, 1.0, TF },
    { "blocksize","set the block size",  OFFSET(blocksize), AV_OPT_TYPE_INT,    {.i64=0},      0, 32768, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adequant);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_adequant = {
    .p.name          = "adequant",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Dynamic Dequantization of input audio."),
    .p.priv_class    = &adequant_class,
    .priv_size       = sizeof(ADequantContext),
    .activate        = activate,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
