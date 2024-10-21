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

#include <float.h>

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct AHoldContext {
    const AVClass *class;

    int hold;
    AVChannelLayout ch_layout;

    void *st;

    int (*filter_init)(AVFilterContext *ctx, void **state,
                       const int nb_channels);
    void (*filter_channel)(void *st, const void *ibuf, void *obuf,
                           const int nb_samples, const int hold,
                           const int ch, const int disabled);
} AHoldContext;

#define DEPTH 16
#include "ahold_template.c"

#undef DEPTH
#define DEPTH 31
#include "ahold_template.c"

#undef DEPTH
#define DEPTH 32
#include "ahold_template.c"

#undef DEPTH
#define DEPTH 64
#include "ahold_template.c"

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AHoldContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S16P:
        s->filter_channel = ahold_channel_s16p;
        s->filter_init = ahold_init_s16p;
        break;
    case AV_SAMPLE_FMT_S32P:
        s->filter_channel = ahold_channel_s32p;
        s->filter_init = ahold_init_s32p;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channel = ahold_channel_fltp;
        s->filter_init = ahold_init_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channel = ahold_channel_dblp;
        s->filter_init = ahold_init_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->filter_init(ctx, &s->st, inlink->ch_layout.nb_channels);
}

static int filter_channels(AVFilterContext *ctx, void *arg,
                           int jobnr, int nb_jobs)
{
    AHoldContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int is_disabled = ff_filter_disabled(ctx);

    for (int ch = start; ch < end; ch++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&in->ch_layout, ch);
        const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;

        s->filter_channel(s->st, in->extended_data[ch],
                          out->extended_data[ch],
                          out->nb_samples, s->hold,
                          ch, is_disabled|bypass);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
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

    td.in = in; td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AHoldContext *s = ctx->priv;

    av_freep(&s->st);
}

#define OFFSET(x) offsetof(AHoldContext, x)
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption ahold_options[] = {
    { "hold",     "set the sample hold",     OFFSET(hold),      AV_OPT_TYPE_BOOL,     {.i64=0},     0, 1, AFT },
    { "channels", "set channels to filter",  OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, AFT },
    { NULL }
};

AVFILTER_DEFINE_CLASS(ahold);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_ahold = {
    .p.name          = "ahold",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply sample hold audio filter."),
    .p.priv_class    = &ahold_class,
    .priv_size       = sizeof(AHoldContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P, AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
};
