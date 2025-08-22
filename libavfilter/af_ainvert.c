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
#include <math.h>
#include <stdio.h>

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"

typedef struct AudioInvertContext {
    const AVClass *class;

    AVChannelLayout ch_layout;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioInvertContext;

#define OFFSET(x) offsetof(AudioInvertContext,x)
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption ainvert_options[] = {
    { "channels", "set channels to filter", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, AFT },
    {NULL}
};

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 16
#include "ainvert_template.c"

#undef DEPTH
#define DEPTH 31
#include "ainvert_template.c"

#undef DEPTH
#define DEPTH 63
#include "ainvert_template.c"

#undef DEPTH
#define DEPTH 32
#include "ainvert_template.c"

#undef DEPTH
#define DEPTH 64
#include "ainvert_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioInvertContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_S16P:
        s->filter_channels = filter_channels_s16p;
        break;
    case AV_SAMPLE_FMT_S32P:
        s->filter_channels = filter_channels_s32p;
        break;
    case AV_SAMPLE_FMT_S64P:
        s->filter_channels = filter_channels_s64p;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        break;
    default:
        av_assert0(0);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioInvertContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        int ret;

        out = ff_graph_frame_alloc(ctx);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        out->nb_samples = in->nb_samples;
        ret = ff_filter_get_buffer(ctx, out);
        if (ret < 0) {
            av_frame_free(&out);
            av_frame_free(&in);
            return ret;
        }

        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (in != out)
        ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const AudioInvertContext *s_src = src->priv;
    AudioInvertContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    if (av_channel_layout_compare(&s_dst->ch_layout, &s_src->ch_layout))
        return av_channel_layout_copy(&s_dst->ch_layout, &s_src->ch_layout);
    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

AVFILTER_DEFINE_CLASS(ainvert);

const FFFilter ff_af_ainvert = {
    .p.name          = "ainvert",
    .p.description   = NULL_IF_CONFIG_SMALL("Invert Audio Polarity."),
    .p.priv_class    = &ainvert_class,
    .priv_size       = sizeof(AudioInvertContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P, AV_SAMPLE_FMT_S64P,
                      AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS |
                       AVFILTER_FLAG_FRAME_THREADS |
                       AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
