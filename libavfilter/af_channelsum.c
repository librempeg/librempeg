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

/**
 * @file
 * audio channel summing filter
 */

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct ChannelSum {
    unsigned out_ch_idx;
    unsigned nb_in_ch;
    int *in_ch_idx;
} ChannelSum;

typedef struct ChannelSumContext {
    const AVClass *class;

    AVChannelLayout *out_channel;
    unsigned nb_out_channel;

    AVChannelLayout *sum_channels;
    unsigned nb_sum_channels;

    AVChannelLayout output_layout;

    ChannelSum *sum;
} ChannelSumContext;

#define OFFSET(x) offsetof(ChannelSumContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_out_channel = {.def="FC",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_sum_channels = {.def="1 channels (USR0)",.size_min=1,.sep='|'};

static const AVOption channelsum_options[] = {
    { "out_channel", "set the output channel", OFFSET(out_channel), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_out_channel}, .flags = A|F },
    { "sum_channels", "set the input channels to sum", OFFSET(sum_channels), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_sum_channels}, .flags = A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(channelsum);

static void channelsum_uninit(AVFilterContext *ctx)
{
    ChannelSumContext *s = ctx->priv;

    if (s->sum) {
        for (unsigned ch = 0; ch < s->nb_out_channel; ch++)
            av_freep(&s->sum[ch].in_ch_idx);

        av_freep(&s->sum);
    }

    av_channel_layout_uninit(&s->output_layout);
}

static av_cold int channelsum_init(AVFilterContext *ctx)
{
    ChannelSumContext *s = ctx->priv;
    int ret;

    for (unsigned ch = 0; ch < s->nb_out_channel; ch++) {
        if (s->out_channel[ch].nb_channels != 1)
            return AVERROR(EINVAL);
    }

    for (unsigned ch = 0; ch < s->nb_sum_channels; ch++) {
        if (s->sum_channels[ch].nb_channels < 1)
            return AVERROR(EINVAL);
    }

    ret = av_channel_layout_custom_init(&s->output_layout, s->nb_out_channel);
    if (ret < 0)
        return ret;

    for (unsigned ch = 0; ch < s->nb_out_channel; ch++) {
        enum AVChannel ch_id = av_channel_layout_channel_from_index(&s->out_channel[ch], 0);

        if (ch_id != AV_CHAN_NONE)
            s->output_layout.u.map[ch].id = ch_id;
    }

    av_channel_layout_retype(&s->output_layout, AV_CHANNEL_ORDER_NATIVE, AV_CHANNEL_LAYOUT_RETYPE_FLAG_LOSSLESS);

    s->sum = av_calloc(s->nb_out_channel, sizeof(*s->sum));
    if (!s->sum)
        return AVERROR(ENOMEM);

    return 0;
}

static int channelsum_query_formats(const AVFilterContext *ctx,
                                    AVFilterFormatsConfig **cfg_in,
                                    AVFilterFormatsConfig **cfg_out)
{
    const ChannelSumContext *s = ctx->priv;
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    AVFilterChannelLayouts *channel_layouts = NULL;
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    ret = ff_add_channel_layout(&channel_layouts, &s->output_layout);
    if (ret < 0)
        return ret;

    return ff_channel_layouts_ref(channel_layouts, &cfg_out[0]->channel_layouts);
}

static int channelsum_config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ChannelSumContext *s = ctx->priv;

    for (unsigned ch = 0; ch < s->nb_out_channel; ch++) {
        const int sidx = FFMIN(ch, s->nb_sum_channels-1);
        enum AVChannel ch_id = av_channel_layout_channel_from_index(&s->out_channel[ch], 0);
        int ch_idx = av_channel_layout_index_from_channel(&s->output_layout, ch_id);

        if (ch_idx < 0)
            ch_idx = ch;
        s->sum[ch].out_ch_idx = ch_idx;

        s->sum[ch].nb_in_ch = s->sum_channels[sidx].nb_channels;

        s->sum[ch].in_ch_idx = av_calloc(s->sum[ch].nb_in_ch, sizeof(*s->sum[0].in_ch_idx));
        if (!s->sum[ch].in_ch_idx)
            return AVERROR(ENOMEM);

        for (unsigned ich = 0; ich < s->sum[ch].nb_in_ch; ich++) {
            enum AVChannel ich_id = av_channel_layout_channel_from_index(&s->sum_channels[sidx], ich);
            int ch_idx = av_channel_layout_index_from_channel(&inlink->ch_layout, ich_id);

            if (ch_idx < 0)
                ch_idx = -1;

            s->sum[ch].in_ch_idx[ich] = ch_idx;
        }
    }

    return 0;
}

static int channelsum_filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    const int sample_size = av_get_bytes_per_sample(in->format);
    AVFilterContext  *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    const ChannelSumContext *s = ctx->priv;
    const int nb_samples= in->nb_samples;
    AVFrame *out;
    int ret;

    out = av_frame_alloc();
    if (!out)
        return AVERROR(ENOMEM);

    out->nb_samples = nb_samples;
    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        return ret;
    }

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        unsigned dst_ch_idx = s->sum[ch].out_ch_idx;
        float *dst = (float *)out->extended_data[dst_ch_idx];
        const int nb_ich = s->sum[ch].nb_in_ch;

        for (int ich = 0; ich < nb_ich; ich++) {
            const int in_ch_idx = s->sum[ch].in_ch_idx[ich];

            if (in_ch_idx < 0)
                continue;

            {
                const float *src = (const float *)in->extended_data[in_ch_idx];

                if (ich == 0) {
                    memcpy(dst, src, nb_samples * sample_size);
                    continue;
                }

                for (int n = 0; n < nb_samples; n++)
                    dst[n] += src[n];
            }
        }
    }

    av_frame_copy_props(out, in);
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static const AVFilterPad channelsum_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = channelsum_filter_frame,
        .config_props = channelsum_config_input,
    },
};

const FFFilter ff_af_channelsum = {
    .p.name        = "channelsum",
    .p.description = NULL_IF_CONFIG_SMALL("Sum audio channels."),
    .p.priv_class  = &channelsum_class,
    .p.flags       = AVFILTER_FLAG_FRAME_THREADS,
    .init          = channelsum_init,
    .uninit        = channelsum_uninit,
    .priv_size     = sizeof(ChannelSumContext),
    FILTER_INPUTS(channelsum_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(channelsum_query_formats),
};
