/*
 * Copyright (c) 2011 Nicolas George <nicolas.george@normalesup.org>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Audio merging filter
 */

#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "audio.h"
#include "formats.h"

typedef struct InputItem {
    int in;
    uint8_t *ins;
    AVFrame *inbuf;
} InputItem;

typedef struct AMergeContext {
    const AVClass *class;
    int nb_inputs;
    int *route; /**< channels routing, see copy_samples */
    int bps;

    InputItem *ii;
} AMergeContext;

#define OFFSET(x) offsetof(AMergeContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption amerge_options[] = {
    { "inputs", "specify the number of inputs", OFFSET(nb_inputs),
      AV_OPT_TYPE_INT, { .i64 = 2 }, 1, INT16_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(amerge);

static av_cold void uninit(AVFilterContext *ctx)
{
    AMergeContext *s = ctx->priv;

    av_freep(&s->ii);
    av_freep(&s->route);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AMergeContext *s = ctx->priv;
    AVChannelLayout outlayout = { 0 };
    AVFilterChannelLayouts *layouts;
    int ret, nb_ch = 0;

    for (int i = 0; i < s->nb_inputs; i++) {
        if (!ctx->inputs[i]->incfg.channel_layouts ||
            ctx->inputs[i]->incfg.channel_layouts->nb_channel_layouts < 1)
            return AVERROR(EAGAIN);
    }

    if ((ret = ff_set_common_all_samplerates2(ctx, cfg_in, cfg_out)) < 0)
        return ret;

    if ((ret = ff_set_common_formats2(ctx, cfg_in, cfg_out, ff_all_formats(AVMEDIA_TYPE_AUDIO))) < 0)
        return ret;

    for (int i = 0; i < s->nb_inputs; i++) {
        AVChannelLayout *in_layout;

        layouts = NULL;
        in_layout = &ctx->inputs[i]->incfg.channel_layouts->channel_layouts[0];
        nb_ch += in_layout->nb_channels;

        if ((ret = ff_add_channel_layout(&layouts, in_layout)) < 0)
            return ret;
        if ((ret = ff_channel_layouts_ref(layouts, &cfg_in[i]->channel_layouts)) < 0)
            return ret;
    }

    layouts = NULL;
    av_channel_layout_default(&outlayout, nb_ch);
    if ((ret = ff_add_channel_layout(&layouts, &outlayout)) < 0)
        return ret;
    return ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AMergeContext *s = ctx->priv;
    int nb_ch = 0, out_idx = 0;

    s->bps = av_get_bytes_per_sample(outlink->format);
    outlink->time_base  = ctx->inputs[0]->time_base;

    for (int i = 0; i < s->nb_inputs; i++) {
        InputItem *ii = &s->ii[i];

        ii->in = ctx->inputs[i]->ch_layout.nb_channels;
        nb_ch += ii->in;
    }

    s->route = av_calloc(nb_ch, sizeof(*s->route));
    if (!s->route)
        return AVERROR(ENOMEM);

    for (int i = 0, j = 0; i < s->nb_inputs; i++) {
        InputItem *ii = &s->ii[i];
        const int input_nb_ch = ii->in;

        for (int c = 0; c < input_nb_ch; c++, j++) {
            int chan = av_channel_layout_channel_from_index(&ctx->inputs[i]->ch_layout, c);

            if (chan != AV_CHAN_NONE) {
                int idx = av_channel_layout_index_from_channel(&ctx->outputs[0]->ch_layout, chan);

                if (idx >= 0)
                    s->route[j] = idx;
            } else {
                s->route[j] = out_idx++;
            }

            for (int n = j-1; n >= 0; n--) {
                if (s->route[n] == s->route[j])
                    s->route[n] = (s->route[n] + 1) % nb_ch;
            }
        }
    }

    return 0;
}

/**
 * Copy samples from several input streams to one output stream.
 * @param nb_inputs number of inputs
 * @param route     routing values;
 * @param ii        InputItem pointer
 * @param outs      pointer to the samples of the output, in packet format;
 *                  must point to a buffer big enough;
 *                  will be left at the end of the copied samples
 * @param ns        number of samples to copy
 * @param nb_ch     number of output channels
 * @param bps       bytes per sample
 */
static inline void copy_samples(int nb_inputs,
                                const int *route, InputItem *ii,
                                uint8_t **outs, int ns, int bps,
                                const int nb_ch)
{
    for (int n = 0; n < ns; n++) {
        for (int i = 0, j = 0; i < nb_inputs; i++) {
            for (int c = 0; c < ii[i].in; c++, j++) {
                memcpy(outs[0] + bps * route[j], ii[i].ins, bps);
                ii[i].ins += bps;
            }
        }

        *outs += nb_ch * bps;
    }
}

static void free_frames(int nb_inputs, InputItem *ii)
{
    for (int i = 0; i < nb_inputs; i++)
        av_frame_free(&ii[i].inbuf);
}

static int try_push_frame(AVFilterContext *ctx, int nb_samples)
{
    AMergeContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_ch = outlink->ch_layout.nb_channels;
    AVFrame *outbuf;
    uint8_t *outs;
    int ret;

    for (int i = 0; i < ctx->nb_inputs; i++) {
        ret = ff_inlink_consume_samples(ctx->inputs[i], nb_samples, nb_samples, &s->ii[i].inbuf);
        if (ret < 0) {
            free_frames(i, s->ii);
            return ret;
        }
        s->ii[i].ins = s->ii[i].inbuf->data[0];
    }

    outbuf = ff_get_audio_buffer(outlink, nb_samples);
    if (!outbuf) {
        free_frames(s->nb_inputs, s->ii);
        return AVERROR(ENOMEM);
    }

    outs = outbuf->data[0];
    outbuf->pts = s->ii[0].inbuf->pts;

    outbuf->nb_samples     = nb_samples;
    outbuf->duration = av_rescale_q(outbuf->nb_samples,
                                    av_make_q(1, outlink->sample_rate),
                                    outlink->time_base);

    if ((ret = av_channel_layout_copy(&outbuf->ch_layout, &outlink->ch_layout)) < 0) {
        free_frames(s->nb_inputs, s->ii);
        av_frame_free(&outbuf);
        return ret;
    }

    while (nb_samples) {
        if (av_sample_fmt_is_planar(outlink->format)) {
            for (int i = 0, j = 0; i < s->nb_inputs; i++) {
                for (int c = 0; c < s->ii[i].in; c++, j++)
                    memcpy(outbuf->extended_data[s->route[j]],
                           s->ii[i].inbuf->extended_data[c], s->bps * nb_samples);
            }

            break;
        }

        /* Unroll the most common sample formats: speed +~350% for the loop,
           +~13% overall (including two common decoders) */
        switch (s->bps) {
        case 1:
            copy_samples(s->nb_inputs, s->route, s->ii, &outs, nb_samples, 1, nb_ch);
            break;
        case 2:
            copy_samples(s->nb_inputs, s->route, s->ii, &outs, nb_samples, 2, nb_ch);
            break;
        case 4:
            copy_samples(s->nb_inputs, s->route, s->ii, &outs, nb_samples, 4, nb_ch);
            break;
        default:
            copy_samples(s->nb_inputs, s->route, s->ii, &outs, nb_samples, s->bps, nb_ch);
            break;
        }

        nb_samples = 0;
    }

    free_frames(s->nb_inputs, s->ii);
    return ff_filter_frame(outlink, outbuf);
}

static int activate(AVFilterContext *ctx)
{
    int ret, nb_samples, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[0], ctx);

    nb_samples = ff_inlink_queued_samples(ctx->inputs[0]);
    for (int i = 1; i < ctx->nb_inputs && nb_samples > 0; i++)
        nb_samples = FFMIN(ff_inlink_queued_samples(ctx->inputs[i]), nb_samples);

    if (nb_samples) {
        ret = try_push_frame(ctx, nb_samples);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < ctx->nb_inputs; i++) {
        if (ff_inlink_queued_samples(ctx->inputs[i]))
            continue;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            ff_outlink_set_status(ctx->outputs[0], status, pts);
            return 0;
        } else if (ff_outlink_frame_wanted(ctx->outputs[0])) {
            ff_inlink_request_frame(ctx->inputs[i]);
            return 0;
        }
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    AMergeContext *s = ctx->priv;

    s->ii = av_calloc(s->nb_inputs, sizeof(*s->ii));
    if (!s->ii)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->nb_inputs; i++) {
        char *name = av_asprintf("in%d", i);
        AVFilterPad pad = {
            .name = name,
            .type = AVMEDIA_TYPE_AUDIO,
        };
        int ret;

        if (!name)
            return AVERROR(ENOMEM);
        if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
            return ret;
    }
    return 0;
}

static const AVFilterPad amerge_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_amerge = {
    .p.name        = "amerge",
    .p.description = NULL_IF_CONFIG_SMALL("Merge two or more audio streams into "
                                          "a single multi-channel stream."),
    .p.priv_class  = &amerge_class,
    .p.inputs      = NULL,
    .priv_size     = sizeof(AMergeContext),
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_OUTPUTS(amerge_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_DYNAMIC_INPUTS,
};
