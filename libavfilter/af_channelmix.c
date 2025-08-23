/*
 * Audio Channel Mix Filter
 *
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

#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct ChannelMixContext {
    const AVClass *class;

    AVFloatDSPContext *fdsp;

    AVChannelLayout out_ch_layout;
    AVFrame **frames;
    double *lastcf;
    int *eofs;
} ChannelMixContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const ChannelMixContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE
    };
    AVFilterChannelLayouts *layouts = NULL;
    int ret;

    ret = ff_add_channel_layout(&layouts, &s->out_ch_layout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    layouts = ff_all_channel_counts();
    if (!layouts)
        return AVERROR(ENOMEM);

    for (int n = 0; n < ctx->nb_inputs; n++) {
        ret = ff_channel_layouts_ref(layouts, &cfg_in[n]->channel_layouts);
        if (ret)
            return ret;
    }

    return ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
}

static av_cold int init(AVFilterContext *ctx)
{
    ChannelMixContext *s = ctx->priv;
    int ret, nb_inputs;

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    nb_inputs = s->out_ch_layout.nb_channels+1;
    s->frames = av_calloc(nb_inputs, sizeof(*s->frames));
    s->eofs = av_calloc(nb_inputs, sizeof(*s->eofs));
    if (!s->frames || !s->eofs)
        return AVERROR(ENOMEM);

    for (int i = 0; i < nb_inputs-1; i++) {
        char *name = av_asprintf("cf%d", i);
        AVFilterPad pad = {
            .name = name,
            .type = AVMEDIA_TYPE_AUDIO,
        };
        if (!name)
            return AVERROR(ENOMEM);
        if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
            return ret;
    }
    return 0;
}

static void free_frames(AVFilterContext *ctx)
{
    ChannelMixContext *s = ctx->priv;

    for (int i = 0; i < ctx->nb_inputs; i++)
        ff_graph_frame_free(ctx, &s->frames[i]);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ChannelMixContext *s = ctx->priv;

    if (s->frames)
        free_frames(ctx);
    av_freep(&s->frames);
    av_freep(&s->lastcf);
    av_freep(&s->eofs);
    av_freep(&s->fdsp);
}

static int filter_channel(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ChannelMixContext *s = ctx->priv;
    const int in_nb_ch = ctx->inputs[0]->ch_layout.nb_channels;
    const int out_nb_ch = s->out_ch_layout.nb_channels;
    const int start = (out_nb_ch * jobnr) / nb_jobs;
    const int end = (out_nb_ch * (jobnr+1)) / nb_jobs;
    AVFrame *out = arg;
    const int out_nb_samples = out->nb_samples;

    if (out->format == AV_SAMPLE_FMT_FLTP) {
        for (int p = start; p < end; p++) {
            ptrdiff_t offset = 0;

            if (s->frames[p+1]) {
                const int p_nb_samples = s->frames[p+1]->nb_samples;

                if (p_nb_samples < out_nb_samples)
                    offset = p_nb_samples & ~(16-1);
                else
                    offset = FFALIGN(out_nb_samples, 16);

                for (int i = 0; i < in_nb_ch; i++) {
                    s->fdsp->vector_fmul_add((float *)out->extended_data[p],
                                             (float *)s->frames[0]->extended_data[i],
                                             (float *)s->frames[p+1]->extended_data[i],
                                             (float *)out->extended_data[p],
                                             offset);
                    {
                        float *ptr = (float *)s->frames[p+1]->extended_data[i];

                        s->lastcf[in_nb_ch * p + i] = ptr[p_nb_samples-1];
                    }
                }
            }

            if (offset < out_nb_samples) {
                const int awanted = FFALIGN(out_nb_samples-offset, 16);

                for (int i = 0; i < in_nb_ch; i++) {
                    s->fdsp->vector_fmac_scalar(((float *)out->extended_data[p]) + offset,
                                                ((float *)s->frames[0]->extended_data[i]) + offset,
                                                s->lastcf[in_nb_ch * p + i],
                                                awanted);
                }
            }
        }
    } else {
        for (int p = start; p < end; p++) {
            ptrdiff_t offset = 0;

            if (s->frames[p+1]) {
                const int p_nb_samples = s->frames[p+1]->nb_samples;

                if (p_nb_samples < out_nb_samples)
                    offset = p_nb_samples & ~(16-1);
                else
                    offset = FFALIGN(out_nb_samples, 16);

                for (int i = 0; i < in_nb_ch; i++) {
                    s->fdsp->vector_dmul_add((double *)out->extended_data[p],
                                             (double *)s->frames[0]->extended_data[i],
                                             (double *)s->frames[p+1]->extended_data[i],
                                             (double *)out->extended_data[p],
                                             offset);
                    {
                        double *ptr = (double *)s->frames[p+1]->extended_data[i];

                        s->lastcf[in_nb_ch * p + i] = ptr[p_nb_samples-1];
                    }
                }
            }

            if (offset < out_nb_samples) {
                const int awanted = FFALIGN(out_nb_samples-offset, 16);

                for (int i = 0; i < in_nb_ch; i++) {
                    s->fdsp->vector_dmac_scalar(((double *)out->extended_data[p]) + offset,
                                                ((double *)s->frames[0]->extended_data[i]) + offset,
                                                s->lastcf[in_nb_ch * p + i],
                                                awanted);
                }
            }
        }
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    ChannelMixContext *s = ctx->priv;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->frames[0]) {
        int ret = ff_inlink_consume_frame(ctx->inputs[0], &s->frames[0]);
        if (ret < 0)
            return ret;
    }

    if (s->frames[0]) {
        const int wanted = s->frames[0]->nb_samples;
        int ret;

        for (int i = 1; i < ctx->nb_inputs; i++) {
            if (s->frames[i] ||
                (s->eofs[i] && ff_inlink_queued_samples(ctx->inputs[i]) <= 0))
                continue;

            ret = ff_inlink_consume_samples(ctx->inputs[i],
                                            wanted, wanted,
                                            &s->frames[i]);
            if (ret < 0)
                return ret;
        }

        {
            int active_inputs = ctx->nb_inputs - 1;
            int available_inputs = 0;

            for (int i = 1; i < ctx->nb_inputs; i++) {
                if (s->frames[i])
                    available_inputs++;
                else if (s->eofs[i] && ff_inlink_queued_samples(ctx->inputs[i]) <= 0)
                    active_inputs--;
            }

            if (ff_filter_disabled(ctx) && (available_inputs == active_inputs)) {
                AVFrame *out = s->frames[0];

                s->frames[0] = NULL;
                free_frames(ctx);

                return ff_filter_frame(outlink, out);
            }

            if (available_inputs == active_inputs) {
                AVFrame *out = ff_get_audio_buffer(outlink, wanted);
                const int out_nb_ch = s->out_ch_layout.nb_channels;

                if (!out) {
                    free_frames(ctx);
                    return AVERROR(ENOMEM);
                }

                av_frame_copy_props(out, s->frames[0]);

                ff_filter_execute(ctx, filter_channel, out, NULL,
                                  FFMIN(out_nb_ch, ff_filter_get_nb_threads(ctx)));

                free_frames(ctx);

                return ff_filter_frame(outlink, out);
            }
        }
    }

    for (int i = 0; i < ctx->nb_inputs; i++) {
        int64_t pts;
        int status;

        if (s->frames[i])
            continue;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            s->eofs[i] = 1;
            if (i == 0) {
                ff_outlink_set_status(outlink, status, pts);
                return 0;
            }
        } else if (!s->eofs[i] && ff_outlink_frame_wanted(outlink)) {
            ff_inlink_request_frame(ctx->inputs[i]);
        }
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ChannelMixContext *s = ctx->priv;
    const int nb_channels = ctx->inputs[0]->ch_layout.nb_channels;

    outlink->time_base = ctx->inputs[0]->time_base;

    s->lastcf = av_calloc((ctx->nb_inputs-1) * nb_channels, sizeof(*s->lastcf));
    if (!s->lastcf)
        return AVERROR(ENOMEM);

    return 0;
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(ChannelMixContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption channelmix_options[] = {
    { "layout", "set the output channel layout", OFFSET(out_ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="stereo"}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(channelmix);

const FFFilter ff_af_channelmix = {
    .p.name        = "channelmix",
    .p.description = NULL_IF_CONFIG_SMALL("Audio channels mixing."),
    .p.priv_class  = &channelmix_class,
    .priv_size     = sizeof(ChannelMixContext),
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_DYNAMIC_INPUTS |
                     AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
