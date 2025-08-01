/*
 * Copyright (c) 2012 Andrey Utkin
 * Copyright (c) 2012 Stefano Sabatini
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

/**
 * @file
 * Filter that changes number of samples on single output operation
 */

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct ASNSContext {
    const AVClass *class;
    int nb_out_samples;  ///< how many samples to output
    int pad;
} ASNSContext;

#define OFFSET(x) offsetof(ASNSContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption asetnsamples_options[] = {
    { "nb_out_samples", "set the number of per-frame output samples", OFFSET(nb_out_samples), AV_OPT_TYPE_INT, {.i64=1024}, 1, INT_MAX, FLAGS },
    { "n",              "set the number of per-frame output samples", OFFSET(nb_out_samples), AV_OPT_TYPE_INT, {.i64=1024}, 1, INT_MAX, FLAGS },
    { "pad", "pad last frame with zeros", OFFSET(pad), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
    { "p",   "pad last frame with zeros", OFFSET(pad), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(asetnsamples);

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    ASNSContext *s = ctx->priv;
    AVFrame *frame = NULL, *pad_frame;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_filter_disabled(ctx))
        ret = ff_inlink_consume_frame(inlink, &frame);
    else
        ret = ff_inlink_consume_samples(inlink, s->nb_out_samples, s->nb_out_samples, &frame);
    if (ret < 0)
        return ret;

    if (ret > 0) {
        if (!s->pad || ff_filter_disabled(ctx) || frame->nb_samples == s->nb_out_samples)
            return ff_filter_frame(outlink, frame);

        pad_frame = ff_get_audio_buffer(outlink, s->nb_out_samples);
        if (!pad_frame) {
            av_frame_free(&frame);
            return AVERROR(ENOMEM);
        }

        ret = av_frame_copy_props(pad_frame, frame);
        if (ret < 0) {
            av_frame_free(&pad_frame);
            av_frame_free(&frame);
            return ret;
        }

        av_samples_copy(pad_frame->extended_data, frame->extended_data,
                        0, 0, frame->nb_samples, frame->ch_layout.nb_channels, frame->format);
        av_samples_set_silence(pad_frame->extended_data, frame->nb_samples,
                               s->nb_out_samples - frame->nb_samples, frame->ch_layout.nb_channels,
                               frame->format);
        av_frame_free(&frame);
        return ff_filter_frame(outlink, pad_frame);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    if (ff_inlink_queued_samples(inlink) >= s->nb_out_samples) {
        ff_filter_set_ready(ctx, 100);
        return 0;
    }
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

const FFFilter ff_af_asetnsamples = {
    .p.name        = "asetnsamples",
    .p.description = NULL_IF_CONFIG_SMALL("Set the number of samples for each output audio frames."),
    .p.priv_class  = &asetnsamples_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size   = sizeof(ASNSContext),
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    .activate    = activate,
    .process_command = ff_filter_process_command,
};
