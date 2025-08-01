/*
 * Copyright (c) 2007 Bobby Bingham
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
 * audio and video splitter
 */

#include <stdio.h>

#include "libavutil/attributes.h"
#include "libavutil/avstring.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "video.h"

typedef struct SplitContext {
    const AVClass *class;
    int nb_outputs;
} SplitContext;

static av_cold int split_init(AVFilterContext *ctx)
{
    SplitContext *s = ctx->priv;
    int i, ret;

    for (i = 0; i < s->nb_outputs; i++) {
        AVFilterPad pad = { 0 };

        pad.type = ctx->filter->inputs[0].type;
        pad.name = av_asprintf("output%d", i);
        if (!pad.name)
            return AVERROR(ENOMEM);

        if ((ret = ff_append_outpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFrame *in;
    int status, ret, nb_eofs = 0;
    int64_t pts;

    for (int i = 0; i < ctx->nb_outputs; i++)
        nb_eofs += ff_outlink_get_status(ctx->outputs[i]) == AVERROR_EOF;

    if (nb_eofs == ctx->nb_outputs) {
        ff_inlink_set_status(inlink, AVERROR_EOF);
        return 0;
    }

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            AVFrame *buf_out;

            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;
            buf_out = av_frame_clone(in);
            if (!buf_out) {
                ret = AVERROR(ENOMEM);
                break;
            }

            ret = ff_filter_frame(ctx->outputs[i], buf_out);
            if (ret < 0)
                break;
        }

        av_frame_free(&in);
        return ret;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;
            ff_outlink_set_status(ctx->outputs[i], status, pts);
        }
        return 0;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i]))
            continue;

        if (ff_outlink_frame_wanted(ctx->outputs[i])) {
            ff_inlink_request_frame(inlink);
            return 0;
        }
    }

    return FFERROR_NOT_READY;
}

#define OFFSET(x) offsetof(SplitContext, x)
#define FLAGS (AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)
static const AVOption options[] = {
    { "outputs", "set number of outputs", OFFSET(nb_outputs), AV_OPT_TYPE_INT, { .i64 = 2 }, 1, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(split, "(a)split", options);

const FFFilter ff_vf_split = {
    .p.name        = "split",
    .p.description = NULL_IF_CONFIG_SMALL("Pass on the input to N video outputs."),
    .p.priv_class  = &split_class,
    .p.flags       = AVFILTER_FLAG_DYNAMIC_OUTPUTS | AVFILTER_FLAG_METADATA_ONLY,
    .priv_size   = sizeof(SplitContext),
    .init        = split_init,
    .activate    = activate,
    FILTER_INPUTS(ff_video_default_filterpad),
};

const FFFilter ff_af_asplit = {
    .p.name        = "asplit",
    .p.description = NULL_IF_CONFIG_SMALL("Pass on the audio input to N audio outputs."),
    .p.priv_class  = &split_class,
    .p.flags       = AVFILTER_FLAG_DYNAMIC_OUTPUTS | AVFILTER_FLAG_METADATA_ONLY,
    .priv_size   = sizeof(SplitContext),
    .init        = split_init,
    .activate    = activate,
    FILTER_INPUTS(ff_audio_default_filterpad),
};
