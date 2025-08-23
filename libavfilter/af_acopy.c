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

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out = ff_get_audio_buffer(outlink, in->nb_samples);
    int ret;

    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    ret = av_frame_copy_props(out, in);
    if (ret < 0)
        goto fail;
    ret = av_frame_copy(out, in);
    if (ret < 0)
        goto fail;
    ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    av_frame_free(&out);
    return ret;
}

static const AVFilterPad acopy_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_acopy = {
    .p.name        = "acopy",
    .p.description = NULL_IF_CONFIG_SMALL("Copy the input audio unchanged to the output."),
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(acopy_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
};
