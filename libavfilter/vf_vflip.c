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
 * video vertical flip filter
 */

#include "libavutil/internal.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct FlipContext {
    int vsub;   ///< vertical chroma subsampling
    int bayer;
} FlipContext;

static int config_input(AVFilterLink *link)
{
    FlipContext *flip = link->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(link->format);

    flip->vsub = desc->log2_chroma_h;
    flip->bayer = !!(desc->flags & AV_PIX_FMT_FLAG_BAYER);

    return 0;
}

static AVFrame *get_video_buffer(AVFilterLink *link, int w, int h)
{
    FlipContext *flip = link->dst->priv;
    AVFrame *frame;
    int i;

    frame = ff_get_video_buffer(link->dst->outputs[0], w, h);
    if (!frame)
        return NULL;

    for (i = 0; i < 4; i ++) {
        int vsub = i == 1 || i == 2 ? flip->vsub : 0;
        int height = AV_CEIL_RSHIFT(h, vsub);

        if (frame->data[i]) {
            frame->data[i] += (height - 1) * frame->linesize[i];
            frame->linesize[i] = -frame->linesize[i];
        }
    }

    return frame;
}

static int flip_bayer(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx  = link->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    uint8_t *inrow = in->data[0], *outrow;
    int i, width = outlink->w << (av_pix_fmt_desc_get(link->format)->comp[0].step > 1);
    int ret;
    if (outlink->h & 1) {
        av_log(ctx, AV_LOG_ERROR, "Bayer vertical flip needs even height\n");
        return AVERROR_INVALIDDATA;
    }

    out = av_frame_alloc();
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        return ret;
    }

    av_frame_copy_props(out, in);
    outrow = out->data[0] + out->linesize[0] * (outlink->h - 2);
    for (i = 0; i < outlink->h >> 1; i++) {
        memcpy(outrow, inrow, width);
        memcpy(outrow + out->linesize[0], inrow + in->linesize[0], width);
        inrow  += 2 *  in->linesize[0];
        outrow -= 2 * out->linesize[0];
    }
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int filter_frame(AVFilterLink *link, AVFrame *frame)
{
    FlipContext *flip = link->dst->priv;
    int i;

    if (flip->bayer)
        return flip_bayer(link, frame);

    for (i = 0; i < 4; i ++) {
        int vsub = i == 1 || i == 2 ? flip->vsub : 0;
        int height = AV_CEIL_RSHIFT(link->h, vsub);

        if (frame->data[i]) {
            frame->data[i] += (height - 1) * frame->linesize[i];
            frame->linesize[i] = -frame->linesize[i];
        }
    }

    return ff_filter_frame(link->dst->outputs[0], frame);
}
static const AVFilterPad inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .get_buffer.video = get_video_buffer,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
    },
};

const FFFilter ff_vf_vflip = {
    .p.name        = "vflip",
    .p.description = NULL_IF_CONFIG_SMALL("Flip the input video vertically."),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_FRAME_THREADS,
    .priv_size   = sizeof(FlipContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};
