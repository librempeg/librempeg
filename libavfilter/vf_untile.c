/*
 * Copyright (c) 2020 Nicolas George
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

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "formats.h"
#include "filters.h"
#include "video.h"

typedef struct UntileContext {
    const AVClass *class;
    int w, h;
    unsigned current;
    unsigned nb_frames;
    AVFrame *frame;
    const AVPixFmtDescriptor *desc;
    int64_t dpts, pts;
    int max_step[4];
} UntileContext;

#define OFFSET(x) offsetof(UntileContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption untile_options[] = {
    { "layout", "set grid size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE,
        {.str = "6x5"}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(untile);

static av_cold int init(AVFilterContext *ctx)
{
    UntileContext *s = ctx->priv;

    if (s->w > UINT_MAX / s->h) {
        av_log(ctx, AV_LOG_ERROR, "Tile size %ux%u is insane.\n",
               s->w, s->h);
        return AVERROR(EINVAL);
    }
    s->nb_frames = s->w * s->h;
    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    int reject_flags = AV_PIX_FMT_FLAG_HWACCEL   |
                       AV_PIX_FMT_FLAG_BITSTREAM |
                       FF_PIX_FMT_FLAG_SW_FLAT_SUB;

    return ff_set_common_formats2(ctx, cfg_in, cfg_out,
                                  ff_formats_pixdesc_filter(0, reject_flags));
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    UntileContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    FilterLink *il = ff_filter_link(inlink);
    FilterLink *ol = ff_filter_link(outlink);
    AVRational dt;

    s->desc = av_pix_fmt_desc_get(outlink->format);
    if (inlink->w % (s->w << s->desc->log2_chroma_w) ||
        inlink->h % (s->h << s->desc->log2_chroma_h)) {
        av_log(ctx, AV_LOG_ERROR,
               "Input resolution %ux%u not multiple of layout %ux%u.\n",
               inlink->w, inlink->h, s->w, s->h);
        return AVERROR(EINVAL);
    }
    outlink->w = inlink->w / s->w;
    outlink->h = inlink->h / s->h;
    outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;
    ol->frame_rate = av_mul_q(il->frame_rate, av_make_q(s->nb_frames, 1));
    if (ol->frame_rate.num)
        dt = av_inv_q(ol->frame_rate);
    else
        dt = av_mul_q(inlink->time_base, av_make_q(1, s->nb_frames));
    outlink->time_base = av_gcd_q(inlink->time_base, dt, AV_TIME_BASE / 2, AV_TIME_BASE_Q);
    s->dpts = av_rescale_q(1, dt, outlink->time_base);
    av_log(ctx, AV_LOG_VERBOSE, "frame interval: %"PRId64"*%d/%d\n",
           s->dpts, dt.num, dt.den);
    av_image_fill_max_pixsteps(s->max_step, NULL, s->desc);
    return 0;
}

static int activate(AVFilterContext *ctx)
{
    UntileContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    int i, x, y, ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);
    if (!s->frame) {
        ret = ff_inlink_consume_frame(inlink, &s->frame);
        if (ret < 0)
            return ret;
        if (ret)
            s->pts = av_rescale_q(s->frame->pts, inlink->time_base, outlink->time_base);
    }
    if (s->frame) {
        if (s->current == s->nb_frames - 1) {
            out = s->frame;
            s->frame = NULL;
        } else {
            out = av_frame_clone(s->frame);
            if (!out)
                return AVERROR(ENOMEM);
        }
        x = outlink->w * (s->current % s->w);
        y = outlink->h * (s->current / s->w);
        out->width = outlink->w;
        out->height = outlink->h;
        out->data[0] += y * out->linesize[0];
        out->data[0] += x * s->max_step[0];
        if (!(s->desc->flags & AV_PIX_FMT_FLAG_PAL)) {
            for (i = 1; i < 3; i ++) {
                if (out->data[i]) {
                    out->data[i] += (y >> s->desc->log2_chroma_h) * out->linesize[i];
                    out->data[i] += (x >> s->desc->log2_chroma_w) * s->max_step[i];
                }
            }
        }
        if (out->data[3]) {
            out->data[3] += y * out->linesize[3];
            out->data[3] += x * s->max_step[3];
        }
        out->pts = s->pts;
        s->pts += s->dpts;
        if (++s->current == s->nb_frames)
            s->current = 0;
        return ff_filter_frame(outlink, out);
    }
    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);
    return FFERROR_NOT_READY;

}

static av_cold void uninit(AVFilterContext *ctx)
{
    UntileContext *s = ctx->priv;

    av_frame_free(&s->frame);
}

static const AVFilterPad untile_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_vf_untile = {
    .p.name        = "untile",
    .p.description = NULL_IF_CONFIG_SMALL("Untile a frame into a sequence of frames."),
    .p.priv_class  = &untile_class,
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(UntileContext),
    FILTER_INPUTS(ff_video_default_filterpad),
    FILTER_OUTPUTS(untile_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
