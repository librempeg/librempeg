/*
 * Copyright (c) 2015 Paul B. Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

typedef struct SwapRectContext {
    const AVClass *class;
    int w, h;
    int x1, y1;
    int x2, y2;

    int nb_planes;
    int pixsteps[4];

    const AVPixFmtDescriptor *desc;
    uint8_t *temp;
} SwapRectContext;

#define OFFSET(x) offsetof(SwapRectContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
static const AVOption swaprect_options[] = {
    { "w",  "set rect width",                     OFFSET(w),  AV_OPT_TYPE_INT, {.i64=100}, 1, UINT16_MAX, .flags = FLAGS },
    { "h",  "set rect height",                    OFFSET(h),  AV_OPT_TYPE_INT, {.i64=100}, 1, UINT16_MAX, .flags = FLAGS },
    { "x1", "set 1st rect x top left coordinate", OFFSET(x1), AV_OPT_TYPE_INT, {.i64=100}, 0, UINT16_MAX, .flags = FLAGS },
    { "y1", "set 1st rect y top left coordinate", OFFSET(y1), AV_OPT_TYPE_INT, {.i64=100}, 0, UINT16_MAX, .flags = FLAGS },
    { "x2", "set 2nd rect x top left coordinate", OFFSET(x2), AV_OPT_TYPE_INT, {.i64=0},   0, UINT16_MAX, .flags = FLAGS },
    { "y2", "set 2nd rect y top left coordinate", OFFSET(y2), AV_OPT_TYPE_INT, {.i64=0},   0, UINT16_MAX, .flags = FLAGS },
    { NULL },
};

AVFILTER_DEFINE_CLASS(swaprect);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    int reject_flags = AV_PIX_FMT_FLAG_PAL     |
                       AV_PIX_FMT_FLAG_HWACCEL |
                       AV_PIX_FMT_FLAG_BITSTREAM;

    return ff_set_common_formats2(ctx, cfg_in, cfg_out,
                                  ff_formats_pixdesc_filter(0, reject_flags));
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    SwapRectContext *s = ctx->priv;
    int x1[4], y1[4];
    int x2[4], y2[4];
    int aw[4], ah[4];
    int lw[4], lh[4];
    int pw[4], ph[4];
    int y, p, w, h;

    w = s->w; h = s->h; x1[0] = s->x1; y1[0] = s->y1; x2[0] = s->x2; y2[0] = s->y2;

    x1[0] = av_clip(x1[0], 0, inlink->w - 1);
    y1[0] = av_clip(y1[0], 0, inlink->h - 1);

    x2[0] = av_clip(x2[0], 0, inlink->w - 1);
    y2[0] = av_clip(y2[0], 0, inlink->h - 1);

    ah[1] = ah[2] = AV_CEIL_RSHIFT(h, s->desc->log2_chroma_h);
    ah[0] = ah[3] = h;
    aw[1] = aw[2] = AV_CEIL_RSHIFT(w, s->desc->log2_chroma_w);
    aw[0] = aw[3] = w;

    w = FFMAX(0, FFMIN3(w, inlink->w - x1[0], inlink->w - x2[0]));
    h = FFMAX(0, FFMIN3(h, inlink->h - y1[0], inlink->h - y2[0]));

    ph[1] = ph[2] = AV_CEIL_RSHIFT(h, s->desc->log2_chroma_h);
    ph[0] = ph[3] = h;
    pw[1] = pw[2] = AV_CEIL_RSHIFT(w, s->desc->log2_chroma_w);
    pw[0] = pw[3] = w;

    lh[1] = lh[2] = AV_CEIL_RSHIFT(inlink->h, s->desc->log2_chroma_h);
    lh[0] = lh[3] = inlink->h;
    lw[1] = lw[2] = AV_CEIL_RSHIFT(inlink->w, s->desc->log2_chroma_w);
    lw[0] = lw[3] = inlink->w;

    x1[1] = x1[2] = (x1[0] >> s->desc->log2_chroma_w);
    x1[0] = x1[3] = x1[0];
    y1[1] = y1[2] = (y1[0] >> s->desc->log2_chroma_h);
    y1[0] = y1[3] = y1[0];

    x2[1] = x2[2] = (x2[0] >> s->desc->log2_chroma_w);
    x2[0] = x2[3] = x2[0];
    y2[1] = y2[2] = (y2[0] >> s->desc->log2_chroma_h);
    y2[0] = y2[3] = y2[0];


    av_assert0(FFMAX(x1[1], x2[1]) + pw[1] <= lw[1]);
    av_assert0(FFMAX(y1[1], y2[1]) + ph[1] <= lh[1]);

    for (p = 0; p < s->nb_planes; p++) {
        if (ph[p] == ah[p] && pw[p] == aw[p]) {
            uint8_t *src = in->data[p] + y1[p] * in->linesize[p] + x1[p] * s->pixsteps[p];
            uint8_t *dst = in->data[p] + y2[p] * in->linesize[p] + x2[p] * s->pixsteps[p];

            for (y = 0; y < ph[p]; y++) {
                memcpy(s->temp, src, pw[p] * s->pixsteps[p]);
                memmove(src, dst, pw[p] * s->pixsteps[p]);
                memcpy(dst, s->temp, pw[p] * s->pixsteps[p]);
                src += in->linesize[p];
                dst += in->linesize[p];
            }
        }
    }

    return ff_filter_frame(outlink, in);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SwapRectContext *s = ctx->priv;

    s->desc = av_pix_fmt_desc_get(inlink->format);
    av_image_fill_max_pixsteps(s->pixsteps, NULL, s->desc);
    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    s->temp = av_malloc_array(inlink->w, s->pixsteps[0]);
    if (!s->temp)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SwapRectContext *s = ctx->priv;

    av_freep(&s->temp);
}

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .flags          = AVFILTERPAD_FLAG_NEEDS_WRITABLE,
        .filter_frame   = filter_frame,
        .config_props   = config_input,
    },
};

const FFFilter ff_vf_swaprect = {
    .p.name        = "swaprect",
    .p.description = NULL_IF_CONFIG_SMALL("Swap 2 rectangular objects in video."),
    .p.priv_class  = &swaprect_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .priv_size     = sizeof(SwapRectContext),
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
