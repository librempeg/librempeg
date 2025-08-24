/*
 * Copyright (c) 2002 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2013 Paul B Mahol
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
 * (de)interleave fields filter
 */

#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

enum FilterMode {
    MODE_NONE,
    MODE_INTERLEAVE,
    MODE_DEINTERLEAVE
};

typedef struct IlContext {
    const AVClass *class;
    int luma_mode, chroma_mode, alpha_mode; ///<FilterMode
    int luma_swap, chroma_swap, alpha_swap;
    int nb_planes;
    int linesize[4], chroma_height;
    int has_alpha;
} IlContext;

#define OFFSET(x) offsetof(IlContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption il_options[] = {
    {"luma_mode",   "select luma mode", OFFSET(luma_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"l",           "select luma mode", OFFSET(luma_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"none",         NULL, 0, AV_OPT_TYPE_CONST, {.i64=MODE_NONE},         0, 0, FLAGS, .unit = "mode"},
    {"interleave",   NULL, 0, AV_OPT_TYPE_CONST, {.i64=MODE_INTERLEAVE},   0, 0, FLAGS, .unit = "mode"},
    {"i",            NULL, 0, AV_OPT_TYPE_CONST, {.i64=MODE_INTERLEAVE},   0, 0, FLAGS, .unit = "mode"},
    {"deinterleave", NULL, 0, AV_OPT_TYPE_CONST, {.i64=MODE_DEINTERLEAVE}, 0, 0, FLAGS, .unit = "mode"},
    {"d",            NULL, 0, AV_OPT_TYPE_CONST, {.i64=MODE_DEINTERLEAVE}, 0, 0, FLAGS, .unit = "mode"},
    {"chroma_mode", "select chroma mode", OFFSET(chroma_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"c",           "select chroma mode", OFFSET(chroma_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"alpha_mode", "select alpha mode", OFFSET(alpha_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"a",          "select alpha mode", OFFSET(alpha_mode), AV_OPT_TYPE_INT, {.i64=MODE_NONE}, MODE_NONE, MODE_DEINTERLEAVE, FLAGS, .unit = "mode"},
    {"luma_swap",   "swap luma fields",   OFFSET(luma_swap),   AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {"ls",          "swap luma fields",   OFFSET(luma_swap),   AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {"chroma_swap", "swap chroma fields", OFFSET(chroma_swap), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {"cs",          "swap chroma fields", OFFSET(chroma_swap), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {"alpha_swap",  "swap alpha fields",  OFFSET(alpha_swap),  AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {"as",          "swap alpha fields",  OFFSET(alpha_swap),  AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    {NULL}
};

AVFILTER_DEFINE_CLASS(il);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    int reject_flags = AV_PIX_FMT_FLAG_PAL | AV_PIX_FMT_FLAG_HWACCEL;

    return ff_set_common_formats2(ctx, cfg_in, cfg_out,
                                  ff_formats_pixdesc_filter(0, reject_flags));
}

static int config_input(AVFilterLink *inlink)
{
    IlContext *s = inlink->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int ret;

    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    s->has_alpha = !!(desc->flags & AV_PIX_FMT_FLAG_ALPHA);
    if ((ret = av_image_fill_linesizes(s->linesize, inlink->format, inlink->w)) < 0)
        return ret;

    s->chroma_height = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

    return 0;
}

static void interleave(uint8_t *dst, uint8_t *src, int w, int h,
                       int dst_linesize, int src_linesize,
                       enum FilterMode mode, int swap)
{
    const int a = swap;
    const int b = 1 - a;
    const int m = h >> 1;
    int y;

    switch (mode) {
    case MODE_DEINTERLEAVE:
        for (y = 0; y < m; y++) {
            memcpy(dst + dst_linesize *  y     , src + src_linesize * (y * 2 + a), w);
            memcpy(dst + dst_linesize * (y + m), src + src_linesize * (y * 2 + b), w);
        }
        break;
    case MODE_NONE:
        for (y = 0; y < m; y++) {
            memcpy(dst + dst_linesize *  y * 2     , src + src_linesize * (y * 2 + a), w);
            memcpy(dst + dst_linesize * (y * 2 + 1), src + src_linesize * (y * 2 + b), w);
        }
        break;
    case MODE_INTERLEAVE:
        for (y = 0; y < m; y++) {
            memcpy(dst + dst_linesize * (y * 2 + a), src + src_linesize *  y     , w);
            memcpy(dst + dst_linesize * (y * 2 + b), src + src_linesize * (y + m), w);
        }
        break;
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *inpicref)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    IlContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_graph_frame_alloc(ctx);
    if (!out) {
        av_frame_free(&inpicref);
        return AVERROR(ENOMEM);
    }

    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&inpicref);
        return ret;
    }

    av_frame_copy_props(out, inpicref);

    interleave(out->data[0], inpicref->data[0],
               s->linesize[0], inlink->h,
               out->linesize[0], inpicref->linesize[0],
               s->luma_mode, s->luma_swap);

    for (int comp = 1; comp < (s->nb_planes - s->has_alpha); comp++) {
        interleave(out->data[comp], inpicref->data[comp],
                   s->linesize[comp], s->chroma_height,
                   out->linesize[comp], inpicref->linesize[comp],
                   s->chroma_mode, s->chroma_swap);
    }

    if (s->has_alpha) {
        int comp = s->nb_planes - 1;
        interleave(out->data[comp], inpicref->data[comp],
                   s->linesize[comp], inlink->h,
                   out->linesize[comp], inpicref->linesize[comp],
                   s->alpha_mode, s->alpha_swap);
    }

    ff_graph_frame_free(ctx, &inpicref);
    return ff_filter_frame(outlink, out);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const IlContext *s_src = src->priv;
    IlContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    s_dst->luma_mode = s_src->luma_mode;
    s_dst->chroma_mode = s_src->chroma_mode;
    s_dst->alpha_mode = s_src->alpha_mode;
    s_dst->luma_swap = s_src->luma_swap;
    s_dst->chroma_swap = s_src->chroma_swap;
    s_dst->alpha_swap = s_src->alpha_swap;

    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_il = {
    .p.name        = "il",
    .p.description = NULL_IF_CONFIG_SMALL("Deinterleave or interleave fields."),
    .p.priv_class  = &il_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_FRAME_THREADS,
    .priv_size     = sizeof(IlContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
