/*
 * Copyright (c) 2016 Floris Sluiter
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
 * Pixel remap filter
 * This filter copies pixel by pixel a source frame to a target frame.
 * It remaps the pixels to a new x,y destination based on two files ymap/xmap.
 * Map files are passed as a parameter and are in PGM format (P2 or P5),
 * where the values are y(rows)/x(cols) coordinates of the source_frame.
 * The *target* frame dimension is based on mapfile dimensions: specified in the
 * header of the mapfile and reflected in the number of datavalues.
 * Dimensions of ymap and xmap must be equal. Datavalues must be positive or zero.
 * Any datavalue in the ymap or xmap which value is higher
 * then the *source* frame height or width is silently ignored, leaving a
 * blank/chromakey pixel. This can safely be used as a feature to create overlays.
 *
 * Algorithm digest:
 * Target_frame[y][x] = Source_frame[ ymap[y][x] ][ [xmap[y][x] ];
 */

#include "libavutil/colorspace.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "drawutils.h"
#include "filters.h"
#include "formats.h"
#include "framesync.h"
#include "video.h"

typedef struct RemapContext {
    const AVClass *class;
    int format;

    int nb_planes;
    int nb_components;
    int step;
    int is_rgb;
    int factor;
    uint8_t rgba_map[4];
    uint8_t fill_rgba[4];
    int fill_color[4];

    FFFrameSync fs;

    AVFrame *frame_source, *frame_xmap, *frame_ymap;

    int (*remap_slice)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} RemapContext;

#define OFFSET(x) offsetof(RemapContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define TFLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption remap_options[] = {
    { "format", "set output format", OFFSET(format), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, FLAGS, .unit = "format" },
        { "color",  "", 0, AV_OPT_TYPE_CONST, {.i64=0},   .flags = FLAGS, .unit = "format" },
        { "gray",   "", 0, AV_OPT_TYPE_CONST, {.i64=1},   .flags = FLAGS, .unit = "format" },
    { "fill", "set the color of the unmapped pixels", OFFSET(fill_rgba), AV_OPT_TYPE_COLOR, {.str="black"}, .flags = TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(remap);

typedef struct ThreadData {
    AVFrame *in, *xin, *yin, *out;
    int nb_planes;
    int nb_components;
    int step;
} ThreadData;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const RemapContext *s = ctx->priv;
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUVA444P,
        AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_RGB24, AV_PIX_FMT_BGR24,
        AV_PIX_FMT_ARGB, AV_PIX_FMT_ABGR, AV_PIX_FMT_RGBA, AV_PIX_FMT_BGRA,
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP,
        AV_PIX_FMT_YUV444P9, AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUV444P12,
        AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV444P16,
        AV_PIX_FMT_YUVA444P9, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA444P12, AV_PIX_FMT_YUVA444P16,
        AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10, AV_PIX_FMT_GBRP12,
        AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
        AV_PIX_FMT_GBRAP10, AV_PIX_FMT_GBRAP12, AV_PIX_FMT_GBRAP16,
        AV_PIX_FMT_RGB48, AV_PIX_FMT_BGR48,
        AV_PIX_FMT_RGBA64, AV_PIX_FMT_BGRA64,
        AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat gray_pix_fmts[] = {
        AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9,
        AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12,
        AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
        AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat map_fmts[] = {
        AV_PIX_FMT_GRAY16,
        AV_PIX_FMT_GRAYF32,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *pix_formats = NULL, *map_formats = NULL;
    int ret;

    pix_formats = ff_make_format_list(s->format ? gray_pix_fmts : pix_fmts);
    if ((ret = ff_formats_ref(pix_formats, &cfg_in[0]->formats)) < 0 ||
        (ret = ff_formats_ref(pix_formats, &cfg_out[0]->formats)) < 0)
        return ret;

    map_formats = ff_make_format_list(map_fmts);
    if ((ret = ff_formats_ref(map_formats, &cfg_in[1]->formats)) < 0)
        return ret;
    return ff_formats_ref(map_formats, &cfg_in[2]->formats);
}

#define SOURCE_TYPE 8
#define MAP_TYPE 16
#include "remap_template.c"

#undef SOURCE_TYPE
#undef MAP_TYPE
#define SOURCE_TYPE 8
#define MAP_TYPE 32
#include "remap_template.c"

#undef SOURCE_TYPE
#undef MAP_TYPE
#define SOURCE_TYPE 16
#define MAP_TYPE 16
#include "remap_template.c"

#undef SOURCE_TYPE
#undef MAP_TYPE
#define SOURCE_TYPE 16
#define MAP_TYPE 32
#include "remap_template.c"

static int set_fill_color(AVFilterContext *ctx)
{
    RemapContext *s = ctx->priv;
    const int factor = s->factor;

    if (s->is_rgb) {
        s->fill_color[s->rgba_map[0]] = s->fill_rgba[0] * factor;
        s->fill_color[s->rgba_map[1]] = s->fill_rgba[1] * factor;
        s->fill_color[s->rgba_map[2]] = s->fill_rgba[2] * factor;
        s->fill_color[s->rgba_map[3]] = s->fill_rgba[3] * factor;
    } else {
        s->fill_color[0] = RGB_TO_Y_BT709(s->fill_rgba[0], s->fill_rgba[1], s->fill_rgba[2]) * factor;
        s->fill_color[1] = RGB_TO_U_BT709(s->fill_rgba[0], s->fill_rgba[1], s->fill_rgba[2], 0) * factor;
        s->fill_color[2] = RGB_TO_V_BT709(s->fill_rgba[0], s->fill_rgba[1], s->fill_rgba[2], 0) * factor;
        s->fill_color[3] = s->fill_rgba[3] * factor;
    }

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    RemapContext *s = ctx->priv;
    const AVPixFmtDescriptor *xdesc = av_pix_fmt_desc_get(ctx->inputs[1]->format);
    const AVPixFmtDescriptor *ydesc = av_pix_fmt_desc_get(ctx->inputs[2]->format);
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int depth = desc->comp[0].depth;
    const int is_float_map = !!(xdesc->flags & AV_PIX_FMT_FLAG_FLOAT) &&
                             !!(ydesc->flags & AV_PIX_FMT_FLAG_FLOAT);

    s->factor = 1 << (depth - 8);
    s->is_rgb = !!(desc->flags & AV_PIX_FMT_FLAG_RGB);
    ff_fill_rgba_map(s->rgba_map, inlink->format);
    s->nb_planes = av_pix_fmt_count_planes(inlink->format);
    s->nb_components = desc->nb_components;

    set_fill_color(ctx);

    if (depth == 8) {
        if (s->nb_planes > 1 || s->nb_components == 1) {
            s->remap_slice = is_float_map ? remap_planar8_linear_slice : remap_planar8_nearest_slice;
        } else {
            s->remap_slice = is_float_map ? remap_packed8_linear_slice : remap_packed8_nearest_slice;
        }
    } else {
        if (s->nb_planes > 1 || s->nb_components == 1) {
            s->remap_slice = is_float_map ? remap_planar16_linear_slice : remap_planar16_nearest_slice;
        } else {
            s->remap_slice = is_float_map ? remap_packed16_linear_slice : remap_packed16_nearest_slice;
        }
    }

    s->step = av_get_padded_bits_per_pixel(desc) >> 3;
    return 0;
}

static int process_frame(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    RemapContext *s = fs->opaque;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;
    int ret;

    out = ff_graph_frame_alloc(ctx);
    if (!out)
        return AVERROR(ENOMEM);

    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }
    av_frame_copy_props(out, s->frame_source);

    td.in  = s->frame_source;
    td.xin = s->frame_xmap;
    td.yin = s->frame_ymap;
    td.out = out;
    td.nb_planes = s->nb_planes;
    td.nb_components = s->nb_components;
    td.step = s->step;
    ff_filter_execute(ctx, s->remap_slice, &td, NULL,
                      FFMIN(outlink->h, ff_filter_get_nb_threads(ctx)));
    out->pts = av_rescale_q(s->fs.pts, s->fs.time_base, outlink->time_base);

    return ff_filter_frame(outlink, out);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    RemapContext *s = ctx->priv;
    AVFilterLink *srclink = ctx->inputs[0];
    AVFilterLink *xlink = ctx->inputs[1];
    AVFilterLink *ylink = ctx->inputs[2];
    FilterLink *il = ff_filter_link(srclink);
    FilterLink *ol = ff_filter_link(outlink);
    FFFrameSyncIn *in;
    int ret;

    if (xlink->w != ylink->w || xlink->h != ylink->h) {
        av_log(ctx, AV_LOG_ERROR, "Second input link %s parameters "
               "(size %dx%d) do not match the corresponding "
               "third input link %s parameters (%dx%d)\n",
               ctx->input_pads[1].name, xlink->w, xlink->h,
               ctx->input_pads[2].name, ylink->w, ylink->h);
        return AVERROR(EINVAL);
    }

    outlink->w = xlink->w;
    outlink->h = xlink->h;
    outlink->sample_aspect_ratio = srclink->sample_aspect_ratio;
    ol->frame_rate = il->frame_rate;

    ret = ff_framesync_init(&s->fs, ctx, 3);
    if (ret < 0)
        return ret;

    in = s->fs.in;
    in[0].time_base = srclink->time_base;
    in[1].time_base = xlink->time_base;
    in[2].time_base = ylink->time_base;
    in[0].sync   = 2;
    in[0].before = EXT_STOP;
    in[0].after  = EXT_STOP;
    in[1].sync   = 1;
    in[1].before = EXT_NULL;
    in[1].after  = EXT_INFINITY;
    in[2].sync   = 1;
    in[2].before = EXT_NULL;
    in[2].after  = EXT_INFINITY;
    s->fs.opaque   = s;
    s->fs.on_event = process_frame;

    ret = ff_framesync_configure(&s->fs);
    outlink->time_base = s->fs.time_base;

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    RemapContext *s = ctx->priv;
    return ff_framesync_activate_frames(&s->fs);
}

static int filter_prepare(AVFilterContext *ctx)
{
    RemapContext *s = ctx->priv;
    int ret;

    ret = ff_framesync_filter_prepare(&s->fs);
    if (ret < 0)
        return ret;

    ff_graph_frame_free(ctx, &s->frame_source);
    ff_graph_frame_free(ctx, &s->frame_xmap);
    ff_graph_frame_free(ctx, &s->frame_ymap);

    ret = ff_framesync_get_frame(&s->fs, 0, &s->frame_source, 1);
    if (ret < 0)
        return ret;

    ret = ff_framesync_get_frame(&s->fs, 1, &s->frame_xmap, 1);
    if (ret < 0) {
        av_frame_free(&s->frame_source);
        return ret;
    }

    ret = ff_framesync_get_frame(&s->fs, 2, &s->frame_ymap, 1);
    if (ret < 0) {
        av_frame_free(&s->frame_source);
        av_frame_free(&s->frame_xmap);
        return ret;
    }

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret = ff_filter_process_command(ctx, cmd, arg);

    if (ret < 0)
        return ret;

    return set_fill_color(ctx);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    RemapContext *s = ctx->priv;

    av_frame_free(&s->frame_source);
    av_frame_free(&s->frame_xmap);
    av_frame_free(&s->frame_ymap);

    ff_framesync_uninit(&s->fs);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const RemapContext *s_src = src->priv;
    RemapContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    memcpy(s_dst->fill_color, s_src->fill_color, sizeof(s_src->fill_color));
    s_dst->fs.pts = s_src->fs.pts;

    ff_graph_frame_free(dst, &s_dst->frame_source);
    ff_graph_frame_free(dst, &s_dst->frame_xmap);
    ff_graph_frame_free(dst, &s_dst->frame_ymap);

    s_dst->frame_source = ff_graph_frame_clone(dst, s_src->frame_source);
    s_dst->frame_xmap   = ff_graph_frame_clone(dst, s_src->frame_xmap);
    s_dst->frame_ymap   = ff_graph_frame_clone(dst, s_src->frame_ymap);
    if (!s_dst->frame_source || !s_dst->frame_xmap || !s_dst->frame_ymap)
        return AVERROR(ENOMEM);

    return 0;
}
#endif

static const AVFilterPad remap_inputs[] = {
    {
        .name         = "source",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
    },
    {
        .name         = "xmap",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
    {
        .name         = "ymap",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
};

static const AVFilterPad remap_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_vf_remap = {
    .p.name        = "remap",
    .p.description = NULL_IF_CONFIG_SMALL("Remap pixels."),
    .p.priv_class  = &remap_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_FRAME_THREADS,
    .priv_size     = sizeof(RemapContext),
    .uninit        = uninit,
    .activate      = activate,
    .filter_prepare = filter_prepare,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(remap_inputs),
    FILTER_OUTPUTS(remap_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = process_command,
};
