/*
 * Copyright (c) 2013 Paul B Mahol
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config_components.h"

#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "drawutils.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

#define PLANE_R 0x01
#define PLANE_G 0x02
#define PLANE_B 0x04
#define PLANE_A 0x08
#define PLANE_Y 0x10
#define PLANE_U 0x20
#define PLANE_V 0x40

typedef struct ExtractPlanesContext {
    const AVClass *class;
    int requested_planes;
    int map[4];
    int linesize[4];
    int is_packed;
    int is_bayer;

    int plane[4];
    int step[4];
    int shift[4];
    int depth[4];
    int offset[4];
} ExtractPlanesContext;

#define OFFSET(x) offsetof(ExtractPlanesContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption extractplanes_options[] = {
    { "planes", "set planes",  OFFSET(requested_planes), AV_OPT_TYPE_FLAGS, {.i64=1}, 1, 0xff, FLAGS, .unit = "flags"},
    {      "y", "set luma plane",  0, AV_OPT_TYPE_CONST, {.i64=PLANE_Y}, 0, 0, FLAGS, .unit = "flags"},
    {      "u", "set u plane",     0, AV_OPT_TYPE_CONST, {.i64=PLANE_U}, 0, 0, FLAGS, .unit = "flags"},
    {      "v", "set v plane",     0, AV_OPT_TYPE_CONST, {.i64=PLANE_V}, 0, 0, FLAGS, .unit = "flags"},
    {      "r", "set red plane",   0, AV_OPT_TYPE_CONST, {.i64=PLANE_R}, 0, 0, FLAGS, .unit = "flags"},
    {      "g", "set green plane", 0, AV_OPT_TYPE_CONST, {.i64=PLANE_G}, 0, 0, FLAGS, .unit = "flags"},
    {      "b", "set blue plane",  0, AV_OPT_TYPE_CONST, {.i64=PLANE_B}, 0, 0, FLAGS, .unit = "flags"},
    {      "a", "set alpha plane", 0, AV_OPT_TYPE_CONST, {.i64=PLANE_A}, 0, 0, FLAGS, .unit = "flags"},
    { NULL }
};

AVFILTER_DEFINE_CLASS(extractplanes);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    AVFilterFormats *formats = NULL;
    const AVPixFmtDescriptor *desc;
    int ret;

    for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
        if (!(desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
              desc->flags & AV_PIX_FMT_FLAG_BITSTREAM ||
              desc->flags & AV_PIX_FMT_FLAG_PAL)) {
            ret = ff_add_format(&formats, fmt);
            if (ret < 0)
                return ret;
        }
    }

    formats->flags = FILTER_SAME_BITDEPTH | FILTER_SAME_ENDIANNESS;
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = NULL;
    for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
        if (!(desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
              desc->flags & AV_PIX_FMT_FLAG_BITSTREAM ||
              desc->flags & AV_PIX_FMT_FLAG_PAL) && desc->nb_components == 1) {
            ret = ff_add_format(&formats, fmt);
            if (ret < 0)
                return ret;
        }
    }

    formats->flags = FILTER_SAME_BITDEPTH | FILTER_SAME_ENDIANNESS;
    for (int i = 0; i < ctx->nb_outputs; i++)
        if ((ret = ff_formats_ref(formats, &cfg_out[i]->formats)) < 0)
            return ret;
    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ExtractPlanesContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int plane_avail, ret;

    plane_avail = ((desc->flags & AV_PIX_FMT_FLAG_RGB) ? PLANE_R|PLANE_G|PLANE_B :
                                                 PLANE_Y |
                                ((desc->nb_components > 2) ? PLANE_U|PLANE_V : 0)) |
                  ((desc->flags & AV_PIX_FMT_FLAG_ALPHA) ? PLANE_A : 0);
    if (s->requested_planes & ~plane_avail) {
        av_log(ctx, AV_LOG_ERROR, "Requested planes not available.\n");
        return AVERROR(EINVAL);
    }
    if ((ret = av_image_fill_linesizes(s->linesize, inlink->format, inlink->w)) < 0)
        return ret;

    s->plane[0] = desc->comp[0].plane;
    s->plane[1] = desc->comp[1].plane;
    s->plane[2] = desc->comp[2].plane;
    s->plane[3] = desc->comp[3].plane;

    s->step[0] = desc->comp[0].step;
    s->step[1] = desc->comp[1].step;
    s->step[2] = desc->comp[2].step;
    s->step[3] = desc->comp[3].step;

    s->shift[0] = desc->comp[0].shift;
    s->shift[1] = desc->comp[1].shift;
    s->shift[2] = desc->comp[2].shift;
    s->shift[3] = desc->comp[3].shift;

    s->depth[0] = desc->comp[0].depth;
    s->depth[1] = desc->comp[1].depth;
    s->depth[2] = desc->comp[2].depth;
    s->depth[3] = desc->comp[3].depth;

    s->offset[0] = desc->comp[0].offset;
    s->offset[1] = desc->comp[1].offset;
    s->offset[2] = desc->comp[2].offset;
    s->offset[3] = desc->comp[3].offset;

    s->is_packed = !(desc->flags & AV_PIX_FMT_FLAG_PLANAR) &&
                    (desc->nb_components > 1);
    s->is_bayer = !!(desc->flags & AV_PIX_FMT_FLAG_BAYER);

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ExtractPlanesContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    const int output = outlink->srcpad - ctx->output_pads;

    if (s->is_bayer) {
        const int idx = s->map[FF_OUTLINK_IDX(outlink)];

        outlink->h = inlink->h >> (idx != 1);
        outlink->w = inlink->w >> 1;
    } else if (s->map[output] == 1 || s->map[output] == 2) {
        outlink->h = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
        outlink->w = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    }

    return 0;
}

static void extract_from_bayer(uint8_t *dst, int dst_linesize,
                               const uint8_t *src, int src_linesize,
                               int width, int height, const int idx,
                               const int format)
{
    int x_off, y_off;

    switch (format) {
    case AV_PIX_FMT_BAYER_RGGB8:
    case AV_PIX_FMT_BAYER_RGGB16BE:
    case AV_PIX_FMT_BAYER_RGGB16LE:
        switch (idx) {
        case 0: x_off = 0; y_off = 0; break;
        case 1: x_off = 1; y_off = 0; break;
        case 2: x_off = 1; y_off = 1; break;
        }
    case AV_PIX_FMT_BAYER_BGGR8:
    case AV_PIX_FMT_BAYER_BGGR16BE:
    case AV_PIX_FMT_BAYER_BGGR16LE:
        switch (idx) {
        case 0: x_off = 1; y_off = 1; break;
        case 1: x_off = 1; y_off = 0; break;
        case 2: x_off = 0; y_off = 0; break;
        }
    case AV_PIX_FMT_BAYER_GRBG8:
    case AV_PIX_FMT_BAYER_GRBG16BE:
    case AV_PIX_FMT_BAYER_GRBG16LE:
        switch (idx) {
        case 0: x_off = 1; y_off = 0; break;
        case 1: x_off = 0; y_off = 0; break;
        case 2: x_off = 0; y_off = 1; break;
        }
    case AV_PIX_FMT_BAYER_GBRG8:
    case AV_PIX_FMT_BAYER_GBRG16BE:
    case AV_PIX_FMT_BAYER_GBRG16LE:
        switch (idx) {
        case 0: x_off = 0; y_off = 1; break;
        case 1: x_off = 0; y_off = 0; break;
        case 2: x_off = 1; y_off = 0; break;
        }
    }

    if (y_off)
        src += src_linesize;

    for (int y = 0; y < height; y++) {
        switch (format) {
        case AV_PIX_FMT_BAYER_BGGR16LE:
        case AV_PIX_FMT_BAYER_BGGR16BE:
        case AV_PIX_FMT_BAYER_RGGB16LE:
        case AV_PIX_FMT_BAYER_RGGB16BE:
            for (int x = 0; x < width; x++)
                AV_WN16(dst + x * 2, AV_RN16(src + x * 4 + x_off * 2));
            dst += dst_linesize;
            src += src_linesize;

            if (idx != 1) {
                src += src_linesize;
            } else {
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, AV_RN16(src + x * 4));
                dst += dst_linesize;
                src += src_linesize;
                y++;
            }
            break;
        case AV_PIX_FMT_BAYER_BGGR8:
        case AV_PIX_FMT_BAYER_RGGB8:
            for (int x = 0; x < width; x++)
                dst[x] = src[x * 2 + x_off];
            dst += dst_linesize;
            src += src_linesize;

            if (idx != 1) {
                src += src_linesize;
            } else {
                for (int x = 0; x < width; x++)
                    dst[x] = src[x * 2];
                dst += dst_linesize;
                src += src_linesize;
                y++;
            }
            break;
        case AV_PIX_FMT_BAYER_GBRG16LE:
        case AV_PIX_FMT_BAYER_GBRG16BE:
        case AV_PIX_FMT_BAYER_GRBG16LE:
        case AV_PIX_FMT_BAYER_GRBG16BE:
            for (int x = 0; x < width; x++)
                AV_WN16(dst + x * 2, AV_RN16(src + x * 4 + x_off * 2));
            dst += dst_linesize;
            src += src_linesize;

            if (idx != 1) {
                src += src_linesize;
            } else {
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, AV_RN16(src + x * 4 + 2));
                dst += dst_linesize;
                src += src_linesize;
                y++;
            }
            break;
        case AV_PIX_FMT_BAYER_GRBG8:
        case AV_PIX_FMT_BAYER_GBRG8:
            for (int x = 0; x < width; x++)
                dst[x] = src[x * 2 + x_off];
            dst += dst_linesize;
            src += src_linesize;

            if (idx != 1) {
                src += src_linesize;
            } else {
                for (int x = 0; x < width; x++)
                    dst[x] = src[x * 2 + 1];
                dst += dst_linesize;
                src += src_linesize;
                y++;
            }
            break;
        }
    }
}

static void extract_from_packed(uint8_t *dst, int dst_linesize,
                                const uint8_t *src, int src_linesize,
                                int width, int height,
                                int depth, int shift, int step, int offset)
{
    for (int y = 0; y < height; y++) {
        switch (step) {
        case 1:
            for (int x = 0; x < width; x++)
                dst[x] = (src[x + offset] >> shift) << (8 - depth);
            break;
        case 2:
            switch (depth) {
            case 4:
            case 5:
            case 6:
            case 8:
                for (int x = 0; x < width; x++)
                    dst[x] = (AV_RN16(src + x * 2 + offset) >> shift) << (8 - depth);
                break;
            case 10:
            case 12:
            case 16:
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, (AV_RN16(src + x * 2 + offset) >> shift)&((1<<depth)-1));
                break;
            }
            break;
        case 3:
            switch (depth) {
            case 8:
                for (int x = 0; x < width; x++)
                    dst[x] = (src[x * 3 + offset] >> shift) << (8 - depth);
                break;
            }
            break;
        case 4:
            switch (depth) {
            case 8:
                for (int x = 0; x < width; x++)
                    dst[x] = (src[x * 4 + offset] >> shift) << (8 - depth);
                break;
            case 10:
            case 12:
            case 16:
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, (AV_RN16(src + x * 4 + offset) >> shift)&((1<<depth)-1));
                break;
            }
            break;
        case 6:
            switch (depth) {
            case 8:
                for (int x = 0; x < width; x++)
                    dst[x] = (src[x * 6 + offset] >> shift) << (8 - depth);
                break;
            case 10:
            case 12:
            case 16:
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, (AV_RL16(src + x * 6 + offset) >> shift)&((1<<depth)-1));
                break;
            }
            break;
        case 8:
            switch (depth) {
            case 10:
            case 12:
            case 16:
                for (int x = 0; x < width; x++)
                    AV_WN16(dst + x * 2, (AV_RN16(src + x * 8 + offset) >> shift)&((1<<depth)-1));
                break;
            }
            break;
        }
        dst += dst_linesize;
        src += src_linesize;
    }
}

static void extract_from_planar(uint8_t *dst, int dst_linesize,
                                const uint8_t *src, int src_linesize,
                                int linesize, int height,
                                int step, int shift, int offset, int depth)
{
    if (step != ((depth + 7) >> 3) || shift != 0) {
        for (int y = 0; y < height; y++) {
            switch (step) {
            case 2:
                switch (depth) {
                case 8:
                    for (int x = 0; x < linesize; x += 2)
                        dst[x>>1] = src[x + offset] >> shift;
                    break;
                case 10:
                case 12:
                case 16:
                    for (int x = 0; x < linesize; x += 2)
                        AV_WN16(dst + (x>>1) * 2, AV_RN16(src + x + offset) >> shift);
                    break;
                }
                break;
            case 4:
                switch (depth) {
                case 10:
                case 12:
                case 16:
                    for (int x = 0; x < linesize; x += 4)
                        AV_WN16(dst + (x>>2) * 2, AV_RN16(src + x + offset) >> shift);
                    break;
                }
                break;
            }
            dst += dst_linesize;
            src += src_linesize;
        }
    } else {
        av_image_copy_plane(dst, dst_linesize, src, src_linesize,
                            linesize, height);
    }
}

static int extract_plane(AVFilterLink *outlink, AVFrame *frame)
{
    AVFilterContext *ctx = outlink->src;
    ExtractPlanesContext *s = ctx->priv;
    const int idx = s->map[FF_OUTLINK_IDX(outlink)];
    AVFrame *out;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out)
        return AVERROR(ENOMEM);
    av_frame_copy_props(out, frame);
    if (idx == 3 /* alpha */)
        out->color_range = AVCOL_RANGE_JPEG;

    if (s->is_bayer) {
        extract_from_bayer(out->data[0], out->linesize[0],
                           frame->data[0], frame->linesize[0],
                           outlink->w, outlink->h, idx, frame->format);
    } else if (s->is_packed) {
        extract_from_packed(out->data[0], out->linesize[0],
                            frame->data[0], frame->linesize[0],
                            outlink->w, outlink->h,
                            s->depth[idx], s->shift[idx],
                            s->step[idx], s->offset[idx]);
    } else {
        extract_from_planar(out->data[0], out->linesize[0],
                            frame->data[s->plane[idx]],
                            frame->linesize[s->plane[idx]],
                            s->linesize[s->plane[idx]],
                            outlink->h, s->step[idx], s->shift[idx],
                            s->offset[idx], s->depth[idx]);
    }

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    int status, ret;
    AVFrame *in;
    int64_t pts;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[i], ctx);
    }

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;

            ret = extract_plane(ctx->outputs[i], in);
            if (ret < 0)
                break;
        }

        av_frame_free(&in);
        if (ret < 0)
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

static av_cold int init(AVFilterContext *ctx)
{
    ExtractPlanesContext *s = ctx->priv;
    int planes = (s->requested_planes & 0xf) | (s->requested_planes >> 4);
    int i, ret;

    for (i = 0; i < 4; i++) {
        char *name;
        AVFilterPad pad = { 0 };

        if (!(planes & (1 << i)))
            continue;

        name = av_asprintf("out%d", ctx->nb_outputs);
        if (!name)
            return AVERROR(ENOMEM);
        s->map[ctx->nb_outputs] = i;
        pad.name = name;
        pad.type = AVMEDIA_TYPE_VIDEO;
        pad.config_props = config_output;

        if ((ret = ff_append_outpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    return 0;
}

static const AVFilterPad extractplanes_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_extractplanes = {
    .p.name        = "extractplanes",
    .p.description = NULL_IF_CONFIG_SMALL("Extract planes as grayscale frames."),
    .p.priv_class  = &extractplanes_class,
    .p.flags       = AVFILTER_FLAG_DYNAMIC_OUTPUTS,
    .priv_size     = sizeof(ExtractPlanesContext),
    .init          = init,
    .activate      = activate,
    FILTER_INPUTS(extractplanes_inputs),
    .p.outputs     = NULL,
    FILTER_QUERY_FUNC2(query_formats),
};

#if CONFIG_ALPHAEXTRACT_FILTER

static av_cold int init_alphaextract(AVFilterContext *ctx)
{
    ExtractPlanesContext *s = ctx->priv;

    s->requested_planes = PLANE_A;
    s->map[0] = 3;

    return 0;
}

static const AVFilterPad alphaextract_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_alphaextract = {
    .p.name         = "alphaextract",
    .p.description  = NULL_IF_CONFIG_SMALL("Extract an alpha channel as a "
                      "grayscale image component."),
    .priv_size      = sizeof(ExtractPlanesContext),
    .init           = init_alphaextract,
    .activate       = activate,
    FILTER_INPUTS(extractplanes_inputs),
    FILTER_OUTPUTS(alphaextract_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif  /* CONFIG_ALPHAEXTRACT_FILTER */
