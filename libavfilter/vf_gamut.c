/*
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

#include "libavutil/colorspace.h"
#include "libavutil/opt.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"

typedef struct GamutContext {
    const AVClass *class;

    uint8_t gamut_rgba[4];
    int gamut_yuv[4];

    int depth;
    int hsub_log2;
    int vsub_log2;

    int (*do_slice)(AVFilterContext *ctx, void *arg,
                    int jobnr, int nb_jobs);
} GamutContext;

static av_always_inline void get_pixel(uint8_t *data[4],
                                       const int linesize[4],
                                       int hsub_log2, int vsub_log2,
                                       int x, int y,
                                       const int width, const int height,
                                       int *cy, int *cu, int *cv, int *ca)
{
    *cy = data[0][linesize[0] * y + x];
    *ca = data[3][linesize[3] * y + x];

    x >>= hsub_log2;
    y >>= vsub_log2;

    *cu = data[1][linesize[1] * y + x];
    *cv = data[2][linesize[2] * y + x];
}

static av_always_inline void get_pixel16(uint8_t *data[4],
                                         const int linesize[4],
                                         int hsub_log2, int vsub_log2,
                                         int x, int y,
                                         const int width, const int height,
                                         int *cy, int *cu, int *cv, int *ca)
{
    *cy = AV_RN16(data[0] + linesize[0] * y + x * 2);
    *ca = AV_RN16(data[3] + linesize[3] * y + x * 2);

    x >>= hsub_log2;
    y >>= vsub_log2;

    *cu = AV_RN16(data[1] + linesize[1] * y + x * 2);
    *cv = AV_RN16(data[2] + linesize[2] * y + x * 2);
}

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

static int is_inside(float y, float u, float v)
{
    float r = y + 1.5747f * (v - 0.5f);
    float g = y - 0.4682f * (v - 0.5f) - 0.1873f * (u - 0.5f);
    float b = y + 1.8556f * (u - 0.5f);

    return (r >= 0.f) && (r <= 1.f) && (g >= 0.f) && (g <= 1.f) && (b >= 0.f) && (b <= 1.f);
}

static int do_gamut_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int slice_start = (in->height * jobnr) / nb_jobs;
    const int slice_end = (in->height * (jobnr + 1)) / nb_jobs;
    const int height = in->height;
    const int width = in->width;
    GamutContext *s = ctx->priv;
    const float factor = 1.f / ((1 << s->depth)-1);
    const int hsub_log2 = s->hsub_log2;
    const int vsub_log2 = s->vsub_log2;
    const int *gamut_yuv = s->gamut_yuv;
    const int *out_linesize = out->linesize;
    uint8_t **out_data = out->data;

    for (int y = slice_start; y < slice_end; y++) {
        const int sy = y >> vsub_log2;

        for (int x = 0; x < width; x++) {
            const int sx = x >> hsub_log2;
            int cy, cu, cv, ca, valid;

            get_pixel(in->data, in->linesize,
                      hsub_log2, vsub_log2, x, y,
                      width, height, &cy, &cu, &cv, &ca);

            valid = is_inside(cy * factor, cu * factor, cv * factor);

            out_data[0][out_linesize[0] *  y +  x] = valid ? cy : gamut_yuv[0];
            out_data[1][out_linesize[1] * sy + sx] = valid ? cu : gamut_yuv[1];
            out_data[2][out_linesize[2] * sy + sx] = valid ? cv : gamut_yuv[2];
            out_data[3][out_linesize[3] *  y +  x] = valid ? ca : gamut_yuv[3];
        }
    }

    return 0;
}

static int do_gamut16_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int slice_start = (in->height * jobnr) / nb_jobs;
    const int slice_end = (in->height * (jobnr + 1)) / nb_jobs;
    const int height = in->height;
    const int width = in->width;
    GamutContext *s = ctx->priv;
    const float factor = 1.f / ((1 << s->depth)-1);
    const int hsub_log2 = s->hsub_log2;
    const int vsub_log2 = s->vsub_log2;
    const int *gamut_yuv = s->gamut_yuv;
    const int *out_linesize = out->linesize;
    uint8_t **out_data = out->data;

    for (int y = slice_start; y < slice_end; y++) {
        const int sy = y >> vsub_log2;

        for (int x = 0; x < width; x++) {
            const int sx = x >> hsub_log2;
            int cy, cu, cv, ca, valid;

            get_pixel16(in->data, in->linesize,
                        hsub_log2, vsub_log2, x, y,
                        width, height, &cy, &cu, &cv, &ca);

            valid  = is_inside(cy * factor, cu * factor, cv * factor);

            AV_WN16(out_data[0] + out_linesize[0] *  y +  x*2, valid ? cy : gamut_yuv[0]);
            AV_WN16(out_data[1] + out_linesize[1] * sy + sx*2, valid ? cu : gamut_yuv[1]);
            AV_WN16(out_data[2] + out_linesize[2] * sy + sx*2, valid ? cv : gamut_yuv[2]);
            AV_WN16(out_data[3] + out_linesize[3] *  y +  x*2, valid ? ca : gamut_yuv[3]);
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    GamutContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;
    int ret;

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

    td.out = out, td.in = in;
    if (ret = ff_filter_execute(ctx, s->do_slice, &td, NULL,
                                FFMIN(out->height, ff_filter_get_nb_threads(ctx))))
        return ret;

    return ff_filter_frame(ctx->outputs[0], out);
}

static av_cold int config_output(AVFilterLink *outlink)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    AVFilterContext *ctx = outlink->src;
    GamutContext *s = ctx->priv;
    int factor;

    s->depth = desc->comp[0].depth;

    factor = 1 << (s->depth - 8);

    s->gamut_yuv[0] = RGB_TO_Y_BT709(s->gamut_rgba[0], s->gamut_rgba[1], s->gamut_rgba[2]) * factor;
    s->gamut_yuv[1] = RGB_TO_U_BT709(s->gamut_rgba[0], s->gamut_rgba[1], s->gamut_rgba[2], 0) * factor;
    s->gamut_yuv[2] = RGB_TO_V_BT709(s->gamut_rgba[0], s->gamut_rgba[1], s->gamut_rgba[2], 0) * factor;
    s->gamut_yuv[3] = s->gamut_rgba[3] * factor;

    s->do_slice = s->depth <= 8 ? do_gamut_slice : do_gamut16_slice;

    return 0;
}

static av_cold int config_input(AVFilterLink *inlink)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    AVFilterContext *ctx = inlink->dst;
    GamutContext *s = ctx->priv;

    s->hsub_log2 = desc->log2_chroma_w;
    s->vsub_log2 = desc->log2_chroma_h;

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    return config_output(ctx->outputs[0]);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const GamutContext *s_src = src->priv;
    GamutContext       *s_dst = dst->priv;

    // only transfer state from main thread to workers
    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    memcpy(s_dst->gamut_yuv, s_src->gamut_yuv, sizeof(s_src->gamut_yuv));

    return 0;
}
#endif

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .filter_frame   = filter_frame,
        .config_props   = config_input,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .config_props   = config_output,
    },
};

#define OFFSET(x) offsetof(GamutContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption gamut_options[] = {
    { "color", "set the gamut key color", OFFSET(gamut_rgba), AV_OPT_TYPE_COLOR, { .str="black@0" }, 0, 0, FLAGS },
    { NULL }
};

static const enum AVPixelFormat gamut_fmts[] = {
    AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_YUVA422P,
    AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA420P9,  AV_PIX_FMT_YUVA422P9,  AV_PIX_FMT_YUVA444P9,
    AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA444P10,
    AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA444P12,
    AV_PIX_FMT_YUVA420P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA444P16,
    AV_PIX_FMT_NONE
};

AVFILTER_DEFINE_CLASS(gamut);

const FFFilter ff_vf_gamut = {
    .p.name        = "gamut",
    .p.description = NULL_IF_CONFIG_SMALL("Filter out pixels that are out of valid range."),
    .p.priv_class  = &gamut_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_FRAME_THREADS,
    .priv_size     = sizeof(GamutContext),
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_PIXFMTS_ARRAY(gamut_fmts),
    .process_command = process_command,
};
