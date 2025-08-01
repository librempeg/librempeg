/*
 * Copyright (c) 2010 Stefano Sabatini
 * Copyright (c) 2010 Baptiste Coudurier
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
 * overlay one video on top of another
 */

#include "avfilter.h"
#include "formats.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/avstring.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/timestamp.h"
#include "filters.h"
#include "drawutils.h"
#include "framesync.h"
#include "video.h"
#include "vf_overlay.h"

typedef struct ThreadData {
    AVFrame *dst, *src;
} ThreadData;

static const char *const var_names[] = {
    "main_w",    "W", ///< width  of the main    video
    "main_h",    "H", ///< height of the main    video
    "overlay_w", "w", ///< width  of the overlay video
    "overlay_h", "h", ///< height of the overlay video
    "hsub",
    "vsub",
    "x",
    "y",
    "n",            ///< number of frame
    "t",            ///< timestamp expressed in seconds
    NULL
};

#define MAIN    0
#define OVERLAY 1

#define R 0
#define G 1
#define B 2
#define A 3

#define Y 0
#define U 1
#define V 2

enum EvalMode {
    EVAL_MODE_INIT,
    EVAL_MODE_FRAME,
    EVAL_MODE_NB
};

static av_cold void uninit(AVFilterContext *ctx)
{
    OverlayContext *s = ctx->priv;

    ff_framesync_uninit(&s->fs);
    av_expr_free(s->x_pexpr); s->x_pexpr = NULL;
    av_expr_free(s->y_pexpr); s->y_pexpr = NULL;
}

static inline int normalize_xy(double d, int chroma_sub)
{
    if (isnan(d))
        return INT_MAX;
    return (int)d & ~((1 << chroma_sub) - 1);
}

static void eval_expr(AVFilterContext *ctx)
{
    OverlayContext *s = ctx->priv;

    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->var_values[VAR_Y] = av_expr_eval(s->y_pexpr, s->var_values, NULL);
    /* It is necessary if x is expressed from y  */
    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->x = normalize_xy(s->var_values[VAR_X], s->hsub);
    s->y = normalize_xy(s->var_values[VAR_Y], s->vsub);
}

static int set_expr(AVExpr **pexpr, const char *expr, const char *option, void *log_ctx)
{
    int ret;
    AVExpr *old = NULL;

    if (*pexpr)
        old = *pexpr;
    ret = av_expr_parse(pexpr, expr, var_names,
                        NULL, NULL, NULL, NULL, 0, log_ctx);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR,
               "Error when evaluating the expression '%s' for %s\n",
               expr, option);
        *pexpr = old;
        return ret;
    }

    av_expr_free(old);
    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    OverlayContext *s = ctx->priv;
    int ret;

    if      (!strcmp(cmd, "x"))
        ret = set_expr(&s->x_pexpr, arg, cmd, ctx);
    else if (!strcmp(cmd, "y"))
        ret = set_expr(&s->y_pexpr, arg, cmd, ctx);
    else
        ret = AVERROR(ENOSYS);

    if (ret < 0)
        return ret;

    if (s->eval_mode == EVAL_MODE_INIT) {
        eval_expr(ctx);
        av_log(ctx, AV_LOG_VERBOSE, "x:%f xi:%d y:%f yi:%d\n",
               s->var_values[VAR_X], s->x,
               s->var_values[VAR_Y], s->y);
    }
    return ret;
}

static const enum AVPixelFormat alpha_pix_fmts[] = {
    AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA444P10,
    AV_PIX_FMT_ARGB, AV_PIX_FMT_ABGR, AV_PIX_FMT_RGBA,
    AV_PIX_FMT_BGRA, AV_PIX_FMT_GBRAP, AV_PIX_FMT_NONE
};

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const OverlayContext *s = ctx->priv;

    /* overlay formats contains alpha, for avoiding conversion with alpha information loss */
    static const enum AVPixelFormat main_pix_fmts_yuv420[] = {
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVA420P,
        AV_PIX_FMT_NV12, AV_PIX_FMT_NV21,
        AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv420[] = {
        AV_PIX_FMT_YUVA420P, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_yuv420p10[] = {
        AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUVA420P10,
        AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv420p10[] = {
        AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_yuv422[] = {
        AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv422[] = {
        AV_PIX_FMT_YUVA422P, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_yuv422p10[] = {
        AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv422p10[] = {
        AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_yuv444[] = {
        AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVA444P, AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv444[] = {
        AV_PIX_FMT_YUVA444P, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_yuv444p10[] = {
        AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_yuv444p10[] = {
        AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_gbrp[] = {
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP, AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_gbrp[] = {
        AV_PIX_FMT_GBRAP, AV_PIX_FMT_NONE
    };

    static const enum AVPixelFormat main_pix_fmts_rgb[] = {
        AV_PIX_FMT_ARGB,  AV_PIX_FMT_RGBA,
        AV_PIX_FMT_ABGR,  AV_PIX_FMT_BGRA,
        AV_PIX_FMT_RGB24, AV_PIX_FMT_BGR24,
        AV_PIX_FMT_NONE
    };
    static const enum AVPixelFormat overlay_pix_fmts_rgb[] = {
        AV_PIX_FMT_ARGB,  AV_PIX_FMT_RGBA,
        AV_PIX_FMT_ABGR,  AV_PIX_FMT_BGRA,
        AV_PIX_FMT_NONE
    };

    const enum AVPixelFormat *main_formats, *overlay_formats;
    AVFilterFormats *formats;
    int ret;

    switch (s->format) {
    case OVERLAY_FORMAT_YUV420:
        main_formats    = main_pix_fmts_yuv420;
        overlay_formats = overlay_pix_fmts_yuv420;
        break;
    case OVERLAY_FORMAT_YUV420P10:
        main_formats    = main_pix_fmts_yuv420p10;
        overlay_formats = overlay_pix_fmts_yuv420p10;
        break;
    case OVERLAY_FORMAT_YUV422:
        main_formats    = main_pix_fmts_yuv422;
        overlay_formats = overlay_pix_fmts_yuv422;
        break;
    case OVERLAY_FORMAT_YUV422P10:
        main_formats    = main_pix_fmts_yuv422p10;
        overlay_formats = overlay_pix_fmts_yuv422p10;
        break;
    case OVERLAY_FORMAT_YUV444:
        main_formats    = main_pix_fmts_yuv444;
        overlay_formats = overlay_pix_fmts_yuv444;
        break;
    case OVERLAY_FORMAT_YUV444P10:
        main_formats    = main_pix_fmts_yuv444p10;
        overlay_formats = overlay_pix_fmts_yuv444p10;
        break;
    case OVERLAY_FORMAT_RGB:
        main_formats    = main_pix_fmts_rgb;
        overlay_formats = overlay_pix_fmts_rgb;
        break;
    case OVERLAY_FORMAT_GBRP:
        main_formats    = main_pix_fmts_gbrp;
        overlay_formats = overlay_pix_fmts_gbrp;
        break;
    case OVERLAY_FORMAT_AUTO:
        return ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, alpha_pix_fmts);
    default:
        av_assert0(0);
    }

    formats = ff_make_format_list(main_formats);
    if ((ret = ff_formats_ref(formats, &cfg_in[MAIN]->formats)) < 0 ||
        (ret = ff_formats_ref(formats, &cfg_out[MAIN]->formats)) < 0)
        return ret;

    return ff_formats_ref(ff_make_format_list(overlay_formats),
                          &cfg_in[OVERLAY]->formats);
}

static int config_input_overlay(AVFilterLink *inlink)
{
    AVFilterContext *ctx  = inlink->dst;
    OverlayContext  *s = inlink->dst->priv;
    int ret;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(inlink->format);

    av_image_fill_max_pixsteps(s->overlay_pix_step, NULL, pix_desc);

    /* Finish the configuration by evaluating the expressions
       now when both inputs are configured. */
    s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = ctx->inputs[MAIN   ]->w;
    s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = ctx->inputs[MAIN   ]->h;
    s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = ctx->inputs[OVERLAY]->w;
    s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = ctx->inputs[OVERLAY]->h;
    s->var_values[VAR_HSUB]  = 1<<pix_desc->log2_chroma_w;
    s->var_values[VAR_VSUB]  = 1<<pix_desc->log2_chroma_h;
    s->var_values[VAR_X]     = NAN;
    s->var_values[VAR_Y]     = NAN;
    s->var_values[VAR_N]     = 0;
    s->var_values[VAR_T]     = NAN;

    if ((ret = set_expr(&s->x_pexpr,      s->x_expr,      "x",      ctx)) < 0 ||
        (ret = set_expr(&s->y_pexpr,      s->y_expr,      "y",      ctx)) < 0)
        return ret;

    s->overlay_is_packed_rgb =
        ff_fill_rgba_map(s->overlay_rgba_map, inlink->format) >= 0;
    s->overlay_has_alpha = ff_fmt_is_in(inlink->format, alpha_pix_fmts);

    if (s->eval_mode == EVAL_MODE_INIT) {
        eval_expr(ctx);
        av_log(ctx, AV_LOG_VERBOSE, "x:%f xi:%d y:%f yi:%d\n",
               s->var_values[VAR_X], s->x,
               s->var_values[VAR_Y], s->y);
    }

    av_log(ctx, AV_LOG_VERBOSE,
           "main w:%d h:%d fmt:%s overlay w:%d h:%d fmt:%s\n",
           ctx->inputs[MAIN]->w, ctx->inputs[MAIN]->h,
           av_get_pix_fmt_name(ctx->inputs[MAIN]->format),
           ctx->inputs[OVERLAY]->w, ctx->inputs[OVERLAY]->h,
           av_get_pix_fmt_name(ctx->inputs[OVERLAY]->format));
    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    OverlayContext *s = ctx->priv;
    int ret;

    if ((ret = ff_framesync_init_dualinput(&s->fs, ctx)) < 0)
        return ret;

    outlink->w = ctx->inputs[MAIN]->w;
    outlink->h = ctx->inputs[MAIN]->h;
    outlink->time_base = ctx->inputs[MAIN]->time_base;

    return ff_framesync_configure(&s->fs);
}

// divide by 255 and round to nearest
// apply a fast variant: (X+127)/255 = ((X+127)*257+257)>>16 = ((X+128)*257)>>16
#define FAST_DIV255(x) ((((x) + 128) * 257) >> 16)

// calculate the unpremultiplied alpha, applying the general equation:
// alpha = alpha_overlay / ( (alpha_main + alpha_overlay) - (alpha_main * alpha_overlay) )
// (((x) << 16) - ((x) << 9) + (x)) is a faster version of: 255 * 255 * x
// ((((x) + (y)) << 8) - ((x) + (y)) - (y) * (x)) is a faster version of: 255 * (x + y)
#define UNPREMULTIPLY_ALPHA(x, y) ((((x) << 16) - ((x) << 9) + (x)) / ((((x) + (y)) << 8) - ((x) + (y)) - (y) * (x)))

#define PTR_ADD(TYPE, ptr, byte_addend) ((TYPE*)((uint8_t*)ptr + (byte_addend)))
#define CPTR_ADD(TYPE, ptr, byte_addend) ((const TYPE*)((const uint8_t*)ptr + (byte_addend)))

/**
 * Blend image in src to destination buffer dst at position (x, y).
 */

static av_always_inline void blend_slice_packed_rgb(AVFilterContext *ctx,
                                   AVFrame *dst, const AVFrame *src,
                                   int main_has_alpha, int x, int y,
                                   int is_straight, int jobnr, int nb_jobs)
{
    OverlayContext *s = ctx->priv;
    int i, imax, j, jmax;
    const int src_w = src->width;
    const int src_h = src->height;
    const int dst_w = dst->width;
    const int dst_h = dst->height;
    uint8_t alpha;          ///< the amount of overlay to blend on to main
    const int dr = s->main_rgba_map[R];
    const int dg = s->main_rgba_map[G];
    const int db = s->main_rgba_map[B];
    const int da = s->main_rgba_map[A];
    const int dstep = s->main_pix_step[0];
    const int sr = s->overlay_rgba_map[R];
    const int sg = s->overlay_rgba_map[G];
    const int sb = s->overlay_rgba_map[B];
    const int sa = s->overlay_rgba_map[A];
    const int sstep = s->overlay_pix_step[0];
    int slice_start, slice_end;
    uint8_t *S, *sp, *d, *dp;

    i = FFMAX(-y, 0);
    imax = FFMIN3(-y + dst_h, FFMIN(src_h, dst_h), y + src_h);

    slice_start = i + (imax * jobnr) / nb_jobs;
    slice_end = i + (imax * (jobnr+1)) / nb_jobs;

    sp = src->data[0] + (slice_start)     * src->linesize[0];
    dp = dst->data[0] + (y + slice_start) * dst->linesize[0];

    for (i = slice_start; i < slice_end; i++) {
        j = FFMAX(-x, 0);
        S = sp + j     * sstep;
        d = dp + (x+j) * dstep;

        for (jmax = FFMIN(-x + dst_w, src_w); j < jmax; j++) {
            alpha = S[sa];

            // if the main channel has an alpha channel, alpha has to be calculated
            // to create an un-premultiplied (straight) alpha value
            if (main_has_alpha && alpha != 0 && alpha != 255) {
                uint8_t alpha_d = d[da];
                alpha = UNPREMULTIPLY_ALPHA(alpha, alpha_d);
            }

            switch (alpha) {
            case 0:
                break;
            case 255:
                d[dr] = S[sr];
                d[dg] = S[sg];
                d[db] = S[sb];
                break;
            default:
                // main_value = main_value * (1 - alpha) + overlay_value * alpha
                // since alpha is in the range 0-255, the result must divided by 255
                d[dr] = is_straight ? FAST_DIV255(d[dr] * (255 - alpha) + S[sr] * alpha) :
                        FFMIN(FAST_DIV255(d[dr] * (255 - alpha)) + S[sr], 255);
                d[dg] = is_straight ? FAST_DIV255(d[dg] * (255 - alpha) + S[sg] * alpha) :
                        FFMIN(FAST_DIV255(d[dg] * (255 - alpha)) + S[sg], 255);
                d[db] = is_straight ? FAST_DIV255(d[db] * (255 - alpha) + S[sb] * alpha) :
                        FFMIN(FAST_DIV255(d[db] * (255 - alpha)) + S[sb], 255);
            }
            if (main_has_alpha) {
                switch (alpha) {
                case 0:
                    break;
                case 255:
                    d[da] = S[sa];
                    break;
                default:
                    // apply alpha compositing: main_alpha += (1-main_alpha) * overlay_alpha
                    d[da] += FAST_DIV255((255 - d[da]) * S[sa]);
                }
            }
            d += dstep;
            S += sstep;
        }
        dp += dst->linesize[0];
        sp += src->linesize[0];
    }
}

#define DEFINE_BLEND_PLANE(depth, T, nbits)                                                                \
static av_always_inline void blend_plane_##depth##_##nbits##bits(AVFilterContext *ctx,                     \
                                         AVFrame *dst, const AVFrame *src,                                 \
                                         int src_w, int src_h,                                             \
                                         int dst_w, int dst_h,                                             \
                                         int i, int hsub, int vsub,                                        \
                                         int x, int y,                                                     \
                                         int main_has_alpha,                                               \
                                         int dst_plane,                                                    \
                                         int dst_offset,                                                   \
                                         int dst_step,                                                     \
                                         int straight,                                                     \
                                         int yuv,                                                          \
                                         int jobnr,                                                        \
                                         int nb_jobs)                                                      \
{                                                                                                          \
    OverlayContext *octx = ctx->priv;                                                                      \
    int src_wp = AV_CEIL_RSHIFT(src_w, hsub);                                                              \
    int src_hp = AV_CEIL_RSHIFT(src_h, vsub);                                                              \
    int dst_wp = AV_CEIL_RSHIFT(dst_w, hsub);                                                              \
    int dst_hp = AV_CEIL_RSHIFT(dst_h, vsub);                                                              \
    int yp = y>>vsub;                                                                                      \
    int xp = x>>hsub;                                                                                      \
    const T max = (1 << nbits) - 1;                                                                        \
    const T mid = (1 << (nbits - 1));                                                                      \
                                                                                                           \
    const int jmin = FFMAX(-yp, 0), jmax = FFMIN3(-yp + dst_hp, FFMIN(src_hp, dst_hp), yp + src_hp);       \
    const int kmin = FFMAX(-xp, 0), kmax = FFMIN(-xp + dst_wp, src_wp);                                    \
    const int slice_start = jmin + (jmax *  jobnr)      / nb_jobs;                                         \
    const int slice_end   = jmin + (jmax * (jobnr + 1)) / nb_jobs;                                         \
                                                                                                           \
    const uint8_t *sp = src->data[i] + (slice_start) * src->linesize[i];                                   \
    uint8_t       *dp = dst->data[dst_plane]                                                               \
                      + (yp + slice_start) * dst->linesize[dst_plane]                                      \
                      + dst_offset;                                                                        \
    const uint8_t *ap = src->data[3] + (slice_start << vsub) * src->linesize[3];                           \
    const uint8_t *dap = main_has_alpha ? dst->data[3] + ((yp + slice_start) << vsub) * dst->linesize[3] : NULL; \
                                                                                                           \
    for (int j = slice_start; j < slice_end; ++j) {                                                        \
        int k = kmin;                                                                                      \
        const T  *s = (const T *)sp + k;                                                                   \
        const T  *a = (const T *)ap + (k << hsub);                                                         \
        const T *da = main_has_alpha ? (T *)dap + ((xp + k) << hsub) : NULL;                               \
        T *d  = (T *)(dp + (xp + k) * dst_step);                                                           \
                                                                                                           \
        if (nbits == 8 && ((vsub && j+1 < src_hp) || !vsub) && octx->blend_row[i]) {                       \
            int c = octx->blend_row[i]((uint8_t*)d, (uint8_t*)da, (uint8_t*)s,                             \
                    (uint8_t*)a, kmax - k, src->linesize[3]);                                              \
                                                                                                           \
            s += c;                                                                                        \
            d  = PTR_ADD(T, d, dst_step * c);                                                              \
            if (main_has_alpha)                                                                            \
                da += (1 << hsub) * c;                                                                     \
            a += (1 << hsub) * c;                                                                          \
            k += c;                                                                                        \
        }                                                                                                  \
        for (; k < kmax; k++) {                                                                            \
            int alpha_v, alpha_h, alpha;                                                                   \
                                                                                                           \
            /* average alpha for color components, improve quality */                                      \
            if (hsub && vsub && j+1 < src_hp && k+1 < src_wp) {                                            \
                const T *next_line = CPTR_ADD(T, a, src->linesize[3]);                                     \
                alpha = (a[0] + next_line[0] +                                                             \
                         a[1] + next_line[1]) >> 2;                                                        \
            } else if (hsub || vsub) {                                                                     \
                alpha_h = hsub && k+1 < src_wp ?                                                           \
                    (a[0] + a[1]) >> 1 : a[0];                                                             \
                alpha_v = vsub && j+1 < src_hp ?                                                           \
                    (a[0] + *CPTR_ADD(T, a, src->linesize[3])) >> 1 : a[0];                                \
                alpha = (alpha_v + alpha_h) >> 1;                                                          \
            } else                                                                                         \
                alpha = a[0];                                                                              \
            /* if the main channel has an alpha channel, alpha has to be calculated */                     \
            /* to create an un-premultiplied (straight) alpha value */                                     \
            if (main_has_alpha && alpha != 0 && alpha != max) {                                            \
                /* average alpha for color components, improve quality */                                  \
                uint8_t alpha_d;                                                                           \
                if (hsub && vsub && j+1 < src_hp && k+1 < src_wp) {                                        \
                    const T *next_line = CPTR_ADD(T, da, dst->linesize[3]);                                \
                    alpha_d = (da[0] + next_line[0] +                                                      \
                               da[1] + next_line[1]) >> 2;                                                 \
                } else if (hsub || vsub) {                                                                 \
                    alpha_h = hsub && k+1 < src_wp ?                                                       \
                        (da[0] + da[1]) >> 1 : da[0];                                                      \
                    alpha_v = vsub && j+1 < src_hp ?                                                       \
                        (da[0] + *CPTR_ADD(T, da, dst->linesize[3])) >> 1 : da[0];                         \
                    alpha_d = (alpha_v + alpha_h) >> 1;                                                    \
                } else                                                                                     \
                    alpha_d = da[0];                                                                       \
                alpha = UNPREMULTIPLY_ALPHA(alpha, alpha_d);                                               \
            }                                                                                              \
            if (straight) {                                                                                \
                if (nbits > 8)                                                                             \
                   *d = (*d * (max - alpha) + *s * alpha) / max;                                           \
                else                                                                                       \
                    *d = FAST_DIV255(*d * (255 - alpha) + *s * alpha);                                     \
            } else {                                                                                       \
                if (nbits > 8) {                                                                           \
                    if (i && yuv)                                                                          \
                        *d = av_clip((*d * (max - alpha) + *s * alpha) / max + *s - mid, -mid, mid) + mid; \
                    else                                                                                   \
                        *d = av_clip_uintp2((*d * (max - alpha) + *s * alpha) / max + *s - (16<<(nbits-8)),\
                                                                                                    nbits);\
                } else {                                                                                   \
                    if (i && yuv)                                                                          \
                        *d = av_clip(FAST_DIV255((*d - mid) * (max - alpha)) + *s - mid, -mid, mid) + mid; \
                    else                                                                                   \
                        *d = av_clip_uint8(FAST_DIV255(*d * (255 - alpha)) + *s - 16);                     \
                }                                                                                          \
            }                                                                                              \
            s++;                                                                                           \
            d  = PTR_ADD(T, d, dst_step);                                                                  \
            if (main_has_alpha)                                                                            \
                da += 1 << hsub;                                                                           \
            a += 1 << hsub;                                                                                \
        }                                                                                                  \
        dp += dst->linesize[dst_plane];                                                                    \
        sp += src->linesize[i];                                                                            \
        ap += (1 << vsub) * src->linesize[3];                                                              \
        if (main_has_alpha)                                                                                \
            dap += (1 << vsub) * dst->linesize[3];                                                         \
    }                                                                                                      \
}
DEFINE_BLEND_PLANE(8,  uint8_t,  8)
DEFINE_BLEND_PLANE(16, uint16_t, 10)

#define DEFINE_ALPHA_COMPOSITE(depth, T, nbits)                                                            \
static inline void alpha_composite_##depth##_##nbits##bits(const AVFrame *src, const AVFrame *dst,         \
                                   int src_w, int src_h,                                                   \
                                   int dst_w, int dst_h,                                                   \
                                   int x, int y,                                                           \
                                   int jobnr, int nb_jobs)                                                 \
{                                                                                                          \
    T alpha;          /* the amount of overlay to blend on to main */                                      \
    const T max = (1 << nbits) - 1;                                                                        \
                                                                                                           \
    const int imin = FFMAX(-y, 0), imax = FFMIN3(-y + dst_h, FFMIN(src_h, dst_h), y + src_h);              \
    const int jmin = FFMAX(-x, 0), jmax = FFMIN(-x + dst_w, src_w);                                        \
    const int slice_start = imin + ( imax *  jobnr)      / nb_jobs;                                        \
    const int slice_end   = imin + ((imax * (jobnr + 1)) / nb_jobs);                                       \
                                                                                                           \
    const uint8_t *sa = src->data[3] +     (slice_start) * src->linesize[3];                               \
    uint8_t       *da = dst->data[3] + (y + slice_start) * dst->linesize[3];                               \
                                                                                                           \
    for (int i = slice_start; i < slice_end; ++i) {                                                        \
        const T *s = (const T *)sa + jmin;                                                                 \
        T *d = (T *)da + x + jmin;                                                                         \
                                                                                                           \
        for (int j = jmin; j < jmax; ++j) {                                                                \
            alpha = *s;                                                                                    \
            if (alpha != 0 && alpha != max) {                                                              \
                uint8_t alpha_d = *d;                                                                      \
                alpha = UNPREMULTIPLY_ALPHA(alpha, alpha_d);                                               \
            }                                                                                              \
            if (alpha == max)                                                                              \
                *d = *s;                                                                                   \
            else if (alpha > 0) {                                                                          \
                /* apply alpha compositing: main_alpha += (1-main_alpha) * overlay_alpha */                \
                if (nbits > 8)                                                                             \
                    *d += (max - *d) * *s / max;                                                           \
                else                                                                                       \
                    *d += FAST_DIV255((max - *d) * *s);                                                    \
            }                                                                                              \
            d += 1;                                                                                        \
            s += 1;                                                                                        \
        }                                                                                                  \
        da += dst->linesize[3];                                                                            \
        sa += src->linesize[3];                                                                            \
    }                                                                                                      \
}
DEFINE_ALPHA_COMPOSITE(8,  uint8_t,  8)
DEFINE_ALPHA_COMPOSITE(16, uint16_t, 10)

#define DEFINE_BLEND_SLICE_YUV(depth, nbits)                                                               \
static av_always_inline void blend_slice_yuv_##depth##_##nbits##bits(AVFilterContext *ctx,                 \
                                             AVFrame *dst, const AVFrame *src,                             \
                                             int hsub, int vsub,                                           \
                                             int main_has_alpha,                                           \
                                             int x, int y,                                                 \
                                             int is_straight,                                              \
                                             int jobnr, int nb_jobs)                                       \
{                                                                                                          \
    OverlayContext *s = ctx->priv;                                                                         \
    const int src_w = src->width;                                                                          \
    const int src_h = src->height;                                                                         \
    const int dst_w = dst->width;                                                                          \
    const int dst_h = dst->height;                                                                         \
                                                                                                           \
    blend_plane_##depth##_##nbits##bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 0, 0,       0,          \
                x, y, main_has_alpha, s->main_desc->comp[0].plane, s->main_desc->comp[0].offset,           \
                s->main_desc->comp[0].step, is_straight, 1, jobnr, nb_jobs);                               \
    blend_plane_##depth##_##nbits##bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 1, hsub, vsub,          \
                x, y, main_has_alpha, s->main_desc->comp[1].plane, s->main_desc->comp[1].offset,           \
                s->main_desc->comp[1].step, is_straight, 1, jobnr, nb_jobs);                               \
    blend_plane_##depth##_##nbits##bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 2, hsub, vsub,          \
                x, y, main_has_alpha, s->main_desc->comp[2].plane, s->main_desc->comp[2].offset,           \
                s->main_desc->comp[2].step, is_straight, 1, jobnr, nb_jobs);                               \
                                                                                                           \
    if (main_has_alpha)                                                                                    \
        alpha_composite_##depth##_##nbits##bits(src, dst, src_w, src_h, dst_w, dst_h, x, y,                \
                                                jobnr, nb_jobs);                                           \
}
DEFINE_BLEND_SLICE_YUV(8, 8)
DEFINE_BLEND_SLICE_YUV(16, 10)

static av_always_inline void blend_slice_planar_rgb(AVFilterContext *ctx,
                                                    AVFrame *dst, const AVFrame *src,
                                                    int hsub, int vsub,
                                                    int main_has_alpha,
                                                    int x, int y,
                                                    int is_straight,
                                                    int jobnr,
                                                    int nb_jobs)
{
    OverlayContext *s = ctx->priv;
    const int src_w = src->width;
    const int src_h = src->height;
    const int dst_w = dst->width;
    const int dst_h = dst->height;

    blend_plane_8_8bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 0, 0,   0, x, y, main_has_alpha,
                s->main_desc->comp[1].plane, s->main_desc->comp[1].offset, s->main_desc->comp[1].step, is_straight, 0,
                jobnr, nb_jobs);
    blend_plane_8_8bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 1, hsub, vsub, x, y, main_has_alpha,
                s->main_desc->comp[2].plane, s->main_desc->comp[2].offset, s->main_desc->comp[2].step, is_straight, 0,
                jobnr, nb_jobs);
    blend_plane_8_8bits(ctx, dst, src, src_w, src_h, dst_w, dst_h, 2, hsub, vsub, x, y, main_has_alpha,
                s->main_desc->comp[0].plane, s->main_desc->comp[0].offset, s->main_desc->comp[0].step, is_straight, 0,
                jobnr, nb_jobs);

    if (main_has_alpha)
        alpha_composite_8_8bits(src, dst, src_w, src_h, dst_w, dst_h, x, y, jobnr, nb_jobs);
}

#define DEFINE_BLEND_SLICE_PLANAR_FMT(format_, blend_slice_fn_suffix_, hsub_, vsub_, main_has_alpha_, direct_) \
static int blend_slice_##format_(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)           \
{                                                                       \
    OverlayContext *s = ctx->priv;                                      \
    ThreadData *td = arg;                                               \
    blend_slice_##blend_slice_fn_suffix_(ctx, td->dst, td->src,         \
                                         hsub_, vsub_, main_has_alpha_, \
                                         s->x, s->y, direct_,           \
                                         jobnr, nb_jobs);               \
    return 0;                                                           \
}

//                            FMT          FN             H  V  A  D
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv420,      yuv_8_8bits,   1, 1, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva420,     yuv_8_8bits,   1, 1, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv420p10,   yuv_16_10bits, 1, 1, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva420p10,  yuv_16_10bits, 1, 1, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv422p10,   yuv_16_10bits, 1, 0, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva422p10,  yuv_16_10bits, 1, 0, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv422,      yuv_8_8bits,   1, 0, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva422,     yuv_8_8bits,   1, 0, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv444,      yuv_8_8bits,   0, 0, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva444,     yuv_8_8bits,   0, 0, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv444p10,   yuv_16_10bits, 0, 0, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva444p10,  yuv_16_10bits, 0, 0, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(gbrp,        planar_rgb,    0, 0, 0, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(gbrap,       planar_rgb,    0, 0, 1, 1)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv420_pm,   yuv_8_8bits,   1, 1, 0, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva420_pm,  yuv_8_8bits,   1, 1, 1, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv422_pm,   yuv_8_8bits,   1, 0, 0, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva422_pm,  yuv_8_8bits,   1, 0, 1, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuv444_pm,   yuv_8_8bits,   0, 0, 0, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(yuva444_pm,  yuv_8_8bits,   0, 0, 1, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(gbrp_pm,     planar_rgb,    0, 0, 0, 0)
DEFINE_BLEND_SLICE_PLANAR_FMT(gbrap_pm,    planar_rgb,    0, 0, 1, 0)

#define DEFINE_BLEND_SLICE_PACKED_FMT(format_, blend_slice_fn_suffix_, main_has_alpha_, direct_) \
static int blend_slice_##format_(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)        \
{                                                                       \
    OverlayContext *s = ctx->priv;                                      \
    ThreadData *td = arg;                                               \
    blend_slice_packed_##blend_slice_fn_suffix_(ctx, td->dst, td->src,  \
                                                main_has_alpha_,        \
                                                s->x, s->y, direct_,    \
                                                jobnr, nb_jobs);        \
    return 0;                                                           \
}

//                            FMT      FN   A  D
DEFINE_BLEND_SLICE_PACKED_FMT(rgb,     rgb, 0, 1)
DEFINE_BLEND_SLICE_PACKED_FMT(rgba,    rgb, 1, 1)
DEFINE_BLEND_SLICE_PACKED_FMT(rgb_pm,  rgb, 0, 0)
DEFINE_BLEND_SLICE_PACKED_FMT(rgba_pm, rgb, 1, 0)

static int config_input_main(AVFilterLink *inlink)
{
    OverlayContext *s = inlink->dst->priv;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(inlink->format);

    av_image_fill_max_pixsteps(s->main_pix_step,    NULL, pix_desc);

    s->hsub = pix_desc->log2_chroma_w;
    s->vsub = pix_desc->log2_chroma_h;

    s->main_desc = pix_desc;

    s->main_is_packed_rgb =
        ff_fill_rgba_map(s->main_rgba_map, inlink->format) >= 0;
    s->main_has_alpha = ff_fmt_is_in(inlink->format, alpha_pix_fmts);
    switch (s->format) {
    case OVERLAY_FORMAT_YUV420:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva420 : blend_slice_yuv420;
        break;
    case OVERLAY_FORMAT_YUV420P10:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva420p10 : blend_slice_yuv420p10;
        break;
    case OVERLAY_FORMAT_YUV422:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva422 : blend_slice_yuv422;
        break;
    case OVERLAY_FORMAT_YUV422P10:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva422p10 : blend_slice_yuv422p10;
        break;
    case OVERLAY_FORMAT_YUV444:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva444 : blend_slice_yuv444;
        break;
    case OVERLAY_FORMAT_YUV444P10:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva444p10 : blend_slice_yuv444p10;
        break;
    case OVERLAY_FORMAT_RGB:
        s->blend_slice = s->main_has_alpha ? blend_slice_rgba : blend_slice_rgb;
        break;
    case OVERLAY_FORMAT_GBRP:
        s->blend_slice = s->main_has_alpha ? blend_slice_gbrap : blend_slice_gbrp;
        break;
    case OVERLAY_FORMAT_AUTO:
        switch (inlink->format) {
        case AV_PIX_FMT_YUVA420P:
            s->blend_slice = blend_slice_yuva420;
            break;
        case AV_PIX_FMT_YUVA420P10:
            s->blend_slice = blend_slice_yuva420p10;
            break;
        case AV_PIX_FMT_YUVA422P:
            s->blend_slice = blend_slice_yuva422;
            break;
        case AV_PIX_FMT_YUVA422P10:
            s->blend_slice = blend_slice_yuva422p10;
            break;
        case AV_PIX_FMT_YUVA444P:
            s->blend_slice = blend_slice_yuva444;
            break;
        case AV_PIX_FMT_YUVA444P10:
            s->blend_slice = blend_slice_yuva444p10;
            break;
        case AV_PIX_FMT_ARGB:
        case AV_PIX_FMT_RGBA:
        case AV_PIX_FMT_BGRA:
        case AV_PIX_FMT_ABGR:
            s->blend_slice = blend_slice_rgba;
            break;
        case AV_PIX_FMT_GBRAP:
            s->blend_slice = blend_slice_gbrap;
            break;
        default:
            av_assert0(0);
            break;
        }
        break;
    }

    if (!s->alpha_format)
        goto end;

    switch (s->format) {
    case OVERLAY_FORMAT_YUV420:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva420_pm : blend_slice_yuv420_pm;
        break;
    case OVERLAY_FORMAT_YUV422:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva422_pm : blend_slice_yuv422_pm;
        break;
    case OVERLAY_FORMAT_YUV444:
        s->blend_slice = s->main_has_alpha ? blend_slice_yuva444_pm : blend_slice_yuv444_pm;
        break;
    case OVERLAY_FORMAT_RGB:
        s->blend_slice = s->main_has_alpha ? blend_slice_rgba_pm : blend_slice_rgb_pm;
        break;
    case OVERLAY_FORMAT_GBRP:
        s->blend_slice = s->main_has_alpha ? blend_slice_gbrap_pm : blend_slice_gbrp_pm;
        break;
    case OVERLAY_FORMAT_AUTO:
        switch (inlink->format) {
        case AV_PIX_FMT_YUVA420P:
            s->blend_slice = blend_slice_yuva420_pm;
            break;
        case AV_PIX_FMT_YUVA422P:
            s->blend_slice = blend_slice_yuva422_pm;
            break;
        case AV_PIX_FMT_YUVA444P:
            s->blend_slice = blend_slice_yuva444_pm;
            break;
        case AV_PIX_FMT_ARGB:
        case AV_PIX_FMT_RGBA:
        case AV_PIX_FMT_BGRA:
        case AV_PIX_FMT_ABGR:
            s->blend_slice = blend_slice_rgba_pm;
            break;
        case AV_PIX_FMT_GBRAP:
            s->blend_slice = blend_slice_gbrap_pm;
            break;
        default:
            av_assert0(0);
            break;
        }
        break;
    }

end:
#if ARCH_X86
    ff_overlay_init_x86(s, s->format, inlink->format,
                        s->alpha_format, s->main_has_alpha);
#endif

    return 0;
}

static int do_blend(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    AVFrame *mainpic, *second;
    OverlayContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    FilterLink *inl = ff_filter_link(inlink);
    int ret;

    ret = ff_framesync_dualinput_get_writable(fs, &mainpic, &second);
    if (ret < 0)
        return ret;
    if (!second)
        return ff_filter_frame(ctx->outputs[0], mainpic);

    if (s->eval_mode == EVAL_MODE_FRAME) {

        s->var_values[VAR_N] = inl->frame_count_out;
        s->var_values[VAR_T] = mainpic->pts == AV_NOPTS_VALUE ?
            NAN : mainpic->pts * av_q2d(inlink->time_base);

        s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = second->width;
        s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = second->height;
        s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = mainpic->width;
        s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = mainpic->height;

        eval_expr(ctx);
        av_log(ctx, AV_LOG_DEBUG, "n:%f t:%f x:%f xi:%d y:%f yi:%d\n",
               s->var_values[VAR_N], s->var_values[VAR_T],
               s->var_values[VAR_X], s->x,
               s->var_values[VAR_Y], s->y);
    }

    if (s->x < mainpic->width  && s->x + second->width  >= 0 &&
        s->y < mainpic->height && s->y + second->height >= 0) {
        ThreadData td;

        td.dst = mainpic;
        td.src = second;
        ff_filter_execute(ctx, s->blend_slice, &td, NULL, FFMIN(FFMAX(1, FFMIN3(s->y + second->height, FFMIN(second->height, mainpic->height), mainpic->height - s->y)),
                                                                ff_filter_get_nb_threads(ctx)));
    }
    return ff_filter_frame(ctx->outputs[0], mainpic);
}

static av_cold int init(AVFilterContext *ctx)
{
    OverlayContext *s = ctx->priv;

    s->fs.on_event = do_blend;
    return 0;
}

static int activate(AVFilterContext *ctx)
{
    OverlayContext *s = ctx->priv;
    return ff_framesync_activate(&s->fs);
}

#define OFFSET(x) offsetof(OverlayContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption overlay_options[] = {
    { "x", "set the x expression", OFFSET(x_expr), AV_OPT_TYPE_STRING, {.str = "0"}, 0, 0, TFLAGS },
    { "y", "set the y expression", OFFSET(y_expr), AV_OPT_TYPE_STRING, {.str = "0"}, 0, 0, TFLAGS },
    { "eof_action", "Action to take when encountering EOF from secondary input ",
        OFFSET(fs.opt_eof_action), AV_OPT_TYPE_INT, { .i64 = EOF_ACTION_REPEAT },
        EOF_ACTION_REPEAT, EOF_ACTION_PASS, .flags = FLAGS, .unit = "eof_action" },
        { "repeat", "Repeat the previous frame.",   0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_REPEAT }, .flags = FLAGS, .unit = "eof_action" },
        { "endall", "End both streams.",            0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_ENDALL }, .flags = FLAGS, .unit = "eof_action" },
        { "pass",   "Pass through the main input.", 0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_PASS },   .flags = FLAGS, .unit = "eof_action" },
    { "eval", "specify when to evaluate expressions", OFFSET(eval_mode), AV_OPT_TYPE_INT, {.i64 = EVAL_MODE_FRAME}, 0, EVAL_MODE_NB-1, FLAGS, .unit = "eval" },
         { "init",  "eval expressions once during initialization", 0, AV_OPT_TYPE_CONST, {.i64=EVAL_MODE_INIT},  .flags = FLAGS, .unit = "eval" },
         { "frame", "eval expressions per-frame",                  0, AV_OPT_TYPE_CONST, {.i64=EVAL_MODE_FRAME}, .flags = FLAGS, .unit = "eval" },
    { "shortest", "force termination when the shortest input terminates", OFFSET(fs.opt_shortest), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, FLAGS },
    { "format", "set output format", OFFSET(format), AV_OPT_TYPE_INT, {.i64=OVERLAY_FORMAT_YUV420}, 0, OVERLAY_FORMAT_NB-1, FLAGS, .unit = "format" },
        { "yuv420", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV420}, .flags = FLAGS, .unit = "format" },
        { "yuv420p10", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV420P10}, .flags = FLAGS, .unit = "format" },
        { "yuv422", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV422}, .flags = FLAGS, .unit = "format" },
        { "yuv422p10", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV422P10}, .flags = FLAGS, .unit = "format" },
        { "yuv444", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV444}, .flags = FLAGS, .unit = "format" },
        { "yuv444p10", "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_YUV444P10}, .flags = FLAGS, .unit = "format" },
        { "rgb",    "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_RGB},    .flags = FLAGS, .unit = "format" },
        { "gbrp",   "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_GBRP},   .flags = FLAGS, .unit = "format" },
        { "auto",   "", 0, AV_OPT_TYPE_CONST, {.i64=OVERLAY_FORMAT_AUTO},   .flags = FLAGS, .unit = "format" },
    { "repeatlast", "repeat overlay of the last overlay frame", OFFSET(fs.opt_repeatlast), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
    { "alpha", "alpha format", OFFSET(alpha_format), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, FLAGS, .unit = "alpha_format" },
        { "straight",      "", 0, AV_OPT_TYPE_CONST, {.i64=0}, .flags = FLAGS, .unit = "alpha_format" },
        { "premultiplied", "", 0, AV_OPT_TYPE_CONST, {.i64=1}, .flags = FLAGS, .unit = "alpha_format" },
    { NULL }
};

FRAMESYNC_DEFINE_CLASS(overlay, OverlayContext, fs);

static const AVFilterPad overlay_inputs[] = {
    {
        .name         = "main",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input_main,
    },
    {
        .name         = "overlay",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input_overlay,
    },
};

static const AVFilterPad overlay_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_vf_overlay = {
    .p.name        = "overlay",
    .p.description = NULL_IF_CONFIG_SMALL("Overlay a video source on top of the input."),
    .p.priv_class  = &overlay_class,
    .preinit       = overlay_framesync_preinit,
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(OverlayContext),
    .activate      = activate,
    .process_command = process_command,
    FILTER_INPUTS(overlay_inputs),
    FILTER_OUTPUTS(overlay_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
