/*
 * Copyright (c) 2017 Thomas Mundt <tmundt75@gmail.com>
 * Copyright (c) 2011 Stefano Sabatini
 * Copyright (c) 2010 Baptiste Coudurier
 * Copyright (c) 2003 Michael Zucchi <notzed@ximian.com>
 *
 * This file is part of Librempeg.
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file
 * temporal field interlace filter, ported from MPlayer/libmpcodecs
 */

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/avassert.h"
#include "avfilter.h"
#include "filters.h"
#include "tinterlace.h"
#include "video.h"

#define OFFSET(x) offsetof(TInterlaceContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption tinterlace_options[] = {
    {"mode",              "select interlace mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=MODE_MERGE}, 0, MODE_NB-1, FLAGS, .unit = "mode"},
    {"merge",             "merge fields",                                 0, AV_OPT_TYPE_CONST, {.i64=MODE_MERGE},             0, 0, FLAGS, .unit = "mode"},
    {"drop_even",         "drop even fields",                             0, AV_OPT_TYPE_CONST, {.i64=MODE_DROP_EVEN},         0, 0, FLAGS, .unit = "mode"},
    {"drop_odd",          "drop odd fields",                              0, AV_OPT_TYPE_CONST, {.i64=MODE_DROP_ODD},          0, 0, FLAGS, .unit = "mode"},
    {"pad",               "pad alternate lines with black",               0, AV_OPT_TYPE_CONST, {.i64=MODE_PAD},               0, 0, FLAGS, .unit = "mode"},
    {"interleave_top",    "interleave top and bottom fields",             0, AV_OPT_TYPE_CONST, {.i64=MODE_INTERLEAVE_TOP},    0, 0, FLAGS, .unit = "mode"},
    {"interleave_bottom", "interleave bottom and top fields",             0, AV_OPT_TYPE_CONST, {.i64=MODE_INTERLEAVE_BOTTOM}, 0, 0, FLAGS, .unit = "mode"},
    {"interlacex2",       "interlace fields from two consecutive frames", 0, AV_OPT_TYPE_CONST, {.i64=MODE_INTERLACEX2},       0, 0, FLAGS, .unit = "mode"},
    {"mergex2",           "merge fields keeping same frame rate",         0, AV_OPT_TYPE_CONST, {.i64=MODE_MERGEX2},           0, 0, FLAGS, .unit = "mode"},

    {"flags",             "set flags", OFFSET(flags), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, INT_MAX, 0, .unit = "flags" },
    {"low_pass_filter",   "enable vertical low-pass filter",              0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_VLPF}, 0, 0, FLAGS, .unit = "flags" },
    {"vlpf",              "enable vertical low-pass filter",              0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_VLPF}, 0, 0, FLAGS, .unit = "flags" },
    {"complex_filter",    "enable complex vertical low-pass filter",      0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_CVLPF},0, 0, FLAGS, .unit = "flags" },
    {"cvlpf",             "enable complex vertical low-pass filter",      0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_CVLPF},0, 0, FLAGS, .unit = "flags" },
    {"exact_tb",          "force a timebase which can represent timestamps exactly", 0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_EXACT_TB}, 0, 0, FLAGS, .unit = "flags" },
    {"bypass_il",         "bypass already interlaced frames",             0, AV_OPT_TYPE_CONST, {.i64 = TINTERLACE_FLAG_BYPASS_IL}, 0, 0, FLAGS, .unit = "flags" },

    {NULL}
};

AVFILTER_DEFINE_CLASS(tinterlace);

static const AVOption interlace_options[] = {
   { "scan",              "scanning mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = MODE_TFF}, 0, 1, FLAGS, .unit = "mode"},
   { "tff",               "top field first",                              0, AV_OPT_TYPE_CONST, {.i64 = MODE_TFF}, 0, 0, FLAGS, .unit = "mode"},
   { "bff",               "bottom field first",                           0, AV_OPT_TYPE_CONST, {.i64 = MODE_BFF}, 0, 0, FLAGS, .unit = "mode"},
   { "lowpass",           "set vertical low-pass filter", OFFSET(lowpass), AV_OPT_TYPE_INT,   {.i64 = VLPF_LIN}, 0, 2, FLAGS, .unit = "lowpass" },
   {     "off",           "disable vertical low-pass filter",             0, AV_OPT_TYPE_CONST, {.i64 = VLPF_OFF}, 0, 0, FLAGS, .unit = "lowpass" },
   {     "linear",        "linear vertical low-pass filter",              0, AV_OPT_TYPE_CONST, {.i64 = VLPF_LIN}, 0, 0, FLAGS, .unit = "lowpass" },
   {     "complex",       "complex vertical low-pass filter",             0, AV_OPT_TYPE_CONST, {.i64 = VLPF_CMP}, 0, 0, FLAGS, .unit = "lowpass" },

   { NULL }
};

AVFILTER_DEFINE_CLASS(interlace);

#define FULL_SCALE_YUVJ_FORMATS \
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P

static const enum AVPixelFormat full_scale_yuvj_pix_fmts[] = {
    FULL_SCALE_YUVJ_FORMATS, AV_PIX_FMT_NONE
};

static const AVRational standard_tbs[] = {
    {1, 25},
    {1, 30},
    {1001, 30000},
};

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_YUV422P10LE,
    AV_PIX_FMT_YUV440P10LE, AV_PIX_FMT_YUV444P10LE,
    AV_PIX_FMT_YUV420P12LE, AV_PIX_FMT_YUV422P12LE,
    AV_PIX_FMT_YUV440P12LE, AV_PIX_FMT_YUV444P12LE,
    AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA420P10LE, AV_PIX_FMT_YUVA422P10LE, AV_PIX_FMT_YUVA444P10LE,
    AV_PIX_FMT_GRAY8, FULL_SCALE_YUVJ_FORMATS,
    AV_PIX_FMT_NONE
};

static void lowpass_line_c(uint8_t *dstp, ptrdiff_t width, const uint8_t *srcp,
                           ptrdiff_t mref, ptrdiff_t pref, int clip_max)
{
    const uint8_t *srcp_above = srcp + mref;
    const uint8_t *srcp_below = srcp + pref;
    int i;
    for (i = 0; i < width; i++) {
        // this calculation is an integer representation of
        // '0.5 * current + 0.25 * above + 0.25 * below'
        // '1 +' is for rounding.
        dstp[i] = (1 + srcp[i] + srcp[i] + srcp_above[i] + srcp_below[i]) >> 2;
    }
}

static void lowpass_line_c_16(uint8_t *dst8, ptrdiff_t width, const uint8_t *src8,
                              ptrdiff_t mref, ptrdiff_t pref, int clip_max)
{
    uint16_t *dstp = (uint16_t *)dst8;
    const uint16_t *srcp = (const uint16_t *)src8;
    const uint16_t *srcp_above = srcp + mref / 2;
    const uint16_t *srcp_below = srcp + pref / 2;
    int i, src_x;
    for (i = 0; i < width; i++) {
        // this calculation is an integer representation of
        // '0.5 * current + 0.25 * above + 0.25 * below'
        // '1 +' is for rounding.
        src_x   = av_le2ne16(srcp[i]) << 1;
        dstp[i] = av_le2ne16((1 + src_x + av_le2ne16(srcp_above[i])
                             + av_le2ne16(srcp_below[i])) >> 2);
    }
}

static void lowpass_line_complex_c(uint8_t *dstp, ptrdiff_t width, const uint8_t *srcp,
                                   ptrdiff_t mref, ptrdiff_t pref, int clip_max)
{
    const uint8_t *srcp_above = srcp + mref;
    const uint8_t *srcp_below = srcp + pref;
    const uint8_t *srcp_above2 = srcp + mref * 2;
    const uint8_t *srcp_below2 = srcp + pref * 2;
    int i, src_x, src_ab;
    for (i = 0; i < width; i++) {
        // this calculation is an integer representation of
        // '0.75 * current + 0.25 * above + 0.25 * below - 0.125 * above2 - 0.125 * below2'
        // '4 +' is for rounding.
        src_x   = srcp[i] << 1;
        src_ab  = srcp_above[i] + srcp_below[i];
        dstp[i] = av_clip_uint8((4 + ((srcp[i] + src_x + src_ab) << 1)
                                - srcp_above2[i] - srcp_below2[i]) >> 3);
        // Prevent over-sharpening:
        // dst must not exceed src when the average of above and below
        // is less than src. And the other way around.
        if (src_ab > src_x) {
            if (dstp[i] < srcp[i])
                dstp[i] = srcp[i];
        } else if (dstp[i] > srcp[i])
            dstp[i] = srcp[i];
    }
}

static void lowpass_line_complex_c_16(uint8_t *dst8, ptrdiff_t width, const uint8_t *src8,
                                      ptrdiff_t mref, ptrdiff_t pref, int clip_max)
{
    uint16_t *dstp = (uint16_t *)dst8;
    const uint16_t *srcp = (const uint16_t *)src8;
    const uint16_t *srcp_above = srcp + mref / 2;
    const uint16_t *srcp_below = srcp + pref / 2;
    const uint16_t *srcp_above2 = srcp + mref;
    const uint16_t *srcp_below2 = srcp + pref;
    int i, dst_le, src_le, src_x, src_ab;
    for (i = 0; i < width; i++) {
        // this calculation is an integer representation of
        // '0.75 * current + 0.25 * above + 0.25 * below - 0.125 * above2 - 0.125 * below2'
        // '4 +' is for rounding.
        src_le = av_le2ne16(srcp[i]);
        src_x  = src_le << 1;
        src_ab = av_le2ne16(srcp_above[i]) + av_le2ne16(srcp_below[i]);
        dst_le = av_clip((4 + ((src_le + src_x + src_ab) << 1)
                         - av_le2ne16(srcp_above2[i])
                         - av_le2ne16(srcp_below2[i])) >> 3, 0, clip_max);
        // Prevent over-sharpening:
        // dst must not exceed src when the average of above and below
        // is less than src. And the other way around.
        if (src_ab > src_x) {
            if (dst_le < src_le)
                dstp[i] = av_le2ne16(src_le);
            else
                dstp[i] = av_le2ne16(dst_le);
        } else if (dst_le > src_le) {
            dstp[i] = av_le2ne16(src_le);
        } else
            dstp[i] = av_le2ne16(dst_le);
    }
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TInterlaceContext *tinterlace = ctx->priv;

    av_frame_free(&tinterlace->cur );
    av_frame_free(&tinterlace->next);
    av_freep(&tinterlace->black_data[0][0]);
    av_freep(&tinterlace->black_data[1][0]);
    ff_ccfifo_uninit(&tinterlace->cc_fifo);
}

static int config_out_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    FilterLink *il = ff_filter_link(inlink);
    FilterLink *ol = ff_filter_link(outlink);
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    TInterlaceContext *tinterlace = ctx->priv;
    int ret, i;

    tinterlace->vsub = desc->log2_chroma_h;
    outlink->w = inlink->w;
    outlink->h = tinterlace->mode == MODE_MERGE || tinterlace->mode == MODE_PAD || tinterlace->mode == MODE_MERGEX2?
        inlink->h*2 : inlink->h;
    if (tinterlace->mode == MODE_MERGE || tinterlace->mode == MODE_PAD || tinterlace->mode == MODE_MERGEX2)
        outlink->sample_aspect_ratio = av_mul_q(inlink->sample_aspect_ratio,
                                                av_make_q(2, 1));

    if (tinterlace->mode == MODE_PAD) {
        uint8_t black[4] = { 0, 0, 0, 16 };
        ret = ff_draw_init2(&tinterlace->draw, outlink->format, outlink->colorspace, outlink->color_range, 0);
        if (ret < 0)
            return ret;
        ff_draw_color(&tinterlace->draw, &tinterlace->color, black);
        /* limited range */
        if (!ff_fmt_is_in(outlink->format, full_scale_yuvj_pix_fmts)) {
            ret = av_image_alloc(tinterlace->black_data[0], tinterlace->black_linesize,
                                 outlink->w, outlink->h, outlink->format, 16);
            if (ret < 0)
                return ret;
            ff_fill_rectangle(&tinterlace->draw, &tinterlace->color, tinterlace->black_data[0],
                              tinterlace->black_linesize, 0, 0, outlink->w, outlink->h);
        }
        /* full range */
        tinterlace->color.comp[0].u8[0] = 0;
        ret = av_image_alloc(tinterlace->black_data[1], tinterlace->black_linesize,
                             outlink->w, outlink->h, outlink->format, 16);
        if (ret < 0)
            return ret;
        ff_fill_rectangle(&tinterlace->draw, &tinterlace->color, tinterlace->black_data[1],
                          tinterlace->black_linesize, 0, 0, outlink->w, outlink->h);
    }
    if (tinterlace->flags & (TINTERLACE_FLAG_VLPF | TINTERLACE_FLAG_CVLPF)
            && !(tinterlace->mode == MODE_INTERLEAVE_TOP
              || tinterlace->mode == MODE_INTERLEAVE_BOTTOM)) {
        av_log(ctx, AV_LOG_WARNING, "low_pass_filter flags ignored with mode %d\n",
                tinterlace->mode);
        tinterlace->flags &= ~(TINTERLACE_FLAG_VLPF | TINTERLACE_FLAG_CVLPF);
    }
    tinterlace->preout_time_base = inlink->time_base;
    if (tinterlace->mode == MODE_INTERLACEX2) {
        tinterlace->preout_time_base.den *= 2;
        ol->frame_rate = av_mul_q(il->frame_rate, (AVRational){2,1});
        outlink->time_base  = av_mul_q(inlink->time_base , (AVRational){1,2});
    } else if (tinterlace->mode == MODE_MERGEX2) {
        ol->frame_rate = il->frame_rate;
        outlink->time_base  = inlink->time_base;
    } else if (tinterlace->mode != MODE_PAD) {
        ol->frame_rate = av_mul_q(il->frame_rate, (AVRational){1,2});
        outlink->time_base  = av_mul_q(inlink->time_base , (AVRational){2,1});
    }

    for (i = 0; i<FF_ARRAY_ELEMS(standard_tbs); i++){
        if (!av_cmp_q(standard_tbs[i], outlink->time_base))
            break;
    }
    if (i == FF_ARRAY_ELEMS(standard_tbs) ||
        (tinterlace->flags & TINTERLACE_FLAG_EXACT_TB))
        outlink->time_base = tinterlace->preout_time_base;

    tinterlace->csp = av_pix_fmt_desc_get(outlink->format);
    if (tinterlace->flags & TINTERLACE_FLAG_CVLPF) {
        if (tinterlace->csp->comp[0].depth > 8)
            tinterlace->lowpass_line = lowpass_line_complex_c_16;
        else
            tinterlace->lowpass_line = lowpass_line_complex_c;
#if ARCH_X86
        ff_tinterlace_init_x86(tinterlace);
#endif
    } else if (tinterlace->flags & TINTERLACE_FLAG_VLPF) {
        if (tinterlace->csp->comp[0].depth > 8)
            tinterlace->lowpass_line = lowpass_line_c_16;
        else
            tinterlace->lowpass_line = lowpass_line_c;
#if ARCH_X86
        ff_tinterlace_init_x86(tinterlace);
#endif
    }

    ret = ff_ccfifo_init(&tinterlace->cc_fifo, ol->frame_rate, ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failure to setup CC FIFO queue\n");
        return ret;
    }

    av_log(ctx, AV_LOG_VERBOSE, "mode:%d filter:%s h:%d -> h:%d\n", tinterlace->mode,
           (tinterlace->flags & TINTERLACE_FLAG_CVLPF) ? "complex" :
           (tinterlace->flags & TINTERLACE_FLAG_VLPF) ? "linear" : "off",
           inlink->h, outlink->h);

    return 0;
}

#define FIELD_UPPER           0
#define FIELD_LOWER           1
#define FIELD_UPPER_AND_LOWER 2

/**
 * Copy picture field from src to dst.
 *
 * @param src_field copy from upper, lower field or both
 * @param interleave leave a padding line between each copied line
 * @param dst_field copy to upper or lower field,
 *        only meaningful when interleave is selected
 * @param flags context flags
 */
static inline
void copy_picture_field(TInterlaceContext *tinterlace,
                        uint8_t *dst[4], int dst_linesize[4],
                        const uint8_t *src[4], int src_linesize[4],
                        enum AVPixelFormat format, int w, int src_h,
                        int src_field, int interleave, int dst_field,
                        int flags)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(format);
    int hsub = desc->log2_chroma_w;
    int plane, vsub = desc->log2_chroma_h;
    int k = src_field == FIELD_UPPER_AND_LOWER ? 1 : 2;
    int h;

    for (plane = 0; plane < desc->nb_components; plane++) {
        int lines = plane == 1 || plane == 2 ? AV_CEIL_RSHIFT(src_h, vsub) : src_h;
        int cols  = plane == 1 || plane == 2 ? AV_CEIL_RSHIFT(    w, hsub) : w;
        uint8_t *dstp = dst[plane];
        const uint8_t *srcp = src[plane];
        int srcp_linesize = src_linesize[plane] * k;
        int dstp_linesize = dst_linesize[plane] * (interleave ? 2 : 1);
        int clip_max = (1 << tinterlace->csp->comp[plane].depth) - 1;

        lines = (lines + (src_field == FIELD_UPPER)) / k;
        if (src_field == FIELD_LOWER)
            srcp += src_linesize[plane];
        if (interleave && dst_field == FIELD_LOWER)
            dstp += dst_linesize[plane];
        // Low-pass filtering is required when creating an interlaced destination from
        // a progressive source which contains high-frequency vertical detail.
        // Filtering will reduce interlace 'twitter' and Moire patterning.
        if (flags & (TINTERLACE_FLAG_VLPF | TINTERLACE_FLAG_CVLPF)) {
            int x = !!(flags & TINTERLACE_FLAG_CVLPF);
            for (h = lines; h > 0; h--) {
                ptrdiff_t pref = src_linesize[plane];
                ptrdiff_t mref = -pref;
                if (h >= (lines - x))  mref = 0; // there is no line above
                else if (h <= (1 + x)) pref = 0; // there is no line below

                tinterlace->lowpass_line(dstp, cols, srcp, mref, pref, clip_max);
                dstp += dstp_linesize;
                srcp += srcp_linesize;
            }
        } else {
            if (tinterlace->csp->comp[plane].depth > 8)
                cols *= 2;
            av_image_copy_plane(dstp, dstp_linesize, srcp, srcp_linesize, cols, lines);
        }
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *picref)
{
    FilterLink *inl = ff_filter_link(inlink);
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *l = ff_filter_link(outlink);
    TInterlaceContext *tinterlace = ctx->priv;
    AVFrame *cur, *next, *out;
    int field, tff, full, ret;

    av_frame_free(&tinterlace->cur);
    tinterlace->cur  = tinterlace->next;
    tinterlace->next = picref;

    ff_ccfifo_extract(&tinterlace->cc_fifo, picref);

    cur = tinterlace->cur;
    next = tinterlace->next;
    /* we need at least two frames */
    if (!tinterlace->cur)
        return 0;

    switch (tinterlace->mode) {
    case MODE_MERGEX2: /* move the odd frame into the upper field of the new image, even into
                        * the lower field, generating a double-height video at same framerate */
    case MODE_MERGE: /* move the odd frame into the upper field of the new image, even into
             * the lower field, generating a double-height video at half framerate */
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out, cur);
        out->height = outlink->h;
        out->flags |= AV_FRAME_FLAG_INTERLACED | AV_FRAME_FLAG_TOP_FIELD_FIRST;
        out->sample_aspect_ratio = av_mul_q(cur->sample_aspect_ratio, av_make_q(2, 1));

        /* write odd frame lines into the upper field of the new frame */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)cur->data, cur->linesize,
                           inlink->format, inlink->w, inlink->h,
                           FIELD_UPPER_AND_LOWER, 1, tinterlace->mode == MODE_MERGEX2 ? (1 + inl->frame_count_out) & 1 ? FIELD_LOWER : FIELD_UPPER : FIELD_UPPER, tinterlace->flags);
        /* write even frame lines into the lower field of the new frame */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)next->data, next->linesize,
                           inlink->format, inlink->w, inlink->h,
                           FIELD_UPPER_AND_LOWER, 1, tinterlace->mode == MODE_MERGEX2 ? (1 + inl->frame_count_out) & 1 ? FIELD_UPPER : FIELD_LOWER : FIELD_LOWER, tinterlace->flags);
        if (tinterlace->mode != MODE_MERGEX2)
            av_frame_free(&tinterlace->next);
        break;

    case MODE_DROP_ODD:  /* only output even frames, odd  frames are dropped; height unchanged, half framerate */
    case MODE_DROP_EVEN: /* only output odd  frames, even frames are dropped; height unchanged, half framerate */
        out = av_frame_clone(tinterlace->mode == MODE_DROP_EVEN ? cur : next);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_free(&tinterlace->next);
        break;

    case MODE_PAD: /* expand each frame to double height, but pad alternate
                    * lines with black; framerate unchanged */
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out, cur);
        out->height = outlink->h;
        out->sample_aspect_ratio = av_mul_q(cur->sample_aspect_ratio, av_make_q(2, 1));

        field = (1 + l->frame_count_in) & 1 ? FIELD_UPPER : FIELD_LOWER;
        full = out->color_range == AVCOL_RANGE_JPEG || ff_fmt_is_in(out->format, full_scale_yuvj_pix_fmts);
        /* copy upper and lower fields */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)cur->data, cur->linesize,
                           inlink->format, inlink->w, inlink->h,
                           FIELD_UPPER_AND_LOWER, 1, field, tinterlace->flags);
        /* pad with black the other field */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)tinterlace->black_data[full], tinterlace->black_linesize,
                           inlink->format, inlink->w, inlink->h,
                           FIELD_UPPER_AND_LOWER, 1, !field, tinterlace->flags);
        break;

        /* interleave upper/lower lines from odd frames with lower/upper lines from even frames,
         * halving the frame rate and preserving image height */
    case MODE_INTERLEAVE_TOP:    /* top    field first */
    case MODE_INTERLEAVE_BOTTOM: /* bottom field first */
        if ((tinterlace->flags & TINTERLACE_FLAG_BYPASS_IL) && (cur->flags & AV_FRAME_FLAG_INTERLACED)) {
            av_log(ctx, AV_LOG_WARNING,
                   "video is already interlaced, adjusting framerate only\n");
            out = av_frame_clone(cur);
            if (!out)
                return AVERROR(ENOMEM);
            out->pts /= 2;  // adjust pts to new framerate
            ff_ccfifo_inject(&tinterlace->cc_fifo, out);
            ret = ff_filter_frame(outlink, out);
            return ret;
        }
        tff = tinterlace->mode == MODE_INTERLEAVE_TOP;
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out, cur);
        out->flags |= AV_FRAME_FLAG_INTERLACED;
        if (tff)
            out->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;
        else
            out->flags &= ~AV_FRAME_FLAG_TOP_FIELD_FIRST;

        /* copy upper/lower field from cur */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)cur->data, cur->linesize,
                           inlink->format, inlink->w, inlink->h,
                           tff ? FIELD_UPPER : FIELD_LOWER, 1, tff ? FIELD_UPPER : FIELD_LOWER,
                           tinterlace->flags);
        /* copy lower/upper field from next */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)next->data, next->linesize,
                           inlink->format, inlink->w, inlink->h,
                           tff ? FIELD_LOWER : FIELD_UPPER, 1, tff ? FIELD_LOWER : FIELD_UPPER,
                           tinterlace->flags);
        av_frame_free(&tinterlace->next);
        break;
    case MODE_INTERLACEX2: /* re-interlace preserving image height, double frame rate */
        /* output current frame first */
        out = av_frame_clone(cur);
        if (!out)
            return AVERROR(ENOMEM);
        out->flags |= AV_FRAME_FLAG_INTERLACED;
        if (cur->pts != AV_NOPTS_VALUE)
            out->pts = cur->pts*2;

        out->pts = av_rescale_q(out->pts, tinterlace->preout_time_base, outlink->time_base);
        ff_ccfifo_inject(&tinterlace->cc_fifo, out);
        if ((ret = ff_filter_frame(outlink, out)) < 0)
            return ret;

        /* output mix of current and next frame */
        tff = !!(next->flags & AV_FRAME_FLAG_TOP_FIELD_FIRST);
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);
        av_frame_copy_props(out, next);
        out->flags |= AV_FRAME_FLAG_INTERLACED;
        if (tff)
            out->flags &= ~AV_FRAME_FLAG_TOP_FIELD_FIRST;
        else
            out->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;

        if (next->pts != AV_NOPTS_VALUE && cur->pts != AV_NOPTS_VALUE)
            out->pts = cur->pts + next->pts;
        else
            out->pts = AV_NOPTS_VALUE;
        /* write current frame second field lines into the second field of the new frame */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)cur->data, cur->linesize,
                           inlink->format, inlink->w, inlink->h,
                           tff ? FIELD_LOWER : FIELD_UPPER, 1, tff ? FIELD_LOWER : FIELD_UPPER,
                           tinterlace->flags);
        /* write next frame first field lines into the first field of the new frame */
        copy_picture_field(tinterlace, out->data, out->linesize,
                           (const uint8_t **)next->data, next->linesize,
                           inlink->format, inlink->w, inlink->h,
                           tff ? FIELD_UPPER : FIELD_LOWER, 1, tff ? FIELD_UPPER : FIELD_LOWER,
                           tinterlace->flags);
        break;
    default:
        av_assert0(0);
    }

    out->pts = av_rescale_q(out->pts, tinterlace->preout_time_base, outlink->time_base);
    out->duration = av_rescale_q(1, av_inv_q(l->frame_rate), outlink->time_base);
    ff_ccfifo_inject(&tinterlace->cc_fifo, out);
    ret = ff_filter_frame(outlink, out);

    return ret;
}

static int init_interlace(AVFilterContext *ctx)
{
    TInterlaceContext *tinterlace = ctx->priv;

    if (tinterlace->mode <= MODE_BFF)
        tinterlace->mode += MODE_INTERLEAVE_TOP;

    tinterlace->flags |= TINTERLACE_FLAG_BYPASS_IL;
    if (tinterlace->lowpass == VLPF_LIN)
        tinterlace->flags |= TINTERLACE_FLAG_VLPF;
    if (tinterlace->lowpass == VLPF_CMP)
        tinterlace->flags |= TINTERLACE_FLAG_CVLPF;

    return 0;
}

static const AVFilterPad tinterlace_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad tinterlace_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_out_props,
    },
};

const FFFilter ff_vf_tinterlace = {
    .p.name        = "tinterlace",
    .p.description = NULL_IF_CONFIG_SMALL("Perform temporal field interlacing."),
    .p.priv_class  = &tinterlace_class,
    .priv_size     = sizeof(TInterlaceContext),
    .uninit        = uninit,
    FILTER_INPUTS(tinterlace_inputs),
    FILTER_OUTPUTS(tinterlace_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};


const FFFilter ff_vf_interlace = {
    .p.name        = "interlace",
    .p.description = NULL_IF_CONFIG_SMALL("Convert progressive video into interlaced."),
    .p.priv_class  = &interlace_class,
    .priv_size     = sizeof(TInterlaceContext),
    .init          = init_interlace,
    .uninit        = uninit,
    FILTER_INPUTS(tinterlace_inputs),
    FILTER_OUTPUTS(tinterlace_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
