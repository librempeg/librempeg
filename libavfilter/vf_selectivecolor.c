/*
 * Copyright (c) 2015-2016 Clément Bœsch <u pkh me>
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
 * @see http://blog.pkh.me/p/22-understanding-selective-coloring-in-adobe-photoshop.html
 * @todo
 * - use integers so it can be made bitexact and a FATE test can be added
 */

#include "libavutil/avassert.h"
#include "libavutil/file.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavcodec/mathops.h" // for mid_pred(), which is a macro so no link dependency
#include "avfilter.h"
#include "drawutils.h"
#include "filters.h"
#include "video.h"

#define R 0
#define G 1
#define B 2
#define A 3

enum color_range {
    // WARNING: do NOT reorder (see parse_psfile())
    RANGE_REDS,
    RANGE_YELLOWS,
    RANGE_GREENS,
    RANGE_CYANS,
    RANGE_BLUES,
    RANGE_MAGENTAS,
    RANGE_WHITES,
    RANGE_NEUTRALS,
    RANGE_BLACKS,
    NB_RANGES
};

enum correction_method {
    CORRECTION_METHOD_ABSOLUTE,
    CORRECTION_METHOD_RELATIVE,
    NB_CORRECTION_METHODS,
};

static const char *const color_names[NB_RANGES] = {
    "red", "yellow", "green", "cyan", "blue", "magenta", "white", "neutral", "black"
};

typedef int (*get_range_scale_func)(int r, int g, int b, int min_val, int max_val);

struct process_range {
    int range_id;
    uint32_t mask;
    get_range_scale_func get_scale;
};

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

typedef struct SelectiveColorContext {
    const AVClass *class;
    int correction_method;
    char *opt_cmyk_adjust[NB_RANGES];
    float cmyk_adjust[NB_RANGES][4];
    struct process_range process_ranges[NB_RANGES]; // color ranges to process
    int nb_process_ranges;
    char *psfile;
    uint8_t rgba_map[4];
    int is_16bit;
    int step;
} SelectiveColorContext;

#define OFFSET(x) offsetof(SelectiveColorContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define RANGE_OPTION(color_name, range) \
    { color_name"s", "adjust "color_name" regions", OFFSET(opt_cmyk_adjust[range]), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, FLAGS }

static const AVOption selectivecolor_options[] = {
    { "correction_method", "select correction method", OFFSET(correction_method), AV_OPT_TYPE_INT, {.i64 = CORRECTION_METHOD_ABSOLUTE}, 0, NB_CORRECTION_METHODS-1, FLAGS, .unit = "correction_method" },
        { "absolute", NULL, 0, AV_OPT_TYPE_CONST, {.i64=CORRECTION_METHOD_ABSOLUTE}, 0, 0, FLAGS, .unit = "correction_method" },
        { "relative", NULL, 0, AV_OPT_TYPE_CONST, {.i64=CORRECTION_METHOD_RELATIVE}, 0, 0, FLAGS, .unit = "correction_method" },
    RANGE_OPTION("red",     RANGE_REDS),
    RANGE_OPTION("yellow",  RANGE_YELLOWS),
    RANGE_OPTION("green",   RANGE_GREENS),
    RANGE_OPTION("cyan",    RANGE_CYANS),
    RANGE_OPTION("blue",    RANGE_BLUES),
    RANGE_OPTION("magenta", RANGE_MAGENTAS),
    RANGE_OPTION("white",   RANGE_WHITES),
    RANGE_OPTION("neutral", RANGE_NEUTRALS),
    RANGE_OPTION("black",   RANGE_BLACKS),
    { "psfile", "set Photoshop selectivecolor file name", OFFSET(psfile), AV_OPT_TYPE_STRING, {.str=NULL}, .flags = FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(selectivecolor);

static int get_rgb_scale(int r, int g, int b, int min_val, int max_val)
{
    return max_val - mid_pred(r, g, b);
}

static int get_cmy_scale(int r, int g, int b, int min_val, int max_val)
{
    return mid_pred(r, g, b) - min_val;
}

#define DECLARE_RANGE_SCALE_FUNCS(nbits)                                                    \
static int get_neutrals_scale##nbits(int r, int g, int b, int min_val, int max_val)         \
{                                                                                           \
    /* 1 - (|max-0.5| + |min-0.5|) */                                                       \
    return (((1<<nbits)-1)*2 - (  abs((max_val<<1) - ((1<<nbits)-1))                        \
                                + abs((min_val<<1) - ((1<<nbits)-1))) + 1) >> 1;            \
}                                                                                           \
                                                                                            \
static int get_whites_scale##nbits(int r, int g, int b, int min_val, int max_val)           \
{                                                                                           \
    /* (min - 0.5) * 2 */                                                                   \
    return (min_val<<1) - ((1<<nbits)-1);                                                   \
}                                                                                           \
                                                                                            \
static int get_blacks_scale##nbits(int r, int g, int b, int min_val, int max_val)           \
{                                                                                           \
    /* (0.5 - max) * 2 */                                                                   \
    return ((1<<nbits)-1) - (max_val<<1);                                                   \
}                                                                                           \

DECLARE_RANGE_SCALE_FUNCS(8)
DECLARE_RANGE_SCALE_FUNCS(16)

static int register_range(SelectiveColorContext *s, int range_id)
{
    const float *cmyk = s->cmyk_adjust[range_id];

    /* If the color range has user settings, register the color range
     * as "to be processed" */
    if (cmyk[0] || cmyk[1] || cmyk[2] || cmyk[3]) {
        struct process_range *pr = &s->process_ranges[s->nb_process_ranges++];

        if (cmyk[0] < -1.0 || cmyk[0] > 1.0 ||
            cmyk[1] < -1.0 || cmyk[1] > 1.0 ||
            cmyk[2] < -1.0 || cmyk[2] > 1.0 ||
            cmyk[3] < -1.0 || cmyk[3] > 1.0) {
            av_log(s, AV_LOG_ERROR, "Invalid %s adjustments (%g %g %g %g). "
                   "Settings must be set in [-1;1] range\n",
                   color_names[range_id], cmyk[0], cmyk[1], cmyk[2], cmyk[3]);
            return AVERROR(EINVAL);
        }

        pr->range_id = range_id;
        pr->mask = 1 << range_id;
        if      (pr->mask & (1<<RANGE_REDS  | 1<<RANGE_GREENS   | 1<<RANGE_BLUES))   pr->get_scale = get_rgb_scale;
        else if (pr->mask & (1<<RANGE_CYANS | 1<<RANGE_MAGENTAS | 1<<RANGE_YELLOWS)) pr->get_scale = get_cmy_scale;
        else if (!s->is_16bit && (pr->mask & 1<<RANGE_WHITES))                       pr->get_scale = get_whites_scale8;
        else if (!s->is_16bit && (pr->mask & 1<<RANGE_NEUTRALS))                     pr->get_scale = get_neutrals_scale8;
        else if (!s->is_16bit && (pr->mask & 1<<RANGE_BLACKS))                       pr->get_scale = get_blacks_scale8;
        else if ( s->is_16bit && (pr->mask & 1<<RANGE_WHITES))                       pr->get_scale = get_whites_scale16;
        else if ( s->is_16bit && (pr->mask & 1<<RANGE_NEUTRALS))                     pr->get_scale = get_neutrals_scale16;
        else if ( s->is_16bit && (pr->mask & 1<<RANGE_BLACKS))                       pr->get_scale = get_blacks_scale16;
        else
            av_assert0(0);
    }
    return 0;
}

static int parse_psfile(AVFilterContext *ctx, const char *fname)
{
    int16_t val;
    int ret, i, version;
    uint8_t *buf;
    size_t size;
    SelectiveColorContext *s = ctx->priv;

    ret = av_file_map(fname, &buf, &size, 0, NULL);
    if (ret < 0)
        return ret;

#define READ16(dst) do {                \
    if (size < 2) {                     \
        ret = AVERROR_INVALIDDATA;      \
        goto end;                       \
    }                                   \
    dst = AV_RB16(buf);                 \
    buf  += 2;                          \
    size -= 2;                          \
} while (0)

    READ16(version);
    if (version != 1)
        av_log(s, AV_LOG_WARNING, "Unsupported selective color file version %d, "
               "the settings might not be loaded properly\n", version);

    READ16(s->correction_method);

    // 1st CMYK entry is reserved/unused
    for (i = 0; i < FF_ARRAY_ELEMS(s->cmyk_adjust[0]); i++) {
        READ16(val);
        if (val)
            av_log(s, AV_LOG_WARNING, "%c value of first CMYK entry is not 0 "
                   "but %d\n", "CMYK"[i], val);
    }

    for (i = 0; i < FF_ARRAY_ELEMS(s->cmyk_adjust); i++) {
        int k;
        for (k = 0; k < FF_ARRAY_ELEMS(s->cmyk_adjust[0]); k++) {
            READ16(val);
            s->cmyk_adjust[i][k] = val / 100.f;
        }
        ret = register_range(s, i);
        if (ret < 0)
            goto end;
    }

end:
    av_file_unmap(buf, size);
    return ret;
}

static int config_input(AVFilterLink *inlink)
{
    int i, ret;
    AVFilterContext *ctx = inlink->dst;
    SelectiveColorContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    s->is_16bit = desc->comp[0].depth > 8;
    s->step = av_get_padded_bits_per_pixel(desc) >> (3 + s->is_16bit);

    ret = ff_fill_rgba_map(s->rgba_map, inlink->format);
    if (ret < 0)
        return ret;

    /* If the following conditions are not met, it will cause trouble while
     * parsing the PS file */
    av_assert0(FF_ARRAY_ELEMS(s->cmyk_adjust) == 10 - 1);
    av_assert0(FF_ARRAY_ELEMS(s->cmyk_adjust[0]) == 4);

    if (s->psfile) {
        ret = parse_psfile(ctx, s->psfile);
        if (ret < 0)
            return ret;
    } else {
        for (i = 0; i < FF_ARRAY_ELEMS(s->opt_cmyk_adjust); i++) {
            const char *opt_cmyk_adjust = s->opt_cmyk_adjust[i];

            if (opt_cmyk_adjust) {
                float *cmyk = s->cmyk_adjust[i];

                sscanf(s->opt_cmyk_adjust[i], "%f %f %f %f", cmyk, cmyk+1, cmyk+2, cmyk+3);
                ret = register_range(s, i);
                if (ret < 0)
                    return ret;
            }
        }
    }

    av_log(s, AV_LOG_VERBOSE, "Adjustments:%s\n", s->nb_process_ranges ? "" : " none");
    for (i = 0; i < s->nb_process_ranges; i++) {
        const struct process_range *pr = &s->process_ranges[i];
        const float *cmyk = s->cmyk_adjust[pr->range_id];

        av_log(s, AV_LOG_VERBOSE, "%8ss: C=%6g M=%6g Y=%6g K=%6g\n",
               color_names[pr->range_id], cmyk[0], cmyk[1], cmyk[2], cmyk[3]);
    }

    return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_RGB24,  AV_PIX_FMT_BGR24,
    AV_PIX_FMT_RGBA,   AV_PIX_FMT_BGRA,
    AV_PIX_FMT_ARGB,   AV_PIX_FMT_ABGR,
    AV_PIX_FMT_0RGB,   AV_PIX_FMT_0BGR,
    AV_PIX_FMT_RGB0,   AV_PIX_FMT_BGR0,
    AV_PIX_FMT_RGB48,  AV_PIX_FMT_BGR48,
    AV_PIX_FMT_RGBA64, AV_PIX_FMT_BGRA64,
    AV_PIX_FMT_NONE
};

static inline int comp_adjust(int scale, float value, float adjust, float k, int correction_method)
{
    const float min = -value;
    const float max = 1.f - value;
    float res = (-1.f - adjust) * k - adjust;
    if (correction_method == CORRECTION_METHOD_RELATIVE)
        res *= max;
    return lrintf(av_clipf(res, min, max) * scale);
}

#define DECLARE_SELECTIVE_COLOR_FUNC(nbits)                                                             \
static inline int selective_color_##nbits(AVFilterContext *ctx, ThreadData *td,                         \
                                          int jobnr, int nb_jobs, int direct, int correction_method)    \
{                                                                                                       \
    int i, x, y;                                                                                        \
    const AVFrame *in = td->in;                                                                         \
    AVFrame *out = td->out;                                                                             \
    const SelectiveColorContext *s = ctx->priv;                                                         \
    const int height = in->height;                                                                      \
    const int width  = in->width;                                                                       \
    const int slice_start = (height *  jobnr   ) / nb_jobs;                                             \
    const int slice_end   = (height * (jobnr+1)) / nb_jobs;                                             \
    const int dst_linesize = out->linesize[0] / ((nbits + 7) / 8);                                      \
    const int src_linesize =  in->linesize[0] / ((nbits + 7) / 8);                                      \
    const uint8_t roffset = s->rgba_map[R];                                                             \
    const uint8_t goffset = s->rgba_map[G];                                                             \
    const uint8_t boffset = s->rgba_map[B];                                                             \
    const uint8_t aoffset = s->rgba_map[A];                                                             \
    uint##nbits##_t *dst = (uint##nbits##_t *)out->data[0] + slice_start * dst_linesize;                \
    const uint##nbits##_t *src = (const uint##nbits##_t *)in->data[0] + slice_start * src_linesize;     \
    const uint##nbits##_t *src_r = (const uint##nbits##_t *)src + roffset;                              \
    const uint##nbits##_t *src_g = (const uint##nbits##_t *)src + goffset;                              \
    const uint##nbits##_t *src_b = (const uint##nbits##_t *)src + boffset;                              \
    const uint##nbits##_t *src_a = (const uint##nbits##_t *)src + aoffset;                              \
    uint##nbits##_t *dst_r = (uint##nbits##_t *)dst + roffset;                                          \
    uint##nbits##_t *dst_g = (uint##nbits##_t *)dst + goffset;                                          \
    uint##nbits##_t *dst_b = (uint##nbits##_t *)dst + boffset;                                          \
    uint##nbits##_t *dst_a = (uint##nbits##_t *)dst + aoffset;                                          \
    const int mid = 1<<(nbits-1);                                                                       \
    const int max = (1<<nbits)-1;                                                                       \
    const float scale = 1.f / max;                                                                      \
                                                                                                        \
    for (y = slice_start; y < slice_end; y++) {                                                         \
        for (x = 0; x < width * s->step; x += s->step) {                                                \
            const int r = src_r[x];                                                                     \
            const int g = src_g[x];                                                                     \
            const int b = src_b[x];                                                                     \
            const int min_color = FFMIN3(r, g, b);                                                      \
            const int max_color = FFMAX3(r, g, b);                                                      \
            const int is_white   = (r > mid && g > mid && b > mid);                                     \
            const int is_neutral = (r || g || b) &&                                                     \
                                   (r != max || g != max || b != max);                                  \
            const int is_black   = (r < mid && g < mid && b < mid);                                     \
            const uint32_t range_flag = (r == max_color) << RANGE_REDS                                  \
                                      | (r == min_color) << RANGE_CYANS                                 \
                                      | (g == max_color) << RANGE_GREENS                                \
                                      | (g == min_color) << RANGE_MAGENTAS                              \
                                      | (b == max_color) << RANGE_BLUES                                 \
                                      | (b == min_color) << RANGE_YELLOWS                               \
                                      | is_white         << RANGE_WHITES                                \
                                      | is_neutral       << RANGE_NEUTRALS                              \
                                      | is_black         << RANGE_BLACKS;                               \
                                                                                                        \
            const float rnorm = r * scale;                                                              \
            const float gnorm = g * scale;                                                              \
            const float bnorm = b * scale;                                                              \
            int adjust_r = 0, adjust_g = 0, adjust_b = 0;                                               \
                                                                                                        \
            for (i = 0; i < s->nb_process_ranges; i++) {                                                \
                const struct process_range *pr = &s->process_ranges[i];                                 \
                                                                                                        \
                if (range_flag & pr->mask) {                                                            \
                    const int scale = pr->get_scale(r, g, b, min_color, max_color);                     \
                                                                                                        \
                    if (scale > 0) {                                                                    \
                        const float *cmyk_adjust = s->cmyk_adjust[pr->range_id];                        \
                        const float adj_c = cmyk_adjust[0];                                             \
                        const float adj_m = cmyk_adjust[1];                                             \
                        const float adj_y = cmyk_adjust[2];                                             \
                        const float k = cmyk_adjust[3];                                                 \
                                                                                                        \
                        adjust_r += comp_adjust(scale, rnorm, adj_c, k, correction_method);             \
                        adjust_g += comp_adjust(scale, gnorm, adj_m, k, correction_method);             \
                        adjust_b += comp_adjust(scale, bnorm, adj_y, k, correction_method);             \
                    }                                                                                   \
                }                                                                                       \
            }                                                                                           \
                                                                                                        \
            if (!direct || adjust_r || adjust_g || adjust_b) {                                          \
                dst_r[x] = av_clip_uint##nbits(r + adjust_r);                                           \
                dst_g[x] = av_clip_uint##nbits(g + adjust_g);                                           \
                dst_b[x] = av_clip_uint##nbits(b + adjust_b);                                           \
                if (!direct && s->step == 4)                                                            \
                    dst_a[x] = src_a[x];                                                                \
            }                                                                                           \
        }                                                                                               \
                                                                                                        \
        src_r += src_linesize;                                                                          \
        src_g += src_linesize;                                                                          \
        src_b += src_linesize;                                                                          \
        src_a += src_linesize;                                                                          \
                                                                                                        \
        dst_r += dst_linesize;                                                                          \
        dst_g += dst_linesize;                                                                          \
        dst_b += dst_linesize;                                                                          \
        dst_a += dst_linesize;                                                                          \
    }                                                                                                   \
    return 0;                                                                                           \
}

#define DEF_SELECTIVE_COLOR_FUNC(name, direct, correction_method, nbits)                                \
static int selective_color_##name##_##nbits(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)    \
{                                                                                                       \
    return selective_color_##nbits(ctx, arg, jobnr, nb_jobs, direct, correction_method);                \
}

#define DEF_SELECTIVE_COLOR_FUNCS(nbits)                                                                \
DECLARE_SELECTIVE_COLOR_FUNC(nbits)                                                                     \
DEF_SELECTIVE_COLOR_FUNC(indirect_absolute, 0, CORRECTION_METHOD_ABSOLUTE, nbits)                       \
DEF_SELECTIVE_COLOR_FUNC(indirect_relative, 0, CORRECTION_METHOD_RELATIVE, nbits)                       \
DEF_SELECTIVE_COLOR_FUNC(  direct_absolute, 1, CORRECTION_METHOD_ABSOLUTE, nbits)                       \
DEF_SELECTIVE_COLOR_FUNC(  direct_relative, 1, CORRECTION_METHOD_RELATIVE, nbits)

DEF_SELECTIVE_COLOR_FUNCS(8)
DEF_SELECTIVE_COLOR_FUNCS(16)

typedef int (*selective_color_func_type)(AVFilterContext *ctx, void *td, int jobnr, int nb_jobs);

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    int direct;
    AVFrame *out;
    ThreadData td;
    const SelectiveColorContext *s = ctx->priv;
    static const selective_color_func_type funcs[2][2][2] = {
        {
            {selective_color_indirect_absolute_8, selective_color_indirect_relative_8},
            {selective_color_direct_absolute_8,   selective_color_direct_relative_8},
        },{
            {selective_color_indirect_absolute_16, selective_color_indirect_relative_16},
            {selective_color_direct_absolute_16,   selective_color_direct_relative_16},
        }
    };

    if (av_frame_is_writable(in)) {
        direct = 1;
        out = in;
    } else {
        direct = 0;
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, funcs[s->is_16bit][direct][s->correction_method],
                      &td, NULL, FFMIN(inlink->h, ff_filter_get_nb_threads(ctx)));

    if (!direct)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static const AVFilterPad selectivecolor_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_selectivecolor = {
    .p.name        = "selectivecolor",
    .p.description = NULL_IF_CONFIG_SMALL("Apply CMYK adjustments to specific color ranges."),
    .p.priv_class  = &selectivecolor_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(SelectiveColorContext),
    FILTER_INPUTS(selectivecolor_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
