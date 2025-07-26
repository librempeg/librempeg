/*
 * Copyright (c) 2020 Paul B Mahol
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

#include "avfilter.h"
#include "filters.h"
#include "video.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/lfg.h"
#include "libavutil/random_seed.h"
#include <float.h>
#include <math.h>

typedef struct FloatColor {
    float c[4];
} FloatColor;

typedef struct GradientsContext {
    const AVClass *class;
    int w, h;
    int type;
    AVRational frame_rate;
    int64_t pts;
    int64_t duration;           ///< duration expressed in microseconds
    float speed;
    float angle;

    uint32_t *color_rgba;
    unsigned nb_colors;

    FloatColor *color_rgbaf;

    int x0, y0, x1, y1;
    float fx0, fy0, fx1, fy1;

    int64_t seed;

    AVLFG lfg;
    int (*draw_slice)(AVFilterContext *ctx, void *arg, int job, int nb_jobs);
} GradientsContext;

#define OFFSET(x) offsetof(GradientsContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define VFT AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_colors = {.def="random random",.size_min=2,.sep=' '};

static const AVOption gradients_options[] = {
    {"size",      "set frame size", OFFSET(w),             AV_OPT_TYPE_IMAGE_SIZE, {.str="640x480"},  0, 0, FLAGS },
    {"s",         "set frame size", OFFSET(w),             AV_OPT_TYPE_IMAGE_SIZE, {.str="640x480"},  0, 0, FLAGS },
    {"rate",      "set frame rate", OFFSET(frame_rate),    AV_OPT_TYPE_VIDEO_RATE, {.str="25"},       0, INT_MAX, FLAGS },
    {"r",         "set frame rate", OFFSET(frame_rate),    AV_OPT_TYPE_VIDEO_RATE, {.str="25"},       0, INT_MAX, FLAGS },
    {"colors",    "set the gradient colors",  OFFSET(color_rgba), AV_OPT_TYPE_COLOR|AR, {.arr=&def_colors}, 0, 0, FLAGS },
    {"c"     ,    "set the gradient colors",  OFFSET(color_rgba), AV_OPT_TYPE_COLOR|AR, {.arr=&def_colors}, 0, 0, FLAGS },
    {"x0",        "set gradient line source x0",      OFFSET(x0), AV_OPT_TYPE_INT, {.i64=-1},        -1, INT_MAX, FLAGS },
    {"y0",        "set gradient line source y0",      OFFSET(y0), AV_OPT_TYPE_INT, {.i64=-1},        -1, INT_MAX, FLAGS },
    {"x1",        "set gradient line destination x1", OFFSET(x1), AV_OPT_TYPE_INT, {.i64=-1},        -1, INT_MAX, FLAGS },
    {"y1",        "set gradient line destination y1", OFFSET(y1), AV_OPT_TYPE_INT, {.i64=-1},        -1, INT_MAX, FLAGS },
    {"seed",      "set the seed",   OFFSET(seed),          AV_OPT_TYPE_INT64,      {.i64=-1},        -1, UINT32_MAX, FLAGS },
    {"duration",  "set video duration", OFFSET(duration),  AV_OPT_TYPE_DURATION,   {.i64=-1},        -1, INT64_MAX, FLAGS },
    {"d",         "set video duration", OFFSET(duration),  AV_OPT_TYPE_DURATION,   {.i64=-1},        -1, INT64_MAX, FLAGS },
    {"speed",     "set gradients rotation speed", OFFSET(speed), AV_OPT_TYPE_FLOAT,{.dbl=0.01},       0, 1, VFT },
    {"type",      "set gradient type", OFFSET(type),       AV_OPT_TYPE_INT,        {.i64=0},          0, 4, VFT, .unit = "type" },
    {"t",         "set gradient type", OFFSET(type),       AV_OPT_TYPE_INT,        {.i64=0},          0, 4, VFT, .unit = "type" },
    { "linear",   "set linear gradient",          0,       AV_OPT_TYPE_CONST,      {.i64=0},          0, 0, VFT, .unit = "type" },
    { "radial",   "set radial gradient",          0,       AV_OPT_TYPE_CONST,      {.i64=1},          0, 0, VFT, .unit = "type" },
    { "circular", "set circular gradient",        0,       AV_OPT_TYPE_CONST,      {.i64=2},          0, 0, VFT, .unit = "type" },
    { "spiral",   "set spiral gradient",          0,       AV_OPT_TYPE_CONST,      {.i64=3},          0, 0, VFT, .unit = "type" },
    { "square",   "set square gradient",          0,       AV_OPT_TYPE_CONST,      {.i64=4},          0, 0, VFT, .unit = "type" },
    {NULL},
};

AVFILTER_DEFINE_CLASS(gradients);

static float lerpf(float a, float b, float x)
{
    const float y = 1.f - x;

    return a * y + b * x;
}

static uint32_t lerp_color(uint32_t ac0, uint32_t ac1, float x)
{
    uint8_t c0[4] = { 0xff, 0xff, 0xff, 0xff };
    uint8_t c1[4] = { 0xff, 0xff, 0xff, 0xff };
    const float y = 1.f - x;

    AV_WN32(c0, ac0);
    AV_WN32(c1, ac1);

    return (lrintf(c0[0] * y + c1[0] * x)) << 0  |
           (lrintf(c0[1] * y + c1[1] * x)) << 8  |
           (lrintf(c0[2] * y + c1[2] * x)) << 16 |
           (lrintf(c0[3] * y + c1[3] * x)) << 24;
}

static uint64_t lerp_color16(uint32_t ac0, uint32_t ac1, float x)
{
    uint8_t c0[4] = { 0xff, 0xff, 0xff, 0xff };
    uint8_t c1[4] = { 0xff, 0xff, 0xff, 0xff };
    const float y = 1.f - x;

    AV_WN32(c0, ac0);
    AV_WN32(c1, ac1);

    return ((uint64_t)llrintf((c0[0] * y + c1[0] * x) * 256)) << 0  |
           ((uint64_t)llrintf((c0[1] * y + c1[1] * x) * 256)) << 16 |
           ((uint64_t)llrintf((c0[2] * y + c1[2] * x) * 256)) << 32 |
           ((uint64_t)llrintf((c0[3] * y + c1[3] * x) * 256)) << 48;
}

static uint32_t lerp_colors(uint32_t *arr, int nb_colors, int nb_wrap_colors, float step)
{
    float scl;
    int i, j;

    if (nb_colors == 1 || step <= 0.0) {
        return arr[0];
    } else if (step >= 1.0) {
        i = nb_colors - 1;
        return arr[i];
    }

    scl = step * (nb_wrap_colors - 1);
    i = floorf(scl);
    j = i + 1;
    if (i >= nb_colors - 1) {
        i = nb_colors - 1;
        j = 0;
    }

    return lerp_color(arr[i], arr[j], scl - i);
}

static uint64_t lerp_colors16(uint32_t *arr, int nb_colors, int nb_wrap_colors, float step)
{
    float scl;
    int i, j;

    if (nb_colors == 1 || step <= 0.0) {
        uint8_t clr[4] = { 0xff, 0xff, 0xff, 0xff };

        AV_WN32(clr, arr[0]);
        return ((uint64_t)clr[0] << 8) | ((uint64_t)clr[1] << 24) | ((uint64_t)clr[2] << 40) | ((uint64_t)clr[3] << 56);
    } else if (step >= 1.0) {
        uint8_t clr[4] = { 0xff, 0xff, 0xff, 0xff };

        i = nb_colors - 1;
        AV_WN32(clr, arr[i]);
        return ((uint64_t)clr[0] << 8) | ((uint64_t)clr[1] << 24) | ((uint64_t)clr[2] << 40) | ((uint64_t)clr[3] << 56);
    }

    scl = step * (nb_wrap_colors - 1);
    i = floorf(scl);
    j = i + 1;
    if (i >= nb_colors - 1) {
        i = nb_colors - 1;
        j = 0;
    }

    return lerp_color16(arr[i], arr[j], scl - i);
}

static void lerp_colors32(FloatColor *arr, int nb_colors,
                          int nb_wrap_colors, float step,
                          float *r, float *g, float *b, float *a)
{
    float scl, x;
    int i, j;

    if (nb_colors == 1 || step <= 0.0) {
        *r = arr[0].c[0];
        *g = arr[0].c[1];
        *b = arr[0].c[2];
        *a = arr[0].c[3];
        return;
    } else if (step >= 1.0) {
        i = nb_colors - 1;
        *r = arr[i].c[0];
        *g = arr[i].c[1];
        *b = arr[i].c[2];
        *a = arr[i].c[3];
        return;
    }

    scl = step * (nb_wrap_colors - 1);
    i = floorf(scl);
    j = i + 1;
    if (i >= nb_colors - 1) {
        i = nb_colors - 1;
        j = 0;
    }
    x = scl - i;

    *r = lerpf(arr[i].c[0], arr[j].c[0], x);
    *g = lerpf(arr[i].c[1], arr[j].c[1], x);
    *b = lerpf(arr[i].c[2], arr[j].c[2], x);
    *a = lerpf(arr[i].c[3], arr[j].c[3], x);
}

static float project(float origin_x, float origin_y,
                     float dest_x, float dest_y,
                     float point_x, float point_y, int type)
{
    float op_x = point_x - origin_x;
    float op_y = point_y - origin_y;
    float od_x = dest_x - origin_x;
    float od_y = dest_y - origin_y;
    float op_x_od;
    float od_s_q;

    switch (type) {
    case 0:
        od_s_q = od_x * od_x + od_y * od_y;
        break;
    case 1:
        od_s_q = sqrtf(od_x * od_x + od_y * od_y);
        break;
    case 2:
    case 3:
        od_s_q = M_PIf * 2.f;
        break;
    case 4:
        od_s_q = fmaxf(fabsf(od_x), fabsf(od_y));
        break;
    }

    switch (type) {
    case 0:
        op_x_od = op_x * od_x + op_y * od_y;
        break;
    case 1:
        op_x_od = sqrtf(op_x * op_x + op_y * op_y);
        break;
    case 2:
        op_x_od = atan2f(op_x, op_y) + M_PIf;
        break;
    case 3:
        op_x_od = fmodf(atan2f(op_x, op_y) + M_PIf + point_x / fmaxf(origin_x, dest_x), 2.f * M_PIf);
        break;
    case 4:
        op_x_od = fmaxf(fabsf(op_x), fabsf(op_y));
        break;
    }

    // Normalize and clamp range.
    return av_clipf(op_x_od / od_s_q, 0.f, 1.f);
}

static int draw_gradients_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    GradientsContext *s = ctx->priv;
    AVFrame *frame = arg;
    const int width  = frame->width;
    const int height = frame->height;
    const int start = (height *  job   ) / nb_jobs;
    const int end   = (height * (job+1)) / nb_jobs;
    const ptrdiff_t linesize = frame->linesize[0] / 4;
    uint32_t *dst = (uint32_t *)frame->data[0] + start * linesize;
    const int type = s->type;

    for (int y = start; y < end; y++) {
        for (int x = 0; x < width; x++) {
            float factor = project(s->fx0, s->fy0, s->fx1, s->fy1, x, y, type);
            dst[x] = lerp_colors(s->color_rgba, s->nb_colors, s->nb_colors + (type >= 2 && type <= 3), factor);
        }

        dst += linesize;
    }

    return 0;
}

static int draw_gradients_slice16(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    GradientsContext *s = ctx->priv;
    AVFrame *frame = arg;
    const int width  = frame->width;
    const int height = frame->height;
    const int start = (height *  job   ) / nb_jobs;
    const int end   = (height * (job+1)) / nb_jobs;
    const ptrdiff_t linesize = frame->linesize[0] / 8;
    uint64_t *dst = (uint64_t *)frame->data[0] + start * linesize;
    const int type = s->type;

    for (int y = start; y < end; y++) {
        for (int x = 0; x < width; x++) {
            float factor = project(s->fx0, s->fy0, s->fx1, s->fy1, x, y, type);
            dst[x] = lerp_colors16(s->color_rgba, s->nb_colors, s->nb_colors + (type >= 2 && type <= 3), factor);
        }

        dst += linesize;
    }

    return 0;
}

static int draw_gradients_slice32_planar(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    GradientsContext *s = ctx->priv;
    AVFrame *frame = arg;
    const int width  = frame->width;
    const int height = frame->height;
    const int start = (height *  job   ) / nb_jobs;
    const int end   = (height * (job+1)) / nb_jobs;
    const ptrdiff_t linesize_g = frame->linesize[0] / 4;
    const ptrdiff_t linesize_b = frame->linesize[1] / 4;
    const ptrdiff_t linesize_r = frame->linesize[2] / 4;
    const ptrdiff_t linesize_a = frame->linesize[3] / 4;
    float *dst_g = (float *)frame->data[0] + start * linesize_g;
    float *dst_b = (float *)frame->data[1] + start * linesize_b;
    float *dst_r = (float *)frame->data[2] + start * linesize_r;
    float *dst_a = (float *)frame->data[3] + start * linesize_a;
    const int type = s->type;

    for (int y = start; y < end; y++) {
        for (int x = 0; x < width; x++) {
            float factor = project(s->fx0, s->fy0, s->fx1, s->fy1, x, y, type);
            lerp_colors32(s->color_rgbaf, s->nb_colors, s->nb_colors + (type >= 2 && type <= 3), factor,
                          &dst_r[x], &dst_g[x], &dst_b[x], &dst_a[x]);
        }

        dst_g += linesize_g;
        dst_b += linesize_b;
        dst_r += linesize_r;
        dst_a += linesize_a;
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    FilterLink *l = ff_filter_link(outlink);
    GradientsContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);

    if (av_image_check_size(s->w, s->h, 0, ctx) < 0)
        return AVERROR(EINVAL);

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->time_base = av_inv_q(s->frame_rate);
    outlink->sample_aspect_ratio = (AVRational) {1, 1};
    l->frame_rate = s->frame_rate;
    if (s->seed == -1)
        s->seed = av_get_random_seed();
    av_lfg_init(&s->lfg, s->seed);

    switch (desc->comp[0].depth) {
    case 8:
        s->draw_slice = draw_gradients_slice;
        break;
    case 16:
        s->draw_slice = draw_gradients_slice16;
        break;
    case 32:
        s->draw_slice = draw_gradients_slice32_planar;
        break;
    default:
        return AVERROR_BUG;
    }

    if (s->x0 < 0 || s->x0 >= s->w)
        s->x0 = av_lfg_get(&s->lfg) % s->w;
    if (s->y0 < 0 || s->y0 >= s->h)
        s->y0 = av_lfg_get(&s->lfg) % s->h;
    if (s->x1 < 0 || s->x1 >= s->w)
        s->x1 = av_lfg_get(&s->lfg) % s->w;
    if (s->y1 < 0 || s->y1 >= s->h)
        s->y1 = av_lfg_get(&s->lfg) % s->h;

    s->color_rgbaf = av_calloc(s->nb_colors, sizeof(*s->color_rgbaf));
    if (!s->color_rgbaf)
        return AVERROR(ENOMEM);

    for (int n = 0; n < s->nb_colors; n++) {
        uint8_t color[4] = { 0xff, 0xff, 0xff, 0xff };

        AV_WN32(color, s->color_rgba[n]);
        for (int c = 0; c < 4; c++) {
            s->color_rgbaf[n].c[c] = color[c] / 255.f;
        }
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    GradientsContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    if (s->duration >= 0 &&
        av_rescale_q(s->pts, outlink->time_base, AV_TIME_BASE_Q) >= s->duration) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink)) {
        AVFrame *frame = ff_get_video_buffer(outlink, s->w, s->h);
        float angle = fmodf(s->angle, 2.f * M_PI);
        const float w2 = s->w / 2.f;
        const float h2 = s->h / 2.f;

        s->angle = angle + s->speed;

        s->fx0 = (s->x0 - w2) * cosf(angle) - (s->y0 - h2) * sinf(angle) + w2;
        s->fy0 = (s->x0 - w2) * sinf(angle) + (s->y0 - h2) * cosf(angle) + h2;

        s->fx1 = (s->x1 - w2) * cosf(angle) - (s->y1 - h2) * sinf(angle) + w2;
        s->fy1 = (s->x1 - w2) * sinf(angle) + (s->y1 - h2) * cosf(angle) + h2;

        if (!frame)
            return AVERROR(ENOMEM);

        frame->flags              |= AV_FRAME_FLAG_KEY;
        frame->flags              &= ~AV_FRAME_FLAG_INTERLACED;
        frame->pict_type           = AV_PICTURE_TYPE_I;
        frame->sample_aspect_ratio = (AVRational) {1, 1};
        frame->pts = s->pts++;
        frame->duration = 1;

        ff_filter_execute(ctx, s->draw_slice, frame, NULL,
                          FFMIN(outlink->h, ff_filter_get_nb_threads(ctx)));

        return ff_filter_frame(outlink, frame);
    }

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    GradientsContext *s = ctx->priv;

    av_freep(&s->color_rgbaf);
}

static const AVFilterPad gradients_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_vsrc_gradients = {
    .p.name        = "gradients",
    .p.description = NULL_IF_CONFIG_SMALL("Draw a gradients."),
    .p.priv_class  = &gradients_class,
    .p.inputs      = NULL,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(GradientsContext),
    FILTER_OUTPUTS(gradients_outputs),
    FILTER_PIXFMTS(AV_PIX_FMT_RGBA, AV_PIX_FMT_RGBA64, AV_PIX_FMT_GBRAPF32),
    .activate      = activate,
    .uninit        = uninit,
    .process_command = ff_filter_process_command,
};
