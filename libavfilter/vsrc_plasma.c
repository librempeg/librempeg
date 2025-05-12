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

typedef struct PlasmaContext {
    const AVClass *class;
    int w, h;
    AVRational frame_rate;
    int64_t pts;
    int64_t duration;

    float point_radius;
    float link_size;

    int64_t seed;

    AVLFG lfg;
    int (*draw_slice)(AVFilterContext *ctx, void *arg, int job, int nb_jobs);
} PlasmaContext;

#define OFFSET(x) offsetof(PlasmaContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define VFT AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption plasma_options[] = {
    {"size",      "set frame size", OFFSET(w),             AV_OPT_TYPE_IMAGE_SIZE, {.str="640x480"},  0, 0, FLAGS },
    {"s",         "set frame size", OFFSET(w),             AV_OPT_TYPE_IMAGE_SIZE, {.str="640x480"},  0, 0, FLAGS },
    {"rate",      "set frame rate", OFFSET(frame_rate),    AV_OPT_TYPE_VIDEO_RATE, {.str="25"},       0, INT_MAX, FLAGS },
    {"r",         "set frame rate", OFFSET(frame_rate),    AV_OPT_TYPE_VIDEO_RATE, {.str="25"},       0, INT_MAX, FLAGS },
    {"seed",      "set the seed",   OFFSET(seed),          AV_OPT_TYPE_INT64,      {.i64=-1},        -1, UINT32_MAX, FLAGS },
    {"duration",  "set video duration", OFFSET(duration),  AV_OPT_TYPE_DURATION,   {.i64=-1},        -1, INT64_MAX, FLAGS },
    {"d",         "set video duration", OFFSET(duration),  AV_OPT_TYPE_DURATION,   {.i64=-1},        -1, INT64_MAX, FLAGS },
    {"pointr",    "set the point radius", OFFSET(point_radius), AV_OPT_TYPE_FLOAT, {.dbl=0.05},       0, 1, VFT },
    {"links",     "set the link size", OFFSET(link_size),  AV_OPT_TYPE_FLOAT,      {.dbl=0.05},       0, 1, VFT },
    {NULL},
};

AVFILTER_DEFINE_CLASS(plasma);

static float lerpf(float a, float b, float x)
{
    const float y = 1.f - x;

    return a * y + b * x;
}

static int draw_plasma_slice32_planar(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    PlasmaContext *s = ctx->priv;
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
    const float min_dim = FFMIN(width, height);
    const float bounds[2] = { width / min_dim, height / min_dim };
    const float point_radius = s->point_radius;
    float speed_r[3], speed_g[3], speed_b[3];
    const float itime = frame->pts * 0.005f;
    const float link_size = s->link_size;

    speed_r[0] = 0.32f * sinf(1.32f * itime);
    speed_r[1] = 0.30f * sinf(1.03f * itime);
    speed_r[2] = 0.40f * sinf(1.32f * itime);

    speed_g[0] = 0.31f * sinf(0.92f * itime);
    speed_g[1] = 0.29f * sinf(0.99f * itime);
    speed_g[2] = 0.38f * sinf(1.24f * itime);

    speed_b[0] = 0.33f * sinf(1.245f * itime);
    speed_b[1] = 0.30f * sinf(1.41f * itime);
    speed_b[2] = 0.41f * sinf(1.11f * itime);

    for (int y = start; y < end; y++) {
        for (int x = 0; x < width; x++) {
            float linkStrengthRG, linkStrengthGB, linkStrengthBR;
            const float uv[2] = { x / min_dim, y / min_dim };
            float point_r[3], point_g[3], point_b[3];
            float vecToR[2], vecToG[2], vecToB[2];
            float dirToR[2], dirToG[2], dirToB[2];
            float sumDistRG, sumDistGB, sumDistBR;
            float distToR, distToG, distToB;
            float contribRonRG, contribRonBR;
            float contribGonRG, contribGonGB;
            float contribBonGB, contribBonBR;
            float dotRG, dotGB, dotBR;
            float midUV[2], norm;

            point_r[0] = point_r[1] = 0.f;
            point_r[2] = 1.f;

            point_g[0] = point_g[1] = 0.f;
            point_g[2] = 1.f;

            point_b[0] = point_b[1] = 0.f;
            point_b[2] = 1.f;

            point_r[0] += speed_r[0];
            point_r[1] += speed_r[1];
            point_r[2] += speed_r[2];

            point_g[0] += speed_g[0];
            point_g[1] += speed_g[1];
            point_g[2] += speed_g[2];

            point_b[0] += speed_b[0];
            point_b[1] += speed_b[1];
            point_b[2] += speed_b[2];

            midUV[0] = bounds[0] * 0.5f;
            midUV[1] = bounds[1] * 0.5f;

            point_r[0] += midUV[0];
            point_r[1] += midUV[1];
            point_g[0] += midUV[0];
            point_g[1] += midUV[1];
            point_b[0] += midUV[0];
            point_b[1] += midUV[1];

            vecToR[0] = point_r[0] - uv[0];
            vecToR[1] = point_r[1] - uv[1];

            vecToG[0] = point_g[0] - uv[0];
            vecToG[1] = point_g[1] - uv[1];

            vecToB[0] = point_b[0] - uv[0];
            vecToB[1] = point_b[1] - uv[1];

            norm = sqrtf(vecToR[0] * vecToR[0] + vecToR[1] * vecToR[1]);
            distToR = norm;
            dirToR[0] = vecToR[0] / norm;
            dirToR[1] = vecToR[1] / norm;

            norm = sqrtf(vecToG[0] * vecToG[0] + vecToG[1] * vecToG[1]);
            distToG = norm;
            dirToG[0] = vecToG[0] / norm;
            dirToG[1] = vecToG[1] / norm;

            norm = sqrtf(vecToB[0] * vecToB[0] + vecToB[1] * vecToB[1]);
            distToB = norm;
            dirToB[0] = vecToB[0] / norm;
            dirToB[1] = vecToB[1] / norm;

            dotRG = dirToR[0] * dirToG[0] + dirToR[1] * dirToG[1];
            dotGB = dirToG[0] * dirToB[0] + dirToG[1] * dirToB[1];
            dotBR = dirToB[0] * dirToR[0] + dirToB[1] * dirToR[1];

            dst_r[x] = 1.f - lerpf(distToR, 0.f, point_radius * point_r[2]);
            dst_g[x] = 1.f - lerpf(distToG, 0.f, point_radius * point_g[2]);
            dst_b[x] = 1.f - lerpf(distToB, 0.f, point_radius * point_b[2]);
            dst_a[x] = 1.f;

            linkStrengthRG = 1.f - lerpf(dotRG, -1.01f, -1.f + (link_size * point_r[2] * point_g[2]));
            linkStrengthGB = 1.f - lerpf(dotGB, -1.01f, -1.f + (link_size * point_g[2] * point_b[2]));
            linkStrengthBR = 1.f - lerpf(dotBR, -1.01f, -1.f + (link_size * point_b[2] * point_r[2]));

            sumDistRG = distToR + distToG;
            sumDistGB = distToG + distToB;
            sumDistBR = distToB + distToR;

            contribRonRG = 1.f - (distToR / sumDistRG);
            contribRonBR = 1.f - (distToR / sumDistBR);

            contribGonRG = 1.f - (distToG / sumDistRG);
            contribGonGB = 1.f - (distToG / sumDistGB);

            contribBonGB = 1.f - (distToB / sumDistGB);
            contribBonBR = 1.f - (distToB / sumDistBR);

            dst_r[x] += (linkStrengthRG * contribRonRG) + (linkStrengthBR * contribRonBR);
            dst_g[x] += (linkStrengthGB * contribGonGB) + (linkStrengthRG * contribGonRG);
            dst_b[x] += (linkStrengthBR * contribBonBR) + (linkStrengthGB * contribBonGB);
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
    PlasmaContext *s = ctx->priv;

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

    s->draw_slice = draw_plasma_slice32_planar;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    PlasmaContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    if (s->duration >= 0 &&
        av_rescale_q(s->pts, outlink->time_base, AV_TIME_BASE_Q) >= s->duration) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink)) {
        AVFrame *frame = ff_get_video_buffer(outlink, s->w, s->h);

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

static const AVFilterPad plasma_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_vsrc_plasma = {
    .p.name        = "plasma",
    .p.description = NULL_IF_CONFIG_SMALL("Draw a plasma."),
    .p.priv_class  = &plasma_class,
    .p.inputs      = NULL,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(PlasmaContext),
    FILTER_OUTPUTS(plasma_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_GBRAPF32),
    .activate      = activate,
    .process_command = ff_filter_process_command,
};
