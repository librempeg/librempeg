/*
 * Copyright (c) 2015 Paul B Mahol
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

#include "libavutil/common.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct ShuffleFramesContext {
    const AVClass *class;
    AVFrame **frames;
    int *map;
    unsigned nb_map;
    int64_t *pts;
    int in_frames;
    int nb_frames;
} ShuffleFramesContext;

static av_cold int init(AVFilterContext *ctx)
{
    ShuffleFramesContext *s = ctx->priv;
    int nb_items = s->nb_map;

    s->frames = av_calloc(nb_items, sizeof(*s->frames));
    s->pts    = av_calloc(nb_items, sizeof(*s->pts));
    if (!s->frames || !s->pts) {
        return AVERROR(ENOMEM);
    }

    for (int n = 0; n < nb_items; n++) {
        if (s->map[n] < -1 || s->map[n] >= nb_items) {
            av_log(ctx, AV_LOG_ERROR, "Index %d out of range: [-1, %d].\n", s->map[n], nb_items - 1);
            return AVERROR(EINVAL);
        }
    }

    s->nb_frames = nb_items;
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext    *ctx = inlink->dst;
    ShuffleFramesContext *s = ctx->priv;
    int ret = 0;

    if (s->in_frames < s->nb_frames) {
        s->frames[s->in_frames] = frame;
        s->pts[s->in_frames] = frame->pts;
        s->in_frames++;
    }

    if (s->in_frames == s->nb_frames) {
        int n, x;

        for (n = 0; n < s->nb_frames; n++) {
            AVFrame *out;

            x = s->map[n];
            if (x >= 0) {
                out = av_frame_clone(s->frames[x]);
                if (!out)
                    return AVERROR(ENOMEM);
                out->pts = s->pts[n];
                ret = ff_filter_frame(ctx->outputs[0], out);
            }
            s->in_frames--;
        }

        for (n = 0; n < s->nb_frames; n++)
            av_frame_free(&s->frames[n]);
    }

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ShuffleFramesContext *s = ctx->priv;

    while (s->in_frames > 0) {
        s->in_frames--;
        av_frame_free(&s->frames[s->in_frames]);
    }

    av_freep(&s->frames);
    av_freep(&s->pts);
}

#define OFFSET(x) offsetof(ShuffleFramesContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_map = {.def="0",.size_min=1,.sep='|'};

static const AVOption shuffleframes_options[] = {
    { "mapping", "set destination indexes of input frames",  OFFSET(map), AV_OPT_TYPE_INT|AR, {.arr=&def_map}, -1, INT16_MAX, FLAGS },
    { NULL },
};

AVFILTER_DEFINE_CLASS(shuffleframes);

static const AVFilterPad shuffleframes_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_shuffleframes = {
    .p.name        = "shuffleframes",
    .p.description = NULL_IF_CONFIG_SMALL("Shuffle video frames."),
    .p.priv_class  = &shuffleframes_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .priv_size     = sizeof(ShuffleFramesContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(shuffleframes_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};
