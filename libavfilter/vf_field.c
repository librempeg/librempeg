/*
 * Copyright (c) 2003 Rich Felker
 * Copyright (c) 2012 Stefano Sabatini
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
 * field filter, based on libmpcodecs/vf_field.c by Rich Felker
 */

#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"

enum FieldType { FIELD_TYPE_TOP = 0, FIELD_TYPE_BOTTOM };

typedef struct FieldContext {
    const AVClass *class;
    int type;                   ///< FieldType
    int nb_planes;              ///< number of planes of the current format
} FieldContext;

#define OFFSET(x) offsetof(FieldContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption field_options[] = {
    {"type", "set field type (top or bottom)", OFFSET(type), AV_OPT_TYPE_INT, {.i64=FIELD_TYPE_TOP}, 0, 1, FLAGS, .unit = "field_type" },
    {"top",    "select top field",    0, AV_OPT_TYPE_CONST, {.i64=FIELD_TYPE_TOP},    0, 0, FLAGS, .unit = "field_type"},
    {"bottom", "select bottom field", 0, AV_OPT_TYPE_CONST, {.i64=FIELD_TYPE_BOTTOM}, 0, 0, FLAGS, .unit = "field_type"},
    {NULL}
};

AVFILTER_DEFINE_CLASS(field);

static int config_props_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    FieldContext *s = ctx->priv;

    s->nb_planes = av_pix_fmt_count_planes(outlink->format);

    outlink->w = inlink->w;
    outlink->h = (inlink->h + (s->type == FIELD_TYPE_TOP)) / 2;

    av_log(ctx, AV_LOG_VERBOSE, "w:%d h:%d type:%s -> w:%d h:%d\n",
           inlink->w, inlink->h, s->type == FIELD_TYPE_BOTTOM ? "bottom" : "top",
           outlink->w, outlink->h);
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterLink *outlink = inlink->dst->outputs[0];
    FieldContext *s = inlink->dst->priv;

    in->height = outlink->h;
    in->flags &= ~AV_FRAME_FLAG_INTERLACED;

    for (int i = 0; i < s->nb_planes; i++) {
        if (s->type == FIELD_TYPE_BOTTOM)
            in->data[i] = in->data[i] + in->linesize[i];
        in->linesize[i] = 2 * in->linesize[i];
    }

    return ff_filter_frame(outlink, in);
}

static const AVFilterPad field_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad field_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_props_output,
    },
};

const FFFilter ff_vf_field = {
    .p.name        = "field",
    .p.description = NULL_IF_CONFIG_SMALL("Extract a field from the input video."),
    .p.priv_class  = &field_class,
    .priv_size     = sizeof(FieldContext),
    FILTER_INPUTS(field_inputs),
    FILTER_OUTPUTS(field_outputs),
};
