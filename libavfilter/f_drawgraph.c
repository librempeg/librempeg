/*
 * Copyright (c) 2015 Paul B Mahol
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

#include "config_components.h"

#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"
#include "video.h"

typedef struct DrawGraphContext {
    const AVClass *class;

    char          **keys;
    unsigned      nb_keys;
    char          **fg_str;
    unsigned      nb_fg_str;
    float         min, max;
    AVExpr        **fg_expr;
    uint8_t       bg[4];
    int           mode;
    int           slide;
    int           w, h;
    AVRational    frame_rate;

    int           nb_samples;

    AVFrame       *out;
    int           x;
    int           *prev_y;
    int           *first;
    float         **values;
    int           *values_size;
    int           nb_values;
    int64_t       prev_pts;
} DrawGraphContext;

#define OFFSET(x) offsetof(DrawGraphContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_keys = {.def="",.size_min=0,.sep='|'};
static const AVOptionArrayDef def_fgs = {.def="0xffff0000|0xff00ff00|0xffff00ff|0xffffff00",.size_min=1,.sep='|'};

static const AVOption drawgraph_options[] = {
    { "keys", "set the metadata key", OFFSET(keys), AV_OPT_TYPE_STRING|AR, {.arr=&def_keys}, 0, 0, FLAGS },
    { "fg", "set the foreground colors", OFFSET(fg_str), AV_OPT_TYPE_STRING|AR, {.arr=&def_fgs}, 0, 0, FLAGS },
    { "bg", "set background color", OFFSET(bg), AV_OPT_TYPE_COLOR, {.str="white"}, 0, 0, FLAGS },
    { "min", "set minimal value", OFFSET(min), AV_OPT_TYPE_FLOAT, {.dbl=-1.}, INT_MIN, INT_MAX, FLAGS },
    { "max", "set maximal value", OFFSET(max), AV_OPT_TYPE_FLOAT, {.dbl=1.}, INT_MIN, INT_MAX, FLAGS },
    { "mode", "set graph mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=2}, 0, 2, FLAGS, .unit = "mode" },
        {"bar", "draw bars", OFFSET(mode), AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, FLAGS, .unit = "mode"},
        {"dot", "draw dots", OFFSET(mode), AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, FLAGS, .unit = "mode"},
        {"line", "draw lines", OFFSET(mode), AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, FLAGS, .unit = "mode"},
    { "slide", "set slide mode", OFFSET(slide), AV_OPT_TYPE_INT, {.i64=0}, 0, 4, FLAGS, .unit = "slide" },
        {"frame", "draw new frames", OFFSET(slide), AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, FLAGS, .unit = "slide"},
        {"replace", "replace old columns with new", OFFSET(slide), AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, FLAGS, .unit = "slide"},
        {"scroll", "scroll from right to left", OFFSET(slide), AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, FLAGS, .unit = "slide"},
        {"rscroll", "scroll from left to right", OFFSET(slide), AV_OPT_TYPE_CONST, {.i64=3}, 0, 0, FLAGS, .unit = "slide"},
        {"picture", "display graph in single frame", OFFSET(slide), AV_OPT_TYPE_CONST, {.i64=4}, 0, 0, FLAGS, .unit = "slide"},
    { "size", "set graph size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="900x256"}, 0, 0, FLAGS },
    { "s", "set graph size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="900x256"}, 0, 0, FLAGS },
    { "rate", "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(drawgraph, "(a)drawgraph", drawgraph_options);

static const char *const var_names[] = {   "MAX",   "MIN",   "VAL", NULL };
enum                                   { VAR_MAX, VAR_MIN, VAR_VAL, VAR_VARS_NB };

static av_cold int init(AVFilterContext *ctx)
{
    DrawGraphContext *s = ctx->priv;
    int ret;

    if (s->max <= s->min) {
        av_log(ctx, AV_LOG_ERROR, "max is same or lower than min\n");
        return AVERROR(EINVAL);
    }

    s->fg_expr = av_calloc(s->nb_fg_str, sizeof(*s->fg_expr));
    if (!s->fg_expr)
        return AVERROR(ENOMEM);

    s->values = av_calloc(s->nb_keys, sizeof(*s->values));
    if (!s->values)
        return AVERROR(ENOMEM);

    s->values_size = av_calloc(s->nb_keys, sizeof(*s->values_size));
    if (!s->values_size)
        return AVERROR(ENOMEM);

    s->prev_y = av_calloc(s->nb_keys, sizeof(*s->prev_y));
    if (!s->prev_y)
        return AVERROR(ENOMEM);

    s->first = av_calloc(s->nb_keys, sizeof(*s->first));
    if (!s->first)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->nb_keys; i++)
        s->first[i] = 1;

    for (int i = 0; i < s->nb_fg_str; i++) {
        if (s->fg_str[i]) {
            ret = av_expr_parse(&s->fg_expr[i], s->fg_str[i], var_names,
                                NULL, NULL, NULL, NULL, 0, ctx);

            if (ret < 0)
                return ret;
        }
    }

    if (s->slide == 4) {
        for (int i = 0; i < s->nb_keys; i++) {
            s->values[i] = av_fast_realloc(NULL, &s->values_size[i], 2000);
            if (!s->values[i])
                return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_RGBA,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *formats = ff_make_format_list(pix_fmts);

    return ff_formats_ref(formats, &cfg_out[0]->formats);
}

static void clear_image(DrawGraphContext *s, AVFrame *out, AVFilterLink *outlink)
{
    int i, j;
    int bg = AV_RN32(s->bg);

    for (i = 0; i < out->height; i++)
        for (j = 0; j < out->width; j++)
            AV_WN32(out->data[0] + i * out->linesize[0] + j * 4, bg);
}

static inline void draw_dot(int fg, int x, int y, AVFrame *out)
{
    AV_WN32(out->data[0] + y * out->linesize[0] + x * 4, fg);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    DrawGraphContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVDictionary *metadata;
    AVDictionaryEntry *e;
    AVFrame *out = s->out;
    AVFrame *clone = NULL;
    int64_t in_pts, out_pts, in_duration;
    int i;

    if (s->slide == 4 && s->nb_values >= s->values_size[0] / sizeof(float)) {
        for (int i = 0; i < s->nb_keys; i++) {
            float *ptr = av_fast_realloc(s->values[i], &s->values_size[i], s->values_size[i] * 2);
            if (!ptr)
                return AVERROR(ENOMEM);
            s->values[i] = ptr;
        }
    }

    if (s->slide != 4 || s->nb_values == 0) {
        if (!s->out || s->out->width  != outlink->w ||
                       s->out->height != outlink->h) {
            av_frame_free(&s->out);
            s->out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
            out = s->out;
            if (!s->out) {
                av_frame_free(&in);
                return AVERROR(ENOMEM);
            }

            clear_image(s, out, outlink);
        }
        av_frame_copy_props(out, in);
    }

    metadata = in->metadata;

    for (i = 0; i < s->nb_keys; i++) {
        double values[VAR_VARS_NB];
        int j, y, x, old;
        uint32_t fg, bg;
        float vf;

        if (s->slide == 4)
            s->values[i][s->nb_values] = NAN;

        e = av_dict_get(metadata, s->keys[i], NULL, 0);
        if (!e || !e->value)
            continue;

        if (av_sscanf(e->value, "%f", &vf) != 1)
            continue;

        vf = av_clipf(vf, s->min, s->max);

        if (s->slide == 4) {
            s->values[i][s->nb_values] = vf;
            continue;
        }

        values[VAR_MIN] = s->min;
        values[VAR_MAX] = s->max;
        values[VAR_VAL] = vf;

        fg = av_expr_eval(s->fg_expr[FFMIN(i, s->nb_fg_str-1)], values, NULL);
        bg = AV_RN32(s->bg);

        if (i == 0 && (s->x >= outlink->w || s->slide == 3)) {
            if (s->slide == 0 || s->slide == 1)
                s->x = 0;

            if (s->slide == 2) {
                s->x = outlink->w - 1;
                for (j = 0; j < outlink->h; j++) {
                    memmove(out->data[0] + j * out->linesize[0] ,
                            out->data[0] + j * out->linesize[0] + 4,
                            (outlink->w - 1) * 4);
                }
            } else if (s->slide == 3) {
                s->x = 0;
                for (j = 0; j < outlink->h; j++) {
                    memmove(out->data[0] + j * out->linesize[0] + 4,
                            out->data[0] + j * out->linesize[0],
                            (outlink->w - 1) * 4);
                }
            } else if (s->slide == 0) {
                clear_image(s, out, outlink);
            }
        }

        x = s->x;
        y = (outlink->h - 1) * (1 - ((vf - s->min) / (s->max - s->min)));

        switch (s->mode) {
        case 0:
            if (i == 0 && (s->slide > 0))
                for (j = 0; j < outlink->h; j++)
                    draw_dot(bg, x, j, out);

            old = AV_RN32(out->data[0] + y * out->linesize[0] + x * 4);
            for (j = y; j < outlink->h; j++) {
                if (old != bg &&
                    (AV_RN32(out->data[0] + j * out->linesize[0] + x * 4) != old) ||
                    AV_RN32(out->data[0] + FFMIN(j+1, outlink->h - 1) * out->linesize[0] + x * 4) != old) {
                    draw_dot(fg, x, j, out);
                    break;
                }
                draw_dot(fg, x, j, out);
            }
            break;
        case 1:
            if (i == 0 && (s->slide > 0))
                for (j = 0; j < outlink->h; j++)
                    draw_dot(bg, x, j, out);
            draw_dot(fg, x, y, out);
            break;
        case 2:
            if (s->first[i]) {
                s->first[i] = 0;
                s->prev_y[i] = y;
            }

            if (i == 0 && (s->slide > 0)) {
                for (j = 0; j < y; j++)
                    draw_dot(bg, x, j, out);
                for (j = outlink->h - 1; j > y; j--)
                    draw_dot(bg, x, j, out);
            }
            if (y <= s->prev_y[i]) {
                for (j = y; j <= s->prev_y[i]; j++)
                    draw_dot(fg, x, j, out);
            } else {
                for (j = s->prev_y[i]; j <= y; j++)
                    draw_dot(fg, x, j, out);
            }
            s->prev_y[i] = y;
            break;
        }
    }

    s->nb_values++;
    s->x++;

    in_pts = in->pts;
    in_duration = in->duration;

    av_frame_free(&in);

    if (s->slide == 4)
        return 0;

    out_pts = av_rescale_q(in_pts, inlink->time_base, outlink->time_base);

    if (out_pts == s->prev_pts)
        return 0;

    clone = av_frame_clone(s->out);
    if (!clone)
        return AVERROR(ENOMEM);

    clone->pts = s->prev_pts = out_pts;
    clone->duration = av_rescale_q(in_duration, inlink->time_base, outlink->time_base);
    return ff_filter_frame(outlink, clone);
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    DrawGraphContext *s = ctx->priv;
    AVFrame *out = s->out;

    if (s->slide == 4 && s->nb_values > 0) {
        int step = ceil(s->nb_values / (float)s->w), l = 0;

        s->x = l;

        for (int k = 0; k < s->nb_values; k++) {
            for (int i = 0; i < s->nb_keys; i++) {
                double values[VAR_VARS_NB];
                int j, y, x, old;
                uint32_t fg, bg;
                float vf = s->values[i][k];

                if (isnan(vf))
                    continue;

                values[VAR_MIN] = s->min;
                values[VAR_MAX] = s->max;
                values[VAR_VAL] = vf;

                fg = av_expr_eval(s->fg_expr[FFMIN(i, s->nb_fg_str-1)], values, NULL);
                bg = AV_RN32(s->bg);

                x = s->x;
                y = (outlink->h - 1) * (1 - ((vf - s->min) / (s->max - s->min)));

                switch (s->mode) {
                case 0:
                    old = AV_RN32(out->data[0] + y * out->linesize[0] + x * 4);
                    for (j = y; j < outlink->h; j++) {
                        if (old != bg &&
                            (AV_RN32(out->data[0] + j * out->linesize[0] + x * 4) != old) ||
                            AV_RN32(out->data[0] + FFMIN(j+1, outlink->h - 1) * out->linesize[0] + x * 4) != old) {
                            draw_dot(fg, x, j, out);
                            break;
                        }
                        draw_dot(fg, x, j, out);
                    }
                    break;
                case 1:
                    draw_dot(fg, x, y, out);
                    break;
                case 2:
                    if (s->first[i]) {
                        s->first[i] = 0;
                        s->prev_y[i] = y;
                    }

                    if (y <= s->prev_y[i]) {
                        for (j = y; j <= s->prev_y[i]; j++)
                            draw_dot(fg, x, j, out);
                    } else {
                        for (j = s->prev_y[i]; j <= y; j++)
                            draw_dot(fg, x, j, out);
                    }
                    s->prev_y[i] = y;
                    break;
                }
            }

            l++;
            if (l >= step) {
                l = 0;
                s->x++;
            }
        }

        s->nb_values = 0;
        out->pts = 0;
        return ff_filter_frame(ctx->outputs[0], s->out);
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    FilterLink *l = ff_filter_link(outlink);
    DrawGraphContext *s = outlink->src->priv;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};
    l->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(l->frame_rate);
    s->prev_pts = AV_NOPTS_VALUE;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    DrawGraphContext *s = ctx->priv;
    int ret, status;
    int64_t pts;
    AVFrame *in;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (inlink->sample_rate > 0) {
        s->nb_samples = FFMAX(1, av_rescale(inlink->sample_rate, s->frame_rate.den, s->frame_rate.num));

        ret = ff_inlink_consume_samples(inlink, s->nb_samples, s->nb_samples, &in);
    } else {
        ret = ff_inlink_consume_frame(inlink, &in);
    }
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    if (s->nb_samples > 0 && ff_inlink_queued_samples(inlink) >= s->nb_samples) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ret = flush_frame(outlink);
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    DrawGraphContext *s = ctx->priv;

    if (s->fg_expr) {
        for (int i = 0; i < s->nb_fg_str; i++)
            av_expr_free(s->fg_expr[i]);
        av_freep(&s->fg_expr);
    }

    if (s->slide != 4)
        av_frame_free(&s->out);

    if (s->values) {
        for (int i = 0; i < s->nb_keys; i++)
            av_freep(&s->values[i]);
        av_freep(&s->values);
    }

    av_freep(&s->values_size);
    av_freep(&s->prev_y);
    av_freep(&s->first);
}

static const AVFilterPad drawgraph_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

#if CONFIG_DRAWGRAPH_FILTER

const FFFilter ff_vf_drawgraph = {
    .p.name        = "drawgraph",
    .p.description = NULL_IF_CONFIG_SMALL("Draw a graph using input video metadata."),
    .p.priv_class  = &drawgraph_class,
    .priv_size     = sizeof(DrawGraphContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_video_default_filterpad),
    FILTER_OUTPUTS(drawgraph_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};

#endif // CONFIG_DRAWGRAPH_FILTER

#if CONFIG_ADRAWGRAPH_FILTER

const FFFilter ff_avf_adrawgraph = {
    .p.name        = "adrawgraph",
    .p.description = NULL_IF_CONFIG_SMALL("Draw a graph using input audio metadata."),
    .p.priv_class  = &drawgraph_class,
    .priv_size     = sizeof(DrawGraphContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(drawgraph_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif // CONFIG_ADRAWGRAPH_FILTER
