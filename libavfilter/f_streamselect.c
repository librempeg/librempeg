/*
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

#include "libavutil/avstring.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"
#include "framesync.h"
#include "video.h"

typedef struct StreamSelectContext {
    const AVClass *class;
    int nb_inputs;
    char *map_str;
    int *map;
    int nb_map;
    int is_audio;
    int64_t *last_pts;
    AVFrame **frames;
    FFFrameSync fs;
} StreamSelectContext;

#define OFFSET(x) offsetof(StreamSelectContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
static const AVOption streamselect_options[] = {
    { "inputs",  "number of input streams",           OFFSET(nb_inputs),  AV_OPT_TYPE_INT,    {.i64=2},    2, INT_MAX,  .flags=FLAGS },
    { "map",     "input indexes to remap to outputs", OFFSET(map_str),    AV_OPT_TYPE_STRING, {.str=NULL},              .flags=TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(streamselect, "(a)streamselect", streamselect_options);

static int process_frame(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    StreamSelectContext *s = fs->opaque;
    AVFrame **in = s->frames;
    int i, j, ret = 0, have_out = 0;

    for (i = 0; i < ctx->nb_inputs; i++) {
        if ((ret = ff_framesync_get_frame(&s->fs, i, &in[i], 0)) < 0)
            return ret;
    }

    for (j = 0; j < ctx->nb_inputs; j++) {
        for (i = 0; i < s->nb_map; i++) {
            FilterLink *outl = ff_filter_link(ctx->outputs[i]);
            if (s->map[i] == j) {
                AVFrame *out;

                if (s->is_audio && s->last_pts[j] == in[j]->pts &&
                    outl->frame_count_in > 0)
                    continue;
                out = av_frame_clone(in[j]);
                if (!out)
                    return AVERROR(ENOMEM);

                out->pts = av_rescale_q(s->fs.pts, s->fs.time_base, ctx->outputs[i]->time_base);
                s->last_pts[j] = in[j]->pts;
                ret = ff_filter_frame(ctx->outputs[i], out);
                have_out = 1;
                if (ret < 0)
                    return ret;
            }
        }
    }

    if (!have_out)
        ff_filter_set_ready(ctx, 100);
    return ret;
}

static int activate(AVFilterContext *ctx)
{
    StreamSelectContext *s = ctx->priv;
    return ff_framesync_activate(&s->fs);
}

static int config_output(AVFilterLink *outlink)
{
    FilterLink *outl     = ff_filter_link(outlink);
    AVFilterContext *ctx = outlink->src;
    StreamSelectContext *s = ctx->priv;
    const int outlink_idx = FF_OUTLINK_IDX(outlink);
    const int inlink_idx  = s->map[outlink_idx];
    AVFilterLink *inlink = ctx->inputs[inlink_idx];
    FilterLink      *inl = ff_filter_link(inlink);
    FFFrameSyncIn *in;
    int i, ret;

    av_log(ctx, AV_LOG_VERBOSE, "config output link %d "
           "with settings from input link %d\n",
           outlink_idx, inlink_idx);

    switch (outlink->type) {
    case AVMEDIA_TYPE_VIDEO:
        outlink->w = inlink->w;
        outlink->h = inlink->h;
        outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;
        outl->frame_rate = inl->frame_rate;
        break;
    case AVMEDIA_TYPE_AUDIO:
        outlink->sample_rate    = inlink->sample_rate;
        outlink->ch_layout.nb_channels       = inlink->ch_layout.nb_channels;
        break;
    }

    outlink->time_base = inlink->time_base;
    outlink->format = inlink->format;

    if (s->fs.opaque == s)
        return 0;

    if ((ret = ff_framesync_init(&s->fs, ctx, ctx->nb_inputs)) < 0)
        return ret;

    in = s->fs.in;
    s->fs.opaque = s;
    s->fs.on_event = process_frame;

    for (i = 0; i < ctx->nb_inputs; i++) {
        in[i].time_base = ctx->inputs[i]->time_base;
        in[i].sync      = 1;
        in[i].before    = EXT_STOP;
        in[i].after     = EXT_STOP;
    }

    s->frames = av_calloc(ctx->nb_inputs, sizeof(*s->frames));
    if (!s->frames)
        return AVERROR(ENOMEM);

    return ff_framesync_configure(&s->fs);
}

static int parse_definition(AVFilterContext *ctx, int nb_pads, int is_input, int is_audio)
{
    const char *padtype = is_input ? "in" : "out";
    int i = 0, ret = 0;

    for (i = 0; i < nb_pads; i++) {
        AVFilterPad pad = { 0 };

        pad.type = is_audio ? AVMEDIA_TYPE_AUDIO : AVMEDIA_TYPE_VIDEO;

        pad.name = av_asprintf("%sput%d", padtype, i);
        if (!pad.name)
            return AVERROR(ENOMEM);

        av_log(ctx, AV_LOG_DEBUG, "Add %s pad %s\n", padtype, pad.name);

        if (is_input) {
            ret = ff_append_inpad_free_name(ctx, &pad);
        } else {
            pad.config_props  = config_output;
            ret = ff_append_outpad_free_name(ctx, &pad);
        }
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int parse_mapping(AVFilterContext *ctx, const char *map)
{
    StreamSelectContext *s = ctx->priv;
    int *new_map;
    int new_nb_map = 0;

    if (!map) {
        av_log(ctx, AV_LOG_ERROR, "mapping definition is not set\n");
        return AVERROR(EINVAL);
    }

    new_map = av_calloc(s->nb_inputs, sizeof(*new_map));
    if (!new_map)
        return AVERROR(ENOMEM);

    while (1) {
        char *p;
        const int n = strtol(map, &p, 0);

        av_log(ctx, AV_LOG_DEBUG, "n=%d map=%p p=%p\n", n, map, p);

        if (map == p)
            break;
        map = p;

        if (new_nb_map >= s->nb_inputs) {
            av_log(ctx, AV_LOG_ERROR, "Unable to map more than the %d "
                   "input pads available\n", s->nb_inputs);
            av_free(new_map);
            return AVERROR(EINVAL);
        }

        if (n < 0 || n >= ctx->nb_inputs) {
            av_log(ctx, AV_LOG_ERROR, "Input stream index %d doesn't exist "
                   "(there is only %d input streams defined)\n",
                   n, s->nb_inputs);
            av_free(new_map);
            return AVERROR(EINVAL);
        }

        av_log(ctx, AV_LOG_VERBOSE, "Map input stream %d to output stream %d\n", n, new_nb_map);
        new_map[new_nb_map++] = n;
    }

    if (!new_nb_map) {
        av_log(ctx, AV_LOG_ERROR, "invalid mapping\n");
        av_free(new_map);
        return AVERROR(EINVAL);
    }

    av_freep(&s->map);
    s->map = new_map;
    s->nb_map = new_nb_map;

    av_log(ctx, AV_LOG_VERBOSE, "%d map set\n", s->nb_map);

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    if (!strcmp(cmd, "map")) {
        int ret = parse_mapping(ctx, arg);

        if (ret < 0)
            return ret;
        return 0;
    }
    return AVERROR(ENOSYS);
}

static av_cold int init(AVFilterContext *ctx)
{
    StreamSelectContext *s = ctx->priv;
    int ret, nb_outputs = 0;
    char *map = s->map_str;

    if (!strcmp(ctx->filter->name, "astreamselect"))
        s->is_audio = 1;

    for (; map;) {
        char *p;

        strtol(map, &p, 0);
        if (map == p)
            break;
        nb_outputs++;
        map = p;
    }

    s->last_pts = av_calloc(s->nb_inputs, sizeof(*s->last_pts));
    if (!s->last_pts)
        return AVERROR(ENOMEM);

    if ((ret = parse_definition(ctx, s->nb_inputs, 1, s->is_audio)) < 0 ||
        (ret = parse_definition(ctx, nb_outputs, 0, s->is_audio)) < 0)
        return ret;

    av_log(ctx, AV_LOG_DEBUG, "Configured with %d inpad and %d outpad\n",
           ctx->nb_inputs, ctx->nb_outputs);

    return parse_mapping(ctx, s->map_str);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    StreamSelectContext *s = ctx->priv;

    av_freep(&s->last_pts);
    av_freep(&s->map);
    av_freep(&s->frames);
    ff_framesync_uninit(&s->fs);
}

const FFFilter ff_vf_streamselect = {
    .p.name          = "streamselect",
    .p.description   = NULL_IF_CONFIG_SMALL("Select video streams"),
    .p.priv_class    = &streamselect_class,
    .p.flags         = AVFILTER_FLAG_DYNAMIC_INPUTS | AVFILTER_FLAG_DYNAMIC_OUTPUTS,
    .init            = init,
    .process_command = process_command,
    .uninit          = uninit,
    .activate        = activate,
    .priv_size       = sizeof(StreamSelectContext),
};

const FFFilter ff_af_astreamselect = {
    .p.name          = "astreamselect",
    .p.description   = NULL_IF_CONFIG_SMALL("Select audio streams"),
    .p.priv_class    = &streamselect_class,
    .p.flags         = AVFILTER_FLAG_DYNAMIC_INPUTS | AVFILTER_FLAG_DYNAMIC_OUTPUTS,
    .init            = init,
    .process_command = process_command,
    .uninit          = uninit,
    .activate        = activate,
    .priv_size       = sizeof(StreamSelectContext),
};
