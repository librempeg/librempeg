/*
 * Copyright (c) 2012 Stefano Sabatini
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

/**
 * @file
 * send commands filter
 */

#include "config_components.h"

#include "libavutil/bprint.h"
#include "libavutil/eval.h"
#include "libavutil/file.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "avfilter.h"
#include "filters.h"
#include "audio.h"
#include "video.h"

#define COMMAND_FLAG_ENTER 1
#define COMMAND_FLAG_LEAVE 2
#define COMMAND_FLAG_EXPR  4
#define COMMAND_FLAG_MASK  (COMMAND_FLAG_ENTER|COMMAND_FLAG_LEAVE|COMMAND_FLAG_EXPR)

static const char *const var_names[] = {
    "N",     /* frame number */
    "T",     /* frame time in seconds */
    "PTS",   /* frame pts */
    "TS",    /* interval start time in seconds */
    "TE",    /* interval end time in seconds */
    "TI",    /* interval interpolated value: TI = (T - TS) / (TE - TS) */
    "W",     /* width for video frames */
    "H",     /* height for video frames */
    NULL
};

enum var_name {
    VAR_N,
    VAR_T,
    VAR_PTS,
    VAR_TS,
    VAR_TE,
    VAR_TI,
    VAR_W,
    VAR_H,
    VAR_VARS_NB
};

typedef struct Interval {
    int64_t start_ts;          ///< start timestamp expressed as microseconds units
    int64_t end_ts;            ///< end   timestamp expressed as microseconds units
    int index;                 ///< unique index for these interval commands
    int enabled;               ///< current time detected inside this interval
} Interval;

typedef struct SendCmdContext {
    const AVClass *class;

    int64_t *start_opt;
    unsigned nb_start_opt;

    int64_t *end_opt;
    unsigned nb_end_opt;

    int *flags_opt;
    unsigned nb_flags_opt;

    char **targets_opt;
    unsigned nb_targets_opt;

    char **commands_opt;
    unsigned nb_commands_opt;

    char **args_opt;
    unsigned nb_args_opt;

    Interval *intervals;
    unsigned nb_intervals;

    AVExpr **e;
} SendCmdContext;

#define OFFSET(x) offsetof(SendCmdContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_VIDEO_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_start    = {.def=NULL,   .size_min=0,.sep='|'};
static const AVOptionArrayDef def_end      = {.def=NULL,   .size_min=0,.sep='|'};
static const AVOptionArrayDef def_flags    = {.def="enter",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_targets  = {.def=NULL,   .size_min=1,.sep='|'};
static const AVOptionArrayDef def_commands = {.def=NULL,   .size_min=1,.sep='|'};
static const AVOptionArrayDef def_args     = {.def=NULL,   .size_min=0,.sep='|'};

static const AVOption options[] = {
    { "start",    "set start times", OFFSET(start_opt),    AV_OPT_TYPE_DURATION|AR, {.arr=&def_start}, INT32_MIN, INT32_MAX, .flags=FLAGS },
    { "end",      "set end times",   OFFSET(end_opt),      AV_OPT_TYPE_DURATION|AR, {.arr=&def_end},   INT32_MIN, INT32_MAX, .flags=FLAGS },
    { "flags",    "set flags",       OFFSET(flags_opt),    AV_OPT_TYPE_FLAGS|AR,    {.arr=&def_flags}, 0, COMMAND_FLAG_MASK, .flags=FLAGS, .unit="flags" },
    {  "enter",   NULL,              0,                    AV_OPT_TYPE_CONST,       {.i64=COMMAND_FLAG_ENTER}, .flags=FLAGS, .unit = "flags" },
    {  "leave",   NULL,              0,                    AV_OPT_TYPE_CONST,       {.i64=COMMAND_FLAG_LEAVE}, .flags=FLAGS, .unit = "flags" },
    {  "expr",    NULL,              0,                    AV_OPT_TYPE_CONST,       {.i64=COMMAND_FLAG_EXPR},  .flags=FLAGS, .unit = "flags" },
    { "targets",  "set targets",     OFFSET(targets_opt),  AV_OPT_TYPE_STRING|AR,   {.arr=&def_targets},       0,         0, .flags=FLAGS },
    { "commands", "set commands",    OFFSET(commands_opt), AV_OPT_TYPE_STRING|AR,   {.arr=&def_commands},      0,         0, .flags=FLAGS },
    { "args",     "set args",        OFFSET(args_opt),     AV_OPT_TYPE_STRING|AR,   {.arr=&def_args},          0,         0, .flags=FLAGS },
    { NULL }
};

static int cmp_intervals(const void *a, const void *b)
{
    const Interval *i1 = a;
    const Interval *i2 = b;
    return 2 * FFDIFFSIGN(i1->start_ts, i2->start_ts) + FFDIFFSIGN(i1->index, i2->index);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SendCmdContext *s = ctx->priv;

    s->nb_intervals = FFMAX(s->nb_start_opt, s->nb_end_opt);
    if (!s->nb_intervals) {
        av_log(ctx, AV_LOG_ERROR, "start and/or end interval not set\n");
        return AVERROR(EINVAL);
    }

    s->intervals = av_calloc(s->nb_intervals, sizeof(*s->intervals));
    if (!s->intervals)
        return AVERROR(ENOMEM);

    s->e = av_calloc(s->nb_args_opt, sizeof(*s->e));
    if (!s->e)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->nb_intervals; i++) {
        Interval *interval = &s->intervals[i];

        if (i < s->nb_start_opt)
            interval->start_ts = av_rescale_q(s->start_opt[i],
                                              AV_TIME_BASE_Q,
                                              inlink->time_base);
        else
            interval->start_ts = INT64_MIN;

        if (i < s->nb_end_opt)
            interval->end_ts = av_rescale_q(s->end_opt[i],
                                            AV_TIME_BASE_Q,
                                            inlink->time_base);
        else
            interval->end_ts = INT64_MAX;

        interval->index = i;
    }

    qsort(s->intervals, s->nb_intervals, sizeof(Interval), cmp_intervals);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SendCmdContext *s = ctx->priv;

    if (s->e) {
        for (int n = 0; n < s->nb_args_opt; n++) {
            av_expr_free(s->e[n]);
            s->e[n] = NULL;
        }
        av_freep(&s->e);
    }
    av_freep(&s->intervals);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *ref)
{
    FilterLink *inl = ff_filter_link(inlink);
    AVFilterContext *ctx = inlink->dst;
    SendCmdContext *s = ctx->priv;
    int64_t ts, duration;
    int i, j, ret;

    if (ref->pts == AV_NOPTS_VALUE)
        goto end;

    ts = ref->pts;
    duration = ref->duration;

#define WITHIN_INTERVAL(ts, duration, start_ts, end_ts) ((ts)+(duration) >= (start_ts) && (ts) < (end_ts))

    for (i = 0; i < s->nb_intervals; i++) {
        Interval *interval = &s->intervals[i];
        int flags = 0;

        if (!interval->enabled && WITHIN_INTERVAL(ts, duration, interval->start_ts, interval->end_ts)) {
            flags += COMMAND_FLAG_ENTER;
            interval->enabled = 1;
        }
        if (interval->enabled && !WITHIN_INTERVAL(ts, duration, interval->start_ts, interval->end_ts)) {
            flags += COMMAND_FLAG_LEAVE;
            interval->enabled = 0;
        }
        if (interval->enabled)
            flags += COMMAND_FLAG_EXPR;

        if (flags) {
            char buf[1024];

            for (j = 0; j < s->nb_commands_opt; j++) {
                const int target_flags = (j < s->nb_flags_opt) ? s->flags_opt[j] : COMMAND_FLAG_ENTER;

                if (target_flags & flags) {
                    char *cmd_arg = (j < s->nb_args_opt) ? s->args_opt[j] : NULL;

                    if ((target_flags & COMMAND_FLAG_EXPR) && cmd_arg) {
                        double var_values[VAR_VARS_NB], res;
                        double start = TS2T(interval->start_ts, AV_TIME_BASE_Q);
                        double end = TS2T(interval->end_ts, AV_TIME_BASE_Q);
                        double current = TS2T(ref->pts, inlink->time_base);

                        var_values[VAR_N]   = inl->frame_count_in;
                        var_values[VAR_PTS] = TS2D(ref->pts);
                        var_values[VAR_T]   = current;
                        var_values[VAR_TS]  = start;
                        var_values[VAR_TE]  = end;
                        var_values[VAR_TI]  = (current - start) / (end - start);
                        var_values[VAR_W]   = ref->width;
                        var_values[VAR_H]   = ref->height;

                        if (!s->e[j]) {
                            ret = av_expr_parse(&s->e[j], cmd_arg, var_names,
                                                NULL, NULL, NULL, NULL, 0, NULL);
                            if (ret < 0) {
                                av_log(ctx, AV_LOG_ERROR, "Invalid expression '%s' for command argument.\n", cmd_arg);
                                av_frame_free(&ref);
                                return ret;
                            }
                        }
                        res = av_expr_eval(s->e[j], var_values, NULL);
                        cmd_arg = av_asprintf("%g", res);
                        if (!cmd_arg) {
                            av_frame_free(&ref);
                            return AVERROR(ENOMEM);
                        }
                    }

                    for (int k = 0; k < s->nb_targets_opt; k++) {
                        av_log(ctx, AV_LOG_VERBOSE,
                               "Processing command #%d target:%s command:%s arg:%s\n",
                               j, s->targets_opt[k], s->commands_opt[j], cmd_arg);
                        ret = avfilter_graph_send_command(inl->graph,
                                                          s->targets_opt[k], s->commands_opt[j], cmd_arg,
                                                          buf, sizeof(buf),
                                                          AVFILTER_CMD_FLAG_ONE);
                        av_log(ctx, AV_LOG_VERBOSE,
                               "Command reply for command #%d: ret:%s res:%s\n",
                               j, av_err2str(ret), buf);
                    }

                    if (target_flags & COMMAND_FLAG_EXPR)
                        av_freep(&cmd_arg);
                }
            }
        }
    }

end:
    switch (inlink->type) {
    case AVMEDIA_TYPE_VIDEO:
    case AVMEDIA_TYPE_AUDIO:
        return ff_filter_frame(inlink->dst->outputs[0], ref);
    }

    return AVERROR(ENOSYS);
}

AVFILTER_DEFINE_CLASS_EXT(sendcmd, "(a)sendcmd", options);

#if CONFIG_SENDCMD_FILTER

static const AVFilterPad sendcmd_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_sendcmd = {
    .p.name        = "sendcmd",
    .p.description = NULL_IF_CONFIG_SMALL("Send commands to filters."),
    .p.priv_class  = &sendcmd_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size   = sizeof(SendCmdContext),
    .uninit      = uninit,
    FILTER_INPUTS(sendcmd_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};

#endif

#if CONFIG_ASENDCMD_FILTER

static const AVFilterPad asendcmd_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_asendcmd = {
    .p.name      = "asendcmd",
    .p.description = NULL_IF_CONFIG_SMALL("Send commands to filters."),
    .p.priv_class= &sendcmd_class,
    .p.flags     = AVFILTER_FLAG_METADATA_ONLY,
    .uninit      = uninit,
    .priv_size   = sizeof(SendCmdContext),
    FILTER_INPUTS(asendcmd_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
};

#endif
