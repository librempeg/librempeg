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

#include "config_components.h"

#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

enum BenchAction {
    ACTION_START,
    ACTION_STOP,
    NB_ACTION
};

typedef struct BenchContext {
    const AVClass *class;
    int action;
    int64_t max, min;
    int64_t sum;
    int n;
    int loglevel;
} BenchContext;

#define OFFSET(x) offsetof(BenchContext, x)
#define DEFINE_OPTIONS(filt_name, FLAGS)                                                                                \
static const AVOption filt_name##_options[] = {                                                                         \
    { "action", "set action", OFFSET(action), AV_OPT_TYPE_INT, {.i64=ACTION_START}, 0, NB_ACTION-1, FLAGS, .unit = "action" },  \
        { "start", "start timer",  0, AV_OPT_TYPE_CONST, {.i64=ACTION_START}, 0, 0, FLAGS, .unit = "action" },      \
        { "stop",  "stop timer",   0, AV_OPT_TYPE_CONST, {.i64=ACTION_STOP},  0, 0, FLAGS, .unit = "action" },      \
    { "framelog", "force frame logging level", OFFSET(loglevel), AV_OPT_TYPE_INT, {.i64 = -1},   INT_MIN, INT_MAX, FLAGS, .unit = "level" }, \
        { "quiet",   "logging disabled",          0, AV_OPT_TYPE_CONST, {.i64 = AV_LOG_QUIET},   0, 0, FLAGS, .unit = "level" }, \
        { "info",    "information logging level", 0, AV_OPT_TYPE_CONST, {.i64 = AV_LOG_INFO},    0, 0, FLAGS, .unit = "level" }, \
        { "verbose", "verbose logging level",     0, AV_OPT_TYPE_CONST, {.i64 = AV_LOG_VERBOSE}, 0, 0, FLAGS, .unit = "level" }, \
    { NULL }                                                                                                            \
}

#define START_TIME_KEY "lavfi.bench.start_time"
#define T2F(v) ((v) / 1000000.)

static av_cold int init(AVFilterContext *ctx)
{
    BenchContext *s = ctx->priv;
    s->min = INT64_MAX;
    s->max = INT64_MIN;

    if (s->loglevel != AV_LOG_INFO &&
        s->loglevel != AV_LOG_QUIET &&
        s->loglevel != AV_LOG_VERBOSE) {
        s->loglevel = AV_LOG_INFO;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    BenchContext *s = ctx->priv;

    if (s->n > 0)
        av_log(ctx, AV_LOG_INFO, "avg:%f max:%f min:%f\n",
               T2F(s->sum / s->n), T2F(s->max), T2F(s->min));
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    BenchContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int64_t t = av_gettime();

    if (t < 0)
        return ff_filter_frame(outlink, in);

    if (s->action == ACTION_START) {
        av_dict_set_int(&in->metadata, START_TIME_KEY, t, 0);
    } else if (s->action == ACTION_STOP) {
        AVDictionaryEntry *e = av_dict_get(in->metadata, START_TIME_KEY, NULL, 0);
        if (e) {
            const int64_t start = strtoll(e->value, NULL, 0);
            const int64_t diff = t - start;
            s->sum += diff;
            s->n++;
            s->min = FFMIN(s->min, diff);
            s->max = FFMAX(s->max, diff);
            if (s->loglevel != AV_LOG_QUIET)
                av_log(ctx, s->loglevel, "t:%f avg:%f max:%f min:%f\n",
                       T2F(diff), T2F(s->sum / s->n), T2F(s->max), T2F(s->min));
        }
        av_dict_set(&in->metadata, START_TIME_KEY, NULL, 0);
    }

    return ff_filter_frame(outlink, in);
}

#if CONFIG_BENCH_FILTER
DEFINE_OPTIONS(bench, AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM);
AVFILTER_DEFINE_CLASS(bench);

static const AVFilterPad bench_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_bench = {
    .p.name        = "bench",
    .p.description = NULL_IF_CONFIG_SMALL("Benchmark part of a filtergraph."),
    .p.priv_class  = &bench_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(BenchContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(bench_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    .process_command = ff_filter_process_command,
};
#endif /* CONFIG_BENCH_FILTER */

#if CONFIG_ABENCH_FILTER
DEFINE_OPTIONS(abench, AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM);
AVFILTER_DEFINE_CLASS(abench);

static const AVFilterPad abench_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_abench = {
    .p.name        = "abench",
    .p.description = NULL_IF_CONFIG_SMALL("Benchmark part of a filtergraph."),
    .p.priv_class  = &abench_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(BenchContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(abench_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    .process_command = ff_filter_process_command,
};
#endif /* CONFIG_ABENCH_FILTER */
