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

#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

typedef struct CrossfeedContext {
    const AVClass *class;

    double range;
    double strength;
    double slope;
    double level_in;
    double level_out;
    int block_samples;
    int block_size;

    void *st;

    int64_t pts;
    int nb_samples;

    int (*init_state)(AVFilterContext *ctx);
    void (*uninit_state)(AVFilterContext *ctx);
    int (*xfeed_frame)(AVFilterLink *inlink, AVFrame *in);
} CrossfeedContext;

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layout = NULL;
    int ret;

    if ((ret = ff_add_format                 (&formats, AV_SAMPLE_FMT_DBL  )) < 0 ||
        (ret = ff_add_format                 (&formats, AV_SAMPLE_FMT_FLT  )) < 0 ||
        (ret = ff_set_common_formats         (ctx     , formats            )) < 0 ||
        (ret = ff_add_channel_layout         (&layout , &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO)) < 0 ||
        (ret = ff_set_common_channel_layouts (ctx     , layout             )) < 0 ||
        (ret = ff_set_common_all_samplerates (ctx                          )) < 0)
        return ret;

    return 0;
}

#define DEPTH 32
#include "crossfeed_template.c"

#undef DEPTH
#define DEPTH 64
#include "crossfeed_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    CrossfeedContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBL:
        s->xfeed_frame = xfeed_frame_dbl;
        s->init_state = init_state_dbl;
        s->uninit_state = uninit_state_dbl;
        break;
    case AV_SAMPLE_FMT_FLT:
        s->xfeed_frame = xfeed_frame_flt;
        s->init_state = init_state_flt;
        s->uninit_state = uninit_state_flt;
        break;
    }

    return s->init_state(ctx);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    CrossfeedContext *s = ctx->priv;
    AVFrame *in = NULL;
    int64_t pts;
    int status;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (s->block_samples > 0) {
        ret = ff_inlink_consume_samples(inlink, s->block_samples, s->block_samples, &in);
    } else {
        ret = ff_inlink_consume_frame(inlink, &in);
    }
    if (ret < 0)
        return ret;
    if (ret > 0)
        return s->xfeed_frame(inlink, in);

    if (s->block_samples > 0 && ff_inlink_queued_samples(inlink) >= s->block_samples) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->block_samples > 0) {
            AVFrame *in = ff_get_audio_buffer(outlink, s->block_samples);
            if (!in)
                return AVERROR(ENOMEM);

            ret = s->xfeed_frame(inlink, in);
        }

        ff_outlink_set_status(outlink, status, pts);

        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    return config_input(ctx->inputs[0]);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CrossfeedContext *s = ctx->priv;

    if (s->uninit_state)
        s->uninit_state(ctx);
}

#define OFFSET(x) offsetof(CrossfeedContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption crossfeed_options[] = {
    { "strength",  "set crossfeed strength",  OFFSET(strength),  AV_OPT_TYPE_DOUBLE, {.dbl=.2}, 0, 1, FLAGS },
    { "range",     "set soundstage wideness", OFFSET(range),     AV_OPT_TYPE_DOUBLE, {.dbl=.5}, 0, 1, FLAGS },
    { "slope",     "set curve slope",         OFFSET(slope),     AV_OPT_TYPE_DOUBLE, {.dbl=.5}, .01, 1, FLAGS },
    { "level_in",  "set level in",            OFFSET(level_in),  AV_OPT_TYPE_DOUBLE, {.dbl=.9}, 0, 1, FLAGS },
    { "level_out", "set level out",           OFFSET(level_out), AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 1, FLAGS },
    { "block_size", "set the block size",     OFFSET(block_size),AV_OPT_TYPE_INT,    {.i64=0}, 0, 32768, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(crossfeed);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_crossfeed = {
    .name           = "crossfeed",
    .description    = NULL_IF_CONFIG_SMALL("Apply headphone crossfeed filter."),
    .priv_size      = sizeof(CrossfeedContext),
    .priv_class     = &crossfeed_class,
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC(query_formats),
    .flags          = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .process_command = process_command,
};
