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

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct SweepContext {
    const AVClass *class;

    double start_frequency;
    double stop_frequency;
    double amplitude;
    double phase;
    double period;
    double position;
    double direction;
    double samples;
    double increment;
    double frequency_range;

    int type;
    int samples_per_frame;
    int sample_rate;
    int64_t duration;

    int64_t pts;

    int (*init_state)(AVFilterContext *ctx);
    void (*output_samples)(AVFilterContext *ctx, AVFrame *frame);
} SweepContext;

#define OFFSET(x) offsetof(SweepContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption sweep_options[] = {
    { "start", "set the sweep start frequency", OFFSET(start_frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0.0001}, 0.0, 0.5, AFT },
    { "stop", "set the sweep stop frequency", OFFSET(stop_frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0.5}, 0.0, 0.5, AFT },
    { "period", "set the single sweep period duration", OFFSET(period), AV_OPT_TYPE_DOUBLE, {.dbl=30}, 0.0, 60, AFT },
    { "amplitude", "set the sweep amplitude", OFFSET(amplitude), AV_OPT_TYPE_DOUBLE, {.dbl=0.2}, 0.0, 1.0, AFT },
    { "type", "set sweep type", OFFSET(type), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, AFT },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, AF },
    { "duration", "set the duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, AF },
    { "samples_per_frame", "set the samples per frame", OFFSET(samples_per_frame), AV_OPT_TYPE_INT,  {.i64=1024}, 64, 65536, AFT },
    {NULL}
};

AVFILTER_DEFINE_CLASS(sweep);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const SweepContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

#define DEPTH 32
#include "sweep_template.c"

#undef DEPTH
#define DEPTH 64
#include "sweep_template.c"

static av_cold int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    SweepContext *s = ctx->priv;

    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->init_state = init_state_fltp;
        s->output_samples = output_samples_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_state = init_state_dblp;
        s->output_samples = output_samples_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    SweepContext *s = ctx->priv;
    int nb_samples = s->samples_per_frame;
    AVFrame *frame;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    if (s->duration) {
        nb_samples = FFMIN(nb_samples, s->duration - s->pts);
        av_assert1(nb_samples >= 0);
        if (!nb_samples) {
            ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
            return 0;
        }
    }

    if (!(frame = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);

    s->output_samples(ctx, frame);

    frame->pts = s->pts;
    frame->duration = nb_samples;
    s->pts += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    SweepContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->init_state(ctx);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_props,
    },
};

const FFFilter ff_asrc_sweep = {
    .p.name          = "sweep",
    .p.description   = NULL_IF_CONFIG_SMALL("Generate Sweep samples."),
    .p.priv_class    = &sweep_class,
    .p.inputs        = NULL,
    .activate        = activate,
    .priv_size       = sizeof(SweepContext),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = process_command,
};
