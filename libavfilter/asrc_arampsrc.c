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
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioRampSourceContext {
    const AVClass *class;

    double *start;
    unsigned nb_start;

    double *stop;
    unsigned nb_stop;

    double *interval;
    unsigned nb_interval;

    AVChannelLayout ch_layout;
    int samples_per_frame;
    int sample_rate;
    int64_t duration;

    int nb_channels;
    double value;
    double step;
    double left;
    unsigned index;
    int64_t pts;

    void (*output_samples)(AVFilterContext *ctx, AVFrame *frame);
} AudioRampSourceContext;

#define OFFSET(x) offsetof(AudioRampSourceContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_start    = {.def="0",  .size_min=1,.sep=' '};
static const AVOptionArrayDef def_stop     = {.def="1",  .size_min=1,.sep=' '};
static const AVOptionArrayDef def_interval = {.def="20", .size_min=1,.sep=' '};

static const AVOption arampsrc_options[] = {
    { "start", "set the ramp start", OFFSET(start), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_start}, -FLT_MAX, FLT_MAX, AFT },
    { "stop", "set the ramp stop", OFFSET(stop), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_stop}, -FLT_MAX, FLT_MAX, AFT },
    { "interval", "set the ramp interval", OFFSET(interval), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_interval}, 0, 999999, AFT },
    { "channel_layout", "set the channel layout", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str = "stereo"}, 0, 0, AF },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, AF },
    { "duration", "set the duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, AF },
    { "samples_per_frame", "set the samples per frame", OFFSET(samples_per_frame), AV_OPT_TYPE_INT,  {.i64=1024}, 64, 65536, AFT },
    {NULL}
};

AVFILTER_DEFINE_CLASS(arampsrc);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AudioRampSourceContext *s = ctx->priv;
    const AVChannelLayout chlayouts[] = { s->ch_layout, { 0 } };
    const int sample_rates[] = { s->sample_rate, -1 };
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
#include "arampsrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "arampsrc_template.c"

static av_cold int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioRampSourceContext *s = ctx->priv;

    s->nb_channels = outlink->ch_layout.nb_channels;
    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);
    s->index = 0;
    s->value = s->start[0];
    s->left = s->interval[0] * outlink->sample_rate;
    s->step = (s->stop[0] - s->start[0]) / s->left;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->output_samples = output_samples_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->output_samples = output_samples_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRampSourceContext *s = ctx->priv;
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

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_props,
    },
};

const FFFilter ff_asrc_arampsrc = {
    .p.name          = "arampsrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Generate Audio Ramp samples."),
    .p.priv_class    = &arampsrc_class,
    .p.inputs        = NULL,
    .activate        = activate,
    .priv_size       = sizeof(AudioRampSourceContext),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
