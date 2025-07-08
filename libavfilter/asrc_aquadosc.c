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

typedef struct AQuadOscContext {
    const AVClass *class;

    double frequency;
    double old_frequency;
    double amplitude;
    double phase;
    double old_phase;
    double offset;

    int samples_per_frame;
    int sample_rate;
    int64_t duration;

    int64_t pts;

    void *st;

    int (*init_state)(AVFilterContext *ctx);
    void (*output_samples)(AVFilterContext *ctx, void *st, AVFrame *frame);
} AQuadOscContext;

#define OFFSET(x) offsetof(AQuadOscContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aquadosc_options[] = {
    { "frequency", "set the oscillator frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0.01}, 0.0, 0.5, AFT },
    { "amplitude", "set the oscillator amplitude", OFFSET(amplitude), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.0, 1.0, AFT },
    { "phase", "set the oscillator phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1.0, 1.0, AFT },
    { "offset", "set the oscillator offset", OFFSET(offset), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, -1.0, 1.0, AFT },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, AF },
    { "duration", "set the duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, AF },
    { "samples_per_frame", "set the samples per frame", OFFSET(samples_per_frame), AV_OPT_TYPE_INT,  {.i64=1024}, 64, 65536, AFT },
    {NULL}
};

AVFILTER_DEFINE_CLASS(aquadosc);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AQuadOscContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_STEREO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

#define DEPTH 16
#include "aquadosc_template.c"

#undef DEPTH
#define DEPTH 31
#include "aquadosc_template.c"

#undef DEPTH
#define DEPTH 32
#include "aquadosc_template.c"

#undef DEPTH
#define DEPTH 64
#include "aquadosc_template.c"

static av_cold int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AQuadOscContext *s = ctx->priv;

    s->old_frequency = -1.0;
    s->old_phase = -2.0;
    s->duration = av_rescale(s->duration, s->sample_rate, AV_TIME_BASE);

    switch (outlink->format) {
    case AV_SAMPLE_FMT_S16P:
        s->init_state = init_state_s16p;
        s->output_samples = output_samples_s16p;
        break;
    case AV_SAMPLE_FMT_S32P:
        s->init_state = init_state_s32p;
        s->output_samples = output_samples_s32p;
        break;
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
    AQuadOscContext *s = ctx->priv;
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

    s->output_samples(ctx, s->st, frame);

    frame->pts = s->pts;
    frame->duration = nb_samples;
    s->pts += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AQuadOscContext *s = ctx->priv;

    av_freep(&s->st);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AQuadOscContext *s = ctx->priv;
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

const FFFilter ff_asrc_aquadosc = {
    .p.name          = "aquadosc",
    .p.description   = NULL_IF_CONFIG_SMALL("Generate Quadrature Oscillator samples."),
    .p.priv_class    = &aquadosc_class,
    .p.inputs        = NULL,
    .activate        = activate,
    .uninit          = uninit,
    .priv_size       = sizeof(AQuadOscContext),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = process_command,
};
