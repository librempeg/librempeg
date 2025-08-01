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

#include <rubberband/rubberband-c.h>

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct RubberBandContext {
    const AVClass *class;
    RubberBandState rbs;

    double tempo, pitch;
    int transients, detector, phase, window,
        smoothing, formant, opitch, channels;
    int64_t nb_samples_out;
    int64_t nb_samples_in;
    int64_t first_pts;
    int64_t last_pts;
    int nb_samples;
    int eof;
} RubberBandContext;

#define OFFSET(x) offsetof(RubberBandContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption rubberband_options[] = {
    { "tempo",      "set tempo scale factor", OFFSET(tempo), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.01, 100, AT },
    { "pitch",      "set pitch scale factor", OFFSET(pitch), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.01, 100, AT },
    { "transients", "set transients", OFFSET(transients), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "transients" },
        { "crisp",  0,                0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionTransientsCrisp},  0, 0, A, .unit = "transients" },
        { "mixed",  0,                0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionTransientsMixed},  0, 0, A, .unit = "transients" },
        { "smooth", 0,                0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionTransientsSmooth}, 0, 0, A, .unit = "transients" },
    { "detector",   "set detector",   OFFSET(detector),   AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "detector" },
        { "compound",   0,            0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionDetectorCompound},   0, 0, A, .unit = "detector" },
        { "percussive", 0,            0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionDetectorPercussive}, 0, 0, A, .unit = "detector" },
        { "soft",       0,            0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionDetectorSoft},       0, 0, A, .unit = "detector" },
    { "phase",      "set phase",      OFFSET(phase),      AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "phase" },
        { "laminar",     0,           0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionPhaseLaminar},     0, 0, A, .unit = "phase" },
        { "independent", 0,           0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionPhaseIndependent}, 0, 0, A, .unit = "phase" },
    { "window",     "set window",     OFFSET(window),     AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "window" },
        { "standard", 0,              0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionWindowStandard}, 0, 0, A, .unit = "window" },
        { "short",    0,              0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionWindowShort},    0, 0, A, .unit = "window" },
        { "long",     0,              0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionWindowLong},     0, 0, A, .unit = "window" },
    { "smoothing",  "set smoothing",  OFFSET(smoothing),  AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "smoothing" },
        { "off",    0,                0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionSmoothingOff}, 0, 0, A, .unit = "smoothing" },
        { "on",     0,                0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionSmoothingOn},  0, 0, A, .unit = "smoothing" },
    { "formant",    "set formant",    OFFSET(formant),    AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "formant" },
        { "shifted",    0,            0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionFormantShifted},   0, 0, A, .unit = "formant" },
        { "preserved",  0,            0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionFormantPreserved}, 0, 0, A, .unit = "formant" },
    { "pitchq",     "set pitch quality", OFFSET(opitch),  AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "pitch" },
        { "quality",     0,           0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionPitchHighQuality},     0, 0, A, .unit = "pitch" },
        { "speed",       0,           0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionPitchHighSpeed},       0, 0, A, .unit = "pitch" },
        { "consistency", 0,           0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionPitchHighConsistency}, 0, 0, A, .unit = "pitch" },
    { "channels",   "set channels",   OFFSET(channels),   AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, A, .unit = "channels" },
        { "apart",    0,              0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionChannelsApart},    0, 0, A, .unit = "channels" },
        { "together", 0,              0,                  AV_OPT_TYPE_CONST, {.i64=RubberBandOptionChannelsTogether}, 0, 0, A, .unit = "channels" },
    { NULL },
};

AVFILTER_DEFINE_CLASS(rubberband);

static av_cold void uninit(AVFilterContext *ctx)
{
    RubberBandContext *s = ctx->priv;

    if (s->rbs)
        rubberband_delete(s->rbs);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    RubberBandContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    int ret = 0, nb_samples;

    if (s->first_pts == AV_NOPTS_VALUE)
        s->first_pts = in->pts;

    rubberband_process(s->rbs, (const float *const *)in->extended_data, in->nb_samples, s->eof);
    s->nb_samples_in += in->nb_samples;

    nb_samples = rubberband_available(s->rbs);
    if (nb_samples > 0) {
        out = ff_get_audio_buffer(outlink, nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        out->pts = s->first_pts + av_rescale_q(s->nb_samples_out,
                     (AVRational){ 1, outlink->sample_rate },
                     outlink->time_base);
        s->last_pts = out->pts;
        nb_samples = rubberband_retrieve(s->rbs, (float *const *)out->extended_data, nb_samples);
        out->nb_samples = nb_samples;
        ret = ff_filter_frame(outlink, out);
        s->nb_samples_out += nb_samples;
    }

    av_frame_free(&in);
    if (ff_inlink_queued_samples(inlink) >= s->nb_samples)
        ff_filter_set_ready(ctx, 100);
    return ret < 0 ? ret : nb_samples;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    RubberBandContext *s = ctx->priv;
    int opts = s->transients|s->detector|s->phase|s->window|
               s->smoothing|s->formant|s->opitch|s->channels|
               RubberBandOptionProcessRealTime;

    if (s->rbs)
        rubberband_delete(s->rbs);
    s->rbs = rubberband_new(inlink->sample_rate, inlink->ch_layout.nb_channels, opts, 1. / s->tempo, s->pitch);
    if (!s->rbs)
        return AVERROR(ENOMEM);

    s->nb_samples = rubberband_get_samples_required(s->rbs);
    s->first_pts = AV_NOPTS_VALUE;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    RubberBandContext *s = ctx->priv;
    AVFrame *in = NULL;
    int64_t pts;
    int status, ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->nb_samples, s->nb_samples, &in);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts))
        s->eof |= status == AVERROR_EOF;

    if (ret < 0)
        return ret;
    if (ret > 0) {
        ret = filter_frame(inlink, in);
        if (ret != 0)
            return ret;
    }

    if (s->eof) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->last_pts);
        return 0;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    RubberBandContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    rubberband_set_time_ratio(s->rbs, 1. / s->tempo);
    rubberband_set_pitch_scale(s->rbs, s->pitch);
    s->nb_samples = rubberband_get_samples_required(s->rbs);

    return 0;
}

static const AVFilterPad rubberband_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_input,
    },
};

const FFFilter ff_af_rubberband = {
    .p.name        = "rubberband",
    .p.description = NULL_IF_CONFIG_SMALL("Apply time-stretching and pitch-shifting."),
    .p.priv_class  = &rubberband_class,
    .priv_size     = sizeof(RubberBandContext),
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(rubberband_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_FLTP),
    .process_command = process_command,
};
