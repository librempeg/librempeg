/*
 * Audio Mix Filter
 * Copyright (c) 2012 Justin Ruggles <justin.ruggles@gmail.com>
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
 * Audio Mix Filter
 *
 * Mixes audio from multiple sources into a single output. The channel layout,
 * sample rate, and sample format will be the same for all inputs and the
 * output.
 */

#include "libavutil/attributes.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/cpu.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

#define INPUT_EOF      1    /**< input has reached EOF (may still be active) */
#define INPUT_EMPTY    2    /**< input have no more samples */
#define INPUT_OFF     (INPUT_EOF|INPUT_EMPTY)

#define DURATION_LONGEST  0
#define DURATION_SHORTEST 1
#define DURATION_FIRST    2

typedef struct InputContext {
    AVFrame *frame;
    uint8_t input_state;       /**< current state of each input */
    float input_scale;         /**< mixing scale factor for this input */
    float weight;              /**< custom weight for this input */
    float scale_norm;          /**< normalization factor for this input */
} InputContext;

typedef struct MixContext {
    const AVClass *class;       /**< class for AVOptions */
    AVFloatDSPContext *fdsp;

    int nb_inputs;              /**< number of inputs */
    int got_inputs;
    int active_inputs;
    int duration_mode;          /**< mode for determining duration */
    float dropout_transition;   /**< transition time when an input drops out */
    float *weights_opt;         /**< array of custom weights for every input */
    unsigned nb_weights;
    int normalize;              /**< if inputs are scaled */

    int64_t eof_pts;

    InputContext *inputs;

    int first_input;
    int nb_samples;
    int nb_channels;            /**< number of channels */
    int sample_rate;            /**< sample rate */
    int planar;
    float weight_sum;           /**< sum of custom weights for every input */
} MixContext;

#define OFFSET(x) offsetof(MixContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define T AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY
static const AVOptionArrayDef def_weights = {.def="1 1",.size_min=1,.sep=' '};
static const AVOption amix_options[] = {
    { "inputs", "Number of inputs",
            OFFSET(nb_inputs), AV_OPT_TYPE_INT, { .i64 = 2 }, 1, INT16_MAX, A|F },
    { "duration", "How to determine the end-of-stream",
            OFFSET(duration_mode), AV_OPT_TYPE_INT, { .i64 = DURATION_LONGEST }, 0,  2, A|F, .unit = "duration" },
        { "longest",  "Duration of longest input",  0, AV_OPT_TYPE_CONST, { .i64 = DURATION_LONGEST  }, 0, 0, A|F, .unit = "duration" },
        { "shortest", "Duration of shortest input", 0, AV_OPT_TYPE_CONST, { .i64 = DURATION_SHORTEST }, 0, 0, A|F, .unit = "duration" },
        { "first",    "Duration of first input",    0, AV_OPT_TYPE_CONST, { .i64 = DURATION_FIRST    }, 0, 0, A|F, .unit = "duration" },
    { "dropout_transition", "Transition time, in seconds, for volume "
                            "renormalization when an input stream ends",
            OFFSET(dropout_transition), AV_OPT_TYPE_FLOAT, { .dbl = 2.0 }, 0, INT_MAX, A|F },
    { "weights", "Set weight for each input",
            OFFSET(weights_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_weights}, INT_MIN, INT_MAX, A|F|T },
    { "normalize", "Scale inputs",
            OFFSET(normalize), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, A|F|T },
    { NULL }
};

AVFILTER_DEFINE_CLASS(amix);

/**
 * Update the scaling factors to apply to each input during mixing.
 *
 * This balances the full volume range between active inputs and handles
 * volume transitions when EOF is encountered on an input but mixing continues
 * with the remaining inputs.
 */
static void calculate_scales(MixContext *s)
{
    float weight_sum = 0.f;

    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        if (ic->frame && !(ic->input_state & INPUT_EOF))
            weight_sum += FFABS(ic->weight);
    }

    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        if (ic->frame) {
            if (ic->scale_norm > weight_sum / FFABS(ic->weight)) {
                ic->scale_norm -= ((s->weight_sum / FFABS(ic->weight)) / s->nb_inputs) *
                                    ic->frame->nb_samples / (s->dropout_transition * s->sample_rate);
                ic->scale_norm = FFMAX(ic->scale_norm, weight_sum / FFABS(ic->weight));
            }
        }
    }

    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        if (ic->frame) {
            if (!s->normalize)
                ic->input_scale = ic->weight;
            else
                ic->input_scale = 1.0f / ic->scale_norm * FFSIGN(ic->weight);
        } else {
            ic->input_scale = 0.0f;
        }
    }
}

static void parse_weights(AVFilterContext *ctx)
{
    MixContext *s = ctx->priv;
    float last_weight = 1.f;
    int i;

    s->weight_sum = 0.f;
    for (i = 0; i < s->nb_weights; i++) {
        InputContext *ic;

        if (i >= s->nb_inputs)
            break;

        ic = &s->inputs[i];
        last_weight = s->weights_opt[i];
        ic->weight = last_weight;
        s->weight_sum += FFABS(last_weight);
    }

    for (; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        ic->weight = last_weight;
        s->weight_sum += FFABS(last_weight);
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MixContext *s      = ctx->priv;
    char buf[64];

    s->got_inputs      = 0;
    s->active_inputs   = s->nb_inputs;
    s->first_input     = -1;
    s->planar          = av_sample_fmt_is_planar(outlink->format);
    s->sample_rate     = outlink->sample_rate;
    outlink->time_base = (AVRational){ 1, outlink->sample_rate };

    s->inputs = av_calloc(s->nb_inputs, sizeof(*s->inputs));
    if (!s->inputs)
        return AVERROR(ENOMEM);

    parse_weights(ctx);

    s->nb_channels = outlink->ch_layout.nb_channels;

    for (int i = 0; i < s->nb_inputs; i++)
        s->inputs[i].scale_norm = s->weight_sum / FFABS(s->inputs[i].weight);
    calculate_scales(s);

    av_channel_layout_describe(&outlink->ch_layout, buf, sizeof(buf));

    av_log(ctx, AV_LOG_VERBOSE,
           "inputs:%d fmt:%s srate:%d cl:%s\n", s->nb_inputs,
           av_get_sample_fmt_name(outlink->format), outlink->sample_rate, buf);

    return 0;
}

static void free_frames(AVFilterContext *ctx)
{
    MixContext *s = ctx->priv;

    s->first_input = -1;
    s->got_inputs = 0;
    s->nb_samples = 0;
    for (int i = 0; i < s->nb_inputs; i++)
        av_frame_free(&s->inputs[i].frame);
}

/**
 * Read samples from the input, mix, and write to the output link.
 */
static int output_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MixContext      *s = ctx->priv;
    AVFrame *out;
    int nb_samples, got_pts = 0;

    switch (s->duration_mode) {
    case DURATION_FIRST:
        nb_samples = s->inputs[0].frame->nb_samples;
        break;
    case DURATION_SHORTEST:
        nb_samples = INT_MAX;
        for (int i = 0; i < s->nb_inputs; i++) {
            if (s->inputs[i].frame) {
                int ns = s->inputs[i].frame->nb_samples;
                nb_samples = FFMIN(nb_samples, ns);
            }
        }
        break;
    case DURATION_LONGEST:
        nb_samples = 0;
        for (int i = 0; i < s->nb_inputs; i++) {
            if (s->inputs[i].frame) {
                int ns = s->inputs[i].frame->nb_samples;
                nb_samples = FFMAX(nb_samples, ns);
            }
        }
        break;
    }

    calculate_scales(s);

    out = ff_get_audio_buffer(outlink, FFALIGN(nb_samples, 16));
    if (!out) {
        free_frames(ctx);
        return AVERROR(ENOMEM);
    }

    av_samples_set_silence(out->extended_data, 0, out->nb_samples,
                           out->ch_layout.nb_channels,
                           out->format);

    out->nb_samples = nb_samples;
    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        if (ic->frame) {
            int align, planes, plane_size, sample_size;

            sample_size= av_get_bytes_per_sample(out->format);
            align      = av_cpu_max_align();
            nb_samples = s->inputs[i].frame->nb_samples;
            planes     = s->planar ? s->nb_channels : 1;
            plane_size = nb_samples * (s->planar ? 1 : s->nb_channels);
            plane_size = FFALIGN(plane_size, align / sample_size);
            plane_size = plane_size & (~15);
            if (plane_size <= 0)
                continue;

            if (out->format == AV_SAMPLE_FMT_FLT ||
                out->format == AV_SAMPLE_FMT_FLTP) {
                for (int p = 0; p < planes; p++) {
                    s->fdsp->vector_fmac_scalar((float *)out->extended_data[p],
                                                (float *) ic->frame->extended_data[p],
                                                ic->input_scale, plane_size);
                }
            } else {
                for (int p = 0; p < planes; p++) {
                    s->fdsp->vector_dmac_scalar((double *)out->extended_data[p],
                                                (double *) ic->frame->extended_data[p],
                                                ic->input_scale, plane_size);
                }
            }

            if (!got_pts) {
                got_pts = 1;
                av_frame_copy_props(out, ic->frame);
            }
        }
    }

    free_frames(ctx);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    MixContext *s = ctx->priv;
    int status, ret;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        if (ic->input_state == INPUT_OFF)
            continue;

        if (ic->frame)
            continue;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &s->eof_pts)) {
            if (status == AVERROR_EOF)
                ic->input_state |= INPUT_EOF;
        }

        if (ff_inlink_queued_samples(ctx->inputs[i]) <= 0)
            ic->input_state |= INPUT_EMPTY;
        else
            ic->input_state &= ~INPUT_EMPTY;

        if (ic->input_state == INPUT_OFF) {
            s->active_inputs--;
            if (!s->active_inputs ||
                (!i && (s->duration_mode == DURATION_FIRST)) ||
                ((s->duration_mode == DURATION_SHORTEST) && s->active_inputs != s->nb_inputs)) {
                ff_outlink_set_status(outlink, AVERROR_EOF, s->eof_pts);
                return 0;
            }
            continue;
        }

        if (s->first_input == -1) {
            ret = ff_inlink_consume_frame(ctx->inputs[i], &ic->frame);
            if (ret < 0)
                return ret;
            if (ret > 0) {
                s->got_inputs++;
                s->nb_samples = ic->frame->nb_samples;
                s->first_input = i;
            }
        } else if (s->nb_samples > 0) {
            ret = ff_inlink_consume_samples(ctx->inputs[i], s->nb_samples, s->nb_samples, &ic->frame);
            if (ret < 0)
                return ret;
            if (ret > 0)
                s->got_inputs++;
        }

        if (!ic->frame && !(ic->input_state & INPUT_EOF)) {
            ff_inlink_request_frame(ctx->inputs[i]);
            return 0;
        }
    }

    if (!ff_outlink_frame_wanted(outlink))
        return 0;

    if ((s->nb_samples > 0) && (s->active_inputs == s->got_inputs))
        return output_frame(outlink);

    return FFERROR_NOT_READY;
}

static av_cold int init(AVFilterContext *ctx)
{
    MixContext *s = ctx->priv;
    int ret;

    for (int i = 0; i < s->nb_inputs; i++) {
        AVFilterPad pad = { 0 };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = av_asprintf("input%d", i);
        if (!pad.name)
            return AVERROR(ENOMEM);

        if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MixContext *s = ctx->priv;

    if (s->inputs) {
        for (int i = 0; i < s->nb_inputs; i++)
            av_frame_free(&s->inputs[i].frame);
        av_freep(&s->inputs);
    }
    av_freep(&s->fdsp);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    MixContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    parse_weights(ctx);
    for (int i = 0; i < s->nb_inputs; i++) {
        InputContext *ic = &s->inputs[i];

        ic->scale_norm = s->weight_sum / FFABS(ic->weight);
    }
    calculate_scales(s);

    return 0;
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_af_amix = {
    .p.name         = "amix",
    .p.description  = NULL_IF_CONFIG_SMALL("Audio streams mixing."),
    .p.priv_class   = &amix_class,
    .p.inputs       = NULL,
    .priv_size      = sizeof(MixContext),
    .init           = init,
    .uninit         = uninit,
    .activate       = activate,
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
                      AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags        = AVFILTER_FLAG_DYNAMIC_INPUTS,
};
