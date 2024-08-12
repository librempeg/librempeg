/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen, Vladimir Sadovnikov and others
 * Copyright (c) 2015 Paul B Mahol
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

#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "internal.h"

typedef struct CompensationDelayContext {
    const AVClass *class;
    int *distance_mm;
    unsigned distance_mm_size;
    int *distance_cm;
    unsigned distance_cm_size;
    int *distance_m;
    unsigned distance_m_size;
    double dry, wet;
    int *temp;
    unsigned temp_size;

    unsigned w_ptr;
    unsigned buf_size;
    AVFrame *delay_frame;
} CompensationDelayContext;

#define OFFSET(x) offsetof(CompensationDelayContext, x)
#define A (AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM)
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_mm = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_cm = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_m  = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_t  = {.def="20",.size_min=1,.sep=' '};

static const AVOption compensationdelay_options[] = {
    { "mm",   "set mm distance",    OFFSET(distance_mm), AV_OPT_TYPE_INT|AR, {.arr=&def_mm}, 0,  10, A },
    { "cm",   "set cm distance",    OFFSET(distance_cm), AV_OPT_TYPE_INT|AR, {.arr=&def_cm}, 0, 100, A },
    { "m",    "set meter distance", OFFSET(distance_m),  AV_OPT_TYPE_INT|AR, {.arr=&def_m},  0, 100, A },
    { "dry",  "set dry amount",     OFFSET(dry),         AV_OPT_TYPE_DOUBLE, {.dbl=0},       0,   1, A },
    { "wet",  "set wet amount",     OFFSET(wet),         AV_OPT_TYPE_DOUBLE, {.dbl=1},       0,   1, A },
    { "temp", "set temperature °C", OFFSET(temp),        AV_OPT_TYPE_INT|AR, {.arr=&def_t},-50,  50, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(compensationdelay);

// The maximum distance for options
#define COMP_DELAY_MAX_DISTANCE            (100.0 * 100.0 + 100.0 * 1.0 + 1.0)
// The actual speed of sound in normal conditions
#define COMP_DELAY_SOUND_SPEED_KM_H(temp)  1.85325 * (643.95 * sqrt(((temp + 273.15) / 273.15)))
#define COMP_DELAY_SOUND_SPEED_CM_S(temp)  (COMP_DELAY_SOUND_SPEED_KM_H(temp) * (1000.0 * 100.0) /* cm/km */ / (60.0 * 60.0) /* s/h */)
#define COMP_DELAY_SOUND_FRONT_DELAY(temp) (1.0 / COMP_DELAY_SOUND_SPEED_CM_S(temp))
// The maximum delay may be reached by this filter
#define COMP_DELAY_MAX_DELAY               (COMP_DELAY_MAX_DISTANCE * COMP_DELAY_SOUND_FRONT_DELAY(50))

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    CompensationDelayContext *s = ctx->priv;

    s->buf_size = 1 << av_ceil_log2(lrint(inlink->sample_rate * COMP_DELAY_MAX_DELAY));
    s->delay_frame = ff_get_audio_buffer(inlink, s->buf_size);
    if (!s->delay_frame)
        return AVERROR(ENOMEM);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    CompensationDelayContext *s = ctx->priv;
    const unsigned b_mask = s->buf_size - 1;
    const unsigned buf_size = s->buf_size;
    const double dry = s->dry;
    const double wet = s->wet;
    unsigned r_ptr, w_ptr = 0;
    AVFrame *out;

    if (ctx->is_disabled) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        const double *src = (const double *)in->extended_data[ch];
        double *dst = (double *)out->extended_data[ch];
        double *buffer = (double *)s->delay_frame->extended_data[ch];
        const int cm_idx = FFMIN(ch, s->distance_cm_size-1);
        const int mm_idx = FFMIN(ch, s->distance_mm_size-1);
        const int m_idx  = FFMIN(ch, s->distance_m_size-1);
        const int t_idx  = FFMIN(ch, s->temp_size-1);
        unsigned delay = (s->distance_m[m_idx] * 100. + s->distance_cm[cm_idx] * 1. + s->distance_mm[mm_idx] * .1) *
            COMP_DELAY_SOUND_FRONT_DELAY(s->temp[t_idx]) * outlink->sample_rate;

        w_ptr =  s->w_ptr;
        r_ptr = (w_ptr + buf_size - delay) & b_mask;

        for (int n = 0; n < in->nb_samples; n++) {
            const double sample = src[n];

            buffer[w_ptr] = sample;
            if (out != in)
                dst[n] = dry * sample + wet * buffer[r_ptr];
            w_ptr = (w_ptr + 1) & b_mask;
            r_ptr = (r_ptr + 1) & b_mask;
        }
    }
    s->w_ptr = w_ptr;

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CompensationDelayContext *s = ctx->priv;

    av_frame_free(&s->delay_frame);
}

static const AVFilterPad compensationdelay_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const AVFilter ff_af_compensationdelay = {
    .name          = "compensationdelay",
    .description   = NULL_IF_CONFIG_SMALL("Audio Compensation Delay Line."),
    .priv_size     = sizeof(CompensationDelayContext),
    .priv_class    = &compensationdelay_class,
    .uninit        = uninit,
    FILTER_INPUTS(compensationdelay_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
