/*
 * Dynamic Audio Normalizer
 * Copyright (c) 2015 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
 *
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

/**
 * @file
 * Dynamic Audio Normalizer
 */

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#define MIN_FILTER_SIZE 3
#define MAX_FILTER_SIZE 301

#define FF_BUFQUEUE_SIZE (MAX_FILTER_SIZE + 1)
#include "libavfilter/bufferqueue.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

static const char * const var_names[] = {
    "ch",           ///< the value of the current channel
    "sn",           ///< number of samples
    "nb_channels",
    "t",            ///< timestamp expressed in seconds
    "sr",           ///< sample rate
    "p",            ///< peak value
    NULL
};

enum var_name {
    VAR_CH,
    VAR_SN,
    VAR_NB_CHANNELS,
    VAR_T,
    VAR_SR,
    VAR_P,
    VAR_VARS_NB
};

typedef struct local_gain {
    double max_gain;
    double threshold;
} local_gain;

typedef struct cqueue {
    double *elements;
    int size;
    int max_size;
    int nb_elements;
} cqueue;

typedef struct DynamicAudioNormalizerContext {
    const AVClass *class;

    struct FFBufQueue queue;

    int frame_len;
    int frame_len_msec;
    int filter_size;
    int dc_correction;
    int channels_coupled;
    int alt_boundary_mode;
    double overlap;
    char *expr_str;

    double peak_value;
    double max_amplification;
    double target_rms;
    double compress_factor;
    double threshold;
    double *prev_amplification_factor;
    double *dc_correction_value;
    double *weights;

    int channels;
    int sample_advance;
    int eof;
    AVChannelLayout ch_layout;
    int64_t pts;

    cqueue **gain_history_original;
    cqueue **gain_history_minimum;
    cqueue **gain_history_smoothed;
    cqueue **threshold_history;

    cqueue *is_enabled;

    AVFrame *window;

    AVExpr *expr;
    double var_values[VAR_VARS_NB];
} DynamicAudioNormalizerContext;

typedef struct ThreadData {
    AVFrame *in, *out;
    int enabled;
} ThreadData;

#define OFFSET(x) offsetof(DynamicAudioNormalizerContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption dynaudnorm_options[] = {
    { "framelen",    "set the frame length in msec",     OFFSET(frame_len_msec),    AV_OPT_TYPE_INT,    {.i64 = 500},   10,  8000, FLAGS },
    { "f",           "set the frame length in msec",     OFFSET(frame_len_msec),    AV_OPT_TYPE_INT,    {.i64 = 500},   10,  8000, FLAGS },
    { "gausssize",   "set the filter size",              OFFSET(filter_size),       AV_OPT_TYPE_INT,    {.i64 = 31},     3,   301, FLAGS },
    { "g",           "set the filter size",              OFFSET(filter_size),       AV_OPT_TYPE_INT,    {.i64 = 31},     3,   301, FLAGS },
    { "peak",        "set the peak value",               OFFSET(peak_value),        AV_OPT_TYPE_DOUBLE, {.dbl = 0.95}, 0.0,   1.0, FLAGS },
    { "p",           "set the peak value",               OFFSET(peak_value),        AV_OPT_TYPE_DOUBLE, {.dbl = 0.95}, 0.0,   1.0, FLAGS },
    { "maxgain",     "set the max amplification",        OFFSET(max_amplification), AV_OPT_TYPE_DOUBLE, {.dbl = 10.0}, 1.0, 100.0, FLAGS },
    { "m",           "set the max amplification",        OFFSET(max_amplification), AV_OPT_TYPE_DOUBLE, {.dbl = 10.0}, 1.0, 100.0, FLAGS },
    { "targetrms",   "set the target RMS",               OFFSET(target_rms),        AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,   1.0, FLAGS },
    { "r",           "set the target RMS",               OFFSET(target_rms),        AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,   1.0, FLAGS },
    { "coupling",    "set channel coupling",             OFFSET(channels_coupled),  AV_OPT_TYPE_BOOL,   {.i64 = 1},      0,     1, FLAGS },
    { "n",           "set channel coupling",             OFFSET(channels_coupled),  AV_OPT_TYPE_BOOL,   {.i64 = 1},      0,     1, FLAGS },
    { "correctdc",   "set DC correction",                OFFSET(dc_correction),     AV_OPT_TYPE_BOOL,   {.i64 = 0},      0,     1, FLAGS },
    { "c",           "set DC correction",                OFFSET(dc_correction),     AV_OPT_TYPE_BOOL,   {.i64 = 0},      0,     1, FLAGS },
    { "altboundary", "set alternative boundary mode",    OFFSET(alt_boundary_mode), AV_OPT_TYPE_BOOL,   {.i64 = 0},      0,     1, FLAGS },
    { "b",           "set alternative boundary mode",    OFFSET(alt_boundary_mode), AV_OPT_TYPE_BOOL,   {.i64 = 0},      0,     1, FLAGS },
    { "compress",    "set the compress factor",          OFFSET(compress_factor),   AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,  30.0, FLAGS },
    { "s",           "set the compress factor",          OFFSET(compress_factor),   AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,  30.0, FLAGS },
    { "threshold",   "set the threshold value",          OFFSET(threshold),         AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,   1.0, FLAGS },
    { "t",           "set the threshold value",          OFFSET(threshold),         AV_OPT_TYPE_DOUBLE, {.dbl = 0.0},  0.0,   1.0, FLAGS },
    { "channels",    "set channels to filter",           OFFSET(ch_layout),         AV_OPT_TYPE_CHLAYOUT,{.str="24c"},   0,     0, FLAGS },
    { "h",           "set channels to filter",           OFFSET(ch_layout),         AV_OPT_TYPE_CHLAYOUT,{.str="24c"},   0,     0, FLAGS },
    { "overlap",     "set the frame overlap",            OFFSET(overlap),           AV_OPT_TYPE_DOUBLE, {.dbl=.0},     0.0,   1.0, FLAGS },
    { "o",           "set the frame overlap",            OFFSET(overlap),           AV_OPT_TYPE_DOUBLE, {.dbl=.0},     0.0,   1.0, FLAGS },
    { "curve",       "set the custom peak mapping curve",OFFSET(expr_str),          AV_OPT_TYPE_STRING, {.str=NULL},      .flags = FLAGS },
    { "v",           "set the custom peak mapping curve",OFFSET(expr_str),          AV_OPT_TYPE_STRING, {.str=NULL},      .flags = FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(dynaudnorm);

static av_cold int init(AVFilterContext *ctx)
{
    DynamicAudioNormalizerContext *s = ctx->priv;

    if (!(s->filter_size & 1)) {
        av_log(ctx, AV_LOG_WARNING, "filter size %d is invalid. Changing to an odd value.\n", s->filter_size);
        s->filter_size |= 1;
    }

    return 0;
}

static inline int frame_size(int sample_rate, int frame_len_msec)
{
    const int frame_size = lrint((double)sample_rate * (frame_len_msec / 1000.0));
    return frame_size + (frame_size % 2);
}

static cqueue *cqueue_create(int size, int max_size)
{
    cqueue *q;

    if (max_size < size)
        return NULL;

    q = av_malloc(sizeof(cqueue));
    if (!q)
        return NULL;

    q->max_size = max_size;
    q->size = size;
    q->nb_elements = 0;

    q->elements = av_malloc_array(max_size, sizeof(double));
    if (!q->elements) {
        av_free(q);
        return NULL;
    }

    return q;
}

static void cqueue_free(cqueue *q)
{
    if (q)
        av_free(q->elements);
    av_free(q);
}

static int cqueue_size(cqueue *q)
{
    return q->nb_elements;
}

static int cqueue_empty(cqueue *q)
{
    return q->nb_elements <= 0;
}

static int cqueue_enqueue(cqueue *q, double element)
{
    av_assert2(q->nb_elements < q->max_size);

    q->elements[q->nb_elements] = element;
    q->nb_elements++;

    return 0;
}

static double cqueue_peek(cqueue *q, int index)
{
    av_assert2(index < q->nb_elements);
    return q->elements[index];
}

static int cqueue_dequeue(cqueue *q, double *element)
{
    av_assert2(!cqueue_empty(q));

    *element = q->elements[0];
    memmove(&q->elements[0], &q->elements[1], (q->nb_elements - 1) * sizeof(double));
    q->nb_elements--;

    return 0;
}

static int cqueue_pop(cqueue *q)
{
    av_assert2(!cqueue_empty(q));

    memmove(&q->elements[0], &q->elements[1], (q->nb_elements - 1) * sizeof(double));
    q->nb_elements--;

    return 0;
}

static void cqueue_resize(cqueue *q, int new_size)
{
    av_assert2(q->max_size >= new_size);
    av_assert2(MIN_FILTER_SIZE <= new_size);

    if (new_size > q->nb_elements) {
        const int side = (new_size - q->nb_elements) / 2;

        memmove(q->elements + side, q->elements, sizeof(double) * q->nb_elements);
        for (int i = 0; i < side; i++)
            q->elements[i] = q->elements[side];
        q->nb_elements = new_size - 1 - side;
    } else {
        int count = (q->size - new_size + 1) / 2;

        while (count-- > 0)
            cqueue_pop(q);
    }

    q->size = new_size;
}

static void init_gaussian_filter(DynamicAudioNormalizerContext *s)
{
    double total_weight = 0.0;
    const double sigma = (((s->filter_size / 2.0) - 1.0) / 3.0) + (1.0 / 3.0);
    double adjust;

    // Pre-compute constants
    const int offset = s->filter_size / 2;
    const double c1 = 1.0 / (sigma * sqrt(2.0 * M_PI));
    const double c2 = 2.0 * sigma * sigma;

    // Compute weights
    for (int i = 0; i < s->filter_size; i++) {
        const int x = i - offset;

        s->weights[i] = c1 * exp(-x * x / c2);
        total_weight += s->weights[i];
    }

    // Adjust weights
    adjust = 1.0 / total_weight;
    for (int i = 0; i < s->filter_size; i++) {
        s->weights[i] *= adjust;
    }
}

static av_cold void uninit(AVFilterContext *ctx)
{
    DynamicAudioNormalizerContext *s = ctx->priv;

    av_freep(&s->prev_amplification_factor);
    av_freep(&s->dc_correction_value);

    for (int c = 0; c < s->channels; c++) {
        if (s->gain_history_original)
            cqueue_free(s->gain_history_original[c]);
        if (s->gain_history_minimum)
            cqueue_free(s->gain_history_minimum[c]);
        if (s->gain_history_smoothed)
            cqueue_free(s->gain_history_smoothed[c]);
        if (s->threshold_history)
            cqueue_free(s->threshold_history[c]);
    }

    av_freep(&s->gain_history_original);
    av_freep(&s->gain_history_minimum);
    av_freep(&s->gain_history_smoothed);
    av_freep(&s->threshold_history);

    cqueue_free(s->is_enabled);
    s->is_enabled = NULL;

    av_freep(&s->weights);

    ff_bufqueue_discard_all(&s->queue);

    av_frame_free(&s->window);
    av_expr_free(s->expr);
    s->expr = NULL;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    DynamicAudioNormalizerContext *s = ctx->priv;
    int ret = 0;

    uninit(ctx);

    s->channels = inlink->ch_layout.nb_channels;
    s->frame_len = frame_size(inlink->sample_rate, s->frame_len_msec);
    av_log(ctx, AV_LOG_DEBUG, "frame len %d\n", s->frame_len);

    s->prev_amplification_factor = av_malloc_array(inlink->ch_layout.nb_channels, sizeof(*s->prev_amplification_factor));
    s->dc_correction_value = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->dc_correction_value));
    s->gain_history_original = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->gain_history_original));
    s->gain_history_minimum = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->gain_history_minimum));
    s->gain_history_smoothed = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->gain_history_smoothed));
    s->threshold_history = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->threshold_history));
    s->weights = av_malloc_array(MAX_FILTER_SIZE, sizeof(*s->weights));
    s->is_enabled = cqueue_create(s->filter_size, MAX_FILTER_SIZE);
    if (!s->prev_amplification_factor || !s->dc_correction_value ||
        !s->gain_history_original || !s->gain_history_minimum ||
        !s->gain_history_smoothed || !s->threshold_history ||
        !s->is_enabled || !s->weights)
        return AVERROR(ENOMEM);

    for (int c = 0; c < inlink->ch_layout.nb_channels; c++) {
        s->prev_amplification_factor[c] = 1.0;

        s->gain_history_original[c] = cqueue_create(s->filter_size, MAX_FILTER_SIZE);
        s->gain_history_minimum[c]  = cqueue_create(s->filter_size, MAX_FILTER_SIZE);
        s->gain_history_smoothed[c] = cqueue_create(s->filter_size, MAX_FILTER_SIZE);
        s->threshold_history[c]     = cqueue_create(s->filter_size, MAX_FILTER_SIZE);

        if (!s->gain_history_original[c] || !s->gain_history_minimum[c] ||
            !s->gain_history_smoothed[c] || !s->threshold_history[c])
            return AVERROR(ENOMEM);
    }

    init_gaussian_filter(s);

    s->window = ff_get_audio_buffer(ctx->outputs[0], s->frame_len * 2);
    if (!s->window)
        return AVERROR(ENOMEM);
    s->sample_advance = FFMAX(1, lrint(s->frame_len * (1. - s->overlap)));

    s->var_values[VAR_SR] = inlink->sample_rate;
    s->var_values[VAR_NB_CHANNELS] = s->channels;

    if (s->expr_str)
        ret = av_expr_parse(&s->expr, s->expr_str, var_names, NULL, NULL,
                            NULL, NULL, 0, ctx);
    return ret;
}

static inline double fade(double prev, double next, int pos, int length)
{
    const double step_size = 1.0 / length;
    const double f0 = 1.0 - (step_size * (pos + 1.0));
    const double f1 = 1.0 - f0;
    return f0 * prev + f1 * next;
}

static av_always_inline double pow_2(const double value)
{
    return value * value;
}

static inline double bound(const double threshold, const double val)
{
    const double CONST = 0.8862269254527580136490837416705725913987747280611935; //sqrt(PI) / 2.0
    return erf(CONST * (val / threshold)) * threshold;
}

static double find_peak_magnitude(AVFrame *frame, int channel)
{
    const int nb_samples = frame->nb_samples;
    double max = DBL_EPSILON;

    if (channel == -1) {
        for (int c = 0; c < frame->ch_layout.nb_channels; c++) {
            double *data_ptr = (double *)frame->extended_data[c];

            for (int i = 0; i < nb_samples; i++)
                max = fmax(max, fabs(data_ptr[i]));
        }
    } else {
        double *data_ptr = (double *)frame->extended_data[channel];

        for (int i = 0; i < nb_samples; i++)
            max = fmax(max, fabs(data_ptr[i]));
    }

    return max;
}

static double compute_frame_rms(AVFrame *frame, int channel)
{
    const int nb_samples = frame->nb_samples;
    double rms_value = 0.0;

    if (channel == -1) {
        for (int c = 0; c < frame->ch_layout.nb_channels; c++) {
            const double *data_ptr = (double *)frame->extended_data[c];

            for (int i = 0; i < nb_samples; i++) {
                rms_value += pow_2(data_ptr[i]);
            }
        }

        rms_value /= nb_samples * frame->ch_layout.nb_channels;
    } else {
        const double *data_ptr = (double *)frame->extended_data[channel];

        for (int i = 0; i < nb_samples; i++) {
            rms_value += pow_2(data_ptr[i]);
        }

        rms_value /= nb_samples;
    }

    return fmax(sqrt(rms_value), DBL_EPSILON);
}

static local_gain get_max_local_gain(DynamicAudioNormalizerContext *s, AVFrame *frame,
                                     int channel)
{
    const double peak_magnitude = find_peak_magnitude(frame, channel);
    const double maximum_gain = s->peak_value / peak_magnitude;
    const double rms_gain = s->target_rms > DBL_EPSILON ? (s->target_rms / compute_frame_rms(frame, channel)) : DBL_MAX;
    double target_gain = DBL_MAX;
    local_gain gain;

    if (s->expr_str) {
        double var_values[VAR_VARS_NB];

        memcpy(var_values, s->var_values, sizeof(var_values));

        var_values[VAR_CH] = channel;
        var_values[VAR_P]  = peak_magnitude;

        target_gain = av_expr_eval(s->expr, var_values, s) / peak_magnitude;
    }

    gain.threshold = peak_magnitude > s->threshold;
    gain.max_gain  = bound(s->max_amplification, fmin(target_gain, fmin(maximum_gain, rms_gain)));

    return gain;
}

static double minimum_filter(cqueue *q)
{
    double min = DBL_MAX;

    for (int i = 0; i < cqueue_size(q); i++) {
        min = fmin(min, cqueue_peek(q, i));
    }

    return min;
}

static double gaussian_filter(DynamicAudioNormalizerContext *s, cqueue *q, cqueue *tq)
{
    const double *weights = s->weights;
    double result = 0.0, tsum = 0.0;

    for (int i = 0; i < cqueue_size(q); i++) {
        double tq_item = cqueue_peek(tq, i);
        double q_item = cqueue_peek(q, i);

        tsum   += tq_item * weights[i];
        result += tq_item * weights[i] * q_item;
    }

    if (tsum == 0.0)
        result = 1.0;

    return result;
}

static void update_gain_history(DynamicAudioNormalizerContext *s, int channel,
                                local_gain gain)
{
    if (cqueue_empty(s->gain_history_original[channel])) {
        const int pre_fill_size = s->filter_size / 2;
        const double initial_value = s->alt_boundary_mode ? gain.max_gain : fmin(1.0, gain.max_gain);

        s->prev_amplification_factor[channel] = initial_value;

        while (cqueue_size(s->gain_history_original[channel]) < pre_fill_size) {
            cqueue_enqueue(s->gain_history_original[channel], initial_value);
            cqueue_enqueue(s->threshold_history[channel], gain.threshold);
        }
    }

    cqueue_enqueue(s->gain_history_original[channel], gain.max_gain);

    while (cqueue_size(s->gain_history_original[channel]) >= s->filter_size) {
        double minimum;

        if (cqueue_empty(s->gain_history_minimum[channel])) {
            const int pre_fill_size = s->filter_size / 2;
            double initial_value = s->alt_boundary_mode ? cqueue_peek(s->gain_history_original[channel], 0) : 1.0;
            int input = pre_fill_size;

            while (cqueue_size(s->gain_history_minimum[channel]) < pre_fill_size) {
                input++;
                initial_value = fmin(initial_value, cqueue_peek(s->gain_history_original[channel], input));
                cqueue_enqueue(s->gain_history_minimum[channel], initial_value);
            }
        }

        minimum = minimum_filter(s->gain_history_original[channel]);

        cqueue_enqueue(s->gain_history_minimum[channel], minimum);

        cqueue_enqueue(s->threshold_history[channel], gain.threshold);

        cqueue_pop(s->gain_history_original[channel]);
    }

    while (cqueue_size(s->gain_history_minimum[channel]) >= s->filter_size) {
        double smoothed, limit;

        smoothed = gaussian_filter(s, s->gain_history_minimum[channel], s->threshold_history[channel]);
        limit    = cqueue_peek(s->gain_history_original[channel], 0);
        smoothed = fmin(smoothed, limit);

        cqueue_enqueue(s->gain_history_smoothed[channel], smoothed);

        cqueue_pop(s->gain_history_minimum[channel]);
        cqueue_pop(s->threshold_history[channel]);
    }
}

static int update_gain_histories(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    DynamicAudioNormalizerContext *s = ctx->priv;
    AVFrame *analyze_frame = arg;
    const int channels = s->channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;

    for (int c = start; c < end; c++)
        update_gain_history(s, c, get_max_local_gain(s, analyze_frame, c));

    return 0;
}

static inline double update_value(double new, double old, double aggressiveness)
{
    av_assert2((aggressiveness >= 0.0) && (aggressiveness <= 1.0));
    return aggressiveness * new + (1.0 - aggressiveness) * old;
}

static inline int bypass_channel(DynamicAudioNormalizerContext *s, AVFrame *frame, int ch)
{
    enum AVChannel channel = av_channel_layout_channel_from_index(&frame->ch_layout, ch);

    return av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
}

static void perform_dc_correction(DynamicAudioNormalizerContext *s, AVFrame *frame)
{
    const int nb_samples = frame->nb_samples;
    const double diff = 1.0 / nb_samples;
    int is_first_frame = cqueue_empty(s->gain_history_original[0]);

    for (int c = 0; c < s->channels; c++) {
        const int bypass = bypass_channel(s, frame, c);
        double *dst_ptr = (double *)frame->extended_data[c];
        double current_average_value = 0.0;
        double prev_value;

        for (int i = 0; i < nb_samples; i++)
            current_average_value += dst_ptr[i] * diff;

        prev_value = is_first_frame ? current_average_value : s->dc_correction_value[c];
        s->dc_correction_value[c] = is_first_frame ? current_average_value : update_value(current_average_value, s->dc_correction_value[c], 0.1);

        for (int i = 0; i < nb_samples && !bypass; i++) {
            dst_ptr[i] -= fade(prev_value, s->dc_correction_value[c], i, nb_samples);
        }
    }
}

static void perform_compression(DynamicAudioNormalizerContext *s, AVFrame *frame)
{
    const int nb_samples = frame->nb_samples;
    const double factor = 1.0 + (30.0 - s->compress_factor) / 20.0;
    const double den = log(1.0 + factor);

    if (s->channels_coupled) {
        for (int c = 0; c < s->channels; c++) {
            double *const dst_ptr = (double *)frame->extended_data[c];
            const int bypass = bypass_channel(s, frame, c);

            if (bypass)
                continue;

            for (int i = 0; i < nb_samples; i++)
                dst_ptr[i] = copysign(log(1.0 + fabs(dst_ptr[i]) * factor) / den, dst_ptr[i]);
        }
    } else {
        for (int c = 0; c < s->channels; c++) {
            const int bypass = bypass_channel(s, frame, c);
            double *dst_ptr = (double *)frame->extended_data[c];

            for (int i = 0; i < nb_samples && !bypass; i++)
                dst_ptr[i] = copysign(log(1.0 + fabs(dst_ptr[i]) * factor) / den, dst_ptr[i]);
        }
    }
}

static int analyze_frame(AVFilterContext *ctx, AVFilterLink *outlink, AVFrame **frame)
{
    FilterLink *outl = ff_filter_link(outlink);
    DynamicAudioNormalizerContext *s = ctx->priv;
    AVFrame *analyze_frame;

    if (s->dc_correction || s->compress_factor > DBL_EPSILON) {
        int ret;

        if (!av_frame_is_writable(*frame)) {
            AVFrame *out = ff_get_audio_buffer(outlink, (*frame)->nb_samples);

            if (!out) {
                av_frame_free(frame);
                return AVERROR(ENOMEM);
            }
            ret = av_frame_copy_props(out, *frame);
            if (ret < 0) {
                av_frame_free(frame);
                av_frame_free(&out);
                return ret;
            }
            ret = av_frame_copy(out, *frame);
            if (ret < 0) {
                av_frame_free(frame);
                av_frame_free(&out);
                return ret;
            }

            av_frame_free(frame);
            *frame = out;
        }
    }

    if (s->dc_correction)
        perform_dc_correction(s, *frame);

    if (s->compress_factor > DBL_EPSILON)
        perform_compression(s, *frame);

    if (s->frame_len != s->sample_advance) {
        const int offset = s->frame_len - s->sample_advance;

        for (int c = 0; c < s->channels; c++) {
            double *src = (double *)s->window->extended_data[c];

            memmove(src, &src[s->sample_advance], offset * sizeof(double));
            memcpy(&src[offset], (*frame)->extended_data[c], (*frame)->nb_samples * sizeof(double));
            memset(&src[offset + (*frame)->nb_samples], 0, (s->sample_advance - (*frame)->nb_samples) * sizeof(double));
        }

        analyze_frame = s->window;
    } else {
        av_samples_copy(s->window->extended_data, (*frame)->extended_data, 0, 0,
                        FFMIN(s->frame_len, (*frame)->nb_samples), (*frame)->ch_layout.nb_channels, (*frame)->format);
        analyze_frame = *frame;
    }

    s->var_values[VAR_SN] = outl->sample_count_in;
    s->var_values[VAR_T] = s->var_values[VAR_SN] * (double)1/outlink->sample_rate;

    if (s->channels_coupled) {
        const local_gain gain = get_max_local_gain(s, analyze_frame, -1);
        for (int c = 0; c < s->channels; c++)
            update_gain_history(s, c, gain);
    } else {
        ff_filter_execute(ctx, update_gain_histories, analyze_frame, NULL,
                          FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));
    }

    return 0;
}

static void amplify_channel(DynamicAudioNormalizerContext *s, AVFrame *in,
                            AVFrame *frame, int enabled, int c)
{
    double prev_amplification_factor = s->prev_amplification_factor[c];
    const int bypass = bypass_channel(s, frame, c);
    const double *src_ptr = (const double *)in->extended_data[c];
    double *dst_ptr = (double *)frame->extended_data[c];
    const int nb_samples = frame->nb_samples;
    double current_amplification_factor;

    cqueue_dequeue(s->gain_history_smoothed[c], &current_amplification_factor);

    for (int i = 0; i < nb_samples && enabled && !bypass; i++) {
        const double amplification_factor = fade(prev_amplification_factor,
                                                 current_amplification_factor, i,
                                                 nb_samples);

        dst_ptr[i] = src_ptr[i] * amplification_factor;
    }

    s->prev_amplification_factor[c] = current_amplification_factor;
}

static int amplify_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    DynamicAudioNormalizerContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int enabled = td->enabled;
    const int channels = s->channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        amplify_channel(s, in, out, enabled, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    DynamicAudioNormalizerContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    int ret;

    while (((s->queue.available >= s->filter_size) ||
            (s->eof && s->queue.available)) &&
           !cqueue_empty(s->gain_history_smoothed[0])) {
        AVFrame *in = ff_bufqueue_get(&s->queue);
        AVFrame *out;
        double is_enabled;

        cqueue_dequeue(s->is_enabled, &is_enabled);

        if (av_frame_is_writable(in)) {
            out = in;
        } else {
            out = ff_get_audio_buffer(outlink, in->nb_samples);
            if (!out) {
                av_frame_free(&in);
                return AVERROR(ENOMEM);
            }
            av_frame_copy_props(out, in);
        }

        td.in = in;
        td.out = out;
        td.enabled = is_enabled > 0.;
        ff_filter_execute(ctx, amplify_channels, &td, NULL,
                          FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));

        s->pts = out->pts + av_rescale_q(out->nb_samples, av_make_q(1, outlink->sample_rate),
                                         outlink->time_base);
        if (out != in)
            av_frame_free(&in);
        ret = ff_filter_frame(outlink, out);
        if (ret < 0)
            return ret;
    }

    ret = analyze_frame(ctx, outlink, &in);
    if (ret < 0)
        return ret;
    if (!s->eof) {
        ff_bufqueue_add(ctx, &s->queue, in);
        cqueue_enqueue(s->is_enabled, !ff_filter_disabled(ctx));
    } else {
        av_frame_free(&in);
    }

    return 1;
}

static int flush_buffer(DynamicAudioNormalizerContext *s, AVFilterLink *inlink,
                        AVFilterLink *outlink)
{
    AVFrame *out = ff_get_audio_buffer(outlink, s->sample_advance);

    if (!out)
        return AVERROR(ENOMEM);

    for (int c = 0; c < s->channels; c++) {
        const double *dc_correction_value = s->dc_correction_value;
        double *dst_ptr = (double *)out->extended_data[c];
        const int alt_boundary_mode = s->alt_boundary_mode;
        const int nb_samples = out->nb_samples;
        const int dc_correction = s->dc_correction;

        for (int i = 0; i < nb_samples; i++) {
            dst_ptr[i] = alt_boundary_mode ? DBL_EPSILON : ((s->target_rms > DBL_EPSILON) ? fmin(s->peak_value, s->target_rms) : s->peak_value);
            if (dc_correction) {
                dst_ptr[i] *= (i & 1) ? -1 : 1;
                dst_ptr[i] += dc_correction_value[c];
            }
        }
    }

    return filter_frame(inlink, out);
}

static int flush(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    DynamicAudioNormalizerContext *s = ctx->priv;

    while (s->eof && cqueue_empty(s->gain_history_smoothed[0])) {
        for (int c = 0; c < s->channels; c++)
            update_gain_history(s, c, (local_gain){ cqueue_peek(s->gain_history_original[c], 0), 1.0});
    }

    return flush_buffer(s, inlink, outlink);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    DynamicAudioNormalizerContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret = 0, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (!s->eof) {
        ret = ff_inlink_consume_samples(inlink, s->sample_advance, s->sample_advance, &in);
        if (ret < 0)
            return ret;
        if (ret > 0) {
            ret = filter_frame(inlink, in);
            if (ret <= 0)
                return ret;
        }

        if (ff_inlink_check_available_samples(inlink, s->sample_advance) > 0) {
            ff_filter_set_ready(ctx, 10);
            return 0;
        }
    }

    if (!s->eof && ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF)
            s->eof = 1;
    }

    if (s->eof && s->queue.available)
        return flush(outlink);

    if (s->eof && !s->queue.available) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (!s->eof)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    DynamicAudioNormalizerContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    int prev_filter_size = s->filter_size;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    s->filter_size |= 1;
    if (prev_filter_size != s->filter_size) {
        init_gaussian_filter(s);

        for (int c = 0; c < s->channels; c++) {
            cqueue_resize(s->gain_history_original[c], s->filter_size);
            cqueue_resize(s->gain_history_minimum[c], s->filter_size);
            cqueue_resize(s->threshold_history[c], s->filter_size);
        }
    }

    s->frame_len = frame_size(inlink->sample_rate, s->frame_len_msec);
    s->sample_advance = FFMAX(1, lrint(s->frame_len * (1. - s->overlap)));
    if (s->expr_str) {
        ret = av_expr_parse(&s->expr, s->expr_str, var_names, NULL, NULL,
                            NULL, NULL, 0, ctx);
        if (ret < 0)
            return ret;
    }
    return 0;
}

static const AVFilterPad dynaudnorm_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .config_props   = config_input,
    },
};

const FFFilter ff_af_dynaudnorm = {
    .p.name        = "dynaudnorm",
    .p.description = NULL_IF_CONFIG_SMALL("Dynamic Audio Normalizer."),
    .p.priv_class  = &dynaudnorm_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(DynamicAudioNormalizerContext),
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(dynaudnorm_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
};
