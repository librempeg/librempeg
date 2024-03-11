/*
 * Copyright (c) 2012 Nicolas George
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/channel_layout.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "internal.h"

#define NOISE_FLOOR_DB_FLT -758
#define MAX_DB_FLT 770
#define MAX_DB 91
#define HISTOGRAM_SIZE 0x10000

typedef struct VolDetectContext {
    uint64_t* histogram; ///< for integer number of samples at each PCM value, for float number of samples at each dB
    uint64_t nb_samples; ///< number of samples
    double sum2;         ///< sum of the squares of the samples
    double max;          ///< maximum sample value
    int is_float;        ///< true if the input is in floating point
} VolDetectContext;

static inline double logdb(double v, enum AVSampleFormat sample_fmt)
{
    if (sample_fmt == AV_SAMPLE_FMT_FLT) {
        if (!v)
            return MAX_DB_FLT;
        return 10.0 * log10(v);
    } else {
        double d = v / (double)(0x8000 * 0x8000);
        if (!v)
            return MAX_DB;
        return -log10(d) * 10;
    }
}

static void update_float_stats(VolDetectContext *vd, float *audio_data)
{
    double max_sample;
    if(isnan(*audio_data) || isinf(*audio_data) || *audio_data == HUGE_VALF || fpclassify(*audio_data) == FP_SUBNORMAL)
        return;
    max_sample = fabsf(*audio_data);
    if (max_sample > vd->max)
        vd->max = max_sample;
    vd->sum2 += *audio_data * *audio_data;
    vd->histogram[lrintf(logdb(*audio_data * *audio_data, AV_SAMPLE_FMT_FLT)) + MAX_DB_FLT]++;
    vd->nb_samples++;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *samples)
{
    AVFilterContext *ctx = inlink->dst;
    VolDetectContext *vd = ctx->priv;
    int nb_channels = samples->ch_layout.nb_channels;
    int nb_planes   = nb_channels;
    int planar      = 0;
    int plane, i;

    planar = av_sample_fmt_is_planar(samples->format);
    if (!planar)
        nb_planes = 1;
    if (vd->is_float) {
        float *audio_data;
        for (plane = 0; plane < nb_planes; plane++) {
            audio_data = (float *)samples->extended_data[plane];
            for (i = 0; i < samples->nb_samples; i++) {
                if (planar) {
                    update_float_stats(vd, &audio_data[i]);
                } else {
                    for (int j = 0; j < nb_channels; j++)
                        update_float_stats(vd, &audio_data[i * nb_channels + j]);
                }
            }
        }
    } else {
        int16_t *pcm;
        for (plane = 0; plane < nb_planes; plane++) {
            pcm = (int16_t *)samples->extended_data[plane];
            for (i = 0; i < samples->nb_samples; i++) {
                if (planar) {
                    vd->histogram[pcm[i] + 0x8000]++;
                    vd->nb_samples++;
                } else {
                    for (int j = 0; j < nb_channels; j++) {
                        vd->histogram[pcm[i * nb_channels + j] + 0x8000]++;
                        vd->nb_samples++;
                    }
                }
            }
        }
    }
    return ff_filter_frame(inlink->dst->outputs[0], samples);
}

static void print_stats(AVFilterContext *ctx)
{
    VolDetectContext *vd = ctx->priv;

    if (!vd->nb_samples)
        return;
    if (vd->is_float) {
        int i, sum = 0;
        av_log(ctx, AV_LOG_INFO, "n_samples: %" PRId64 "\n", vd->nb_samples);
        av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", logdb(vd->sum2 / vd->nb_samples, AV_SAMPLE_FMT_FLT));
        av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", logdb(vd->max * vd->max, AV_SAMPLE_FMT_FLT));
        for (i = MAX_DB_FLT - NOISE_FLOOR_DB_FLT; i >= 0 && !vd->histogram[i]; i--);
        for (; i >= 0 && sum < vd->nb_samples / 1000; i--) {
            if (!vd->histogram[i])
                continue;
            av_log(ctx, AV_LOG_INFO, "histogram_%ddb: %" PRId64 "\n", MAX_DB_FLT - i, vd->histogram[i]);
            sum += vd->histogram[i];
        }
    } else {
        int i, max_volume, shift;
        uint64_t nb_samples = 0, power = 0, nb_samples_shift = 0, sum = 0;
        uint64_t histdb[MAX_DB + 1] = {0};
        for (i = 0; i < 0x10000; i++)
            nb_samples += vd->histogram[i];
        av_log(ctx, AV_LOG_INFO, "n_samples: %" PRId64 "\n", nb_samples);
        /*
            * If nb_samples > 1<<34, there is a risk of overflow in the
            * multiplication or the sum: shift all histogram values to avoid that.
            * The total number of samples must be recomputed to avoid rounding
            * errors.
        */
        shift = av_log2(nb_samples >> 33);
        for (i = 0; i < 0x10000; i++) {
            nb_samples_shift += vd->histogram[i] >> shift;
            power += (i - 0x8000) * (i - 0x8000) * (vd->histogram[i] >> shift);
        }
        if (!nb_samples_shift)
            return;
        power = (power + nb_samples_shift / 2) / nb_samples_shift;
        av_assert0(power <= 0x8000 * 0x8000);
        av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", -logdb((double)power, AV_SAMPLE_FMT_S16));
        max_volume = 0x8000;
        while (max_volume > 0 && !vd->histogram[0x8000 + max_volume] &&
                !vd->histogram[0x8000 - max_volume])
            max_volume--;
        av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", -logdb((double)(max_volume * max_volume), AV_SAMPLE_FMT_S16));
        for (i = 0; i < 0x10000; i++)
            histdb[(int)logdb((double)(i - 0x8000) * (i - 0x8000), AV_SAMPLE_FMT_S16)] += vd->histogram[i];
        for (i = 0; i <= MAX_DB && !histdb[i]; i++);
        for (; i <= MAX_DB && sum < nb_samples / 1000; i++) {
            av_log(ctx, AV_LOG_INFO, "histogram_%ddb: %" PRId64 "\n", i, histdb[i]);
            sum += histdb[i];
        }
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    VolDetectContext *vd = ctx->priv;

    vd->is_float = outlink->format == AV_SAMPLE_FMT_FLT ||
                   outlink->format == AV_SAMPLE_FMT_FLTP;

    if (!vd->is_float) {
        /*
        * Number of samples at each PCM value.
        * Only used for integer formats.
        * For 16 bit signed PCM there are 65536.
        * histogram[0x8000 + i] is the number of samples at value i.
        * The extra element is there for symmetry.
        */
        vd->histogram = av_calloc(HISTOGRAM_SIZE + 1, sizeof(uint64_t));
        if (!vd->histogram)
            return AVERROR(ENOMEM);
    } else {
        /*
        * The histogram is used to store the number of samples at each dB
        * instead of the number of samples at each PCM value.
        * The range of dB is from -758 to 770.
        */
        vd->histogram = av_calloc(MAX_DB_FLT - NOISE_FLOOR_DB_FLT + 1, sizeof(uint64_t));
        if (!vd->histogram)
            return AVERROR(ENOMEM);
    }
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VolDetectContext *vd = ctx->priv;
    print_stats(ctx);
    if (vd->histogram)
        av_freep(&vd->histogram);
}

static const AVFilterPad volumedetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad volumedetect_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const AVFilter ff_af_volumedetect = {
    .name          = "volumedetect",
    .description   = NULL_IF_CONFIG_SMALL("Detect audio volume."),
    .priv_size     = sizeof(VolDetectContext),
    .uninit        = uninit,
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(volumedetect_inputs),
    FILTER_OUTPUTS(volumedetect_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16,
                      AV_SAMPLE_FMT_S16P,
                      AV_SAMPLE_FMT_FLT,
                      AV_SAMPLE_FMT_FLTP),
};
