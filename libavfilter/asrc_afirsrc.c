/*
 * Copyright (c) 2020 Paul B Mahol
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

#include <float.h>

#include "libavutil/cpu.h"
#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/eval.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "window_func.h"

typedef struct AudioFIRSourceContext {
    const AVClass *class;

    float *freq_points_opt;
    unsigned nb_freq;
    float *magnitude_opt;
    unsigned nb_magnitude;
    float *phase_opt;
    unsigned nb_phase;
    int nb_taps;
    int sample_rate;
    int nb_samples;
    int win_func;
    int preset;
    int interp;
    int phaset;

    AVComplexFloat *complexf;
    float *freq;
    float *magnitude;
    int freq_size;
    int magnitude_size;
    int phase_size;

    float *taps;
    float *win;
    int64_t pts;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} AudioFIRSourceContext;

#define OFFSET(x) offsetof(AudioFIRSourceContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_freqs = {.def="0 1",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_mags  = {.def="1 1",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_phase = {.def="0 0",.size_min=2,.sep=' '};

static const AVOption afirsrc_options[] = {
    { "taps",      "set number of taps",   OFFSET(nb_taps),         AV_OPT_TYPE_INT,    {.i64=1025}, 9, UINT16_MAX, FLAGS },
    { "t",         "set number of taps",   OFFSET(nb_taps),         AV_OPT_TYPE_INT,    {.i64=1025}, 9, UINT16_MAX, FLAGS },
    { "frequency", "set frequency points", OFFSET(freq_points_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_freqs}, 0, 1, FLAGS },
    { "f",         "set frequency points", OFFSET(freq_points_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_freqs}, 0, 1, FLAGS },
    { "magnitude", "set magnitude values", OFFSET(magnitude_opt),   AV_OPT_TYPE_FLOAT|AR, {.arr=&def_mags},  0, 1, FLAGS },
    { "m",         "set magnitude values", OFFSET(magnitude_opt),   AV_OPT_TYPE_FLOAT|AR, {.arr=&def_mags},  0, 1, FLAGS },
    { "phase",     "set phase values",     OFFSET(phase_opt),       AV_OPT_TYPE_FLOAT|AR, {.arr=&def_phase},-1, 1, FLAGS },
    { "p",         "set phase values",     OFFSET(phase_opt),       AV_OPT_TYPE_FLOAT|AR, {.arr=&def_phase},-1, 1, FLAGS },
    { "sample_rate", "set sample rate",    OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100},  1, INT_MAX,    FLAGS },
    { "r",           "set sample rate",    OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100},  1, INT_MAX,    FLAGS },
    { "nb_samples", "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "n",          "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    WIN_FUNC_OPTION("win_func", OFFSET(win_func), FLAGS, WFUNC_BLACKMAN),
    WIN_FUNC_OPTION("w",        OFFSET(win_func), FLAGS, WFUNC_BLACKMAN),
    {NULL}
};

AVFILTER_DEFINE_CLASS(afirsrc);

static av_cold int init(AVFilterContext *ctx)
{
    AudioFIRSourceContext *s = ctx->priv;

    if (!(s->nb_taps & 1)) {
        av_log(ctx, AV_LOG_WARNING, "Number of taps %d must be odd length.\n", s->nb_taps);
        s->nb_taps |= 1;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioFIRSourceContext *s = ctx->priv;

    av_freep(&s->win);
    av_freep(&s->taps);
    av_freep(&s->freq);
    av_freep(&s->magnitude);
    av_freep(&s->complexf);
    av_tx_uninit(&s->tx_ctx);
    av_tx_uninit(&s->itx_ctx);
}

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AudioFIRSourceContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLT,
        AV_SAMPLE_FMT_NONE
    };
    int ret = ff_set_sample_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static void lininterp(AVComplexFloat *complexf,
                      const float *freq,
                      const float *magnitude,
                      const float *phase,
                      int m, int minterp)
{
    for (int i = 0; i < minterp; i++) {
        for (int j = 1; j < m; j++) {
            const float x = i / (float)minterp;

            if (x <= freq[j]) {
                const float mg = (x - freq[j-1]) / (freq[j] - freq[j-1]) * (magnitude[j] - magnitude[j-1]) + magnitude[j-1];
                const float ph = (x - freq[j-1]) / (freq[j] - freq[j-1]) * (phase[j] - phase[j-1]) + phase[j-1];

                complexf[i].re = mg * cosf(ph * M_PI);
                complexf[i].im = mg * sinf(ph * M_PI);
                break;
            }
        }
    }
}

static av_cold int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFIRSourceContext *s = ctx->priv;
    float overlap, scale, *output;
    int fft_size, middle, ret;

    if (s->nb_freq != s->nb_magnitude && s->nb_freq != s->nb_phase && s->nb_freq >= 2) {
        av_log(ctx, AV_LOG_ERROR, "Number of frequencies, magnitudes and phases must be same and >= 2.\n");
        return AVERROR(EINVAL);
    }

    for (int i = 0; i < s->nb_freq; i++) {
        if (i == 0 && s->freq_points_opt[i] != 0.f) {
            av_log(ctx, AV_LOG_ERROR, "First frequency must be 0.\n");
            return AVERROR(EINVAL);
        }

        if (i == s->nb_freq - 1 && s->freq_points_opt[i] != 1.f) {
            av_log(ctx, AV_LOG_ERROR, "Last frequency must be 1.\n");
            return AVERROR(EINVAL);
        }

        if (i && s->freq_points_opt[i] < s->freq_points_opt[i-1]) {
            av_log(ctx, AV_LOG_ERROR, "Frequencies must be in increasing order.\n");
            return AVERROR(EINVAL);
        }
    }

    fft_size = 1 << (av_log2(s->nb_taps) + 1);
    s->complexf = av_calloc(fft_size * 2, sizeof(*s->complexf));
    if (!s->complexf)
        return AVERROR(ENOMEM);

    scale = 1.f / fft_size;
    ret = av_tx_init(&s->tx_ctx, &s->tx_fn, AV_TX_FLOAT_RDFT, 1, fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    s->taps = av_calloc(s->nb_taps, sizeof(*s->taps));
    if (!s->taps)
        return AVERROR(ENOMEM);

    s->win = av_calloc(s->nb_taps, sizeof(*s->win));
    if (!s->win)
        return AVERROR(ENOMEM);

    generate_window_func(s->win, s->nb_taps, s->win_func, &overlap);

    lininterp(s->complexf, s->freq_points_opt, s->magnitude_opt, s->phase_opt, s->nb_freq, fft_size / 2);

    s->tx_fn(s->tx_ctx, s->complexf + fft_size, s->complexf, sizeof(*s->complexf));

    middle = s->nb_taps / 2;
    output = (float *)(s->complexf + fft_size);

    for (int i = 0; i <= middle; i++) {
        s->taps[         i] = output[middle - i] * s->win[i];
        s->taps[middle + i] = output[i] * s->win[middle + i];
    }

    s->pts = 0;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFIRSourceContext *s = ctx->priv;
    AVFrame *frame;
    int nb_samples;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    nb_samples = FFMIN(s->nb_samples, s->nb_taps - s->pts);
    if (nb_samples <= 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (!(frame = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);

    memcpy(frame->data[0], s->taps + s->pts, nb_samples * sizeof(float));

    frame->pts = s->pts;
    s->pts    += nb_samples;
    return ff_filter_frame(outlink, frame);
}

static const AVFilterPad afirsrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_asrc_afirsrc = {
    .p.name        = "afirsrc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate a FIR coefficients audio stream."),
    .p.priv_class  = &afirsrc_class,
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(AudioFIRSourceContext),
    FILTER_OUTPUTS(afirsrc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
