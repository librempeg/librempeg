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

/**
 * @file
 * FIR coefficients generator: takes two mono streams describing a frequency
 * response (magnitude in dB and unwrapped phase in radians, both sampled
 * uniformly from DC to Nyquist, end points included) and outputs one mono
 * stream holding the FIR coefficients that realise that response.
 */

#include <math.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#define ALIAS_WARN_DB (-100.0)

typedef struct AFIRGenContext {
    const AVClass *class;

    int size;               ///< real FFT size (also max accepted input length)
    int taps;               ///< number of output coefficients
    double beta;            ///< Kaiser beta
    int phase_has_delay;    ///< phase input already contains the filter delay

    AVTXContext *tx;
    av_tx_fn tx_fn;

    AVFrame *in[2];         ///< collected input samples: 0 = magnitude [dB], 1 = phase [rad]
    int nb_in[2];
    int eof[2];
    int64_t eof_pts;
    int done;

    AVComplexDouble *spec;  ///< size/2+1 bins; the inverse RDFT overwrites it
    double *ir;             ///< size real samples
    double *win;            ///< taps
} AFIRGenContext;

#define OFFSET(x) offsetof(AFIRGenContext, x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption afirgen_options[] = {
    { "size", "set FFT size, also the maximum input length in samples",
        OFFSET(size), AV_OPT_TYPE_INT, {.i64=65536}, 4, 1 << 24, AF },
    { "taps", "set number of output FIR coefficients",
        OFFSET(taps), AV_OPT_TYPE_INT, {.i64=1025}, 1, 1 << 24, AF },
    { "beta", "set Kaiser window beta",
        OFFSET(beta), AV_OPT_TYPE_DOUBLE, {.dbl=8.0}, 0, 200, AF },
    { "phase_has_delay", "phase input already includes the filter delay; "
                         "otherwise a delay of (taps-1)/2 samples is added",
        OFFSET(phase_has_delay), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(afirgen);

static av_cold int init(AVFilterContext *ctx)
{
    AFIRGenContext *s = ctx->priv;
    double scale;
    int ret;

    if (s->taps > s->size) {
        av_log(ctx, AV_LOG_ERROR, "taps (%d) must not exceed FFT size (%d).\n",
               s->taps, s->size);
        return AVERROR(EINVAL);
    }

    scale = 1.0 / s->size;
    ret = av_tx_init(&s->tx, &s->tx_fn, AV_TX_DOUBLE_RDFT, 1, s->size, &scale, 0);
    if (ret < 0)
        return ret;

    s->spec = av_calloc(s->size / 2 + 1, sizeof(*s->spec));
    s->ir   = av_calloc(s->size, sizeof(*s->ir));
    s->win  = av_calloc(s->taps, sizeof(*s->win));
    if (!s->spec || !s->ir || !s->win)
        return AVERROR(ENOMEM);

    {
        const double n = s->taps;
        const double norm = 1.0 / av_bessel_i0(s->beta);

        for (int k = 0; k < s->taps; k++) {
            double x = n > 1 ? 2.0 * k / (n - 1) - 1.0 : 0.0;

            s->win[k] = av_bessel_i0(s->beta * sqrt(FFMAX(0.0, 1.0 - x * x))) * norm;
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AFIRGenContext *s = ctx->priv;

    av_tx_uninit(&s->tx);
    av_frame_free(&s->in[0]);
    av_frame_free(&s->in[1]);
    av_freep(&s->spec);
    av_freep(&s->ir);
    av_freep(&s->win);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AFIRGenContext *s = ctx->priv;

    outlink->sample_rate = ctx->inputs[0]->sample_rate;
    outlink->time_base   = (AVRational){ 1, outlink->sample_rate };

    for (int i = 0; i < 2; i++) {
        s->in[i] = av_frame_alloc();
        if (!s->in[i])
            return AVERROR(ENOMEM);

        s->in[i]->nb_samples = s->size;
        int ret = ff_filter_get_buffer(ctx, s->in[i]);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int append_frame(AVFilterContext *ctx, int idx, const AVFrame *frame)
{
    AFIRGenContext *s = ctx->priv;
    static const char *const names[2] = { "magnitude", "phase" };
    const int nb_samples = frame->nb_samples;

    if (nb_samples > s->size - s->nb_in[idx]) {
        av_log(ctx, AV_LOG_ERROR, "Input '%s' is too long: more than %d samples "
               "(the FFT size) were supplied.\n", names[idx], s->size);
        return AVERROR(EINVAL);
    }

    for (int ch = 0; ch < frame->ch_layout.nb_channels; ch++) {
        const double *src = (const double *)frame->extended_data[ch];
        double *dst = (double *)s->in[idx]->extended_data[ch];

        for (int n = 0; n < nb_samples; n++) {
            double v = src[n];

            if (idx == 0 ? (isnan(v) || v == INFINITY) : !isfinite(v)) {
                av_log(ctx, AV_LOG_ERROR, "Invalid (non-finite) %s sample at index %d, channel %d.\n",
                       names[idx], s->nb_in[idx] + n, ch);
                return AVERROR_INVALIDDATA;
            }

            // -inf dB is legal (zero gain); floor it so interpolation stays finite
            dst[s->nb_in[idx] + n] = idx == 0 ? FFMAX(v, -400.0) : v;
        }
    }

    s->nb_in[idx] += nb_samples;

    return 0;
}

static int design_and_send(AVFilterContext *ctx)
{
    AFIRGenContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_bins = s->size / 2 + 1;
    const int M = s->nb_in[0];
    const double delay = (s->taps - 1) * 0.5;
    double peak = 0.0, tail = 0.0;
    AVFrame *out;
    double *dst;

    if (M <= 0) {
        av_log(ctx, AV_LOG_ERROR, "No input samples.\n");
        return AVERROR(EINVAL);
    }

    if (M != s->nb_in[1]) {
        av_log(ctx, AV_LOG_ERROR, "Magnitude and phase inputs must have the same "
               "length (%d vs %d samples).\n", s->nb_in[0], s->nb_in[1]);
        return AVERROR(EINVAL);
    }

    out = ff_get_audio_buffer(outlink, s->taps);
    if (!out)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        const double *phase = (const double *)s->in[1]->extended_data[ch];
        const double *magn = (const double *)s->in[0]->extended_data[ch];

        for (int k = 0; k < nb_bins; k++) {
            const double pos = M > 1 ? (double)k * (M - 1) / (s->size / 2) : 0.0;
            const int i0 = FFMIN((int)pos, M - 1);
            const int i1 = FFMIN(i0 + 1, M - 1);
            const double t = pos - i0;
            const double db = magn[i0] * (1.0 - t) + magn[i1] * t;
            double ph = phase[i0] * (1.0 - t) + phase[i1] * t;
            const double mag = exp(db * (M_LN10 / 20.0));

            if (!s->phase_has_delay)
                ph -= 2.0 * M_PI * k / s->size * delay;   // centre the response

            s->spec[k].re = mag * cos(ph);
            s->spec[k].im = mag * sin(ph);
        }

        s->spec[0].im = 0.0;                    // DC and Nyquist bins of a real
        s->spec[nb_bins - 1].im = 0.0;          // signal are purely real

        s->tx_fn(s->tx, s->ir, s->spec, sizeof(*s->spec));

        // Time-domain aliasing check: the response is centred at `delay`, so the
        // point furthest away (circularly) is delay + size/2.
        for (int n = 0; n < s->size; n++)
            peak = FFMAX(peak, fabs(s->ir[n]));
        {
            const int centre = (int)lrint(delay + s->size / 2) % s->size;
            const int half = FFMAX(1, s->size / 40);

            for (int j = -half; j <= half; j++)
                tail = FFMAX(tail, fabs(s->ir[((centre + j) % s->size + s->size) % s->size]));
        }

        if (peak <= 0.0) {
            av_log(ctx, AV_LOG_WARNING, "Target response is all zero for %d channel.\n", ch);
        } else {
            const double alias_db = 20.0 * log10(FFMAX(tail, 1e-300) / peak);

            av_log(ctx, AV_LOG_INFO, "channel %d, %d input samples, FFT size %d, %d taps, "
                   "delay %.1f samples, far-end impulse level %.1f dB.\n",
                   ch, M, s->size, s->taps, delay, alias_db);
            if (alias_db > ALIAS_WARN_DB)
                av_log(ctx, AV_LOG_WARNING, "Channel %d impulse response only decays to %.1f dB "
                       "at the far end of the FFT: time-domain aliasing is likely. "
                       "Increase 'size' or smooth sharp features in the inputs.\n",
                       ch, alias_db);
        }

        double *ir = s->ir;
        const double *win = s->win;
        dst = (double *)out->extended_data[ch];
        for (int n = 0; n < s->taps; n++)
            dst[n] = ir[n] * win[n];
    }

    out->pts = 0;

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AFIRGenContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    for (int i = 0; i < 2; i++) {
        AVFilterLink *inlink = ctx->inputs[i];
        AVFrame *frame;
        int status;
        int64_t pts;

        if (s->eof[i])
            continue;

        while ((ret = ff_inlink_consume_frame(inlink, &frame)) > 0) {
            ret = append_frame(ctx, i, frame);
            av_frame_free(&frame);
            if (ret < 0)
                return ret;
        }

        if (ret < 0)
            return ret;

        if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
            s->eof[i] = 1;
            s->eof_pts = FFMAX(s->eof_pts, pts);
        }
    }

    if (s->eof[0] && s->eof[1] && !s->done) {
        s->done = 1;
        ret = design_and_send(ctx);
        if (ret < 0)
            return ret;

        ff_outlink_set_status(outlink, AVERROR_EOF, s->taps);

        return 0;
    }

    if (!s->done) {
        for (int i = 0; i < 2; i++) {
            if (!s->eof[i] && ff_inlink_queued_frames(ctx->inputs[i]) == 0)
                ff_inlink_request_frame(ctx->inputs[i]);
        }
    }

    return FFERROR_NOT_READY;
}

static const AVFilterPad inputs[] = {
    {
        .name = "magnitude",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "phase",
        .type = AVMEDIA_TYPE_AUDIO,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_afirgen = {
    .p.name        = "afirgen",
    .p.description = NULL_IF_CONFIG_SMALL("Generate FIR coefficients from magnitude (dB) and unwrapped phase (rad) streams."),
    .p.priv_class  = &afirgen_class,
    .priv_size     = sizeof(AFIRGenContext),
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
};
