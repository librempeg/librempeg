/*
 * Copyright (c) 2018 Paul B Mahol
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

#include "libavutil/audio_fifo.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

static const char *filter_modes[] = {"clicks", "clips", "surges" };

typedef struct DeclickChannel {
    double *auxiliary;
    double *detection;
    double *acoefficients;
    double *acorrelation;
    double *tmp;
    double *interpolated;
    double *matrix;
    int matrix_size;
    double *vector;
    int vector_size;
    double *y;
    int y_size;
    uint8_t *click;
    int *index;
    unsigned *histogram;
    int histogram_size;
} DeclickChannel;

typedef struct AudioDeclickContext {
    const AVClass *class;

    double w;
    double overlap;
    double threshold;
    double ar;
    double burst;
    int method;
    int nb_hbins;

    int mode;
    int ar_order;
    int nb_surge_samples;
    int nb_burst_samples;
    int window_size;
    int hop_size;
    int overlap_skip;

    AVFrame *enabled;
    AVFrame *in;
    AVFrame *out;
    AVFrame *buffer;
    AVFrame *is;

    DeclickChannel *chan;

    int64_t pts;
    int nb_channels;
    uint64_t nb_samples;
    uint64_t detected_errors;
    int samples_left;
    int eof;

    AVAudioFifo *efifo;
    AVAudioFifo *fifo;
    double *window_func_lut;

    int (*detector)(struct AudioDeclickContext *s, DeclickChannel *c,
                    double sigmae, double *detection,
                    double *acoefficients, uint8_t *click, int *index,
                    const double *src, double *dst);
} AudioDeclickContext;

#define OFFSET(x) offsetof(AudioDeclickContext, x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption adeclick_options[] = {
    { "window", "set window size",     OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=55}, 10,  100, AF },
    { "w", "set window size",          OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=55}, 10,  100, AF },
    { "overlap", "set window overlap", OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75}, 50,   95, AF },
    { "o", "set window overlap",       OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75}, 50,   95, AF },
    { "arorder", "set autoregression order", OFFSET(ar),  AV_OPT_TYPE_DOUBLE, {.dbl=2},   0,   25, AF },
    { "a", "set autoregression order", OFFSET(ar),        AV_OPT_TYPE_DOUBLE, {.dbl=2},   0,   25, AF },
    { "threshold", "set threshold",    OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=2},   1,  100, AF },
    { "t", "set threshold",            OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=2},   1,  100, AF },
    { "burst", "set burst fusion",     OFFSET(burst),     AV_OPT_TYPE_DOUBLE, {.dbl=2},   0,   10, AF },
    { "b", "set burst fusion",         OFFSET(burst),     AV_OPT_TYPE_DOUBLE, {.dbl=2},   0,   10, AF },
    { "method", "set overlap method",  OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=0},   0,    1, AF, .unit = "m" },
    { "m", "set overlap method",       OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=0},   0,    1, AF, .unit = "m" },
    { "add", "overlap-add",            0,                 AV_OPT_TYPE_CONST,  {.i64=0},   0,    0, AF, .unit = "m" },
    { "a", "overlap-add",              0,                 AV_OPT_TYPE_CONST,  {.i64=0},   0,    0, AF, .unit = "m" },
    { "save", "overlap-save",          0,                 AV_OPT_TYPE_CONST,  {.i64=1},   0,    0, AF, .unit = "m" },
    { "s", "overlap-save",             0,                 AV_OPT_TYPE_CONST,  {.i64=1},   0,    0, AF, .unit = "m" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adeclick);

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioDeclickContext *s = ctx->priv;
    int i;

    s->pts = AV_NOPTS_VALUE;
    s->window_size = FFMAX(100, inlink->sample_rate * s->w / 1000.);
    s->ar_order = FFMAX(s->window_size * s->ar / 100., 1);
    s->nb_burst_samples = s->window_size * s->burst / 1000.;
    s->hop_size = FFMAX(1, s->window_size * (1. - (s->overlap / 100.)));

    s->window_func_lut = av_calloc(s->window_size, sizeof(*s->window_func_lut));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);

    {
        double *tx_in[2], *tx_out[2];
        AVTXContext *tx, *itx;
        av_tx_fn tx_fn, itx_fn;
        int ret, tx_size;
        double scale;

        tx_size = 1 << (32 - ff_clz(s->window_size));

        scale = 1.0;
        ret = av_tx_init(&tx, &tx_fn, AV_TX_DOUBLE_RDFT, 0, tx_size, &scale, 0);
        if (ret < 0)
            return ret;

        scale = 1.0 / tx_size;
        ret = av_tx_init(&itx, &itx_fn, AV_TX_DOUBLE_RDFT, 1, tx_size, &scale, 0);
        if (ret < 0)
            return ret;

        tx_in[0]  = av_calloc(tx_size + 2, sizeof(*tx_in[0]));
        tx_in[1]  = av_calloc(tx_size + 2, sizeof(*tx_in[1]));
        tx_out[0] = av_calloc(tx_size + 2, sizeof(*tx_out[0]));
        tx_out[1] = av_calloc(tx_size + 2, sizeof(*tx_out[1]));
        if (!tx_in[0] || !tx_in[1] || !tx_out[0] || !tx_out[1])
            return AVERROR(ENOMEM);

        for (int n = 0; n < s->window_size - s->hop_size; n++)
            tx_in[0][n] = 1.0;

        for (int n = 0; n < s->hop_size; n++)
            tx_in[1][n] = 1.0;

        tx_fn(tx, tx_out[0], tx_in[0], sizeof(double));
        tx_fn(tx, tx_out[1], tx_in[1], sizeof(double));

        for (int n = 0; n <= tx_size/2; n++) {
            double re0 = tx_out[0][2*n];
            double im0 = tx_out[0][2*n+1];
            double re1 = tx_out[1][2*n];
            double im1 = tx_out[1][2*n+1];

            tx_in[0][2*n]   = re0 * re1 - im0 * im1;
            tx_in[0][2*n+1] = re0 * im1 + re1 * im0;
        }

        itx_fn(itx, tx_out[0], tx_in[0], sizeof(AVComplexDouble));

        scale = 1.0 / (s->window_size - s->hop_size);
        for (int n = 0; n < s->window_size; n++)
            s->window_func_lut[n] = tx_out[0][n] * scale;

        av_tx_uninit(&tx);
        av_tx_uninit(&itx);

        av_freep(&tx_in[0]);
        av_freep(&tx_in[1]);
        av_freep(&tx_out[0]);
        av_freep(&tx_out[1]);
    }

    av_frame_free(&s->in);
    av_frame_free(&s->out);
    av_frame_free(&s->buffer);
    av_frame_free(&s->is);
    s->enabled = ff_get_audio_buffer(inlink, s->window_size);
    s->in = ff_get_audio_buffer(inlink, s->window_size);
    s->out = ff_get_audio_buffer(inlink, s->window_size);
    s->buffer = ff_get_audio_buffer(inlink, s->window_size * 2);
    s->is = ff_get_audio_buffer(inlink, s->window_size);
    if (!s->in || !s->out || !s->buffer || !s->is || !s->enabled)
        return AVERROR(ENOMEM);

    s->efifo = av_audio_fifo_alloc(inlink->format, 1, s->window_size);
    if (!s->efifo)
        return AVERROR(ENOMEM);
    s->fifo = av_audio_fifo_alloc(inlink->format, inlink->ch_layout.nb_channels, s->window_size);
    if (!s->fifo)
        return AVERROR(ENOMEM);
    s->overlap_skip = s->method ? (s->window_size - s->hop_size) / 2 : 0;
    if (s->overlap_skip > 0) {
        av_audio_fifo_write(s->fifo, (void **)s->in->extended_data,
                            s->overlap_skip);
    }

    s->nb_channels = inlink->ch_layout.nb_channels;
    s->chan = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->chan));
    if (!s->chan)
        return AVERROR(ENOMEM);

    for (i = 0; i < inlink->ch_layout.nb_channels; i++) {
        DeclickChannel *c = &s->chan[i];

        c->detection = av_calloc(s->window_size, sizeof(*c->detection));
        c->auxiliary = av_calloc(s->ar_order + 1, sizeof(*c->auxiliary));
        c->acoefficients = av_calloc(s->ar_order + 1, sizeof(*c->acoefficients));
        c->acorrelation = av_calloc(s->ar_order + 1, sizeof(*c->acorrelation));
        c->tmp = av_calloc(s->ar_order, sizeof(*c->tmp));
        c->click = av_calloc(s->window_size, sizeof(*c->click));
        c->index = av_calloc(s->window_size, sizeof(*c->index));
        c->interpolated = av_calloc(s->window_size, sizeof(*c->interpolated));
        if (!c->auxiliary || !c->acoefficients || !c->detection || !c->click ||
            !c->index || !c->interpolated || !c->acorrelation || !c->tmp)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void autocorrelation(const double *input, int order, int size,
                            double *output, double scale)
{
    int i, j;

    for (i = 0; i <= order; i++) {
        double value = 0.;

        for (j = i; j < size; j++)
            value += input[j] * input[j - i];

        output[i] = value * scale;
    }
}

static double autoregression(const double *samples, int ar_order,
                             int nb_samples, double *k, double *r, double *a)
{
    double alpha;
    int i, j;

    memset(a, 0, ar_order * sizeof(*a));

    autocorrelation(samples, ar_order, nb_samples, r, 1. / nb_samples);

    /* Levinson-Durbin algorithm */
    k[0] = a[0] = -r[1] / r[0];
    alpha = r[0] * (1. - k[0] * k[0]);
    for (i = 1; i < ar_order; i++) {
        double epsilon = 0.;

        for (j = 0; j < i; j++)
            epsilon += a[j] * r[i - j];
        epsilon += r[i + 1];

        k[i] = -epsilon / alpha;
        alpha *= (1. - k[i] * k[i]);
        for (j = i - 1; j >= 0; j--)
            k[j] = a[j] + k[i] * a[i - j - 1];
        for (j = 0; j <= i; j++)
            a[j] = k[j];
    }

    k[0] = 1.;
    for (i = 1; i <= ar_order; i++)
        k[i] = a[i - 1];

    return sqrt(alpha);
}

static int isfinite_array(double *samples, int nb_samples)
{
    int i;

    for (i = 0; i < nb_samples; i++)
        if (!isfinite(samples[i]))
            return 0;

    return 1;
}

static int find_index(int *index, int value, int size)
{
    int i, start, end;

    if ((value < index[0]) || (value > index[size - 1]))
        return 1;

    i = start = 0;
    end = size - 1;

    while (start <= end) {
        i = (end + start) / 2;
        if (index[i] == value)
            return 0;
        if (value < index[i])
            end = i - 1;
        if (value > index[i])
            start = i + 1;
    }

    return 1;
}

static int factorization(double *matrix, int n)
{
    int i, j, k;

    for (i = 0; i < n; i++) {
        const int in = i * n;
        double value;

        value = matrix[in + i];
        for (j = 0; j < i; j++)
            value -= matrix[j * n + j] * matrix[in + j] * matrix[in + j];

        if (value == 0.) {
            return -1;
        }

        matrix[in + i] = value;
        for (j = i + 1; j < n; j++) {
            const int jn = j * n;
            double x;

            x = matrix[jn + i];
            for (k = 0; k < i; k++)
                x -= matrix[k * n + k] * matrix[in + k] * matrix[jn + k];
            matrix[jn + i] = x / matrix[in + i];
        }
    }

    return 0;
}

static int do_interpolation(DeclickChannel *c, double *matrix,
                            double *vector, int n, double *out)
{
    int i, j, ret;
    double *y;

    ret = factorization(matrix, n);
    if (ret < 0)
        return ret;

    av_fast_malloc(&c->y, &c->y_size, n * sizeof(*c->y));
    y = c->y;
    if (!y)
        return AVERROR(ENOMEM);

    for (i = 0; i < n; i++) {
        const int in = i * n;
        double value;

        value = vector[i];
        for (j = 0; j < i; j++)
            value -= matrix[in + j] * y[j];
        y[i] = value;
    }

    for (i = n - 1; i >= 0; i--) {
        out[i] = y[i] / matrix[i * n + i];
        for (j = i + 1; j < n; j++)
            out[i] -= matrix[j * n + i] * out[j];
    }

    return 0;
}

static int interpolation(DeclickChannel *c, const double *src, int ar_order,
                         double *acoefficients, int *index, int nb_errors,
                         double *auxiliary, double *interpolated)
{
    double *vector, *matrix;
    int i, j;

    av_fast_malloc(&c->matrix, &c->matrix_size, nb_errors * nb_errors * sizeof(*c->matrix));
    matrix = c->matrix;
    if (!matrix)
        return AVERROR(ENOMEM);

    av_fast_malloc(&c->vector, &c->vector_size, nb_errors * sizeof(*c->vector));
    vector = c->vector;
    if (!vector)
        return AVERROR(ENOMEM);

    autocorrelation(acoefficients, ar_order, ar_order + 1, auxiliary, 1.);

    for (i = 0; i < nb_errors; i++) {
        const int im = i * nb_errors;

        for (j = i; j < nb_errors; j++) {
            if (abs(index[j] - index[i]) <= ar_order) {
                matrix[j * nb_errors + i] = matrix[im + j] = auxiliary[abs(index[j] - index[i])];
            } else {
                matrix[j * nb_errors + i] = matrix[im + j] = 0;
            }
        }
    }

    for (i = 0; i < nb_errors; i++) {
        double value = 0.;

        for (j = -ar_order; j <= ar_order; j++)
            if (find_index(index, index[i] - j, nb_errors))
                value -= src[index[i] - j] * auxiliary[abs(j)];

        vector[i] = value;
    }

    return do_interpolation(c, matrix, vector, nb_errors, interpolated);
}

static int detect_clips(AudioDeclickContext *s, DeclickChannel *c,
                        double unused0,
                        double *unused1, double *unused2,
                        uint8_t *clip, int *index,
                        const double *src, double *dst)
{
    const double threshold = s->threshold;
    double max_amplitude = 0;
    unsigned *histogram;
    int i, nb_clips = 0;

    av_fast_malloc(&c->histogram, &c->histogram_size, s->nb_hbins * sizeof(*c->histogram));
    if (!c->histogram)
        return AVERROR(ENOMEM);
    histogram = c->histogram;
    memset(histogram, 0, sizeof(*histogram) * s->nb_hbins);

    for (i = 0; i < s->window_size; i++) {
        const unsigned index = fmin(fabs(src[i]), 1) * (s->nb_hbins - 1);

        histogram[index]++;
        dst[i] = src[i];
        clip[i] = 0;
    }

    for (i = s->nb_hbins - 1; i > 1; i--) {
        if (histogram[i]) {
            if (histogram[i] / (double)FFMAX(histogram[i - 1], 1) > threshold) {
                max_amplitude = i / (double)s->nb_hbins;
            }
            break;
        }
    }

    if (max_amplitude > 0.) {
        for (i = 0; i < s->window_size; i++) {
            clip[i] = fabs(src[i]) >= max_amplitude;
        }
    }

    memset(clip, 0, s->ar_order * sizeof(*clip));
    memset(clip + (s->window_size - s->ar_order), 0, s->ar_order * sizeof(*clip));

    for (i = s->ar_order; i < s->window_size - s->ar_order; i++)
        if (clip[i])
            index[nb_clips++] = i;

    return nb_clips;
}

static int detect_clicks(AudioDeclickContext *s, DeclickChannel *c,
                         double sigmae,
                         double *detection, double *acoefficients,
                         uint8_t *click, int *index,
                         const double *src, double *dst)
{
    const double threshold = s->threshold;
    int i, j, nb_clicks = 0, prev = -1;

    memset(detection, 0, s->window_size * sizeof(*detection));

    for (i = s->ar_order; i < s->window_size; i++) {
        for (j = 0; j <= s->ar_order; j++) {
            detection[i] += acoefficients[j] * src[i - j];
        }
    }

    for (i = 0; i < s->window_size; i++) {
        click[i] = fabs(detection[i]) > sigmae * threshold;
        dst[i] = src[i];
    }

    for (i = 0; i < s->window_size; i++) {
        if (!click[i])
            continue;

        if (prev >= 0 && (i > prev + 1) && (i <= s->nb_burst_samples + prev))
            for (j = prev + 1; j < i; j++)
                click[j] = 1;
        prev = i;
    }

    memset(click, 0, s->ar_order * sizeof(*click));
    memset(click + (s->window_size - s->ar_order), 0, s->ar_order * sizeof(*click));

    for (i = s->ar_order; i < s->window_size - s->ar_order; i++)
        if (click[i])
            index[nb_clicks++] = i;

    return nb_clicks;
}

static int detect_surges(AudioDeclickContext *s, DeclickChannel *c,
                         double sigmae,
                         double *detection, double *acoefficients,
                         uint8_t *surge, int *index,
                         const double *src, double *dst)
{
    const double threshold = s->threshold;
    const int size = s->nb_surge_samples * 2;
    int i, j, nb_surges = 0;

    memset(detection, 0, s->window_size * sizeof(*detection));

    for (i = s->ar_order; i < s->window_size; i++) {
        for (j = 0; j <= s->ar_order; j++) {
            detection[i] += acoefficients[j] * src[i - j];
        }
    }

    for (i = 0; i < s->window_size; i++) {
        surge[i] = fabs(detection[i]) > sigmae * threshold;
        dst[i] = src[i];
    }

    for (i = 0; i < s->window_size;) {
        if (!surge[i++])
            continue;
        memset(surge + FFMAX(i - size/2, 0), 1, FFMIN(size, s->window_size - i));
        i += size/2;
    }

    memset(surge, 0, s->ar_order * sizeof(*surge));
    memset(surge + (s->window_size - s->ar_order), 0, s->ar_order * sizeof(*surge));

    for (i = s->ar_order; i < s->window_size - s->ar_order; i++)
        if (surge[i])
            index[nb_surges++] = i;

    return nb_surges;
}

typedef struct ThreadData {
    AVFrame *out;
} ThreadData;

static int filter_channel(AVFilterContext *ctx, void *arg, int ch, int nb_jobs)
{
    AudioDeclickContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    const double *src = (const double *)s->in->extended_data[ch];
    double *is = (double *)s->is->extended_data[ch];
    double *dst = (double *)s->out->extended_data[ch];
    double *ptr = (double *)out->extended_data[ch];
    double *buf = (double *)s->buffer->extended_data[ch];
    const double *w = s->window_func_lut;
    DeclickChannel *c = &s->chan[ch];
    double sigmae;
    int j, ret;

    sigmae = autoregression(src, s->ar_order, s->window_size, c->acoefficients, c->acorrelation, c->tmp);

    if (isfinite_array(c->acoefficients, s->ar_order + 1)) {
        double *interpolated = c->interpolated;
        int *index = c->index;
        int nb_errors;

        nb_errors = s->detector(s, c, sigmae, c->detection, c->acoefficients,
                                c->click, index, src, dst);
        if (nb_errors > 0) {
            double *enabled = (double *)s->enabled->extended_data[0];

            ret = interpolation(c, src, s->ar_order, c->acoefficients, index,
                                nb_errors, c->auxiliary, interpolated);
            if (ret < 0)
                return ret;

            av_audio_fifo_peek(s->efifo, (void**)s->enabled->extended_data, s->window_size);

            for (j = 0; j < nb_errors; j++) {
                if (enabled[index[j]]) {
                    dst[index[j]] = interpolated[j];
                    is[index[j]] = 1;
                }
            }
        }
    } else {
        memcpy(dst, src, s->window_size * sizeof(*dst));
    }

    if (s->method == 0) {
        for (j = 0; j < s->window_size; j++)
            buf[j] += dst[j] * w[j];
    } else {
        const int skip = s->overlap_skip;

        for (j = 0; j < s->hop_size; j++)
            buf[j] = dst[skip + j];
    }
    for (j = 0; j < s->hop_size; j++)
        ptr[j] = buf[j];

    memmove(buf, buf + s->hop_size, (s->window_size * 2 - s->hop_size) * sizeof(*buf));
    memmove(is, is + s->hop_size, (s->window_size - s->hop_size) * sizeof(*is));
    memset(buf + s->window_size * 2 - s->hop_size, 0, s->hop_size * sizeof(*buf));
    memset(is + s->window_size - s->hop_size, 0, s->hop_size * sizeof(*is));

    return 0;
}

static int filter_frame(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioDeclickContext *s = ctx->priv;
    AVFrame *out = NULL;
    int ret = 0, j, ch, detected_errors = 0;
    ThreadData td;

    out = ff_get_audio_buffer(outlink, s->hop_size);
    if (!out)
        return AVERROR(ENOMEM);

    ret = av_audio_fifo_peek(s->fifo, (void **)s->in->extended_data,
                             s->window_size);
    if (ret < 0)
        goto fail;

    td.out = out;
    ret = ff_filter_execute(ctx, filter_channel, &td, NULL, inlink->ch_layout.nb_channels);
    if (ret < 0)
        goto fail;

    for (ch = 0; ch < s->in->ch_layout.nb_channels; ch++) {
        double *is = (double *)s->is->extended_data[ch];

        for (j = 0; j < s->hop_size; j++) {
            if (is[j])
                detected_errors++;
        }
    }

    av_audio_fifo_drain(s->fifo, s->hop_size);
    av_audio_fifo_drain(s->efifo, s->hop_size);

    if (s->samples_left > 0)
        out->nb_samples = FFMIN(s->hop_size, s->samples_left);

    out->pts = s->pts;
    s->pts += av_rescale_q(s->hop_size, (AVRational){1, outlink->sample_rate}, outlink->time_base);

    s->detected_errors += detected_errors;
    s->nb_samples += out->nb_samples * inlink->ch_layout.nb_channels;

    ret = ff_filter_frame(outlink, out);
    if (ret < 0)
        return ret;

    if (s->samples_left > 0) {
        s->samples_left -= s->hop_size;
        if (s->samples_left <= 0)
            av_audio_fifo_drain(s->fifo, av_audio_fifo_size(s->fifo));
    }

fail:
    if (ret < 0)
        av_frame_free(&out);
    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioDeclickContext *s = ctx->priv;
    AVFrame *in;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->window_size, s->window_size, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        double *e = (double *)s->enabled->extended_data[0];

        if (s->pts == AV_NOPTS_VALUE)
            s->pts = in->pts;

        ret = av_audio_fifo_write(s->fifo, (void **)in->extended_data,
                                  in->nb_samples);
        for (int i = 0; i < in->nb_samples; i++)
            e[i] = !ff_filter_disabled(ctx);

        av_audio_fifo_write(s->efifo, (void**)s->enabled->extended_data, in->nb_samples);
        av_frame_free(&in);
        if (ret < 0)
            return ret;
    }

    if (av_audio_fifo_size(s->fifo) >= s->window_size ||
        s->samples_left > 0)
        return filter_frame(inlink);

    if (av_audio_fifo_size(s->fifo) >= s->window_size) {
        ff_filter_set_ready(ctx, 100);
        return 0;
    }

    if (!s->eof && ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF) {
            s->eof = 1;
            s->samples_left = av_audio_fifo_size(s->fifo) - s->overlap_skip;
            ff_filter_set_ready(ctx, 100);
            return 0;
        }
    }

    if (s->eof && s->samples_left <= 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (!s->eof)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioDeclickContext *s = ctx->priv;

    if (!strcmp(ctx->filter->name, "adesurge")) {
        s->mode = 2;
        s->detector = detect_surges;
    } else if (!strcmp(ctx->filter->name, "adeclip")) {
        s->mode = 1;
        s->detector = detect_clips;
    } else {
        s->mode = 0;
        s->detector = detect_clicks;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioDeclickContext *s = ctx->priv;
    int i;

    if (s->nb_samples > 0)
        av_log(ctx, AV_LOG_INFO, "Detected %s in %"PRId64" of %"PRId64" samples (%g%%).\n",
               filter_modes[s->mode], s->detected_errors,
               s->nb_samples, 100. * s->detected_errors / s->nb_samples);

    av_audio_fifo_free(s->fifo);
    av_audio_fifo_free(s->efifo);
    av_freep(&s->window_func_lut);
    av_frame_free(&s->enabled);
    av_frame_free(&s->in);
    av_frame_free(&s->out);
    av_frame_free(&s->buffer);
    av_frame_free(&s->is);

    if (s->chan) {
        for (i = 0; i < s->nb_channels; i++) {
            DeclickChannel *c = &s->chan[i];

            av_freep(&c->detection);
            av_freep(&c->auxiliary);
            av_freep(&c->acoefficients);
            av_freep(&c->acorrelation);
            av_freep(&c->tmp);
            av_freep(&c->click);
            av_freep(&c->index);
            av_freep(&c->interpolated);
            av_freep(&c->matrix);
            c->matrix_size = 0;
            av_freep(&c->histogram);
            c->histogram_size = 0;
            av_freep(&c->vector);
            c->vector_size = 0;
            av_freep(&c->y);
            c->y_size = 0;
        }
    }
    av_freep(&s->chan);
    s->nb_channels = 0;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_adeclick = {
    .p.name        = "adeclick",
    .p.description = NULL_IF_CONFIG_SMALL("Remove impulsive noise from input audio."),
    .p.priv_class  = &adeclick_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size     = sizeof(AudioDeclickContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
};

static const AVOption adeclip_options[] = {
    { "window", "set window size",     OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=55},     10,  100, AF },
    { "w", "set window size",          OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=55},     10,  100, AF },
    { "overlap", "set window overlap", OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75},     50,   95, AF },
    { "o", "set window overlap",       OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75},     50,   95, AF },
    { "arorder", "set autoregression order", OFFSET(ar),  AV_OPT_TYPE_DOUBLE, {.dbl=8},       0,   25, AF },
    { "a", "set autoregression order", OFFSET(ar),        AV_OPT_TYPE_DOUBLE, {.dbl=8},       0,   25, AF },
    { "threshold", "set threshold",    OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=10},      1,  100, AF },
    { "t", "set threshold",            OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=10},      1,  100, AF },
    { "hsize", "set histogram size",   OFFSET(nb_hbins),  AV_OPT_TYPE_INT,    {.i64=1000},  100, 9999, AF },
    { "n", "set histogram size",       OFFSET(nb_hbins),  AV_OPT_TYPE_INT,    {.i64=1000},  100, 9999, AF },
    { "method", "set overlap method",  OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=0},       0,    1, AF, .unit = "m" },
    { "m", "set overlap method",       OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=0},       0,    1, AF, .unit = "m" },
    { "add", "overlap-add",            0,                 AV_OPT_TYPE_CONST,  {.i64=0},       0,    0, AF, .unit = "m" },
    { "a", "overlap-add",              0,                 AV_OPT_TYPE_CONST,  {.i64=0},       0,    0, AF, .unit = "m" },
    { "save", "overlap-save",          0,                 AV_OPT_TYPE_CONST,  {.i64=1},       0,    0, AF, .unit = "m" },
    { "s", "overlap-save",             0,                 AV_OPT_TYPE_CONST,  {.i64=1},       0,    0, AF, .unit = "m" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adeclip);

const FFFilter ff_af_adeclip = {
    .p.name        = "adeclip",
    .p.description = NULL_IF_CONFIG_SMALL("Remove clipping from input audio."),
    .p.priv_class  = &adeclip_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size     = sizeof(AudioDeclickContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
};

static const AVOption adesurge_options[] = {
    { "window", "set window size",     OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=85},     10,  100, AF },
    { "w", "set window size",          OFFSET(w),         AV_OPT_TYPE_DOUBLE, {.dbl=85},     10,  100, AF },
    { "overlap", "set window overlap", OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75},     50,   95, AF },
    { "o", "set window overlap",       OFFSET(overlap),   AV_OPT_TYPE_DOUBLE, {.dbl=75},     50,   95, AF },
    { "arorder", "set autoregression order", OFFSET(ar),  AV_OPT_TYPE_DOUBLE, {.dbl=0.5},     0,   25, AF },
    { "a", "set autoregression order", OFFSET(ar),        AV_OPT_TYPE_DOUBLE, {.dbl=0.5},     0,   25, AF },
    { "threshold", "set threshold",    OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=20},      1,  100, AF },
    { "t", "set threshold",            OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=20},      1,  100, AF },
    { "surges", "set surge size",      OFFSET(nb_surge_samples), AV_OPT_TYPE_INT, {.i64=5},   1,   50, AF },
    { "s", "set surge size",           OFFSET(nb_surge_samples), AV_OPT_TYPE_INT, {.i64=5},   1,   50, AF },
    { "method", "set overlap method",  OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=1},       0,    1, AF, "m" },
    { "m", "set overlap method",       OFFSET(method),    AV_OPT_TYPE_INT,    {.i64=1},       0,    1, AF, "m" },
    { "add", "overlap-add",            0,                 AV_OPT_TYPE_CONST,  {.i64=0},       0,    0, AF, "m" },
    { "a", "overlap-add",              0,                 AV_OPT_TYPE_CONST,  {.i64=0},       0,    0, AF, "m" },
    { "save", "overlap-save",          0,                 AV_OPT_TYPE_CONST,  {.i64=1},       0,    0, AF, "m" },
    { "s", "overlap-save",             0,                 AV_OPT_TYPE_CONST,  {.i64=1},       0,    0, AF, "m" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adesurge);

const FFFilter ff_af_adesurge = {
    .p.name        = "adesurge",
    .p.description = NULL_IF_CONFIG_SMALL("Remove surges from input audio."),
    .p.priv_class  = &adesurge_class,
    .priv_size     = sizeof(AudioDeclickContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
