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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#define ftype double

typedef struct ctype {
    ftype re, im;
} ctype;

typedef struct ALSFIRSRCContext {
    const AVClass *class;

    int sample_rate;
    int nb_taps;
    size_t grid_size;
    int *grid_opt;
    unsigned grid_opt_size;
    double *freq0_opt;
    unsigned freq0_opt_size;
    double *freq1_opt;
    unsigned freq1_opt_size;
    double *mag_opt;
    unsigned mag_opt_size;
    double *gdelay_opt;
    unsigned gdelay_opt_size;
    double *weight_opt;
    unsigned weight_opt_size;

    ctype *D, *e1, *dvec, *evec;
    ftype *a, *b, *W, *om, *t, *T;
    double *taps;

    int nb_samples;
    int64_t pts;
} ALSFIRSRCContext;

#define OFFSET(x) offsetof(ALSFIRSRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_grid   = {.def="200 100",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_freq0  = {.def="0 1200",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_freq1  = {.def="1100 192000",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_mag    = {.def="1 0",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_gdelay = {.def="25 25",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_weight = {.def="0.00001 1000",.size_min=1,.sep=' '};

static const AVOption alsfirsrc_options[] = {
    { "taps",        "set the number of taps",   OFFSET(nb_taps),     AV_OPT_TYPE_INT,       {.i64=161},         9, UINT16_MAX, FLAGS },
    { "t",           "set the number of taps",   OFFSET(nb_taps),     AV_OPT_TYPE_INT,       {.i64=161},         9, UINT16_MAX, FLAGS },
    { "grid",        "set the grid density",     OFFSET(grid_opt),    AV_OPT_TYPE_INT|AR,    {.arr=&def_grid},   1, UINT16_MAX, FLAGS },
    { "g",           "set the grid density",     OFFSET(grid_opt),    AV_OPT_TYPE_INT|AR,    {.arr=&def_grid},   1, UINT16_MAX, FLAGS },
    { "fstart",      "set the frequency start",  OFFSET(freq0_opt),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_freq0},  0, INT_MAX/2,  FLAGS },
    { "fstop",       "set the frequency stop",   OFFSET(freq1_opt),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_freq1},  1, INT_MAX/2,  FLAGS },
    { "magnitude",   "set the magnitude points", OFFSET(mag_opt),     AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_mag},    0, UINT16_MAX, FLAGS },
    { "m",           "set the magnitude points", OFFSET(mag_opt),     AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_mag},    0, UINT16_MAX, FLAGS },
    { "gdelay",    "set the group delay points", OFFSET(gdelay_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_gdelay}, INT16_MIN, INT16_MAX, FLAGS },
    { "d",         "set the group delay points", OFFSET(gdelay_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_gdelay}, INT16_MIN, INT16_MAX, FLAGS },
    { "weight",      "set the weighting points", OFFSET(weight_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_weight}, 0, INT_MAX,    FLAGS },
    { "w",           "set the weighting points", OFFSET(weight_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_weight}, 0, INT_MAX,    FLAGS },
    { "sample_rate", "set the sample rate",      OFFSET(sample_rate), AV_OPT_TYPE_INT,       {.i64=44100},       1, INT_MAX,    FLAGS },
    { "r",           "set the sample rate",      OFFSET(sample_rate), AV_OPT_TYPE_INT,       {.i64=44100},       1, INT_MAX,    FLAGS },
    { "nb_samples",  "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "n",           "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(alsfirsrc);

static av_cold void uninit(AVFilterContext *ctx)
{
    ALSFIRSRCContext *s = ctx->priv;

    av_freep(&s->t);
    av_freep(&s->T);
    av_freep(&s->D);
    av_freep(&s->W);
    av_freep(&s->om);
    av_freep(&s->e1);
    av_freep(&s->dvec);
    av_freep(&s->evec);
    av_freep(&s->a);
    av_freep(&s->b);
    av_freep(&s->taps);
}

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const ALSFIRSRCContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_DBL,
        AV_SAMPLE_FMT_NONE
    };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static void cmul(ctype a, ctype b, ctype *c)
{
    c->re = a.re * b.re - a.im * b.im;
    c->im = a.re * b.im + a.im * b.re;
}

static void flip(ftype *T, const ftype *t, const int pad, const int N)
{
    if (pad)
        T[0] = 0.0;
    for (int n = 0; n < N; n++)
        T[n+pad] = t[N-n-1];
}

static av_cold int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ALSFIRSRCContext *s = ctx->priv;

    s->grid_size = 0;
    for (int n = 0; n < s->grid_opt_size; n++)
        s->grid_size += s->grid_opt[n];
    if (s->grid_size >= UINT16_MAX) {
        av_log(ctx, AV_LOG_ERROR, "Sum of the grid density points (%zu) is not smaller or equal than (%d)\n", s->grid_size, UINT16_MAX);
        return AVERROR(EINVAL);
    }

    s->t = av_calloc(s->nb_taps+1, sizeof(*s->t));
    s->T = av_calloc(s->nb_taps+1, sizeof(*s->t));
    s->D = av_calloc(s->grid_size, sizeof(*s->D));
    s->W = av_calloc(s->grid_size, sizeof(*s->W));
    s->om = av_calloc(s->grid_size, sizeof(*s->om));
    s->e1 = av_calloc(s->grid_size, sizeof(*s->e1));
    s->dvec = av_calloc(s->grid_size, sizeof(*s->dvec));
    s->evec = av_calloc(s->grid_size, sizeof(*s->evec));
    s->taps = av_calloc(s->nb_taps, sizeof(*s->taps));
    s->a = av_calloc(s->nb_taps+1, sizeof(*s->a));
    s->b = av_calloc(s->nb_taps+1, sizeof(*s->b));
    if (!s->D || !s->W || !s->om || !s->e1 || !s->dvec ||
        !s->evec || !s->taps || !s->a || !s->b || !s->t || !s->T)
        return AVERROR(ENOMEM);

    {
        const double F = s->sample_rate * 0.5;
        ctype *dvec = s->dvec;
        ctype *evec = s->evec;
        ctype *e1 = s->e1;
        ctype *D = s->D;
        const int L = s->grid_size;
        const int N = s->nb_taps;
        double *x = s->taps;
        ftype *om = s->om;
        ftype *t = s->t;
        ftype *T = s->T;
        ftype *W = s->W;
        ftype *a = s->a;
        ftype *b = s->b;
        unsigned l = 0;
        ftype alpha;
        double prev;

        for (int i = 0; i < FFMIN(s->freq0_opt_size, s->freq1_opt_size); i++) {
            const unsigned grid_idx = FFMIN(i, s->grid_opt_size-1);
            const double freq0 = av_clipd(s->freq0_opt[i], s->freq0_opt[FFMAX(i-1,0)], F);
            const double freq1 = av_clipd(s->freq1_opt[i], s->freq1_opt[FFMAX(i-1,0)], F);
            const int ll = s->grid_opt[grid_idx];

            for (int n = 0; n < ll; n++) {
                om[l+n] = freq0 + (freq1-freq0) * (((double)n) / ll);
                om[l+n] /= F;
            }
            l += ll;
            av_assert0(l <= L);
            prev = freq1;
        }

        for (int n = l; n < L; n++) {
            om[n] = prev + (F-prev) * (((n-l)+0.5) / (L-l));
            om[n] /= F;
        }

        for (int n = 0; n < L; n++)
            av_log(ctx, AV_LOG_DEBUG, "om[%d]=%g\n", n, om[n]);

        l = 0;
        for (int i = 0; i < s->weight_opt_size; i++) {
            const unsigned grid_idx = FFMIN(i, s->grid_opt_size-1);
            const int ll = s->grid_opt[grid_idx];
            double weight = s->weight_opt[i];

            for (int n = 0; n < ll; n++)
                W[l+n] = weight;
            l += ll;
            av_assert0(l <= L);
            prev = weight;
        }

        for (int n = l; n < L; n++)
            W[n] = prev;

        for (int n = 0; n < L; n++)
            av_log(ctx, AV_LOG_DEBUG, "W[%d]=%g\n", n, W[n]);

        l = 0;
        for (int i = 0; i < s->mag_opt_size; i++) {
            const unsigned grid_idx = FFMIN(i, s->grid_opt_size-1);
            const int ll = s->grid_opt[grid_idx];
            double mag = s->mag_opt[i];

            for (int n = 0; n < ll; n++)
                D[l+n].re = mag;
            l += ll;
            av_assert0(l <= L);
            prev = mag;
        }

        for (int n = l; n < L; n++)
            D[n].re = prev;

        for (int n = 0; n < L; n++)
            av_log(ctx, AV_LOG_DEBUG, "mag[%d]=%g\n", n, D[n].re);

        l = 0;
        for (int i = 0; i < s->gdelay_opt_size; i++) {
            const unsigned grid_idx = FFMIN(i, s->grid_opt_size-1);
            const int ll = s->grid_opt[grid_idx];
            double gdelay = s->gdelay_opt[i];

            for (int n = 0; n < ll; n++)
                D[l+n].im = gdelay;
            l += ll;
            av_assert0(l <= L);
            prev = gdelay;
        }

        for (int n = l; n < L; n++)
            D[n].im = prev;

        for (int n = 0; n < L; n++)
            av_log(ctx, AV_LOG_DEBUG, "gdelay[%d]=%g\n", n, D[n].im);

        for (int n = 0; n < L; n++) {
            ftype phase = D[n].im * om[n];
            ftype mag = D[n].re;

            if (n == L-1)
                phase = ceil(phase);
            phase *= M_PI;

            D[n].re =  mag * cos(phase);
            D[n].im = -mag * sin(phase);

            av_log(ctx, AV_LOG_DEBUG, "phase[%d]=%g\n", n, phase / M_PI);
        }

        for (int n = 0; n < L; n++) {
            ftype e = om[n] * M_PI;

            e1[n].re = cos(e);
            e1[n].im = sin(e);

            evec[n].re = 1.0;
            evec[n].im = 0.0;

            dvec[n] = D[n];
        }

        for (int n = 0; n < N; n++) {
            for (int l = 0; l < L; l++) {
                a[n] += W[l] * evec[l].re;
                b[n] += W[l] * dvec[l].re;
            }

            a[n] /= L;
            b[n] /= L;

            av_assert2(isnormal(a[n]));
            av_assert2(isnormal(b[n]));

            for (int l = 0; l < L; l++) {
                cmul(evec[l], e1[l], &evec[l]);
                cmul(dvec[l], e1[l], &dvec[l]);

                av_assert2(isnormal(evec[l].re));
                av_assert2(isnormal(evec[l].im));
                av_assert2(isnormal(dvec[l].re));
                av_assert2(isnormal(dvec[l].im));
            }
        }

        for (int n = 0; n < N; n++)
            av_log(ctx, AV_LOG_DEBUG, "a[%d]=%g\n", n, a[n]);

        for (int n = 0; n < N; n++)
            av_log(ctx, AV_LOG_DEBUG, "b[%d]=%g\n", n, b[n]);

        alpha = a[0];
        x[0] = b[0]/a[0];
        t[0] = 1.0;

        for (int n = 0; n < N-1; n++) {
            ftype k = 0.0;

            for (int i = n+1; i >= 1; i--)
                k += a[i] * t[n+1-i];

            k = -k/alpha;

            alpha *= (1.0 - k*k);
            av_assert2(isnormal(alpha));

            flip(T, t, 1, n+1);
            for (int i = 0; i < n+2; i++)
                t[i] += k*T[i];

            k = 0.0;

            for (int i = n+1; i >= 1; i--)
                k += a[i] * x[n+1-i];

            k = (b[n+1]-k)/alpha;

            flip(T, t, 0, n+2);
            for (int i = 0; i < n+2; i++)
                x[i] += k*T[i];
        }

        for (int n = 0; n < N; n++) {
            if (fabs(x[n]) > 1.0) {
                av_log(ctx, AV_LOG_ERROR, "|x[%d]|=|%g| > 1.0, try increasing grid size\n", n, x[n]);
                return AVERROR(EINVAL);
            }
            av_log(ctx, AV_LOG_DEBUG, "x[%d]=%g\n", n, x[n]);
        }
    }

    s->pts = 0;

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    ALSFIRSRCContext *s = ctx->priv;
    int nb_samples;
    AVFrame *out;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    nb_samples = FFMIN(s->nb_samples, s->nb_taps - s->pts);
    if (nb_samples <= 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (!(out = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);

    memcpy(out->data[0], s->taps + s->pts, nb_samples * sizeof(double));

    out->pts = s->pts;
    s->pts += nb_samples;

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad alsfirsrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_asrc_alsfirsrc = {
    .p.name        = "alsfirsrc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate a FIR coefficients audio stream with Least-Squares method."),
    .p.priv_class  = &alsfirsrc_class,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(ALSFIRSRCContext),
    .p.inputs      = NULL,
    FILTER_OUTPUTS(alsfirsrc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
