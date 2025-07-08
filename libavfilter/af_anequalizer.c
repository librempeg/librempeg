/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen and others
 * Copyright (c) 2015 Paul B Mahol
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

#include "libavutil/avstring.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"

#define FILTER_ORDER 4

enum FilterType {
    BUTTERWORTH,
    CHEBYSHEV1,
    CHEBYSHEV2,
    NB_TYPES
};

typedef struct FoHist {
    double num[4];
    double denum[4];
} FoHist;

typedef struct FoSection {
    double a0, a1, a2, a3, a4;
    double b0, b1, b2, b3, b4;
} FoSection;

typedef struct EqualizatorFilter {
    AVChannelLayout *channel;
    int ignore;
    int type;

    double freq;
    double gain;
    double width;

    FoSection section[2];
} EqualizatorFilter;

typedef struct AudioNEqualizerContext {
    const AVClass *class;

    double *freq;
    unsigned nb_freq;

    double *width;
    unsigned nb_width;

    double *gain;
    unsigned nb_gain;

    int *type;
    unsigned nb_type;

    AVChannelLayout *channel;
    unsigned nb_channel;

    int nb_filters;
    int nb_allocated;
    EqualizatorFilter *filters;
    FoHist *history;
} AudioNEqualizerContext;

#define OFFSET(x) offsetof(AudioNEqualizerContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define R AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_freq = {.def="500",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_width = {.def="10",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_gain = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_type = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_channel = {.def="24c",.size_min=1,.sep=' '};

static const AVOption anequalizer_options[] = {
    { "freq",  "set the freq values per filter",  OFFSET(freq),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_freq},  0,   INT_MAX, (A|F|R) },
    { "width", "set the width values per filter", OFFSET(width), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_width}, 0,   INT_MAX, (A|F|R) },
    { "gain",  "set the gain values per filter",  OFFSET(gain),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_gain},  0,        10, (A|F|R) },
    { "type",  "set the type values per filter",  OFFSET(type),  AV_OPT_TYPE_INT|AR,    {.arr=&def_type},  0,NB_TYPES-1, (A|F|R), "type" },
    {   "butterworth",  NULL,  0, AV_OPT_TYPE_CONST, {.i64=BUTTERWORTH}, 0, 0, (A|F|R), "type"},
    {   "chebyshev1",   NULL,  0, AV_OPT_TYPE_CONST, {.i64=CHEBYSHEV1},  0, 0, (A|F|R), "type"},
    {   "chebyshev2",   NULL,  0, AV_OPT_TYPE_CONST, {.i64=CHEBYSHEV2},  0, 0, (A|F|R), "type"},
    { "channel", "set the channels values per filter",  OFFSET(channel),  AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_channel}, 0, 0, (A|F|R) },
    { NULL }
};

AVFILTER_DEFINE_CLASS(anequalizer);

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioNEqualizerContext *s = ctx->priv;

    av_freep(&s->history);
    av_freep(&s->filters);
    s->nb_filters = 0;
    s->nb_allocated = 0;
}

static void butterworth_fo_section(FoSection *S, double beta,
                                   double si, double g, double g0,
                                   double D, double c0)
{
    if (c0 == 1 || c0 == -1) {
        S->b0 = (g*g*beta*beta + 2*g*g0*si*beta + g0*g0)/D;
        S->b1 = 2*c0*(g*g*beta*beta - g0*g0)/D;
        S->b2 = (g*g*beta*beta - 2*g0*g*beta*si + g0*g0)/D;
        S->b3 = 0;
        S->b4 = 0;

        S->a0 = 1;
        S->a1 = 2*c0*(beta*beta - 1)/D;
        S->a2 = (beta*beta - 2*beta*si + 1)/D;
        S->a3 = 0;
        S->a4 = 0;
    } else {
        S->b0 = (g*g*beta*beta + 2*g*g0*si*beta + g0*g0)/D;
        S->b1 = -4*c0*(g0*g0 + g*g0*si*beta)/D;
        S->b2 = 2*(g0*g0*(1 + 2*c0*c0) - g*g*beta*beta)/D;
        S->b3 = -4*c0*(g0*g0 - g*g0*si*beta)/D;
        S->b4 = (g*g*beta*beta - 2*g*g0*si*beta + g0*g0)/D;

        S->a0 = 1;
        S->a1 = -4*c0*(1 + si*beta)/D;
        S->a2 = 2*(1 + 2*c0*c0 - beta*beta)/D;
        S->a3 = -4*c0*(1 - si*beta)/D;
        S->a4 = (beta*beta - 2*si*beta + 1)/D;
    }
}

static void butterworth_bp_filter(EqualizatorFilter *f,
                                  int N, double w0, double wb,
                                  double G, double Gb, double G0)
{
    double g, c0, g0, beta;
    double epsilon;
    int r =  N % 2;
    int L = (N - r) / 2;
    int i;

    if (G == 0 && G0 == 0) {
        f->section[0].a0 = 1;
        f->section[0].b0 = 1;
        f->section[1].a0 = 1;
        f->section[1].b0 = 1;
        return;
    }

    G  = ff_exp10(G/20);
    Gb = ff_exp10(Gb/20);
    G0 = ff_exp10(G0/20);

    epsilon = sqrt((G * G - Gb * Gb) / (Gb * Gb - G0 * G0));
    g  = pow(G,  1.0 / N);
    g0 = pow(G0, 1.0 / N);
    beta = pow(epsilon, -1.0 / N) * tan(wb/2);
    c0 = cos(w0);

    for (i = 1; i <= L; i++) {
        double ui = (2.0 * i - 1) / N;
        double si = sin(M_PI * ui / 2.0);
        double Di = beta * beta + 2 * si * beta + 1;

        butterworth_fo_section(&f->section[i - 1], beta, si, g, g0, Di, c0);
    }
}

static void chebyshev1_fo_section(FoSection *S, double a,
                                  double c, double tetta_b,
                                  double g0, double si, double b,
                                  double D, double c0)
{
    if (c0 == 1 || c0 == -1) {
        S->b0 = (tetta_b*tetta_b*(b*b+g0*g0*c*c) + 2*g0*b*si*tetta_b*tetta_b + g0*g0)/D;
        S->b1 = 2*c0*(tetta_b*tetta_b*(b*b+g0*g0*c*c) - g0*g0)/D;
        S->b2 = (tetta_b*tetta_b*(b*b+g0*g0*c*c) - 2*g0*b*si*tetta_b + g0*g0)/D;
        S->b3 = 0;
        S->b4 = 0;

        S->a0 = 1;
        S->a1 = 2*c0*(tetta_b*tetta_b*(a*a+c*c) - 1)/D;
        S->a2 = (tetta_b*tetta_b*(a*a+c*c) - 2*a*si*tetta_b + 1)/D;
        S->a3 = 0;
        S->a4 = 0;
    } else {
        S->b0 = ((b*b + g0*g0*c*c)*tetta_b*tetta_b + 2*g0*b*si*tetta_b + g0*g0)/D;
        S->b1 = -4*c0*(g0*g0 + g0*b*si*tetta_b)/D;
        S->b2 = 2*(g0*g0*(1 + 2*c0*c0) - (b*b + g0*g0*c*c)*tetta_b*tetta_b)/D;
        S->b3 = -4*c0*(g0*g0 - g0*b*si*tetta_b)/D;
        S->b4 = ((b*b + g0*g0*c*c)*tetta_b*tetta_b - 2*g0*b*si*tetta_b + g0*g0)/D;

        S->a0 = 1;
        S->a1 = -4*c0*(1 + a*si*tetta_b)/D;
        S->a2 = 2*(1 + 2*c0*c0 - (a*a + c*c)*tetta_b*tetta_b)/D;
        S->a3 = -4*c0*(1 - a*si*tetta_b)/D;
        S->a4 = ((a*a + c*c)*tetta_b*tetta_b - 2*a*si*tetta_b + 1)/D;
    }
}

static void chebyshev1_bp_filter(EqualizatorFilter *f,
                                 int N, double w0, double wb,
                                 double G, double Gb, double G0)
{
    double a, b, c0, g0, alfa, beta, tetta_b;
    double epsilon;
    int r =  N % 2;
    int L = (N - r) / 2;
    int i;

    if (G == 0 && G0 == 0) {
        f->section[0].a0 = 1;
        f->section[0].b0 = 1;
        f->section[1].a0 = 1;
        f->section[1].b0 = 1;
        return;
    }

    G  = ff_exp10(G/20);
    Gb = ff_exp10(Gb/20);
    G0 = ff_exp10(G0/20);

    epsilon = sqrt((G*G - Gb*Gb) / (Gb*Gb - G0*G0));
    g0 = pow(G0,1.0/N);
    alfa = pow(1.0/epsilon    + sqrt(1 + 1/(epsilon*epsilon)), 1.0/N);
    beta = pow(G/epsilon + Gb * sqrt(1 + 1/(epsilon*epsilon)), 1.0/N);
    a = 0.5 * (alfa - 1.0/alfa);
    b = 0.5 * (beta - g0*g0*(1/beta));
    tetta_b = tan(wb/2);
    c0 = cos(w0);

    for (i = 1; i <= L; i++) {
        double ui = (2.0*i-1.0)/N;
        double ci = cos(M_PI*ui/2.0);
        double si = sin(M_PI*ui/2.0);
        double Di = (a*a + ci*ci)*tetta_b*tetta_b + 2.0*a*si*tetta_b + 1;

        chebyshev1_fo_section(&f->section[i - 1], a, ci, tetta_b, g0, si, b, Di, c0);
    }
}

static void chebyshev2_fo_section(FoSection *S, double a,
                                  double c, double tetta_b,
                                  double g, double si, double b,
                                  double D, double c0)
{
    if (c0 == 1 || c0 == -1) {
        S->b0 = (g*g*tetta_b*tetta_b + 2*tetta_b*g*b*si + b*b + g*g*c*c)/D;
        S->b1 = 2*c0*(g*g*tetta_b*tetta_b - b*b - g*g*c*c)/D;
        S->b2 = (g*g*tetta_b*tetta_b - 2*tetta_b*g*b*si + b*b + g*g*c*c)/D;
        S->b3 = 0;
        S->b4 = 0;

        S->a0 = 1;
        S->a1 = 2*c0*(tetta_b*tetta_b - a*a - c*c)/D;
        S->a2 = (tetta_b*tetta_b - 2*tetta_b*a*si + a*a + c*c)/D;
        S->a3 = 0;
        S->a4 = 0;
    } else {
        S->b0 = (g*g*tetta_b*tetta_b + 2*g*b*si*tetta_b + b*b + g*g*c*c)/D;
        S->b1 = -4*c0*(b*b + g*g*c*c + g*b*si*tetta_b)/D;
        S->b2 = 2*((b*b + g*g*c*c)*(1 + 2*c0*c0) - g*g*tetta_b*tetta_b)/D;
        S->b3 = -4*c0*(b*b + g*g*c*c - g*b*si*tetta_b)/D;
        S->b4 = (g*g*tetta_b*tetta_b - 2*g*b*si*tetta_b + b*b + g*g*c*c)/D;

        S->a0 = 1;
        S->a1 = -4*c0*(a*a + c*c + a*si*tetta_b)/D;
        S->a2 = 2*((a*a + c*c)*(1 + 2*c0*c0) - tetta_b*tetta_b)/D;
        S->a3 = -4*c0*(a*a + c*c - a*si*tetta_b)/D;
        S->a4 = (tetta_b*tetta_b - 2*a*si*tetta_b + a*a + c*c)/D;
    }
}

static void chebyshev2_bp_filter(EqualizatorFilter *f,
                                 int N, double w0, double wb,
                                 double G, double Gb, double G0)
{
    double a, b, c0, tetta_b;
    double epsilon, g, eu, ew;
    int r =  N % 2;
    int L = (N - r) / 2;
    int i;

    if (G == 0 && G0 == 0) {
        f->section[0].a0 = 1;
        f->section[0].b0 = 1;
        f->section[1].a0 = 1;
        f->section[1].b0 = 1;
        return;
    }

    G  = ff_exp10(G/20);
    Gb = ff_exp10(Gb/20);
    G0 = ff_exp10(G0/20);

    epsilon = sqrt((G*G - Gb*Gb) / (Gb*Gb - G0*G0));
    g  = pow(G, 1.0 / N);
    eu = pow(epsilon + sqrt(1 + epsilon*epsilon), 1.0/N);
    ew = pow(G0*epsilon + Gb*sqrt(1 + epsilon*epsilon), 1.0/N);
    a = (eu - 1.0/eu)/2.0;
    b = (ew - g*g/ew)/2.0;
    tetta_b = tan(wb/2);
    c0 = cos(w0);

    for (i = 1; i <= L; i++) {
        double ui = (2.0 * i - 1.0)/N;
        double ci = cos(M_PI * ui / 2.0);
        double si = sin(M_PI * ui / 2.0);
        double Di = tetta_b*tetta_b + 2*a*si*tetta_b + a*a + ci*ci;

        chebyshev2_fo_section(&f->section[i - 1], a, ci, tetta_b, g, si, b, Di, c0);
    }
}

static double butterworth_compute_bw_gain_db(double gain)
{
    double bw_gain = 0;

    if (gain <= -6)
        bw_gain = gain + 3;
    else if(gain > -6 && gain < 6)
        bw_gain = gain * 0.5;
    else if(gain >= 6)
        bw_gain = gain - 3;

    return bw_gain;
}

static double chebyshev1_compute_bw_gain_db(double gain)
{
    double bw_gain = 0;

    if (gain <= -6)
        bw_gain = gain + 1;
    else if(gain > -6 && gain < 6)
        bw_gain = gain * 0.9;
    else if(gain >= 6)
        bw_gain = gain - 1;

    return bw_gain;
}

static double chebyshev2_compute_bw_gain_db(double gain)
{
    double bw_gain = 0;

    if (gain <= -6)
        bw_gain = -3;
    else if(gain > -6 && gain < 6)
        bw_gain = gain * 0.3;
    else if(gain >= 6)
        bw_gain = 3;

    return bw_gain;
}

static inline double hz_2_rad(double x, double fs)
{
    return 2 * M_PI * x / fs;
}

static void equalizer(EqualizatorFilter *f, double sample_rate)
{
    double w0 = hz_2_rad(f->freq,  sample_rate);
    double wb = hz_2_rad(f->width, sample_rate);
    double bw_gain;

    switch (f->type) {
    case BUTTERWORTH:
        bw_gain = butterworth_compute_bw_gain_db(f->gain);
        butterworth_bp_filter(f, FILTER_ORDER, w0, wb, f->gain, bw_gain, 0);
        break;
    case CHEBYSHEV1:
        bw_gain = chebyshev1_compute_bw_gain_db(f->gain);
        chebyshev1_bp_filter(f, FILTER_ORDER, w0, wb, f->gain, bw_gain, 0);
        break;
    case CHEBYSHEV2:
        bw_gain = chebyshev2_compute_bw_gain_db(f->gain);
        chebyshev2_bp_filter(f, FILTER_ORDER, w0, wb, f->gain, bw_gain, 0);
        break;
    }
}

static int add_filter(AudioNEqualizerContext *s, AVFilterLink *inlink)
{
    equalizer(&s->filters[s->nb_filters], inlink->sample_rate);
    if (s->nb_filters >= s->nb_allocated - 1) {
        EqualizatorFilter *filters;

        filters = av_calloc(s->nb_allocated, 2 * sizeof(*s->filters));
        if (!filters)
            return AVERROR(ENOMEM);
        memcpy(filters, s->filters, sizeof(*s->filters) * s->nb_allocated);
        av_free(s->filters);
        s->filters = filters;
        s->nb_allocated *= 2;
    }
    s->nb_filters++;

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioNEqualizerContext *s = ctx->priv;
    int ret = 0, nb_args;

    nb_args = FFMIN3(FFMIN(s->nb_freq, s->nb_width), FFMIN(s->nb_gain, s->nb_type), s->nb_channel);
    s->nb_allocated = nb_args;
    s->history = av_calloc(nb_args * inlink->ch_layout.nb_channels * 2, sizeof(*s->history));
    s->filters = av_calloc(nb_args, sizeof(*s->filters));
    if (!s->filters || !s->history) {
        s->nb_allocated = 0;
        return AVERROR(ENOMEM);
    }

    for (int n = 0; n < nb_args; n++) {
        s->filters[s->nb_filters].freq = s->freq[n];
        s->filters[s->nb_filters].gain = 20.*log10(s->gain[n]);
        s->filters[s->nb_filters].width = s->width[n];
        s->filters[s->nb_filters].type = s->type[n];
        s->filters[s->nb_filters].channel = &s->channel[n];

        if (s->filters[s->nb_filters].freq < 0 ||
            s->filters[s->nb_filters].freq > inlink->sample_rate / 2.0)
            s->filters[s->nb_filters].ignore = 1;

        ret = add_filter(s, inlink);
        if (ret < 0)
            break;
    }

    return ret;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret = AVERROR(ENOSYS);

    return ret;
}

static inline double section_process(const FoSection *S, FoHist *H, double in)
{
    double out;

    out = S->b0 * in;
    out+= S->b1 * H->num[0] - H->denum[0] * S->a1;
    out+= S->b2 * H->num[1] - H->denum[1] * S->a2;
    out+= S->b3 * H->num[2] - H->denum[2] * S->a3;
    out+= S->b4 * H->num[3] - H->denum[3] * S->a4;

    H->num[3] = H->num[2];
    H->num[2] = H->num[1];
    H->num[1] = H->num[0];
    H->num[0] = in;

    H->denum[3] = H->denum[2];
    H->denum[2] = H->denum[1];
    H->denum[1] = H->denum[0];
    H->denum[0] = out;

    return out;
}

static double process_sample(const FoSection *s1, FoHist *h, double in)
{
    double p0 = in, p1;

    for (int i = 0; i < FILTER_ORDER / 2; i++) {
        p1 = section_process(&s1[i], &h[i], p0);
        p0 = p1;
    }

    return p1;
}

static int filter_channels(AVFilterContext *ctx, void *arg,
                           int jobnr, int nb_jobs)
{
    AudioNEqualizerContext *s = ctx->priv;
    AVFrame *buf = arg;
    const int start = (buf->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (buf->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&buf->ch_layout, ch);

        for (int i = 0; i < s->nb_filters; i++) {
            EqualizatorFilter *f = &s->filters[i];
            const int bypass = av_channel_layout_index_from_channel(f->channel, channel) < 0;
            FoHist *h = &s->history[i + s->nb_filters * ch * 2];
            double *bptr;

            if (f->gain == 0.0 || f->ignore || bypass)
                continue;

            bptr = (double *)buf->extended_data[ch];
            for (int n = 0; n < buf->nb_samples; n++) {
                double sample = bptr[n];

                sample  = process_sample(f->section, h, sample);
                bptr[n] = sample;
            }
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];

    if (!ff_filter_disabled(ctx))
        ff_filter_execute(ctx, filter_channels, buf, NULL,
                          FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    return ff_filter_frame(outlink, buf);
}

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .flags          = AVFILTERPAD_FLAG_NEEDS_WRITABLE,
        .config_props   = config_input,
        .filter_frame   = filter_frame,
    },
};

const FFFilter ff_af_anequalizer = {
    .p.name        = "anequalizer",
    .p.description = NULL_IF_CONFIG_SMALL("Apply high-order audio parametric multi band equalizer."),
    .p.priv_class  = &anequalizer_class,
    .priv_size     = sizeof(AudioNEqualizerContext),
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
