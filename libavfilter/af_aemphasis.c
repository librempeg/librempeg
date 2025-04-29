/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen, Damien Zammit and others
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
#include "avfilter.h"
#include "filters.h"
#include "audio.h"

enum FilterType {
    COL,
    EMI,
    BSI,
    RIAA,
    CD,
    FM50,
    FM75,
    IECN78,
    TELDEC,
    LONDON,
    NARTB,
    BL300,
    NB_TYPES
};

typedef struct BiquadCoeffs {
    double b0, b1, b2, a1, a2;
} BiquadCoeffs;

typedef struct RIAACurve {
    BiquadCoeffs r1;
} RIAACurve;

typedef struct AudioEmphasisContext {
    const AVClass *class;
    int mode, type;
    double level_in, level_out;

    RIAACurve rc;

    AVFrame *w;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioEmphasisContext;

#define OFFSET(x) offsetof(AudioEmphasisContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aemphasis_options[] = {
    { "level_in",      "set input gain", OFFSET(level_in),  AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 64, FLAGS },
    { "level_out",    "set output gain", OFFSET(level_out), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 64, FLAGS },
    { "mode",         "set filter mode", OFFSET(mode), AV_OPT_TYPE_INT,   {.i64=0}, 0, 1, FLAGS, .unit = "mode" },
    { "reproduction",              NULL,            0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, FLAGS, .unit = "mode" },
    { "production",                NULL,            0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, FLAGS, .unit = "mode" },
    { "type",         "set filter type", OFFSET(type), AV_OPT_TYPE_INT,   {.i64=CD}, 0, NB_TYPES-1, FLAGS, .unit = "type" },
    { "col",                 "Columbia",            0, AV_OPT_TYPE_CONST, {.i64=COL}, 0, 0, FLAGS, .unit = "type" },
    { "emi",                      "EMI",            0, AV_OPT_TYPE_CONST, {.i64=EMI}, 0, 0, FLAGS, .unit = "type" },
    { "bsi",              "BSI (78RPM)",            0, AV_OPT_TYPE_CONST, {.i64=BSI}, 0, 0, FLAGS, .unit = "type" },
    { "riaa",                    "RIAA",            0, AV_OPT_TYPE_CONST, {.i64=RIAA},0, 0, FLAGS, .unit = "type" },
    { "cd",         "Compact Disc (CD)",            0, AV_OPT_TYPE_CONST, {.i64=CD},  0, 0, FLAGS, .unit = "type" },
    { "50fm",               "50µs (FM)",            0, AV_OPT_TYPE_CONST, {.i64=FM50},0, 0, FLAGS, .unit = "type" },
    { "75fm",               "75µs (FM)",            0, AV_OPT_TYPE_CONST, {.i64=FM75},0, 0, FLAGS, .unit = "type" },
    { "iecn78",               "IEC N78",            0, AV_OPT_TYPE_CONST, {.i64=IECN78},0, 0,FLAGS,.unit = "type" },
    { "teldec",                "TELDEC",            0, AV_OPT_TYPE_CONST, {.i64=TELDEC},0, 0,FLAGS,.unit = "type" },
    { "london",                "LONDON",            0, AV_OPT_TYPE_CONST, {.i64=LONDON},0, 0,FLAGS,.unit = "type" },
    { "nartb",                  "NARTB",            0, AV_OPT_TYPE_CONST, {.i64=NARTB}, 0, 0,FLAGS,.unit = "type" },
    { "bl300",           "Blumlein 300",            0, AV_OPT_TYPE_CONST, {.i64=BL300}, 0, 0,FLAGS,.unit = "type" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aemphasis);

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 32
#include "aemphasis_template.c"

#undef DEPTH
#define DEPTH 64
#include "aemphasis_template.c"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioEmphasisContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

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

    td.in = in; td.out = out;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (in != out)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static double freq_gain(BiquadCoeffs *c, double freq, double sr)
{
    double w = 2.0*M_PI*freq/sr;
    double b0 = c->b0;
    double b1 = c->b1;
    double b2 = c->b2;
    double a0 = 1.0;
    double a1 = c->a1;
    double a2 = c->a2;
    double numerator = b0*b0 + b1*b1 + b2*b2 + 2.0*(b0*b1 + b1*b2)*cos(w) + 2.0*b0*b2*cos(2.0*w);
    double denominator = a0*a0 + a1*a1 + a2*a2 + 2.0*(a0*a1 + a1*a2)*cos(w) + 2.0*a0*a2*cos(2.0*w);

    return sqrt(numerator / denominator);
}

static int config_input(AVFilterLink *inlink)
{
    double i, j, k, b0, b1, b2, a1, a2, tau1, tau2, tau3, nf;
    double gain, sr = inlink->sample_rate;
    AVFilterContext *ctx = inlink->dst;
    AudioEmphasisContext *s = ctx->priv;
    BiquadCoeffs coeffs;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    if (!s->w)
        s->w = ff_get_audio_buffer(inlink, 2);
    if (!s->w)
        return AVERROR(ENOMEM);

    nf = 1000.0;

    switch (s->type) {
    case COL: //"Columbia"
        tau1 = 0.001590;
        tau2 = 0.000318;
        tau3 = 0.000100;
        break;
    case EMI: //"EMI"
        tau1 = 0.002500;
        tau2 = 0.000500;
        tau3 = 0.000070;
        break;
    case BSI: //"BSI(78rpm)"
        tau1 = 0.003180;
        tau2 = 0.000353;
        tau3 = 0.000050;
        break;
    case IECN78: //"IEC N78"
        tau1 = 0.003180;
        tau2 = 0.000450;
        tau3 = 0.000050;
        break;
    case TELDEC: //"TELDEC"
        tau1 = 0.003180;
        tau2 = 0.000318;
        tau3 = 0.000050;
        break;
    case LONDON: //"LONDON"
        tau1 = 0.001590;
        tau2 = 0.000318;
        tau3 = 0.000050;
        break;
    case NARTB: //"NARTB"
        tau1 = 0.003180;
        tau2 = 0.000318;
        tau3 = 0.000100;
        break;
    case BL300: //"Blumlein 300"
        tau1 = 0.000000;
        tau2 = 0.000531;
        tau3 = 0.000000;
        break;
    case RIAA: //"RIAA"
    default:
        tau1 = 0.003180;
        tau2 = 0.000318;
        tau3 = 0.000075;
        break;
    case CD: //"CD Mastering"
        tau1 = 0.000050;
        tau2 = 0.000015;
        tau3 = 0.000000;
        nf = 100.0;
        break;
    case FM50: //"50µs FM (Europe)"
        tau1 = 0.000050;
        tau2 = 0.000000;
        tau3 = 0.000000;
        nf = 100.0;
        break;
    case FM75: //"75µs FM (US)"
        tau1 = 0.000075;
        tau2 = 0.000000;
        tau3 = 0.000000;
        nf = 100.0;
        break;
    }

    i = (tau1 > 0.0) ? -exp(-1.0/(sr*tau1)) : 0.0;
    j = (tau2 > 0.0) ? -exp(-1.0/(sr*tau2)) : 0.0;
    k = (tau3 > 0.0) ? -exp(-1.0/(sr*tau3)) : 0.0;

    a1 = j;
    a2 = 0.0;
    b0 = 1.0;
    b1 = i + k;
    b2 = i * k;

    if (!s->mode) {
        FFSWAP(double, a1, b1);
        FFSWAP(double, a2, b2);
    }

    coeffs.b0 = b0;
    coeffs.b1 = b1;
    coeffs.b2 = b2;
    coeffs.a1 = a1;
    coeffs.a2 = a2;

    gain = 1.0 / freq_gain(&coeffs, nf, sr);
    // divide one filter's x[n-m] coefficients by that value
    s->rc.r1.b0 = coeffs.b0 * gain;
    s->rc.r1.b1 = coeffs.b1 * gain;
    s->rc.r1.b2 = coeffs.b2 * gain;
    s->rc.r1.a1 = coeffs.a1;
    s->rc.r1.a2 = coeffs.a2;

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    return config_input(ctx->inputs[0]);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioEmphasisContext *s = ctx->priv;

    av_frame_free(&s->w);
}

static const AVFilterPad aemphasis_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_aemphasis = {
    .p.name        = "aemphasis",
    .p.description = NULL_IF_CONFIG_SMALL("Audio emphasis."),
    .p.priv_class  = &aemphasis_class,
    .priv_size     = sizeof(AudioEmphasisContext),
    .uninit        = uninit,
    FILTER_INPUTS(aemphasis_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_FLTP),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
