/*
 * Copyright (c) 2013 Paul B Mahol
 * Copyright (c) 2006-2008 Rob Sykes <robs@users.sourceforge.net>
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

#include "config_components.h"

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

enum FilterType {
    biquad,
    equalizer,
    bass,
    treble,
    bandpass,
    bandreject,
    allpass,
    highpass,
    lowpass,
    lowshelf,
    highshelf,
    tiltshelf,
    transform,
    peakpass,
};

enum WidthType {
    NONE,
    HERTZ,
    OCTAVE,
    QFACTOR,
    SLOPE,
    KHERTZ,
    NB_WTYPE,
};

enum TransformType {
    DI,
    DII,
    TDI,
    TDII,
    LATT,
    SVF,
    ZDF,
    WDF,
    NB_TTYPE,
};

typedef struct BiquadsContext {
    const AVClass *class;

    enum FilterType filter_type;
    int width_type;
    int order;
    int csg;
    int transform_type;
    int precision;
    int block_samples;

    int bypass;

    double fz, fp;
    double qz, qp;

    double gain;
    double frequency;
    double width;
    double mix;
    double sigma_factor;
    AVChannelLayout ch_layout;
    int normalize;

    double a[3];
    double b[3];

    double oa[3];
    double ob[3];

    AVFrame *block[3];

    void *st;

    int block_align;

    int64_t pts;
    int nb_channels;
    int nb_samples;

    int (*init_state)(AVFilterContext *ctx, void **st,
                      const int nb_channels, const int block_samples, const int reset,
                      const double a[3], const double b[3], const double mix);
    void (*clip_reset)(AVFilterContext *ctx, void *st, const int nb_channels);
    void (*filter)(void *st, const void *ibuf, void *obuf, int len,
                   int ch, int disabled);
} BiquadsContext;

#define CLIP_RESET 1
#define BIQUAD_DI 1
#define BIQUAD_DII 1
#define BIQUAD_TDI 1
#define BIQUAD_TDII 1
#define BIQUAD_LATT 1
#define BIQUAD_SVF 1
#define BIQUAD_WDF 1
#define BIQUAD_ZDF 1

#define DEPTH 8
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 16
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 31
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 32
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 64
#include "biquads_template.c"

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const BiquadsContext *s = ctx->priv;
    static const enum AVSampleFormat auto_sample_fmts[] = {
        AV_SAMPLE_FMT_U8P,
        AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_NONE
    };
    const enum AVSampleFormat *sample_fmts_list = sample_fmts;
    int ret;

    switch (s->precision) {
    case 0:
        sample_fmts[0] = AV_SAMPLE_FMT_S16P;
        break;
    case 1:
        sample_fmts[0] = AV_SAMPLE_FMT_S32P;
        break;
    case 2:
        sample_fmts[0] = AV_SAMPLE_FMT_FLTP;
        break;
    case 3:
        sample_fmts[0] = AV_SAMPLE_FMT_DBLP;
        break;
    case 4:
        sample_fmts[0] = AV_SAMPLE_FMT_U8P;
        break;
    default:
        sample_fmts_list = auto_sample_fmts;
        break;
    }
    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts_list);
    if (ret < 0)
        return ret;

    return 0;
}

static void convert_dir2latt(BiquadsContext *s)
{
    double k0, k1, v0, v1, v2;

    k1 = s->a[2];
    k0 = s->a[1] / (1. + k1);
    v2 = s->b[2];
    v1 = s->b[1] - v2 * s->a[1];
    v0 = s->b[0] - v1 * k0 - v2 * k1;

    s->a[1] = k0;
    s->a[2] = k1;
    s->b[0] = v0;
    s->b[1] = v1;
    s->b[2] = v2;
}

static void convert_dir2svf(BiquadsContext *s)
{
    double a[2];
    double b[3];

    a[0] = -s->a[1];
    a[1] = -s->a[2];
    b[0] = s->b[1] - s->a[1] * s->b[0];
    b[1] = s->b[2] - s->a[2] * s->b[0];
    b[2] = s->b[0];

    s->a[1] = a[0];
    s->a[2] = a[1];
    s->b[0] = b[0];
    s->b[1] = b[1];
    s->b[2] = b[2];
}

static void convert_dir2wdf(BiquadsContext *s)
{
    double a[3];
    double b[3];

    b[0] = s->b[0];
    b[1] = ((s->b[2] - s->b[1] - s->b[0]) * 0.5);
    b[2] = ((s->b[0] - s->b[1] - s->b[2]) * 0.5);
    a[0] = 1.0;
    a[1] = (s->a[2] - s->a[1] - 1.0) * 0.5;
    a[2] = (1.0 - s->a[1] - s->a[2]) * 0.5;

    s->a[0] = a[0];
    s->a[1] = a[1];
    s->a[2] = a[2];
    s->b[0] = b[0];
    s->b[1] = b[1];
    s->b[2] = b[2];
}

static void convert_dir2zdf(BiquadsContext *s)
{
    double a0 = s->b[0];
    double a1 = s->b[1];
    double a2 = s->b[2];
    double b1 = s->a[1];
    double b2 = s->a[2];

    double t1 = sqrt(fabs(-1.0 - b1 - b2));
    double t2 = sqrt(fabs(-1.0 + b1 - b2));
    double g = t1 / t2;
    double k = 2.0 * (1.0 - b2) / (t1 * t2);
    double D = 1.0 + g * (g + k);
    double g0 = 1.0 / D;
    double m0 = (a0 - a1 + a2) / (1.0 - b1 + b2);
    double m1 = 2.0 * (a2 - a0) / (t1 * t2);
    double m2 = (a0 + a1 + a2) / (1.0 + b1 + b2);

    s->a[0] = g;
    s->a[1] = g + k;
    s->a[2] = g0;
    s->b[0] = m0;
    s->b[1] = m1;
    s->b[2] = m2;
}

static double convert_width2qfactor(double width,
                                    double frequency,
                                    double gain,
                                    double sample_rate,
                                    int width_type)
{
    double w0 = 2. * M_PI * frequency / sample_rate;
    double A = ff_exp10(gain / 40.);
    double ret;

    switch (width_type) {
    case NONE:
    case QFACTOR:
        ret = width;
        break;
    case HERTZ:
        ret = frequency / width;
        break;
    case KHERTZ:
        ret = frequency / (width * 1000.);
        break;
    case OCTAVE:
        ret = 1. / (2. * sinh(log(2.) / 2. * width * w0 / sin(w0)));
        break;
    case SLOPE:
        ret = 1. / sqrt((A + 1. / A) * (1. / width - 1.) + 2.);
        break;
    default:
        av_assert0(0);
        break;
    }

    return ret;
}

static double vxy(double x, double y, double s)
{
    return sqrt(x*x*x*x + 2.0*s*s*x*x*(2.0*y*y-1.0) + s*s*s*s);
}

static double kxy(double x, double y, double s)
{
    return x*x*(2.0*y*y-1.0) + s*s;
}

static double rox(double x, double s)
{
    return (M_PI - sqrt(x*x+s*s)) / (M_PI + sqrt(x*x + s*s));
}

static double ro1xy(double x, double y, double s)
{
    const double vxy0 = vxy(x, y, s);

    return (2.0*M_PI*M_PI - 2.0*vxy0) / (M_PI*M_PI + vxy0 + M_PI*M_SQRT2*sqrt(fmax(0.0, vxy0 + kxy(x, y, s))));
}

static double ro2xy(double x, double y, double s)
{
    const double vxy0 = vxy(x, y, s);

    return (M_PI*M_PI + vxy0 - M_PI*M_SQRT2*sqrt(fmax(0.0, vxy0 + kxy(x, y, s)))) / (M_PI*M_PI + vxy0 + M_PI*M_SQRT2*sqrt(fmax(0.0, vxy0 + kxy(x, y, s))));
}

static int config_filter(AVFilterLink *outlink, int reset)
{
    AVFilterContext *ctx    = outlink->src;
    BiquadsContext *s       = ctx->priv;
    AVFilterLink *inlink    = ctx->inputs[0];
    const double gain = s->gain * ((s->filter_type == tiltshelf) + 1.);
    const double g = ff_exp10(gain / 20.0);
    const int sample_rate = inlink->sample_rate;
    const double w = sample_rate / (s->frequency + DBL_EPSILON);
    double qfactor, a1, a2, b1, b2, G, e;
    const double sigma_factor = s->sigma_factor;
    const double fclim = fmin(s->frequency,0.75*sample_rate*0.5);
    const double wlim = sample_rate / fclim;
    const double warp = cos(2.0*M_PI/wlim);
    const double sigma = (sigma_factor >= 0.0) ? sigma_factor : sqrt(wlim*wlim-(1.0+warp)/(1.0-warp)*M_PI*M_PI);

    s->bypass = (((w < 0.) && reset) || (s->width <= 0.)) && (s->filter_type != biquad && s->filter_type != transform);
    if (s->bypass) {
        av_log(ctx, AV_LOG_WARNING, "Invalid frequency and/or width!\n");
        return 0;
    }

    qfactor = convert_width2qfactor(s->width, s->frequency, s->gain, sample_rate, s->width_type);
    e = 1.0 / (2.0 * qfactor);

    switch (s->filter_type) {
    case biquad:
        s->a[0] = s->oa[0];
        s->a[1] = s->oa[1];
        s->a[2] = s->oa[2];
        s->b[0] = s->ob[0];
        s->b[1] = s->ob[1];
        s->b[2] = s->ob[2];
        break;
    case equalizer:
        a1 = ro1xy(w, e * sqrt(g), sigma);
        a2 = ro2xy(w, e * sqrt(g), sigma);
        b1 = ro1xy(w, e * (1.0/sqrt(g)), sigma);
        b2 = ro2xy(w, e * (1.0/sqrt(g)), sigma);
        G = 1.0 / (1.0 + a1 + a2);
        break;
    case bass:
    case tiltshelf:
    case lowshelf:
        if (s->order == 1) {
            a1 = rox(w/sqrt(g), sigma);
            a2 = 0.0;
            b1 = rox(w*sqrt(g), sigma);
            b2 = 0.0;
            G = g / (1.0 + a1);
        } else {
            a1 = ro1xy(w/sqrt(sqrt(g)), e, sigma);
            a2 = ro2xy(w/sqrt(sqrt(g)), e, sigma);
            b1 = ro1xy(w*sqrt(sqrt(g)), e, sigma);
            b2 = ro2xy(w*sqrt(sqrt(g)), e, sigma);
            G = g / (1.0 + a1 + a2);
        }
        break;
    case treble:
    case highshelf:
        if (s->order == 1) {
            a1 = rox(w*sqrt(g), sigma);
            a2 = 0.0;
            b1 = rox(w/sqrt(g), sigma);
            b2 = 0.0;
            G = 1.0 / (1.0 + a1);
        } else {
            a1 = ro1xy(w*sqrt(sqrt(g)), e, sigma);
            a2 = ro2xy(w*sqrt(sqrt(g)), e, sigma);
            b1 = ro1xy(w/sqrt(sqrt(g)), e, sigma);
            b2 = ro2xy(w/sqrt(sqrt(g)), e, sigma);
            G = 1.0 / (1.0 + a1 + a2);
        }
        break;
    case bandpass:
        a1 = rox(0.0, sigma) + -1.0;
        a2 = rox(0.0, sigma) * -1.0;
        b1 = ro1xy(w, e, sigma);
        b2 = ro2xy(w, e, sigma);
        G = (w/(2.0*M_PI)) * ((2.0-s->csg) * e / (1.0 + rox(0.0, sigma)));
        break;
    case bandreject:
        a1 = ro1xy(w, 0.0, sigma);
        a2 = ro2xy(w, 0.0, sigma);
        b1 = ro1xy(w, e, sigma);
        b2 = ro2xy(w, e, sigma);
        G = 1.0 / (1.0 + a1 + a2);
        break;
    case peakpass:
        a1 = ro1xy(w, 0.0, sigma);
        a2 = ro2xy(w, 0.0, sigma);
        b1 = ro1xy(w, e, sigma);
        b2 = ro2xy(w, e, sigma);
        G = 1.0 / (1.0 + a1 + a2);
        break;
    case lowpass:
        if (s->order == 1) {
            a1 = rox(0.0, sigma);
            a2 = 0.0;
            b1 = rox(w, sigma);
            b2 = 0.0;
            G = 1.0 / (1.0 + a1);
        } else {
            a1 = 2.0 * rox(0.0, sigma);
            a2 = rox(0.0, sigma) * rox(0.0, sigma);
            b1 = ro1xy(w, e, sigma);
            b2 = ro2xy(w, e, sigma);
            G = 1.0 / (1.0 + a1 + a2);
        }
        break;
    case highpass:
        if (s->order == 1) {
            a1 = -1.0;
            a2 = 0.0;
            b1 = rox(w, sigma);
            b2 = 0.0;
            G = w / (2.0 * M_PI);
        } else {
            a1 = -2.0;
            a2 = 1.0;
            b1 = ro1xy(w, e, sigma);
            b2 = ro2xy(w, e, sigma);
            G = w*w/(4.0*M_PI*M_PI);
        }
        break;
    case allpass:
        if (s->order == 1) {
            a1 = 1.0 / rox(w, sigma);
            a2 = 0.0;
            b1 = rox(w, sigma);
            b2 = 0.0;
            G = 1.0 / (1.0 + a1);
        } else {
            a1 = ro1xy(w, e, sigma) / ro2xy(w, e, sigma);
            a2 = 1.0 / ro2xy(w, e, sigma);
            b1 = ro1xy(w, e, sigma);
            b2 = ro2xy(w, e, sigma);
            G = 1.0 / (1.0 + a1 + a2);
        }
        break;
    case transform:
        {
            const double fs = sample_rate;
            const double fz = s->fz, qz = s->qz;
            const double fp = s->fp, qp = s->qp;

            const double w0z = 2.0*M_PI*fz / fs, w0p = 2.0*M_PI*fp / fs;
            const double cos_w0z = cos(w0z), cos_w0p = cos(w0p);
            const double alpha_z = sin(w0z) / (2.0*qz), alpha_p = sin(w0p) / (2.0*qp);
            const double kz = 2.0/(1.0+cos_w0z), kp = 2.0/(1.0+cos_w0p);

            s->b[0] = ( 1.0 + alpha_z) * kz;
            s->b[1] = (-2.0 * cos_w0z) * kz;
            s->b[2] = ( 1.0 - alpha_z) * kz;
            s->a[0] = ( 1.0 + alpha_p) * kp;
            s->a[1] = (-2.0 * cos_w0p) * kp;
            s->a[2] = ( 1.0 - alpha_p) * kp;
        }
        break;
    default:
        av_assert0(0);
    }

    if (s->filter_type != biquad &&
        s->filter_type != transform) {
        s->a[0] = 1.0;
        s->a[1] = b1;
        s->a[2] = b2;
        s->b[0] = G * (1.0 + b1 + b2) * 1.0;
        s->b[1] = G * (1.0 + b1 + b2) * a1;
        s->b[2] = G * (1.0 + b1 + b2) * a2;
    }

    av_log(ctx, AV_LOG_VERBOSE, "a=%f %f %f:b=%f %f %f\n",
           s->a[0], s->a[1], s->a[2],
           s->b[0], s->b[1], s->b[2]);

    s->a[1] /= s->a[0];
    s->a[2] /= s->a[0];
    s->b[0] /= s->a[0];
    s->b[1] /= s->a[0];
    s->b[2] /= s->a[0];
    s->a[0] /= s->a[0];

    if (s->normalize && fabs(s->b[0] + s->b[1] + s->b[2]) > 1e-6) {
        double factor = (s->a[0] + s->a[1] + s->a[2]) /
                        (s->b[0] + s->b[1] + s->b[2]);

        s->b[0] *= factor;
        s->b[1] *= factor;
        s->b[2] *= factor;
    }

    switch (s->filter_type) {
    case tiltshelf:
        s->b[0] /= g;
        s->b[1] /= g;
        s->b[2] /= g;
        break;
    }

    if (reset && s->block_samples > 0) {
        for (int i = 0; i < 3; i++) {
            if (!s->block[i])
                s->block[i] = ff_get_audio_buffer(outlink, s->block_samples * 2);
            if (!s->block[i])
                return AVERROR(ENOMEM);
            av_samples_set_silence(s->block[i]->extended_data, 0, s->block_samples * 2,
                                   s->block[i]->ch_layout.nb_channels, s->block[i]->format);
        }
    }

    switch (s->transform_type) {
    case DI:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_di_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_di_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_di_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_di_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_di_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case DII:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_dii_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_dii_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_dii_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_dii_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_dii_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case TDI:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_tdi_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_tdi_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_tdi_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_tdi_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_tdi_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case TDII:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_tdii_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_tdii_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_tdii_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_tdii_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_tdii_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case LATT:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_latt_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_latt_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_latt_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_latt_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_latt_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case SVF:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_svf_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_svf_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_svf_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_svf_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_svf_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case WDF:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_wdf_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_wdf_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_wdf_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_wdf_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_wdf_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    case ZDF:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_U8P:
            s->filter = biquad_zdf_u8p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->filter = biquad_zdf_s16p;
            break;
        case AV_SAMPLE_FMT_S32P:
            s->filter = biquad_zdf_s32p;
            break;
        case AV_SAMPLE_FMT_FLTP:
            s->filter = biquad_zdf_fltp;
            break;
        case AV_SAMPLE_FMT_DBLP:
            s->filter = biquad_zdf_dblp;
            break;
        default: av_assert0(0);
        }
        break;
    default:
        av_assert0(0);
    }

    s->block_align = av_get_bytes_per_sample(inlink->format);

    if (s->transform_type == LATT)
        convert_dir2latt(s);
    else if (s->transform_type == SVF)
        convert_dir2svf(s);
    else if (s->transform_type == WDF)
        convert_dir2wdf(s);
    else if (s->transform_type == ZDF)
        convert_dir2zdf(s);

    return s->init_state(ctx, &s->st, outlink->ch_layout.nb_channels,
                         s->block_samples, reset, s->a, s->b, s->mix);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    BiquadsContext *s = ctx->priv;

    s->nb_channels = outlink->ch_layout.nb_channels;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_U8P:
        s->init_state = init_biquad_u8p;
        s->clip_reset = clip_reset_u8p;
        break;
    case AV_SAMPLE_FMT_S16P:
        s->init_state = init_biquad_s16p;
        s->clip_reset = clip_reset_s16p;
        break;
    case AV_SAMPLE_FMT_S32P:
        s->init_state = init_biquad_s32p;
        s->clip_reset = clip_reset_s32p;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->init_state = init_biquad_fltp;
        s->clip_reset = clip_reset_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_state = init_biquad_dblp;
        s->clip_reset = clip_reset_dblp;
        break;
    default:
        av_assert0(0);
    }

    return config_filter(outlink, 1);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static void reverse_samples(AVFrame *out, AVFrame *in, int p,
                            int oo, int io, int nb_samples)
{
    switch (out->format) {
    case AV_SAMPLE_FMT_U8P: {
        const uint8_t *src = ((const uint8_t *)in->extended_data[p]) + io;
        uint8_t *dst = ((uint8_t *)out->extended_data[p]) + oo;
        for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
            dst[i] = src[j];
    }
        break;
    case AV_SAMPLE_FMT_S16P: {
        const int16_t *src = ((const int16_t *)in->extended_data[p]) + io;
        int16_t *dst = ((int16_t *)out->extended_data[p]) + oo;
        for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
            dst[i] = src[j];
    }
        break;
    case AV_SAMPLE_FMT_S32P: {
        const int32_t *src = ((const int32_t *)in->extended_data[p]) + io;
        int32_t *dst = ((int32_t *)out->extended_data[p]) + oo;
        for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
            dst[i] = src[j];
    }
        break;
    case AV_SAMPLE_FMT_FLTP: {
        const float *src = ((const float *)in->extended_data[p]) + io;
        float *dst = ((float *)out->extended_data[p]) + oo;
        for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
            dst[i] = src[j];
    }
        break;
    case AV_SAMPLE_FMT_DBLP: {
        const double *src = ((const double *)in->extended_data[p]) + io;
        double *dst = ((double *)out->extended_data[p]) + oo;
        for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
            dst[i] = src[j];
    }
        break;
    }
}

static int filter_channel(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFilterLink *inlink = ctx->inputs[0];
    ThreadData *td = arg;
    AVFrame *buf = td->in;
    AVFrame *out_buf = td->out;
    BiquadsContext *s = ctx->priv;
    const int start = (buf->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (buf->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int disabled = ff_filter_disabled(ctx);

    for (int ch = start; ch < end; ch++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&inlink->ch_layout, ch);

        if (av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0) {
            if (buf != out_buf)
                memcpy(out_buf->extended_data[ch], buf->extended_data[ch],
                       buf->nb_samples * s->block_align);
            continue;
        }

        if (!s->block_samples) {
            s->filter(s->st, buf->extended_data[ch], out_buf->extended_data[ch], buf->nb_samples,
                      ch, disabled);
        } else {
            s->filter(s->st, buf->extended_data[ch], s->block[0]->extended_data[ch] + s->block_align * s->block_samples,
                      buf->nb_samples, ch, disabled);
            memset(s->block[0]->extended_data[ch] + s->block_align * (s->block_samples + buf->nb_samples),
                   0, (s->block_samples - buf->nb_samples) * s->block_align);
            reverse_samples(s->block[1], s->block[0], ch, 0, 0, 2 * s->block_samples);
            s->filter(s->st, s->block[1]->extended_data[ch], s->block[1]->extended_data[ch], 2 * s->block_samples,
                      s->nb_channels+ch, disabled);
            reverse_samples(s->block[2], s->block[1], ch, 0, 0, 2 * s->block_samples);
            memcpy(out_buf->extended_data[ch], s->block[2]->extended_data[ch],
                   s->block_samples * s->block_align);
            memcpy(s->block[0]->extended_data[ch], s->block[0]->extended_data[ch] + s->block_align * s->block_samples,
                   s->block_samples * s->block_align);
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AVFilterContext  *ctx = inlink->dst;
    BiquadsContext *s     = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out_buf;
    ThreadData td;
    int drop = 0;

    if (s->bypass)
        return ff_filter_frame(outlink, buf);

    if (av_frame_is_writable(buf) && s->block_samples == 0) {
        out_buf = buf;
    } else {
        out_buf = ff_get_audio_buffer(outlink, s->block_samples > 0 ? s->block_samples : buf->nb_samples);
        if (!out_buf) {
            av_frame_free(&buf);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out_buf, buf);
    }

    if (s->block_samples > 0 && s->pts == AV_NOPTS_VALUE)
        drop = 1;
    td.in = buf;
    td.out = out_buf;
    ff_filter_execute(ctx, filter_channel, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    s->clip_reset(ctx, s->st, outlink->ch_layout.nb_channels);

    if (s->block_samples > 0) {
        int nb_samples = buf->nb_samples;
        int64_t pts = buf->pts;

        out_buf->pts = s->pts;
        out_buf->nb_samples = s->nb_samples;
        s->pts = pts;
        s->nb_samples = nb_samples;
    }

    if (buf != out_buf)
        ff_graph_frame_free(ctx, &buf);

    if (!drop)
        return ff_filter_frame(outlink, out_buf);
    else {
        av_frame_free(&out_buf);
        ff_filter_set_ready(ctx, 10);
        return 0;
    }
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    BiquadsContext *s = ctx->priv;
    AVFrame *in = NULL;
    int64_t pts;
    int status;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (s->block_samples > 0) {
        ret = ff_inlink_consume_samples(inlink, s->block_samples, s->block_samples, &in);
    } else {
        ret = ff_inlink_consume_frame(inlink, &in);
    }
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    if (s->block_samples > 0 && ff_inlink_queued_samples(inlink) >= s->block_samples) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->block_samples > 0) {
            in = ff_get_audio_buffer(outlink, s->block_samples);
            if (!in)
                return AVERROR(ENOMEM);

            ret = filter_frame(inlink, in);
        }

        ff_outlink_set_status(outlink, status, pts);

        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_filter(outlink, 0);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    BiquadsContext *s = ctx->priv;

    for (int i = 0; i < 3; i++)
        av_frame_free(&s->block[i]);
    av_freep(&s->st);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(BiquadsContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

#define DEFINE_BIQUAD_FILTER_2(name_, description_, priv_class_)        \
static av_cold int name_##_init(AVFilterContext *ctx)                   \
{                                                                       \
    BiquadsContext *s = ctx->priv;                                      \
    s->filter_type = name_;                                             \
    s->pts = AV_NOPTS_VALUE;                                            \
    return 0;                                                           \
}                                                                       \
                                                         \
const FFFilter ff_af_##name_ = {                         \
    .p.name        = #name_,                             \
    .p.description = NULL_IF_CONFIG_SMALL(description_), \
    .p.priv_class  = &priv_class_##_class,               \
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS |       \
                     AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL, \
    .priv_size     = sizeof(BiquadsContext),             \
    .init          = name_##_init,                       \
    .activate      = activate,                           \
    .uninit        = uninit,                             \
    FILTER_INPUTS(ff_audio_default_filterpad),           \
    FILTER_OUTPUTS(outputs),                             \
    FILTER_QUERY_FUNC2(query_formats),                   \
    .process_command = process_command,                  \
}

#define DEFINE_BIQUAD_FILTER(name, description)                         \
    AVFILTER_DEFINE_CLASS(name);                                        \
    DEFINE_BIQUAD_FILTER_2(name, description, name)

#define MATCH_FACTOR_OPTION(x)                                                                                   \
    {"match_factor", "set match factor", OFFSET(sigma_factor), AV_OPT_TYPE_DOUBLE, {.dbl=x}, -1.0, M_PI, FLAGS}, \
    {"s",            "set match factor", OFFSET(sigma_factor), AV_OPT_TYPE_DOUBLE, {.dbl=x}, -1.0, M_PI, FLAGS}

#define WIDTH_OPTION(x)                                                                   \
    {"width", "set width", OFFSET(width), AV_OPT_TYPE_DOUBLE, {.dbl=x}, 0, 99999, FLAGS}, \
    {"w",     "set width", OFFSET(width), AV_OPT_TYPE_DOUBLE, {.dbl=x}, 0, 99999, FLAGS}

#define ORDER_OPTION(x)                                                                \
    {"order", "set filter order", OFFSET(order), AV_OPT_TYPE_INT, {.i64=x}, 1, 2, AF}, \
    {"o",     "set filter order", OFFSET(order), AV_OPT_TYPE_INT, {.i64=x}, 1, 2, AF}

#define WIDTH_TYPE_OPTION(x)                                                                                                                \
    {"width_type", "set filter-width type", OFFSET(width_type), AV_OPT_TYPE_INT, {.i64=x}, HERTZ, NB_WTYPE-1, FLAGS, .unit = "width_type"}, \
    {"t",          "set filter-width type", OFFSET(width_type), AV_OPT_TYPE_INT, {.i64=x}, HERTZ, NB_WTYPE-1, FLAGS, .unit = "width_type"}, \
    {"h", "Hz", 0, AV_OPT_TYPE_CONST, {.i64=HERTZ}, 0, 0, FLAGS, .unit = "width_type"},                                                     \
    {"q", "Q-Factor", 0, AV_OPT_TYPE_CONST, {.i64=QFACTOR}, 0, 0, FLAGS, .unit = "width_type"},                                             \
    {"o", "octave", 0, AV_OPT_TYPE_CONST, {.i64=OCTAVE}, 0, 0, FLAGS, .unit = "width_type"},                                                \
    {"s", "slope", 0, AV_OPT_TYPE_CONST, {.i64=SLOPE}, 0, 0, FLAGS, .unit = "width_type"},                                                  \
    {"k", "kHz", 0, AV_OPT_TYPE_CONST, {.i64=KHERTZ}, 0, 0, FLAGS, .unit = "width_type"}

#define MIX_CHANNELS_NORMALIZE_OPTION(x, y, z)                                                                \
    {"mix", "set mix", OFFSET(mix), AV_OPT_TYPE_DOUBLE, {.dbl=x}, 0, 1, FLAGS},                               \
    {"m",   "set mix", OFFSET(mix), AV_OPT_TYPE_DOUBLE, {.dbl=x}, 0, 1, FLAGS},                               \
    {"channels", "set channels to filter", OFFSET(ch_layout),  AV_OPT_TYPE_CHLAYOUT, {.str=y}, 0, 0, FLAGS},  \
    {"c",        "set channels to filter", OFFSET(ch_layout),  AV_OPT_TYPE_CHLAYOUT, {.str=y}, 0, 0, FLAGS},  \
    {"normalize", "normalize coefficients", OFFSET(normalize), AV_OPT_TYPE_BOOL, {.i64=z}, 0, 1, FLAGS},      \
    {"n",         "normalize coefficients", OFFSET(normalize), AV_OPT_TYPE_BOOL, {.i64=z}, 0, 1, FLAGS}

#define TRANSFORM_OPTION(x)                                                                                                              \
    {"transform", "set transform type", OFFSET(transform_type), AV_OPT_TYPE_INT, {.i64=x}, 0, NB_TTYPE-1, AF, .unit = "transform_type"}, \
    {"a",         "set transform type", OFFSET(transform_type), AV_OPT_TYPE_INT, {.i64=x}, 0, NB_TTYPE-1, AF, .unit = "transform_type"}, \
    {"di",   "direct form I",  0, AV_OPT_TYPE_CONST, {.i64=DI}, 0, 0, AF, .unit = "transform_type"},                                     \
    {"dii",  "direct form II", 0, AV_OPT_TYPE_CONST, {.i64=DII}, 0, 0, AF, .unit = "transform_type"},                                    \
    {"tdi",  "transposed direct form I",  0, AV_OPT_TYPE_CONST, {.i64=TDI},  0, 0, AF, .unit = "transform_type"},                        \
    {"tdii", "transposed direct form II", 0, AV_OPT_TYPE_CONST, {.i64=TDII}, 0, 0, AF, .unit = "transform_type"},                        \
    {"latt", "lattice-ladder form", 0, AV_OPT_TYPE_CONST, {.i64=LATT}, 0, 0, AF, .unit = "transform_type"},                              \
    {"svf",  "state variable filter form", 0, AV_OPT_TYPE_CONST, {.i64=SVF}, 0, 0, AF, .unit = "transform_type"},                        \
    {"zdf",  "zero-delay filter form", 0, AV_OPT_TYPE_CONST, {.i64=ZDF}, 0, 0, AF, .unit = "transform_type"},                            \
    {"wdf",  "wave digital filter form", 0, AV_OPT_TYPE_CONST, {.i64=WDF}, 0, 0, AF, .unit = "transform_type"}

#define PRECISION_OPTION(x)                                                                                                 \
    {"precision", "set filtering precision", OFFSET(precision), AV_OPT_TYPE_INT, {.i64=x}, -1, 4, AF, .unit = "precision"}, \
    {"r",         "set filtering precision", OFFSET(precision), AV_OPT_TYPE_INT, {.i64=x}, -1, 4, AF, .unit = "precision"}, \
    {"auto", "automatic",            0, AV_OPT_TYPE_CONST, {.i64=-1}, 0, 0, AF, .unit = "precision"},                       \
    {"u8",  "unsigned 8-bit",        0, AV_OPT_TYPE_CONST, {.i64=4},  0, 0, AF, .unit = "precision"},                       \
    {"s16", "signed 16-bit",         0, AV_OPT_TYPE_CONST, {.i64=0},  0, 0, AF, .unit = "precision"},                       \
    {"s32", "signed 32-bit",         0, AV_OPT_TYPE_CONST, {.i64=1},  0, 0, AF, .unit = "precision"},                       \
    {"f32", "floating-point single", 0, AV_OPT_TYPE_CONST, {.i64=2},  0, 0, AF, .unit = "precision"},                       \
    {"f64", "floating-point double", 0, AV_OPT_TYPE_CONST, {.i64=3},  0, 0, AF, .unit = "precision"}

#define BLOCKSIZE_OPTION(x)                                                                              \
    {"blocksize", "set the block size", OFFSET(block_samples), AV_OPT_TYPE_INT, {.i64=x}, 0, 32768, AF}, \
    {"b",         "set the block size", OFFSET(block_samples), AV_OPT_TYPE_INT, {.i64=x}, 0, 32768, AF}

#if CONFIG_EQUALIZER_FILTER
static const AVOption equalizer_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(1.0),
    {"gain", "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    {"g",    "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(equalizer, "Apply two-pole peaking equalization (EQ) filter.");
#endif  /* CONFIG_EQUALIZER_FILTER */
#if CONFIG_BASS_FILTER || CONFIG_LOWSHELF_FILTER
static const AVOption bass_lowshelf_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.5),
    {"gain", "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    {"g",    "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    ORDER_OPTION(2),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

AVFILTER_DEFINE_CLASS_EXT(bass_lowshelf, "bass/lowshelf", bass_lowshelf_options);
#if CONFIG_BASS_FILTER
DEFINE_BIQUAD_FILTER_2(bass, "Boost or cut lower frequencies.", bass_lowshelf);
#endif  /* CONFIG_BASS_FILTER */

#if CONFIG_LOWSHELF_FILTER
DEFINE_BIQUAD_FILTER_2(lowshelf, "Apply a low shelf filter.", bass_lowshelf);
#endif  /* CONFIG_LOWSHELF_FILTER */
#endif  /* CONFIG_BASS_FILTER || CONFIG LOWSHELF_FILTER */
#if CONFIG_TREBLE_FILTER || CONFIG_HIGHSHELF_FILTER || CONFIG_TILTSHELF_FILTER
static const AVOption treble_highshelf_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.5),
    {"gain", "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    {"g",    "set gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -900, 900, FLAGS},
    ORDER_OPTION(2),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

AVFILTER_DEFINE_CLASS_EXT(treble_highshelf, "treble/high/tiltshelf",
                          treble_highshelf_options);

#if CONFIG_TREBLE_FILTER
DEFINE_BIQUAD_FILTER_2(treble, "Boost or cut upper frequencies.", treble_highshelf);
#endif  /* CONFIG_TREBLE_FILTER */

#if CONFIG_HIGHSHELF_FILTER
DEFINE_BIQUAD_FILTER_2(highshelf, "Apply a high shelf filter.", treble_highshelf);
#endif  /* CONFIG_HIGHSHELF_FILTER */

#if CONFIG_TILTSHELF_FILTER
DEFINE_BIQUAD_FILTER_2(tiltshelf, "Apply a tilt shelf filter.", treble_highshelf);
#endif
#endif  /* CONFIG_TREBLE_FILTER || CONFIG_HIGHSHELF_FILTER || CONFIG_TILTSHELF_FILTER */

#if CONFIG_BANDPASS_FILTER
static const AVOption bandpass_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.5),
    {"csg",   "use constant skirt gain", OFFSET(csg), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS},
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(bandpass, "Apply a two-pole Butterworth band-pass filter.");
#endif  /* CONFIG_BANDPASS_FILTER */
#if CONFIG_BANDREJECT_FILTER
static const AVOption bandreject_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.5),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(bandreject, "Apply a two-pole Butterworth band-reject filter.");
#endif  /* CONFIG_BANDREJECT_FILTER */
#if CONFIG_LOWPASS_FILTER
static const AVOption lowpass_options[] = {
    {"frequency", "set frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=500}, 0, 999999, FLAGS},
    {"f",         "set frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=500}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.707),
    ORDER_OPTION(2),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(lowpass, "Apply a low-pass filter with 3dB point frequency.");
#endif  /* CONFIG_LOWPASS_FILTER */
#if CONFIG_HIGHPASS_FILTER
static const AVOption highpass_options[] = {
    {"frequency", "set frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.707),
    ORDER_OPTION(2),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(highpass, "Apply a high-pass filter with 3dB point frequency.");
#endif  /* CONFIG_HIGHPASS_FILTER */
#if CONFIG_ALLPASS_FILTER
static const AVOption allpass_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.707),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    ORDER_OPTION(2),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(allpass, "Apply an all-pass filter.");
#endif  /* CONFIG_ALLPASS_FILTER */
#if CONFIG_BIQUAD_FILTER
static const AVOption biquad_options[] = {
    {"a0", NULL, OFFSET(oa[0]), AV_OPT_TYPE_DOUBLE, {.dbl=1}, INT32_MIN, INT32_MAX, FLAGS},
    {"a1", NULL, OFFSET(oa[1]), AV_OPT_TYPE_DOUBLE, {.dbl=0}, INT32_MIN, INT32_MAX, FLAGS},
    {"a2", NULL, OFFSET(oa[2]), AV_OPT_TYPE_DOUBLE, {.dbl=0}, INT32_MIN, INT32_MAX, FLAGS},
    {"b0", NULL, OFFSET(ob[0]), AV_OPT_TYPE_DOUBLE, {.dbl=0}, INT32_MIN, INT32_MAX, FLAGS},
    {"b1", NULL, OFFSET(ob[1]), AV_OPT_TYPE_DOUBLE, {.dbl=0}, INT32_MIN, INT32_MAX, FLAGS},
    {"b2", NULL, OFFSET(ob[2]), AV_OPT_TYPE_DOUBLE, {.dbl=0}, INT32_MIN, INT32_MAX, FLAGS},
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(biquad, "Apply a biquad IIR filter with the given coefficients.");
#endif  /* CONFIG_BIQUAD_FILTER */
#if CONFIG_TRANSFORM_FILTER
static const AVOption transform_options[] = {
    {"fz", "source frequency",      OFFSET(fz), AV_OPT_TYPE_DOUBLE, {.dbl=1000}, 0, 999999, FLAGS},
    {"qz", "source Q-Factor",       OFFSET(qz), AV_OPT_TYPE_DOUBLE, {.dbl=1},    0, 99999,  FLAGS},
    {"fp", "destination frequency", OFFSET(fp), AV_OPT_TYPE_DOUBLE, {.dbl=1000}, 0, 999999, FLAGS},
    {"qp", "destination Q-Factor",  OFFSET(qp), AV_OPT_TYPE_DOUBLE, {.dbl=1},    0, 99999,  FLAGS},
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(transform, "Apply a Linkwitz transform IIR filter with the given Hz/Q.");
#endif  /* CONFIG_TRANSFORM_FILTER */

#if CONFIG_PEAKPASS_FILTER
static const AVOption peakpass_options[] = {
    {"frequency", "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    {"f",         "set central frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=3000}, 0, 999999, FLAGS},
    WIDTH_TYPE_OPTION(QFACTOR),
    WIDTH_OPTION(0.5),
    MIX_CHANNELS_NORMALIZE_OPTION(1, "24c", 0),
    TRANSFORM_OPTION(DI),
    PRECISION_OPTION(-1),
    BLOCKSIZE_OPTION(0),
    MATCH_FACTOR_OPTION(-1.0),
    {NULL}
};

DEFINE_BIQUAD_FILTER(peakpass, "Apply a peak-pass filter.");
#endif  /* CONFIG_PEAKPASS_FILTER */
