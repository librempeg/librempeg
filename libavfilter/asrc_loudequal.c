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
 * Loudness compensation FIR coefficients audio source
 */

#include <inttypes.h>
#include <stdio.h>

#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"

enum {
    INTERP_LINEAR,
    INTERP_CUBIC,
    INTERP_SPLINE,
    NB_INTERP
};

static const float iso_freq[] = {
    20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000, 20000
};

static const float iso_curves[][31] = {
    { 76.5517, 65.6189, 55.1228, 45.5340, 37.6321, 30.8650, 25.0238, 20.51, 16.6458, 13.1160, 10.0883, 7.5436, 5.1137, 3.0589, 1.4824, 0.3029, -0.3026, -0.0103, 1.0335, -1.1863, -4.1116, -7.0462, -9.0260, -8.4944, -4.4829, 3.2817, 9.8291, 10.4757, 8.3813, 14.10, 79.65 },
    { 83.75, 75.7579, 68.2089, 61.1365, 54.9638, 49.0098, 43.2377, 38.1338, 33.4772, 28.7734, 24.8417, 21.3272, 18.0522, 15.1379, 12.9768, 11.1791, 9.9918, 9.9996, 11.2621, 10.4291, 7.2744, 4.4508, 3.0404, 3.7961, 7.4583, 14.3483, 20.9841, 23.4306, 22.3269, 25.17, 81.47 },
    { 89.5781, 82.6513, 75.9764, 69.6171, 64.0178, 58.5520, 53.1898, 48.3809, 43.9414, 39.3702, 35.5126, 31.9922, 28.6866, 25.6703, 23.4263, 21.4825, 20.1011, 20.0052, 21.4618, 21.4013, 18.1515, 15.3844, 14.2559, 15.1415, 18.6349, 25.0196, 31.5227, 34.4256, 33.0444, 34.67, 84.18 },
    { 99.8539, 93.9444, 88.1659, 82.6287, 77.7849, 73.0825, 68.4779, 64.3711, 60.5855, 56.7022, 53.4087, 50.3992, 47.5775, 44.9766, 43.0507, 41.3392, 40.0618, 40.01, 41.8195, 42.5076, 39.2296, 36.5090, 35.6089, 36.6492, 40.0077, 45.8283, 51.7968, 54.2841, 51.4859, 51.96, 92.77 },
    { 109.5113, 104.2279, 99.0779, 94.1773, 89.9635, 85.9434, 82.0534, 78.6546, 75.5635, 72.4743, 69.8643, 67.5348, 65.3917, 63.4510, 62.0512, 60.8150, 59.8867, 60.0116, 62.1549, 63.1894, 59.9616, 57.2552, 56.4239, 57.5699, 60.8882, 66.3613, 71.6640, 73.1551, 68.6308, 68.43, 104.92 },
    { 118.99, 114.2326, 109.6457, 105.3367, 101.7214, 98.3618, 95.1729, 92.4797, 90.0892, 87.8162, 85.9166, 84.3080, 82.8934, 81.6786, 80.8634, 80.1736, 79.6691, 80.0121, 82.4834, 83.7408, 80.5867, 77.8847, 77.0748, 78.3124, 81.6182, 86.8087, 91.4062, 91.7361, 85.4068, 84.67, 118.95 },
    { 128.41, 124.15, 120.11, 116.38, 113.35, 110.65, 108.16, 106.17, 104.48, 103.03, 101.85, 100.97, 100.3, 99.83, 99.62, 99.5, 99.44, 100.01, 102.81, 104.25, 101.18, 98.48, 97.67, 99, 102.3, 107.23, 111.11, 110.23, 102.07, 100.83, 133.73 },
};

static const float iso_phons[] = { 0, 10, 20, 40, 60, 80, 100 };
static const float inter_phons[] = { 30, 50, 70, 90 };

typedef struct LoudEqualContext {
    const AVClass  *class;

    int     sample_rate;
    int     nb_samples;
    float   attenuated;
    float   reference;
    int     nb_taps;
    int     interp;
    int64_t pts;

    float inter_curves[4][31];

    float reference_curve[31];
    float attenuated_curve[31];
    float db_diff_curve[31];

    AVComplexFloat *spline;
    AVComplexFloat *kernel;

    AVTXContext    *itx_ctx;
    av_tx_fn        itx_fn;
} LoudEqualContext;

#define OFFSET(x) offsetof(LoudEqualContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption loudequal_options[] = {
    { "attenuated",  "set attenuated volume", OFFSET(attenuated), AV_OPT_TYPE_FLOAT, {.dbl =  72},  50, 100, FLAGS },
    { "a",           "set attenuated volume", OFFSET(attenuated), AV_OPT_TYPE_FLOAT, {.dbl =  72},  50, 100, FLAGS },
    { "reference",   "set reference volume",  OFFSET(reference),  AV_OPT_TYPE_FLOAT, {.dbl =  82},  50, 100, FLAGS },
    { "e",           "set reference volume",  OFFSET(reference),  AV_OPT_TYPE_FLOAT, {.dbl =  82},  50, 100, FLAGS },
    { "sample_rate", "set sample rate",    OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64 = 44100}, 1, INT_MAX, FLAGS },
    { "r",           "set sample rate",    OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64 = 44100}, 1, INT_MAX, FLAGS },
    { "nb_samples",  "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "n",           "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "nb_taps",     "set the number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64 = 4096}, 1024, 65535, FLAGS },
    { "t",           "set the number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64 = 4096}, 1024, 65535, FLAGS },
    { "interp","set the interpolation", OFFSET(interp), AV_OPT_TYPE_INT, {.i64=0}, 0, NB_INTERP-1, FLAGS, .unit = "interp" },
    { "i",     "set the interpolation", OFFSET(interp), AV_OPT_TYPE_INT, {.i64=0}, 0, NB_INTERP-1, FLAGS, .unit = "interp" },
    { "linear", NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_LINEAR}, 0, 0, FLAGS, .unit = "interp" },
    { "cubic",  NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_CUBIC},  0, 0, FLAGS, .unit = "interp" },
    { "spline", NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_SPLINE}, 0, 0, FLAGS, .unit = "interp" },
    { NULL }
};

static void gen_inter_curves(AVFilterContext *ctx)
{
    LoudEqualContext *s = ctx->priv;

    for (int n = 0; n < FF_ARRAY_ELEMS(inter_phons); n++) {
        const float phon = inter_phons[n];
        const float lower_phon = phon - 10.f;
        const float upper_phon = phon + 10.f;
        int lower_idx = 0, upper_idx = 0;
        const float weight = 0.5f;

        for (int i = 0; i < FF_ARRAY_ELEMS(iso_phons); i++) {
            if (iso_phons[i] == lower_phon)
                lower_idx = i;

            if (iso_phons[i] == upper_phon)
                upper_idx = i;
        }

        for (int i = 0; i < FF_ARRAY_ELEMS(iso_curves[0]); i++) {
            float lower_curve = iso_curves[lower_idx][i];
            float upper_curve = iso_curves[upper_idx][i];
            float interpolated_curve = lower_curve * (1.f - weight) + upper_curve * weight;

            s->inter_curves[n][i] = interpolated_curve;
        }
    }
}

static void gen_curves(AVFilterContext *ctx, const float phon, float *out)
{
    LoudEqualContext *s = ctx->priv;
    const float lower_phon = floorf(phon / 10.f) * 10.f;
    const float upper_phon = ceilf(phon / 10.f) * 10.f;
    const float *lower_curve;
    const float *upper_curve;
    float weight;

    for (int i = 0; i < FF_ARRAY_ELEMS(iso_phons); i++) {
        if (iso_phons[i] == lower_phon)
            lower_curve = iso_curves[i];

        if (iso_phons[i] == upper_phon)
            upper_curve = iso_curves[i];
    }

    for (int i = 0; i < FF_ARRAY_ELEMS(inter_phons); i++) {
        if (inter_phons[i] == lower_phon)
            lower_curve = s->inter_curves[i];

        if (inter_phons[i] == upper_phon)
            upper_curve = s->inter_curves[i];
    }

    weight = (phon - lower_phon) / 10.f;
    for (int i = 0; i < FF_ARRAY_ELEMS(iso_curves[0]); i++)
        out[i] = lower_curve[i] * (1.f - weight) + upper_curve[i] * weight;
}

static void eq_interp(AVComplexFloat *complexf,
                      const float *freq,
                      const float *magnitude,
                      int m, int interp, int minterp,
                      const float factor)
{
    for (int i = 0; i < minterp; i++) {
        for (int j = 0; j < m; j++) {
            const float x = factor * i;

            if (x <= freq[j+1]) {
                float g = 1.f;

                if (interp == INTERP_LINEAR) {
                    const float d  = freq[j+1] - freq[j];
                    const float d0 = x - freq[j];
                    const float d1 = freq[j+1] - x;
                    const float g0 = magnitude[j];
                    const float g1 = magnitude[j+1];

                    if (d0 && d1) {
                        g = (d0 * g1 + d1 * g0) / d;
                    } else if (d0) {
                        g = g1;
                    } else {
                        g = g0;
                    }
                } else if (interp == INTERP_CUBIC) {
                    if (x <= freq[j]) {
                        g = magnitude[j];
                    } else {
                        float x1, x2, x3;
                        float a, b, c, d;
                        float m0, m1, m2, msum;
                        const float unit = freq[j+1] - freq[j];

                        m0 = j != 0 ? unit * (magnitude[j] - magnitude[j-1]) / (freq[j] - freq[j-1]) : 0;
                        m1 = magnitude[j+1] - magnitude[j];
                        m2 = j != minterp - 1 ? unit * (magnitude[j+2] - magnitude[j+1]) / (freq[j+2] - freq[j+1]) : 0;

                        msum = fabsf(m0) + fabsf(m1);
                        m0 = msum > 0.f ? (fabsf(m0) * m1 + fabsf(m1) * m0) / msum : 0.f;
                        msum = fabsf(m1) + fabsf(m2);
                        m1 = msum > 0.f ? (fabsf(m1) * m2 + fabsf(m2) * m1) / msum : 0.f;

                        d = magnitude[j];
                        c = m0;
                        b = 3.f * magnitude[j+1] - m1 - 2.f * c - 3.f * d;
                        a = magnitude[j+1] - b - c - d;

                        x1 = (x - freq[j]) / unit;
                        x2 = x1 * x1;
                        x3 = x2 * x1;

                        g = a * x3 + b * x2 + c * x1 + d;
                    }
                } else if (interp == INTERP_SPLINE) {
                    const float d = freq[j+1] - freq[j];
                    const float u = (x - freq[j]) / d;
                    const float c0 = magnitude[j];
                    const float c1 = 1/2.f*(magnitude[j+1]-magnitude[j-1]);
                    const float c2 = magnitude[j-1] - 5/2.f*magnitude[j] + 2*magnitude[j+1] - 1/2.f*magnitude[j+2];
                    const float c3 = 1/2.f*(magnitude[j+2]-magnitude[j-1]) + 3/2.f*(magnitude[j]-magnitude[j+1]);

                    g = ((c3 * u + c2) * u + c1) * u + c0;
                }

                complexf[i].re = g;
                complexf[i].im = 0;
                complexf[minterp * 2 - i - 1].re = g;
                complexf[minterp * 2 - i - 1].im = 0;

                break;
            }
        }
    }
}

static av_cold int init(AVFilterContext *ctx)
{
    LoudEqualContext *s = ctx->priv;
    const int M = 1 << av_ceil_log2(s->nb_taps);
    float factor, reference_db_diff, scale = 1.f;
    int freq_idx = 0, L = M * 2, ret;

    gen_inter_curves(ctx);
    gen_curves(ctx, s->attenuated, s->attenuated_curve);
    gen_curves(ctx, s->reference, s->reference_curve);

    for (int i = 0; i < FF_ARRAY_ELEMS(iso_curves[0]); i++)
        s->db_diff_curve[i] = s->reference_curve[i] - s->attenuated_curve[i];

    for (int i = 0; i < FF_ARRAY_ELEMS(iso_freq); i++) {
        if (iso_freq[i] == 1000.f)
            freq_idx = i;
    }

    reference_db_diff = s->db_diff_curve[freq_idx];

    ret = av_tx_init(&s->itx_ctx, &s->itx_fn, AV_TX_FLOAT_FFT, 1, L, &scale, 0);
    if (ret < 0)
        return ret;

    s->spline = av_calloc(L, sizeof(*s->spline));
    s->kernel = av_calloc(L, sizeof(*s->kernel));
    if (!s->spline || !s->kernel)
        return AVERROR(ENOMEM);

    factor = FFMIN(s->sample_rate * 0.5f, iso_freq[FF_ARRAY_ELEMS(iso_freq) - 1]) / (float)s->nb_taps;
    eq_interp(s->spline, iso_freq, s->db_diff_curve, FF_ARRAY_ELEMS(iso_freq), s->interp, s->nb_taps, factor);

    for (int i = 0; i < L; i++)
        s->spline[i].re = ff_exp10f((reference_db_diff - s->spline[i].re) / 20.f);

    s->itx_fn(s->itx_ctx, s->kernel, s->spline, sizeof(*s->kernel));

    {
        const int mid = s->nb_taps/2;

        s->spline[0].re = 0.f;
        for (int i = 0; i < mid; i++) {
            s->spline[mid-i].re = s->kernel[i].re / L;
            s->spline[mid+i].re = s->kernel[i].re / L;
        }
        s->spline[s->nb_taps-1].re = 0.f;
    }

    av_tx_uninit(&s->itx_ctx);
    av_freep(&s->kernel);

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const LoudEqualContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLT,
                                                       AV_SAMPLE_FMT_NONE };

    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    LoudEqualContext *s = ctx->priv;
    AVFrame *frame;
    int nb_samples;
    float *out;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    nb_samples = FFMIN(s->nb_samples, s->nb_taps - s->pts);
    if (nb_samples <= 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    frame = ff_get_audio_buffer(outlink, nb_samples);
    if (!frame)
        return AVERROR(ENOMEM);

    out = (float *)frame->extended_data[0];
    for (int i = s->pts; i < s->pts + nb_samples; i++)
        out[i-s->pts] = s->spline[i].re;

    frame->pts = s->pts;
    s->pts    += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    LoudEqualContext *s = ctx->priv;

    outlink->sample_rate = s->sample_rate;
    s->pts = 0;

    return 0;
}

static void uninit(AVFilterContext *ctx)
{
    LoudEqualContext *s = ctx->priv;

    av_freep(&s->spline);
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

AVFILTER_DEFINE_CLASS(loudequal);

FFFilter ff_asrc_loudequal = {
    .p.name        = "loudequal",
    .p.description = NULL_IF_CONFIG_SMALL("Generate loudness compensation FIR coefficients."),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    .priv_size     = sizeof(LoudEqualContext),
    .p.priv_class  = &loudequal_class,
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
