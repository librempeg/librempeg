/*
 * Copyright (c) 2022 Paul B Mahol
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
#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/cpu.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"
#include "video.h"
#include "avfilter.h"

enum FrequencyScale {
    FSCALE_LINEAR,
    FSCALE_LOG,
    FSCALE_BARK,
    FSCALE_MEL,
    FSCALE_ERBS,
    FSCALE_SQRT,
    FSCALE_CBRT,
    FSCALE_QDRT,
    FSCALE_FM,
    FSCALE_GREENWOOD,
    NB_FSCALE
};

enum IntensityScale {
    ISCALE_LOG,
    ISCALE_LINEAR,
    ISCALE_SQRT,
    ISCALE_CBRT,
    ISCALE_QDRT,
    NB_ISCALE
};

enum FrequencyWeight {
    WEIGHTING_0,
    WEIGHTING_A,
    WEIGHTING_B,
    WEIGHTING_C,
    WEIGHTING_D,
    NB_WEIGHT
};

enum DataMode    { MAGNITUDE, PHASE, DELAY, NB_DATA };
enum ChannelMode { COMBINED, SEPARATE, NB_CMODES };
enum DisplayMode { LINE, BAR, DOT, NB_MODE };

typedef struct ShowFreqsContext {
    const AVClass *class;
    int w, h;
    int mode;
    int data_mode;
    uint32_t *colors;
    unsigned nb_colors;
    AVRational frame_rate;
    AVTXContext **fft, **ifft;
    av_tx_fn tx_fn, itx_fn;
    int fft_size, ifft_size;
    int channel_mode;
    char *ch_layout_str;
    int64_t pts;
    int64_t old_pts;
    int weighting_type;
    float *frequency_weight;
    float *frequency_band;
    AVComplexFloat **kernel;
    unsigned *index;
    int *kernel_start, *kernel_stop;
    AVFrame *cache;
    AVFrame *fft_in;
    AVFrame *fft_out;
    AVFrame *dst_x;
    AVFrame *src_x;
    AVFrame *ifft_in;
    AVFrame *ifft_out;
    AVFrame *ch_out;
    AVFrame *over;
    AVFrame *yh_out;
    int nb_threads;
    int nb_channels;
    int nb_draw_channels;
    AVChannelLayout ch_layout;
    int nb_consumed_samples;
    int hop_size, ihop_size;
    int hop_index, ihop_index;
    int input_padding_size, output_padding_size;
    int input_sample_count, output_sample_count;
    int frequency_band_count;
    float logarithmic_basis;
    int intensity_scale;
    int frequency_scale;
    float minimum_frequency, maximum_frequency;
    float minimum_intensity, maximum_intensity;
    float deviation;
    uint8_t *bypass;

    AVFloatDSPContext *fdsp;
} ShowFreqsContext;

#define OFFSET(x) offsetof(ShowFreqsContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_colors = {.def="red|green|blue|yellow|orange|lime|pink|magenta|brown",.size_min=1,.sep='|'};

static const AVOption showfreqs_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "640x512"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "640x512"}, 0, 0, FLAGS },
    { "rate", "set video rate",  OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate",  OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, FLAGS },
    { "scale", "set frequency scale", OFFSET(frequency_scale), AV_OPT_TYPE_INT,  {.i64=0}, 0, NB_FSCALE-1, FLAGS, .unit="scale" },
    {  "linear",  "linear",           0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_LINEAR}, 0, 0, FLAGS, .unit="scale" },
    {  "log",     "logarithmic",      0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_LOG},    0, 0, FLAGS, .unit="scale" },
    {  "bark",    "bark",             0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_BARK},   0, 0, FLAGS, .unit="scale" },
    {  "mel",     "mel",              0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_MEL},    0, 0, FLAGS, .unit="scale" },
    {  "erbs",    "erbs",             0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_ERBS},   0, 0, FLAGS, .unit="scale" },
    {  "sqrt",    "sqrt",             0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_SQRT},   0, 0, FLAGS, .unit="scale" },
    {  "cbrt",    "cbrt",             0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_CBRT},   0, 0, FLAGS, .unit="scale" },
    {  "qdrt",    "qdrt",             0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_QDRT},   0, 0, FLAGS, .unit="scale" },
    {  "fm",      "fm",               0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_FM},     0, 0, FLAGS, .unit="scale" },
    {  "gwood",   "greenwood",        0,                       AV_OPT_TYPE_CONST,{.i64=FSCALE_GREENWOOD},0,0,FLAGS, .unit="scale" },
    { "iscale", "set intensity scale", OFFSET(intensity_scale),AV_OPT_TYPE_INT,  {.i64=0},   0, NB_ISCALE-1, FLAGS, .unit="iscale" },
    {  "linear",  "linear",           0,                       AV_OPT_TYPE_CONST,{.i64=ISCALE_LINEAR}, 0, 0, FLAGS, .unit="iscale" },
    {  "log",     "logarithmic",      0,                       AV_OPT_TYPE_CONST,{.i64=ISCALE_LOG},    0, 0, FLAGS, .unit="iscale" },
    {  "sqrt",    "sqrt",             0,                       AV_OPT_TYPE_CONST,{.i64=ISCALE_SQRT},   0, 0, FLAGS, .unit="iscale" },
    {  "cbrt",    "cbrt",             0,                       AV_OPT_TYPE_CONST,{.i64=ISCALE_CBRT},   0, 0, FLAGS, .unit="iscale" },
    {  "qdrt",    "qdrt",             0,                       AV_OPT_TYPE_CONST,{.i64=ISCALE_QDRT},   0, 0, FLAGS, .unit="iscale" },
    { "min",  "set minimum frequency", OFFSET(minimum_frequency), AV_OPT_TYPE_FLOAT, {.dbl = 20.},    1, 192000, FLAGS },
    { "max",  "set maximum frequency", OFFSET(maximum_frequency), AV_OPT_TYPE_FLOAT, {.dbl = 20000.}, 1, 192000, FLAGS },
    { "imin", "set minimum intensity", OFFSET(minimum_intensity), AV_OPT_TYPE_FLOAT, {.dbl = 0.}, 0, 1, FLAGS },
    { "imax", "set maximum intensity", OFFSET(maximum_intensity), AV_OPT_TYPE_FLOAT, {.dbl = 1.}, 0, 1, FLAGS },
    { "logb", "set logarithmic basis", OFFSET(logarithmic_basis), AV_OPT_TYPE_FLOAT, {.dbl = 0.0001}, 0, 1, FLAGS },
    { "deviation", "set frequency deviation", OFFSET(deviation), AV_OPT_TYPE_FLOAT, {.dbl = 1.}, 0, 100, FLAGS },
    { "mode", "set output mode", OFFSET(mode), AV_OPT_TYPE_INT,  {.i64=0}, 0, NB_MODE-1, FLAGS, .unit="mode" },
        { "line", "show lines",  0, AV_OPT_TYPE_CONST, {.i64=LINE}, 0, 0, FLAGS, .unit = "mode" },
        { "bar",  "show bars",   0, AV_OPT_TYPE_CONST, {.i64=BAR},  0, 0, FLAGS, .unit = "mode" },
        { "dot",  "show dots",   0, AV_OPT_TYPE_CONST, {.i64=DOT},  0, 0, FLAGS, .unit = "mode" },
    { "colors", "set channels colors", OFFSET(colors), AV_OPT_TYPE_COLOR|AR, {.arr = &def_colors}, 0, 0, FLAGS },
    { "cmode", "set channel mode", OFFSET(channel_mode), AV_OPT_TYPE_INT, {.i64=COMBINED}, 0, NB_CMODES-1, FLAGS, .unit = "cmode" },
        { "combined", "show all channels in same window",  0, AV_OPT_TYPE_CONST, {.i64=COMBINED}, 0, 0, FLAGS, .unit = "cmode" },
        { "separate", "show each channel in own window",   0, AV_OPT_TYPE_CONST, {.i64=SEPARATE}, 0, 0, FLAGS, .unit = "cmode" },
    { "data", "set data mode", OFFSET(data_mode), AV_OPT_TYPE_INT, {.i64=MAGNITUDE}, 0, NB_DATA-1, FLAGS, .unit = "data" },
        { "magnitude", "show magnitude",  0, AV_OPT_TYPE_CONST, {.i64=MAGNITUDE}, 0, 0, FLAGS, .unit = "data" },
        { "phase",     "show phase",      0, AV_OPT_TYPE_CONST, {.i64=PHASE},     0, 0, FLAGS, .unit = "data" },
        { "delay",     "show group delay",0, AV_OPT_TYPE_CONST, {.i64=DELAY},     0, 0, FLAGS, .unit = "data" },
    { "channels", "set channels to draw", OFFSET(ch_layout_str), AV_OPT_TYPE_STRING, {.str="all"}, 0, 0, FLAGS },
    { "weighting", "set the frequency weighting", OFFSET(weighting_type), AV_OPT_TYPE_INT, {.i64=0}, 0, NB_WEIGHT-1, FLAGS, .unit="weight" },
    {  "none", "no weighting", 0, AV_OPT_TYPE_CONST,{.i64=WEIGHTING_0}, 0, 0, FLAGS, .unit="weight" },
    {  "A", "A-weighting", 0, AV_OPT_TYPE_CONST,{.i64=WEIGHTING_A}, 0, 0, FLAGS, .unit="weight" },
    {  "B", "B-weighting", 0, AV_OPT_TYPE_CONST,{.i64=WEIGHTING_B}, 0, 0, FLAGS, .unit="weight" },
    {  "C", "C-weighting", 0, AV_OPT_TYPE_CONST,{.i64=WEIGHTING_C}, 0, 0, FLAGS, .unit="weight" },
    {  "D", "D-weighting", 0, AV_OPT_TYPE_CONST,{.i64=WEIGHTING_D}, 0, 0, FLAGS, .unit="weight" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(showfreqs);

static av_cold void uninit(AVFilterContext *ctx)
{
    ShowFreqsContext *s = ctx->priv;

    av_freep(&s->frequency_weight);
    av_freep(&s->frequency_band);
    av_freep(&s->kernel_start);
    av_freep(&s->kernel_stop);
    av_freep(&s->bypass);
    av_freep(&s->index);

    av_frame_free(&s->cache);
    av_frame_free(&s->fft_in);
    av_frame_free(&s->fft_out);
    av_frame_free(&s->dst_x);
    av_frame_free(&s->src_x);
    av_frame_free(&s->ifft_in);
    av_frame_free(&s->ifft_out);
    av_frame_free(&s->ch_out);
    av_frame_free(&s->over);
    av_frame_free(&s->yh_out);

    if (s->fft) {
        for (int n = 0; n < s->nb_threads; n++)
            av_tx_uninit(&s->fft[n]);
        av_freep(&s->fft);
    }

    if (s->ifft) {
        for (int n = 0; n < s->nb_threads; n++)
            av_tx_uninit(&s->ifft[n]);
        av_freep(&s->ifft);
    }

    if (s->kernel) {
        for (int n = 0; n < s->frequency_band_count; n++)
            av_freep(&s->kernel[n]);
    }
    av_freep(&s->kernel);

    av_freep(&s->fdsp);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    AVFilterFormats *formats = NULL;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_RGBA, AV_PIX_FMT_NONE };
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    return 0;
}

static float frequency_weight_curve(float f, int type)
{
    float w;

    switch (type) {
    case WEIGHTING_A:
        w = (powf(12194.f, 2.f) * powf(f, 4.f)) /
            ((powf(f, 2.f) + powf(20.6, 2.f)) *
             sqrtf((powf(f, 2.f) + powf(107.7f, 2.f)) *
                   (powf(f, 2.f) + powf(737.9f, 2.f))) *
             (powf(f, 2.f) + powf(12194.f, 2.f)));
        break;
    case WEIGHTING_B:
        w = (powf(12194.f, 2.f) * powf(f, 3.f)) /
            ((powf(f, 2.f) + powf(20.6, 2.f)) *
             sqrtf(powf(f, 2.f) + powf(158.5f, 2.f)) *
             (powf(f, 2.f) + powf(12194.f, 2.f)));
        break;
    case WEIGHTING_C:
        w = (powf(12194.f, 2.f) * powf(f, 2.f)) /
            ((powf(f, 2.f) + powf(20.6, 2.f)) *
             (powf(f, 2.f) + powf(12194.f, 2.f)));
        break;
    case WEIGHTING_D:
        w = (f / 6.8966888496476e-5f) *
            sqrtf(((powf(1037918.48f - powf(f, 2.f), 2.f) + 1080768.16f * powf(f, 2.f)) /
                   powf(9837328.f - powf(f, 2.f), 2.f) + 11723776.f * powf(f, 2.f)) /
                  ((powf(f, 2.f) + 79919.29f) * (powf(f, 2.f) + 1345600.f)));
        break;
    default:
        w = 1.f;
        break;
    }

    return w;
}

static float frequency_weight(float f, int type)
{
    float w;

    switch (type) {
    case WEIGHTING_A:
    case WEIGHTING_B:
    case WEIGHTING_C:
    case WEIGHTING_D:
        w = frequency_weight_curve(f, type) / frequency_weight_curve(1000.f, type);
        break;
    default:
        w = 1.f;
        break;
    }

    return w;
}

static float frequency_band(float *frequency_band,
                            float *frequency_weights,
                            float fs,
                            int frequency_band_count,
                            float frequency_range,
                            float frequency_offset,
                            int frequency_scale, float deviation,
                            int weighting_type)
{
    float ret = 0.f;

    deviation = sqrtf(deviation / (4.f * M_PI)); // Heisenberg Gabor Limit
    for (int y = 0; y < frequency_band_count; y++) {
        float frequency = frequency_range * (1.f - (float)y / frequency_band_count) + frequency_offset;
        float frequency_derivative = frequency_range / frequency_band_count;

        switch (frequency_scale) {
        case FSCALE_LOG:
            frequency = powf(2.f, frequency);
            frequency_derivative *= logf(2.f) * frequency;
            break;
        case FSCALE_BARK:
            frequency = 600.f * sinhf(frequency / 6.f);
            frequency_derivative *= sqrtf(frequency * frequency + 360000.f) / 6.f;
            break;
        case FSCALE_MEL:
            frequency = 700.f * (powf(10.f, frequency / 2595.f) - 1.f);
            frequency_derivative *= (frequency + 700.f) * logf(10.f) / 2595.f;
            break;
        case FSCALE_ERBS:
            frequency = 676170.4f / (47.06538f - expf(frequency * 0.08950404f)) - 14678.49f;
            frequency_derivative *= (frequency * frequency + 14990.4f * frequency + 4577850.f) / 160514.f;
            break;
        case FSCALE_SQRT:
            frequency = frequency * frequency;
            frequency_derivative *= 2.f * sqrtf(frequency);
            break;
        case FSCALE_CBRT:
            frequency = frequency * frequency * frequency;
            frequency_derivative *= 3.f * powf(frequency, 2.f / 3.f);
            break;
        case FSCALE_QDRT:
            frequency = frequency * frequency * frequency * frequency;
            frequency_derivative *= 4.f * powf(frequency, 3.f / 4.f);
            break;
        case FSCALE_FM:
            frequency = 2.f * powf(frequency, 3.f / 2.f) / 3.f;
            frequency_derivative *= sqrtf(frequency);
            break;
        case FSCALE_GREENWOOD:
            frequency = 165.4f * (expf(frequency / 512.18f) - 1.f);
            frequency_derivative *= (frequency / 165.4f + 1.f) / 3.09661f;
            break;
        }

        frequency_band[y*2  ] = frequency;
        frequency_band[y*2+1] = frequency_derivative * deviation;

        frequency_weights[y] = frequency_weight(frequency, weighting_type);

        ret = 1.f / (frequency_derivative * deviation);
    }

    return ret;
}

static float remap_log(ShowFreqsContext *s, float value, float weight, int iscale, float log_factor)
{
    const float max = s->maximum_intensity;
    const float min = s->minimum_intensity;
    float ret;

    value *= weight;
    value += min;

    switch (iscale) {
    case ISCALE_LINEAR:
        ret = max - expf(value / log_factor);
        break;
    case ISCALE_LOG:
        value = logf(value) * log_factor;
        ret = max - av_clipf(value, 0.f, 1.f);
        break;
    case ISCALE_SQRT:
        value = max - expf(value / log_factor);
        ret = sqrtf(value);
        break;
    case ISCALE_CBRT:
        value = max - expf(value / log_factor);
        ret = cbrtf(value);
        break;
    case ISCALE_QDRT:
        value = max - expf(value / log_factor);
        ret = powf(value, 0.25f);
        break;
    }

    return av_clipf(ret, 0.f, 1.f);
}

static int run_channel_cwt_prepare(AVFilterContext *ctx, void *arg, int jobnr, int ch)
{
    ShowFreqsContext *s = ctx->priv;
    const int hop_size = s->hop_size;
    AVFrame *in = arg;
    float *cache = (float *)s->cache->extended_data[ch];
    AVComplexFloat *src = (AVComplexFloat *)s->fft_in->extended_data[ch];
    AVComplexFloat *dst = (AVComplexFloat *)s->fft_out->extended_data[ch];
    const float *input = (const float *)in->extended_data[ch];
    const int offset = (s->input_padding_size - hop_size) >> 1;

    memcpy(cache + s->hop_index, input, in->nb_samples * sizeof(*cache));

    if (s->hop_index + in->nb_samples < hop_size)
        return 0;

    memset(src, 0, sizeof(*src) * s->fft_size);
    for (int n = 0; n < hop_size; n++)
        src[n+offset].re = cache[n];

    s->tx_fn(s->fft[jobnr], dst, src, sizeof(*src));

    return 0;
}

#define DRAW_DOT_COLOR(x, y) \
do { \
    dst[x + y*linesize] |= color; \
} while (0)

#define DRAW_COLOR(x) \
do { \
    dst[x] |= color; \
} while (0)

#define DRAW_LINE_COLOR(x0, y0, x1, y1) \
do { \
    int dx = FFABS(x1-x0), sx = x0 < x1 ? 1 : -1; \
    int dy = FFABS(y1-y0), sy = y0 < y1 ? 1 : -1; \
    int err = (dx>dy ? dx : -dy) / 2, e2; \
 \
    for (;;) { \
        DRAW_DOT_COLOR(x0, y0); \
 \
        if (x0 == x1 && y0 == y1) \
            break; \
 \
        e2 = err; \
 \
        if (e2 >-dx) { \
            err -= dy; \
            x0 += sx; \
        } \
 \
        if (e2 < dy) { \
            err += dx; \
            y0 += sy; \
        } \
    } \
} while (0)

static int draw(AVFilterContext *ctx, AVFrame *out)
{
    ShowFreqsContext *s = ctx->priv;
    const float log_factor = 1.f/logf(s->logarithmic_basis);
    const float *weights = s->frequency_weight;
    const int count = s->frequency_band_count;
    const int nb_channels = s->nb_channels;
    const int iscale = s->intensity_scale;
    const int ihop_index = s->ihop_index;
    const int ihop_size = s->ihop_size;
    const uint8_t *bypass = s->bypass;
    const int max_ch_size = (s->channel_mode == COMBINED) ? s->h : s->h/FFMAX(s->nb_draw_channels, 1);
    const int ch_step = (s->channel_mode == COMBINED) ? 0 : max_ch_size;
    const int data_mode = s->data_mode;
    const int mode = s->mode;
    int ch_offset = 0;

    for (int ch = 0; ch < nb_channels; ch++) {
        if (bypass[ch])
            continue;

        for (int x = 0; x < count; x++) {
            const int next = (x >= count-1) ? x-1 : x+1;
            const float weight = weights[x];
            const AVComplexFloat *src = ((const AVComplexFloat *)s->ch_out->extended_data[x]) +
                ch * ihop_size + ihop_index;
            const AVComplexFloat *nsrc = ((const AVComplexFloat *)s->ch_out->extended_data[next]) +
                ch * ihop_size + ihop_index;
            int *yh = ((int *)s->yh_out->extended_data[ch]) + count-x-1;
            float Y;

            switch (data_mode) {
            case MAGNITUDE:
                Y = hypotf(src[0].re, src[0].im);
                Y = remap_log(s, Y, weight, iscale, log_factor);
                break;
            case PHASE:
                Y = atan2f(src[0].im, src[0].re);
                Y = av_clipf((M_PIf + Y) / (2.f * M_PIf), 0.f, 1.f);
                break;
            case DELAY:
                Y = av_clipf((M_PIf - atan2f(src[0].im * nsrc[0].re - nsrc[0].im * src[0].re,
                                             src[0].re * nsrc[0].re + nsrc[0].im * src[0].im)) / (2.f * M_PIf), 0.f, 1.f);
                break;
            }

            yh[0] = max_ch_size-av_clip(Y*max_ch_size, 1, max_ch_size);
        }
    }

    for (int ch = 0; ch < nb_channels; ch++) {
        const int *yh = ((const int *)s->yh_out->extended_data[ch]);
        const ptrdiff_t linesize = out->linesize[0]/4;
        uint32_t *odst = (uint32_t *)(out->data[0]);
        const int ch_idx = FFMIN(ch, s->nb_colors-1);
        const uint32_t color = s->colors[ch_idx];

        if (bypass[ch])
            continue;

        switch (mode) {
        case BAR:
            for (int x = 0; x < count; x++) {
                uint32_t *dst = odst + (ch_offset + yh[x]) * linesize;

                for (int y = yh[x]; y < max_ch_size; y++) {
                    DRAW_COLOR(x);

                    dst += linesize;
                }
            }
            break;
        case DOT:
            {
                uint32_t *dst = odst;

                for (int x = 0; x < count; x++) {
                    const int y = yh[x] + ch_offset;
                    DRAW_DOT_COLOR(x, y);
                }
            }
            break;
        case LINE:
            {
                uint32_t *dst = odst;

                for (int x = 0; x < count; x++) {
                    int x0 = FFMAX(x-1, 0);
                    int y0 = yh[x0] + ch_offset;
                    const int y = yh[x] + ch_offset;

                    DRAW_LINE_COLOR(x0, y0, x, y);
                }
            }
            break;
        }

        ch_offset += ch_step;
    }

    return 0;
}

static int run_channel_cwt(AVFilterContext *ctx, int ch, int jobnr, int nb_jobs)
{
    ShowFreqsContext *s = ctx->priv;
    const AVComplexFloat *fft_out = (const AVComplexFloat *)s->fft_out->extended_data[ch];
    AVComplexFloat *isrc = (AVComplexFloat *)s->ifft_in->extended_data[jobnr];
    AVComplexFloat *idst = (AVComplexFloat *)s->ifft_out->extended_data[jobnr];
    const int output_padding_size = s->output_padding_size;
    const int input_padding_size = s->input_padding_size;
    const float scale = 1.f / input_padding_size;
    const int ihop_size = s->ihop_size;
    const int count = s->frequency_band_count;
    const int start = (count * jobnr) / nb_jobs;
    const int end = (count * (jobnr+1)) / nb_jobs;
    const int coffset = ch * ihop_size;

    for (int y = start; y < end; y++) {
        AVComplexFloat *chout = ((AVComplexFloat *)s->ch_out->extended_data[y]) + coffset;
        AVComplexFloat *over = ((AVComplexFloat *)s->over->extended_data[ch]) + y * ihop_size;
        AVComplexFloat *dstx = (AVComplexFloat *)s->dst_x->extended_data[jobnr];
        AVComplexFloat *srcx = (AVComplexFloat *)s->src_x->extended_data[jobnr];
        const AVComplexFloat *kernel = s->kernel[y];
        const unsigned *index = (const unsigned *)s->index;
        const int kernel_start = s->kernel_start[y];
        const int kernel_stop = s->kernel_stop[y];
        const int kernel_range = kernel_stop - kernel_start + 1;
        int offset;

        if (kernel_start >= 0) {
            offset = 0;
            memcpy(srcx, fft_out + kernel_start, sizeof(*fft_out) * kernel_range);
        } else {
            offset = -kernel_start;
            memcpy(srcx+offset, fft_out, sizeof(*fft_out) * (kernel_range-offset));
            memcpy(srcx, fft_out+input_padding_size-offset, sizeof(*fft_out)*offset);
        }

        s->fdsp->vector_fmul_scalar((float *)srcx, (const float *)srcx, scale, FFALIGN(kernel_range * 2, 4));
        s->fdsp->vector_fmul((float *)dstx, (const float *)srcx,
                             (const float *)kernel, FFALIGN(kernel_range * 2, 16));

        memset(isrc, 0, sizeof(*isrc) * output_padding_size);
        if (offset == 0) {
            const unsigned *kindex = index + kernel_start;
            for (int i = 0; i < kernel_range; i++) {
                const unsigned n = kindex[i];

                isrc[n].re += dstx[i].re;
                isrc[n].im += dstx[i].im;
            }
        } else {
            for (int i = 0; i < kernel_range; i++) {
                const unsigned n = (i-kernel_start) & (output_padding_size-1);

                isrc[n].re += dstx[i].re;
                isrc[n].im += dstx[i].im;
            }
        }

        s->itx_fn(s->ifft[jobnr], idst, isrc, sizeof(*isrc));

        memcpy(chout, idst, sizeof(*chout) * ihop_size);
        for (int n = 0; n < ihop_size; n++) {
            chout[n].re += over[n].re;
            chout[n].im += over[n].im;
        }
        memcpy(over, idst + ihop_size, sizeof(*over) * ihop_size);
    }

    return 0;
}

static int run_channels_cwt(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ShowFreqsContext *s = ctx->priv;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        if (s->bypass[ch])
            continue;

        run_channel_cwt(ctx, ch, jobnr, nb_jobs);
    }

    return 0;
}

static int compute_kernel(AVFilterContext *ctx)
{
    ShowFreqsContext *s = ctx->priv;
    const int size = s->input_padding_size;
    const int osize = s->output_padding_size;
    const int output_sample_count = s->output_sample_count;
    const int fsize = s->frequency_band_count;
    int *kernel_start = s->kernel_start;
    int *kernel_stop = s->kernel_stop;
    unsigned *index = s->index;
    int range_min = INT_MAX;
    int range_max = 0, ret = 0;
    float *tkernel;

    tkernel = av_malloc_array(size, sizeof(*tkernel));
    if (!tkernel) {
        av_freep(&tkernel);
        return AVERROR(ENOMEM);
    }

    for (int y = 0; y < fsize; y++) {
        AVComplexFloat *kernel = s->kernel[y];
        int start = INT_MIN, stop = INT_MAX;
        const float dfrequency = s->frequency_band[y*2+1];
        const float frequency = s->frequency_band[y*2];
        const float deviation = 1.f / (dfrequency *
                                       output_sample_count);
        const int a = FFMAX(frequency-12.f*sqrtf(1.f/deviation)-0.5f, -size);
        const int b = FFMIN(frequency+12.f*sqrtf(1.f/deviation)-0.5f, size+a);
        int kernel_size;
        const int range = -a;

        memset(tkernel, 0, size * sizeof(*tkernel));
        for (int n = a; n < b; n++) {
            const float f = n+0.5f-frequency;

            tkernel[n+range] = expf(-f*f*deviation);
        }

        for (int n = a; n < b; n++) {
            if (tkernel[n+range] != 0.f) {
                if (tkernel[n+range] > FLT_MIN)
                    av_log(ctx, AV_LOG_DEBUG, "out of range kernel %g\n", tkernel[n+range]);
                start = n;
                break;
            }
        }

        for (int n = b; n >= a; n--) {
            if (tkernel[n+range] != 0.f) {
                if (tkernel[n+range] > FLT_MIN)
                    av_log(ctx, AV_LOG_DEBUG, "out of range kernel %g\n", tkernel[n+range]);
                stop = n;
                break;
            }
        }

        if (start == INT_MIN || stop == INT_MAX) {
            ret = AVERROR(EINVAL);
            break;
        }

        kernel_start[y] = start;
        kernel_stop[y] = stop;
        kernel_size = stop-start+1;

        kernel = av_calloc(FFALIGN(kernel_size, 16), sizeof(*kernel));
        if (!kernel) {
            ret = AVERROR(ENOMEM);
            break;
        }

        for (int n = 0; n < kernel_size; n++) {
            kernel[n].re = tkernel[n+range+start];
            kernel[n].im = tkernel[n+range+start];
        }

        range_min = FFMIN(range_min, kernel_size);
        range_max = FFMAX(range_max, kernel_size);

        s->kernel[y] = kernel;
    }

    for (int n = 0; n < size; n++)
        index[n] = n & (osize - 1);

    av_log(ctx, AV_LOG_DEBUG, "range_min: %d\n", range_min);
    av_log(ctx, AV_LOG_DEBUG, "range_max: %d\n", range_max);

    av_freep(&tkernel);

    return ret;
}

static int config_output(AVFilterLink *outlink)
{
    FilterLink *l = ff_filter_link(outlink);
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowFreqsContext *s = ctx->priv;
    const float limit_frequency = inlink->sample_rate * 0.5f;
    float maximum_frequency = fminf(s->maximum_frequency, limit_frequency);
    float minimum_frequency = s->minimum_frequency;
    float scale = 1.f, factor;
    int ret;

    if (minimum_frequency >= maximum_frequency) {
        av_log(ctx, AV_LOG_ERROR, "min frequency (%f) >= (%f) max frequency\n",
               minimum_frequency, maximum_frequency);
        return AVERROR(EINVAL);
    }

    uninit(ctx);

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    s->frequency_band_count = s->w;

    switch (s->frequency_scale) {
    case FSCALE_LOG:
        minimum_frequency = logf(minimum_frequency) / logf(2.f);
        maximum_frequency = logf(maximum_frequency) / logf(2.f);
        break;
    case FSCALE_BARK:
        minimum_frequency = 6.f * asinhf(minimum_frequency / 600.f);
        maximum_frequency = 6.f * asinhf(maximum_frequency / 600.f);
        break;
    case FSCALE_MEL:
        minimum_frequency = 2595.f * log10f(1.f + minimum_frequency / 700.f);
        maximum_frequency = 2595.f * log10f(1.f + maximum_frequency / 700.f);
        break;
    case FSCALE_ERBS:
        minimum_frequency = 11.17268f * logf(1.f + (46.06538f * minimum_frequency) / (minimum_frequency + 14678.49f));
        maximum_frequency = 11.17268f * logf(1.f + (46.06538f * maximum_frequency) / (maximum_frequency + 14678.49f));
        break;
    case FSCALE_SQRT:
        minimum_frequency = sqrtf(minimum_frequency);
        maximum_frequency = sqrtf(maximum_frequency);
        break;
    case FSCALE_CBRT:
        minimum_frequency = cbrtf(minimum_frequency);
        maximum_frequency = cbrtf(maximum_frequency);
        break;
    case FSCALE_QDRT:
        minimum_frequency = powf(minimum_frequency, 0.25f);
        maximum_frequency = powf(maximum_frequency, 0.25f);
        break;
    case FSCALE_FM:
        minimum_frequency = powf(9.f * (minimum_frequency * minimum_frequency) / 4.f, 1.f / 3.f);
        maximum_frequency = powf(9.f * (maximum_frequency * maximum_frequency) / 4.f, 1.f / 3.f);
        break;
    case FSCALE_GREENWOOD:
        minimum_frequency = 512.18f * logf(1.f + minimum_frequency / 165.4f);
        maximum_frequency = 512.18f * logf(1.f + maximum_frequency / 165.4f);
        break;
    }

    s->frequency_weight = av_calloc(s->frequency_band_count, sizeof(*s->frequency_weight));
    if (!s->frequency_weight)
        return AVERROR(ENOMEM);

    s->frequency_band = av_calloc(s->frequency_band_count,
                                  sizeof(*s->frequency_band) * 2);
    if (!s->frequency_band)
        return AVERROR(ENOMEM);

    s->nb_consumed_samples = inlink->sample_rate *
                             frequency_band(s->frequency_band, s->frequency_weight,
                                            inlink->sample_rate,
                                            s->frequency_band_count, maximum_frequency - minimum_frequency,
                                            minimum_frequency, s->frequency_scale, s->deviation,
                                            s->weighting_type);
    s->nb_consumed_samples = FFMIN(s->nb_consumed_samples, 131072);

    s->nb_threads = FFMIN(s->frequency_band_count, ff_filter_get_nb_threads(ctx));
    s->nb_channels = inlink->ch_layout.nb_channels;
    s->old_pts = AV_NOPTS_VALUE;

    s->input_sample_count = 1 << (32 - ff_clz(s->nb_consumed_samples));
    s->input_padding_size = 1 << (32 - ff_clz(s->input_sample_count));
    s->output_sample_count = FFMAX(1, av_rescale_q(s->input_sample_count, s->frame_rate, av_make_q(inlink->sample_rate, 1)));
    s->output_padding_size = 1 << (32 - ff_clz(s->output_sample_count));

    s->hop_size  = s->input_sample_count;
    s->ihop_size = s->output_padding_size >> 1;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};

    s->fft_size = FFALIGN(s->input_padding_size, av_cpu_max_align());
    s->ifft_size = FFALIGN(s->output_padding_size, av_cpu_max_align());

    s->fft = av_calloc(s->nb_threads, sizeof(*s->fft));
    if (!s->fft)
        return AVERROR(ENOMEM);

    for (int n = 0; n < s->nb_threads; n++) {
        ret = av_tx_init(&s->fft[n], &s->tx_fn, AV_TX_FLOAT_FFT, 0, s->input_padding_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->ifft = av_calloc(s->nb_threads, sizeof(*s->ifft));
    if (!s->ifft)
        return AVERROR(ENOMEM);

    for (int n = 0; n < s->nb_threads; n++) {
        ret = av_tx_init(&s->ifft[n], &s->itx_fn, AV_TX_FLOAT_FFT, 1, s->output_padding_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->fft_in = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->fft_out = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->dst_x = av_frame_alloc();
    s->src_x = av_frame_alloc();
    s->kernel = av_calloc(s->frequency_band_count, sizeof(*s->kernel));
    s->cache = ff_get_audio_buffer(inlink, s->hop_size);
    s->over = ff_get_audio_buffer(inlink, s->frequency_band_count * 2 * s->ihop_size);
    s->yh_out = ff_get_audio_buffer(inlink, s->frequency_band_count);
    s->ifft_in = av_frame_alloc();
    s->ifft_out = av_frame_alloc();
    s->ch_out = av_frame_alloc();
    s->index = av_calloc(s->input_padding_size, sizeof(*s->index));
    s->kernel_start = av_calloc(s->frequency_band_count, sizeof(*s->kernel_start));
    s->kernel_stop = av_calloc(s->frequency_band_count, sizeof(*s->kernel_stop));
    if (!s->fft_in || !s->fft_out || !s->src_x || !s->dst_x || !s->over ||
        !s->ifft_in || !s->ifft_out || !s->kernel_start || !s->kernel_stop || !s->ch_out ||
        !s->cache || !s->index || !s->yh_out || !s->kernel)
        return AVERROR(ENOMEM);

    s->ch_out->format     = inlink->format;
    s->ch_out->nb_samples = 2 * s->ihop_size * inlink->ch_layout.nb_channels;
    s->ch_out->ch_layout.nb_channels = s->frequency_band_count;
    ret = av_frame_get_buffer(s->ch_out, 0);
    if (ret < 0)
        return ret;

    av_samples_set_silence(s->ch_out->extended_data, 0,
                           s->ch_out->nb_samples,
                           s->ch_out->ch_layout.nb_channels,
                           s->ch_out->format);

    s->ifft_in->format     = inlink->format;
    s->ifft_in->nb_samples = s->ifft_size * 2;
    s->ifft_in->ch_layout.nb_channels = s->nb_threads;
    ret = av_frame_get_buffer(s->ifft_in, 0);
    if (ret < 0)
        return ret;

    s->ifft_out->format     = inlink->format;
    s->ifft_out->nb_samples = s->ifft_size * 2;
    s->ifft_out->ch_layout.nb_channels = s->nb_threads;
    ret = av_frame_get_buffer(s->ifft_out, 0);
    if (ret < 0)
        return ret;

    s->src_x->format     = inlink->format;
    s->src_x->nb_samples = s->fft_size * 2;
    s->src_x->ch_layout.nb_channels = s->nb_threads;
    ret = av_frame_get_buffer(s->src_x, 0);
    if (ret < 0)
        return ret;

    s->dst_x->format     = inlink->format;
    s->dst_x->nb_samples = s->fft_size * 2;
    s->dst_x->ch_layout.nb_channels = s->nb_threads;
    ret = av_frame_get_buffer(s->dst_x, 0);
    if (ret < 0)
        return ret;

    factor = s->input_padding_size / (float)inlink->sample_rate;
    for (int n = 0; n < s->frequency_band_count; n++) {
        s->frequency_band[2*n  ] *= factor;
        s->frequency_band[2*n+1] *= factor;
    }

    av_log(ctx, AV_LOG_DEBUG, "factor: %f\n", factor);
    av_log(ctx, AV_LOG_DEBUG, "nb_consumed_samples: %d\n", s->nb_consumed_samples);
    av_log(ctx, AV_LOG_DEBUG, "hop_size: %d\n", s->hop_size);
    av_log(ctx, AV_LOG_DEBUG, "ihop_size: %d\n", s->ihop_size);
    av_log(ctx, AV_LOG_DEBUG, "input_sample_count: %d\n", s->input_sample_count);
    av_log(ctx, AV_LOG_DEBUG, "input_padding_size: %d\n", s->input_padding_size);
    av_log(ctx, AV_LOG_DEBUG, "output_sample_count: %d\n", s->output_sample_count);
    av_log(ctx, AV_LOG_DEBUG, "output_padding_size: %d\n", s->output_padding_size);

    l->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(l->frame_rate);

    ret = compute_kernel(ctx);
    if (ret < 0)
        return ret;

    ret = av_channel_layout_copy(&s->ch_layout, &inlink->ch_layout);
    if (ret < 0)
        return ret;
    s->nb_draw_channels = s->nb_channels;

    s->bypass = av_calloc(s->nb_channels, sizeof(*s->bypass));
    if (!s->bypass)
        return AVERROR(ENOMEM);

    if (strcmp(s->ch_layout_str, "all")) {
        int nb_draw_channels = 0;

        av_channel_layout_from_string(&s->ch_layout,
                                      s->ch_layout_str);

        for (int ch = 0; ch < s->nb_channels; ch++) {
            const enum AVChannel channel = av_channel_layout_channel_from_index(&inlink->ch_layout, ch);

            s->bypass[ch] = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
            nb_draw_channels += s->bypass[ch] == 0;
        }
        s->nb_draw_channels = nb_draw_channels;
    }

    return 0;
}

static int output_frame(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    ShowFreqsContext *s = ctx->priv;

    if (s->old_pts < s->pts) {
        AVFrame *out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out)
            return AVERROR(ENOMEM);

        for (int y = 0; y < outlink->h; y++)
            memset(out->data[0] + y * out->linesize[0], 0, outlink->w*4);

        draw(ctx, out);

        s->old_pts = out->pts = s->pts;
        s->pts++;
        out->duration = 1;
        s->ihop_index++;
        if (s->ihop_index >= s->ihop_size)
            s->ihop_index = s->hop_index = 0;

        return ff_filter_frame(outlink, out);
    }

    return 1;
}

static int run_channels_cwt_prepare(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ShowFreqsContext *s = ctx->priv;
    const int count = s->nb_channels;
    const int start = (count * jobnr) / nb_jobs;
    const int end = (count * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        if (s->bypass[ch])
            continue;

        run_channel_cwt_prepare(ctx, arg, jobnr, ch);
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    ShowFreqsContext *s = ctx->priv;
    int ret = 0, status;
    AVFrame *fin = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (s->hop_index < s->hop_size) {
        ret = ff_inlink_consume_samples(inlink, 1, s->hop_size - s->hop_index, &fin);
        if (ret < 0)
            return ret;

        if (ret > 0) {
            if (s->hop_index == 0) {
                s->pts = av_rescale_q(fin->pts - s->hop_size/2, inlink->time_base, outlink->time_base);
                if (s->old_pts == AV_NOPTS_VALUE)
                    s->old_pts = s->pts - 1;
            }
            ff_filter_execute(ctx, run_channels_cwt_prepare, fin, NULL,
                              FFMIN(s->nb_threads, s->nb_channels));
            s->hop_index += fin->nb_samples;
            av_frame_free(&fin);
        }
    }

    if (s->hop_index >= s->hop_size) {
        if (s->ihop_index == 0) {
            ff_filter_execute(ctx, run_channels_cwt, NULL, NULL,
                              s->nb_threads);
        }

        ret = output_frame(ctx);
        if (ret != 1)
            return ret;
    }


    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF) {
            ff_filter_set_ready(ctx, 10);
            ff_outlink_set_status(outlink, AVERROR_EOF, pts);
            return 0;
        }
    }

    if (ff_inlink_queued_samples(inlink) > 0 || s->ihop_index ||
        s->hop_index >= s->hop_size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink)) {
        ff_inlink_request_frame(inlink);
        return 0;
    }

    return FFERROR_NOT_READY;
}

static const AVFilterPad showfreqs_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_avf_showfreqs = {
    .p.name        = "showfreqs",
    .p.description = NULL_IF_CONFIG_SMALL("Convert input audio to a frequencies video output."),
    .p.priv_class  = &showfreqs_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .uninit        = uninit,
    .priv_size     = sizeof(ShowFreqsContext),
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(showfreqs_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .activate      = activate,
};
