/*
 * Copyright (c) 2008-2009 Rob Sykes <robs@users.sourceforge.net>
 * Copyright (c) 2017 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct SincContext {
    const AVClass *class;

    int sample_rate, nb_samples;
    float phase, Fc0, Fc1, tbw0, tbw1;
    double att, beta;
    int num_taps[2];
    int round;

    int n;
    void *coeffs;
    int64_t pts;

    int sample_size;

    AVTXContext *tx, *itx;
    av_tx_fn tx_fn, itx_fn;
} SincContext;

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    SincContext *s = ctx->priv;
    AVFrame *frame = NULL;
    int nb_samples;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    nb_samples = FFMIN(s->nb_samples, s->n - s->pts);
    if (nb_samples <= 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (!(frame = ff_get_audio_buffer(outlink, nb_samples)))
        return AVERROR(ENOMEM);

    memcpy(frame->data[0], (uint8_t *)s->coeffs + s->pts * s->sample_size, nb_samples * s->sample_size);

    frame->pts = s->pts;
    s->pts    += nb_samples;

    return ff_filter_frame(outlink, frame);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const SincContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_NONE };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

#define DEPTH 32
#include "sinc_template.c"

#undef DEPTH
#define DEPTH 64
#include "sinc_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    SincContext *s = ctx->priv;

    outlink->sample_rate = s->sample_rate;
    s->pts = 0;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLT:
        s->sample_size = sizeof(float);
        return generate_flt(ctx);
    case AV_SAMPLE_FMT_DBL:
        s->sample_size = sizeof(double);
        return generate_dbl(ctx);
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SincContext *s = ctx->priv;

    av_freep(&s->coeffs);
    av_tx_uninit(&s->tx);
    av_tx_uninit(&s->itx);
}

static const AVFilterPad sinc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define OFFSET(x) offsetof(SincContext, x)

static const AVOption sinc_options[] = {
    { "sample_rate", "set sample rate",                               OFFSET(sample_rate), AV_OPT_TYPE_INT,   {.i64=44100},  1, INT_MAX, AF },
    { "r",           "set sample rate",                               OFFSET(sample_rate), AV_OPT_TYPE_INT,   {.i64=44100},  1, INT_MAX, AF },
    { "nb_samples",  "set the number of samples per requested frame", OFFSET(nb_samples),  AV_OPT_TYPE_INT,   {.i64=1024},   1, INT_MAX, AF },
    { "n",           "set the number of samples per requested frame", OFFSET(nb_samples),  AV_OPT_TYPE_INT,   {.i64=1024},   1, INT_MAX, AF },
    { "hp",          "set high-pass filter frequency",                OFFSET(Fc0),         AV_OPT_TYPE_FLOAT, {.dbl=0},      0, INT_MAX, AF },
    { "lp",          "set low-pass filter frequency",                 OFFSET(Fc1),         AV_OPT_TYPE_FLOAT, {.dbl=0},      0, INT_MAX, AF },
    { "phase",       "set filter phase response",                     OFFSET(phase),       AV_OPT_TYPE_FLOAT, {.dbl=50},     0,     100, AF },
    { "beta",        "set kaiser window beta",                        OFFSET(beta),        AV_OPT_TYPE_DOUBLE,{.dbl=-1},    -1,     256, AF },
    { "att",         "set stop-band attenuation",                     OFFSET(att),         AV_OPT_TYPE_DOUBLE,{.dbl=120},   40,     300, AF },
    { "round",       "enable rounding",                               OFFSET(round),       AV_OPT_TYPE_BOOL,  {.i64=0},      0,       1, AF },
    { "ht",          "set high-pass filter transition band-width",    OFFSET(tbw0),        AV_OPT_TYPE_FLOAT, {.dbl=0.05},   0,     1.0, AF },
    { "lt",          "set low-pass filter transition band-width",     OFFSET(tbw1),        AV_OPT_TYPE_FLOAT, {.dbl=0.05},   0,     1.0, AF },
    { "hptaps",      "set number of taps for high-pass filter",       OFFSET(num_taps[0]), AV_OPT_TYPE_INT,   {.i64=0},      0, 1048575, AF },
    { "lptaps",      "set number of taps for low-pass filter",        OFFSET(num_taps[1]), AV_OPT_TYPE_INT,   {.i64=0},      0, 1048575, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(sinc);

const FFFilter ff_asrc_sinc = {
    .p.name        = "sinc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate a sinc kaiser-windowed low-pass, high-pass, band-pass, or band-reject FIR coefficients."),
    .p.priv_class  = &sinc_class,
    .priv_size     = sizeof(SincContext),
    .uninit        = uninit,
    .activate      = activate,
    FILTER_OUTPUTS(sinc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
