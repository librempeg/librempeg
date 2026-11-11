/*
 * Copyright (c) 2026 Paul B Mahol
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

typedef struct AudioFIRWSourceContext {
    const AVClass *class;

    int curve;
    int invert;
    int nb_taps;
    int sample_rate;
    int nb_samples;
    double normf;
    double phase;
    double percentage;

    int64_t pts;

    void *taps;
    void *realf;
    void *complexf;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} AudioFIRWSourceContext;

#define OFFSET(x) offsetof(AudioFIRWSourceContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

typedef struct WPreset {
    char name[8];
    char description[24];
    double tau[3];
} WPreset;

static const WPreset w_curve[] = {
    { "col",    "Columbia",          { 0.001590, 0.000318, 0.000100 } },
    { "emi",    "EMI",               { 0.002500, 0.000500, 0.000070 } },
    { "bsi",    "BSI (78RPM)",       { 0.003180, 0.000353, 0.000050 } },
    { "riaa",   "RIAA",              { 0.003180, 0.000318, 0.000075 } },
    { "cd",     "Compact Disc (CD)", { 0.000050, 0.000015, 0.000000 } },
    { "50fm",   "50µs (FM)",         { 0.000050, 0.000000, 0.000000 } },
    { "75fm",   "75µs (FM)",         { 0.000075, 0.000000, 0.000000 } },
    { "iecn78", "IEC N78",           { 0.003180, 0.000450, 0.000050 } },
    { "teldec", "TELDEC",            { 0.003180, 0.000318, 0.000050 } },
    { "london", "LONDON",            { 0.001590, 0.000318, 0.000050 } },
    { "nartb",  "NARTB",             { 0.003180, 0.000318, 0.000100 } },
    { "bl300",  "Blumlein 300",      { INFINITY, 0.000531, 0.000000 } },
};

static const AVOption afirwsrc_options[] = {
    { "curve","set the weighting curve", OFFSET(curve), AV_OPT_TYPE_INT, {.i64=0}, 0, FF_ARRAY_ELEMS(w_curve)-1, FLAGS, .unit = "curve" },
    { "c",    "set the weighting curve", OFFSET(curve), AV_OPT_TYPE_INT, {.i64=0}, 0, FF_ARRAY_ELEMS(w_curve)-1, FLAGS, .unit = "curve" },
    { w_curve[ 0].name, w_curve[ 0].description, 0, AV_OPT_TYPE_CONST, {.i64= 0}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 1].name, w_curve[ 1].description, 0, AV_OPT_TYPE_CONST, {.i64= 1}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 2].name, w_curve[ 2].description, 0, AV_OPT_TYPE_CONST, {.i64= 2}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 3].name, w_curve[ 3].description, 0, AV_OPT_TYPE_CONST, {.i64= 3}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 4].name, w_curve[ 4].description, 0, AV_OPT_TYPE_CONST, {.i64= 4}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 5].name, w_curve[ 5].description, 0, AV_OPT_TYPE_CONST, {.i64= 5}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 6].name, w_curve[ 6].description, 0, AV_OPT_TYPE_CONST, {.i64= 6}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 7].name, w_curve[ 7].description, 0, AV_OPT_TYPE_CONST, {.i64= 7}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 8].name, w_curve[ 8].description, 0, AV_OPT_TYPE_CONST, {.i64= 8}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[ 9].name, w_curve[ 9].description, 0, AV_OPT_TYPE_CONST, {.i64= 9}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[10].name, w_curve[10].description, 0, AV_OPT_TYPE_CONST, {.i64=10}, 0, 0, FLAGS, .unit = "curve" },
    { w_curve[11].name, w_curve[11].description, 0, AV_OPT_TYPE_CONST, {.i64=11}, 0, 0, FLAGS, .unit = "curve" },
    { "taps", "set the number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64=4096}, 16, UINT16_MAX, FLAGS },
    { "t",    "set the number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64=4096}, 16, UINT16_MAX, FLAGS },
    { "invert", "set the weigthing inversion", OFFSET(invert), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "i",      "set the weigthing inversion", OFFSET(invert), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "normalization", "set the normalization frequency", OFFSET(normf), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 1, INT_MAX, FLAGS },
    { "f",             "set the normalization frequency", OFFSET(normf), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 1, INT_MAX, FLAGS },
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, FLAGS },
    { "r",           "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100}, 1, INT_MAX, FLAGS },
    { "nb_samples", "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "n",          "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "phase","set the phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1, 1, FLAGS },
    { "h",    "set the phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1, 1, FLAGS },
    { "percentage", "set the magnitude percentage", OFFSET(percentage), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 0, 100, FLAGS },
    { "e",          "set the magnitude percentage", OFFSET(percentage), AV_OPT_TYPE_DOUBLE, {.dbl=100}, 0, 100, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(afirwsrc);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AudioFIRWSourceContext *s = ctx->priv;
    static const AVChannelLayout chlayouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_DBL,
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

#define DEPTH 32
#include "afirwsrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "afirwsrc_template.c"

static av_cold int config_output(AVFilterLink *outlink)
{
    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLT:
        return config_output_flt(outlink);
    case AV_SAMPLE_FMT_DBL:
        return config_output_dbl(outlink);
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFIRWSourceContext *s = ctx->priv;
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

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLT:
        memcpy(frame->data[0], (float *)s->taps + s->pts, nb_samples * sizeof(float));
        break;
    case AV_SAMPLE_FMT_DBL:
        memcpy(frame->data[0], (double *)s->taps + s->pts, nb_samples * sizeof(double));
        break;
    }

    frame->pts = s->pts;
    s->pts    += nb_samples;
    return ff_filter_frame(outlink, frame);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioFIRWSourceContext *s = ctx->priv;

    av_freep(&s->taps);
    av_freep(&s->realf);
    av_freep(&s->complexf);

    av_tx_uninit(&s->tx_ctx);
    av_tx_uninit(&s->itx_ctx);
}

static const AVFilterPad afirwsrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_asrc_afirwsrc = {
    .p.name        = "afirwsrc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate a FIR weighting coefficients audio stream."),
    .p.priv_class  = &afirwsrc_class,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(AudioFIRWSourceContext),
    FILTER_OUTPUTS(afirwsrc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
