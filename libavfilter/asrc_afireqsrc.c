/*
 * Copyright (c) 2020 Paul B Mahol
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

enum {
    INTERP_LINEAR,
    INTERP_CUBIC,
    INTERP_COSINE,
    INTERP_SMOOTH,
    NB_INTERP
};

typedef struct AudioFIRSourceContext {
    const AVClass *class;

    float *freq_points_opt;
    unsigned nb_freq;
    float *magnitude_opt;
    unsigned nb_magnitude;
    int nb_taps;
    int sample_rate;
    int nb_samples;
    int preset;
    int interp;
    int phaset;

    int64_t pts;

    void *taps;
    void *realf;
    void *complexf;
    void *freq;
    void *magnitude;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} AudioFIRSourceContext;

#define OFFSET(x) offsetof(AudioFIRSourceContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

#define DEFAULT_BANDS "25 40 63 100 160 250 400 630 1000 1600 2500 4000 6300 10000 16000 24000"
#define DEFAULT_GAINS "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"

static const AVOptionArrayDef def_bands = {.def=DEFAULT_BANDS,.size_min=1,.sep=' '};
static const AVOptionArrayDef def_gains = {.def=DEFAULT_GAINS,.size_min=1,.sep=' '};

typedef struct EqPreset {
    char name[16];
    float gains[16];
} EqPreset;

static const EqPreset eq_presets[] = {
    { "flat",          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "acoustic",      { 5.0, 4.5, 4.0, 3.5, 1.5, 1.0, 1.5, 1.5, 2.0, 3.0, 3.5, 4.0, 3.7, 3.0, 3.0 } },
    { "bass",          { 10.0, 8.8, 8.5, 6.5, 2.5, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "beats",         { -5.5, -5.0, -4.5, -4.2, -3.5, -3.0, -1.9, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "classic",       { -0.3, 0.3, -3.5, -9.0, -1.0, 0.0, 1.8, 2.1, 0.0, 0.0, 0.0, 4.4, 9.0, 9.0, 9.0 } },
    { "clear",         { 3.5, 5.5, 6.5, 9.5, 8.0, 6.5, 3.5, 2.5, 1.3, 5.0, 7.0, 9.0, 10.0, 11.0, 9.0 } },
    { "deep bass",     { 12.0, 8.0, 0.0, -6.7, -12.0, -9.0, -3.5, -3.5, -6.1, 0.0, -3.0, -5.0, 0.0, 1.2, 3.0 } },
    { "dubstep",       { 12.0, 10.0, 0.5, -1.0, -3.0, -5.0, -5.0, -4.8, -4.5, -2.5, -1.0, 0.0, -2.5, -2.5, 0.0 } },
    { "electronic",    { 4.0, 4.0, 3.5, 1.0, 0.0, -0.5, -2.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 4.0, 4.5 } },
    { "hardstyle",     { 6.1, 7.0, 12.0, 6.1, -5.0, -12.0, -2.5, 3.0, 6.5, 0.0, -2.2, -4.5, -6.1, -9.2, -10.0 } },
    { "hip-hop",       { 4.5, 4.3, 4.0, 2.5, 1.5, 3.0, -1.0, -1.5, -1.5, 1.5, 0.0, -1.0, 0.0, 1.5, 3.0 } },
    { "jazz",          { 0.0, 0.0, 0.0, 2.0, 4.0, 5.9, -5.9, -4.5, -2.5, 2.5, 1.0, -0.8, -0.8, -0.8, -0.8 } },
    { "metal",         { 10.5, 10.5, 7.5, 0.0, 2.0, 5.5, 0.0, 0.0, 0.0, 6.1, 0.0, 0.0, 6.1, 10.0, 12.0 } },
    { "movie",         { 3.0, 3.0, 6.1, 8.5, 9.0, 7.0, 6.1, 6.1, 5.0, 8.0, 3.5, 3.5, 8.0, 10.0, 8.0 } },
    { "pop",           { 0.0, 0.0, 0.0, 0.0, 0.0, 1.3, 2.0, 2.5, 5.0, -1.5, -2.0, -3.0, -3.0, -3.0, -3.0 } },
    { "r&b",           { 3.0, 3.0, 7.0, 6.1, 4.5, 1.5, -1.5, -2.0, -1.5, 2.0, 2.5, 3.0, 3.5, 3.8, 4.0 } },
    { "rock",          { 0.0, 0.0, 0.0, 3.0, 3.0, -10.0, -4.0, -1.0, 0.8, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0 } },
    { "vocal booster", { -1.5, -2.0, -3.0, -3.0, -0.5, 1.5, 3.5, 3.5, 3.5, 3.0, 2.0, 1.5, 0.0, 0.0, -1.5 } },
};

static const AVOption afireqsrc_options[] = {
    { "preset","set equalizer preset", OFFSET(preset), AV_OPT_TYPE_INT, {.i64=0}, -1, FF_ARRAY_ELEMS(eq_presets)-1, FLAGS, .unit = "preset" },
    { "p",     "set equalizer preset", OFFSET(preset), AV_OPT_TYPE_INT, {.i64=0}, -1, FF_ARRAY_ELEMS(eq_presets)-1, FLAGS, .unit = "preset" },
    { "custom",            NULL, 0, AV_OPT_TYPE_CONST, {.i64=-1}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 0].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 0}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 1].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 1}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 2].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 2}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 3].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 3}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 4].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 4}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 5].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 5}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 6].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 6}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 7].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 7}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 8].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 8}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[ 9].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64= 9}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[10].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=10}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[11].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=11}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[12].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=12}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[13].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=13}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[14].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=14}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[15].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=15}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[16].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=16}, 0, 0, FLAGS, .unit = "preset" },
    { eq_presets[17].name, NULL, 0, AV_OPT_TYPE_CONST, {.i64=17}, 0, 0, FLAGS, .unit = "preset" },
    { "gains", "set gain values per band", OFFSET(magnitude_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, INT16_MIN, INT16_MAX, FLAGS },
    { "g",     "set gain values per band", OFFSET(magnitude_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, INT16_MIN, INT16_MAX, FLAGS },
    { "bands", "set central frequency values per band", OFFSET(freq_points_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "b",     "set central frequency values per band", OFFSET(freq_points_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "taps", "set number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64=4096}, 16, UINT16_MAX, FLAGS },
    { "t",    "set number of taps", OFFSET(nb_taps), AV_OPT_TYPE_INT, {.i64=4096}, 16, UINT16_MAX, FLAGS },
    { "sample_rate", "set sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100},  1, INT_MAX,    FLAGS },
    { "r",           "set sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=44100},  1, INT_MAX,    FLAGS },
    { "nb_samples", "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "n",          "set the number of samples per requested frame", OFFSET(nb_samples), AV_OPT_TYPE_INT, {.i64 = 1024}, 1, INT_MAX, FLAGS },
    { "interp","set the interpolation", OFFSET(interp), AV_OPT_TYPE_INT, {.i64=0}, 0, NB_INTERP-1, FLAGS, .unit = "interp" },
    { "i",     "set the interpolation", OFFSET(interp), AV_OPT_TYPE_INT, {.i64=0}, 0, NB_INTERP-1, FLAGS, .unit = "interp" },
    { "linear", NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_LINEAR}, 0, 0, FLAGS, .unit = "interp" },
    { "cubic",  NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_CUBIC},  0, 0, FLAGS, .unit = "interp" },
    { "cosine", NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_COSINE}, 0, 0, FLAGS, .unit = "interp" },
    { "smooth", NULL, 0, AV_OPT_TYPE_CONST, {.i64=INTERP_SMOOTH}, 0, 0, FLAGS, .unit = "interp" },
    { "phase","set the phase", OFFSET(phaset), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS, .unit = "phase" },
    { "h",    "set the phase", OFFSET(phaset), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS, .unit = "phase" },
    { "linear", "linear phase",  0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, FLAGS, .unit = "phase" },
    { "min",    "minimum phase", 0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, FLAGS, .unit = "phase" },
    {NULL}
};

AVFILTER_DEFINE_CLASS(afireqsrc);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const AudioFIRSourceContext *s = ctx->priv;
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
#include "afireqsrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "afireqsrc_template.c"

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
    AudioFIRSourceContext *s = ctx->priv;
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
    AudioFIRSourceContext *s = ctx->priv;

    av_freep(&s->taps);
    av_freep(&s->freq);
    av_freep(&s->magnitude);
    av_freep(&s->realf);
    av_freep(&s->complexf);

    av_tx_uninit(&s->tx_ctx);
    av_tx_uninit(&s->itx_ctx);
}

static const AVFilterPad afireqsrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_asrc_afireqsrc = {
    .p.name        = "afireqsrc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate a FIR equalizer coefficients audio stream."),
    .p.priv_class  = &afireqsrc_class,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(AudioFIRSourceContext),
    FILTER_OUTPUTS(afireqsrc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
