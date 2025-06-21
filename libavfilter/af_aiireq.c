/*
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

#include <float.h>
#include <math.h>

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"

typedef struct AudioIIREQContext {
    const AVClass *class;

    float *band_opt;
    unsigned nb_bands;
    float *gain_opt;
    unsigned nb_gains;
    int *section_opt;
    unsigned nb_sections;

    float overall_gain;

    void *eqs;
    void *chs;

    void (*update_filter)(AVFilterContext *ctx);
    void (*filter_channel)(AVFilterContext *ctx, AVFrame *out, AVFrame *in, int ch);
} AudioIIREQContext;

#define OFFSET(x) offsetof(AudioIIREQContext, x)
#define FLAGS (AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)
#define TFLAGS (AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM)
#define AR AV_OPT_TYPE_FLAG_ARRAY

#define DEFAULT_BANDS "25 40 63 100 160 250 400 630 1000 1600 2500 4000 6300 10000 16000 24000"
#define DEFAULT_GAINS "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
#define DEFAULT_SECTIONS "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"

static const AVOptionArrayDef def_bands = {.def=DEFAULT_BANDS,.size_min=1,.sep=' '};
static const AVOptionArrayDef def_gains = {.def=DEFAULT_GAINS,.size_min=1,.sep=' '};
static const AVOptionArrayDef def_sections = {.def=DEFAULT_SECTIONS,.size_min=1,.sep=' '};

static const AVOption aiireq_options[] = {
    { "gains", "set gain values per band", OFFSET(gain_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, -64, 64, TFLAGS },
    { "g",     "set gain values per band", OFFSET(gain_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, -64, 64, TFLAGS },
    { "bands", "set central frequency values per band", OFFSET(band_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "b",     "set central frequency values per band", OFFSET(band_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "sections", "set number of sections per band", OFFSET(section_opt), AV_OPT_TYPE_INT|AR, {.arr=&def_sections}, 1, 6, FLAGS },
    { "s",        "set number of sections per band", OFFSET(section_opt), AV_OPT_TYPE_INT|AR, {.arr=&def_sections}, 1, 6, FLAGS },
    { "gain",  "set output gain",                    OFFSET(overall_gain), AV_OPT_TYPE_FLOAT, {.dbl=1.0},           0, 2, TFLAGS },
    { "a",     "set output gain",                    OFFSET(overall_gain), AV_OPT_TYPE_FLOAT, {.dbl=1.0},           0, 2, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aiireq);

#undef DEPTH
#define DEPTH 32
#include "aiireq_template.c"

#undef DEPTH
#define DEPTH 64
#include "aiireq_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        return init_filter_flt(ctx);
    case AV_SAMPLE_FMT_DBLP:
        return init_filter_dbl(ctx);
    default:
        av_assert0(0);
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioIIREQContext *s = ctx->priv;

    av_freep(&s->eqs);
    av_freep(&s->chs);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg,
                           int jobnr, int nb_jobs)
{
    AudioIIREQContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->filter_channel(ctx, out, in, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in) && !ff_filter_disabled(ctx)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;

    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (in != out)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AudioIIREQContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    s->update_filter(ctx);

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .config_props   = config_input,
        .filter_frame   = filter_frame,
    },
};

const FFFilter ff_af_aiireq = {
    .p.name        = "aiireq",
    .p.description = NULL_IF_CONFIG_SMALL("Apply audio IIR multi band equalizer."),
    .p.priv_class  = &aiireq_class,
    .priv_size     = sizeof(AudioIIREQContext),
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
