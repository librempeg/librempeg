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

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intmath.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

#define MIN_NONZERO 32          /* fewer nonzero samples => block is ambiguous */
#define INV_SQRT12  0.28867513459481287

enum { MODE_ROUND, MODE_TRUNC };

typedef struct ADequantContext {
    const AVClass *class;

    int    native;
    int    bits;
    int    mode;
    int    block;
    double tol;
    int    iters;
    double tau;
    int    outbits;

    int     overlap;
    int     rdft_size;
    int     nb_channels;
    int     trim_size;
    int     flush_size;
    int64_t last_pts;

    double  sq;
    void    *window;
    void    *state;

    int (*adequant_init)(AVFilterContext *ctx);
    void (*adequant_uninit)(AVFilterContext *ctx);
    int (*adequant_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
} ADequantContext;

#define OFFSET(x) offsetof(ADequantContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS FLAGS | AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption adequant_options[] = {
    { "native",  "bit depth of the input audio",
                 OFFSET(native),  AV_OPT_TYPE_INT,    { .i64 = 24 },    8, 32, TFLAGS },
    { "bits",    "force a fixed effective bit depth",
                 OFFSET(bits),    AV_OPT_TYPE_INT,    { .i64 = 0 },     0, 32, TFLAGS },
    { "mode",    "quantizer type of the source",
                 OFFSET(mode),    AV_OPT_TYPE_INT,    { .i64 = MODE_ROUND }, 0, 1, TFLAGS, .unit = "mode" },
    {   "round", "cell centered in the middle", 0, AV_OPT_TYPE_CONST, { .i64 = MODE_ROUND }, 0, 0, TFLAGS, .unit = "mode" },
    {   "trunc", "bits dropped",                0, AV_OPT_TYPE_CONST, { .i64 = MODE_TRUNC }, 0, 0, TFLAGS, .unit = "mode" },
    { "block",   "processing block size in samples",
                 OFFSET(block),   AV_OPT_TYPE_INT,    { .i64 = 1024 },  64, 65536, FLAGS },
    { "tol",     "fraction of nonzero samples that must lie on the lattice",
                 OFFSET(tol),     AV_OPT_TYPE_DOUBLE, { .dbl = 0.995 }, 0.5, 1.0, TFLAGS },
    { "iters",   "number of reconstruction iterations",
                 OFFSET(iters),   AV_OPT_TYPE_INT,    { .i64 = 120 },   1, 5000, TFLAGS },
    { "tau",     "initial threshold in units of quantization-noise std",
                 OFFSET(tau),     AV_OPT_TYPE_DOUBLE, { .dbl = 1.0 },   0, 50, TFLAGS },
    { "outbits", "bit depth of the output audio",
                 OFFSET(outbits), AV_OPT_TYPE_INT,    { .i64 = 24 },    16, 32, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adequant);

#define DEPTH 64
#include "adequant_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ADequantContext *s = ctx->priv;

    if (s->outbits < s->native)
        av_log(ctx, AV_LOG_WARNING, "outbits is below the source depth\n");

    s->rdft_size = s->block;
    s->overlap = s->rdft_size/2;
    s->trim_size = s->overlap/2;
    s->flush_size = s->overlap/2;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S32P:
        s->adequant_channel = adequant_channel_dblp;
        s->adequant_uninit = adequant_uninit_dblp;
        s->adequant_init = adequant_init_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->adequant_init(ctx);
}

typedef struct ThreadData {
    AVFrame *in;
    AVFrame *out;
} ThreadData;

static int adequant_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ADequantContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    const int start = ff_slice_pos(out->ch_layout.nb_channels, jobnr, nb_jobs);
    const int end = ff_slice_pos(out->ch_layout.nb_channels, jobnr+1, nb_jobs);

    for (int ch = start; ch < end; ch++)
        s->adequant_channel(ctx, in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ADequantContext *s = ctx->priv;
    int extra_samples, nb_samples;
    ThreadData td;
    AVFrame *out;

    extra_samples = in->nb_samples % s->overlap;
    if (extra_samples)
        extra_samples = FFMIN(s->overlap - extra_samples, s->flush_size);
    nb_samples = in->nb_samples;
    if (extra_samples > 0) {
        nb_samples += extra_samples;
        s->flush_size -= extra_samples;
    }

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, adequant_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = in->pts;
    out->pts -= av_rescale_q(s->rdft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);
    s->last_pts = out->pts + out->duration;

    if (s->trim_size > 0 && s->trim_size < out->nb_samples) {
        for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
            out->extended_data[ch] += s->trim_size * av_get_bytes_per_sample(out->format);

        out->nb_samples -= s->trim_size;
        s->trim_size = 0;
    } else if (s->trim_size > 0) {
        s->trim_size -= out->nb_samples;
        ff_graph_frame_free(ctx, &out);
        ff_graph_frame_free(ctx, &in);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    ADequantContext *s = ctx->priv;
    int ret, status, wanted;
    AVFrame *in = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    wanted = s->overlap;
    ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_samples(inlink) >= s->overlap) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;

    if (s->adequant_uninit)
        s->adequant_uninit(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_adequant = {
    .p.name        = "adequant",
    .p.description = NULL_IF_CONFIG_SMALL("Apply Dynamic Dequantization of input audio."),
    .p.priv_class  = &adequant_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(ADequantContext),
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_S32P),
    .process_command = ff_filter_process_command,
};
