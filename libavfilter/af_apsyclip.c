/*
 * Copyright (c) 2014 - 2021 Jason Jang
 * Copyright (c) 2021 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioPsyClipContext {
    const AVClass *class;

    float *bands_opt;
    unsigned nb_bands_opt;
    float *gains_opt;
    unsigned nb_gains_opt;

    double level_in;
    double level_out;
    double clip_level;
    double adaptive;
    int auto_level;
    int max_iterations;
    int min_iterations;
    int diff_only;

    int num_psy_bins;
    int nb_bins;
    int fft_size;
    int overlap;

    int trim_size;
    int flush_size;
    int64_t last_pts;

    void *st;

    AVFrame *in;
    AVFrame *in_buffer;
    AVFrame *in_frame;
    AVFrame *out_dist_frame;
    AVFrame *windowed_frame;
    AVFrame *clipping_delta;
    AVFrame *spectrum_buf;
    AVFrame *mask_curve;

    int (*psy_init)(AVFilterContext *ctx);
    int (*psy_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
    int (*psy_flush_channel)(AVFilterContext *ctx, AVFrame *out, const int ch);
    void (*psy_uninit)(AVFilterContext *ctx);
} AudioPsyClipContext;

#define OFFSET(x) offsetof(AudioPsyClipContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

#define DEFAULT_BANDS "0 125 250 500 1000 2000 4000 8000 16000 20000"
#define DEFAULT_GAINS "14 14 16 18 20 20 20 17 14 -10"

static const AVOptionArrayDef def_bands = {.def=DEFAULT_BANDS,.size_min=2,.sep=' '};
static const AVOptionArrayDef def_gains = {.def=DEFAULT_GAINS,.size_min=2,.sep=' '};

static const AVOption apsyclip_options[] = {
    { "level_in",   "set input level",         OFFSET(level_in),   AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,   64, FLAGS },
    { "level_out",  "set output level",        OFFSET(level_out),  AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,   64, FLAGS },
    { "clip",       "set clip level",          OFFSET(clip_level), AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,    1, FLAGS },
    { "diff",       "enable difference",       OFFSET(diff_only),  AV_OPT_TYPE_BOOL,   {.i64=0},      0,    1, FLAGS },
    { "adaptive",   "set adaptive distortion", OFFSET(adaptive),   AV_OPT_TYPE_DOUBLE, {.dbl=0.5},    0,    1, FLAGS },
    { "iterations", "set max iterations",      OFFSET(max_iterations), AV_OPT_TYPE_INT,{.i64=10},     1,   20, FLAGS },
    { "min_iterations", "set min iterations",  OFFSET(min_iterations), AV_OPT_TYPE_INT,{.i64=1},      1,   20, FLAGS },
    { "level",      "set auto level",          OFFSET(auto_level), AV_OPT_TYPE_BOOL,   {.i64=0},      0,    1, FLAGS },
    { "bands", "set frequency values per band",OFFSET(bands_opt),  AV_OPT_TYPE_FLOAT|AR,{.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "gains", "set gain values per band",     OFFSET(gains_opt),  AV_OPT_TYPE_FLOAT|AR,{.arr=&def_gains}, INT_MIN, INT_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(apsyclip);

#define DEPTH 32
#include "apsyclip_template.c"

#undef DEPTH
#define DEPTH 64
#include "apsyclip_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioPsyClipContext *s = ctx->priv;

    s->fft_size = inlink->sample_rate > 100000 ? 1024 : inlink->sample_rate > 50000 ? 512 : 256;
    s->nb_bins = s->fft_size / 2 + 1;
    s->overlap = s->fft_size / 4;
    s->trim_size = s->fft_size - s->overlap;
    s->flush_size = s->fft_size - s->overlap;

    // The psy masking calculation is O(n^2),
    // so skip it for frequencies not covered by base sampling rates (i.e. 44.1k)
    if (inlink->sample_rate <= 50000) {
        s->num_psy_bins = s->fft_size / 2;
    } else if (inlink->sample_rate <= 100000) {
        s->num_psy_bins = s->fft_size / 4;
    } else {
        s->num_psy_bins = s->fft_size / 8;
    }

    s->in_buffer      = ff_get_audio_buffer(inlink, s->fft_size*2);
    s->in_frame       = ff_get_audio_buffer(inlink, s->fft_size);
    s->out_dist_frame = ff_get_audio_buffer(inlink, s->fft_size);
    s->windowed_frame = ff_get_audio_buffer(inlink, s->fft_size);
    s->clipping_delta = ff_get_audio_buffer(inlink, s->fft_size);
    s->spectrum_buf   = ff_get_audio_buffer(inlink, s->fft_size+2);
    s->mask_curve     = ff_get_audio_buffer(inlink, s->nb_bins);
    if (!s->in_buffer || !s->in_frame ||
        !s->out_dist_frame || !s->windowed_frame ||
        !s->clipping_delta || !s->spectrum_buf || !s->mask_curve)
        return AVERROR(ENOMEM);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->psy_init = psy_init_fltp;
        s->psy_uninit = psy_uninit_fltp;
        s->psy_channel = psy_channel_fltp;
        s->psy_flush_channel = psy_flush_channel_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->psy_init = psy_init_dblp;
        s->psy_uninit = psy_uninit_dblp;
        s->psy_channel = psy_channel_dblp;
        s->psy_flush_channel = psy_flush_channel_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->psy_init(ctx);
}

static int psy_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioPsyClipContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->psy_channel(ctx, s->in, out, ch);

    return 0;
}

static int psy_flush_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioPsyClipContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->psy_flush_channel(ctx, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPsyClipContext *s = ctx->priv;
    int extra_samples, nb_samples;
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

    s->in = in;
    ff_filter_execute(ctx, psy_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = in->pts;
    out->pts -= av_rescale_q(s->fft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);

    s->last_pts = out->pts + out->duration;

    if (s->trim_size > 0 && s->trim_size < out->nb_samples) {
        for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
            out->extended_data[ch] += s->trim_size * av_get_bytes_per_sample(out->format);

        out->nb_samples -= s->trim_size;
        s->trim_size = 0;
    } else if (s->trim_size > 0) {
        s->trim_size -= out->nb_samples;
        av_frame_free(&out);
        av_frame_free(&in);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    s->in = NULL;
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioPsyClipContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = FFMIN(s->flush_size, s->overlap);
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        ff_filter_execute(ctx, psy_flush_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));


        out->pts = s->last_pts;
        out->duration = av_rescale_q(out->nb_samples,
                                     (AVRational){1, outlink->sample_rate},
                                     outlink->time_base);
        s->last_pts += out->duration;

        ret = ff_filter_frame(outlink, out);
        if (ret < 0)
            break;
    }

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPsyClipContext *s = ctx->priv;
    int ret, status, available, wanted;
    AVFrame *in = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    available = ff_inlink_queued_samples(inlink);
    wanted = FFMAX(s->overlap, (available / s->overlap) * s->overlap);
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
        if (s->flush_size > 0)
            ret = flush_frame(outlink);

        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioPsyClipContext *s = ctx->priv;

    av_frame_free(&s->in_buffer);
    av_frame_free(&s->in_frame);
    av_frame_free(&s->out_dist_frame);
    av_frame_free(&s->windowed_frame);
    av_frame_free(&s->clipping_delta);
    av_frame_free(&s->spectrum_buf);
    av_frame_free(&s->mask_curve);

    if (s->psy_uninit)
        s->psy_uninit(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_apsyclip = {
    .p.name          = "apsyclip",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Psychoacoustic Clipper."),
    .p.priv_class    = &apsyclip_class,
    .priv_size       = sizeof(AudioPsyClipContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
