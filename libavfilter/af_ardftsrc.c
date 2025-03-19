/*
 * Copyright (c) 2023 Paul B Mahol
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
#include "formats.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioRDFTSRCContext {
    const AVClass *class;

    int in_planar;
    int out_planar;
    int pass;
    int quality;
    int sample_rate;
    int in_rdft_size;
    int out_rdft_size;
    int in_nb_samples;
    int tr_nb_samples;
    int taper_samples;
    int out_nb_samples;
    int in_offset;
    int out_offset;
    int channels;
    float bandwidth;
    int64_t delay;
    int64_t last_out_pts;
    int trim_size;
    int flush_size;

    void *taper;

    AVFrame *in;

    void *state;

    int (*flush_src)(AVFilterContext *ctx, AVFrame *out, const int ch);

    int (*do_src)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                  const int ch, const int soffset, const int doffset);

    void (*src_uninit)(AVFilterContext *ctx);
} AudioRDFTSRCContext;

#define OFFSET(x) offsetof(AudioRDFTSRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ardftsrc_options[] = {
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS },
    { "quality", "set the quality", OFFSET(quality), AV_OPT_TYPE_INT, {.i64=1024}, 1, INT32_MAX, FLAGS },
    { "bandwidth", "set the bandwidth", OFFSET(bandwidth), AV_OPT_TYPE_FLOAT, {.dbl=0.95}, 0, 1, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(ardftsrc);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioRDFTSRCContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    int ret, sample_rates[] = { s->sample_rate, -1 };
    AVFilterFormats *formats;

    formats = ff_make_format_list(sample_fmts);
    if (formats)
        formats->flags = FILTER_SAME_BITDEPTH;
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(sample_fmts);
    if (formats)
        formats->flags = FILTER_SAME_BITDEPTH;
    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    if ((ret = ff_formats_ref(ff_all_samplerates(),
                              &cfg_in[0]->samplerates)) < 0)
        return ret;

    if (!s->sample_rate) {
        return ff_formats_ref(ff_all_samplerates(),
                              &cfg_out[0]->samplerates);
    }

    return ff_formats_ref(ff_make_format_list(sample_rates),
                          &cfg_out[0]->samplerates);
}

#define DEPTH 16
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 32
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "ardftsrc_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    int max_nb_samples, ret;
    int64_t factor;

    if (inlink->sample_rate == outlink->sample_rate) {
        s->pass = 1;
        return 0;
    }

    outlink->time_base = (AVRational) {1, outlink->sample_rate};

    av_reduce(&s->in_nb_samples, &s->out_nb_samples,
              inlink->sample_rate, outlink->sample_rate, INT_MAX);

    factor = lrint(2.0*ceil(s->quality/(2.0*s->out_nb_samples)));
    max_nb_samples = 2*FFMAX(s->in_nb_samples, s->out_nb_samples);
    factor = FFMIN(factor, INT32_MAX/max_nb_samples);
    s->in_nb_samples *= factor;
    s->out_nb_samples *= factor;

    s->out_planar = av_sample_fmt_is_planar(ctx->outputs[0]->format);
    s->in_planar = av_sample_fmt_is_planar(inlink->format);
    s->in_rdft_size = s->in_nb_samples * 2;
    s->out_rdft_size = s->out_nb_samples * 2;
    s->out_offset = s->trim_size = (s->out_rdft_size - s->out_nb_samples) >> 1;
    s->in_offset = s->flush_size = (s->in_rdft_size - s->in_nb_samples) >> 1;
    s->delay = av_rescale_q(s->in_offset, (AVRational){ 1, inlink->sample_rate }, inlink->time_base);
    s->tr_nb_samples = FFMIN(s->in_nb_samples, s->out_nb_samples);
    s->taper_samples = lrint(s->tr_nb_samples * (1.0-s->bandwidth));
    av_log(ctx, AV_LOG_DEBUG, "factor: %"PRId64" | %d => %d | delay: %"PRId64"\n", factor, s->in_rdft_size, s->out_rdft_size, s->delay);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S16P:
        s->flush_src = flush_s16p;
        s->do_src = src_s16p;
        s->src_uninit = src_uninit_s16p;
        ret = src_init_s16p(ctx);
        break;
    case AV_SAMPLE_FMT_FLT:
    case AV_SAMPLE_FMT_FLTP:
        s->flush_src = flush_fltp;
        s->do_src = src_fltp;
        s->src_uninit = src_uninit_fltp;
        ret = src_init_fltp(ctx);
        break;
    case AV_SAMPLE_FMT_DBL:
    case AV_SAMPLE_FMT_DBLP:
        s->flush_src = flush_dblp;
        s->do_src = src_dblp;
        s->src_uninit = src_uninit_dblp;
        ret = src_init_dblp(ctx);
        break;
    default:
        return AVERROR_BUG;
    }

    return ret;
}

static int flush_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->flush_src(ctx, out, ch);

    return 0;
}

static int src_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        for (int soffset = 0, doffset = 0; soffset < s->in->nb_samples;
             soffset += s->in_nb_samples, doffset += s->out_nb_samples)
            s->do_src(ctx, s->in, out, ch, soffset, doffset);
    }

    return 0;
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioRDFTSRCContext *s = ctx->priv;
    const int nb_samples = av_rescale(s->flush_size, s->out_nb_samples, s->in_nb_samples);
    AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

    if (!out)
        return AVERROR(ENOMEM);

    s->flush_size = 0;

    ff_filter_execute(ctx, flush_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->sample_rate = outlink->sample_rate;
    out->pts = s->last_out_pts;
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    return ff_filter_frame(outlink, out);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    int ret, in_samples;
    AVFrame *out;

    if (s->pass)
        return ff_filter_frame(outlink, in);

    in_samples = (in->nb_samples < s->in_nb_samples) ? FFMIN(in->nb_samples+s->in_offset, s->in_nb_samples) : in->nb_samples;
    s->flush_size -= FFMAX(in_samples-in->nb_samples, 0);

    out = ff_get_audio_buffer(outlink, av_rescale(in_samples, s->out_nb_samples, s->in_nb_samples));
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    av_frame_copy_props(out, in);

    ff_filter_execute(ctx, src_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->sample_rate = outlink->sample_rate;
    out->pts = av_rescale_q(in->pts - (s->trim_size ? 0 : s->delay), inlink->time_base, outlink->time_base);
    if (s->trim_size > 0) {
        for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
            out->extended_data[ch] += s->trim_size * av_get_bytes_per_sample(out->format);
    }

    out->nb_samples -= s->trim_size;
    s->trim_size = 0;

    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    s->last_out_pts = out->pts + out->duration;

    ret = ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    s->in = NULL;
    return ret < 0 ? ret : 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    int ret, status;
    AVFrame *in = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (inlink->sample_rate == outlink->sample_rate) {
        ret = ff_inlink_consume_frame(inlink, &in);
    } else {
        const int available = ff_inlink_queued_samples(inlink);
        const int is_eof = ff_inlink_check_available_samples(inlink, available + 1) == 1;
        const int wanted = is_eof ? available : FFMAX(s->in_nb_samples, (available / s->in_nb_samples) * s->in_nb_samples);

        ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    }

    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (inlink->sample_rate != outlink->sample_rate) {
        if (ff_inlink_queued_samples(inlink) >= s->in_nb_samples) {
            ff_filter_set_ready(ctx, 10);
            return 0;
        }
    } else {
        if (ff_inlink_queued_frames(inlink) >= 1) {
            ff_filter_set_ready(ctx, 10);
            return 0;
        }
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->out_offset > 0 && s->flush_size > 0)
            ret = flush_frame(outlink);

        pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;

    if (s->src_uninit)
        s->src_uninit(ctx);
}

static AVFrame *get_in_audio_buffer(AVFilterLink *inlink, int nb_samples)
{
    AVFilterContext *ctx = inlink->dst;
    AudioRDFTSRCContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_audio_buffer   (inlink, nb_samples) :
        ff_default_get_audio_buffer(inlink, nb_samples);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .get_buffer.audio = get_in_audio_buffer,
    },
};

const FFFilter ff_af_ardftsrc = {
    .p.name          = "ardftsrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Real Discrete Fourier Transform Sample Rate Conversion."),
    .p.priv_class    = &ardftsrc_class,
    .priv_size       = sizeof(AudioRDFTSRCContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
};
