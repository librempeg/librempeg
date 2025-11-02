/*
 * Copyright (c) 2023 Paul B Mahol
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

#include "libavutil/refstruct.h"
#include "libavutil/thread.h"
#include "libavutil/threadprogress.h"

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
    int out_depth;
    int pass;
    int done_flush;
    int quality;
    int shape;
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
    float phaset;
    int64_t delay;
    int64_t last_in_pts;
    int64_t last_out_pts;
    int pad_size;
    int trim_size;
    int flush_size;
    int64_t first_pts;
    int64_t eof_in_pts;
    int64_t eof_out_pts;
    int status;

    void *over;
    AVRefStructPool *progress_pool;
    ThreadProgress  *prev_progress;
    ThreadProgress  *progress;

    void *taper;
    void *phase;

    AVFrame *in;
    AVFrame *out;

    void *state;

    int64_t (*last_in_pts_fn)(AVFilterContext *ctx);

    int (*flush_src)(AVFilterContext *ctx, AVFrame *out, const int ch);

    int (*do_src_in)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch,
                     const int soffset, const int doffset);

    int (*do_src_out)(AVFilterContext *ctx, AVFrame *out, const int ch,
                      const int offset, const int mode);

    void (*copy_over)(AVFilterContext *ctx);

    void (*src_uninit)(AVFilterContext *ctx);
} AudioRDFTSRCContext;

#define OFFSET(x) offsetof(AudioRDFTSRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ardftsrc_options[] = {
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS },
    { "quality", "set the quality", OFFSET(quality), AV_OPT_TYPE_INT, {.i64=1024}, 1, INT32_MAX, FLAGS },
    { "bandwidth", "set the bandwidth", OFFSET(bandwidth), AV_OPT_TYPE_FLOAT, {.dbl=0.95}, 0, 1, FLAGS },
    { "phase", "set the phase", OFFSET(phaset), AV_OPT_TYPE_FLOAT, {.dbl=0}, -1, 1, FLAGS },
    { "shape", "enable noise shaping", OFFSET(shape), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(ardftsrc);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioRDFTSRCContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_U8,  AV_SAMPLE_FMT_U8P,
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    int ret, sample_rates[] = { s->sample_rate, -1 };
    AVFilterFormats *formats;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(sample_fmts);
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

#define DEPTH 8
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 16
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 32
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 33
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 65
#include "ardftsrc_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    const int64_t channels = outlink->ch_layout.nb_channels;
    AudioRDFTSRCContext *s = ctx->priv;
    int max_nb_samples, ret;
    int64_t factor;

    if (inlink->sample_rate == outlink->sample_rate) {
        s->pass = 1;
        return 0;
    }

    s->first_pts = s->eof_in_pts = s->eof_out_pts = AV_NOPTS_VALUE;

    outlink->time_base = (AVRational) {1, outlink->sample_rate};

    av_reduce(&s->in_nb_samples, &s->out_nb_samples,
              inlink->sample_rate, outlink->sample_rate, INT_MAX);

    factor = lrint(2.0*ceil(s->quality/(2.0*s->out_nb_samples)));
    max_nb_samples = 2*FFMAX(s->in_nb_samples, s->out_nb_samples);
    factor = FFMIN(factor, INT32_MAX/max_nb_samples);
    s->in_nb_samples *= factor;
    s->out_nb_samples *= factor;

    s->out_planar = av_sample_fmt_is_planar(outlink->format);
    s->out_depth = av_get_bytes_per_sample(outlink->format) * 8 + (outlink->format == AV_SAMPLE_FMT_FLTP ||
                                                                   outlink->format == AV_SAMPLE_FMT_DBLP ||
                                                                   outlink->format == AV_SAMPLE_FMT_FLT  ||
                                                                   outlink->format == AV_SAMPLE_FMT_DBL);
    s->in_planar = av_sample_fmt_is_planar(inlink->format);
    s->in_rdft_size = s->in_nb_samples * 2;
    s->out_rdft_size = s->out_nb_samples * 2;
    s->flush_size = s->out_offset = s->trim_size = (s->out_rdft_size - s->out_nb_samples) >> 1;
    s->in_offset = (s->in_rdft_size - s->in_nb_samples) >> 1;
    s->delay = av_rescale_q(s->in_offset, (AVRational){ 1, inlink->sample_rate }, inlink->time_base);
    s->tr_nb_samples = FFMIN(s->in_nb_samples, s->out_nb_samples);
    s->taper_samples = lrint(s->tr_nb_samples * (1.0-s->bandwidth));
    av_log(ctx, AV_LOG_DEBUG, "factor: %"PRId64" | %d => %d | delay: %"PRId64"\n", factor, s->in_rdft_size, s->out_rdft_size, s->delay);

#if CONFIG_AVFILTER_THREAD_FRAME
    if (!ff_filter_is_frame_thread(ctx)) {
        s->over = av_refstruct_allocz(FFMAX(av_get_bytes_per_sample(inlink->format), 8) * channels * s->out_nb_samples);
        if (!s->over)
            return AVERROR(ENOMEM);
    }
#else
    s->over = av_calloc(channels * s->out_nb_samples, FFMAX(av_get_bytes_per_sample(inlink->format), 8));
    if (!s->over)
        return AVERROR(ENOMEM);
#endif

    switch (inlink->format) {
    case AV_SAMPLE_FMT_U8:
    case AV_SAMPLE_FMT_U8P:
        s->last_in_pts_fn = last_in_pts_u8p;
        s->flush_src = flush_u8p;
        s->do_src_in = src_in_u8p;
        s->do_src_out = src_out_u8p;
        s->src_uninit = src_uninit_u8p;
        s->copy_over = copy_over_u8p;
        ret = src_init_u8p(ctx);
        break;
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S16P:
        s->last_in_pts_fn = last_in_pts_s16p;
        s->flush_src = flush_s16p;
        s->do_src_in = src_in_s16p;
        s->do_src_out = src_out_s16p;
        s->src_uninit = src_uninit_s16p;
        s->copy_over = copy_over_s16p;
        ret = src_init_s16p(ctx);
        break;
    case AV_SAMPLE_FMT_S32:
    case AV_SAMPLE_FMT_S32P:
        s->last_in_pts_fn = last_in_pts_s32p;
        s->flush_src = flush_s32p;
        s->do_src_in = src_in_s32p;
        s->do_src_out = src_out_s32p;
        s->src_uninit = src_uninit_s32p;
        s->copy_over = copy_over_s32p;
        ret = src_init_s32p(ctx);
        break;
    case AV_SAMPLE_FMT_FLT:
    case AV_SAMPLE_FMT_FLTP:
        s->last_in_pts_fn = last_in_pts_fltp;
        s->flush_src = flush_fltp;
        s->do_src_in = src_in_fltp;
        s->do_src_out = src_out_fltp;
        s->src_uninit = src_uninit_fltp;
        s->copy_over = copy_over_fltp;
        ret = src_init_fltp(ctx);
        break;
    case AV_SAMPLE_FMT_DBL:
    case AV_SAMPLE_FMT_DBLP:
        s->last_in_pts_fn = last_in_pts_dblp;
        s->flush_src = flush_dblp;
        s->do_src_in = src_in_dblp;
        s->do_src_out = src_out_dblp;
        s->src_uninit = src_uninit_dblp;
        s->copy_over = copy_over_dblp;
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

static int src_in_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        for (int soffset = 0, doffset = 0; soffset < in->nb_samples;
             soffset += s->in_nb_samples, doffset += s->out_nb_samples) {
            s->do_src_in(ctx, in, out, ch, soffset, doffset);
        }
    }

    return 0;
}

static int src_out_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_src_out(ctx, out, ch, 0, 1);

    return 0;
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioRDFTSRCContext *s = ctx->priv;
    const int nb_samples = s->flush_size;
    AVFrame *out;
    int ret;

    if (nb_samples <= 0) {
        s->done_flush = 1;
        return 0;
    }

    s->flush_size = 0;

    out = av_frame_alloc();
    if (!out)
        return AVERROR(ENOMEM);

    out->nb_samples = nb_samples;
    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }

    ff_filter_execute(ctx, flush_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->sample_rate = outlink->sample_rate;
    out->pts = s->eof_out_pts;
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);
    out->pts -= out->duration;

    s->done_flush = 1;
    return ff_filter_frame(outlink, out);
}

static int filter_frame(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    int ret, trim_size, in_samples;
    AVFrame *out = s->out, *in = s->in;

    if (in == NULL) {
        if (s->flush_size > 0 && !s->done_flush &&
            s->eof_in_pts != AV_NOPTS_VALUE &&
            s->last_in_pts_fn(ctx) >= s->eof_in_pts) {
            flush_frame(outlink);
            return 0;
        }
        return (s->flush_size == 0 || s->done_flush) ? AVERROR_EOF : 0;
    }

    if (s->pass) {
        s->in = NULL;
        return ff_filter_frame(outlink, in);
    }

    if (s->pad_size > 0) {
        in_samples = in->nb_samples + s->pad_size;
    } else {
        in_samples = in->nb_samples;
    }

    out->nb_samples = av_rescale(in_samples, s->out_nb_samples, s->in_nb_samples);
    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        s->in = NULL;
        return ret;
    }

    av_frame_copy_props(out, in);

    ff_filter_execute(ctx, src_in_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

#if CONFIG_AVFILTER_THREAD_FRAME
    if (s->prev_progress)
        ff_thread_progress_await(s->prev_progress, INT_MAX);
#endif

    ff_filter_execute(ctx, src_out_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    s->copy_over(ctx);

#if CONFIG_AVFILTER_THREAD_FRAME
    if (s->progress)
        ff_thread_progress_report(s->progress, INT_MAX);
#endif

    out->sample_rate = outlink->sample_rate;
    trim_size = s->trim_size * (s->first_pts == in->pts);
    out->pts = av_rescale_q(in->pts - (trim_size ? 0 : s->delay), inlink->time_base, outlink->time_base);
    if (trim_size > 0 && trim_size < out->nb_samples) {
        if (s->out_planar) {
            for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
                out->extended_data[ch] += trim_size * av_get_bytes_per_sample(out->format);
        } else {
            out->data[0] += trim_size * out->ch_layout.nb_channels * av_get_bytes_per_sample(out->format);
        }
        out->nb_samples -= trim_size;
        s->trim_size = 0;
    } else if (trim_size > 0) {
        s->trim_size -= out->nb_samples;
        av_frame_free(&out);
        av_frame_free(&in);
        s->in = NULL;

        ff_inlink_request_frame(inlink);

        return 0;
    }

    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    s->last_out_pts = out->pts + out->duration;

    ff_graph_frame_free(ctx, &in);
    s->in = NULL;

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];

    return filter_frame(inlink);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;

    av_refstruct_unref(&s->progress_pool);
    av_refstruct_unref(&s->prev_progress);
    av_refstruct_unref(&s->progress);
#if CONFIG_AVFILTER_THREAD_FRAME
    av_refstruct_unref(&s->over);
#else
    av_freep(&s->over);
#endif
    av_frame_free(&s->in);

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

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const AudioRDFTSRCContext *s_src = src->priv;
    AudioRDFTSRCContext       *s_dst = dst->priv;

    s_dst->last_out_pts = FFMAX(s_dst->last_out_pts, s_src->last_out_pts);
    s_dst->last_in_pts = FFMAX(s_dst->last_in_pts, s_src->last_in_pts);
    s_dst->eof_out_pts = FFMAX(s_dst->eof_out_pts, s_src->eof_out_pts);
    s_dst->eof_in_pts = FFMAX(s_dst->eof_in_pts, s_src->eof_in_pts);
    s_dst->flush_size = FFMIN(s_dst->flush_size, s_src->flush_size);
    s_dst->trim_size = FFMIN(s_dst->trim_size, s_src->trim_size);
    s_dst->status = FFMAX(s_dst->status, s_src->status);

    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    s_dst->out = s_src->out;
    s_dst->first_pts = s_src->first_pts;
    s_dst->pass = s_src->pass;
    s_dst->channels = s_src->channels;
    s_dst->in_nb_samples = s_src->in_nb_samples;
    s_dst->out_nb_samples = s_src->out_nb_samples;

    av_refstruct_replace(&s_dst->over,          s_src->over);
    av_refstruct_replace(&s_dst->prev_progress, s_src->prev_progress);
    av_refstruct_replace(&s_dst->progress,      s_src->progress);

    av_frame_free(&s_dst->in);
    if (s_src->in) {
        s_dst->in = ff_graph_frame_clone(dst, s_src->in);
        if (!s_dst->in)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static int progress_init(AVRefStructOpaque opaque, void *obj)
{
    return ff_thread_progress_init(obj, 1);
}

static void progress_reset(AVRefStructOpaque opaque, void *obj)
{
    ff_thread_progress_reset(obj);
}

static void progress_free(AVRefStructOpaque opaque, void *obj)
{
    ff_thread_progress_destroy(obj);
}
#endif

static int filter_prepare(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AVFrame *in = NULL;
    int ret, status;
    int64_t pts;

    av_frame_free(&s->in);

    ret = ff_outlink_get_status(outlink);
    if (ret) {
        ff_inlink_set_status(inlink, ret);
        s->eof_out_pts = s->last_out_pts;
        ff_outlink_set_status(outlink, AVERROR_EOF, s->eof_out_pts);
        return AVERROR_EOF;
    }

    if (!ff_outlink_frame_wanted(outlink))
        return AVERROR(EAGAIN);

    if (s->pass) {
        ret = ff_inlink_consume_frame(inlink, &s->in);
        if (ret < 0)
            return ret;

        if (ret > 0)
            return 0;

        if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
            ff_outlink_set_status(outlink, status, pts);
            return AVERROR_EOF;
        }
    } else {
        int available, wanted;

        available = ff_inlink_queued_samples(inlink);
        if (s->status && available == 0) {
            if (!s->done_flush && s->flush_size > 0) {
                ff_filter_set_ready(ctx, 10);
                return 0;
            }

            ff_outlink_set_status(outlink, AVERROR_EOF, s->eof_out_pts);
            return AVERROR_EOF;
        }

        wanted = FFMAX(s->in_nb_samples, (available / s->in_nb_samples) * s->in_nb_samples);

        ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
        if (ret < 0)
            return ret;

        if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
            s->status = status;
            s->eof_in_pts = pts;
            s->eof_out_pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);

            if (ret == 0)
                ff_filter_set_ready(ctx, 10);
        }
    }

    if (ret > 0) {
        if (s->first_pts == AV_NOPTS_VALUE)
            s->first_pts = in->pts;
        s->in = in;
        s->last_in_pts = in->pts + in->duration;
        if (s->eof_in_pts == s->last_in_pts) {
            s->pad_size = FFMAX(0, FFMIN(s->in_offset, s->in_nb_samples - in->nb_samples));
            s->flush_size -= av_rescale(s->pad_size, s->out_nb_samples, s->in_nb_samples);
        }

#if CONFIG_AVFILTER_THREAD_FRAME
        if (ctx->thread_type & AVFILTER_THREAD_FRAME_FILTER) {
            if (!s->progress_pool) {
                s->progress_pool = av_refstruct_pool_alloc_ext(sizeof(ThreadProgress), 0, NULL,
                                                               progress_init, progress_reset,
                                                               progress_free, NULL);
                if (!s->progress_pool)
                    return AVERROR(ENOMEM);
            }

            av_refstruct_unref(&s->prev_progress);
            s->prev_progress = s->progress;

            s->progress = av_refstruct_pool_get(s->progress_pool);
            if (!s->progress)
                return AVERROR(ENOMEM);
        }
#endif
        s->out = ff_graph_frame_alloc(ctx);
        if (!s->out) {
            av_frame_free(&in);
            s->in = NULL;
            return AVERROR(ENOMEM);
        }

        return 0;
    } else {
        if (ff_outlink_frame_wanted(outlink))
            ff_inlink_request_frame(inlink);
    }

    return AVERROR(EAGAIN);
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
    .filter_prepare  = filter_prepare,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state  = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_FRAME_THREADS,
    .activate        = activate,
};
