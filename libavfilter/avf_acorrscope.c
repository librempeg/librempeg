/*
 * Copyright (c) 2019 Paul B Mahol
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

#include "libavutil/tx.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "audio.h"
#include "formats.h"
#include "video.h"
#include "avfilter.h"
#include "filters.h"
#include "window_func.h"

typedef struct AudioCorrScopeContext {
    const AVClass *class;
    int w, h;
    int nb_channels;
    AVRational frame_rate;
    AVTXContext **fft;          ///< Fast Fourier Transform context
    AVComplexFloat **fft_data;  ///< bins holder for each (displayed) channels
    float **fft_tdata;
    float *window_func_lut;     ///< Window function LUT
    av_tx_fn *tx_fn;
    int win_func;
    int win_size;
    int buf_size;
    int consumed;
    int hop_size;
    int frame;
    float contrast;
    float fade;
    int64_t pts;
    AVFrame *out;
} AudioCorrScopeContext;

#define OFFSET(x) offsetof(AudioCorrScopeContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define TFLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption acorrscope_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "800x600"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "800x600"}, 0, 0, FLAGS },
    { "resolution","set frequency resolution", OFFSET(frame), AV_OPT_TYPE_INT, {.i64=1}, 1, 6, FLAGS },
    WIN_FUNC_OPTION("win_func", OFFSET(win_func), FLAGS, WFUNC_HANNING),
    { "rate", "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "fade", "set the fade", OFFSET(fade), AV_OPT_TYPE_FLOAT, {.dbl=0.85}, 0, 1, TFLAGS },
    { "contrast", "set the contrast", OFFSET(contrast), AV_OPT_TYPE_FLOAT, {.dbl=1}, 0, 10, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acorrscope);

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCorrScopeContext *s = ctx->priv;

    for (int i = 0; i < s->nb_channels; i++) {
        if (s->fft)
            av_tx_uninit(&s->fft[i]);
        if (s->fft_data)
            av_freep(&s->fft_data[i]);
        if (s->fft_tdata)
            av_freep(&s->fft_tdata[i]);
    }

    av_freep(&s->fft);
    av_freep(&s->tx_fn);
    av_freep(&s->fft_data);
    av_freep(&s->fft_tdata);
    av_freep(&s->window_func_lut);
    av_frame_free(&s->out);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    AVFilterFormats *formats = NULL;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_GBRPF32, AV_PIX_FMT_NONE };
    static const AVChannelLayout layouts[] = {
        AV_CHANNEL_LAYOUT_STEREO,
        { .nb_channels = 0 },
    };
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    return ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
}

static int run_channel_fft(AVFilterContext *ctx, AVFrame *in, int ch)
{
    AudioCorrScopeContext *s = ctx->priv;
    const float *window_func_lut = s->window_func_lut;
    const float *src = (const float *)in->extended_data[ch];
    const int nb_samples = in->nb_samples;
    float *dst = s->fft_tdata[ch];

    for (int n = 0; n < nb_samples; n++)
        dst[n] = src[n] * window_func_lut[n];

    for (int n = nb_samples; n < s->buf_size; n++)
        dst[n] = 0.f;

    s->tx_fn[ch](s->fft[ch], s->fft_data[ch], dst, sizeof(*dst));

    return 0;
}

static int run_channels_fft(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFrame *in = arg;
    const int channels = in->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        run_channel_fft(ctx, in, ch);

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    FilterLink *l = ff_filter_link(outlink);
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioCorrScopeContext *s = ctx->priv;
    float overlap;
    int ret;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};

    l->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(l->frame_rate);
    s->hop_size = FFMAX(1, av_rescale(inlink->sample_rate, s->frame_rate.den, s->frame_rate.num));
    s->win_size = s->frame << av_ceil_log2(s->hop_size);
    s->buf_size = s->win_size*2 + 2;
    s->pts = AV_NOPTS_VALUE;
    s->nb_channels = inlink->ch_layout.nb_channels;

    for (int i = 0; i < s->nb_channels; i++) {
        if (s->fft)
            av_tx_uninit(&s->fft[i]);
        if (s->fft_data)
            av_freep(&s->fft_data[i]);
        if (s->fft_tdata)
            av_freep(&s->fft_tdata[i]);
    }

    av_freep(&s->fft);
    av_freep(&s->tx_fn);
    av_freep(&s->fft_data);
    av_freep(&s->fft_tdata);

    s->fft = av_calloc(s->nb_channels, sizeof(*s->fft));
    s->tx_fn = av_calloc(s->nb_channels, sizeof(*s->tx_fn));
    s->fft_data = av_calloc(s->nb_channels, sizeof(*s->fft_data));
    s->fft_tdata = av_calloc(s->nb_channels, sizeof(*s->fft_tdata));
    if (!s->fft || !s->fft_data || !s->fft_tdata || !s->tx_fn)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->nb_channels; i++) {
        float scale = 1.f;

        ret = av_tx_init(&s->fft[i], &s->tx_fn[i], AV_TX_FLOAT_RDFT,
                         0, s->win_size*2, &scale, 0);
        if (ret < 0)
            return ret;

        s->fft_tdata[i] = av_calloc(s->buf_size, sizeof(**s->fft_tdata));
        if (!s->fft_tdata[i])
            return AVERROR(ENOMEM);

        s->fft_data[i] = av_calloc(s->buf_size/2+1, sizeof(**s->fft_data));
        if (!s->fft_data[i])
            return AVERROR(ENOMEM);
    }

    /* pre-calc windowing function */
    s->window_func_lut =
        av_realloc_f(s->window_func_lut, s->hop_size,
                     sizeof(*s->window_func_lut));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);
    generate_window_func(s->window_func_lut, s->hop_size, s->win_func, &overlap);

    return 0;
}

#define RE(y, ch) s->fft_data[ch][y].re
#define IM(y, ch) s->fft_data[ch][y].im

static void draw_dot(uint8_t *dst, float step)
{
    float *x = (float *)dst;

    x[0] += step;
}

static int fade(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioCorrScopeContext *s = ctx->priv;
    const int glinesize = s->out->linesize[0];
    const int blinesize = s->out->linesize[1];
    const int rlinesize = s->out->linesize[2];
    const int width = s->out->width;
    const int height = s->out->height;
    const int slice_start = (height *  jobnr   ) / nb_jobs;
    const int slice_end   = (height * (jobnr+1)) / nb_jobs;
    const float fv = s->fade;

    if (fv <= 0.f) {
        for (int i = slice_start; i < slice_end; i++) {
            memset(s->out->data[0] + i * glinesize, 0, width*4);
            memset(s->out->data[1] + i * blinesize, 0, width*4);
            memset(s->out->data[2] + i * rlinesize, 0, width*4);
        }
        return 0;
    }

    if (fv > 0.f) {
        float *g = (float *)(s->out->data[0] + slice_start * glinesize);
        float *b = (float *)(s->out->data[1] + slice_start * blinesize);
        float *r = (float *)(s->out->data[2] + slice_start * rlinesize);

        for (int i = slice_start; i < slice_end; i++) {
            for (int j = 0; j < width; j++) {
                if (g[j] != 0.f)
                    g[j] *= fv;
                if (b[j] != 0.f)
                    b[j] *= fv;
                if (r[j] != 0.f)
                    r[j] *= fv;

                g[j] = isnormal(g[j]) ? g[j] : 0.f;
                b[j] = isnormal(b[j]) ? b[j] : 0.f;
                r[j] = isnormal(r[j]) ? r[j] : 0.f;
            }

            g += glinesize/4;
            b += blinesize/4;
            r += rlinesize/4;
        }
    }

    return 0;
}

static int draw_spatial(AVFilterLink *inlink, int64_t pts)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioCorrScopeContext *s = ctx->priv;
    const float contrast = s->contrast;
    uint8_t *dst_g, *dst_b, *dst_r;
    ptrdiff_t linesize_g;
    ptrdiff_t linesize_b;
    ptrdiff_t linesize_r;
    const int h = s->h;
    const int w = s->w;
    const int w1 = w-1;
    const int z = s->win_size + 1;
    AVFrame *clone;
    int ret;

    if (!s->out || s->out->width  != outlink->w ||
                         s->out->height != outlink->h) {
        av_frame_free(&s->out);
        s->out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!s->out)
            return AVERROR(ENOMEM);

        linesize_g = s->out->linesize[0];
        linesize_b = s->out->linesize[1];
        linesize_r = s->out->linesize[2];

        s->out->sample_aspect_ratio = (AVRational){1,1};
        for (int i = 0; i < outlink->h; i++) {
            memset(s->out->data[0] + i * linesize_g,   0, w);
            memset(s->out->data[1] + i * linesize_b, 128, w);
            memset(s->out->data[2] + i * linesize_r, 128, w);
        }
    }

    ret = ff_inlink_make_frame_writable(outlink, &s->out);
    if (ret < 0)
        return ret;

    dst_g = s->out->data[0];
    dst_b = s->out->data[1];
    dst_r = s->out->data[2];

    linesize_g = s->out->linesize[0];
    linesize_b = s->out->linesize[1];
    linesize_r = s->out->linesize[2];

    ff_filter_execute(ctx, fade, NULL, NULL, FFMIN(outlink->h, ff_filter_get_nb_threads(ctx)));

    for (int j = 0; j < z; j++) {
        const int idx = z - 1 - j;
        const float lre = RE(idx, 0);
        const float lim = IM(idx, 0);
        const float rre = RE(idx, 1);
        const float rim = IM(idx, 1);
        const float re = rre * lre + rim * lim;
        const float im = lre * rim - lim * rre;
        const float corr = atan2f(im, re);
        const float X = ((corr / M_PIf) + 1.f) * 0.5f;
        const float Y = av_clipf((hypotf(lre, lim) + hypotf(rre, rim)) * contrast, 0.f, 1.f);
        const float e = j * 2.f*M_PIf / z;
        int x = w1 - X * w1, y = j * h / (float)z;
        float co = Y * cosf(e) * 0.5f;
        float cg = Y * sinf(e) * 0.5f;
        float g, b, r, t;

        t = Y - cg;
        r = t + co;
        g = Y + cg;
        b = t - co;

        draw_dot(dst_g + linesize_g * y + x*4, g);
        draw_dot(dst_b + linesize_b * y + x*4, b);
        draw_dot(dst_r + linesize_r * y + x*4, r);
    }

    s->out->pts = pts;
    s->out->duration = 1;

    clone = av_frame_clone(s->out);
    if (!clone)
        return AVERROR(ENOMEM);

    return ff_filter_frame(outlink, clone);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioCorrScopeContext *s = ctx->priv;
    AVFrame *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->hop_size, s->hop_size, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        int64_t pts = av_rescale_q(in->pts, inlink->time_base, outlink->time_base);

        ff_filter_execute(ctx, run_channels_fft, in, NULL, s->nb_channels);
        av_frame_free(&in);

        if (s->pts != pts) {
            s->pts = pts;
            return draw_spatial(inlink, pts);
        }
    }

    if (ff_inlink_queued_samples(inlink) >= s->hop_size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const FFFilter ff_avf_acorrscope = {
    .p.name        = "acorrscope",
    .p.description = NULL_IF_CONFIG_SMALL("Convert input audio to a correlation scope video output."),
    .p.priv_class  = &acorrscope_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .uninit        = uninit,
    .priv_size     = sizeof(AudioCorrScopeContext),
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .activate      = activate,
    .process_command = ff_filter_process_command,
};
