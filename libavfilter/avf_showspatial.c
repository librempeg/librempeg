/*
 * Copyright (c) 2019 Paul B Mahol
 *
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

#include "libavutil/tx.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "audio.h"
#include "formats.h"
#include "video.h"
#include "avfilter.h"
#include "filters.h"
#include "internal.h"
#include "window_func.h"

typedef struct ShowSpatialContext {
    const AVClass *class;
    int w, h;
    AVRational frame_rate;
    AVTXContext *fft[2];          ///< Fast Fourier Transform context
    AVComplexFloat *fft_data[2];  ///< bins holder for each (displayed) channels
    float *window[2];
    float *fft_tdata[2];
    float *window_func_lut;       ///< Window function LUT
    av_tx_fn tx_fn[2];
    int win_func;
    int win_size;
    int buf_size;
    int consumed;
    int hop_size;
    int frame;
    int64_t pts;
} ShowSpatialContext;

#define OFFSET(x) offsetof(ShowSpatialContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption showspatial_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "512x512"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "512x512"}, 0, 0, FLAGS },
    { "resolution","set frequency resolution", OFFSET(frame), AV_OPT_TYPE_INT, {.i64=1}, 1, 6, FLAGS },
    WIN_FUNC_OPTION("win_func", OFFSET(win_func), FLAGS, WFUNC_HANNING),
    { "rate", "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(showspatial);

static av_cold void uninit(AVFilterContext *ctx)
{
    ShowSpatialContext *s = ctx->priv;

    for (int i = 0; i < 2; i++)
        av_tx_uninit(&s->fft[i]);
    for (int i = 0; i < 2; i++) {
        av_freep(&s->fft_data[i]);
        av_freep(&s->fft_tdata[i]);
        av_freep(&s->window[i]);
    }
    av_freep(&s->window_func_lut);
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layout = NULL;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_YUV444P, AV_PIX_FMT_NONE };
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref         (formats, &inlink->outcfg.formats        )) < 0 ||
        (ret = ff_add_channel_layout  (&layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO)) < 0 ||
        (ret = ff_channel_layouts_ref (layout , &inlink->outcfg.channel_layouts)) < 0)
        return ret;

    formats = ff_all_samplerates();
    if ((ret = ff_formats_ref(formats, &inlink->outcfg.samplerates)) < 0)
        return ret;

    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &outlink->incfg.formats)) < 0)
        return ret;

    return 0;
}

static int run_channel_fft(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ShowSpatialContext *s = ctx->priv;
    const float *window_func_lut = s->window_func_lut;
    const int ch = jobnr;
    const float *p = s->window[ch];
    const int nb_samples = s->win_size;
    float *dst = s->fft_tdata[ch];

    for (int n = 0; n < nb_samples; n++)
        dst[n] = p[n] * window_func_lut[n];

    s->tx_fn[ch](s->fft[ch], s->fft_data[ch], dst, sizeof(*dst));

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowSpatialContext *s = ctx->priv;
    float overlap;
    int ret;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};

    outlink->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(outlink->frame_rate);
    s->hop_size = FFMAX(1, av_rescale(inlink->sample_rate, s->frame_rate.den, s->frame_rate.num));
    s->win_size = s->frame << av_ceil_log2(s->hop_size);
    s->buf_size = s->win_size + 2;

    for (int i = 0; i < 2; i++) {
        av_tx_uninit(&s->fft[i]);
        av_freep(&s->fft_data[i]);
        av_freep(&s->fft_tdata[i]);
        av_freep(&s->window[i]);
    }

    for (int i = 0; i < 2; i++) {
        float scale = 1.f / s->win_size;
        ret = av_tx_init(&s->fft[i], &s->tx_fn[i], AV_TX_FLOAT_RDFT,
                         0, s->win_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < 2; i++) {
        s->window[i] = av_calloc(s->win_size, sizeof(**s->window));
        if (!s->window[i])
            return AVERROR(ENOMEM);

        s->fft_tdata[i] = av_calloc(s->buf_size, sizeof(**s->fft_tdata));
        if (!s->fft_tdata[i])
            return AVERROR(ENOMEM);

        s->fft_data[i] = av_calloc(s->buf_size/2+1, sizeof(**s->fft_data));
        if (!s->fft_data[i])
            return AVERROR(ENOMEM);
    }

    /* pre-calc windowing function */
    s->window_func_lut =
        av_realloc_f(s->window_func_lut, s->win_size,
                     sizeof(*s->window_func_lut));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);
    generate_window_func(s->window_func_lut, s->win_size, s->win_func, &overlap);

    return 0;
}

#define RE(y, ch) s->fft_data[ch][y].re
#define IM(y, ch) s->fft_data[ch][y].im

static void draw_dot(uint8_t *dst, ptrdiff_t linesize, int value)
{
    dst[0] = value;
    dst[1] = value;
    dst[-1] = value;
    dst[linesize] = value;
    dst[-linesize] = value;
}

static int draw_spatial(AVFilterLink *inlink, int64_t in_pts)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ShowSpatialContext *s = ctx->priv;
    uint8_t *dst_y, *dst_u, *dst_v;
    ptrdiff_t linesize_y;
    ptrdiff_t linesize_u;
    ptrdiff_t linesize_v;
    AVFrame *outpicref;
    int h = s->h - 2;
    int w = s->w - 2;
    int z = s->win_size / 2 + 1;
    int64_t pts = av_rescale_q(in_pts, inlink->time_base, outlink->time_base);

    outpicref = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!outpicref)
        return AVERROR(ENOMEM);

    dst_y = outpicref->data[0];
    dst_u = outpicref->data[1];
    dst_v = outpicref->data[2];

    linesize_y = outpicref->linesize[0];
    linesize_u = outpicref->linesize[1];
    linesize_v = outpicref->linesize[2];

    outpicref->sample_aspect_ratio = (AVRational){1,1};
    for (int i = 0; i < outlink->h; i++) {
        memset(dst_y + i * linesize_y,   0, outlink->w);
        memset(dst_u + i * linesize_u, 128, outlink->w);
        memset(dst_v + i * linesize_v, 128, outlink->w);
    }

    for (int j = 0; j < z; j++) {
        const int idx = z - 1 - j;
        float l_re = RE(idx, 0);
        float l_im = IM(idx, 0);
        float r_re = RE(idx, 1);
        float r_im = IM(idx, 1);
        float re = l_re * r_re + l_im * r_im;
        float im = r_re * l_im - r_im * l_re;
        float l = hypotf(l_re, l_im);
        float r = hypotf(r_re, r_im);
        float sum = atan2f(r, l);
        float diffp = fabsf(atan2f(im, re) / M_PIf);
        float diff = sum / M_PI_2f;
        int cu = av_clip((1.f-diff) * 255.f, 0, 255);
        int cv = av_clip(     diff  * 255.f, 0, 255);
        int cy = av_clip(     diff  * 255.f, 0, 255);
        int x, y;

        x = av_clip(w * diff,  0, w - 2) + 1;
        y = av_clip(h * diffp, 0, h - 2) + 1;

        draw_dot(dst_y + linesize_y * y + x, linesize_y, cy);
        draw_dot(dst_u + linesize_u * y + x, linesize_u, cu);
        draw_dot(dst_v + linesize_v * y + x, linesize_v, cv);
    }

    outpicref->pts = pts;
    outpicref->duration = 1;

    return ff_filter_frame(outlink, outpicref);
}

static int spatial_activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    ShowSpatialContext *s = ctx->priv;
    AVFrame *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->hop_size, s->hop_size, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        const int64_t pts = in->pts;

        for (int ch = 0; ch < 2; ch++) {
            const int offset = s->win_size - s->hop_size;
            float *src = s->window[ch];

            memmove(src, &src[s->hop_size], offset * sizeof(*src));
            memcpy(&src[offset], in->extended_data[ch], in->nb_samples * sizeof(*src));
        }

        av_frame_free(&in);
        ff_filter_execute(ctx, run_channel_fft, in, NULL, 2);

        return draw_spatial(inlink, pts);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static const AVFilterPad showspatial_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const AVFilter ff_avf_showspatial = {
    .name          = "showspatial",
    .description   = NULL_IF_CONFIG_SMALL("Convert input audio to a spatial video output."),
    .uninit        = uninit,
    .priv_size     = sizeof(ShowSpatialContext),
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(showspatial_outputs),
    FILTER_QUERY_FUNC(query_formats),
    .activate      = spatial_activate,
    .priv_class    = &showspatial_class,
    .flags         = AVFILTER_FLAG_SLICE_THREADS,
};
