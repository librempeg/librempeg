/*
 * Copyright (c) 2012 Stefano Sabatini
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

/**
 * @file
 * audio to video multimedia filter
 */

#include "config_components.h"

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"
#include "video.h"
#include "internal.h"

enum ShowWavesMode {
    MODE_POINT,
    MODE_LINE,
    MODE_P2P,
    MODE_CENTERED_LINE,
    MODE_NB,
};

enum ShowWavesScale {
    SCALE_LIN,
    SCALE_LOG,
    SCALE_SQRT,
    SCALE_CBRT,
    SCALE_NB,
};

enum ShowWavesDrawMode {
    DRAW_SCALE,
    DRAW_FULL,
    DRAW_NB,
};

enum ShowWavesFilterMode {
    FILTER_AVERAGE,
    FILTER_PEAK,
    FILTER_NB,
};

struct frame_node {
    AVFrame *frame;
    struct frame_node *next;
};

typedef struct ShowWavesContext {
    const AVClass *class;
    int w, h;
    AVRational rate;
    uint32_t *colors;
    unsigned nb_colors;
    int16_t *buf_idy;    /* y coordinate of previous sample for each channel */
    int16_t *history;
    int history_filled;
    int history_nb_samples;
    AVFrame *outpicref;
    AVRational n, q, c;
    int step_size;
    int pixstep;
    int mode;                   ///< ShowWavesMode
    int scale;                  ///< ShowWavesScale
    int draw_mode;              ///< ShowWavesDrawMode
    int split_channels;
    int filter_mode;
    uint8_t *fg;
    int64_t in_pts;

    int (*get_h)(int16_t sample, int height);
    void (*draw_sample)(uint8_t *buf, int height, int linesize,
                        int16_t *prev_y, const uint8_t color[4], int h);

    /* single picture */
    int single_pic;
    struct frame_node *audio_frames;
    struct frame_node *last_frame;
    int64_t total_samples;
    int64_t *sum; /* abs sum of the samples per channel */
} ShowWavesContext;

#define OFFSET(x) offsetof(ShowWavesContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_colors = {.def="red|green|blue|yellow|orange|lime|pink|magenta|brown",.size_min=1,.sep='|'};

static const AVOption showwaves_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "600x240"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "600x240"}, 0, 0, FLAGS },
    { "mode", "select display mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=MODE_POINT}, 0, MODE_NB-1,        .flags=FLAGS, .unit="mode"},
        { "point", "draw a point for each sample",         0, AV_OPT_TYPE_CONST, {.i64=MODE_POINT},         .flags=FLAGS, .unit="mode"},
        { "line",  "draw a line for each sample",          0, AV_OPT_TYPE_CONST, {.i64=MODE_LINE},          .flags=FLAGS, .unit="mode"},
        { "p2p",   "draw a line between samples",          0, AV_OPT_TYPE_CONST, {.i64=MODE_P2P},           .flags=FLAGS, .unit="mode"},
        { "cline", "draw a centered line for each sample", 0, AV_OPT_TYPE_CONST, {.i64=MODE_CENTERED_LINE}, .flags=FLAGS, .unit="mode"},
    { "n",    "set how many samples to show in the same point", OFFSET(n), AV_OPT_TYPE_RATIONAL, {.i64 = 0}, 0, INT_MAX, FLAGS },
    { "rate", "set video rate", OFFSET(rate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(rate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, FLAGS },
    { "split_channels", "draw channels separately", OFFSET(split_channels), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, FLAGS },
    { "colors", "set channels colors", OFFSET(colors), AV_OPT_TYPE_COLOR|AR, {.arr=&def_colors}, 0, 0, FLAGS },
    { "scale", "set amplitude scale", OFFSET(scale), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, SCALE_NB-1, FLAGS, .unit="scale" },
        { "lin", "linear",         0, AV_OPT_TYPE_CONST, {.i64=SCALE_LIN}, .flags=FLAGS, .unit="scale"},
        { "log", "logarithmic",    0, AV_OPT_TYPE_CONST, {.i64=SCALE_LOG}, .flags=FLAGS, .unit="scale"},
        { "sqrt", "square root",   0, AV_OPT_TYPE_CONST, {.i64=SCALE_SQRT}, .flags=FLAGS, .unit="scale"},
        { "cbrt", "cubic root",    0, AV_OPT_TYPE_CONST, {.i64=SCALE_CBRT}, .flags=FLAGS, .unit="scale"},
    { "draw", "set draw mode", OFFSET(draw_mode), AV_OPT_TYPE_INT, {.i64 = DRAW_SCALE}, 0, DRAW_NB-1, FLAGS, .unit="draw" },
        { "scale", "scale pixel values for each drawn sample", 0, AV_OPT_TYPE_CONST, {.i64=DRAW_SCALE}, .flags=FLAGS, .unit="draw"},
        { "full",  "draw every pixel for sample directly",     0, AV_OPT_TYPE_CONST, {.i64=DRAW_FULL},  .flags=FLAGS, .unit="draw"},
    { NULL }
};

AVFILTER_DEFINE_CLASS(showwaves);

static av_cold void uninit(AVFilterContext *ctx)
{
    ShowWavesContext *s = ctx->priv;

    av_frame_free(&s->outpicref);
    av_freep(&s->buf_idy);
    av_freep(&s->history);
    av_freep(&s->fg);

    if (s->single_pic) {
        struct frame_node *node = s->audio_frames;
        while (node) {
            struct frame_node *tmp = node;

            node = node->next;
            av_frame_free(&tmp->frame);
            av_freep(&tmp);
        }
        av_freep(&s->sum);
        s->last_frame = NULL;
    }
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts = NULL;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_RGBA, AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE };
    int ret;

    /* set input audio formats */
    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &inlink->outcfg.formats)) < 0)
        return ret;

    layouts = ff_all_channel_layouts();
    if ((ret = ff_channel_layouts_ref(layouts, &inlink->outcfg.channel_layouts)) < 0)
        return ret;

    formats = ff_all_samplerates();
    if ((ret = ff_formats_ref(formats, &inlink->outcfg.samplerates)) < 0)
        return ret;

    /* set output video format */
    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &outlink->incfg.formats)) < 0)
        return ret;

    return 0;
}

static int get_lin_h(int16_t sample, int height)
{
    return height/2 - av_rescale(sample, height/2, INT16_MAX);
}

static int get_lin_h2(int16_t sample, int height)
{
    return av_rescale(FFABS(sample), height, INT16_MAX);
}

static int get_log_h(int16_t sample, int height)
{
    return height/2 - FFSIGN(sample) * (log10(1 + FFABS(sample)) * (height/2) / log10(1 + INT16_MAX));
}

static int get_log_h2(int16_t sample, int height)
{
    return log10(1 + FFABS(sample)) * height / log10(1 + INT16_MAX);
}

static int get_sqrt_h(int16_t sample, int height)
{
    return height/2 - FFSIGN(sample) * (sqrt(FFABS(sample)) * (height/2) / sqrt(INT16_MAX));
}

static int get_sqrt_h2(int16_t sample, int height)
{
    return sqrt(FFABS(sample)) * height / sqrt(INT16_MAX);
}

static int get_cbrt_h(int16_t sample, int height)
{
    return height/2 - FFSIGN(sample) * (cbrt(FFABS(sample)) * (height/2) / cbrt(INT16_MAX));
}

static int get_cbrt_h2(int16_t sample, int height)
{
    return cbrt(FFABS(sample)) * height / cbrt(INT16_MAX);
}

static void draw_sample_point_rgba_scale(uint8_t *buf, int height, int linesize,
                                         int16_t *prev_y,
                                         const uint8_t color[4], int h)
{
    if (h >= 0 && h < height) {
        buf[h * linesize + 0] += color[0];
        buf[h * linesize + 1] += color[1];
        buf[h * linesize + 2] += color[2];
        buf[h * linesize + 3] += color[3];
    }
}

static void draw_sample_point_rgba_full(uint8_t *buf, int height, int linesize,
                                   int16_t *prev_y,
                                   const uint8_t color[4], int h)
{
    uint32_t clr = AV_RN32(color);
    if (h >= 0 && h < height)
        AV_WN32(buf + h * linesize, clr);
}

static void draw_sample_line_rgba_scale(uint8_t *buf, int height, int linesize,
                                        int16_t *prev_y,
                                        const uint8_t color[4], int h)
{
    int start   = height/2;
    int end     = av_clip(h, 0, height-1);
    uint8_t *bufk;
    if (start > end)
        FFSWAP(int16_t, start, end);
    bufk = buf + start * linesize;
    for (int k = start; k < end; k++, bufk += linesize) {
        bufk[0] += color[0];
        bufk[1] += color[1];
        bufk[2] += color[2];
        bufk[3] += color[3];
    }
}

static void draw_sample_line_rgba_full(uint8_t *buf, int height, int linesize,
                                       int16_t *prev_y,
                                       const uint8_t color[4], int h)
{
    int start   = height/2;
    int end     = av_clip(h, 0, height-1);
    uint32_t clr = AV_RN32(color);
    uint8_t *bufk;
    if (start > end)
        FFSWAP(int16_t, start, end);
    bufk = buf + start * linesize;
    for (int k = start; k < end; k++, bufk += linesize)
        AV_WN32(bufk, clr);
}

static void draw_sample_p2p_rgba_scale(uint8_t *buf, int height, int linesize,
                                       int16_t *prev_y,
                                       const uint8_t color[4], int h)
{
    if (*prev_y < 0)
        *prev_y = h;
    if (h >= 0 && h < height) {
        buf[h * linesize + 0] += color[0];
        buf[h * linesize + 1] += color[1];
        buf[h * linesize + 2] += color[2];
        buf[h * linesize + 3] += color[3];
        if (h != *prev_y) {
            int start = *prev_y;
            uint8_t *bufk;
            int end = av_clip(h, 0, height-1);
            *prev_y = end;
            if (start > end)
                FFSWAP(int16_t, start, end);
            bufk = buf + (start + 1) * linesize;
            for (int k = start + 1; k < end; k++, bufk += linesize) {
                bufk[0] += color[0];
                bufk[1] += color[1];
                bufk[2] += color[2];
                bufk[3] += color[3];
            }
        }
    }
}

static void draw_sample_p2p_rgba_full(uint8_t *buf, int height, int linesize,
                                      int16_t *prev_y,
                                      const uint8_t color[4], int h)
{
    uint32_t clr = AV_RN32(color);
    if (*prev_y < 0)
        *prev_y = h;
    if (h >= 0 && h < height) {
        AV_WN32(buf + h * linesize, clr);
        if (h != *prev_y) {
            int start = *prev_y;
            uint8_t *bufk;
            int end = av_clip(h, 0, height-1);
            *prev_y = end;
            if (start > end)
                FFSWAP(int16_t, start, end);
            bufk = buf + (start + 1) * linesize;
            for (int k = start + 1; k < end; k++, bufk += linesize)
                AV_WN32(bufk, clr);
        }
    }
}

static void draw_sample_cline_rgba_scale(uint8_t *buf, int height, int linesize,
                                         int16_t *prev_y,
                                         const uint8_t color[4], int h)
{
    const int start = (height - h) / 2;
    const int end   = start + h;
    uint8_t *bufk = buf + start * linesize;
    for (int k = start; k < end; k++, bufk += linesize) {
        bufk[0] += color[0];
        bufk[1] += color[1];
        bufk[2] += color[2];
        bufk[3] += color[3];
    }
}

static void draw_sample_cline_rgba_full(uint8_t *buf, int height, int linesize,
                                         int16_t *prev_y,
                                         const uint8_t color[4], int h)
{
    uint32_t clr = AV_RN32(color);
    const int start = (height - h) / 2;
    const int end   = start + h;
    uint8_t *bufk = buf + start * linesize;
    for (int k = start; k < end; k++, bufk += linesize)
        AV_WN32(bufk, clr);
}

static void draw_sample_point_gray(uint8_t *buf, int height, int linesize,
                                   int16_t *prev_y,
                                   const uint8_t color[4], int h)
{
    if (h >= 0 && h < height)
        buf[h * linesize] += color[0];
}

static void draw_sample_line_gray(uint8_t *buf, int height, int linesize,
                                  int16_t *prev_y,
                                  const uint8_t color[4], int h)
{
    int k;
    int start   = height/2;
    int end     = av_clip(h, 0, height-1);
    if (start > end)
        FFSWAP(int16_t, start, end);
    for (k = start; k < end; k++)
        buf[k * linesize] += color[0];
}

static void draw_sample_p2p_gray(uint8_t *buf, int height, int linesize,
                                 int16_t *prev_y,
                                 const uint8_t color[4], int h)
{
    int k;
    if (*prev_y < 0)
        *prev_y = h;
    if (h >= 0 && h < height) {
        buf[h * linesize] += color[0];
        if (h != *prev_y) {
            int start = *prev_y;
            int end = av_clip(h, 0, height-1);
            *prev_y = end;
            if (start > end)
                FFSWAP(int16_t, start, end);
            for (k = start + 1; k < end; k++)
                buf[k * linesize] += color[0];
        }
    }
}

static void draw_sample_cline_gray(uint8_t *buf, int height, int linesize,
                                   int16_t *prev_y,
                                   const uint8_t color[4], int h)
{
    int k;
    const int start = (height - h) / 2;
    const int end   = start + h;
    for (k = start; k < end; k++)
        buf[k * linesize] += color[0];
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowWavesContext *s = ctx->priv;
    int nb_channels = inlink->ch_layout.nb_channels;
    uint8_t x;
    int ch;

    if (s->single_pic) {
        outlink->frame_rate = av_make_q(1, 1);
    } else {
        outlink->frame_rate = s->rate;
    }

    if (!FF_ALLOCZ_TYPED_ARRAY(s->buf_idy, nb_channels))
        return AVERROR(ENOMEM);

    for (int i = 0; i < nb_channels; i++)
        s->buf_idy[i] = -1;

    s->c = av_make_q(0, 1);

    s->step_size = FFMAX(1, av_rescale(inlink->sample_rate, s->rate.den, s->rate.num));
    if (s->n.num <= 0 || s->n.den <= 0 || s->n.num < s->n.den)
        av_reduce(&s->n.num, &s->n.den, s->step_size, s->w, 256);
    if (s->n.num <= 0 || s->n.den <= 0 || s->n.num < s->n.den)
        s->n.num = s->n.den = 1;

    s->history_filled = 0;
    s->history_nb_samples = nb_channels * av_rescale_rnd(s->step_size, s->n.num,
                                                         s->n.den, AV_ROUND_UP);
    s->history = av_calloc(2 * s->history_nb_samples, sizeof(*s->history));
    if (!s->history)
        return AVERROR(ENOMEM);

    outlink->sample_aspect_ratio = (AVRational){1,1};
    outlink->time_base = av_inv_q(outlink->frame_rate);
    outlink->w = s->w;
    outlink->h = s->h;

    av_log(ctx, AV_LOG_VERBOSE, "s:%dx%d r:%f n:%f h:%d\n",
           s->w, s->h, av_q2d(outlink->frame_rate), av_q2d(s->n), s->step_size);

    switch (outlink->format) {
    case AV_PIX_FMT_GRAY8:
        switch (s->mode) {
        case MODE_POINT:         s->draw_sample = draw_sample_point_gray; break;
        case MODE_LINE:          s->draw_sample = draw_sample_line_gray;  break;
        case MODE_P2P:           s->draw_sample = draw_sample_p2p_gray;   break;
        case MODE_CENTERED_LINE: s->draw_sample = draw_sample_cline_gray; break;
        default:
            return AVERROR_BUG;
        }
        s->pixstep = 1;
        break;
    case AV_PIX_FMT_RGBA:
        switch (s->mode) {
        case MODE_POINT:         s->draw_sample = s->draw_mode == DRAW_SCALE ? draw_sample_point_rgba_scale : draw_sample_point_rgba_full; break;
        case MODE_LINE:          s->draw_sample = s->draw_mode == DRAW_SCALE ? draw_sample_line_rgba_scale  : draw_sample_line_rgba_full;  break;
        case MODE_P2P:           s->draw_sample = s->draw_mode == DRAW_SCALE ? draw_sample_p2p_rgba_scale   : draw_sample_p2p_rgba_full;   break;
        case MODE_CENTERED_LINE: s->draw_sample = s->draw_mode == DRAW_SCALE ? draw_sample_cline_rgba_scale : draw_sample_cline_rgba_full; break;
        default:
            return AVERROR_BUG;
        }
        s->pixstep = 4;
        break;
    }

    switch (s->scale) {
    case SCALE_LIN:
        switch (s->mode) {
        case MODE_POINT:
        case MODE_LINE:
        case MODE_P2P:           s->get_h = get_lin_h;  break;
        case MODE_CENTERED_LINE: s->get_h = get_lin_h2; break;
        default:
            return AVERROR_BUG;
        }
        break;
    case SCALE_LOG:
        switch (s->mode) {
        case MODE_POINT:
        case MODE_LINE:
        case MODE_P2P:           s->get_h = get_log_h;  break;
        case MODE_CENTERED_LINE: s->get_h = get_log_h2; break;
        default:
            return AVERROR_BUG;
        }
        break;
    case SCALE_SQRT:
        switch (s->mode) {
        case MODE_POINT:
        case MODE_LINE:
        case MODE_P2P:           s->get_h = get_sqrt_h;  break;
        case MODE_CENTERED_LINE: s->get_h = get_sqrt_h2; break;
        default:
            return AVERROR_BUG;
        }
        break;
    case SCALE_CBRT:
        switch (s->mode) {
        case MODE_POINT:
        case MODE_LINE:
        case MODE_P2P:           s->get_h = get_cbrt_h;  break;
        case MODE_CENTERED_LINE: s->get_h = get_cbrt_h2; break;
        default:
            return AVERROR_BUG;
        }
        break;
    }

    s->fg = av_malloc_array(nb_channels, 4 * sizeof(*s->fg));
    if (!s->fg)
        return AVERROR(ENOMEM);

    if (s->draw_mode == DRAW_SCALE) {
        /* multiplication factor, pre-computed to avoid in-loop divisions */
        x = 255.f / ((s->split_channels ? 1 : nb_channels) * av_q2d(s->n));
    } else {
        x = 255;
    }
    if (outlink->format == AV_PIX_FMT_RGBA) {
        uint8_t fg[4] = { 0xff, 0xff, 0xff, 0xff };

        for (ch = 0; ch < nb_channels; ch++) {
            const int idx = FFMIN(ch, s->nb_colors-1);

            AV_WN32(fg, s->colors[idx]);

            s->fg[4*ch + 0] = fg[0] * x / 255.;
            s->fg[4*ch + 1] = fg[1] * x / 255.;
            s->fg[4*ch + 2] = fg[2] * x / 255.;
            s->fg[4*ch + 3] = fg[3] * x / 255.;
        }
    } else {
        for (ch = 0; ch < nb_channels; ch++)
            s->fg[4 * ch + 0] = x;
    }

    return 0;
}

inline static int push_frame(AVFilterLink *outlink, AVFrame *out, int64_t pts)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowWavesContext *s = ctx->priv;
    int ret;

    out->duration = 1;
    out->pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);

    ret = ff_filter_frame(outlink, out);
    for (int i = 0; i < inlink->ch_layout.nb_channels; i++)
        s->buf_idy[i] = -1;
    return ret;
}

static int push_single_pic(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    ShowWavesContext *s = ctx->priv;
    int64_t n = 0, column_max_samples = s->total_samples / outlink->w;
    int64_t remaining_samples = s->total_samples - (column_max_samples * outlink->w);
    int64_t last_column_samples = column_max_samples + remaining_samples;
    AVFrame *out = s->outpicref;
    struct frame_node *node;
    const int nb_channels = inlink->ch_layout.nb_channels;
    const int ch_height = s->split_channels ? outlink->h / nb_channels : outlink->h;
    const int linesize = out->linesize[0];
    const int pixstep = s->pixstep;
    int col = 0;
    int64_t *sum = s->sum;

    if (column_max_samples == 0) {
        av_log(ctx, AV_LOG_ERROR, "Too few samples\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_DEBUG, "Create frame averaging %"PRId64" samples per column\n", column_max_samples);

    memset(sum, 0, nb_channels * sizeof(*sum));

    for (node = s->audio_frames; node; node = node->next) {
        int i;
        const AVFrame *frame = node->frame;
        const int16_t *p = (const int16_t *)frame->data[0];

        for (i = 0; i < frame->nb_samples; i++) {
            int64_t max_samples = col == outlink->w - 1 ? last_column_samples: column_max_samples;
            int ch;

            switch (s->filter_mode) {
            case FILTER_AVERAGE:
                for (ch = 0; ch < nb_channels; ch++)
                    sum[ch] += abs(p[ch + i*nb_channels]);
                break;
            case FILTER_PEAK:
                for (ch = 0; ch < nb_channels; ch++)
                    sum[ch] = FFMAX(sum[ch], abs(p[ch + i*nb_channels]));
                break;
            }

            n++;
            if (n == max_samples) {
                for (ch = 0; ch < nb_channels; ch++) {
                    int16_t sample = sum[ch] / (s->filter_mode == FILTER_AVERAGE ? max_samples : 1);
                    uint8_t *buf = out->data[0] + col * pixstep;
                    int h;

                    if (s->split_channels)
                        buf += ch*ch_height*linesize;
                    av_assert0(col < outlink->w);
                    h = s->get_h(sample, ch_height);
                    s->draw_sample(buf, ch_height, linesize, &s->buf_idy[ch], &s->fg[ch * 4], h);
                    sum[ch] = 0;
                }
                col++;
                n = 0;
            }
        }
    }

    s->outpicref = NULL;
    return push_frame(outlink, out, 0);
}


static int request_frame(AVFilterLink *outlink)
{
    ShowWavesContext *s = outlink->src->priv;
    AVFilterLink *inlink = outlink->src->inputs[0];
    int ret;

    ret = ff_request_frame(inlink);
    if (ret == AVERROR_EOF && s->outpicref) {
        push_single_pic(outlink);
    }

    return ret;
}

static AVFrame *alloc_out_frame(ShowWavesContext *s,
                                AVFilterLink *outlink)
{
    AVFrame *out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out)
        return NULL;
    for (int y = 0; y < outlink->h; y++)
        memset(out->data[0] + y*out->linesize[0], 0, outlink->w * s->pixstep);
    return out;
}

static av_cold int init(AVFilterContext *ctx)
{
    ShowWavesContext *s = ctx->priv;

    if (!strcmp(ctx->filter->name, "showwavespic")) {
        s->single_pic = 1;
        s->mode = MODE_CENTERED_LINE;
    }

    return 0;
}

#if CONFIG_SHOWWAVES_FILTER

static int showwaves_filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ShowWavesContext *s = ctx->priv;
    int16_t *history = s->history;
    const int nb_channels = inlink->ch_layout.nb_channels;
    const int pixstep = s->pixstep;
    const int ch_height = s->split_channels ? outlink->h / nb_channels : outlink->h;
    const int split_channels = s->split_channels;
    const AVRational in_q = av_inv_q(s->n);
    const AVRational u_q = av_make_q(1, 1);
    int16_t *buf_idy = s->buf_idy;
    const uint8_t *fg = s->fg;
    const int w = s->w;
    ptrdiff_t linesize;
    uint8_t *dst;
    AVFrame *out;

    if (in) {
        const int nb_samples = in->nb_samples;
        const int16_t *p = (const int16_t *)in->data[0];

        memcpy(history + s->history_filled, p, nb_samples * nb_channels * sizeof(*history));
        s->history_filled += nb_samples * nb_channels;
        s->in_pts = in->pts;
        av_frame_free(&in);
    }

    if (s->history_filled < s->history_nb_samples) {
        FF_FILTER_FORWARD_STATUS(inlink, outlink);
        ff_inlink_request_frame(inlink);
        return 0;
    }

    out = alloc_out_frame(s, outlink);
    if (out == NULL)
        return AVERROR(ENOMEM);

    /* draw data in the buffer */
    dst = out->data[0];
    linesize = out->linesize[0];
    for (int i = 0, k = 0;;) {
        s->c = av_add_q(s->c, in_q);
        if (av_cmp_q(s->c, u_q) >= 0) {
            s->c = av_sub_q(s->c, u_q);
            i++;
            if (i >= w)
                break;
        }

        for (int j = 0; j < nb_channels; j++, k++) {
            uint8_t *buf = dst + i * pixstep;
            int h;

            if (split_channels)
                buf += j*ch_height*linesize;
            h = s->get_h(history[k], ch_height);
            s->draw_sample(buf, ch_height, linesize,
                           &buf_idy[j], &fg[j * 4], h);
        }
    }

    s->history_filled -= s->step_size * nb_channels;
    memmove(history, history + s->step_size * nb_channels, s->history_filled * sizeof(*history));

    return push_frame(outlink, out, s->in_pts);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    ShowWavesContext *s = ctx->priv;
    AVFrame *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->step_size, s->step_size, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return showwaves_filter_frame(inlink, in);

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static const AVFilterPad showwaves_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
};

const AVFilter ff_avf_showwaves = {
    .name          = "showwaves",
    .description   = NULL_IF_CONFIG_SMALL("Convert input audio to a video output."),
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(ShowWavesContext),
    FILTER_INPUTS(ff_audio_default_filterpad),
    .activate      = activate,
    FILTER_OUTPUTS(showwaves_outputs),
    FILTER_QUERY_FUNC(query_formats),
    .priv_class    = &showwaves_class,
};

#endif // CONFIG_SHOWWAVES_FILTER

#if CONFIG_SHOWWAVESPIC_FILTER

#define OFFSET(x) offsetof(ShowWavesContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption showwavespic_options[] = {
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "600x240"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str = "600x240"}, 0, 0, FLAGS },
    { "split_channels", "draw channels separately", OFFSET(split_channels), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, FLAGS },
    { "colors", "set channels colors", OFFSET(colors), AV_OPT_TYPE_STRING, {.str = "red|green|blue|yellow|orange|lime|pink|magenta|brown" }, 0, 0, FLAGS },
    { "scale", "set amplitude scale", OFFSET(scale), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, SCALE_NB-1, FLAGS, .unit="scale" },
        { "lin", "linear",         0, AV_OPT_TYPE_CONST, {.i64=SCALE_LIN}, .flags=FLAGS, .unit="scale"},
        { "log", "logarithmic",    0, AV_OPT_TYPE_CONST, {.i64=SCALE_LOG}, .flags=FLAGS, .unit="scale"},
        { "sqrt", "square root",   0, AV_OPT_TYPE_CONST, {.i64=SCALE_SQRT}, .flags=FLAGS, .unit="scale"},
        { "cbrt", "cubic root",    0, AV_OPT_TYPE_CONST, {.i64=SCALE_CBRT}, .flags=FLAGS, .unit="scale"},
    { "draw", "set draw mode", OFFSET(draw_mode), AV_OPT_TYPE_INT, {.i64 = DRAW_SCALE}, 0, DRAW_NB-1, FLAGS, .unit="draw" },
        { "scale", "scale pixel values for each drawn sample", 0, AV_OPT_TYPE_CONST, {.i64=DRAW_SCALE}, .flags=FLAGS, .unit="draw"},
        { "full",  "draw every pixel for sample directly",     0, AV_OPT_TYPE_CONST, {.i64=DRAW_FULL},  .flags=FLAGS, .unit="draw"},
    { "filter", "set filter mode", OFFSET(filter_mode), AV_OPT_TYPE_INT, {.i64 = FILTER_AVERAGE}, 0, FILTER_NB-1, FLAGS, .unit="filter" },
        { "average", "use average samples", 0, AV_OPT_TYPE_CONST, {.i64=FILTER_AVERAGE}, .flags=FLAGS, .unit="filter"},
        { "peak",    "use peak samples",    0, AV_OPT_TYPE_CONST, {.i64=FILTER_PEAK},    .flags=FLAGS, .unit="filter"},
    { NULL }
};

AVFILTER_DEFINE_CLASS(showwavespic);

static int showwavespic_config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ShowWavesContext *s = ctx->priv;

    if (s->single_pic) {
        s->sum = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->sum));
        if (!s->sum)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static int showwavespic_filter_frame(AVFilterLink *inlink, AVFrame *insamples)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ShowWavesContext *s = ctx->priv;
    int ret = 0;

    if (s->single_pic) {
        struct frame_node *f;

        if (!s->outpicref)
            s->outpicref = alloc_out_frame(s, outlink);
        if (!s->outpicref) {
            ret = AVERROR(ENOMEM);
            goto end;
        }

        /* queue the audio frame */
        f = av_malloc(sizeof(*f));
        if (!f) {
            ret = AVERROR(ENOMEM);
            goto end;
        }
        f->frame = insamples;
        f->next = NULL;
        if (!s->last_frame) {
            s->audio_frames =
            s->last_frame   = f;
        } else {
            s->last_frame->next = f;
            s->last_frame = f;
        }
        s->total_samples += insamples->nb_samples;

        return 0;
    }

end:
    av_frame_free(&insamples);
    return ret;
}

static const AVFilterPad showwavespic_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = showwavespic_config_input,
        .filter_frame = showwavespic_filter_frame,
    },
};

static const AVFilterPad showwavespic_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
        .request_frame = request_frame,
    },
};

const AVFilter ff_avf_showwavespic = {
    .name          = "showwavespic",
    .description   = NULL_IF_CONFIG_SMALL("Convert input audio to a video output single picture."),
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(ShowWavesContext),
    FILTER_INPUTS(showwavespic_inputs),
    FILTER_OUTPUTS(showwavespic_outputs),
    FILTER_QUERY_FUNC(query_formats),
    .priv_class    = &showwavespic_class,
};

#endif // CONFIG_SHOWWAVESPIC_FILTER
