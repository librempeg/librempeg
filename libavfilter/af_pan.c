/*
 * Copyright (c) 2002 Anders Johansson <ajh@atri.curtin.edu.au>
 * Copyright (c) 2011 Clément Bœsch <u pkh me>
 * Copyright (c) 2011 Nicolas George <nicolas.george@normalesup.org>
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

/**
 * @file
 * Audio panning filter (channels mixing)
 * Original code written by Anders Johansson for MPlayer,
 * reimplemented for FFmpeg.
 */

#include <stdio.h>
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#define MAX_CHANNELS 64

typedef struct PanContext {
    const AVClass *class;
    AVChannelLayout layout;
    char **args;
    unsigned nb_args;
    double gain[MAX_CHANNELS][MAX_CHANNELS];
    uint8_t need_renorm[MAX_CHANNELS];
    int need_renumber;

    int pure_gains;
    /* channel mapping specific */
    int channel_map[MAX_CHANNELS];
} PanContext;

static void skip_spaces(char **arg)
{
    int len = 0;

    sscanf(*arg, " %n", &len);
    *arg += len;
}

static int parse_channel_name(char **arg, int *rchannel, int *rnamed)
{
    char buf[8];
    int len, channel_id = 0;

    skip_spaces(arg);
    /* try to parse a channel name, e.g. "FL" */
    if (sscanf(*arg, "%7[A-Z]%n", buf, &len) >= 1) {
        channel_id = av_channel_from_string(buf);
        if (channel_id < 0)
            return channel_id;

        *rchannel = channel_id;
        *rnamed = 1;
        *arg += len;
        return 0;
    }
    /* try to parse a channel number, e.g. "c2" */
    if (sscanf(*arg, "c%d%n", &channel_id, &len) >= 1 &&
        channel_id >= 0 && channel_id < MAX_CHANNELS) {
        *rchannel = channel_id;
        *rnamed = 0;
        *arg += len;
        return 0;
    }
    return AVERROR(EINVAL);
}

static int are_gains_pure(const PanContext *pan)
{
    int i, j;

    for (i = 0; i < MAX_CHANNELS; i++) {
        int nb_gain = 0;

        for (j = 0; j < MAX_CHANNELS; j++) {
            double gain = pan->gain[i][j];

            /* channel mapping is effective only if 0% or 100% of a channel is
             * selected... */
            if (gain != 0. && gain != 1.)
                return 0;
            /* ...and if the output channel is only composed of one input */
            if (gain && nb_gain++)
                return 0;
        }
    }
    return 1;
}

static av_cold int init(AVFilterContext *ctx)
{
    PanContext *const pan = ctx->priv;
    char *args = NULL;
    int out_ch_id, in_ch_id, len, named, ret, sign = 1;
    int nb_in_channels[2] = { 0, 0 }; // number of unnamed and named input channels
    int used_out_ch[MAX_CHANNELS] = {0};
    double gain;

    if (!pan->layout.nb_channels || !pan->nb_args) {
        av_log(ctx, AV_LOG_ERROR,
               "pan filter needs a output channel layout and a set "
               "of channel definitions as parameter\n");
        return AVERROR(EINVAL);
    }

    if (pan->layout.nb_channels > MAX_CHANNELS) {
        av_log(ctx, AV_LOG_ERROR,
               "af_pan supports a maximum of %d channels. "
               "Feel free to ask for a higher limit.\n", MAX_CHANNELS);
        ret = AVERROR_PATCHWELCOME;
        goto fail;
    }

    /* parse channel specifications */
    for (int n = 0; n < pan->nb_args; n++) {
        const char *arg0 = pan->args[n];
        int used_in_ch[MAX_CHANNELS] = {0};
        char *arg;

        av_freep(&args);
        args = arg = av_strdup(pan->args[n]);
        if (!args) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        /* channel name */
        if (parse_channel_name(&arg, &out_ch_id, &named)) {
            av_log(ctx, AV_LOG_ERROR,
                   "Expected out channel name, got \"%.8s\"\n", arg);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        if (named) {
            if ((out_ch_id = av_channel_layout_index_from_channel(&pan->layout, out_ch_id)) < 0) {
                av_log(ctx, AV_LOG_ERROR,
                       "Channel \"%.8s\" does not exist in the chosen layout\n", arg0);
                ret = AVERROR(EINVAL);
                goto fail;
            }
        }
        if (out_ch_id < 0 || out_ch_id >= pan->layout.nb_channels) {
            av_log(ctx, AV_LOG_ERROR,
                   "Invalid out channel name \"%.8s\"\n", arg0);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        if (used_out_ch[out_ch_id]) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can not reference out channel %d twice\n", out_ch_id);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        used_out_ch[out_ch_id] = 1;
        skip_spaces(&arg);
        if (*arg == '=') {
            arg++;
        } else if (*arg == '<') {
            pan->need_renorm[out_ch_id] = 1;
            arg++;
        } else {
            av_log(ctx, AV_LOG_ERROR,
                   "Syntax error after channel name in \"%.8s\"\n", arg0);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        /* gains */
        sign = 1;
        while (1) {
            gain = 1;
            if (sscanf(arg, "%lf%n *%n", &gain, &len, &len) >= 1)
                arg += len;
            if (parse_channel_name(&arg, &in_ch_id, &named)){
                av_log(ctx, AV_LOG_ERROR,
                       "Expected in channel name, got \"%.8s\"\n", arg);
                 ret = AVERROR(EINVAL);
                 goto fail;
            }
            nb_in_channels[named]++;
            if (nb_in_channels[!named]) {
                av_log(ctx, AV_LOG_ERROR,
                       "Can not mix named and numbered channels\n");
                ret = AVERROR(EINVAL);
                goto fail;
            }
            if (used_in_ch[in_ch_id]) {
                av_log(ctx, AV_LOG_ERROR,
                       "Can not reference in channel %d twice\n", in_ch_id);
                ret = AVERROR(EINVAL);
                goto fail;
            }
            used_in_ch[in_ch_id] = 1;
            pan->gain[out_ch_id][in_ch_id] = sign * gain;
            skip_spaces(&arg);
            if (!*arg)
                break;
            if (*arg == '-') {
                sign = -1;
            } else if (*arg != '+') {
                av_log(ctx, AV_LOG_ERROR, "Syntax error near \"%.8s\"\n", arg);
                ret = AVERROR(EINVAL);
                goto fail;
            } else {
                sign = 1;
            }
            arg++;
        }
    }
    pan->need_renumber = !!nb_in_channels[1];
    pan->pure_gains = are_gains_pure(pan);

    ret = 0;
fail:
    av_freep(&args);
    return ret;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const PanContext *pan = ctx->priv;
    AVFilterChannelLayouts *layouts;
    int ret;

    // inlink supports any channel layout
    layouts = ff_all_channel_counts();
    if ((ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts)) < 0)
        return ret;

    // outlink supports only requested output channel layout
    layouts = NULL;
    if ((ret = ff_add_channel_layout(&layouts, &pan->layout)) < 0)
        return ret;
    return ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
}

static int config_props(AVFilterLink *link)
{
    AVFilterContext *ctx = link->dst;
    PanContext *pan = ctx->priv;
    char buf[1024], *cur;
    int i, j, k, r;
    double t;

    if (pan->need_renumber) {
        // input channels were given by their name: renumber them
        for (i = j = 0; i < MAX_CHANNELS; i++) {
            if (av_channel_layout_index_from_channel(&link->ch_layout, i) >= 0) {
                for (k = 0; k < pan->layout.nb_channels; k++)
                    pan->gain[k][j] = pan->gain[k][i];
                j++;
            }
        }
    }

    // sanity check; can't be done in query_formats since the inlink
    // channel layout is unknown at that time
    if (link->ch_layout.nb_channels > MAX_CHANNELS ||
        pan->layout.nb_channels > MAX_CHANNELS) {
        av_log(ctx, AV_LOG_ERROR,
               "af_pan supports a maximum of %d channels. "
               "Feel free to ask for a higher limit.\n", MAX_CHANNELS);
        return AVERROR_PATCHWELCOME;
    }

    // gains are pure, init the channel mapping
    if (pan->pure_gains) {
        // get channel map from the pure gains
        for (i = 0; i < pan->layout.nb_channels; i++) {
            int ch_id = -1;
            for (j = 0; j < link->ch_layout.nb_channels; j++) {
                if (pan->gain[i][j]) {
                    ch_id = j;
                    break;
                }
            }
            pan->channel_map[i] = ch_id;
        }
    } else {
        // renormalize
        for (i = 0; i < pan->layout.nb_channels; i++) {
            if (!pan->need_renorm[i])
                continue;
            t = 0;
            for (j = 0; j < link->ch_layout.nb_channels; j++)
                t += fabs(pan->gain[i][j]);
            if (t > -1E-5 && t < 1E-5) {
                // t is almost 0 but not exactly, this is probably a mistake
                if (t)
                    av_log(ctx, AV_LOG_WARNING,
                           "Degenerate coefficients while renormalizing\n");
                continue;
            }
            for (j = 0; j < link->ch_layout.nb_channels; j++)
                pan->gain[i][j] /= t;
        }
    }

    // summary
    for (i = 0; i < pan->layout.nb_channels; i++) {
        cur = buf;
        for (j = 0; j < link->ch_layout.nb_channels; j++) {
            r = snprintf(cur, buf + sizeof(buf) - cur, "%s%.3g i%d",
                         j ? " + " : "", pan->gain[i][j], j);
            cur += FFMIN(buf + sizeof(buf) - cur, r);
        }
        av_log(ctx, AV_LOG_VERBOSE, "o%d = %s\n", i, buf);
    }
    // add channel mapping summary if possible
    if (pan->pure_gains) {
        av_log(ctx, AV_LOG_INFO, "Pure channel mapping detected:");
        for (i = 0; i < pan->layout.nb_channels; i++)
            if (pan->channel_map[i] < 0)
                av_log(ctx, AV_LOG_INFO, " M");
            else
                av_log(ctx, AV_LOG_INFO, " %d", pan->channel_map[i]);
        av_log(ctx, AV_LOG_INFO, "\n");
        return 0;
    }
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    int ret;
    int n = in->nb_samples;
    AVFilterLink *const outlink = ctx->outputs[0];
    const int planar = av_sample_fmt_is_planar(outlink->format);
    AVFrame *out = ff_get_audio_buffer(outlink, n);
    const int out_channels = out->ch_layout.nb_channels;
    const int in_channels = in->ch_layout.nb_channels;
    PanContext *pan = ctx->priv;

    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);
    if ((ret = av_channel_layout_copy(&out->ch_layout, &outlink->ch_layout)) < 0) {
        av_frame_free(&out);
        av_frame_free(&in);
        return ret;
    }

    if (pan->pure_gains) {
        const int bps = av_get_bytes_per_sample(outlink->format);

        if (planar) {
            for (int ch = 0; ch < out_channels; ch++) {
                const int in_ch = pan->channel_map[ch];
                uint8_t *dst = out->extended_data[ch];

                if (in_ch < 0) {
                    const uint8_t fill = (bps == 8) ? 128 : 0;

                    memset(dst, fill, sizeof(*dst) * n * bps);
                } else {
                    const uint8_t *src = in->extended_data[in_ch];

                    memcpy(dst, src, sizeof(*dst) * n * bps);
                }
            }
        } else {
            switch (outlink->format) {
            case AV_SAMPLE_FMT_U8:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const uint8_t *src = in->data[0] + in_ch;
                    uint8_t *dst = out->data[0] + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 128;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            case AV_SAMPLE_FMT_S16:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const int16_t *src = ((const int16_t *)in->data[0]) + in_ch;
                    int16_t *dst = ((int16_t *)out->data[0]) + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 0;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            case AV_SAMPLE_FMT_S32:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const int32_t *src = ((const int32_t *)in->data[0]) + in_ch;
                    int32_t *dst = ((int32_t *)out->data[0]) + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 0;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            case AV_SAMPLE_FMT_S64:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const int64_t *src = ((const int64_t *)in->data[0]) + in_ch;
                    int64_t *dst = ((int64_t *)out->data[0]) + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 0;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            case AV_SAMPLE_FMT_FLT:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const float *src = ((const float *)in->data[0]) + in_ch;
                    float *dst = ((float *)out->data[0]) + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 0;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            case AV_SAMPLE_FMT_DBL:
                for (int ch = 0; ch < out_channels; ch++) {
                    const int in_ch = pan->channel_map[ch];
                    const double *src = ((const double *)in->data[0]) + in_ch;
                    double *dst = ((double *)out->data[0]) + ch;

                    if (in_ch < 0) {
                        for (int i = 0; i < n; i++) {
                            dst[0] = 0;
                            dst += out_channels;
                        }
                    } else {
                        for (int i = 0; i < n; i++) {
                            dst[0] = src[0];
                            src += in_channels;
                            dst += out_channels;
                        }
                    }
                }
                break;
            }
        }
    } else {
        for (int ch = 0; ch < out_channels; ch++) {
            const double *gains = pan->gain[ch];

            if (planar) {
                switch (outlink->format) {
                case AV_SAMPLE_FMT_U8P:
                    for (int i = 0; i < n; i++) {
                        uint8_t *dst = out->extended_data[ch];
                        float sum = 0.f;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const uint8_t *src = (const uint8_t *)in->extended_data[ich];

                            sum += gains[ich] * (src[i] - 0x80);
                        }

                        dst[i] = av_clip_int8(sum) + 0x80;
                    }
                    break;
                case AV_SAMPLE_FMT_S16P:
                    for (int i = 0; i < n; i++) {
                        int16_t *dst = (int16_t *)out->extended_data[ch];
                        float sum = 0.f;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const int16_t *src = (const int16_t *)in->extended_data[ich];

                            sum += gains[ich] * src[i];
                        }

                        dst[i] = av_clip_int16(sum);
                    }
                    break;
                case AV_SAMPLE_FMT_S32P:
                    for (int i = 0; i < n; i++) {
                        int32_t *dst = (int32_t *)out->extended_data[ch];
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const int32_t *src = (const int32_t *)in->extended_data[ich];

                            sum += gains[ich] * src[i];
                        }

                        dst[i] = av_clipl_int32(sum);
                    }
                    break;
                case AV_SAMPLE_FMT_S64P:
                    for (int i = 0; i < n; i++) {
                        int64_t *dst = (int64_t *)out->extended_data[ch];
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const int64_t *src = (const int64_t *)in->extended_data[ich];

                            sum += gains[ich] * src[i];
                        }

                        dst[i] = av_clipl_int32(sum);
                    }
                    break;
                case AV_SAMPLE_FMT_FLTP:
                    for (int i = 0; i < n; i++) {
                        float *dst = (float *)out->extended_data[ch];
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const float *src = (const float *)in->extended_data[ich];

                            sum += gains[ich] * src[i];
                        }

                        dst[i] = sum;
                    }
                    break;
                case AV_SAMPLE_FMT_DBLP:
                    for (int i = 0; i < n; i++) {
                        double *dst = (double *)out->extended_data[ch];
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++) {
                            const double *src = (const double *)in->extended_data[ich];

                            sum += gains[ich] * src[i];
                        }

                        dst[i] = sum;
                    }
                    break;
                }
            } else {
                switch (outlink->format) {
                case AV_SAMPLE_FMT_U8:
                    for (int i = 0; i < n; i++) {
                        const uint8_t *src = ((const uint8_t *)in->data[0]) + in_channels * i;
                        uint8_t *dst = out->data[0] + out_channels * i + ch;
                        float sum = 0.f;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * (src[ich] - 0x80);

                        dst[0] = av_clip_int8(sum) + 0x80;
                    }
                    break;
                case AV_SAMPLE_FMT_S16:
                    for (int i = 0; i < n; i++) {
                        const int16_t *src = ((const int16_t *)in->data[0]) + in_channels * i;
                        int16_t *dst = ((int16_t *)out->data[0]) + out_channels * i + ch;
                        float sum = 0.f;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * src[ich];

                        dst[0] = av_clip_int16(lrintf(sum));
                    }
                    break;
                case AV_SAMPLE_FMT_S32:
                    for (int i = 0; i < n; i++) {
                        const int32_t *src = ((const int32_t *)in->data[0]) + in_channels * i;
                        int32_t *dst = ((int32_t *)out->data[0]) + out_channels * i + ch;
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * src[ich];

                        dst[0] = av_clipl_int32(sum);
                    }
                    break;
                case AV_SAMPLE_FMT_S64:
                    for (int i = 0; i < n; i++) {
                        const int64_t *src = ((const int64_t *)in->data[0]) + in_channels * i;
                        int64_t *dst = ((int64_t *)out->data[0]) + out_channels * i + ch;
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * src[ich];

                        dst[0] = av_clipl_int32(sum);
                    }
                    break;
                case AV_SAMPLE_FMT_FLT:
                    for (int i = 0; i < n; i++) {
                        const float *src = ((const float *)in->data[0]) + in_channels * i;
                        float *dst = ((float *)out->data[0]) + out_channels * i + ch;
                        float sum = 0.f;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * src[ich];

                        dst[0] = sum;
                    }
                    break;
                case AV_SAMPLE_FMT_DBL:
                    for (int i = 0; i < n; i++) {
                        const double *src = ((const double *)in->data[0]) + in_channels * i;
                        double *dst = ((double *)out->data[0]) + out_channels * i + ch;
                        double sum = 0.0;

                        for (int ich = 0; ich < in_channels; ich++)
                            sum += gains[ich] * src[ich];

                        dst[0] = sum;
                    }
                    break;
                }
            }
        }
    }

    ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
}

#define OFFSET(x) offsetof(PanContext, x)
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_mix = {.def="FC=FC",.size_min=1,.sep='|'};

static const AVOption pan_options[] = {
    { "layout", "set the output channel layout", OFFSET(layout), AV_OPT_TYPE_CHLAYOUT,  { .str = "mono" },   0, 0, AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { "mix", "set the output channel mix gains", OFFSET(args),   AV_OPT_TYPE_STRING|AR, { .arr = &def_mix }, 0, 0, AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { NULL }
};

AVFILTER_DEFINE_CLASS(pan);

static const AVFilterPad pan_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_props,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_pan = {
    .p.name        = "pan",
    .p.description = NULL_IF_CONFIG_SMALL("Remix channels with coefficients (panning)."),
    .p.priv_class  = &pan_class,
    .priv_size     = sizeof(PanContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(pan_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
