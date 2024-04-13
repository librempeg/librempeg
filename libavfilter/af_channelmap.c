/*
 * Copyright (c) 2012 Google, Inc.
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
 * audio channel mapping filter
 */

#include <ctype.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"

#include "audio.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"

struct ChannelMap {
    int in_channel;
    int out_channel;
    int in_channel_idx;
    int out_channel_idx;
    uint8_t in_empty;
    uint8_t out_empty;
};

enum MappingMode {
    MAP_NONE,
    MAP_ONE_INT,
    MAP_ONE_STR,
    MAP_PAIR_INT_INT,
    MAP_PAIR_INT_STR,
    MAP_PAIR_STR_INT,
    MAP_PAIR_STR_STR
};

#define IN_EMPTY  1
#define OUT_EMPTY 2

#define MAX_CH 64
typedef struct ChannelMapContext {
    const AVClass *class;
    char **mapping_str;
    unsigned nb_mapping_str;
    AVChannelLayout output_layout;
    struct ChannelMap map[MAX_CH];
    int nch;
    int flags;
    enum MappingMode mode;
} ChannelMapContext;

#define OFFSET(x) offsetof(ChannelMapContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_map = {.def=NULL,.size_min=0,.sep='|'};

static const AVOption channelmap_options[] = {
    { "map", "set the list of input channel numbers in output order.",
          OFFSET(mapping_str),        AV_OPT_TYPE_STRING|AR, {.arr = &def_map}, .flags = A|F },
    { "channel_layout", "Output channel layout.",
          OFFSET(output_layout),      AV_OPT_TYPE_CHLAYOUT, .flags = A|F },
    { "flags", "set the mapping flags.",
          OFFSET(flags),              AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 3, .flags = A|F, .unit = "flags" },
    {  "in_empty",  "allow empty input channels",  0, AV_OPT_TYPE_CONST, {.i64 = IN_EMPTY},  .flags = A|F, .unit = "flags" },
    {  "out_empty", "allow empty output channels", 0, AV_OPT_TYPE_CONST, {.i64 = OUT_EMPTY}, .flags = A|F, .unit = "flags" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(channelmap);

static const char* split(const char *message, char delim) {
    const char *next = strchr(message, delim);
    return next;
}

static int get_channel_idx(const char **map, int *ch, char delim, int max_nb_channels)
{
    const char *next;
    int len;
    int n = 0;
    if (!*map)
        return AVERROR(EINVAL);
    next = split(*map, delim);
    if (!next && delim == '-')
        return AVERROR(EINVAL);
    len = strlen(*map);
    sscanf(*map, "%d%n", ch, &n);
    if (n != len)
        return AVERROR(EINVAL);
    if (*ch < 0 || *ch >= max_nb_channels)
        return AVERROR(EINVAL);
    *map = next;
    return 0;
}

static int get_channel(const char **map, int *ch, char delim)
{
    const char *next = split(*map, delim);
    if (!next && delim == '-')
        return AVERROR(EINVAL);
    *ch = av_channel_from_string(*map);
    if (*ch < 0)
        return AVERROR(EINVAL);
    *map = next;
    return 0;
}

static int check_idx_and_id(AVFilterContext *ctx, int channel_idx, int channel, AVChannelLayout *ch_layout, const char *io,
                            const int flag)
{
    ChannelMapContext *s = ctx->priv;
    char channel_name[64];
    char layout_name[256];
    int nb_channels = ch_layout->nb_channels;

    if (channel_idx < 0 || channel_idx >= nb_channels) {
        if (s->flags & flag)
            return 1;
        av_channel_layout_describe(ch_layout, layout_name, sizeof(layout_name));
        if (channel >= 0) {
            av_channel_name(channel_name, sizeof(channel_name), channel);
            av_log(ctx, AV_LOG_ERROR,
                   "%sput channel '%s' not available from %sput layout '%s'\n",
                   io, channel_name, io, layout_name);
        } else {
            av_log(ctx, AV_LOG_ERROR,
                   "%sput channel #%d not available from %sput layout '%s'\n",
                   io, channel_idx, io, layout_name);
        }
        return AVERROR(EINVAL);
    }

    return 0;
}

static av_cold int channelmap_init(AVFilterContext *ctx)
{
    ChannelMapContext *s = ctx->priv;
    enum MappingMode mode;
    int64_t out_ch_mask = 0;
    uint64_t presence_mask;
    int i;

    if (!s->nb_mapping_str) {
        mode = MAP_NONE;
    } else {
        const char *mapping = s->mapping_str[0];
        const char *dash = strchr(mapping, '-');
        if (!dash) {  // short mapping
            if (av_isdigit(*mapping))
                mode = MAP_ONE_INT;
            else
                mode = MAP_ONE_STR;
        } else if (av_isdigit(*mapping)) {
            if (av_isdigit(*(dash+1)))
                mode = MAP_PAIR_INT_INT;
            else
                mode = MAP_PAIR_INT_STR;
        } else {
            if (av_isdigit(*(dash+1)))
                mode = MAP_PAIR_STR_INT;
            else
                mode = MAP_PAIR_STR_STR;
        }
    }

    if (s->nb_mapping_str > MAX_CH) {
        av_log(ctx, AV_LOG_ERROR, "Too many channels mapped: '%d'.\n", s->nb_mapping_str);
        return AVERROR(EINVAL);
    }

    for (i = 0; i < MAX_CH; i++) {
        s->map[i].in_channel_idx  = -1;
        s->map[i].out_channel_idx = -1;
        s->map[i].in_channel      = -1;
        s->map[i].out_channel     = -1;
    }

    for (i = 0; i < s->nb_mapping_str; i++) {
        const char *mapping = s->mapping_str[i];
        int in_ch_idx = -1, out_ch_idx = -1;
        int in_ch = -1, out_ch = -1;
        static const char err[] = "Failed to parse channel map\n";
        switch (mode) {
        case MAP_ONE_INT:
            if (get_channel_idx(&mapping, &in_ch_idx, 0, MAX_CH) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel_idx  = in_ch_idx;
            s->map[i].out_channel_idx = i;
            break;
        case MAP_ONE_STR:
            if (get_channel(&mapping, &in_ch, 0) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel      = in_ch;
            s->map[i].out_channel_idx = i;
            break;
        case MAP_PAIR_INT_INT:
            if (get_channel_idx(&mapping, &in_ch_idx, '-', MAX_CH) < 0 ||
                get_channel_idx(&mapping, &out_ch_idx, 0, MAX_CH) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel_idx  = in_ch_idx;
            s->map[i].out_channel_idx = out_ch_idx;
            break;
        case MAP_PAIR_INT_STR:
            if (get_channel_idx(&mapping, &in_ch_idx, '-', MAX_CH) < 0 ||
                get_channel(&mapping, &out_ch, 0) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel_idx  = in_ch_idx;
            s->map[i].out_channel     = out_ch;
            if (out_ch < 63)
                out_ch_mask |= 1ULL << out_ch;
            else
                out_ch_mask = -1;
            break;
        case MAP_PAIR_STR_INT:
            if (get_channel(&mapping, &in_ch, '-') < 0 ||
                get_channel_idx(&mapping, &out_ch_idx, 0, MAX_CH) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel      = in_ch;
            s->map[i].out_channel_idx = out_ch_idx;
            break;
        case MAP_PAIR_STR_STR:
            if (get_channel(&mapping, &in_ch, '-') < 0 ||
                get_channel(&mapping, &out_ch, 0) < 0) {
                av_log(ctx, AV_LOG_ERROR, err);
                return AVERROR(EINVAL);
            }
            s->map[i].in_channel = in_ch;
            s->map[i].out_channel = out_ch;
            if (out_ch < 63)
                out_ch_mask |= 1ULL << out_ch;
            else
                out_ch_mask = -1;
            break;
        }
    }
    s->mode          = mode;
    s->nch           = s->nb_mapping_str;
    if (s->output_layout.nb_channels == 0) {
        if (out_ch_mask > 0)
            av_channel_layout_from_mask(&s->output_layout, out_ch_mask);
        else if (s->nb_mapping_str)
            av_channel_layout_default(&s->output_layout, s->nb_mapping_str);
    }

    if (mode == MAP_NONE) {
        int i;
        s->nch = s->output_layout.nb_channels;
        for (i = 0; i < s->nch; i++) {
            s->map[i].in_channel_idx  = i;
            s->map[i].out_channel_idx = i;
        }
    } else if (s->nch != s->output_layout.nb_channels) {
        char buf[256];
        av_channel_layout_describe(&s->output_layout, buf, sizeof(buf));
        av_log(ctx, AV_LOG_ERROR,
               "Output channel layout %s does not match the number of channels mapped %d.\n",
               buf, s->nch);
        return AVERROR(EINVAL);
    }

    if (!s->output_layout.nb_channels) {
        av_log(ctx, AV_LOG_ERROR, "Output channel layout is not set and "
               "cannot be guessed from the maps.\n");
        return AVERROR(EINVAL);
    }

    if (mode == MAP_PAIR_INT_STR || mode == MAP_PAIR_STR_STR) {
        for (i = 0; i < s->nch; i++) {
            s->map[i].out_channel_idx = av_channel_layout_index_from_channel(
                &s->output_layout, s->map[i].out_channel);
        }
    }

    presence_mask = 0;
    for (i = 0; i < s->nch; i++) {
        uint64_t idx_mask;
        int ret = check_idx_and_id(ctx, s->map[i].out_channel_idx, s->map[i].out_channel, &s->output_layout, "out", OUT_EMPTY);
        if (ret < 0)
            return ret;
        s->map[i].out_empty = ret;
        if (s->map[i].out_empty)
            continue;
        idx_mask = (1ULL << s->map[i].out_channel_idx);
        if (presence_mask & idx_mask) {
            char layout_name[256];
            av_channel_layout_describe(&s->output_layout, layout_name, sizeof(layout_name));
            av_log(ctx, AV_LOG_ERROR, "Mapping %d assigns channel #%d twice in output layout '%s'.\n",
                   i + 1, s->map[i].out_channel_idx, layout_name);
            return AVERROR(EINVAL);
        }
        presence_mask |= idx_mask;
    }

    return 0;
}

static int channelmap_query_formats(AVFilterContext *ctx)
{
    ChannelMapContext *s = ctx->priv;
    AVFilterChannelLayouts *channel_layouts = NULL;
    int ret;

    if ((ret = ff_set_common_formats    (ctx,  ff_planar_sample_fmts()))  < 0 ||
        (ret = ff_set_common_all_samplerates(ctx                              )) < 0 ||
        (ret = ff_add_channel_layout(&channel_layouts, &s->output_layout)) < 0 ||
        (ret = ff_channel_layouts_ref(channel_layouts,
                                      &ctx->outputs[0]->incfg.channel_layouts)) < 0)
        return ret;

    return ff_channel_layouts_ref(ff_all_channel_counts(),
                                  &ctx->inputs[0]->outcfg.channel_layouts);
}

static int channelmap_filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext  *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    const ChannelMapContext *s = ctx->priv;
    const int sample_size = av_get_bytes_per_sample(in->format);
    const int nch_out = s->nch;
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    for (int ch = 0; ch < nch_out; ch++) {
        if (s->map[ch].out_empty || s->map[ch].in_empty)
            continue;
        memcpy(out->extended_data[s->map[ch].out_channel_idx],
               in->extended_data[s->map[ch].in_channel_idx],
               in->nb_samples * sample_size);
    }

    av_frame_copy_props(out, in);
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int channelmap_config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ChannelMapContext *s = ctx->priv;
    int err = 0;

    for (int i = 0; i < s->nch; i++) {
        struct ChannelMap *m = &s->map[i];
        int ret;

        if (s->mode == MAP_PAIR_STR_INT || s->mode == MAP_PAIR_STR_STR || s->mode == MAP_ONE_STR) {
            m->in_channel_idx = av_channel_layout_index_from_channel(
                &inlink->ch_layout, m->in_channel);
        }

        ret = check_idx_and_id(ctx, m->in_channel_idx, m->in_channel, &inlink->ch_layout, "in", IN_EMPTY);
        if (ret < 0)
            err = AVERROR(EINVAL);
        m->in_empty = ret == 1;
    }

    return err;
}

static const AVFilterPad channelmap_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .filter_frame   = channelmap_filter_frame,
        .config_props   = channelmap_config_input,
    },
};

const AVFilter ff_af_channelmap = {
    .name          = "channelmap",
    .description   = NULL_IF_CONFIG_SMALL("Remap audio channels."),
    .init          = channelmap_init,
    .priv_size     = sizeof(ChannelMapContext),
    .priv_class    = &channelmap_class,
    FILTER_INPUTS(channelmap_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC(channelmap_query_formats),
};
