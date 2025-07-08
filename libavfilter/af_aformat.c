/*
 * Copyright (c) 2011 Mina Nagy Zaki
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
 * format audio filter
 */

#include "config_components.h"

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AFormatContext {
    const AVClass   *class;

    enum AVSampleFormat *formats;
    unsigned nb_formats;
    int *sample_rates;
    unsigned nb_sample_rates;
    AVChannelLayout *channel_layouts;
    unsigned nb_channel_layouts;
} AFormatContext;

#define OFFSET(x) offsetof(AFormatContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY
static const AVOptionArrayDef def_array = {.def=NULL,.size_min=0,.sep='|'};
static const AVOption aformat_options[] = {
    { "sample_formats",  "set the list of sample formats",  OFFSET(formats),     AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "f",               "set the list of sample formats",  OFFSET(formats),     AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "sample_rates",    "set the list of sample rates",    OFFSET(sample_rates),    AV_OPT_TYPE_INT|AR,    {.arr=&def_array}, .flags = A|F, .min = 1, .max = INT_MAX },
    { "r",               "set the list of sample rates",    OFFSET(sample_rates),    AV_OPT_TYPE_INT|AR,    {.arr=&def_array}, .flags = A|F, .min = 1, .max = INT_MAX },
    { "channel_layouts", "set the list of channel layouts", OFFSET(channel_layouts), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_array}, .flags = A|F },
    { "cl",              "set the list of channel layouts", OFFSET(channel_layouts), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_array}, .flags = A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aformat);

static av_cold int init(AVFilterContext *ctx)
{
    AFormatContext *s = ctx->priv;

    // terminate format lists for ff_set*_from_list()
    if (s->nb_formats) {
        void *tmp = av_realloc_array(s->formats, s->nb_formats + 1,
                                     sizeof(*s->formats));
        if (!tmp)
            return AVERROR(ENOMEM);
        s->formats = tmp;
        s->formats[s->nb_formats] = AV_SAMPLE_FMT_NONE;

    }
    if (s->nb_sample_rates) {
        void *tmp = av_realloc_array(s->sample_rates, s->nb_sample_rates + 1,
                                     sizeof(*s->sample_rates));
        if (!tmp)
            return AVERROR(ENOMEM);
        s->sample_rates = tmp;
        s->sample_rates[s->nb_sample_rates] = -1;
    }
    if (s->nb_channel_layouts) {
        void *tmp = av_realloc_array(s->channel_layouts, s->nb_channel_layouts + 1,
                                     sizeof(*s->channel_layouts));
        if (!tmp)
            return AVERROR(ENOMEM);
        s->channel_layouts = tmp;
        s->channel_layouts[s->nb_channel_layouts] = (AVChannelLayout){ .nb_channels = 0 };
    }

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AFormatContext *s = ctx->priv;
    int ret;

    if (s->nb_formats) {
        if (!strcmp(ctx->filter->name, "anoformat")) {
            ret = ff_set_common_formats_from_list2_invert(ctx, cfg_in, cfg_out, s->formats, AVMEDIA_TYPE_AUDIO);
        } else {
            ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, s->formats);
        }
        if (ret < 0)
            return ret;
    }

    if (s->nb_sample_rates) {
        ret = ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, s->sample_rates);
        if (ret < 0)
            return ret;
    }

    if (s->nb_channel_layouts) {
        ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, s->channel_layouts);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_AUDIO,
        .get_buffer.audio = ff_null_get_audio_buffer,
    },
};

#if CONFIG_AFORMAT_FILTER
const FFFilter ff_af_aformat = {
    .p.name        = "aformat",
    .p.description = NULL_IF_CONFIG_SMALL("Convert the input audio to one of the specified formats."),
    .p.priv_class  = &aformat_class,
    .priv_size     = sizeof(AFormatContext),
    .init          = init,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_AFORMAT_FILTER */

#if CONFIG_ANOFORMAT_FILTER
static const AVOption anoformat_options[] = {
    { "sample_formats", "set the list of sample formats", OFFSET(formats), AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "f",           "set the list of sample formats", OFFSET(formats), AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { NULL }
};

AVFILTER_DEFINE_CLASS(anoformat);

const FFFilter ff_af_anoformat = {
    .p.name        = "anoformat",
    .p.description = NULL_IF_CONFIG_SMALL("Force libavfilter not to use any of the specified sample formats for the input to the next filter."),
    .p.priv_class  = &anoformat_class,
    .priv_size     = sizeof(AFormatContext),
    .init          = init,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_ANOFORMAT_FILTER */
