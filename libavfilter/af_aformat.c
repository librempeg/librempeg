/*
 * Copyright (c) 2011 Mina Nagy Zaki
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
 * format audio filter
 */

#include "config_components.h"

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AFormatContext {
    const AVClass   *class;

    AVFilterFormats *formats;
    AVFilterFormats *sample_rates;
    AVFilterChannelLayouts *channel_layouts;

    enum AVSampleFormat *formats_opt;
    unsigned nb_formats_opt;
    int *sample_rates_opt;
    unsigned nb_sample_rates_opt;
    AVChannelLayout *channel_layouts_opt;
    unsigned nb_channel_layouts_opt;
} AFormatContext;

#define OFFSET(x) offsetof(AFormatContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY
static const AVOptionArrayDef def_array = {.def=NULL,.size_min=0,.sep='|'};
static const AVOption aformat_options[] = {
    { "sample_fmts",     "set the list of sample formats",  OFFSET(formats_opt),     AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "f",               "set the list of sample formats",  OFFSET(formats_opt),     AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "sample_rates",    "set the list of sample rates",    OFFSET(sample_rates_opt),    AV_OPT_TYPE_INT|AR,    {.arr=&def_array}, .flags = A|F, .min = 1, .max = INT_MAX },
    { "r",               "set the list of sample rates",    OFFSET(sample_rates_opt),    AV_OPT_TYPE_INT|AR,    {.arr=&def_array}, .flags = A|F, .min = 1, .max = INT_MAX },
    { "channel_layouts", "set the list of channel layouts", OFFSET(channel_layouts_opt), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_array}, .flags = A|F },
    { "cl",              "set the list of channel layouts", OFFSET(channel_layouts_opt), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_array}, .flags = A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aformat);

#define PARSE_FORMATS(values, nb_values, type, list, op, add_to_list) \
do {                                                                  \
    int ret;                                                          \
                                                                      \
    for (int n = 0; n < nb_values; n++) {                             \
        const type fmt = values[n];                                   \
                                                                      \
        if ((ret = add_to_list(&list, op fmt)) < 0) {                 \
            return ret;                                               \
        }                                                             \
    }                                                                 \
} while (0)

static av_cold int invert_formats(AVFilterFormats **fmts,
                                  AVFilterFormats *allfmts)
{
    if (!allfmts)
        return AVERROR(ENOMEM);
    if (!*fmts) {
        /* empty fmt list means no restriction, regardless of filter type */
        ff_formats_unref(&allfmts);
        return 0;
    }

    for (int i = 0; i < allfmts->nb_formats; i++) {
        for (int j = 0; j < (*fmts)->nb_formats; j++) {
            if (allfmts->formats[i] == (*fmts)->formats[j]) {
                /* format is forbidden, remove it from allfmts list */
                memmove(&allfmts->formats[i], &allfmts->formats[i+1],
                        (allfmts->nb_formats - (i+1)) * sizeof(*allfmts->formats));
                allfmts->nb_formats--;
                i--; /* repeat loop with same idx */
                break;
            }
        }
    }

    ff_formats_unref(fmts);
    *fmts = allfmts;
    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    AFormatContext *s = ctx->priv;
    int ret;

    PARSE_FORMATS(s->formats_opt, s->nb_formats_opt, enum AVSampleFormat,
                  s->formats, , ff_add_format);
    PARSE_FORMATS(s->sample_rates_opt, s->nb_sample_rates_opt, int,
                  s->sample_rates, , ff_add_format);
    PARSE_FORMATS(s->channel_layouts_opt, s->nb_channel_layouts_opt,
                  AVChannelLayout, s->channel_layouts, &, ff_add_channel_layout);

    if (!strcmp(ctx->filter->name, "anoformat")) {
        if ((ret = invert_formats(&s->formats, ff_all_formats(AVMEDIA_TYPE_AUDIO))) < 0)
            return ret;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AFormatContext *s = ctx->priv;

    ff_formats_unref(&s->formats);
    ff_formats_unref(&s->sample_rates);
    ff_channel_layouts_unref(&s->channel_layouts);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AFormatContext *s = ctx->priv;
    int ret;

    if (s->formats) {
        ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, s->formats);
        if (ret < 0)
            return ret;
    }

    if (s->sample_rates) {
        ret = ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, s->sample_rates);
        if (ret < 0)
            return ret;
    }

    if (s->channel_layouts) {
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
const AVFilter ff_af_aformat = {
    .name          = "aformat",
    .description   = NULL_IF_CONFIG_SMALL("Convert the input audio to one of the specified formats."),
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(AFormatContext),
    .priv_class    = &aformat_class,
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_AFORMAT_FILTER */

#if CONFIG_ANOFORMAT_FILTER
static const AVOption anoformat_options[] = {
    { "sample_fmts", "set the list of sample formats", OFFSET(formats_opt), AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { "f",           "set the list of sample formats", OFFSET(formats_opt), AV_OPT_TYPE_SAMPLE_FMT|AR, {.arr=&def_array}, .flags = A|F, .min = AV_SAMPLE_FMT_NONE+1, .max = AV_SAMPLE_FMT_NB-1 },
    { NULL }
};

AVFILTER_DEFINE_CLASS(anoformat);

const AVFilter ff_af_anoformat = {
    .name          = "anoformat",
    .description   = NULL_IF_CONFIG_SMALL("Force libavfilter not to use any of the specified sample formats for the input to the next filter."),
    .priv_class    = &anoformat_class,
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(AFormatContext),
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_ANOFORMAT_FILTER */
