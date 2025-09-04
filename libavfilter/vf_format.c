/*
 * Copyright (c) 2007 Bobby Bingham
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
 * format and noformat video filters
 */

#include "config_components.h"

#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

typedef struct FormatContext {
    const AVClass *class;
    int *pix_fmts;
    unsigned nb_pix_fmts;
    char **csps;
    unsigned nb_csps;
    char **ranges;
    unsigned nb_ranges;
    char **alphamodes;
    unsigned nb_alphamodes;

    AVFilterFormats *formats; ///< parsed from `pix_fmts`
    AVFilterFormats *color_spaces; ///< parsed from `csps`
    AVFilterFormats *color_ranges; ///< parsed from `ranges`
    AVFilterFormats *alpha_modes; ///< parsed from `alphamodes`
} FormatContext;

static av_cold void uninit(AVFilterContext *ctx)
{
    FormatContext *s = ctx->priv;
    ff_formats_unref(&s->formats);
    ff_formats_unref(&s->color_spaces);
    ff_formats_unref(&s->color_ranges);
    ff_formats_unref(&s->alpha_modes);
}

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
    FormatContext *s = ctx->priv;
    int ret;

    for (unsigned n = 0; n < s->nb_pix_fmts; n++) {
        enum AVPixelFormat pix_fmt = s->pix_fmts[n];

        if ((ret = ff_add_format(&s->formats, pix_fmt)) < 0)
            return ret;
    }

    for (unsigned n = 0; n < s->nb_csps; n++) {
        const char *cur = s->csps[n];

        if ((ret = av_color_space_from_name(cur)) < 0 ||
            (ret = ff_add_format(&s->color_spaces, ret)) < 0)
            return ret;
    }

    for (unsigned n = 0; n < s->nb_ranges; n++) {
        const char *cur = s->ranges[n];

        if ((ret = av_color_range_from_name(cur)) < 0 ||
            (ret = ff_add_format(&s->color_ranges, ret)) < 0)
            return ret;
    }

    for (unsigned n = 0; n < s->nb_alphamodes; n++) {
        const char *cur = s->alphamodes[n];

        if ((ret = av_alpha_mode_from_name(cur)) < 0 ||
            (ret = ff_add_format(&s->alpha_modes, ret)) < 0)
            return ret;
    }

    if (!strcmp(ctx->filter->name, "noformat")) {
        if ((ret = invert_formats(&s->formats,      ff_all_formats(AVMEDIA_TYPE_VIDEO))) < 0 ||
            (ret = invert_formats(&s->color_spaces, ff_all_color_spaces())) < 0 ||
            (ret = invert_formats(&s->color_ranges, ff_all_color_ranges())) < 0 ||
            (ret = invert_formats(&s->alpha_modes,  ff_all_alpha_modes())) < 0)
            return ret;
    }

    /* hold on to a ref for the lifetime of the filter */
    if (s->formats      && (ret = ff_formats_ref(s->formats,      &s->formats)) < 0 ||
        s->color_spaces && (ret = ff_formats_ref(s->color_spaces, &s->color_spaces)) < 0 ||
        s->color_ranges && (ret = ff_formats_ref(s->color_ranges, &s->color_ranges)) < 0 ||
        s->alpha_modes  && (ret = ff_formats_ref(s->alpha_modes,  &s->alpha_modes)) < 0)
        return ret;

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const FormatContext *s = ctx->priv;
    int ret;

    if (s->formats      && (ret = ff_set_common_formats2     (ctx, cfg_in, cfg_out, s->formats)) < 0 ||
        s->color_spaces && (ret = ff_set_common_color_spaces2(ctx, cfg_in, cfg_out, s->color_spaces)) < 0 ||
        s->color_ranges && (ret = ff_set_common_color_ranges2(ctx, cfg_in, cfg_out, s->color_ranges)) < 0 ||
        s->alpha_modes  && (ret = ff_set_common_alpha_modes2(ctx, cfg_in, cfg_out, s->alpha_modes)) < 0)
        return ret;

    return 0;
}


#define OFFSET(x) offsetof(FormatContext, x)
#define AR AV_OPT_TYPE_FLAG_ARRAY
static const AVOptionArrayDef def_array = {.def=NULL,.size_min=0,.sep='|'};
static const AVOption options[] = {
    { "pixel_formats", "set the list of pixel formats", OFFSET(pix_fmts), AV_OPT_TYPE_PIXEL_FMT|AR, {.arr=&def_array}, .min = 0, .max = INT_MAX, .flags = AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { "color_spaces", "set the list of color spaces", OFFSET(csps), AV_OPT_TYPE_STRING|AR, {.arr=&def_array}, .flags = AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { "color_ranges", "set the list of color ranges", OFFSET(ranges), AV_OPT_TYPE_STRING|AR, {.arr=&def_array}, .flags = AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { "alpha_modes", "set the list of alpha modes", OFFSET(alphamodes), AV_OPT_TYPE_STRING|AR, {.arr=&def_array}, .flags = AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM },
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(format, "(no)format", options);

static const AVFilterPad inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .get_buffer.video = ff_null_get_video_buffer,
    },
};

#if CONFIG_FORMAT_FILTER
const FFFilter ff_vf_format = {
    .p.name        = "format",
    .p.description = NULL_IF_CONFIG_SMALL("Convert the input video to one of the specified pixel formats."),
    .p.priv_class  = &format_class,

    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,

    .init          = init,
    .uninit        = uninit,

    .priv_size     = sizeof(FormatContext),

    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),

    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_FORMAT_FILTER */

#if CONFIG_NOFORMAT_FILTER
const FFFilter ff_vf_noformat = {
    .p.name        = "noformat",
    .p.description = NULL_IF_CONFIG_SMALL("Force libavfilter not to use any of the specified pixel formats for the input to the next filter."),
    .p.priv_class  = &format_class,

    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,

    .init          = init,
    .uninit        = uninit,

    .priv_size     = sizeof(FormatContext),

    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),

    FILTER_QUERY_FUNC2(query_formats),
};
#endif /* CONFIG_NOFORMAT_FILTER */
