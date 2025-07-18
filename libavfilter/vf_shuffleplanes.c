/*
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

#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

typedef struct ShufflePlanesContext {
    const AVClass *class;

    /* number of planes in the selected pixel format */
    int planes;

    /* mapping indices */
    int map[4];

    /* set to 1 if some plane is used more than once, so we need to make a copy */
    int copy;
} ShufflePlanesContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    AVFilterFormats *formats = NULL;
    const ShufflePlanesContext *s = ctx->priv;
    int fmt, ret, i;

    for (fmt = 0; av_pix_fmt_desc_get(fmt); fmt++) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(fmt);
        int planes = av_pix_fmt_count_planes(fmt);

        if (!(desc->flags & AV_PIX_FMT_FLAG_PAL) &&
            !(desc->flags & AV_PIX_FMT_FLAG_HWACCEL)) {
            for (i = 0; i < 4; i++) {
                if (s->map[i] >= planes)
                    break;

                if ((desc->log2_chroma_h || desc->log2_chroma_w) &&
                    (i == 1 || i == 2) != (s->map[i] == 1 || s->map[i] == 2))
                    break;
            }

            if (i != 4)
                continue;
            if ((ret = ff_add_format(&formats, fmt)) < 0) {
                return ret;
            }
        }
    }

    return ff_set_common_formats2(ctx, cfg_in, cfg_out, formats);
}

static av_cold int shuffleplanes_config_input(AVFilterLink *inlink)
{
    AVFilterContext    *ctx = inlink->dst;
    ShufflePlanesContext *s = ctx->priv;
    int used[4] = { 0 };
    int i;

    s->copy   = 0;
    s->planes = av_pix_fmt_count_planes(inlink->format);

    for (i = 0; i < s->planes; i++) {
        if (used[s->map[i]])
            s->copy = 1;
        used[s->map[i]]++;
    }

    return 0;
}

static int shuffleplanes_filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext          *ctx = inlink->dst;
    ShufflePlanesContext       *s = ctx->priv;
    uint8_t *shuffled_data[4]     = { NULL };
    int      shuffled_linesize[4] = { 0 };
    int i, ret;

    for (i = 0; i < s->planes; i++) {
        shuffled_data[i]     = frame->data[s->map[i]];
        shuffled_linesize[i] = frame->linesize[s->map[i]];
    }
    memcpy(frame->data,     shuffled_data,     sizeof(shuffled_data));
    memcpy(frame->linesize, shuffled_linesize, sizeof(shuffled_linesize));

    if (s->copy) {
        AVFrame *copy = ff_get_video_buffer(ctx->outputs[0], frame->width, frame->height);

        if (!copy) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        av_frame_copy(copy, frame);

        ret = av_frame_copy_props(copy, frame);
        if (ret < 0) {
            av_frame_free(&copy);
            goto fail;
        }

        av_frame_free(&frame);
        frame = copy;
    }

    return ff_filter_frame(ctx->outputs[0], frame);
fail:
    av_frame_free(&frame);
    return ret;
}

#define OFFSET(x) offsetof(ShufflePlanesContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption shuffleplanes_options[] = {
    { "map0", "Index of the input plane to be used as the first output plane ",  OFFSET(map[0]), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 3, FLAGS },
    { "map1", "Index of the input plane to be used as the second output plane ", OFFSET(map[1]), AV_OPT_TYPE_INT, { .i64 = 1 }, 0, 3, FLAGS },
    { "map2", "Index of the input plane to be used as the third output plane ",  OFFSET(map[2]), AV_OPT_TYPE_INT, { .i64 = 2 }, 0, 3, FLAGS },
    { "map3", "Index of the input plane to be used as the fourth output plane ", OFFSET(map[3]), AV_OPT_TYPE_INT, { .i64 = 3 }, 0, 3, FLAGS },
    { NULL },
};

AVFILTER_DEFINE_CLASS(shuffleplanes);

static const AVFilterPad shuffleplanes_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .config_props     = shuffleplanes_config_input,
        .filter_frame     = shuffleplanes_filter_frame,
    },
};

const FFFilter ff_vf_shuffleplanes = {
    .p.name        = "shuffleplanes",
    .p.description = NULL_IF_CONFIG_SMALL("Shuffle video planes."),
    .p.priv_class  = &shuffleplanes_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .priv_size    = sizeof(ShufflePlanesContext),
    FILTER_INPUTS(shuffleplanes_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
