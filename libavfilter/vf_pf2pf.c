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

#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "avfilter.h"
#include "video.h"
#include "filters.h"
#include "formats.h"

static const int8_t step_offset_tab[][2] = {
    {  1, 0 }, {  2, -1 },
    {  2, 0 }, {  2,  1 },
    {  3, 0 }, {  3,  1 },
    {  3, 2 }, {  4,  0 },
    {  4, 1 }, {  4,  2 },
    {  4, 3 }, {  6,  0 },
    {  6, 2 }, {  6,  3 },
    {  6, 4 }, {  8,  0 },
    {  8, 2 }, {  8,  4 },
    {  8, 6 }, { 12,  0 },
    { 12, 4 }, { 12,  8 },
    { 16, 0 }, { 16,  4 },
    { 16, 8 }, { 16, 12 },
};

typedef void (*pf2pf_generic_fun)(uint8_t **dstp,
                                  const uint8_t **srcp,
                                  const int *dst_linesizep,
                                  const int *src_linesizep,
                                  const int w, const int h,
                                  const int dst_plane,
                                  const int dst_step,
                                  const int dst_shift,
                                  const int dst_offset,
                                  const int dst_depth,
                                  const int src_plane,
                                  const int src_step,
                                  const int src_shift,
                                  const int src_offset,
                                  const int src_depth);

typedef void (*pf2pf_special_fun)(uint8_t **dstp,
                                  const uint8_t **srcp,
                                  const int *dst_linesizep,
                                  const int *src_linesizep,
                                  const int w, const int h,
                                  const int dst_plane,
                                  const int dst_shift,
                                  const int dst_depth,
                                  const int src_plane,
                                  const int src_shift,
                                  const int src_depth);

typedef struct PF2PFContext {
    const AVClass *class;

    const AVPixFmtDescriptor *dst_desc, *src_desc;

    int linesize[4];
    int format;
    int pass;

    void (*special[4])(uint8_t **dstp,
                       const uint8_t **srcp,
                       const int *dst_linesizep,
                       const int *src_linesizep,
                       const int w, const int h,
                       const int dst_plane,
                       const int dst_shift,
                       const int dst_depth,
                       const int src_plane,
                       const int src_shift,
                       const int src_depth);

    void (*generic[4])(uint8_t **dstp,
                       const uint8_t **srcp,
                       const int *dst_linesizep,
                       const int *src_linesizep,
                       const int w, const int h,
                       const int dst_plane,
                       const int dst_step,
                       const int dst_shift,
                       const int dst_offset,
                       const int dst_depth,
                       const int src_plane,
                       const int src_step,
                       const int src_shift,
                       const int src_offset,
                       const int src_depth);
} PF2PFContext;

#define OFFSET(x) offsetof(PF2PFContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption pf2pf_options[] = {
    { "format", "set the pixel format", OFFSET(format), AV_OPT_TYPE_PIXEL_FMT, {.i64=AV_PIX_FMT_NONE}, AV_PIX_FMT_NONE, AV_PIX_FMT_NB-1, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(pf2pf);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const PF2PFContext *s = ctx->priv;
    AVFilterFormats *formats;
    int ret;

    if (s->format != AV_PIX_FMT_NONE) {
        const AVPixFmtDescriptor *src_desc;
        const AVPixFmtDescriptor *desc;
        int fmt;

        formats = NULL;
        src_desc = av_pix_fmt_desc_get(s->format);
        for (fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
            if (!(desc->flags & AV_PIX_FMT_FLAG_PAL ||
                  desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
                  desc->flags & AV_PIX_FMT_FLAG_BITSTREAM ||
                  (src_desc->nb_components > 1 && desc->log2_chroma_w != src_desc->log2_chroma_w) ||
                  (src_desc->nb_components > 1 && desc->log2_chroma_h != src_desc->log2_chroma_h) ||
                  (desc->flags & AV_PIX_FMT_FLAG_RGB) != (src_desc->flags & AV_PIX_FMT_FLAG_RGB)) &&
                (ret = ff_add_format(&formats, fmt)) < 0)
                return ret;
        }
    } else {
        const AVPixFmtDescriptor *desc;

        formats = NULL;
        for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
            if (!(desc->flags & AV_PIX_FMT_FLAG_PAL ||
                  desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
                  desc->flags & AV_PIX_FMT_FLAG_BITSTREAM) &&
                (ret = ff_add_format(&formats, fmt)) < 0)
                return ret;
        }
    }

    formats->flags = FILTER_SAME_RGB_FLAG | FILTER_SAME_SUBSAMPLING;
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    if (s->format != AV_PIX_FMT_NONE) {
        formats = NULL;

        ret = ff_add_format(&formats, s->format);
        if (ret)
            return ret;

        formats->flags = FILTER_SAME_RGB_FLAG | FILTER_SAME_SUBSAMPLING;
        return ff_formats_ref(formats, &cfg_out[0]->formats);
    }

    {
        const AVPixFmtDescriptor *desc;

        formats = NULL;
        for (int fmt = 0; desc = av_pix_fmt_desc_get(fmt); fmt++) {
            if (!(desc->flags & AV_PIX_FMT_FLAG_PAL ||
                  desc->flags & AV_PIX_FMT_FLAG_HWACCEL ||
                  desc->flags & AV_PIX_FMT_FLAG_BITSTREAM) &&
                (ret = ff_add_format(&formats, fmt)) < 0)
                return ret;
        }

        formats->flags = FILTER_SAME_RGB_FLAG | FILTER_SAME_SUBSAMPLING;
    }

    return ff_formats_ref(formats, &cfg_out[0]->formats);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DST_DEPTH 8
#include "pf2pf_dst_endian_template.c"

#undef DST_DEPTH
#define DST_DEPTH 16
#include "pf2pf_dst_endian_template.c"

#undef DST_DEPTH
#define DST_DEPTH 32
#include "pf2pf_dst_endian_template.c"

static pf2pf_special_fun special[2][2][26][2][2][26] = {
#undef DST_DEPTH
#define DST_DEPTH 8
#include "pf2pf_dst_endian_entry.c"

#undef DST_DEPTH
#define DST_DEPTH 16
#include "pf2pf_dst_endian_entry.c"
};

static pf2pf_generic_fun generic[4][2][4][2] = {
#undef DST_DEPTH
#define DST_DEPTH 8
#include "pf2pf_dst_endian_entry_generic.c"

#undef DST_DEPTH
#define DST_DEPTH 16
#include "pf2pf_dst_endian_entry_generic.c"

#undef DST_DEPTH
#define DST_DEPTH 32
#include "pf2pf_dst_endian_entry_generic.c"
};

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    PF2PFContext *s = ctx->priv;
    int ret;

    if (outlink->format == inlink->format) {
        s->pass = 1;
        return 0;
    }

    s->dst_desc = av_pix_fmt_desc_get(outlink->format);
    s->src_desc = av_pix_fmt_desc_get(inlink->format);

    for (int i = 0; i < s->dst_desc->nb_components; i++) {
        unsigned dst_be = !!(s->dst_desc->flags & AV_PIX_FMT_FLAG_BE);
        unsigned src_be = !!(s->src_desc->flags & AV_PIX_FMT_FLAG_BE);
        unsigned dst_by = ((s->dst_desc->comp[i].depth + 7) / 8) - 1;
        unsigned src_by = ((s->src_desc->comp[i].depth + 7) / 8) - 1;
        unsigned dst_so = UINT_MAX, src_so = UINT_MAX;

        for (int j = 0; j < FF_ARRAY_ELEMS(step_offset_tab); j++) {
            if (step_offset_tab[j][0] == s->dst_desc->comp[i].step &&
                step_offset_tab[j][1] == s->dst_desc->comp[i].offset) {
                dst_so = j;
                break;
            }
        }

        for (int j = 0; j < FF_ARRAY_ELEMS(step_offset_tab); j++) {
            if (step_offset_tab[j][0] == s->src_desc->comp[i].step &&
                step_offset_tab[j][1] == s->src_desc->comp[i].offset) {
                src_so = j;
                break;
            }
        }

        if (src_by > 3)
            continue;

        s->generic[i] = generic[src_by][src_be][dst_by][dst_be];

        if (src_so == UINT_MAX || dst_so == UINT_MAX)
            continue;

        if (src_by > 1)
            continue;

        s->special[i] = special[src_by][src_be][src_so][dst_by][dst_be][dst_so];
    }

    if ((ret = av_image_fill_linesizes(s->linesize, outlink->format, inlink->w)) < 0)
        return ret;

    return 0;
}

static int do_pf2pf(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    PF2PFContext *s = ctx->priv;
    const int nb_components = s->dst_desc->nb_components;
    const int *linesize = s->linesize;
    ThreadData *td = arg;
    AVFrame *restrict out = td->out;
    AVFrame *restrict in = td->in;
    const int w = in->width;
    const int h = in->height;
    const int start = (h * jobnr) / nb_jobs;
    const int end = (h * (jobnr+1)) / nb_jobs;

    for (int comp = 0; comp < nb_components; comp++) {
        if (out->data[comp]) {
            const int dst_sh = (comp > 0) ? s->dst_desc->log2_chroma_h : 0;
            const int cstart = start >> dst_sh;
            const int cend = end >> dst_sh;
            uint8_t *dst_data = out->data[comp] + cstart * out->linesize[comp];

            for (int y = cstart; y < cend; y++) {
                memset(dst_data, 0, linesize[comp]);
                dst_data += out->linesize[comp];
            }
        }
    }

    for (int comp = 0; comp < nb_components; comp++) {
        const int dst_plane = s->dst_desc->comp[comp].plane;
        const int dst_step  = s->dst_desc->comp[comp].step;
        const int dst_shift = s->dst_desc->comp[comp].shift;
        const int dst_offset= s->dst_desc->comp[comp].offset;
        const int dst_depth = s->dst_desc->comp[comp].depth;
        const int src_plane = s->src_desc->comp[comp].plane;
        const int src_step  = s->src_desc->comp[comp].step;
        const int src_shift = s->src_desc->comp[comp].shift;
        const int src_offset= s->src_desc->comp[comp].offset;
        const int src_depth = s->src_desc->comp[comp].depth;
        const int dst_sw = (comp > 0) ? s->dst_desc->log2_chroma_w : 0;
        const int dst_sh = (comp > 0) ? s->dst_desc->log2_chroma_h : 0;
        uint8_t *dst_data[4] = {NULL}, *src_data[4] = {NULL};
        const int cstart = start >> dst_sh;
        const int cend = end >> dst_sh;
        const int cw = w >> dst_sw;

        for (int i = 0; i < 4; i++) {
            if (out->data[i])
                dst_data[i] = out->data[i] + cstart * out->linesize[i];
            if (in->data[i])
                src_data[i] = in->data[i] + cstart * in->linesize[i];
        }

        if (s->special[comp]) {
            s->special[comp]((uint8_t **)dst_data, (const uint8_t **)src_data, out->linesize, in->linesize,
                             cw, cend - cstart,
                             dst_plane, dst_shift, dst_depth,
                             src_plane, src_shift, src_depth);
        } else if (s->generic[comp]) {
            s->generic[comp]((uint8_t **)dst_data, (const uint8_t **)src_data, out->linesize, in->linesize,
                             cw, cend - cstart,
                             dst_plane, dst_step, dst_shift, dst_offset, dst_depth,
                             src_plane, src_step, src_shift, src_offset, src_depth);
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    PF2PFContext *s = ctx->priv;
    AVFrame *out;

    if (s->pass) {
        out = in;
    } else {
        ThreadData td;
        int ret, nb_jobs;

        out = ff_graph_frame_alloc(ctx);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        ret = ff_filter_get_buffer(ctx, out);
        if (ret < 0) {
            av_frame_free(&out);
            av_frame_free(&in);
            return ret;
        }

        td.in = in;
        td.out = out;

        nb_jobs = in->height;

        ff_filter_execute(ctx, do_pf2pf, &td, NULL,
                          FFMIN(nb_jobs, ff_filter_get_nb_threads(ctx)));

        av_frame_copy_props(out, in);
        ff_graph_frame_free(ctx, &in);
    }

    return ff_filter_frame(outlink, out);
}

static AVFrame *get_in_video_buffer(AVFilterLink *inlink, int w, int h)
{
    AVFilterContext *ctx = inlink->dst;
    PF2PFContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_video_buffer   (inlink, w, h) :
        ff_default_get_video_buffer(inlink, w, h);
}

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
        .get_buffer.video = get_in_video_buffer,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_pf2pf = {
    .p.name        = "pf2pf",
    .p.description = NULL_IF_CONFIG_SMALL("Switch video pixel format."),
    .p.priv_class  = &pf2pf_class,
    .priv_size     = sizeof(PF2PFContext),
    FILTER_QUERY_FUNC2(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_FRAME_THREADS,
};
