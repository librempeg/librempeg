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

#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioCL2CLContext {
    const AVClass *class;

    AVChannelLayout ch_layout;
    AVChannelLayout default_in_layout;
    int pass;
    int in_planar, out_planar;

    int (*do_cl2cl)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioCL2CLContext;

#define OFFSET(x) offsetof(AudioCL2CLContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption acl2cl_options[] = {
    { "channel_layout", "set the channel layout", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="none"}, 0, 0, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(acl2cl);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioCL2CLContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_U8,  AV_SAMPLE_FMT_U8P,
        AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_S64, AV_SAMPLE_FMT_S64P,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    AVFilterChannelLayouts *layouts;
    AVFilterFormats *formats;
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(sample_fmts);
    if (formats)
        formats->flags = FILTER_SAME_BITDEPTH;
    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    layouts = ff_all_channel_counts();
    if (!layouts)
        return AVERROR(ENOMEM);

    if ((ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts)) < 0)
        return ret;

    if (s->ch_layout.nb_channels > 0) {
        layouts = NULL;

        ret = ff_add_channel_layout(&layouts, &s->ch_layout);
        if (ret)
            return ret;

        return ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
    }

    layouts = ff_all_channel_counts();
    if (!layouts)
        return AVERROR(ENOMEM);

    return ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 8
#include "acl2cl_template.c"

#undef DEPTH
#define DEPTH 16
#include "acl2cl_template.c"

#undef DEPTH
#define DEPTH 31
#include "acl2cl_template.c"

#undef DEPTH
#define DEPTH 32
#include "acl2cl_template.c"

#undef DEPTH
#define DEPTH 63
#include "acl2cl_template.c"

#undef DEPTH
#define DEPTH 64
#include "acl2cl_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioCL2CLContext *s = ctx->priv;

    av_channel_layout_default(&s->default_in_layout, inlink->ch_layout.nb_channels);

    s->out_planar = av_sample_fmt_is_planar(outlink->format);
    s->in_planar = av_sample_fmt_is_planar(inlink->format);

    if (!av_channel_layout_compare(&outlink->ch_layout,
                                   &inlink->ch_layout)) {
        s->pass = 1;
        return 0;
    }

    switch (outlink->format) {
    case AV_SAMPLE_FMT_U8:
    case AV_SAMPLE_FMT_U8P:
        s->do_cl2cl = do_cl2cl_u8p;
        break;
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S16P:
        s->do_cl2cl = do_cl2cl_s16p;
        break;
    case AV_SAMPLE_FMT_S32:
    case AV_SAMPLE_FMT_S32P:
        s->do_cl2cl = do_cl2cl_s32p;
        break;
    case AV_SAMPLE_FMT_S64:
    case AV_SAMPLE_FMT_S64P:
        s->do_cl2cl = do_cl2cl_s64p;
        break;
    case AV_SAMPLE_FMT_FLT:
    case AV_SAMPLE_FMT_FLTP:
        s->do_cl2cl = do_cl2cl_fltp;
        break;
    case AV_SAMPLE_FMT_DBL:
    case AV_SAMPLE_FMT_DBLP:
        s->do_cl2cl = do_cl2cl_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioCL2CLContext *s = ctx->priv;
    AVFrame *out;

    if (s->pass) {
        out = in;
    } else {
        ThreadData td;

        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        td.in = in;
        td.out = out;

        ff_filter_execute(ctx, s->do_cl2cl, &td, NULL,
                          FFMIN(outlink->ch_layout.nb_channels,
                                ff_filter_get_nb_threads(ctx)));

        av_frame_copy_props(out, in);
        ff_graph_frame_free(ctx, &in);
    }

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static AVFrame *get_in_audio_buffer(AVFilterLink *inlink, int nb_samples)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCL2CLContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_audio_buffer   (inlink, nb_samples) :
        ff_default_get_audio_buffer(inlink, nb_samples);
}

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_input,
        .get_buffer.audio = get_in_audio_buffer,
    },
};

const FFFilter ff_af_acl2cl = {
    .p.name        = "acl2cl",
    .p.description = NULL_IF_CONFIG_SMALL("Switch audio channel layout."),
    .p.priv_class  = &acl2cl_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(AudioCL2CLContext),
    .activate      = activate,
    FILTER_QUERY_FUNC2(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
};
