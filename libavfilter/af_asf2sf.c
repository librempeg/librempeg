/*
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

#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioSF2SFContext {
    const AVClass *class;

    int format;
    int output_sample_bits;
    int pass;

    int (*do_sf2sf)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioSF2SFContext;

#define OFFSET(x) offsetof(AudioSF2SFContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption asf2sf_options[] = {
    { "format", "set the sample format", OFFSET(format), AV_OPT_TYPE_SAMPLE_FMT, {.i64=AV_SAMPLE_FMT_NONE}, AV_SAMPLE_FMT_NONE, AV_SAMPLE_FMT_NB-1, FLAGS },
    { "output_sample_bits", "set the number of output sample bits", OFFSET(output_sample_bits), AV_OPT_TYPE_INT, {.i64=0 }, 0, 64, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(asf2sf);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioSF2SFContext *s = ctx->priv;
    AVFilterFormats *formats;
    int ret;

    formats = ff_all_formats(AVMEDIA_TYPE_AUDIO);
    if (!formats)
        return AVERROR(ENOMEM);

    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    if (s->format != AV_SAMPLE_FMT_NONE) {
        formats = NULL;

        ret = ff_add_format(&formats, s->format);
        if (ret)
            return ret;

        return ff_formats_ref(formats, &cfg_out[0]->formats);
    }

    formats = ff_all_formats(AVMEDIA_TYPE_AUDIO);
    if (!formats)
        return AVERROR(ENOMEM);

    return ff_formats_ref(formats, &cfg_out[0]->formats);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DST_DEPTH 8
#include "asf2sf_template.c"

#undef DST_DEPTH
#define DST_DEPTH 16
#include "asf2sf_template.c"

#undef DST_DEPTH
#define DST_DEPTH 31
#include "asf2sf_template.c"

#undef DST_DEPTH
#define DST_DEPTH 32
#include "asf2sf_template.c"

#undef DST_DEPTH
#define DST_DEPTH 63
#include "asf2sf_template.c"

#undef DST_DEPTH
#define DST_DEPTH 64
#include "asf2sf_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioSF2SFContext *s = ctx->priv;

    if (outlink->format == inlink->format) {
        s->pass = 1;
        return 0;
    }

    switch (inlink->format) {
    case AV_SAMPLE_FMT_U8P:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_u8_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_u8_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_u8_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_u8_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_u8_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_u8_to_packed_dbl; break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_planar_u8_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_planar_u8_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_planar_u8_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_planar_u8_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_planar_u8_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S16P:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_s16_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_s16_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_s16_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_s16_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_s16_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_s16_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_planar_s16_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_planar_s16_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_planar_s16_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_planar_s16_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_planar_s16_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S32P:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_s32_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_s32_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_s32_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_s32_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_s32_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_s32_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_planar_s32_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_planar_s32_to_planar_s16; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_planar_s32_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_planar_s32_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_planar_s32_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_FLTP:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_flt_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_flt_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_flt_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_flt_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_flt_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_flt_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_planar_flt_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_planar_flt_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_planar_flt_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_planar_flt_to_planar_s64; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_planar_flt_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S64P:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_s64_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_s64_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_s64_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_s64_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_s64_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_s64_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_planar_s64_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_planar_s64_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_planar_s64_to_planar_s32; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_planar_s64_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_planar_s64_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_DBLP:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_planar_dbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_planar_dbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_planar_dbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_planar_dbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_planar_dbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_planar_dbl_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_planar_dbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_planar_dbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_planar_dbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_planar_dbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_planar_dbl_to_planar_flt; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_U8:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_packed_u8_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_packed_u8_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_packed_u8_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_packed_u8_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_packed_u8_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_u8_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_u8_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_u8_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_u8_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_u8_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_u8_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S16:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_packed_s16_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_packed_s16_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_packed_s16_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_packed_s16_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_packed_s16_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_s16_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_s16_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_s16_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_s16_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_s16_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_s16_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S32:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_packed_s32_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_packed_s32_to_packed_s16; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_packed_s32_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_packed_s32_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_packed_s32_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_s32_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_s32_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_s32_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_s32_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_s32_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_s32_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_FLT:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_packed_flt_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_packed_flt_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_packed_flt_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_packed_flt_to_packed_s64; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_packed_flt_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_flt_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_flt_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_flt_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_flt_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_flt_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_flt_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_DBL:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_packed_dbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_packed_dbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_packed_dbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  s->do_sf2sf = sf2sf_packed_dbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_packed_dbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_dbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_dbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_dbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_dbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_dbl_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_dbl_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S64:
        switch (outlink->format) {
        case AV_SAMPLE_FMT_U8:   s->do_sf2sf = sf2sf_packed_s64_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  s->do_sf2sf = sf2sf_packed_s64_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  s->do_sf2sf = sf2sf_packed_s64_to_packed_s32; break;
        case AV_SAMPLE_FMT_FLT:  s->do_sf2sf = sf2sf_packed_s64_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  s->do_sf2sf = sf2sf_packed_s64_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  s->do_sf2sf = sf2sf_packed_s64_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: s->do_sf2sf = sf2sf_packed_s64_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: s->do_sf2sf = sf2sf_packed_s64_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: s->do_sf2sf = sf2sf_packed_s64_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: s->do_sf2sf = sf2sf_packed_s64_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: s->do_sf2sf = sf2sf_packed_s64_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
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
    AudioSF2SFContext *s = ctx->priv;
    AVFrame *out;

    if (s->pass) {
        out = in;
    } else {
        int ret, nb_jobs;
        ThreadData td;

        out = av_frame_alloc();
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        out->nb_samples = in->nb_samples;
        ret = ff_filter_get_buffer(ctx, out);
        if (ret < 0) {
            av_frame_free(&out);
            av_frame_free(&in);
            return ret;
        }

        td.in = in;
        td.out = out;

        nb_jobs = (in->nb_samples + 8191) / 8192;

        ff_filter_execute(ctx, s->do_sf2sf, &td, NULL,
                          FFMIN(nb_jobs, ff_filter_get_nb_threads(ctx)));

        av_frame_copy_props(out, in);
        av_frame_free(&in);
    }

    return ff_filter_frame(outlink, out);
}

static AVFrame *get_in_audio_buffer(AVFilterLink *inlink, int nb_samples)
{
    AVFilterContext *ctx = inlink->dst;
    AudioSF2SFContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_audio_buffer   (inlink, nb_samples) :
        ff_default_get_audio_buffer(inlink, nb_samples);
}

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .filter_frame  = filter_frame,
        .get_buffer.audio = get_in_audio_buffer,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_asf2sf = {
    .p.name        = "asf2sf",
    .p.description = NULL_IF_CONFIG_SMALL("Switch audio sample format."),
    .p.priv_class  = &asf2sf_class,
    .priv_size     = sizeof(AudioSF2SFContext),
    FILTER_QUERY_FUNC2(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_FRAME_THREADS,
};
