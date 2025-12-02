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

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioQuantContext {
    const AVClass *class;

    int bits;
    int noise_shaper;
    int precision;
    int channels;

    AVFrame *in;

    void *state;

    int (*init_state)(AVFilterContext *ctx);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioQuantContext;

#define OFFSET(x) offsetof(AudioQuantContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aquant_options[] = {
    { "bitdepth",  "set output quantization precision", OFFSET(bits), AV_OPT_TYPE_INT,   {.i64=16},8,24,AT },
    { "noise-shaping", "enable noise-shaping",  OFFSET(noise_shaper), AV_OPT_TYPE_BOOL,  {.i64=1}, 0, 1, A },
    { "precision", "set input processing precision", OFFSET(precision),AV_OPT_TYPE_INT,  {.i64=0}, 0, 2, A, .unit = "precision" },
    {   "auto",  "set auto processing precision",                  0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, A, .unit = "precision" },
    {   "float", "set single-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, A, .unit = "precision" },
    {   "double","set double-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, A, .unit = "precision" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aquant);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioQuantContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[3][3] = {
        { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE },
        { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE },
        { AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE },
    };

    return ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out,
                                            sample_fmts[s->precision]);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioQuantContext *s = ctx->priv;
    AVFrame *out, *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;

    if (ret > 0) {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);

        s->in = in;
        ff_filter_execute(ctx, s->filter_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        ff_graph_frame_free(ctx, &in);

        return ff_filter_frame(outlink, out);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

#define DEPTH 32
#include "aquant_template.c"

#undef DEPTH
#define DEPTH 64
#include "aquant_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioQuantContext *s = ctx->priv;

    s->channels = outlink->ch_layout.nb_channels;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->init_state = init_state_double;
        s->filter_channels = s->noise_shaper ? shaper_channels_double : filter_channels_double;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->init_state = init_state_float;
        s->filter_channels = s->noise_shaper ? shaper_channels_float : filter_channels_float;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_state(ctx);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioQuantContext *s = ctx->priv;

    av_freep(&s->state);
}

static const AVFilterPad outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    }
};

const FFFilter ff_af_aquant = {
    .p.name         = "aquant",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply Quantization with optional Noise-Shaping to audio stream."),
    .p.priv_class   = &aquant_class,
    .priv_size      = sizeof(AudioQuantContext),
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
