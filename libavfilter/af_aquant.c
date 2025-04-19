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

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
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
    int got_input;

    AVFrame *frame[2];

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
    AudioQuantContext *s = ctx->priv;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[0], ctx);
    if (s->noise_shaper)
        FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[1], ctx);

    if (!s->frame[0]) {
        int ret = ff_inlink_consume_frame(ctx->inputs[0], &s->frame[0]);
        if (ret < 0)
            return ret;
    }

    if (s->noise_shaper && s->frame[0] && !s->frame[1]) {
        int ret = ff_inlink_consume_samples(ctx->inputs[1],
                                            s->frame[0]->nb_samples,
                                            s->frame[0]->nb_samples,
                                            &s->frame[1]);
        if (ret < 0)
            return ret;

        s->got_input = ret > 0;
    }

    if (!s->noise_shaper && s->frame[0]) {
        AVFrame *out;

        out = ff_get_audio_buffer(ctx->outputs[0], s->frame[0]->nb_samples);
        if (!out) {
            av_frame_free(&s->frame[0]);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, s->frame[0]);

        ff_filter_execute(ctx, s->filter_channels, out, NULL,
                          FFMIN(ctx->outputs[0]->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        av_frame_free(&s->frame[0]);

        return ff_filter_frame(ctx->outputs[0], out);
    } else if (s->got_input && s->frame[0] && s->frame[1]) {
        AVFrame *out[2];
        int ret;

        out[0] = ff_get_audio_buffer(ctx->outputs[0], s->frame[0]->nb_samples);
        out[1] = ff_get_audio_buffer(ctx->outputs[0], s->frame[0]->nb_samples);
        if (!out[0] || !out[1]) {
            av_frame_free(&out[0]);
            av_frame_free(&out[1]);
            av_frame_free(&s->frame[0]);
            av_frame_free(&s->frame[1]);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out[0], s->frame[0]);
        av_frame_copy_props(out[1], s->frame[1]);

        ff_filter_execute(ctx, s->filter_channels, out, NULL,
                          FFMIN(ctx->outputs[0]->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        av_frame_free(&s->frame[0]);
        av_frame_free(&s->frame[1]);

        ret = ff_filter_frame(ctx->outputs[0], out[0]);
        if (ret < 0) {
            av_frame_free(&out[1]);
            return ret;
        }
        return ff_filter_frame(ctx->outputs[1], out[1]);
    } else if (!s->got_input && s->frame[0] && !s->frame[1]) {
        AVFrame *out[2];
        int ret;

        out[0] = ff_get_audio_buffer(ctx->outputs[0], s->frame[0]->nb_samples);
        out[1] = ff_get_audio_buffer(ctx->outputs[0], s->frame[0]->nb_samples);
        if (!out[0] || !out[1]) {
            av_frame_free(&out[0]);
            av_frame_free(&out[1]);
            av_frame_free(&s->frame[0]);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out[0], s->frame[0]);

        ff_filter_execute(ctx, s->filter_channels, out, NULL,
                          FFMIN(ctx->outputs[0]->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        av_frame_free(&s->frame[0]);

        ret = ff_filter_frame(ctx->outputs[0], out[0]);
        if (ret < 0) {
            av_frame_free(&out[1]);
            return ret;
        }
        return ff_filter_frame(ctx->outputs[1], out[1]);
    }

    for (int i = 0; i < 1 + s->noise_shaper; i++) {
        int64_t pts;
        int status;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            ff_outlink_set_status(ctx->outputs[0], status, pts);
            if (s->noise_shaper)
                ff_outlink_set_status(ctx->outputs[1], status, pts);
            return 0;
        }
    }

    if (ff_outlink_frame_wanted(ctx->outputs[0])) {
        for (int i = 0; i < 1 + s->noise_shaper; i++) {
            if (s->frame[i])
                continue;
            ff_inlink_request_frame(ctx->inputs[i]);
            return 0;
        }
    }
    return 0;
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

    if (FF_OUTLINK_IDX(outlink))
        return 0;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = s->noise_shaper ? shaper_channels_double : filter_channels_double;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = s->noise_shaper ? shaper_channels_float : filter_channels_float;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioQuantContext *s = ctx->priv;

    if (s->noise_shaper) {
        AVFilterPad pad;
        int ret;

        pad = (AVFilterPad) {
            .name = "filtered_error",
            .type = AVMEDIA_TYPE_AUDIO,
        };

        ret = ff_append_inpad(ctx, &pad);
        if (ret < 0)
            return ret;

        pad = (AVFilterPad) {
            .name = "unfiltered_error",
            .type = AVMEDIA_TYPE_AUDIO,
            .config_props = config_output,
        };

        ret = ff_append_outpad(ctx, &pad);
        if (ret < 0)
            return ret;
    }

    return 0;
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
    .init           = init,
    .activate       = activate,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags        = AVFILTER_FLAG_DYNAMIC_OUTPUTS |
                      AVFILTER_FLAG_DYNAMIC_INPUTS |
                      AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
