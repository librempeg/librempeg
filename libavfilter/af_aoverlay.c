/*
 * Copyright (c) 2023 Harshit Karwal
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AOverlayContext {
    const AVClass *class;
    AVFrame *main_frame;
    int64_t pts;
    int overlay_eof;
    int done_fade;
    int is_disabled;
    int mode;

    int64_t cf_duration;
    int64_t cf_samples;

    void (*crossfade_samples)(uint8_t **dst, uint8_t * const *cf0,
                              uint8_t * const *cf1,
                              int nb_samples, int channels);
} AOverlayContext;

static const enum AVSampleFormat sample_fmts[] = {
    AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_FLTP,
    AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P,
    AV_SAMPLE_FMT_NONE
};

#define OFFSET(x) offsetof(AOverlayContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption aoverlay_options[] = {
    { "duration", "set duration for cross fade between the inputs", OFFSET(cf_duration), AV_OPT_TYPE_DURATION, {.i64 = 100000}, 0, 6000000, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aoverlay);

#define CROSSFADE_PLANAR(name, type)                                                 \
static void crossfade_samples_## name ##p(uint8_t **dst, uint8_t * const *cf0,       \
                                          uint8_t * const *cf1,                      \
                                          int nb_samples, int channels)              \
{                                                                                    \
    for (int i = 0; i < nb_samples; i++) {                                           \
        double main_gain = av_clipd(1.0 * (nb_samples - 1 - i) / nb_samples, 0, 1.); \
        double overlay_gain = av_clipd(1.0 * i / nb_samples, 0, 1.);                 \
        for (int c = 0; c < channels; c++) {                                         \
            type *d = (type *)dst[c];                                                \
            const type *s0 = (type *)cf0[c];                                         \
            const type *s1 = (type *)cf1[c];                                         \
                                                                                     \
            d[i] = s0[i] * main_gain + s1[i] * overlay_gain;                         \
        }                                                                            \
    }                                                                                \
}

CROSSFADE_PLANAR(dbl, double)
CROSSFADE_PLANAR(flt, float)
CROSSFADE_PLANAR(s16, int16_t)
CROSSFADE_PLANAR(s32, int32_t)

static av_cold void uninit(AVFilterContext *ctx)
{
    AOverlayContext *s = ctx->priv;

    av_frame_free(&s->main_frame);
}

static int crossfade_frame(AOverlayContext *s, AVFrame *main_frame, AVFrame *overlay_frame,
                           AVFilterLink *outlink, AVFrame **out)
{
    *out = ff_get_audio_buffer(outlink, main_frame->nb_samples);
    if (!(*out))
        return AVERROR(ENOMEM);
    av_frame_copy_props(*out, main_frame);

    if (s->mode > 0)
        FFSWAP(AVFrame *, main_frame, overlay_frame);

    s->crossfade_samples((*out)->extended_data,
                         main_frame->extended_data,
                         overlay_frame->extended_data,
                         main_frame->nb_samples,
                         (*out)->ch_layout.nb_channels);

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    const int filter_is_disabled = ff_filter_disabled(ctx);
    AOverlayContext *s = ctx->priv;
    AVFilterLink *main_inlink = ctx->inputs[0];
    AVFilterLink *overlay_inlink = ctx->inputs[1];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *overlay_frame = NULL;
    int status, ret;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (ff_filter_link(main_inlink)->frame_count_out > 0) {
        if (s->is_disabled < 0) {
            s->is_disabled = filter_is_disabled;
            s->done_fade = 1;
        }

        if (s->is_disabled != filter_is_disabled) {
            s->mode = filter_is_disabled - s->is_disabled;
            s->is_disabled = filter_is_disabled;
            s->done_fade = 0;
        }
    }

    if (!s->main_frame) {
        if (s->cf_samples > 0)
            ret = ff_inlink_consume_samples(main_inlink, s->cf_samples, s->cf_samples, &s->main_frame);
        else
            ret = ff_inlink_consume_frame(main_inlink, &s->main_frame);
        if (ret < 0)
            return ret;
    } else if (!s->overlay_eof) {
        ret = ff_inlink_consume_samples(overlay_inlink, s->main_frame->nb_samples,
                                        s->main_frame->nb_samples, &overlay_frame);
        if (ret < 0)
            return ret;
    }

    if (s->main_frame && (overlay_frame || s->overlay_eof)) {
        if (filter_is_disabled) {
            if (s->done_fade) {
                av_frame_free(&overlay_frame);
                ret = ff_filter_frame(outlink, s->main_frame);
                s->main_frame = NULL;
            } else if (overlay_frame) {
                AVFrame *out_frame = NULL;

                ret = crossfade_frame(s, s->main_frame, overlay_frame, outlink, &out_frame);
                s->done_fade = 1;

                av_frame_free(&s->main_frame);
                av_frame_free(&overlay_frame);
                ret = ff_filter_frame(outlink, out_frame);
            }

            return ret;
        } else if (overlay_frame && !s->done_fade) {
            AVFrame *out_frame = NULL;

            ret = crossfade_frame(s, s->main_frame, overlay_frame, outlink, &out_frame);
            s->done_fade = 1;

            av_frame_free(&s->main_frame);
            av_frame_free(&overlay_frame);
            return ff_filter_frame(outlink, out_frame);
        } else if (overlay_frame) {
            av_frame_free(&s->main_frame);
            return ff_filter_frame(outlink, overlay_frame);
        } else {
            ret = ff_filter_frame(outlink, s->main_frame);
            s->main_frame = NULL;
            return ret;
        }
    }

    if (ff_inlink_acknowledge_status(overlay_inlink, &status, &pts)) {
        s->overlay_eof = 1;
    }

    if (ff_inlink_acknowledge_status(main_inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink)) {
        if (!s->main_frame)
            ff_inlink_request_frame(main_inlink);
        if (!s->overlay_eof)
            ff_inlink_request_frame(overlay_inlink);
        return 0;
    }

    return FFERROR_NOT_READY;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AOverlayContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP: s->crossfade_samples = crossfade_samples_dblp; break;
    case AV_SAMPLE_FMT_FLTP: s->crossfade_samples = crossfade_samples_fltp; break;
    case AV_SAMPLE_FMT_S16P: s->crossfade_samples = crossfade_samples_s16p; break;
    case AV_SAMPLE_FMT_S32P: s->crossfade_samples = crossfade_samples_s32p; break;
    default: return AVERROR_BUG;
    }

    if (s->cf_duration)
        s->cf_samples = av_rescale(s->cf_duration, outlink->sample_rate, AV_TIME_BASE);

    s->is_disabled = -1;

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name = "main",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "overlay",
        .type = AVMEDIA_TYPE_AUDIO,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .config_props   = config_output,
    },
};

const FFFilter ff_af_aoverlay = {
    .p.name         = "aoverlay",
    .p.description  = NULL_IF_CONFIG_SMALL("Replace a specified section of an audio stream with another audio input."),
    .p.priv_class   = &aoverlay_class,
    .priv_size      = sizeof(AOverlayContext),
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
