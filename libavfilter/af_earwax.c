/*
 * Copyright (c) 2011 Mina Nagy Zaki
 * Copyright (c) 2000 Edward Beingessner And Sundry Contributors.
 * This source code is freely redistributable and may be used for any purpose.
 * This copyright notice must be maintained.  Edward Beingessner And Sundry
 * Contributors are not responsible for the consequences of using this
 * software.
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
 * Stereo Widening Effect. Adds audio cues to move stereo image in
 * front of the listener. Adapted from the libsox earwax effect.
 */

#include "libavutil/channel_layout.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

#define NUMTAPS 32

static const int8_t filt[NUMTAPS * 2] = {
/* 30°  330° */
    4,   -6,     /* 32 tap stereo FIR filter. */
    4,  -11,     /* One side filters as if the */
   -1,   -5,     /* signal was from 30 degrees */
    3,    3,     /* from the ear, the other as */
   -2,    5,     /* if 330 degrees. */
   -5,    0,
    9,    1,
    6,    3,     /*                         Input                         */
   -4,   -1,     /*                   Left         Right                  */
   -5,   -3,     /*                __________   __________                */
   -2,   -5,     /*               |          | |          |               */
   -7,    1,     /*           .---|  Hh,0(f) | |  Hh,0(f) |---.           */
    6,   -7,     /*          /    |__________| |__________|    \          */
   30,  -29,     /*         /                \ /                \         */
   12,   -3,     /*        /                  X                  \        */
  -11,    4,     /*       /                  / \                  \       */
   -3,    7,     /*  ____V_____   __________V   V__________   _____V____  */
  -20,   23,     /* |          | |          |   |          | |          | */
    2,    0,     /* | Hh,30(f) | | Hh,330(f)|   | Hh,330(f)| | Hh,30(f) | */
    1,   -6,     /* |__________| |__________|   |__________| |__________| */
  -14,   -5,     /*      \     ___      /           \      ___     /      */
   15,  -18,     /*       \   /   \    /    _____    \    /   \   /       */
    6,    7,     /*        `->| + |<--'    /     \    `-->| + |<-'        */
   15,  -10,     /*           \___/      _/       \_      \___/           */
  -14,   22,     /*               \     / \       / \     /               */
   -7,   -2,     /*                `--->| |       | |<---'                */
   -4,    9,     /*                     \_/       \_/                     */
    6,  -12,     /*                                                       */
    6,   -6,     /*                       Headphones                      */
    0,  -11,
    0,   -5,
    4,    0};

typedef struct EarwaxContext {
    int16_t filter[2][NUMTAPS];
    int16_t taps[4][NUMTAPS * 2];

    AVFrame *frame[2];
} EarwaxContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_NONE,
    };
    static const AVChannelLayout layouts[] = {
        AV_CHANNEL_LAYOUT_STEREO,
        { .nb_channels = 0 },
    };
    static const int sample_rates[] = { 44100, -1 };

    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    ret = ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
    if (ret < 0)
        return ret;

    return 0;
}

//FIXME: replace with DSPContext.scalarproduct_int16
static inline int16_t *scalarproduct(const int16_t *in, const int16_t *endin,
                                     const int16_t *filter, int16_t *out)
{
    int32_t sample;
    int16_t j;

    while (in < endin) {
        sample = 0;
        for (j = 0; j < NUMTAPS; j++)
            sample += in[j] * filter[j];
        *out = av_clip_int16(sample >> 7);
        out++;
        in++;
    }

    return out;
}

static int config_input(AVFilterLink *inlink)
{
    EarwaxContext *s = inlink->dst->priv;

    for (int i = 0; i < NUMTAPS; i++) {
        s->filter[0][i] = filt[i * 2];
        s->filter[1][i] = filt[i * 2 + 1];
    }

    return 0;
}

static void convolve(AVFilterContext *ctx, AVFrame *in,
                     int input_ch, int output_ch,
                     int filter_ch, int tap_ch)
{
    EarwaxContext *s = ctx->priv;
    int16_t *taps, *endin, *dst, *src;
    int len;

    taps  = s->taps[tap_ch];
    dst   = (int16_t *)s->frame[input_ch]->data[output_ch];
    src   = (int16_t *)in->data[input_ch];

    len = FFMIN(NUMTAPS, in->nb_samples);
    // copy part of new input and process with saved input
    memcpy(taps+NUMTAPS, src, len * sizeof(*taps));
    dst = scalarproduct(taps, taps + len, s->filter[filter_ch], dst);

    // process current input
    if (in->nb_samples >= NUMTAPS) {
        endin = src + in->nb_samples - NUMTAPS;
        scalarproduct(src, endin, s->filter[filter_ch], dst);

        // save part of input for next round
        memcpy(taps, endin, NUMTAPS * sizeof(*taps));
    } else {
        memmove(taps, taps + in->nb_samples, NUMTAPS * sizeof(*taps));
    }
}

static void mix(AVFilterContext *ctx, AVFrame *out, AVFrame *in,
                int output_ch, int f0, int f1, int i0, int i1)
{
    EarwaxContext *s = ctx->priv;
    const int16_t *srcl = (const int16_t *)s->frame[f0]->data[i0];
    const int16_t *srcr = (const int16_t *)s->frame[f1]->data[i1];
    const int16_t *src = (const int16_t *)in->data[output_ch];
    int16_t *dst = (int16_t *)out->data[output_ch];
    const int nb_samples = out->nb_samples;

    if (ff_filter_disabled(ctx)) {
        memcpy(dst, src, nb_samples * sizeof(*dst));
        return;
    }

    for (int n = 0; n < nb_samples; n++)
        dst[n] = av_clip_int16(srcl[n] + srcr[n]);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    EarwaxContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out = ff_get_audio_buffer(outlink, in->nb_samples);

    for (int ch = 0; ch < 2; ch++) {
        if (!s->frame[ch] || s->frame[ch]->nb_samples < in->nb_samples) {
            av_frame_free(&s->frame[ch]);
            s->frame[ch] = ff_get_audio_buffer(outlink, in->nb_samples);
            if (!s->frame[ch]) {
                av_frame_free(&in);
                av_frame_free(&out);
                return AVERROR(ENOMEM);
            }
        }
    }

    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    convolve(ctx, in, 0, 0, 0, 0);
    convolve(ctx, in, 0, 1, 1, 1);
    convolve(ctx, in, 1, 0, 0, 2);
    convolve(ctx, in, 1, 1, 1, 3);

    mix(ctx, out, in, 0, 0, 1, 1, 0);
    mix(ctx, out, in, 1, 0, 1, 0, 1);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    EarwaxContext *s = ctx->priv;

    av_frame_free(&s->frame[0]);
    av_frame_free(&s->frame[1]);
}

static const AVFilterPad earwax_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_earwax = {
    .p.name         = "earwax",
    .p.description  = NULL_IF_CONFIG_SMALL("Widen the stereo image."),
    .priv_size      = sizeof(EarwaxContext),
    .uninit         = uninit,
    FILTER_INPUTS(earwax_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
