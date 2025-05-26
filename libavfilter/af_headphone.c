/*
 * Copyright (C) 2017 Paul B Mahol
 * Copyright (C) 2013-2015 Andreas Fuchs, Wolfgang Hrauda
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

#include <math.h>

#include "libavutil/channel_layout.h"
#include "libavutil/cpu.h"
#include "libavutil/float_dsp.h"
#include "libavutil/intmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"

#define TIME_DOMAIN      0
#define FREQUENCY_DOMAIN 1

#define HRIR_STEREO 0
#define HRIR_MULTI  1

typedef struct HeadphoneContext {
    const AVClass *class;

    char **map;
    unsigned nb_maps;
    int type;

    int lfe_channel;

    int have_hrirs;
    int eof_hrirs;

    int ir_len;
    int air_len;
    int atx_len;

    int nb_hrir_inputs;

    int nb_irs;

    float gain;
    float lfe_gain, gain_lfe;

    void *ringbuffer[2];
    int write[2];

    int buffer_length;
    int n_tx;
    int size;
    int hrir_fmt;

    void *data_ir[2];
    void *temp_src[2];
    void *out_tx[2];
    void *in_tx[2];
    void *temp_afft[2];

    AVTXContext *tx_ctx[2], *itx_ctx[2];
    av_tx_fn tx_fn[2], itx_fn[2];
    void *data_hrtf[2];

    float  (*scalarproduct_flt)(const float  *v1, const float  *v2, int len);
    double (*scalarproduct_dbl)(const double *v1, const double *v2, size_t len);
    struct hrir_inputs {
        int          ir_len;
        int          eof;
    } hrir_in[64];
    AVChannelLayout map_channel_layout;
    enum AVChannel mapping[64];
    uint8_t  hrir_map[64];

    int (*convert_coeffs)(AVFilterContext *ctx, AVFilterLink *inlink);
    int (*convolute)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} HeadphoneContext;

static int parse_channel_name(const char *arg, enum AVChannel *rchannel)
{
    int channel = av_channel_from_string(arg);

    if (channel < 0 || channel >= 64)
        return AVERROR(EINVAL);
    *rchannel = channel;
    return 0;
}

static void parse_map(AVFilterContext *ctx)
{
    HeadphoneContext *s = ctx->priv;
    uint64_t used_channels = 0;

    for (int i = 0; i < s->nb_maps; i++) {
        enum AVChannel out_channel;
        const char *arg = s->map[i];

        if (s->nb_irs >= FF_ARRAY_ELEMS(s->mapping))
            break;

        if (parse_channel_name(arg, &out_channel)) {
            av_log(ctx, AV_LOG_WARNING, "Failed to parse \'%s\' as channel name.\n", arg);
            continue;
        }
        if (used_channels & (1ULL << out_channel)) {
            av_log(ctx, AV_LOG_WARNING, "Ignoring duplicate channel '%s'.\n", arg);
            continue;
        }
        used_channels        |= (1ULL << out_channel);
        s->mapping[s->nb_irs] = out_channel;
        s->nb_irs++;
    }
    av_channel_layout_from_mask(&s->map_channel_layout, used_channels);

    if (s->hrir_fmt == HRIR_MULTI)
        s->nb_hrir_inputs = 1;
    else
        s->nb_hrir_inputs = s->nb_irs;
}

typedef struct ThreadData {
    AVFrame *in, *out;
    int *n_clippings;
} ThreadData;

static int check_ir(AVFilterLink *inlink, int input_number)
{
    AVFilterContext *ctx = inlink->dst;
    HeadphoneContext *s = ctx->priv;
    int ir_len, max_ir_len;

    ir_len = ff_inlink_queued_samples(inlink);
    max_ir_len = 65536;
    if (ir_len > max_ir_len) {
        av_log(ctx, AV_LOG_ERROR, "Too big length of IRs: %d > %d.\n", ir_len, max_ir_len);
        return AVERROR(EINVAL);
    }
    s->hrir_in[input_number].ir_len = ir_len;
    s->ir_len = FFMAX(ir_len, s->ir_len);

    if (ff_inlink_check_available_samples(inlink, ir_len + 1) == 1) {
        s->hrir_in[input_number].eof = 1;
        return 1;
    }

    if (!s->hrir_in[input_number].eof) {
        ff_inlink_request_frame(inlink);
        return 0;
    }

    return 0;
}

static int headphone_frame(HeadphoneContext *s, AVFrame *in, AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    int n_clippings[2] = { 0 };
    ThreadData td;
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);
    out->pts = in->pts;

    td.in = in; td.out = out;
    td.n_clippings = n_clippings;

    ff_filter_execute(ctx, s->convolute, &td, NULL, 2);

    if (n_clippings[0] + n_clippings[1] > 0) {
        av_log(ctx, AV_LOG_WARNING, "%d of %d samples clipped. Please reduce gain.\n",
               n_clippings[0] + n_clippings[1], out->nb_samples * 2);
    }

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    HeadphoneContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in = NULL;
    int i, ret;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);
    if (!s->eof_hrirs) {
        int eof = 1;
        for (i = 0; i < s->nb_hrir_inputs; i++) {
            AVFilterLink *input = ctx->inputs[i + 1];

            if (s->hrir_in[i].eof)
                continue;

            if ((ret = check_ir(input, i)) <= 0)
                return ret;

            if (s->hrir_in[i].eof) {
                if (!ff_inlink_queued_samples(input)) {
                    av_log(ctx, AV_LOG_ERROR, "No samples provided for "
                           "HRIR stream %d.\n", i);
                    return AVERROR_INVALIDDATA;
                }
            } else {
                eof = 0;
            }
        }
        if (!eof) {
            ff_filter_set_ready(ctx, 100);
            return 0;
        }
        s->eof_hrirs = 1;

        ret = s->convert_coeffs(ctx, inlink);
        if (ret < 0)
            return ret;
    } else if (!s->have_hrirs)
        return AVERROR_EOF;

    if ((ret = ff_inlink_consume_samples(inlink, s->size, s->size, &in)) > 0)
        ret = headphone_frame(s, in, outlink);

    if (ret < 0)
        return ret;

    if (ff_inlink_queued_samples(inlink) >= s->size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    if (ff_outlink_frame_wanted(outlink))
        ff_inlink_request_frame(inlink);

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_DBL,
        AV_SAMPLE_FMT_NONE,
    };
    const HeadphoneContext *s = ctx->priv;
    AVFilterChannelLayouts *layouts = NULL;
    AVFilterChannelLayouts *stereo_layout = NULL;
    AVFilterChannelLayouts *hrir_layouts = NULL;
    int ret, i;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    layouts = ff_all_channel_layouts();
    if (!layouts)
        return AVERROR(ENOMEM);

    ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&stereo_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO);
    if (ret)
        return ret;
    ret = ff_channel_layouts_ref(stereo_layout, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    if (s->hrir_fmt == HRIR_MULTI) {
        hrir_layouts = ff_all_channel_counts();
        if (!hrir_layouts)
            return AVERROR(ENOMEM);
        ret = ff_channel_layouts_ref(hrir_layouts, &cfg_in[1]->channel_layouts);
        if (ret)
            return ret;
    } else {
        for (i = 1; i <= s->nb_hrir_inputs; i++) {
            ret = ff_channel_layouts_ref(stereo_layout, &cfg_in[i]->channel_layouts);
            if (ret)
                return ret;
        }
    }

    return 0;
}

#define DEPTH 32
#include "headphone_template.c"

#undef DEPTH
#define DEPTH 64
#include "headphone_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    HeadphoneContext *s = ctx->priv;

    if (s->nb_irs < inlink->ch_layout.nb_channels) {
        av_log(ctx, AV_LOG_ERROR, "Number of HRIRs must be >= %d.\n", inlink->ch_layout.nb_channels);
        return AVERROR(EINVAL);
    }

    s->lfe_channel = av_channel_layout_index_from_channel(&inlink->ch_layout,
                                                          AV_CHAN_LOW_FREQUENCY);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLT:
    case AV_SAMPLE_FMT_FLTP:
        s->convert_coeffs = convert_coeffs_flt;
        s->convolute = s->type ? headphone_fast_convolute_flt : headphone_convolute_flt;
        break;
    case AV_SAMPLE_FMT_DBL:
    case AV_SAMPLE_FMT_DBLP:
        s->convert_coeffs = convert_coeffs_dbl;
        s->convolute = s->type ? headphone_fast_convolute_dbl : headphone_convolute_dbl;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    HeadphoneContext *s = ctx->priv;
    int i, ret;

    if (!s->nb_maps) {
        av_log(ctx, AV_LOG_ERROR, "Valid mapping must be set.\n");
        return AVERROR(EINVAL);
    }

    parse_map(ctx);

    for (i = 0; i < s->nb_hrir_inputs; i++) {
        char *name = av_asprintf("hrir%d", i);
        AVFilterPad pad = {
            .name         = name,
            .type         = AVMEDIA_TYPE_AUDIO,
        };
        if (!name)
            return AVERROR(ENOMEM);
        if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    if (s->type == TIME_DOMAIN) {
        AVFloatDSPContext *fdsp = avpriv_float_dsp_alloc(0);
        if (!fdsp)
            return AVERROR(ENOMEM);
        s->scalarproduct_flt = fdsp->scalarproduct_float;
        s->scalarproduct_dbl = fdsp->scalarproduct_double;
        av_free(fdsp);
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    HeadphoneContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    if (s->hrir_fmt == HRIR_MULTI) {
        AVFilterLink *hrir_link = ctx->inputs[1];

        if (hrir_link->ch_layout.nb_channels < inlink->ch_layout.nb_channels * 2) {
            av_log(ctx, AV_LOG_ERROR, "Number of channels in HRIR stream must be >= %d.\n", inlink->ch_layout.nb_channels * 2);
            return AVERROR(EINVAL);
        }
    }

    s->gain_lfe = expf((s->gain - 3 * inlink->ch_layout.nb_channels + s->lfe_gain) / 20 * M_LN10);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    HeadphoneContext *s = ctx->priv;

    av_tx_uninit(&s->itx_ctx[0]);
    av_tx_uninit(&s->itx_ctx[1]);
    av_tx_uninit(&s->tx_ctx[0]);
    av_tx_uninit(&s->tx_ctx[1]);
    av_freep(&s->data_ir[0]);
    av_freep(&s->data_ir[1]);
    av_freep(&s->ringbuffer[0]);
    av_freep(&s->ringbuffer[1]);
    av_freep(&s->temp_src[0]);
    av_freep(&s->temp_src[1]);
    av_freep(&s->out_tx[0]);
    av_freep(&s->out_tx[1]);
    av_freep(&s->in_tx[0]);
    av_freep(&s->in_tx[1]);
    av_freep(&s->temp_afft[0]);
    av_freep(&s->temp_afft[1]);
    av_freep(&s->data_hrtf[0]);
    av_freep(&s->data_hrtf[1]);
}

#define OFFSET(x) offsetof(HeadphoneContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_map = {.def=NULL,.size_min=1,.sep='|'};

static const AVOption headphone_options[] = {
    { "map",       "set channels convolution mappings",  OFFSET(map),   AV_OPT_TYPE_STRING|AR, {.arr=&def_map},        .flags = FLAGS },
    { "gain",      "set gain in dB",                     OFFSET(gain),     AV_OPT_TYPE_FLOAT,  {.dbl=0},     -20,  40, .flags = FLAGS },
    { "lfe",       "set lfe gain in dB",                 OFFSET(lfe_gain), AV_OPT_TYPE_FLOAT,  {.dbl=0},     -20,  40, .flags = FLAGS },
    { "type",      "set processing",                     OFFSET(type),     AV_OPT_TYPE_INT,    {.i64=1},       0,   1, .flags = FLAGS, .unit = "type" },
    { "time",      "time domain",                        0,                AV_OPT_TYPE_CONST,  {.i64=0},       0,   0, .flags = FLAGS, .unit = "type" },
    { "freq",      "frequency domain",                   0,                AV_OPT_TYPE_CONST,  {.i64=1},       0,   0, .flags = FLAGS, .unit = "type" },
    { "size",      "set frame size",                     OFFSET(size),     AV_OPT_TYPE_INT,    {.i64=1024},1024,96000, .flags = FLAGS },
    { "hrir",      "set hrir format",                    OFFSET(hrir_fmt), AV_OPT_TYPE_INT,    {.i64=HRIR_STEREO}, 0, 1, .flags = FLAGS, .unit = "hrir" },
    { "stereo",    "hrir files have exactly 2 channels", 0,                AV_OPT_TYPE_CONST,  {.i64=HRIR_STEREO}, 0, 0, .flags = FLAGS, .unit = "hrir" },
    { "multich",   "single multichannel hrir file",      0,                AV_OPT_TYPE_CONST,  {.i64=HRIR_MULTI},  0, 0, .flags = FLAGS, .unit = "hrir" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(headphone);

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_input,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const FFFilter ff_af_headphone = {
    .p.name        = "headphone",
    .p.description = NULL_IF_CONFIG_SMALL("Apply headphone binaural spatialization with HRTFs in additional streams."),
    .p.priv_class  = &headphone_class,
    .priv_size     = sizeof(HeadphoneContext),
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_DYNAMIC_INPUTS,
};
