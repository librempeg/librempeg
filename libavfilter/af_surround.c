/*
 * Copyright (c) 2017 Paul B Mahol
 *
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

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"
#include "window_func_opt.h"

enum SurroundChannel {
    SC_FL = 1, SC_FR, SC_FC, SC_LF, SC_BL, SC_BR, SC_BC, SC_SL, SC_SR,
    SC_TC, SC_TFC, SC_TFL, SC_TFR, SC_TBC, SC_TBL, SC_TBR,
    SC_LF2, SC_TSL, SC_TSR, SC_BFC, SC_BFL, SC_BFR, SC_FLC, SC_FRC,
    SC_NB,
};

static const int8_t ch_dif[SC_NB] = {
    [SC_FC]  =  0,
    [SC_LF]  =  0,
    [SC_BC]  =  0,
    [SC_LF2] =  0,
    [SC_TC]  =  0,
    [SC_TFC] =  0,
    [SC_TBC] =  0,
    [SC_BFC] =  0,
    [SC_FL]  =  1,
    [SC_BL]  =  1,
    [SC_SL]  =  1,
    [SC_TFL] =  1,
    [SC_TBL] =  1,
    [SC_TSL] =  1,
    [SC_BFL] =  1,
    [SC_FLC] =  1,
    [SC_FR]  = -1,
    [SC_BR]  = -1,
    [SC_SR]  = -1,
    [SC_TFR] = -1,
    [SC_TBR] = -1,
    [SC_TSR] = -1,
    [SC_BFR] = -1,
    [SC_FRC] = -1,
};

static const int sc_map[64] = {
    [AV_CHAN_FRONT_LEFT      ] = SC_FL,
    [AV_CHAN_FRONT_RIGHT     ] = SC_FR,
    [AV_CHAN_FRONT_CENTER    ] = SC_FC,
    [AV_CHAN_LOW_FREQUENCY   ] = SC_LF,
    [AV_CHAN_BACK_LEFT       ] = SC_BL,
    [AV_CHAN_BACK_RIGHT      ] = SC_BR,
    [AV_CHAN_BACK_CENTER     ] = SC_BC,
    [AV_CHAN_SIDE_LEFT       ] = SC_SL,
    [AV_CHAN_SIDE_RIGHT      ] = SC_SR,
    [AV_CHAN_TOP_CENTER      ] = SC_TC,
    [AV_CHAN_TOP_FRONT_CENTER] = SC_TFC,
    [AV_CHAN_TOP_FRONT_LEFT  ] = SC_TFL,
    [AV_CHAN_TOP_FRONT_RIGHT ] = SC_TFR,
    [AV_CHAN_TOP_BACK_CENTER ] = SC_TBC,
    [AV_CHAN_TOP_BACK_LEFT   ] = SC_TBL,
    [AV_CHAN_TOP_BACK_RIGHT  ] = SC_TBR,
    [AV_CHAN_LOW_FREQUENCY_2 ] = SC_LF2,
    [AV_CHAN_TOP_SIDE_LEFT   ] = SC_TSL,
    [AV_CHAN_TOP_SIDE_RIGHT  ] = SC_TSR,
    [AV_CHAN_BOTTOM_FRONT_CENTER] = SC_BFC,
    [AV_CHAN_BOTTOM_FRONT_LEFT  ] = SC_BFL,
    [AV_CHAN_BOTTOM_FRONT_RIGHT ] = SC_BFR,
    [AV_CHAN_FRONT_LEFT_OF_CENTER] = SC_FLC,
    [AV_CHAN_FRONT_RIGHT_OF_CENTER] = SC_FRC,
};

typedef struct AudioSurroundContext {
    const AVClass *class;

    AVChannelLayout out_ch_layout;
    AVChannelLayout in_ch_layout;

    float *f_i;
    unsigned nb_f_i;
    float *f_o;
    unsigned nb_f_o;

    float *smooth;
    unsigned nb_smooth;
    float angle;
    float *shift;
    unsigned nb_shift;
    float *depth;
    unsigned nb_depth;
    float *focus;
    unsigned nb_focus;

    int   smooth_init;
    int   lfe_mode;
    int   win_size;
    int   win_func;
    float win_gain;
    float overlap;

    float *f_x;
    unsigned nb_f_x;
    float *f_y;
    unsigned nb_f_y;
    float *f_z;
    unsigned nb_f_z;

    int output_lfe;
    int create_lfe;
    int lowcutf;
    int highcutf;

    float lowcut;
    float highcut;

    int nb_in_channels;
    int nb_out_channels;

    AVFrame *factors;
    AVFrame *sfactors;
    AVFrame *input_in;
    AVFrame *input;
    AVFrame *output;
    AVFrame *x_out;
    AVFrame *y_out;
    AVFrame *z_out;
    AVFrame *output_out;
    AVFrame *output_sum;
    AVFrame *output_dif;
    AVFrame *overlap_buffer;
    AVFrame *window;

    void *input_levels;
    void *output_levels;
    void *smooth_levels;

    void *x_pos;
    void *y_pos;
    void *z_pos;

    void *sum;
    void *dif;
    void *cnt;
    void *lfe;

    int trim_size;
    int flush_size;
    int64_t last_pts;
    int rdft_size;
    int hop_size;
    AVTXContext **rdft, **irdft;
    av_tx_fn tx_fn, itx_fn;
    void *window_func_lut;

    void (*filter)(AVFilterContext *ctx);
    void (*set_input_levels)(AVFilterContext *ctx);
    void (*set_output_levels)(AVFilterContext *ctx);
    void (*set_smooth_levels)(AVFilterContext *ctx);
    void (*upmix)(AVFilterContext *ctx, int ch);
    int (*fft_channel)(AVFilterContext *ctx, AVFrame *out, int ch);
    int (*ifft_channel)(AVFilterContext *ctx, AVFrame *out, int ch);
    void (*calculate_factors)(AVFilterContext *ctx, int ch, int chan);
    void (*stereo_copy)(AVFilterContext *ctx, int ch, int chan);
    void (*do_transform)(AVFilterContext *ctx, int ch);
    void (*bypass_transform)(AVFilterContext *ctx, int ch, int is_lfe);
    int (*transform_xy)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioSurroundContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    const AudioSurroundContext *s = ctx->priv;
    AVFilterChannelLayouts *layouts;
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    layouts = NULL;
    ret = ff_add_channel_layout(&layouts, &s->out_ch_layout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    layouts = NULL;
    ret = ff_add_channel_layout(&layouts, &s->in_ch_layout);
    if (ret)
        return ret;

    return ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts);
}

static void stereo_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    s->calculate_factors(ctx, ch, chan);

    s->stereo_copy(ctx, ch, chan);

    s->do_transform(ctx, ch);
}

static void l2_1_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        s->bypass_transform(ctx, ch, 1);
        return;
    default:
        s->calculate_factors(ctx, ch, chan);
        break;
    }

    s->stereo_copy(ctx, ch, chan);

    s->do_transform(ctx, ch);
}

static void surround_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        s->bypass_transform(ctx, ch, 0);
        return;
    default:
        s->calculate_factors(ctx, ch, chan);
        break;
    }

    s->stereo_copy(ctx, ch, chan);

    s->do_transform(ctx, ch);
}

static void l3_1_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        s->bypass_transform(ctx, ch, 0);
        return;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        s->bypass_transform(ctx, ch, 1);
        return;
    default:
        s->calculate_factors(ctx, ch, chan);
        break;
    }

    s->stereo_copy(ctx, ch, chan);

    s->do_transform(ctx, ch);
}

#define DEPTH 32
#include "surround_template.c"

#undef DEPTH
#define DEPTH 64
#include "surround_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        return config_input_fltp(ctx);
    case AV_SAMPLE_FMT_DBLP:
        return config_input_dblp(ctx);
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        return config_output_fltp(ctx);
    case AV_SAMPLE_FMT_DBLP:
        return config_output_dblp(ctx);
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int can_upmix(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;

    for (int ch = 0; ch < s->out_ch_layout.nb_channels; ch++) {
        const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

        if ((chan < 0 || chan >= FF_ARRAY_ELEMS(sc_map)) || sc_map[chan] <= 0)
            return 0;
    }

    return 1;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    char in_name[128], out_name[128];
    int64_t in_channel_layout;

    if (s->lowcutf >= s->highcutf) {
        av_log(ctx, AV_LOG_ERROR, "Low cut-off '%d' should be less than high cut-off '%d'.\n",
               s->lowcutf, s->highcutf);
        return AVERROR(EINVAL);
    }

    in_channel_layout  = s->in_ch_layout.order == AV_CHANNEL_ORDER_NATIVE ?
                         s->in_ch_layout.u.mask : 0;

    s->create_lfe = av_channel_layout_index_from_channel(&s->out_ch_layout,
                                                         AV_CHAN_LOW_FREQUENCY) >= 0;

    if (!can_upmix(ctx))
        goto fail;

    switch (in_channel_layout) {
    case AV_CH_LAYOUT_STEREO:
    case AV_CH_LAYOUT_2POINT1:
    case AV_CH_LAYOUT_SURROUND:
    case AV_CH_LAYOUT_3POINT1:
        break;
    default:
fail:
        av_channel_layout_describe(&s->out_ch_layout, out_name, sizeof(out_name));
        av_channel_layout_describe(&s->in_ch_layout, in_name, sizeof(in_name));
        av_log(ctx, AV_LOG_ERROR, "Unsupported upmix: '%s' -> '%s'.\n",
               in_name, out_name);
        return AVERROR(EINVAL);
    }

    return 0;
}

static int fft_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    AVFrame *in = arg;
    const int start = (s->in_ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (s->in_ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->fft_channel(ctx, in, ch);

    return 0;
}

static int ifft_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        if (s->upmix)
            s->upmix(ctx, ch);
        s->ifft_channel(ctx, out, ch);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSurroundContext *s = ctx->priv;
    int nb_samples;
    AVFrame *out;

    ff_filter_execute(ctx, fft_channels, in, NULL,
                      FFMIN(inlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->filter(ctx);

    if (in) {
        int extra_samples = in->nb_samples % s->hop_size;

        if (extra_samples)
            extra_samples = FFMIN(s->hop_size - extra_samples, s->flush_size);
        nb_samples = in->nb_samples;
        if (extra_samples > 0) {
            nb_samples += extra_samples;
            s->flush_size -= extra_samples;
        }
    } else {
        nb_samples = FFMIN(s->flush_size, s->hop_size);
        s->flush_size -= nb_samples;
    }

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }


    ff_filter_execute(ctx, s->transform_xy, NULL, NULL,
                      FFMIN(s->rdft_size,
                            ff_filter_get_nb_threads(ctx)));

    ff_filter_execute(ctx, ifft_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->smooth_init = 1;

    if (in) {
        av_frame_copy_props(out, in);
        out->pts -= av_rescale_q(s->win_size - s->hop_size, av_make_q(1, outlink->sample_rate), outlink->time_base);
    } else {
        out->pts = s->last_pts;
    }
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    s->last_pts = out->pts + out->duration;

    if (s->trim_size > 0 && s->trim_size < out->nb_samples) {
        for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
            out->extended_data[ch] += s->trim_size * av_get_bytes_per_sample(out->format);

        out->nb_samples -= s->trim_size;
        s->trim_size = 0;
    } else if (s->trim_size > 0) {
        s->trim_size -= out->nb_samples;
        av_frame_free(&out);
        av_frame_free(&in);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioSurroundContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        ret = filter_frame(ctx->inputs[0], NULL);
        if (ret < 0)
            break;
    }

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSurroundContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->hop_size, s->hop_size, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_samples(inlink) >= s->hop_size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->flush_size > 0)
            ret = flush_frame(outlink);

        ff_outlink_set_status(outlink, status, s->last_pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;

    av_frame_free(&s->factors);
    av_frame_free(&s->sfactors);
    av_frame_free(&s->window);
    av_frame_free(&s->input_in);
    av_frame_free(&s->input);
    av_frame_free(&s->output);
    av_frame_free(&s->output_out);
    av_frame_free(&s->output_sum);
    av_frame_free(&s->output_dif);
    av_frame_free(&s->overlap_buffer);
    av_frame_free(&s->x_out);
    av_frame_free(&s->y_out);
    av_frame_free(&s->z_out);

    for (int ch = 0; ch < s->nb_in_channels; ch++)
        av_tx_uninit(&s->rdft[ch]);
    for (int ch = 0; ch < s->nb_out_channels; ch++)
        av_tx_uninit(&s->irdft[ch]);
    av_freep(&s->input_levels);
    av_freep(&s->output_levels);
    av_freep(&s->smooth_levels);
    av_freep(&s->rdft);
    av_freep(&s->irdft);
    av_freep(&s->window_func_lut);

    av_freep(&s->x_pos);
    av_freep(&s->y_pos);
    av_freep(&s->z_pos);
    av_freep(&s->sum);
    av_freep(&s->dif);
    av_freep(&s->cnt);
    av_freep(&s->lfe);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    AudioSurroundContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    s->hop_size = FFMAX(1, lrint(s->win_size * (1.0 - s->overlap)));

    s->set_input_levels(ctx);
    s->set_output_levels(ctx);
    s->set_smooth_levels(ctx);

    return 0;
}

#define OFFSET(x) offsetof(AudioSurroundContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_smooth = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_o  = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_i  = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_x  = {.def="8",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_y  = {.def="8",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_z  = {.def="8",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_shift= {.def="0 0 0",.size_min=1,.size_max=3,.sep=' '};
static const AVOptionArrayDef def_depth= {.def="0 0 0",.size_min=1,.size_max=3,.sep=' '};
static const AVOptionArrayDef def_focus= {.def="0 0 0",.size_min=1,.size_max=3,.sep=' '};

static const AVOption surround_options[] = {
    { "chl_out",   "set output channel layout", OFFSET(out_ch_layout),     AV_OPT_TYPE_CHLAYOUT, {.str="5.1"}, 0,   0, FLAGS },
    { "chl_in",    "set input channel layout",  OFFSET(in_ch_layout),      AV_OPT_TYPE_CHLAYOUT, {.str="stereo"},0, 0, FLAGS },
    { "level_out", "set output channels levels",OFFSET(f_o),               AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_o}, 0,30, TFLAGS },
    { "level_in",  "set input channels levels", OFFSET(f_i),               AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_i}, 0,30, TFLAGS },
    { "lfe",       "output LFE",                OFFSET(output_lfe),        AV_OPT_TYPE_BOOL,     {.i64=1},     0,   1, TFLAGS },
    { "lfe_low",   "LFE low cut off",           OFFSET(lowcutf),           AV_OPT_TYPE_INT,      {.i64=128},  20, 256, FLAGS },
    { "lfe_high",  "LFE high cut off",          OFFSET(highcutf),          AV_OPT_TYPE_INT,      {.i64=256},  80, 512, FLAGS },
    { "lfe_mode",  "set LFE channel mode",      OFFSET(lfe_mode),          AV_OPT_TYPE_INT,      {.i64=0},     0,   1, TFLAGS, .unit = "lfe_mode" },
    {  "add",      "just add LFE channel",                  0,             AV_OPT_TYPE_CONST,    {.i64=0},     0,   1, TFLAGS, .unit = "lfe_mode" },
    {  "sub",      "subtract LFE channel with others",      0,             AV_OPT_TYPE_CONST,    {.i64=1},     0,   1, TFLAGS, .unit = "lfe_mode" },
    { "angle",     "set soundfield transform angle",     OFFSET(angle),    AV_OPT_TYPE_FLOAT,    {.dbl=90},    0, 360, TFLAGS },
    { "shift",     "set soundfield shift per X/Y/Z axis",OFFSET(shift),    AV_OPT_TYPE_FLOAT|AR, {.arr=&def_shift},-1,1,TFLAGS },
    { "depth",     "set soundfield depth per X/Y/Z axis",OFFSET(depth),    AV_OPT_TYPE_FLOAT|AR, {.arr=&def_depth},-1,1,TFLAGS },
    { "focus",     "set soundfield focus per X/Y/Z axis",OFFSET(focus),    AV_OPT_TYPE_FLOAT|AR, {.arr=&def_focus},-1,1,TFLAGS },
    { "spread_x",  "set output channels X-axis spread",  OFFSET(f_x),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_x}, 0.0, 15, TFLAGS },
    { "spread_y",  "set output channels Y-axis spread",  OFFSET(f_y),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_y}, 0.0, 15, TFLAGS },
    { "spread_z",  "set output channels Z-axis spread",  OFFSET(f_z),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_z}, 0.0, 15, TFLAGS },
    { "smooth",    "set output channels temporal smoothness strength", OFFSET(smooth),AV_OPT_TYPE_FLOAT|AR, {.arr=&def_smooth},0,1,TFLAGS },
    WIN_FUNC_OPTION("win_func", OFFSET(win_func), FLAGS, WFUNC_SINE),
    { "overlap", "set window overlap", OFFSET(overlap), AV_OPT_TYPE_FLOAT, {.dbl=0.5}, 0, 1, TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(surround);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_surround = {
    .p.name         = "surround",
    .p.description  = NULL_IF_CONFIG_SMALL("Apply audio surround upmix filter."),
    .p.priv_class   = &surround_class,
    .priv_size      = sizeof(AudioSurroundContext),
    .init           = init,
    .uninit         = uninit,
    .activate       = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags        = AVFILTER_FLAG_SLICE_THREADS |
                      AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .process_command = process_command,
};
