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
#include "window_func.h"

#define DEPTH 32

#if DEPTH == 32
#define MPI M_PIf
#define MPI2 M_PI_2f
#define MLN10 M_LN10f
#define ftype float
#define ctype AVComplexFloat
#define HYPOT hypotf
#define ATAN2 atan2f
#define FABS fabsf
#define SIN sinf
#define COS cosf
#define POW powf
#define TAN tanf
#define SQRT sqrtf
#define FMIN fminf
#define FMAX fmaxf
#define EPSILON FLT_EPSILON
#define CLIP av_clipf
#define SAMPLE_FORMAT AV_SAMPLE_FMT_FLTP
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define MPI M_PI
#define MPI2 M_PI_2
#define MLN10 M_LN10
#define ftype double
#define ctype AVComplexDouble
#define HYPOT hypot
#define ATAN2 atan2
#define FABS fabs
#define SIN sin
#define COS cos
#define POW pow
#define TAN tan
#define SQRT sqrt
#define FMIN fmin
#define FMAX fmax
#define EPSILON DBL_EPSILON
#define CLIP av_clipd
#define SAMPLE_FORMAT AV_SAMPLE_FMT_DBLP
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define F(x) ((ftype)(x))

enum SurroundChannel {
    SC_FL = 1, SC_FR, SC_FC, SC_LF, SC_BL, SC_BR, SC_BC, SC_SL, SC_SR,
    SC_TC, SC_TFC, SC_TFL, SC_TFR, SC_TBC, SC_TBL, SC_TBR,
    SC_LF2,
    SC_NB,
};

static const int ch_map[SC_NB] = {
    [SC_FL]  = AV_CHAN_FRONT_LEFT,
    [SC_FR]  = AV_CHAN_FRONT_RIGHT,
    [SC_FC]  = AV_CHAN_FRONT_CENTER,
    [SC_LF]  = AV_CHAN_LOW_FREQUENCY,
    [SC_BL]  = AV_CHAN_BACK_LEFT,
    [SC_BR]  = AV_CHAN_BACK_RIGHT,
    [SC_BC]  = AV_CHAN_BACK_CENTER,
    [SC_SL]  = AV_CHAN_SIDE_LEFT,
    [SC_SR]  = AV_CHAN_SIDE_RIGHT,
    [SC_TC]  = AV_CHAN_TOP_CENTER,
    [SC_TFC] = AV_CHAN_TOP_FRONT_CENTER,
    [SC_TFL] = AV_CHAN_TOP_FRONT_LEFT,
    [SC_TFR] = AV_CHAN_TOP_FRONT_RIGHT,
    [SC_TBC] = AV_CHAN_TOP_BACK_CENTER,
    [SC_TBL] = AV_CHAN_TOP_BACK_LEFT,
    [SC_TBR] = AV_CHAN_TOP_BACK_RIGHT,
    [SC_LF2] = AV_CHAN_LOW_FREQUENCY_2,
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
    float shift;
    float depth;
    float focus;

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

    ftype *input_levels;
    ftype *output_levels;
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
    AVFrame *output_mag;
    AVFrame *output_ph;
    AVFrame *output_out;
    AVFrame *overlap_buffer;
    AVFrame *window;

    ftype *x_pos;
    ftype *y_pos;
    ftype *z_pos;
    ftype *l_phase;
    ftype *r_phase;
    ftype *c_phase;
    ftype *c_mag;
    ftype *lfe_mag;
    ftype *lfe_phase;
    ftype *mag_total;

    int rdft_size;
    int hop_size;
    AVTXContext **rdft, **irdft;
    av_tx_fn tx_fn, itx_fn;
    float *window_func_lut;

    void (*filter)(AVFilterContext *ctx);
    void (*upmix)(AVFilterContext *ctx, int ch);
} AudioSurroundContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        SAMPLE_FORMAT,
        AV_SAMPLE_FMT_NONE,
    };
    const AudioSurroundContext *s = ctx->priv;
    AVFilterChannelLayouts *layouts = NULL;
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

    ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts);
    if (ret)
        return ret;

    return 0;
}

static void set_input_levels(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;

    for (int ch = 0;  ch < s->nb_in_channels; ch++) {
        const int fch = FFMIN3(ch, s->nb_f_i-1, SC_NB-1);

        s->input_levels[ch] = s->f_i[fch];
    }
}

static void set_output_levels(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;

    for (int ch = 0;  ch < s->nb_out_channels; ch++) {
        const int fch = FFMIN3(ch, s->nb_f_o-1, SC_NB-1);

        s->output_levels[ch] = s->f_o[fch];
    }
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioSurroundContext *s = ctx->priv;
    int ret;

    s->rdft = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->rdft));
    if (!s->rdft)
        return AVERROR(ENOMEM);
    s->nb_in_channels = inlink->ch_layout.nb_channels;

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        ftype scale = 1.0 / sqrt(s->win_size);

        ret = av_tx_init(&s->rdft[ch], &s->tx_fn, TX_TYPE,
                         0, s->win_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->input_levels = av_malloc_array(s->nb_in_channels, sizeof(*s->input_levels));
    if (!s->input_levels)
        return AVERROR(ENOMEM);

    set_input_levels(ctx);

    s->window = ff_get_audio_buffer(inlink, s->win_size);
    if (!s->window)
        return AVERROR(ENOMEM);

    s->input_in = ff_get_audio_buffer(inlink, s->win_size);
    if (!s->input_in)
        return AVERROR(ENOMEM);

    s->input = ff_get_audio_buffer(inlink, s->win_size + 2);
    if (!s->input)
        return AVERROR(ENOMEM);

    s->lowcut = 1.f * s->lowcutf / (inlink->sample_rate * 0.5) * s->win_size;
    s->highcut = 1.f * s->highcutf / (inlink->sample_rate * 0.5) * s->win_size;

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioSurroundContext *s = ctx->priv;
    int ret;

    s->irdft = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->irdft));
    if (!s->irdft)
        return AVERROR(ENOMEM);
    s->nb_out_channels = outlink->ch_layout.nb_channels;

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        ftype iscale = 1.0 / sqrt(s->win_size);

        ret = av_tx_init(&s->irdft[ch], &s->itx_fn, TX_TYPE,
                         1, s->win_size, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    s->output_levels = av_malloc_array(s->nb_out_channels, sizeof(*s->output_levels));
    if (!s->output_levels)
        return AVERROR(ENOMEM);

    set_output_levels(ctx);

    s->rdft_size = s->win_size / 2 + 1;

    s->factors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->sfactors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_ph = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_mag = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_out = ff_get_audio_buffer(outlink, s->win_size + 1);
    s->output = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->overlap_buffer = ff_get_audio_buffer(outlink, s->win_size * 2);
    s->x_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->y_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->z_out = ff_get_audio_buffer(outlink, s->rdft_size);
    if (!s->overlap_buffer || !s->factors || !s->sfactors ||
        !s->output || !s->output_out || !s->output_mag || !s->output_ph ||
        !s->x_out || !s->y_out || !s->z_out)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        ftype *src = (ftype *)s->sfactors->extended_data[ch];

        for (int n = 0; n < s->rdft_size; n++)
            src[n] = F(1.0) / s->nb_out_channels;
    }

    s->x_pos = av_calloc(s->rdft_size, sizeof(*s->x_pos));
    s->y_pos = av_calloc(s->rdft_size, sizeof(*s->y_pos));
    s->z_pos = av_calloc(s->rdft_size, sizeof(*s->z_pos));
    s->l_phase = av_calloc(s->rdft_size, sizeof(*s->l_phase));
    s->r_phase = av_calloc(s->rdft_size, sizeof(*s->r_phase));
    s->c_mag   = av_calloc(s->rdft_size, sizeof(*s->c_mag));
    s->c_phase = av_calloc(s->rdft_size, sizeof(*s->c_phase));
    s->mag_total = av_calloc(s->rdft_size, sizeof(*s->mag_total));
    s->lfe_mag = av_calloc(s->rdft_size, sizeof(*s->lfe_mag));
    s->lfe_phase = av_calloc(s->rdft_size, sizeof(*s->lfe_phase));
    if (!s->x_pos || !s->y_pos || !s->z_pos ||
        !s->l_phase || !s->r_phase || !s->lfe_phase || !s->c_phase ||
        !s->mag_total || !s->lfe_mag || !s->c_mag)
        return AVERROR(ENOMEM);

    return 0;
}

static ftype sqrf(ftype x)
{
    return x * x;
}

static ftype r_distance(ftype a)
{
    return FMIN(SQRT(F(1.0) + sqrf(TAN(a))), SQRT(F(1.0) + sqrf(F(1.0) / TAN(a))));
}

static void angle_transform(ftype *x, ftype *y, ftype angle)
{
    ftype reference, r, a;

    if (angle == F(90.0))
        return;

    reference = angle * MPI / F(180.0);
    r = HYPOT(*x, *y);
    a = ATAN2(*x, *y);

    r /= r_distance(a);

    if (FABS(a) <= M_PI_4f)
        a *= reference / M_PI_2f;
    else
        a = MPI + (F(-2.0) * MPI + reference) * (MPI - FABS(a)) * FFDIFFSIGN(a, F(0.0)) / (F(3.0) * M_PI_2f);

    r *= r_distance(a);

    *x = CLIP(SIN(a) * r, F(-1.0), F(1.0));
    *y = CLIP(COS(a) * r, F(-1.0), F(1.0));
}

static void shift_transform(ftype *y, const ftype shift)
{
    if (shift == F(0.0))
        return;

    *y = CLIP(*y + shift, F(-1.0), F(1.0));
}

static void depth_transform(ftype *y, const ftype depth)
{
    if (depth == F(0.0))
        return;

    if (depth < F(0.0) && *y > F(0.0))
        return;

    if (depth > F(0.0) && *y < F(0.0))
        return;

    *y = CLIP(*y + *y * depth, F(-1.0), F(1.0));
}

static void focus_transform(ftype *x, ftype *y, ftype focus)
{
    ftype a, r, ra;

    if (focus == F(0.0))
        return;

    a = ATAN2(*x, *y);
    ra = r_distance(a);
    r = CLIP(HYPOT(*x, *y) / ra, F(0.0), F(1.0));
    r = focus > F(0.0) ? F(1.0) - POW(F(1.0) - r, F(1.0) + focus * F(20.0)) : POW(r, F(1.0) - focus * F(20.0));
    r *= ra;
    *x = CLIP(SIN(a) * r, F(-1.0), F(1.0));
    *y = CLIP(COS(a) * r, F(-1.0), F(1.0));
}

static void stereo_position(const ftype l, const ftype r, const ftype ph,
                            ftype *x, ftype *y, ftype *z)
{
    const ftype a = F(2.0) * (ATAN2(l, r) / MPI2) - F(1.0);
    const ftype p = F(1.0) - F(2.0) * FABS(ph);
    const ftype v = (ph >= F(0.0)) ? F(1.0)-F(2.0)*FABS(F(0.5)-ph) : F(-1.0)+F(2.0)*FABS(F(0.5)+ph);

    *x = CLIP(a, F(-1.0), F(1.0));
    *y = CLIP(p, F(-1.0), F(1.0));
    *z = CLIP(v, F(-1.0), F(1.0));
}

static inline void get_lfe(int output_lfe, int n, ftype lowcut, ftype highcut,
                           ftype *lfe_mag, ftype c_mag, ftype *mag_total, int lfe_mode)
{
    if (output_lfe && n < highcut) {
        *lfe_mag    = n < lowcut ? F(1.0) : F(0.5)*(F(1.0)+COS(MPI*(lowcut-n)/(lowcut-highcut)));
        *lfe_mag   *= c_mag;
        if (lfe_mode)
            *mag_total -= *lfe_mag;
    } else {
        *lfe_mag = F(0.0);
    }
}

static void calculate_factors(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    ftype *x_out = (ftype *)s->x_out->extended_data[ch];
    ftype *y_out = (ftype *)s->y_out->extended_data[ch];
    ftype *z_out = (ftype *)s->z_out->extended_data[ch];
    const ftype f_x = s->f_x[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_x-1)];
    const ftype f_y = s->f_y[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_y-1)];
    const ftype f_z = s->f_z[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_z-1)];
    const int rdft_size = s->rdft_size;
    const ftype *x = s->x_pos;
    const ftype *y = s->y_pos;
    const ftype *z = s->z_pos;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = F(1.0) - FABS(x[n]);
        break;
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_BACK_LEFT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = (x[n] + F(1.0)) * F(0.5);
        break;
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = (F(1.0) - x[n]) * F(0.5);
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = x[n];
        break;
    }

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = (y[n] + F(1.0)) * F(0.5);
        break;
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = F(1.0) - FABS(y[n]);
        break;
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = (F(1.0) - y[n]) * F(0.5);
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = y[n];
        break;
    }

    switch (chan) {
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = (z[n] + F(1.0)) * F(0.5);
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = F(1.0) - FABS(z[n]);
        break;
    }

    for (int n = 0; n < rdft_size; n++)
        factor[n] = POW(x_out[n], f_x) *
                    POW(y_out[n], f_y) *
                    POW(z_out[n], f_z);
}

static void do_transform(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *sfactor = (ftype *)s->sfactors->extended_data[ch];
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    ctype *dst = (ctype *)s->output->extended_data[ch];
    const int rdft_size = s->rdft_size;
    const ftype smooth = s->smooth[FFMIN(ch, s->nb_smooth-1)];

    if (smooth > F(0.0)) {
        for (int n = 0; n < rdft_size; n++)
            sfactor[n] = smooth * factor[n] + (F(1.0) - smooth) * sfactor[n];

        factor = sfactor;
    }

    for (int n = 0; n < rdft_size; n++)
        omag[n] *= factor[n];

    for (int n = 0; n < rdft_size; n++) {
        const ftype mag = omag[n];
        const ftype ph = oph[n];

        dst[n].re = mag * COS(ph);
        dst[n].im = mag * SIN(ph);
    }
}

static void stereo_copy(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    const ftype *mag_total = s->mag_total;
    const int rdft_size = s->rdft_size;
    const ftype *c_phase = s->c_phase;
    const ftype *l_phase = s->l_phase;
    const ftype *r_phase = s->r_phase;
    const ftype *lfe_mag = s->lfe_mag;
    const ftype *c_mag = s->c_mag;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        memcpy(omag, c_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(omag, lfe_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
        memcpy(omag, mag_total, rdft_size * sizeof(*omag));
        break;
    default:
        break;
    }

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
        memcpy(oph, c_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
        memcpy(oph, l_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
        memcpy(oph, r_phase, rdft_size * sizeof(*oph));
        break;
    default:
        break;
    }
}

static void stereo_lfe_copy(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    const ftype *mag_total = s->mag_total;
    const ftype *lfe_phase = s->lfe_phase;
    const int rdft_size = s->rdft_size;
    const ftype *c_phase = s->c_phase;
    const ftype *l_phase = s->l_phase;
    const ftype *r_phase = s->r_phase;
    const ftype *lfe_mag = s->lfe_mag;
    const ftype *c_mag = s->c_mag;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        memcpy(omag, c_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(omag, lfe_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
        memcpy(omag, mag_total, rdft_size * sizeof(*omag));
        break;
    default:
        break;
    }

    switch (chan) {
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(oph, lfe_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
        memcpy(oph, c_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
        memcpy(oph, l_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
        memcpy(oph, r_phase, rdft_size * sizeof(*oph));
        break;
    default:
        break;
    }
}

static void stereo_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    calculate_factors(ctx, ch, chan);

    stereo_copy(ctx, ch, chan);

    do_transform(ctx, ch);
}

static void l2_1_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        calculate_factors(ctx, ch, -1);
        break;
    default:
        calculate_factors(ctx, ch, chan);
        break;
    }

    stereo_lfe_copy(ctx, ch, chan);

    do_transform(ctx, ch);
}

static void surround_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        calculate_factors(ctx, ch, -1);
        break;
    default:
        calculate_factors(ctx, ch, chan);
        break;
    }

    stereo_copy(ctx, ch, chan);

    do_transform(ctx, ch);
}

static void l3_1_upmix(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        calculate_factors(ctx, ch, -1);
        break;
    default:
        calculate_factors(ctx, ch, chan);
        break;
    }

    stereo_lfe_copy(ctx, ch, chan);

    do_transform(ctx, ch);
}

static void filter_stereo(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *magtotal = s->mag_total;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype c_phase = ATAN2(l_im + r_im, l_re + r_re);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype mag_total = HYPOT(l_mag, r_mag);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype phase = ATAN2(im, re) / MPI;
        ftype mag_sum = l_mag + r_mag;
        ftype c_mag = mag_sum * F(0.5);
        ftype x, y, z;

        stereo_position(l_mag, r_mag, phase, &x, &y, &z);
        get_lfe(output_lfe, n, lowcut, highcut, &lfemag[n], c_mag, &mag_total, lfe_mode);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        magtotal[n] = mag_total;
    }
}

static void filter_2_1(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srclfe = (const ftype *)s->input->extended_data[2];
    const int rdft_size = s->rdft_size;
    ftype *magtotal = s->mag_total;
    ftype *lfephase = s->lfe_phase;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype lfe_re = srclfe[2 * n], lfe_im = srclfe[2 * n + 1];
        ftype c_phase = ATAN2(l_im + r_im, l_re + r_re);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype lfe_mag = HYPOT(lfe_re, lfe_im);
        ftype lfe_phase = ATAN2(lfe_im, lfe_re);
        ftype mag_total = HYPOT(l_mag, r_mag);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype phase = ATAN2(im, re) / MPI;
        ftype mag_sum = l_mag + r_mag;
        ftype c_mag = mag_sum * F(0.5);
        ftype x, y, z;

        stereo_position(l_mag, r_mag, phase, &x, &y, &z);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        lfemag[n] = lfe_mag;
        lfephase[n] = lfe_phase;
        magtotal[n] = mag_total;
    }
}

static void filter_surround(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srcc = (const ftype *)s->input->extended_data[2];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *magtotal = s->mag_total;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype c_re = srcc[2 * n], c_im = srcc[2 * n + 1];
        ftype c_phase = ATAN2(c_im, c_re);
        ftype c_mag = HYPOT(c_re, c_im);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype mag_total = HYPOT(l_mag, r_mag);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype phase = ATAN2(im, re) / MPI;
        ftype x, y, z;

        stereo_position(l_mag, r_mag, phase, &x, &y, &z);
        get_lfe(output_lfe, n, lowcut, highcut, &lfemag[n], c_mag, &mag_total, lfe_mode);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        magtotal[n] = mag_total;
    }
}

static void filter_3_1(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srcc = (const ftype *)s->input->extended_data[2];
    const ftype *srclfe = (const ftype *)s->input->extended_data[3];
    const int rdft_size = s->rdft_size;
    ftype *magtotal = s->mag_total;
    ftype *lfephase = s->lfe_phase;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype lfe_re = srclfe[2 * n], lfe_im = srclfe[2 * n + 1];
        ftype c_re = srcc[2 * n], c_im = srcc[2 * n + 1];
        ftype c_phase = ATAN2(c_im, c_re);
        ftype c_mag = HYPOT(c_re, c_im);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype lfe_mag = HYPOT(lfe_re, lfe_im);
        ftype lfe_phase = ATAN2(lfe_im, lfe_re);
        ftype mag_total = HYPOT(l_mag, r_mag);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype phase = ATAN2(im, re) / MPI;
        ftype x, y, z;

        stereo_position(l_mag, r_mag, phase, &x, &y, &z);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        lfemag[n] = lfe_mag;
        lfephase[n] = lfe_phase;
        magtotal[n] = mag_total;
    }
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
    float overlap;

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
        s->filter = filter_stereo;
        s->upmix = stereo_upmix;
        break;
    case AV_CH_LAYOUT_2POINT1:
        s->filter = filter_2_1;
        s->upmix = l2_1_upmix;
        break;
    case AV_CH_LAYOUT_SURROUND:
        s->filter = filter_surround;
        s->upmix = surround_upmix;
        break;
    case AV_CH_LAYOUT_3POINT1:
        s->filter = filter_3_1;
        s->upmix = l3_1_upmix;
        break;
    default:
fail:
        av_channel_layout_describe(&s->out_ch_layout, out_name, sizeof(out_name));
        av_channel_layout_describe(&s->in_ch_layout, in_name, sizeof(in_name));
        av_log(ctx, AV_LOG_ERROR, "Unsupported upmix: '%s' -> '%s'.\n",
               in_name, out_name);
        return AVERROR(EINVAL);
    }

    s->window_func_lut = av_calloc(s->win_size, sizeof(*s->window_func_lut));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);

    generate_window_func(s->window_func_lut, s->win_size, s->win_func, &overlap);
    if (s->overlap == 1)
        s->overlap = overlap;

    s->hop_size = FFMAX(1, s->win_size * (1. - s->overlap));

    {
        float max = 0.f, *temp_lut = av_calloc(s->win_size, sizeof(*temp_lut));
        if (!temp_lut)
            return AVERROR(ENOMEM);

        for (int j = 0; j < s->win_size; j += s->hop_size) {
            for (int i = 0; i < s->win_size; i++)
                temp_lut[(i + j) % s->win_size] += s->window_func_lut[i];
        }

        for (int i = 0; i < s->win_size; i++)
            max = FMAX(temp_lut[i], max);
        av_freep(&temp_lut);

        s->win_gain = 1.f / max;
    }

    return 0;
}

static int fft_channel(AVFilterContext *ctx, AVFrame *in, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *src = (ftype *)s->input_in->extended_data[ch];
    ftype *win = (ftype *)s->window->extended_data[ch];
    const float *window_func_lut = s->window_func_lut;
    const int offset = s->input_in->nb_samples - s->hop_size;
    const ftype level_in = s->input_levels[ch];
    const int win_size = s->win_size;

    memmove(src, &src[s->hop_size], offset * sizeof(ftype));
    memcpy(&src[offset], in->extended_data[ch], in->nb_samples * sizeof(ftype));
    memset(&src[offset + in->nb_samples], 0, (s->hop_size - in->nb_samples) * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        win[n] = src[n] * window_func_lut[n] * level_in;

    s->tx_fn(s->rdft[ch], (ftype *)s->input->extended_data[ch], win, sizeof(ftype));

    return 0;
}

static int fft_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFrame *in = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        fft_channel(ctx, in, ch);

    return 0;
}

static int ifft_channel(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const float *window_func_lut = s->window_func_lut;
    const ftype level_out = s->output_levels[ch] * s->win_gain;
    const int win_size = s->win_size;
    ftype *dst, *ptr;

    dst = (ftype *)s->output_out->extended_data[ch];
    ptr = (ftype *)s->overlap_buffer->extended_data[ch];
    s->itx_fn(s->irdft[ch], dst, (ftype *)s->output->extended_data[ch], sizeof(ctype));

    memmove(ptr, ptr + s->hop_size, win_size * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        ptr[n] += dst[n] * window_func_lut[n] * level_out;

    dst = (ftype *)out->extended_data[ch];
    memcpy(dst, ptr, s->hop_size * sizeof(ftype));

    return 0;
}

static int transform_xy(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ftype angle = s->angle;
    const ftype focus = s->focus;
    const ftype shift = s->shift;
    const ftype depth = s->depth;
    ftype *x = s->x_pos;
    ftype *y = s->y_pos;

    for (int n = start; n < end; n++) {
        angle_transform(&x[n], &y[n], angle);
        shift_transform(&y[n], shift);
        depth_transform(&y[n], depth);
        focus_transform(&x[n], &y[n], focus);
    }

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
        ifft_channel(ctx, out, ch);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSurroundContext *s = ctx->priv;
    AVFrame *out;

    ff_filter_execute(ctx, fft_channels, in, NULL,
                      FFMIN(inlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    s->filter(ctx);

    out = ff_get_audio_buffer(outlink, s->hop_size);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    if (s->angle != 0.90f ||
        s->shift != 0.f ||
        s->depth != 0.f ||
        s->focus != 0.f)
    ff_filter_execute(ctx, transform_xy, NULL, NULL,
                      FFMIN(s->rdft_size,
                            ff_filter_get_nb_threads(ctx)));

    ff_filter_execute(ctx, ifft_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    av_frame_copy_props(out, in);
    out->nb_samples = in->nb_samples;
    out->pts -= av_rescale_q(s->win_size - s->hop_size, av_make_q(1, outlink->sample_rate), outlink->time_base);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSurroundContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret = 0, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->hop_size, s->hop_size, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        ret = filter_frame(inlink, in);
    if (ret < 0)
        return ret;

    if (ff_inlink_queued_samples(inlink) >= s->hop_size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return 0;
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
    av_frame_free(&s->output_ph);
    av_frame_free(&s->output_mag);
    av_frame_free(&s->output_out);
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
    av_freep(&s->rdft);
    av_freep(&s->irdft);
    av_freep(&s->window_func_lut);

    av_freep(&s->x_pos);
    av_freep(&s->y_pos);
    av_freep(&s->z_pos);
    av_freep(&s->l_phase);
    av_freep(&s->r_phase);
    av_freep(&s->c_mag);
    av_freep(&s->c_phase);
    av_freep(&s->mag_total);
    av_freep(&s->lfe_mag);
    av_freep(&s->lfe_phase);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    AudioSurroundContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    s->hop_size = FFMAX(1, s->win_size * (1. - s->overlap));

    set_input_levels(ctx);
    set_output_levels(ctx);

    return 0;
}

#define OFFSET(x) offsetof(AudioSurroundContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_smooth = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_o  = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_i  = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_x  = {.def="0.5",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_y  = {.def="0.5",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_f_z  = {.def="0.5",.size_min=1,.sep=' '};

static const AVOption surround_options[] = {
    { "chl_out",   "set output channel layout", OFFSET(out_ch_layout),     AV_OPT_TYPE_CHLAYOUT, {.str="5.1"}, 0,   0, FLAGS },
    { "chl_in",    "set input channel layout",  OFFSET(in_ch_layout),      AV_OPT_TYPE_CHLAYOUT, {.str="stereo"},0, 0, FLAGS },
    { "level_out", "set channels output levels",OFFSET(f_o),               AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_o}, 0,10, TFLAGS },
    { "level_in",  "set channels input levels", OFFSET(f_i),               AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_i}, 0,10, TFLAGS },
    { "lfe",       "output LFE",                OFFSET(output_lfe),        AV_OPT_TYPE_BOOL,     {.i64=1},     0,   1, TFLAGS },
    { "lfe_low",   "LFE low cut off",           OFFSET(lowcutf),           AV_OPT_TYPE_INT,      {.i64=128},   0, 256, FLAGS },
    { "lfe_high",  "LFE high cut off",          OFFSET(highcutf),          AV_OPT_TYPE_INT,      {.i64=256},   0, 512, FLAGS },
    { "lfe_mode",  "set LFE channel mode",      OFFSET(lfe_mode),          AV_OPT_TYPE_INT,      {.i64=0},     0,   1, TFLAGS, .unit = "lfe_mode" },
    {  "add",      "just add LFE channel",                  0,             AV_OPT_TYPE_CONST,    {.i64=0},     0,   1, TFLAGS, .unit = "lfe_mode" },
    {  "sub",      "subtract LFE channel with others",      0,             AV_OPT_TYPE_CONST,    {.i64=1},     0,   1, TFLAGS, .unit = "lfe_mode" },
    { "angle",     "set soundfield transform angle",     OFFSET(angle),    AV_OPT_TYPE_FLOAT,    {.dbl=90},    0, 360, TFLAGS },
    { "shift",     "set soundfield shift amount",        OFFSET(shift),    AV_OPT_TYPE_FLOAT,    {.dbl=0},    -1,   1, TFLAGS },
    { "depth",     "set soundfield depth amount",        OFFSET(depth),    AV_OPT_TYPE_FLOAT,    {.dbl=0},    -1,   1, TFLAGS },
    { "focus",     "set soundfield transform focus",     OFFSET(focus),    AV_OPT_TYPE_FLOAT,    {.dbl=0},    -1,   1, TFLAGS },
    { "spread_x",  "set channels X-axis spread",         OFFSET(f_x),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_x}, .06, 15, TFLAGS },
    { "spread_y",  "set channels Y-axis spread",         OFFSET(f_y),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_y}, .06, 15, TFLAGS },
    { "spread_z",  "set channels Z-axis spread",         OFFSET(f_z),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_f_z}, .06, 15, TFLAGS },
    { "smooth",    "set temporal smoothness strength",   OFFSET(smooth),   AV_OPT_TYPE_FLOAT|AR, {.arr=&def_smooth},0,1,TFLAGS },
    { "win_size", "set window size",                     OFFSET(win_size), AV_OPT_TYPE_INT,  {.i64=4096},1024,65536,FLAGS },
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
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const AVFilter ff_af_surround = {
    .name           = "surround",
    .description    = NULL_IF_CONFIG_SMALL("Apply audio surround upmix filter."),
    .priv_size      = sizeof(AudioSurroundContext),
    .priv_class     = &surround_class,
    .init           = init,
    .uninit         = uninit,
    .activate       = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .flags          = AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
