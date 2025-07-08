/*
 * Copyright (c) 2024 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "formats.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AASRCContext {
    const AVClass *class;

    int pass;
    int sample_rate;
    int channels;
    double t_inc;

    AVFrame *in;

    void *state;

    void (*do_aasrc)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
    int (*nb_output_samples)(AVFilterContext *ctx);
    void (*aasrc_uninit)(AVFilterContext *ctx);
} AASRCContext;

#define OFFSET(x) offsetof(AASRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption aasrc_options[] = {
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(aasrc);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AASRCContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    int ret, sample_rates[] = { s->sample_rate, -1 };

    ret = ff_add_format(&formats, AV_SAMPLE_FMT_S16P);
    if (ret)
        return ret;
    ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    if (ret)
        return ret;
    ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    if (ret)
        return ret;
    ret = ff_set_common_formats2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    if ((ret = ff_formats_ref(ff_all_samplerates(),
                              &cfg_in[0]->samplerates)) < 0)
        return ret;

    if (!s->sample_rate) {
        return ff_formats_ref(ff_all_samplerates(),
                              &cfg_out[0]->samplerates);
    }

    return ff_formats_ref(ff_make_format_list(sample_rates),
                          &cfg_out[0]->samplerates);
}

static const double ps1[][2] =
{
    { 0.96637235322842, -2.65273372307310 },
    { 0.89626967235734, -2.59860038103081 },
    { 0.81440696349573, -2.47839598145581 },
    { 0.71673488065413, -2.26785948965747 },
    { 0.60901667084191, -1.93229493507569 },
    { 0.50829206119013, -1.43723297659565 },
    { 0.43674907667108, -0.77529073927169 },
    { 0.41102514780717,  0.00000000000000 },
};

static const double rs1[][2] =
{
    {  1.559454965418541e-02, -8.711330296099162e-02 },
    { -2.821143538662592e-01,  1.931070842024560e-01 },
    {  8.057326481185847e-01,  1.604232271503586e-01 },
    { -9.406390985581723e-01, -1.229353144143673e+00 },
    {  3.054992725022572e-02,  2.511596664326740e+00 },
    {  1.911629155795335e+00, -3.046222365096745e+00 },
    { -3.979491574060914e+00,  2.124135337714532e+00 },
    {  2.438749735037705e+00, -1.110054033101757e-15 },
};

#define DEPTH 16
#include "aasrc_template.c"

#undef DEPTH
#define DEPTH 32
#include "aasrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "aasrc_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AASRCContext *s = ctx->priv;
    int ret;

    if (inlink->sample_rate == outlink->sample_rate) {
        s->pass = 1;
        return 0;
    }

    outlink->time_base = (AVRational) {1, outlink->sample_rate};
    s->channels = inlink->ch_layout.nb_channels;
    s->t_inc = inlink->sample_rate / ((double)outlink->sample_rate);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S16P:
        s->do_aasrc = aasrc_s16p;
        s->aasrc_uninit = aasrc_uninit_s16p;
        s->nb_output_samples = nb_output_samples_s16p;
        ret = aasrc_init_s16p(ctx);
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->do_aasrc = aasrc_fltp;
        s->aasrc_uninit = aasrc_uninit_fltp;
        s->nb_output_samples = nb_output_samples_fltp;
        ret = aasrc_init_fltp(ctx);
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_aasrc = aasrc_dblp;
        s->aasrc_uninit = aasrc_uninit_dblp;
        s->nb_output_samples = nb_output_samples_dblp;
        ret = aasrc_init_dblp(ctx);
        break;
    default:
        return AVERROR_BUG;
    }

    return ret;
}

static int aasrc_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AASRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_aasrc(ctx, s->in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AASRCContext *s = ctx->priv;
    int nb_out_samples;
    AVFrame *out;
    int ret;

    nb_out_samples = lrint(ceil(in->nb_samples / s->t_inc));

    out = ff_get_audio_buffer(outlink, nb_out_samples);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    av_frame_copy_props(out, in);
    ff_filter_execute(ctx, aasrc_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->nb_samples = s->nb_output_samples(ctx);
    out->sample_rate = outlink->sample_rate;
    out->pts = av_rescale_q(in->pts, inlink->time_base, outlink->time_base);
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);
    ret = ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    s->in = NULL;
    return ret < 0 ? ret : 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AASRCContext *s = ctx->priv;
    int ret, status;
    AVFrame *in;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        if (s->pass)
            return ff_filter_frame(outlink, in);
        else
            return filter_frame(inlink, in);
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AASRCContext *s = ctx->priv;

    if (s->aasrc_uninit)
        s->aasrc_uninit(ctx);
}

static AVFrame *get_in_audio_buffer(AVFilterLink *inlink, int nb_samples)
{
    AVFilterContext *ctx = inlink->dst;
    const AASRCContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_audio_buffer   (inlink, nb_samples) :
        ff_default_get_audio_buffer(inlink, nb_samples);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .get_buffer.audio = get_in_audio_buffer,
    },
};

static AVFrame *get_out_audio_buffer(AVFilterLink *outlink, int nb_samples)
{
    AVFilterContext *ctx = outlink->src;
    const AASRCContext *s = ctx->priv;

    return s->pass ?
        ff_null_get_audio_buffer   (outlink, nb_samples) :
        ff_default_get_audio_buffer(outlink, nb_samples);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .get_buffer.audio = get_out_audio_buffer,
    },
};

const FFFilter ff_af_aasrc = {
    .p.name          = "aasrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Arbitrary Audio Sample Rate Conversion."),
    .p.priv_class    = &aasrc_class,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(AASRCContext),
    .activate        = activate,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
