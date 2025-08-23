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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/sfc64.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

enum FilterType {
    DC_TYPE,
    AC_TYPE,
    SQ_TYPE,
    PS_TYPE,
    RN_TYPE,
    NB_TYPES,
};

typedef struct ADenormContext {
    const AVClass *class;

    double level;
    double level_db;
    int type;
    int64_t in_samples;
    FFSFC64 *prng_state;

    void (*filter[NB_TYPES])(AVFilterContext *ctx, void *dst,
                             const void *src, const int nb_samples,
                             const int ch);
} ADenormContext;

#define DEPTH 32
#include "adenorm_template.c"

#undef DEPTH
#define DEPTH 64
#include "adenorm_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ADenormContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->filter[DC_TYPE] = dc_denorm_fltp;
        s->filter[AC_TYPE] = ac_denorm_fltp;
        s->filter[SQ_TYPE] = sq_denorm_fltp;
        s->filter[PS_TYPE] = ps_denorm_fltp;
        s->filter[RN_TYPE] = rn_denorm_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter[DC_TYPE] = dc_denorm_dblp;
        s->filter[AC_TYPE] = ac_denorm_dblp;
        s->filter[SQ_TYPE] = sq_denorm_dblp;
        s->filter[PS_TYPE] = ps_denorm_dblp;
        s->filter[RN_TYPE] = rn_denorm_dblp;
        break;
    default:
        av_assert0(0);
    }

    s->prng_state = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->prng_state));
    if (!s->prng_state)
        return AVERROR(ENOMEM);
    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        FFSFC64 *state = &s->prng_state[ch];

        ff_sfc64_init(state, ch, ch, ch, 12);
    }

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ADenormContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        s->filter[s->type](ctx, out->extended_data[ch],
                           in->extended_data[ch],
                           in->nb_samples, ch);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    ADenormContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    s->level = exp(s->level_db / 20. * M_LN10);
    td.in = in; td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    s->in_samples += in->nb_samples;

    if (out != in)
        ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ADenormContext *s = ctx->priv;

    av_freep(&s->prng_state);
}

static const AVFilterPad adenorm_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad adenorm_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(ADenormContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption adenorm_options[] = {
    { "level", "set level", OFFSET(level_db), AV_OPT_TYPE_DOUBLE, {.dbl=-351},   -451,        -90, FLAGS },
    { "type",  "set type",  OFFSET(type),     AV_OPT_TYPE_INT,    {.i64=DC_TYPE},   0, NB_TYPES-1, FLAGS, .unit = "type" },
    { "dc",    NULL,  0, AV_OPT_TYPE_CONST, {.i64=DC_TYPE}, 0, 0, FLAGS, .unit = "type"},
    { "ac",    NULL,  0, AV_OPT_TYPE_CONST, {.i64=AC_TYPE}, 0, 0, FLAGS, .unit = "type"},
    { "square",NULL,  0, AV_OPT_TYPE_CONST, {.i64=SQ_TYPE}, 0, 0, FLAGS, .unit = "type"},
    { "pulse", NULL,  0, AV_OPT_TYPE_CONST, {.i64=PS_TYPE}, 0, 0, FLAGS, .unit = "type"},
    { "random",NULL,  0, AV_OPT_TYPE_CONST, {.i64=RN_TYPE}, 0, 0, FLAGS, .unit = "type"},
    { NULL }
};

AVFILTER_DEFINE_CLASS(adenorm);

const FFFilter ff_af_adenorm = {
    .p.name          = "adenorm",
    .p.description   = NULL_IF_CONFIG_SMALL("Remedy denormals by adding extremely low-level noise."),
    .p.priv_class    = &adenorm_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(ADenormContext),
    .uninit          = uninit,
    .process_command = ff_filter_process_command,
    FILTER_INPUTS(adenorm_inputs),
    FILTER_OUTPUTS(adenorm_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
};
