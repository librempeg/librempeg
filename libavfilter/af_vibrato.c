/*
 * Copyright (c) 2015 Kyle Swanson <k@ylo.ph>.
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
#include "avfilter.h"
#include "filters.h"
#include "audio.h"
#include "generate_wave_table.h"

typedef struct VibratoContext {
    const AVClass *class;
    double freq;
    double depth;
    int channels;

    AVFrame *buf, *in;
    int *buf_index;
    int buf_size;

    void *wave_table;
    int *wave_table_index;
    int wave_table_size;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} VibratoContext;

#define OFFSET(x) offsetof(VibratoContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption vibrato_options[] = {
    { "f", "set frequency in hertz",    OFFSET(freq),    AV_OPT_TYPE_DOUBLE,   {.dbl = 5.0},   0.1,   20000.0, FLAGS },
    { "d", "set depth as percentage",   OFFSET(depth),   AV_OPT_TYPE_DOUBLE,   {.dbl = 0.5},   0.00,  1.0,     TFLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(vibrato);

#define DEPTH 32
#include "vibrato_template.c"

#undef DEPTH
#define DEPTH 64
#include "vibrato_template.c"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    VibratoContext *s = ctx->priv;
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

    s->in = in;
    ff_filter_execute(ctx, s->filter_channels, out, NULL,
                      FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));

    s->in = NULL;
    if (in != out)
        ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VibratoContext *s = ctx->priv;

    av_freep(&s->wave_table_index);
    av_freep(&s->wave_table);
    av_freep(&s->buf_index);
    av_frame_free(&s->buf);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    VibratoContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->channels = inlink->ch_layout.nb_channels;
    s->buf_size = lrint(inlink->sample_rate * 0.005 + 0.5);
    s->buf_index = av_calloc(s->channels, sizeof(s->buf_index));
    s->buf = ff_get_audio_buffer(ctx->outputs[0], s->buf_size);
    if (!s->buf || !s->buf_index)
        return AVERROR(ENOMEM);

    s->wave_table_size = lrint(inlink->sample_rate / s->freq + 0.5);
    s->wave_table_index = av_calloc(s->channels, sizeof(s->wave_table_index));
    s->wave_table = av_calloc(s->wave_table_size, av_get_bytes_per_sample(inlink->format));
    if (!s->wave_table || !s->wave_table_index)
        return AVERROR(ENOMEM);
    ff_generate_wave_table(WAVE_SIN, av_get_packed_sample_fmt(inlink->format), s->wave_table,
                           s->wave_table_size, 0.0, s->buf_size - 1, 3.0 * M_PI_2);

    return 0;
}

static const AVFilterPad vibrato_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_vibrato = {
    .p.name        = "vibrato",
    .p.description = NULL_IF_CONFIG_SMALL("Apply vibrato effect."),
    .p.priv_class  = &vibrato_class,
    .priv_size     = sizeof(VibratoContext),
    .uninit        = uninit,
    FILTER_INPUTS(vibrato_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
