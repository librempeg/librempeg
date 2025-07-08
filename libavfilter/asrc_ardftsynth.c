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
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "libavutil/sfc64.h"
#include "libavutil/random_seed.h"

#define EVENT_MAXD_BITS 8
#define EVENT_MAXD (1 << EVENT_MAXD_BITS)
#define MAX_EVENT_BITS 8
#define MAX_EVENTS (1 << MAX_EVENT_BITS)

typedef struct SynthEvent {
    unsigned dur;
    unsigned idx;
    unsigned har;
    unsigned har_step;
    int idx_dir;
    int idx_per;
    int idx_dir_dur;
    int idx_cur_dur;
    int amp_dir;
    int amp_per;
    int amp_dir_dur;
    int amp_cur_dur;
    double amp;
    double amp_step;
    double har_factor;
} SynthEvent;

typedef struct PRNGState {
    uint64_t value;
    unsigned size;

    FFSFC64 state;
} PRNGState;

typedef struct ChannelContext {
    PRNGState prng;

    SynthEvent event[MAX_EVENTS];

    AVComplexDouble *input;
    double *overlap;
    double *output;

    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;
} ChannelContext;

typedef struct ARDFTSynth {
    const AVClass *class;
    int sample_rate;
    AVChannelLayout chlayout;
    int window_size;
    int events;
    int64_t duration;
    int64_t duration_opt;
    int64_t seed;
    int64_t pts;

    double *win;

    ChannelContext *cc;
} ARDFTSynth;

#define OFFSET(x) offsetof(ARDFTSynth, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ardftsynth_options[] = {
    { "events",       "set number of events",OFFSET(events),       AV_OPT_TYPE_INT,       {.i64 = 32},        0., MAX_EVENTS, FLAGS },
    { "sample_rate",  "set sample rate",     OFFSET(sample_rate),  AV_OPT_TYPE_INT,       {.i64 = 48000},    15,  INT_MAX,    FLAGS },
    { "duration",     "set duration",        OFFSET(duration_opt), AV_OPT_TYPE_DURATION,  {.i64 = 0},         0,  INT64_MAX,  FLAGS },
    { "channel_layout","set channel layout", OFFSET(chlayout),     AV_OPT_TYPE_CHLAYOUT,  {.str = "stereo"},  0,  0,          FLAGS },
    { "seed",         "set random seed",     OFFSET(seed),         AV_OPT_TYPE_INT64,     {.i64 = -1},       -1,  UINT_MAX,   FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(ardftsynth);

static av_cold int query_formats(const AVFilterContext *ctx,
                                 AVFilterFormatsConfig **cfg_in,
                                 AVFilterFormatsConfig **cfg_out)
{
    const ARDFTSynth *s = ctx->priv;
    AVFilterChannelLayouts *chlayouts = NULL;
    int sample_rates[] = { s->sample_rate, -1 };
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_add_channel_layout(&chlayouts, &s->chlayout);
    if (ret)
        return ret;

    ret = ff_set_common_channel_layouts2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static av_cold int config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ARDFTSynth *s = ctx->priv;
    double scale;
    int ret;

    if (s->seed == -1)
        s->seed = av_get_random_seed();

    s->cc = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->cc));
    if (!s->cc)
        return AVERROR(ENOMEM);

    s->window_size = 4096;
    scale = 1.0 / s->window_size;

    s->win = av_calloc(s->window_size, sizeof(*s->win));
    if (!s->win)
        return AVERROR(ENOMEM);

    for (int n = 0; n < s->window_size; n++)
        s->win[n] = sin(M_PI*n/(s->window_size-1));

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        ChannelContext *cc = &s->cc[ch];

        cc->overlap = av_calloc(s->window_size, sizeof(*cc->overlap));
        if (!cc->overlap)
            return AVERROR(ENOMEM);

        cc->output = av_calloc(s->window_size, sizeof(*cc->output));
        if (!cc->output)
            return AVERROR(ENOMEM);

        cc->input = av_calloc((s->window_size/2 + 128), sizeof(*cc->input));
        if (!cc->input)
            return AVERROR(ENOMEM);

        ff_sfc64_init(&cc->prng.state, s->seed, ch, ch, 12);

        ret = av_tx_init(&cc->tx_ctx,  &cc->tx_fn, AV_TX_DOUBLE_RDFT, 1, s->window_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->duration = 1;
    if (s->duration_opt > 0)
        s->duration = av_rescale(s->duration_opt, s->sample_rate, AV_TIME_BASE);

    return 0;
}

#define DEPTH 64

static uint64_t get_prng(PRNGState *p, unsigned size)
{
    const uint64_t mask = (size >= 64) ? UINT64_MAX : ((1ULL << size)-1LL);
    unsigned read, left = size;
    uint64_t ret = 0;

    while (left > 0) {
        if (p->size == 0) {
            p->value = ff_sfc64_get(&p->state);
            p->size = 64;
        }

        read = FFMIN(p->size, left);
        if (read < 64) {
            ret <<= read;
            ret |= p->value & ((1ULL << read)-1LL);
        } else {
            ret |= p->value;
        }
        p->value >>= read;
        p->size -= read;
        left -= read;
    }

    return ret & mask;
}

static int synth_channels(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ARDFTSynth *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out = arg;
    const int nb_channels = outlink->ch_layout.nb_channels;
    const int start = (nb_channels *  job   ) / nb_jobs;
    const int end   = (nb_channels * (job+1)) / nb_jobs;
    const int nb_samples = s->window_size;
    const int half_size = nb_samples/2;
    const double *win = s->win;
    AVComplexDouble phase;

    for (int ch = start; ch < end; ch++) {
        ChannelContext *cc = &s->cc[ch];
        AVComplexDouble *input = cc->input;
        SynthEvent *events = cc->event;
        double *overlap = cc->overlap;
        double *output = cc->output;
        PRNGState *p = &cc->prng;
        phase.re = cos(M_PI*45.0/180.0);
        phase.im = sin(M_PI*45.0/180.0);

        for (int i = 1; i < half_size; i++) {
            const double rnd = 0.01*get_prng(p, DEPTH)/((double)(1ULL<<(DEPTH-1)));

            input[i].re = rnd * phase.re;
            input[i].im = rnd * phase.im;
        }

        for (int i = 0; i < s->events; i++) {
            SynthEvent *event = &events[i];

            if (event->dur == 0) {
                event->dur = 1 + get_prng(p, EVENT_MAXD_BITS);
                event->idx = get_prng(p, av_ceil_log2(half_size));
                event->amp = 1.0 + 10.0*get_prng(p, DEPTH)/((double)(1ULL<<(DEPTH-1)));
                event->har = get_prng(p, 4);
                if (event->har) {
                    event->har_step = 1 + get_prng(p, 6);
                    event->har_factor = 1.0 - get_prng(p, 8) / 256.0;
                }
                if (!event->idx)
                    event->idx++;
                if (get_prng(p, 1)) {
                    event->idx_dir = get_prng(p, 1) ? -1 : 1;
                    event->idx_per = get_prng(p, 1);
                } else {
                    event->idx_dir = 0;
                    event->idx_per = 0;
                }
                if (get_prng(p, 1)) {
                    event->amp_dir = get_prng(p, 1) ? -1 : 1;
                    event->amp_per = get_prng(p, 1);
                    event->amp_step = event->amp * (0.1 + 4.0*get_prng(p, DEPTH)/((double)(1ULL<<(DEPTH-1))));
                } else {
                    event->amp_dir = 0;
                    event->amp_per = 0;
                    event->amp_step = 0.0;
                }
                event->idx_dir_dur = event->idx_cur_dur = get_prng(p, EVENT_MAXD_BITS);
                event->amp_dir_dur = event->amp_cur_dur = get_prng(p, EVENT_MAXD_BITS);
            } else {
                if (event->idx_dir) {
                    if (event->idx_cur_dur-- > 0)
                        event->idx += event->idx_dir;
                    if (event->idx_cur_dur == 0 && event->idx_per) {
                        event->idx_cur_dur = event->idx_dir_dur;
                        event->idx_dir *= -1;
                    }
                    event->idx = av_clip(event->idx, 1, half_size-1);
                }

                if (event->amp_dir) {
                    if (event->amp_cur_dur-- > 0)
                        event->amp += event->amp_dir * event->amp_step;
                    if (event->amp_cur_dur == 0 && event->amp_per) {
                        event->amp_cur_dur = event->amp_dir_dur;
                        event->amp_dir *= -1;
                    }
                    event->amp = av_clipd(event->amp, 0, 11.0);
                }

                event->dur--;
            }

            input[event->idx].re += event->amp * phase.re;
            input[event->idx].im += event->amp * phase.im;

            {
                double har_factor = event->har_factor;

                for (int h = 0; h < event->har; h++) {
                    unsigned idx = event->idx + (h+1) * event->har_step;

                    if (idx < half_size) {
                        input[idx].re += har_factor * event->amp * phase.re;
                        input[idx].im += har_factor * event->amp * phase.im;

                        har_factor *= har_factor;
                    }
                }
            }
        }

        cc->tx_fn(cc->tx_ctx, cc->output, cc->input, sizeof(AVComplexDouble));

        for (int i = 0; i < nb_samples; i++) {
            output[i] *= win[i];
            overlap[i] += output[i];
        }
        memcpy(out->extended_data[ch], overlap, half_size * sizeof(*overlap));

        memmove(overlap, overlap + half_size, sizeof(*overlap) * half_size);
        memset(overlap + half_size, 0, sizeof(*overlap) * half_size);
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    ARDFTSynth *s = ctx->priv;
    const int nb_samples = s->window_size;
    const int half_size = nb_samples/2;
    AVFrame *out;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    if (s->duration == 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    out = ff_get_audio_buffer(outlink, half_size);
    if (!out)
        return AVERROR(ENOMEM);

    ff_filter_execute(ctx, synth_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = s->pts;
    s->pts += half_size;
    if (s->duration_opt > 0)
        s->duration -= FFMIN(s->duration, half_size);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ARDFTSynth *s = ctx->priv;

    if (s->cc) {
        for (int ch = 0; ch < s->chlayout.nb_channels; ch++) {
            ChannelContext *cc = &s->cc[ch];

            av_freep(&cc->overlap);
            av_freep(&cc->output);
            av_freep(&cc->input);

            av_tx_uninit(&cc->tx_ctx);
        }
    }
    av_freep(&s->cc);

    av_freep(&s->win);
}

static const AVFilterPad ardftsynth_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_props,
    },
};

const FFFilter ff_asrc_ardftsynth = {
    .p.name        = "ardftsynth",
    .p.description = NULL_IF_CONFIG_SMALL("Synthesize audio signal."),
    .p.priv_class  = &ardftsynth_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(ARDFTSynth),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_OUTPUTS(ardftsynth_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
