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

#undef CLIP
#undef FABS
#undef FEXP
#undef FLOG
#undef FMAX
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define CLIP av_clipf
#define FABS fabsf
#define FEXP expf
#define FLOG log1pf
#define FMAX fmaxf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define CLIP av_clipd
#define FABS fabs
#define FEXP exp
#define FLOG log1p
#define FMAX fmax
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define LIN2LOG(x) (FLOG(x))
#define LOG2LIN(x) (FEXP(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define PEAKS(empty_value,op,sample, psample)\
    if (!empty && psample == dd[front]) {    \
        dd[front] = empty_value;             \
        if (back != front) {                 \
            front--;                         \
            if (front < 0)                   \
                front = n - 1;               \
        }                                    \
        empty = (front == back) &&           \
                (dd[front] == empty_value);  \
    }                                        \
                                             \
    while (!empty && sample op dd[front]) {  \
        dd[front] = empty_value;             \
        if (back == front) {                 \
            empty = 1;                       \
            break;                           \
        }                                    \
        front--;                             \
        if (front < 0)                       \
            front = n - 1;                   \
    }                                        \
                                             \
    while (!empty && sample op dd[back]) {   \
        dd[back] = empty_value;              \
        if (back == front) {                 \
            empty = 1;                       \
            break;                           \
        }                                    \
        back++;                              \
        if (back >= n)                       \
            back = 0;                        \
    }                                        \
                                             \
    if (!empty) {                            \
        back--;                              \
        if (back < 0)                        \
            back = n - 1;                    \
    }

static void fn(queue_sample)(PeakContext *cc,
                             const ftype i, const ftype x)
{
    ftype *ii = cc->iqueue;
    ftype *dd = cc->dqueue;
    ftype *qq = cc->queue;
    int front = cc->front;
    int back = cc->back;
    int empty, n, pos = cc->position;
    ftype px = FABS(qq[pos]) + FLT_MIN;
    ftype ax = FABS(x) + FLT_MIN;

    qq[pos] = x;
    ii[pos] = i;
    pos++;
    if (pos >= cc->nb_samples)
        pos = 0;
    cc->position = pos;

    n = cc->nb_samples;

    empty = (front == back) && (dd[front] == F(0.0));
    PEAKS(F(0.0), >, ax, px)

    dd[back] = ax;

    cc->front = front;
    cc->back = back;
}

static ftype fn(get_sc_delayed)(PeakContext *cc, const int delay)
{
    ftype *qq = cc->queue;
    int idx;
    if (delay < cc->position) {
        idx = cc->position - delay - 1;
    } else {
        idx = cc->nb_samples + cc->position - delay - 1;
    }
    return qq[idx];
}

static ftype fn(get_delayed)(PeakContext *cc, const int delay)
{
    ftype *ii = cc->iqueue;
    int idx;
    if (delay < cc->position) {
        idx = cc->position - delay - 1;
    } else {
        idx = cc->nb_samples + cc->position - delay - 1;
    }
    return ii[idx];
}

static ftype fn(get_peak)(PeakContext *cc)
{
    ftype *dd = cc->dqueue;
    return dd[cc->front];
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioTransientContext *s = ctx->priv;
    AVFrame *sc = s->sc ? s->sc : s->in;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const ftype arange = s->arange;
    const ftype rrange = s->rrange;
    const ftype aratio = s->aratio;
    const ftype rratio = s->rratio;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;

    for (int ch = start; ch < end; ch++) {
        const ftype *sc_src = (const ftype *)sc->extended_data[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        PeakContext *ac = &s->ac[ch], *rc = &s->rc[ch], *dc = &s->dc[ch];
        const int rhold = dc->nb_samples-1;
        const int dhold = ac->nb_samples-1;
        const int delay = rc->nb_samples-1;

        for (int n = 0; n < nb_samples; n++) {
            ftype sc_sample = sc_src[n], sample = src[n];
            ftype ddelayed_sample, ddelayed_sc_sample;
            ftype rdelayed_sample, rdelayed_sc_sample;

            rdelayed_sc_sample = fn(get_sc_delayed)(dc, rhold);
            rdelayed_sample = fn(get_delayed)(dc, rhold);
            fn(queue_sample)(rc, rdelayed_sample, rdelayed_sc_sample);

            ddelayed_sc_sample = fn(get_sc_delayed)(ac, dhold);
            ddelayed_sample = fn(get_delayed)(ac, dhold);
            fn(queue_sample)(dc, ddelayed_sample, ddelayed_sc_sample);

            fn(queue_sample)(ac, sample, sc_sample);

            if (is_disabled) {
                dst[n] = fn(get_delayed)(rc, delay);
            } else {
                ftype factor, apeak, dpeak, rpeak;
                ftype attack, release;

                apeak = fn(get_peak)(ac);
                dpeak = fn(get_peak)(dc);
                rpeak = fn(get_peak)(rc);

                av_assert2(rpeak >= dpeak);
                av_assert2(dpeak >= apeak);

                apeak = LIN2LOG(apeak);
                dpeak = LIN2LOG(dpeak);
                rpeak = LIN2LOG(rpeak);

                attack  = CLIP(aratio * FMAX(dpeak - apeak, F(0.0)), -arange, arange);
                release = CLIP(rratio * FMAX(rpeak - dpeak, F(0.0)), -rrange, rrange);

                factor = LOG2LIN(attack + release);
                dst[n] = fn(get_delayed)(rc, delay) * factor;
            }
        }
    }

    return 0;
}
