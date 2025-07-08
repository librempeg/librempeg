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

#undef EPS
#undef FABS
#undef FEXP
#undef FLOG
#undef FMA
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define EPS FLT_EPSILON
#define FABS fabsf
#define FEXP exp2f
#define FLOG log2f
#define FMA fmaf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define EPS DBL_EPSILON
#define FABS fabs
#define FEXP exp2
#define FLOG log2
#define FMA fma
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define LIN2LOG(x) (FLOG((x)))
#define LOG2LIN(x) (FEXP((x)))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *sq;
    int nb_samples;
    int position;
    int hold;
    int sign;
    ftype drop;
    ftype step;
    ftype inacc;
    ftype gain;
    ftype lgain;
} fn(StateContext);

typedef struct fn(rtype) {
    ftype s;
    int pos;
} fn(rtype);

static int fn(init_state)(AVFilterContext *ctx)
{
    AudioLimiterContext *s = ctx->priv;
    fn(StateContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        st->nb_samples = 1 << av_ceil_log2(s->l_size);
        st->lgain = F(1.0);
        st->sq = av_calloc(st->nb_samples, sizeof(*st->sq));
        if (!st->sq)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AudioLimiterContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    for (int ch = 0; ch < s->nb_channels && stc; ch++) {
        fn(StateContext) *st = &stc[ch];

        av_freep(&st->sq);
    }

    av_freep(&s->st);
}

static av_always_inline fn(rtype) fn(queue_sample)(const ftype s,
                                                   ftype *sq,
                                                   const int n, int pos)
{
    const ftype prev_s = sq[pos];
    fn(rtype) r;

    sq[pos] = s;
    pos++;
    pos &= n;

    r.s = prev_s;
    r.pos = pos;

    return r;
}

static int fn(filter_channels_link)(AVFilterContext *ctx, AVFrame *out)
{
    AudioLimiterContext *s = ctx->priv;
    AVFrame *sc = s->sc ? s->sc : s->in;
    AVFrame *in = s->in;
    const int nb_channels = in->ch_layout.nb_channels;
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    const ftype release = s->release;
    const ftype attack = s->attack;
    const ftype limit = s->limit;
    const ftype llimit = LIN2LOG(limit);
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[0];
    const int stn = st->nb_samples-1;
    const ftype llevel = s->level ? -llimit : F(0.0);
    const ftype scale = s->level ? F(1.0)/limit : F(1.0);
    const uint8_t **srce = (const uint8_t **)in->extended_data;
    const uint8_t **sce = (const uint8_t **)sc->extended_data;
    uint8_t **dste = (uint8_t **)out->extended_data;
    ftype inacc = st->inacc;
    ftype lgain = st->lgain;
    ftype gain = st->gain;
    ftype drop = st->drop;
    ftype step = st->step;
    int pos = st->position;
    int hold = st->hold;
    int sign = st->sign;

    for (int n = 0; n < nb_samples; n++) {
        ftype max_sc_sample = F(0.0);
        int new_sign;
        fn(rtype) r;

        hold++;
        for (int ch = 0; ch < nb_channels; ch++) {
            const ftype *sc_src = (const ftype *)sce[ch];
            const ftype abs_sample = FABS(sc_src[n]);

            if (abs_sample > max_sc_sample) {
                max_sc_sample = abs_sample;
                new_sign = FFDIFFSIGN(sc_src[n], F(0.0));
            }
        }

        if (new_sign) {
            if (new_sign != sign) {
                hold = 0;
                sign = new_sign;
                drop = gain * release;
            }
        }

        if (inacc <= EPS && gain > EPS) {
            inacc = F(0.0);
            step = F(0.0);

            if (hold == 0) {
                if (gain > drop) {
                    gain -= drop;
                } else {
                    drop = F(0.0);
                    gain = F(0.0);
                }
            } else if (hold > 0) {
                hold--;
                if (hold == 0)
                    drop = gain * release;
            }
        }

        if (max_sc_sample - limit > EPS) {
            const ftype new_gain = LIN2LOG(max_sc_sample)-llimit;
            const ftype new_dgain = new_gain-gain;

            if ((new_dgain > EPS) && (new_dgain - inacc > EPS)) {
                inacc = new_dgain;
                step = FMA(inacc, attack, step);
            }
        }

        gain += step;
        inacc -= step;

        lgain = (gain > EPS) ? LOG2LIN(-gain+llevel) : scale;
        for (int ch = 0; ch < nb_channels; ch++) {
            const ftype *src = (const ftype *)srce[ch];
            ftype *dst = (ftype *)dste[ch];
            fn(StateContext) *st = &stc[ch];
            const ftype sample = src[n];
            ftype *sq = st->sq;

            r = fn(queue_sample)(sample, sq, stn, pos);

            if (is_disabled || lgain == F(1.0)) {
                dst[n] = r.s;
            } else {
                dst[n] = lgain*r.s;
            }
        }

        pos = r.pos;
    }

    st->sign = sign;
    st->hold = hold;
    st->position = pos;
    st->drop = drop;
    st->step = step;
    st->gain = gain;
    st->inacc = inacc;
    st->lgain = lgain;

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioLimiterContext *s = ctx->priv;
    AVFrame *sc = s->sc ? s->sc : s->in;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    const ftype limit = s->limit;
    const ftype llimit = LIN2LOG(limit);
    const ftype release = s->release;
    const ftype attack = s->attack;
    const ftype llevel = s->level ? -llimit : F(0.0);
    const ftype scale = s->level ? F(1.0)/limit : F(1.0);
    fn(StateContext) *stc = s->st;

    for (int ch = start; ch < end; ch++) {
        const ftype *sc_src = (const ftype *)sc->extended_data[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(StateContext) *st = &stc[ch];
        ftype inacc = st->inacc;
        ftype lgain = st->lgain;
        ftype gain = st->gain;
        ftype drop = st->drop;
        ftype step = st->step;
        const int stn = st->nb_samples-1;
        int pos = st->position;
        int sign = st->sign;
        int hold = st->hold;
        ftype *sq = st->sq;

        for (int n = 0; n < nb_samples; n++) {
            const int new_sign = FFDIFFSIGN(sc_src[n], F(0.0));
            const ftype sc_sample = FABS(sc_src[n]);
            const ftype sample = src[n];
            fn(rtype) r;

            hold++;
            if (new_sign) {
                if (new_sign != sign) {
                    hold = 0;
                    sign = new_sign;
                    drop = gain * release;
                }
            }

            r = fn(queue_sample)(sample, sq, stn, pos);
            pos = r.pos;

            if (inacc <= EPS && gain > EPS) {
                inacc = F(0.0);
                step = F(0.0);

                if (hold == 0) {
                    if (gain > drop) {
                        gain -= drop;
                    } else {
                        drop = F(0.0);
                        gain = F(0.0);
                    }
                } else if (hold > 0) {
                    hold--;
                    if (hold == 0)
                        drop = gain * release;
                }
            }

            if (sc_sample - limit > EPS) {
                const ftype new_gain = LIN2LOG(sc_sample)-llimit;
                const ftype new_dgain = new_gain-gain;

                if ((new_dgain > EPS) && (new_dgain - inacc > EPS)) {
                    inacc = new_dgain;
                    step = FMA(inacc, attack, step);
                }
            }

            gain += step;
            inacc -= step;

            lgain = (gain > EPS) ? LOG2LIN(-gain+llevel) : scale;
            if (is_disabled || lgain == F(1.0)) {
                dst[n] = r.s;
            } else {
                dst[n] = lgain*r.s;
            }
        }

        st->sign = sign;
        st->hold = hold;
        st->position = pos;
        st->drop = drop;
        st->step = step;
        st->gain = gain;
        st->inacc = inacc;
        st->lgain = lgain;
    }

    return 0;
}
