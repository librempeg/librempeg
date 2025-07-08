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

#undef ftype
#undef FPOW
#undef FABS
#undef FMIN
#undef FMAX
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FPOW powf
#define FABS fabsf
#define FMIN fminf
#define FMAX fmaxf
#define ftype float
#else
#define SAMPLE_FORMAT dblp
#define FPOW pow
#define FABS fabs
#define FMIN fmin
#define FMAX fmax
#define ftype double
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

typedef struct fn(StateContext) {
    ftype *sorted, *cache;

    unsigned filled, idx, size, front, back;

    ftype attack, release, hold, hold_count, current, beta;
} fn(StateContext);

static void fn(envelope_uninit)(AVFilterContext *ctx)
{
    AudioEnvelopeContext *s = ctx->priv;
    fn(StateContext) *st = s->st;

    for (int ch = 0; ch < s->nb_channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&stc->cache);
        av_freep(&stc->sorted);
    }

    av_freep(&s->st);
}

static int fn(envelope_init)(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioEnvelopeContext *s = ctx->priv;
    const int nb_channels = outlink->ch_layout.nb_channels;
    const int sample_rate = outlink->sample_rate;
    fn(StateContext) *st;
    int look;

    look = FFMAX(lrint(s->look * 2.0 * sample_rate), 1);
    s->trim_size = s->flush_size = s->hlook = look / 2;
    s->nb_channels = nb_channels;

    if (!s->st)
        s->st = av_calloc(nb_channels, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);

    st = s->st;
    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *stc = &st[ch];
        const ftype attack = s->attack[FFMIN(ch, s->nb_attack-1)];
        const ftype release = s->release[FFMIN(ch, s->nb_release-1)];
        const ftype hold = s->hold[FFMIN(ch, s->nb_hold-1)];

        if (attack > F(1.0) / sample_rate)
            stc->attack = F(1.0) / (attack * sample_rate);
        else
            stc->attack = F(1.0);

        if (release > F(1.0) / sample_rate)
            stc->release = F(1.0) / (release * sample_rate);
        else
            stc->release = F(1.0);

        if (hold > F(1.0) / sample_rate)
            stc->hold = F(1.0) / (hold * sample_rate);
        else
            stc->hold = F(1.0);

        stc->beta = F(1.0) - FPOW(F(1.0) - stc->attack, s->hlook+1);
        stc->hold_count = F(0.0);
        stc->size = look;
        if (!stc->sorted) {
            stc->sorted = av_calloc(look, sizeof(*stc->sorted));

            if (stc->sorted) {
                for (int n = 0; n < look; n++)
                    stc->sorted[n] = F(-1.0);
            }
        }
        if (!stc->cache) {
            stc->cache = av_calloc(look, sizeof(*stc->cache));

            if (stc->cache) {
                for (int n = 0; n < look; n++)
                    stc->cache[n] = F(-1.0);
            }
        }
        if (!stc->sorted || !stc->cache)
            return AVERROR(ENOMEM);
    }

    return 0;
}

#define PEAKS(empty_value,op,sample, psample)\
    if (!empty && psample == ss[front]) {    \
        ss[front] = empty_value;             \
        if (back != front) {                 \
            front--;                         \
            if (front < 0)                   \
                front = n - 1;               \
        }                                    \
        empty = (front == back) &&           \
                (ss[front] == empty_value);  \
    }                                        \
                                             \
    while (!empty && sample op ss[front]) {  \
        ss[front] = empty_value;             \
        if (back == front) {                 \
            empty = 1;                       \
            break;                           \
        }                                    \
        front--;                             \
        if (front < 0)                       \
            front = n - 1;                   \
    }                                        \
                                             \
    while (!empty && sample op ss[back]) {   \
        ss[back] = empty_value;              \
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

static ftype fn(compute_peak)(ftype *ss, const ftype ax, const ftype px,
                              const int n, int *ffront, int *bback)
{
    const ftype empty_value = F(-1.0);
    int front = *ffront;
    int back = *bback;
    int empty = front == back && ss[front] == empty_value;
    ftype r;

    PEAKS(empty_value, >, ax, px)

    ss[back] = ax;
    r = ss[front];

    *ffront = front;
    *bback = back;

    return r;
}

static int fn(do_envelope)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioEnvelopeContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    const unsigned size = stc->size;
    ftype *sorted = stc->sorted;
    ftype *cache = stc->cache;
    const ftype release = stc->release;
    ftype hold_count = stc->hold_count;
    const ftype attack = stc->attack;
    unsigned filled = stc->filled;
    ftype current = stc->current;
    const ftype beta = stc->beta;
    const ftype hold = stc->hold;
    unsigned front = stc->front;
    unsigned back = stc->back;
    unsigned idx = stc->idx;

    for (int n = 0; n < nb_samples; n++) {
        const ftype r = FABS(src[n]);
        ftype p, prev;

        if (filled < size) {
            if (filled == 0)
                current = r;
            prev = cache[idx];
            cache[idx] = r;
            filled++;
        } else {
            prev = cache[idx];
            cache[idx] = r;
        }
        idx++;
        if (idx >= size)
            idx = 0;

        p = fn(compute_peak)(sorted, r, prev, size, &front, &back);

        if (p > current) {
            current += (p/beta-current) * attack;
            hold_count = F(0.0);
        } else if (p < current) {
            if (hold_count >= F(1.0))
                current -= (current-p) * release;
            else
                hold_count += hold;
        }

        dst[n] = disabled ? src[n] : current;
    }

    stc->hold_count = hold_count;
    stc->current = current;
    stc->filled = filled;
    stc->front = front;
    stc->back = back;
    stc->idx = idx;

    return 0;
}

static int fn(do_envelope_link)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioEnvelopeContext *s = ctx->priv;
    const int nb_channels = s->nb_channels;
    const uint8_t **srce = (const uint8_t **)in->extended_data;
    const ftype *srci = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    const unsigned size = stc->size;
    ftype *sorted = stc->sorted;
    ftype *cache = stc->cache;
    const ftype release = stc->release;
    ftype hold_count = stc->hold_count;
    const ftype attack = stc->attack;
    unsigned filled = stc->filled;
    ftype current = stc->current;
    const ftype beta = stc->beta;
    const ftype hold = stc->hold;
    unsigned front = stc->front;
    unsigned back = stc->back;
    unsigned idx = stc->idx;

    for (int n = 0; n < nb_samples; n++) {
        ftype r = F(0.0), p, prev;

        for (int chi = 0; chi < nb_channels; chi++) {
            const ftype *src = (const ftype *)srce[chi];
            const ftype cr = FABS(src[n]);

            r = FMAX(cr, r);
        }

        if (filled < size) {
            if (filled == 0)
                current = r;
            prev = cache[idx];
            cache[idx] = r;
            filled++;
        } else {
            prev = cache[idx];
            cache[idx] = r;
        }
        idx++;
        if (idx >= size)
            idx = 0;

        p = fn(compute_peak)(sorted, r, prev, size, &front, &back);

        if (p > current) {
            current += (p/beta-current) * attack;
            hold_count = F(0.0);
        } else if (p < current) {
            if (hold_count >= F(1.0))
                current -= (current-p) * release;
            else
                hold_count += hold;
        }

        dst[n] = disabled ? srci[n] : current;
    }

    stc->hold_count = hold_count;
    stc->current = current;
    stc->filled = filled;
    stc->front = front;
    stc->back = back;
    stc->idx = idx;

    return 0;
}
