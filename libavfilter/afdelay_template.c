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

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *buffer;
    ftype v0;
    ftype alpha;
    ftype state;
    ftype delay_frac;
    size_t size;
    size_t delay_int;
    unsigned position;
} fn(StateContext);

static void fn(update_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        const int idx = FFMIN(ch, s->nb_delays-1);
        fn(StateContext) *st = &stc[ch];
        const double delay = fmin(s->delays_opt[idx], st->size);

        st->delay_int = lrint(floor(delay));
        st->delay_frac = delay - st->delay_int;
        if (st->delay_frac < F(0.618) && st->delay_int >= 1) {
            st->delay_frac += F(1.0);
            st->delay_int--;
        }
        st->alpha = (F(1.0) - st->delay_frac) / (F(1.0) + st->delay_frac);
    }
}

static int fn(init_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->nb_channels; ch++) {
        const int idx = FFMIN(ch, s->nb_delays-1);
        const double delay = s->delays_opt[idx];
        fn(StateContext) *st = &stc[ch];

        st->size = st->delay_int = lrint(floor(delay));
        st->delay_frac = delay - st->delay_int;
        if (st->delay_frac < F(0.618) && st->delay_int >= 1) {
            st->delay_frac += F(1.0);
            st->delay_int--;
        }
        st->alpha = (F(1.0) - st->delay_frac) / (F(1.0) + st->delay_frac);
        st->buffer = av_calloc(st->delay_int+1, sizeof(*st->buffer));
        if (!st->buffer)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    for (int ch = 0; ch < s->nb_channels && stc; ch++) {
        fn(StateContext) *st = &stc[ch];

        av_freep(&st->buffer);
    }

    av_freep(&s->st);
}

static void fn(filter_channel)(AVFilterContext *ctx, const int nb_samples,
                               const uint8_t *ssrc, uint8_t *ddst, const int ch)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[ch];
    const ftype alpha = st->alpha;
    const ftype *src = (const ftype *)ssrc;
    const size_t delay_int = st->delay_int;
    unsigned position = st->position;
    ftype *buffer = st->buffer;
    ftype *dst = (ftype *)ddst;
    ftype state = st->state;
    ftype v0 = st->v0;

    for (int n = 0; n < nb_samples; n++) {
        const ftype in = src[n];
        ftype v1;

        v1 = buffer[position];
        buffer[position] = in;
        position++;
        if (position >= delay_int)
            position = 0;
        state = v0 + alpha * (v1 - state);
        dst[n] = state;
        v0 = v1;
    }

    st->v0 = v0;
    st->state = state;
    st->position = position;
}
