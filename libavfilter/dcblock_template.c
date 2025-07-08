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

#undef ltype
#undef stype
#undef CLIP
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ltype int32_t
#define stype int16_t
#define CLIP av_clip_int16
#define SAMPLE_FORMAT s16p
#else
#define ltype int64_t
#define stype int32_t
#define CLIP av_clipl_int32
#define SAMPLE_FORMAT s32p
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ltype A;
    ltype acc;
    ltype prev_x, prev_y;
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    DCBlockContext *s = ctx->priv;
    fn(StateContext) *stc;

    if (!s->st)
        s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];
        double pole = exp(-s->cut * 2.0 * M_PI / inlink->sample_rate);

        st->A = llrint((1LL<<(DEPTH-1))*(1.0-pole));
    }

    return 0;
}

static void fn(dc_block)(fn(StateContext) *st, stype *dst,
                         const stype *src, const int nb_samples)
{
    const ltype A = st->A;
    ltype prev_y = st->prev_y;
    ltype prev_x = st->prev_x;
    ltype acc = st->acc;

    for (int n = 0; n < nb_samples; n++) {
        acc -= prev_x;
        prev_x = (ltype)src[n] * (ltype)(1LL << (DEPTH-1));
        acc += prev_x;
        acc -= A * prev_y;
        prev_y = acc >> (DEPTH-1);
        dst[n] = CLIP(prev_y);
    }

    st->prev_y = prev_y;
    st->prev_x = prev_x;
    st->acc = acc;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    DCBlockContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    fn(StateContext) *stc = s->st;

    for (int ch = start; ch < end; ch++) {
        const stype *src = (const stype *)in->extended_data[ch];
        fn(StateContext) *st = &stc[ch];
        stype *dst = (stype *)out->extended_data[ch];

        fn(dc_block)(st, dst, src, in->nb_samples);
    }

    return 0;
}
