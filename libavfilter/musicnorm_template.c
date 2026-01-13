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
#undef FEXP
#undef FEXP10
#undef FLOG10
#undef FEPS
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FEXP expf
#define FEXP10 ff_exp10f
#define FLOG10 log10f
#define FEPS FLT_EPSILON
#define SAMPLE_FORMAT fltp
#define ftype float
#else
#define FEXP exp
#define FEXP10 ff_exp10
#define FLOG10 log10
#define FEPS DBL_EPSILON
#define SAMPLE_FORMAT dblp
#define ftype double
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

#define DIP_LUT_MIN_DB   (-100.0)
#define DIP_LUT_MAX_DB   (0.0)
#define DIP_LUT_STEP_DB  (0.25)
#define DIP_LUT_SIZE lrint((DIP_LUT_MAX_DB-DIP_LUT_MIN_DB) / DIP_LUT_STEP_DB + 2)

typedef struct fn(StateContext) {
    ftype fs;
    ftype D, Lc, sigma;
    unsigned lut_size;
    ftype *lut;
    ftype det_a;
    ftype att_a;
    ftype rel_a;
    ftype p;
    ftype g_db;
} fn(StateContext);

static ftype fn(get_coef)(ftype fs, ftype ms)
{
    if (ms <= F(0.0))
        return F(0.0);

    return FEXP(F(-1.0) / (fs * ms * F(0.001)));
}

static void fn(build_lut)(fn(StateContext) *d)
{
    const ftype inv2s2 = F(1.0) / (F(2.0) * d->sigma * d->sigma);

    for (int i = 0; i < d->lut_size; i++) {
        ftype L = DIP_LUT_MIN_DB + i * DIP_LUT_STEP_DB;
        ftype x = L - d->Lc;
        d->lut[i] = d->D * FEXP(-x * x * inv2s2);
        d->lut[i] = (d->lut[i] <= FEPS) ? F(0.0) : d->lut[i];
    }
}

static void fn(set_curve)(fn(StateContext) *d, ftype D, ftype Lc, ftype sigma)
{
    d->D = D;
    d->Lc = Lc;
    d->sigma = (sigma > F(0.1)) ? sigma : F(0.1);

    fn(build_lut)(d);
}

static void fn(musicnorm_uninit)(AVFilterContext *ctx)
{
    MusicNormalizerContext *s = ctx->priv;
    fn(StateContext) *st = s->st;

    for (int ch = 0; ch < s->nb_channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&stc->lut);
    }

    av_freep(&s->st);
}

static int fn(musicnorm_init)(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    MusicNormalizerContext *s = ctx->priv;
    const int nb_channels = outlink->ch_layout.nb_channels;
    const int fs = outlink->sample_rate;
    fn(StateContext) *st;

    if (!s->st)
        s->st = av_calloc(nb_channels, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);

    st = s->st;
    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *stc = &st[ch];

        stc->det_a = fn(get_coef)(fs, s->det_ms);
        stc->att_a = fn(get_coef)(fs, s->attack_ms);
        stc->rel_a = fn(get_coef)(fs, s->release_ms);

        stc->lut_size = DIP_LUT_SIZE;
        if (!stc->lut)
            stc->lut = av_calloc(stc->lut_size, sizeof(*stc->lut));
        if (!stc->lut)
            return AVERROR(ENOMEM);

        fn(set_curve)(stc, s->D, F(20.0) * FLOG10(s->Lc), s->sigma);
    }

    return 0;
}

static ftype fn(static_gain_db)(const fn(StateContext) *d, ftype level_db)
{
    ftype pos = (level_db - DIP_LUT_MIN_DB) * (F(1.0) / DIP_LUT_STEP_DB);
    if (pos <= F(0.0))
        return d->lut[0];
    int i = (int)pos;
    if (i >= d->lut_size - 1)
        return d->lut[d->lut_size - 1];
    ftype f = pos - (ftype)i;
    return d->lut[i] + f * (d->lut[i + 1] - d->lut[i]);
}

static ftype fn(detect_gain)(fn(StateContext) *d, ftype x)
{
    d->p = d->det_a * d->p + (F(1.0) - d->det_a) * (x * x);

    ftype L_db   = F(10.0) * FLOG10(d->p);
    ftype target = fn(static_gain_db)(d, L_db);

    ftype a = (target < d->g_db) ? d->att_a : d->rel_a;
    d->g_db = a * d->g_db + (F(1.0) - a) * target;

    return FEXP10(d->g_db * F(0.05));
}

static int fn(do_musicnorm)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    MusicNormalizerContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];

    for (int n = 0; n < nb_samples; n++) {
        const ftype x = src[n];
        ftype g;

        g = fn(detect_gain)(stc, x);
        dst[n] = disabled ? x : x * g;
    }

    return 0;
}
