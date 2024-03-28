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
#undef cname
#undef SAMPLE_FORMAT
#undef ONE
#undef TWO
#undef SIN
#undef COS
#undef FMOD
#undef MPI
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define ftype float
#define cname s->cf
#define ONE 1.f
#define TWO 2.f
#define SIN sinf
#define COS cosf
#define FMOD fmodf
#define MPI M_PIf
#else
#define SAMPLE_FORMAT dblp
#define ftype double
#define cname s->cd
#define ONE 1.0
#define TWO 2.0
#define SIN sin
#define COS cos
#define FMOD fmod
#define MPI M_PI
#endif

#define cn(x)      s->c##x

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype prev;
    ftype c[4][MAX_NB_COEFFS*2];
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx, int nb_channels)
{
    AFreqShift *s = ctx->priv;
    fn(StateContext) *stc;

    s->state = av_calloc(nb_channels, sizeof(*stc));
    if (!s->state)
        return AVERROR(ENOMEM);
    return 0;
}

static void fn(pfilter_channel)(AVFilterContext *ctx, int ch,
                                AVFrame *in, AVFrame *out)
{
    AFreqShift *s = ctx->priv;
    const int nb_samples = in->nb_samples;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *i1 = stc->c[0];
    ftype *o1 = stc->c[1];
    ftype *i2 = stc->c[2];
    ftype *o2 = stc->c[3];
    const int nb_coeffs = s->nb_coeffs;
    const ftype *c = cname;
    const ftype level = s->level;
    ftype shift = s->shift * MPI;
    ftype cos_theta = COS(shift);
    ftype sin_theta = SIN(shift);
    ftype prev = stc->prev;

    for (int n = 0; n < nb_samples; n++) {
        ftype xn1 = src[n], xn2 = prev;
        ftype I, Q;

        prev = xn1;
        for (int j = 0; j < nb_coeffs; j++) {
            I = c[j] * (xn1 + o2[j]) - i2[j];
            i2[j] = i1[j];
            i1[j] = xn1;
            o2[j] = o1[j];
            o1[j] = I;
            xn1 = I;
        }

        for (int j = nb_coeffs; j < nb_coeffs*2; j++) {
            Q = c[j] * (xn2 + o2[j]) - i2[j];
            i2[j] = i1[j];
            i1[j] = xn2;
            o2[j] = o1[j];
            o1[j] = Q;
            xn2 = Q;
        }

        dst[n] = (I * cos_theta - Q * sin_theta) * level;
    }

    stc->prev = prev;
}

static void fn(ffilter_channel)(AVFilterContext *ctx, int ch,
                                AVFrame *in, AVFrame *out)
{
    AFreqShift *s = ctx->priv;
    const int nb_samples = in->nb_samples;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *i1 = stc->c[0];
    ftype *o1 = stc->c[1];
    ftype *i2 = stc->c[2];
    ftype *o2 = stc->c[3];
    const int nb_coeffs = s->nb_coeffs;
    const ftype *c = cname;
    const ftype level = s->level;
    ftype ts = ONE / in->sample_rate;
    int64_t N = s->in_samples;
    ftype shift = s->shift;
    ftype prev = stc->prev;

    for (int n = 0; n < nb_samples; n++) {
        ftype xn1 = src[n], xn2 = prev;
        ftype I, Q, theta;

        prev = xn1;
        for (int j = 0; j < nb_coeffs; j++) {
            I = c[j] * (xn1 + o2[j]) - i2[j];
            i2[j] = i1[j];
            i1[j] = xn1;
            o2[j] = o1[j];
            o1[j] = I;
            xn1 = I;
        }

        for (int j = nb_coeffs; j < nb_coeffs*2; j++) {
            Q = c[j] * (xn2 + o2[j]) - i2[j];
            i2[j] = i1[j];
            i1[j] = xn2;
            o2[j] = o1[j];
            o1[j] = Q;
            xn2 = Q;
        }

        theta = TWO * MPI * FMOD(shift * (N + n) * ts, ONE);
        dst[n] = (I * COS(theta) - Q * SIN(theta)) * level;
    }

    stc->prev = prev;
}
