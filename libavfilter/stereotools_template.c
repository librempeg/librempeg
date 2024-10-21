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

#undef ATAN
#undef LRINT
#undef FABS
#undef FMAX
#undef FMIN
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ATAN atanf
#define LRINT lrintf
#define FABS fabsf
#define FMIN fminf
#define FMAX fmaxf
#define ftype float
#define SAMPLE_FORMAT flt
#else
#define ATAN atan
#define LRINT lrint
#define FABS fabs
#define FMIN fmin
#define FMAX fmax
#define ftype double
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(do_filter)(AVFilterContext *ctx, AVFrame *out, AVFrame *in)
{
    const int is_disabled = ff_filter_disabled(ctx);
    AVFilterLink *inlink = ctx->inputs[0];
    ftype *dst = (ftype *)out->data[0];
    StereoToolsContext *s = ctx->priv;
    const ftype delay = s->delay;
    int nbuf = LRINT(inlink->sample_rate * (FABS(delay) / F(1000.0)));
    const ftype *src = (const ftype *)in->data[0];
    const ftype phase_sin_coef = s->phase_sin_coef;
    const ftype phase_cos_coef = s->phase_cos_coef;
    const ftype sb = s->base < F(0.0) ? s->base * F(0.5) : s->base;
    const ftype inv_atan_shape = s->inv_atan_shape;
    const ftype sbal = F(1.0) + s->sbal;
    const ftype mpan = F(1.0) + s->mpan;
    const ftype slev = s->slev;
    const ftype mlev = s->mlev;
    const ftype balance_in = s->balance_in;
    const ftype balance_out = s->balance_out;
    const int nb_samples = in->nb_samples;
    const ftype level_in = s->level_in;
    const ftype level_out = s->level_out;
    const ftype sc_level = s->sc_level;
    const int bmode_out = s->bmode_out;
    const int bmode_in = s->bmode_in;
    const int softclip = s->softclip;
    const int mode = s->mode;
    const int phase_l = s->phase_l;
    const int phase_r = s->phase_r;
    const int length = s->length;
    const int mask = length - 1;
    const int mute_l = s->mute_l;
    const int mute_r = s->mute_r;
    ftype *buffer = s->buffer;
    int pos = s->pos;

    nbuf -= nbuf & 1;
    for (int n = 0; n < nb_samples; n++, src += 2, dst += 2) {
        ftype L = src[0], R = src[1], l, r, m, S, gl, gr, gd;

        L *= level_in;
        R *= level_in;

        gl = F(1.0) - FMAX(F(0.0), balance_in);
        gr = F(1.0) + FMIN(F(0.0), balance_in);

        switch (bmode_in) {
        case 1:
            gd = gl - gr;
            gl = F(1.0) + gd;
            gr = F(1.0) - gd;
            break;
        case 2:
            if (balance_in < F(0.0)) {
                gr = FMAX(F(0.5), gr);
                gl = F(1.0) / gr;
            } else if (balance_in > F(0.0)) {
                gl = FMAX(F(0.5), gl);
                gr = F(1.0) / gl;
            }
            break;
        }
        L *= gl;
        R *= gr;

        if (softclip) {
            R = inv_atan_shape * ATAN(R * sc_level);
            L = inv_atan_shape * ATAN(L * sc_level);
        }

        switch (mode) {
        case 0:
            m = (L + R) * F(0.5);
            S = (L - R) * F(0.5);
            l = m * mlev * FMIN(F(1.0), F(2.0) - mpan) + S * slev * FMIN(F(1.0), F(2.0) - sbal);
            r = m * mlev * FMIN(F(1.0),          mpan) - S * slev * FMIN(F(1.0),          sbal);
            L = l;
            R = r;
            break;
        case 1:
            l = L * FMIN(F(1.0), F(2.0) - sbal);
            r = R * FMIN(F(1.0),          sbal);
            L = F(0.5) * (l + r) * mlev;
            R = F(0.5) * (l - r) * slev;
            break;
        case 2:
            l = L * mlev * FMIN(F(1.0), F(2.0) - mpan) + R * slev * FMIN(F(1.0), F(2.0) - sbal);
            r = L * mlev * FMIN(F(1.0),          mpan) - R * slev * FMIN(F(1.0),          sbal);
            L = l;
            R = r;
            break;
        case 3:
            R = L;
            break;
        case 4:
            L = R;
            break;
        case 5:
            L = (L + R) * F(0.5);
            R = L;
            break;
        case 6:
            l = L;
            L = R;
            R = l;
            m = (L + R) * F(0.5);
            S = (L - R) * F(0.5);
            l = m * mlev * FMIN(F(1.0), F(2.0) - mpan) + S * slev * FMIN(F(1.0), F(2.0) - sbal);
            r = m * mlev * FMIN(F(1.0),          mpan) - S * slev * FMIN(F(1.0),          sbal);
            L = l;
            R = r;
            break;
        case 7:
            l = L * mlev * FMIN(F(1.0), F(2.0) - mpan) + R * slev * FMIN(F(1.0), F(2.0) - sbal);
            L = l;
            R = l;
            break;
        case 8:
            r = L * mlev * FMIN(F(1.0),          mpan) - R * slev * FMIN(F(1.0),          sbal);
            L = r;
            R = r;
            break;
        case 9:
            l = L * mlev * FMIN(F(1.0), F(2.0) - mpan) + R * slev * FMIN(F(1.0), F(2.0) - sbal);
            r = L * mlev * FMIN(F(1.0),          mpan) - R * slev * FMIN(F(1.0),          sbal);
            L = r;
            R = l;
            break;
        case 10:
            L = (L - R) * F(0.5);
            R = L;
            break;
        }

        L *= F(1.0) - mute_l;
        R *= F(1.0) - mute_r;

        L *= (F(2.0) * (F(1.0) - phase_l)) - F(1.0);
        R *= (F(2.0) * (F(1.0) - phase_r)) - F(1.0);

        buffer[pos  ] = L;
        buffer[pos+1] = R;

        if (delay > F(0.0)) {
            R = buffer[(pos - nbuf + 1 + length) & mask];
        } else if (delay < F(0.0)) {
            L = buffer[(pos - nbuf + length)     & mask];
        }

        l = L + sb * L - sb * R;
        r = R + sb * R - sb * L;

        L = l;
        R = r;

        l = L * phase_cos_coef - R * phase_sin_coef;
        r = L * phase_sin_coef + R * phase_cos_coef;

        L = l;
        R = r;

        pos = (pos + 2) & mask;

        gl = F(1.0) - FMAX(F(0.0), balance_out);
        gr = F(1.0) + FMIN(F(0.0), balance_out);
        switch (bmode_out) {
        case 1:
            gd = gl - gr;
            gl = F(1.0) + gd;
            gr = F(1.0) - gd;
            break;
        case 2:
            if (balance_out < F(0.0)) {
                gr = FMAX(F(0.5), gr);
                gl = F(1.0) / gr;
            } else if (balance_out > F(0.0)) {
                gl = FMAX(F(0.5), gl);
                gr = F(1.0) / gl;
            }
            break;
        }

        L *= gl;
        R *= gr;

        L *= level_out;
        R *= level_out;

        if (is_disabled) {
            dst[0] = src[0];
            dst[1] = src[1];
        } else {
            dst[0] = L;
            dst[1] = R;
        }
    }

    s->pos = pos;
}
