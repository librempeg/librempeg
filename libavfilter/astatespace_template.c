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
#undef SQRT
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SQRT sqrtf
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SQRT sqrt
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype b[5];
    ftype z[2];

    ftype w;
    ftype theta;
    ftype sigma;
    ftype zeta;
} fn(StateContext);

static ftype fn(warp_sigma)(const int order, const ftype w)
{
    switch (order) {
    case 1:
        return F(0.40824999) * (F(0.05843357) - w * w) / (F(0.04593294) - w * w);
    case 2:
        return F(0.57735268) * (F(0.11686715) - w * w) / (F(0.09186588) - w * w);
    default:
        return F(0.0);
    }
}

static ftype fn(v1)(const ftype x, const ftype sigma)
{
    return SQRT(x + sigma * sigma);
}

static ftype fn(v2)(const ftype x, const ftype y, const ftype sigma_sq)
{
    const ftype t = x * (y + y) - x;

    return SQRT(x * x + sigma_sq * (t + t + sigma_sq));
}

static ftype fn(k2)(const ftype x, const ftype y, const ftype sigma_sq)
{
    const ftype t = x * (y + y) - x;

    return t + sigma_sq;
}

static void fn(update_blocks)(fn(StateContext) *st,
                              const int filter_type, const int order)
{
    const ftype sigma = st->sigma;
    const ftype zeta = st->zeta;
    const ftype w = st->w;
    ftype *b = st->b;

    switch (order) {
    case 1:
        switch (filter_type) {
        case highshelf:
            b[0] = F(1.0) / (F(0.5) + fn(v1)(w * w / b[2], sigma));
            break;
        case lowshelf:
            b[0] = F(1.0) / (F(0.5) + fn(v1)(w * w * b[2], sigma));
            break;
        default:
            b[0] = F(1.0) / (F(0.5) + fn(v1)(w * w, sigma));
            break;
        }
        switch (filter_type) {
        case lowpass:
            b[1] = F(0.5) + sigma;
            break;
        case highshelf:
            b[1] = F(1.0) / (F(0.5) + fn(v1)(w * w * b[2], sigma));
            break;
        case lowshelf:
            b[1] = F(1.0) / (F(0.5) + fn(v1)(w * w / b[2], sigma));
            break;
        case allpass:
            b[1] = F(0.5) - fn(v1)(w * w, sigma);
            break;
        default:
            b[1] = w;
            break;
        }
        break;
    case 2:
        switch (filter_type) {
        case allpass:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = b[1];
                b[3] = F(0.5) + v - SQRT(v + k);
            }
            break;
        case lowpass:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * sigma_sq / b[1];
                b[3] = F(0.5) + sigma_sq + F(M_SQRT2) * sigma;
            }
            break;
        case highpass:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * w_sq / b[1];
                b[3] = w_sq;
            }
            break;
        case bandpass:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(4.0) * w * zeta * sigma / b[1];
                b[3] = F(2.0) * w * zeta * (sigma + F(M_SQRT1_2));
            }
            break;
        case bandreject:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * (w_sq - sigma_sq) / b[1];
                b[3] = F(0.5) + w_sq - sigma_sq;
            }
            break;
        case lowshelf:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq * SQRT(b[4]), zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq * SQRT(b[4]), zeta_sq, sigma_sq);
                const ftype v_a = fn(v2)(w_sq / b[4], zeta_sq, sigma_sq);
                const ftype k_a = fn(k2)(w_sq / b[4], zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * v_a / b[1];
                b[3] = F(0.5) + v_a + SQRT(v_a + k_a);
            }
            break;
        case highshelf:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq / SQRT(b[4]), zeta_sq, sigma_sq);
                const ftype k = fn(k2)(w_sq / SQRT(b[4]), zeta_sq, sigma_sq);
                const ftype v_a = fn(v2)(w_sq * b[4], zeta_sq, sigma_sq);
                const ftype k_a = fn(k2)(w_sq * b[4], zeta_sq, sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * v_a / b[1];
                b[3] = F(0.5) + v_a + SQRT(v_a + k_a);
            }
            break;
        case equalizer:
            {
                const ftype w_sq = w * w;
                const ftype sigma_sq = sigma * sigma;
                const ftype zeta_sq = zeta * zeta;
                const ftype v = fn(v2)(w_sq, zeta_sq / b[4], sigma_sq);
                const ftype k = fn(k2)(w_sq, zeta_sq / b[4], sigma_sq);
                const ftype v_a = fn(v2)(w_sq, zeta_sq * b[4], sigma_sq);
                const ftype k_a = fn(k2)(w_sq, zeta_sq * b[4], sigma_sq);

                b[0] = F(1.0) / (v + SQRT(v + k) + F(0.5));
                b[1] = SQRT(v + v);
                b[2] = F(2.0) * v_a / b[1];
                b[3] = F(0.5) + v_a + SQRT(v_a + k_a);
            }
            break;
        }
        break;
    }
}

static void fn(set_fc)(AVFilterContext *ctx, const double new_fc)
{
    StateSpaceContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    const int nb_channels = s->nb_channels;
    const int filter_type = s->filter_type;
    const int order = s->order;
    const int warp = s->warp;
    const ftype fs = s->fs;
    const ftype x = new_fc;
    const ftype y = (order <= 1) ? F(2.0) : SQRT(F(2.0));
    const ftype z = (order <= 1) ? F(1.0) : SQRT(F(2.0));

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        st->w = fs / (y * F(M_PI) * x);
        if (warp && st->w > z * F(1.0/M_PI))
            st->sigma = fn(warp_sigma)(order, st->w);
        fn(update_blocks)(st, filter_type, order);
    }
}

static void fn(set_gain)(AVFilterContext *ctx, const double gain)
{
    StateSpaceContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    const int nb_channels = s->nb_channels;
    const int filter_type = s->filter_type;
    const int order = s->order;
    const int idx = (order <= 1) ? 2 : 4;

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        st->b[idx] = gain;
        fn(update_blocks)(st, filter_type, order);
    }
}

static void fn(set_sigma)(AVFilterContext *ctx, const double S)
{
    StateSpaceContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    const int nb_channels = s->nb_channels;
    const int filter_type = s->filter_type;
    const int order = s->order;
    const ftype z = (order <= 1) ? F(1.0) : SQRT(F(2.0));

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        st->sigma = S / (z * F(M_PI));
        fn(update_blocks)(st, filter_type, order);
    }
}

static void fn(set_damping)(AVFilterContext *ctx, const double x)
{
    StateSpaceContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    const int nb_channels = s->nb_channels;
    const int filter_type = s->filter_type;
    const int order = s->order;
    const ftype zeta = x;

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        st->zeta = zeta;
        fn(update_blocks)(st, filter_type, order);
    }
}

static void fn(highpass1)(void *st, const void *input, void *output, const int len,
                          const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype z = stc->z[0];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z) * b[0];
        const ftype y = theta * b[1];

        z = z + theta;

        dst[n] = y;
    }

    z = isnormal(z) ? z : F(0.0);
    stc->z[0] = z;
}

static void fn(lowshelf1)(void *st, const void *input, void *output, const int len,
                          const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype z = stc->z[0];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z) * b[0];
        const ftype y = theta * b[1] + z;

        z = z + theta;

        dst[n] = y * b[2];
    }

    z = isnormal(z) ? z : F(0.0);
    stc->z[0] = z;
}

static void fn(lowpass1)(void *st, const void *input, void *output, const int len,
                         const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype z = stc->z[0];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z) * b[0];
        const ftype y = theta * b[1] + z;

        z = z + theta;

        dst[n] = y;
    }

    z = isnormal(z) ? z : F(0.0);
    stc->z[0] = z;
}

static void fn(lowshelf2)(void *st, const void *input, void *output, const int len,
                          const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype *z = stc->z;
    ftype z0 = z[0];
    ftype z1 = z[1];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z0 - z1 * b[1]) * b[0];
        const ftype y = theta * b[3] + z1 * b[2] + z0;

        z0 = z0 + theta;
        z1 = -z1 - theta * b[1];
        dst[n] = y * b[4];
    }

    z[0] = isnormal(z0) ? z0 : F(0.0);
    z[1] = isnormal(z1) ? z1 : F(0.0);
}

static void fn(highpass2)(void *st, const void *input, void *output, const int len,
                          const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype *z = stc->z;
    ftype z0 = z[0];
    ftype z1 = z[1];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z0 - z1 * b[1]) * b[0];
        const ftype y = theta * b[3] + z1 * b[2];

        z0 = z0 + theta;
        z1 = -z1 - theta * b[1];
        dst[n] = y;
    }

    z[0] = isnormal(z0) ? z0 : F(0.0);
    z[1] = isnormal(z1) ? z1 : F(0.0);
}

static void fn(lowpass2)(void *st, const void *input, void *output, const int len,
                         const int ch, const int disabled)
{
    const ftype *restrict src = input;
    ftype *restrict dst = output;
    fn(StateContext) *state = st;
    fn(StateContext) *stc = &state[ch];
    const ftype *b = stc->b;
    ftype *z = stc->z;
    ftype z0 = z[0];
    ftype z1 = z[1];

    for (int n = 0; n < len; n++) {
        const ftype x = src[n];
        const ftype theta = (x - z0 - z1 * b[1]) * b[0];
        const ftype y = theta * b[3] + z1 * b[2] + z0;

        z0 = z0 + theta;
        z1 = -z1 - theta * b[1];
        dst[n] = y;
    }

    z[0] = isnormal(z0) ? z0 : F(0.0);
    z[1] = isnormal(z1) ? z1 : F(0.0);
}

static int fn(init_statespace)(AVFilterContext *ctx, void **st,
                               const int nb_channels)
{
    StateSpaceContext *s = ctx->priv;
    const int filter_type = s->filter_type;
    const int order = s->order;
    fn(StateContext) *stc;

    if (!st[0])
        st[0] = av_calloc(nb_channels, sizeof(*stc));
    if (!st[0])
        return AVERROR(ENOMEM);

    s->set_fc = fn(set_fc);
    s->set_gain = fn(set_gain);
    s->set_sigma = fn(set_sigma);
    s->set_damping = fn(set_damping);

    switch (order) {
    case 1:
        switch (filter_type) {
        case lowshelf:
            s->filter = fn(lowshelf1);
            break;
        case highpass:
            s->filter = fn(highpass1);
            break;
        default:
            s->filter = fn(lowpass1);
            break;
        }
        break;
    case 2:
        switch (filter_type) {
        case lowshelf:
            s->filter = fn(lowshelf2);
            break;
        case highpass:
        case bandpass:
            s->filter = fn(highpass2);
            break;
        default:
            s->filter = fn(lowpass2);
            break;
        }
        break;
    default:
        return AVERROR_BUG;
    }

    s->set_fc(ctx, s->fc);
    s->set_gain(ctx, s->gain);
    s->set_sigma(ctx, s->S);
    s->set_damping(ctx, s->zeta);

    return 0;
}
