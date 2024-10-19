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
#undef EPS
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define ftype float
#define EPS FLT_EPSILON
#else
#define SAMPLE_FORMAT dblp
#define ftype double
#define EPS DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b) a##_##b
#define fn2(a,b) fn3(a,b)
#define fn(a)    fn2(a, SAMPLE_FORMAT)

typedef struct fn(NotchContext) {
    ftype k0, c, d, x[2];
} fn(NotchContext);

static int fn(anotch_init)(AVFilterContext *ctx, void **st,
                           const int nb_channels)
{
    fn(NotchContext) *stc;

    st[0] = av_calloc(nb_channels, sizeof(*stc));
    if (!st[0])
        return AVERROR(ENOMEM);

    return 0;
}

static void fn(anotch_channel)(void *state, const void *ibuf, void *obuf,
                               int nb_samples, double factor,
                               int ch, int disabled)
{
    fn(NotchContext) *st = state;
    fn(NotchContext) *stc = &st[ch];
    const ftype f = factor;
    const ftype *src = ibuf;
    ftype *dst = obuf;

    for (int n = 0; n < nb_samples; n++) {
        ftype A, B, C, D, k0, x = src[n];

        B = x + stc->x[1];
        A = F(2.0) * stc->x[0];
        D = f * stc->d + A*A;
        C = f * stc->c + A*B;
        k0 = -C/(D + EPS);

        if (!disabled) {
            dst[n] = stc->k0 * stc->x[0] + k0 * stc->x[0] + stc->x[1] + x;
        } else {
            dst[n] = x;
        }

        stc->x[1] = stc->x[0];
        stc->x[0] = x;
        stc->c = C;
        stc->d = D;
        stc->k0 = k0;
    }
}
