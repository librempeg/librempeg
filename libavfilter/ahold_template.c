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
#undef SAMPLE_FORMAT

#if DEPTH == 16
#define SAMPLE_FORMAT s16p
#define ftype int16_t
#elif DEPTH == 31
#define SAMPLE_FORMAT s32p
#define ftype int32_t
#elif DEPTH == 32
#define SAMPLE_FORMAT fltp
#define ftype float
#else
#define SAMPLE_FORMAT dblp
#define ftype double
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b) a##_##b
#define fn2(a,b) fn3(a,b)
#define fn(a)    fn2(a, SAMPLE_FORMAT)

typedef struct fn(HoldContext) {
    ftype hold_sample;
    int hold;
} fn(HoldContext);

static int fn(ahold_init)(AVFilterContext *ctx, void **st,
                           const int nb_channels)
{
    fn(HoldContext) *stc;

    st[0] = av_calloc(nb_channels, sizeof(*stc));
    if (!st[0])
        return AVERROR(ENOMEM);

    return 0;
}

static void fn(ahold_channel)(void *state, const void *ibuf, void *obuf,
                              const int nb_samples, const int new_hold,
                              const int ch, const int disabled)
{
    fn(HoldContext) *st = state;
    fn(HoldContext) *stc = &st[ch];
    ftype hold_sample = stc->hold_sample;
    int hold = stc->hold;
    const ftype *src = ibuf;
    ftype *dst = obuf;

    if (hold != new_hold) {
        hold = new_hold;
        hold_sample = src[0];
    }

    if (disabled || !hold) {
        for (int n = 0; n < nb_samples; n++)
            dst[n] = src[n];
    } else {
        for (int n = 0; n < nb_samples; n++)
            dst[n] = hold_sample;
    }

    stc->hold = hold;
    stc->hold_sample = hold_sample;
}
