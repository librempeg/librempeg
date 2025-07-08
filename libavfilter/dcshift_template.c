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
#undef IMAX
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ltype int32_t
#define stype int16_t
#define CLIP av_clip_int16
#define IMAX INT16_MAX
#define SAMPLE_FORMAT s16p
#else
#define ltype int64_t
#define stype int32_t
#define CLIP av_clipl_int32
#define IMAX INT32_MAX
#define SAMPLE_FORMAT s32p
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(dc_shift_limiter)(const ltype dcshift, const ltype limitergain,
                                 const ltype threshold,
                                 stype *dst, const stype *src, const int nb_samples)
{
    for (int n = 0; n < nb_samples; n++) {
        ltype d = src[n];

        if (d > threshold && dcshift > 0) {
            d = (d - threshold) * limitergain / (IMAX - threshold) + threshold;
        } else if (d < -threshold && dcshift < 0) {
            d = (d + threshold) * limitergain / (IMAX - threshold) - threshold;
        }

        d += dcshift;
        dst[n] = CLIP(d);
    }
}

static void fn(dc_shift)(const ltype dcshift, stype *dst,
                         const stype *src, const int nb_samples)
{
    for (int n = 0; n < nb_samples; n++)
        dst[n] = CLIP(src[n] + dcshift);
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    DCShiftContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const ltype dcshift = s->dcshift * IMAX;
    const ltype limitergain = IMAX * s->limitergain;
    const ltype threshold = FFMAX(limitergain - FFABS(dcshift), 0);

    for (int ch = start; ch < end; ch++) {
        const stype *src = (const stype *)in->extended_data[ch];
        stype *dst = (stype *)out->extended_data[ch];

        if (limitergain > 0)
            fn(dc_shift_limiter)(dcshift, limitergain, threshold, dst, src, in->nb_samples);
        else
            fn(dc_shift)(dcshift, dst, src, in->nb_samples);
    }

    return 0;
}
