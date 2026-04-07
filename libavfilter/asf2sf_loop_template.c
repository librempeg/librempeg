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

#include "avfilter.h"
#include "audio.h"

#define fn3(a,b,c,d,e) a##_##b##_##c##_to_##d##_##e
#define fn2(a,b,c,d,e) fn3(a,b,c,d,e)
#define fn(a)          fn2(a, SRC_P, SRC_F, DST_P, DST_F)

static void fn(sf2sf_loop)(dtype *restrict dst,
                           const stype *restrict src,
                           const int nb_samples,
                           const int nb_channels)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += nb_channels)
        dst[dpidx] = fnc(convert)(src[spidx]);
}

static void fn(sf2sf_loop2)(dtype *restrict dst0,
                            dtype *restrict dst1,
                            const stype *restrict src0,
                            const stype *restrict src1,
                            const int nb_samples)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += 2) {
        dst0[dpidx] = fnc(convert)(src0[spidx]);
        dst1[dpidx] = fnc(convert)(src1[spidx]);
    }
}

static void fn(sf2sf_loop3)(dtype *restrict dst0,
                            dtype *restrict dst1,
                            dtype *restrict dst2,
                            const stype *restrict src0,
                            const stype *restrict src1,
                            const stype *restrict src2,
                            const int nb_samples)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += 3) {
        dst0[dpidx] = fnc(convert)(src0[spidx]);
        dst1[dpidx] = fnc(convert)(src1[spidx]);
        dst2[dpidx] = fnc(convert)(src2[spidx]);
    }
}

static void fn(sf2sf_loop4)(dtype *restrict dst0,
                            dtype *restrict dst1,
                            dtype *restrict dst2,
                            dtype *restrict dst3,
                            const stype *restrict src0,
                            const stype *restrict src1,
                            const stype *restrict src2,
                            const stype *restrict src3,
                            const int nb_samples)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += 4) {
        dst0[dpidx] = fnc(convert)(src0[spidx]);
        dst1[dpidx] = fnc(convert)(src1[spidx]);
        dst2[dpidx] = fnc(convert)(src2[spidx]);
        dst3[dpidx] = fnc(convert)(src3[spidx]);
    }
}

static void fn(sf2sf_loop6)(dtype *restrict dst0,
                            dtype *restrict dst1,
                            dtype *restrict dst2,
                            dtype *restrict dst3,
                            dtype *restrict dst4,
                            dtype *restrict dst5,
                            const stype *restrict src0,
                            const stype *restrict src1,
                            const stype *restrict src2,
                            const stype *restrict src3,
                            const stype *restrict src4,
                            const stype *restrict src5,
                            const int nb_samples)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += 6) {
        dst0[dpidx] = fnc(convert)(src0[spidx]);
        dst1[dpidx] = fnc(convert)(src1[spidx]);
        dst2[dpidx] = fnc(convert)(src2[spidx]);
        dst3[dpidx] = fnc(convert)(src3[spidx]);
        dst4[dpidx] = fnc(convert)(src4[spidx]);
        dst5[dpidx] = fnc(convert)(src5[spidx]);
    }
}

static void fn(sf2sf_loop8)(dtype *restrict dst0,
                            dtype *restrict dst1,
                            dtype *restrict dst2,
                            dtype *restrict dst3,
                            dtype *restrict dst4,
                            dtype *restrict dst5,
                            dtype *restrict dst6,
                            dtype *restrict dst7,
                            const stype *restrict src0,
                            const stype *restrict src1,
                            const stype *restrict src2,
                            const stype *restrict src3,
                            const stype *restrict src4,
                            const stype *restrict src5,
                            const stype *restrict src6,
                            const stype *restrict src7,
                            const int nb_samples)
{
    for (int n = 0, m = 0; n < nb_samples; n++, m += 8) {
        dst0[dpidx] = fnc(convert)(src0[spidx]);
        dst1[dpidx] = fnc(convert)(src1[spidx]);
        dst2[dpidx] = fnc(convert)(src2[spidx]);
        dst3[dpidx] = fnc(convert)(src3[spidx]);
        dst4[dpidx] = fnc(convert)(src4[spidx]);
        dst5[dpidx] = fnc(convert)(src5[spidx]);
        dst6[dpidx] = fnc(convert)(src6[spidx]);
        dst7[dpidx] = fnc(convert)(src7[spidx]);
    }
}

static int fn(sf2sf)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *restrict out = td->out;
    AVFrame *restrict in = td->in;
    const int nb_channels = in->ch_layout.nb_channels;
    const int nb_samples = in->nb_samples;

    if (SRC_p == 1 && DST_p == 1) {
        const int start = (nb_channels * jobnr) / nb_jobs;
        const int end = (nb_channels * (jobnr+1)) / nb_jobs;

        for (int ch = start; ch < end; ch++) {
            const stype *const restrict src = fn_src_ptr(in, ch, 0);
            dtype *restrict dst = fn_dst_ptr(out, ch, 0);

            fn(sf2sf_loop)(dst, src, nb_samples, nb_channels);
        }
    } else {
        const int start = (nb_samples * jobnr) / nb_jobs;
        const int end = (nb_samples * (jobnr+1)) / nb_jobs;

        if (nb_channels == 1) {
            const stype *const restrict src = fn_src_ptr(in, 0, start);
            dtype *restrict dst = fn_dst_ptr(out, 0, start);

            fn(sf2sf_loop)(dst, src, end-start, 1);
        } else if (nb_channels == 2) {
            const stype *const restrict src0 = fn_src_ptr(in, 0, start);
            const stype *const restrict src1 = fn_src_ptr(in, 1, start);
            dtype *restrict dst0 = fn_dst_ptr(out, 0, start);
            dtype *restrict dst1 = fn_dst_ptr(out, 1, start);

            fn(sf2sf_loop2)(dst0, dst1, src0, src1, end-start);
        } else if (nb_channels == 3) {
            const stype *const restrict src0 = fn_src_ptr(in, 0, start);
            const stype *const restrict src1 = fn_src_ptr(in, 1, start);
            const stype *const restrict src2 = fn_src_ptr(in, 2, start);
            dtype *restrict dst0 = fn_dst_ptr(out, 0, start);
            dtype *restrict dst1 = fn_dst_ptr(out, 1, start);
            dtype *restrict dst2 = fn_dst_ptr(out, 2, start);

            fn(sf2sf_loop3)(dst0, dst1, dst2, src0, src1, src2, end-start);
        } else if (nb_channels == 4) {
            const stype *const restrict src0 = fn_src_ptr(in, 0, start);
            const stype *const restrict src1 = fn_src_ptr(in, 1, start);
            const stype *const restrict src2 = fn_src_ptr(in, 2, start);
            const stype *const restrict src3 = fn_src_ptr(in, 3, start);
            dtype *restrict dst0 = fn_dst_ptr(out, 0, start);
            dtype *restrict dst1 = fn_dst_ptr(out, 1, start);
            dtype *restrict dst2 = fn_dst_ptr(out, 2, start);
            dtype *restrict dst3 = fn_dst_ptr(out, 3, start);

            fn(sf2sf_loop4)(dst0, dst1, dst2, dst3,
                            src0, src1, src2, src3, end-start);
        } else if (nb_channels == 6) {
            const stype *const restrict src0 = fn_src_ptr(in, 0, start);
            const stype *const restrict src1 = fn_src_ptr(in, 1, start);
            const stype *const restrict src2 = fn_src_ptr(in, 2, start);
            const stype *const restrict src3 = fn_src_ptr(in, 3, start);
            const stype *const restrict src4 = fn_src_ptr(in, 4, start);
            const stype *const restrict src5 = fn_src_ptr(in, 5, start);
            dtype *restrict dst0 = fn_dst_ptr(out, 0, start);
            dtype *restrict dst1 = fn_dst_ptr(out, 1, start);
            dtype *restrict dst2 = fn_dst_ptr(out, 2, start);
            dtype *restrict dst3 = fn_dst_ptr(out, 3, start);
            dtype *restrict dst4 = fn_dst_ptr(out, 4, start);
            dtype *restrict dst5 = fn_dst_ptr(out, 5, start);

            fn(sf2sf_loop6)(dst0, dst1, dst2, dst3, dst4, dst5,
                            src0, src1, src2, src3, src4, src5,
                            end-start);
        } else if (nb_channels == 8) {
            const stype *const restrict src0 = fn_src_ptr(in, 0, start);
            const stype *const restrict src1 = fn_src_ptr(in, 1, start);
            const stype *const restrict src2 = fn_src_ptr(in, 2, start);
            const stype *const restrict src3 = fn_src_ptr(in, 3, start);
            const stype *const restrict src4 = fn_src_ptr(in, 4, start);
            const stype *const restrict src5 = fn_src_ptr(in, 5, start);
            const stype *const restrict src6 = fn_src_ptr(in, 6, start);
            const stype *const restrict src7 = fn_src_ptr(in, 7, start);
            dtype *restrict dst0 = fn_dst_ptr(out, 0, start);
            dtype *restrict dst1 = fn_dst_ptr(out, 1, start);
            dtype *restrict dst2 = fn_dst_ptr(out, 2, start);
            dtype *restrict dst3 = fn_dst_ptr(out, 3, start);
            dtype *restrict dst4 = fn_dst_ptr(out, 4, start);
            dtype *restrict dst5 = fn_dst_ptr(out, 5, start);
            dtype *restrict dst6 = fn_dst_ptr(out, 6, start);
            dtype *restrict dst7 = fn_dst_ptr(out, 7, start);

            fn(sf2sf_loop8)(dst0, dst1, dst2, dst3, dst4, dst5, dst6, dst7,
                            src0, src1, src2, src3, src4, src5, src6, src7,
                            end-start);
        } else {
            for (int ch = 0; ch < nb_channels; ch++) {
                const stype *const restrict src = fn_src_ptr(in, ch, start);
                dtype *restrict dst = fn_dst_ptr(out, ch, start);

                fn(sf2sf_loop)(dst, src, end-start, nb_channels);
            }
        }
    }

    return 0;
}
