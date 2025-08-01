/*
 * This file is part of Librempeg.
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "config.h"

#include <float.h>
#include <stdint.h>

#include "libavfilter/af_afirdsp.h"
#include "libavutil/internal.h"
#include "libavutil/mem_internal.h"
#include "checkasm.h"

#define LEN 256

#define randomize_buffer(buf)                 \
do {                                          \
    int i;                                    \
    double bmg[2], stddev = 10.0, mean = 0.0; \
                                              \
    for (i = 0; i < BUF_SIZE; i += 2) {        \
        av_bmg_get(&checkasm_lfg, bmg);       \
        buf[i]     = bmg[0] * stddev + mean;  \
        buf[i + 1] = bmg[1] * stddev + mean;  \
    }                                         \
} while(0);

static void test_fcmul_add(AudioFIRDSPContext *fir)
{
#define BUF_SIZE LEN*2+8
    LOCAL_ALIGNED_32(float, src0, [BUF_SIZE]);
    LOCAL_ALIGNED_32(float, src1, [BUF_SIZE]);
    LOCAL_ALIGNED_32(float, src2, [BUF_SIZE]);

    randomize_buffer(src0);
    randomize_buffer(src1);
    randomize_buffer(src2);

    if (check_func(fir->fcmul_add, "fcmul_add")) {
        LOCAL_ALIGNED_32(float, cdst, [BUF_SIZE]);
        LOCAL_ALIGNED_32(float, odst, [BUF_SIZE]);
        int i;

        declare_func(void, float *sum, const float *t, const float *c,
                     ptrdiff_t len);

        memcpy(cdst, src0, (BUF_SIZE) * sizeof(float));
        memcpy(odst, src0, (BUF_SIZE) * sizeof(float));
        call_ref(cdst, src1, src2, LEN);
        call_new(odst, src1, src2, LEN);
        for (i = 0; i <= LEN*2; i++) {
            int idx = i & ~1;
            float cre = src2[idx];
            float cim = src2[idx + 1];
            float tre = src1[idx];
            float tim = src1[idx + 1];
            double t = fabs(src0[i]) +
                       fabs(tre) + fabs(tim) + fabs(cre) + fabs(cim) +
                       fabs(tre * cre) + fabs(tim * cim) +
                       fabs(tre * cim) + fabs(tim * cre) +
                       fabs(tre * cre - tim * cim) +
                       fabs(tre * cim + tim * cre) +
                       fabs(cdst[i]) + 1.0;
            if (!float_near_abs_eps(cdst[i], odst[i], t * 2 * FLT_EPSILON)) {
                fprintf(stderr, "%d: %- .12f - %- .12f = % .12g\n",
                        i, cdst[i], odst[i], cdst[i] - odst[i]);
                fail();
                break;
            }
        }
        memcpy(odst, src0, (BUF_SIZE) * sizeof(float));
        bench_new(odst, src1, src2, LEN);
    }

    report("fcmul_add");
}

static void test_dcmul_add(AudioFIRDSPContext *fir)
{
#define BUF_SIZE LEN*2+8
    LOCAL_ALIGNED_32(double, src0, [BUF_SIZE]);
    LOCAL_ALIGNED_32(double, src1, [BUF_SIZE]);
    LOCAL_ALIGNED_32(double, src2, [BUF_SIZE]);

    randomize_buffer(src0);
    randomize_buffer(src1);
    randomize_buffer(src2);

    if (check_func(fir->dcmul_add, "dcmul_add")) {
        LOCAL_ALIGNED_32(double, cdst, [BUF_SIZE]);
        LOCAL_ALIGNED_32(double, odst, [BUF_SIZE]);
        int i;

        declare_func(void, double *sum, const double *t, const double *c,
                     ptrdiff_t len);

        memcpy(cdst, src0, (BUF_SIZE) * sizeof(double));
        memcpy(odst, src0, (BUF_SIZE) * sizeof(double));
        call_ref(cdst, src1, src2, LEN);
        call_new(odst, src1, src2, LEN);
        for (i = 0; i <= LEN*2; i++) {
            int idx = i & ~1;
            double cre = src2[idx];
            double cim = src2[idx + 1];
            double tre = src1[idx];
            double tim = src1[idx + 1];
            double t = fabs(src0[i]) +
                       fabs(tre) + fabs(tim) + fabs(cre) + fabs(cim) +
                       fabs(tre * cre) + fabs(tim * cim) +
                       fabs(tre * cim) + fabs(tim * cre) +
                       fabs(tre * cre - tim * cim) +
                       fabs(tre * cim + tim * cre) +
                       fabs(cdst[i]) + 1.0;
            if (!double_near_abs_eps(cdst[i], odst[i], t * 2 * FLT_EPSILON)) {
                fprintf(stderr, "%d: %- .12f - %- .12f = % .12g\n",
                        i, cdst[i], odst[i], cdst[i] - odst[i]);
                fail();
                break;
            }
        }
        memcpy(odst, src0, (BUF_SIZE) * sizeof(double));
        bench_new(odst, src1, src2, LEN);
    }

    report("dcmul_add");
}


void checkasm_check_afir(void)
{
    AudioFIRDSPContext fir = { 0 };

    ff_afir_init(&fir);
    test_fcmul_add(&fir);
    test_dcmul_add(&fir);
}
