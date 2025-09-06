/*
 * Header file for hardcoded AAC cube-root table
 *
 * Copyright (c) 2010 Reimar Döffinger <Reimar.Doeffinger@gmx.de>
 *
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

#ifndef AVCODEC_CBRT_TABLEGEN_H
#define AVCODEC_CBRT_TABLEGEN_H

#include <assert.h>
#include <stdint.h>
#include <math.h>
#include "libavutil/attributes.h"
#include "libavutil/intfloat.h"
#include "libavcodec/aac_defines.h"
#include "cbrt_data.h"

#if USE_FIXED
#define CBRT(x) lrint((x) * 8192)
#else
#define CBRT(x) av_float2int((float)(x))
#endif

union CBRT AAC_RENAME(ff_cbrt_tab_internal);

av_cold void AAC_RENAME(ff_cbrt_tableinit)(void)
{
    static_assert(2 * sizeof(AAC_RENAME(ff_cbrt_tab_internal).cbrt_tab[0])
                  >= sizeof(AAC_RENAME(ff_cbrt_tab_internal).tmp[0]),
                  "unexpected sizeofs");
    // We reuse ff_cbrt_tab_internal.tmp as a LUT (of doubles) for the roots
    // of the odd integers: tmp[idx] contains (2 * idx + 1)^{4/3}.

    for (int idx = 0; idx < TMP_LUT_SIZE; ++idx)
        AAC_RENAME(ff_cbrt_tab_internal).tmp[idx] = 1;

    /* have to take care of non-squarefree numbers; notice that sqrt(LUT_SIZE) = 90;
     * idx == 44 corresponds to 89. */
    for (int idx = 1; idx < 45; ++idx) {
        if (AAC_RENAME(ff_cbrt_tab_internal).tmp[idx] == 1) {
            int i = 2 * idx + 1;
            double cbrt_val = i * cbrt(i);
            for (int k = i; k < LUT_SIZE; k *= i) {
                // We only have to handle k, 3 * k, 5 * k,...,
                // because only these are odd. The corresponding indices are
                // k >> 1, (k >> 1) + k, (k >> 1) + 2 * k,...
                for (int idx2 = k >> 1; idx2 < TMP_LUT_SIZE; idx2 += k)
                    AAC_RENAME(ff_cbrt_tab_internal).tmp[idx2] *= cbrt_val;
            }
        }
    }

    for (int idx = 45; idx < TMP_LUT_SIZE; ++idx) {
        if (AAC_RENAME(ff_cbrt_tab_internal).tmp[idx] == 1) {
            int i = 2 * idx + 1;
            double cbrt_val = i * cbrt(i);
            for (int idx2 = idx; idx2 < TMP_LUT_SIZE; idx2 += i)
                AAC_RENAME(ff_cbrt_tab_internal).tmp[idx2] *= cbrt_val;
        }
    }

    double cbrt_2 = 2 * cbrt(2);
    for (int idx = TMP_LUT_SIZE - 1; idx >= 0; --idx) {
        double cbrt_val = AAC_RENAME(ff_cbrt_tab_internal).tmp[idx];
        // Due to i * sizeof(ff_cbrt_tab_internal.cbrt_tab[0]) >=
        // 2 * idx * sizeof(ff_cbrt_tab_internal.cbrt_tab[0])  >= idx * sizeof(double)
        // we don't clobber the double-LUT entries with index < idx
        // in the loop below. This is why we process idx in descending order.
        for (int i = 2 * idx + 1; i < LUT_SIZE; i *= 2) {
            AAC_RENAME(ff_cbrt_tab_internal).cbrt_tab[i] = CBRT(cbrt_val);
            cbrt_val *= cbrt_2;
        }
    }
    AAC_RENAME(ff_cbrt_tab_internal).cbrt_tab[0] = CBRT(0);
}

#endif /* AVCODEC_CBRT_TABLEGEN_H */
