/*
 * Copyright (c) 2011 Mans Rullgard <mans@mansr.com>
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

#include <stdint.h>

#include "libavutil/arm/cpu.h"
#include "libavutil/attributes.h"
#include "libavcodec/ac3dsp.h"
#include "config.h"

void ff_ac3_exponent_min_neon(uint8_t *exp, int num_reuse_blocks, int nb_coefs);
void ff_float_to_fixed24_neon(int32_t *dst, const float *src, size_t len);
void ff_ac3_extract_exponents_neon(uint8_t *exp, int32_t *coef, int nb_coefs);
void ff_ac3_sum_square_butterfly_int32_neon(int64_t sum[4],
                                            const int32_t *coef0,
                                            const int32_t *coef1,
                                            int len);
void ff_ac3_sum_square_butterfly_float_neon(float sum[4],
                                            const float *coef0,
                                            const float *coef1,
                                            int len);

void ff_ac3_bit_alloc_calc_bap_armv6(int16_t *mask, int16_t *psd,
                                     int start, int end,
                                     int snr_offset, int floor,
                                     const uint8_t *bap_tab, uint8_t *bap);

void ff_ac3_update_bap_counts_arm(uint16_t mant_cnt[16], uint8_t *bap, int len);

av_cold void ff_ac3dsp_init_arm(AC3DSPContext *c)
{
    int cpu_flags = av_get_cpu_flags();

    c->update_bap_counts         = ff_ac3_update_bap_counts_arm;

    if (have_armv6(cpu_flags)) {
        c->bit_alloc_calc_bap    = ff_ac3_bit_alloc_calc_bap_armv6;
    }

    if (have_neon(cpu_flags)) {
        c->ac3_exponent_min      = ff_ac3_exponent_min_neon;
        c->float_to_fixed24      = ff_float_to_fixed24_neon;
        c->extract_exponents     = ff_ac3_extract_exponents_neon;
        c->sum_square_butterfly_int32 = ff_ac3_sum_square_butterfly_int32_neon;
        c->sum_square_butterfly_float = ff_ac3_sum_square_butterfly_float_neon;
    }
}
