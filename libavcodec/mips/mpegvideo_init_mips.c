/*
 * Copyright (c) 2015 Manojkumar Bhosale (Manojkumar.Bhosale@imgtec.com)
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

#include "libavutil/attributes.h"
#include "libavutil/mips/cpu.h"
#include "libavcodec/mpegvideo_unquantize.h"
#include "h263dsp_mips.h"
#include "mpegvideo_mips.h"

av_cold void ff_mpv_unquantize_init_mips(MPVUnquantDSPContext *s,
                                         int bitexact, int q_scale_type)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_mmi(cpu_flags)) {
        s->dct_unquantize_h263_intra = ff_dct_unquantize_h263_intra_mmi;
        s->dct_unquantize_h263_inter = ff_dct_unquantize_h263_inter_mmi;
        s->dct_unquantize_mpeg1_intra = ff_dct_unquantize_mpeg1_intra_mmi;
        s->dct_unquantize_mpeg1_inter = ff_dct_unquantize_mpeg1_inter_mmi;

        if (!bitexact)
            if (!q_scale_type)
                s->dct_unquantize_mpeg2_intra = ff_dct_unquantize_mpeg2_intra_mmi;
    }

    if (have_msa(cpu_flags)) {
        s->dct_unquantize_h263_intra = ff_dct_unquantize_h263_intra_msa;
        s->dct_unquantize_h263_inter = ff_dct_unquantize_h263_inter_msa;
        if (!q_scale_type)
            s->dct_unquantize_mpeg2_inter = ff_dct_unquantize_mpeg2_inter_msa;
    }
}
