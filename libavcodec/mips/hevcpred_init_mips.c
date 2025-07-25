/*
 * Copyright (c) 2015 Shivraj Patil (Shivraj.Patil@imgtec.com)
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

#include "libavutil/mips/cpu.h"
#include "config.h"
#include "libavutil/attributes.h"
#include "libavcodec/mips/hevcpred_mips.h"

void ff_hevc_pred_init_mips(HEVCPredContext *c, const int bit_depth)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_msa(cpu_flags)) {
        if (bit_depth == 8) {
            c->intra_pred[2] = ff_intra_pred_8_16x16_msa;
            c->intra_pred[3] = ff_intra_pred_8_32x32_msa;
            c->pred_planar[0] = ff_hevc_intra_pred_planar_0_msa;
            c->pred_planar[1] = ff_hevc_intra_pred_planar_1_msa;
            c->pred_planar[2] = ff_hevc_intra_pred_planar_2_msa;
            c->pred_planar[3] = ff_hevc_intra_pred_planar_3_msa;
            c->pred_dc = ff_hevc_intra_pred_dc_msa;
            c->pred_angular[0] = ff_pred_intra_pred_angular_0_msa;
            c->pred_angular[1] = ff_pred_intra_pred_angular_1_msa;
            c->pred_angular[2] = ff_pred_intra_pred_angular_2_msa;
            c->pred_angular[3] = ff_pred_intra_pred_angular_3_msa;
        }
    }
}
