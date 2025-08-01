/*
 * Copyright (c) 2016 Zhou Xiaoyong <zhouxiaoyong@loongson.cn>
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
#include "libavutil/attributes.h"
#include "libavcodec/vc1dsp.h"
#include "vc1dsp_mips.h"
#include "config.h"

#define FN_ASSIGN(OP, X, Y, INSN) \
    dsp->OP##vc1_mspel_pixels_tab[1][X+4*Y] = ff_##OP##vc1_mspel_mc##X##Y##INSN; \
    dsp->OP##vc1_mspel_pixels_tab[0][X+4*Y] = ff_##OP##vc1_mspel_mc##X##Y##_16##INSN

av_cold void ff_vc1dsp_init_mips(VC1DSPContext *dsp)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_mmi(cpu_flags)) {
 #if _MIPS_SIM != _ABIO32
        dsp->vc1_inv_trans_8x8    = ff_vc1_inv_trans_8x8_mmi;
        dsp->vc1_inv_trans_4x8    = ff_vc1_inv_trans_4x8_mmi;
        dsp->vc1_inv_trans_8x4    = ff_vc1_inv_trans_8x4_mmi;
#endif
        dsp->vc1_inv_trans_4x4    = ff_vc1_inv_trans_4x4_mmi;
        dsp->vc1_inv_trans_8x8_dc = ff_vc1_inv_trans_8x8_dc_mmi;
        dsp->vc1_inv_trans_4x8_dc = ff_vc1_inv_trans_4x8_dc_mmi;
        dsp->vc1_inv_trans_8x4_dc = ff_vc1_inv_trans_8x4_dc_mmi;
        dsp->vc1_inv_trans_4x4_dc = ff_vc1_inv_trans_4x4_dc_mmi;

        dsp->vc1_h_overlap        = ff_vc1_h_overlap_mmi;
        dsp->vc1_v_overlap        = ff_vc1_v_overlap_mmi;
        dsp->vc1_h_s_overlap      = ff_vc1_h_s_overlap_mmi;
        dsp->vc1_v_s_overlap      = ff_vc1_v_s_overlap_mmi;

        dsp->vc1_v_loop_filter4  = ff_vc1_v_loop_filter4_mmi;
        dsp->vc1_h_loop_filter4  = ff_vc1_h_loop_filter4_mmi;
        dsp->vc1_v_loop_filter8  = ff_vc1_v_loop_filter8_mmi;
        dsp->vc1_h_loop_filter8  = ff_vc1_h_loop_filter8_mmi;
        dsp->vc1_v_loop_filter16 = ff_vc1_v_loop_filter16_mmi;
        dsp->vc1_h_loop_filter16 = ff_vc1_h_loop_filter16_mmi;

        FN_ASSIGN(put_, 0, 0, _mmi);
        FN_ASSIGN(put_, 0, 1, _mmi);
        FN_ASSIGN(put_, 0, 2, _mmi);
        FN_ASSIGN(put_, 0, 3, _mmi);

        FN_ASSIGN(put_, 1, 0, _mmi);
        //FN_ASSIGN(put_, 1, 1, _mmi);//FIXME
        //FN_ASSIGN(put_, 1, 2, _mmi);//FIXME
        //FN_ASSIGN(put_, 1, 3, _mmi);//FIXME

        FN_ASSIGN(put_, 2, 0, _mmi);
        //FN_ASSIGN(put_, 2, 1, _mmi);//FIXME
        //FN_ASSIGN(put_, 2, 2, _mmi);//FIXME
        //FN_ASSIGN(put_, 2, 3, _mmi);//FIXME

        FN_ASSIGN(put_, 3, 0, _mmi);
        //FN_ASSIGN(put_, 3, 1, _mmi);//FIXME
        //FN_ASSIGN(put_, 3, 2, _mmi);//FIXME
        //FN_ASSIGN(put_, 3, 3, _mmi);//FIXME

        FN_ASSIGN(avg_, 0, 0, _mmi);
        FN_ASSIGN(avg_, 0, 1, _mmi);
        FN_ASSIGN(avg_, 0, 2, _mmi);
        FN_ASSIGN(avg_, 0, 3, _mmi);

        FN_ASSIGN(avg_, 1, 0, _mmi);
        //FN_ASSIGN(avg_, 1, 1, _mmi);//FIXME
        //FN_ASSIGN(avg_, 1, 2, _mmi);//FIXME
        //FN_ASSIGN(avg_, 1, 3, _mmi);//FIXME

        FN_ASSIGN(avg_, 2, 0, _mmi);
        //FN_ASSIGN(avg_, 2, 1, _mmi);//FIXME
        //FN_ASSIGN(avg_, 2, 2, _mmi);//FIXME
        //FN_ASSIGN(avg_, 2, 3, _mmi);//FIXME

        FN_ASSIGN(avg_, 3, 0, _mmi);
        //FN_ASSIGN(avg_, 3, 1, _mmi);//FIXME
        //FN_ASSIGN(avg_, 3, 2, _mmi);//FIXME
        //FN_ASSIGN(avg_, 3, 3, _mmi);//FIXME

        dsp->put_no_rnd_vc1_chroma_pixels_tab[0] = ff_put_no_rnd_vc1_chroma_mc8_mmi;
        dsp->avg_no_rnd_vc1_chroma_pixels_tab[0] = ff_avg_no_rnd_vc1_chroma_mc8_mmi;
        dsp->put_no_rnd_vc1_chroma_pixels_tab[1] = ff_put_no_rnd_vc1_chroma_mc4_mmi;
        dsp->avg_no_rnd_vc1_chroma_pixels_tab[1] = ff_avg_no_rnd_vc1_chroma_mc4_mmi;
    }

    if (have_msa(cpu_flags)) {
        dsp->vc1_inv_trans_8x8 = ff_vc1_inv_trans_8x8_msa;
        dsp->vc1_inv_trans_4x8 = ff_vc1_inv_trans_4x8_msa;
        dsp->vc1_inv_trans_8x4 = ff_vc1_inv_trans_8x4_msa;

        FN_ASSIGN(put_, 1, 1, _msa);
        FN_ASSIGN(put_, 1, 2, _msa);
        FN_ASSIGN(put_, 1, 3, _msa);
        FN_ASSIGN(put_, 2, 1, _msa);
        FN_ASSIGN(put_, 2, 2, _msa);
        FN_ASSIGN(put_, 2, 3, _msa);
        FN_ASSIGN(put_, 3, 1, _msa);
        FN_ASSIGN(put_, 3, 2, _msa);
        FN_ASSIGN(put_, 3, 3, _msa);
    }
}
