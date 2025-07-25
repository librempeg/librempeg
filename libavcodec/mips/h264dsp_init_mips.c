/*
 * Copyright (c) 2015 Parag Salasakar (Parag.Salasakar@imgtec.com)
 * Copyright (c) 2015 Zhou Xiaoyong <zhouxiaoyong@loongson.cn>
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
#include "h264dsp_mips.h"

av_cold void ff_h264dsp_init_mips(H264DSPContext *c, const int bit_depth,
                                  const int chroma_format_idc)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_mmi(cpu_flags)) {
        if (bit_depth == 8) {
            c->h264_add_pixels4_clear = ff_h264_add_pixels4_8_mmi;
            c->h264_idct_add = ff_h264_idct_add_8_mmi;
            c->h264_idct8_add = ff_h264_idct8_add_8_mmi;
            c->h264_idct_dc_add = ff_h264_idct_dc_add_8_mmi;
            c->h264_idct8_dc_add = ff_h264_idct8_dc_add_8_mmi;
            c->h264_idct_add16 = ff_h264_idct_add16_8_mmi;
            c->h264_idct_add16intra = ff_h264_idct_add16intra_8_mmi;
            c->h264_idct8_add4 = ff_h264_idct8_add4_8_mmi;

            if (chroma_format_idc <= 1)
                c->h264_idct_add8 = ff_h264_idct_add8_8_mmi;
            else
                c->h264_idct_add8 = ff_h264_idct_add8_422_8_mmi;

            c->h264_luma_dc_dequant_idct = ff_h264_luma_dc_dequant_idct_8_mmi;

            c->weight_h264_pixels_tab[0] = ff_h264_weight_pixels16_8_mmi;
            c->weight_h264_pixels_tab[1] = ff_h264_weight_pixels8_8_mmi;
            c->weight_h264_pixels_tab[2] = ff_h264_weight_pixels4_8_mmi;

            c->biweight_h264_pixels_tab[0] = ff_h264_biweight_pixels16_8_mmi;
            c->biweight_h264_pixels_tab[1] = ff_h264_biweight_pixels8_8_mmi;
            c->biweight_h264_pixels_tab[2] = ff_h264_biweight_pixels4_8_mmi;

            c->h264_v_loop_filter_chroma       = ff_deblock_v_chroma_8_mmi;
            c->h264_v_loop_filter_chroma_intra = ff_deblock_v_chroma_intra_8_mmi;

            if (chroma_format_idc <= 1) {
                c->h264_h_loop_filter_chroma =
                    ff_deblock_h_chroma_8_mmi;
                c->h264_h_loop_filter_chroma_intra =
                    ff_deblock_h_chroma_intra_8_mmi;
            }

            c->h264_v_loop_filter_luma = ff_deblock_v_luma_8_mmi;
            c->h264_v_loop_filter_luma_intra = ff_deblock_v_luma_intra_8_mmi;
            c->h264_h_loop_filter_luma = ff_deblock_h_luma_8_mmi;
            c->h264_h_loop_filter_luma_intra = ff_deblock_h_luma_intra_8_mmi;
        }
    }

    if (have_msa(cpu_flags)) {
        if (chroma_format_idc <= 1)
            c->h264_loop_filter_strength = ff_h264_loop_filter_strength_msa;
        if (bit_depth == 8) {
            c->h264_v_loop_filter_luma = ff_h264_v_lpf_luma_inter_msa;
            c->h264_h_loop_filter_luma = ff_h264_h_lpf_luma_inter_msa;
            c->h264_h_loop_filter_luma_mbaff =
                ff_h264_h_loop_filter_luma_mbaff_msa;
            c->h264_v_loop_filter_luma_intra = ff_h264_v_lpf_luma_intra_msa;
            c->h264_h_loop_filter_luma_intra = ff_h264_h_lpf_luma_intra_msa;
            c->h264_h_loop_filter_luma_mbaff_intra =
                ff_h264_h_loop_filter_luma_mbaff_intra_msa;
            c->h264_v_loop_filter_chroma = ff_h264_v_lpf_chroma_inter_msa;

            if (chroma_format_idc <= 1)
                c->h264_h_loop_filter_chroma = ff_h264_h_lpf_chroma_inter_msa;
            else
                c->h264_h_loop_filter_chroma =
                    ff_h264_h_loop_filter_chroma422_msa;

            if (chroma_format_idc > 1)
                c->h264_h_loop_filter_chroma_mbaff =
                    ff_h264_h_loop_filter_chroma422_mbaff_msa;

            c->h264_v_loop_filter_chroma_intra =
                ff_h264_v_lpf_chroma_intra_msa;

            if (chroma_format_idc <= 1)
                c->h264_h_loop_filter_chroma_intra =
                    ff_h264_h_lpf_chroma_intra_msa;

            /* Weighted MC */
            c->weight_h264_pixels_tab[0] = ff_weight_h264_pixels16_8_msa;
            c->weight_h264_pixels_tab[1] = ff_weight_h264_pixels8_8_msa;
            c->weight_h264_pixels_tab[2] = ff_weight_h264_pixels4_8_msa;

            c->biweight_h264_pixels_tab[0] = ff_biweight_h264_pixels16_8_msa;
            c->biweight_h264_pixels_tab[1] = ff_biweight_h264_pixels8_8_msa;
            c->biweight_h264_pixels_tab[2] = ff_biweight_h264_pixels4_8_msa;

            c->h264_idct_add = ff_h264_idct_add_msa;
            c->h264_idct8_add = ff_h264_idct8_addblk_msa;
            c->h264_idct_dc_add = ff_h264_idct4x4_addblk_dc_msa;
            c->h264_idct8_dc_add = ff_h264_idct8_dc_addblk_msa;
            c->h264_idct_add16 = ff_h264_idct_add16_msa;
            c->h264_idct8_add4 = ff_h264_idct8_add4_msa;

            if (chroma_format_idc <= 1)
                c->h264_idct_add8 = ff_h264_idct_add8_msa;
            else
                c->h264_idct_add8 = ff_h264_idct_add8_422_msa;

            c->h264_idct_add16intra = ff_h264_idct_add16_intra_msa;
            c->h264_luma_dc_dequant_idct = ff_h264_deq_idct_luma_dc_msa;
        }
    }
}
