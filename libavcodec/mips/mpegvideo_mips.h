/*
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

#ifndef AVCODEC_MIPS_MPEGVIDEO_MIPS_H
#define AVCODEC_MIPS_MPEGVIDEO_MIPS_H

#include "libavcodec/mpegvideo.h"
#include "libavcodec/mpegvideoenc.h"

void ff_dct_unquantize_h263_intra_mmi(MpegEncContext *s, int16_t *block,
        int n, int qscale);
void ff_dct_unquantize_h263_inter_mmi(MpegEncContext *s, int16_t *block,
        int n, int qscale);
void ff_dct_unquantize_mpeg1_intra_mmi(MpegEncContext *s, int16_t *block,
        int n, int qscale);
void ff_dct_unquantize_mpeg1_inter_mmi(MpegEncContext *s, int16_t *block,
        int n, int qscale);
void ff_dct_unquantize_mpeg2_intra_mmi(MpegEncContext *s, int16_t *block,
        int n, int qscale);
void ff_denoise_dct_mmi(MPVEncContext *s, int16_t *block);

#endif /* AVCODEC_MIPS_MPEGVIDEO_MIPS_H */
