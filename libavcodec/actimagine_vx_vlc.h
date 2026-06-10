/*
 * This file is part of Librempeg.
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

#ifndef AVCODEC_ACTIMAGINE_VX_VLC_H
#define AVCODEC_ACTIMAGINE_VX_VLC_H

#include <stdint.h>
#include "vlc.h"

#define FF_H264_CAVLC_COEFF_TOKEN_VLC_BITS    8
#define FF_H264_CAVLC_TOTAL_ZEROS_VLC_BITS    9
#define FF_H264_CAVLC_RUN_VLC_BITS            3
#define FF_H264_CAVLC_RUN7_VLC_BITS           6

extern const uint8_t ff_h264_cavlc_coeff_token_table_index[17];

extern const unsigned int ff_h264_cavlc_suffix_limit[7];

extern VLC ff_h264_cavlc_coeff_token_vlc[4];
extern VLC ff_h264_cavlc_total_zeros_vlc[15];
extern VLC ff_h264_cavlc_run_vlc[6];
extern VLC ff_h264_cavlc_run7_vlc;

int ff_h264_cavlc_data_init_vlc(void);

#endif /* AVCODEC_ACTIMAGINE_VX_VLC_H */
