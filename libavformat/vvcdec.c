/*
 * RAW H.266 / VVC video demuxer
 * Copyright (c) 2020 Nuo Mi <nuomi2021@gmail.com>
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

#include "libavcodec/vvc.h"

#include "avformat.h"
#include "rawdec.h"

static int check_temporal_id(uint8_t nuh_temporal_id_plus1, int type)
{
    if (nuh_temporal_id_plus1 == 0)
        return 0;

    if (nuh_temporal_id_plus1 != 1) {
        if (type >= VVC_IDR_W_RADL && type <= VVC_RSV_IRAP_11
                || type == VVC_DCI_NUT || type == VVC_OPI_NUT
                || type == VVC_VPS_NUT || type == VVC_SPS_NUT
                || type == VVC_EOS_NUT || type == VVC_EOB_NUT)
            return 0;
    }

    return 1;
}

static int vvc_probe(const AVProbeData *p)
{
    uint32_t code = -1;
    int sps = 0, pps = 0, irap = 0;
    int valid_pps = 0, valid_irap = 0;
    int i;

    for (i = 0; i < p->buf_size - 1; i++) {
        code = (code << 8) + p->buf[i];
        if ((code & 0xffffff00) == 0x100) {
            uint8_t nal2 = p->buf[i + 1];
            int type = (nal2 & 0xF8) >> 3;

            if (code & 0x80) // forbidden_zero_bit
                return 0;

            if (!check_temporal_id(nal2 & 0x7, type))
                return 0;

            switch (type) {
            case VVC_SPS_NUT:       sps++;  break;
            case VVC_PPS_NUT:
                pps++;
                if (sps)
                    valid_pps++;
                break;
            case VVC_IDR_N_LP:
            case VVC_IDR_W_RADL:
            case VVC_CRA_NUT:
            case VVC_GDR_NUT:
                irap++;
                if (valid_pps)
                    valid_irap++;
                break;
            }
        }
    }

    if (valid_irap)
        return AVPROBE_SCORE_EXTENSION + 1; // 1 more than .mpg
    if (sps && pps && irap)
        return AVPROBE_SCORE_EXTENSION / 2;
    if (sps || pps || irap)
        return AVPROBE_SCORE_EXTENSION / 4;
    return 0;
}

FF_DEF_RAWVIDEO_DEMUXER(vvc, "raw H.266/VVC video", vvc_probe, "h266,266,vvc", AV_CODEC_ID_VVC)
