/*
 * WavPack (v1-v3) extradata functions
 * Copyright (c) 2026 Librempeg
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
#include "libavcodec/avcodec.h"
#include "libavcodec/codec_par.h"
#include "wav_wavpack_export.h"

static void upload_to_wavpack_old_extradata_version(AVCodecParameters *codecpar, uint16_t version)
{
    if ((codecpar->codec_id == AV_CODEC_ID_WAVPACK_OLD) && (codecpar->extradata) && (codecpar->extradata_size == sizeof(WavPackV2Header))) {
        WavPackV2Header *hdr = (WavPackV2Header*)codecpar->extradata;
        hdr->version = version;
    }
}

static void upload_to_wavpack_old_extradata_minbits(AVCodecParameters *codecpar, uint16_t minbits)
{
    if ((codecpar->codec_id == AV_CODEC_ID_WAVPACK_OLD) && (codecpar->extradata) && (codecpar->extradata_size == sizeof(WavPackV2Header))) {
        WavPackV2Header *hdr = (WavPackV2Header*)codecpar->extradata;
        hdr->minbits = minbits;
    }
}

static void upload_to_wavpack_old_extradata_v3_flags(AVCodecParameters *codecpar, uint16_t v3_flags)
{
    if ((codecpar->codec_id == AV_CODEC_ID_WAVPACK_OLD) && (codecpar->extradata) && (codecpar->extradata_size == sizeof(WavPackV2Header))) {
        WavPackV2Header *hdr = (WavPackV2Header*)codecpar->extradata;
        hdr->v3_flags = v3_flags;
    }
}
