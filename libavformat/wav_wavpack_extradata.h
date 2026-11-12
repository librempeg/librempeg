/*
 * WavPack (v1-v3) extradata header
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

#ifndef AVFORMAT_WAV_WAVPACK_EXTRADATA_H
#define AVFORMAT_WAV_WAVPACK_EXTRADATA_H

#include "libavcodec/avcodec.h"
#include "libavcodec/codec_par.h"
#include "wav_wavpack_export.h"

static void upload_to_wavpack_old_extradata_version(AVCodecParameters *codecpar, uint16_t version);
static void upload_to_wavpack_old_extradata_minbits(AVCodecParameters *codecpar, uint16_t minbits);
static void upload_to_wavpack_old_extradata_v3_flags(AVCodecParameters *codecpar, uint16_t v3_flags);

#endif /* AVFORMAT_WAV_WAVPACK_EXTRADATA_H */
