/*
 * RAW SBC demuxer
 * Copyright (C) 2017  Aurelien Jacobs <aurel@gnuage.org>
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

#include "avformat.h"
#include "demux.h"
#include "rawdec.h"

const FFInputFormat ff_sbc_demuxer = {
    .p.name         = "sbc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw SBC (low-complexity subband codec)"),
    .p.extensions   = "sbc,msbc",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &ff_raw_demuxer_class,
    .raw_codec_id   = AV_CODEC_ID_SBC,
    .read_header    = ff_raw_audio_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .priv_data_size = sizeof(FFRawDemuxerContext),
};
