/*
 * CRC encoder (for codec/format testing)
 * Copyright (c) 2002 Fabrice Bellard
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

#include <inttypes.h>

#include "libavutil/adler32.h"
#include "avformat.h"
#include "mux.h"

typedef struct CRCState {
    uint32_t crcval;
} CRCState;

static int crc_init(struct AVFormatContext *s)
{
    CRCState *crc = s->priv_data;

    /* init CRC */
    crc->crcval = 1;

    return 0;
}

static int crc_write_packet(struct AVFormatContext *s, AVPacket *pkt)
{
    CRCState *crc = s->priv_data;
    crc->crcval = av_adler32_update(crc->crcval, pkt->data, pkt->size);
    return 0;
}

static int crc_write_trailer(struct AVFormatContext *s)
{
    CRCState *crc = s->priv_data;

    avio_printf(s->pb, "CRC=0x%08"PRIx32"\n", crc->crcval);

    return 0;
}

const FFOutputFormat ff_crc_muxer = {
    .p.name            = "crc",
    .p.long_name       = NULL_IF_CONFIG_SMALL("CRC testing"),
    .priv_data_size    = sizeof(CRCState),
    .p.audio_codec     = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec     = AV_CODEC_ID_RAWVIDEO,
    .init              = crc_init,
    .write_packet      = crc_write_packet,
    .write_trailer     = crc_write_trailer,
    .p.flags           = AVFMT_NOTIMESTAMPS,
};
