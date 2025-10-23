/*
 * Raw AC-4 muxer
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

#include "libavutil/intreadwrite.h"
#include "libavcodec/codec_id.h"
#include "libavcodec/packet.h"
#include "libavutil/crc.h"
#include "libavutil/opt.h"
#include "avformat.h"
#include "mux.h"

typedef struct AC4Context {
    AVClass *class;
    int write_crc;
} AC4Context;

static int ac4_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    AC4Context *ac4 = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pkt->size > 2 &&
        AV_RB16(pkt->data) == 0xAC40 ||
        AV_RB16(pkt->data) == 0xAC41) {
        avio_write(pb, pkt->data, pkt->size);
        return 0;
    }

    if (!pkt->size)
        return AVERROR_INVALIDDATA;

    if (ac4->write_crc)
        avio_wb16(pb, 0xAC41);
    else
        avio_wb16(pb, 0xAC40);

    if (pkt->size >= 0xffff) {
        avio_wb16(pb, 0xffff);
        avio_wb24(pb, pkt->size);
    } else {
        avio_wb16(pb, pkt->size);
    }

    avio_write(pb, pkt->data, pkt->size);

    if (ac4->write_crc) {
        uint16_t crc = av_crc(av_crc_get_table(AV_CRC_16_ANSI), 0, pkt->data, pkt->size);
        avio_wl16(pb, crc);
    }

    return 0;
}

#define ENC AV_OPT_FLAG_ENCODING_PARAM
#define OFFSET(obj) offsetof(AC4Context, obj)
static const AVOption ac4_options[] = {
    { "write_crc",  "enable checksum", OFFSET(write_crc), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, ENC},
    { NULL },
};

static const AVClass ac4_muxer_class = {
    .class_name     = "AC4 muxer",
    .option         = ac4_options,
    .version        = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_ac4_muxer = {
    .p.name            = "ac4",
    .p.long_name       = NULL_IF_CONFIG_SMALL("raw AC-4"),
    .p.mime_type       = "audio/ac4",
    .p.extensions      = "ac4",
    .priv_data_size    = sizeof(AC4Context),
    .p.audio_codec     = AV_CODEC_ID_AC4,
    .p.video_codec     = AV_CODEC_ID_NONE,
    .p.subtitle_codec  = AV_CODEC_ID_NONE,
    .flags_internal    = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                         FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
    .write_packet      = ac4_write_packet,
    .p.priv_class      = &ac4_muxer_class,
    .p.flags           = AVFMT_NOTIMESTAMPS,
};
