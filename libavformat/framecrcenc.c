/*
 * frame CRC encoder (for codec/format testing)
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
#include "libavutil/avstring.h"

#include "libavcodec/codec_id.h"
#include "libavcodec/codec_par.h"
#include "libavcodec/packet.h"

#include "avformat.h"
#include "internal.h"
#include "mux.h"

static int framecrc_write_header(struct AVFormatContext *s)
{
    int i;
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AVCodecParameters *par = st->codecpar;
        if (par->extradata) {
            uint32_t crc = av_adler32_update(0, par->extradata, par->extradata_size);
            avio_printf(s->pb, "#extradata %d: %8d, 0x%08"PRIx32"\n",
                        i, par->extradata_size, crc);
        }
    }

    return ff_framehash_write_header(s);
}

static int framecrc_write_packet(struct AVFormatContext *s, AVPacket *pkt)
{
    uint32_t crc = av_adler32_update(0, pkt->data, pkt->size);
    char buf[256];

    snprintf(buf, sizeof(buf), "%d, %10"PRId64", %10"PRId64", %8"PRId64", %8d, 0x%08"PRIx32,
             pkt->stream_index, pkt->dts, pkt->pts, pkt->duration, pkt->size, crc);
    if (pkt->flags != AV_PKT_FLAG_KEY)
        av_strlcatf(buf, sizeof(buf), ", F=0x%0X", pkt->flags);
    if (pkt->side_data_elems) {
        av_strlcatf(buf, sizeof(buf), ", S=%d", pkt->side_data_elems);

        for (int i = 0; i < pkt->side_data_elems; i++) {
            av_strlcatf(buf, sizeof(buf), ", %8"SIZE_SPECIFIER,
                        pkt->side_data[i].size);
        }
    }
    av_strlcatf(buf, sizeof(buf), "\n");
    avio_write(s->pb, buf, strlen(buf));
    return 0;
}

const FFOutputFormat ff_framecrc_muxer = {
    .p.name            = "framecrc",
    .p.long_name       = NULL_IF_CONFIG_SMALL("framecrc testing"),
    .p.audio_codec     = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec     = AV_CODEC_ID_RAWVIDEO,
    .write_header      = framecrc_write_header,
    .write_packet      = framecrc_write_packet,
    .p.flags           = AVFMT_VARIABLE_FPS | AVFMT_TS_NONSTRICT |
                         AVFMT_TS_NEGATIVE | AVFMT_NODIMENSIONS,
};
