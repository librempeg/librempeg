/*
 * Copyright (c) 2009 Michael Niedermayer
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
#include "internal.h"


static int probe(const AVProbeData *p)
{
    // the single file I have starts with that, I do not know if others do, too
    if(   p->buf[0] == 1
       && p->buf[1] == 1
       && p->buf[2] == 3
       && p->buf[3] == 0xB8
       && p->buf[4] == 0x80
       && p->buf[5] == 0x60
      )
        return AVPROBE_SCORE_MAX-2;

    return 0;
}

static int read_header(AVFormatContext *s)
{
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_MPEG4;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
    avpriv_set_pts_info(st, 64, 1, 90000);

    return 0;

}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, size, pts, type, flags;
    int first_pkt      = 0;
    int frame_complete = 0;

    while (!frame_complete) {

        type  = avio_rb16(s->pb); // 257 or 258
        size  = avio_rb16(s->pb);
        flags = avio_rb16(s->pb); //some flags, 0x80 indicates end of frame
                avio_rb16(s->pb); //packet number
        pts   = avio_rb32(s->pb);
                avio_rb32(s->pb); //6A 13 E3 88

        frame_complete = flags & 0x80;

        size -= 12;
        if (size < 1)
            return -1;

        if (type == 258) {
            avio_skip(s->pb, size);
            frame_complete = 0;
            continue;
        }

        if (!first_pkt) {
            ret = av_get_packet(s->pb, pkt, size);
            if (ret < 0)
                return ret;
            first_pkt = 1;
            pkt->pts  = pts;
            pkt->pos -= 16;
        } else {
            ret = av_append_packet(s->pb, pkt, size);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "failed to grow packet\n");
                return ret;
            }
        }
        if (ret < size) {
            av_log(s, AV_LOG_ERROR, "Truncated packet! Read %d of %d bytes\n",
                   ret, size);
            pkt->flags |= AV_PKT_FLAG_CORRUPT;
            break;
        }
    }
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_iv8_demuxer = {
    .p.name         = "iv8",
    .p.long_name    = NULL_IF_CONFIG_SMALL("IndigoVision 8000 video"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
