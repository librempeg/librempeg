/*
 * RPZA demuxer
 * Copyright (c) 2026 Paul B Mahol
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if ((int)AV_RB32(p->buf) < 0x56)
        return 0;
    if (AV_RB32(p->buf+4) != MKBETAG('r','p','z','a'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int width, height;
    AVRational fps;
    int64_t offset;
    AVStream *st;

    offset = avio_rb32(pb);
    avio_skip(pb, 12);
    fps.num = avio_rb16(pb);
    fps.den = avio_rb16(pb);
    avio_skip(pb, 12);
    width = avio_rb16(pb);
    height = avio_rb16(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par              = st->codecpar;
    par->codec_type  = AVMEDIA_TYPE_VIDEO;
    par->codec_id    = AV_CODEC_ID_RPZA;
    st->start_time   = 0;
    st->duration     =
    st->nb_frames    = 1;
    st->avg_frame_rate = fps;
    par->width       = width;
    par->height      = height;

    avpriv_set_pts_info(st, 64, st->avg_frame_rate.den, st->avg_frame_rate.num);

    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    uint32_t size;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    size = avio_rb32(pb);

    ret = av_new_packet(pkt, (size & 0xFFFFFF));
    if (ret < 0)
        return ret;

    ret = avio_read(pb, pkt->data+4, FFMAX(size & 0xFFFFFF, 4) - 4);
    if (ret < 0)
        return ret;
    AV_WB32(pkt->data, size);

    pkt->flags |= AV_PKT_FLAG_KEY;
    pkt->stream_index = 0;
    pkt->duration = 1;
    pkt->pos = pos;

    return 0;
}

const FFInputFormat ff_rpza_demuxer = {
    .p.name         = "rpza",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RPZA Video"),
    .p.extensions   = "rpza",
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
