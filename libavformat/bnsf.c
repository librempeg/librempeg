/*
 * BNSF demuxer
 * Copyright (c) 2025 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/intreadwrite.h"
#include "libavcodec/bytestream.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int bnsf_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('B', 'N', 'S', 'F'))
        return 0;

    if (AV_RL32(p->buf + 12) != MKTAG('s', 'f', 'm', 't'))
        return 0;

    return AVPROBE_SCORE_MAX / 3;
}

static int bnsf_read_header(AVFormatContext *s)
{
    AVCodecParameters *par;
    AVIOContext *pb = s->pb;
    AVStream *st;
    uint32_t size, chunk, codec;
    int found_sdat = 0, found_sfmt = 0;
    int64_t data_offset = 0;

    avio_skip(pb, 8);
    codec = avio_rb32(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    par = st->codecpar;

    while (!avio_feof(pb) && !(found_sdat && found_sfmt)) {
        chunk = avio_rl32(pb);
        size  = avio_rb32(pb);
        switch (chunk) {
        case MKTAG('s', 'f', 'm', 't'):
            avio_skip(pb, 2);
            par->codec_type  = AVMEDIA_TYPE_AUDIO;
            if (codec == MKBETAG('I','S','2','2'))
                par->codec_id    = AV_CODEC_ID_G719;
            else if (codec == MKBETAG('I','S','1','4'))
                par->codec_id    = AV_CODEC_ID_G722_1;
            par->codec_tag   = 0;
            par->ch_layout.nb_channels = avio_rb16(pb);
            par->sample_rate = avio_rb32(pb);
            st->start_time = 0;
            st->duration     = avio_rb32(pb);
            avio_skip(pb, 4);
            par->block_align = avio_rb16(pb);
            found_sfmt = 1;
            if (size > 18)
                avio_skip(pb, size - 18);
            break;
        case MKTAG('s', 'd', 'a', 't'):
            data_offset = avio_tell(pb);
            found_sdat = 1;
            break;
        case MKTAG('l', 'o', 'o', 'p'):
        default:
            avio_skip(pb, size);
            break;
        }
    }

    if (!found_sdat && !found_sfmt)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, data_offset, SEEK_SET);
    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

static int bnsf_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int ret;

    ret = av_get_packet(s->pb, pkt, par->block_align);
    pkt->duration = 960;
    return ret;
}

const FFInputFormat ff_bnsf_demuxer = {
    .p.name         = "bnsf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BNSF (Bandai Namco Sound Format)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bnsf",
    .read_probe     = bnsf_probe,
    .read_header    = bnsf_read_header,
    .read_packet    = bnsf_read_packet,
};
