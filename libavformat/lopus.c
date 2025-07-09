/*
 * LOPUS demuxer
 * Copyright (c) 2025 Paul B Mahol
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

typedef struct LOPUSDemuxContext {
    int64_t data_end;
} LOPUSDemuxContext;

static int lopus_probe(const AVProbeData *p)
{
    int offset = 0;

    if (AV_RL32(p->buf) == MKTAG('O','P','U','S'))
        offset = 24;

    if (p->buf_size < offset + 9)
        return 0;

    if (AV_RL32(p->buf+offset) == 0x80000001 &&
        AV_RL32(p->buf+offset+4) == 0x18 &&
        p->buf[9] > 0)
        return AVPROBE_SCORE_MAX;

    return 0;
}

static int lopus_read_header(AVFormatContext *s)
{
    LOPUSDemuxContext *lc = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t chunk, data_offset, duration = 0, offset = 0;
    int skip, ret, nb_channels;
    AVStream *st;

    chunk = avio_rl32(pb);
    if (chunk == MKTAG('O','P','U','S')) {
        avio_skip(pb, 4);
        duration = avio_rl32(pb);
        avio_skip(pb, 16);
        offset = 24;
    }
    avio_skip(pb, 5);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = nb_channels = avio_r8(pb);
    if (st->codecpar->ch_layout.nb_channels < 1)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 2);

    st->codecpar->sample_rate = avio_rl32(pb);
    if (!st->codecpar->sample_rate)
        st->codecpar->sample_rate = 48000;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;

    data_offset = avio_rl32(pb) + offset;
    avio_skip(pb, 8);
    skip = avio_rl16(pb);
    avio_skip(pb, 2);

    ret = ff_alloc_extradata(st->codecpar, 19 + 2 + nb_channels);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    st->codecpar->extradata[9] = nb_channels;
    AV_WL16(st->codecpar->extradata + 10, skip);

    if (avio_tell(pb) > data_offset)
        return AVERROR_INVALIDDATA;
    chunk = avio_rl32(pb);
    if (chunk == 0x80000005 && nb_channels <= 8) {
        avio_skip(pb, 4);
        st->codecpar->extradata[18] = 1;
        st->codecpar->extradata[19] = avio_r8(pb);
        st->codecpar->extradata[20] = avio_r8(pb);
        avio_skip(pb, 2);

        for (int ch = 0; ch < nb_channels; ch++)
            st->codecpar->extradata[21 + ch] = avio_r8(pb);

        chunk = avio_rl32(pb);
    } else if (chunk != 0x80000004) {
        avio_skip(pb, data_offset - avio_tell(pb));
        chunk = avio_rl32(pb);
    }
    if (chunk != 0x80000004)
        return AVERROR_INVALIDDATA;

    lc->data_end = avio_tell(pb) + avio_rl32(pb);

    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int lopus_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    LOPUSDemuxContext *lc = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t size;
    int ret;

    if (pos >= lc->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rb32(pb);
    if (size < 2)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_lopus_demuxer = {
    .p.name         = "lopus",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Switch OPUS "),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "lopus",
    .priv_data_size = sizeof(LOPUSDemuxContext),
    .read_probe     = lopus_probe,
    .read_header    = lopus_read_header,
    .read_packet    = lopus_read_packet,
};
