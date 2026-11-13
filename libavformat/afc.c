/*
 * AFC demuxer
 * Copyright (c) 2012 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct AFCDemuxContext {
    int64_t data_end;
} AFCDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RN32(p->buf) == 0)
        return 0;

    if (p->buf_size < 16)
        return 0;
    if (AV_RN32(p->buf+4) == 0)
        return 0;
    if (AV_RB16(p->buf+8) <= 0)
        return 0;
    if (AV_RB16(p->buf+10) != 4)
        return 0;
    if (AV_RB16(p->buf+12) != 16)
        return 0;
    if (AV_RN16(p->buf+14) == 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    AFCDemuxContext *c = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;
    int rate;

    c->data_end = avio_rb32(pb) + 32LL;
    duration = avio_rb32(pb);
    rate = avio_rb16(pb);
    avio_skip(pb, 22);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_AFC;
    st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 9 * 2;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AFCDemuxContext *c = s->priv_data;
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    if (pos >= c->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    const int block_size = ff_pcm_default_packet_size(st->codecpar);
    const int size = FFMIN(block_size, c->data_end - pos);

    ret = av_get_packet(pb, pkt, size);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_afc_demuxer = {
    .p.name         = "afc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Nintendo AFC"),
    .p.extensions   = "afc",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(AFCDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
