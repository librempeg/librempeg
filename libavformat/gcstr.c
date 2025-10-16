/*
 * GC Square Enix STR demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct GCSTRContext {
    int64_t stop_offset;
} GCSTRContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','T','R','\0'))
        return 0;
    if (p->buf_size < 28)
        return 0;
    if (AV_RB32(p->buf+4) != 0)
        return 0;
    if ((int)AV_RB32(p->buf+24) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    GCSTRContext *str = s->priv_data;
    int ret, channels, rate;
    AVIOContext *pb = s->pb;
    int64_t blocks;
    AVStream *st;

    avio_skip(pb, 12);
    blocks = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    channels = avio_rb32(pb);
    if (channels <= 0 || channels > INT_MAX/0x1000)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = blocks * 14LL;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate ? 44100 : 32000;
    st->codecpar->block_align = 0x1000 * channels;
    str->stop_offset = 0x1000 + blocks * 8LL * channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x20, SEEK_SET);
    if ((ret = ff_alloc_extradata(st->codecpar, 32 * channels)) < 0)
        return ret;
    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + ch * 32, 32);
        avio_skip(pb, 0xe);
    }

    avio_seek(pb, 0x1000, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    GCSTRContext *str = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    pos = avio_tell(pb);
    if (pos >= str->stop_offset)
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_gcstr_demuxer = {
    .p.name         = "gcstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix GameCube STR"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(GCSTRContext),
    .p.extensions   = "str",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
