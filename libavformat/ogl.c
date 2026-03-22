/*
 * OGL demuxer
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
#include "libavcodec/bytestream.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct OGLContext {
    int64_t data_end;
} OGLContext;

static int read_probe(const AVProbeData *p)
{
    int score = AVPROBE_SCORE_MAX;

    if (p->buf_size < 0x26)
        return 0;

    if (memcmp(p->buf+0x17, "vorbis", 6))
        return 0;
    if (p->buf[0x21] == 0)
        return 0;
    if (AV_RL32(p->buf+0x0c) == 0)
        score -= 10;
    if (AV_RL32(p->buf+0x10) == 0)
        return 0;
    if ((int)AV_RL32(p->buf+0x22) <= 0)
        return 0;

    return score;
}

static int read_header(AVFormatContext *s)
{
    int64_t data_size, duration, loop_start, start_offset;
    int ret, nb_channels, rate, extradata_size, size;
    OGLContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    PutByteContext pbyte;
    AVStream *st;

    avio_skip(pb, 4);
    loop_start = avio_rl32(pb);
    avio_skip(pb, 4);
    duration = avio_rl32(pb);
    data_size = avio_rl32(pb);
    avio_skip(pb, 13);
    nb_channels = avio_r8(pb);
    rate = avio_rl32(pb);
    extradata_size = 0;
    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x14, SEEK_SET);
    size = avio_rl16(pb) >> 2;
    if (size != 0x1e)
        return AVERROR_INVALIDDATA;
    extradata_size += size;
    avio_skip(pb, size);
    size = avio_rl16(pb) >> 2;
    extradata_size += size;
    avio_skip(pb, size);
    size = avio_rl16(pb) >> 2;
    extradata_size += size;
    avio_skip(pb, size);
    start_offset = avio_tell(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    n->data_end = data_size;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_VORBIS;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;

    ret = ff_alloc_extradata(st->codecpar, extradata_size + 6);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    bytestream2_init_writer(&pbyte, st->codecpar->extradata, st->codecpar->extradata_size);

    avio_seek(pb, 0x14, SEEK_SET);
    size = avio_rl16(pb) >> 2;
    bytestream2_put_be16(&pbyte, size);
    while (size > 0) {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;
        bytestream2_put_byte(&pbyte, avio_r8(pb));
        size--;
    }
    size = avio_rl16(pb) >> 2;
    bytestream2_put_be16(&pbyte, size);
    while (size > 0) {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;
        bytestream2_put_byte(&pbyte, avio_r8(pb));
        size--;
    }
    size = avio_rl16(pb) >> 2;
    bytestream2_put_be16(&pbyte, size);
    while (size > 0) {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;
        bytestream2_put_byte(&pbyte, avio_r8(pb));
        size--;
    }

    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (loop_start > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    OGLContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret, size;

    if (pos >= n->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rl16(pb) >> 2;
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_ogl_demuxer = {
    .p.name         = "ogl",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Shin'en OGL"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ogl",
    .priv_data_size = sizeof(OGLContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
