/*
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int ueba_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) == MKBETAG('U','E','B','A') && p->buf[4] == 1 &&
        (p->buf[5] == 1 || p->buf[5] == 2) && AV_RL32(p->buf + 8))
        return AVPROBE_SCORE_MAX / 2;
    return 0;
}

static int ueba_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int nb_seek_entries, ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 5);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_BINKAUDIO_DCT;
    st->codecpar->ch_layout.nb_channels = avio_r8(pb);
    avio_skip(pb, 2);
    st->codecpar->sample_rate = avio_rl32(pb);
    st->duration = avio_rl32(pb);
    avio_skip(pb, 8);
    nb_seek_entries = avio_rl16(pb);
    avio_skip(pb, 2 + nb_seek_entries * 2);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if ((ret = ff_alloc_extradata(st->codecpar, 1)) < 0)
        return ret;
    st->codecpar->extradata[0] = '2';

    return 0;
}

static int ueba_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    AVStream *st = s->streams[0];
    int64_t pos;
    int magic, size, duration, ret;

    pos = avio_tell(pb);

    magic = avio_rl16(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;
    if (magic != 0x9999)
        return AVERROR_INVALIDDATA;

    size = avio_rl16(pb);
    if (size == 0xFFFF) {
        size = avio_rl16(pb);
        duration = avio_rl16(pb);
    } else {
        duration = av_get_audio_frame_duration2(st->codecpar, 0);
    }
    if (size <= 0)
        return AVERROR(EIO);

    ret = av_new_packet(pkt, size + 4);
    if (ret < 0)
        return ret;

    AV_WL32(pkt->data, size);
    avio_read(pb, pkt->data + 4, size);

    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->duration = duration;

    return 0;
}

const FFInputFormat ff_ueba_demuxer = {
    .p.name         = "ueba",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Unreal Engine Bink Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = ueba_probe,
    .read_header    = ueba_read_header,
    .read_packet    = ueba_read_packet,
};
