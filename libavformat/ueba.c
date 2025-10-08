/*
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
    if (AV_RL32(p->buf) != MKBETAG('U','E','B','A'))
        return 0;
    if (p->buf_size < 20)
        return 0;
    if (p->buf[4] != 1)
        return 0;
    if (p->buf[5] == 0)
        return 0;
    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;
    if (AV_RL32(p->buf + 12) == 0)
        return 0;
    if ((int)AV_RL16(p->buf + 18) != 1)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int nb_seek_entries, ret, rate, channels;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 5);
    channels = avio_r8(pb);
    avio_skip(pb, 2);
    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    avio_skip(pb, 8);
    nb_seek_entries = avio_rl16(pb);
    avio_skip(pb, 2 + nb_seek_entries * 2);
    if (channels == 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_id = AV_CODEC_ID_BINKAUDIO_DCT;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if ((ret = ff_alloc_extradata(st->codecpar, 1)) < 0)
        return ret;
    st->codecpar->extradata[0] = '2';

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
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
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
