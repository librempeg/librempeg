/*
 * Square Sounds VS demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    for (int pos = 0; pos + 20 < p->buf_size; pos += 0x800) {
        if (AV_RB32(p->buf+pos) != MKBETAG('V','S','\0','\0'))
            return 0;

        if (AV_RL32(p->buf+pos+16) == 0)
            return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t flags;
    AVStream *st;
    int pitch;

    avio_skip(pb, 4);
    flags = avio_rl32(pb);
    avio_skip(pb, 8);
    pitch = avio_rl32(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = 1 + !!(flags & 1);
    st->codecpar->sample_rate = av_rescale_rnd(pitch, 48000, 4096, AV_ROUND_UP);
    st->codecpar->bit_rate = 8LL * st->codecpar->ch_layout.nb_channels * 16 *
                                   st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret = AVERROR_EOF, channels;
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int64_t pos;

    channels = st->codecpar->ch_layout.nb_channels;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);

    ret = av_new_packet(pkt, (0x800 - 0x20) * channels);
    if (ret < 0)
        return ret;
    for (int ch = 0; ch < channels; ch++) {
        avio_skip(pb, 0x20);

        if (avio_feof(pb))
            return AVERROR_EOF;

        ret = avio_read(pb, pkt->data + (0x800 - 0x20) * ch, 0x800 - 0x20);
        if (ret != 0x800-0x20)
            return AVERROR(EIO);
    }

    pkt->pos = pos;
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_vs00_demuxer = {
    .p.name         = "vs00",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Sounds VS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "vs",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
