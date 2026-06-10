/*
 * LDAC demuxer
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
#include "avio.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    for (int i = 0; i + 3 < p->buf_size;) {
        uint32_t header = AV_RB32(p->buf + i);
        int size = ((header >> 10) & 0x1ff) + 4;

        if ((header >> 24) != 0xaa)
            return 0;

        i += size;
    }

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, frame_size, frame_samples;
    AVIOContext *pb = s->pb;
    uint32_t header;
    AVStream *st;

    header = avio_rb32(pb);
    rate = (header >> 21) & 7;
    channels = (header >> 19) & 3;
    frame_size = ((header >> 10) & 0x1ff) + 4;
    frame_samples = 1 << (7 + (rate > 1));

    switch (channels) {
    case 0:
        channels = 1;
        break;
    case 1:
    case 2:
        channels = 2;
        break;
    default:
        channels = 0;
        break;
    }

    switch (rate) {
    case 0:
        rate = 44100;
        break;
    case 1:
        rate = 48000;
        break;
    case 2:
        rate = 88200;
        break;
    case 3:
        rate = 96000;
        break;
    default:
        rate = 0;
        break;
    }

    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_LDAC;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->bit_rate = 8LL * frame_size * rate / frame_samples;

    avpriv_set_pts_info(st, 64, 1, rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret, size, rate;
    uint32_t header;

    header = avio_rb32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if ((header >> 24) != 0xaa)
        return AVERROR_INVALIDDATA;

    rate = (header >> 21) & 7;
    size = ((header >> 10) & 0x1ff) + 4;

    avio_seek(pb, pos, SEEK_SET);
    ret = av_get_packet(s->pb, pkt, size);
    if (ret < 0)
        return ret;

    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->duration = (rate < 2) ? 128 : ((rate < 4) ? 256 : 0);

    return 0;
}

const FFInputFormat ff_ldac_demuxer = {
    .p.name         = "ldac",
    .p.long_name    = NULL_IF_CONFIG_SMALL("LDAC"),
    .p.extensions   = "ldac",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
