/*
 * MGS2S demuxer
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
    if (AV_RN32(p->buf) == 0)
        return 0;

    if (p->buf_size < 12)
        return 0;
    if (AV_RB16(p->buf+4) != 0x7f)
        return 0;
    if (AV_RB16(p->buf+6) <= 0)
        return 0;
    if (p->buf[8]/2 == 0)
        return 0;
    if (AV_RL16(p->buf+10) != 17)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int streams, rate, channels;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 6);
    rate = avio_rb16(pb);
    streams = avio_r8(pb)/2;
    channels = 2;
    if (streams <= 0 || rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < streams; n++) {
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_MS;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = 0x800;
        st->codecpar->bit_rate = 8LL * channels * 0x800 * rate / (2 + (0x800 - 7 * channels) * 2);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    pos = avio_tell(pb);
    ret = av_get_packet(s->pb, pkt, 0x800);

    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = ((pos - 0x800) / 0x1000) % s->nb_streams;

    return ret;
}

const FFInputFormat ff_mgs2s_demuxer = {
    .p.name         = "mgs2s",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Metal Gear Solid 2: Substance"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
