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
    if (p->buf[8] == 0)
        return 0;
    if (AV_RL16(p->buf+10) != 17)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int align, streams, rate, channels;
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    start = avio_size(pb) - avio_rb32(pb);
    start = FFABS(start);
    start = FFMIN(start, 0x800);
    align = (start == 0x10) ? 0x400 : 0x800;

    avio_skip(pb, 2);
    rate = avio_rb16(pb);
    if (align == 0x800) {
        streams = avio_r8(pb)/2;
        channels = 2;
    } else {
        channels = avio_r8(pb);
        streams = 1;
        align *= channels;
    }
    if (streams <= 0 || rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < streams; n++) {
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = (start == 0x10) ? AV_CODEC_ID_ADPCM_MS_MONO : AV_CODEC_ID_ADPCM_MS;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align;
        st->codecpar->bit_rate = 8LL * channels * align * rate / (2 + (align - 7 * channels) * 2);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    const int64_t offset = ffformatcontext(s)->data_offset;
    int block_align = s->streams[0]->codecpar->block_align;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    pos = avio_tell(pb);
    if (offset == 0x10)
        block_align *= 2;
    ret = av_get_packet(s->pb, pkt, block_align);

    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    if (offset != 0x10)
        pkt->stream_index = ((pos - offset) / 0x1000) % s->nb_streams;
    else
        pkt->stream_index = 0;

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
