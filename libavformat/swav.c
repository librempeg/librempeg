/*
 * SWAV demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','W','A','V'))
        return 0;

    if (p->buf_size < 26)
        return 0;

    if (AV_RB16(p->buf+4) != 0xFFFE)
        return 0;

    if (AV_RB16(p->buf+6) != 0x1)
        return 0;

    if (AV_RB32(p->buf+16) != MKBETAG('D','A','T','A'))
        return 0;

    if (p->buf[25] > 2)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int codec, rate, ret, loop_flag;
    int64_t start = 0x24, loop_start, loop_end;
    AVStream *st;

    avio_skip(pb, 24);
    codec = avio_r8(pb);
    loop_flag = avio_r8(pb) != 0;
    rate = avio_rl16(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 2);
    loop_start = avio_rl16(pb);
    loop_end = avio_rl32(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = rate;

    switch (codec) {
    case 0:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S8;
        st->codecpar->bits_per_coded_sample = 8;
        st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;
        break;
    case 1:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        st->codecpar->bits_per_coded_sample = 16;
        st->codecpar->block_align = 512 * st->codecpar->ch_layout.nb_channels;
        break;
    case 2:
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WS;
        st->codecpar->bits_per_coded_sample = 4;
        st->codecpar->block_align = 0x40 * st->codecpar->ch_layout.nb_channels;
        ret = ff_alloc_extradata(st->codecpar, 2);
        if (ret < 0)
            return ret;
        AV_WL16(st->codecpar->extradata, 3);
        start += 4;
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (loop_flag) {
        loop_start = loop_start * 32 / st->codecpar->bits_per_coded_sample;
        loop_end = loop_end * 32 / st->codecpar->bits_per_coded_sample + loop_start;
        if (codec == 2) {
            loop_start -= 32 / st->codecpar->bits_per_coded_sample;
            loop_end -= 32 / st->codecpar->bits_per_coded_sample;
        }

        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
        av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_swav_demuxer = {
    .p.name         = "swav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SWAV (Nintendo DS SWAV)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "swav",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
