/*
 * Midway ADS demuxer
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
#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 36)
        return 0;

    if (memcmp(p->buf, "dhSS", 4) ||
        memcmp(p->buf+32, "dbSS", 4))
        return 0;

    if ((int)AV_RB32(p->buf + 12) <= 0)
        return 0;

    if ((int)AV_RB32(p->buf + 16) <= 0)
        return 0;

    if ((int)AV_RB32(p->buf + 20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, align, codec, rate, channels;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    AVStream *st;

    avio_skip(pb, 8);
    codec = avio_rb32(pb);
    rate = avio_rb32(pb);
    channels = avio_rb32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case 0x20:
        align = avio_rb32(pb);
        if (align <= 0 || align > INT_MAX/channels)
            return AVERROR_INVALIDDATA;
        start_offset = 0x28 + 0x60 * channels;
        codec = AV_CODEC_ID_ADPCM_NDSP;
        break;
    case 0x21:
        start_offset = 0x28;
        align = 0x24;
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = channels * align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (codec == AV_CODEC_ID_ADPCM_NDSP) {
        avio_seek(pb, 40, SEEK_SET);
        st->duration = avio_rb32(pb);
        avio_seek(pb, 68, SEEK_SET);
        if ((ret = ff_alloc_extradata(st->codecpar, 32 * channels)) < 0)
            return ret;
        for (int ch = 0; ch < channels; ch++) {
            avio_read(pb, st->codecpar->extradata + ch * 32, 32);
            avio_skip(pb, 64);
        }
    } else {
        st->codecpar->bit_rate = 36LL * channels * 8 * rate / 64;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_adsbe_demuxer = {
    .p.name         = "adsbe",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Midway ADS"),
    .p.extensions   = "ads",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
