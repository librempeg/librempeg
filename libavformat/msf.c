/*
 * MSF demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "MSF", 3))
        return 0;

    if ((int)AV_RB32(p->buf+8) <= 0)
        return 0;

    if ((int)AV_RB32(p->buf+16) <= 0)
        return 0;

    if (AV_RB32(p->buf+4) > 7)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int ret, channels, rate;
    unsigned codec;
    AVStream *st;

    avio_skip(pb, 4);

    codec = avio_rb32(pb);
    channels = avio_rb32(pb);
    if (channels <= 0 || channels >= INT_MAX / 1024)
        return AVERROR_INVALIDDATA;
    avio_rb32(pb);
    rate = avio_rb32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;
    // avio_rb32(pb); /* byte flags with encoder info */
    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;

    switch (codec) {
    case 0:
        st->codecpar->block_align = 2 * channels;
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
        break;
    case 1:
        st->codecpar->block_align = 2 * channels;
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        break;
    case 3:
        st->codecpar->block_align = 16 * channels;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                        st->codecpar->sample_rate / 28;
        break;
    case 4:
    case 5:
    case 6:
        st->codecpar->block_align = (codec == 4 ? 96 : codec == 5 ? 152 : 192) * channels;
        st->codecpar->bit_rate = 8LL * st->codecpar->block_align *
                                       st->codecpar->sample_rate / 1024;
        if (st->codecpar->ch_layout.nb_channels > UINT16_MAX / 2048)
            return AVERROR_INVALIDDATA;
        ret = ff_alloc_extradata(st->codecpar, 14);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        AV_WL16(st->codecpar->extradata, 1); /* version */
        AV_WL16(st->codecpar->extradata+2, 2048 * st->codecpar->ch_layout.nb_channels); /* unknown size */
        AV_WL16(st->codecpar->extradata+6, codec == 4 ? 1 : 0); /* joint stereo */
        AV_WL16(st->codecpar->extradata+8, codec == 4 ? 1 : 0); /* joint stereo (repeat?) */
        AV_WL16(st->codecpar->extradata+10, 1);
        st->codecpar->codec_id = AV_CODEC_ID_ATRAC3;
        break;
    case 7:
        st->codecpar->block_align = 1024;
        st->codecpar->codec_id = AV_CODEC_ID_MP3;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    default:
        avpriv_request_sample(s, "Codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }
    avio_seek(pb, 0x40, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_msf_demuxer = {
    .p.name         = "msf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS3 MSF (MultiStream File)"),
    .p.extensions   = "msf",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
