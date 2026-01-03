/*
 * ACE demuxer
 * Copyright (c) 2020 Paul B Mahol
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
    uint32_t asc;

    if (AV_RB32(p->buf) != MKBETAG('A','A','C',' '))
        return 0;

    if (p->buf_size < 0x44)
        return 0;

    switch (AV_RB32(p->buf + 0x28)) {
    case 2:
        break;
    case 3:
        asc = AV_RB32(p->buf + 0x40);
        if (asc < 0x44 || asc > p->buf_size - 4)
            return 0;
        if (AV_RB32(p->buf + asc) != MKBETAG('A','S','C',' '))
            return 0;
        break;
    default:
        return 0;
    }

    return AVPROBE_SCORE_MAX / 2 + 1;
}

static int read_header(AVFormatContext *s)
{
    int streams = 0, version, ret, codec, rate = 0, nb_channels = 0;
    uint32_t chunk, chunk_pos, size = 0;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int64_t duration = 0;
    AVStream *st;

    avio_skip(pb, 0x28);
    version = avio_rb32(pb);
    switch (version) {
    case 2:
        chunk_pos = avio_rb32(pb);
        avio_skip(pb, chunk_pos - avio_tell(pb));
        chunk = avio_rb32(pb);
        if (chunk == MKBETAG('A','S','C',' ')) {
            avio_skip(pb, 0xcc);
            chunk = avio_rb32(pb);
            if (chunk != MKBETAG('W','A','V','E'))
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 12);

            chunk = avio_rb16(pb);
            switch (chunk) {
            case 0x0400:
                avio_skip(pb, 2);
                streams = avio_rb16(pb);
                codec = avio_rb16(pb);
                avio_skip(pb, 8);
                size = avio_rb32(pb);
                rate = avio_rb32(pb);
                avio_skip(pb, 16);
                duration = avio_rb32(pb);
                avio_skip(pb, 20);
                chunk_pos += avio_rb32(pb);

                avio_skip(pb, 16);
                for (int i = 0; i < streams; i++) {
                    nb_channels += avio_r8(pb);
                    avio_skip(pb, 0x2f);
                }

                avio_skip(pb, chunk_pos - avio_tell(pb));
                break;
            default:
                break;
            }
        } else {
            return AVERROR_INVALIDDATA;
        }
        break;
    case 3:
        avio_skip(pb, 0x14);
        chunk_pos = avio_rb32(pb);
        if (chunk_pos < 0x44)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, chunk_pos - 0x44);
        if (avio_rb32(pb) != MKBETAG('A','S','C',' '))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 0xec);
        codec = avio_rb32(pb);
        nb_channels = avio_rb32(pb);
        size = avio_rb32(pb);
        rate = avio_rb32(pb);
        avio_skip(pb, 16);
        break;
    default:
        avpriv_request_sample(s, "version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    if (size == 0)
        return AVERROR_INVALIDDATA;

    if (nb_channels <= 0 || nb_channels > 8)
        return AVERROR_INVALIDDATA;

    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    par = st->codecpar;
    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    par->ch_layout.nb_channels = nb_channels;
    par->sample_rate = rate;

    switch (codec) {
    case 4:
    case 5:
    case 6:
        par->codec_id = AV_CODEC_ID_ATRAC3;
        par->block_align = (codec == 4 ? 0x60 : codec == 5 ? 0x98 : 0xC0) * nb_channels;
        st->duration = (size / par->block_align) * 1024LL;

        ret = ff_alloc_extradata(par, 14);
        if (ret < 0)
            return ret;

        AV_WL16(par->extradata, 1);
        AV_WL16(par->extradata+2, 2048 * nb_channels);
        AV_WL16(par->extradata+4, 0);
        AV_WL16(par->extradata+6, codec == 4 ? 1 : 0);
        AV_WL16(par->extradata+8, codec == 4 ? 1 : 0);
        AV_WL16(par->extradata+10, 1);
        AV_WL16(par->extradata+12, 0);
        break;
    case 0x165:
        if (duration > 0)
            st->duration = duration;
        par->codec_id = AV_CODEC_ID_XMA2;
        par->block_align = 0x800;
        ret = ff_alloc_extradata(par, 34);
        if (ret < 0)
            return ret;
        memset(par->extradata, 0, par->extradata_size);
        AV_WL16(par->extradata, streams);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
        break;
    default:
        avpriv_request_sample(s, "codec 0x%04X", codec);
        return AVERROR_PATCHWELCOME;
    }

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

const FFInputFormat ff_ace_demuxer = {
    .p.name         = "ace",
    .p.long_name    = NULL_IF_CONFIG_SMALL("tri-Ace Audio Container"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
