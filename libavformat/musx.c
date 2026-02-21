/*
 * MUSX demuxer
 * Copyright (c) 2016 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    unsigned version;

    if (AV_RB32(p->buf) != MKBETAG('M','U','S','X'))
        return 0;

    version = AV_RL32(p->buf + 8);
    if (version != 10 &&
        version != 6 &&
        version != 5 &&
        version != 4 &&
        version != 1 &&
        version != 201)
        return 0;

    return AVPROBE_SCORE_MAX / 5 * 2;
}

static int read_header(AVFormatContext *s)
{
    unsigned type, version, coding, offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    version = avio_rl32(pb);
    if (version != 10 &&
        version != 6 &&
        version != 5 &&
        version != 4 &&
        version != 1 &&
        version != 201) {
        avpriv_request_sample(s, "Unsupported version: %d", version);
        return AVERROR_PATCHWELCOME;
    }
    avio_skip(pb, 4);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (version == 201 || version == 1) {
        avio_skip(pb, 8);
        offset = avio_rl32(pb);
        st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->ch_layout.nb_channels = 2;
        st->codecpar->sample_rate = 32000;
        st->codecpar->block_align = 0x80 * st->codecpar->ch_layout.nb_channels;
        st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                        st->codecpar->sample_rate / 28;
    } else if (version == 10) {
        type = avio_rl32(pb);
        st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
        offset = 0x800;
        switch (type) {
        case MKTAG('P', 'S', '3', '_'):
            st->codecpar->ch_layout.nb_channels = 2;
            st->codecpar->sample_rate = 44100;
            avio_skip(pb, 44);
            coding = avio_rl32(pb);
            if (coding == MKTAG('D', 'A', 'T', '4') ||
                coding == MKTAG('D', 'A', 'T', '8')) {
                avio_skip(pb, 4);
                st->codecpar->ch_layout.nb_channels   = avio_rl32(pb);
                if (st->codecpar->ch_layout.nb_channels <= 0 ||
                    st->codecpar->ch_layout.nb_channels > INT_MAX / 0x20)
                    return AVERROR_INVALIDDATA;
                st->codecpar->sample_rate = avio_rl32(pb);
            }
            st->codecpar->codec_id   = AV_CODEC_ID_ADPCM_IMA_DAT4;
            st->codecpar->block_align = 0x20 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->bit_rate = 32LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 56;
            break;
        case MKTAG('W', 'I', 'I', '_'):
            avio_skip(pb, 44);
            coding = avio_rl32(pb);
            if (coding != MKTAG('D', 'A', 'T', '4') &&
                coding != MKTAG('D', 'A', 'T', '8')) {
                avpriv_request_sample(s, "Unsupported coding: %X", coding);
                return AVERROR_PATCHWELCOME;
            }
            avio_skip(pb, 4);
            st->codecpar->codec_id   = AV_CODEC_ID_ADPCM_IMA_DAT4;
            st->codecpar->ch_layout.nb_channels = avio_rl32(pb);
            if (st->codecpar->ch_layout.nb_channels <= 0 ||
                st->codecpar->ch_layout.nb_channels > INT_MAX / 0x20)
                return AVERROR_INVALIDDATA;
            st->codecpar->sample_rate = avio_rl32(pb);
            st->codecpar->block_align = 0x20 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->bit_rate = 32LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 56;
            break;
        case MKTAG('X', 'E', '_', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_IMA_DAT4;
            st->codecpar->ch_layout.nb_channels = 2;
            st->codecpar->sample_rate = 32000;
            st->codecpar->block_align = 0x20 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->bit_rate = 32LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 56;
            break;
        case MKTAG('P', 'S', 'P', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->ch_layout.nb_channels = 2;
            st->codecpar->sample_rate = 32768;
            st->codecpar->block_align = 0x80 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 28;
            break;
        case MKTAG('P', 'S', '2', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->ch_layout.nb_channels = 2;
            st->codecpar->sample_rate = 32000;
            st->codecpar->block_align = 0x80 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 28;
            break;
        default:
            avpriv_request_sample(s, "Unsupported type: %X", type);
            return AVERROR_PATCHWELCOME;
        }
    } else if (version == 6 || version == 5 || version == 4) {
        type = avio_rl32(pb);
        avio_skip(pb, 20);
        st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = 2;
        switch (type) {
        case MKTAG('G', 'C', '_', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_IMA_DAT4;
            st->codecpar->block_align = 0x20 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->sample_rate = 32000;
            st->codecpar->bit_rate = 32LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 56;
            offset = avio_rb32(pb);
            break;
        case MKTAG('P', 'S', '2', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->block_align = 0x80 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->sample_rate = 32000;
            st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 28;
            offset = avio_rl32(pb);
            break;
        case MKTAG('X', 'B', '_', '_'):
            st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_IMA_DAT4;
            st->codecpar->block_align = 0x20 * st->codecpar->ch_layout.nb_channels;
            st->codecpar->sample_rate = 44100;
            st->codecpar->bit_rate = 32LL * st->codecpar->ch_layout.nb_channels * 8 *
                                            st->codecpar->sample_rate / 56;
            offset = avio_rl32(pb);
            break;
        default:
            avpriv_request_sample(s, "Unsupported type: %X", type);
            return AVERROR_PATCHWELCOME;
        }
    } else {
        av_assert0(0);
    }
    st->start_time = 0;

    avio_seek(pb, offset, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int ret;

    ret = av_get_packet(s->pb, pkt, par->block_align);
    if (pkt->size >= 4 && AV_RB32(pkt->data) == AV_RB32("\xab\xab\xab\xab")) {
        pkt->size = 0;
        return AVERROR_EOF;
    }

    return ret;
}

const FFInputFormat ff_musx_demuxer = {
    .p.name         = "musx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Eurocom MUSX"),
    .p.extensions   = "sfx,musx",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
