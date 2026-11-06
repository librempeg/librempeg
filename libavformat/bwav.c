/*
 * BWAV demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct BWAVStream {
    int64_t start_offset;
    int64_t stop_offset;
} BWAVStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('B','W','A','V'))
        return 0;

    if (p->buf_size < 32)
        return 0;
    if (AV_RB16(p->buf+4) != 0xFFFE)
        return 0;
    if (AV_RL16(p->buf+6) != 1)
        return 0;
    if ((int)AV_RL32(p->buf+20) <= 0)
        return 0;
    if (AV_RL32(p->buf+28) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const BWAVStream *bs1 = s1->priv_data;
    const BWAVStream *bs2 = s2->priv_data;

    return FFDIFFSIGN(bs1->start_offset, bs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int ret, codec, rate, nb_channels, align;
    int64_t offset, duration;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 14);
    nb_channels = avio_rl16(pb);
    codec = avio_rl16(pb);
    avio_skip(pb, 2);
    rate = avio_rl32(pb);
    avio_skip(pb, 4);
    duration = avio_rl32(pb);

    if (nb_channels > 1) {
        avio_seek(pb, 0x8c, SEEK_SET);
        align = avio_rl32(pb);
    } else {
        align = 8;
    }

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16LE_PLANAR;
        break;
    case 1:
        codec = AV_CODEC_ID_ADPCM_NDSP_LE;
        break;
    case 2:
        codec = AV_CODEC_ID_OPUS;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (align <= 0 || rate <= 0 || nb_channels <= 0 || nb_channels > INT_MAX/align)
        return AVERROR_INVALIDDATA;

    if (codec == AV_CODEC_ID_OPUS) {
        for (int ch = 0; ch < nb_channels; ch++) {
            int64_t bst_offset, bst_size;
            BWAVStream *bst;
            int skip = 0;

            avio_seek(pb, 0x10 + 0x4C * ch + 0x34, SEEK_SET);
            bst_offset = avio_rl32(pb);
            avio_seek(pb, bst_offset + 0x24, SEEK_SET);
            bst_size = avio_rl32(pb) + 0x28;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            bst = av_mallocz(sizeof(*bst));
            if (!bst)
                return AVERROR(ENOMEM);

            bst->start_offset = bst_offset + 0x28;
            bst->stop_offset = bst_offset + bst_size;

            st->start_time = 0;
            st->priv_data = bst;
            st->duration = duration;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = codec;
            st->codecpar->sample_rate = rate;
            st->codecpar->ch_layout.nb_channels = 1;

            ret = ff_alloc_extradata(st->codecpar, 19 + (2 + 1));
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

            memcpy(st->codecpar->extradata, "OpusHead", 8);
            st->codecpar->extradata[8] = 1;
            st->codecpar->extradata[9] = 1;
            AV_WL16(st->codecpar->extradata + 10, skip);
            AV_WL32(st->codecpar->extradata + 12, 48000);

            ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;
            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }

        qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
        for (int n = 0; n < s->nb_streams; n++) {
            AVStream *st = s->streams[n];

            st->index = n;
        }

        {
            AVStream *st = s->streams[0];
            BWAVStream *bst = st->priv_data;

            avio_seek(pb, bst->start_offset, SEEK_SET);
        }
    } else {
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->sample_rate = rate;
        st->codecpar->ch_layout.nb_channels = nb_channels;
        st->codecpar->block_align = align * nb_channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_NDSP_LE) {
            avio_seek(pb, 0x20, SEEK_SET);
            if ((ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
                return ret;

            for (int ch = 0; ch < st->codecpar->ch_layout.nb_channels; ch++) {
                avio_read(pb, st->codecpar->extradata + ch * 32, 32);
                avio_skip(pb, 0x2c);
            }
        }

        avio_seek(pb, 0x44, SEEK_SET);
        offset = avio_rl32(pb);
        avio_seek(pb, offset, SEEK_SET);

    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (s->streams[0]->codecpar->codec_id != AV_CODEC_ID_OPUS) {
        return ff_pcm_read_packet(s, pkt);
    } else {
        int64_t pos, block_size;
        AVIOContext *pb = s->pb;
        int ret = AVERROR_EOF;

        for (int i = 0; i < s->nb_streams; i++) {
            AVStream *st = s->streams[i];
            BWAVStream *bst = st->priv_data;

            if (avio_feof(pb))
                return AVERROR_EOF;

            pos = avio_tell(pb);
            if (pos >= bst->start_offset && pos + 8 < bst->stop_offset) {
                block_size = avio_rb32(pb);
                if (block_size < 2)
                    return AVERROR_INVALIDDATA;

                avio_skip(pb, 4);
                int64_t cpos = avio_tell(pb);
                block_size = FFMIN(bst->stop_offset - cpos, block_size);

                ret = av_get_packet(pb, pkt, block_size);
                pkt->pos = pos;
                pkt->stream_index = st->index;
                break;
            } else if (pos >= bst->stop_offset && i+1 < s->nb_streams) {
                AVStream *st_next = s->streams[i+1];
                BWAVStream *bst_next = st_next->priv_data;
                if (bst_next->start_offset > pos)
                    avio_seek(pb, bst_next->start_offset, SEEK_SET);
            }
        }

        return ret;
    }
}

const FFInputFormat ff_bwav_demuxer = {
    .p.name         = "bwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BWAV (NintendoWare BWAV)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bwav",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
