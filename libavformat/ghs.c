/*
 * GHS demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef unsigned int (*avio_r32)(AVIOContext *s);
typedef unsigned int (*avio_r16)(AVIOContext *s);

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('G','H','S',' '))
        return 0;
    if (AV_RB32(p->buf+4) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct GHSStream {
    int64_t start_offset;
    int64_t stop_offset;
} GHSStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const GHSStream *gs1 = s1->priv_data;
    const GHSStream *gs2 = s2->priv_data;

    return FFDIFFSIGN(gs1->start_offset, gs2->start_offset);
}

static int guess_endian32(AVIOContext *pb)
{
    uint8_t buf[4];

    if (avio_read(pb, buf, sizeof(buf)) != sizeof(buf))
        return AVERROR(EIO);

    return AV_RL32(buf) > AV_RB32(buf);
}

static int read_header(AVFormatContext *s)
{
    int64_t stream_offset = 0, first_start_offset;
    int big_endian, nb_streams;
    AVIOContext *pb = s->pb;
    uint32_t version_test;
    avio_r32 avio_r32;
    avio_r16 avio_r16;
    int ret;

    avio_skip(pb, 4);
    ret = guess_endian32(pb);
    if (ret < 0)
        return ret;
    big_endian = ret;

    avio_r32 = big_endian ? avio_rb32 : avio_rl32;
    avio_r16 = big_endian ? avio_rb16 : avio_rl16;

    avio_seek(pb, 4, SEEK_SET);
    nb_streams = avio_r32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 8);
    version_test = avio_rl32(pb);
    if (version_test < 16) {
        avio_seek(pb, 8, SEEK_SET);

        for (int i = 0; i < nb_streams; i++) {
            int codec, align, channels, rate;
            uint32_t config_data;
            int64_t stream_size;
            GHSStream *gst;
            AVStream *st;

            codec = avio_r32(pb);
            stream_size = avio_r32(pb);
            channels = avio_r32(pb);
            rate = avio_r32(pb);
            avio_skip(pb, 16);
            config_data = avio_r32(pb);

            switch (codec) {
            case 0x0000:
                codec = AV_CODEC_ID_PCM_S16LE;
                align = 2;
                break;
            case 0x0001:
                codec = AV_CODEC_ID_ADPCM_HEVAG;
                align = 16;
                break;
            case 0x0002:
                codec = AV_CODEC_ID_ATRAC9;
                align = 1;
                break;
            default:
                avpriv_request_sample(s, "codec 0x%X", codec);
                stream_offset += stream_size;
                continue;
            }

            if (rate <= 0 || channels <= 0 || align >= INT_MAX/channels)
                return AVERROR_INVALIDDATA;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            gst = av_mallocz(sizeof(*gst));
            if (!gst)
                return AVERROR(ENOMEM);

            st->priv_data = gst;

            gst->start_offset = stream_offset;
            gst->stop_offset = gst->start_offset;
            gst->stop_offset += stream_size;

            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = codec;
            st->codecpar->sample_rate = rate;
            st->codecpar->ch_layout.nb_channels = channels;
            st->codecpar->block_align = align * channels;

            if (codec == AV_CODEC_ID_ATRAC9) {
                ret = ff_alloc_extradata(st->codecpar, 12);
                if (ret < 0)
                    return ret;
                memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
                config_data = av_bswap32(config_data);
                AV_WB32(st->codecpar->extradata + 4, config_data);

                st->codecpar->block_align = (1 << ((config_data>>3) & 3)) * (((config_data >> 5) & 0x7ff) + 1);
            }

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

            stream_offset += stream_size;
        }

        int64_t start = avio_tell(pb);
        if (avio_rb32(pb) == MKBETAG('S','T','P','R')) {
            start += avio_r32(pb);
            start += 8;
        }

        for (int n = 0; n < s->nb_streams; n++) {
            AVStream *st = s->streams[n];
            GHSStream *gst = st->priv_data;

            gst->start_offset += start;
            gst->stop_offset += start;
        }
    } else {
        avio_seek(pb, 12, SEEK_SET);

        for (int i = 0; i < nb_streams; i++) {
            int64_t stream_size, stream_offset, duration;
            int codec, align, channels, rate;
            GHSStream *gst;
            AVStream *st;

            codec = avio_r16(pb);
            channels = avio_r16(pb);
            rate = avio_r32(pb);
            avio_skip(pb, 4);
            align = avio_r16(pb);
            avio_skip(pb, 10);
            duration = avio_r32(pb);
            avio_skip(pb, 48);
            stream_offset = avio_r32(pb);
            stream_size = avio_r32(pb);
            avio_skip(pb, 16);

            switch (codec) {
            case 0x0001:
                codec = AV_CODEC_ID_PCM_S16LE;
                break;
            case 0x0002:
                codec = AV_CODEC_ID_ADPCM_MS;
                break;
            case 0x0166:
                codec = AV_CODEC_ID_XMA2;
                align = 2048;
                break;
            default:
                avpriv_request_sample(s, "codec 0x%X", codec);
                continue;
            }

            if (rate <= 0 || channels <= 0 || align <= 0)
                return AVERROR_INVALIDDATA;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            gst = av_mallocz(sizeof(*gst));
            if (!gst)
                return AVERROR(ENOMEM);

            st->priv_data = gst;

            gst->start_offset = stream_offset;
            gst->stop_offset = gst->start_offset;
            gst->stop_offset += stream_size;

            st->start_time = 0;
            if (duration > 0)
                st->duration = duration;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = codec;
            st->codecpar->sample_rate = rate;
            st->codecpar->ch_layout.nb_channels = channels;
            st->codecpar->block_align = align;

            if (codec == AV_CODEC_ID_XMA2) {
                ret = ff_alloc_extradata(st->codecpar, 34);
                if (ret < 0)
                    return ret;
                memset(st->codecpar->extradata, 0, 34);
                AV_WL16(st->codecpar->extradata, (channels+1)/2);
                ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            }

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        if (n == 0) {
            GHSStream *gst = st->priv_data;

            first_start_offset = gst->start_offset;
        }

        st->index = n;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        GHSStream *gst = st->priv_data;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= gst->start_offset && pos < gst->stop_offset) {
            int size = ff_pcm_default_packet_size(par);
            size = FFMIN(size, gst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);
            pkt->stream_index = st->index;
            break;
        } else if (pos >= gst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            GHSStream *gst_next = st_next->priv_data;

            if (gst_next->start_offset > pos)
                avio_seek(pb, gst_next->start_offset, SEEK_SET);
        }
    }
    return ret;
}

const FFInputFormat ff_ghs_demuxer = {
    .p.name         = "ghs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Hexadrive GHS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "gtd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
