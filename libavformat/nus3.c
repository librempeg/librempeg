/*
 * NUS3 demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct NUS3Stream {
    int index;
    int64_t start, stop, name;
} NUS3Stream;

typedef struct NUS3DemuxContext {
    int nb_streams;
    NUS3Stream *streams;

    int current_stream;
} NUS3DemuxContext;

static int nus3_probe(const AVProbeData *p)
{
    int score = 50;

    if (AV_RL32(p->buf) != MKTAG('N','U','S','3'))
        return 0;

    if (p->buf_size < 16)
        return 0;

    if (AV_RL32(p->buf+8) != MKTAG('A','U','D','I'))
        return 0;

    if (AV_RL32(p->buf+12) == MKTAG('I','N','D','X'))
        score += 50;

    return score;
}

static int nus3_read_header(AVFormatContext *s)
{
    NUS3DemuxContext *nus3 = s->priv_data;
    AVIOContext *pb = s->pb;
    int found = 0;

    avio_skip(pb, 12);

    while (!avio_feof(pb)) {
        uint32_t chunk, size;

        chunk = avio_rb32(pb);
        size = avio_rl32(pb);

        switch (chunk) {
        case MKBETAG('I','N','D','X'):
            if (!nus3->streams) {
                nus3->nb_streams = avio_rl32(pb);
                if (nus3->nb_streams < 1)
                    return AVERROR_INVALIDDATA;

                nus3->streams = av_calloc(nus3->nb_streams, sizeof(*nus3->streams));
                if (!nus3->streams)
                    return AVERROR(ENOMEM);

                size -= 4;
            }
            break;
        case MKBETAG('A','D','O','F'):
            if (size < 8LL * nus3->nb_streams)
                return AVERROR_INVALIDDATA;

            for (int n = 0; n < nus3->nb_streams; n++) {
                if (avio_feof(pb))
                    return AVERROR_INVALIDDATA;

                nus3->streams[n].index = -1;
                nus3->streams[n].start = avio_rl32(pb);
                nus3->streams[n].stop = nus3->streams[n].start + avio_rl32(pb);

                size -= 8;
            }

            found = 1;
            break;
        case MKBETAG('N','M','O','F'):
            if (size < 4LL * nus3->nb_streams)
                return AVERROR_INVALIDDATA;

            for (int n = 0; n < nus3->nb_streams; n++) {
                if (avio_feof(pb))
                    return AVERROR_INVALIDDATA;

                nus3->streams[n].name = avio_rl32(pb);

                size -= 4;
            }
            break;
        case MKBETAG('P','A','C','K'):
        case MKBETAG('J','U','N','K'):
        default:
            break;
        }

        if (found)
            break;

        avio_skip(pb, size);
    }

    if (!found)
        return AVERROR_INVALIDDATA;

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    avio_seek(pb, nus3->streams[0].start, SEEK_SET);

    return 0;
}

static int nus3_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NUS3DemuxContext *nus3 = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos, offset;
    int size, ret;

    if (nus3->streams[0].index == -1) {
        uint32_t codec, duration, skip = 0;
        int nb_channels, rate;
        AVStream *st;

        codec = avio_rb32(pb);
        switch (codec) {
        case MKBETAG('O','P','U','S'):
            codec = AV_CODEC_ID_OPUS;
            avio_skip(pb, 4);
            duration = avio_rb32(pb);
            avio_skip(pb, 52);
            pos = avio_tell(pb);
            if (avio_rl32(pb) != 0x80000001)
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 5);
            nb_channels = avio_r8(pb);
            avio_skip(pb, 2);
            rate = avio_rl32(pb);
            offset = avio_rl32(pb) + 8LL;
            avio_skip(pb, 10);
            skip = avio_rl16(pb);
            if (avio_rl32(pb) != 0x80000004)
                return AVERROR_INVALIDDATA;

            avio_seek(pb, pos + offset, SEEK_SET);
            break;
        default:
            avpriv_request_sample(s, "codec 0x%04X", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (rate <= 0 || nb_channels <= 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        nus3->streams[0].index = st->index;

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = nb_channels;
        st->codecpar->sample_rate = rate;;
        st->codecpar->codec_id = codec;
        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        ret = ff_alloc_extradata(st->codecpar, 19 + 2 + nb_channels);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

        st->codecpar->extradata[9] = nb_channels;
        AV_WL16(st->codecpar->extradata + 10, skip);
    }

    pos = avio_tell(pb);
    if (pos >= nus3->streams[0].stop)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rb32(pb);
    if (size < 2 &&
        size > nus3->streams[0].stop - pos - 8)
        return AVERROR(EIO);

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = nus3->streams[0].index;

    return ret;
}

static int nus3_read_seek(AVFormatContext *s, int stream_index,
                          int64_t timestamp, int flags)
{
    NUS3DemuxContext *nus3 = s->priv_data;

    nus3->current_stream = FFMAX(stream_index, 0);

    return -1;
}

static int nus3_read_close(AVFormatContext *s)
{
    NUS3DemuxContext *nus3 = s->priv_data;

    av_freep(&nus3->streams);

    return 0;
}

const FFInputFormat ff_nus3_demuxer = {
    .p.name         = "nus3",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco NUS3"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nus3audio",
    .priv_data_size = sizeof(NUS3DemuxContext),
    .read_probe     = nus3_probe,
    .read_header    = nus3_read_header,
    .read_packet    = nus3_read_packet,
    .read_seek      = nus3_read_seek,
    .read_close     = nus3_read_close,
};
