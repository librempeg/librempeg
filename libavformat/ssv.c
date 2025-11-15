/*
 * SSV demuxer
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
#include "avio_internal.h"

#define OPAOPA 0x4f50414f50410000

typedef struct SSVDemuxContext {
    int32_t read_chunk;
    int64_t pos;

    int video_stream_index;
    int audio_stream_index;
} SSVDemuxContext;

static int ssv_read_probe(const AVProbeData *p)
{
    if (p->buf_size < 8)
        return 0;

    if (memcmp(p->buf, "KAMEKAME", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int ssv_read_header(AVFormatContext *s)
{
    SSVDemuxContext *ssv = s->priv_data;
    AVIOContext *pb = s->pb;
    uint64_t state = 0;
    int found = 0;

    ssv->video_stream_index = -1;
    ssv->audio_stream_index = -1;

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    while (!avio_feof(pb)) {
        state <<= 8;
        state |= avio_r8(pb);

        if (state == OPAOPA) {
            found = 1;
            break;
        }
    }

    if (found == 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, -8, SEEK_CUR);

    return 0;
}

static int ssv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SSVDemuxContext *ssv = s->priv_data;
    int32_t packet_size, video_size;
    AVIOContext *pb = s->pb;
    uint64_t chunk;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ssv->read_chunk > 32) {
        chunk = avio_rb32(pb);
        if (chunk != MKBETAG('d','y','n','a'))
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 28);

        if (ssv->audio_stream_index == -1) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id   = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->sample_rate = 48000;
            st->codecpar->ch_layout.nb_channels = 2;
            st->codecpar->block_align = 0x7260;

            st->start_time = 0;
            ssv->audio_stream_index = st->index;

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }

        ret = av_get_packet(pb, pkt, 0x7260);
        if (ret < 0)
            return ret;

        pkt->stream_index = ssv->audio_stream_index;
        pkt->pos = ssv->pos;

        avio_skip(pb, FFMAX(0, ssv->read_chunk - 32 - 0x7260));

        ssv->read_chunk = 0;

        return 0;
    }

    ssv->pos = avio_tell(pb);
    chunk = avio_rb64(pb);
    if (chunk != OPAOPA)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 8);
    packet_size = avio_rl32(pb);
    video_size = avio_rl32(pb);
    if (video_size <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);

    ssv->read_chunk = avio_rl32(pb);
    if (ssv->read_chunk && ssv->read_chunk <= 32)
        return AVERROR_INVALIDDATA;

    if (ssv->video_stream_index == -1) {
        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id   = AV_CODEC_ID_MPEG2VIDEO;

        st->start_time = 0;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
        ssv->video_stream_index = st->index;
    }

    ret = av_get_packet(pb, pkt, video_size);
    if (ret < 0)
        return ret;

    pkt->stream_index = ssv->video_stream_index;
    pkt->pos = ssv->pos;
    pkt->duration = 1;

    return 0;
}

static int ssv_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    SSVDemuxContext *ssv = s->priv_data;

    ssv->read_chunk = 0;

    return -1;
}

const FFInputFormat ff_ssv_demuxer = {
    .p.name         = "ssv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SSV Video"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SSVDemuxContext),
    .read_probe     = ssv_read_probe,
    .read_header    = ssv_read_header,
    .read_packet    = ssv_read_packet,
    .read_seek      = ssv_read_seek,
};
