/*
 * DERF Video demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct DERFDemuxContext {
    int width, height;
    int mono_stream_index;
    int stereo_stream_index;
    int video_stream_index;
} DERFDemuxContext;

static int derfvideo_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('D','E','R','F'))
        return 0;

    if (AV_RL32(p->buf+4) <= 4)
        return 0;

    if (p->buf_size < 14)
        return 0;

    if (AV_RL16(p->buf+8) == 0)
        return 0;
    if (AV_RL16(p->buf+10) == 0)
        return 0;
    if (AV_RL16(p->buf+12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int derfvideo_read_header(AVFormatContext *s)
{
    DERFDemuxContext *d = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 10);

    s->ctx_flags |= AVFMTCTX_NOHEADER;
    d->width = avio_rl16(pb);
    d->height = avio_rl16(pb);
    d->stereo_stream_index = -1;
    d->video_stream_index = -1;
    d->mono_stream_index = -1;

    avio_skip(pb, 22);

    return 0;
}

static int derfvideo_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DERFDemuxContext *d = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    while (1) {
        uint32_t chunk_size;
        int chunk_type;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        chunk_type = avio_rb16(pb);
        avio_skip(pb, 2);
        chunk_size = avio_rl32(pb);
        if (chunk_size <= 16)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 8);
        chunk_size -= 16;

        switch (chunk_type) {
        case (('C'<<8)|'S'):
            if (d->stereo_stream_index == -1) {
                AVStream *st = avformat_new_stream(s, NULL);

                if (!st)
                    return AVERROR(ENOMEM);

                st->start_time = 0;
                st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                st->codecpar->codec_id   = AV_CODEC_ID_DERF_DPCM;
                av_channel_layout_default(&st->codecpar->ch_layout, 2);
                st->codecpar->sample_rate = 22050;
                avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
                d->stereo_stream_index = st->index;
            }

            ret = av_get_packet(pb, pkt, chunk_size);
            pkt->stream_index = d->stereo_stream_index;
            pkt->pos = pos;
            return ret;
        case (('C'<<8)|'M'):
            if (d->mono_stream_index == -1) {
                AVStream *st = avformat_new_stream(s, NULL);

                if (!st)
                    return AVERROR(ENOMEM);

                st->start_time = 0;
                st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                st->codecpar->codec_id   = AV_CODEC_ID_DERF_DPCM;
                av_channel_layout_default(&st->codecpar->ch_layout, 1);
                st->codecpar->sample_rate = 22050;
                avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
                d->mono_stream_index = st->index;
            }

            ret = av_get_packet(pb, pkt, chunk_size);
            pkt->stream_index = d->mono_stream_index;
            pkt->pos = pos;
            return ret;
        case (('F'<<8)|'K'):
        case (('K'<<8)|'B'):
        default:
            avio_skip(pb, chunk_size);
            break;
        }
    }

    return ret;
}

const FFInputFormat ff_derfvideo_demuxer = {
    .p.name         = "derfvideo",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Xilam DERF Video"),
    .p.extensions   = "vds",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(DERFDemuxContext),
    .read_probe     = derfvideo_probe,
    .read_header    = derfvideo_read_header,
    .read_packet    = derfvideo_read_packet,
};
