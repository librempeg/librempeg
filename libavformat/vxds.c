/*
 * Actimagine VX video demuxer
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

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct VXContext {
     int block_idx;
     int block_count;
} VXContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 28)
        return 0;

    if (AV_RB32(p->buf) != MKBETAG('V','X','D','S'))
        return 0;
    if ((int)AV_RL32(p->buf+4) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;
    if (AV_RL32(p->buf+16) == 0)
        return 0;
    if (AV_RL32(p->buf+20) <= 0 ||
        AV_RL32(p->buf+20) >= 256)
        return 0;
    if ((int)AV_RL32(p->buf+24) < 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, quant, rate, nb_frames, width, height, fps, nb_audio_streams, nb_entries;
    av_unused int64_t audio_offset, video_offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    nb_frames = avio_rl32(pb);
    width = avio_rl32(pb);
    height = avio_rl32(pb);
    fps = avio_rl32(pb);
    quant = avio_rl32(pb);
    rate = avio_rl32(pb);
    nb_audio_streams = avio_rl32(pb);
    avio_skip(pb, 4);
    audio_offset = avio_rl32(pb);
    video_offset = avio_rl32(pb);
    nb_entries = avio_rl32(pb);
    if (nb_frames <= 0 || width <= 0 || height <= 0 || rate < 0 || nb_entries <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->nb_frames = st->duration = nb_frames;
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_ACTIMAGINE_VX;
    st->codecpar->width = width;
    st->codecpar->height = height;

    avpriv_set_pts_info(st, 64, 1 << 16, fps);

    ret = ff_alloc_extradata(st->codecpar, 4);
    if (ret < 0)
        return ret;
    AV_WL32(st->codecpar->extradata, quant);

    if (nb_audio_streams > 0) {
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret, pkt_size;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    pkt_size = avio_rl16(pb);
    if (pkt_size <= 2)
        return AVERROR_EOF;
    avio_skip(pb, 2);
    pkt_size += 2;
    ret = av_new_packet(pkt, pkt_size);
    if (ret < 0)
        return ret;
    AV_WL32(pkt->data, 0);
    avio_read(pb, pkt->data+4, pkt_size-4);

    pkt->pos = pos;
    pkt->duration = 1;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_vxds_demuxer = {
    .p.name         = "vxds",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Actimagine VX DS"),
    .p.extensions   = "vx",
    .priv_data_size = sizeof(VXContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
