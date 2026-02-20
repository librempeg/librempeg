/*
 * IPS video demuxer
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
#include "libavutil/mem.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct IPSContext {
     uint8_t *packet_stream_index;
     int audio_stream_index;
     int video_stream_index;
     int sample_rate;
     int nb_channels;
     int start_offset;
     int fps;
     int64_t video_start;
} IPSContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 44)
        return 0;

    if (AV_RB32(p->buf+8) != MKBETAG('I', 'P', 'S', '0'))
        return 0;

    if (AV_RB32(p->buf+40) != MKBETAG('A', 'P', 'C', 'M'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    IPSContext *ips = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 17);
    ips->video_stream_index = -1;
    ips->audio_stream_index = -1;
    ips->fps = avio_r8(pb);
    avio_skip(pb, 1);
    ips->nb_channels = avio_r8(pb);
    ips->sample_rate = avio_rl16(pb);
    avio_skip(pb, 2);
    ips->start_offset = avio_rl16(pb);
    if (ips->start_offset <= 0x40)
        return AVERROR_INVALIDDATA;

    ips->packet_stream_index = av_calloc(ips->start_offset - 0x40, sizeof(*ips->packet_stream_index));
    if (!ips->packet_stream_index)
        return AVERROR(ENOMEM);

    avio_seek(pb, 0x40, SEEK_SET);
    avio_read(pb, ips->packet_stream_index, ips->start_offset - 0x40);

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    IPSContext *ips = s->priv_data;
    int ret, index, idx, pkt_size;
    AVIOContext *pb = s->pb;
    int64_t pos;

    pos = avio_tell(pb);
    idx = (pos - ips->start_offset) / 0x1000;
    if ((idx>>3) >= (ips->start_offset - 0x40))
        return AVERROR_EOF;

    index = !!(ips->packet_stream_index[idx >> 3] & (1 << (idx & 7)));

    if (index) {
        pkt_size = 0x1000;
        if (ips->audio_stream_index < 0) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->block_align = 0x100 * ips->nb_channels;
            av_channel_layout_default(&st->codecpar->ch_layout, ips->nb_channels);
            st->codecpar->sample_rate = ips->sample_rate;
            st->start_time = 0;
            ips->audio_stream_index = st->index;

            avpriv_set_pts_info(st, 64, 1, ips->sample_rate);
        }
        index = ips->audio_stream_index;
    } else {
        pkt_size = 0x1000;
        if (ips->video_stream_index < 0) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            avio_skip(pb, 8);

            st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            st->codecpar->codec_id = AV_CODEC_ID_IPU;
            st->codecpar->width = avio_rl16(pb);
            st->codecpar->height = avio_rl16(pb);
            st->start_time = 0;
            st->duration =
            st->nb_frames = avio_rl32(pb);
            ips->video_stream_index = st->index;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

            avpriv_set_pts_info(st, 64, 1, ips->fps);

            ips->video_start = pos;
            pkt_size -= 16;
        } else if (pos == ips->video_start) {
            avio_skip(pb, 16);
            pkt_size -= 16;
        }
        index = ips->video_stream_index;
    }

    ret = av_get_packet(pb, pkt, pkt_size);

    pkt->pos = pos;
    pkt->stream_index = index;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

static int read_close(AVFormatContext *s)
{
    IPSContext *ips = s->priv_data;

    av_freep(&ips->packet_stream_index);

    return 0;
}

const FFInputFormat ff_ips_demuxer = {
    .p.name         = "ips",
    .p.long_name    = NULL_IF_CONFIG_SMALL("IPS PS2 Movie"),
    .p.extensions   = "ips",
    .priv_data_size = sizeof(IPSContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
};
