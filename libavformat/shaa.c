/*
 * SHAA/SHSA demuxer
 * Copyright (c) 2025 smiRaphi
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

static int shaa_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "SHAA", 4) || AV_RL32(p->buf + 4) != 2)
        return 0;

    if (p->buf_size < 24)
        return 0;

    if ((int32_t)AV_RL32(p->buf + 20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int shaa_read_header(AVFormatContext *s)
{
    int ret;
    uint8_t format;
    uint32_t start_offset, coefs_offset, loop_start, loop_end, title_length;
    char title[1024];
    AVStream *st;
    AVIOContext *pb = s->pb;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->block_align = 512;

    avio_skip(pb, 8);
    start_offset = avio_rl32(pb);
    avio_skip(pb, 4);
    format = avio_r8(pb);
    avio_skip(pb, 3);
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->duration = avio_rl32(pb);
    coefs_offset = avio_tell(pb) + avio_rl32(pb);
    avio_skip(pb, 4);

    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    if (loop_end > 0) {
        av_dict_set_int(&s->metadata, "loop_start", loop_start, 0);
        av_dict_set_int(&s->metadata, "loop_end", loop_end, 0);
    }

    title_length = avio_rl32(pb);
    if (title_length > 0) {
        ret = avio_get_str(pb, title_length, title, sizeof(title));
        if (ret < 0)
            return ret;
        av_dict_set(&s->metadata, "title", title, 0);
    }

    if (format == 1) {
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
    } else if (format == 2) {
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
        avio_seek(pb, coefs_offset, SEEK_SET);
        ret = ff_get_extradata(s, st->codecpar, pb, 0x20);
        if (ret < 0)
            return ret;
    } else {
        avpriv_request_sample(st, "format 0x%X", format);
        return AVERROR_PATCHWELCOME;
    }
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);
    return 0;
}

static int shaa_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_shaa_demuxer = {
    .p.name         = "shaa",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SHAA/SHSA (Nintendo Alarmo)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "shaa,shsa",
    .read_probe     = shaa_probe,
    .read_header    = shaa_read_header,
    .read_packet    = shaa_read_packet,
};
