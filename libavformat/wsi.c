/*
 * Wii WSI demuxer
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

static int read_probe(const AVProbeData *p)
{
    int32_t channels, prev_block_size = 0;
    int64_t offset;
    int score = 0;

    offset = AV_RB32(p->buf);
    channels = AV_RB32(p->buf + 4);
    if (channels <= 0 || channels > 2)
        return 0;

    while (offset < p->buf_size - 4) {
        int32_t block_size = AV_RB32(p->buf + offset);

        if (block_size <= 0)
            return 0;

        offset += block_size;
        if (prev_block_size > 0 && block_size == prev_block_size)
            score += 7;

        for (int ch = 1; ch < channels; ch++) {
            int32_t next_block_size = AV_RB32(p->buf + offset);

            if (next_block_size != block_size)
                return 0;

            score += 7;
            if (score >= AVPROBE_SCORE_MAX)
                break;
            offset += block_size;
        }

        prev_block_size = block_size;
    }

    score = FFMIN(score, AVPROBE_SCORE_MAX);

    return score;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, block_size, ret;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    start = avio_rb32(pb);
    channels = avio_rb32(pb);
    avio_seek(pb, start, SEEK_SET);
    block_size = avio_rb32(pb);
    avio_skip(pb, 12);
    duration = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    if (rate <= 0 || channels <= 0 || block_size <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start+0x2c, SEEK_SET);
    ret = ff_alloc_extradata(st->codecpar, 32 * channels + 1);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
        avio_skip(pb, block_size - 32);
    }
    st->codecpar->extradata[32 * channels] = 0x60;

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    const int channels = s->streams[0]->codecpar->ch_layout.nb_channels;
    AVIOContext *pb = s->pb;
    int block_size, ret;
    int64_t pos;

    pos = avio_tell(pb);
    block_size = avio_rb32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    avio_skip(pb, 12);
    if (block_size <= 16)
        return AVERROR_INVALIDDATA;
    block_size -= 16;

    ret = av_new_packet(pkt, block_size * channels);
    if (ret < 0)
        return ret;
    for (int ch = 0; ch < channels; ch++) {
        if (ch)
            avio_skip(pb, 16);
        avio_read(pb, pkt->data + block_size * ch, block_size);
    }

    pkt->stream_index = 0;
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_wsi_demuxer = {
    .p.name         = "wsi",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Wii WSI"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wsi",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
