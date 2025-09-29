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

typedef struct WSIDemuxContext {
    int64_t start;
    int16_t *coeffs;
} WSIDemuxContext;

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
    WSIDemuxContext *wsi = s->priv_data;
    int rate, channels, block_size;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    wsi->start = start = avio_rb32(pb);
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
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_THP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start+0x2c, SEEK_SET);
    wsi->coeffs = av_calloc(channels, 16 * sizeof(*wsi->coeffs));
    if (!wsi->coeffs)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, (uint8_t *)(wsi->coeffs + ch * 16), 32);
        avio_skip(pb, block_size - 32);
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    const int channels = s->streams[0]->codecpar->ch_layout.nb_channels;
    WSIDemuxContext *wsi = s->priv_data;
    int block_size, ret, skip = 16;
    AVIOContext *pb = s->pb;
    int64_t pos;

    pos = avio_tell(pb);
    if (pos == wsi->start)
        skip += 0x60;
    block_size = avio_rb32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (block_size <= skip + 8 || block_size > (INT_MAX - 8)/channels - 32 - 4)
        return AVERROR_INVALIDDATA;

    ret = av_new_packet(pkt, ((block_size - skip) + 32 + 4) * channels + 8);
    if (ret < 0)
        return ret;

    AV_WB32(pkt->data, ((block_size - skip) + 32 + 4) * channels + 8);
    AV_WB32(pkt->data + 4, ((block_size - skip) / 8) * 14);
    for (int ch = 0; ch < channels; ch++) {
        memcpy(pkt->data + 8 + 32 * ch, wsi->coeffs + 16 * ch, 32);
        AV_WB32(pkt->data + 8 + 32 * channels + 4 * ch, 0);
        avio_skip(pb, 12 + 4 * (ch > 0) + skip - 16);
        avio_read(pb, pkt->data + 8 + (32 + 4) * channels + ch * (block_size - skip), block_size - skip);
    }

    pkt->stream_index = 0;
    pkt->duration = ((block_size - skip) / 8) * 14;
    pkt->pos = pos;

    return ret;
}

static int read_close(AVFormatContext *s)
{
    WSIDemuxContext *wsi = s->priv_data;

    av_freep(&wsi->coeffs);

    return 0;
}

const FFInputFormat ff_wsi_demuxer = {
    .p.name         = "wsi",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Wii WSI"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(WSIDemuxContext),
    .p.extensions   = "wsi",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
};
