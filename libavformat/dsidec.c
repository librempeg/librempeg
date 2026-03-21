/*
 * DSI video demuxer
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

typedef struct DSIBlock {
     int type;
     int size;
     int64_t start;
} DSIBlock;

typedef struct DSIContext {
     int audio_stream_index;
     int video_stream_index;

     int block_idx;
     int block_count;
     DSIBlock block[256];
} DSIContext;

static int read_probe(const AVProbeData *p)
{
    int type;

    if (p->buf_size < 0x40)
        return 0;

    if (AV_RL32(p->buf) != 2)
        return 0;
    if (AV_RL32(p->buf+4) != 0x40)
        return 0;
    type = AV_RB16(p->buf+8);

    if (!(type >= 0xc0 && type <= 0xcf) &&
        !(type >= 0xd0 && type <= 0xef))
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int read_header(AVFormatContext *s)
{
    DSIContext *dsi = s->priv_data;

    dsi->video_stream_index = -1;
    dsi->audio_stream_index = -1;

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, index, pkt_size, type, count;
    DSIContext *dsi = s->priv_data;
    int64_t pos, start_offset;
    AVIOContext *pb = s->pb;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (dsi->block_idx >= dsi->block_count) {
        pos = avio_tell(pb);
        if (pos & 0x3ffff)
            avio_skip(pb, 0x40000 - (pos & 0x3ffff));
        pos = avio_tell(pb);
        count = avio_rl32(pb);
        if (count <= 0 || count > 256)
            return AVERROR_INVALIDDATA;

        dsi->block_count = count;
        for (int n = 0; n < count; n++) {
            DSIBlock *block = &dsi->block[n];

            block->start = pos + avio_rl32(pb);
            block->type = avio_rb16(pb);
            avio_skip(pb, 2);
            block->size = avio_rl32(pb);
            if (block->size <= 0)
                return AVERROR_INVALIDDATA;
        }

        dsi->block_idx = 0;
    }

    type = dsi->block[dsi->block_idx].type;
    start_offset = dsi->block[dsi->block_idx].start;
    pkt_size = dsi->block[dsi->block_idx].size;
    dsi->block_idx++;

    if (type >= 0xd0 && type <= 0xef) {
        if (dsi->audio_stream_index < 0) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->block_align = 256 * 2;
            st->codecpar->sample_rate = 48000;
            st->codecpar->ch_layout.nb_channels = 2;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            dsi->audio_stream_index = st->index;

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }

        index = dsi->audio_stream_index;
    } else if (type >= 0xc0 && type <= 0xcf) {
        if (dsi->video_stream_index < 0) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            st->codecpar->codec_id = AV_CODEC_ID_MPEG2VIDEO;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            dsi->video_stream_index = st->index;

            avpriv_set_pts_info(st, 64, 1, 30);
        }

        index = dsi->video_stream_index;
    } else {
        return AVERROR_INVALIDDATA;
    }

    avio_seek(pb, start_offset, SEEK_SET);
    ret = av_get_packet(pb, pkt, pkt_size);

    pkt->pos = pos;
    pkt->stream_index = index;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    DSIContext *dsi = s->priv_data;

    dsi->block_idx = 0;
    dsi->block_count = 0;

    return -1;
}

const FFInputFormat ff_dsi_demuxer = {
    .p.name         = "dsi",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DSI PS2 Movie"),
    .p.extensions   = "dsi",
    .priv_data_size = sizeof(DSIContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
