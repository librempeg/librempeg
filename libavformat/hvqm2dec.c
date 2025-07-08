/*
 * HVQM2 demuxer
 * Copyright (c) 2024 Paul B Mahol
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

typedef struct HVQM2Block {
    int16_t type;
    int16_t format;
    int64_t start;
    int64_t stop;
} HVQM2Block;

typedef struct HVQM2Context {
    uint32_t nb_blocks;
    HVQM2Block *block;
    uint32_t current_block;
} HVQM2Context;

static int hvqm2_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "HVQM2 1.0", 9))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int hvqm2_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *vst, *ast;
    uint32_t usec_per_frame;
    int audio_format, ret;

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    avio_skip(pb, 16);
    avio_skip(pb, 4);

    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id   = AV_CODEC_ID_HVQM2;
    vst->codecpar->width      = avio_rb16(pb);
    vst->codecpar->height     = avio_rb16(pb);

    ret = ff_get_extradata(s, vst->codecpar, pb, 4);
    if (ret < 0)
        return ret;

    vst->start_time = 0;
    vst->nb_frames = avio_rb32(pb);
    vst->duration = vst->nb_frames;
    usec_per_frame = avio_rb32(pb);
    avio_skip(pb, 8);

    avpriv_set_pts_info(vst, 64, usec_per_frame, 1000000);

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    audio_format              = avio_r8(pb);
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->ch_layout.nb_channels = avio_r8(pb);
    ast->codecpar->bits_per_coded_sample = avio_r8(pb);
    switch (audio_format) {
    case 0:
        ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        break;
    case 1:
        ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_HVQM2;
        break;
    }
    avio_skip(pb, 5);
    ast->codecpar->sample_rate = avio_rb32(pb);
    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
    ffstream(ast)->need_parsing = AVSTREAM_PARSE_HEADERS;

    avio_skip(pb, 4);

    return 0;
}

static int hvqm2_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    HVQM2Context *hvqm2 = s->priv_data;
    AVIOContext *pb = s->pb;
    int record_type, frame_type, ret, new_block = 1;
    int64_t pos;
    int32_t size;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);

    for (int i = 0; i < hvqm2->nb_blocks; i++) {
        HVQM2Block *block = &hvqm2->block[i];

        if (pos >= block->start && pos < block->stop) {
            hvqm2->current_block = i;
            new_block = 0;
            break;
        }
    }

    if (new_block) {
        HVQM2Block *block;

        hvqm2->block = av_realloc_f(hvqm2->block, sizeof(*hvqm2->block), hvqm2->nb_blocks+1);
        if (!hvqm2->block)
            return AVERROR(ENOMEM);
        block = &hvqm2->block[hvqm2->nb_blocks];
        block->start = pos;
        block->type = record_type = avio_rb16(pb);
        block->format = frame_type = avio_rb16(pb);
        size = avio_rb32(pb);
        block->stop = block->start + size + 8LL;

        hvqm2->nb_blocks++;
    } else {
        if (pos == hvqm2->block[hvqm2->current_block].start) {
            avio_skip(pb, 8);
            record_type = hvqm2->block[hvqm2->current_block].type;
            frame_type = hvqm2->block[hvqm2->current_block].format;
            size = hvqm2->block[hvqm2->current_block].stop -
                   hvqm2->block[hvqm2->current_block].start - 8LL;
        } else {
            return AVERROR(EIO);
        }
    }

    ret = av_new_packet(pkt, size + 2);
    if (ret < 0)
        return ret;

    AV_WB16(pkt->data, frame_type);
    ret = avio_read(pb, pkt->data + 2, size);
    if (ret < 0)
        return ret;

    pkt->pos = pos;
    pkt->stream_index = record_type ? 0 : 1;
    if (frame_type == 0)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

static int hvqm2_read_close(AVFormatContext *s)
{
    HVQM2Context *hvqm2 = s->priv_data;

    av_freep(&hvqm2->block);

    return 0;
}

const FFInputFormat ff_hvqm2_demuxer = {
    .p.name         = "hvqm2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("HVQM2"),
    .priv_data_size = sizeof(HVQM2Context),
    .read_probe     = hvqm2_probe,
    .read_header    = hvqm2_read_header,
    .read_packet    = hvqm2_read_packet,
    .read_close     = hvqm2_read_close,
    .p.extensions   = "hvqm",
    .p.flags        = AVFMT_GENERIC_INDEX,
};
