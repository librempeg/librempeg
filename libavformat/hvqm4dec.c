/*
 * HVQM4 demuxer
 * Copyright (c) 2021 Paul B Mahol
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

typedef struct HVQM4Block {
    int64_t start;
    int64_t stop;
} HVQM4Block;

typedef struct HVQM4Context {
    uint32_t nb_blocks;
    HVQM4Block *block;
    uint32_t current_block;
} HVQM4Context;

static int hvqm4_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "HVQM4 1.3", 9) &&
        memcmp(p->buf, "HVQM4 1.5", 9))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int hvqm4_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *vst, *ast;
    uint32_t header_size, usec_per_frame;
    int audio_format, ret;

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    avio_skip(pb, 8);
    ret = ff_get_extradata(s, vst->codecpar, pb, 1);
    if (ret < 0)
        return ret;
    avio_skip(pb, 7);

    header_size = avio_rb32(pb);
    avio_skip(pb, 8);
    vst->start_time = 0;
    vst->nb_frames = avio_rb32(pb);
    vst->duration = vst->nb_frames;
    avio_skip(pb, 4);
    usec_per_frame = avio_rb32(pb);
    avio_skip(pb, 12);

    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id   = AV_CODEC_ID_HVQM4;
    vst->codecpar->width      = avio_rb16(pb);
    vst->codecpar->height     = avio_rb16(pb);

    avio_skip(pb, 2);
    avio_skip(pb, 2);

    avpriv_set_pts_info(vst, 64, usec_per_frame, 1000000);

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->ch_layout.nb_channels = avio_r8(pb);
    ast->codecpar->bits_per_coded_sample = avio_r8(pb);
    audio_format              = avio_r8(pb);
    switch (audio_format) {
    case 0:
        ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_HVQM4;
        break;
    case 1:
        ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        break;
    }
    avio_skip(pb, 1);
    ast->codecpar->sample_rate = avio_rb32(pb);
    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
    ffstream(ast)->need_parsing = AVSTREAM_PARSE_HEADERS;

    avio_skip(pb, header_size - avio_tell(pb));

    return 0;
}

static int hvqm4_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    HVQM4Context *hvqm4 = s->priv_data;
    AVIOContext *pb = s->pb;
    int media_type, frame_type, ret, new_block = 1;
    int64_t pos, ppos;
    int32_t size;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);

    for (int i = 0; i < hvqm4->nb_blocks; i++) {
        HVQM4Block *block = &hvqm4->block[i];

        if (pos >= block->start && pos < block->stop) {
            hvqm4->current_block = i;
            new_block = 0;
            break;
        }
    }

    if (new_block) {
        HVQM4Block *block;

        hvqm4->block = av_realloc_f(hvqm4->block, sizeof(*hvqm4->block), hvqm4->nb_blocks+1);
        if (!hvqm4->block)
            return AVERROR(ENOMEM);
        block = &hvqm4->block[hvqm4->nb_blocks];
        block->start = pos;
        avio_skip(pb, 4);
        block->stop = block->start + avio_rb32(pb) + 20;
        avio_skip(pb, 8);
        avio_skip(pb, 4);

        hvqm4->nb_blocks++;
    } else {
        if (pos == hvqm4->block[hvqm4->current_block].start)
            avio_skip(pb, 20);
    }

    ppos = avio_tell(pb);
    media_type = avio_rb16(pb);
    frame_type = avio_rb16(pb);
    size = avio_rb32(pb);
    ret = av_new_packet(pkt, size + 2);
    if (ret < 0)
        return ret;

    AV_WB16(pkt->data, frame_type);
    ret = avio_read(pb, pkt->data + 2, size);
    if (ret < 0)
        return ret;

    pkt->pos = ppos;
    pkt->stream_index = media_type ? 0 : 1;
    if ((frame_type == 0x10 && media_type == 1) ||
        media_type == 0)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

static int hvqm4_read_close(AVFormatContext *s)
{
    HVQM4Context *hvqm4 = s->priv_data;

    av_freep(&hvqm4->block);

    return 0;
}

const FFInputFormat ff_hvqm4_demuxer = {
    .p.name         = "hvqm4",
    .p.long_name    = NULL_IF_CONFIG_SMALL("HVQM4"),
    .priv_data_size = sizeof(HVQM4Context),
    .read_probe     = hvqm4_probe,
    .read_header    = hvqm4_read_header,
    .read_packet    = hvqm4_read_packet,
    .read_close     = hvqm4_read_close,
    .p.extensions   = "h4m",
    .p.flags        = AVFMT_GENERIC_INDEX,
};
