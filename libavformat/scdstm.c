/*
 * Sonic CD STM demuxer
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

typedef struct SCDSTMDemuxContext {
    int audio;
} SCDSTMDemuxContext;

static int scdstm_probe(const AVProbeData *p)
{
    int off;
    uint16_t mode = AV_RB16(p->buf);

    if (mode == 0x3038 || mode == 0x3135 || mode == 0x4E4F || mode == 0x5253 || mode == 0x5352)
        off = 0x2800;
    else
        off = 0x8000;

    if (p->buf_size < off + 2)
        return 0;

    mode = AV_RB16(p->buf + off);
    if (mode == 0x3038 || mode == 0x3135 || mode == 0x4E4F || mode == 0x5253 || mode == 0x5352 || (mode == 0 && AV_RB16(p->buf + off + 2) == 0x06EA))
        return AVPROBE_SCORE_EXTENSION + 1;
    return 0;
}

static int scdstm_read_header(AVFormatContext *s)
{
    uint16_t mode;
    uint32_t vf_size;
    AVStream *ast, *vst;
    AVIOContext *pb = s->pb;
    SCDSTMDemuxContext *scd = s->priv_data;

    mode = avio_rb16(pb);
    if (mode == 0x3038 || mode == 0x3135 || mode == 0x4E4F || mode == 0x5253 || mode == 0x5352 || (mode == 0 && avio_rb16(pb + 2) == 0x06EA)) {
        scd->audio = 0;
        vf_size = 0x2800;
    } else {
        scd->audio = 2;
        vf_size = 0x1D800;
    }

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_SONCDVID;
    vst->codecpar->block_align = vf_size;
    vst->codecpar->framerate = (AVRational){ 2, 15 };

    if (scd->audio) {
        ast = avformat_new_stream(s, NULL);
        if (!ast)
            return AVERROR(ENOMEM);
        ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        ast->codecpar->codec_id = AV_CODEC_ID_PCM_SB8;

        ast->codecpar->block_align = 0x8000;
        ast->codecpar->sample_rate = 16384;
        ast->codecpar->ch_layout.nb_channels = 2;
        ast->codecpar->bit_rate = 8 * ast->codecpar->ch_layout.nb_channels * ast->codecpar->sample_rate;

        vst->duration = avio_size(pb) / (ast->codecpar->block_align + vst->codecpar->block_align) * 8;
        avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
    } else
        vst->duration = avio_size(pb) / vst->codecpar->block_align * 8;

    avpriv_set_pts_info(vst, 64, 1, vst->codecpar->framerate.den);

    avio_seek(pb, 0, SEEK_SET);
    return 0;
}

static int scdstm_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int stream_idx;
    AVIOContext *pb = s->pb;
    SCDSTMDemuxContext *scd = s->priv_data;

    if ((avio_tell(pb) + 0x1000) > avio_size(pb))
        return AVERROR_EOF;

    if (scd->audio == 2) {
        stream_idx = 1;
        scd->audio = 1;
    } else {
        stream_idx = 0;
        if (scd->audio)
            scd->audio = 2;
    }

    av_get_packet(pb, pkt, s->streams[stream_idx]->codecpar->block_align);
    pkt->stream_index = stream_idx;

    return 0;
}

const FFInputFormat ff_scdstm_demuxer = {
    .p.name         = "scdstm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sonic CD STreaM"),
    .p.extensions   = "stm",
    .priv_data_size = sizeof(SCDSTMDemuxContext),
    .read_probe     = scdstm_probe,
    .read_header    = scdstm_read_header,
    .read_packet    = scdstm_read_packet,
};
