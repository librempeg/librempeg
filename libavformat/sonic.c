/*
 * SONIC demuxer
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
#include "libavutil/parseutils.h"
#include "libavutil/opt.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define VIDEO_SIZE 0x1d800
#define AUDIO_SIZE 0x8000

typedef struct SonicDemuxContext {
    uint8_t video_data[VIDEO_SIZE];
    int64_t pos;
    int have_video;
} SonicDemuxContext;

static int sonic_read_probe(const AVProbeData *p)
{
    int score = AVPROBE_SCORE_EXTENSION;
    const uint8_t *buf = p->buf;

    if (p->buf_size < 0x8000 + 16)
        return 0;

    return score * 0;
}

static int sonic_read_header(AVFormatContext *s)
{
    SonicDemuxContext *sonic = s->priv_data;
    AVStream *ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->start_time = 0;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->codec_tag = 0;
    ast->codecpar->codec_id = AV_CODEC_ID_PCM_SONIC;
    ast->codecpar->sample_rate = 16384;
    av_channel_layout_default(&ast->codecpar->ch_layout, 2);

    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

    AVStream *vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    vst->start_time = 0;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_tag = 0;
    vst->codecpar->format = AV_PIX_FMT_PAL8;
    vst->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
    vst->codecpar->width = 32 * 8;
    vst->codecpar->height = 14 * 8;

    avpriv_set_pts_info(vst, 64, 2, 15);

    sonic->have_video = 0;

    return 0;
}

static int sonic_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SonicDemuxContext *sonic = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (sonic->have_video) {
        sonic->pos = avio_tell(pb);

        if ((ret = av_new_packet(pkt, sizeof(sonic->video_data))) < 0)
            return ret;
        memcpy(pkt->data, sonic->video_data, sizeof(sonic->video_data));
        pkt->pos = sonic->pos;
        pkt->duration = 1;
        pkt->stream_index = 1;

        sonic->have_video = 0;
    } else {
        ret = av_get_packet(pb, pkt, AUDIO_SIZE);
        if (ret < 0)
            return ret;
        pkt->pos = sonic->pos;
        pkt->stream_index = 0;
        ret = avio_read(pb, sonic->video_data, sizeof(sonic->video_data));

        sonic->have_video = ret == sizeof(sonic->video_data);
    }

    return ret;
}

const FFInputFormat ff_sonic_demuxer = {
    .p.name         = "sonic",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sonic STM video"),
    .p.extensions   = "stm",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SonicDemuxContext),
    .read_probe     = sonic_read_probe,
    .read_header    = sonic_read_header,
    .read_packet    = sonic_read_packet,
};
