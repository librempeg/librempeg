/*
 * AUDV demuxer
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

typedef struct AUDVContext {
    int64_t video_offset;
} AUDVContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 20)
        return 0;

    if (AV_RB32(p->buf+8) != MKBETAG('A','U','D','V'))
        return 0;
    if (AV_RB32(p->buf+16) != MKBETAG('K','V','A','G'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AUDVContext *audv = s->priv_data;
    int64_t video_offset = 16;
    AVIOContext *pb = s->pb;
    int rate, channels;
    AVStream *ast, *vst;

    avio_skip(pb, 12);
    video_offset += avio_rl32(pb);
    audv->video_offset = video_offset;
    avio_skip(pb, 8);
    rate = avio_rl32(pb);
    channels = avio_rl16(pb) + 1;
    if (rate <= 0 || channels == 0)
        return AVERROR_INVALIDDATA;

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->start_time = 0;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_SSI;
    ast->codecpar->ch_layout.nb_channels = channels;
    ast->codecpar->sample_rate = rate;
    ast->codecpar->block_align = channels;

    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

    avio_seek(pb, 0x1e, SEEK_SET);

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    vst->start_time = 0;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_CINEPAK;

    avpriv_set_pts_info(vst, 64, 1, 15);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AUDVContext *audv = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret, pkt_size;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (pos >= audv->video_offset) {
        avio_skip(pb, 4);
        pkt_size = avio_rl32(pb);
        if (pkt_size <= 0)
            return AVERROR_INVALIDDATA;
    } else {
        pkt_size = FFMIN(1024, audv->video_offset - pos);
    }

    ret = av_get_packet(pb, pkt, pkt_size);
    if (ret != pkt_size)
        return AVERROR_INVALIDDATA;
    pkt->stream_index = pos >= audv->video_offset;
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_audv_demuxer = {
    .p.name         = "audv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Mattel AUDV"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(AUDVContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
