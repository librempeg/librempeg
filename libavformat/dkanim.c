/*
 * DK Animation demuxer
 *
 * Copyright (C) 2025 Paul B Mahol
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
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct DKAnimDemuxContext {
    AVPacket *video_pkt;

    int64_t video_start_pos;
    int image_factor;
} DKAnimDemuxContext;

static int dkanim_probe(const AVProbeData *p)
{
    int score = 0;

    if (AV_RL16(p->buf) != 1)
        return 0;

    if (AV_RL16(p->buf+2) == 0)
        return 0;

    for (int i = 4; i+6 < p->buf_size; i++) {
        if (!memcmp(p->buf+i, "funky!", 6)) {
            score += 2;
            if (score >= AVPROBE_SCORE_MAX)
                break;
        }
    }

    return score;
}

static int dkanim_read_header(AVFormatContext *s)
{
    int nb_chunks, audio_size, sample_rate, codec, nb_channels, bps, ba, fps;
    DKAnimDemuxContext *d = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *ast, *vst;

    avio_skip(pb, 2);
    nb_chunks = avio_rl16(pb);
    avio_skip(pb, 2);

    fps = avio_rl16(pb);
    d->image_factor = avio_rl16(pb);
    audio_size = avio_rl16(pb);
    avio_skip(pb, 2);
    codec = avio_rl16(pb);
    nb_channels = avio_rl16(pb);
    sample_rate = avio_rl32(pb);
    avio_skip(pb, 4);
    ba = avio_rl16(pb);
    bps = avio_rl16(pb);

    if (nb_channels <= 0 || sample_rate <= 0 || bps <= 0 || ba <= 0)
        return AVERROR_INVALIDDATA;

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->start_time = 0;
    ast->codecpar->block_align = ba;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->bits_per_coded_sample = bps;
    switch (codec) {
    case 1:
        ast->codecpar->codec_id = ff_get_pcm_codec_id(ast->codecpar->bits_per_coded_sample,
                                                      0, 0, 0);
        break;
    case 2:
        ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_MS;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }
    ast->codecpar->ch_layout.nb_channels = nb_channels;
    ast->codecpar->sample_rate = sample_rate;

    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

    avio_seek(pb, 0xE + audio_size, SEEK_SET);

    d->video_pkt = av_packet_alloc();
    if (!d->video_pkt)
        return AVERROR(ENOMEM);

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    vst->start_time = 0;
    vst->nb_frames = nb_chunks;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_DKANIM;

    avpriv_set_pts_info(vst, 64, 1, fps);

    d->video_start_pos = -1;

    return 0;
}

static int dkanim_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DKAnimDemuxContext *d = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = 0, audio_size, video_size;
    uint8_t video_header[12];
    int64_t pos = avio_tell(pb);

    if (d->video_pkt->size > 0) {
        av_packet_move_ref(pkt, d->video_pkt);
        pkt->pos = d->video_pkt->pos;
        pkt->stream_index = 1;
        pkt->duration = 1;
        return 0;
    }

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (avio_rl16(pb) != 1)
        return AVERROR_INVALIDDATA;
    audio_size = avio_rl32(pb);
    avio_read(pb, video_header, 12);
    video_size = avio_rl32(pb);
    avio_skip(pb, 6);

    if (audio_size > 0) {
        ret = av_get_packet(pb, pkt, audio_size);
        if (ret < 0)
            return ret;
        pkt->stream_index = 0;
        pkt->pos = pos;
    }

    if (video_size > 0) {
        if (video_size >= INT_MAX - sizeof(video_header))
            return AVERROR_INVALIDDATA;

        ret = av_new_packet(d->video_pkt, video_size + sizeof(video_header));
        if (ret < 0)
            return ret;
        memcpy(d->video_pkt->data, video_header, sizeof(video_header));
        avio_read(pb, d->video_pkt->data + sizeof(video_header), video_size);
        d->video_pkt->pos = pos;
        if (d->video_start_pos < 0)
            d->video_start_pos = pos;
        if (pos == d->video_start_pos)
            d->video_pkt->flags |= AV_PKT_FLAG_KEY;
    } else {
        ret = av_new_packet(d->video_pkt, 1);
        if (ret < 0)
            return ret;
        memset(d->video_pkt->data, 0, 1);
        if (d->video_start_pos < 0)
            d->video_start_pos = pos;
        if (pos == d->video_start_pos)
            d->video_pkt->flags |= AV_PKT_FLAG_KEY;
    }

    if (audio_size == 0)
        return FFERROR_REDO;
    return ret;
}

static int dkanim_read_seek(AVFormatContext *s, int stream_index,
                            int64_t timestamp, int flags)
{
    DKAnimDemuxContext *d = s->priv_data;

    av_packet_unref(d->video_pkt);

    return -1;
}

static int dkanim_read_close(AVFormatContext *s)
{
    DKAnimDemuxContext *d = s->priv_data;

    av_packet_free(&d->video_pkt);

    return 0;
}

const FFInputFormat ff_dkanim_demuxer = {
    .p.name         = "dkanim",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DK Animation"),
    .p.extensions   = "ani",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(DKAnimDemuxContext),
    .read_probe     = dkanim_probe,
    .read_header    = dkanim_read_header,
    .read_packet    = dkanim_read_packet,
    .read_seek      = dkanim_read_seek,
    .read_close     = dkanim_read_close,
};
