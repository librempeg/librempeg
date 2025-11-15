/*
 * VPK demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct VPKDemuxContext {
    int64_t data_start;
    unsigned block_count;
    unsigned current_block;
    unsigned last_block_size;
} VPKDemuxContext;

static int vpk_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKBETAG('V','P','K',' '))
        return 0;
    if (p->buf_size < 32)
        return 0;
    if ((int)AV_RL32(p->buf + 12) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int vpk_read_header(AVFormatContext *s)
{
    VPKDemuxContext *vpk = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t offset;
    unsigned samples_per_block;
    AVStream *st;

    vpk->current_block = 0;
    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);
    st->duration           = avio_rl32(pb) * 28 / 16;
    st->start_time = 0;
    offset = avio_rl32(pb);
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->block_align = avio_rl32(pb);
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0 || st->codecpar->block_align <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = avio_rl32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    samples_per_block      = ((st->codecpar->block_align / st->codecpar->ch_layout.nb_channels) * 28LL) / 16;
    if (samples_per_block <= 0)
        return AVERROR_INVALIDDATA;
    vpk->block_count       = (st->duration + (samples_per_block - 1)) / samples_per_block;
    vpk->last_block_size   = (st->duration % samples_per_block) * 16 * st->codecpar->ch_layout.nb_channels / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    vpk->data_start = offset;
    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int vpk_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    VPKDemuxContext *vpk = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret, i;

    vpk->current_block++;
    if (vpk->current_block == vpk->block_count) {
        unsigned size = vpk->last_block_size / par->ch_layout.nb_channels;
        unsigned skip = (par->block_align - vpk->last_block_size) / par->ch_layout.nb_channels;
        uint64_t pos = avio_tell(pb);

        ret = av_new_packet(pkt, vpk->last_block_size);
        if (ret < 0)
            return ret;
        for (i = 0; i < par->ch_layout.nb_channels; i++) {
            ret = ffio_read_size(pb, pkt->data + i * size, size);
            avio_skip(pb, skip);
            if (ret < 0)
                return ret;
        }
        pkt->pos = pos;
        pkt->stream_index = 0;
    } else if (vpk->current_block < vpk->block_count) {
        ret = av_get_packet(pb, pkt, par->block_align);
        pkt->stream_index = 0;
    } else {
        return AVERROR_EOF;
    }

    return ret;
}

static int vpk_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    AVStream *st = s->streams[stream_index];
    AVCodecParameters *par = st->codecpar;
    VPKDemuxContext *vpk = s->priv_data;
    int samples_per_block;
    int64_t ret = 0;

    samples_per_block = av_get_audio_frame_duration2(par, par->block_align);
    if (samples_per_block > 0)
        timestamp /= samples_per_block;
    else
        return -1;
    ret = avio_seek(s->pb, vpk->data_start + timestamp * par->block_align, SEEK_SET);
    if (ret < 0)
        return ret;

    vpk->current_block = timestamp;
    avpriv_update_cur_dts(s, st, timestamp * samples_per_block);
    return 0;
}

const FFInputFormat ff_vpk_demuxer = {
    .p.name         = "vpk",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS2 VPK"),
    .p.extensions   = "vpk",
    .priv_data_size = sizeof(VPKDemuxContext),
    .read_probe     = vpk_probe,
    .read_header    = vpk_read_header,
    .read_packet    = vpk_read_packet,
    .read_seek      = vpk_read_seek,
};
