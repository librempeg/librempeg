/*
 * SDD demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct SDDStream {
    int64_t start_offset;
    int64_t stop_offset;
} SDDStream;

typedef struct SDDDemuxContext {
    int current_stream;
} SDDDemuxContext;

static int read_probe(const AVProbeData *p)
{
    int64_t offset;

    if (AV_RB32(p->buf) != MKBETAG('D','S','B','H'))
        return 0;

    offset = AV_RL32(p->buf + 4);
    if (p->buf_size < offset + 4)
        return 0;
    if (AV_RB32(p->buf + offset) != MKBETAG('D','S','B','D'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SDDStream *ss1 = s1->priv_data;
    const SDDStream *ss2 = s2->priv_data;

    return FFDIFFSIGN(ss1->start_offset, ss2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, channels, align, codec, nb_streams, header_size, name_size;
    int64_t name_offset, entry_offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    header_size = avio_rl32(pb);
    nb_streams = (header_size - 0x20) / 0x20;
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        uint8_t name[17] = { 0 };
        SDDStream *sst;

        name_offset = 0x20 + n * 0x20;
        avio_seek(pb, name_offset, SEEK_SET);
        name_size = avio_get_str(pb, 16, name, sizeof(name));

        entry_offset = name_offset + name_size;
        avio_seek(pb, entry_offset, SEEK_SET);
        codec = avio_r8(pb);
        avio_skip(pb, 1);
        channels = avio_r8(pb);
        rate = avio_rl32(pb);

        switch (codec) {
        case 1:
            codec = AV_CODEC_ID_ADPCM_NDSP;
            align = 8;
            break;
        case 2:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 2;
            break;
        case 3:
            codec = AV_CODEC_ID_ADPCM_PSX;
            align = 16;
            break;
        default:
            avpriv_request_sample(s, "codec %X", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (rate <= 0 || channels <= 0 || align <= 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        sst->start_offset = avio_rl32(pb);
        sst->start_offset += header_size + 0x20;
        sst->stop_offset = sst->start_offset;
        sst->stop_offset += avio_rl32(pb);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align * channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        if (name[0] != '\0')
            av_dict_set(&st->metadata, "title", name, 0);

        if (codec == AV_CODEC_ID_ADPCM_NDSP) {
            avio_seek(pb, sst->start_offset + 28, SEEK_SET);

            ret = ff_get_extradata(s, st->codecpar, pb, 32 * channels);
            if (ret < 0)
                return ret;

            sst->start_offset += 0x60;
            st->duration = (sst->stop_offset - sst->start_offset) / (8 * channels) * 14;
        } else if (codec == AV_CODEC_ID_ADPCM_PSX) {
            st->duration = (sst->stop_offset - sst->start_offset) / (16 * channels) * 28;
        } else if (codec == AV_CODEC_ID_PCM_S16LE) {
            sst->start_offset += 12;
            st->duration = (sst->stop_offset - sst->start_offset) / (2 * channels);
        }
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    {
        AVStream *st = s->streams[0];
        SDDStream *sst = st->priv_data;

        avio_seek(pb, sst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SDDDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SDDStream *sst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    sst = st->priv_data;
    if (do_seek)
        avio_seek(pb, sst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= sst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, sst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
        pkt->pos = pos;
    }

    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        ctx->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    SDDDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    SDDStream *sst;
    AVStream *st;
    int64_t pos;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    sst = st->priv_data;
    pos = avio_tell(pb);
    if (pos < sst->start_offset) {
        avio_seek(pb, sst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_sdd_demuxer = {
    .p.name         = "sdd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Doki Denki DSBH"),
    .p.extensions   = "sdd",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SDDDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
