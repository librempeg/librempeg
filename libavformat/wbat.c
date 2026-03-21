/*
 * Firebrand WBAT demuxer
 * Copyright (C) 2026 Paul B Mahol
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

#include <stddef.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/internal.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "internal.h"
#include "avformat.h"
#include "demux.h"
#include "pcm.h"

typedef struct WBATStream {
    int64_t start_offset;
    int64_t stop_offset;
} WBATStream;

typedef struct WBATDemuxContext {
    int current_stream;
} WBATDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKTAG('W','B','A','T'))
        return 0;

    if (p->buf_size < 8)
        return 0;
    if (AV_RB16(p->buf + 4) == 0xFEFF) {
        if (AV_RB16(p->buf + 6) < 6 ||
            AV_RB16(p->buf + 6) > 7)
            return 0;
    } else {
        if (AV_RL16(p->buf + 6) < 6 ||
            AV_RL16(p->buf + 6) > 7)
            return 0;
    }

    return AVPROBE_SCORE_MAX;
}

typedef unsigned int (*avio_r32)(AVIOContext *s);
typedef unsigned int (*avio_r16)(AVIOContext *s);

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const WBATStream *ws1 = s1->priv_data;
    const WBATStream *ws2 = s2->priv_data;

    return FFDIFFSIGN(ws1->start_offset, ws2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int bom, nb_streams, version;
    int64_t names_size, offset;
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32;
    avio_r16 avio_r16;

    avio_skip(pb, 4);
    bom = avio_rb16(pb);
    if (bom == 0xFEFF) {
        avio_r32 = avio_rb32;
        avio_r16 = avio_rb16;
    } else {
        avio_r32 = avio_rl32;
        avio_r16 = avio_rl16;
    }
    version = avio_r16(pb);
    if (version != 6 && version != 7)
        return AVERROR_INVALIDDATA;

    nb_streams = avio_r32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;
    names_size = avio_r32(pb);
    offset = 0x1C + names_size;
    offset += avio_r32(pb) * 4LL;
    offset += avio_r32(pb) * 4LL;

    for (int n = 0; n < nb_streams; n++) {
        int64_t name_offset, stream_offset, stream_size, duration, config_size = 0;
        int nb_channels, codec, rate, ret, align;
        char title[129] = { 0 };
        WBATStream *wst;
        AVStream *st;

        avio_seek(pb, offset + n * 0x24LL, SEEK_SET);
        name_offset = avio_r32(pb) + 0x1C;
        codec = avio_r32(pb);
        rate = avio_r32(pb);
        nb_channels = avio_r32(pb);
        avio_skip(pb, 8);
        stream_offset = avio_r32(pb) + offset + 0x24LL * nb_streams;
        stream_size = avio_r32(pb);
        duration = avio_r32(pb);

        switch (codec) {
        case 0:
            codec = (bom == 0xFEFF) ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
            align = 2 * nb_channels;
            break;
        case 1:
            codec = AV_CODEC_ID_PCM_S8;
            align = 1 * nb_channels;
            break;
        case 2:
            config_size = (32LL+20LL) * nb_channels + 12LL * nb_channels;
            codec = (bom == 0xFEFF) ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
            stream_offset += config_size;
            stream_size -= config_size;
            align = stream_size;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unknown codec: %d\n", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (rate <= 0 || nb_channels <= 0 || align <= 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        wst = av_mallocz(sizeof(*wst));
        if (!wst)
            return AVERROR(ENOMEM);
        st->priv_data = wst;

        wst->start_offset = stream_offset;
        wst->stop_offset = wst->start_offset + stream_size;

        st->start_time = 0;
        st->duration = duration / nb_channels;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = nb_channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align;

        if (config_size > 0) {
            ret = ff_alloc_extradata(st->codecpar, nb_channels * 32);
            if (ret < 0)
                return ret;

            avio_seek(pb, stream_offset - config_size, SEEK_SET);
            for (int ch = 0; ch < nb_channels; ch++) {
                avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
                avio_skip(pb, 20);
            }
        }

        avio_seek(pb, name_offset, SEEK_SET);
        if ((ret = avio_get_str(pb, INT_MAX, title, sizeof(title))) < 0)
            return ret;

        if (title[0] != '\0')
            av_dict_set(&st->metadata, "title", title, 0);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;

        if (n == 0) {
            WBATStream *wst = st->priv_data;

            avio_seek(pb, wst->start_offset, SEEK_SET);
        }
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    WBATDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    WBATStream *wst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    wst = st->priv_data;
    if (do_seek)
        avio_seek(pb, wst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= wst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, wst->stop_offset - pos);

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
    WBATDemuxContext *ctx = s->priv_data;
    WBATStream *wst;
    AVStream *st;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    wst = st->priv_data;
    {
        AVIOContext *pb = s->pb;
        int64_t pos = avio_tell(pb);

        if (pos < wst->start_offset) {
            avio_seek(pb, wst->start_offset, SEEK_SET);
            return 0;
        }

        return -1;
    }
}

const FFInputFormat ff_wbat_demuxer = {
    .p.name         = "wbat",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Firebrand Wave Batch"),
    .p.extensions   = "wavebatch",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(WBATDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
