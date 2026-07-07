/*
 * Gameloft VXN demuxer
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

typedef struct VXNStream {
    int64_t start_offset;
    int64_t stop_offset;
} VXNStream;

typedef struct VXNDemuxContext {
    int current_stream;
} VXNDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('V','o','x','N'))
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const VXNStream *vs1 = s1->priv_data;
    const VXNStream *vs2 = s2->priv_data;

    return FFDIFFSIGN(vs1->start_offset, vs2->start_offset);
}

static int find_chunk(AVIOContext *pb, uint32_t chunk_id)
{
    while (!avio_feof(pb)) {
        uint32_t chunk_type = avio_rb32(pb);
        uint32_t chunk_size = avio_rl32(pb);

        if (chunk_type == 0xFFFFFFFF || chunk_size == 0xFFFFFFFF)
            return AVERROR_INVALIDDATA;

        if (chunk_type == chunk_id)
            return 0;

        avio_skip(pb, chunk_size);
    }

    return AVERROR_INVALIDDATA;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, channels, align, codec, nb_streams, bits;
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    ret = find_chunk(pb, MKBETAG('A','f','m','t'));
    if (ret < 0)
        return ret;

    codec = avio_rl16(pb);
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    align = avio_rl16(pb);
    bits = avio_rl16(pb);
    switch (codec) {
    case 1:
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    case 2:
        codec = AV_CODEC_ID_ADPCM_MS;
        break;
    case 17:
        codec = AV_CODEC_ID_ADPCM_IMA_WAV;
        bits = 4;
        break;
    default:
        avpriv_request_sample(s, "codec %04x", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || align <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    ret = find_chunk(pb, MKBETAG('S','e','g','m'));
    if (ret < 0)
        return ret;

    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        VXNStream *vst;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        vst = av_mallocz(sizeof(*vst));
        if (!vst)
            return AVERROR(ENOMEM);
        st->priv_data = vst;

        vst->start_offset = avio_rl32(pb);
        vst->stop_offset = vst->start_offset;
        vst->stop_offset += avio_rl32(pb);

        st->start_time = 0;
        st->duration = avio_rl32(pb);
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align;
        st->codecpar->bits_per_coded_sample = bits;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        avio_skip(pb, 12);
    }

    ret = find_chunk(pb, MKBETAG('D','a','t','a'));
    if (ret < 0)
        return ret;
    start = avio_tell(pb);

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        VXNStream *vst = st->priv_data;

        st->index = n;
        vst->start_offset += start;
        vst->stop_offset += start;
    }

    {
        AVStream *st = s->streams[0];
        VXNStream *vst = st->priv_data;

        avio_seek(pb, vst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    VXNDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    VXNStream *vst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    vst = st->priv_data;
    if (do_seek)
        avio_seek(pb, vst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= vst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, vst->stop_offset - pos);

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
    VXNDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    VXNStream *vst;
    AVStream *st;
    int64_t pos;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    vst = st->priv_data;
    pos = avio_tell(pb);
    if (pos < vst->start_offset) {
        avio_seek(pb, vst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_vxn_demuxer = {
    .p.name         = "vxn",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Gameloft VXN"),
    .p.extensions   = "vxn",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(VXNDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
