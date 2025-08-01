/*
 * BCWAV demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct BCWAVStream {
    int64_t start_offset;
    int64_t stop_offset;
    int64_t chnf_offset;
    int64_t coef_offset;
} BCWAVStream;

typedef struct BCWAVDemuxContext {
    int little_endian;
} BCWAVDemuxContext;

static int probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) == MKTAG('C','W','A','V') &&
        (AV_RL16(p->buf + 4) == 0xFFFE ||
         AV_RL16(p->buf + 4) == 0xFEFF))
        return AVPROBE_SCORE_MAX / 3 * 2;
    return 0;
}

static int fwav_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) == MKTAG('F','W','A','V') &&
        (AV_RL16(p->buf + 4) == 0xFFFE ||
         AV_RL16(p->buf + 4) == 0xFEFF))
        return AVPROBE_SCORE_MAX / 3 * 2;
    return 0;
}

static av_always_inline unsigned int read16(AVFormatContext *s)
{
    BCWAVDemuxContext *b = s->priv_data;
    if (b->little_endian)
        return avio_rl16(s->pb);
    else
        return avio_rb16(s->pb);
}

static av_always_inline unsigned int read32(AVFormatContext *s)
{
    BCWAVDemuxContext *b = s->priv_data;

    if (b->little_endian)
        return avio_rl32(s->pb);
    else
        return avio_rb32(s->pb);
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const BCWAVStream *bs1 = s1->priv_data;
    const BCWAVStream *bs2 = s2->priv_data;

    return FFDIFFSIGN(bs1->start_offset, bs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    uint32_t info_offset = -1, data_offset = -1, duration;
    int bom, codec, chunk, sample_rate, nb_channels;
    BCWAVDemuxContext *b = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t chtb_offset;
    uint16_t nb_chunks;

    avio_skip(s->pb, 4);

    bom = avio_rb16(s->pb);
    if (bom != 0xFEFF && bom != 0xFFFE) {
        av_log(s, AV_LOG_ERROR, "invalid byte order: %X\n", bom);
        return AVERROR_INVALIDDATA;
    }

    if (bom == 0xFFFE)
        b->little_endian = 1;

    avio_skip(pb, 10);
    nb_chunks = read16(s);
    avio_skip(pb, 2);

    for (int n = 0; n < nb_chunks; n++) {
        uint16_t chunk_marker;
        uint32_t chunk_offset;

        chunk_marker = read16(s);
        avio_skip(pb, 2);
        chunk_offset = read32(s);
        avio_skip(pb, 4);

        switch (chunk_marker) {
        case 0x7000:
            info_offset = chunk_offset;
            break;
        case 0x7001:
            data_offset = chunk_offset;
            break;
        default:
            return AVERROR_INVALIDDATA;
        }
    }

    if (data_offset == -1 || info_offset == -1)
        return AVERROR_INVALIDDATA;

    if (info_offset > avio_tell(pb))
        avio_skip(pb, info_offset - avio_tell(pb));

    chunk = avio_rb32(pb);
    if (chunk != MKBETAG('I','N','F','O'))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    codec = avio_r8(pb);
    if (codec > 3)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 3);
    sample_rate = read32(s);
    if (sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    duration = read32(s);
    avio_skip(pb, 4);
    chtb_offset = avio_tell(pb);
    nb_channels = read32(s);
    if (nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    for (int ch = 0; ch < nb_channels; ch++) {
        uint16_t chunk_marker;
        BCWAVStream *bc;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        bc = av_calloc(1, sizeof(*bc));
        if (!bc)
            return AVERROR(ENOMEM);

        st->id = ch;
        st->priv_data = bc;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->start_time = 0;
        st->duration = duration;
        st->codecpar->sample_rate = sample_rate;
        st->codecpar->ch_layout.nb_channels = 1;

        switch (codec) {
        case 0:
            st->codecpar->codec_id = AV_CODEC_ID_PCM_S8;
            st->codecpar->block_align = 128;
            break;
        case 1:
            st->codecpar->codec_id = b->little_endian ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
            st->codecpar->block_align = 128 * 2;
            break;
        case 2:
            st->codecpar->codec_id = b->little_endian ? AV_CODEC_ID_ADPCM_NDSP_LE : AV_CODEC_ID_ADPCM_NDSP;
            st->codecpar->block_align = 8 * 8;
            break;
        }

        chunk_marker = read16(s);
        if (chunk_marker != 0x7100)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 2);

        bc->chnf_offset = chtb_offset + read32(s);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    for (int ch = 0; ch < nb_channels; ch++) {
        AVStream *st = s->streams[ch];
        BCWAVStream *bc = st->priv_data;

        avio_seek(pb, bc->chnf_offset, SEEK_SET);
        avio_skip(pb, 4);
        bc->start_offset = data_offset + 8 + read32(s);
        avio_skip(pb, 4);
        bc->coef_offset = bc->chnf_offset + read32(s);
        if (ch > 0) {
            AVStream *prev_st = s->streams[ch-1];
            BCWAVStream *prev_bc = prev_st->priv_data;

            prev_bc->stop_offset = bc->start_offset;
        }
    }

    for (int ch = 0; ch < nb_channels && codec == 2; ch++) {
        AVStream *st = s->streams[ch];
        BCWAVStream *bc = st->priv_data;
        int ret;

        avio_seek(pb, bc->coef_offset, SEEK_SET);
        ret = ff_get_extradata(s, st->codecpar, pb, 32);
        if (ret < 0)
            return ret;
    }

    if (data_offset > avio_tell(pb)) {
        AVStream *st = s->streams[nb_channels-1];
        BCWAVStream *bc = st->priv_data;

        avio_skip(pb, data_offset + 4LL - avio_tell(pb));
        bc->stop_offset = avio_tell(pb) + read32(s);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        BCWAVStream *bcwav = st->priv_data;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos < bcwav->start_offset) {
            avio_skip(pb, bcwav->start_offset - pos);
            pos = avio_tell(pb);
        }

        if (pos >= bcwav->start_offset && pos < bcwav->stop_offset) {
            int packet_size = FFMIN(par->block_align, bcwav->stop_offset - pos);

            ret = av_get_packet(pb, pkt, packet_size);
            pkt->stream_index = st->id;
            break;
        } else if (pos >= bcwav->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            BCWAVStream *bcwav_next = st_next->priv_data;

            if (bcwav_next->start_offset > pos)
                avio_skip(pb, bcwav_next->start_offset - pos);
        }
    }
    return ret;
}

const FFInputFormat ff_bcwav_demuxer = {
    .p.name         = "bcwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BCWAV (NintendoWare CWAV)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bcwav,adpcm,bms,sfx,str,zic",
    .priv_data_size = sizeof(BCWAVDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};

const FFInputFormat ff_bfwav_demuxer = {
    .p.name         = "bfwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BFWAV (NintendoWare FWAV)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bfwav,fwav",
    .priv_data_size = sizeof(BCWAVDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = fwav_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
