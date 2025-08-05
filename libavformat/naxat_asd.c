/*
 * Naxat ASD demuxer
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

static int read_header(AVStream *st, AVIOContext *pb)
{
    uint16_t format;

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

    format = avio_rl16(pb);
    if (format != 1){
        avpriv_request_sample(st, "format 0x%X", format);
        return AVERROR_PATCHWELCOME;
    }

    st->codecpar->ch_layout.nb_channels = avio_rl16(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    if (format == 1) {
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int naxat_asd_probe(const AVProbeData *p)
{
    uint32_t data_size;
    int score = 40;

    if (0x20 >= p->buf_size)
        return 0;
    data_size = AV_RL32(p->buf);
    // data size has to be smaller than whole buffer
    if (data_size == 0 || data_size + 0x20 <= p->buf_size)
        return 0;
    // 2nd data size
    if (data_size != AV_RL32(p->buf + 4))
        return 0;
    // format
    if (AV_RL16(p->buf + 8) != 1)
        score -= 40;
    // channels
    if (AV_RL16(p->buf + 10) <= 2)
        score += 10;
    // sample rate;
    if (AV_RL32(p->buf + 12) == 22050)
        score += 10;
    // block size
    if (AV_RL32(p->buf + 24) == 0)
        score += 10;
    // bps
    if (AV_RL32(p->buf + 28) == 0)
        score += 10;

    return score;
}

static int naxat_asd_read_header(AVFormatContext *s)
{
    int ret;
    AVStream *st;
    AVIOContext *pb = s->pb;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 8);
    ret = read_header(st, pb);

    avio_seek(pb, 0x20, SEEK_SET);
    return ret;
}

static int naxat_asd_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t block_size;
    AVIOContext *pb = s->pb;

    pkt->pos = avio_tell(pb);
    block_size = avio_size(pb) - pkt->pos;
    if (block_size > s->streams[0]->codecpar->block_align)
        block_size = s->streams[0]->codecpar->block_align;
    return av_get_packet(pb, pkt, block_size);
}

const FFInputFormat ff_naxat_asd_demuxer = {
    .p.name         = "naxat_asd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Naxat ASD"),
    .p.extensions   = "asd",
    .read_probe     = naxat_asd_probe,
    .read_header    = naxat_asd_read_header,
    .read_packet    = naxat_asd_read_packet,
};

typedef struct NaxatASDStream {
    uint32_t start_offset;
    uint32_t end_offset;
} NaxatASDStream;

static int naxat_asd_bnk_probe(const AVProbeData *p)
{
    int score = AVPROBE_SCORE_MAX / 2;

    // identifier?
    if (AV_RL16(p->buf) != 0x1001)
        return 0;
    // channels
    if (AV_RL16(p->buf + 2) <= 2)
        score += 10;
    // sample rate
    if (AV_RL32(p->buf + 4) == 22050)
        score += 10;
    // block size
    if (AV_RL32(p->buf + 8) == 0)
        score += 10;
    // bps
    if (AV_RL32(p->buf + 0xc) == 0)
        score += 10;
    // first offset has to be bigger than smallest header
    if (AV_RL32(p->buf + 0xf) < 0x1A0)
        return 0;

    return score;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const NaxatASDStream *fs1 = s1->priv_data;
    const NaxatASDStream *fs2 = s2->priv_data;

    return FFDIFFSIGN(fs1->start_offset, fs2->start_offset);
}

static int naxat_asd_bnk_read_header(AVFormatContext *s)
{
    int ret;
    uint32_t size, header_end, start;
    int64_t addr;
    NaxatASDStream *first_ast;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 0x10);

    header_end = 0xFFFFFFFF;
    for (int i = 0; i < 0xFFFF; i++) {
        NaxatASDStream *ast;
        AVStream *st;

        if (avio_feof(pb))
            return AVERROR_EOF;
        addr = avio_tell(pb);
        if (header_end <= addr)
            break;
        start = avio_rl32(pb);
        if (start == 0xFFFFFFFF)
            continue;
        addr += 4;
        if (header_end > start)
            header_end = start;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        st->priv_data = ast;
        ast->start_offset = start + 0x20;

        avio_seek(pb, start, SEEK_SET);
        size = avio_rl32(pb);
        if (size == 0 || size != avio_rl32(pb))
            return AVERROR_INVALIDDATA;
        ast->end_offset = ast->start_offset + size;
        ret = read_header(st, pb);
        if (ret < 0)
            return ret;
        avio_seek(pb, addr, SEEK_SET);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        if (n == 0)
            first_ast = st->priv_data;

        st->index = n;
    }

    avio_seek(pb, first_ast->start_offset, SEEK_SET);
    return 0;
}

static int naxat_asd_bnk_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret;
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;

    ret = AVERROR_EOF;
    for (int i = 0; i < s->nb_streams; i++)
    {
        AVStream *st = s->streams[i];
        NaxatASDStream *ast = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= ast->start_offset && pos < ast->end_offset) {
            block_size = FFMIN(ast->end_offset - pos, st->codecpar->block_align);

            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;
            break;
        } else if (pos >= ast->end_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            NaxatASDStream *ast_next = st_next->priv_data;
            if (ast_next->start_offset > pos)
                avio_seek(pb, ast_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_naxat_asd_bank_demuxer = {
    .p.name         = "naxat_asd_bank",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Naxat ASD Bank"),
    .p.extensions   = "asd",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = naxat_asd_bnk_probe,
    .read_header    = naxat_asd_bnk_read_header,
    .read_packet    = naxat_asd_bnk_read_packet,
};
