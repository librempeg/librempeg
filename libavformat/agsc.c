/*
 * AGSC demuxer
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

typedef struct AGSCStream {
    uint32_t start_offset;
    uint32_t end_offset;
} AGSCStream;

static int agsc_probe(const AVProbeData *p)
{
    int64_t off;

    // version 2
    if (AV_RB32(p->buf) == 1)
        off = 4;
    // version 1
    else if (!memcmp(p->buf, "Audio/\0", 7))
        off = 8;
    else if (!memcmp(p->buf, "Audio//\0", 8))
        off = 9;
    else
        return 0;

    int i;
    for (i = 0; i < 0x30; ++i) {
        if (p->buf[off + i] == '\0' && i != 0)
            break;
        else if (!(p->buf[off + i] >= 0x20 || p->buf[off + i] <= 0x7E))
            return 0;
    }
    if (i == (0x30 - 1))
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const AGSCStream *fs1 = s1->priv_data;
    const AGSCStream *fs2 = s2->priv_data;

    return FFDIFFSIGN(fs1->start_offset, fs2->start_offset);
}

static int agsc_read_header(AVFormatContext *s)
{
    uint8_t version;
    uint32_t unk1_size, unk2_size, data_size, head_size, head_offset, data_offset, loop_start, loop_end, coefs_offset, min_coefs_offset;
    int64_t addr, nb_streams;
    int ret;
    AVIOContext *pb = s->pb;

    if (avio_rb32(pb) == 1)
        version = 2;
    else {
        char testchr[7];

        avio_seek(pb, 0, SEEK_SET);
        avio_get_str(pb, 8, testchr, sizeof(testchr));
        if (!strncmp(testchr,"Audio/",7))
            version = 1;
        else
            return AVERROR_INVALIDDATA;
    }

    char bank_name[0x30-1];
    ret = avio_get_str(pb, 0x30, bank_name, sizeof(bank_name));
    if (ret < 0)
        return ret;
    av_dict_set(&s->metadata, "title", bank_name, 0);

    if (version == 1) {
        unk1_size = avio_rb32(pb);
        avio_skip(pb, unk1_size);
        unk2_size = avio_rb32(pb);
        avio_skip(pb, unk2_size);

        data_size = avio_rb32(pb);
        data_offset = avio_tell(pb);
        avio_skip(pb, data_size);

        head_size = avio_rb32(pb);
        head_offset = avio_tell(pb);
    } else if (version == 2) {
        avio_skip(pb, 2);
        unk1_size = avio_rb32(pb);
        unk2_size = avio_rb32(pb);
        head_size = avio_rb32(pb);
        data_size = avio_rb32(pb);

        head_offset = avio_tell(pb) + unk1_size + unk2_size;
        data_offset = head_offset + head_size;
    } else {
        data_size = 0;
    }

    if (data_size == 0 || head_size < 0x48) {
        av_log(s, AV_LOG_WARNING, "AGSC Bank is empty");
        return 0;
    }

    nb_streams = (head_size - 4) / 0x48;
    avio_seek(pb, head_offset, SEEK_SET);
    min_coefs_offset = 0xFFFFFFFF;
    for (int i = 0; i < nb_streams; i++) {
        avio_skip(pb, 4);
        if (avio_tell(pb) >= min_coefs_offset)
            break;

        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        AGSCStream *ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->priv_data = ast;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
        st->codecpar->ch_layout.nb_channels = 1;

        ast->start_offset = avio_rb32(pb) + data_offset;
        avio_skip(pb, 4);
        if (avio_rb16(pb) != 0x3c00)
            return AVERROR_INVALIDDATA;
        st->codecpar->sample_rate = avio_rb16(pb);
        st->duration = avio_rb32(pb);
        st->codecpar->block_align = 512;
        if (i == (nb_streams - 1))
            ast->end_offset = data_offset + data_size;
        if (i > 0) {
            AVStream *prev_st = s->streams[i-1];
            AGSCStream *prev_ast = prev_st->priv_data;
            prev_ast->end_offset = ast->start_offset;
        }

        loop_start = avio_rb32(pb);
        loop_end = avio_rb32(pb);
        if (loop_end != 0) {
            av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
            av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
        }
        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        coefs_offset = avio_rb32(pb) + head_offset;
        if (coefs_offset < min_coefs_offset)
            min_coefs_offset = coefs_offset;
        addr = avio_tell(pb);
        avio_seek(pb, coefs_offset + 8, SEEK_SET);
        ret = ff_alloc_extradata(st->codecpar, 0x20);
        if (ret < 0)
            return ret;
        avio_read(pb, st->codecpar->extradata, 0x20);
        avio_seek(pb, addr, SEEK_SET);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    avio_seek(pb, data_offset, SEEK_SET);
    return 0;
}

static int agsc_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AGSCStream *ast = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= ast->start_offset && pos < ast->end_offset) {
            block_size = ast->end_offset - pos;
            if (block_size < st->codecpar->block_align)
                block_size = st->codecpar->block_align;
            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;
            break;
        } else if (pos >= ast->end_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            AGSCStream *ast_next = st_next->priv_data;
            if (ast_next->start_offset > pos)
                avio_seek(pb, ast_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_agsc_demuxer = {
    .p.name         = "agsc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AGSC Bank (Metroid Prime 1/2 GC)"),
    .p.extensions   = "agsc",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = agsc_probe,
    .read_header    = agsc_read_header,
    .read_packet    = agsc_read_packet,
};
