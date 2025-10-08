/*
 * CFDF demuxer
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

typedef struct CFDFStream {
    int64_t start_offset;
    int64_t stop_offset;
} CFDFStream;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 1024)
        return 0;
    if (AV_RB32(p->buf+0x20) != MKBETAG('L','P','P','A') ||
        AV_RB32(p->buf+0x24) != MKBETAG('L','P','P','A'))
        return 0;
    if ((int)AV_RL32(p->buf+0x14) <= 0 ||
        AV_RL32(p->buf+0x14) > INT16_MAX)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const CFDFStream *xs1 = s1->priv_data;
    const CFDFStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

typedef struct CFDFChunk {
    int id;
    int is_named;
    int is_segmented_chunk;
    int codec_flag;
    int64_t offset;
    int64_t container_pos;
    int32_t size;
    int32_t sample_rate;
    int32_t uncompressed_size;
    uint8_t name[15 + 1];
} CFDFChunk;

static int find_segmented_chunk_size(CFDFChunk *chunks, int total_chunks)
{
    int32_t most_common_size = -1;
    int max_count = 0;

    if (total_chunks < 1)
        return -1;

    for (int i = 0; i < total_chunks; i++) {
        if (chunks[i].is_named)
            continue;

        int current_count = 0;
        for (int j = 0; j < total_chunks; j++) {
            if (chunks[j].is_named)
                continue;

            if (chunks[i].uncompressed_size == chunks[j].uncompressed_size)
                current_count++;
        }

        if (current_count > max_count) {
            max_count = current_count;
            most_common_size = chunks[i].uncompressed_size;
        }
    }

    return most_common_size;
}

static int read_header(AVFormatContext *s)
{
    int segmented_chunks, containers, nb_streams;
    int segmented_chunk_size, total_chunks = 0;
    int64_t first_start_offset;
    AVIOContext *pb = s->pb;
    CFDFChunk *chunks;

    avio_skip(pb, 0x14);
    containers = avio_rl32(pb);
    if (containers <= 0 || containers > INT16_MAX)
        return AVERROR_INVALIDDATA;

    chunks = av_calloc(containers, sizeof(*chunks));
    if (!chunks)
        return AVERROR(ENOMEM);

    avio_seek(pb, 0x400, SEEK_SET);
    for (int ci = 0; ci < containers; ci++) {
        CFDFChunk *chunk = &chunks[ci];

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        chunk->container_pos = avio_rl32(pb);
    }

    for (int ci = 0; ci < containers; ci++) {
        CFDFChunk *rchunk = &chunks[ci];
        CFDFChunk *chunk = &chunks[total_chunks];
        int64_t header_pos = rchunk->container_pos + 8LL;
        int valid = 1;

        chunk->id = ci;
        avio_seek(pb, header_pos - 4, SEEK_SET);
        chunk->size = avio_rl32(pb);
        avio_skip(pb, 0x1a);
        chunk->codec_flag = avio_rl16(pb);
        if (chunk->codec_flag != 1 &&
            chunk->codec_flag != 2)
            valid = 0;
        chunk->sample_rate = avio_rl32(pb);
        if (chunk->sample_rate <= 0)
            valid = 0;
        avio_skip(pb, 4);
        chunk->uncompressed_size = avio_rl32(pb);
        avio_skip(pb, 4);
        chunk->offset = header_pos + avio_rl32(pb);
        if (valid)
            total_chunks++;
    }

    if (total_chunks <= 0)
        return AVERROR_INVALIDDATA;

    {
        avio_seek(pb, 0x400, SEEK_SET);
        int64_t container0_pos = avio_rl32(pb);
        int md_container_id = -1;

        if (container0_pos > 0) {
            avio_seek(pb, container0_pos + 0x28, SEEK_SET);
            md_container_id = avio_rl32(pb);
        }

        if (md_container_id >= 0 && md_container_id < containers) {
            avio_seek(pb, 0x400 + md_container_id * 4, SEEK_SET);
            int64_t md_pos = avio_rl32(pb);

            if (md_pos > 0) {
                const int64_t md_payload_pos = md_pos + 0x08;
                int64_t current_record_offset = md_payload_pos + 0x08;
                avio_seek(pb, md_payload_pos + 0x04, SEEK_SET);
                int records = avio_rl16(pb);

                for (int i = 0; i < records; i++) {
                    avio_seek(pb, current_record_offset + 0x04, SEEK_SET);
                    int chunk_id = avio_rl32(pb);
                    avio_skip(pb, 2);
                    uint8_t name_len = avio_r8(pb);
                    uint8_t name_buffer[15 + 1] = {0};

                    avio_seek(pb, current_record_offset + 11, SEEK_SET);
                    for (int k = 0; k < 15; k++)
                        name_buffer[k] = avio_r8(pb);

                    if (avio_feof(pb))
                        return AVERROR_INVALIDDATA;

                    if (name_len < 15)
                        name_buffer[name_len] = '\0';

                    for (int j = 0; j < total_chunks; j++) {
                        if (chunks[j].id == chunk_id) {
                            strncpy(chunks[j].name, name_buffer, sizeof(chunks[j].name) - 1);

                            chunks[j].name[sizeof(chunks[j].name) - 1] = '\0';
                            chunks[j].is_named = 1;

                            break;
                        }
                    }

                    current_record_offset += 0x1A;
                }
            }
        }
    }

    segmented_chunks = 0;
    segmented_chunk_size = find_segmented_chunk_size(chunks, total_chunks);

    if (segmented_chunk_size > 0) {
        av_unused int candidates = 0;

        for (int i = 0; i < total_chunks; i++) {
            if (!chunks[i].is_named && chunks[i].uncompressed_size == segmented_chunk_size)
                candidates++;
        }

        for (int i = 0; i < total_chunks; i++) {
            if (!chunks[i].is_named && chunks[i].uncompressed_size == segmented_chunk_size) {
                chunks[i].is_segmented_chunk = 1;
                segmented_chunks++;
            }
        }
    }

    nb_streams = (total_chunks - segmented_chunks) + (segmented_chunks > 0);
    if (segmented_chunks > 0) {
    }

    for (int si = 0; si < nb_streams; si++) {
        CFDFChunk *chunk = &chunks[si];
        CFDFStream *xst;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        xst = av_mallocz(sizeof(*xst));
        if (!xst)
            return AVERROR(ENOMEM);
        st->priv_data = xst;

        st->start_time = 0;
        st->duration = chunk->uncompressed_size / chunk->codec_flag;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = (chunk->codec_flag == 1) ? AV_CODEC_ID_ADPCM_CFDF : AV_CODEC_ID_CFDF_DPCM;
        st->codecpar->sample_rate = chunk->sample_rate;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->block_align = 256;

        xst->start_offset = chunk->offset;
        xst->stop_offset = chunk->offset;
        xst->stop_offset += chunk->size;
        if (chunk->name[0] != '\0')
            av_dict_set(&st->metadata, "title", chunk->name, 0);
    }

    av_freep(&chunks);
    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        CFDFStream *xst = st->priv_data;

        first_start_offset = xst->start_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        CFDFStream *xst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= xst->start_offset && pos < xst->stop_offset) {
            const int size = FFMIN(par->block_align, xst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= xst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            CFDFStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_cfdf_demuxer = {
    .p.name         = "cfdf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CFDF (Cyberflix DreamFactory)"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "trk",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
