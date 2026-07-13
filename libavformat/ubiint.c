/*
 * INT video demuxer
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
#include "avio_internal.h"

typedef struct INTBlock {
     int size;
} INTBlock;

typedef struct INTChunk {
     int64_t offset;
} INTChunk;

typedef struct UBIINTContext {
    INTBlock *ablock[5];
    INTBlock *vblock;
    INTChunk *chunk;
} UBIINTContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x02000)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    UBIINTContext *ubiint = s->priv_data;
    int width, height, layers, nb_chunks, rate, frames_per_chunk;
    int64_t offset, nb_frames, nb_aframes, start_offset;
    AVIOContext *pb = s->pb;
    uint32_t fps, size[4];
    AVStream *vst, *ast;
    int ret;

    avio_skip(pb, 4);
    fps = avio_rl32(pb);
    frames_per_chunk = avio_rl32(pb);
    if (frames_per_chunk <= 0)
        return AVERROR_INVALIDDATA;

    nb_chunks = avio_rl32(pb);
    layers = avio_rl32(pb);
    avio_skip(pb, 4);
    size[0] = avio_rl32(pb);
    size[1] = avio_rl32(pb);
    size[2] = avio_rl32(pb);
    size[3] = avio_rl32(pb);
    avio_skip(pb, 4);

    ubiint->chunk = av_calloc(nb_chunks, sizeof(*ubiint->chunk));
    if (!ubiint->chunk)
        return AVERROR(ENOMEM);

    for (int n = 0; n < nb_chunks; n++)
        ubiint->chunk[n].offset = avio_rl32(pb);

    offset = nb_chunks * 4LL + 0x2c;
    avio_seek(pb, offset + 0x24+0x40, SEEK_SET);
    width  = avio_rl32(pb);
    height = avio_rl32(pb);
    avio_skip(pb, 8);
    nb_frames = avio_rl32(pb);
    if (nb_chunks < nb_frames/5)
        return AVERROR_INVALIDDATA;

    if (layers < 0 || layers > 5)
        return AVERROR_INVALIDDATA;

    if (width <= 0 || height <= 0 ||
        width > INT_MAX/16 ||
        height > INT_MAX/16)
        return AVERROR_INVALIDDATA;
    width *= 16;
    height *= 16;

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    avio_skip(pb, 12);
    ubiint->vblock = av_calloc(nb_frames, sizeof(*ubiint->vblock));
    if (!ubiint->vblock)
        return AVERROR(ENOMEM);

    int64_t prev_size = avio_rl32(pb);
    for (int n = 0; n < nb_frames; n++) {
        int64_t next_size = avio_rl32(pb);

        ubiint->vblock[n].size = next_size - prev_size;
        if (ubiint->vblock[n].size <= 0)
            break;

        prev_size = next_size;
    }

    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_IPU;
    vst->codecpar->width = width;
    vst->codecpar->height = height;
    vst->codecpar->profile = 1;
    vst->start_time = 0;
    vst->duration =
    vst->nb_frames = nb_frames;
    vst->avg_frame_rate = av_d2q(av_int2float(fps), 10000);
    ffstream(vst)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(vst, 64, vst->avg_frame_rate.den, vst->avg_frame_rate.num);
    nb_aframes = (size[2] - 20) / 4;
    start_offset = offset + size[0];
    start_offset += size[2] * layers;

    start_offset = FFALIGN(start_offset, 64);

    for (int l = 0; l < layers && size[2] > 0 && nb_aframes > 0; l++) {
        avio_seek(pb, offset + size[0] + l * size[2], SEEK_SET);

        avio_skip(pb, 16);
        rate = avio_rl32(pb);

        if (rate > 0) {
            ast = avformat_new_stream(s, NULL);
            if (!ast)
                return AVERROR(ENOMEM);

            ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
            ast->codecpar->ch_layout.nb_channels = 2;
            ast->codecpar->sample_rate = rate;
            ast->start_time = 0;

            avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

            ubiint->ablock[l] = av_calloc(nb_aframes, sizeof(*ubiint->ablock[0]));
            if (!ubiint->ablock[l])
                return AVERROR(ENOMEM);

            int64_t prev_size = avio_rl32(pb);
            for (int n = 0; n < nb_aframes; n++) {
                int64_t next_size = avio_rl32(pb);

                ubiint->ablock[l][n].size = next_size - prev_size;
                if (ubiint->ablock[l][n].size <= 0)
                    break;

                prev_size = next_size;
            }
        }
    }

    int64_t pkt_off = start_offset;
    int64_t ts = 0;
    for (int i = 0, ci = 0, cidx = 0, an = 0; i < nb_frames; i++) {
        int size = ubiint->vblock[i].size;

        if (ci >= frames_per_chunk) {
            pkt_off = start_offset + ubiint->chunk[cidx++].offset;
            ci = 0;
        }
        ci++;

        if ((ret = av_add_index_entry(vst, pkt_off, i, size, 0, AVINDEX_KEYFRAME)) < 0)
            break;

        pkt_off += size;
        if (!((i+1) % frames_per_chunk)) {
            for (int l = 0; l < layers; l++) {
                int size = ubiint->ablock[l][an].size;
                AVStream *ast = s->streams[l+1];

                if ((ret = av_add_index_entry(ast, pkt_off, ts, size, 0, AVINDEX_KEYFRAME)) < 0)
                    return ret;

                pkt_off += size;
                ts += size/4;
            }

            an++;
        }
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, index = -1, size = 0;
    AVIOContext *pb = s->pb;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        FFStream *sti = ffstream(st);
        AVIndexEntry *e;

        for (int n = 0; n < sti->nb_index_entries; n++) {
            e = &sti->index_entries[n];
            if (pos == e->pos) {
                index = i;
                size = e->size;
                break;
            }
        }

        if (index >= 0)
            break;
    }

    if (index < 0) {
        avio_skip(pb, 1);
        return FFERROR_REDO;
    }

    ret = av_get_packet(pb, pkt, size);
    pkt->stream_index = index;
    pkt->pos = pos;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    AVIndexEntry *ie;
    int64_t pos;
    int index;

    index = ff_index_search_timestamp(ffstream(st)->index_entries,
                                      ffstream(st)->nb_index_entries, ts, flags);
    if (index < 0) {
        return AVERROR(EINVAL);
    } else {
        ie = &ffstream(st)->index_entries[index];
    }
    ffstream(st)->cur_dts = ie->timestamp;
    pos = ie->pos;

    avio_seek(pb, pos, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    UBIINTContext *ubiint = s->priv_data;

    for (int i = 0; i < 5; i++)
        av_freep(&ubiint->ablock[i]);
    av_freep(&ubiint->vblock);
    av_freep(&ubiint->chunk);

    return 0;
}

const FFInputFormat ff_ubiint_demuxer = {
    .p.name         = "ubiint",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PlayStation 2 INT"),
    .p.extensions   = "int",
    .priv_data_size = sizeof(UBIINTContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
    .read_seek      = read_seek,
};
