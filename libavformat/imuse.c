/*
 * iMUSE demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct IMUSEContext {
    int index;
    int is_comp;
} IMUSEContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) == MKBETAG('M','C','M','P')) {
        if (AV_RB16(p->buf+4) == 0)
            return 0;
    } else if (AV_RB32(p->buf) == MKBETAG('C','O','M','P')) {
        if ((int)AV_RB32(p->buf+4) <= 0)
            return 0;
    } else {
        return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t head_offset, data_offset, data_size, offset, ts;
    int channels = 0, rate = 0, nb_entries;
    IMUSEContext *imc = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t chunk_id;
    AVStream *st;

    chunk_id = avio_rb32(pb);
    if (chunk_id == MKBETAG('C','O','M','P')) {
        nb_entries = avio_rb32(pb);

        if (nb_entries < 0 || nb_entries > INT_MAX/16)
            return AVERROR_INVALIDDATA;

        head_offset = 16 + nb_entries * 16LL + 2;
        imc->is_comp = 1;
    } else {
        nb_entries = avio_rb16(pb);

        head_offset = 6 + nb_entries * 9LL;
        avio_seek(pb, head_offset, SEEK_SET);
        head_offset += 2 + avio_rb16(pb);
    }

    avio_seek(pb, head_offset, SEEK_SET);
    chunk_id = avio_rb32(pb);
    if (chunk_id == MKBETAG('i','M','U','S')) {
        int64_t map_size, map_offset, offset, name_offset = 0;
        int header_found = 0;

        avio_skip(pb, 4);
        if (avio_rb32(pb) != MKBETAG('M','A','P',' '))
            return AVERROR_INVALIDDATA;

        map_size = avio_rb32(pb);
        map_offset = head_offset + 16;

        offset = map_offset;
        while (offset < map_offset + map_size) {
            uint32_t type, size;

            avio_seek(pb, offset, SEEK_SET);
            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;

            type = avio_rb32(pb);
            size = avio_rb32(pb);
            offset += 8;

            switch (type) {
            case MKBETAG('F','R','M','T'):
                if (header_found)
                    return AVERROR_INVALIDDATA;
                header_found = 1;
                avio_skip(pb, 12);
                rate = avio_rb32(pb);
                channels = avio_rb32(pb);
                break;
            case MKBETAG('T','E','X','T'):
                if (!name_offset)
                    name_offset = offset + 4;
                break;
            default:
                break;
            }

            offset += size;
        }

        if (!header_found)
            return AVERROR_INVALIDDATA;

        avio_seek(pb, head_offset + 16 + map_size, SEEK_SET);
        if (avio_rb32(pb) != MKBETAG('D','A','T','A'))
            return AVERROR_INVALIDDATA;
        data_size = avio_rb32(pb);
        data_offset = avio_tell(pb);
    } else if (chunk_id == MKBETAG('R','I','F','F')) {
        return AVERROR_PATCHWELCOME;
    } else {
        return AVERROR_INVALIDDATA;
    }

    if (channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = data_size / channels / 2;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_VIMA;
    st->codecpar->profile = imc->is_comp;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (imc->is_comp) {
        avio_seek(pb, 16, SEEK_SET);
        ts = 0;
        for (int i = 0; i < nb_entries; i++) {
            int64_t poffset, size;
            uint8_t flags;
            int data, ret;

            poffset = avio_rb32(pb);
            size = avio_rb32(pb);
            flags = avio_rb32(pb);
            avio_skip(pb, 4);
            data = 0x2000;

            if (i > 0 && (flags & 0x0D || flags & 0x0F)) {
                ret = av_add_index_entry(st, poffset, ts, size, data/channels/2, AVINDEX_KEYFRAME);
                if (ret < 0)
                    return ret;
                ts += data/channels/2;
            }
        }
    } else {
        avio_seek(pb, 6, SEEK_SET);
        offset = data_offset;
        ts = 0;
        for (int i = 0; i < nb_entries; i++) {
            uint8_t flags;
            int data, ret;
            int64_t size;

            flags = avio_r8(pb);
            data = avio_rb32(pb);
            size = avio_rb32(pb);

            if (flags & 1) {
                ret = av_add_index_entry(st, offset, ts, size, data/channels/2, AVINDEX_KEYFRAME);
                if (ret < 0)
                    return ret;
                offset += size;
                ts += data/channels/2;
            }
        }
    }

    avio_seek(pb, data_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    IMUSEContext *imc = s->priv_data;
    AVStream *st = s->streams[0];
    FFStream *sti = ffstream(st);
    AVIOContext *pb = s->pb;
    int ret;

    if (imc->index >= sti->nb_index_entries ||
        imc->index < 0)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    const AVIndexEntry *const e = &sti->index_entries[imc->index];

    avio_seek(pb, e->pos, SEEK_SET);
    ret = av_new_packet(pkt, e->size+4);
    avio_read(pb, pkt->data + 4, e->size);
    AV_WB32(pkt->data, e->min_distance);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->duration = e->min_distance;
    pkt->stream_index = 0;
    pkt->pos = e->pos;
    imc->index++;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    IMUSEContext *imc = s->priv_data;
    AVStream *st = s->streams[0];
    int64_t pos;
    int ret;

    if (!(s->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return -1;

    ret = av_index_search_timestamp(st, timestamp, flags);
    if (ret < 0)
        return ret;

    pos = ffstream(st)->index_entries[ret].pos;
    pos = avio_seek(s->pb, pos, SEEK_SET);
    if (pos < 0)
        return pos;

    imc->index = ret;

    return 0;
}

const FFInputFormat ff_imuse_demuxer = {
    .p.name         = "imuse",
    .p.long_name    = NULL_IF_CONFIG_SMALL("iMUSE (LucasArts Interactive Streaming Engine)"),
    .priv_data_size = sizeof(IMUSEContext),
    .p.extensions   = "imc,imx",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
