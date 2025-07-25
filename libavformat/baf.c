/*
 * BAF demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef unsigned int (*avio_r32)(AVIOContext *s);

static int read_probe(const AVProbeData *p)
{
    uint32_t version;

    if (AV_RB32(p->buf) != MKBETAG('B','A','N','K'))
        return 0;

    if (AV_RB32(p->buf+4) == 0)
        return 0;

    if (AV_RL32(p->buf+4) > AV_RB32(p->buf+4)) {
        version = AV_RB32(p->buf+8);
    } else {
        version = AV_RL32(p->buf+8);
    }

    if (version < 3 || version > 5)
        return 0;

    if (AV_RB32(p->buf+12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

typedef struct BAFStream {
    int64_t start_offset;
    int64_t stop_offset;
} BAFStream;

static int read_header(AVFormatContext *s)
{
    uint32_t offset, version, nb_tracks, nb_streams = 0, codec, start_offset, stream_size;
    uint32_t first_start_offset;
    uint8_t stream_name[33] = { 0 };
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32;
    AVStream *st;
    int ret;

    avio_skip(pb, 4);
    offset = avio_rb32(pb);
    if (offset > av_bswap32(offset)) {
        avio_r32 = avio_rl32;
        offset = av_bswap32(offset);
    } else {
        avio_r32 = avio_rb32;
    }

    version = avio_r32(pb);
    nb_tracks = avio_r32(pb);

    if (version < 3 || version > 5)
        return AVERROR_INVALIDDATA;

    if (offset < avio_tell(pb))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, offset - avio_tell(pb));

    for (int n = 0; n < nb_tracks; n++) {
        int64_t metadata_end = avio_tell(pb);
        uint32_t tag, sub_tracks;
        BAFStream *bst;

        tag = avio_rb32(pb);
        if (tag != MKBETAG('W','A','V','E') &&
            tag != MKBETAG('C','U','E',' '))
            return AVERROR_INVALIDDATA;

        metadata_end += avio_r32(pb);

        if (tag == MKBETAG('C','U','E',' '))
            goto next;

        codec = avio_r32(pb);
        ret = avio_get_str(pb, sizeof(stream_name)-1, stream_name, sizeof(stream_name));
        if (ret < 0)
            return ret;
        if (ret < sizeof(stream_name)-1)
            avio_skip(pb, sizeof(stream_name) - ret - 1);
        start_offset = avio_r32(pb);
        if (n == 0)
            first_start_offset = start_offset;
        stream_size = avio_r32(pb);

        bst = av_mallocz(sizeof(BAFStream));
        if (!bst)
            return AVERROR(ENOMEM);
        bst->start_offset = start_offset;
        bst->stop_offset = bst->start_offset + stream_size;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->id = nb_streams++;
        st->priv_data = bst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->start_time = 0;

        switch (codec) {
        case 3:
            st->codecpar->codec_id = (avio_r32 == avio_rb32) ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
            switch (version) {
            case 3:
                avio_skip(pb, 4);
                st->codecpar->sample_rate = avio_r32(pb);
                if (st->codecpar->sample_rate <= 0)
                    return AVERROR_INVALIDDATA;

                avio_skip(pb, 4);
                st->codecpar->ch_layout.nb_channels = avio_r32(pb);
                if (st->codecpar->ch_layout.nb_channels == 0)
                    return AVERROR_INVALIDDATA;
                break;
            case 4:
                avio_skip(pb, 8);
                st->codecpar->sample_rate = avio_r32(pb);
                if (st->codecpar->sample_rate <= 0)
                    return AVERROR_INVALIDDATA;

                avio_skip(pb, 4);
                st->codecpar->ch_layout.nb_channels = avio_r32(pb);
                if (st->codecpar->ch_layout.nb_channels == 0)
                    return AVERROR_INVALIDDATA;
                break;
            case 5:
                avio_skip(pb, 12);
                st->codecpar->sample_rate = avio_r32(pb);
                if (st->codecpar->sample_rate <= 0)
                    return AVERROR_INVALIDDATA;
                break;
            }
            st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;
            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
            break;
        case 7:
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSXC;
            avio_skip(pb, 12);
            st->codecpar->sample_rate = avio_r32(pb);
            if (st->codecpar->sample_rate <= 0)
                return AVERROR_INVALIDDATA;

            st->duration = avio_r32(pb);
            avio_skip(pb, 1);
            sub_tracks = avio_r8(pb);
            sub_tracks = FFMAX(1, sub_tracks);
            avio_skip(pb, 1);
            st->codecpar->ch_layout.nb_channels = sub_tracks * avio_r8(pb);
            if (st->codecpar->ch_layout.nb_channels == 0)
                return AVERROR_INVALIDDATA;

            st->codecpar->block_align = 33 * st->codecpar->ch_layout.nb_channels;
            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
            break;
        case 8:
        case 9:
            st->codecpar->codec_id = (codec == 8) ? AV_CODEC_ID_XMA1 : AV_CODEC_ID_XMA2;
            switch (version) {
            case 4:
                avio_skip(pb, 8);
                break;
            case 5:
                avio_skip(pb, 12);
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
            st->codecpar->sample_rate = avio_r32(pb);
            if (st->codecpar->sample_rate <= 0)
                return AVERROR_INVALIDDATA;

            st->duration = avio_r32(pb);
            st->codecpar->ch_layout.nb_channels = avio_r32(pb);
            if (st->codecpar->ch_layout.nb_channels == 0)
                return AVERROR_INVALIDDATA;

            if ((ret = ff_alloc_extradata(st->codecpar, 8 + 20 * ((st->codecpar->ch_layout.nb_channels + 1) / 2))) < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            st->codecpar->extradata[4] = (st->codecpar->ch_layout.nb_channels + 1) / 2;
            for (int i = 0; i < st->codecpar->extradata[4]; i++)
                st->codecpar->extradata[8 + 20 * i + 17] = FFMIN(2, st->codecpar->ch_layout.nb_channels - i * 2);
            st->codecpar->block_align = 2048;
            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
            break;
        default:
            return AVERROR_INVALIDDATA;
        }

next:
        if (avio_tell(pb) < metadata_end)
            avio_skip(pb, metadata_end - avio_tell(pb));
    }

    if (first_start_offset < avio_tell(pb))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, first_start_offset - avio_tell(pb));

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        BAFStream *bst = st->priv_data;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= bst->start_offset && pos < bst->stop_offset) {
            ret = av_get_packet(pb, pkt, par->block_align);
            pkt->stream_index = st->id;
            break;
        } else if (pos >= bst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            BAFStream *bst_next = st_next->priv_data;

            if (bst_next->start_offset > pos)
                avio_skip(pb, bst_next->start_offset - pos);
        }
    }
    return ret;
}

const FFInputFormat ff_baf_demuxer = {
    .p.name         = "baf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BAF (Bizarre Creations Bank File)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "baf",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
