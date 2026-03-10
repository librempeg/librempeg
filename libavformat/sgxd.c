/*
 * SGXD demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "riff.h"

typedef struct SGXDStream {
    int64_t start_offset;
    int64_t stop_offset;

    AVFormatContext *ctx;
} SGXDStream;

typedef struct SGXDContext {
    int current_stream;
    AVIOContext *pb;
} SGXDContext;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SGXDStream *ss1 = s1->priv_data;
    const SGXDStream *ss2 = s2->priv_data;

    return FFDIFFSIGN(ss1->start_offset, ss2->start_offset);
}

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','G','X','D'))
        return 0;

    return AVPROBE_SCORE_MAX/3;
}

static int find_chunk(AVIOContext *pb, uint32_t chunk_id, int64_t start_offset)
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
    int64_t base2_offset, base3_offset, data_offset, offset;
    SGXDContext *sgxd = s->priv_data;
    int ret, is_sgd, nb_streams;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 8);
    base2_offset = avio_rl32(pb);
    base3_offset = avio_rl32(pb);

    is_sgd = !!(base3_offset & (1 << 31));
    if (!is_sgd && avio_size(pb) != base2_offset)
        is_sgd = 1;

    if (is_sgd) {
        data_offset = base2_offset;
    } else {
        data_offset = 0;
    }

    if (!is_sgd) {
        char *sgh_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!sgh_file_name)
            return AVERROR(ENOMEM);

        len = strlen(sgh_file_name);
        if (len > 3) {
            sgh_file_name[len-1] = 'b';
        } else {
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_open(s, &sgxd->pb, sgh_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&sgh_file_name);
        if (ret < 0)
            return ret;
    }

    ret = find_chunk(pb, AV_RB32("WAVE"), 16);
    if (ret < 0)
        return ret;

    avio_skip(pb, 4);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    offset = avio_tell(pb);
    for (int n = 0; n < nb_streams; n++) {
        int64_t name_offset, duration, start_offset, stop_offset;
        int codec, channels, rate, align, riff = 0;
        SGXDStream *sst;
        AVStream *st;

        avio_seek(pb, offset + 0x38 * n, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 4);
        name_offset = avio_rl32(pb);
        codec = avio_r8(pb);
        channels = avio_r8(pb);
        avio_skip(pb, 2);
        rate = avio_rl32(pb);
        avio_skip(pb, 16);
        duration = avio_rl32(pb);
        avio_skip(pb, 8);
        stop_offset = avio_rl32(pb);
        start_offset = avio_rl32(pb);
        start_offset += data_offset;
        stop_offset += start_offset;
        if (channels <= 0 || rate <= 0)
            return AVERROR_INVALIDDATA;

        switch (codec) {
        case 0x00:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 2 * channels;
            break;
        case 0x01:
            codec = AV_CODEC_ID_PCM_S16BE;
            align = 2 * channels;
            break;
        case 0x04:
            riff = 1;
            break;
        default:
            return AVERROR_PATCHWELCOME;
        }

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->block_align = align;

        if (riff) {
            AVIOContext *pb = sgxd->pb ? sgxd->pb : s->pb;

            avio_seek(pb, start_offset + 16, SEEK_SET);

            if ((ret = ff_get_wav_header(s, pb, st->codecpar, avio_rl32(pb), 0)) < 0)
                return ret;

            ret = find_chunk(pb, AV_RB32("data"), avio_tell(pb));
            if (ret < 0)
                return ret;

            start_offset = avio_tell(pb);
        }

        sst->start_offset = start_offset;
        sst->stop_offset = stop_offset;

        if (name_offset > 0) {
            char title[129] = {0};

            avio_seek(pb, name_offset, SEEK_SET);
            if ((ret = avio_get_str(pb, INT_MAX, title, sizeof(title))) < 0)
                return ret;

            if (title[0] != '\0')
                av_dict_set(&st->metadata, "title", title, 0);
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVIOContext *pb = sgxd->pb ? sgxd->pb : s->pb;
        AVStream *st = s->streams[0];
        SGXDStream *sst = st->priv_data;

        avio_seek(pb, sst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SGXDContext *sgxd = s->priv_data;
    AVIOContext *pb = sgxd->pb ? sgxd->pb : s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SGXDStream *sst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (sgxd->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[sgxd->current_stream];
    sst = st->priv_data;
    if (do_seek)
        avio_seek(pb, sst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= sst->stop_offset) {
        do_seek = 1;
        sgxd->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = st->codecpar->block_align;
        const int size = FFMIN(block_size, sst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    }
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        sgxd->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    SGXDContext *sgxd = s->priv_data;
    AVIOContext *pb = sgxd->pb ? sgxd->pb : s->pb;
    const int sti = av_clip(stream_index, 0, s->nb_streams-1);
    AVStream *st = s->streams[sti];
    int64_t pos;

    {
        AVIndexEntry *ie;
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
    }

    sgxd->current_stream = sti;
    avio_seek(pb, pos, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    SGXDContext *sgxd = s->priv_data;

    s->io_close2(s, sgxd->pb);

    return 0;
}

const FFInputFormat ff_sgxd_demuxer = {
    .p.name         = "sgxd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony/SCEI SGX"),
    .priv_data_size = sizeof(SGXDContext),
    .p.extensions   = "sgh,sgd,sgb",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
