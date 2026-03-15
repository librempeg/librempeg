/*
 * SNDX demuxer
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

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/mathops.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct SNDXContext {
    AVClass     *class;
    AVIOContext *pb;
    int current_stream;
} SNDXContext;

typedef struct SNDXStream {
    int64_t start_offset;
    int64_t stop_offset;
} SNDXStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S', 'X', 'D', 'F') &&
        AV_RB32(p->buf) != MKBETAG('S', 'X', 'D', 'S'))
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SNDXStream *ss1 = s1->priv_data;
    const SNDXStream *ss2 = s2->priv_data;

    return FFDIFFSIGN(ss1->start_offset, ss2->start_offset);
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
    SNDXContext *sndx = s->priv_data;
    int64_t data_size, offset;
    AVIOContext *pb = s->pb;
    int ret, nb_streams;

    avio_skip(pb, 8);
    data_size = avio_rl32(pb);

    avio_seek(pb, 0x60, SEEK_SET);
    ret = find_chunk(pb, AV_RB32("WAVE"));
    if (ret < 0)
        return ret;

    avio_skip(pb, 4);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    offset = avio_tell(pb);
    for (int n = 0; n < nb_streams; n++) {
        int64_t duration, stream_offset, header_offset;
        int codec, channels, sample_rate, align;
        int64_t loop_start, loop_end;
        uint32_t stream_size, flags;
        SNDXStream *sst;
        AVStream *st;

        avio_seek(pb, offset + 4LL * n, SEEK_SET);
        header_offset = offset + 4LL * n + avio_rl32(pb);
        avio_seek(pb, header_offset, SEEK_SET);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        flags = avio_rl32(pb);
        codec = avio_r8(pb);
        channels = avio_r8(pb);
        avio_skip(pb, 2);
        sample_rate = avio_rl32(pb);
        avio_skip(pb, 8);
        duration = avio_rl32(pb);
        loop_start = avio_rl32(pb);
        loop_end = avio_rl32(pb);
        stream_size = avio_rl32(pb);
        stream_offset = avio_rl32(pb);

        if (channels <= 0 || sample_rate <= 0)
            return AVERROR_INVALIDDATA;

        switch (codec) {
        case 0x20:
            codec = AV_CODEC_ID_ADPCM_PSX;
            align = 16;
            break;
        case 0x21:
            codec = AV_CODEC_ID_ADPCM_HEVAG;
            align = 32 * 0x10;
            break;
        case 0x42:
            codec = AV_CODEC_ID_ATRAC9;
            align = 512;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unsupported codec: %d\n", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (align >= INT_MAX/channels)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->sample_rate = sample_rate;
        st->codecpar->block_align = align * channels;
        st->codecpar->ch_layout.nb_channels = channels;
        if (codec == AV_CODEC_ID_ATRAC9) {
            int64_t extra_offset = header_offset + 0x28;
            uint32_t config = 0;

            if (!(flags & 1))
                return AVERROR_INVALIDDATA;

            avio_seek(pb, extra_offset, SEEK_SET);

            while (!avio_feof(pb)) {
                uint32_t tag = avio_rb32(pb);

                if (tag == 0x0A010000 || tag == 0x0A010600) {
                    config = avio_rl32(pb);
                    break;
                }
            }
            if (!config)
                return AVERROR_INVALIDDATA;

            ret = ff_alloc_extradata(st->codecpar, 12);
            if (ret < 0)
                return ret;

            AV_WL32(st->codecpar->extradata, 3);
            AV_WB32(st->codecpar->extradata+4, config);
            st->codecpar->block_align = (1 << ((config>>3) & 3)) * (((config >> 5) & 0x7ff) + 1);

            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        }

        if (loop_start > 0)
            av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
        if (loop_end > 0)
            av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        if (!(flags & 2))
            stream_offset += header_offset + 0x24;

        sst->start_offset = stream_offset;
        sst->stop_offset = sst->start_offset;
        sst->stop_offset += stream_size;

        if ((flags & 2) && avio_size(pb) < data_size && sndx->pb == NULL) {
            char *sndx_file_name = av_strdup(s->url);
            AVDictionary *tmp = NULL;
            int len;

            if (!sndx_file_name)
                return AVERROR(ENOMEM);

            len = strlen(sndx_file_name);
            if (len > 3) {
                sndx_file_name[len-1] = '2';
            } else {
                return AVERROR_INVALIDDATA;
            }

            ret = s->io_open(s, &sndx->pb, sndx_file_name, AVIO_FLAG_READ, &tmp);
            av_freep(&sndx_file_name);
            if (ret < 0)
                return ret;
        } else if (!(flags & 2) && sndx->pb != NULL && (stream_offset + stream_size > avio_size(pb))) {
            return AVERROR_INVALIDDATA;
        }
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        SNDXStream *sst = st->priv_data;
        AVIOContext *pb = sndx->pb ? sndx->pb : s->pb;

        avio_seek(pb, sst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SNDXContext *sndx = s->priv_data;
    AVIOContext *pb = sndx->pb ? sndx->pb : s->pb;
    AVCodecParameters *par;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SNDXStream *sst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (sndx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[sndx->current_stream];
    sst = st->priv_data;
    par = st->codecpar;

    if (do_seek)
        avio_seek(pb, sst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= sst->stop_offset) {
        do_seek = 1;
        sndx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int size = FFMIN(par->block_align, sst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
        pkt->stream_index = st->index;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    SNDXContext *sndx = s->priv_data;
    AVIOContext *pb = sndx->pb ? sndx->pb : s->pb;
    SNDXStream *sst;
    AVStream *st;
    int64_t pos;

    sndx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[sndx->current_stream];
    sst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < sst->start_offset) {
        avio_seek(pb, sst->start_offset, SEEK_SET);
        return 0;
    }

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

    avio_seek(pb, pos, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    SNDXContext *sndx = s->priv_data;

    s->io_close2(s, sndx->pb);

    return 0;
}

const FFInputFormat ff_sndx_demuxer = {
    .p.name         = "sndx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony SNDX"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sxd1,sxd,sxd3",
    .priv_data_size = sizeof(SNDXContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
