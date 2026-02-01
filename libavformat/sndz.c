/*
 * SNDZ demuxer
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

typedef struct SNDZContext {
    AVClass     *class;
    AVIOContext *pb;
    int current_stream;
} SNDZContext;

typedef struct SNDZStream {
    int64_t start_offset;
    int64_t stop_offset;
} SNDZStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S', 'N', 'D', 'Z'))
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SNDZStream *xs1 = s1->priv_data;
    const SNDZStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    SNDZContext *sndz = s->priv_data;
    int streamed, entries, entry_size, data_size;
    AVIOContext *pb = s->pb;
    int ret, nb_streams;
    int64_t offset = 0;

    avio_skip(pb, 8);
    data_size = avio_rl32(pb);
    if (data_size <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x70, SEEK_SET);
    avio_skip(pb, avio_rl32(pb) - 4);
    entries = avio_rl32(pb);
    if (entries <= 0)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < entries; i++) {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        if (avio_rb32(pb) == MKBETAG('W','A','V','S')) {
            avio_skip(pb, 4);
            offset += avio_rl32(pb);
            break;
        } else {
            avio_skip(pb, 8);
        }
    }

    if (offset == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, offset);
    avio_skip(pb, avio_rl32(pb) - 4);

    if (avio_rb32(pb) != MKBETAG('W','A','V','D'))
        return AVERROR_INVALIDDATA;

    entry_size = avio_rl32(pb);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0 || entry_size <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        int64_t entry_end = avio_tell(pb) + entry_size;
        int64_t name_offset, duration, stream_offset;
        int codec, channels, sample_rate, align;
        int32_t loop_start, loop_end;
        uint32_t config, stream_size;
        char title[31] = { 0 };
        SNDZStream *sst;
        AVStream *st;

        streamed = avio_rl32(pb);
        name_offset = avio_rl32(pb);
        name_offset += avio_tell(pb) - 4;
        avio_skip(pb, 8);
        codec = avio_r8(pb);
        channels = avio_r8(pb);
        avio_skip(pb, 2);
        sample_rate = avio_rl32(pb);
        duration = avio_rl32(pb);
        config = avio_rl32(pb);
        loop_start = avio_rl32(pb);
        loop_end = avio_rl32(pb);
        stream_size = avio_rl32(pb);
        stream_offset = avio_rl32(pb);

        if (channels <= 0 || sample_rate <= 0)
            return AVERROR_INVALIDDATA;

        switch (codec) {
        case 0x02:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 256*2;
            break;
        case 0x04:
            codec = AV_CODEC_ID_PCM_S24LE;
            align = 256*3;
            break;
        case 0x08:
            codec = AV_CODEC_ID_PCM_F32LE;
            align = 256*4;
            break;
        case 0x20:
            codec = AV_CODEC_ID_ADPCM_HEVAG;
            align = 32 * 0x10;
            break;
        case 0x21:
            codec = AV_CODEC_ID_ATRAC9;
            align = 512;
            break;
        default:
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
            ret = ff_alloc_extradata(st->codecpar, 12);
            if (ret < 0)
                return ret;

            AV_WL32(st->codecpar->extradata, 3);
            AV_WB32(st->codecpar->extradata+4, config);
            st->codecpar->block_align = 4 * (((config >> 5) & 0x7ff) + 1);

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

        sst->start_offset = stream_offset;
        sst->stop_offset = sst->start_offset;
        sst->stop_offset += stream_size;

        avio_seek(pb, name_offset, SEEK_SET);
        avio_get_str(pb, 30, title, sizeof(title));
        if (title[0])
            av_dict_set(&st->metadata, "title", title, 0);

        avio_seek(pb, entry_end, SEEK_SET);

        if (streamed && avio_size(pb) < data_size && sndz->pb == NULL) {
            char *sndz_file_name = av_strdup(s->url);
            AVDictionary *tmp = NULL;
            int len;

            if (!sndz_file_name)
                return AVERROR(ENOMEM);

            len = strlen(sndz_file_name);
            if (len > 3) {
                sndz_file_name[len-1] = '2';
            } else {
                return AVERROR_INVALIDDATA;
            }

            ret = s->io_open(s, &sndz->pb, sndz_file_name, AVIO_FLAG_READ, &tmp);
            av_freep(&sndz_file_name);
            if (ret < 0)
                return ret;
        } else if (sndz->pb != NULL && (stream_offset + stream_size > avio_size(pb))) {
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
        SNDZStream *sst = st->priv_data;
        AVIOContext *pb = sndz->pb ? sndz->pb : s->pb;

        avio_seek(pb, sst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SNDZContext *sndz = s->priv_data;
    AVIOContext *pb = sndz->pb ? sndz->pb : s->pb;
    AVCodecParameters *par;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SNDZStream *sst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (sndz->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[sndz->current_stream];
    sst = st->priv_data;
    par = st->codecpar;

    if (do_seek)
        avio_seek(pb, sst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= sst->stop_offset) {
        do_seek = 1;
        sndz->current_stream++;
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
    SNDZContext *sndz = s->priv_data;
    AVIOContext *pb = sndz->pb ? sndz->pb : s->pb;
    SNDZStream *sst;
    AVStream *st;
    int64_t pos;

    sndz->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[sndz->current_stream];
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
    SNDZContext *sndz = s->priv_data;

    s->io_close2(s, sndz->pb);

    return 0;
}

const FFInputFormat ff_sndz_demuxer = {
    .p.name         = "sndz",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony SNDZ"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "szd1,szd,szd3",
    .priv_data_size = sizeof(SNDZContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
