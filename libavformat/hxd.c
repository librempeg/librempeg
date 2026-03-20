/*
 * HXD demuxer
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
#include "libavcodec/mathops.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct HXDContext {
    AVClass     *class;
    AVIOContext *pb;
    int current_stream;
} HXDContext;

typedef struct HXDStream {
    int64_t start_offset;
    int64_t stop_offset;
} HXDStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKTAG('H', 'X', 'D', '\0'))
        return 0;

    if (p->buf_size < 24)
        return 0;
    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const HXDStream *hs1 = s1->priv_data;
    const HXDStream *hs2 = s2->priv_data;

    return FFDIFFSIGN(hs1->start_offset, hs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int ret, nb_streams, bank, header_size, align, channels;
    HXDContext *hxd = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 8);
    nb_streams = avio_rl32(pb);
    bank = avio_rl32(pb);
    header_size = avio_rl32(pb);
    align = avio_rl32(pb);

    if (bank) {
        channels = 1;
        align = 1024;
    } else {
        channels = nb_streams;
        nb_streams = 1;
    }

    if (channels <= 0 || nb_streams <= 0 || align <= 0 || header_size <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    {
        char *str_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!str_file_name)
            return AVERROR(ENOMEM);

        len = strlen(str_file_name);
        if (len > 3) {
            if (bank) {
                str_file_name[len-3] = 'b';
                str_file_name[len-2] = 'd';
                str_file_name[len-1] = '\0';
            } else {
                str_file_name[len-3] = 's';
                str_file_name[len-2] = 't';
                str_file_name[len-1] = 'r';
            }
        } else {
            av_freep(&str_file_name);
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_open(s, &hxd->pb, str_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&str_file_name);
        if (ret < 0)
            return ret;
    }

    for (int n = 0; n < nb_streams; n++) {
        int64_t stream_offset, loop_start, loop_end;
        int rate, loop_flag;
        uint32_t flags;
        HXDStream *hst;
        AVStream *st;

        avio_seek(pb, 0x20LL + n * 0x1CLL, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        rate =  avio_rl32(pb);
        stream_offset = avio_rl32(pb);
        avio_skip(pb, 8);
        flags =  avio_rl32(pb);
        loop_start = avio_rl32(pb) * 0x20LL;
        loop_end = avio_rl32(pb) * 0x20LL;
        loop_flag = flags & 0x20;
        if (rate <= 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        hst = av_mallocz(sizeof(*hst));
        if (!hst)
            return AVERROR(ENOMEM);
        st->priv_data = hst;

        hst->start_offset = stream_offset;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align * channels;
        st->codecpar->ch_layout.nb_channels = channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        if (loop_start > 0 && loop_flag)
            av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
        if (loop_end > 0 && loop_flag)
            av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        HXDStream *hst = st->priv_data;

        if (n + 1 < s->nb_streams) {
            AVStream *nst = s->streams[n+1];
            HXDStream *hnst = nst->priv_data;

            hst->stop_offset = hnst->start_offset;
        } else {
            hst->stop_offset = avio_size(hxd->pb);
        }
        st->index = n;
        st->duration = (hst->stop_offset - hst->start_offset) / (16LL*channels) * 28LL;
    }

    {
        AVStream *st = s->streams[0];
        HXDStream *hst = st->priv_data;
        AVIOContext *pb = hxd->pb;

        avio_seek(pb, hst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    HXDContext *hxd = s->priv_data;
    AVIOContext *pb = hxd->pb;
    AVCodecParameters *par;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    HXDStream *hst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (hxd->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[hxd->current_stream];
    hst = st->priv_data;
    par = st->codecpar;

    if (do_seek)
        avio_seek(pb, hst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= hst->stop_offset) {
        do_seek = 1;
        hxd->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int size = FFMIN(par->block_align, hst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
        pkt->stream_index = st->index;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    HXDContext *hxd = s->priv_data;
    AVIOContext *pb = hxd->pb;
    HXDStream *hst;
    AVStream *st;
    int64_t pos;

    hxd->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[hxd->current_stream];
    hst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < hst->start_offset) {
        avio_seek(pb, hst->start_offset, SEEK_SET);
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
    HXDContext *hxd = s->priv_data;

    s->io_close2(s, hxd->pb);

    return 0;
}

const FFInputFormat ff_hxd_demuxer = {
    .p.name         = "hxd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Tecmo PS2 HXD"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "hxd",
    .priv_data_size = sizeof(HXDContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
