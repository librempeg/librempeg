/*
 * SSP/SD9 demuxer
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
#include "riff.h"

typedef struct SD9Stream {
    int64_t start_offset;
    int64_t stop_offset;
} SD9Stream;

static int sd9_probe_sub(const AVProbeData *p, int off)
{
    if (AV_RL32(p->buf + off + 0x00) != MKTAG('S','D','9', 0 ) || AV_RL32(p->buf + off + 0x04) != 0x20 || AV_RB16(p->buf + off + 0x0C) != 0x3132 ||
        AV_RL32(p->buf + off + 0x20) != MKTAG('R','I','F','F') || AV_RL32(p->buf + off + 0x28) != MKTAG('W','A','V','E') ||
        AV_RL32(p->buf + off + 0x2C) != MKTAG('f','m','t',' ') || AV_RL32(p->buf + off + 0x66) != MKTAG('f','a','c','t') ||
        AV_RL32(p->buf + off + 0x6A) != 4                      || AV_RL32(p->buf + off + 0x72) != MKTAG('d','a','t','a'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sd9_probe(const AVProbeData *p)
{
    return sd9_probe_sub(p, 0);
}

static int ssp_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf + 0x10) < 0x4C || AV_RL32(p->buf + 0x10) != AV_RL32(p->buf + 0x48) || AV_RL32(p->buf + 0x14) < 1)
        return 0;

    return sd9_probe_sub(p, AV_RL32(p->buf + 0x10));
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SD9Stream *ss1 = s1->priv_data;
    const SD9Stream *ss2 = s2->priv_data;

    return FFDIFFSIGN(ss1->start_offset, ss2->start_offset);
}

static int sd9_read_stream(AVFormatContext *s, AVStream *st, int64_t start_offset)
{
    int ret;
    int16_t loop_count;
    int32_t wav_offset, loop_start, loop_end;
    AVIOContext *pb = s->pb;
    SD9Stream *sst = av_mallocz(sizeof(SD9Stream));
    if (!sst)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->priv_data = sst;

    avio_seek(pb, start_offset, SEEK_SET);
    if (avio_rl32(pb) != MKTAG('S','D','9', 0 ))
        return AVERROR_INVALIDDATA;

    wav_offset = avio_rl32(pb);
    sst->start_offset = start_offset + 0x5A + wav_offset;

    avio_skip(pb, 6);
    loop_count = avio_rl16(pb);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    avio_skip(pb, 8);

    if (loop_count > 0)
        av_dict_set_int(&st->metadata, "loop_count", loop_count, 0);

    avio_skip(pb, wav_offset - 0x20);
    if (avio_rl32(pb) != MKTAG('R','I','F','F'))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    if (avio_rl32(pb) != MKTAG('W','A','V','E') || avio_rl32(pb) != MKTAG('f','m','t',' '))
        return AVERROR_INVALIDDATA;

    if ((ret = ff_get_wav_header(s, pb, st->codecpar, avio_rl32(pb), 0)) < 0)
        return ret;
    if (st->codecpar->codec_id != AV_CODEC_ID_ADPCM_MS) {
        avpriv_request_sample(s, "codec id isn't MSADPCM");
        return AVERROR_PATCHWELCOME;
    }

    if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_MS && st->codecpar->ch_layout.nb_channels > 2 &&
        st->codecpar->block_align < INT_MAX / st->codecpar->ch_layout.nb_channels)
        st->codecpar->block_align *= st->codecpar->ch_layout.nb_channels;

    if (avio_rl32(pb) != MKTAG('f','a','c','t') || avio_rl32(pb) != 4)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);

    if (avio_rl32(pb) != MKTAG('d','a','t','a'))
        return AVERROR_INVALIDDATA;
    sst->stop_offset = sst->start_offset + avio_rl32(pb);

    if (loop_count > 0 || loop_end > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start / 2 / st->codecpar->ch_layout.nb_channels, 0);
    if (loop_end > 0)
        av_dict_set_int(&st->metadata, "loop_end", loop_end / 2 / st->codecpar->ch_layout.nb_channels, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int ssp_read_header(AVFormatContext *s)
{
    int ret;
    uint32_t first;
    int64_t pos;
    char title[0x10];
    AVIOContext *pb = s->pb;

    if ((ret = avio_get_str(pb, 0x10, title, sizeof(title))) < 0)
        return ret;
    av_dict_set(&s->metadata, "title", title, 0);
    avio_seek(pb, 0x10, SEEK_SET);

    first = avio_rl32(pb);

    avio_seek(pb, 0x48, SEEK_SET);
    while (first > avio_tell(pb)) {
        uint32_t offset = avio_rl32(pb);
        if (offset == 0)
            continue;

        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        pos = avio_tell(pb);
        if ((ret = sd9_read_stream(s, st, offset)) < 0)
            return ret;
        avio_seek(pb, pos, SEEK_SET);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        SD9Stream *sst = st->priv_data;

        avio_seek(pb, sst->start_offset, SEEK_SET);
    }

    return 0;
}

static int sd9_read_header(AVFormatContext *s)
{
    int ret;
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if ((ret = sd9_read_stream(s, st, 0)) < 0)
        return ret;

    {
        AVStream *st = s->streams[0];
        SD9Stream *sst = st->priv_data;

        avio_seek(s->pb, sst->start_offset, SEEK_SET);
    }
    return 0;
}

static int sd9_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        SD9Stream *sst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= sst->start_offset && pos < sst->stop_offset) {
            block_size = FFMIN(sst->stop_offset - pos, st->codecpar->block_align);

            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;
            break;
        } else if (pos >= sst->stop_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            SD9Stream *sst_next = st_next->priv_data;
            if (sst_next->start_offset > pos)
                avio_seek(pb, sst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_ssp_demuxer = {
    .p.name         = "ssp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SSP (Konami)"),
    .p.extensions   = "ssp",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = ssp_probe,
    .read_header    = ssp_read_header,
    .read_packet    = sd9_read_packet,
};

const FFInputFormat ff_sd9_demuxer = {
    .p.name         = "sd9",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SD9 (Konami)"),
    .p.extensions   = "sd9",
    .read_probe     = sd9_probe,
    .read_header    = sd9_read_header,
    .read_packet    = sd9_read_packet,
};
