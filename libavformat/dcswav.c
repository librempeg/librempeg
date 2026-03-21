/*
 * DCS/WAV demuxer
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
#include "pcm.h"

typedef struct DCSWAVContext {
    AVClass     *class;
    AVIOContext *pb;
} DCSWAVContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x40)
        return 0;

    if (AV_RB32(p->buf) != MKBETAG('R','I','F','F'))
        return 0;
    if (AV_RB32(p->buf+8) != MKBETAG('W','A','V','E'))
        return 0;
    if (AV_RB32(p->buf+12) != MKBETAG('4','X','.','v'))
        return 0;
    if (AV_RB32(p->buf+60) != MKBETAG('@','n','a','m'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    char title[129] = { 0 };
    int ret, channels, rate;
    int64_t fmt_offset;
    AVStream *st;

    avio_skip(pb, 0x40);
    fmt_offset = 0x44LL + avio_rl32(pb);
    fmt_offset += fmt_offset & 1;
    avio_seek(pb, fmt_offset, SEEK_SET);
    if (avio_rb32(pb) != MKBETAG('f','m','t',' '))
        return AVERROR_INVALIDDATA;
    fmt_offset += 8;
    avio_seek(pb, fmt_offset, SEEK_SET);
    if (avio_rl16(pb) != 0x05)
        return AVERROR_INVALIDDATA;
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    if (channels <= 0 || rate <= 0 || channels > INT_MAX/0x4000)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_AICA;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = 0x4000 * channels;

    avio_seek(pb, 0x44, SEEK_SET);
    if ((ret = avio_get_str(pb, INT_MAX, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&st->metadata, "title", title, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    {
        DCSWAVContext *dcswav = s->priv_data;
        char *dcs_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!dcs_file_name)
            return AVERROR(ENOMEM);

        len = strlen(dcs_file_name);
        if (len > 3) {
            if (dcs_file_name[len-3] == 'w') {
                dcs_file_name[len-3] = 'd';
                dcs_file_name[len-2] = 'c';
                dcs_file_name[len-1] = 's';
            } else {
                dcs_file_name[len-3] = 'D';
                dcs_file_name[len-2] = 'C';
                dcs_file_name[len-1] = 'S';
            }
        } else {
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_open(s, &dcswav->pb, dcs_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&dcs_file_name);
        if (ret < 0)
            return ret;

        st->duration = (avio_size(dcswav->pb) * 2LL) / channels;
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DCSWAVContext *dcswav = s->priv_data;
    AVStream *st = s->streams[0];
    AVCodecParameters *par = st->codecpar;
    AVIOContext *pb = dcswav->pb;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    DCSWAVContext *dcswav = s->priv_data;
    const int sti = av_clip(stream_index, 0, s->nb_streams-1);
    AVStream *st = s->streams[sti];
    AVIOContext *pb = dcswav->pb;
    int64_t pos;

    if (ts < 0)
        ts = 0;

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
    DCSWAVContext *dcswav = s->priv_data;

    s->io_close2(s, dcswav->pb);

    return 0;
}

const FFInputFormat ff_dcswav_demuxer = {
    .p.name         = "dcswav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Utero Dreamcast DCS/WAV"),
    .priv_data_size = sizeof(DCSWAVContext),
    .p.extensions   = "dcswav",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
