/*
 * Wii BNS demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    int64_t bns_offset = 0;

    if (p->buf_size >= 0x48 &&
        AV_RB32(p->buf + 0x40) == MKBETAG('I','M','E','T')) {
        bns_offset = AV_RB32(p->buf + 0x44);
        if (p->buf_size < bns_offset + 0x58)
            return 0;

        bns_offset += AV_RB32(p->buf + bns_offset + 0x54);
    }

    if (p->buf_size < bns_offset+8)
        return 0;

    if (AV_RB32(p->buf + bns_offset) == MKBETAG('I','M','D','5'))
        bns_offset += 0x20;
    if (AV_RB32(p->buf + bns_offset) != MKBETAG('B','N','S',' '))
        return 0;
    if (AV_RB32(p->buf + bns_offset + 4) != 0xFEFF0100u)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, info_offset = -1, data_offset = -1, offset, duration, bns_offset;
    int ret, rate, nb_channels, chunk_count, loop_flag;
    uint32_t loop_start, chunk_id, data_size;
    AVIOContext *pb = s->pb;
    AVStream *st;

    chunk_id = avio_rb32(pb);
    if (chunk_id == MKBETAG('I','M','D','5')) {
        avio_skip(pb, 28);
        chunk_id = avio_rb32(pb);
    }
    if (chunk_id == MKBETAG('B','N','S',' ')) {
        bns_offset = avio_tell(pb) - 4;
        chunk_id = avio_rb32(pb);
        if (chunk_id != 0xFEFF0100u)
            return AVERROR_INVALIDDATA;
    } else {
        avio_skip(pb, 60);
        chunk_id = avio_rb32(pb);
        if (chunk_id != MKBETAG('I','M','E','T'))
            return AVERROR_INVALIDDATA;
        avio_seek(pb, avio_rb32(pb), SEEK_SET);
        avio_skip(pb, 12);
        avio_skip(pb, avio_rb32(pb));
        chunk_id = avio_rb32(pb);
        if (chunk_id == MKBETAG('I','M','D','5')) {
            avio_skip(pb, 28);
            chunk_id = avio_rb32(pb);
        }
        if (chunk_id != MKBETAG('B','N','S',' '))
            return AVERROR_INVALIDDATA;
        bns_offset = avio_tell(pb) - 4;
        chunk_id = avio_rb32(pb);
        if (chunk_id != 0xFEFF0100u)
            return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 6);
    chunk_count = avio_rb16(pb);
    if (chunk_count < 2)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < chunk_count; n++) {
        int64_t chunk_offset, chunk_size;

        chunk_offset = bns_offset + avio_rb32(pb);
        chunk_size = avio_rb32(pb);
        offset = avio_tell(pb);

        avio_seek(pb, chunk_offset, SEEK_SET);
        chunk_id = avio_rb32(pb);
        switch (chunk_id) {
        case MKBETAG('I','N','F','O'):
            info_offset = chunk_offset + 8;
            break;
        case MKBETAG('D','A','T','A'):
            data_offset = chunk_offset + 8;
            data_size = chunk_size;
            break;
        }

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        avio_seek(pb, offset, SEEK_SET);
    }

    if (info_offset < 0 || data_offset < 0 || data_size <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, info_offset, SEEK_SET);
    if (avio_r8(pb))
        return AVERROR_INVALIDDATA;
    loop_flag = avio_r8(pb);
    nb_channels = avio_r8(pb);
    avio_skip(pb, 1);
    rate = avio_rb16(pb);
    avio_skip(pb, 2);
    loop_start = avio_rb32(pb);
    duration = avio_rb32(pb);
    offset = info_offset + avio_rb32(pb);

    if (nb_channels == 0 || rate == 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = data_size;

    if (loop_start > 0 && loop_flag)
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    for (int c = 0; c < nb_channels; c++) {
        int64_t channel_info_offset, channel_data_offset, channel_dsp_offset;

        avio_seek(pb, offset, SEEK_SET);
        channel_info_offset = info_offset + avio_rb32(pb);
        offset = avio_tell(pb);
        avio_seek(pb, channel_info_offset, SEEK_SET);
        channel_data_offset = data_offset + avio_rb32(pb);
        channel_dsp_offset  = info_offset + avio_rb32(pb);

        if (c == 0)
            start_offset = channel_data_offset;
        else
            start_offset = FFMIN(start_offset, channel_data_offset);
        avio_seek(pb, channel_dsp_offset, SEEK_SET);
        avio_read(pb, st->codecpar->extradata + 32 * c, 32);
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_wiibns_demuxer = {
    .p.name         = "wiibns",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Wii BNS (Banner Sound)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bns,lbin,bin",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
