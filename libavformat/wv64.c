/*
 * WAV64 demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 5)
        return 0;

    if (memcmp(p->buf, "WV64\x02", 5))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int align, bits, rate, channels, codec;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    avio_skip(pb, 5);
    codec = avio_r8(pb);
    channels = avio_r8(pb);
    bits = avio_r8(pb);
    rate = avio_rb32(pb);
    duration = avio_rb32(pb);
    avio_skip(pb, 4);
    start = avio_rb32(pb);
    if (rate <= 0 || channels <= 0 || bits <= 0)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case 0:
    case 2:
        codec = (bits > 8) ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S8;
        align = (bits+7)/8;
        break;
    case 1:
        codec = AV_CODEC_ID_ADPCM_WV64;
        align = 9;
        break;
    case 3:
        codec = AV_CODEC_ID_OPUS;
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;

    if (codec == AV_CODEC_ID_OPUS)
        ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;
    if (codec == AV_CODEC_ID_ADPCM_WV64) {
        int64_t size = start - avio_tell(pb);
        int ret;

        if (size <= 0 || size > INT16_MAX)
            return AVERROR_INVALIDDATA;

        if ((ret = ff_get_extradata(s, st->codecpar, pb, size)) < 0)
            return ret;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;

    if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
        int size, ret;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        size = avio_rb16(pb);
        ret = av_get_packet(pb, pkt, size);

        if (avio_tell(pb) & 1)
            avio_skip(pb, 1);

        pkt->pos = pos;

        return ret;
    } else {
        return ff_pcm_read_packet(s, pkt);
    }
}

const FFInputFormat ff_wv64_demuxer = {
    .p.name         = "wv64",
    .p.long_name    = NULL_IF_CONFIG_SMALL("WAV64"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wav64",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
