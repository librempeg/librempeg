/*
 * Sample Vision demuxer
 * Copyright (c) 2024 Paul B Mahol
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

static int smp_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "SOUND SAMPLE DATA ", 18))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int smp_read_header(AVFormatContext *s)
{
    AVCodecParameters *par;
    AVIOContext *pb = s->pb;
    AVStream *st;
    int64_t pos;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 18);
    avio_skip(pb, 4);
    avio_skip(pb, 30);
    avio_skip(pb, 60);

    par = st->codecpar;
    st->start_time = 0;
    st->duration = avio_rl32(pb);
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->ch_layout.nb_channels = 1;
    par->bits_per_coded_sample = 16;
    par->codec_id = AV_CODEC_ID_PCM_S16LE;
    par->block_align = 2;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL)) {
        av_log(s, AV_LOG_ERROR,
               "Cannot determine additional parameters\n");
        return AVERROR_INVALIDDATA;
    }

    pos = avio_tell(pb);
    avio_seek(pb, st->duration*2LL, SEEK_CUR);

    avio_skip(pb, 2);
    for (int i = 0; i < 8; i++)
        avio_skip(pb, 4+4+1+2);

    for (int i = 0; i < 8; i++) {
        avio_skip(pb, 10);
        avio_skip(pb, 4);
    }
    avio_skip(pb, 1);
    par->sample_rate = avio_rl32(pb);
    if (par->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, pos, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);
    return 0;
}

static int smp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int size;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (avio_tell(pb) >= st->duration*2LL)
        return AVERROR_EOF;

    size = FFMIN(st->duration*2LL - avio_tell(pb), 2 * 512);

    return av_get_packet(pb, pkt, size);
}

const FFInputFormat ff_smp_demuxer = {
    .p.name         = "smp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sample Vision Audio"),
    .p.extensions   = "smp",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = smp_probe,
    .read_header    = smp_read_header,
    .read_packet    = smp_read_packet,
};
