/*
 * CSMP demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('C','S','M','P'))
        return 0;

    if (AV_RB32(p->buf+4) != 1)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int64_t offset = 0, duration;
    char name[1025] = { 0 };
    AVIOContext *pb = s->pb;
    int ret, rate;
    AVStream *st;

    avio_skip(pb, 8);

    while (!avio_feof(pb)) {
        uint32_t chunk, size, data = 0;

        chunk = avio_rb32(pb);
        size = avio_rb32(pb);

        switch (chunk) {
        case MKBETAG('D','A','T','A'):
            data = 1;
            offset = avio_tell(pb);
            break;
        case MKBETAG('N','A','M','E'):
            ret = avio_get_str(pb, size, name, sizeof(name));
            size -= ret;

            avio_skip(pb, size);
            break;
        case MKBETAG('I','N','F','O'):
        default:
            avio_skip(pb, size);
            break;
        }

        if (data)
            break;
    }

    duration = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 1024;

    if (name[0])
        av_dict_set(&st->metadata, "name", name, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, offset + 0x1c, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 32)) < 0)
        return ret;

    avio_seek(pb, offset + 0x40, SEEK_SET);

    return 0;
}

const FFInputFormat ff_csmp_demuxer = {
    .p.name         = "csmp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CSMP (Retro Studios Metroid Prime)"),
    .p.extensions   = "csmp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
