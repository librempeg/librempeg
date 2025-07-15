/*
 * BNVIB format demuxer
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
#include "libavutil/intfloat.h"
#include "libavutil/dict.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int bnvib_probe(const AVProbeData *p)
{
    uint32_t type = AV_RL32(p->buf);

    if (type != 0x4 && type != 0xC && type != 0x10)
        return 0;

    if (p->buf[4] != 3)
        return 0;

    if (AV_RL16(p->buf + 6) == 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int bnvib_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t type, rate;
    AVStream *st;

    type = avio_rl32(pb);
    avio_skip(pb, 2);
    rate = avio_rl16(pb);
    if (rate == 0)
        return AVERROR_INVALIDDATA;

    switch (type) {
    case 0x04:
        avio_skip(pb, 4);
        break;
    case 0x0C:
        avio_skip(pb, 4+4+4);
        break;
    case 0x10:
        avio_skip(pb, 4+4+4+4);
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;
    st->codecpar->ch_layout.nb_channels = 4;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 64;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_bnvib_demuxer = {
    .p.name         = "bnvib",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BNVIB (Binary NX Vibration)"),
    .p.extensions   = "bnvib",
    .read_probe     = bnvib_probe,
    .read_header    = bnvib_read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
