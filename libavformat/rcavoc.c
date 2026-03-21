/*
 * RCA VOC demuxer.
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
#include "demux.h"
#include "internal.h"

typedef struct RCAVOCContext {
    int type;
} RCAVOCContext;

static int read_probe(const AVProbeData *p)
{
    int version;

    if (p->buf_size < 27)
        return 0;

    if (memcmp(p->buf + 6, "_VOC_File", 9))
        return 0;
    version = p->buf[26];
    if (version != 0 && version != 3)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    RCAVOCContext *rca = s->priv_data;
    AVIOContext *pb = s->pb;
    int skip;

    avio_skip(pb, 15);
    skip = avio_r8(pb);
    avio_skip(pb, 10);
    rca->type = avio_r8(pb);
    skip = (skip == 12) ? 30 : 254;
    avio_skip(pb, 16 * skip);

    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_RCAVOC;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->profile = rca->type;
    st->codecpar->sample_rate = 8000;
    st->codecpar->bit_rate = (rca->type ? 40LL : 7LL) * st->codecpar->sample_rate * 8 / 80LL;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    RCAVOCContext *rca = s->priv_data;
    const int size = rca->type ? 40 : 7;
    AVIOContext *pb = s->pb;
    int ret, zbeg;

    ret = av_get_packet(pb, pkt, size);
    if (pkt->size != size)
        return AVERROR_INVALIDDATA;

    for (zbeg = size; !pkt->data[zbeg-1] && --zbeg;)
        ;
    if ((size - zbeg) / 20 == 1) {
        avio_seek(pb, -20, SEEK_CUR);
        return ret;
    } else if (!zbeg) {
        return FFERROR_REDO;
    } else {
        return ret;
    }
}

const FFInputFormat ff_rcavoc_demuxer = {
    .p.name         = "rcavoc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RCA VOC"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(RCAVOCContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
