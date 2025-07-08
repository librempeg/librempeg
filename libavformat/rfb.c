/*
 * RFB .rfb demuxer
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
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

static int rfb_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    uint32_t skip;
    AVStream *st;
    int ret;

    avio_skip(pb, 32);
    avio_skip(pb, 16);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    par = st->codecpar;
    par->codec_type = AVMEDIA_TYPE_VIDEO;
    par->codec_id = AV_CODEC_ID_RFB;

    if ((ret = ff_get_extradata(s, par, pb, 20)) < 0)
        return ret;

    par->width = AV_RB16(par->extradata);
    par->height = AV_RB16(par->extradata+2);

    skip = avio_rb32(pb);
    if (avio_rb32(pb) == 0)
        skip = avio_rb32(pb);
    else
        skip -= 4;
    avio_skip(pb, skip);

    avpriv_set_pts_info(st, 64, 1, 1000);

    return 0;
}

static int rfb_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int64_t size, padding;
    int64_t pts;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pts = avio_rb32(pb);
    size = avio_rb32(pb);
    padding = FFALIGN(size, 4) - size;
    ret = av_get_packet(pb, pkt, size);
    if (ret < 0)
        return ret;
    if (padding > 0)
        avio_skip(pb, padding);
    pkt->pos = pos;
    pkt->pts = pts;
    pkt->stream_index = 0;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

static int rfb_probe(const AVProbeData *pd)
{
    if (memcmp(pd->buf, "FBS 001.", 8))
        return 0;
    if (memcmp(pd->buf+16, "RFB 003.", 8))
        return 0;
    return AVPROBE_SCORE_MAX;
}

const FFInputFormat ff_rfb_demuxer = {
    .p.name         = "rfb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RFB"),
    .p.extensions   = "rfb",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = rfb_probe,
    .read_header    = rfb_read_header,
    .read_packet    = rfb_read_packet,
};
