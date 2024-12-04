/*
 * ILDA Image Data Transfer Format demuxer
 * Copyright (c) 2020 Peter Ross (pross@xvid.org)
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * ILDA Image Data Transfer Format demuxer
 */

#include "libavutil/intreadwrite.h"
#include "libavcodec/bytestream.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "libavutil/opt.h"

typedef struct {
    AVClass *class;
    int width, height;
    AVRational framerate;
} ILDAContext;

static const uint8_t record_size[] = {8, 6, 3, 3, 10, 8};

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 8 || memcmp(p->buf, "ILDA", 4) || p->buf[4] || p->buf[5] || p->buf[6] || p->buf[7] >= FF_ARRAY_ELEMS(record_size))
        return 0;
    return AVPROBE_SCORE_MAX / 4 + 1;
}

static int read_header(AVFormatContext *avctx)
{
    ILDAContext * s = avctx->priv_data;
    AVStream *st = avformat_new_stream(avctx, NULL);

    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_ILDA;
    st->codecpar->width      = s->width;
    st->codecpar->height     = s->height;
    avpriv_set_pts_info(st, 64, s->framerate.den, s->framerate.num);
    st->avg_frame_rate = s->framerate;
    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    uint8_t buf[32];
    int type, nb_entries, ret;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);

    if (avio_read(pb, buf, sizeof(buf)) != sizeof(buf))
        return AVERROR(EIO);

    if (AV_RB32(buf) != MKBETAG('I', 'L', 'D', 'A')) {
        av_log(s, AV_LOG_ERROR, "unexpected tag\n");
        return AVERROR(EIO);
    }

    type = buf[7];
    if (type >= FF_ARRAY_ELEMS(record_size))
        return AVERROR(EIO);

    nb_entries = AV_RB16(buf + 24);
    if (!nb_entries) {
        avio_seek(pb, -sizeof(buf), SEEK_CUR);
        return AVERROR(EOF);
    }

    ret = av_new_packet(pkt, sizeof(buf) + nb_entries * record_size[type]);
    if (ret < 0)
        return ret;

    memcpy(pkt->data, buf, sizeof(buf));

    ret = avio_read(pb, pkt->data + sizeof(buf), nb_entries * record_size[type]);
    if (ret < 0)
       return ret;

    pkt->pos = pos;
    pkt->pts = AV_RB16(buf + 26);
    pkt->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

#define OFFSET(x) offsetof(ILDAContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "video_size", "A string describing frame size, such as 640x480 or hd720.", OFFSET(width), AV_OPT_TYPE_IMAGE_SIZE, {.str = NULL}, 0, 0, DEC },
    { "framerate", "", OFFSET(framerate), AV_OPT_TYPE_VIDEO_RATE, {.str = "10"}, 0, INT_MAX, DEC },
    { NULL },
};

static const AVClass ilda_demuxer_class = {
    .class_name     = "ILDA demuxer",
    .item_name      = av_default_item_name,
    .option         = options,
    .version        = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_ilda_demuxer = {
    .p.name         = "ildd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ILDA Image Data Transfer Format"),
    .p.priv_class   = &ilda_demuxer_class,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(ILDAContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
