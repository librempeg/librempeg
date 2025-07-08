/*
 * ILDA Image Data Transfer Format demuxer
 * Copyright (c) 2020 Peter Ross (pross@xvid.org)
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

/**
 * @file
 * ILDA Image Data Transfer Format demuxer
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct ILDAContext {
    AVClass *class;
    int width, height;
    AVRational framerate;
} ILDAContext;

static const uint8_t record_size[] = { 8, 6, 3, 3, 10, 8 };

static int read_probe(const AVProbeData *p)
{
    int score = 0;

    for (int i = 0; i+32 < p->buf_size;) {
        int nb_entries, type;

        type = p->buf[i+7];
        if (memcmp(p->buf+i, "ILDA", 4) ||
            p->buf[i+4] || p->buf[i+5] || p->buf[i+6] ||
            type >= FF_ARRAY_ELEMS(record_size))
            break;
        nb_entries = AV_RB16(p->buf + i+24);
        score += 3;
        if (score >= AVPROBE_SCORE_MAX)
            break;
        i += 32 + record_size[type] * nb_entries;
    }

    return FFMIN(AVPROBE_SCORE_MAX, score);
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

    if (AV_RB32(buf) != MKBETAG('I', 'L', 'D', 'A'))
        return AVERROR_INVALIDDATA;

    type = buf[7];
    if (type >= FF_ARRAY_ELEMS(record_size))
        return AVERROR_INVALIDDATA;

    nb_entries = AV_RB16(buf + 24);
    if (!nb_entries)
        return AVERROR_EOF;

    ret = av_new_packet(pkt, sizeof(buf) + nb_entries * record_size[type]);
    if (ret < 0)
        return ret;

    memcpy(pkt->data, buf, sizeof(buf));

    ret = avio_read(pb, pkt->data + sizeof(buf), nb_entries * record_size[type]);
    if (ret < 0)
       return ret;

    pkt->pos = pos;
    pkt->pts = AV_RB16(buf + 26);
    pkt->duration = 1;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

#define OFFSET(x) offsetof(ILDAContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "video_size", "set frame size", OFFSET(width), AV_OPT_TYPE_IMAGE_SIZE, {.str = NULL}, 0, 0, DEC },
    { "framerate", "set frame rate", OFFSET(framerate), AV_OPT_TYPE_VIDEO_RATE, {.str = "10"}, 0, INT_MAX, DEC },
    { NULL },
};

static const AVClass ilda_demuxer_class = {
    .class_name     = "ILDA demuxer",
    .item_name      = av_default_item_name,
    .option         = options,
    .version        = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_ilda_demuxer = {
    .p.name         = "ilda",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ILDA Image Data Transfer Format"),
    .p.priv_class   = &ilda_demuxer_class,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ild",
    .priv_data_size = sizeof(ILDAContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
