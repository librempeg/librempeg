/*
 * Chronomaster DFA Format Demuxer
 * Copyright (c) 2011 Konstantin Shishkov
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

#include <inttypes.h>

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int dfa_probe(const AVProbeData *p)
{
    if (p->buf_size < 4 || AV_RL32(p->buf) != MKTAG('D', 'F', 'I', 'A'))
        return 0;

    if (AV_RL32(p->buf + 16) != 0x80)
        return AVPROBE_SCORE_MAX / 4;

    return AVPROBE_SCORE_MAX;
}

static int dfa_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int frames, ret;
    int version;
    uint32_t mspf;

    if (avio_rl32(pb) != MKTAG('D', 'F', 'I', 'A')) {
        av_log(s, AV_LOG_ERROR, "Invalid magic for DFA\n");
        return AVERROR_INVALIDDATA;
    }

    version = avio_rl16(pb);
    frames = avio_rl16(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_DFA;
    st->codecpar->width      = avio_rl16(pb);
    st->codecpar->height     = avio_rl16(pb);
    mspf = avio_rl32(pb);
    if (!mspf) {
        av_log(s, AV_LOG_WARNING, "Zero FPS reported, defaulting to 10\n");
        mspf = 100;
    }
    avpriv_set_pts_info(st, 24, mspf, 1000);
    avio_skip(pb, 128 - 16); // padding
    st->duration = frames;

    if ((ret = ff_alloc_extradata(st->codecpar, 2)) < 0)
        return ret;
    AV_WL16(st->codecpar->extradata, version);
    if (version == 0x100)
        st->sample_aspect_ratio = (AVRational){2, 1};

    return 0;
}

static int dfa_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    uint32_t frame_size;
    int ret, first = 1;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (av_get_packet(pb, pkt, 12) != 12)
        return AVERROR(EIO);
    while (!avio_feof(pb)) {
        if (!first) {
            ret = av_append_packet(pb, pkt, 12);
            if (ret < 0) {
                return ret;
            }
        } else
            first = 0;
        frame_size = AV_RL32(pkt->data + pkt->size - 8);
        if (frame_size > INT_MAX - 4) {
            av_log(s, AV_LOG_ERROR, "Too large chunk size: %"PRIu32"\n", frame_size);
            return AVERROR(EIO);
        }
        if (AV_RL32(pkt->data + pkt->size - 12) == MKTAG('E', 'O', 'F', 'R')) {
            if (frame_size) {
                av_log(s, AV_LOG_WARNING,
                       "skipping %"PRIu32" bytes of end-of-frame marker chunk\n",
                       frame_size);
                avio_skip(pb, frame_size);
            }
            return 0;
        }
        ret = av_append_packet(pb, pkt, frame_size);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

const FFInputFormat ff_dfa_demuxer = {
    .p.name         = "dfa",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Chronomaster DFA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = dfa_probe,
    .read_header    = dfa_read_header,
    .read_packet    = dfa_read_packet,
};
