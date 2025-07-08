/*
 * TAC demuxer
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

#include "libavutil/avassert.h"
#include "libavutil/intreadwrite.h"
#include "avio_internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "rawdec.h"

#define BLOCK_SIZE 0x4E000

static int tac_probe(const AVProbeData *p)
{
    uint32_t offset = AV_RL32(p->buf), start = 256;
    int score = 0;

    if (AV_RL16(p->buf+0xc) < 1)
        return 0;

    if (offset > (BLOCK_SIZE-256) ||
        offset < 32)
        return 0;

    for (int i = offset; i < offset + 256 && i < p->buf_size; i++) {
        if (p->buf[i] & 0x80)
            start++;
    }

    offset += start;
    for (int i = offset; i + 4 < p->buf_size;) {
        if (AV_RL32(p->buf+i) == 0xFFFFFFFF) {
            score += 30;
            i += BLOCK_SIZE - (i % BLOCK_SIZE);
        } else {
            if ((AV_RL16(p->buf+i+2) & 0x7FFF) <= 8) {
                score = 0;
                break;
            }
            i += (AV_RL16(p->buf+i+2) & 0x7FFF) + 8;
            score++;
        }
    }

    return FFMIN(AVPROBE_SCORE_MAX, score);
}

static int tac_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int ret, extra = 0;
    uint32_t offset;
    AVStream *st;

    if ((ret = ffio_ensure_seekback(pb, BLOCK_SIZE)) < 0)
        return ret;

    offset = avio_rl32(pb);
    if (offset > BLOCK_SIZE-256)
        return AVERROR_INVALIDDATA;
    avio_seek(pb, offset, SEEK_SET);

    for (int i = 0; i < 256; i++)
        extra += !!(avio_r8(pb) & 0x80);

    if (offset + 256+extra > BLOCK_SIZE)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    par = st->codecpar;

    if ((ret = ff_alloc_extradata(par, 32+256+extra)) < 0)
        return ret;

    avio_seek(pb, 0, SEEK_SET);
    avio_read(pb, par->extradata, 32);
    avio_seek(pb, offset, SEEK_SET);
    avio_read(pb, par->extradata+32, 256+extra);

    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id = AV_CODEC_ID_TAC;
    par->ch_layout.nb_channels = 2;
    st->start_time = 0;
    st->duration = (AV_RL16(par->extradata+0xC)- 1) * 1024 + (AV_RL16(par->extradata+0xE) + 1);
    par->sample_rate = 48000;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_tac_demuxer = {
    .p.name         = "tac",
    .p.long_name    = NULL_IF_CONFIG_SMALL("tri-Ace PS2"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &ff_raw_demuxer_class,
    .priv_data_size = sizeof(FFRawDemuxerContext),
    .read_probe     = tac_probe,
    .read_header    = tac_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_TAC,
};
