/*
 * RedSpark muxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "avio.h"
#include "mux.h"
#include "internal.h"
#include "avio_internal.h"

#define START_OFFSET 0x1000

typedef struct RedSparkMuxContext {
    int64_t stop_offset;
    int64_t duration;
} RedSparkMuxContext;

static int redspark_write_header(AVFormatContext *s)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    const int coef_off = 0x54 + par->ch_layout.nb_channels * 8;
    AVIOContext *pb = s->pb;

    if (par->block_align != 8 * par->ch_layout.nb_channels) {
        av_log(s, AV_LOG_ERROR, "block_align != %d\n", 8 * par->ch_layout.nb_channels);
        return AVERROR(EINVAL);
    }

    if (par->extradata_size < 32 * par->ch_layout.nb_channels)
        return AVERROR(EINVAL);

    avio_wb32(pb, MKBETAG('R','e','d','S'));
    avio_wb32(pb, MKBETAG('p','a','r','k'));
    ffio_fill(pb, 0, 12 - avio_tell(pb));
    avio_wl32(pb, 0x00010000);
    ffio_fill(pb, 0, 0x18 - avio_tell(pb));
    avio_wl32(pb, START_OFFSET);
    avio_wl16(pb, 0);
    avio_wl16(pb, 1);
    avio_wl32(pb, -1);
    ffio_fill(pb, 0, 0x3c - avio_tell(pb));
    avio_wl32(pb, par->sample_rate);
    avio_wl32(pb, -1);
    avio_wl32(pb, 0);
    avio_wl32(pb, 0);
    avio_wl16(pb, 0);
    avio_wl16(pb, par->ch_layout.nb_channels);

    ffio_fill(pb, 0, coef_off - avio_tell(pb));

    for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        avio_write(pb, par->extradata + ch * 32, 32);
        ffio_fill(pb, 0, 14);
    }

    ffio_fill(pb, 0, START_OFFSET - avio_tell(pb));

    return 0;
}

static int redspark_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    RedSparkMuxContext *r = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_write(pb, pkt->data, pkt->size);
    r->duration += pkt->duration;

    return 0;
}

static int redspark_write_trailer(AVFormatContext *s)
{
    RedSparkMuxContext *r = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        r->stop_offset = avio_tell(pb);

        avio_seek(pb, 0x08, SEEK_SET);
        avio_wl32(pb, r->stop_offset);

        avio_seek(pb, 0x20, SEEK_SET);
        avio_wl32(pb, r->stop_offset);

        avio_seek(pb, 0x40, SEEK_SET);
        avio_wl32(pb, r->duration);
        avio_seek(pb, r->stop_offset, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_redspark_muxer = {
    .p.name           = "redspark",
    .p.long_name      = NULL_IF_CONFIG_SMALL("RedSpark"),
    .p.flags          = AVFMT_TS_NONSTRICT,
    .p.extensions     = "rsd",
    .p.audio_codec    = AV_CODEC_ID_ADPCM_THP_LE,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .priv_data_size   = sizeof(RedSparkMuxContext),
    .write_header     = redspark_write_header,
    .write_packet     = redspark_write_packet,
    .write_trailer    = redspark_write_trailer,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
};
