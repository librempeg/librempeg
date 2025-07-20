/*
 * BNVIB muxer
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
#include "mux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct BNVIBMuxContext {
    int64_t size;
} BNVIBMuxContext;

static int bnvib_write_header(AVFormatContext *s)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;

    if (par->ch_layout.nb_channels != 4 ||
        par->sample_rate > UINT16_MAX) {
        av_log(s, AV_LOG_ERROR, "Number of channels not 4 or sample rate > %d\n", UINT16_MAX);
        return AVERROR(EINVAL);
    }

    avio_wl32(pb, 0x4);
    avio_w8(pb, 3);
    avio_w8(pb, 0);
    avio_wl16(pb, par->sample_rate);
    avio_wl32(pb, -1);

    return 0;
}

static int bnvib_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    BNVIBMuxContext *b = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_write(pb, pkt->data, pkt->size);
    b->size += pkt->size;

    return 0;
}

static int bnvib_write_trailer(AVFormatContext *s)
{
    BNVIBMuxContext *b = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        int64_t file_size = avio_tell(pb);

        avio_seek(pb, 8, SEEK_SET);
        avio_wl32(pb, b->size);
        avio_seek(pb, file_size, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_bnvib_muxer = {
    .p.name           = "bnvib",
    .p.long_name      = NULL_IF_CONFIG_SMALL("BNVIB (Binary NX Vibration)"),
    .p.flags          = AVFMT_TS_NONSTRICT | AVFMT_NOTIMESTAMPS,
    .p.extensions     = "bnvib",
    .p.audio_codec    = AV_CODEC_ID_PCM_U8,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .priv_data_size   = sizeof(BNVIBMuxContext),
    .write_header     = bnvib_write_header,
    .write_packet     = bnvib_write_packet,
    .write_trailer    = bnvib_write_trailer,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
};
