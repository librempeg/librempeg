/*
 * IDSP muxer
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

typedef struct IDSPMuxContext {
    int64_t duration;
} IDSPMuxContext;

static int idsp_write_header(AVFormatContext *s)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;

    if (par->extradata_size < 32 * par->ch_layout.nb_channels)
        return AVERROR(EINVAL);

    avio_wb32(pb, MKBETAG('I','D','S','P'));
    avio_wb32(pb, 0xFFFFFFFF);
    avio_wb32(pb, par->sample_rate);
    avio_wb32(pb, par->ch_layout.nb_channels);
    avio_wb32(pb, par->block_align / par->ch_layout.nb_channels);

    for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        const int16_t *src = (const int16_t *)(par->extradata + 32*ch);

        for (int n = 0; n < 16; n++)
            avio_wb16(pb, src[n]);
        ffio_fill(pb, 0, 14);
    }

    ffio_fill(pb, 0, 0x70 - avio_tell(pb));

    return 0;
}

static int idsp_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    IDSPMuxContext *i = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_write(pb, pkt->data, pkt->size);
    i->duration += pkt->duration;

    return 0;
}

static int idsp_write_trailer(AVFormatContext *s)
{
    IDSPMuxContext *i = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        int64_t file_size = avio_tell(pb);

        avio_seek(pb, 4, SEEK_SET);
        avio_wb32(pb, i->duration);
        avio_seek(pb, file_size, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_idsp_muxer = {
    .p.name           = "idsp",
    .p.long_name      = NULL_IF_CONFIG_SMALL("IDSP (Inevitable Entertainment)"),
    .p.flags          = AVFMT_TS_NONSTRICT,
    .p.extensions     = "idsp",
    .p.audio_codec    = AV_CODEC_ID_ADPCM_NDSP_LE,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .priv_data_size   = sizeof(IDSPMuxContext),
    .write_header     = idsp_write_header,
    .write_packet     = idsp_write_packet,
    .write_trailer    = idsp_write_trailer,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
};
