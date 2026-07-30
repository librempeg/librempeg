/*
 * PSXSTR muxer
 * Copyright (c) 2026 Paul B Mahol
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
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "libavcodec/packet_internal.h"
#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"

#define STR_SECTOR_SIZE 2352

typedef struct PSXSTRMuxContext {
    AVClass *class;

    unsigned sector_counter;

    uint8_t submode;
    uint8_t coding;

    uint8_t sector[STR_SECTOR_SIZE];
} PSXSTRMuxContext;

static int write_header(AVFormatContext *ctx)
{
    AVCodecParameters *par = ctx->streams[0]->codecpar;
    PSXSTRMuxContext *s = ctx->priv_data;

    s->sector_counter = 150;

    if (par->ch_layout.nb_channels < 1 ||
        par->ch_layout.nb_channels > 2)
        return AVERROR(EINVAL);

    if (par->sample_rate != 18900 &&
        par->sample_rate != 37800)
        return AVERROR(EINVAL);

    s->submode = (1<<2) | (1<<5) | (1<<6);
    s->coding = (par->ch_layout.nb_channels == 2) ? 1 : 0;
    if (par->sample_rate == 18900)
        s->coding |= (1<<2);

    return 0;
}

static uint32_t edc_crc32(const uint8_t *src, const int N)
{
    const uint32_t poly = 0xD8018001U;
    uint32_t edc = 0;

    for (int n = 0; n < N; n++) {
        edc ^= 0xFF & src[n];

        for (int j = 0; j < 8; j++)
            edc = (edc >> 1) ^ (poly * (edc & 0x1));
    }

    return edc;
}

#define BCD(x) ((x) + ((x) / 10) * 6)

static int write_packet(AVFormatContext *ctx, AVPacket *pkt)
{
    PSXSTRMuxContext *s = ctx->priv_data;
    AVIOContext *pb = ctx->pb;
    uint32_t crc;

    if (pkt->size != 2304)
        return AVERROR(EINVAL);

    s->sector[0]  = 0;
    memset(s->sector+1, 0xFF, 10);
    s->sector[11] = 0;

    s->sector[12] = BCD(s->sector_counter / 4500);
    s->sector[13] = BCD((s->sector_counter / 75) % 60);
    s->sector[14] = BCD(s->sector_counter % 75);
    s->sector[15] = s->sector[15] = 2;

    s->sector[16] = s->sector[20] = 1;
    s->sector[17] = s->sector[21] = 0;
    s->sector[18] = s->sector[22] = s->submode;
    s->sector[19] = s->sector[23] = s->coding;

    memcpy(s->sector + 24, pkt->data, pkt->size);
    crc = edc_crc32(s->sector + 16, 2332);
    AV_WL32(s->sector + 2348, crc);

    avio_write(pb, s->sector, sizeof(s->sector));

    s->sector_counter += 8;

    return 0;
}

const FFOutputFormat ff_str_muxer = {
    .p.name           = "psxstr",
    .p.long_name      = NULL_IF_CONFIG_SMALL("Sony Playstation STR"),
    .priv_data_size   = sizeof(PSXSTRMuxContext),
    .p.audio_codec    = AV_CODEC_ID_ADPCM_XA,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
    .write_header     = write_header,
    .write_packet     = write_packet,
};
