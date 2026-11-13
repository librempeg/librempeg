/*
 * WIIADPCM demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct WIIADPCMContext {
    int block_size;
    int nb_channels;
} WIIADPCMContext;

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "WIIADPCM", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int zero, ret, rate, offset, nb_channels = 1, block_size = 0;
    WIIADPCMContext *wii = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t coeff_offset;

    avio_skip(pb, 8);
    offset = avio_rb32(pb);
    if (offset <= 0)
        return AVERROR_INVALIDDATA;

    while (!avio_feof(pb)) {
        int next_offset = avio_rb32(pb);

        if (nb_channels >= INT_MAX)
            return AVERROR_INVALIDDATA;

        if (next_offset > 0) {
            block_size = FFABS(next_offset - offset);
            nb_channels++;

            if (next_offset < offset) {
                wii->block_size = next_offset;
                break;
            }

            offset = next_offset;
        } else if (next_offset < 0) {
            return AVERROR_INVALIDDATA;
        } else if (block_size > 0) {
            wii->block_size = block_size;
            nb_channels++;
            break;
        }
    }

    do {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        zero = avio_rb32(pb) == 0;
    } while (zero);

    avio_seek(pb, -4, SEEK_CUR);

    coeff_offset = avio_tell(pb);

    if (wii->block_size <= 0 || wii->block_size >= INT_MAX/nb_channels)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 8);

    wii->nb_channels = nb_channels;

    rate = avio_rb32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 0;
    st->codecpar->bit_rate = 8LL * nb_channels * 8 * rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, 32 * nb_channels);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < nb_channels; ch++) {
        avio_seek(pb, coeff_offset + ch * wii->block_size + 0x1c, SEEK_SET);

        ret = avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
        if (ret < 0)
            return ret;
    }

    avio_seek(pb, coeff_offset + 0x60, SEEK_SET);
    if (avio_tell(pb) >= wii->block_size)
        return AVERROR_INVALIDDATA;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    WIIADPCMContext *wii = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (pos < wii->block_size * wii->nb_channels) {
        int64_t offset = ffformatcontext(s)->data_offset;

        ret = av_new_packet(pkt, (wii->block_size - offset) * wii->nb_channels);
        if (ret < 0)
            return ret;

        for (int ch = 0; ch < wii->nb_channels; ch++) {
            if ((avio_tell(pb) % wii->block_size) == 0)
                avio_skip(pb, offset);

            ret = ffio_read_size(pb, pkt->data + (wii->block_size - offset) * ch, wii->block_size - offset);
            if (ret < 0)
                return ret;
        }
    } else {
        ret = av_get_packet(pb, pkt, wii->block_size * wii->nb_channels);
        if (ret < 0)
            return ret;
    }

    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_wiiadpcm_demuxer = {
    .p.name         = "wiiadpcm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Exient WiiADPCM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "apdcm",
    .priv_data_size = sizeof(WIIADPCMContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
