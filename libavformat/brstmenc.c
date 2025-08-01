/*
 * BRSTM muxer
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

typedef struct BRSTMMuxContext {
    int nb_channels;

    int64_t stop_offset;
    int64_t duration;

    int num_tracks;
    int track_desc_type;

    int32_t total_blocks;
    int32_t block_size;
    int32_t block_samples;
    int32_t final_block_size;
    int32_t final_block_samples;

    int64_t head_chunk_offset;
    int64_t adpc_chunk_offset;
    int64_t data_chunk_offset;
    uint32_t head_chunk_size;
    uint32_t adpc_chunk_size;
    uint32_t data_chunk_size;

    int64_t head1_offset;
    int64_t head2_offset;
    int64_t head3_offset;

    uint8_t track_volume[8];
    uint8_t track_panning[8];
    uint8_t track_num_channels[8];
    uint8_t track_lchannel_id[8];
    uint8_t track_rchannel_id[8];
    int64_t head2_track_info_offsets[8];
    int64_t head3_ch_info_offsets[16];
} BRSTMMuxContext;

static void brstm_w8(AVIOContext *pb, uint8_t value, const int64_t off)
{
    const int64_t pos = avio_tell(pb);

    avio_seek(pb, off, SEEK_SET);
    avio_w8(pb, value);
    avio_seek(pb, pos+1*(pos==off), SEEK_SET);
}

static void brstm_w16(AVIOContext *pb, uint16_t value, const int64_t off)
{
    const int64_t pos = avio_tell(pb);

    avio_seek(pb, off, SEEK_SET);
    avio_wb16(pb, value);
    avio_seek(pb, pos+2*(pos==off), SEEK_SET);
}

static void brstm_w32(AVIOContext *pb, uint32_t value, const int64_t off)
{
    const int64_t pos = avio_tell(pb);

    avio_seek(pb, off, SEEK_SET);
    avio_wb32(pb, value);
    avio_seek(pb, pos+4*(pos==off), SEEK_SET);
}

static int brstm_write_header(AVFormatContext *s)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    BRSTMMuxContext *r = s->priv_data;
    AVIOContext *pb = s->pb;
    int codec;

    switch (par->codec_id) {
    case AV_CODEC_ID_PCM_S8_PLANAR:
        codec = 0;
        break;
    case AV_CODEC_ID_PCM_S16BE_PLANAR:
        codec = 1;
        break;
    case AV_CODEC_ID_ADPCM_THP:
        codec = 2;
        break;
    default:
        return AVERROR(EINVAL);
    }

    if (par->sample_rate > 65535)
        return AVERROR(EINVAL);

    if (par->ch_layout.nb_channels > 255)
        return AVERROR(EINVAL);
    r->nb_channels = par->ch_layout.nb_channels;

    avio_wb32(pb, MKBETAG('R','S','T','M'));
    avio_wb16(pb, 0xFEFF);
    avio_wb16(pb, 0x0100);
    avio_wb32(pb, -1);
    avio_wb16(pb, 0x40);
    avio_wb16(pb, 2);

    ffio_fill(pb, 0, 12 * 4);

    r->head_chunk_offset = avio_tell(pb);
    avio_wb32(pb, MKBETAG('H','E','A','D'));
    avio_wb32(pb, 0x0);

    for (int i = 0; i < 3; i++) {
        avio_wb32(pb, 0x01000000);
        avio_wb32(pb, 0x00000000);
    }

    r->head1_offset = avio_tell(pb) - r->head_chunk_offset - 8;
    brstm_w32(pb, r->head1_offset, r->head_chunk_offset + 0x0C);

    brstm_w8(pb, codec, avio_tell(pb));
    brstm_w8(pb, 0x0, avio_tell(pb)); // loop flag
    brstm_w8(pb, par->ch_layout.nb_channels, avio_tell(pb));
    brstm_w8(pb, 0x0, avio_tell(pb)); // padding
    brstm_w16(pb, par->sample_rate, avio_tell(pb));
    brstm_w16(pb, 0x0, avio_tell(pb)); // padding
    brstm_w32(pb, 0x0, avio_tell(pb)); // loop start
    brstm_w32(pb, 0x0, avio_tell(pb)); // duration
    brstm_w32(pb, 0x0, avio_tell(pb)); // audio offset
    brstm_w32(pb, 0x0, avio_tell(pb)); // total blocks
    brstm_w32(pb, 0x0, avio_tell(pb)); // block size
    brstm_w32(pb, 0x0, avio_tell(pb)); // block samples
    brstm_w32(pb, 0x0, avio_tell(pb)); // final block size
    brstm_w32(pb, 0x0, avio_tell(pb)); // final block samples
    brstm_w32(pb, 0x0, avio_tell(pb)); // final padded block size
    brstm_w32(pb, 0x0, avio_tell(pb)); // ADPC samples per entry
    brstm_w32(pb, 0x0, avio_tell(pb)); // ADPC bytes per entry

    r->head2_offset = avio_tell(pb) - r->head_chunk_offset - 8;
    brstm_w32(pb, r->head2_offset, r->head_chunk_offset + 0x14);

    brstm_w8(pb, r->num_tracks, avio_tell(pb));
    brstm_w8(pb, r->track_desc_type, avio_tell(pb));
    brstm_w16(pb, 0, avio_tell(pb)); // padding
    // offset table
    for (int i = 0; i < r->num_tracks; i++) {
        brstm_w8(pb, 1, avio_tell(pb));
        brstm_w8(pb, r->track_desc_type, avio_tell(pb));
        brstm_w16(pb, 0x0, avio_tell(pb)); // padding
        brstm_w32(pb, 0x0, avio_tell(pb)); // offset to track description, will be written later from head2_track_info_offsets
    }
    // track descriptions
    for (int i = 0; i < r->num_tracks; i++) {
        // write offset to offset table
        r->head2_track_info_offsets[i] = avio_tell(pb) - r->head_chunk_offset - 8;
        brstm_w32(pb, r->head2_track_info_offsets[i], r->head_chunk_offset + r->head2_offset + 12 + 8*i + 4);
        // write additional type 1 data
        if (r->track_desc_type == 1) {
            brstm_w8(pb, r->track_volume[i], avio_tell(pb));
            brstm_w8(pb, r->track_panning[i], avio_tell(pb));
            ffio_fill(pb, 0, 6); // padding
        }
        // standard data
        brstm_w8(pb, r->track_num_channels[i], avio_tell(pb));
        brstm_w8(pb, r->track_lchannel_id [i], avio_tell(pb));
        brstm_w8(pb, r->track_rchannel_id [i], avio_tell(pb));
        brstm_w8(pb, 0, avio_tell(pb)); // padding
    }

    r->head3_offset = avio_tell(pb) - r->head_chunk_offset - 8;
    brstm_w32(pb, r->head3_offset, r->head_chunk_offset + 0x1C);
    // head3 header
    brstm_w8(pb, par->ch_layout.nb_channels, avio_tell(pb));
    ffio_fill(pb, 0, 3);
    // offset table
    for (int i = 0; i < par->ch_layout.nb_channels; i++) {
        brstm_w32(pb, 0x01000000, avio_tell(pb)); // marker
        brstm_w32(pb, 0x0, avio_tell(pb)); // offset to channel information, will be written later from head3_ch_info_offsets
    }
    // channel info
    for (int i = 0; i < par->ch_layout.nb_channels; i++) {
        // write offset to offset table
        r->head3_ch_info_offsets[i] = avio_tell(pb) - r->head_chunk_offset - 8;
        brstm_w32(pb, r->head3_ch_info_offsets[i], r->head_chunk_offset + r->head3_offset + 12 + 8*i + 4);
        // write channel info
        brstm_w32(pb, 0x01000000, avio_tell(pb)); // marker
        if (codec == 2) {
            // this information exists only in ADPCM files
            brstm_w32(pb, avio_tell(pb) - r->head_chunk_offset - 4, avio_tell(pb)); // offset to ADPCM coefs?
            // write coefs
            avio_write(pb, par->extradata, 32 * par->ch_layout.nb_channels);

            brstm_w16(pb, 0x0, avio_tell(pb)); // Gain, always zero
            brstm_w16(pb, 0x0, avio_tell(pb)); // Initial scale, will be written later
            brstm_w16(pb, 0x0, avio_tell(pb)); // HS1, always zero
            brstm_w16(pb, 0x0, avio_tell(pb)); // HS2, always zero
            brstm_w16(pb, 0x0, avio_tell(pb)); // Loop predictor scale, will be written later
            brstm_w16(pb, 0x0, avio_tell(pb)); // Loop HS1
            brstm_w16(pb, 0x0, avio_tell(pb)); // Loop HS2
            brstm_w16(pb, 0x0, avio_tell(pb)); // padding
        } else {
            brstm_w32(pb, 0x0, avio_tell(pb)); // padding / empty offset to ADPCM coefs for PCM files
        }
    }

    r->head_chunk_size = avio_tell(pb) - r->head_chunk_offset;
    ffio_fill(pb, 0, 6); // padding
    while (avio_tell(pb) & 31) {
        brstm_w8(pb, 0, avio_tell(pb));
        r->head_chunk_size = avio_tell(pb) - r->head_chunk_offset;
    }
    // write head chunk length
    brstm_w32(pb, r->head_chunk_size, r->head_chunk_offset + 4);

    r->data_chunk_offset = avio_tell(pb);
    avio_wb32(pb, MKBETAG('D','A','T','A'));
    avio_wb32(pb, 0x0);

    return 0;
}

static int brstm_write_packet(AVFormatContext *s, AVPacket *avpkt)
{
    BRSTMMuxContext *r = s->priv_data;
    AVIOContext *pb = s->pb;

    avio_write(pb, avpkt->data, avpkt->size);
    r->duration += avpkt->duration;
    r->total_blocks++;

    if (!r->block_size)
        r->block_size = avpkt->size;
    if (!r->block_samples)
        r->block_samples = avpkt->duration;

    r->final_block_size = avpkt->size;
    r->final_block_samples = avpkt->duration;

    return 0;
}

static int brstm_write_trailer(AVFormatContext *s)
{
    BRSTMMuxContext *r = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        r->stop_offset = avio_tell(pb);
        r->data_chunk_size = r->stop_offset - r->data_chunk_offset;

        avio_seek(pb, 0x08, SEEK_SET);
        avio_wb32(pb, r->stop_offset);

        avio_seek(pb, 0x10, SEEK_SET);
        avio_wb32(pb, r->head_chunk_offset);
        avio_wb32(pb, r->head_chunk_size);

        avio_seek(pb, 0x18, SEEK_SET);
        avio_wb32(pb, r->adpc_chunk_offset);
        avio_wb32(pb, r->adpc_chunk_size);

        avio_seek(pb, 0x20, SEEK_SET);
        avio_wb32(pb, r->data_chunk_offset);
        avio_wb32(pb, r->data_chunk_size);

        avio_seek(pb, r->data_chunk_offset + 4, SEEK_SET);
        avio_wb32(pb, r->data_chunk_size);

        avio_seek(pb, r->head_chunk_offset + 44, SEEK_SET);
        avio_wb32(pb, r->duration);
        avio_wb32(pb, r->data_chunk_offset + 8);
        avio_wb32(pb, r->total_blocks);
        avio_wb32(pb, r->block_size / r->nb_channels);
        avio_wb32(pb, r->block_samples);
        avio_wb32(pb, r->final_block_size / r->nb_channels);
        avio_wb32(pb, r->final_block_samples);
        avio_wb32(pb, r->final_block_size);
    }

    return 0;
}

const FFOutputFormat ff_brstm_muxer = {
    .p.name           = "brstm",
    .p.long_name      = NULL_IF_CONFIG_SMALL("BRSTM (Binary Revolution Stream)"),
    .p.flags          = AVFMT_TS_NONSTRICT,
    .p.extensions     = "brstm",
    .p.audio_codec    = AV_CODEC_ID_PCM_S16BE_PLANAR,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .priv_data_size   = sizeof(BRSTMMuxContext),
    .write_header     = brstm_write_header,
    .write_packet     = brstm_write_packet,
    .write_trailer    = brstm_write_trailer,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH,
};
