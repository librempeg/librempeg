/*
 * OOR demuxer
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
#include "libavcodec/bytestream.h"
#include "libavcodec/get_bits.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct VorbisCustom {
    int current_packet;
    int packet_count;
    uint16_t packet_size[256];
    uint8_t flags;
} VorbisCustom;

typedef struct OORContext {
    VorbisCustom data;
} OORContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 16)
        return 0;

    if (p->buf[0] != 8 && AV_RB32(p->buf) != 0x48000000)
        return 0;

    static const uint8_t empty_granule[9] = { 0 };
    int head_pos, page_version;
    if (p->buf[0] == 0x48 && !memcmp(p->buf+1, empty_granule, 9)) {
        head_pos = 10;
        page_version = 1;
    } else if (p->buf[0] == 8) {
        head_pos = 1;
        page_version = 0;
    } else {
        return 0;
    }

    uint16_t head = AV_RB16(p->buf+head_pos);
    int version = (head >> 14) & 0x03;
    int channels = (head >> 11) & 0x07;
    int sr_selector = (head >> 9) & 0x03;
    int srate = (head >> 1) & 0xFF;

    if (version != page_version || channels == 0)
        return 0;

    if (sr_selector == 3 && srate > 10)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct OORPage {
    uint8_t version;
    uint8_t flags;
    int64_t granule;
    int padding1;
    int padding2;
} OORPage;

static void oor_read_page(GetBitContext *gb, OORPage *page)
{
    uint32_t granule_hi = 0, granule_lo = 0;

    page->version = get_bits(gb, 2);
    page->flags   = get_bits(gb, 4);

    switch (page->version) {
    case 0:
        page->granule  = 0;
        page->padding1 = 0;
        page->padding2 = 0;
        break;
    case 1:
        page->padding1  = get_bits(gb, 2);
        granule_hi      = get_bits_long(gb, 32);
        granule_lo      = get_bits_long(gb, 32);
        page->granule   = ((uint64_t)granule_hi << 32) | granule_lo;
        page->padding2  = get_bits(gb, 6);
        break;
    default:
        break;
    }
}

typedef struct OORHeader {
    uint8_t pre_padding;
    uint8_t version;
    uint8_t channels;
    int sample_rate;
    uint8_t unknown1;
    uint8_t unknown2;
    uint8_t unknown3;
    int64_t last_granule;
    uint8_t blocksize0_exp;
    uint8_t blocksize1_exp;
    uint8_t framing;
    uint8_t post_padding;
} OORHeader;

static void oor_read_header(GetBitContext *gb, OORHeader *hdr)
{
    hdr->pre_padding = get_bits(gb, 2);
    hdr->version = get_bits(gb, 2);
    hdr->channels = get_bits(gb, 3);
    int sr_selector = get_bits(gb, 2);

    if (sr_selector == 3) {
        int sr_index = get_bits(gb, 8);

        switch (hdr->version) {
        case 0:
            switch(sr_index) {
            case 3: hdr->sample_rate = 32000; break;
            case 4: hdr->sample_rate = 48000; break;
            case 5: hdr->sample_rate = 96000; break;
            default: hdr->sample_rate = 0; break;
            }
            break;
        case 1:
            switch(sr_index) {
            case 4: hdr->sample_rate = 32000; break;
            case 5: hdr->sample_rate = 48000; break;
            case 6: hdr->sample_rate = 64000; break;
            case 7: hdr->sample_rate = 88200; break;
            case 8: hdr->sample_rate = 96000; break;
            default: hdr->sample_rate = 0; break;
            }
            break;
        default:
            break;
        }

        if (hdr->sample_rate == 0)
            hdr->sample_rate = 8000;
    } else {
        hdr->sample_rate = 11025 * (1 << sr_selector);
    }

    if (hdr->version == 1) {
        uint32_t granule_hi, granule_lo;

        hdr->unknown1 = get_bits(gb, 1);
        hdr->unknown2 = get_bits(gb, 1);
        hdr->unknown3 = get_bits(gb, 7);

        granule_hi = get_bits_long(gb, 32);
        granule_lo = get_bits_long(gb, 32);
        hdr->last_granule = ((uint64_t)granule_hi << 32) | granule_lo;
    }

    hdr->blocksize1_exp = get_bits(gb, 4);
    hdr->blocksize0_exp = get_bits(gb, 4);
    hdr->framing = get_bits(gb, 1);

    align_get_bits(gb);
}

typedef struct OORSetup {
    int type;
    int codebook_id;
} OORSetup;

static void oor_read_setup(GetBitContext *gb, OORSetup *setup)
{
    setup->type = get_bits(gb, 2);
    setup->codebook_id = get_bits(gb, 6);
}

static int oor_validate_header_page(OORPage *page, OORHeader *hdr)
{
    if (page->version > 1 || page->flags != 0x02)
        return AVERROR_INVALIDDATA;

    if (page->granule != 0 || page->padding1 != 0 || page->padding2 != 0)
        return AVERROR_INVALIDDATA;

    if (hdr->pre_padding != 0 || hdr->version > 1 || hdr->version != page->version)
        return AVERROR_INVALIDDATA;
    if (hdr->channels == 0 || hdr->sample_rate == 0)
        return AVERROR_INVALIDDATA;

    if (hdr->version == 1 && hdr->last_granule == 0)
        return AVERROR_INVALIDDATA;

    if (hdr->blocksize0_exp < 6 || hdr->blocksize0_exp > 13 || hdr->blocksize1_exp < 6 || hdr->blocksize1_exp > 13)
        return AVERROR_INVALIDDATA;
    if (hdr->framing != 1)
        return AVERROR_INVALIDDATA;

    if (hdr->post_padding != 0)
        return AVERROR_INVALIDDATA;

    return 0;
}

#define OOR_FLAG_EOS     (1<<0)
#define OOR_FLAG_BOS     (1<<1)
#define OOR_FLAG_PARTIAL (1<<2)

typedef struct OORSize {
    uint8_t vps_bits;
    uint8_t padding1;
    uint8_t packet_count;
    uint8_t bps_selector;
    int16_t base_packet_size;
    int16_t variable_packet_size[256];
    uint8_t post_padding;
} OORSize;

static void oor_read_size(GetBitContext *gb, OORSize *size)
{
    size->vps_bits = get_bits(gb, 4);
    size->padding1 = get_bits(gb, 1);
    size->packet_count = get_bits(gb, 8);
    size->bps_selector = get_bits(gb, 2);

    switch (size->bps_selector) {
    case 0:
        size->base_packet_size = 0;
        break;
    case 1:
        size->base_packet_size = get_bits(gb, 8);
        break;
    case 2:
        size->base_packet_size = get_bits(gb, 11);
        break;
    case 3:
    default:
        size->base_packet_size = 0;
        break;
    }

    for (int i = 0; i < size->packet_count; i++) {
        if (size->vps_bits)
            size->variable_packet_size[i] = get_bits(gb, size->vps_bits);
        else
            size->variable_packet_size[i] = 0;
    }

    align_get_bits(gb);
}

static int oor_validate_audio_page(OORPage *page, OORSize *size, OORHeader *hdr)
{
    if (hdr != NULL && page->version != hdr->version)
        return -1;
    if (page->flags & OOR_FLAG_BOS)
        return -1;
    if (page->padding1 != 0 || page->padding2 != 0)
        return -1;
    if (size->padding1 != 0 || size->bps_selector == 3)
        return -1;
    if (size->post_padding != 0)
        return -1;
    if (size->packet_count == 0)
        return -1;

    return 0;
}

static int read_page_info(AVIOContext *pb, VorbisCustom *data, int64_t page_offset)
{
    uint8_t buf[0x200] = { 0 };
    int bytes, ret;

    avio_seek(pb, page_offset, SEEK_SET);
    bytes = avio_read(pb, buf, sizeof(buf));
    if (bytes != sizeof(buf))
        return AVERROR_INVALIDDATA;

    GetBitContext gbit, *gb = &gbit;
    ret = init_get_bits8(gb, buf, sizeof(buf));
    if (ret < 0)
        return ret;

    OORPage page = { 0 };
    OORSize size = { 0 };

    oor_read_page(gb, &page);
    oor_read_size(gb, &size);
    if (oor_validate_audio_page(&page, &size, NULL) < 0)
        return AVERROR_INVALIDDATA;

    data->flags = page.flags;
    data->packet_count = size.packet_count;

    for (int i = 0; i < size.packet_count; i++)
        data->packet_size[i] = size.base_packet_size + size.variable_packet_size[i];

    int page_size = get_bits_count(gb) >> 3;

    return page_size;
}

static int oor_read_packet(AVIOContext *pb, PutByteContext *pby,
                           VorbisCustom *data, int64_t *p_offset, int *key)
{
    int read_size = 0;

    if (key)
        *key = 0;

    while (1) {
        if (data->current_packet == 0) {
            if (data->flags & OOR_FLAG_EOS)
                return AVERROR_EOF;

            int page_size = read_page_info(pb, data, *p_offset);
            if (!page_size)
                return AVERROR(EIO);

            if (key)
                *key = 1;
            *p_offset += page_size;

            if (data->packet_count == 0)
                return AVERROR_INVALIDDATA;
        }

        int packet_size = data->packet_size[data->current_packet];
        data->current_packet++;

        if (data->current_packet == data->packet_count)
            data->current_packet = 0;

        avio_seek(pb, *p_offset, SEEK_SET);
        for (int i = 0; i < packet_size; i++) {
            int byte = avio_r8(pb);

            bytestream2_put_byte(pby, byte);
        }

        *p_offset += packet_size;
        read_size += packet_size;

        int is_last = data->current_packet == 0;
        if (!(is_last && (data->flags & OOR_FLAG_PARTIAL)))
            break;
    }

    return read_size;
}

static int read_header(AVFormatContext *s)
{
    OORContext *oor = s->priv_data;
    PutByteContext pbyte, *pby = &pbyte;
    GetBitContext gbit, *gb = &gbit;
    int ret, extradata_size = 0x8000;
    AVIOContext *pb = s->pb;
    uint8_t buffer[32] = { 0};
    OORSetup setup = { 0 };
    OORHeader hdr = { 0 };
    OORPage page = { 0 };
    int64_t start_offset;
    AVStream *st;

    avio_read(pb, buffer, sizeof(buffer));
    ret = init_get_bits8(gb, buffer, sizeof(buffer));
    if (ret < 0)
        return ret;

    oor_read_page(gb, &page);
    oor_read_header(gb, &hdr);

    ret = oor_validate_header_page(&page, &hdr);
    if (ret < 0)
        return ret;

    start_offset = get_bits_count(gb) >> 3;

    if (hdr.channels <= 0 || hdr.sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_VORBIS;
    st->codecpar->ch_layout.nb_channels = hdr.channels;
    st->codecpar->sample_rate = hdr.sample_rate;

    ret = ff_alloc_extradata(st->codecpar, extradata_size);
    if (ret < 0)
        return ret;

    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    uint8_t blocksizes = (hdr.blocksize0_exp << 4) | (hdr.blocksize1_exp);
    bytestream2_init_writer(pby, st->codecpar->extradata, st->codecpar->extradata_size);
    bytestream2_put_be16(pby, 30);
    bytestream2_put_byte(pby, 1);
    bytestream2_put_buffer(pby, "vorbis", 6);
    bytestream2_put_le32(pby, 0);
    bytestream2_put_byte(pby, hdr.channels);
    bytestream2_put_le32(pby, hdr.sample_rate);
    bytestream2_put_le32(pby, 0); /* bitrate_maximum (optional hint) */
    bytestream2_put_le32(pby, 0); /* bitrate_nominal (optional hint) */
    bytestream2_put_le32(pby, 0); /* bitrate_minimum (optional hint) */
    bytestream2_put_byte(pby, blocksizes);
    bytestream2_put_byte(pby, 1);

    bytestream2_put_be16(pby, 25);
    bytestream2_put_byte(pby, 3);
    bytestream2_put_buffer(pby, "vorbis", 6);
    bytestream2_put_le32(pby, 9);
    bytestream2_put_buffer(pby, "librempeg", 9);
    bytestream2_put_le32(pby, 0);
    bytestream2_put_byte(pby, 1);

    int offset = bytestream2_tell_p(pby);
    bytestream2_put_be16(pby, 0);
    bytestream2_put_byte(pby, 5);
    bytestream2_put_buffer(pby, "vorbis", 6);

    avio_seek(pb, start_offset, SEEK_SET);
    int64_t p_offset = avio_tell(pb);

    {
        PutByteContext pbyte, *pby = &pbyte;
        uint8_t info[16] = { 0 };

        bytestream2_init_writer(pby, info, sizeof(info));

        int info_size = oor_read_packet(pb, pby, &oor->data, &p_offset, NULL);
        if (info_size != 1)
            return AVERROR_INVALIDDATA;

        ret = init_get_bits8(gb, info, sizeof(info));
        if (ret < 0)
            return ret;

        oor_read_setup(gb, &setup);
    }

    if (!setup.codebook_id) {
        int setup_size = oor_read_packet(pb, pby, &oor->data, &p_offset, NULL);

        if (setup_size < 0)
            return setup_size;

        if (offset >= st->codecpar->extradata_size-1)
            return AVERROR_INVALIDDATA;

        AV_WB16(st->codecpar->extradata + offset, bytestream2_tell_p(pby) - offset-2);
    }

    st->codecpar->extradata_size = bytestream2_tell_p(pby);

    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    start_offset = p_offset;
    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    OORContext *oor = s->priv_data;
    PutByteContext pbyte, *pby = &pbyte;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb), p_offset;
    int ret, key;

    if (avio_feof(pb))
        return AVERROR_EOF;

    p_offset = pos;
    ret = av_new_packet(pkt, 0x4000);
    if (ret < 0)
        return ret;

    bytestream2_init_writer(pby, pkt->data, pkt->size);
    ret = oor_read_packet(pb, pby, &oor->data, &p_offset, &key);
    if (ret < 0)
        return ret;

    av_shrink_packet(pkt, ret);
    pkt->pos = pos;
    pkt->stream_index = 0;
    if (key)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_oor_demuxer = {
    .p.name         = "oor",
    .p.long_name    = NULL_IF_CONFIG_SMALL("OptimizedObsforR"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "oor",
    .priv_data_size = sizeof(OORContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
