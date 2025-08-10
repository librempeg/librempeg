/*
 * Sony Playstation (PSX) STR File Demuxer
 * Copyright (c) 2003 The FFmpeg project
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
 * PSX STR file demuxer
 * by Mike Melanson (melanson@pcisys.net)
 * This module handles streams that have been ripped from Sony Playstation
 * CD games. This demuxer can handle either raw STR files (which are just
 * concatenations of raw compact disc sectors) or STR files with 0x2C-byte
 * RIFF headers, followed by CD sectors.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "avio_internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define RIFF_TAG MKTAG('R', 'I', 'F', 'F')
#define CDXA_TAG MKTAG('C', 'D', 'X', 'A')

#define RAW_CD_SECTOR_SIZE      2352
#define RAW_CD_SECTOR_DATA_SIZE 2304
#define RAW_DATA_SIZE           2048
#define VIDEO_DATA_CHUNK_SIZE   0x7E0
#define VIDEO_DATA_HEADER_SIZE  0x38
#define RIFF_HEADER_SIZE        0x2C

#define CDXA_TYPE_MASK     0x0E
#define CDXA_TYPE_DATA     0x08
#define CDXA_TYPE_AUDIO    0x04
#define CDXA_TYPE_VIDEO    0x02
#define CDXA_TYPE_EMPTY    0x00

#define STR_MAGIC 0x60010180
#define STR_MAGIC_A 0x60010000
#define STR_MAGIC_B 0x60010001
#define STR_MAGIC_C 0x60010100
#define STR_MAGIC_D 0x60010101

typedef struct StrChannel {
    /* video parameters */
    int video_stream_index;
    AVPacket tmp_pkt;

    /* audio parameters */
    int audio_stream_index;
} StrChannel;

typedef struct StrDemuxContext {
    int mode;

    /* a STR file can contain up to 32 channels of data */
    StrChannel channels[32];
} StrDemuxContext;

static const uint8_t sync_header[12] = {0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00};

static int str_probe(const AVProbeData *p)
{
    const uint8_t *sector = p->buf;
    const uint8_t *end = sector + p->buf_size;
    int aud = 0, vid = 0, score = 0;
    int prev_mode = -1, mode = -1;
    int sector_size, offset;

    if (p->buf_size < RAW_CD_SECTOR_SIZE)
        return 0;

    if ((AV_RL32(&p->buf[0]) == RIFF_TAG) &&
        (AV_RL32(&p->buf[8]) == CDXA_TAG)) {

        /* RIFF header seen; skip 0x2C bytes */
        sector += RIFF_HEADER_SIZE;
    }

    while (end - sector >= RAW_CD_SECTOR_SIZE) {
        prev_mode = mode;
        /* look for CD sync header (00, 0xFF x 10, 00) */
        if (!memcmp(sector, sync_header, sizeof(sync_header))) {
            sector_size = RAW_CD_SECTOR_SIZE;
            mode = 0;
            score += 9;
            offset = 0x10;
        } else {
            if (AV_RB32(sector) == STR_MAGIC ||
                AV_RB32(sector) == STR_MAGIC_A ||
                AV_RB32(sector) == STR_MAGIC_B ||
                AV_RB32(sector) == STR_MAGIC_C ||
                AV_RB32(sector) == STR_MAGIC_D) {
                sector_size = 2048;
                mode = 2;
                score += 4;
                offset = -8;
            } else {
                sector_size = 2336;
                mode = 1;
                score += 7;
                offset = 0;
            }
        }

        if (prev_mode >= 0 && prev_mode != mode)
            return 0;

        if (mode == 2)
            goto skip;

        if (sector[offset + 1] >= 32)
            return 0;

        switch (sector[offset + 2] & CDXA_TYPE_MASK) {
        case CDXA_TYPE_DATA:
        case CDXA_TYPE_VIDEO: {
                int current_sector = AV_RL16(&sector[offset + 0xC]);
                int sector_count   = AV_RL16(&sector[offset + 0xE]);
                int frame_size = AV_RL32(&sector[offset + 0x14]);

                if (!(  frame_size >= 0
                     && current_sector < sector_count
                     && sector_count*VIDEO_DATA_CHUNK_SIZE >= frame_size)) {
                    return 0;
                }
                vid++;

            }
            break;
        case CDXA_TYPE_AUDIO:
            if (sector[offset + 0x3] & 0x2A)
                return 0;
            aud++;
            break;
        default:
            if (sector[offset + 0x2] & CDXA_TYPE_MASK)
                return 0;
        }
skip:
        sector += sector_size;
    }
    /* MPEG files (like those ripped from VCDs) can also look like this;
     * only return half certainty */
    if (mode == 2 || vid+aud > 3)  return FFMIN(score, AVPROBE_SCORE_MAX);
    else if (vid+aud) return 1;
    else              return 0;
}

static int str_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    StrDemuxContext *str = s->priv_data;
    uint8_t sector[RAW_CD_SECTOR_SIZE];
    int start, ret;

    /* skip over any RIFF header */
    if ((ret = ffio_ensure_seekback(pb, RIFF_HEADER_SIZE)) < 0)
        return ret;

    if (avio_read(pb, sector, RIFF_HEADER_SIZE) != RIFF_HEADER_SIZE)
        return AVERROR(EIO);
    if (AV_RL32(&sector[0]) == RIFF_TAG)
        start = RIFF_HEADER_SIZE;
    else
        start = 0;

    avio_seek(pb, start, SEEK_SET);

    for (int i = 0; i < 32; i++) {
        str->channels[i].video_stream_index =
        str->channels[i].audio_stream_index = -1;
    }

    s->ctx_flags |= AVFMTCTX_NOHEADER;
    str->mode = -1;

    return 0;
}

static int str_read_packet(AVFormatContext *s,
                           AVPacket *ret_pkt)
{
    AVIOContext *pb = s->pb;
    StrDemuxContext *str = s->priv_data;
    uint8_t sector[RAW_CD_SECTOR_SIZE];
    int channel, ret;
    AVPacket *pkt;
    AVStream *st;
    int64_t pos;

    pos = avio_tell(pb);

    while (!avio_feof(pb)) {
        int read = avio_read(pb, sector, RAW_DATA_SIZE);

        if (read == AVERROR_EOF)
            return AVERROR_EOF;

        if (read != RAW_DATA_SIZE)
            return AVERROR(EIO);

        if ((str->mode == 0) || !memcmp(sector, sync_header, sizeof(sync_header))) {
            int read = avio_read(pb, sector + RAW_DATA_SIZE, sizeof(sector) - RAW_DATA_SIZE);

            if (str->mode < 0)
                str->mode = 0;

            if (read == AVERROR_EOF)
                return AVERROR_EOF;

            if (read != sizeof(sector)-RAW_DATA_SIZE)
                return AVERROR(EIO);
        } else if ((str->mode == 3) ||
                   AV_RB32(sector) == STR_MAGIC_A ||
                   AV_RB32(sector) == STR_MAGIC_B ||
                   AV_RB32(sector) == STR_MAGIC_C ||
                   AV_RB32(sector) == STR_MAGIC_D) {
            if (str->mode < 0)
                str->mode = 3;

            memmove(sector + 0x18, sector, RAW_DATA_SIZE);
            memset(sector, 0, 0x18);
            sector[0x12] = (AV_RB32(sector + 0x18) == STR_MAGIC) ? CDXA_TYPE_VIDEO : CDXA_TYPE_AUDIO;
            memset(sector + 0x18 + RAW_DATA_SIZE, 0, sizeof(sector) - 0x18 - RAW_DATA_SIZE);
        } else if ((str->mode == 2) || AV_RB32(sector) == STR_MAGIC) {
            if (str->mode < 0)
                str->mode = 2;

            memmove(sector + 0x18, sector, RAW_DATA_SIZE);
            memset(sector, 0, 0x18);
            sector[0x12] = (AV_RB32(sector + 0x18) == STR_MAGIC) ? CDXA_TYPE_VIDEO : CDXA_TYPE_AUDIO;
            memset(sector + 0x18 + RAW_DATA_SIZE, 0, sizeof(sector) - 0x18 - RAW_DATA_SIZE);
        } else if (str->mode == 1 || str->mode < 0) {
            if (str->mode < 0)
                str->mode = 1;

            memmove(sector + 0x10, sector, RAW_DATA_SIZE);
            memset(sector, 0, 0x10);
            avio_read(pb, sector + 0x10 + RAW_DATA_SIZE, sizeof(sector) - RAW_DATA_SIZE - 0x10);
        }

        channel = sector[0x11];
        if (channel >= 32)
            return AVERROR_INVALIDDATA;

        switch (sector[0x12] & CDXA_TYPE_MASK) {

        case CDXA_TYPE_DATA:
        case CDXA_TYPE_VIDEO:
            {
                int current_sector = AV_RL16(&sector[0x1C]);
                int sector_count   = AV_RL16(&sector[0x1E]);
                int frame_size = AV_RL32(&sector[0x24]);

                if (!(  frame_size >= 0
                     && current_sector < sector_count
                     && sector_count*VIDEO_DATA_CHUNK_SIZE >= frame_size)) {
                    av_log(s, AV_LOG_ERROR, "Invalid parameters %d %d %d\n", current_sector, sector_count, frame_size);
                    break;
                }

                if (str->channels[channel].video_stream_index < 0) {
                    /* allocate a new AVStream */
                    st = avformat_new_stream(s, NULL);
                    if (!st)
                        return AVERROR(ENOMEM);
                    avpriv_set_pts_info(st, 64, 1, 15);

                    str->channels[channel].video_stream_index = st->index;

                    st->start_time = 0;
                    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
                    st->codecpar->codec_id   = AV_CODEC_ID_MDEC;
                    st->codecpar->codec_tag  = 0;  /* no fourcc */
                    st->codecpar->width      = AV_RL16(&sector[0x28]);
                    st->codecpar->height     = AV_RL16(&sector[0x2A]);
                }

                /* if this is the first sector of the frame, allocate a pkt */
                pkt = &str->channels[channel].tmp_pkt;

                if (pkt->size != sector_count*VIDEO_DATA_CHUNK_SIZE) {
                    if (pkt->data)
                        av_log(s, AV_LOG_ERROR, "mismatching sector_count\n");
                    av_packet_unref(pkt);
                    ret = av_new_packet(pkt, sector_count * VIDEO_DATA_CHUNK_SIZE);
                    if (ret < 0)
                        return ret;
                    memset(pkt->data, 0, sector_count*VIDEO_DATA_CHUNK_SIZE);

                    pkt->pos = pos;
                    pkt->stream_index =
                        str->channels[channel].video_stream_index;
                }

                memcpy(pkt->data + current_sector*VIDEO_DATA_CHUNK_SIZE,
                       sector + VIDEO_DATA_HEADER_SIZE,
                       VIDEO_DATA_CHUNK_SIZE);

                if (current_sector == sector_count-1) {
                    pkt->size = frame_size;
                    *ret_pkt = *pkt;
                    pkt->data = NULL;
                    pkt->size = -1;
                    pkt->buf = NULL;
                    return 0;
                }
            }
            break;

        case CDXA_TYPE_AUDIO:
            if (str->channels[channel].audio_stream_index < 0) {
                int fmt = sector[0x13];
                /* allocate a new AVStream */
                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);

                str->channels[channel].audio_stream_index = st->index;

                st->start_time = 0;
                st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
                st->codecpar->codec_tag   = 0;  /* no fourcc */

                if (str->mode == 3) {
                    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
                    av_channel_layout_default(&st->codecpar->ch_layout, 2);
                    st->codecpar->sample_rate = 44100;
                    st->codecpar->block_align = 1680 * st->codecpar->ch_layout.nb_channels;
                    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
                } else {
                    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_XA;
                    av_channel_layout_default(&st->codecpar->ch_layout, (fmt & 1) + 1);
                    st->codecpar->sample_rate = (fmt & 4) ? 18900 : 37800;
                    st->codecpar->block_align = 128;
                }
                st->codecpar->bit_rate = (int64_t)st->codecpar->sample_rate * st->codecpar->ch_layout.nb_channels * 128 * 8LL / 224;

                avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
            }
            pkt = ret_pkt;

            if (str->mode == 3) {
                if ((ret = av_new_packet(pkt, 1680)) < 0)
                    return ret;
                memcpy(pkt->data, sector + 232, 1680);
            } else {
                if ((ret = av_new_packet(pkt, 2304)) < 0)
                    return ret;
                memcpy(pkt->data, sector + 24, 2304);
            }

            pkt->pos = pos;
            pkt->stream_index = str->channels[channel].audio_stream_index;
            return 0;
        case CDXA_TYPE_EMPTY: /* CD-ROM XA, May 1991, 4.3.2.3 */
            /* NOTE this also catches 0x80 (EOF bit) because of CDXA_TYPE_MASK */
            /* TODO consider refactoring so as to explicitly handle each case? */
            break;
        default:
            av_log(s, AV_LOG_WARNING, "Unknown sector type %02X\n", sector[0x12]);
            /* drop the sector and move on */
            break;
        }
    }

    return AVERROR_EOF;
}

static int str_read_close(AVFormatContext *s)
{
    StrDemuxContext *str = s->priv_data;

    for (int i = 0; i < 32; i++) {
        if (str->channels[i].tmp_pkt.data)
            av_packet_unref(&str->channels[i].tmp_pkt);
    }

    return 0;
}

const FFInputFormat ff_str_demuxer = {
    .p.name         = "psxstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony Playstation STR"),
    .p.flags        = AVFMT_NO_BYTE_SEEK | AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(StrDemuxContext),
    .read_probe     = str_probe,
    .read_header    = str_read_header,
    .read_packet    = str_read_packet,
    .read_close     = str_read_close,
};
