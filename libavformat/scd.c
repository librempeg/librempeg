/*
 * Square Enix SCD demuxer
 * Copyright (C) 2021 Zane van Iperen (zane@zanevaniperen.com)
 *
 * Based off documentation:
 *   http://ffxivexplorer.fragmenterworks.com/research/scd%20files.txt
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

#include <stddef.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/internal.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavformat/internal.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"

#define SCD_MAGIC              ((uint64_t)MKBETAG('S', 'E', 'D', 'B') << 32 | \
                                          MKBETAG('S', 'S', 'C', 'F'))
#define SCD_MIN_HEADER_SIZE    20
#define SCD_OFFSET_HEADER_SIZE 28
#define SCD_TRACK_HEADER_SIZE  32

#define SCD_TRACK_ID_PCM        0
#define SCD_TRACK_ID_PCM_LE     1
#define SCD_TRACK_ID_ADPCM_PSX  3
#define SCD_TRACK_ID_OGG        6
#define SCD_TRACK_ID_MP3        7
#define SCD_TRACK_ID_ADPCMNDSP 10
#define SCD_TRACK_ID_XMA2      11
#define SCD_TRACK_ID_MS_ADPCM  12
#define SCD_TRACK_ID_ADPCM_NDSP 21
#define SCD_TRACK_ID_ATRAC9    22
#define SCD_TRACK_ID_DUMMY     0xffffffffu

typedef struct SCDOffsetTable {
    uint16_t  count;
    uint32_t  offset;
    uint32_t *entries;
} SCDOffsetTable;

typedef struct SCDHeader {
    uint64_t magic;         /* SEDBSSCF                                     */
    uint32_t version;       /* Verison number. We only know about 3.        */
    uint16_t flags;
    uint16_t header_size;   /* Total size of this header.                   */
    uint32_t file_size;     /* Is often 0, just ignore it.                  */

    SCDOffsetTable table0;  /* Table 0, no idea. 56 uint32's/entry.         */
    SCDOffsetTable table1;  /* Table 1, contains the track info.            */
    SCDOffsetTable table2;  /* Table 2, no idea. 40 uint32's/entry.         */
    uint16_t unk2;          /* Unknown, not a count.                        */
    uint32_t unk3;          /* Unknown, not an offset.                      */
    uint32_t unk4;          /* Unknown, offset to offset.                   */
} SCDHeader;

typedef struct SCDTrackHeader {
    uint32_t length;
    uint32_t num_channels;
    uint32_t sample_rate;
    uint32_t data_type;
    uint32_t loop_start;
    uint32_t loop_end;
    uint32_t extradata_size;
    uint32_t aux_count;

    int xor_byte;
    int64_t start_offset;
    int64_t stop_xor_offset;
    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;

    int stream_index;
    int64_t absolute_offset;
} SCDTrackHeader;

typedef struct SCDDemuxContext {
    SCDHeader        hdr;
    SCDTrackHeader  *tracks;
    int              current_track;
    int              nb_streams;
} SCDDemuxContext;

static int scd_probe(const AVProbeData *p)
{
    if (AV_RB64(p->buf) != SCD_MAGIC)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int scd_read_table(AVFormatContext *s, const int be, SCDOffsetTable *table)
{
    AVIOContext *pb = s->pb;
    int64_t ret;

    if ((ret = avio_seek(pb, table->offset, SEEK_SET)) < 0)
        return ret;

    if ((table->entries = av_calloc(table->count, sizeof(uint32_t))) == NULL)
        return ret;

    if ((ret = avio_read(pb, (unsigned char*)table->entries, table->count * sizeof(uint32_t))) < 0)
        return ret;

    for (size_t i = 0; i < table->count; i++)
        table->entries[i] = be ? AV_RB32(table->entries + i) : AV_RL32(table->entries + i);

    av_log(s, AV_LOG_TRACE, "Table, size = %u, offset = %u\n", table->count, table->offset);
    for (size_t i = 0; i < table->count; i++)
        av_log(s, AV_LOG_TRACE, "  [%02zu]: %u\n", i, table->entries[i]);

    return 0;
}

static int scd_read_offsets(AVFormatContext *s, const int be)
{
    int64_t ret;
    SCDDemuxContext  *ctx = s->priv_data;
    uint8_t buf[SCD_OFFSET_HEADER_SIZE];

    if ((ret = avio_read(s->pb, buf, SCD_OFFSET_HEADER_SIZE)) < 0)
        return ret;

    ctx->hdr.table0.count  = be ? AV_RB16(buf +  0) : AV_RL16(buf +  0);
    ctx->hdr.table1.count  = be ? AV_RB16(buf +  2) : AV_RL16(buf +  2);
    ctx->hdr.table2.count  = be ? AV_RB16(buf +  4) : AV_RL16(buf +  4);
    ctx->hdr.unk2          = be ? AV_RB16(buf +  6) : AV_RL16(buf +  6);
    ctx->hdr.table0.offset = be ? AV_RB32(buf +  8) : AV_RL32(buf +  8);
    ctx->hdr.table1.offset = be ? AV_RB32(buf + 12) : AV_RL32(buf + 12);
    ctx->hdr.table2.offset = be ? AV_RB32(buf + 16) : AV_RL32(buf + 16);
    ctx->hdr.unk3          = be ? AV_RB32(buf + 20) : AV_RL32(buf + 20);
    ctx->hdr.unk4          = be ? AV_RB32(buf + 24) : AV_RL32(buf + 24);

    if ((ret = scd_read_table(s, be, &ctx->hdr.table0)) < 0)
        return ret;

    if ((ret = scd_read_table(s, be, &ctx->hdr.table1)) < 0)
        return ret;

    if ((ret = scd_read_table(s, be, &ctx->hdr.table2)) < 0)
        return ret;

    return 0;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    SCDTrackHeader *track = opaque;
    AVFormatContext *s = track->parent;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    ret = avio_read(pb, buf, buf_size);
    if (ret > 0 && pos >= track->start_offset && pos < track->stop_xor_offset) {
        const int64_t stop_offset = FFMIN(pos + ret, track->stop_xor_offset);
        const int xor_byte = track->xor_byte;

        for (int n = pos; n < stop_offset; n++)
            buf[n-pos] ^= xor_byte;
    }

    return ret;
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    SCDTrackHeader *track = opaque;
    AVFormatContext *s = track->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + track->start_offset, whence);
}

static int scd_read_track(AVFormatContext *s, SCDTrackHeader *track, int index, const int be)
{
    AVIOContext *pb = s->pb;
    int64_t ret;
    uint32_t hoffset, chunk;
    AVStream *st;
    AVCodecParameters *par;
    SCDDemuxContext *ctx = s->priv_data;
    uint8_t buf[SCD_TRACK_HEADER_SIZE];

    hoffset = ctx->hdr.table1.entries[index];

    if ((ret = avio_seek(pb, hoffset, SEEK_SET)) < 0)
        return ret;

    if ((ret = avio_read(pb, buf, SCD_TRACK_HEADER_SIZE)) < 0)
        return ret;

    track->length       = be ? AV_RB32(buf +  0) : AV_RL32(buf +  0);
    track->num_channels = be ? AV_RB32(buf +  4) : AV_RL32(buf +  4);
    track->sample_rate  = be ? AV_RB32(buf +  8) : AV_RL32(buf +  8);
    track->data_type    = be ? AV_RB32(buf + 12) : AV_RL32(buf + 12);
    track->loop_start   = be ? AV_RB32(buf + 16) : AV_RL32(buf + 16);
    track->loop_end     = be ? AV_RB32(buf + 20) : AV_RL32(buf + 20);
    track->extradata_size = be ? AV_RB32(buf + 24) : AV_RL32(buf + 24);
    track->aux_count    = be ? AV_RB32(buf + 28) : AV_RL32(buf + 28);

    /* Sanity checks */
    if ((track->data_type != SCD_TRACK_ID_DUMMY) &&
        (track->num_channels <= 0 || track->num_channels > 8 ||
         track->sample_rate <= 0 || track->sample_rate >= 192000 ||
         track->loop_start > track->loop_end))
        return AVERROR_INVALIDDATA;

    track->absolute_offset = hoffset + SCD_TRACK_HEADER_SIZE + track->extradata_size;

    if (track->data_type == SCD_TRACK_ID_DUMMY)
        return 0;

    if ((st = avformat_new_stream(s, NULL)) == NULL)
        return AVERROR(ENOMEM);

    par               = st->codecpar;
    par->codec_type   = AVMEDIA_TYPE_AUDIO;
    par->ch_layout.nb_channels = track->num_channels;
    par->sample_rate  = track->sample_rate;
    st->index         = ctx->nb_streams++;
    st->start_time    = 0;
    track->stream_index = st->index;

    /* TODO: Check this with other types. Drakengard 3 MP3s have 47999 instead of 48000. */
    if (track->data_type == SCD_TRACK_ID_MP3)
        par->sample_rate += 1;

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    if (track->loop_start < track->loop_end) {
        if (av_dict_set_int(&st->metadata, "loop_start", track->loop_start, 0) < 0)
            return AVERROR(ENOMEM);

        if (av_dict_set_int(&st->metadata, "loop_end", track->loop_end, 0) < 0)
            return AVERROR(ENOMEM);
    }

    if (track->aux_count > 0) {
        uint32_t chunk_id = avio_rb32(pb);
        int64_t chunk_size = be ? avio_rb32(pb) : avio_rl32(pb);

        if (chunk_id == MKBETAG('M','A','R','K')) {
            avio_skip(pb, chunk_size - 8);
        } else {
            avio_seek(pb, -8, SEEK_CUR);
        }
    }

    switch (track->data_type) {
    case SCD_TRACK_ID_PCM:
        par->codec_id              = AV_CODEC_ID_PCM_S16BE;
        par->bits_per_coded_sample = 16;
        par->block_align           = par->bits_per_coded_sample * par->ch_layout.nb_channels / 8;
        par->block_align          *= 256;
        break;
    case SCD_TRACK_ID_PCM_LE:
        par->codec_id              = AV_CODEC_ID_PCM_S16LE;
        par->bits_per_coded_sample = 16;
        par->block_align           = par->bits_per_coded_sample * par->ch_layout.nb_channels / 8;
        par->block_align          *= 256;
        break;
    case SCD_TRACK_ID_ADPCM_PSX:
        par->codec_id              = AV_CODEC_ID_ADPCM_PSX;
        par->block_align           = 16 * par->ch_layout.nb_channels;
        break;
    case SCD_TRACK_ID_MP3:
        par->codec_id              = AV_CODEC_ID_MP3;
        par->block_align           = 1024;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    case SCD_TRACK_ID_XMA2:
        par->codec_id              = AV_CODEC_ID_XMA2;
        par->block_align           = 0x800;
        avio_skip(pb, 18);
        ret = ff_get_extradata(s, par, pb, 34);
        if (ret < 0)
            return ret;
        if (be) {
            AV_WB16(par->extradata, AV_RL16(par->extradata));
            AV_WB32(par->extradata+2, AV_RL32(par->extradata+2));
        }

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    case SCD_TRACK_ID_MS_ADPCM:
        par->codec_id              = AV_CODEC_ID_ADPCM_MS;
        avio_skip(pb, 12);
        par->block_align           = be ? avio_rb16(pb) : avio_rl16(pb);
        if (par->block_align == 0)
            return AVERROR_INVALIDDATA;
        break;
    case SCD_TRACK_ID_ADPCM_NDSP:
    case SCD_TRACK_ID_ADPCMNDSP:
        par->codec_id              = be ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
        par->block_align           = 0x800 * par->ch_layout.nb_channels;
        par->bit_rate              = 8LL * st->codecpar->ch_layout.nb_channels * 8 *
                                           st->codecpar->sample_rate / 14;

        avio_seek(pb, track->absolute_offset + 0x1c, SEEK_SET);

        ret = ff_alloc_extradata(par, 32 * par->ch_layout.nb_channels + 1);
        if (ret < 0)
            return ret;

        for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
            ret = avio_read(pb, par->extradata + 32 * ch, 32);
            if (ret < 0)
                return ret;

            avio_skip(pb, 0x800-32);
        }

        par->extradata[32 * par->ch_layout.nb_channels] = 0x60;
        break;
    case SCD_TRACK_ID_ATRAC9:
        par->codec_id              = AV_CODEC_ID_ATRAC9;
        chunk = avio_rb32(pb);
        if (chunk == MKBETAG('M','A','R','K')) {
            int size = be ? avio_rb32(pb) : avio_rl32(pb);

            avio_skip(pb, size - 4);
        }
        par->block_align           = be ? avio_rb16(pb) : avio_rl16(pb);
        avio_skip(pb, 2);
        if (track->extradata_size < 8+12)
            return AVERROR_INVALIDDATA;

        ret = ff_get_extradata(s, par, pb, 12);
        if (ret < 0)
            return ret;

        if (!be) {
            AV_WB32(par->extradata+4, AV_RL32(par->extradata+4));
            AV_WB32(par->extradata+8, AV_RL32(par->extradata+8));
        }

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    case SCD_TRACK_ID_OGG:
        {
            int version, seek_table_size = -1, vorb_header_size = -1;
            int64_t skip;

            version = avio_r8(pb);
            avio_skip(pb, 1);
            track->xor_byte = avio_r8(pb);
            if (version == 2) {
                avio_skip(pb, 13);
                seek_table_size = be ? avio_rb32(pb) : avio_rl32(pb);
                vorb_header_size = be ? avio_rb32(pb) : avio_rl32(pb);
                avio_skip(pb, 8);
            }

            if (seek_table_size < 0 ||
                vorb_header_size < 0)
                return AVERROR_INVALIDDATA;

            avio_skip(pb, seek_table_size);

            track->start_offset = avio_tell(pb);
            track->stop_xor_offset = track->start_offset + vorb_header_size;
            if (!(track->xctx = avformat_alloc_context()))
                return AVERROR(ENOMEM);

            if ((ret = ff_copy_whiteblacklists(track->xctx, s)) < 0) {
                avformat_free_context(track->xctx);
                track->xctx = NULL;

                return ret;
            }

            ffio_init_context(&track->apb, NULL, 0, 0, track,
                              read_data, NULL, seek_data);

            track->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
            track->xctx->probesize = 0;
            track->xctx->max_analyze_duration = 0;
            track->xctx->interrupt_callback = s->interrupt_callback;
            track->xctx->pb = &track->apb.pub;
            track->xctx->io_open = NULL;
            track->xctx->skip_initial_bytes = 0;
            track->parent = s;

            avio_seek(pb, track->start_offset, SEEK_SET);
            if ((ret = avformat_open_input(&track->xctx, "", NULL, NULL)) < 0)
                return ret;

            st->id = track->xctx->streams[0]->id;
            st->duration = track->xctx->streams[0]->duration;
            st->time_base = track->xctx->streams[0]->time_base;
            st->start_time = track->xctx->streams[0]->start_time;
            st->pts_wrap_bits = track->xctx->streams[0]->pts_wrap_bits;
            st->codecpar->codec_id = track->xctx->streams[0]->codecpar->codec_id;
            st->codecpar->bit_rate = track->xctx->streams[0]->codecpar->bit_rate;
            st->codecpar->sample_rate = track->xctx->streams[0]->codecpar->sample_rate;
            st->codecpar->block_align = track->xctx->streams[0]->codecpar->block_align;
            if ((ret = av_channel_layout_copy(&st->codecpar->ch_layout, &track->xctx->streams[0]->codecpar->ch_layout)) < 0)
                return ret;

            if ((ret = ff_alloc_extradata(st->codecpar, track->xctx->streams[0]->codecpar->extradata_size)))
                return ret;
            memcpy(st->codecpar->extradata, track->xctx->streams[0]->codecpar->extradata, track->xctx->streams[0]->codecpar->extradata_size);

            ret = av_dict_copy(&st->metadata, track->xctx->streams[0]->metadata, 0);
            if (ret < 0)
                return ret;

            ffstream(st)->request_probe = 0;
            ffstream(st)->need_parsing = ffstream(track->xctx->streams[0])->need_parsing;

            skip = avio_tell(pb) - track->absolute_offset;
            track->absolute_offset += skip;
            track->length -= skip;
        }
        break;
    default:
        par->codec_id              = AV_CODEC_ID_NONE;
        avpriv_request_sample(s, "data type %u", track->data_type);
    }

    return 0;
}

static int scd_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    SCDDemuxContext *ctx = s->priv_data;
    uint8_t buf[SCD_MIN_HEADER_SIZE];
    int64_t ret;
    int be = 0;

    if ((ret = ffio_read_size(pb, buf, SCD_MIN_HEADER_SIZE)) < 0)
        return ret;

    ctx->hdr.magic       = AV_RB64(buf +  0);
    ctx->hdr.flags       = AV_RB16(buf + 12);
    be = (ctx->hdr.flags >> 8) == 0x01;
    ctx->hdr.version     = be ? AV_RB32(buf +  8) : AV_RL32(buf + 8);
    ctx->hdr.header_size = be ? AV_RB16(buf + 14) : AV_RL16(buf + 14);
    ctx->hdr.file_size   = be ? AV_RB32(buf + 16) : AV_RL32(buf + 16);

    if (ctx->hdr.magic != SCD_MAGIC)
        return AVERROR_INVALIDDATA;

    if (ctx->hdr.version != 3 && ctx->hdr.version != 4) {
        avpriv_request_sample(s, "SCD version %u", ctx->hdr.version);
        return AVERROR_PATCHWELCOME;
    }

    if (ctx->hdr.header_size < SCD_MIN_HEADER_SIZE)
        return AVERROR_INVALIDDATA;

    if ((ret = avio_skip(pb, ctx->hdr.header_size - SCD_MIN_HEADER_SIZE)) < 0)
        return ret;

    if ((ret = scd_read_offsets(s, be)) < 0)
        return ret;

    ctx->tracks = av_calloc(ctx->hdr.table1.count, sizeof(SCDTrackHeader));
    if (ctx->tracks == NULL)
        return AVERROR(ENOMEM);

    for (int i = 0; i < ctx->hdr.table1.count; i++) {
        if ((ret = scd_read_track(s, ctx->tracks + i, i, be)) < 0)
            return ret;
    }

    if (ctx->hdr.table1.count == 0)
        return 0;

    if ((ret = avio_seek(pb, ctx->tracks[0].absolute_offset, SEEK_SET)) < 0)
        return ret;

    return 0;
}

static int scd_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t ret, track_start, track_stop;
    SCDDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    SCDTrackHeader *trk;
    int64_t current_pos;
    int size;

redo:
    current_pos = avio_tell(pb);
    if (ctx->current_track >= ctx->hdr.table1.count)
        return AVERROR_EOF;

    do {
        trk = ctx->tracks + ctx->current_track;
        track_stop = trk->absolute_offset + trk->length;
        par = s->streams[trk->stream_index]->codecpar;

        if (trk->data_type != SCD_TRACK_ID_DUMMY &&
            track_stop > current_pos)
            break;

        ctx->current_track++;
    } while (ctx->current_track < ctx->hdr.table1.count);

    track_start = trk->absolute_offset;
    if (current_pos < track_start)
        avio_skip(pb, track_start - current_pos);

    track_stop = trk->absolute_offset + trk->length;

    if (trk->xctx) {
        ret = av_read_frame(trk->xctx, pkt);
        if (ret == AVERROR_EOF) {
            ctx->current_track++;
            goto redo;
        }
    } else {
        size = FFMIN(par->block_align, track_stop - current_pos);
        if (size == 0) {
            ctx->current_track++;
            goto redo;
        }

        ret = av_get_packet(pb, pkt, size);
        if (ret == AVERROR_EOF) {
            return ret;
        } else if (ret < 0) {
            return ret;
        }
    }

    if (trk->data_type == SCD_TRACK_ID_PCM) {
        pkt->pts      = (current_pos - trk->absolute_offset) / (par->ch_layout.nb_channels * sizeof(uint16_t));
        pkt->duration = size / (par->ch_layout.nb_channels * sizeof(int16_t));
    }

    pkt->flags        &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index  = trk->stream_index;

    return 0;
}

static int scd_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    SCDDemuxContext *ctx = s->priv_data;
    SCDTrackHeader *trk;

    ctx->current_track = av_clip(stream_index, 0, s->nb_streams-1);
    trk = ctx->tracks + ctx->current_track;
    if (trk->xctx)
        return av_seek_frame(trk->xctx, 0, timestamp, flags);

    return -1;
}

static int scd_read_close(AVFormatContext *s)
{
    SCDDemuxContext *ctx = s->priv_data;

    av_freep(&ctx->hdr.table0.entries);
    av_freep(&ctx->hdr.table1.entries);
    av_freep(&ctx->hdr.table2.entries);
    av_freep(&ctx->tracks);

    return 0;
}

const FFInputFormat ff_scd_demuxer = {
    .p.name         = "scd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix SCD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SCDDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = scd_probe,
    .read_header    = scd_read_header,
    .read_packet    = scd_read_packet,
    .read_seek      = scd_read_seek,
    .read_close     = scd_read_close,
};
