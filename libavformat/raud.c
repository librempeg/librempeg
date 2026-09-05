/*
 * Rockstar AUD demuxer
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

#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define SUBBLOCK_SIZE 0x800
#define MAX_NB_STREAMS 12

typedef struct SubBlocksTabEntry {
    int64_t block_pos;
    int nb_subblocks;
    int subblock_size;
    int subblock_offset;
    int subblock_samples;
} SubBlocksTabEntry;

typedef struct RAUDContext {
    int be;
    int block_size;
    int block_subblocks;
    int header_size;
    int pkt_prefix_size;

    int nb_subblocks[MAX_NB_STREAMS];
    int subblock_size[MAX_NB_STREAMS];
    int subblock_offset[MAX_NB_STREAMS];

    int64_t block_pos;
    int stream_index;
    int subblock_index;

    int history_index[MAX_NB_STREAMS];
    int history_size[MAX_NB_STREAMS];
    uint8_t *history[MAX_NB_STREAMS];

    AVFifo *fifo[MAX_NB_STREAMS];
} RAUDContext;

static int read_probe(const AVProbeData *p)
{
    int be = AV_RB32(p->buf) == 0;
    uint64_t offset;

    if (be)
        offset = AV_RB64(p->buf);
    else
        offset = AV_RL64(p->buf);
    if (offset > 0x20000 || offset < 0x1c)
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RB32(p->buf+16) != 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int64_t start, duration, table_offset, offset, ch_tab_offset, ch_tab_end;
    int nb_streams, rate, be, codec, align = 0;
    unsigned int (*avio_r32)(AVIOContext *pb);
    unsigned int (*avio_r16)(AVIOContext *pb);
    uint64_t (*avio_r64)(AVIOContext *pb);
    RAUDContext *raud = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    raud->be = be = avio_rb32(pb) == 0;
    avio_r64 = be ? avio_rb64 : avio_rl64;
    avio_r32 = be ? avio_rb32 : avio_rl32;
    avio_r16 = be ? avio_rb16 : avio_rl16;
    avio_seek(pb, 0, SEEK_SET);
    table_offset = avio_r64(pb);
    if (table_offset > 0x20000 || table_offset < 0x1c)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    raud->block_size = avio_r32(pb);
    if (raud->block_size <= 0)
        return AVERROR_INVALIDDATA;
    raud->block_subblocks = raud->block_size / SUBBLOCK_SIZE;
    if (raud->block_subblocks <= 0)
        return AVERROR_INVALIDDATA;

    nb_streams = avio_r32(pb);
    if (nb_streams < 0)
        return AVERROR_INVALIDDATA;

    if (nb_streams == 0) {
        avio_seek(pb, 20, SEEK_SET);
        ch_tab_offset = offset = avio_r64(pb);
        ch_tab_end = avio_r64(pb);
        nb_streams = avio_r32(pb);
        if (nb_streams <= 0 || nb_streams > FF_ARRAY_ELEMS(raud->nb_subblocks))
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 4);
        start = avio_r32(pb);
        offset += nb_streams * 16LL;
        avio_seek(pb, offset + 16, SEEK_SET);
        duration = avio_r32(pb);
        avio_skip(pb, 8);
        codec = avio_r32(pb);
        avio_seek(pb, table_offset + 4, SEEK_SET);
        rate = avio_r32(pb);
        if (codec == 1024) {
            for (int si = 0; si < nb_streams; si++) {
                raud->history_size[si] = duration / (SUBBLOCK_SIZE*2);
                raud->history[si] = av_calloc(raud->history_size[si], 3);
                if (!raud->history[si])
                    return AVERROR(ENOMEM);

                avio_seek(pb, ch_tab_offset + 16 * si, SEEK_SET);
                int64_t ch_info_offset = avio_r64(pb);

                avio_seek(pb, ch_tab_offset + ch_info_offset + 0x38 + 16 * nb_streams, SEEK_SET);

                for (int n = 0; n < raud->history_size[si]; n++) {
                    if (avio_feof(pb))
                        return AVERROR_INVALIDDATA;

                    if (avio_tell(pb) >= ch_tab_end)
                        break;

                    uint16_t pred = avio_r16(pb);
                    uint8_t step = avio_r8(pb);

                    raud->history[si][n*3] = step;
                    AV_WL16(&raud->history[si][n*3+1], pred);
                }
            }
        }
    } else {
        return AVERROR_PATCHWELCOME;
    }

    switch (codec) {
    case 1:
        codec = be ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
        align = 2;
        break;
    case 0:
        codec = AV_CODEC_ID_XMA1;
        align = 0x800;
        break;
    case 256:
        codec = AV_CODEC_ID_MP3;
        align = 1024;
        break;
    case 1024:
        codec = AV_CODEC_ID_ADPCM_IMA_RAUD;
        raud->pkt_prefix_size = 4;
        align = 1;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_INVALIDDATA;
    }

    if (rate <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        raud->fifo[n] = av_fifo_alloc2(32, sizeof(SubBlocksTabEntry), AV_FIFO_FLAG_AUTO_GROW);
        if (!raud->fifo[n])
            return AVERROR(ENOMEM);

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align;

        if (codec == AV_CODEC_ID_MP3) {
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        } else if (codec == AV_CODEC_ID_XMA1) {
            int ret = ff_alloc_extradata(st->codecpar, 8 + 20);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            st->codecpar->extradata[4] = 1;
            st->codecpar->extradata[8+17] = 1;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    avio_seek(pb, start, SEEK_SET);

    if (s->nb_streams > 3)
        raud->header_size = 0x1000;
    else
        raud->header_size = 0x800;

    return 0;
}

static int have_entry(AVFifo *fifo, const int64_t pos)
{
    size_t index = av_fifo_can_read(fifo);

    if (index <= 0)
        return 0;

    for (int i = 0; i < index; i++) {
        SubBlocksTabEntry entry;

        av_fifo_peek(fifo, &entry, 1, i);
        if (entry.block_pos >= pos)
            return 1;
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    RAUDContext *raud = s->priv_data;
    AVIOContext *pb = s->pb;
    const int be = raud->be;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (((pos - ffformatcontext(s)->data_offset) % raud->block_size) == 0) {
        int64_t seek_info_entry_size;
        int64_t seek_info_offset;
        int64_t seek_info_size;
        int add_entry;

        add_entry = !have_entry(raud->fifo[0], pos);
        raud->block_pos = pos;
        raud->stream_index = 0;
        raud->subblock_index = 0;

        seek_info_offset = be ? avio_rb64(pb) : avio_rl64(pb);
        seek_info_size = be ? avio_rb64(pb) : avio_rl64(pb);
        seek_info_entry_size = (seek_info_size - seek_info_offset) / s->nb_streams;

        if (avio_feof(pb))
            return AVERROR_EOF;

        if (seek_info_offset <= 0 || seek_info_size <= 0)
            return AVERROR_INVALIDDATA;

        memset(raud->nb_subblocks, 0, sizeof(raud->nb_subblocks));
        memset(raud->subblock_size, 0, sizeof(raud->subblock_size));
        memset(raud->subblock_offset, 0, sizeof(raud->subblock_offset));

        for (int n = 0; n < s->nb_streams; n++) {
            const int64_t entry_pos = pos + seek_info_offset + seek_info_entry_size * n;
            int subblock_size = 0;
            int subblock_samples;
            int subblock_offset;
            int nb_subblocks;

            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;

            avio_seek(pb, entry_pos, SEEK_SET);
            subblock_offset = be ? avio_rb32(pb) : avio_rl32(pb);
            nb_subblocks = be ? avio_rb32(pb) : avio_rl32(pb);
            avio_skip(pb, 4);
            subblock_samples = be ? avio_rb32(pb) : avio_rl32(pb);

            if (subblock_offset < 0 || nb_subblocks <= 0)
                return AVERROR_INVALIDDATA;

            raud->subblock_offset[n] = subblock_offset;
            raud->nb_subblocks[n] = nb_subblocks;
            if (seek_info_entry_size >= 24) {
                avio_seek(pb, entry_pos + 20, SEEK_SET);
                subblock_size = be ? avio_rb32(pb) : avio_rl32(pb);

                if (subblock_size < 0)
                    return AVERROR_INVALIDDATA;

                raud->subblock_size[n] = subblock_size;
            }

            if (add_entry) {
                SubBlocksTabEntry entry;

                entry.block_pos = pos;
                entry.subblock_size = subblock_size;
                entry.nb_subblocks = nb_subblocks;
                entry.subblock_offset = subblock_offset;
                entry.subblock_samples = subblock_samples;

                av_fifo_write(raud->fifo[n], &entry, 1);
            }
        }

        if (raud->subblock_offset[raud->stream_index] + raud->subblock_index >= raud->block_subblocks)
            return AVERROR_INVALIDDATA;

        int pkt_size;
        if (raud->subblock_size[raud->stream_index] > 0) {
            pkt_size = FFMIN(raud->subblock_size[raud->stream_index] - SUBBLOCK_SIZE * raud->subblock_index, SUBBLOCK_SIZE);
        } else {
            pkt_size = SUBBLOCK_SIZE;
        }

        if (pkt_size > 0) {
            ret = av_new_packet(pkt, pkt_size + raud->pkt_prefix_size);
            if (ret < 0)
                return ret;

            avio_seek(pb, pos + raud->header_size + SUBBLOCK_SIZE * (raud->subblock_offset[raud->stream_index] + raud->subblock_index), SEEK_SET);
            avio_read(pb, pkt->data + raud->pkt_prefix_size, pkt_size);
            if (raud->pkt_prefix_size > 0) {
                const int n = av_clip(raud->history_index[raud->stream_index], 0, raud->history_size[raud->stream_index]-1);
                const int step = raud->history[raud->stream_index][n*3];
                const int pred = AV_RL16(&raud->history[raud->stream_index][n*3+1]);

                pkt->data[0] = step;
                pkt->data[1] = 0;
                AV_WL16(pkt->data + 2, pred);
            }

            pkt->stream_index = raud->stream_index;
            pkt->pts = raud->history_index[raud->stream_index] * (SUBBLOCK_SIZE*2LL);
            raud->history_index[raud->stream_index]++;
        }

        raud->subblock_index++;
        if (raud->subblock_index >= raud->nb_subblocks[raud->stream_index]) {
            raud->subblock_index = 0;
            raud->stream_index++;
            if (raud->stream_index >= s->nb_streams) {
                raud->subblock_index = 0;
                raud->stream_index = 0;

                avio_seek(pb, raud->block_pos + raud->block_size, SEEK_SET);
            }
        }

        if (pkt_size <= 0)
            return FFERROR_REDO;
    } else {
        pos = raud->block_pos;

        if (raud->subblock_offset[raud->stream_index] + raud->subblock_index >= raud->block_subblocks)
            return AVERROR_INVALIDDATA;

        int pkt_size;
        if (raud->subblock_size[raud->stream_index] > 0) {
            pkt_size = FFMIN(raud->subblock_size[raud->stream_index] - SUBBLOCK_SIZE * raud->subblock_index, SUBBLOCK_SIZE);
        } else {
            pkt_size = SUBBLOCK_SIZE;
        }

        if (pkt_size > 0) {
            ret = av_new_packet(pkt, pkt_size + raud->pkt_prefix_size);
            if (ret < 0)
                return ret;

            avio_seek(pb, pos + raud->header_size + SUBBLOCK_SIZE * (raud->subblock_offset[raud->stream_index] + raud->subblock_index), SEEK_SET);
            avio_read(pb, pkt->data + raud->pkt_prefix_size, pkt_size);
            if (raud->pkt_prefix_size > 0) {
                const int n = av_clip(raud->history_index[raud->stream_index], 0, raud->history_size[raud->stream_index]-1);
                const int step = raud->history[raud->stream_index][n*3];
                const int pred = AV_RL16(&raud->history[raud->stream_index][n*3+1]);

                pkt->data[0] = step;
                pkt->data[1] = 0;
                AV_WL16(pkt->data + 2, pred);
            }

            pkt->stream_index = raud->stream_index;
            pkt->pts = raud->history_index[raud->stream_index] * (SUBBLOCK_SIZE*2LL);
            raud->history_index[raud->stream_index]++;
        }

        raud->subblock_index++;
        if (raud->subblock_index >= raud->nb_subblocks[raud->stream_index]) {
            raud->subblock_index = 0;
            raud->stream_index++;
            if (raud->stream_index >= s->nb_streams) {
                raud->subblock_index = 0;
                raud->stream_index = 0;

                avio_seek(pb, raud->block_pos + raud->block_size, SEEK_SET);
            }
        }

        if (pkt_size <= 0)
            return FFERROR_REDO;
    }

    pkt->pos = pos;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    RAUDContext *raud = s->priv_data;
    AVIOContext *pb = s->pb;
    SubBlocksTabEntry entry;
    const int be = raud->be;
    int64_t new_timestamp = 0;
    int subblock_offset = 0;
    int subblock_index = 0;
    int entry_index = -1;
    int64_t new_pos = 0;
    int start_entry = 0;
    int nb_entries;

    stream_index = av_clip(stream_index, 0, s->nb_streams-1);

    memset(raud->history_index, 0, sizeof(raud->history_index));

redo:

    nb_entries = av_fifo_can_read(raud->fifo[stream_index]);
    for (int n = start_entry; n < nb_entries; n++) {
        for (int si = 0; si < s->nb_streams; si++) {
            av_fifo_peek(raud->fifo[si], &entry, 1, n);

            if (si == stream_index) {
                subblock_index = 0;
                new_pos = entry.block_pos;

                subblock_offset = entry.subblock_offset;

                for (int i = 0; i < entry.nb_subblocks; i++) {
                    if (new_timestamp >= timestamp)
                        break;

                    new_timestamp += SUBBLOCK_SIZE * 2;
                    raud->history_index[si]++;
                    subblock_index++;
                }

                if (new_timestamp >= timestamp) {
                    entry_index = n;
                    break;
                }
            } else {
                raud->history_index[si] += entry.nb_subblocks;
            }
        }

        if (entry_index >= 0)
            break;
    }

    if (entry_index < 0) {
        while (!avio_feof(pb)) {
            int64_t next_pos = (new_pos > 0) ? new_pos + raud->block_size : ffformatcontext(s)->data_offset;
            int64_t seek_info_entry_size;
            int64_t seek_info_offset;
            int64_t seek_info_size;

            avio_seek(pb, next_pos, SEEK_SET);

            seek_info_offset = be ? avio_rb64(pb) : avio_rl64(pb);
            seek_info_size = be ? avio_rb64(pb) : avio_rl64(pb);
            seek_info_entry_size = (seek_info_size - seek_info_offset) / s->nb_streams;

            if (seek_info_offset <= 0 || seek_info_size <= 0)
                return -1;

            for (int si = 0; si < s->nb_streams; si++) {
                const int64_t entry_pos = next_pos + seek_info_offset + seek_info_entry_size * si;
                SubBlocksTabEntry new_entry;
                int new_subblock_offset;
                int subblock_size = 0;
                int subblock_samples;
                int nb_subblocks;

                avio_seek(pb, entry_pos, SEEK_SET);
                new_subblock_offset = be ? avio_rb32(pb) : avio_rl32(pb);
                nb_subblocks = be ? avio_rb32(pb) : avio_rl32(pb);
                avio_skip(pb, 4);
                subblock_samples = be ? avio_rb32(pb) : avio_rl32(pb);

                if (new_subblock_offset < 0 || nb_subblocks <= 0)
                    return -1;

                if (seek_info_entry_size >= 24) {
                    avio_seek(pb, entry_pos + 20, SEEK_SET);
                    subblock_size = be ? avio_rb32(pb) : avio_rl32(pb);

                    if (subblock_size < 0)
                        return -1;
                }

                new_entry.block_pos = next_pos;
                new_entry.nb_subblocks = nb_subblocks;
                new_entry.subblock_size = subblock_size;
                new_entry.subblock_offset = new_subblock_offset;
                new_entry.subblock_samples = subblock_samples;

                av_fifo_write(raud->fifo[si], &new_entry, 1);
            }

            start_entry = av_fifo_can_read(raud->fifo[0]) - 1;

            goto redo;
        }
    }

    raud->stream_index = stream_index;
    raud->subblock_index = subblock_index;

    for (int si = 0; si < s->nb_streams; si++) {
        av_fifo_peek(raud->fifo[si], &entry, 1, entry_index);

        raud->nb_subblocks[si] = entry.nb_subblocks;
        raud->subblock_size[si] = entry.subblock_size;
        raud->subblock_offset[si] = entry.subblock_offset;
    }

    raud->block_pos = new_pos;

    avio_seek(pb, new_pos + (subblock_offset + subblock_index) * 0x800LL, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    RAUDContext *raud = s->priv_data;

    for (int n = 0; n < s->nb_streams; n++)
        av_fifo_freep2(&raud->fifo[n]);

    return 0;
}

const FFInputFormat ff_raud_demuxer = {
    .p.name         = "raud",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Rockstar Games AUD"),
    .priv_data_size = sizeof(RAUDContext),
    .p.extensions   = "ivaud",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
