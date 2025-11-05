/*
 * BAKA demuxer
 * Copyright (c) 2025 smiRaphi
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
#include "demux.h"
#include "internal.h"

typedef struct BakaDemuxContext {
    int64_t data_end;
} BakaDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('B','A','K','A') ||
        AV_RL32(p->buf+4) == 0 ||
        AV_RL32(p->buf+8) != MKTAG(' ','A','H','O'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    BakaDemuxContext *bc = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t data_offset = 0;
    AVStream *st;

    if (avio_rl32(pb) != MKTAG('B','A','K','A'))
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    if (avio_rl32(pb) != MKTAG(' ','A','H','O'))
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
    st->codecpar->sample_rate = 44100;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->block_align = 2 * 2 * 512;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    while (!avio_feof(pb)) {
        int64_t next_off;
        uint32_t tag;
        int64_t size;

        tag = avio_rl32(pb);
        size = avio_rb32(pb);
        if (avio_feof(pb))
            break;
        next_off = avio_tell(pb) + size;

        switch (tag) {
        case MKTAG('P','A','P','A'):
            avio_skip(pb, 2);
            st->duration = avio_rb32(pb);
            break;
        case MKTAG('M','A','M','A'):
            if (data_offset) {
                av_log(s, AV_LOG_WARNING, "Multiple MAMA (DATA) tags found\n");
            } else {
                data_offset = avio_tell(pb);
                bc->data_end = data_offset + size;
            }
            break;
        case MKTAG('M','A','R','K'):
            {
                int16_t count = avio_rb16(pb);

                for (int n = 0; n < count; n++) {
                    int64_t mark;
                    uint8_t name_length;
                    char *name;

                    avio_skip(pb, 2);
                    mark = avio_rb32(pb) * st->codecpar->ch_layout.nb_channels;

                    name_length = avio_r8(pb);
                    name = av_mallocz(name_length + 1);
                    if (!name)
                        return AVERROR(ENOMEM);
                    avio_read(pb, name, name_length);
                    name[name_length] = 0;

                    if (!strcmp(name, "beg loop")) {
                        if (mark > 0)
                            av_dict_set_int(&st->metadata, "loop_start", mark, 0);
                    } else if (!strcmp(name, "end loop")) {
                        if (mark > 0 && (!st->duration || mark < st->duration))
                            av_dict_set_int(&st->metadata, "loop_end", mark, 0);
                    } else {
                        avpriv_new_chapter(s, n, st->time_base, mark, AV_NOPTS_VALUE, name);
                    }

                    av_free(name);
                    avio_skip(pb, 1);
                }
            }
            break;
        default:
            av_log(s, AV_LOG_DEBUG, "Unknown tag(%08X) found\n", tag);
            break;
        }

        avio_seek(pb, next_off, SEEK_SET);
    }

    if (data_offset <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, data_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    BakaDemuxContext *bc = s->priv_data;
    AVStream *st = s->streams[0];
    int block_align = st->codecpar->block_align;
    int64_t pos = avio_tell(s->pb);

    if (pos >= bc->data_end)
        return AVERROR_EOF;

    return av_get_packet(s->pb, pkt, FFMIN(block_align, bc->data_end - pos));
}

const FFInputFormat ff_baka_demuxer = {
    .p.name         = "baka",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BAKA (KCE Tokyo)"),
    .priv_data_size = sizeof(BakaDemuxContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
