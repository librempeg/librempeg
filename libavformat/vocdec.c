/*
 * Creative Voice File demuxer.
 * Copyright (c) 2006  Aurelien Jacobs <aurel@gnuage.org>
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
#include "demux.h"
#include "voc.h"
#include "internal.h"

static int voc_probe(const AVProbeData *p)
{
    int version, check;

    if (memcmp(p->buf, ff_voc_magic, sizeof(ff_voc_magic) - 1))
        return 0;
    version = AV_RL16(p->buf + 22);
    check = AV_RL16(p->buf + 24);
    if (~version + 0x1234 != check)
        return 10;

    return AVPROBE_SCORE_MAX;
}

static int voc_read_header(AVFormatContext *s)
{
    VocDecContext *voc = s->priv_data;
    AVIOContext *pb = s->pb;
    int header_size;

    avio_skip(pb, 20);
    header_size = avio_rl16(pb) - 22;
    if (header_size != 4) {
        av_log(s, AV_LOG_ERROR, "unknown header size: %d\n", header_size);
        return AVERROR(ENOSYS);
    }
    avio_skip(pb, header_size);

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    voc->remaining_size = 0;
    return 0;
}

static int voc_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (!s->nb_streams) {
        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    }
    return ff_voc_get_packet(s, pkt, s->streams[0], 0);
}

static int voc_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    VocDecContext *voc = s->priv_data;
    AVStream *st;
    FFStream *sti;
    int index;

    if (s->nb_streams < 1) {
        av_log(s, AV_LOG_ERROR, "cannot seek while no stream was found yet\n");
        return AVERROR(EINVAL);
    }

    st = s->streams[stream_index];
    sti = ffstream(st);
    index = av_index_search_timestamp(st, timestamp, flags);

    if (index >= 0 && index < sti->nb_index_entries - 1) {
        const AVIndexEntry *const e = &sti->index_entries[index];
        avio_seek(s->pb, e->pos, SEEK_SET);
        voc->pts = e->timestamp;
        voc->remaining_size = e->size;
        return 0;
    } else if (sti->nb_index_entries && sti->index_entries[0].timestamp <= timestamp) {
        const AVIndexEntry *const e = &sti->index_entries[sti->nb_index_entries - 1];
        // prepare context for seek_frame_generic()
        voc->pts = e->timestamp;
        voc->remaining_size = e->size;
    }
    return -1;
}

const FFInputFormat ff_voc_demuxer = {
    .p.name         = "voc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Creative Voice"),
    .p.codec_tag    = ff_voc_codec_tags_list,
    .priv_data_size = sizeof(VocDecContext),
    .read_probe     = voc_probe,
    .read_header    = voc_read_header,
    .read_packet    = voc_read_packet,
    .read_seek      = voc_read_seek,
};
