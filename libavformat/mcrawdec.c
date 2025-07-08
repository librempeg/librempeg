/*
 * MotionCam .mcraw demuxer
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

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"

#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

static int mcraw_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t type, size, magic, num_offsets;
    int sample_rate = 0, channels = 0;
    AVDictionaryEntry *entry;
    int64_t start_offset, pos;
    AVStream *vst, *ast;
    char *meta;
    int ret;

    avio_skip(pb, 8);
    type = avio_rl32(pb);
    if (type != 3)
        return AVERROR_INVALIDDATA;
    size = avio_rl32(pb);

    meta = av_calloc(size, sizeof(*meta));
    if (!meta)
        return AVERROR(ENOMEM);

    ret = avio_get_str(pb, size, meta, size);
    if (ret < 0)
        return ret;

    ret = av_dict_parse_string(&s->metadata, meta, ":", ",", 0);
    if (ret < 0)
        return ret;

    av_freep(&meta);

    pos = avio_tell(pb);

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_MCRAW;
    avpriv_set_pts_info(vst, 64, 1, 1000000000);

    entry = av_dict_get(s->metadata, "\"audioSampleRate\"", NULL, AV_DICT_IGNORE_SUFFIX);
    if (entry && entry->value)
        sample_rate = atoi(entry->value);
    entry = av_dict_get(s->metadata, "\"audioChannels\"", NULL, AV_DICT_IGNORE_SUFFIX);
    if (entry && entry->value)
        channels = atoi(entry->value);

    if (sample_rate > 0 && channels > 0) {
        ast = avformat_new_stream(s, NULL);
        if (!ast)
            return AVERROR(ENOMEM);

        ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        ast->codecpar->sample_rate = sample_rate;
        ast->codecpar->ch_layout.nb_channels = channels;
        avpriv_set_pts_info(ast, 64, 1, 1000000000);
    }

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        ret = avio_seek(pb, avio_size(pb) - 24, SEEK_SET);
        if (ret < 0)
            goto end;
        type = avio_rl32(pb);
        size = avio_rl32(pb);
        magic = avio_rl32(pb);
        if (magic != 0x8A905612)
            goto end;
        if (type)
            goto end;
        if (size != 16)
            goto end;

        num_offsets = avio_rl32(pb);
        start_offset = avio_rl64(pb);

        ret = avio_seek(pb, start_offset, SEEK_SET);
        if (ret < 0)
            goto end;

        for (unsigned n = 0; n < num_offsets; n++) {
            int64_t offset = avio_rl64(pb);
            int64_t pts = avio_rl64(pb);

            ret = av_add_index_entry(vst, offset, pts, 0, 0, 0);
            if (ret < 0)
                return ret;
        }

end:
        avio_seek(pb, pos, SEEK_SET);
    }

    return 0;
}

static int mcraw_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t type = avio_rl32(pb);
    uint32_t size = avio_rl32(pb);
    int ret;

    if (size == 0 || avio_feof(pb))
        return AVERROR_EOF;

    while (type != 2 && type != 5) {
        avio_skip(pb, size);

        type = avio_rl32(pb);
        size = avio_rl32(pb);
        if (size == 0 || avio_feof(pb))
            return AVERROR_EOF;
    }

    ret = av_get_packet(pb, pkt, size);
    if (ret < 0)
        return ret;
    pkt->pos = pos;

    pkt->stream_index = (type == 5 && s->nb_streams > 0) ? 1 : 0;
    if (type == 5) {
        ret = ffio_ensure_seekback(pb, 16);
        if (ret < 0)
            return ret;

        type = avio_rl32(pb);
        if (type == 6) {
            size = avio_rl32(pb);
            if (size == 8) {
                pkt->pts = avio_rl64(pb);
            } else {
                avio_seek(pb, -8, SEEK_CUR);
            }
        } else {
            avio_seek(pb, -4, SEEK_CUR);
        }
    } else if (type == 2) {
        ret = ffio_ensure_seekback(pb, 8);
        if (ret < 0)
            return ret;

        type = avio_rl32(pb);
        size = avio_rl32(pb);
        if (type == 3) {
            const int offset = pkt->size;
            char *pts_str = NULL;

            ret = av_append_packet(pb, pkt, size);
            if (ret < 0)
                return ret;
            pts_str = av_strnstr(pkt->data + offset, "\"timestamp\":\"", pkt->size-offset);
            if (pts_str)
                pkt->pts = atol(pts_str + 13);
        } else {
            avio_seek(pb, -8, SEEK_CUR);
        }
        pkt->duration = 1;
    }

    return 0;
}

static int mcraw_probe(const AVProbeData *pd)
{
    if (strncmp(pd->buf, "MOTION ", 7))
        return 0;
    if (pd->buf[7] != 3)
        return 0;
    if (AV_RL32(&pd->buf[8]) != 3)
        return 0;
    return AVPROBE_SCORE_MAX;
}

const FFInputFormat ff_mcraw_demuxer = {
    .p.name         = "mcraw",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MotionCam RAW"),
    .p.extensions   = "mcraw",
    .p.flags        = AVFMT_GENERIC_INDEX | AVFMT_NO_BYTE_SEEK | AVFMT_TS_DISCONT | AVFMT_TS_NONSTRICT | AVFMT_SEEK_TO_PTS,
    .read_probe     = mcraw_probe,
    .read_header    = mcraw_read_header,
    .read_packet    = mcraw_read_packet,
};
