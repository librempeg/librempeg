/*
 * THP Demuxer
 * Copyright (c) 2007 Marco Gerards
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
#include "libavutil/intfloat.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct ThpDemuxContext {
    int          version;
    int64_t      first_frame;
    uint32_t     first_framesz;
    int64_t      next_frame;
    unsigned     next_framesz;
    uint32_t     last_frame;
    int          compoff;
    AVRational   fps;
    int          video_stream_index;
    int          audio_stream_index;
    int          compcount;
    uint8_t      components[16];
    AVStream*    vst;
    int          has_audio;
    AVPacket    *audio_pkt;
} ThpDemuxContext;

static int thp_probe(const AVProbeData *p)
{
    double d;
    /* check file header */
    if (AV_RL32(p->buf) != MKTAG('T', 'H', 'P', '\0'))
        return 0;

    d = av_int2float(AV_RB32(p->buf + 16));
    if (d < 0.1 || d > 1000 || isnan(d))
        return AVPROBE_SCORE_MAX/4;

    return AVPROBE_SCORE_MAX;
}

static int thp_read_header(AVFormatContext *s)
{
    ThpDemuxContext *thp = s->priv_data;
    AVStream *st;
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    uint32_t maxsize, framecnt;

    /* Read the file header.  */
                           avio_rb32(pb); /* Skip Magic.  */
    thp->version         = avio_rb32(pb);

                           avio_rb32(pb); /* Max buf size.  */
                           avio_rb32(pb); /* Max samples.  */

    thp->fps             = av_d2q(av_int2float(avio_rb32(pb)), INT_MAX);
    if (thp->fps.den <= 0 || thp->fps.num < 0)
        return AVERROR_INVALIDDATA;
    framecnt             = avio_rb32(pb);
    thp->next_framesz    =
    thp->first_framesz   = avio_rb32(pb);
    maxsize              = avio_rb32(pb);
    if (fsize > 0 && (!maxsize || fsize < maxsize))
        maxsize = fsize;
    ffiocontext(pb)->maxsize = fsize;

    thp->compoff         = avio_rb32(pb);
                           avio_rb32(pb); /* offsetDataOffset.  */
    thp->next_frame      =
    thp->first_frame     = avio_rb32(pb);
    thp->last_frame      = avio_rb32(pb);

    /* Read the component structure.  */
    avio_seek (pb, thp->compoff, SEEK_SET);
    thp->compcount       = avio_rb32(pb);

    if (thp->compcount > FF_ARRAY_ELEMS(thp->components))
        return AVERROR_INVALIDDATA;

    /* Read the list of component types.  */
    avio_read(pb, thp->components, 16);

    for (int i = 0; i < thp->compcount; i++) {
        if (thp->components[i] == 0) {
            if (thp->vst)
                break;

            /* Video component.  */
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            /* The denominator and numerator are switched because 1/fps
               is required.  */
            avpriv_set_pts_info(st, 64, thp->fps.den, thp->fps.num);
            st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            st->codecpar->codec_id = AV_CODEC_ID_THP;
            st->codecpar->codec_tag = 0;  /* no fourcc */
            st->codecpar->width = avio_rb32(pb);
            st->codecpar->height = avio_rb32(pb);
            st->codecpar->sample_rate = av_q2d(thp->fps);
            st->start_time = 0;
            st->nb_frames = st->duration = framecnt;
            thp->vst = st;
            thp->video_stream_index = st->index;

            if (thp->version == 0x11000)
                avio_rb32(pb); /* Unknown.  */
        } else if (thp->components[i] == 1) {
            if (thp->has_audio != 0)
                break;

            /* Audio component.  */
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_THP;
            st->codecpar->codec_tag = 0;  /* no fourcc */
            st->codecpar->ch_layout.nb_channels = avio_rb32(pb);
            st->codecpar->sample_rate = avio_rb32(pb); /* Frequency.  */
            st->start_time = 0;
            st->duration = avio_rb32(pb);

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

            thp->audio_stream_index = st->index;
            thp->has_audio = 1;
        }
    }

    if (!thp->vst)
        return AVERROR_INVALIDDATA;

    thp->audio_pkt = av_packet_alloc();
    if (!thp->audio_pkt)
        return AVERROR(ENOMEM);

    avio_seek(pb, thp->first_frame, SEEK_SET);

    return 0;
}

static int thp_read_packet(AVFormatContext *s,
                            AVPacket *pkt)
{
    ThpDemuxContext *thp = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (thp->audio_pkt->size == 0) {
        uint32_t size, audio_size = 0;

        if (pos == thp->first_frame) {
            thp->next_frame = thp->first_frame;
            thp->next_framesz = thp->first_framesz;
        } else if (pos != thp->next_frame) {
            int64_t prev_size, next_size;

            avio_skip(pb, 4);
            prev_size = avio_rb32(pb);
            avio_seek(pb, -(prev_size + 8), SEEK_CUR);
            next_size = avio_rb32(pb);

            avio_seek(pb, pos, SEEK_SET);
            thp->next_frame = pos;
            thp->next_framesz = next_size;
        }

        /* Locate the next frame and read out its size.  */
        thp->next_frame += FFMAX(thp->next_framesz, 1);
        thp->next_framesz = avio_rb32(pb);
        avio_rb32(pb); /* Previous total size.  */
        size = avio_rb32(pb); /* Video size of this frame.  */

        if (thp->has_audio)
            audio_size = avio_rb32(pb);

        ret = av_get_packet(pb, pkt, size);
        if (ret < 0)
            return ret;
        if (ret != size)
            return AVERROR(EIO);

        if (audio_size > 0) {
            ret = av_get_packet(pb, thp->audio_pkt, audio_size);
            if (ret < 0)
                return ret;
            thp->audio_pkt->pos = pos;
        }

        avio_skip(pb, thp->next_frame - avio_tell(pb));

        pkt->pos = pos;
        pkt->stream_index = thp->video_stream_index;
    } else {
        av_packet_move_ref(pkt, thp->audio_pkt);
        pkt->stream_index = thp->audio_stream_index;

        if (pkt->size >= 8)
            pkt->duration = AV_RB32(&pkt->data[4]);
    }

    return 0;
}

static int thp_read_seek(AVFormatContext *s, int stream_index,
                         int64_t timestamp, int flags)
{
    ThpDemuxContext *thp = s->priv_data;

    thp->next_frame = 0;
    av_packet_unref(thp->audio_pkt);

    return -1;
}

static int thp_read_close(AVFormatContext *s)
{
    ThpDemuxContext *thp = s->priv_data;

    av_packet_free(&thp->audio_pkt);

    return 0;
}

const FFInputFormat ff_thp_demuxer = {
    .p.name         = "thp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("THP"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(ThpDemuxContext),
    .read_probe     = thp_probe,
    .read_header    = thp_read_header,
    .read_packet    = thp_read_packet,
    .read_seek      = thp_read_seek,
    .read_close     = thp_read_close,
};
