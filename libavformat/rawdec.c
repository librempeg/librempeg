/*
 * RAW demuxers
 * Copyright (c) 2001 Fabrice Bellard
 * Copyright (c) 2005 Alex Beregszaszi
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

#include "config_components.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "rawdec.h"
#include "libavutil/opt.h"

#define RAW_PACKET_SIZE 1024

int ff_raw_read_partial_packet(AVFormatContext *s, AVPacket *pkt)
{
    FFRawDemuxerContext *raw = s->priv_data;
    int ret, size;

    size = raw->raw_packet_size;

    if ((ret = av_new_packet(pkt, size)) < 0)
        return ret;

    pkt->pos= avio_tell(s->pb);
    pkt->stream_index = 0;
    ret = avio_read_partial(s->pb, pkt->data, size);
    if (ret < 0) {
        av_packet_unref(pkt);
        return ret;
    }
    av_shrink_packet(pkt, ret);
    return ret;
}

int ff_raw_audio_read_header(AVFormatContext *s)
{
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = ffifmt(s->iformat)->raw_codec_id;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    st->start_time = 0;
    /* the parameters will be extracted from the compressed bitstream */

    return 0;
}

/* MPEG-1/H.263 input */
int ff_raw_video_read_header(AVFormatContext *s)
{
    AVStream *st;
    FFStream *sti;
    FFRawVideoDemuxerContext *s1 = s->priv_data;
    int ret = 0;


    st = avformat_new_stream(s, NULL);
    if (!st) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    sti = ffstream(st);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = ffifmt(s->iformat)->raw_codec_id;
    sti->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    st->avg_frame_rate = s1->framerate;
    avpriv_set_pts_info(st, 64, 1, 1200000);

fail:
    return ret;
}

int ff_raw_subtitle_read_header(AVFormatContext *s)
{
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id = ffifmt(s->iformat)->raw_codec_id;
    st->start_time = 0;
    return 0;
}

static int raw_data_read_header(AVFormatContext *s)
{
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    st->codecpar->codec_type = AVMEDIA_TYPE_DATA;
    st->codecpar->codec_id = ffifmt(s->iformat)->raw_codec_id;
    st->start_time = 0;
    return 0;
}

/* Note: Do not forget to add new entries to the Makefile as well. */

#define OFFSET(x) offsetof(FFRawVideoDemuxerContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption rawvideo_options[] = {
    { "framerate", "", OFFSET(framerate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, DEC},
    { "raw_packet_size", "", OFFSET(raw_packet_size), AV_OPT_TYPE_INT, {.i64 = RAW_PACKET_SIZE }, 1, INT_MAX, DEC},
    { NULL },
};
#undef OFFSET

const AVClass ff_rawvideo_demuxer_class = {
    .class_name = "generic raw video demuxer",
    .option     = rawvideo_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#define OFFSET(x) offsetof(FFRawDemuxerContext, x)
static const AVOption raw_options[] = {
    { "raw_packet_size", "", OFFSET(raw_packet_size), AV_OPT_TYPE_INT, {.i64 = RAW_PACKET_SIZE }, 1, INT_MAX, DEC},
    { NULL },
};

const AVClass ff_raw_demuxer_class = {
    .class_name = "generic raw demuxer",
    .option     = raw_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_DATA_DEMUXER
const FFInputFormat ff_data_demuxer = {
    .p.name         = "data",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw data"),
    .p.flags        = AVFMT_NOTIMESTAMPS,
    .p.priv_class   = &ff_raw_demuxer_class,
    .read_header    = raw_data_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_NONE,
    .priv_data_size = sizeof(FFRawDemuxerContext),\
};
#endif

#if CONFIG_MJPEG_DEMUXER
static int mjpeg_probe(const AVProbeData *p)
{
    int i;
    int state = -1;
    int nb_invalid = 0;
    int nb_frames = 0;

    for (i = 0; i < p->buf_size - 1; i++) {
        int c;
        if (p->buf[i] != 0xFF)
            continue;
        c = p->buf[i+1];
        switch (c) {
        case 0xD8:
            state = 0xD8;
            break;
        case 0xC0:
        case 0xC1:
        case 0xC2:
        case 0xC3:
        case 0xC5:
        case 0xC6:
        case 0xC7:
        case 0xF7:
            if (state == 0xD8) {
                state = 0xC0;
            } else
                nb_invalid++;
            break;
        case 0xDA:
            if (state == 0xC0) {
                state = 0xDA;
            } else
                nb_invalid++;
            break;
        case 0xD9:
            if (state == 0xDA) {
                state = 0xD9;
                nb_frames++;
            } else
                nb_invalid++;
            break;
        default:
            if (  (c >= 0x02 && c <= 0xBF)
                || c == 0xC8) {
                nb_invalid++;
            }
        }
    }

    if (nb_invalid*4 + 1 < nb_frames) {
        static const char ct_jpeg[] = "\r\nContent-Type: image/jpeg\r\n";
        int i;

        for (i=0; i<FFMIN(p->buf_size - (int)sizeof(ct_jpeg), 100); i++)
            if (!memcmp(p->buf + i, ct_jpeg, sizeof(ct_jpeg) - 1))
                return AVPROBE_SCORE_EXTENSION;

        if (nb_invalid == 0 && nb_frames > 2)
            return AVPROBE_SCORE_EXTENSION / 2;
        return AVPROBE_SCORE_EXTENSION / 4;
    }
    if (!nb_invalid && nb_frames)
        return AVPROBE_SCORE_EXTENSION / 4;

    return 0;
}

FF_DEF_RAWVIDEO_DEMUXER2(mjpeg, "raw MJPEG video", mjpeg_probe, "mjpg,mjpeg,mpo", AV_CODEC_ID_MJPEG, AVFMT_GENERIC_INDEX|AVFMT_NOTIMESTAMPS)
#endif
