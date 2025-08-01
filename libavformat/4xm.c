/*
 * 4X Technologies .4xm File Demuxer (no muxer)
 * Copyright (c) 2003  The FFmpeg project
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
 * 4X Technologies file demuxer
 * by Mike Melanson (melanson@pcisys.net)
 * for more information on the .4xm file format, visit:
 *   http://www.pcisys.net/~melanson/codecs/
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/intfloat.h"
#include "libavutil/mem.h"
#include "libavcodec/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define     RIFF_TAG MKTAG('R', 'I', 'F', 'F')
#define  FOURXMV_TAG MKTAG('4', 'X', 'M', 'V')
#define     LIST_TAG MKTAG('L', 'I', 'S', 'T')
#define     HEAD_TAG MKTAG('H', 'E', 'A', 'D')
#define     TRK__TAG MKTAG('T', 'R', 'K', '_')
#define     MOVI_TAG MKTAG('M', 'O', 'V', 'I')
#define     VTRK_TAG MKTAG('V', 'T', 'R', 'K')
#define     STRK_TAG MKTAG('S', 'T', 'R', 'K')
#define     std__TAG MKTAG('s', 't', 'd', '_')
#define     name_TAG MKTAG('n', 'a', 'm', 'e')
#define     vtrk_TAG MKTAG('v', 't', 'r', 'k')
#define     strk_TAG MKTAG('s', 't', 'r', 'k')
#define     ifrm_TAG MKTAG('i', 'f', 'r', 'm')
#define     pfrm_TAG MKTAG('p', 'f', 'r', 'm')
#define     cfrm_TAG MKTAG('c', 'f', 'r', 'm')
#define     ifr2_TAG MKTAG('i', 'f', 'r', '2')
#define     pfr2_TAG MKTAG('p', 'f', 'r', '2')
#define     cfr2_TAG MKTAG('c', 'f', 'r', '2')
#define     snd__TAG MKTAG('s', 'n', 'd', '_')

#define vtrk_SIZE 0x44
#define strk_SIZE 0x28

#define GET_LIST_HEADER() \
    fourcc_tag = avio_rl32(pb); \
    size       = avio_rl32(pb); \
    if (fourcc_tag != LIST_TAG) { \
        ret = AVERROR_INVALIDDATA; \
        goto fail; \
    } \
    fourcc_tag = avio_rl32(pb);

typedef struct AudioTrack {
    int sample_rate;
    int bits;
    int channels;
    int stream_index;
    int adpcm;
} AudioTrack;

typedef struct FourxmDemuxContext {
    int video_stream_index;
    int track_count;
    AudioTrack *tracks;

    AVRational fps;
} FourxmDemuxContext;

static int fourxm_probe(const AVProbeData *p)
{
    if ((AV_RL32(&p->buf[0]) != RIFF_TAG) ||
        (AV_RL32(&p->buf[8]) != FOURXMV_TAG))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int parse_vtrk(AVFormatContext *s,
                      FourxmDemuxContext *fourxm, uint8_t *buf, int size,
                      int left)
{
    AVStream *st;
    /* check that there is enough data */
    if (size != vtrk_SIZE || left < size + 8) {
        return AVERROR_INVALIDDATA;
    }

    /* allocate a new AVStream */
    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avpriv_set_pts_info(st, 64, fourxm->fps.den, fourxm->fps.num*2);

    fourxm->video_stream_index = st->index;

    st->codecpar->codec_type     = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id       = AV_CODEC_ID_4XM;

    st->codecpar->extradata      = av_mallocz(4 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!st->codecpar->extradata)
        return AVERROR(ENOMEM);
    st->codecpar->extradata_size = 4;
    AV_WL32(st->codecpar->extradata, AV_RL32(buf + 16));
    st->codecpar->width  = AV_RL32(buf + 36);
    st->codecpar->height = AV_RL32(buf + 40);

    return 0;
}


static int parse_strk(AVFormatContext *s,
                      FourxmDemuxContext *fourxm, uint8_t *buf, int size,
                      int left)
{
    AVStream *st;
    int track;
    /* check that there is enough data */
    if (size != strk_SIZE || left < size + 8)
        return AVERROR_INVALIDDATA;

    track = AV_RL32(buf + 8);
    if ((unsigned)track >= UINT_MAX / sizeof(AudioTrack) - 1 ||
        track >= s->max_streams) {
        av_log(s, AV_LOG_ERROR, "current_track too large\n");
        return AVERROR_INVALIDDATA;
    }

    if (track + 1 > fourxm->track_count) {
        if (av_reallocp_array(&fourxm->tracks, track + 1, sizeof(AudioTrack)))
            return AVERROR(ENOMEM);
        memset(&fourxm->tracks[fourxm->track_count], 0,
               sizeof(AudioTrack) * (track + 1 - fourxm->track_count));
        fourxm->track_count = track + 1;
    } else {
        if (fourxm->tracks[track].bits)
            return AVERROR_INVALIDDATA;
    }
    fourxm->tracks[track].adpcm       = AV_RL32(buf + 12);
    fourxm->tracks[track].channels    = AV_RL32(buf + 36);
    fourxm->tracks[track].sample_rate = AV_RL32(buf + 40);
    fourxm->tracks[track].bits        = AV_RL32(buf + 44);

    if (fourxm->tracks[track].channels    <= 0 ||
        fourxm->tracks[track].channels     > FF_SANE_NB_CHANNELS ||
        fourxm->tracks[track].sample_rate <= 0 ||
        fourxm->tracks[track].bits        <= 0 ||
        fourxm->tracks[track].bits         > INT_MAX / FF_SANE_NB_CHANNELS) {
        av_log(s, AV_LOG_ERROR, "audio header invalid\n");
        return AVERROR_INVALIDDATA;
    }
    if (!fourxm->tracks[track].adpcm && fourxm->tracks[track].bits<8) {
        av_log(s, AV_LOG_ERROR, "bits unspecified for non ADPCM\n");
        return AVERROR_INVALIDDATA;
    }

    if (fourxm->tracks[track].sample_rate > INT64_MAX / fourxm->tracks[track].bits / fourxm->tracks[track].channels) {
        av_log(s, AV_LOG_ERROR, "Overflow during bit rate calculation %d * %d * %d\n",
               fourxm->tracks[track].sample_rate, fourxm->tracks[track].bits, fourxm->tracks[track].channels);
        return AVERROR_INVALIDDATA;
    }

    /* allocate a new AVStream */
    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = track;
    avpriv_set_pts_info(st, 64, 1, fourxm->tracks[track].sample_rate);

    fourxm->tracks[track].stream_index = st->index;

    st->codecpar->codec_type            = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_tag             = 0;
    st->codecpar->ch_layout.nb_channels = fourxm->tracks[track].channels;
    st->codecpar->sample_rate           = fourxm->tracks[track].sample_rate;
    st->codecpar->bits_per_coded_sample = fourxm->tracks[track].bits;
    st->codecpar->bit_rate              = (int64_t)st->codecpar->ch_layout.nb_channels *
                                          st->codecpar->sample_rate *
                                          st->codecpar->bits_per_coded_sample;
    st->codecpar->block_align           = st->codecpar->ch_layout.nb_channels *
                                          st->codecpar->bits_per_coded_sample;

    if (fourxm->tracks[track].adpcm){
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_4XM;
    } else if (st->codecpar->bits_per_coded_sample == 8) {
        st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;
    } else
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;

    return 0;
}

static int fourxm_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned int fourcc_tag;
    unsigned int size;
    int header_size;
    FourxmDemuxContext *fourxm = s->priv_data;
    unsigned char *header = NULL;
    int i, ret;

    fourxm->track_count = 0;
    fourxm->tracks      = NULL;
    fourxm->fps         = (AVRational){1,1};
    fourxm->video_stream_index = -1;

    /* skip the first 3 32-bit numbers */
    avio_skip(pb, 12);

    /* check for LIST-HEAD */
    GET_LIST_HEADER();
    header_size = size - 4;
    if (fourcc_tag != HEAD_TAG || header_size < 0)
        return AVERROR_INVALIDDATA;

    /* allocate space for the header and load the whole thing */
    header = av_malloc(header_size);
    if (!header)
        return AVERROR(ENOMEM);
    if (avio_read(pb, header, header_size) != header_size) {
        av_free(header);
        return AVERROR(EIO);
    }

    /* take the lazy approach and search for any and all vtrk and strk chunks */
    for (i = 0; i < header_size - 8; i++) {
        fourcc_tag = AV_RL32(&header[i]);
        size       = AV_RL32(&header[i + 4]);
        if (size > header_size - i - 8 && (fourcc_tag == vtrk_TAG || fourcc_tag == strk_TAG)) {
            av_log(s, AV_LOG_ERROR, "chunk larger than array %d>%d\n", size, header_size - i - 8);
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        if (fourcc_tag == std__TAG) {
            if (header_size - i < 16) {
                av_log(s, AV_LOG_ERROR, "std TAG truncated\n");
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            fourxm->fps = av_d2q(av_int2float(AV_RL32(&header[i + 12])), 10000);
        } else if (fourcc_tag == vtrk_TAG) {
            if ((ret = parse_vtrk(s, fourxm, header + i, size,
                                  header_size - i)) < 0)
                goto fail;

            i += 8 + size;
        } else if (fourcc_tag == strk_TAG) {
            if ((ret = parse_strk(s, fourxm, header + i, size,
                                  header_size - i)) < 0)
                goto fail;

            i += 8 + size;
        }
    }

    /* skip over the LIST-MOVI chunk (which is where the stream should be */
    GET_LIST_HEADER();
    if (fourcc_tag != MOVI_TAG) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }

    av_free(header);

    return 0;
fail:
    av_free(header);
    return ret;
}

static int fourxm_read_packet(AVFormatContext *s,
                              AVPacket *pkt)
{
    FourxmDemuxContext *fourxm = s->priv_data;
    AVIOContext *pb            = s->pb;
    unsigned int fourcc_tag;
    unsigned int size;
    int ret = 0;
    unsigned int track_number;
    int packet_read = 0;
    unsigned char header[8];
    int64_t audio_frame_count;
    int64_t pos = avio_tell(pb);

    while (!packet_read) {
        if ((ret = avio_read(s->pb, header, 8)) < 0)
            return ret;
        fourcc_tag = AV_RL32(&header[0]);
        size       = AV_RL32(&header[4]);
        if (avio_feof(pb))
            return AVERROR(EIO);
        switch (fourcc_tag) {
        case LIST_TAG:
            /* skip the LIST-* tag and move on to the next fourcc */
            avio_rl32(pb);
            break;

        case ifrm_TAG:
        case pfrm_TAG:
        case cfrm_TAG:
        case ifr2_TAG:
        case pfr2_TAG:
        case cfr2_TAG:
            /* allocate 8 more bytes than 'size' to account for fourcc
             * and size */
            if (size > INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE - 8)
                return AVERROR_INVALIDDATA;
            if (fourxm->video_stream_index < 0)
                return AVERROR_INVALIDDATA;
            if ((ret = av_new_packet(pkt, size + 8)) < 0)
                return ret;
            pkt->stream_index = fourxm->video_stream_index;
            pkt->duration     = 1;
            pkt->pos          = pos;
            memcpy(pkt->data, header, 8);
            ret = avio_read(s->pb, &pkt->data[8], size);

            if (ret < 0) {
                av_packet_unref(pkt);
            } else {
                packet_read = 1;
                av_shrink_packet(pkt, ret + 8);
                switch (fourcc_tag) {
                case ifrm_TAG:
                case ifr2_TAG:
                    pkt->flags |= AV_PKT_FLAG_KEY;
                    break;
                default:
                    break;
                }
            }
            break;

        case snd__TAG:
            track_number = avio_rl32(pb);
            avio_skip(pb, 4);
            size -= 8;

            if (track_number < fourxm->track_count &&
                fourxm->tracks[track_number].channels > 0) {
                ret = av_get_packet(s->pb, pkt, size);
                if (ret < 0)
                    return ret;
                pkt->stream_index =
                    fourxm->tracks[track_number].stream_index;
                packet_read = 1;

                /* pts accounting */
                audio_frame_count = size;
                if (fourxm->tracks[track_number].adpcm)
                    audio_frame_count -= 4 * (fourxm->tracks[track_number].channels);
                audio_frame_count /= fourxm->tracks[track_number].channels;
                if (fourxm->tracks[track_number].adpcm) {
                    audio_frame_count *= 2;
                } else
                    audio_frame_count /=
                        (fourxm->tracks[track_number].bits / 8);
                pkt->duration = audio_frame_count;
                pkt->flags   |= AV_PKT_FLAG_KEY;
                pkt->pos      = pos;
            } else {
                avio_skip(pb, size);
            }
            break;

        default:
            avio_skip(pb, size);
            break;
        }
    }
    return ret;
}

static int fourxm_read_close(AVFormatContext *s)
{
    FourxmDemuxContext *fourxm = s->priv_data;

    av_freep(&fourxm->tracks);

    return 0;
}

const FFInputFormat ff_fourxm_demuxer = {
    .p.name         = "4xm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("4X Technologies"),
    .priv_data_size = sizeof(FourxmDemuxContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = fourxm_probe,
    .read_header    = fourxm_read_header,
    .read_packet    = fourxm_read_packet,
    .read_close     = fourxm_read_close,
};
