/*
 * codec2 muxer and demuxers
 * Copyright (c) 2017 Tomas Härdin
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

#include "libavcodec/codec2utils.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "avio_internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "mux.h"
#include "rawenc.h"
#include "pcm.h"

#define CODEC2_HEADER_SIZE 7
#define CODEC2_MAGIC       0xC0DEC2

//the lowest version we should ever run across is 0.8
//we may run across later versions as the format evolves
#define EXPECTED_CODEC2_MAJOR_VERSION 0
#define EXPECTED_CODEC2_MINOR_VERSION 8

typedef struct {
    const AVClass *class;
    int mode;
    int frames_per_packet;
} Codec2Context;

static int codec2_probe(const AVProbeData *p)
{
    //must start wih C0 DE C2
    if (AV_RB24(p->buf) != CODEC2_MAGIC) {
        return 0;
    }

    //no .c2 files prior to 0.8
    //be strict about major version while we're at it
    if (p->buf[3] != EXPECTED_CODEC2_MAJOR_VERSION ||
        p->buf[4] <  EXPECTED_CODEC2_MINOR_VERSION) {
        return 0;
    }

    //32 bits of identification -> low score
    return AVPROBE_SCORE_EXTENSION + 1;
}

//Mimics codec2_samples_per_frame()
static int codec2_mode_frame_size(AVFormatContext *s, int mode)
{
    int frame_size_table[CODEC2_MODE_MAX+1] = {
        160,    // 3200
        160,    // 2400
        320,    // 1600
        320,    // 1400
        320,    // 1300
        320,    // 1200
        320,    // 700
        320,    // 700B
        320,    // 700C
    };

    if (mode < 0 || mode > CODEC2_MODE_MAX) {
        av_log(s, AV_LOG_ERROR, "unknown codec2 mode %i, can't find frame_size\n", mode);
        return 0;
    } else {
        return frame_size_table[mode];
    }
}

//Mimics (codec2_bits_per_frame()+7)/8
static int codec2_mode_block_align(AVFormatContext *s, int mode)
{
    int block_align_table[CODEC2_MODE_MAX+1] = {
        8,      // 3200
        6,      // 2400
        8,      // 1600
        7,      // 1400
        7,      // 1300
        6,      // 1200
        4,      // 700
        4,      // 700B
        4,      // 700C
    };

    if (mode < 0 || mode > CODEC2_MODE_MAX) {
        av_log(s, AV_LOG_ERROR, "unknown codec2 mode %i, can't find block_align\n", mode);
        return 0;
    } else {
        return block_align_table[mode];
    }
}

//Computes bitrate from mode, with frames rounded up to the nearest octet.
//So 700 bit/s (28 bits/frame) becomes 800 bits/s (32 bits/frame).
static int codec2_mode_bit_rate(AVFormatContext *s, int mode)
{
    int frame_size  = codec2_mode_frame_size(s, mode);
    int block_align = codec2_mode_block_align(s, mode);

    if (frame_size <= 0 || block_align <= 0) {
        return 0;
    }

    return 8 * 8000 * block_align / frame_size;
}

static int codec2_read_header_common(AVFormatContext *s, AVStream *st)
{
    int mode = codec2_mode_from_extradata(st->codecpar->extradata);

    st->codecpar->codec_type        = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id          = AV_CODEC_ID_CODEC2;
    st->codecpar->sample_rate       = 8000;
    st->codecpar->format            = AV_SAMPLE_FMT_S16;
    st->codecpar->ch_layout         = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    st->codecpar->bit_rate          = codec2_mode_bit_rate(s, mode);
    st->codecpar->frame_size        = codec2_mode_frame_size(s, mode);
    st->codecpar->block_align       = codec2_mode_block_align(s, mode);

    if (st->codecpar->bit_rate <= 0 ||
        st->codecpar->frame_size <= 0 ||
        st->codecpar->block_align <= 0) {
        return AVERROR_INVALIDDATA;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int codec2_read_header(AVFormatContext *s)
{
    AVStream *st = avformat_new_stream(s, NULL);
    int ret, version;

    if (!st) {
        return AVERROR(ENOMEM);
    }

    if (avio_rb24(s->pb) != CODEC2_MAGIC) {
        av_log(s, AV_LOG_ERROR, "not a .c2 file\n");
        return AVERROR_INVALIDDATA;
    }

    ret = ff_alloc_extradata(st->codecpar, CODEC2_EXTRADATA_SIZE);
    if (ret) {
        return ret;
    }

    ret = ffio_read_size(s->pb, st->codecpar->extradata, CODEC2_EXTRADATA_SIZE);
    if (ret < 0) {
        return ret;
    }

    version = AV_RB16(st->codecpar->extradata);
    if ((version >> 8) != EXPECTED_CODEC2_MAJOR_VERSION) {
        avpriv_report_missing_feature(s, "Major version %i", version >> 8);
        return AVERROR_PATCHWELCOME;
    }

    ffformatcontext(s)->data_offset = CODEC2_HEADER_SIZE;

    return codec2_read_header_common(s, st);
}

static int codec2_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    Codec2Context *c2 = s->priv_data;
    AVStream *st = s->streams[0];
    int ret, size, n, block_align, frame_size;

    block_align = st->codecpar->block_align;
    frame_size  = st->codecpar->frame_size;

    if (block_align <= 0 || frame_size <= 0 || c2->frames_per_packet <= 0) {
        return AVERROR(EINVAL);
    }

    //try to read desired number of frames, compute n from to actual number of bytes read
    size = c2->frames_per_packet * block_align;
    ret = av_get_packet(s->pb, pkt, size);
    if (ret < 0) {
        return ret;
    }

    //only set duration - compute_pkt_fields() and ff_pcm_read_seek() takes care of everything else
    //tested by spamming the seek functionality in ffplay
    n = ret / block_align;
    pkt->duration = n * frame_size;

    return ret;
}

static int codec2_write_header(AVFormatContext *s)
{
    AVStream *st = s->streams[0];

    if (st->codecpar->extradata_size != CODEC2_EXTRADATA_SIZE) {
        av_log(s, AV_LOG_ERROR, ".c2 files require exactly %i bytes of extradata (got %i)\n",
               CODEC2_EXTRADATA_SIZE, st->codecpar->extradata_size);
        return AVERROR(EINVAL);
    }

    avio_wb24(s->pb, CODEC2_MAGIC);
    avio_write(s->pb, st->codecpar->extradata, CODEC2_EXTRADATA_SIZE);

    return 0;
}

static int codec2raw_read_header(AVFormatContext *s)
{
    Codec2Context *c2 = s->priv_data;
    AVStream *st;
    int ret;

    if (c2->mode < 0) {
        //FIXME: using a default value of -1 for mandatory options is an incredibly ugly hack
        av_log(s, AV_LOG_ERROR, "-mode must be set in order to make sense of raw codec2 files\n");
        return AVERROR(EINVAL);
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        return AVERROR(ENOMEM);
    }

    ret = ff_alloc_extradata(st->codecpar, CODEC2_EXTRADATA_SIZE);
    if (ret) {
        return ret;
    }

    codec2_make_extradata(st->codecpar->extradata, c2->mode);

    return codec2_read_header_common(s, st);
}

//transcoding report2074.c2 to wav went from 7.391s to 5.322s with -frames_per_packet 1000 compared to default, same sha1sum
#define FRAMES_PER_PACKET \
    { "frames_per_packet", "Number of frames to read at a time. Higher = faster decoding, lower granularity", \
      offsetof(Codec2Context, frames_per_packet), AV_OPT_TYPE_INT, {.i64 = 1}, 1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM}

static const AVOption codec2_options[] = {
    FRAMES_PER_PACKET,
    { NULL },
};

static const AVOption codec2raw_options[] = {
    CODEC2_AVOPTIONS("codec2 mode [mandatory]", Codec2Context, -1, -1, AV_OPT_FLAG_DECODING_PARAM),
    FRAMES_PER_PACKET,
    { NULL },
};

static const AVClass codec2_demux_class = {
    .class_name = "codec2 demuxer",
    .option     = codec2_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEMUXER,
};

static const AVClass codec2raw_demux_class = {
    .class_name = "codec2raw demuxer",
    .option     = codec2raw_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEMUXER,
};

#if CONFIG_CODEC2_DEMUXER
const FFInputFormat ff_codec2_demuxer = {
    .p.name         = "codec2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("codec2 .c2 demuxer"),
    .p.extensions   = "c2",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &codec2_demux_class,
    .priv_data_size = sizeof(Codec2Context),
    .read_probe     = codec2_probe,
    .read_header    = codec2_read_header,
    .read_packet    = codec2_read_packet,
    .read_seek      = ff_pcm_read_seek,
    .raw_codec_id   = AV_CODEC_ID_CODEC2,
};
#endif

#if CONFIG_CODEC2_MUXER
const FFOutputFormat ff_codec2_muxer = {
    .p.name         = "codec2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("codec2 .c2 muxer"),
    .p.extensions   = "c2",
    .p.audio_codec  = AV_CODEC_ID_CODEC2,
    .p.video_codec  = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .p.flags        = AVFMT_NOTIMESTAMPS,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
    .write_header   = codec2_write_header,
    .write_packet   = ff_raw_write_packet,
};
#endif

#if CONFIG_CODEC2RAW_DEMUXER
const FFInputFormat ff_codec2raw_demuxer = {
    .p.name         = "codec2raw",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw codec2 demuxer"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &codec2raw_demux_class,
    .priv_data_size = sizeof(Codec2Context),
    .read_header    = codec2raw_read_header,
    .read_packet    = codec2_read_packet,
    .read_seek      = ff_pcm_read_seek,
    .raw_codec_id   = AV_CODEC_ID_CODEC2,
};
#endif
