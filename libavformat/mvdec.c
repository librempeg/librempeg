/*
 * Silicon Graphics Movie demuxer
 * Copyright (c) 2012 Peter Ross
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
 * Silicon Graphics Movie demuxer
 */

#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/intfloat.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/rational.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct MvContext {
    int nb_video_tracks;
    int nb_audio_tracks;

    int eof_count;        ///< number of streams that have finished
    int stream_index;     ///< current stream index
    int frame[2];         ///< frame nb for current stream

    int acompression;     ///< compression level for audio stream
    int aformat;          ///< audio format
} MvContext;

/* these magic numbers are defined in moviefile.h on Silicon Grahpics IRIX */
#define MOVIE_SOUND  1
#define MOVIE_SILENT 2

#define AUDIO_FORMAT_SIGNED 401

static int mv_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) == MKBETAG('M', 'O', 'V', 'I') &&
        AV_RB16(p->buf + 4) < 3)
        return AVPROBE_SCORE_MAX;
    return 0;
}

static char *var_read_string(AVIOContext *pb, int size)
{
    int n;
    char *str;

    if (size < 0 || size == INT_MAX)
        return NULL;

    str = av_malloc(size + 1);
    if (!str)
        return NULL;
    n = avio_get_str(pb, size, str, size + 1);
    if (n < size)
        avio_skip(pb, size - n);
    return str;
}

static int var_read_int(AVIOContext *pb, int size)
{
    int v;
    char *s = var_read_string(pb, size);
    if (!s)
        return 0;
    v = strtol(s, NULL, 10);
    av_free(s);
    return v;
}

static AVRational var_read_float(AVIOContext *pb, int size)
{
    AVRational v;
    char *s = var_read_string(pb, size);
    if (!s)
        return (AVRational) { 0, 0 };
    v = av_d2q(av_strtod(s, NULL), INT_MAX);
    av_free(s);
    return v;
}

static void var_read_metadata(AVFormatContext *avctx, const char *tag, int size)
{
    char *value = var_read_string(avctx->pb, size);
    if (value)
        av_dict_set(&avctx->metadata, tag, value, AV_DICT_DONT_STRDUP_VAL);
}

static int set_channels(AVFormatContext *avctx, AVStream *st, int channels)
{
    if (channels <= 0) {
        av_log(avctx, AV_LOG_ERROR, "Channel count %d invalid.\n", channels);
        return AVERROR_INVALIDDATA;
    }
    av_channel_layout_default(&st->codecpar->ch_layout, channels);
    return 0;
}

/**
 * Parse global variable
 * @return < 0 if unknown
 */
static int parse_global_var(AVFormatContext *avctx, AVStream *st,
                            const char *name, int size)
{
    MvContext *mv = avctx->priv_data;
    AVIOContext *pb = avctx->pb;
    if (!strcmp(name, "__NUM_I_TRACKS")) {
        mv->nb_video_tracks = var_read_int(pb, size);
    } else if (!strcmp(name, "__NUM_A_TRACKS")) {
        mv->nb_audio_tracks = var_read_int(pb, size);
    } else if (!strcmp(name, "COMMENT") || !strcmp(name, "TITLE")) {
        var_read_metadata(avctx, name, size);
    } else if (!strcmp(name, "LOOP_MODE") || !strcmp(name, "NUM_LOOPS") ||
               !strcmp(name, "OPTIMIZED")) {
        avio_skip(pb, size); // ignore
    } else
        return AVERROR_INVALIDDATA;

    return 0;
}

/**
 * Parse audio variable
 * @return < 0 if unknown
 */
static int parse_audio_var(AVFormatContext *avctx, AVStream *st,
                           const char *name, int size)
{
    MvContext *mv = avctx->priv_data;
    AVIOContext *pb = avctx->pb;
    if (!strcmp(name, "__DIR_COUNT")) {
        st->nb_frames = var_read_int(pb, size);
    } else if (!strcmp(name, "AUDIO_FORMAT")) {
        mv->aformat = var_read_int(pb, size);
    } else if (!strcmp(name, "COMPRESSION")) {
        mv->acompression = var_read_int(pb, size);
    } else if (!strcmp(name, "DEFAULT_VOL")) {
        var_read_metadata(avctx, name, size);
    } else if (!strcmp(name, "NUM_CHANNELS")) {
        return set_channels(avctx, st, var_read_int(pb, size));
    } else if (!strcmp(name, "SAMPLE_RATE")) {
        int sample_rate = var_read_int(pb, size);
        if (sample_rate <= 0)
            return AVERROR_INVALIDDATA;
        st->codecpar->sample_rate = sample_rate;
        avpriv_set_pts_info(st, 33, 1, st->codecpar->sample_rate);
    } else if (!strcmp(name, "SAMPLE_WIDTH")) {
        uint64_t bpc = var_read_int(pb, size) * (uint64_t)8;
        if (bpc > 16)
            return AVERROR_INVALIDDATA;
        st->codecpar->bits_per_coded_sample = bpc;
    } else
        return AVERROR_INVALIDDATA;

    return 0;
}

/**
 * Parse video variable
 * @return < 0 if unknown
 */
static int parse_video_var(AVFormatContext *avctx, AVStream *st,
                           const char *name, int size)
{
    AVIOContext *pb = avctx->pb;
    if (!strcmp(name, "__DIR_COUNT")) {
        st->nb_frames = st->duration = var_read_int(pb, size);
    } else if (!strcmp(name, "COMPRESSION")) {
        char *str = var_read_string(pb, size);
        if (!str)
            return AVERROR_INVALIDDATA;
        if (!strcmp(str, "1")) {
            st->codecpar->codec_id = AV_CODEC_ID_MVC1;
        } else if (!strcmp(str, "2")) {
            st->codecpar->format = AV_PIX_FMT_ABGR;
            st->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
        } else if (!strcmp(str, "3")) {
            st->codecpar->codec_id = AV_CODEC_ID_SGIRLE;
        } else if (!strcmp(str, "10")) {
            st->codecpar->codec_id = AV_CODEC_ID_MJPEG;
        } else if (!strcmp(str, "MVC2")) {
            st->codecpar->codec_id = AV_CODEC_ID_MVC2;
        } else {
            avpriv_request_sample(avctx, "Video compression %s", str);
        }
        av_free(str);
    } else if (!strcmp(name, "FPS")) {
        AVRational fps = var_read_float(pb, size);
        avpriv_set_pts_info(st, 64, fps.den, fps.num);
        st->avg_frame_rate = fps;
    } else if (!strcmp(name, "HEIGHT")) {
        st->codecpar->height = var_read_int(pb, size);
    } else if (!strcmp(name, "PIXEL_ASPECT")) {
        st->sample_aspect_ratio = var_read_float(pb, size);
        av_reduce(&st->sample_aspect_ratio.num, &st->sample_aspect_ratio.den,
                  st->sample_aspect_ratio.num, st->sample_aspect_ratio.den,
                  INT_MAX);
    } else if (!strcmp(name, "WIDTH")) {
        st->codecpar->width = var_read_int(pb, size);
    } else if (!strcmp(name, "ORIENTATION")) {
        if (var_read_int(pb, size) == 1101) {
            if (!st->codecpar->extradata) {
                st->codecpar->extradata = av_strdup("BottomUp");
                if (!st->codecpar->extradata)
                    return AVERROR(ENOMEM);
                st->codecpar->extradata_size = 9;
            }
        }
    } else if (!strcmp(name, "Q_SPATIAL") || !strcmp(name, "Q_TEMPORAL")) {
        var_read_metadata(avctx, name, size);
    } else if (!strcmp(name, "INTERLACING") || !strcmp(name, "PACKING")) {
        avio_skip(pb, size); // ignore
    } else
        return AVERROR_INVALIDDATA;

    return 0;
}

static int read_table(AVFormatContext *avctx, AVStream *st,
                       int (*parse)(AVFormatContext *avctx, AVStream *st,
                                    const char *name, int size))
{
    unsigned count;
    int i;

    AVIOContext *pb = avctx->pb;
    avio_skip(pb, 4);
    count = avio_rb32(pb);
    avio_skip(pb, 4);
    for (i = 0; i < count; i++) {
        char name[17];
        int size;

        if (avio_feof(pb))
            return AVERROR_EOF;

        if (avio_read(pb, name, 16) != 16)
            return AVERROR_INVALIDDATA;
        name[sizeof(name) - 1] = 0;
        size = avio_rb32(pb);
        if (size < 0) {
            av_log(avctx, AV_LOG_ERROR, "entry size %d is invalid\n", size);
            return AVERROR_INVALIDDATA;
        }
        if (parse(avctx, st, name, size) < 0) {
            avpriv_request_sample(avctx, "Variable %s", name);
            avio_skip(pb, size);
        }
    }
    return 0;
}

static void read_index(AVIOContext *pb, AVStream *st)
{
    uint64_t timestamp = 0;
    int i;
    for (i = 0; i < st->nb_frames; i++) {
        uint32_t pos  = avio_rb32(pb);
        uint32_t size = avio_rb32(pb);
        avio_skip(pb, 8);
        if (avio_feof(pb))
            return ;
        av_add_index_entry(st, pos, timestamp, size, 0, AVINDEX_KEYFRAME);
        if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            timestamp += size / (st->codecpar->ch_layout.nb_channels * 2LL);
        } else {
            timestamp++;
        }
    }
}

static int mv_read_header(AVFormatContext *avctx)
{
    MvContext *mv = avctx->priv_data;
    AVIOContext *pb = avctx->pb;
    AVStream *ast = NULL, *vst = NULL; //initialization to suppress warning
    int version, i;
    int ret;

    avio_skip(pb, 4);

    version = avio_rb16(pb);
    if (version == 2) {
        uint64_t timestamp;
        int v;
        uint32_t bytes_per_sample;
        AVRational fps;

        avio_skip(pb, 10);

        fps = av_d2q(av_int2double(avio_rb64(pb)), INT_MAX);

        /* allocate audio track first to prevent unnecessary seeking
         * (audio packet always precede video packet for a given frame) */
        v = avio_rb16(pb);
        if (v == MOVIE_SOUND) {
            /* movie has sound so allocate an audio stream */
            ast = avformat_new_stream(avctx, NULL);
            if (!ast)
                return AVERROR(ENOMEM);
        } else if (v != MOVIE_SILENT)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 2);

        vst = avformat_new_stream(avctx, NULL);
        if (!vst)
            return AVERROR(ENOMEM);
        avpriv_set_pts_info(vst, 64, fps.den, fps.num);
        vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        vst->avg_frame_rate = fps;
        vst->duration = vst->nb_frames = avio_rb32(pb);
        v = avio_rb32(pb);
        switch (v) {
        case 1:
            vst->codecpar->codec_id = AV_CODEC_ID_MVC1;
            break;
        case 2:
            vst->codecpar->format = AV_PIX_FMT_ARGB;
            vst->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
            break;
        default:
            avpriv_request_sample(avctx, "Video compression %i", v);
            break;
        }
        vst->codecpar->codec_tag = 0;
        vst->codecpar->width     = avio_rb32(pb);
        vst->codecpar->height    = avio_rb32(pb);
        avio_skip(pb, 12);

        if (ast) {
            ast->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
            ast->nb_frames = vst->nb_frames;
            ast->codecpar->sample_rate = avio_rb32(pb);
            if (ast->codecpar->sample_rate <= 0) {
                av_log(avctx, AV_LOG_ERROR, "Invalid sample rate %d\n", ast->codecpar->sample_rate);
                return AVERROR_INVALIDDATA;
            }
            avpriv_set_pts_info(ast, 33, 1, ast->codecpar->sample_rate);

            bytes_per_sample = avio_rb32(pb);

            v = avio_rb32(pb);
            if (v == AUDIO_FORMAT_SIGNED) {
                switch (bytes_per_sample) {
                case 1:
                    ast->codecpar->codec_id = AV_CODEC_ID_PCM_S8;
                    break;
                case 2:
                    ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
                    break;
                default:
                    avpriv_request_sample(avctx, "Audio sample size %i bytes", bytes_per_sample);
                    break;
                }
            } else {
                avpriv_request_sample(avctx, "Audio compression (format %i)", v);
            }

            if (bytes_per_sample == 0)
                return AVERROR_INVALIDDATA;

            if (set_channels(avctx, ast, avio_rb32(pb)) < 0)
                return AVERROR_INVALIDDATA;

            avio_skip(pb, 8);
        } else
            avio_skip(pb, 24); /* skip meaningless audio metadata */

        var_read_metadata(avctx, "title", 0x80);
        var_read_metadata(avctx, "comment", 0x100);
        avio_skip(pb, 0x80);

        timestamp = 0;
        for (i = 0; i < vst->nb_frames; i++) {
            uint32_t pos   = avio_rb32(pb);
            uint32_t asize = avio_rb32(pb);
            uint32_t vsize = avio_rb32(pb);
            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 8);
            if (ast) {
                av_add_index_entry(ast, pos, timestamp, asize, 0, AVINDEX_KEYFRAME);
                timestamp += asize / (ast->codecpar->ch_layout.nb_channels * (uint64_t)bytes_per_sample);
            }
            av_add_index_entry(vst, pos + asize, i, vsize, 0, AVINDEX_KEYFRAME);
        }
    } else if (!version && avio_rb16(pb) == 3) {
        avio_skip(pb, 4);

        if ((ret = read_table(avctx, NULL, parse_global_var)) < 0)
            return ret;

        if (mv->nb_audio_tracks < 0  || mv->nb_video_tracks < 0 ||
           (mv->nb_audio_tracks == 0 && mv->nb_video_tracks == 0)) {
            av_log(avctx, AV_LOG_ERROR, "Stream count is invalid.\n");
            return AVERROR_INVALIDDATA;
        }

        if (mv->nb_audio_tracks > 1) {
            avpriv_request_sample(avctx, "Multiple audio streams support");
            return AVERROR_PATCHWELCOME;
        } else if (mv->nb_audio_tracks) {
            ast = avformat_new_stream(avctx, NULL);
            if (!ast)
                return AVERROR(ENOMEM);
            ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            if ((ret = read_table(avctx, ast, parse_audio_var)) < 0)
                return ret;
            if (mv->acompression == 100 &&
                mv->aformat == AUDIO_FORMAT_SIGNED &&
                ast->codecpar->bits_per_coded_sample == 16) {
                ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
            } else {
                avpriv_request_sample(avctx,
                                      "Audio compression %i (format %i, sr %i)",
                                      mv->acompression, mv->aformat,
                                      ast->codecpar->bits_per_coded_sample);
                ast->codecpar->codec_id = AV_CODEC_ID_NONE;
            }
            if (ast->codecpar->ch_layout.nb_channels <= 0) {
                av_log(avctx, AV_LOG_ERROR, "No valid channel count found.\n");
                return AVERROR_INVALIDDATA;
            }
        }

        if (mv->nb_video_tracks > 1) {
            avpriv_request_sample(avctx, "Multiple video streams support");
            return AVERROR_PATCHWELCOME;
        } else if (mv->nb_video_tracks) {
            vst = avformat_new_stream(avctx, NULL);
            if (!vst)
                return AVERROR(ENOMEM);
            vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            if ((ret = read_table(avctx, vst, parse_video_var))<0)
                return ret;
        }

        if (mv->nb_audio_tracks)
            read_index(pb, ast);

        if (mv->nb_video_tracks)
            read_index(pb, vst);
    } else {
        avpriv_request_sample(avctx, "Version %i", version);
        return AVERROR_PATCHWELCOME;
    }

    return 0;
}

static int mv_read_packet(AVFormatContext *avctx, AVPacket *pkt)
{
    MvContext *mv = avctx->priv_data;
    AVIOContext *pb = avctx->pb;
    AVStream *st = avctx->streams[mv->stream_index];
    FFStream *const sti = ffstream(st);
    const AVIndexEntry *index;
    int frame = mv->frame[mv->stream_index];
    int64_t ret;
    uint64_t pos;

    if (frame < sti->nb_index_entries) {
        index = &sti->index_entries[frame];
        pos   = avio_tell(pb);
        if (index->pos > pos)
            avio_skip(pb, index->pos - pos);
        else if (index->pos < pos) {
            if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
                return AVERROR(EIO);
            ret = avio_seek(pb, index->pos, SEEK_SET);
            if (ret < 0)
                return ret;
        }
        ret = av_get_packet(pb, pkt, index->size);
        if (ret < 0)
            return ret;

        pkt->stream_index = mv->stream_index;
        pkt->pts          = index->timestamp;
        pkt->flags       |= AV_PKT_FLAG_KEY;

        mv->frame[mv->stream_index]++;
        mv->eof_count = 0;
    } else {
        mv->eof_count++;
        if (mv->eof_count >= avctx->nb_streams)
            return AVERROR_EOF;

        // avoid returning 0 without a packet
        return AVERROR(EAGAIN);
    }

    mv->stream_index++;
    if (mv->stream_index >= avctx->nb_streams)
        mv->stream_index = 0;

    return 0;
}

static int mv_read_seek(AVFormatContext *avctx, int stream_index,
                        int64_t timestamp, int flags)
{
    MvContext *mv = avctx->priv_data;
    AVStream *st = avctx->streams[stream_index];
    int frame, i;

    if ((flags & AVSEEK_FLAG_FRAME) || (flags & AVSEEK_FLAG_BYTE))
        return AVERROR(ENOSYS);

    if (!(avctx->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return AVERROR(EIO);

    frame = av_index_search_timestamp(st, timestamp, flags);
    if (frame < 0)
        return AVERROR_INVALIDDATA;

    for (i = 0; i < avctx->nb_streams; i++)
        mv->frame[i] = frame;
    return 0;
}

const FFInputFormat ff_mv_demuxer = {
    .p.name         = "mv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Silicon Graphics Movie"),
    .priv_data_size = sizeof(MvContext),
    .read_probe     = mv_probe,
    .read_header    = mv_read_header,
    .read_packet    = mv_read_packet,
    .read_seek      = mv_read_seek,
};
