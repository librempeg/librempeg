/*
 * THP muxer
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

#include "libavutil/avassert.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "libavcodec/packet_internal.h"
#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"

typedef struct THPMuxContext {
    AVClass *class;

    int64_t start;
    int header_len;

    int have_audio;
    int have_video;

    int audio_stream_index;
    int video_stream_index;

    int first_audio;
    int first_video;

    uint32_t next_total_size;
    uint32_t prev_total_size;

    uint32_t max_buffer_size;
    uint32_t max_audio_samples;
    float fps;
    uint32_t num_frames;
    uint32_t first_frame_size;
    uint32_t data_size;
    uint32_t component_data_offset;
    uint32_t offsets_data_offset;
    uint32_t first_frame_offset;
    uint32_t last_frame_offset;

    PacketList queue;

    PacketList audio_queue;
    PacketList video_queue;
} THPMuxContext;

static int write_header(AVFormatContext *ctx)
{
    THPMuxContext *thp = ctx->priv_data;
    AVIOContext *pb = ctx->pb;

    thp->audio_stream_index = -1;
    thp->video_stream_index = -1;

    avio_write(pb, "THP\0", 4);
    avio_wb32(pb, 0x00010000);

    ffio_fill(pb, 0, 10 * 4);
    thp->component_data_offset = avio_tell(pb);

    avio_wb32(pb, ctx->nb_streams);
    for (int n = 0; n < ctx->nb_streams; n++) {
        AVCodecParameters *par = ctx->streams[n]->codecpar;

        if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
            avio_w8(pb, 0);
            thp->video_stream_index = n;
            thp->fps = ctx->streams[n]->time_base.den / (float)ctx->streams[n]->time_base.num;
        } else if (par->codec_type == AVMEDIA_TYPE_AUDIO) {
            avio_w8(pb, 1);
            thp->audio_stream_index = n;
        } else {
            return AVERROR(EINVAL);
        }
    }

    ffio_fill(pb, 0xff, 16-ctx->nb_streams);

    for (int n = 0; n < ctx->nb_streams; n++) {
        AVCodecParameters *par = ctx->streams[n]->codecpar;

        if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
            avio_wb32(pb, par->width);
            avio_wb32(pb, par->height);
        } else if (par->codec_type == AVMEDIA_TYPE_AUDIO) {
            avio_wb32(pb, par->ch_layout.nb_channels);
            avio_wb32(pb, par->sample_rate);
            avio_wb32(pb, ctx->streams[n]->duration);
        } else {
            return AVERROR(EINVAL);
        }
    }

    thp->first_frame_offset = avio_tell(pb);
    thp->header_len = (thp->audio_stream_index >= 0) ? 16 : 12;

    return 0;
}

static int write_packet(AVFormatContext *ctx, AVPacket *pkt)
{
    THPMuxContext *thp = ctx->priv_data;
    AVIOContext *pb = ctx->pb;
    uint32_t buffer_size;
    int ret;

    if (thp->audio_stream_index == -1 || thp->video_stream_index == -1) {
        if (!thp->queue.head) {
            if (thp->first_frame_size == 0)
                thp->first_frame_size = pkt->size;
            return avpriv_packet_list_put(&thp->queue, pkt, NULL, 0);
        }

        thp->next_total_size = pkt->size + thp->header_len;
        thp->start = avio_tell(pb);
        avio_wb32(pb, thp->next_total_size);

        ret = avpriv_packet_list_put(&thp->queue, pkt, NULL, 0);
        if (ret < 0)
            return ret;

        ret = avpriv_packet_list_get(&thp->queue, pkt);
        if (ret < 0)
            return ret;

        thp->last_frame_offset = thp->start;

        if (pkt->stream_index == thp->audio_stream_index)
            thp->max_audio_samples = FFMAX(pkt->duration, thp->max_audio_samples);

        avio_wb32(pb, thp->prev_total_size);
        if (thp->video_stream_index == -1) {
            avio_wb32(pb, 0);
            avio_wb32(pb, pkt->size);
        } else {
            avio_wb32(pb, pkt->size);
        }
        avio_write(pb, pkt->data, pkt->size);

        av_packet_unref(pkt);

        thp->num_frames++;
        buffer_size = avio_tell(pb) - thp->start;
        thp->max_buffer_size = FFMAX(thp->max_buffer_size, buffer_size);
        thp->prev_total_size = buffer_size;
    } else {
        int64_t offset, eoffset;

        if (pkt->stream_index == thp->video_stream_index) {
            if (!thp->video_queue.head) {
                if (thp->first_video == 0) {
                    thp->first_frame_size += pkt->size;
                    thp->first_video = 1;
                }
                return avpriv_packet_list_put(&thp->video_queue, pkt, NULL, 0);
            }

            thp->next_total_size += pkt->size;

            ret = avpriv_packet_list_put(&thp->video_queue, pkt, NULL, 0);
            if (ret < 0)
                return ret;
            thp->have_video = 1;
        } else if (pkt->stream_index == thp->audio_stream_index) {
            if (!thp->audio_queue.head) {
                if (thp->first_audio == 0) {
                    thp->first_frame_size += pkt->size;
                    thp->first_audio = 1;
                }
                return avpriv_packet_list_put(&thp->audio_queue, pkt, NULL, 0);
            }

            thp->next_total_size += pkt->size;

            ret = avpriv_packet_list_put(&thp->audio_queue, pkt, NULL, 0);
            if (ret < 0)
                return ret;
            thp->have_audio = 1;
        }

        if (!thp->have_audio || !thp->have_video)
            return 0;

        ret = avpriv_packet_list_get(&thp->video_queue, pkt);
        if (ret < 0)
            return ret;

        thp->start = avio_tell(pb);
        avio_wb32(pb, thp->next_total_size + thp->header_len);
        thp->next_total_size = 0;

        avio_wb32(pb, thp->prev_total_size);
        avio_wb32(pb, pkt->size);
        offset = avio_tell(pb);
        avio_wb32(pb, 0);
        avio_write(pb, pkt->data, pkt->size);
        eoffset = avio_tell(pb);

        av_packet_unref(pkt);

        thp->last_frame_offset = thp->start;

        ret = avpriv_packet_list_get(&thp->audio_queue, pkt);
        if (ret < 0)
            return ret;

        thp->max_audio_samples = FFMAX(pkt->duration, thp->max_audio_samples);

        avio_seek(pb, offset, SEEK_SET);
        avio_wb32(pb, pkt->size);
        avio_seek(pb, eoffset, SEEK_SET);
        avio_write(pb, pkt->data, pkt->size);
        av_packet_unref(pkt);

        thp->num_frames++;
        buffer_size = avio_tell(pb) - thp->start;
        thp->max_buffer_size = FFMAX(thp->max_buffer_size, buffer_size);
        thp->prev_total_size = buffer_size;
        thp->have_audio = thp->have_video = 0;
    }

    return 0;
}

static int write_trailer(AVFormatContext *ctx)
{
    THPMuxContext *thp = ctx->priv_data;
    AVIOContext *pb = ctx->pb;
    uint32_t buffer_size;
    int ret;

    if (thp->audio_stream_index == -1 || thp->video_stream_index == -1) {
        if (thp->queue.head) {
            AVPacket *const pkt = ffformatcontext(ctx)->pkt;

            ret = avpriv_packet_list_get(&thp->queue, pkt);
            if (ret < 0)
                return ret;

            thp->start = avio_tell(pb);
            avio_wb32(pb, 0);
            thp->last_frame_offset = thp->start;

            if (pkt->stream_index == thp->audio_stream_index)
                thp->max_audio_samples = FFMAX(pkt->duration, thp->max_audio_samples);

            avio_wb32(pb, thp->prev_total_size);
            if (thp->video_stream_index == -1) {
                avio_wb32(pb, 0);
                avio_wb32(pb, pkt->size);
            } else {
                avio_wb32(pb, pkt->size);
            }
            avio_write(pb, pkt->data, pkt->size);

            thp->num_frames++;
            buffer_size = avio_tell(pb) - thp->start;
            thp->max_buffer_size = FFMAX(thp->max_buffer_size, buffer_size);
            thp->prev_total_size = buffer_size;
        }
    }

    thp->data_size = avio_tell(pb);

    avio_seek(pb, 8, SEEK_SET);
    avio_wb32(pb, thp->max_buffer_size);
    avio_wb32(pb, thp->max_audio_samples);
    avio_wb32(pb, av_float2int(thp->fps));
    avio_wb32(pb, thp->num_frames);
    avio_wb32(pb, thp->first_frame_size + thp->header_len);
    avio_wb32(pb, thp->data_size);
    avio_wb32(pb, thp->component_data_offset);
    avio_wb32(pb, thp->offsets_data_offset);
    avio_wb32(pb, thp->first_frame_offset);
    avio_wb32(pb, thp->last_frame_offset);

    return 0;
}

static void deinit(AVFormatContext *s)
{
    THPMuxContext *thp = s->priv_data;

    avpriv_packet_list_free(&thp->queue);
    avpriv_packet_list_free(&thp->audio_queue);
    avpriv_packet_list_free(&thp->video_queue);
}

const FFOutputFormat ff_thp_muxer = {
    .p.name         = "thp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("THP"),
    .p.extensions   = "thp",
    .priv_data_size = sizeof(THPMuxContext),
    .p.audio_codec  = AV_CODEC_ID_ADPCM_THP,
    .p.video_codec  = AV_CODEC_ID_THP,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
    .write_header   = write_header,
    .write_packet   = write_packet,
    .write_trailer  = write_trailer,
    .deinit         = deinit,
};
