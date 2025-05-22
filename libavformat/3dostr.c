/*
 * 3DO STR demuxer
 * Copyright (c) 2015 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct ThreeDOSTRContext {
    int video_stream_index;
    int audio_stream_index;
    int nb_channels;
} ThreeDOSTRContext;

static int threedostr_probe(const AVProbeData *p)
{
    for (int i = 0; i < p->buf_size;) {
        unsigned chunk = AV_RL32(p->buf + i);
        unsigned size  = AV_RB32(p->buf + i + 4);

        if (size < 8 || p->buf_size - i < size)
            return 0;
        i += 8;
        size -= 8;
        switch (chunk) {
        case MKTAG('C','T','R','L'):
            break;
        case MKTAG('S','N','D','S'):
            if (size < 56)
                return 0;
            i += 8;
            if (AV_RL32(p->buf + i) != MKTAG('S','H','D','R'))
                return 0;
            i += 28;

            if (AV_RB32(p->buf + i) <= 0)
                return 0;
            i += 4;
            if (AV_RB32(p->buf + i) <= 0)
                return 0;
            i += 4;
            if (AV_RL32(p->buf + i) == MKTAG('S','D','X','2'))
                return AVPROBE_SCORE_MAX;
            else
                return 0;
            break;
        case MKTAG('S','H','D','R'):
            if (size > 0x78) {
                i += 0x78;
                size -= 0x78;
            }
            break;
        default:
            break;
        }

        i += size;
    }

    return 0;
}

static int threedostr_read_header(AVFormatContext *s)
{
    ThreeDOSTRContext *ctx = s->priv_data;

    ctx->video_stream_index = -1;
    ctx->audio_stream_index = -1;

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    return 0;
}

static int threedostr_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    unsigned chunk, vcodec = 0, codec = 0, size, ctrl_size = -1;
    ThreeDOSTRContext *ctx = s->priv_data;
    AVStream *st, *vst;
    int64_t pos;
    int ret = 0;
    int next_actual_chunk = 0;

    while (!avio_feof(s->pb)) {
        pos   = avio_tell(s->pb);
        chunk = avio_rl32(s->pb);
        size  = avio_rb32(s->pb);
        
        if (chunk == MKTAG('F', 'I', 'L', 'L') && (size == MKBETAG('C', 'T', 'R', 'L')) || (size == MKBETAG('F', 'I', 'L', 'M')))
            next_actual_chunk = 1;
        
        if (!size)
            continue;

        if (size < 8)
            return AVERROR_INVALIDDATA;
        
        if (!next_actual_chunk)
            size -= 8;

        switch (chunk) {
        case MKTAG('C','T','R','L'):
            ctrl_size = size;
            break;
        case MKTAG('S','N','D','S'):
            if (ctx->audio_stream_index >= 0) {
                int packet_size;

                if (size <= 16)
                    return AVERROR_INVALIDDATA;
                avio_skip(s->pb, 8);
                if (avio_rl32(s->pb) != MKTAG('S','S','M','P'))
                    return AVERROR_INVALIDDATA;
                packet_size = avio_rb32(s->pb);
                if (packet_size > size - 16 || packet_size <= 0)
                    return AVERROR_INVALIDDATA;
                size -= 16;
                ret = av_get_packet(s->pb, pkt, packet_size);
                pkt->pos = pos;
                pkt->stream_index = ctx->audio_stream_index;
                pkt->duration = packet_size / ctx->nb_channels;
                avio_skip(s->pb, size - packet_size);
                return ret;
            }

            if (size < 56)
                return AVERROR_INVALIDDATA;
            avio_skip(s->pb, 8);
            if (avio_rl32(s->pb) != MKTAG('S','H','D','R'))
                return AVERROR_INVALIDDATA;
            avio_skip(s->pb, 24);

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            ctx->audio_stream_index = st->index;
            st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
            st->codecpar->sample_rate = avio_rb32(s->pb);
            st->codecpar->ch_layout.nb_channels = ctx->nb_channels = avio_rb32(s->pb);
            if (st->codecpar->ch_layout.nb_channels <= 0 || st->codecpar->sample_rate <= 0)
                return AVERROR_INVALIDDATA;
            codec                  = avio_rl32(s->pb);
            avio_skip(s->pb, 4);
            if (ctrl_size == 20 || ctrl_size == 16 || ctrl_size == 3 || ctrl_size == -1)
                st->duration       = (avio_rb32(s->pb) - 1) / st->codecpar->ch_layout.nb_channels;
            else
                st->duration       = avio_rb32(s->pb) * 16 / st->codecpar->ch_layout.nb_channels;
            st->start_time = 0;
            size -= 56;

            switch (codec) {
            case MKTAG('S','D','X','2'):
                st->codecpar->codec_id    = AV_CODEC_ID_SDX2_DPCM;
                st->codecpar->block_align = 1 * st->codecpar->ch_layout.nb_channels;
                break;
            default:
                avpriv_request_sample(s, "codec %X", codec);
                return AVERROR_PATCHWELCOME;
            }

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

            break;
        case MKTAG('S','H','D','R'):
            if (size >  0x78) {
                avio_skip(s->pb, 0x74);
                size -= 0x78;
                if (avio_rl32(s->pb) == MKTAG('C','T','R','L') && size > 4) {
                    ctrl_size = avio_rb32(s->pb);
                    size -= 4;
                }
            }
            break;
        case MKTAG('S','A','N','M'):
            if (ctx->video_stream_index >= 0) {
                if (size <= 16)
                    return AVERROR_INVALIDDATA;
                avio_skip(s->pb, 12);
                size -= 12;
                ret = av_get_packet(s->pb, pkt, size);
                pkt->pos = pos;
                pkt->stream_index = ctx->video_stream_index;
                pkt->duration = 1;
                return ret;
            }

            avio_skip(s->pb, 8);
            if (avio_rl32(s->pb) == MKTAG('A','H','D','R')) {
                vst = avformat_new_stream(s, NULL);
                if (!vst)
                    return AVERROR(ENOMEM);

                vcodec = chunk;
                ctx->video_stream_index    = vst->index;
                vst->start_time            = 0;
                vst->codecpar->codec_type  = AVMEDIA_TYPE_VIDEO;
                vst->codecpar->width       = 258;
                vst->codecpar->height      = 258;

                switch (vcodec) {
                case MKTAG('S','A','N','M'):
                    vst->codecpar->codec_id = AV_CODEC_ID_SANM;
                    avpriv_set_pts_info(vst, 64, 1, 15);
                    break;
                default:
                    avpriv_request_sample(s, "codec %X", codec);
                    return AVERROR_PATCHWELCOME;
                }
            }
            size -= 12;
            break;
        case MKTAG('F','I','L','M'):
            if (ctx->video_stream_index >= 0) {
                if (size <= 20)
                    return AVERROR_INVALIDDATA;
                avio_skip(s->pb, 20);
                size -= 20;
                ret = av_get_packet(s->pb, pkt, size);
                pkt->pos = pos;
                pkt->stream_index = ctx->video_stream_index;
                pkt->duration = 1;
                return ret;
            }

            avio_skip(s->pb, 8);
            if (avio_rl32(s->pb) == MKTAG('F','H','D','R')) {
                vst = avformat_new_stream(s, NULL);
                if (!vst)
                    return AVERROR(ENOMEM);
                avio_skip(s->pb, 4);

                vcodec = avio_rl32(s->pb);
                ctx->video_stream_index    = vst->index;
                vst->codecpar->codec_type  = AVMEDIA_TYPE_VIDEO;
                vst->codecpar->height      = avio_rb32(s->pb);
                vst->codecpar->width       = avio_rb32(s->pb);
                avio_skip(s->pb, 4);
                vst->start_time            = 0;
                vst->duration = vst->nb_frames = avio_rb32(s->pb);

                switch (vcodec) {
                case MKTAG('c','v','i','d'):
                    vst->codecpar->codec_id = AV_CODEC_ID_CINEPAK;
                    break;
                default:
                    avpriv_request_sample(s, "codec %X", codec);
                    return AVERROR_PATCHWELCOME;
                }

                avpriv_set_pts_info(vst, 64, 1, 15);

                size -= 24;
            }
            size -= 12;
            break;
        default:
            av_log(s, AV_LOG_DEBUG, "skipping unknown chunk: %X\n", chunk);
            break;
        }
        
        if (!next_actual_chunk)
            avio_skip(s->pb, size);
        else {
            avio_seek(s->pb, -4, SEEK_CUR);
            next_actual_chunk = 0;
        }
    }

    return AVERROR_EOF;
}

const FFInputFormat ff_threedostr_demuxer = {
    .p.name         = "3dostr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("3DO STR"),
    .p.extensions   = "str",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(ThreeDOSTRContext),
    .read_probe     = threedostr_probe,
    .read_header    = threedostr_read_header,
    .read_packet    = threedostr_read_packet,
};
