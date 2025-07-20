/*
 * MO demuxer
 * Copyright (c) 2025 Paul B Mahol
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct MODemuxContext {
    AVPacket *audio_pkt;
    int multi_audio;
    int nb_audio_tracks;
    int current_audio_track;
} MODemuxContext;

static int mo_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('M','O','C','5'))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int mo_read_header(AVFormatContext *s)
{
    MODemuxContext *m = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *vst, *ast;
    int audio_codec;
    int64_t offset;
    AVRational fps;

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);
    offset = avio_rl32(pb) + 8;
    avio_skip(pb, 4);
    fps.num = avio_rl32(pb);
    vst->duration = avio_rl32(pb);
    fps.den = 0x100;
    avio_skip(pb, 8);

    vst->start_time           = 0;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id   = AV_CODEC_ID_MOBICLIP;
    vst->codecpar->width      = avio_rl32(pb);
    vst->codecpar->height     = avio_rl32(pb);

    avpriv_set_pts_info(vst, 64, fps.den, fps.num);

    m->audio_pkt = av_packet_alloc();
    if (!m->audio_pkt)
        return AVERROR(ENOMEM);

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    avio_seek(pb, 0xd4, SEEK_SET);
    audio_codec = avio_rl16(pb);
    avio_skip(pb, 2);

    ast->start_time = 0;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

    switch (audio_codec) {
    case 0x3341:
    case 0x3241:
        ast->codecpar->sample_rate = avio_rl32(pb);
        ast->codecpar->ch_layout.nb_channels = avio_rl32(pb);
        ast->codecpar->codec_id = AV_CODEC_ID_FASTAUDIO;
        ast->codecpar->block_align = 40 * ast->codecpar->ch_layout.nb_channels;
        ffstream(ast)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
        break;
    case 0x3941:
    case 0x3841:
        ast->codecpar->sample_rate = avio_rl32(pb);
        ast->codecpar->ch_layout.nb_channels = avio_rl32(pb);
        ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_MO;
        ast->codecpar->block_align = 132 * ast->codecpar->ch_layout.nb_channels;
        ffstream(ast)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
        break;
    case 0x4D41:
        m->multi_audio = 1;
        m->nb_audio_tracks = avio_rl32(pb);
        if (m->nb_audio_tracks < 1 ||
            m->nb_audio_tracks > 8)
            return AVERROR_INVALIDDATA;

        audio_codec = avio_rl16(pb);
        switch (audio_codec) {
        case 0x3341:
        case 0x3241:
            ast->codecpar->codec_id = AV_CODEC_ID_FASTAUDIO;
            ast->codecpar->block_align = 40 * ast->codecpar->ch_layout.nb_channels;
            break;
        case 0x3941:
        case 0x3841:
            ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_MO;
            ast->codecpar->block_align = 132 * ast->codecpar->ch_layout.nb_channels;
            break;
        default:
            avpriv_request_sample(s, "codec 0x%X", audio_codec);
            return AVERROR_PATCHWELCOME;
        }

        ast->codecpar->sample_rate = avio_rl32(pb);
        ast->codecpar->ch_layout.nb_channels = avio_rl32(pb);
        ffstream(ast)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

        for (int n = 1; n < m->nb_audio_tracks; n++) {
            ast = avformat_new_stream(s, NULL);
            if (!ast)
                return AVERROR(ENOMEM);

            audio_codec = avio_rl16(pb);
            switch (audio_codec) {
            case 0x3341:
            case 0x3241:
                ast->codecpar->codec_id = AV_CODEC_ID_FASTAUDIO;
                ast->codecpar->block_align = 40 * ast->codecpar->ch_layout.nb_channels;
                break;
            case 0x3941:
            case 0x3841:
                ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_MO;
                ast->codecpar->block_align = 132 * ast->codecpar->ch_layout.nb_channels;
                break;
            default:
                avpriv_request_sample(s, "codec 0x%X", audio_codec);
                return AVERROR_PATCHWELCOME;
            }

            ast->start_time = 0;
            ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            ast->codecpar->sample_rate = avio_rl32(pb);
            ast->codecpar->ch_layout.nb_channels = avio_rl32(pb);
            ffstream(ast)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
        }
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", audio_codec);
        return AVERROR_PATCHWELCOME;
    }

    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int mo_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    MODemuxContext *m = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t size, vsize;
    int64_t pos;
    int ret;

    if (m->audio_pkt->size > 0) {
        if (m->multi_audio) {
            uint32_t header_size = 4 * m->nb_audio_tracks;
            uint32_t offset, packet_size, offset_add;

            if (m->audio_pkt->size <= header_size)
                return AVERROR_INVALIDDATA;

            offset_add = AV_RL32(m->audio_pkt->data+4*m->current_audio_track);
            if (offset_add >= m->audio_pkt->size - header_size)
                return AVERROR_INVALIDDATA;
            offset = header_size + offset_add;

            if (m->current_audio_track + 1 < m->nb_audio_tracks) {
                packet_size = AV_RL32(m->audio_pkt->data+4*(m->current_audio_track+1));
                if (packet_size <= offset)
                    return AVERROR_INVALIDDATA;
                packet_size -= offset;
            } else {
                packet_size = AV_RL32(m->audio_pkt->data+4*m->current_audio_track);
                if (packet_size >= m->audio_pkt->size - header_size)
                    return AVERROR_INVALIDDATA;
                packet_size = m->audio_pkt->size - packet_size - header_size;
            }

            if (m->audio_pkt->size - packet_size < offset)
                return AVERROR_INVALIDDATA;

            if ((ret = av_new_packet(pkt, packet_size)) < 0)
                return ret;
            memcpy(pkt->data, m->audio_pkt->data + offset, packet_size);

            pkt->stream_index = 1 + m->current_audio_track;
            m->current_audio_track++;
            if (m->current_audio_track >= m->nb_audio_tracks) {
                m->current_audio_track = 0;
                av_packet_unref(m->audio_pkt);
            }
        } else {
            av_packet_move_ref(pkt, m->audio_pkt);
            pkt->stream_index = 1;
        }
        return 0;
    }

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    size = (avio_rl32(pb) & ~1) - 4;
    vsize = avio_rl32(pb);
    if (size < vsize)
        return AVERROR_INVALIDDATA;

    ret = av_get_packet(pb, pkt, vsize);
    if (ret < 0)
        return ret;

    if (size > vsize) {
        ret = av_get_packet(pb, m->audio_pkt, size - vsize);
        if (ret < 0)
            return ret;
        m->audio_pkt->pos = pos;
    }

    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

static int mo_read_seek(AVFormatContext *s, int stream_index,
                        int64_t timestamp, int flags)
{
    MODemuxContext *m = s->priv_data;

    m->current_audio_track = 0;
    av_packet_unref(m->audio_pkt);

    return -1;
}

static int mo_read_close(AVFormatContext *s)
{
    MODemuxContext *m = s->priv_data;

    av_packet_free(&m->audio_pkt);

    return 0;
}

const FFInputFormat ff_mo_demuxer = {
    .p.name         = "mo",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MobiClip MO"),
    .p.extensions   = "mo",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(MODemuxContext),
    .read_probe     = mo_probe,
    .read_header    = mo_read_header,
    .read_packet    = mo_read_packet,
    .read_seek      = mo_read_seek,
    .read_close     = mo_read_close,
};
