/*
 * TT AD demuxer
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
#include "avio_internal.h"
#include "internal.h"

typedef struct TTADDemuxContext {
    AVFormatContext *xctx;
    FFIOContext apb;
    int64_t start_offset;
} TTADDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('F','M','T',' '))
        return 0;
    if (p->buf_size < 21)
        return 0;
    if (AV_RL16(p->buf+10) != 0x01 &&
        AV_RL16(p->buf+10) != 0x0a)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;
    if (p->buf[20] == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    AVFormatContext *s = opaque;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    AVFormatContext *s = opaque;
    TTADDemuxContext *tt = s->priv_data;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + tt->start_offset, whence);
}

static int read_header(AVFormatContext *s)
{
    int ret, channels, align = 1024, rate, codec;
    TTADDemuxContext *tt = s->priv_data;
    int64_t duration, offset = 0x08;
    AVIOContext *pb = s->pb;
    FFStream *sti;
    AVStream *st;

    avio_skip(pb, 4);
    offset += avio_rl32(pb);
    avio_skip(pb, 2);
    codec = avio_rl16(pb);
    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    channels = avio_r8(pb);
    if (channels == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 3);
    if (codec == 0x0a) {
        align = avio_rl16(pb);
        if (align > INT_MAX/channels)
            return AVERROR_INVALIDDATA;
    }

    switch (codec) {
    case 0x01:
        codec = AV_CODEC_ID_VORBIS;
        break;
    case 0x0a:
        codec = AV_CODEC_ID_ADPCM_IMA_WAV_MONO;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    avio_seek(pb, offset, SEEK_SET);
    while (!avio_feof(pb)) {
        uint32_t chunk_size;
        uint32_t chunk_id;
        int got_data = 0;

        chunk_id = avio_rb32(pb);
        chunk_size = avio_rl32(pb);

        switch (chunk_id) {
        case MKBETAG('S','E','E','K'):
        case MKBETAG('R','M','S',' '):
            avio_skip(pb, chunk_size);
            break;
        case MKBETAG('D','A','T','A'):
            got_data = 1;
            break;
        default:
            return AVERROR_INVALIDDATA;
        }

        if (got_data) {
            offset += 8LL;
            break;
        }

        offset += 8LL + chunk_size;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    if (codec == AV_CODEC_ID_ADPCM_IMA_WAV_MONO)
        st->codecpar->bits_per_coded_sample = 4;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, offset, SEEK_SET);

    if (codec != AV_CODEC_ID_VORBIS)
        return 0;

    if (!(tt->xctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(tt->xctx, s)) < 0) {
        avformat_free_context(tt->xctx);
        tt->xctx = NULL;

        return ret;
    }

    ffio_init_context(&tt->apb, NULL, 0, 0, s,
                      read_data, NULL, seek_data);

    tt->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    tt->xctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    tt->xctx->probesize = 0;
    tt->xctx->max_analyze_duration = 0;
    tt->xctx->interrupt_callback = s->interrupt_callback;
    tt->xctx->pb = &tt->apb.pub;
    tt->xctx->skip_initial_bytes = 0;
    tt->xctx->io_open = NULL;

    ret = avformat_open_input(&tt->xctx, "", NULL, NULL);
    if (ret < 0)
        return ret;

    ret = avformat_find_stream_info(tt->xctx, NULL);
    if (ret < 0)
        return ret;

    st->id = tt->xctx->streams[0]->id;
    st->duration = tt->xctx->streams[0]->duration;
    st->time_base = tt->xctx->streams[0]->time_base;
    st->start_time = tt->xctx->streams[0]->start_time;
    st->pts_wrap_bits = tt->xctx->streams[0]->pts_wrap_bits;

    ret = avcodec_parameters_copy(st->codecpar, tt->xctx->streams[0]->codecpar);
    if (ret < 0)
        return ret;

    ret = av_dict_copy(&st->metadata, tt->xctx->streams[0]->metadata, 0);
    if (ret < 0)
        return ret;

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = ffstream(tt->xctx->streams[0])->need_parsing;
    tt->start_offset = avio_tell(pb);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    TTADDemuxContext *tt = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (tt->xctx) {
        ret = av_read_frame(tt->xctx, pkt);
        pkt->stream_index = 0;
    } else {
        uint32_t chunk_id;
        int64_t pos;

        pos = avio_tell(pb);
        ret = ffio_ensure_seekback(pb, 4);
        if (ret < 0)
            return ret;

        chunk_id = avio_rb32(pb);
        switch (chunk_id) {
        case MKBETAG('F','R','S','T'):
        case MKBETAG('L','A','S','T'):
        case MKBETAG('L','S','R','T'):
        case MKBETAG('L','E','N','D'):
            avio_skip(pb, 4);
            break;
        default:
            avio_seek(pb, -4, SEEK_CUR);
            break;
        }

        ret = av_get_packet(pb, pkt, par->block_align);
        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
        pkt->stream_index = 0;
        pkt->pos = pos;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    TTADDemuxContext *tt = s->priv_data;

    if (tt->xctx)
        return av_seek_frame(tt->xctx, 0, ts, flags);
    else
        return -1;
}

static int read_close(AVFormatContext *s)
{
    TTADDemuxContext *tt = s->priv_data;

    avformat_close_input(&tt->xctx);

    return 0;
}

const FFInputFormat ff_ttad_demuxer = {
    .p.name         = "ttad",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Traveller's Tales AD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "audio_data",
    .priv_data_size = sizeof(TTADDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
