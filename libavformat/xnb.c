/*
 * XNB demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct XNBStream {
    int64_t start_offset;
    int64_t stop_offset;
} XNBStream;

typedef struct XNBContext {
    int current_stream;
} XNBContext;

static int read_probe(const AVProbeData *p)
{
    if ((AV_RB32(p->buf) & 0xFFFFFF00) != MKBETAG('X','N','B','\0'))
        return 0;

    if (p->buf_size < 36)
        return 0;
    if (p->buf[4] != 3 && p->buf[4] != 4 && p->buf[4] != 5)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static const char *type_sound = "Microsoft.Xna.Framework.Content.SoundEffectReader";
static const char *type_int32 = "Microsoft.Xna.Framework.Content.Int32Reader";
static const char *type_song = "Microsoft.Xna.Framework.Content.SongReader";
static const char *type_ogg = "SoundEffectFromOggReader";

static int read_header(AVFormatContext *s)
{
    int flags, ret, nb_channels, rate, codec, type_count, string_len, align, nb_streams = 1;
    int platform, is_song = 0, is_ogg = 0, is_sound = 0, big_endian, bps, is_at9 = 0;
    int64_t data_size, start_offset, duration = 0;
    char reader_name[256] = { 0 };
    char song_name[256] = { 0 };
    AVIOContext *pb = s->pb;
    XNBStream *xst;
    AVStream *st;

    avio_skip(pb, 3);
    platform = avio_r8(pb);
    big_endian = platform == 'x';
    avio_skip(pb, 1);
    flags = avio_r8(pb);
    if (flags & 0xc0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x0a, SEEK_SET);

    type_count = avio_r8(pb);
    string_len = avio_r8(pb);
    ret = avio_get_str(pb, string_len+1, reader_name, sizeof(reader_name));
    if (ret < 0)
        return ret;

    if (strcmp(reader_name, type_sound) == 0) {
        if (type_count != 1)
            return AVERROR_INVALIDDATA;
        is_sound = 1;
    } else if (!strncmp(reader_name, type_ogg, strlen(type_ogg))) {
        if (type_count != 1)
            return AVERROR_INVALIDDATA;
        is_ogg = 1;
    } else if (strcmp(reader_name, type_song) == 0) {
        if (type_count != 2)
            return AVERROR_INVALIDDATA;
        is_song = 1;
    } else {
        return AVERROR_INVALIDDATA;
    }

    if (is_song) {
        avio_skip(pb, 3);

        string_len = avio_r8(pb);
        ret = avio_get_str(pb, string_len+1, reader_name, sizeof(reader_name));
        if (ret < 0)
            return AVERROR_INVALIDDATA;

        if (strcmp(reader_name, type_int32))
            return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 4);
    if (avio_r8(pb) != 1)
        return AVERROR_INVALIDDATA;

    if (is_sound || is_ogg) {
        unsigned (*avio_r32)(AVIOContext *pb) = big_endian ? avio_rb32 : avio_rl32;
        unsigned (*avio_r16)(AVIOContext *pb) = big_endian ? avio_rb16 : avio_rl16;
        int64_t fmt_chunk_size;

        fmt_chunk_size = avio_rl32(pb);
        if (fmt_chunk_size < 16)
            return AVERROR_INVALIDDATA;

        codec = avio_r16(pb);
        nb_channels = avio_r16(pb);
        rate = avio_r32(pb);
        avio_skip(pb, 4);
        align = avio_r16(pb);
        bps = avio_r16(pb);
        if (nb_channels <= 0)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, fmt_chunk_size - 16);
        data_size = avio_rl32(pb);
        start_offset = avio_tell(pb);
        switch (codec) {
        case 0x01:
            codec = bps <= 8 ? AV_CODEC_ID_PCM_U8 : big_endian ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
            align = 128;
            break;
        case 0x02:
            if (align <= 0)
                return AVERROR_INVALIDDATA;
            codec = AV_CODEC_ID_ADPCM_MS;
            break;
        case 0x11:
            if (align <= 0)
                return AVERROR_INVALIDDATA;
            codec = AV_CODEC_ID_ADPCM_IMA_WAV;
            bps = 4;
            break;
        case 0x166:
            codec = AV_CODEC_ID_XMA2;
            align = 0x800;
            break;
        case 0xFFFE:
            is_at9 = 1;
            break;
        case 0xFFFF:
            if (platform != 'S')
                return AVERROR_INVALIDDATA;
            duration = avio_r32(pb);
            avio_skip(pb, 4);
            rate = avio_r32(pb);
            codec = big_endian ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
            nb_streams = nb_channels;
            nb_channels = 1;
            align = 1024;
            break;
        default:
            avpriv_request_sample(s, "codec %X", codec);
            return AVERROR_PATCHWELCOME;
        }
    } else if (is_song) {
        string_len = avio_r8(pb);

        ret = avio_get_str(pb, string_len+1, song_name, sizeof(song_name));
        if (ret < 0)
            return ret;
        start_offset = 0;
        avpriv_request_sample(s, "external stream");
        return AVERROR_PATCHWELCOME;
    } else {
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    xst = av_mallocz(sizeof(*xst));
    if (!xst)
        return AVERROR(ENOMEM);
    st->priv_data = xst;

    if (is_ogg || is_at9) {
        avpriv_request_sample(s, "subfile stream");
        return AVERROR_PATCHWELCOME;
    }

    if (codec == AV_CODEC_ID_ADPCM_NDSP ||
        codec == AV_CODEC_ID_ADPCM_NDSP_LE) {
        avio_skip(pb, 16);
        ret = ff_get_extradata(s, st->codecpar, pb, 32);
        if (ret < 0)
            return ret;
        avio_skip(pb, 4);
    }

    xst->start_offset = start_offset;
    xst->stop_offset = start_offset + data_size / nb_streams;

    st->start_time = 0;
    if (duration > 0)
        st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->bits_per_coded_sample = bps;
    st->codecpar->block_align = align * nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    if (codec == AV_CODEC_ID_ADPCM_NDSP ||
        codec == AV_CODEC_ID_ADPCM_NDSP_LE) {
        xst->start_offset += 64;
    }

    if (nb_streams > 1) {
        XNBStream *xst0 = xst;
        AVStream *st0 = st;

        for (int n = 1; n < nb_streams; n++) {
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            xst = av_mallocz(sizeof(*xst));
            if (!xst)
                return AVERROR(ENOMEM);
            st->priv_data = xst;

            xst->start_offset = xst0->stop_offset;
            xst->stop_offset = xst->start_offset + data_size / nb_streams;

            st->start_time = 0;
            if (duration > 0)
                st->duration = duration;

            ret = avcodec_parameters_copy(st->codecpar, st0->codecpar);
            if (ret < 0)
                return ret;

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

            if (codec == AV_CODEC_ID_ADPCM_NDSP ||
                codec == AV_CODEC_ID_ADPCM_NDSP_LE) {
                avio_seek(pb, xst->start_offset, SEEK_SET);

                avio_skip(pb, 16+12);
                ret = ff_get_extradata(s, st->codecpar, pb, 32);
                if (ret < 0)
                    return ret;
                avio_skip(pb, 4);

                xst->start_offset += 64;
            }

            xst0 = xst;
        }
    }

    {
        XNBStream *xst = s->streams[0]->priv_data;

        avio_seek(pb, xst->start_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    XNBContext *xnb = s->priv_data;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    XNBStream *xst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (xnb->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[xnb->current_stream];
    xst = st->priv_data;
    par = st->codecpar;

    if (do_seek)
        avio_seek(pb, xst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= xst->stop_offset) {
        do_seek = 1;
        xnb->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int size = FFMIN(par->block_align, xst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
        pkt->stream_index = st->index;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    XNBContext *xnb = s->priv_data;
    AVIOContext *pb = s->pb;
    XNBStream *xst;
    AVStream *st;
    int64_t pos;

    xnb->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[xnb->current_stream];
    xst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < xst->start_offset) {
        avio_seek(pb, xst->start_offset, SEEK_SET);
        return 0;
    }

    {
        AVIndexEntry *ie;
        int index;

        index = ff_index_search_timestamp(ffstream(st)->index_entries,
                                          ffstream(st)->nb_index_entries, ts, flags);
        if (index < 0) {
            return AVERROR(EINVAL);
        } else {
            ie = &ffstream(st)->index_entries[index];
        }
        ffstream(st)->cur_dts = ie->timestamp;
        pos = ie->pos;
    }

    avio_seek(pb, pos, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xnb_demuxer = {
    .p.name         = "xnb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("XNA Game Studio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xnb",
    .priv_data_size = sizeof(XNBContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
