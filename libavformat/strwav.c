/*
 * STR+WAV demuxer
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

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/mathops.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct STRWAVContext {
    AVClass     *class;
    AVIOContext *pb;
} STRWAVContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0)
        return 0;
    if (AV_RB32(p->buf+4) != 0x00000800 &&
        AV_RB32(p->buf+4) != 0x01000800 &&
        AV_RB32(p->buf+4) != 0x00000900)
        return 0;
    if (av_match_ext(p->filename, "wav") == 0)
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

static uint32_t read_rl32(AVIOContext *pb, const int64_t offset)
{
    avio_seek(pb, offset, SEEK_SET);

    return avio_rl32(pb);
}

static int read_header(AVFormatContext *s)
{
    int ret, codec = 0, sample_rate = 0, channels = 0, tracks = 0, align = 0, bps = 0;
    int64_t duration = 0, header_size;
    AVIOContext *pb = s->pb;
    uint32_t header, flags;
    AVStream *st;

    avio_skip(pb, 4);
    header_size = avio_size(pb);
    header = avio_rb32(pb);

    if ((header == 0x00000800 || header == 0x00000900 || header == 0x01000800) &&
        read_rl32(pb, 0x24) == read_rl32(pb, 0xB0) &&
        read_rl32(pb, 0x28) == 0x10 &&
        ((read_rl32(pb, 0xE0) + read_rl32(pb, 0xE4) * 0x40 == header_size) ||
         (read_rl32(pb, 0xc0) * 4 + read_rl32(pb, 0xc4) == header_size))) {
        duration = read_rl32(pb, 0x20);
        sample_rate = read_rl32(pb, 0x24);
        flags = read_rl32(pb, 0x2c);
        tracks = read_rl32(pb, 0x70);
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        align = (tracks > 1) ? 0xD800/2 : 0xD800;
        channels = tracks * ((flags & 0x02) ? 2 : 1);
        bps = 4;
    } else if ((header == 0x00000800 || header == 0x00000900) &&
               read_rl32(pb, 0x24) == read_rl32(pb, 0x70) &&
               read_rl32(pb, 0x78) * 4 + read_rl32(pb, 0x7c) == header_size) {
        duration = read_rl32(pb, 0x20);
        sample_rate = read_rl32(pb, 0x24);
        flags = read_rl32(pb, 0x2c);
        tracks = read_rl32(pb, 0x40);
        codec = AV_CODEC_ID_ADPCM_PSX;
        align = (tracks > 1) ? 0x4000 : 0x8000;
        channels = tracks * ((flags & 0x02) ? 2 : 1);
    }

    if (channels <= 0 || sample_rate <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = sample_rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->bits_per_coded_sample = bps;
    st->codecpar->ch_layout.nb_channels = channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    {
        STRWAVContext *str = s->priv_data;
        char *str_file_name = av_asprintf("%s.str", s->url);
        AVDictionary *tmp = NULL;

        if (!str_file_name)
            return AVERROR(ENOMEM);

        ret = s->io_open(s, &str->pb, str_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&str_file_name);
        if (ret < 0) {
            const int url_len = strlen(s->url);

            if (url_len > 4) {
                str_file_name = av_strdup(s->url);

                if (!str_file_name)
                    return AVERROR(ENOMEM);

                str_file_name[url_len-3] = 's';
                str_file_name[url_len-2] = 't';
                str_file_name[url_len-1] = 'r';

                ret = s->io_open(s, &str->pb, str_file_name, AVIO_FLAG_READ, &tmp);
                av_freep(&str_file_name);
            }
        }
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    STRWAVContext *str = s->priv_data;
    AVStream *st = s->streams[0];
    AVCodecParameters *par = st->codecpar;
    AVIOContext *pb = str->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    STRWAVContext *str = s->priv_data;
    AVStream *st = s->streams[0];
    int block_align, byte_rate;
    int64_t pos;

    if (ts < 0)
        ts = 0;

    block_align = st->codecpar->block_align;
    byte_rate = 36LL * st->codecpar->ch_layout.nb_channels *
                       st->codecpar->sample_rate / 64;
    pos = av_rescale_rnd(ts * byte_rate,
                         st->time_base.num,
                         st->time_base.den * (int64_t)block_align,
                         (flags & AVSEEK_FLAG_BACKWARD) ? AV_ROUND_DOWN : AV_ROUND_UP);
    pos *= block_align;
    ffstream(st)->cur_dts = av_rescale(pos, st->time_base.den, byte_rate * (int64_t)st->time_base.num);
    avio_seek(str->pb, pos, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    STRWAVContext *str = s->priv_data;

    s->io_close2(s, str->pb);

    return 0;
}

const FFInputFormat ff_strwav_demuxer = {
    .p.name         = "strwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Blitz Games STR+WAV"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .priv_data_size = sizeof(STRWAVContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
