/*
 * FSB demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "avformat.h"
#include "avio.h"
#include "demux.h"
#include "internal.h"

static int fsb_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "FSB", 3) || p->buf[3] - '0' < 1 || p->buf[3] - '0' > 5)
        return 0;
    if (AV_RL32(p->buf + 4) != 1)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int fsb_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned format, version, c;
    int minor_version;
    int64_t offset;
    AVCodecParameters *par;
    AVStream *st = avformat_new_stream(s, NULL);
    FFStream *sti;
    int ret;

    avio_skip(pb, 3); // "FSB"
    version = avio_r8(pb) - '0';
    if (version != 5 && version != 4 && version != 3) {
        avpriv_request_sample(s, "version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    minor_version = avio_rl32(pb);

    if (!st)
        return AVERROR(ENOMEM);
    sti = ffstream(st);
    par = st->codecpar;
    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    par->codec_tag   = 0;

    if (version == 3) {
        offset = avio_rl32(pb) + 0x18;
        avio_skip(pb, 44);
        st->duration = avio_rl32(pb);
        avio_skip(pb, 12);
        format = avio_rl32(pb);
        par->sample_rate = avio_rl32(pb);
        if (par->sample_rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 6);
        par->ch_layout.nb_channels = avio_rl16(pb);
        if (!par->ch_layout.nb_channels)
            return AVERROR_INVALIDDATA;

        if (format & 0x00000100) {
            par->codec_id    = AV_CODEC_ID_PCM_S16LE;
            par->block_align = 4096 * par->ch_layout.nb_channels;
        } else if (format & 0x00400000) {
            par->bits_per_coded_sample = 4;
            par->codec_id    = AV_CODEC_ID_ADPCM_IMA_WAV;
            par->block_align = 36 * par->ch_layout.nb_channels;
        } else if (format & 0x00800000) {
            par->codec_id    = AV_CODEC_ID_ADPCM_PSX;
            par->block_align = 16 * par->ch_layout.nb_channels;
        } else if (format & 0x02000000) {
            par->codec_id    = AV_CODEC_ID_ADPCM_THP;
            par->block_align = 8 * par->ch_layout.nb_channels;
            if (par->ch_layout.nb_channels > INT_MAX / 32)
                return AVERROR_INVALIDDATA;
            ret = ff_alloc_extradata(par, 32 * par->ch_layout.nb_channels);
            if (ret < 0)
                return ret;
            avio_seek(pb, 0x68, SEEK_SET);
            for (c = 0; c < par->ch_layout.nb_channels; c++) {
                avio_read(pb, par->extradata + 32 * c, 32);
                avio_skip(pb, 14);
            }
        } else {
            avpriv_request_sample(s, "format 0x%X", format);
            return AVERROR_PATCHWELCOME;
        }
    } else if (version == 4) {
        offset = avio_rl32(pb) + 0x30;
        avio_skip(pb, 80);
        st->duration = avio_rl32(pb);

        format = avio_rb32(pb);
        switch(format) {
        case 0x40001001:
        case 0x00001005:
        case 0x40001081:
        case 0x40200001:
            par->codec_id = AV_CODEC_ID_XMA2;
            break;
        case 0x40000802:
            par->codec_id = AV_CODEC_ID_ADPCM_THP;
            break;
        case 0x40220010:
        case 0x20010:
            par->codec_id = AV_CODEC_ID_MP3;
            break;
        default:
            avpriv_request_sample(s, "format 0x%X", format);
            return AVERROR_PATCHWELCOME;
        }

        par->sample_rate = avio_rl32(pb);
        if (par->sample_rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 6);

        par->ch_layout.nb_channels = avio_rl16(pb);
        if (!par->ch_layout.nb_channels)
            return AVERROR_INVALIDDATA;

        switch (par->codec_id) {
        case AV_CODEC_ID_XMA2:
            par->block_align = 2048;
            ret = ff_alloc_extradata(par, 34);
            if (ret < 0)
                return ret;
            memset(par->extradata, 0, 34);
            AV_WL16(par->extradata, 1);
            sti->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        case AV_CODEC_ID_ADPCM_THP:
            if (par->ch_layout.nb_channels > INT_MAX / 32)
                return AVERROR_INVALIDDATA;
            ret = ff_alloc_extradata(par, 32 * par->ch_layout.nb_channels);
            if (ret < 0)
                return ret;
            avio_seek(pb, 0x80, SEEK_SET);
            for (c = 0; c < par->ch_layout.nb_channels; c++) {
                avio_read(pb, par->extradata + 32 * c, 32);
                avio_skip(pb, 14);
            }
            par->block_align = 8 * par->ch_layout.nb_channels;
            break;
        case AV_CODEC_ID_MP3:
            par->block_align = 1024;
            sti->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        }
    } else if (version == 5) {
        uint64_t sample_mode;
        int sample_header_size;
        int sample_data_size;
        int name_table_size;
        int sample_rate;
        int base_hsize;
        int channels;
        int codec;

        avio_rl32(pb);
        sample_header_size = avio_rl32(pb);
        name_table_size = avio_rl32(pb);
        sample_data_size = avio_rl32(pb);
        codec = avio_rl32(pb);
        if (minor_version == 1)
            base_hsize = 0x3C;
        else
            base_hsize = 0x40;

        avio_seek(pb, base_hsize, SEEK_SET);

        sample_mode = avio_rl64(pb);
        st->duration = ((sample_mode >> 34) & 0x3FFFFFFF);
        offset = ((sample_mode >> 7) & 0x07FFFFFF) << 5;
        offset += base_hsize;
        offset += sample_header_size;
        offset += name_table_size;

        switch ((sample_mode >> 5) & 0x03) {
        case 0:  channels = 1; break;
        case 1:  channels = 2; break;
        case 2:  channels = 6; break;
        case 3:  channels = 8; break;
        default: return AVERROR_INVALIDDATA;
        }

        switch ((sample_mode >> 1) & 0x0f) {
        case 0:  sample_rate = 4000;  break;
        case 1:  sample_rate = 8000;  break;
        case 2:  sample_rate = 11000; break;
        case 3:  sample_rate = 11025; break;
        case 4:  sample_rate = 16000; break;
        case 5:  sample_rate = 22050; break;
        case 6:  sample_rate = 24000; break;
        case 7:  sample_rate = 32000; break;
        case 8:  sample_rate = 44100; break;
        case 9:  sample_rate = 48000; break;
        case 10: sample_rate = 96000; break;
        default: return AVERROR_INVALIDDATA;
        }

        if (sample_mode & 0x01) {
            uint32_t extraflag, extraflag_type, extraflag_size, extraflag_end;

            do {
                extraflag = avio_rl32(pb);
                extraflag_type = (extraflag >> 25) & 0x7F;
                extraflag_size = (extraflag >> 1) & 0xFFFFFF;
                extraflag_end  = extraflag & 0x01;

                if (avio_feof(pb))
                    break;

                switch (extraflag_type) {
                case 0x01:
                    channels = avio_r8(pb);
                    if (channels == 0)
                        return AVERROR_INVALIDDATA;
                    break;
                case 0x02:
                    sample_rate = avio_rl32(pb);
                    if (sample_rate <= 0)
                        return AVERROR_INVALIDDATA;
                    break;
                case 0x05:
                    avio_skip(pb, 4);
                    break;
                case 0x07:
                    switch (codec) {
                    case 0x6:
                        ret = ff_alloc_extradata(par, 32 * channels);
                        if (ret < 0)
                            return ret;

                        for (int ch = 0; ch < channels; ch++) {
                            for (int n = 0; n < 16; n++)
                                AV_WL16(par->extradata + n * 2, avio_rb16(pb));
                            avio_skip(pb, 0xe);
                        }
                        break;
                    }
                    break;
                case 0x0e:
                    channels = channels * avio_rl32(pb);
                    break;
                default:
                    avio_skip(pb, extraflag_size);
                    break;
                }
            } while (extraflag_end != 0);
        }

        par->ch_layout.nb_channels = channels;
        par->sample_rate = sample_rate;

        switch (codec) {
        case 0x01:
            par->codec_id = AV_CODEC_ID_PCM_U8;
            par->block_align = 1024 * channels;
            break;
        case 0x02:
            par->codec_id = AV_CODEC_ID_PCM_S16LE;
            par->block_align = 512 * channels;
            break;
        case 0x05:
            par->codec_id = AV_CODEC_ID_PCM_F32LE;
            par->block_align = 256 * channels;
            break;
        case 0x06:
            par->codec_id = AV_CODEC_ID_ADPCM_THP_LE;
            par->block_align = 0x8 * channels;
            break;
        case 0x0A:
            par->codec_id = AV_CODEC_ID_XMA2;
            par->block_align = 2048;
            ret = ff_alloc_extradata(par, 34);
            if (ret < 0)
                return ret;
            memset(par->extradata, 0, 34);
            AV_WL16(par->extradata, 1);
            sti->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        case 0x0B:
            par->codec_id = AV_CODEC_ID_MP3;
            par->block_align = 1024;
            sti->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        case 0x10:
            par->codec_id = AV_CODEC_ID_ADPCM_FMOD;
            par->block_align = 0x8C * channels;
            break;
        default:
            avpriv_request_sample(s, "codec 0x%X", codec);
            return AVERROR_PATCHWELCOME;
        }
     } else {
        av_assert0(0);
    }

    avio_skip(pb, offset - avio_tell(pb));

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

static int fsb_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int64_t pos;
    int ret;

    if (avio_feof(s->pb))
        return AVERROR_EOF;

    pos = avio_tell(s->pb);
    if (par->codec_id == AV_CODEC_ID_ADPCM_THP &&
        par->ch_layout.nb_channels > 1) {
        int i, ch;

        ret = av_new_packet(pkt, par->block_align);
        if (ret < 0)
            return ret;
        for (i = 0; i < 4; i++) {
            for (ch = 0; ch < par->ch_layout.nb_channels; ch++) {
                pkt->data[ch * 8 + i * 2 + 0] = avio_r8(s->pb);
                pkt->data[ch * 8 + i * 2 + 1] = avio_r8(s->pb);
            }
        }
        ret = 0;
    } else {
        ret = av_get_packet(s->pb, pkt, par->block_align);
    }

    if (par->codec_id == AV_CODEC_ID_XMA2 && pkt->size >= 1)
        pkt->duration = (pkt->data[0] >> 2) * 512;

    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_fsb_demuxer = {
    .p.name         = "fsb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("FMOD Sample Bank"),
    .p.extensions   = "fsb",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe  = fsb_probe,
    .read_header = fsb_read_header,
    .read_packet = fsb_read_packet,
};
