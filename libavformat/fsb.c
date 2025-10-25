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
#include "libavcodec/bytestream.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio.h"
#include "demux.h"
#include "internal.h"
#include "fsbvorbis_data.h"

typedef struct FSBStream {
    int64_t name_offset;
    int64_t start_offset;
    int64_t stop_offset;
    uint8_t *extradata;
    int extradata_size;
} FSBStream;

static int load_vorbis_cb(const uint32_t setup_id, const uint8_t **buf, int *buf_size)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(ff_fsbvb_cb); i++) {
        if (ff_fsbvb_cb[i].id != setup_id)
            continue;

        buf[0] = ff_fsbvb_cb[i].codebooks;
        buf_size[0] = ff_fsbvb_cb[i].size;

        return 0;
    }

    return AVERROR(EINVAL);
}

static int fsb_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "FSB", 3) || p->buf[3] - '0' < 1 || p->buf[3] - '0' > 5)
        return 0;

    if (p->buf[3] - '0' == 5) {
        if (AV_RL32(p->buf + 4) != 0 &&
            AV_RL32(p->buf + 4) != 1)
            return 0;
    } else {
        if ((int32_t)AV_RL32(p->buf + 4) <= 0)
            return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const FSBStream *fs1 = s1->priv_data;
    const FSBStream *fs2 = s2->priv_data;

    return FFDIFFSIGN(fs1->start_offset, fs2->start_offset);
}

static int fsb_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned format, version, nb_streams;
    int minor_version, flags = 0;
    AVCodecParameters *par;
    int64_t offset;
    FSBStream *fst;
    AVStream *st;
    FFStream *sti;
    int ret;

    avio_skip(pb, 3); // "FSB"
    version = avio_r8(pb) - '0';
    if (version != 5 && version != 4 && version != 3 && version != 1) {
        avpriv_request_sample(s, "version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    if (version == 5) {
        minor_version = avio_rl32(pb);
    } else {
        nb_streams = avio_rl32(pb);
    }

    if (version == 1) {
        avio_skip(pb, 8);

        for (int si = 0; si < nb_streams; si++) {
            int32_t loop_start, loop_end;
            char title[0x21] = { 0 };

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            fst = av_mallocz(sizeof(*fst));
            if (!fst)
                return AVERROR(ENOMEM);
            st->priv_data = fst;

            st->start_time = 0;
            sti = ffstream(st);
            par = st->codecpar;
            par->codec_type = AVMEDIA_TYPE_AUDIO;

            ret = avio_get_str(pb, 0x20, title, sizeof(title));
            if (title[0])
                av_dict_set(&st->metadata, "title", title, 0);
            avio_skip(pb, 0x20-ret);

            st->duration = avio_rl32(pb);
            if (si == 0) {
                fst->start_offset = 0x10 + 0x40 * nb_streams;
                offset = fst->start_offset;
            } else {
                FSBStream *fst_prev = s->streams[si-1]->priv_data;

                fst->start_offset = fst_prev->stop_offset;
            }
            fst->stop_offset = fst->start_offset + avio_rl32(pb);

            par->sample_rate = avio_rl32(pb);
            if (par->sample_rate <= 0)
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 8);
            format = avio_rl32(pb);

            loop_start = avio_rl32(pb);
            loop_end = avio_rl32(pb);
            if (loop_start > 0 && loop_start < loop_end) {
                av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
                av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
            }

            par->ch_layout.nb_channels = 1 + !!(format & 0x00000040);
            if (format & 0x00800000) {
                par->codec_id = AV_CODEC_ID_ADPCM_PSX;
                par->block_align = 16 * par->ch_layout.nb_channels;
            } else {
                avpriv_request_sample(s, "format 0x%X", format);
                return AVERROR_PATCHWELCOME;
            }

            avpriv_set_pts_info(st, 64, 1, par->sample_rate);
        }
    } else if (version == 4 || version == 3 || version == 2) {
        int32_t sample_header_min;
        int32_t base_header_size;
        int32_t sample_headers_size;
        int64_t header_offset;
        int64_t extradata_offset;
        int64_t start_offset;
        int64_t stream_size;
        int rate, nb_channels;

        switch (version) {
        case 2:
            sample_header_min = 0x40;
            base_header_size = 0x10;
            break;
        case 3:
            sample_header_min = 0x40;
            base_header_size = 0x18;
            break;
        case 4:
            sample_header_min = 0x50;
            base_header_size = 0x30;
            break;
        }
        header_offset = base_header_size;

        sample_headers_size = avio_rl32(pb);
        start_offset = offset = sample_headers_size + base_header_size;
        avio_skip(pb, 4);
        if (base_header_size > 0x10) {
            minor_version = avio_rl32(pb);
            flags = avio_rl32(pb);
        } else {
            minor_version = flags = 0;
        }

        if (minor_version == 0x00030001) {
            sample_header_min = 0x50;
        } else if (minor_version != 0 && minor_version != 0x00030000  && minor_version != 0x00040000) {
            return AVERROR_INVALIDDATA;
        }

        for (int si = 0; si < nb_streams; si++) {
            uint32_t stream_header_size;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            fst = av_mallocz(sizeof(*fst));
            if (!fst)
                return AVERROR(ENOMEM);
            st->priv_data = fst;

            st->start_time = 0;
            sti = ffstream(st);
            par = st->codecpar;
            par->codec_type  = AVMEDIA_TYPE_AUDIO;
            par->codec_tag   = 0;

            fst->start_offset = start_offset;

            if ((flags & 0x00000002) && si > 0) {
                stream_header_size = 8;
                avio_seek(pb, header_offset, SEEK_SET);
                st->duration = avio_rl32(pb);
                stream_size = avio_rl32(pb);

                if (format & 0x01000000) {
                    avio_skip(pb, 12);
                    stream_header_size += 16 + avio_rl32(pb);
                }
            } else {
                avio_seek(pb, header_offset, SEEK_SET);
                stream_header_size = avio_rl16(pb);
                fst->name_offset = header_offset + 2;
                avio_skip(pb, 30);
                st->duration = avio_rl32(pb);
                stream_size = avio_rl32(pb);
                avio_skip(pb, 8);
                format = avio_rl32(pb);
                rate = avio_rl32(pb);
                avio_skip(pb, 6);
                nb_channels = avio_rl16(pb);
            }

            fst->stop_offset = fst->start_offset + stream_size;

            if ((format & 0x01000000) && minor_version == 0x00040000) {
                par->codec_id = AV_CODEC_ID_XMA2;
            } else if (format & 0x01000000) {
                par->codec_id = AV_CODEC_ID_XMA1;
            } else if (format & 0x02000000) {
                if (flags & 0x00000010)
                    par->codec_id = AV_CODEC_ID_ADPCM_NDSP;
                else
                    par->codec_id = AV_CODEC_ID_ADPCM_NDSP_SI;
            } else if (format & 0x00400000) {
                par->bits_per_coded_sample = 4;
                par->codec_id = AV_CODEC_ID_ADPCM_IMA_XBOX;
                par->block_align = 36 * nb_channels;
            } else if (format & 0x00000200) {
                par->codec_id = AV_CODEC_ID_MP3;
            } else if (format & 0x00800000) {
                par->codec_id = AV_CODEC_ID_ADPCM_PSX;
            } else if (format & 0x00000008) {
                if (format & 0x00000080)
                    par->codec_id = AV_CODEC_ID_PCM_U8;
                else
                    par->codec_id = AV_CODEC_ID_PCM_S8;
            } else if (format & 0x00000010) {
                if (flags & 0x00000008)
                    par->codec_id = AV_CODEC_ID_PCM_S16BE;
                else
                    par->codec_id = AV_CODEC_ID_PCM_S16LE;
            } else {
                avpriv_request_sample(s, "format 0x%X", format);
                return AVERROR_PATCHWELCOME;
            }

            par->sample_rate = rate;
            if (par->sample_rate <= 0)
                return AVERROR_INVALIDDATA;

            par->ch_layout.nb_channels = nb_channels;
            if (!par->ch_layout.nb_channels)
                return AVERROR_INVALIDDATA;

            extradata_offset = header_offset + sample_header_min;

            switch (par->codec_id) {
            case AV_CODEC_ID_XMA1:
                par->block_align = 2048;
                ret = ff_alloc_extradata(par, 8 + 20);
                if (ret < 0)
                    return ret;
                memset(par->extradata, 0, par->extradata_size);
                par->extradata[4] = 1;
                par->extradata[8+17] = nb_channels;
                sti->need_parsing = AVSTREAM_PARSE_FULL;
                break;
            case AV_CODEC_ID_XMA2:
                par->block_align = 2048;
                ret = ff_alloc_extradata(par, 34);
                if (ret < 0)
                    return ret;
                memset(par->extradata, 0, par->extradata_size);
                AV_WL16(par->extradata, 1);
                sti->need_parsing = AVSTREAM_PARSE_FULL_RAW;
                break;
            case AV_CODEC_ID_ADPCM_NDSP:
            case AV_CODEC_ID_ADPCM_NDSP_SI:
                if (par->ch_layout.nb_channels > INT_MAX / 32)
                    return AVERROR_INVALIDDATA;
                ret = ff_alloc_extradata(par, 32 * par->ch_layout.nb_channels);
                if (ret < 0)
                    return ret;
                avio_seek(pb, extradata_offset, SEEK_SET);
                for (int c = 0; c < par->ch_layout.nb_channels; c++) {
                    avio_read(pb, par->extradata + 32 * c, 32);
                    avio_skip(pb, 14);
                }
                par->block_align = 8 * par->ch_layout.nb_channels;
                break;
            case AV_CODEC_ID_ADPCM_PSX:
                par->block_align = 16 * par->ch_layout.nb_channels;
                break;
            case AV_CODEC_ID_MP3:
                par->block_align = 1024;
                sti->need_parsing = AVSTREAM_PARSE_FULL;
                break;
            case AV_CODEC_ID_PCM_S16BE:
            case AV_CODEC_ID_PCM_S16LE:
            case AV_CODEC_ID_PCM_S8:
            case AV_CODEC_ID_PCM_U8:
                par->block_align = 1024 * par->ch_layout.nb_channels;
                break;
            }

            avpriv_set_pts_info(st, 64, 1, par->sample_rate);

            header_offset += stream_header_size;
            start_offset += stream_size;

            if (flags & 0x00000040) {
                if (start_offset & 31)
                    start_offset += 32 - (start_offset & 31);
            }
        }

        for (int si = 0; si < nb_streams; si++) {
            AVStream *st = s->streams[si];
            FSBStream *fst = st->priv_data;
            char title[31] = { 0 };

            avio_seek(pb, fst->name_offset, SEEK_SET);

            avio_get_str(pb, 30, title, sizeof(title));

            if (title[0])
                av_dict_set(&st->metadata, "title", title, 0);
        }
    } else if (version == 5) {
        uint64_t sample_mode;
        int64_t start_offset;
        int64_t sample_header_size;
        int64_t sample_data_size;
        int64_t name_table_size;
        int sample_rate;
        int base_hsize;
        int channels;
        int codec;

        nb_streams = avio_rl32(pb);
        sample_header_size = avio_rl32(pb);
        name_table_size = avio_rl32(pb);
        sample_data_size = avio_rl32(pb);
        codec = avio_rl32(pb);
        if (minor_version == 1) {
            base_hsize = 0x3C;
            avio_skip(pb, 4);
            flags = avio_rl32(pb);
        } else {
            flags = 0;
            base_hsize = 0x40;
        }

        avio_seek(pb, base_hsize, SEEK_SET);

        for (int si = 0; si < nb_streams; si++) {
            FSBStream *fst;

            if (avio_feof(pb))
                return AVERROR_EOF;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            fst = av_mallocz(sizeof(*fst));
            if (!fst)
                return AVERROR(ENOMEM);

            st->start_time = 0;
            st->priv_data = fst;
            sti = ffstream(st);
            par = st->codecpar;
            par->codec_type  = AVMEDIA_TYPE_AUDIO;
            par->codec_tag   = 0;

            sample_mode = avio_rl64(pb);
            st->duration = ((sample_mode >> 34) & 0x3FFFFFFF);
            start_offset = ((sample_mode >> 7) & 0x07FFFFFF) << 5;
            start_offset += base_hsize;
            start_offset += sample_header_size;
            start_offset += name_table_size;
            if (si == 0)
                offset = start_offset;
            fst->start_offset = start_offset;

            if (si > 0) {
                FSBStream *fst = s->streams[si-1]->priv_data;

                fst->stop_offset = start_offset;
            }

            if (si == nb_streams-1)
                fst->stop_offset = offset + sample_data_size;

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
                    case 0x07:
                        switch (codec) {
                        case 0x06:
                            {
                                const int blocks = extraflag_size / 46;

                                ret = ff_alloc_extradata(par, 32 * blocks);
                                if (ret < 0)
                                    return ret;

                                for (int ch = 0; ch < blocks; ch++) {
                                    avio_read(pb, par->extradata + ch * 32, 32);
                                    avio_skip(pb, 14);
                                }
                            }
                            break;
                        }
                        break;
                    case 0x09:
                        switch (codec) {
                        case 0x0D:
                            ret = ff_alloc_extradata(par, 12);
                            if (ret < 0)
                                return ret;
                            memset(par->extradata, 0, par->extradata_size);

                            avio_read(pb, par->extradata + 4, 4);
                            extraflag_size -= 4;
                            if (par->extradata[4] != 0xFE && extraflag_size >= 4) {
                                par->block_align = AV_RL16(par->extradata + 4);
                                avio_read(pb, par->extradata + 4, 4);
                                extraflag_size -= 4;
                            } else {
                                avio_read(pb, par->extradata + 8, 4);
                                extraflag_size -= 4;
                            }

                            break;
                        }
                        avio_skip(pb, extraflag_size);
                        break;
                    case 0x0b:
                        fst->extradata = av_calloc(extraflag_size, sizeof(*fst->extradata));
                        if (!fst->extradata)
                            return AVERROR(ENOMEM);
                        avio_read(pb, fst->extradata, extraflag_size);
                        fst->extradata_size = extraflag_size;
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
                par->codec_id = (flags & 0x01) ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
                par->block_align = 512 * channels;
                break;
            case 0x03:
                par->codec_id = (flags & 0x01) ? AV_CODEC_ID_PCM_S24BE : AV_CODEC_ID_PCM_S24LE;
                par->block_align = 768 * channels;
                break;
            case 0x04:
                par->codec_id = (flags & 0x01) ? AV_CODEC_ID_PCM_S32BE : AV_CODEC_ID_PCM_S32LE;
                par->block_align = 512 * channels;
                break;
            case 0x05:
                par->codec_id = (flags & 0x01) ? AV_CODEC_ID_PCM_F32BE : AV_CODEC_ID_PCM_F32LE;
                par->block_align = 256 * channels;
                break;
            case 0x06:
                par->codec_id = (flags & 0x02) ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_SI;
                par->block_align = 0x8 * channels;
                break;
            case 0x07:
                par->codec_id = (channels > 2) ? AV_CODEC_ID_NONE : AV_CODEC_ID_ADPCM_IMA_XBOX;
                par->bits_per_coded_sample = 4;
                par->block_align = 0x24 * channels;
                break;
            case 0x08:
                if (flags & 0x02) {
                    avpriv_request_sample(s, "block layered ADPCM PSX");
                    return AVERROR_PATCHWELCOME;
                }
                par->codec_id = AV_CODEC_ID_ADPCM_PSX;
                par->block_align = 0x10 * channels;
                break;
            case 0x09:
                par->codec_id = AV_CODEC_ID_ADPCM_HEVAG;
                par->block_align = 0x10 * channels;
                break;
            case 0x0A:
                par->codec_id = AV_CODEC_ID_XMA2;
                par->block_align = 2048;
                ret = ff_alloc_extradata(par, 34);
                if (ret < 0)
                    return ret;
                memset(par->extradata, 0, par->extradata_size);
                AV_WL16(par->extradata, 1);
                sti->need_parsing = AVSTREAM_PARSE_FULL_RAW;
                break;
            case 0x0B:
                par->codec_id = AV_CODEC_ID_MP3;
                par->block_align = 1024;
                sti->need_parsing = AVSTREAM_PARSE_FULL;
                break;
            case 0x0D:
                par->codec_id = AV_CODEC_ID_ATRAC9;
                if (par->block_align == 0)
                    par->block_align = 1024;
                sti->need_parsing = AVSTREAM_PARSE_FULL;
                break;
            case 0x0F:
                par->codec_id = AV_CODEC_ID_VORBIS;
                sti->need_parsing = AVSTREAM_PARSE_FULL;
                par->block_align = 0;
                {
                    uint32_t setup_id = AV_RL32(fst->extradata);
                    const uint8_t *cb = NULL;
                    PutByteContext pbc;
                    uint8_t block_size;
                    int cb_size = 0;
                    int bl0, bl1;

                    ret = load_vorbis_cb(setup_id, &cb, &cb_size);
                    if (ret < 0)
                        return ret;

                    ret = ff_alloc_extradata(par, cb_size + 2048);
                    if (ret < 0)
                        return ret;

                    bl0 = av_log2(2048);
                    bl1 = av_log2(256);
                    block_size = (bl0 << 4) | bl1;

                    bytestream2_init_writer(&pbc, par->extradata, par->extradata_size);
                    bytestream2_put_be16(&pbc, 30);
                    bytestream2_put_byte(&pbc, 0x01);
                    bytestream2_put_buffer(&pbc, "vorbis", 6);
                    bytestream2_put_le32(&pbc, 0);
                    bytestream2_put_byte(&pbc, channels);
                    bytestream2_put_le32(&pbc, sample_rate);
                    bytestream2_put_le32(&pbc, 0);
                    bytestream2_put_le32(&pbc, 0);
                    bytestream2_put_le32(&pbc, 0);
                    bytestream2_put_byte(&pbc, block_size);
                    bytestream2_put_byte(&pbc, 0x01);

                    bytestream2_put_be16(&pbc, 20);
                    bytestream2_put_byte(&pbc, 0x03);
                    bytestream2_put_buffer(&pbc, "vorbis", 6);
                    bytestream2_put_le32(&pbc, 0x04);
                    bytestream2_put_buffer(&pbc, "lavf", 4);
                    bytestream2_put_le32(&pbc, 0x00);
                    bytestream2_put_byte(&pbc, 0x01);

                    bytestream2_put_be16(&pbc, cb_size);
                    bytestream2_put_buffer(&pbc, cb, cb_size);

                    par->extradata_size = bytestream2_tell_p(&pbc);

                    av_freep(&fst->extradata);
                    fst->extradata_size = 0;
                }
                break;
            case 0x10:
                par->codec_id = AV_CODEC_ID_ADPCM_FMOD;
                par->block_align = 0x8C * channels;
                break;
            default:
                avpriv_request_sample(s, "codec 0x%X", codec);
                return AVERROR_PATCHWELCOME;
            }

            avpriv_set_pts_info(st, 64, 1, par->sample_rate);
        }

        avio_seek(pb, base_hsize + sample_header_size, SEEK_SET);

        if (name_table_size > 0) {
            for (int si = 0; si < nb_streams; si++) {
                FSBStream *fst = s->streams[si]->priv_data;

                if (avio_feof(pb))
                    return AVERROR_EOF;

                fst->name_offset  = base_hsize + sample_header_size;
                fst->name_offset += avio_rl32(pb);
            }

            for (int si = 0; si < nb_streams; si++) {
                AVStream *st = s->streams[si];
                FSBStream *fst = st->priv_data;
                char title[1025] = { 0 };

                avio_seek(pb, fst->name_offset, SEEK_SET);

                avio_get_str(pb, INT_MAX, title, sizeof(title));

                if (title[0])
                    av_dict_set(&st->metadata, "title", title, 0);
            }
        }
    } else {
        av_assert0(0);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int fsb_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        FSBStream *fst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= fst->start_offset && pos < fst->stop_offset) {
            if (par->block_align > 0) {
                ret = av_get_packet(pb, pkt, par->block_align);
            } else {
                const int size = avio_rl16(pb);

                ret = av_get_packet(pb, pkt, size);
            }

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= fst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            FSBStream *fst_next = st_next->priv_data;

            if (fst_next->start_offset > pos)
                avio_skip(pb, fst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_fsb_demuxer = {
    .p.name         = "fsb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("FMOD Sample Bank"),
    .p.extensions   = "fsb",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = fsb_probe,
    .read_header    = fsb_read_header,
    .read_packet    = fsb_read_packet,
};
