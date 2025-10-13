/*
 * XWB demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct XWBStream {
    int64_t start_offset;
    int64_t stop_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} XWBStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('W','B','N','D') &&
        AV_RL32(p->buf) != MKBETAG('W','B','N','D'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const XWBStream *xs1 = s1->priv_data;
    const XWBStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    XWBStream *xst = opaque;
    AVFormatContext *s = xst->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    XWBStream *xst = opaque;
    AVFormatContext *s = xst->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + xst->start_offset, whence);
}

typedef unsigned int (*avio_r32)(AVIOContext *s);

static int read_header(AVFormatContext *s)
{
    int64_t suboffset, offset, first_start_offset, entry_offset, entry_size, data_offset;
    int entry_alignment = 0, entry_elem_size, version, is_crackdown = 0;
    int nb_streams, ret, bps, rate, channels, tag, block_align = 1024, le;
    int64_t names_offset, names_size, names_entry_size;
    int64_t base_offset, duration = 0;
    uint32_t base_flags, format = 0;
    av_unused uint32_t entry_flags;
    av_unused int64_t extra_offset;
    av_unused int64_t extra_size;
    av_unused int64_t base_size;
    int64_t stream_size, stream_offset;
    AVIOContext *pb = s->pb;
    uint8_t title[65] = {0};
    avio_r32 avio_r32;

    if (avio_rb32(pb) == MKBETAG('W','B','N','D')) {
        avio_r32 = avio_rl32;
        le = 1;
    } else {
        avio_r32 = avio_rb32;
        le = 0;
    }
    version = avio_r32(pb);
    avio_skip(pb, 4);
    if (version == 0x87) {
        version = 41;
        is_crackdown = 1;
    }

    if (version <= 1) {
        nb_streams = avio_r32(pb);
        if (nb_streams <= 0)
            return AVERROR_INVALIDDATA;

        if ((ret = avio_get_str(pb, 0x10, title, sizeof(title))) < 0)
            return ret;
        if (title[0] != '\0')
            av_dict_set(&s->metadata, "title", title, 0);

        base_offset = 0;
        base_size = 0;
        names_offset = 0;
        names_size = 0;
        names_entry_size = 0;
        extra_offset = 0;
        extra_size = 0;
        entry_offset = 0x50;
        entry_elem_size = 0x14;
        entry_size = entry_elem_size * nb_streams;
        data_offset = entry_offset + entry_size;
    } else {
        offset = version <= 41 ? 0x08 : 0x0c;
        avio_seek(pb, offset, SEEK_SET);

        base_offset = avio_r32(pb);
        base_size = avio_r32(pb);
        entry_offset = avio_r32(pb);
        entry_size = avio_r32(pb);

        if (version <= 3) {
            names_offset = avio_r32(pb);
            names_size = avio_r32(pb);
            names_entry_size = 0x40;
            extra_offset = 0;
            extra_size = 0;
            suboffset = 0x04*2;
        } else if (version <= 38) {
            names_offset = avio_r32(pb);
            names_size = avio_r32(pb);
            names_entry_size = 0x40;
            extra_offset = avio_r32(pb);
            extra_size = avio_r32(pb);
            suboffset = 0x04*2 + 0x04*2;
        } else {
            extra_offset = avio_r32(pb);
            extra_size = avio_r32(pb);
            names_offset = avio_r32(pb);
            names_size = avio_r32(pb);
            names_entry_size = 0x40;
            suboffset = 0x04*2 + 0x04*2;
        }

        avio_seek(pb, offset + 0x10 + suboffset, SEEK_SET);
        data_offset = avio_r32(pb);
        avio_skip(pb, 4);

        if (base_offset <= 0)
            return AVERROR_INVALIDDATA;

        offset = base_offset;
        base_flags = avio_r32(pb);
        nb_streams = avio_r32(pb);
        if (nb_streams <= 0)
            return AVERROR_INVALIDDATA;

        if ((ret = avio_get_str(pb, 0x40, title, sizeof(title))) < 0)
            return ret;
        if (title[0] != '\0')
            av_dict_set(&s->metadata, "title", title, 0);

        suboffset = 0x08 + (version <= 3 ? 0x10 : 0x40);
        avio_seek(pb, offset + suboffset, SEEK_SET);
        entry_elem_size = avio_r32(pb);
        avio_skip(pb, 4);
        entry_alignment = avio_r32(pb);
        format = avio_r32(pb);
    }

    for (int si = 0; si < nb_streams; si++) {
        XWBStream *xst;
        AVStream *st;
        int codec;

        avio_seek(pb, entry_offset + si * entry_elem_size, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        if ((base_flags & 0x00020000) && is_crackdown) {
            uint32_t entry, size_sectors, sector_offset;

            entry = avio_r32(pb);
            size_sectors = (entry >> 19) & 0x1FFF;
            sector_offset = entry & 0x7FFFF;
            stream_size = size_sectors * entry_alignment;
            duration = avio_r32(pb);

            stream_offset = data_offset + sector_offset * entry_alignment;
        } else if (base_flags & 0x00020000) {
            av_unused uint32_t size_deviation;
            uint32_t entry, sector_offset;

            entry = avio_r32(pb);
            size_deviation = (entry >> 21) & 0x7FF;
            sector_offset = entry & 0x1FFFFF;
            stream_offset = data_offset + sector_offset * entry_alignment;
            stream_size = 0;
        } else if (version <= 1) {
            format = avio_r32(pb);
            stream_offset = data_offset + avio_r32(pb);
            stream_size = avio_r32(pb);
        } else {
            uint32_t entry_info = avio_r32(pb);

            if (version <= 3) {
                entry_flags = entry_info;
            } else {
                entry_flags = (entry_info) & 0xF;
                duration = (entry_info >> 4) & 0x0FFFFFFF;
            }

            format = avio_r32(pb);
            stream_offset = data_offset + avio_r32(pb);
            stream_size = avio_r32(pb);
        }

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        xst = av_mallocz(sizeof(*xst));
        if (!xst)
            return AVERROR(ENOMEM);
        st->priv_data = xst;

        xst->start_offset = stream_offset;
        if (stream_size > 0) {
            xst->stop_offset = stream_offset;
            xst->stop_offset += stream_size;
        } else {
            xst->stop_offset = 0;
        }

        if (version <= 1) {
            bps = (format >> 31) & 0x1;
            rate = (format >> 4) & 0x7FFFFFF;
            channels = (format >> 1) & 0x7;
            tag = (format) & 0x1;
        } else if (version <= 3) {
            bps = (format >> 31) & 0x1;
            rate = (format >> 5) & 0x3FFFFFF;
            channels = (format >> 2) & 0x7;
            tag = (format) & 0x3;
        } else if (version <= 34) {
            bps = (format >> 31) & 0x1;
            block_align = (format >> 24) & 0xFF;
            rate = (format >> 4) & 0x7FFFF;
            channels = (format >> 1) & 0x7;
            tag = (format) & 0x1;
        } else {
            bps = (format >> 31) & 0x1;
            block_align = (format >> 23) & 0xFF;
            rate = (format >> 5) & 0x3FFFF;
            channels = (format >> 2) & 0x7;
            tag = (format) & 0x3;
        }

        if (rate <= 0 || channels <= 0)
            return AVERROR_INVALIDDATA;

        if (version <= 1) {
            switch (tag){
            case 0:
                switch (bps) {
                case 0:
                    codec = AV_CODEC_ID_PCM_U8;
                    break;
                case 1:
                    codec = le ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
                    break;
                default:
                    return AVERROR_INVALIDDATA;
                }
                break;
            case 1:
                codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        } else if (version <= 3) {
            switch (tag){
            case 0:
                switch (bps) {
                case 0:
                    codec = AV_CODEC_ID_PCM_U8;
                    break;
                case 1:
                    codec = le ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
                    break;
                default:
                    return AVERROR_INVALIDDATA;
                }
                break;
            case 1:
                codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
                break;
            case 2:
                codec = AV_CODEC_ID_WMAV2;
                break;
            case 3:
                codec = 0;
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        } else if (version <= 41) {
            switch (tag) {
            case 0:
                switch (bps) {
                case 0:
                    codec = AV_CODEC_ID_PCM_U8;
                    break;
                case 1:
                    codec = le ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
                    break;
                default:
                    return AVERROR_INVALIDDATA;
                }
                break;
            case 1:
                codec = version <= 38 ? AV_CODEC_ID_XMA1 : AV_CODEC_ID_XMA2;
                break;
            case 2:
                codec = AV_CODEC_ID_ADPCM_MS;
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        } else {
            switch (tag) {
            case 0:
                switch (bps) {
                case 0:
                    codec = AV_CODEC_ID_PCM_U8;
                    break;
                case 1:
                    codec = le ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
                    break;
                default:
                    return AVERROR_INVALIDDATA;
                }
                break;
            case 1:
                codec = AV_CODEC_ID_XMA2;
                break;
            case 2:
                codec = AV_CODEC_ID_ADPCM_MS;
                break;
            case 3:
                codec = 0;
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        }

        if (duration > 0)
            st->duration = duration;
        st->start_time = 0;
        st->codecpar->codec_id = codec;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = block_align * channels;

        if (codec == AV_CODEC_ID_XMA1 || codec == AV_CODEC_ID_XMA2) {
            ret = ff_alloc_extradata(st->codecpar, 34);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, 34);
            AV_WL16(st->codecpar->extradata, 1);
            st->codecpar->block_align = 2048;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        } else if (codec == AV_CODEC_ID_WMAV2) {
            if (!(xst->xctx = avformat_alloc_context()))
                return AVERROR(ENOMEM);

            if ((ret = ff_copy_whiteblacklists(xst->xctx, s)) < 0) {
                avformat_free_context(xst->xctx);
                xst->xctx = NULL;

                return ret;
            }

            ffio_init_context(&xst->apb, NULL, 0, 0, xst,
                              read_data, NULL, seek_data);

            xst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
            xst->xctx->probesize = 0;
            xst->xctx->max_analyze_duration = 0;
            xst->xctx->interrupt_callback = s->interrupt_callback;
            xst->xctx->pb = &xst->apb.pub;
            xst->xctx->io_open = NULL;
            xst->xctx->skip_initial_bytes = 0;
            xst->parent = s;

            avio_seek(pb, xst->start_offset, SEEK_SET);
            ret = avformat_open_input(&xst->xctx, "", NULL, NULL);
            if (ret < 0)
                return ret;

            st->id = xst->xctx->streams[0]->id;
            st->duration = xst->xctx->streams[0]->duration;
            st->time_base = xst->xctx->streams[0]->time_base;
            st->start_time = xst->xctx->streams[0]->start_time;
            st->pts_wrap_bits = xst->xctx->streams[0]->pts_wrap_bits;
            st->codecpar->codec_id = xst->xctx->streams[0]->codecpar->codec_id;
            st->codecpar->bit_rate = xst->xctx->streams[0]->codecpar->bit_rate;
            st->codecpar->sample_rate = xst->xctx->streams[0]->codecpar->sample_rate;
            st->codecpar->block_align = xst->xctx->streams[0]->codecpar->block_align;
            ret = av_channel_layout_copy(&st->codecpar->ch_layout, &xst->xctx->streams[0]->codecpar->ch_layout);
            if (ret < 0)
                return ret;

            ret = ff_alloc_extradata(st->codecpar, xst->xctx->streams[0]->codecpar->extradata_size);
            if (ret < 0)
                return ret;
            memcpy(st->codecpar->extradata, xst->xctx->streams[0]->codecpar->extradata,
                   xst->xctx->streams[0]->codecpar->extradata_size);

            ret = av_dict_copy(&st->metadata, xst->xctx->streams[0]->metadata, 0);
            if (ret < 0)
                return ret;

            ffstream(st)->request_probe = 0;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

            xst->start_offset = avio_tell(pb);
        }

        if (names_offset > 0 && names_entry_size > 0 && names_size > 0) {
            avio_seek(pb, names_offset + si * names_entry_size, SEEK_SET);
            if ((ret = avio_get_str(pb, names_entry_size, title, sizeof(title))) < 0)
                return ret;
            if (title[0] != '\0')
                av_dict_set(&st->metadata, "title", title, 0);
        }
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
        if (n < s->nb_streams-1) {
            AVStream *stn = s->streams[n+1];
            XWBStream *xstn = stn->priv_data;
            XWBStream *xst = st->priv_data;

            if (xst->stop_offset == 0)
                xst->stop_offset = xstn->start_offset;
        } else {
            XWBStream *xst = st->priv_data;

            if (xst->stop_offset == 0)
                xst->stop_offset = avio_size(pb);
        }
    }

    {
        AVStream *st = s->streams[0];
        XWBStream *xst = st->priv_data;

        first_start_offset = xst->start_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        XWBStream *xst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= xst->start_offset && pos < xst->stop_offset) {
            if (xst->xctx) {
                ret = av_read_frame(xst->xctx, pkt);
                pkt->stream_index = st->index;
                if (ret == AVERROR_EOF)
                    continue;
            } else {
                const int size = FFMIN(par->block_align, xst->stop_offset - pos);

                ret = av_get_packet(pb, pkt, size);

                pkt->pos = pos;
                pkt->stream_index = st->index;

                break;
            }
        } else if (pos >= xst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            XWBStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    int current_stream;
    XWBStream *xst;
    AVStream *st;

    current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[current_stream];
    xst = st->priv_data;

    if (xst->xctx)
        return av_seek_frame(xst->xctx, 0, ts, flags);
    else
        return -1;
}

const FFInputFormat ff_xwb_demuxer = {
    .p.name         = "xwb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("XWB (Microsoft Wave Bank)"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xwb",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
