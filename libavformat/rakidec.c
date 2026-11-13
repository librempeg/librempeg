/*
 * UbiArt RAKI demuxer
 * Copyright (c) 2026 Martijn Brouwer
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

#include "libavutil/common.h"
#include "libavutil/intreadwrite.h"

#include "avformat.h"
#include "avio.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "riff.h"
#include "pcm.h"

#define RAKI_MAX_CHUNKS 64

typedef struct RakiChunk {
    int64_t offset;
    int64_t size;
    int present;
} RakiChunk;

enum RakiPacketType {
    RAKI_PACKETS_CONTIGUOUS,
    RAKI_PACKETS_NX_OPUS,
};

typedef struct RakiDemuxContext {
    int big_endian;
    int packet_type;
    int64_t raki_offset;
    int64_t data_start;
    int64_t data_offset;

    RakiChunk fmt;
    RakiChunk data;
    RakiChunk adin;
    RakiChunk dspl;
    RakiChunk dspr;
    RakiChunk dats;
    RakiChunk datl;
    RakiChunk datr;
    RakiChunk msf;
} RakiDemuxContext;

static unsigned read_u16(AVIOContext *pb, int big_endian)
{
    return big_endian ? avio_rb16(pb) : avio_rl16(pb);
}

static unsigned read_u32(AVIOContext *pb, int big_endian)
{
    return big_endian ? avio_rb32(pb) : avio_rl32(pb);
}

static int raki_platform_is_big_endian(uint32_t platform)
{
    return platform == MKTAG('W', 'i', 'i', ' ') ||
           platform == MKTAG('C', 'a', 'f', 'e') ||
           platform == MKTAG('P', 'S', '3', ' ') ||
           platform == MKTAG('X', '3', '6', '0');
}

static int read_probe(const AVProbeData *p)
{
    uint32_t platform, chunks;
    const uint8_t *buf;
    int big_endian;

    if (p->buf_size < 32)
        return 0;

    if (AV_RB32(p->buf) == MKBETAG('R','A','K','I'))
        buf = p->buf;
    else if (AV_RB32(p->buf + 4) == MKBETAG('R','A','K','I'))
        buf = p->buf + 4;
    else
        return 0;

    platform = AV_RL32(buf + 8);
    big_endian = raki_platform_is_big_endian(platform);
    chunks = big_endian ? AV_RB32(buf + 0x18) : AV_RL32(buf + 0x18);

    if (!chunks)
        return 0;

    if (chunks > RAKI_MAX_CHUNKS)
        return AVPROBE_SCORE_MAX/4;

    return AVPROBE_SCORE_MAX;
}

static int raki_set_chunk(RakiDemuxContext *raki, uint32_t id,
                          int64_t offset, int64_t size)
{
    RakiChunk *chunk = NULL;

    switch (id) {
    case MKTAG('f', 'm', 't', ' '): chunk = &raki->fmt;  break;
    case MKTAG('d', 'a', 't', 'a'): chunk = &raki->data; break;
    case MKTAG('A', 'd', 'I', 'n'): chunk = &raki->adin; break;
    case MKTAG('d', 's', 'p', 'L'): chunk = &raki->dspl; break;
    case MKTAG('d', 's', 'p', 'R'): chunk = &raki->dspr; break;
    case MKTAG('d', 'a', 't', 'S'): chunk = &raki->dats; break;
    case MKTAG('d', 'a', 't', 'L'): chunk = &raki->datl; break;
    case MKTAG('d', 'a', 't', 'R'): chunk = &raki->datr; break;
    case MKTAG('M', 's', 'f', ' '): chunk = &raki->msf;  break;
    default:
        return 0;
    }

    if (chunk->present)
        return AVERROR_INVALIDDATA;

    chunk->offset  = offset;
    chunk->size    = size;
    chunk->present = 1;

    return 0;
}

static int raki_seek_chunk(AVFormatContext *s, const RakiChunk *chunk)
{
    if (avio_seek(s->pb, chunk->offset, SEEK_SET) < 0)
        return AVERROR(EIO);
    return 0;
}

static enum AVCodecID raki_pcm_codec_id(unsigned tag, unsigned bits, int big_endian)
{
    if (tag == 1) {
        switch (bits) {
        case 8:  return AV_CODEC_ID_PCM_U8;
        case 16: return big_endian ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
        case 24: return big_endian ? AV_CODEC_ID_PCM_S24BE : AV_CODEC_ID_PCM_S24LE;
        case 32: return big_endian ? AV_CODEC_ID_PCM_S32BE : AV_CODEC_ID_PCM_S32LE;
        }
    } else if (tag == 3) {
        switch (bits) {
        case 32: return big_endian ? AV_CODEC_ID_PCM_F32BE : AV_CODEC_ID_PCM_F32LE;
        case 64: return big_endian ? AV_CODEC_ID_PCM_F64BE : AV_CODEC_ID_PCM_F64LE;
        }
    }

    return AV_CODEC_ID_NONE;
}

static int raki_is_pcm_codec(enum AVCodecID codec_id)
{
    switch (codec_id) {
    case AV_CODEC_ID_PCM_U8:
    case AV_CODEC_ID_PCM_S16LE:
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_S24LE:
    case AV_CODEC_ID_PCM_S24BE:
    case AV_CODEC_ID_PCM_S32LE:
    case AV_CODEC_ID_PCM_S32BE:
    case AV_CODEC_ID_PCM_F32LE:
    case AV_CODEC_ID_PCM_F32BE:
    case AV_CODEC_ID_PCM_F64LE:
    case AV_CODEC_ID_PCM_F64BE:
        return 1;
    default:
        return 0;
    }
}

static int raki_set_data_region(AVFormatContext *s, const RakiChunk *chunk)
{
    RakiDemuxContext *raki = s->priv_data;

    raki->data_offset = chunk->offset;
    if (avio_seek(s->pb, raki->data_offset, SEEK_SET) < 0)
        return AVERROR(EIO);

    return 0;
}

static int raki_read_wave_format(AVFormatContext *s, AVStream *st)
{
    RakiDemuxContext *raki = s->priv_data;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par = st->codecpar;
    const RakiChunk *payload = raki->msf.present ? &raki->msf : &raki->data;
    unsigned tag = 0, channels, sample_rate, block_align, bits;
    int frame_duration;
    int ret;

    ret = raki_seek_chunk(s, &raki->fmt);
    if (ret < 0)
        return ret;

    if (!raki->big_endian) {
        if (raki->fmt.size > INT_MAX)
            return AVERROR_INVALIDDATA;
        ret = ff_get_wav_header(s, pb, par, raki->fmt.size, 0);
        if (ret < 0)
            return ret;
    } else {
        enum AVCodecID codec_id;

        tag          = read_u16(pb, 1);
        channels     = read_u16(pb, 1);
        sample_rate  = read_u32(pb, 1);
        avio_skip(pb, 4);
        block_align  = read_u16(pb, 1);
        bits         = read_u16(pb, 1);

        codec_id = raki_pcm_codec_id(tag, bits, 1);
        if (codec_id == AV_CODEC_ID_NONE) {
            if (tag == 0x0055)
                codec_id = AV_CODEC_ID_MP3;
            else {
                avpriv_request_sample(s, "big-endian WAVE format 0x%04x", tag);
                return AVERROR_PATCHWELCOME;
            }
        }

        par->codec_type            = AVMEDIA_TYPE_AUDIO;
        par->codec_id              = codec_id;
        par->codec_tag             = tag;
        par->sample_rate           = sample_rate;
        par->block_align           = block_align;
        par->bits_per_coded_sample = bits;
        av_channel_layout_default(&par->ch_layout, channels);
    }

    if (par->codec_id != AV_CODEC_ID_ADPCM_MS &&
        par->codec_id != AV_CODEC_ID_MP3 &&
        !raki_is_pcm_codec(par->codec_id)) {
        avpriv_request_sample(s, "WAVE format 0x%04x", par->codec_tag);
        return AVERROR_PATCHWELCOME;
    }
    if (!par->ch_layout.nb_channels || par->ch_layout.nb_channels > 64 ||
        !par->sample_rate || !par->block_align)
        return AVERROR_INVALIDDATA;

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    ret = raki_set_data_region(s, payload);
    if (ret < 0)
        return ret;

    if (par->codec_id == AV_CODEC_ID_MP3) {
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        par->block_align = 4096;
    } else if (par->codec_id == AV_CODEC_ID_ADPCM_MS) {
        frame_duration = av_get_audio_frame_duration2(par, par->block_align);
        if (frame_duration <= 0)
            return AVERROR_INVALIDDATA;

        st->duration = payload->size / par->block_align * frame_duration;
    } else {
        st->duration = payload->size / par->block_align;
    }

    return 0;
}

static int raki_read_xma2(AVFormatContext *s, AVStream *st)
{
    RakiDemuxContext *raki = s->priv_data;
    AVCodecParameters *par = st->codecpar;
    AVIOContext *pb = s->pb;
    unsigned play_begin, play_length, loop_begin, loop_length, block_count;
    unsigned channel_mask, samples_encoded, bytes_per_block;
    int channels, sample_rate, num_streams;
    unsigned tag, extra_size;
    uint8_t *extra;
    int ret;

    ret = raki_seek_chunk(s, &raki->fmt);
    if (ret < 0)
        return ret;

    tag          = read_u16(pb, raki->big_endian);
    channels     = read_u16(pb, raki->big_endian);
    sample_rate  = read_u32(pb, raki->big_endian);
    avio_skip(pb, 4);
    avio_skip(pb, 2); /* WAVE block alignment */
    avio_skip(pb, 2);
    extra_size   = read_u16(pb, raki->big_endian);
    num_streams  = read_u16(pb, raki->big_endian);
    channel_mask = read_u32(pb, raki->big_endian);
    samples_encoded = read_u32(pb, raki->big_endian);
    bytes_per_block = read_u32(pb, raki->big_endian);
    play_begin   = read_u32(pb, raki->big_endian);
    play_length  = read_u32(pb, raki->big_endian);
    loop_begin   = read_u32(pb, raki->big_endian);
    loop_length  = read_u32(pb, raki->big_endian);

    if (tag != 0x0166 || extra_size != 34 || channels <= 0 ||
        num_streams <= 0 || channels < num_streams ||
        channels > 2 * num_streams || sample_rate <= 0 ||
        (channel_mask && av_popcount(channel_mask) != channels) ||
        !samples_encoded || play_begin > samples_encoded ||
        (play_length && play_length > samples_encoded - play_begin) ||
        (loop_length && (loop_begin > samples_encoded ||
                         loop_length > samples_encoded - loop_begin)) || !bytes_per_block)
        return AVERROR_INVALIDDATA;

    ret = ff_alloc_extradata(par, 34);
    if (ret < 0)
        return ret;

    extra = par->extradata;
    AV_WL16(extra,      num_streams);
    AV_WL32(extra + 2,  channel_mask);
    AV_WL32(extra + 6,  samples_encoded);
    AV_WL32(extra + 10, bytes_per_block);
    AV_WL32(extra + 14, play_begin);
    AV_WL32(extra + 18, play_length);
    AV_WL32(extra + 22, loop_begin);
    AV_WL32(extra + 26, loop_length);
    extra[30] = avio_r8(pb);
    extra[31] = avio_r8(pb);
    block_count = read_u16(pb, raki->big_endian);
    AV_WL16(extra + 32, block_count);

    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id = AV_CODEC_ID_XMA2;
    par->codec_tag = tag;
    par->sample_rate = sample_rate;
    par->block_align = 0x800;
    if (channel_mask) {
        ret = av_channel_layout_from_mask(&par->ch_layout, channel_mask);
        if (ret < 0)
            return ret;
    } else {
        av_channel_layout_default(&par->ch_layout, channels);
    }
    avpriv_set_pts_info(st, 64, 1, sample_rate);
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    st->duration = play_length ? play_length : samples_encoded;

    ret = raki_set_data_region(s, &raki->data);
    if (ret < 0)
        return ret;

    return 0;
}

static int raki_read_dsp(AVFormatContext *s, AVStream *st)
{
    RakiDemuxContext *raki = s->priv_data;
    AVCodecParameters *par = st->codecpar;
    int channels, sample_rate;
    AVIOContext *pb = s->pb;
    int ret;

    ret = raki_seek_chunk(s, &raki->fmt);
    if (ret < 0)
        return ret;

    avio_skip(pb, 2);
    channels = read_u16(pb, raki->big_endian);
    sample_rate = read_u32(pb, raki->big_endian);

    if (channels <= 0 || sample_rate <= 0 || channels >= INT_MAX/8)
        return AVERROR_INVALIDDATA;

    st->start_time = 0;
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    par->ch_layout.nb_channels = channels;
    par->sample_rate = sample_rate;
    par->block_align = 8 * channels;
    par->bit_rate = 8LL * channels * 8 * sample_rate / 14;

    ret = ff_alloc_extradata(par, 32 * channels);
    if (ret < 0)
        return ret;

    if (avio_seek(pb, 0x30 + raki->raki_offset, SEEK_SET) < 0)
        return AVERROR(EIO);

    int64_t coeff_offset = avio_rb32(pb);
    for (int ch = 0; ch < channels; ch++) {
        if (avio_seek(pb, coeff_offset + 0x1c + 0x60 * ch, SEEK_SET) < 0)
            return AVERROR(EIO);

        ret = ffio_read_size(pb, par->extradata + 32 * ch, 32);
        if (ret < 0)
            return ret;
    }

    avpriv_set_pts_info(st, 64, 1, sample_rate);

    if (avio_seek(pb, raki->data_start, SEEK_SET) < 0)
        return AVERROR(EIO);

    return 0;
}

static int raki_read_nx_opus(AVFormatContext *s, AVStream *st)
{
    RakiDemuxContext *raki = s->priv_data;
    AVCodecParameters *par = st->codecpar;
    unsigned data_info_offset, data_signature, pre_skip, payload_size;
    unsigned signature, header_size, channel_field, input_rate;
    AVIOContext *pb = s->pb;
    int channels, ret;

    ret = raki_seek_chunk(s, &raki->data);
    if (ret < 0)
        return ret;

    signature = avio_rl32(pb);
    header_size = avio_rl32(pb);
    channel_field = avio_rl32(pb);
    channels = channel_field >> 8;
    input_rate = avio_rl32(pb);
    data_info_offset = avio_rl32(pb);
    avio_skip(pb, 8);
    pre_skip = avio_rl32(pb);

    if (signature != 0x80000001 || header_size < 0x18 ||
        channel_field != channels << 8 || channels <= 0 ||
        !input_rate || input_rate > INT_MAX || pre_skip > UINT16_MAX ||
        data_info_offset < 0x20 ||
        data_info_offset > raki->data.size - 8 ||
        header_size > data_info_offset - 8)
        return AVERROR_INVALIDDATA;
    if (avio_seek(pb, raki->data.offset + data_info_offset, SEEK_SET) < 0)
        return AVERROR(EIO);
    data_signature = avio_rl32(pb);
    payload_size   = avio_rl32(pb);
    if (data_signature != 0x80000004 || !payload_size ||
        payload_size > raki->data.size - data_info_offset - 8)
        return AVERROR_INVALIDDATA;

    ret = ff_alloc_extradata(par, 19);
    if (ret < 0)
        return ret;
    memcpy(par->extradata, "OpusHead", 8);
    par->extradata[8] = 1;
    par->extradata[9] = channels;
    AV_WL16(par->extradata + 10, pre_skip);
    AV_WL32(par->extradata + 12, input_rate);
    AV_WL16(par->extradata + 16, 0);
    par->extradata[18] = 0;

    par->codec_type      = AVMEDIA_TYPE_AUDIO;
    par->codec_id        = AV_CODEC_ID_OPUS;
    par->sample_rate     = 48000;
    par->initial_padding = pre_skip;
    par->seek_preroll    = 3840;
    av_channel_layout_default(&par->ch_layout, channels);
    avpriv_set_pts_info(st, 64, 1, 48000);
    ffstream(st)->start_skip_samples = pre_skip;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;

    if (raki->adin.present) {
        unsigned samples;

        if (avio_seek(pb, raki->adin.offset, SEEK_SET) < 0)
            return AVERROR(EIO);

        samples = avio_rl32(pb);
        if (!samples)
            return AVERROR_INVALIDDATA;

        int64_t expected_samples = av_rescale(samples, 48000, input_rate);
        if (expected_samples)
            st->duration = expected_samples;
    }

    raki->packet_type = RAKI_PACKETS_NX_OPUS;
    raki->data_offset = raki->data.offset + data_info_offset + 8;
    if (avio_seek(pb, raki->data_offset, SEEK_SET) < 0)
        return AVERROR(EIO);

    return 0;
}

static int read_header(AVFormatContext *s)
{
    uint32_t type, header_size, chunk_count;
    RakiDemuxContext *raki = s->priv_data;
    AVIOContext *pb = s->pb;
    uint64_t platform;
    int64_t table_end;
    AVStream *st;
    int ret;

    if (avio_rl32(pb) == MKTAG('R', 'A', 'K', 'I')) {
        raki->raki_offset = 0;
    } else {
        if (avio_rl32(pb) != MKTAG('R', 'A', 'K', 'I'))
            return AVERROR_INVALIDDATA;
        raki->raki_offset = 4;
    }

    avio_skip(pb, 4); /* version */
    platform = avio_rl32(pb);
    type     = avio_rl32(pb);
    raki->big_endian = raki_platform_is_big_endian(platform);
    header_size = read_u32(pb, raki->big_endian);
    raki->data_start = read_u32(pb, raki->big_endian);
    chunk_count = read_u32(pb, raki->big_endian);
    avio_skip(pb, 4);

    if (!chunk_count || chunk_count > RAKI_MAX_CHUNKS)
        return AVERROR_INVALIDDATA;
    table_end = raki->raki_offset + 0x20 + 12LL * chunk_count;
    if (table_end > header_size || header_size > raki->data_start)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < chunk_count; i++) {
        uint32_t id = avio_rl32(pb);
        uint32_t offset = read_u32(pb, raki->big_endian);
        uint32_t size = read_u32(pb, raki->big_endian);

        ret = raki_set_chunk(raki, id, offset, size);
        if (ret < 0)
            return ret;
    }

    if (!raki->data.present) {
        raki->data.offset  = raki->data_start;
        raki->data.present = 1;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    switch ((platform << 32) | type) {
    case ((((uint64_t)MKTAG('N', 'x', ' ', ' ')) << 32) | MKTAG('N', 'x', ' ', ' ')):
        ret = raki_read_nx_opus(s, st);
        break;
    case ((((uint64_t)MKTAG('W', 'i', 'i', ' ')) << 32) | MKTAG('a', 'd', 'p', 'c')):
    case ((((uint64_t)MKTAG('C', 'a', 'f', 'e')) << 32) | MKTAG('a', 'd', 'p', 'c')):
        ret = raki_read_dsp(s, st);
        break;
    case ((((uint64_t)MKTAG('X', '3', '6', '0')) << 32) | MKTAG('x', 'm', 'a', '2')):
    case ((((uint64_t)MKTAG('D', 'u', 'r', 'a')) << 32) | MKTAG('x', 'm', 'a', '2')):
    case ((((uint64_t)MKTAG('S', 'c', 'a', 'r')) << 32) | MKTAG('x', 'm', 'a', '2')):
        ret = raki_read_xma2(s, st);
        break;
    case ((((uint64_t)MKTAG('P', 'S', '3', ' ')) << 32) | MKTAG('p', 'c', 'm', ' ')):
    case ((((uint64_t)MKTAG('P', 'S', '3', ' ')) << 32) | MKTAG('m', 'p', '3', ' ')):
    case ((((uint64_t)MKTAG('P', 'S', '3', ' ')) << 32) | MKTAG('a', 'd', 'p', 'c')):
    case ((((uint64_t)MKTAG('W', 'i', 'n', ' ')) << 32) | MKTAG('p', 'c', 'm', ' ')):
    case ((((uint64_t)MKTAG('W', 'i', 'n', ' ')) << 32) | MKTAG('a', 'd', 'p', 'c')):
    case ((((uint64_t)MKTAG('W', 'i', 'n', ' ')) << 32) | MKTAG('m', 'p', '3', ' ')):
    case ((((uint64_t)MKTAG('W', 'i', 'i', ' ')) << 32) | MKTAG('p', 'c', 'm', ' ')):
        ret = raki_read_wave_format(s, st);
        break;
    default:
        avpriv_request_sample(s, "platform/type: 0x%08lx/0x%08x", platform, type);
        return AVERROR_PATCHWELCOME;
    }

    return ret;
}

static int raki_read_nx_opus_packet(AVIOContext *pb, AVPacket *pkt)
{
    int size;

    size = avio_rb32(pb);
    avio_skip(pb, 4);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (size <= 0)
        return AVERROR_INVALIDDATA;

    return av_get_packet(pb, pkt, size);
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    RakiDemuxContext *raki = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    switch (raki->packet_type) {
    case RAKI_PACKETS_NX_OPUS:
        ret = raki_read_nx_opus_packet(pb, pkt);
        break;
    default:
        ret = ff_pcm_read_packet(s, pkt);
        break;
    }

    if (ret >= 0) {
        pkt->stream_index = 0;
        pkt->pos = pos;
    }

    return ret;
}

const FFInputFormat ff_raki_demuxer = {
    .p.name         = "raki",
    .p.long_name    = NULL_IF_CONFIG_SMALL("UbiArt RAKI"),
    .p.extensions   = "raki,rak,ckd",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(RakiDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
