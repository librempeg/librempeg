/*
 * SquareSoft->SquareEnix PS2 FMV video+audio demuxer
 * Copyright (c) 2026 librempeg
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

#include "avio.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/intfloat.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "libavcodec/ipu_instructions.h"
#include "libavcodec/packet_internal.h"

/*
 * todo list for this demuxer:
 *
 * 1. dat+sz file format can house B-picture IPU frame data,
 *    may require uploaded (as-is) I-picture IPU frame data to parse it correctly tho.
 *
 * 2. dat file format can house multiple I-picture IPU frames in one group out of many.
 *    example: group 1 = 9 frames, group 2 = 14 frames, group 3 = 21 frames, group 4 = 11 frames, etc.
 *    SOLVED by processing them like such.
 *
 * 3. dat+sz file format can house PSX ADPCM data, always fixed to 0x1c30 size.
 *    layout of said data goes as follows:
 *    0x00-0x0f - 2 int32 hists per channel (in practice 2x2 int32 vars)
 *    0x10-0x1c2f - PSX ADPCM with block size of (9 * 4) = 36 bytes per channel.
 *    (in practice it's (9 * 4) * 2 = 72 bytes).
 *
 * 4. dat+sz file format can house IMA ADPCM data, always fixed to 0x1900 size.
 *    (said data, however, is just not present in any SQUARESOFT PS2 game period)
 *    block size is fixed to 128 bytes per channel (in practice, (128 * 2) = 256 bytes)
 *    and has no header, literally every byte is just 4-bit encoded sample data.
 *
 * 5. dat+sz file format can house raw PCM16+AC-3 data into one grouped block.
 *    said block may hold either one or two audio tracks into both PCM16+AC-3 portions.
 *
 *    example:
 *    PCM16 audio track 1 is within 0-0x6400 range,
 *    PCM16 audio track 2 is within 0x6400-0xc800 range.
 *    AC-3 audio track 1 is within 0xc800-0xe800 range (padding included).
 *    AC-3 audio track 2 is within 0xe800-0x10800 range (padding included).
 *
 *    AC-3 data portion is optional, but if present
 *    may require 0-byte padding, all aligned into 4-8 bytes.
 *    or last last AC-3 frame may be 0x3f8 bytes, which as a consequence
 *    is incomplete and will error out upon playback should it not be taken care of.
 *    both cases must be compensated for with an empty AC-3 frame block, one hardcoded to 0x700 bytes.
 *
 * 6. dat+sz file format can sometimes house compressed raw PCM16 data, and only that.
 *    said data may either be grouped with AC-3 data or stored stand-alone.
 *    (in which case the above two cases wrt. that AC-3 codec are expected to be handed out very similarly)
 *    this practice is only seen with FFX PS2 FMVs.
 *
 * 7. PS2 FMV format may come in dat+sz file pairs, or may be just standalone dat file.
 *    additionally, FFX PS2 ver, the game that uses the former pair format,
 *    do not call such files by name.
 *    said format also had latter standalone file format being revised over time.
 *    (v1->v2->v3)
 *
 * 8. check libavformat\demux.h and libavformat\avformat
 *    on how to fill this info over time.
 *
 * 9. proper seek and close functions.
 */

typedef struct SquarePS2FMVDATSZIPUContext
{
    unsigned long int frame_size;
    unsigned long int frame_blocks;
    //
    int prev_x;
    int prev_y;
    int x; // width
    int y; // height;
    int is_p_picture;
    int ipic_q_scale_code;
    int ppic_q_scale_code;
    int ivf_ascn_cfg;
    int mp1_qscst_cfg;
} SquarePS2FMVDATSZIPUContext;

typedef struct SquarePS2FMVDATSZDemuxContext {
    AVIOContext *dat;
    AVIOContext *sz;
    int intra_quant_twomatrix;
    size_t intra_quant_matrix_offset;
    size_t non_intra_quant_matrix_offset;
    size_t frame_size_pos;
    SquarePS2FMVDATSZIPUContext ipu;
    int frame_res_set;
    int frame_res_changed;
    int stop_reading_iqtable;
    int new_packet;
    int incoming_packet;
    size_t pkt_size;
    size_t pkt_pos;
    int pkt_i;
} SquarePS2FMVDATSZDemuxContext;

typedef struct SquarePS2FMVDemuxContext {
    int is_standalone_v1_dat;
    int dat_sz_is_dat;
    int dat_sz_is_sz;
    SquarePS2FMVDATSZDemuxContext dat_sz;
    int intra_quant_exists;
    int intra_quant_read;
    int video_stream_index;
    unsigned int block_frames;
    unsigned int block_size;
    unsigned int block_offset[2];
    int audio_streams;
    int *audio_stream_index;
} SquarePS2FMVDemuxContext;

static int grab_basepath_from_url(const char *base_url, char **target)
{
    if (!base_url)
        return AVERROR_INVALIDDATA;

    int url_size = 0;
    const char *moving_url = base_url;
    while (1)
    {
        if (!(*moving_url))
            break;
        moving_url++;
        url_size++;
    }

    if (!url_size)
        return AVERROR_INVALIDDATA;

    int sym_pos = url_size-1;
    moving_url = base_url;
    while (sym_pos > 0)
    {
        if ((moving_url[sym_pos] == '\\')
            ||
            (moving_url[sym_pos] == '/'))
            break;
        sym_pos--;
    }
    sym_pos++;

    if (!(*target))
    {
        *target = av_malloc(sym_pos);

        if (!(*target))
            return AVERROR_INVALIDDATA;

        int fill_pos = 0;
        int read_pos = 0;
        char *fill_target = *target;
        while (fill_pos < sym_pos)
        {
            fill_target[fill_pos] = moving_url[read_pos];
            fill_pos++;
            read_pos++;
        }

        fill_target[fill_pos] = 0;
    }

    return 0;
}

static int grab_basename_from_url(const char *base_url, char **target)
{
    if (!base_url)
        return AVERROR_INVALIDDATA;

    int url_size = 0;
    const char *moving_url = base_url;
    while (1)
    {
        if (!(*moving_url))
            break;
        moving_url++;
        url_size++;
    }

    if (!url_size)
        return AVERROR_INVALIDDATA;

    int sym_pos = url_size-1;
    moving_url = base_url;
    while (sym_pos > 0)
    {
        if ((moving_url[sym_pos] == '\\')
            ||
            (moving_url[sym_pos] == '/'))
            break;
        sym_pos--;
    }
    sym_pos++;

    int bext_pos = sym_pos;
    while (bext_pos < url_size)
    {
        if (moving_url[bext_pos] == '.')
            break;
        bext_pos++;
    }

    if (!(*target))
    {
        int basename_size = bext_pos - sym_pos;
        *target = av_malloc(basename_size+1);

        if (!(*target))
            return AVERROR_INVALIDDATA;

        int fill_pos = 0;
        int read_pos = sym_pos;
        char *fill_target = *target;
        while (fill_pos < basename_size)
        {
            fill_target[fill_pos] = moving_url[read_pos];
            fill_pos++;
            read_pos++;
        }

        fill_target[fill_pos] = 0;
    }

    return 0;
}

static int append_this_string(char **string, char **append_target)
{
    if (!(*append_target))
        return AVERROR_INVALIDDATA;

    int append_size = 0;
    const char *append_data = *append_target;
    while (1)
    {
        if (!(*append_data))
            break;
        append_data++;
        append_size++;
    }

    if (!append_size)
        return AVERROR_INVALIDDATA;

    int append_start_pos = 0;
    if (!(*string))
    {
        *string = av_malloc(append_size+1);

        if (!(*string))
            return AVERROR_INVALIDDATA;

        int append_data_pos = 0;
        char *string_write_data = *string;
        append_data = *append_target;
        while (append_start_pos < append_size)
        {
            string_write_data[append_start_pos] = append_data[append_data_pos];
            append_start_pos++;
            append_data_pos++;
        }

        string_write_data[append_start_pos] = 0;
    }
    else
    {
        int string_size = 0;
        const char *string_data = *string;
        while (1)
        {
            if (!(*string_data))
                break;
            string_data++;
            string_size++;
        }

        if (!string_size)
            return AVERROR_INVALIDDATA;

        append_start_pos = string_size;
        string_size += append_size;

        char *string_new = av_realloc(*string, string_size+1);
        if (!(string_new))
            return AVERROR_INVALIDDATA;

        *string = string_new;
        int append_data_pos = 0;
        char *string_write_data = *string;
        append_data = *append_target;
        while (append_start_pos < string_size)
        {
            string_write_data[append_start_pos] = append_data[append_data_pos];
            append_start_pos++;
            append_data_pos++;
        }

        string_write_data[append_start_pos] = 0;
    }

    return 0;
}

static void erase_this_string(char **target)
{
    if ((*target))
        av_freep(target);
}

static int erase_from_string(char **string, char *erase_target)
{
    return 0;
}

static int convert_string_to_lowercase(char **string)
{
    return 0;
}

static int probe_try_standalone_v1_dat(const AVProbeData *p)
{
    int ret = 0;

    if (AV_RB32(p->buf) == 0)
        ret++;

    if (p->buf_size < 36)
        return 0;

    if (AV_RL16(p->buf + 4) != 0)
        ret++;

    if (AV_RL16(p->buf + 8) != 0)
        ret++;

    if (AV_RL16(p->buf + 12) != 0)
        ret++;

    if (AV_RL16(p->buf + 14) != 0)
        ret++;

    if (AV_RL32(p->buf + 16) == 0xffffffff)
        ret++;

    if (AV_RL32(p->buf + 20) == 0xffffffff)
        ret++;

    if (AV_RL32(p->buf + 24) == 0xffffffff)
        ret++;

    if (AV_RL32(p->buf + 28) == 0xffffffff)
        ret++;

    if (AV_RL32(p->buf + 32) == 0xffffffff)
        ret++;

    if (ret < 10)
        return 0;

    return ret;
}

static int probe_try_dat_sz_sz(const AVProbeData *p)
{
    int ret = 0;
    unsigned char im_byte, nim_byte;
    int intra_matrix = 0, non_intra_matrix = 0;
    int toc_pos = 0;
    signed long toc_4byte;
    int toc_not_empty = 0;

    // intra matrix (optional feature of sz format)
    for (int i = 0; i < 64; i++)
    {
        im_byte = AV_RB8(p->buf + i);
        switch (im_byte)
        {
            case 0x08:
            case 0x10:
            case 0x13:
            case 0x16:
            case 0x18:
            case 0x1a:
            case 0x1b:
            case 0x1d:
            case 0x20:
            case 0x22:
            case 0x23:
            case 0x25:
            case 0x26:
            case 0x28:
            case 0x2e:
            case 0x30:
            case 0x38:
            case 0x3a:
            case 0x45:
            case 0x53:
                intra_matrix++;
                break;
            default:
                break;
        }
    }

    if ((intra_matrix == 64) | (intra_matrix < 64))
        ret++;

    // non-intra matrix (optional feature of sz format)
    for (int i = 0; i < 64; i++)
    {
        nim_byte = AV_RB8(p->buf + 64 + i);
        switch (nim_byte)
        {
            case 0x10:
            case 0x11:
            case 0x12:
            case 0x13:
            case 0x14:
            case 0x15:
            case 0x16:
            case 0x17:
            case 0x18:
            case 0x19:
            case 0x1a:
            case 0x1b:
            case 0x1c:
            case 0x1e:
            case 0x1f:
            case 0x21:
                non_intra_matrix++;
                break;
            default:
                break;
        }
    }

    if ((non_intra_matrix == 64) | (non_intra_matrix < 64))
        ret++;

    if ((intra_matrix == 64) && (non_intra_matrix == 64))
        toc_pos = 128;

    while (toc_pos < p->buf_size) {
        toc_4byte = AV_RL32(p->buf + toc_pos);
        if (toc_4byte && (toc_4byte < 0x1e600))
            toc_not_empty++;
        else
            break;
        toc_pos += 4;
    }

    if (toc_not_empty)
        ret++;

    if (ret < 3)
        return 0;

    return ret;
}

static int probe_try_dat_sz_dat(const AVProbeData *p)
{
    int ret = 0;
    unsigned long int config1, config2;
    char cfg_b;

    if (AV_RL32(p->buf) < 0xe300)
        ret++;

    if (AV_RL32(p->buf + 4) < 0x510)
        ret++;

    config1 = AV_RB32(p->buf + 8);
    config2 = AV_RB32(p->buf + 12);

    cfg_b = (config1 >> 24) & 0xff;
    if (!cfg_b)
        ret++;

    cfg_b = (config1 >> 16) & 0xff;
    if (cfg_b == 2)
        ret++;

    cfg_b = (config1 >> 8) & 0xff;
    if (cfg_b == 2)
        ret++;

    cfg_b = config1 & 0xff;
    if ((cfg_b == 0x14) | (cfg_b == 0x28))
        ret++;

    cfg_b = (config2 >> 24) & 0xff;
    if ((cfg_b == 0x1c) | (cfg_b == 0x20))
        ret++;

    cfg_b = (config2 >> 16) & 0xff;
    if (!(cfg_b) | (cfg_b == 2))
        ret++;

    cfg_b = (config2 >> 8) & 0xff;
    if (!(cfg_b) | (cfg_b == 1))
        ret++;

    cfg_b = config2 & 0xff;
    if (!cfg_b)
        ret++;

    if (AV_RB32(p->buf + 16) >= 0xf1000000UL)
        ret++;

    if (ret < 11)
        return 0;

    return ret;
}

static int read_probe(const AVProbeData *p)
{
    if (!(probe_try_standalone_v1_dat(p))) {
        if (!(probe_try_dat_sz_dat(p))) {
            if (!(probe_try_dat_sz_sz(p))) {
                return 0;
            } else
                goto goto_score;
        }
        else
            goto goto_score;
    } else
        goto goto_score;

goto_score:
    return AVPROBE_SCORE_MAX;
}

static int header_try_standalone_v1_dat(AVFormatContext *s)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;

    if (!avio_rl32(pb))
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl16(pb))
        ps2fmv->is_standalone_v1_dat++;
    avio_rl16(pb);
    if (avio_rl16(pb))
        ps2fmv->is_standalone_v1_dat++;
    avio_rl16(pb);
    if (avio_rl16(pb))
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl16(pb))
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl32(pb) == 0xffffffff)
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl32(pb) == 0xffffffff)
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl32(pb) == 0xffffffff)
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl32(pb) == 0xffffffff)
        ps2fmv->is_standalone_v1_dat++;
    if (avio_rl32(pb) == 0xffffffff)
        ps2fmv->is_standalone_v1_dat++;

    if (ps2fmv->is_standalone_v1_dat < 10)
        return 0;

    return 1;
}

static int header_try_dat_sz_sz(AVFormatContext *s)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;
    AVIOContext *pb = s->pb;
    unsigned char im_byte, nim_byte;
    int intra_matrix = 0, non_intra_matrix = 0;
    int toc_pos = 0;
    signed long toc_4byte;
    int toc_not_empty = 0;

    // intra matrix (optional feature of sz format)
    for (int i = 0; i < 64; i++)
    {
        im_byte = avio_r8(pb);
        switch (im_byte)
        {
            case 0x08:
            case 0x10:
            case 0x13:
            case 0x16:
            case 0x18:
            case 0x1a:
            case 0x1b:
            case 0x1d:
            case 0x20:
            case 0x22:
            case 0x23:
            case 0x25:
            case 0x26:
            case 0x28:
            case 0x2e:
            case 0x30:
            case 0x38:
            case 0x3a:
            case 0x45:
            case 0x53:
                intra_matrix++;
                break;
            default:
                break;
        }
    }

    if ((intra_matrix == 64) | (intra_matrix < 64))
        ps2fmv->dat_sz_is_sz++;

    // non-intra matrix (optional feature of sz format)
    for (int i = 0; i < 64; i++)
    {
        nim_byte = avio_r8(pb);
        switch (nim_byte)
        {
            case 0x10:
            case 0x11:
            case 0x12:
            case 0x13:
            case 0x14:
            case 0x15:
            case 0x16:
            case 0x17:
            case 0x18:
            case 0x19:
            case 0x1a:
            case 0x1b:
            case 0x1c:
            case 0x1e:
            case 0x1f:
            case 0x21:
                non_intra_matrix++;
                break;
            default:
                break;
        }
    }

    if ((non_intra_matrix == 64) | (non_intra_matrix < 64))
        ps2fmv->dat_sz_is_sz++;

    if ((intra_matrix == 64) && (non_intra_matrix == 64))
        toc_pos = 128;
    avio_seek(pb, toc_pos, SEEK_SET);

    while (toc_pos < pb->buffer_size) {
        toc_4byte = avio_rl32(pb);
        if (toc_4byte && (toc_4byte < 0x1e600))
            toc_not_empty++;
        else
            break;
        toc_pos += 4;
    }

    if (toc_not_empty)
        ps2fmv->dat_sz_is_sz++;

    if (ps2fmv->dat_sz_is_sz < 3)
        return 0;

    if ((intra_matrix == 64) && (non_intra_matrix == 64))
        dat_sz->intra_quant_twomatrix = 1;

    return 1;
}

static int header_try_dat_sz_dat(AVFormatContext *s)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;
    unsigned long int config1, config2;
    char cfg_b;

    if (avio_rl32(pb) < 0xe300)
        ps2fmv->dat_sz_is_dat++;

    if (avio_rl32(pb) < 0x510)
        ps2fmv->dat_sz_is_dat++;

    config1 = avio_rb32(pb);
    config2 = avio_rb32(pb);

    cfg_b = (config1 >> 24) & 0xff;
    if (!cfg_b)
        ps2fmv->dat_sz_is_dat++;

    cfg_b = (config1 >> 16) & 0xff;
    if (cfg_b == 2)
        ps2fmv->dat_sz_is_dat++;

    cfg_b = (config1 >> 8) & 0xff;
    if (cfg_b == 2)
        ps2fmv->dat_sz_is_dat++;

    cfg_b = config1 & 0xff;
    if ((cfg_b == 0x14) | (cfg_b == 0x28))
        ps2fmv->dat_sz_is_dat++;

    cfg_b = (config2 >> 24) & 0xff;
    if ((cfg_b == 0x1c) | (cfg_b == 0x20))
        ps2fmv->dat_sz_is_dat++;

    cfg_b = (config2 >> 16) & 0xff;
    if (!(cfg_b) | (cfg_b == 2))
        ps2fmv->dat_sz_is_dat++;

    cfg_b = (config2 >> 8) & 0xff;
    if (!(cfg_b) | (cfg_b == 1))
        ps2fmv->dat_sz_is_dat++;

    cfg_b = config2 & 0xff;
    if (!cfg_b)
        ps2fmv->dat_sz_is_dat++;

    if (avio_rb32(pb) >= 0xf1000000UL)
        ps2fmv->dat_sz_is_dat++;

    if (ps2fmv->dat_sz_is_dat < 11)
        return 0;

    return 1;
}

static int read_header(AVFormatContext *s)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st = NULL;
    int64_t start_offset = 0;
    uint16_t nb_frames, frame_h, frame_w;
    int ret;

    /* (todo) again, only the standalone v1 dat format is parsed here. */

    if (!(header_try_standalone_v1_dat(s))) {
        avio_seek(pb, 0, SEEK_SET);
        if (!(header_try_dat_sz_dat(s))) {
            avio_seek(pb, 0, SEEK_SET);
            if (!(header_try_dat_sz_sz(s))) {
                return AVERROR_INVALIDDATA;
            } else
                goto goto_parse;
        } else
            goto goto_parse;
    } else
        goto goto_parse;

goto_parse:
    avio_seek(pb, 0, SEEK_SET);

    if (ps2fmv->is_standalone_v1_dat == 10)
    {
        avio_skip(pb, 8);
        nb_frames = avio_rl16(pb);
        avio_rl16(pb);
        frame_w = avio_rl16(pb);
        frame_h = avio_rl16(pb);

        if (nb_frames <= 0 || frame_w <= 0 || frame_h <= 0)
            return AVERROR_INVALIDDATA;

        start_offset = 36;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->codecpar->codec_type   = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id     = AV_CODEC_ID_IPU;
        st->codecpar->width        = frame_w;
        st->codecpar->height       = frame_h;
        st->start_time             = 0;
        st->duration               =
        st->nb_frames              = nb_frames;
        avpriv_set_pts_info(st, 64, 1, 25);
        ps2fmv->video_stream_index = st->index;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        if ((ret = ff_alloc_extradata(st->codecpar, 3)) < 0)
            return ret;

        if (st->codecpar->extradata_size != 3)
            return AVERROR_INVALIDDATA;

        AV_WB8(st->codecpar->extradata, 1);
        AV_WB8(st->codecpar->extradata+1, 0);
        AV_WB8(st->codecpar->extradata+2, 1);

        avio_seek(pb, start_offset, SEEK_SET);
    } else if ((ps2fmv->dat_sz_is_sz == 3) | (ps2fmv->dat_sz_is_dat == 11)) {
        SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;

        char *basepath = NULL;
        char *basename = NULL;
        char *pair_url = NULL;

        ret = grab_basepath_from_url(s->url, &basepath);
        if (ret < 0)
            return ret;

        ret = grab_basename_from_url(s->url, &basename);
        if (ret < 0)
            return ret;

        if (ps2fmv->dat_sz_is_dat == 11)
            dat_sz->dat = pb;
        else if (ps2fmv->dat_sz_is_sz == 3)
            dat_sz->sz = pb;

        ret = append_this_string(&pair_url, &basepath);
        if (ret < 0)
            return ret;

        ret = append_this_string(&pair_url, &basename);
        if (ret < 0)
            return ret;

        if (ps2fmv->dat_sz_is_dat == 11) {
            char *dot_sz = (char*)".SZ";
            ret = append_this_string(&pair_url, &dot_sz);
            if (ret < 0)
                return ret;

            if ((avio_open(&dat_sz->sz, pair_url, AVIO_FLAG_READ)) < 0) {
                erase_from_string(&pair_url, dot_sz);
                convert_string_to_lowercase(&dot_sz);

                ret = append_this_string(&pair_url, &dot_sz);
                if (ret < 0)
                    return ret;

                ret = avio_open(&dat_sz->sz, pair_url, AVIO_FLAG_READ);
                if (ret < 0)
                    return ret;
            }
        } else if (ps2fmv->dat_sz_is_sz == 3) {
            char *dot_dat = (char*)".DAT";
            ret = append_this_string(&pair_url, &dot_dat);
            if (ret < 0)
                return ret;

            if ((avio_open(&dat_sz->dat, pair_url, AVIO_FLAG_READ)) < 0) {
                erase_from_string(&pair_url, dot_dat);
                convert_string_to_lowercase(&dot_dat);

                ret = append_this_string(&pair_url, &dot_dat);
                if (ret < 0)
                    return ret;

                ret = avio_open(&dat_sz->dat, pair_url, AVIO_FLAG_READ);
                if (ret < 0)
                    return ret;
            }
        }

        erase_this_string(&basepath);
        erase_this_string(&basename);
        erase_this_string(&pair_url);

        avio_seek(dat_sz->sz, 0, SEEK_SET);
        avio_seek(dat_sz->dat, 0, SEEK_SET);

        ps2fmv->video_stream_index = -1;

        s->ctx_flags |= AVFMTCTX_NOHEADER;
    }

    return 0;
}

static int square_ps2fmv_set_iq_read_from_sz(SquarePS2FMVDemuxContext *ps2fmv)
{
    if (!ps2fmv)
        return -1;

    SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;
    size_t sz_pos = avio_tell(dat_sz->sz);

    if (sz_pos > 0)
        return -2;

    if ((dat_sz->intra_quant_twomatrix) && !(ps2fmv->intra_quant_exists)) {
        dat_sz->intra_quant_matrix_offset = 0;
        dat_sz->non_intra_quant_matrix_offset = 64;
        dat_sz->frame_size_pos = 128;
        ps2fmv->intra_quant_exists = 1;
    }

    return 1;
}

static int square_ps2fmv_read_iq_from_sz(AVFormatContext *s)
{
    if (!s)
        return AVERROR_INVALIDDATA;

    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;
    AVStream *st = NULL;
    int ret;

    if (!dat_sz->sz)
        return AVERROR_INVALIDDATA;

    if (dat_sz->stop_reading_iqtable)
        return AVERROR_INVALIDDATA;

    size_t sz_pos = avio_tell(dat_sz->sz);
    if (sz_pos)
        return AVERROR_INVALIDDATA;

    if ((dat_sz->intra_quant_twomatrix) && (ps2fmv->intra_quant_exists) && (!ps2fmv->intra_quant_read)) {
        if (ps2fmv->video_stream_index == -1) {
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->codecpar->codec_type   = AVMEDIA_TYPE_VIDEO;
            st->codecpar->codec_id     = AV_CODEC_ID_IPU;
            st->start_time             = 0;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_TIMESTAMPS;
            avpriv_set_pts_info(st, 64, 1, 25);
            ps2fmv->video_stream_index = st->index;

            if ((ret = ff_alloc_extradata(st->codecpar, 3 + 64 + 64)) < 0)
                return ret;

            if (st->codecpar->extradata_size != (3 + 64 + 64))
                return AVERROR_INVALIDDATA;

            AV_WB8(st->codecpar->extradata, 1);
            AV_WB8(st->codecpar->extradata+1, 0);
            AV_WB8(st->codecpar->extradata+2, 1);
            avio_seek(dat_sz->sz, dat_sz->intra_quant_matrix_offset, SEEK_SET);
            avio_read(dat_sz->sz, st->codecpar->extradata+3, 64);
            avio_seek(dat_sz->sz, dat_sz->non_intra_quant_matrix_offset, SEEK_SET);
            avio_read(dat_sz->sz, st->codecpar->extradata+(3+64), 64);

            ps2fmv->intra_quant_read = 1;
        }
    }

    dat_sz->stop_reading_iqtable = 1;

    return 0;
}

static int square_ps2fmv_read_sz_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (!s)
        return AVERROR_INVALIDDATA;

    if (!pkt)
        return AVERROR_INVALIDDATA;

    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;
    SquarePS2FMVDATSZIPUContext *ipu = &dat_sz->ipu;

    if (!(dat_sz->dat))
        return AVERROR_INVALIDDATA;

    AVStream *st = NULL;
    int ret;

    if (dat_sz->new_packet == 1) {
        dat_sz->pkt_i = 0;
        dat_sz->pkt_pos = 0;
        dat_sz->new_packet = 0;
    }

    unsigned long int config1, config2, config3, config4;
    while (dat_sz->pkt_pos < dat_sz->pkt_size)
    {
        config1 = avio_rl32(dat_sz->dat);
        config2 = avio_rl32(dat_sz->dat);
        config3 = avio_rb32(dat_sz->dat);
        config4 = avio_rb32(dat_sz->dat);
        dat_sz->pkt_pos += 16;

        if (dat_sz->pkt_i == 0) {
            ipu->prev_x = ipu->x;
            ipu->prev_y = ipu->y;

            dat_sz->frame_res_changed = ((ipu->prev_x != ipu->x) && (ipu->prev_y != ipu->y)) ? 1 : 0;

            ipu->frame_size = config1;
            ipu->frame_blocks = config2;
            ipu->is_p_picture = (config3 >> 24) & 0xff;
            ipu->ipic_q_scale_code = (config3 >> 16) & 0xff;
            ipu->ppic_q_scale_code = (config3 >> 8) & 0xff;
            ipu->x = (config3 & 0xff) << 4;
            ipu->y = ((config4 >> 24) & 0xff) << 4;
            ipu->ivf_ascn_cfg = (config4 >> 8) & 0xff;
            ipu->mp1_qscst_cfg = config4 & 0xff;

            if (ps2fmv->video_stream_index == -1) {
                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);

                st->codecpar->codec_type   = AVMEDIA_TYPE_VIDEO;
                st->codecpar->codec_id     = AV_CODEC_ID_IPU;
                st->codecpar->width        = ipu->x;
                st->codecpar->height       = ipu->y;
                dat_sz->frame_res_set      = 1;
                st->start_time             = 0;
                ffstream(st)->need_parsing = AVSTREAM_PARSE_TIMESTAMPS;
                avpriv_set_pts_info(st, 64, 1, 25);
                ps2fmv->video_stream_index = st->index;  

                #if 0
                if ((ret = ff_alloc_extradata(st->codecpar, 2 + 64 + 64 + sizeof(int))) < 0)
                    return ret;

                if (st->codecpar->extradata_size != (2 + 64 + 64  + sizeof(int)))
                    return AVERROR_INVALIDDATA;
                #else
                if ((ret = ff_alloc_extradata(st->codecpar, 3)) < 0)
                    return ret;

                if (st->codecpar->extradata_size != 3)
                    return AVERROR_INVALIDDATA;
                #endif

                AV_WB8(st->codecpar->extradata, 1);
                AV_WB8(st->codecpar->extradata+1, 0);
                AV_WB8(st->codecpar->extradata+2, 1);
            } else {
                if (!s->streams)
                    return AVERROR_INVALIDDATA;
                if ((ps2fmv->video_stream_index < 0) | (ps2fmv->video_stream_index > s->nb_streams))
                    return AVERROR_INVALIDDATA;

                st = s->streams[ps2fmv->video_stream_index];
                if (st) {
                    st->codecpar->width        = ipu->x;
                    st->codecpar->height       = ipu->y;
                    dat_sz->frame_res_set      = 1;
                }
            }
            #if 0
            AV_WB32(st->codecpar->extradata+(2+64+64), ipu->frame_blocks);
            #endif

            ret = av_get_packet(dat_sz->dat, pkt, ipu->frame_size);
            if (ret < 0)
                return ret;
            if (ret != ps2fmv->block_size)
                return AVERROR_INVALIDDATA;

            pkt->pos = dat_sz->pkt_pos;
            pkt->duration = 1;
            pkt->stream_index = ps2fmv->video_stream_index;

            avio_skip(dat_sz->dat, ipu->frame_size);
            return ret;
        }
    }

    if (dat_sz->pkt_pos == dat_sz->pkt_size)
        dat_sz->incoming_packet = 2;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;
    int cannot_process_this_block;

    #if 0
    IPUInstructionQueue_QueuedInst = 0;
    #endif

    if (ps2fmv->is_standalone_v1_dat == 10)
    {
        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);

        unsigned int *current_block_offset = &ps2fmv->block_offset[0];
        unsigned int *next_block_offset = &ps2fmv->block_offset[1];

        ps2fmv->block_size = avio_rl32(pb);
        ps2fmv->block_frames = avio_rl32(pb);

        cannot_process_this_block =
            (!ps2fmv->block_size) || (ps2fmv->block_size > 0x40000);

        *(current_block_offset) = pos + 8;
        *(next_block_offset) = *(current_block_offset) + ps2fmv->block_size;

        if (!cannot_process_this_block) {
            ret = av_get_packet(pb, pkt, ps2fmv->block_size);
            if (ret < 0)
                return ret;
            if (ret != ps2fmv->block_size)
                return AVERROR_INVALIDDATA;

            pkt->pos = *(current_block_offset);
            pkt->duration = ps2fmv->block_frames;
            pkt->stream_index = ps2fmv->video_stream_index;

            if (!s->streams)
                return AVERROR_INVALIDDATA;
            if ((ps2fmv->video_stream_index < 0) | (ps2fmv->video_stream_index > s->nb_streams))
                return AVERROR_INVALIDDATA;

            #if 0
            AVStream *st = s->streams[ps2fmv->video_stream_index];
            #endif
            uint8_t *ipu_queue_inst_args = av_packet_new_side_data(pkt, AV_PKT_DATA_NEW_EXTRADATA, sizeof(IPUInstructionQueueArgumentContext));
            if (ipu_queue_inst_args) {
                #if 0
                ff_ipu_queue_ctrl_exec(pkt, (IPUControlRegister){ 0, 0, 0, 1, 0, 0, 1, 1 }, 1);
                ff_ipu_queue_inst_exec_from_start(pkt, (IPUBlockCommand){ IPU_CMD_IDEC }, (IPUIBDECArguments){ 1, 0, 0, 0, 0 }, 0, 0, st->codecpar->width, st->codecpar->height, 1);
                #endif
            }

            avio_seek(pb, *(next_block_offset), SEEK_SET);

            return ret;
        } else {
            // (TODO) last block is reported as having 0 bytes and no actual frames in it
            // (denoted by block_frames being 0xffffffff instead).
            // despite this, there is a possibility that a block
            // that *isn't* stored at the tail end of the file may have similar block info data,
            // which consists of "block size" and "block frames", respectively.
            return AVERROR_EOF;
        }
    } else if ((ps2fmv->dat_sz_is_sz == 3) | (ps2fmv->dat_sz_is_dat == 11)) {
        SquarePS2FMVDATSZDemuxContext *dat_sz = &ps2fmv->dat_sz;

        if ((avio_feof(dat_sz->sz)) && (avio_feof(dat_sz->dat)))
            return AVERROR_EOF;

        size_t sz_pos = avio_tell(dat_sz->sz);
        size_t dat_pos = avio_tell(dat_sz->dat);

        ret = square_ps2fmv_set_iq_read_from_sz(ps2fmv);
        if (ret < 0)
            return ret;

        while ((!(avio_feof(dat_sz->sz))) && (!(avio_feof(dat_sz->dat))))
        {
            sz_pos = avio_tell(dat_sz->sz);
            dat_pos = avio_tell(dat_sz->dat);
            if ((sz_pos == 0) && (!(dat_sz->stop_reading_iqtable))) {
                ret = square_ps2fmv_read_iq_from_sz(s);
                if (ret < 0)
                    return ret;

                avio_seek(dat_sz->sz, dat_sz->frame_size_pos, SEEK_SET);
            }

            if (!(dat_sz->incoming_packet)) {
                dat_sz->pkt_size = avio_rl32(dat_sz->sz);
                if (!dat_sz->pkt_size)
                    return AVERROR_EOF;

                dat_sz->incoming_packet = 1;
                dat_sz->new_packet = 1;
            } else {
                ret = square_ps2fmv_read_sz_packet(s, pkt);
                if (ret < 0)
                    return ret;

                return ret;
            }

            if (dat_sz->incoming_packet == 2) {
                avio_seek(dat_sz->dat, dat_pos + dat_sz->pkt_size, SEEK_SET);
                dat_sz->incoming_packet = 0;
            }
        }
    }

    return AVERROR_EOF;
}

const FFInputFormat ff_square_ps2fmv_demuxer = {
    .p.name = "square_ps2fmv",
    .p.long_name = NULL_IF_CONFIG_SMALL("SquareSoft->SquareEnix PS2 FMV"),
    .p.flags = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SquarePS2FMVDemuxContext),
    .read_probe = read_probe,
    .read_header = read_header,
    .read_packet = read_packet,
};
