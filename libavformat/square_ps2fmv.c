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

#include "libavutil/intreadwrite.h"
#include "libavutil/intfloat.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

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
 * 7. PS2 FMV format may come in dat+sz file pairs,
 * or may be just standalone dat file.
 * additionally, certain PS2 games that use the former pair format,
 * do not call such files by name.
 * said format also had latter standalone file format being revised over time.
 * (v1->v2->v3)
 *
 * 8. check libavformat\demux.h and libavformat\avformat
 * on how to fill this info over time.
 *
 * 9. proper seek and close functions.
 */

typedef struct SquarePS2FMVDemuxContext {
    int is_standalone_v1_dat;
    int video_stream_index;
    unsigned int block_frames;
    unsigned int block_size;
    unsigned int block_offset[2];
} SquarePS2FMVDemuxContext;

static int square_ps2fmv_read_probe(const AVProbeData *p)
{
    int is_standalone_v1_dat = 0;

    /* (todo) only standalone v1 dat format is parsed for now. */

    if (AV_RB32(p->buf) == 0)
        is_standalone_v1_dat++;

    if (p->buf_size < 36)
        return 0;

    if (AV_RL16(p->buf + 4) != 0)
        is_standalone_v1_dat++;

    if (AV_RL16(p->buf + 8) != 0)
        is_standalone_v1_dat++;

    if (AV_RL16(p->buf + 12) != 0)
        is_standalone_v1_dat++;

    if (AV_RL16(p->buf + 14) != 0)
        is_standalone_v1_dat++;

    if (AV_RL32(p->buf + 16) == 0xffffffff)
        is_standalone_v1_dat++;

    if (AV_RL32(p->buf + 20) == 0xffffffff)
        is_standalone_v1_dat++;

    if (AV_RL32(p->buf + 24) == 0xffffffff)
        is_standalone_v1_dat++;

    if (AV_RL32(p->buf + 28) == 0xffffffff)
        is_standalone_v1_dat++;

    if (AV_RL32(p->buf + 32) == 0xffffffff)
        is_standalone_v1_dat++;

    if (is_standalone_v1_dat < 10)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int square_ps2fmv_read_header(AVFormatContext *s)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st = NULL;
    int64_t start_offset = 0;
    uint16_t nb_frames, frame_h, frame_w;

    /* (todo) again, only the standalone v1 dat format is parsed here. */

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
        return AVERROR_INVALIDDATA;

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

        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_IPU;
        st->codecpar->width    = frame_w;
        st->codecpar->height   = frame_h;
        st->start_time         = 0;
        st->duration           =
        st->nb_frames          = nb_frames;
        avpriv_set_pts_info(st, 64, 1, 25);
        ps2fmv->video_stream_index = st->index;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int square_ps2fmv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SquarePS2FMVDemuxContext *ps2fmv = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (ps2fmv->is_standalone_v1_dat == 10)
    {
        unsigned int *current_block_offset = &ps2fmv->block_offset[0];
        unsigned int *next_block_offset = &ps2fmv->block_offset[1];

        ps2fmv->block_size = avio_rl32(pb);
        ps2fmv->block_frames = avio_rl32(pb);

        *(current_block_offset) = pos + 8;
        *(next_block_offset) = *(current_block_offset) + ps2fmv->block_size;

        if ((!ps2fmv->block_size)
            &&
            (ps2fmv->block_frames != 0xffffffff)) {
            ret = av_get_packet(pb, pkt, ps2fmv->block_size);
            if (ret < 0)
                return ret;
            if (ret != ps2fmv->block_size)
                return AVERROR_INVALIDDATA;

            pkt->pos = *(current_block_offset);
            pkt->duration = ps2fmv->block_frames;
            pkt->stream_index = ps2fmv->video_stream_index;

            avio_seek(pb, *(next_block_offset), SEEK_SET);
        }
    }

    return 0;
}

const FFInputFormat ff_square_ps2fmv_demuxer = {
    .p.name = "square_ps2fmv",
    .p.long_name = NULL_IF_CONFIG_SMALL("SquareSoft->SquareEnix PS2 FMV"),
    .p.flags = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SquarePS2FMVDemuxContext),
    .read_probe = square_ps2fmv_read_probe,
    .read_header = square_ps2fmv_read_header,
    .read_packet = square_ps2fmv_read_packet,
};
