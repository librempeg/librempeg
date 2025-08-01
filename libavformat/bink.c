/*
 * Bink demuxer
 * Copyright (c) 2008-2010 Peter Ross (pross@xvid.org)
 * Copyright (c) 2009 Daniel Verkamp (daniel@drv.nu)
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

/**
 * @file
 * Bink demuxer
 *
 * Technical details here:
 *  http://wiki.multimedia.cx/index.php?title=Bink_Container
 */

#include <inttypes.h>

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

enum BinkAudFlags {
    BINK_AUD_16BITS = 0x4000, ///< prefer 16-bit output
    BINK_AUD_STEREO = 0x2000,
    BINK_AUD_USEDCT = 0x1000,
};

#define BINK_EXTRADATA_SIZE     1
#define BINK_MAX_AUDIO_TRACKS   256
#define BINK_MAX_WIDTH          7680
#define BINK_MAX_HEIGHT         4800
#define SMUSH_BLOCK_SIZE        512

typedef struct BinkDemuxContext {
    uint32_t file_size;

    uint32_t num_audio_tracks;
    int current_track;      ///< audio track to return in next packet
    int64_t video_pts;
    int64_t audio_pts[BINK_MAX_AUDIO_TRACKS];

    uint32_t remain_packet_size;
    int flags;
    int smush_size;
} BinkDemuxContext;

static int probe(const AVProbeData *p)
{
    const uint8_t *b = p->buf;
    int smush = AV_RN32(p->buf) == AV_RN32("SMUS");

    do {
        if (((b[0] == 'B' && b[1] == 'I' && b[2] == 'K' && /* Bink 1 */
             (b[3] == 'b' || b[3] == 'f' || b[3] == 'g' || b[3] == 'h' || b[3] == 'i' ||
              b[3] == 'k')) ||
             (b[0] == 'K' && b[1] == 'B' && b[2] == '2' && /* Bink 2 */
             (b[3] == 'a' || b[3] == 'd' || b[3] == 'f' || b[3] == 'g' || b[3] == 'h' ||
              b[3] == 'i' || b[3] == 'j' || b[3] == 'k'))) &&
            AV_RL32(b+8) > 0 &&  // num_frames
            AV_RL32(b+20) > 0 && AV_RL32(b+20) <= BINK_MAX_WIDTH &&
            AV_RL32(b+24) > 0 && AV_RL32(b+24) <= BINK_MAX_HEIGHT &&
            AV_RL32(b+28) > 0 && AV_RL32(b+32) > 0)  // fps num,den
            return AVPROBE_SCORE_MAX;
        b += SMUSH_BLOCK_SIZE;
    } while (smush && b < p->buf + p->buf_size - 32);
    return 0;
}

static int read_header(AVFormatContext *s)
{
    BinkDemuxContext *bink = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t fps_num, fps_den;
    AVStream *const vst  = avformat_new_stream(s, NULL);
    FFStream *const vsti = ffstream(vst);
    unsigned int i;
    uint32_t pos, next_pos;
    uint16_t flags;
    int next_keyframe = 1;
    int keyframe;
    int ret;
    uint32_t signature;
    uint8_t revision;

    if (!vst)
        return AVERROR(ENOMEM);

    vst->codecpar->codec_tag = avio_rl32(pb);
    if (vst->codecpar->codec_tag == AV_RL32("SMUS")) {
        do {
            bink->smush_size += SMUSH_BLOCK_SIZE;
            avio_skip(pb, SMUSH_BLOCK_SIZE - 4);
            vst->codecpar->codec_tag = avio_rl32(pb);
        } while (!avio_feof(pb) && (vst->codecpar->codec_tag & 0xFFFFFF) != AV_RL32("BIK"));
        if (avio_feof(pb)) {
            av_log(s, AV_LOG_ERROR, "invalid SMUSH header: BIK not found\n");
            return AVERROR_INVALIDDATA;
        }
    }

    bink->file_size = avio_rl32(pb) + 8;
    vst->duration   = avio_rl32(pb);

    if (vst->duration > 1000000) {
        av_log(s, AV_LOG_ERROR, "invalid header: more than 1000000 frames\n");
        return AVERROR(EIO);
    }

    if (avio_rl32(pb) > bink->file_size) {
        av_log(s, AV_LOG_ERROR,
               "invalid header: largest frame size greater than file size\n");
        return AVERROR(EIO);
    }

    avio_skip(pb, 4);

    vst->codecpar->width  = avio_rl32(pb);
    vst->codecpar->height = avio_rl32(pb);

    fps_num = avio_rl32(pb);
    fps_den = avio_rl32(pb);
    if (fps_num == 0 || fps_den == 0) {
        av_log(s, AV_LOG_ERROR,
               "invalid header: invalid fps (%"PRIu32"/%"PRIu32")\n",
               fps_num, fps_den);
        return AVERROR(EIO);
    }
    avpriv_set_pts_info(vst, 64, fps_den, fps_num);
    vst->avg_frame_rate = av_inv_q(vst->time_base);

    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id   = AV_CODEC_ID_BINKVIDEO;

    if ((vst->codecpar->codec_tag & 0xFFFFFF) == MKTAG('K', 'B', '2', 0)) {
        av_log(s, AV_LOG_WARNING, "Bink 2 video is not implemented\n");
        vst->codecpar->codec_id = AV_CODEC_ID_NONE;
    }

    if ((ret = ff_get_extradata(s, vst->codecpar, pb, 4)) < 0)
        return ret;

    bink->num_audio_tracks = avio_rl32(pb);

    if (bink->num_audio_tracks > BINK_MAX_AUDIO_TRACKS) {
        av_log(s, AV_LOG_ERROR,
               "invalid header: more than "AV_STRINGIFY(BINK_MAX_AUDIO_TRACKS)" audio tracks (%"PRIu32")\n",
               bink->num_audio_tracks);
        return AVERROR(EIO);
    }

    signature = (vst->codecpar->codec_tag & 0xFFFFFF);
    revision = ((vst->codecpar->codec_tag >> 24) % 0xFF);

    if ((signature == AV_RL32("BIK") && (revision == 'k')) ||
        (signature == AV_RL32("KB2") && (revision == 'i' || revision == 'j' || revision == 'k')))
        avio_skip(pb, 4); /* unknown new field */

    if (bink->num_audio_tracks) {
        avio_skip(pb, 4 * bink->num_audio_tracks); /* max decoded size */

        for (i = 0; i < bink->num_audio_tracks; i++) {
            AVStream *const ast = avformat_new_stream(s, NULL);
            if (!ast)
                return AVERROR(ENOMEM);
            ast->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
            ast->codecpar->codec_tag   = 0;
            ast->codecpar->sample_rate = avio_rl16(pb);
            avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
            flags = avio_rl16(pb);
            ast->codecpar->codec_id = flags & BINK_AUD_USEDCT ?
                                   AV_CODEC_ID_BINKAUDIO_DCT : AV_CODEC_ID_BINKAUDIO_RDFT;
            if (flags & BINK_AUD_STEREO) {
                ast->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
            } else {
                ast->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
            }
            if ((ret = ff_alloc_extradata(ast->codecpar, 4)) < 0)
                return ret;
            AV_WL32(ast->codecpar->extradata, vst->codecpar->codec_tag);
        }

        for (i = 0; i < bink->num_audio_tracks; i++)
            s->streams[i + 1]->id = avio_rl32(pb);
    }

    /* frame index table */
    next_pos = avio_rl32(pb);
    for (i = 0; i < vst->duration; i++) {
        pos = next_pos;
        keyframe = next_keyframe;
        if (i == vst->duration - 1) {
            next_pos = bink->file_size;
            next_keyframe = 0;
        } else {
            next_pos = avio_rl32(pb);
            next_keyframe = next_pos & 1;
        }
        pos &= ~1;
        next_pos &= ~1;

        if (next_pos <= pos) {
            av_log(s, AV_LOG_ERROR, "invalid frame index table\n");
            return AVERROR(EIO);
        }
        if ((ret = av_add_index_entry(vst, pos, i, next_pos - pos, 0,
                                      keyframe ? AVINDEX_KEYFRAME : 0)) < 0)
            return ret;
    }

    if (vsti->index_entries)
        avio_seek(pb, vsti->index_entries[0].pos + bink->smush_size, SEEK_SET);
    else
        avio_skip(pb, 4);

    bink->current_track = -1;
    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    BinkDemuxContext *bink = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret;

    if (bink->current_track < 0) {
        int index_entry;
        AVStream *st = s->streams[0]; // stream 0 is video stream with index
        FFStream *const sti = ffstream(st);

        if (bink->video_pts >= st->duration)
            return AVERROR_EOF;

        index_entry = av_index_search_timestamp(st, bink->video_pts,
                                                AVSEEK_FLAG_ANY);
        if (index_entry < 0) {
            av_log(s, AV_LOG_ERROR,
                   "could not find index entry for frame %"PRId64"\n",
                   bink->video_pts);
            return AVERROR(EIO);
        }

        bink->remain_packet_size = sti->index_entries[index_entry].size;
        bink->flags              = sti->index_entries[index_entry].flags;
        bink->current_track = 0;
    }

    while (bink->current_track < bink->num_audio_tracks) {
        uint32_t audio_size = avio_rl32(pb);
        if (audio_size > bink->remain_packet_size - 4) {
            av_log(s, AV_LOG_ERROR,
                   "frame %"PRId64": audio size in header (%"PRIu32") > size of packet left (%"PRIu32")\n",
                   bink->video_pts, audio_size, bink->remain_packet_size);
            return AVERROR(EIO);
        }
        bink->remain_packet_size -= 4 + audio_size;
        bink->current_track++;
        if (audio_size >= 4) {
            /* get one audio packet per track */
            if ((ret = av_get_packet(pb, pkt, audio_size)) < 0)
                return ret;
            pkt->stream_index = bink->current_track;
            pkt->pts = bink->audio_pts[bink->current_track - 1];

            /* Each audio packet reports the number of decompressed samples
               (in bytes). We use this value to calculate the audio PTS */
            if (pkt->size >= 4)
                bink->audio_pts[bink->current_track -1] +=
                    AV_RL32(pkt->data) / (2 * s->streams[bink->current_track]->codecpar->ch_layout.nb_channels);
            return 0;
        } else {
            avio_skip(pb, audio_size);
        }
    }

    /* get video packet */
    if ((ret = av_get_packet(pb, pkt, bink->remain_packet_size)) < 0)
        return ret;
    pkt->stream_index = 0;
    pkt->pts = bink->video_pts++;
    if (bink->flags & AVINDEX_KEYFRAME)
        pkt->flags |= AV_PKT_FLAG_KEY;

    /* -1 instructs the next call to read_packet() to read the next frame */
    bink->current_track = -1;

    return 0;
}

static int read_seek(AVFormatContext *s, int stream_index, int64_t timestamp, int flags)
{
    BinkDemuxContext *bink = s->priv_data;
    AVStream *vst = s->streams[0];
    FFStream *const vsti = ffstream(vst);
    int64_t ret;

    if (!(s->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return -1;

    /* seek to the first frame */
    ret = avio_seek(s->pb, vsti->index_entries[0].pos + bink->smush_size, SEEK_SET);
    if (ret < 0)
        return ret;

    bink->video_pts = 0;
    memset(bink->audio_pts, 0, sizeof(bink->audio_pts));
    bink->current_track = -1;
    return 0;
}

const FFInputFormat ff_bink_demuxer = {
    .p.name         = "bink",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Bink"),
    .p.flags        = AVFMT_SHOW_IDS,
    .priv_data_size = sizeof(BinkDemuxContext),
    .read_probe     = probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
