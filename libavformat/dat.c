/*
 * DAT (Digital Audio Tape) demuxer
 * Copyright (c) 2024 Paul B Mahol
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

#define DAT_PACKET_SIZE 5822
#define DAT_OFFSET 5760

static const uint32_t encoded_rate[] = { 48000, 44100, 32000, 0 };
static const uint16_t encoded_size[] = { 5760, 5292, 5760, 0 };
static const uint8_t encoded_chans[] = { 2, 4, 0, 0 };
static const enum AVCodecID encoded_codec[] = {
    AV_CODEC_ID_PCM_S16LE, AV_CODEC_ID_PCM_DAT,
    AV_CODEC_ID_NONE, AV_CODEC_ID_NONE,
};

static int valid_frame(const uint8_t *frame)
{
    const uint8_t *scode = frame+DAT_OFFSET;
    const uint8_t *subid = scode+7*8;
    const uint8_t *mainid = subid+4;
    int chan_index = (mainid[0] >> 0) & 0x3;
    int rate_index = (mainid[0] >> 2) & 0x3;
    int enc_index  = (mainid[1] >> 6) & 0x3;
    int dataid     = (subid[0] >> 0) & 0xf;

    if (dataid != 0 || encoded_codec[enc_index] == AV_CODEC_ID_NONE ||
        encoded_chans[chan_index] == 0 ||
        encoded_rate[rate_index] == 0)
        return 0;

    return 1;
}

static int read_probe(const AVProbeData *p)
{
    const int cnt = p->buf_size / DAT_PACKET_SIZE;
    int score = 0;

    for (int i = 0; i < cnt; i++) {
        const int ret = valid_frame(&p->buf[i * DAT_PACKET_SIZE]);

        score += ret;
        if (ret == 0) {
            score = 0;
            break;
        }
    }

    score /= 4;

    return FFMIN(score, AVPROBE_SCORE_MAX);
}

static int read_header(AVFormatContext *s)
{
    s->ctx_flags |= AVFMTCTX_NOHEADER;

    return 0;
}

static int parse_frame(uint8_t *frame, AVCodecParameters *par)
{
    uint8_t *scode = frame+DAT_OFFSET;
    uint8_t *subid = scode+7*8;
    uint8_t *mainid = subid+4;
    int chan_index = (mainid[0] >> 0) & 0x3;
    int rate_index = (mainid[0] >> 2) & 0x3;
    int enc_index  = (mainid[1] >> 6) & 0x3;
    int dataid     = (subid[0] >> 0) & 0xf;

    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->block_align = DAT_PACKET_SIZE;
    par->codec_id = encoded_codec[enc_index];
    av_channel_layout_default(&par->ch_layout, encoded_chans[chan_index]);
    par->sample_rate = encoded_rate[rate_index];
    par->bit_rate = (8LL * par->block_align * par->sample_rate) / FFMAX(1, av_get_audio_frame_duration2(par, encoded_size[rate_index]));

    if (dataid != 0 || par->codec_id == AV_CODEC_ID_NONE ||
        par->ch_layout.nb_channels <= 0 ||
        par->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    return encoded_size[rate_index];
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    uint8_t data[DAT_PACKET_SIZE];
    AVIOContext *pb = s->pb;
    int ret, pcm_size;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (avio_read(pb, data, sizeof(data)) != sizeof(data))
        return AVERROR_EOF;

    if (s->nb_streams == 0) {
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        pcm_size = parse_frame(data, st->codecpar);

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    } else {
        int old_sample_rate = s->streams[0]->codecpar->sample_rate;

        pcm_size = parse_frame(data, s->streams[0]->codecpar);

        if (old_sample_rate != s->streams[0]->codecpar->sample_rate) {
            ret = ff_add_param_change(pkt, 0, 0, s->streams[0]->codecpar->sample_rate, 0, 0);
            if (ret < 0)
                return ret;
        }
    }

    if (pcm_size < 0)
        return FFERROR_REDO;

    if ((ret = av_new_packet(pkt, pcm_size)) < 0)
        return ret;
    memcpy(pkt->data, data, pcm_size);

    pkt->stream_index = 0;
    pkt->pos = pos;

    return 0;
}

const FFInputFormat ff_dat_demuxer = {
    .p.name         = "dat",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DAT (Digital Audio Tape)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dat",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = ff_pcm_read_seek,
};
