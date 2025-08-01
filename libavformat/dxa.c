/*
 * DXA demuxer
 * Copyright (c) 2007 Konstantin Shishkov
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

#include <inttypes.h>

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "riff.h"

#define DXA_EXTRA_SIZE  9

typedef struct DXAContext {
    int frames;
    int has_sound;
    int bpc;
    uint32_t bytes_left;
    int64_t wavpos, vidpos;
    int readvid;
}DXAContext;

static int dxa_probe(const AVProbeData *p)
{
    int w, h;
    if (p->buf_size < 15)
        return 0;
    w = AV_RB16(p->buf + 11);
    h = AV_RB16(p->buf + 13);
    /* check file header */
    if (p->buf[0] == 'D' && p->buf[1] == 'E' &&
        p->buf[2] == 'X' && p->buf[3] == 'A' &&
        w && w <= 2048 && h && h <= 2048)
        return AVPROBE_SCORE_MAX;
    else
        return 0;
}

static int dxa_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    DXAContext *c = s->priv_data;
    AVStream *st, *ast;
    uint32_t tag;
    int32_t fps;
    int w, h;
    int num, den;
    int flags;
    int ret;

    tag = avio_rl32(pb);
    if (tag != MKTAG('D', 'E', 'X', 'A'))
        return AVERROR_INVALIDDATA;
    flags = avio_r8(pb);
    c->frames = avio_rb16(pb);
    if(!c->frames){
        av_log(s, AV_LOG_ERROR, "File contains no frames ???\n");
        return AVERROR_INVALIDDATA;
    }

    fps = avio_rb32(pb);
    if(fps > 0){
        den = 1000;
        num = fps;
    }else if (fps < 0 && fps > INT_MIN){
        den = 100000;
        num = -fps;
    }else{
        den = 10;
        num = 1;
    }
    w = avio_rb16(pb);
    h = avio_rb16(pb);
    c->has_sound = 0;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    // Parse WAV data header
    if(avio_rl32(pb) == MKTAG('W', 'A', 'V', 'E')){
        uint32_t size, fsize;
        c->has_sound = 1;
        size = avio_rb32(pb);
        c->vidpos = avio_tell(pb) + size;
        avio_skip(pb, 16);
        fsize = avio_rl32(pb);

        ast = avformat_new_stream(s, NULL);
        if (!ast)
            return AVERROR(ENOMEM);
        ret = ff_get_wav_header(s, pb, ast->codecpar, fsize, 0);
        if (ret < 0)
            return ret;
        if (ast->codecpar->sample_rate > 0)
            avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);
        // find 'data' chunk
        while(avio_tell(pb) < c->vidpos && !avio_feof(pb)){
            tag = avio_rl32(pb);
            fsize = avio_rl32(pb);
            if(tag == MKTAG('d', 'a', 't', 'a')) break;
            avio_skip(pb, fsize);
        }
        c->bpc = (fsize + (int64_t)c->frames - 1) / c->frames;
        if (c->bpc < 0)
            return AVERROR_INVALIDDATA;
        if(ast->codecpar->block_align) {
            if (c->bpc > INT_MAX - ast->codecpar->block_align + 1)
                return AVERROR_INVALIDDATA;
            c->bpc = ((c->bpc - 1 + ast->codecpar->block_align) / ast->codecpar->block_align) * ast->codecpar->block_align;
        }
        c->bytes_left = fsize;
        c->wavpos = avio_tell(pb);
        avio_seek(pb, c->vidpos, SEEK_SET);
    }

    /* now we are ready: build format streams */
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_DXA;
    st->codecpar->width      = w;
    st->codecpar->height     = h;
    av_reduce(&den, &num, den, num, (1UL<<31)-1);
    avpriv_set_pts_info(st, 33, num, den);
    /* flags & 0x80 means that image is interlaced,
     * flags & 0x40 means that image has double height
     * either way set true height
     */
    if(flags & 0xC0){
        st->codecpar->height >>= 1;
    }
    c->readvid = !c->has_sound;
    c->vidpos  = avio_tell(pb);
    s->start_time = 0;
    s->duration = av_rescale(c->frames, AV_TIME_BASE * (int64_t)num, den);
    av_log(s, AV_LOG_DEBUG, "%d frame(s)\n",c->frames);

    return 0;
}

static int dxa_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DXAContext *c = s->priv_data;
    int ret;
    uint32_t size;
    uint8_t buf[DXA_EXTRA_SIZE], pal[768+4];
    int pal_size = 0;

    if(!c->readvid && c->has_sound && c->bytes_left){
        c->readvid = 1;
        avio_seek(s->pb, c->wavpos, SEEK_SET);
        size = FFMIN(c->bytes_left, c->bpc);
        ret = av_get_packet(s->pb, pkt, size);
        pkt->stream_index = 1;
        if(ret != size)
            return AVERROR(EIO);
        c->bytes_left -= size;
        c->wavpos = avio_tell(s->pb);
        return 0;
    }
    avio_seek(s->pb, c->vidpos, SEEK_SET);
    while(!avio_feof(s->pb) && c->frames){
        uint32_t tag;
        if ((ret = avio_read(s->pb, buf, 4)) != 4) {
            av_log(s, AV_LOG_ERROR, "failed reading chunk type\n");
            return ret < 0 ? ret : AVERROR_INVALIDDATA;
        }
        tag = AV_RL32(buf);
        switch (tag) {
        case MKTAG('N', 'U', 'L', 'L'):
            if ((ret = av_new_packet(pkt, 4 + pal_size)) < 0)
                return ret;
            pkt->stream_index = 0;
            if(pal_size) memcpy(pkt->data, pal, pal_size);
            memcpy(pkt->data + pal_size, buf, 4);
            c->frames--;
            c->vidpos = avio_tell(s->pb);
            c->readvid = 0;
            return 0;
        case MKTAG('C', 'M', 'A', 'P'):
            pal_size = 768+4;
            memcpy(pal, buf, 4);
            avio_read(s->pb, pal + 4, 768);
            break;
        case MKTAG('F', 'R', 'A', 'M'):
            if ((ret = avio_read(s->pb, buf + 4, DXA_EXTRA_SIZE - 4)) != DXA_EXTRA_SIZE - 4) {
                av_log(s, AV_LOG_ERROR, "failed reading dxa_extra\n");
                return ret < 0 ? ret : AVERROR_INVALIDDATA;
            }
            size = AV_RB32(buf + 5);
            if(size > 0xFFFFFF){
                av_log(s, AV_LOG_ERROR, "Frame size is too big: %"PRIu32"\n",
                       size);
                return AVERROR_INVALIDDATA;
            }
            ret = av_new_packet(pkt, size + DXA_EXTRA_SIZE + pal_size);
            if (ret < 0)
                return ret;
            memcpy(pkt->data + pal_size, buf, DXA_EXTRA_SIZE);
            ret = avio_read(s->pb, pkt->data + DXA_EXTRA_SIZE + pal_size, size);
            if(ret != size){
                return AVERROR(EIO);
            }
            if(pal_size) memcpy(pkt->data, pal, pal_size);
            pkt->stream_index = 0;
            c->frames--;
            c->vidpos = avio_tell(s->pb);
            c->readvid = 0;
            return 0;
        default:
            av_log(s, AV_LOG_ERROR, "Unknown tag %s\n", av_fourcc2str(tag));
            return AVERROR_INVALIDDATA;
        }
    }
    return AVERROR_EOF;
}

const FFInputFormat ff_dxa_demuxer = {
    .p.name         = "dxa",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DXA"),
    .priv_data_size = sizeof(DXAContext),
    .read_probe     = dxa_probe,
    .read_header    = dxa_read_header,
    .read_packet    = dxa_read_packet,
};
