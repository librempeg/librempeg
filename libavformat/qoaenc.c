/*
 * QOA muxer
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
#include "libavutil/dict.h"
#include "avformat.h"
#include "avio_internal.h"
#include "mux.h"
#include "rawenc.h"

typedef struct QOAContext {
    int64_t samples;
} QOAContext;

static int qoa_write_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;

    ffio_wfourcc(pb, "qoaf");
    avio_wb32(pb, -1);

    return 0;
}

static int qoa_write_packet(AVFormatContext *ctx, AVPacket *pkt)
{
    QOAContext *s = ctx->priv_data;

    if (pkt->size < 8)
        return AVERROR(EINVAL);
    s->samples += AV_RB16(pkt->data + 4);

    avio_write(ctx->pb, pkt->data, pkt->size);

    return 0;
}

static int qoa_write_trailer(AVFormatContext *ctx)
{
    QOAContext *s = ctx->priv_data;
    AVIOContext *pb = ctx->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        int64_t file_size = avio_tell(pb);

        avio_seek(pb, 4, SEEK_SET);
        avio_wb32(pb, s->samples);
        avio_seek(pb, file_size, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_qoa_muxer = {
    .p.name            = "qoa",
    .p.long_name       = NULL_IF_CONFIG_SMALL("QOA"),
    .p.extensions      = "qoa",
    .priv_data_size    = sizeof(QOAContext),
    .p.audio_codec     = AV_CODEC_ID_QOA,
    .p.video_codec     = AV_CODEC_ID_NONE,
    .p.subtitle_codec  = AV_CODEC_ID_NONE,
    .write_header      = qoa_write_header,
    .write_packet      = qoa_write_packet,
    .write_trailer     = qoa_write_trailer,
    .p.flags           = AVFMT_NOTIMESTAMPS,
    .flags_internal    = FF_OFMT_FLAG_MAX_ONE_OF_EACH,
};
