/*
 * extract pts as timecode v2, as defined by mkvtoolnix
 * Copyright (c) 2009 David Conrad
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
#include "internal.h"
#include "mux.h"

static int write_header(AVFormatContext *s)
{
    static const char *header = "# timecode format v2\n";
    avio_write(s->pb, header, strlen(header));
    avpriv_set_pts_info(s->streams[0], 64, 1, 1000);
    return 0;
}

static int write_packet(AVFormatContext *s, AVPacket *pkt)
{
    char buf[256];
    if (pkt->stream_index)
        av_log(s, AV_LOG_WARNING, "More than one stream unsupported\n");
    snprintf(buf, sizeof(buf), "%" PRId64 "\n", pkt->dts);
    avio_write(s->pb, buf, strlen(buf));
    return 0;
}

const FFOutputFormat ff_mkvtimestamp_v2_muxer = {
    .p.name        = "mkvtimestamp_v2",
    .p.long_name   = NULL_IF_CONFIG_SMALL("extract pts as timecode v2 format, as defined by mkvtoolnix"),
    .p.audio_codec = AV_CODEC_ID_NONE,
    .p.video_codec = AV_CODEC_ID_RAWVIDEO,
    .write_header = write_header,
    .write_packet = write_packet,
};
