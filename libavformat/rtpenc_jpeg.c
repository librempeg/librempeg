/*
 * RTP JPEG-compressed video Packetizer, RFC 2435
 * Copyright (c) 2012 Samuel Pitoiset
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

#include "libavcodec/bytestream.h"
#include "libavcodec/mjpeg.h"
#include "libavcodec/jpegtables.h"
#include "libavutil/intreadwrite.h"
#include "rtpenc.h"

void ff_rtp_send_jpeg(AVFormatContext *s1, const uint8_t *buf, int size)
{
    RTPMuxContext *s = s1->priv_data;
    const uint8_t *qtables[4] = { NULL };
    int nb_qtables = 0;
    uint8_t type = 2; /* initialized non-0/1 value for RTP/JPEG type check*/
    uint8_t w, h;
    uint8_t *p;
    int off = 0; /* fragment offset of the current JPEG frame */
    int len;
    int i;
    int default_huffman_tables = 0;

    s->buf_ptr   = s->buf;
    s->timestamp = s->cur_timestamp;

    /* convert video pixel dimensions from pixels to blocks */
    w = AV_CEIL_RSHIFT(s1->streams[0]->codecpar->width, 3);
    h = AV_CEIL_RSHIFT(s1->streams[0]->codecpar->height, 3);

    /* preparse the header for getting some info */
    for (i = 0; i < size; i++) {
        if (buf[i] != 0xff)
            continue;

        if (buf[i + 1] == DQT) {
            int tables, j;
            if (buf[i + 4] & 0xF0)
                av_log(s1, AV_LOG_WARNING,
                       "Only 8-bit precision is supported.\n");

            /* a quantization table is 64 bytes long */
            tables = AV_RB16(&buf[i + 2]) / 65;
            if (i + 5 + tables * 65 > size) {
                av_log(s1, AV_LOG_ERROR, "Too short JPEG header. Aborted!\n");
                return;
            }
            if (nb_qtables + tables > 4) {
                av_log(s1, AV_LOG_ERROR, "Invalid number of quantisation tables\n");
                return;
            }

            for (j = 0; j < tables; j++)
                qtables[nb_qtables + j] = buf + i + 5 + j * 65;
            nb_qtables += tables;
        } else if (buf[i + 1] == SOF0) {
            if (buf[i + 14] != 17 || buf[i + 17] != 17) {
                av_log(s1, AV_LOG_ERROR,
                       "Only 1x1 chroma blocks are supported. Aborted!\n");
                return;
            }

            /*
             * Find out the sampling factor in SOF0.
             * In SOF0, hsample/vsample is inserted in form of (2<<4) | (type ? 2 : 1).
             * First 4-bit is hsample while Last 4-bit is vsample.
             */

            /* Luma channel sampling factor in 4:2:2 chroma subsampling are 2x1 */
            if (buf[i + 11] == 33) {
                type = 0;
            /* Luma channel sampling factor in 4:2:0 chroma subsampling are 2x2 */
            } else if (buf[i + 11] == 34) {
                type = 1;
            } else {
                av_log(s1, AV_LOG_ERROR, "Unsupported pixel format\n");
                return;
            }
        } else if (buf[i + 1] == DHT) {
            int dht_size = AV_RB16(&buf[i + 2]);
            default_huffman_tables |= 1 << 4;
            i += 3;
            dht_size -= 2;
            if (i + dht_size >= size)
                continue;
            while (dht_size > 0)
                switch (buf[i + 1]) {
                case 0x00:
                    if (   dht_size >= 29
                        && !memcmp(buf + i +  2, ff_mjpeg_bits_dc_luminance + 1, 16)
                        && !memcmp(buf + i + 18, ff_mjpeg_val_dc, 12)) {
                        default_huffman_tables |= 1;
                        i += 29;
                        dht_size -= 29;
                    } else {
                        i += dht_size;
                        dht_size = 0;
                    }
                    break;
                case 0x01:
                    if (   dht_size >= 29
                        && !memcmp(buf + i +  2, ff_mjpeg_bits_dc_chrominance + 1, 16)
                        && !memcmp(buf + i + 18, ff_mjpeg_val_dc, 12)) {
                        default_huffman_tables |= 1 << 1;
                        i += 29;
                        dht_size -= 29;
                    } else {
                        i += dht_size;
                        dht_size = 0;
                    }
                    break;
                case 0x10:
                    if (   dht_size >= 179
                        && !memcmp(buf + i +  2, ff_mjpeg_bits_ac_luminance   + 1, 16)
                        && !memcmp(buf + i + 18, ff_mjpeg_val_ac_luminance, 162)) {
                        default_huffman_tables |= 1 << 2;
                        i += 179;
                        dht_size -= 179;
                    } else {
                        i += dht_size;
                        dht_size = 0;
                    }
                    break;
                case 0x11:
                    if (   dht_size >= 179
                        && !memcmp(buf + i +  2, ff_mjpeg_bits_ac_chrominance + 1, 16)
                        && !memcmp(buf + i + 18, ff_mjpeg_val_ac_chrominance, 162)) {
                        default_huffman_tables |= 1 << 3;
                        i += 179;
                        dht_size -= 179;
                    } else {
                        i += dht_size;
                        dht_size = 0;
                    }
                    break;
                default:
                    i += dht_size;
                    dht_size = 0;
                    continue;
            }
        } else if (buf[i + 1] == SOS) {
            /* SOS is last marker in the header */
            i += AV_RB16(&buf[i + 2]) + 2;
            if (i > size) {
                av_log(s1, AV_LOG_ERROR,
                       "Insufficient data. Aborted!\n");
                return;
            }
            break;
        }
    }

    /* Check validity of RTP/JPEG type */
    if (type != 0 && type != 1) {
        av_log(s1, AV_LOG_ERROR,
               "Invalid RTP/JPEG type\n");
        return;
    }

    if (default_huffman_tables && default_huffman_tables != 31) {
        av_log(s1, AV_LOG_ERROR,
               "RFC 2435 requires standard Huffman tables for jpeg\n");
        return;
    }
    if (nb_qtables && nb_qtables != 2)
        av_log(s1, AV_LOG_WARNING,
               "RFC 2435 suggests two quantization tables, %d provided\n",
               nb_qtables);

    /* skip JPEG header */
    buf  += i;
    size -= i;

    for (i = size - 2; i >= 0; i--) {
        if (buf[i] == 0xff && buf[i + 1] == EOI) {
            /* Remove the EOI marker */
            size = i;
            break;
        }
    }

    p = s->buf_ptr;
    while (size > 0) {
        int hdr_size = 8;

        if (off == 0 && nb_qtables)
            hdr_size += 4 + 64 * nb_qtables;

        /* payload max in one packet */
        len = FFMIN(size, s->max_payload_size - hdr_size);

        /* set main header */
        bytestream_put_byte(&p, 0);
        bytestream_put_be24(&p, off);
        bytestream_put_byte(&p, type);
        bytestream_put_byte(&p, 255);
        bytestream_put_byte(&p, w);
        bytestream_put_byte(&p, h);

        if (off == 0 && nb_qtables) {
            /* set quantization tables header */
            bytestream_put_byte(&p, 0);
            bytestream_put_byte(&p, 0);
            bytestream_put_be16(&p, 64 * nb_qtables);

            for (i = 0; i < nb_qtables; i++)
                bytestream_put_buffer(&p, qtables[i], 64);
        }

        /* copy payload data */
        memcpy(p, buf, len);

        /* marker bit is last packet in frame */
        ff_rtp_send_data(s1, s->buf, len + hdr_size, size == len);

        buf  += len;
        size -= len;
        off  += len;
        p     = s->buf;
    }
}
