/*
 * CPiA video decoder.
 * Copyright (c) 2010 Hans de Goede <hdegoede@redhat.com>
 *
 * This decoder is based on the LGPL code available at
 * https://v4l4j.googlecode.com/svn/v4l4j/trunk/libvideo/libv4lconvert/cpia1.c
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
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"


#define FRAME_HEADER_SIZE 64
#define MAGIC_0         0x19    /**< First header byte */
#define MAGIC_1         0x68    /**< Second header byte */
#define SUBSAMPLE_420      0
#define SUBSAMPLE_422      1
#define YUVORDER_YUYV      0
#define YUVORDER_UYVY      1
#define NOT_COMPRESSED     0
#define COMPRESSED         1
#define NO_DECIMATION      0
#define DECIMATION_ENAB    1
#define EOL             0xfd    /**< End Of Line marker */
#define EOI             0xff    /**< End Of Image marker */


typedef struct {
    AVFrame *frame;
} CpiaContext;


static int cpia_decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                             int *got_frame, AVPacket* avpkt)
{
    CpiaContext* const cpia = avctx->priv_data;
    int i,j,ret;

    const uint8_t *const header = avpkt->data;
    const uint8_t *src;
    int src_size;
    uint16_t linelength;
    uint8_t skip;

    AVFrame *frame = cpia->frame;
    uint8_t *y, *u, *v, *y_end, *u_end, *v_end;

    // Check header
    if ( avpkt->size < FRAME_HEADER_SIZE + avctx->height * 3
      || header[0] != MAGIC_0 || header[1] != MAGIC_1
      || (header[17] != SUBSAMPLE_420 && header[17] != SUBSAMPLE_422)
      || (header[18] != YUVORDER_YUYV && header[18] != YUVORDER_UYVY)
      || (header[28] != NOT_COMPRESSED && header[28] != COMPRESSED)
      || (header[29] != NO_DECIMATION && header[29] != DECIMATION_ENAB)
    ) {
        av_log(avctx, AV_LOG_ERROR, "Invalid header!\n");
        return AVERROR_INVALIDDATA;
    }

    // currently unsupported properties
    if (header[17] == SUBSAMPLE_422) {
        avpriv_report_missing_feature(avctx, "4:2:2 subsampling");
        return AVERROR_PATCHWELCOME;
    }
    if (header[18] == YUVORDER_UYVY) {
        avpriv_report_missing_feature(avctx, "YUV byte order UYVY");
        return AVERROR_PATCHWELCOME;
    }
    if (header[29] == DECIMATION_ENAB) {
        avpriv_report_missing_feature(avctx, "Decimation");
        return AVERROR_PATCHWELCOME;
    }

    src = header + FRAME_HEADER_SIZE;
    src_size = avpkt->size - FRAME_HEADER_SIZE;

    if (header[28] == NOT_COMPRESSED) {
        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->flags |= AV_FRAME_FLAG_KEY;
    } else {
        frame->pict_type = AV_PICTURE_TYPE_P;
        frame->flags &= ~AV_FRAME_FLAG_KEY;
    }

    // Get buffer filled with previous frame
    if ((ret = ff_reget_buffer(avctx, frame, 0)) < 0)
        return ret;


    for ( i = 0;
          i < frame->height;
          i++, src += linelength, src_size -= linelength
    ) {
        // Read line length, two byte little endian
        linelength = AV_RL16(src);
        src += 2;
        src_size -= 2;

        if (src_size < linelength) {
            frame->decode_error_flags = FF_DECODE_ERROR_INVALID_BITSTREAM;
            av_log(avctx, AV_LOG_WARNING, "Frame ended unexpectedly!\n");
            break;
        }
        if (src[linelength - 1] != EOL) {
            frame->decode_error_flags = FF_DECODE_ERROR_INVALID_BITSTREAM;
            av_log(avctx, AV_LOG_WARNING, "Wrong line length %d or line not terminated properly (found 0x%02x)!\n", linelength, src[linelength - 1]);
            break;
        }

        /* Update the data pointers. Y data is on every line.
         * U and V data on every second line
         */
        y = &frame->data[0][i * frame->linesize[0]];
        u = &frame->data[1][(i >> 1) * frame->linesize[1]];
        v = &frame->data[2][(i >> 1) * frame->linesize[2]];
        y_end = y + frame->linesize[0] - 1;
        u_end = u + frame->linesize[1] - 1;
        v_end = v + frame->linesize[2] - 1;

        if ((i & 1) && header[17] == SUBSAMPLE_420) {
            /* We are on an odd line and 420 subsample is used.
             * On this line only Y values are specified, one per pixel.
             */
            for (j = 0; j < linelength - 1; j++) {
                if (y > y_end) {
                    frame->decode_error_flags = FF_DECODE_ERROR_INVALID_BITSTREAM;
                    av_log(avctx, AV_LOG_WARNING, "Decoded data exceeded linesize!\n");
                    break;
                }
                if ((src[j] & 1) && header[28] == COMPRESSED) {
                    /* It seems that odd lines are always uncompressed, but
                     * we do it according to specification anyways.
                     */
                    skip = src[j] >> 1;
                    y += skip;
                } else {
                    *(y++) = src[j];
                }
            }
        } else if (header[17] == SUBSAMPLE_420) {
            /* We are on an even line and 420 subsample is used.
             * On this line each pair of pixels is described by four bytes.
             */
            for (j = 0; j < linelength - 4; ) {
                if (y + 1 > y_end || u > u_end || v > v_end) {
                    frame->decode_error_flags = FF_DECODE_ERROR_INVALID_BITSTREAM;
                    av_log(avctx, AV_LOG_WARNING, "Decoded data exceeded linesize!\n");
                    break;
                }
                if ((src[j] & 1) && header[28] == COMPRESSED) {
                    // Skip amount of pixels and move forward one byte
                    skip = src[j] >> 1;
                    y += skip;
                    u += skip >> 1;
                    v += skip >> 1;
                    j++;
                } else {
                    // Set image data as specified and move forward 4 bytes
                    *(y++) = src[j];
                    *(u++) = src[j+1];
                    *(y++) = src[j+2];
                    *(v++) = src[j+3];
                    j += 4;
                }
            }
        }
    }

    *got_frame = 1;
    if ((ret = av_frame_ref(rframe, cpia->frame)) < 0)
        return ret;

    return avpkt->size;
}

static av_cold int cpia_decode_init(AVCodecContext *avctx)
{
    CpiaContext *s = avctx->priv_data;

    // output pixel format
    avctx->pix_fmt = AV_PIX_FMT_YUV420P;

    s->frame = av_frame_alloc();
    if (!s->frame)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int cpia_decode_end(AVCodecContext *avctx)
{
    CpiaContext *s = avctx->priv_data;

    av_frame_free(&s->frame);

    return 0;
}

const FFCodec ff_cpia_decoder = {
    .p.name         = "cpia",
    CODEC_LONG_NAME("CPiA video format"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CPIA,
    .priv_data_size = sizeof(CpiaContext),
    .init           = cpia_decode_init,
    .close          = cpia_decode_end,
    FF_CODEC_DECODE_CB(cpia_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
