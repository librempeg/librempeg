/*
 * Copyright 2012 Stefano Sabatini <stefasab gmail com>
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

#include <stdint.h>

#include "libavutil/dict.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/pixfmt.h"

#include "libavformat/avformat.h"

#include "libavcodec/avcodec.h"

#include "lavfutils.h"

int ff_load_image(uint8_t *data[4], int linesize[4],
                  int *w, int *h, enum AVPixelFormat *pix_fmt,
                  const char *filename, void *log_ctx)
{
    const AVInputFormat *iformat = NULL;
    AVFormatContext *format_ctx = NULL;
    const AVCodec *codec;
    AVCodecContext *codec_ctx = NULL;
    AVCodecParameters *par;
    AVFrame *frame = NULL;
    int ret = 0;
    AVPacket pkt;
    AVDictionary *opt=NULL;

    iformat = av_find_input_format("image2pipe");
    if ((ret = avformat_open_input(&format_ctx, filename, iformat, NULL)) < 0) {
        av_log(log_ctx, AV_LOG_ERROR,
               "Failed to open input file '%s'\n", filename);
        return ret;
    }

    if ((ret = avformat_find_stream_info(format_ctx, NULL)) < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Find stream info failed\n");
        goto end;
    }

    par = format_ctx->streams[0]->codecpar;
    codec = avcodec_find_decoder(par->codec_id);
    if (!codec) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to find codec\n");
        ret = AVERROR(EINVAL);
        goto end;
    }

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to alloc video decoder context\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = avcodec_parameters_to_context(codec_ctx, par);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to copy codec parameters to decoder context\n");
        goto end;
    }

    av_dict_set(&opt, "thread_type", "slice", 0);
    if ((ret = avcodec_open2(codec_ctx, codec, &opt)) < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to open codec\n");
        goto end;
    }

    if (!(frame = av_frame_alloc()) ) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to alloc frame\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = av_read_frame(format_ctx, &pkt);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to read frame from file\n");
        goto end;
    }

    ret = avcodec_send_packet(codec_ctx, &pkt);
    av_packet_unref(&pkt);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Error submitting a packet to decoder\n");
        goto end;
    }

    ret = avcodec_receive_frame(codec_ctx, frame);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to decode image from file\n");
        goto end;
    }

    *w       = frame->width;
    *h       = frame->height;
    *pix_fmt = frame->format;

    if ((ret = av_image_alloc(data, linesize, *w, *h, *pix_fmt, 16)) < 0)
        goto end;
    ret = 0;

    av_image_copy2(data, linesize, frame->data, frame->linesize,
                   *pix_fmt, *w, *h);

end:
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&format_ctx);
    av_frame_free(&frame);
    av_dict_free(&opt);

    if (ret < 0)
        av_log(log_ctx, AV_LOG_ERROR, "Error loading image file '%s'\n", filename);
    return ret;
}
