/*
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

#include "framepool.h"
#include "libavutil/avassert.h"
#include "libavutil/avutil.h"
#include "libavutil/buffer.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/imgutils_internal.h"
#include "libavutil/mem.h"
#include "libavutil/pixfmt.h"
#include "libavutil/thread.h"

struct FFFramePool {

    enum AVMediaType type;

    AVMutex mutex;

    /* video */
    int width;
    int height;

    /* audio */
    int planes;
    int channels;
    int nb_samples;

    /* common */
    int format;
    int align;
    int linesize[4];
    AVBufferPool *pools[4];

};

FFFramePool *ff_frame_pool_video_init(AVBufferRef* (*alloc)(size_t size),
                                      int width,
                                      int height,
                                      enum AVPixelFormat format,
                                      int align)
{
    int i, ret;
    FFFramePool *pool;
    ptrdiff_t linesizes[4];
    size_t sizes[4];

    pool = av_mallocz(sizeof(FFFramePool));
    if (!pool)
        return NULL;

    if (ff_mutex_init(&pool->mutex, NULL))
        goto fail;

    pool->type = AVMEDIA_TYPE_VIDEO;
    pool->width = width;
    pool->height = height;
    pool->format = format;
    pool->align = align;

    if ((ret = av_image_check_size2(width, height, INT64_MAX, format, 0, NULL)) < 0) {
        goto fail;
    }

    if (!pool->linesize[0]) {
        ret = av_image_fill_linesizes(pool->linesize, pool->format,
                                      FFALIGN(pool->width, align));
        if (ret < 0) {
            goto fail;
        }

        for (i = 0; i < 4 && pool->linesize[i]; i++) {
            pool->linesize[i] = FFALIGN(pool->linesize[i], pool->align);
            if ((pool->linesize[i] & (pool->align - 1)))
                goto fail;
        }
    }

    for (i = 0; i < 4; i++)
        linesizes[i] = pool->linesize[i];

    if (av_image_fill_plane_sizes(sizes, pool->format,
                                  pool->height,
                                  linesizes) < 0) {
        goto fail;
    }

    for (i = 0; i < 4 && sizes[i]; i++) {
        if (sizes[i] > SIZE_MAX - align)
            goto fail;
        pool->pools[i] = av_buffer_pool_init(sizes[i] + align, alloc);
        if (!pool->pools[i])
            goto fail;
    }

    return pool;

fail:
    ff_frame_pool_uninit(&pool);
    return NULL;
}

int ff_frame_pool_audio_resize(FFFramePool *pool,
                               AVBufferRef* (*alloc)(size_t size),
                               int channels,
                               int nb_samples,
                               enum AVSampleFormat format,
                               int align)
{
    int ret, planar;

    ff_mutex_lock(&pool->mutex);
    planar = av_sample_fmt_is_planar(format);

    pool->type = AVMEDIA_TYPE_AUDIO;
    pool->planes = planar ? channels : 1;
    pool->channels = channels;
    pool->nb_samples = nb_samples;
    pool->format = format;
    pool->align = align;

    ret = av_samples_get_buffer_size(&pool->linesize[0], channels,
                                     nb_samples, format, 0);
    if (ret < 0)
        goto fail;

    if (pool->linesize[0] > SIZE_MAX - align) {
        ret = AVERROR(EINVAL);
        goto fail;
    }
    av_buffer_pool_uninit(&pool->pools[0]);
    pool->pools[0] = av_buffer_pool_init(pool->linesize[0] + align, NULL);
    if (!pool->pools[0]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ff_mutex_unlock(&pool->mutex);

    return 0;

fail:
    ff_mutex_unlock(&pool->mutex);
    return ret;
}

FFFramePool *ff_frame_pool_audio_init(AVBufferRef* (*alloc)(size_t size),
                                      int channels,
                                      int nb_samples,
                                      enum AVSampleFormat format,
                                      int align)
{
    int ret, planar;
    FFFramePool *pool;

    pool = av_mallocz(sizeof(FFFramePool));
    if (!pool)
        return NULL;

    planar = av_sample_fmt_is_planar(format);

    if (ff_mutex_init(&pool->mutex, NULL))
        goto fail;

    pool->type = AVMEDIA_TYPE_AUDIO;
    pool->planes = planar ? channels : 1;
    pool->channels = channels;
    pool->nb_samples = nb_samples;
    pool->format = format;
    pool->align = align;

    ret = av_samples_get_buffer_size(&pool->linesize[0], channels,
                                     nb_samples, format, 0);
    if (ret < 0)
        goto fail;

    if (pool->linesize[0] > SIZE_MAX - align)
        goto fail;
    pool->pools[0] = av_buffer_pool_init(pool->linesize[0] + align, NULL);
    if (!pool->pools[0])
        goto fail;

    return pool;

fail:
    ff_frame_pool_uninit(&pool);
    return NULL;
}

int ff_frame_pool_get_video_config(FFFramePool *pool,
                                   int *width,
                                   int *height,
                                   enum AVPixelFormat *format,
                                   int *align)
{
    if (!pool)
        return AVERROR(EINVAL);

    ff_mutex_lock(&pool->mutex);

    av_assert0(pool->type == AVMEDIA_TYPE_VIDEO);

    *width = pool->width;
    *height = pool->height;
    *format = pool->format;
    *align = pool->align;

    ff_mutex_unlock(&pool->mutex);

    return 0;
}

int ff_frame_pool_get_audio_config(FFFramePool *pool,
                                   int *channels,
                                   int *nb_samples,
                                   enum AVSampleFormat *format,
                                   int *align)
{
    if (!pool)
        return AVERROR(EINVAL);

    ff_mutex_lock(&pool->mutex);

    av_assert0(pool->type == AVMEDIA_TYPE_AUDIO);

    *channels = pool->channels;
    *nb_samples = pool->nb_samples;
    *format = pool->format;
    *align = pool->align;

    ff_mutex_unlock(&pool->mutex);

    return 0;
}

AVFrame *ff_frame_pool_get(FFFramePool *pool, AVFifo *fifo_empty_frames)
{
    int i;
    AVFrame *frame;
    const AVPixFmtDescriptor *desc;

    if (fifo_empty_frames && av_fifo_can_read(fifo_empty_frames) > 0) {
        av_fifo_read(fifo_empty_frames, &frame, 1);
    } else {
        frame = av_frame_alloc();
    }
    if (!frame)
        return NULL;

    ff_mutex_lock(&pool->mutex);

    switch(pool->type) {
    case AVMEDIA_TYPE_VIDEO:
        desc = av_pix_fmt_desc_get(pool->format);
        if (!desc) {
            goto fail;
        }

        frame->width = pool->width;
        frame->height = pool->height;
        frame->format = pool->format;

        for (i = 0; i < 4; i++) {
            frame->linesize[i] = pool->linesize[i];
            if (!pool->pools[i])
                break;

            frame->buf[i] = av_buffer_pool_get(pool->pools[i]);
            if (!frame->buf[i])
                goto fail;

            frame->data[i] = (uint8_t *)FFALIGN((uintptr_t)frame->buf[i]->data, pool->align);
        }

        if (desc->flags & AV_PIX_FMT_FLAG_PAL) {
            enum AVPixelFormat format =
                pool->format == AV_PIX_FMT_PAL8 ? AV_PIX_FMT_BGR8 : pool->format;

            av_assert0(frame->data[1] != NULL);
            if (avpriv_set_systematic_pal2((uint32_t *)frame->data[1], format) < 0)
                goto fail;
        }

        frame->extended_data = frame->data;
        break;
    case AVMEDIA_TYPE_AUDIO:
        frame->nb_samples = pool->nb_samples;
        frame->ch_layout.nb_channels = pool->channels;
        frame->format = pool->format;
        frame->linesize[0] = pool->linesize[0];

        if (pool->planes > AV_NUM_DATA_POINTERS) {
            frame->extended_data = av_calloc(pool->planes,
                                             sizeof(*frame->extended_data));
            frame->nb_extended_buf = pool->planes - AV_NUM_DATA_POINTERS;
            frame->extended_buf  = av_calloc(frame->nb_extended_buf,
                                             sizeof(*frame->extended_buf));
            if (!frame->extended_data || !frame->extended_buf)
                goto fail;
        } else {
            frame->extended_data = frame->data;
            av_assert0(frame->nb_extended_buf == 0);
        }

        if (!pool->pools[0])
            break;

        for (i = 0; i < FFMIN(pool->planes, AV_NUM_DATA_POINTERS); i++) {
            frame->buf[i] = av_buffer_pool_get(pool->pools[0]);
            if (!frame->buf[i])
                goto fail;
            frame->extended_data[i] = frame->data[i] =
                (uint8_t *)FFALIGN((uintptr_t)frame->buf[i]->data, pool->align);
        }
        for (i = 0; i < frame->nb_extended_buf; i++) {
            frame->extended_buf[i] = av_buffer_pool_get(pool->pools[0]);
            if (!frame->extended_buf[i])
                goto fail;
            frame->extended_data[i + AV_NUM_DATA_POINTERS] =
                (uint8_t *)FFALIGN((uintptr_t)frame->extended_buf[i]->data, pool->align);
        }

        break;
    default:
        av_assert0(0);
    }

    ff_mutex_unlock(&pool->mutex);

    return frame;
fail:
    ff_mutex_unlock(&pool->mutex);

    av_frame_free(&frame);
    return NULL;
}

void ff_frame_pool_uninit(FFFramePool **pool)
{
    if (!pool || !*pool)
        return;

    ff_mutex_lock(&(*pool)->mutex);

    for (int i = 0; i < 4; i++)
        av_buffer_pool_uninit(&(*pool)->pools[i]);

    ff_mutex_unlock(&(*pool)->mutex);

    ff_mutex_destroy(&(*pool)->mutex);

    av_freep(pool);
}
