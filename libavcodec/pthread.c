/*
 * Copyright (c) 2004 Roman Shaposhnik
 * Copyright (c) 2008 Alexander Strange (astrange@ithinksw.com)
 *
 * Many thanks to Steven M. Schultz for providing clever ideas and
 * to Michael Niedermayer <michaelni@gmx.at> for writing initial
 * implementation.
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
 * Multithreading support functions
 * @see doc/multithreading.txt
 */

#include "libavutil/attributes.h"
#include "libavutil/thread.h"

#include "avcodec.h"
#include "avcodec_internal.h"
#include "codec_internal.h"
#include "pthread_internal.h"

/**
 * Set the threading algorithms used.
 *
 * Threading requires more than one thread.
 * Frame threading requires entire frames to be passed to the codec,
 * and introduces extra decoding delay, so is incompatible with low_delay.
 *
 * @param avctx The context.
 */
static av_cold void validate_thread_parameters(AVCodecContext *avctx)
{
    int frame_threading_supported = (avctx->codec->capabilities & AV_CODEC_CAP_FRAME_THREADS)
                                && !(avctx->flags  & AV_CODEC_FLAG_LOW_DELAY)
                                && !(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS);
    if (avctx->thread_count == 1) {
        avctx->active_thread_type = 0;
    } else if (frame_threading_supported && (avctx->thread_type & FF_THREAD_FRAME)) {
        avctx->active_thread_type = FF_THREAD_FRAME;
    } else if (avctx->codec->capabilities & AV_CODEC_CAP_SLICE_THREADS &&
               avctx->thread_type & FF_THREAD_SLICE) {
        avctx->active_thread_type = FF_THREAD_SLICE;
    } else if (!(ffcodec(avctx->codec)->caps_internal & FF_CODEC_CAP_AUTO_THREADS)) {
        avctx->thread_count       = 1;
        avctx->active_thread_type = 0;
    }

    if (avctx->thread_count > MAX_AUTO_THREADS)
        av_log(avctx, AV_LOG_WARNING,
               "Application has requested %d threads. Using a thread count greater than %d is not recommended.\n",
               avctx->thread_count, MAX_AUTO_THREADS);
}

av_cold int ff_thread_init(AVCodecContext *avctx)
{
    validate_thread_parameters(avctx);

    if (avctx->active_thread_type&FF_THREAD_SLICE)
        return ff_slice_thread_init(avctx);
    else if (avctx->active_thread_type&FF_THREAD_FRAME)
        return ff_frame_thread_init(avctx);

    return 0;
}

av_cold void ff_thread_free(AVCodecContext *avctx)
{
    if (avctx->active_thread_type&FF_THREAD_FRAME)
        ff_frame_thread_free(avctx, avctx->thread_count);
    else
        ff_slice_thread_free(avctx);
}
