/*
 * Copyright (c) 2013 Georg Martius <georg dot martius at web dot de>
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

#include "libavutil/mem.h"
#include "vidstabutils.h"

const enum AVPixelFormat ff_vidstab_pix_fmts[] =  {
    AV_PIX_FMT_YUV444P,  AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUV411P,  AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_YUV440P,  AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_RGB24,    AV_PIX_FMT_BGR24,   AV_PIX_FMT_RGBA,
    AV_PIX_FMT_NONE
};

/** convert AV's pixelformat to vid.stab pixelformat */
VSPixelFormat ff_av2vs_pixfmt(AVFilterContext *ctx, enum AVPixelFormat pf)
{
    switch (pf) {
    case AV_PIX_FMT_YUV420P:  return PF_YUV420P;
    case AV_PIX_FMT_YUV422P:  return PF_YUV422P;
    case AV_PIX_FMT_YUV444P:  return PF_YUV444P;
    case AV_PIX_FMT_YUV410P:  return PF_YUV410P;
    case AV_PIX_FMT_YUV411P:  return PF_YUV411P;
    case AV_PIX_FMT_YUV440P:  return PF_YUV440P;
    case AV_PIX_FMT_YUVA420P: return PF_YUVA420P;
    case AV_PIX_FMT_GRAY8:    return PF_GRAY8;
    case AV_PIX_FMT_RGB24:    return PF_RGB24;
    case AV_PIX_FMT_BGR24:    return PF_BGR24;
    case AV_PIX_FMT_RGBA:     return PF_RGBA;
    default:
        av_log(ctx, AV_LOG_ERROR, "cannot deal with pixel format %i\n", pf);
        return PF_NONE;
    }
}

/** struct to hold a valid context for logging from within vid.stab lib */
typedef struct VS2AVLogCtx {
    const AVClass *class;
} VS2AVLogCtx;

/** wrapper to log vs_log into av_log */
static int vs2av_log(int type, const char *tag, const char *format, ...)
{
    va_list ap;
    VS2AVLogCtx ctx;
    AVClass class = {
        .class_name = tag,
        .option     = 0,
        .version    = LIBAVUTIL_VERSION_INT,
        .category   = AV_CLASS_CATEGORY_FILTER,
    };
    ctx.class = &class;
    va_start(ap, format);
    av_vlog(&ctx, type, format, ap);
    va_end(ap);
    return VS_OK;
}

/** sets the memory allocation function and logging constants to av versions */
void ff_vs_init(void)
{
    vs_malloc  = av_malloc;
    vs_zalloc  = av_mallocz;
    vs_realloc = av_realloc;
    vs_free    = av_free;

    VS_ERROR_TYPE = AV_LOG_ERROR;
    VS_WARN_TYPE  = AV_LOG_WARNING;
    VS_INFO_TYPE  = AV_LOG_INFO;
    VS_MSG_TYPE   = AV_LOG_VERBOSE;

    vs_log   = vs2av_log;

    VS_ERROR = 0;
    VS_OK    = 1;
}
