/*
 * Copyright (c) 2015 Paul B Mahol
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

#include <tesseract/capi.h>

#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct OCRContext {
    const AVClass *class;

    char *datapath;
    char *language;
    char *whitelist;
    char *blacklist;

    TessBaseAPI *tess;
} OCRContext;

#define OFFSET(x) offsetof(OCRContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ocr_options[] = {
    { "datapath",  "set datapath",            OFFSET(datapath),  AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "language",  "set language",            OFFSET(language),  AV_OPT_TYPE_STRING, {.str="eng"}, 0, 0, FLAGS },
    { "whitelist", "set character whitelist", OFFSET(whitelist), AV_OPT_TYPE_STRING, {.str="0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ.:;,-+_!?\"'[]{}()<>|/\\=*&%$#@!~ "}, 0, 0, FLAGS },
    { "blacklist", "set character blacklist", OFFSET(blacklist), AV_OPT_TYPE_STRING, {.str=""},    0, 0, FLAGS },
    { NULL }
};

static av_cold int init(AVFilterContext *ctx)
{
    OCRContext *s = ctx->priv;

    s->tess = TessBaseAPICreate();
    if (TessBaseAPIInit3(s->tess, s->datapath, s->language) == -1) {
        av_log(ctx, AV_LOG_ERROR, "failed to init tesseract\n");
        return AVERROR(EINVAL);
    }

    if (!TessBaseAPISetVariable(s->tess, "tessedit_char_whitelist", s->whitelist)) {
        av_log(ctx, AV_LOG_ERROR, "failed to set whitelist\n");
        return AVERROR(EINVAL);
    }

    if (!TessBaseAPISetVariable(s->tess, "tessedit_char_blacklist", s->blacklist)) {
        av_log(ctx, AV_LOG_ERROR, "failed to set blacklist\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_DEBUG, "Tesseract version: %s\n", TessVersion());

    return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_YUVJ411P,
    AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_NONE
};

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVDictionary **metadata = &in->metadata;
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    OCRContext *s = ctx->priv;
    char *result;
    int *confs;

    result = TessBaseAPIRect(s->tess, in->data[0], 1,
                             in->linesize[0], 0, 0, in->width, in->height);
    confs = TessBaseAPIAllWordConfidences(s->tess);
    av_dict_set(metadata, "lavfi.ocr.text", result, 0);
    for (int i = 0; confs[i] != -1; i++) {
        char number[256];

        snprintf(number, sizeof(number), "%d ", confs[i]);
        av_dict_set(metadata, "lavfi.ocr.confidence", number, AV_DICT_APPEND);
    }

    TessDeleteText(result);
    TessDeleteIntArray(confs);

    return ff_filter_frame(outlink, in);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    OCRContext *s = ctx->priv;

    TessBaseAPIEnd(s->tess);
    TessBaseAPIDelete(s->tess);
}

AVFILTER_DEFINE_CLASS(ocr);

static const AVFilterPad ocr_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_ocr = {
    .p.name        = "ocr",
    .p.description = NULL_IF_CONFIG_SMALL("Optical Character Recognition."),
    .p.priv_class  = &ocr_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(OCRContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(ocr_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
