/*
 * Copyright (C) 2019 Philip Langdale <philipl@overt.org>
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

#include "libavutil/avassert.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_cuda_internal.h"
#include "libavutil/cuda_check.h"

#include "filters.h"
#include "yadif.h"

#include "cuda/load_helper.h"

extern const unsigned char ff_vf_bwdif_cuda_ptx_data[];
extern const unsigned int ff_vf_bwdif_cuda_ptx_len;

typedef struct DeintCUDAContext {
    YADIFContext yadif;

    AVCUDADeviceContext *hwctx;
    AVBufferRef         *device_ref;
    AVBufferRef         *input_frames_ref;
    AVHWFramesContext   *input_frames;

    CUmodule    cu_module;
    CUfunction  cu_func_uchar;
    CUfunction  cu_func_uchar2;
    CUfunction  cu_func_ushort;
    CUfunction  cu_func_ushort2;
} DeintCUDAContext;

#define DIV_UP(a, b) ( ((a) + (b) - 1) / (b) )
#define ALIGN_UP(a, b) (((a) + (b) - 1) & ~((b) - 1))
#define BLOCKX 32
#define BLOCKY 16

#define CHECK_CU(x) FF_CUDA_CHECK_DL(ctx, s->hwctx->internal->cuda_dl, x)

static CUresult call_kernel(AVFilterContext *ctx, CUfunction func,
                            CUdeviceptr prev, CUdeviceptr cur, CUdeviceptr next,
                            CUarray_format format, int channels,
                            int src_width,  // Width is pixels per channel
                            int src_height, // Height is pixels per channel
                            int src_pitch,  // Pitch is bytes
                            CUdeviceptr dst,
                            int dst_width,  // Width is pixels per channel
                            int dst_height, // Height is pixels per channel
                            int dst_pitch,  // Pitch is pixels per channel
                            int parity, int tff, int clip_max)
{
    DeintCUDAContext *s = ctx->priv;
    YADIFContext *y = &s->yadif;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    CUtexObject tex_prev = 0, tex_cur = 0, tex_next = 0;
    int is_field_end = y->current_field == YADIF_FIELD_END;
    int ret;

    void *args[] = { &dst, &tex_prev, &tex_cur, &tex_next,
                     &dst_width, &dst_height, &dst_pitch,
                     &src_width, &src_height, &parity, &tff,
                     &is_field_end, &clip_max };

    CUDA_TEXTURE_DESC tex_desc = {
        .filterMode = CU_TR_FILTER_MODE_POINT,
        .flags = CU_TRSF_READ_AS_INTEGER,
    };

    CUDA_RESOURCE_DESC res_desc = {
        .resType = CU_RESOURCE_TYPE_PITCH2D,
        .res.pitch2D.format = format,
        .res.pitch2D.numChannels = channels,
        .res.pitch2D.width = src_width,
        .res.pitch2D.height = src_height,
        .res.pitch2D.pitchInBytes = src_pitch,
    };

    res_desc.res.pitch2D.devPtr = (CUdeviceptr)prev;
    ret = CHECK_CU(cu->cuTexObjectCreate(&tex_prev, &res_desc, &tex_desc, NULL));
    if (ret < 0)
        goto exit;

    res_desc.res.pitch2D.devPtr = (CUdeviceptr)cur;
    ret = CHECK_CU(cu->cuTexObjectCreate(&tex_cur, &res_desc, &tex_desc, NULL));
    if (ret < 0)
        goto exit;

    res_desc.res.pitch2D.devPtr = (CUdeviceptr)next;
    ret = CHECK_CU(cu->cuTexObjectCreate(&tex_next, &res_desc, &tex_desc, NULL));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(cu->cuLaunchKernel(func,
                                      DIV_UP(dst_width, BLOCKX), DIV_UP(dst_height, BLOCKY), 1,
                                      BLOCKX, BLOCKY, 1,
                                      0, s->hwctx->stream, args, NULL));

exit:
    if (tex_prev)
        CHECK_CU(cu->cuTexObjectDestroy(tex_prev));
    if (tex_cur)
        CHECK_CU(cu->cuTexObjectDestroy(tex_cur));
    if (tex_next)
        CHECK_CU(cu->cuTexObjectDestroy(tex_next));

    return ret;
}

static void filter(AVFilterContext *ctx, AVFrame *dst,
                   int parity, int tff)
{
    DeintCUDAContext *s = ctx->priv;
    YADIFContext *y = &s->yadif;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    CUcontext dummy;
    int i, ret;

    ret = CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
    if (ret < 0)
        return;

    for (i = 0; i < y->csp->nb_components; i++) {
        CUfunction func;
        CUarray_format format;
        int pixel_size, channels, clip_max;
        const AVComponentDescriptor *comp = &y->csp->comp[i];

        if (comp->plane < i) {
            // We process planes as a whole, so don't reprocess
            // them for additional components
            continue;
        }

        pixel_size = (comp->depth + comp->shift) / 8;
        channels = comp->step / pixel_size;
        if (pixel_size > 2 || channels > 2) {
            av_log(ctx, AV_LOG_ERROR, "Unsupported pixel format: %s\n", y->csp->name);
            goto exit;
        }
        switch (pixel_size) {
        case 1:
            func = channels == 1 ? s->cu_func_uchar : s->cu_func_uchar2;
            format = CU_AD_FORMAT_UNSIGNED_INT8;
            break;
        case 2:
            func = channels == 1 ? s->cu_func_ushort : s->cu_func_ushort2;
            format = CU_AD_FORMAT_UNSIGNED_INT16;
            break;
        default:
            av_log(ctx, AV_LOG_ERROR, "Unsupported pixel format: %s\n", y->csp->name);
            goto exit;
        }

        clip_max = (1 << (comp->depth + comp->shift)) - 1;

        av_log(ctx, AV_LOG_TRACE,
               "Deinterlacing plane %d: pixel_size: %d channels: %d\n",
               comp->plane, pixel_size, channels);
        call_kernel(ctx, func,
                    (CUdeviceptr)y->prev->data[i],
                    (CUdeviceptr)y->cur->data[i],
                    (CUdeviceptr)y->next->data[i],
                    format, channels,
                    AV_CEIL_RSHIFT(y->cur->width, i ? y->csp->log2_chroma_w : 0),
                    AV_CEIL_RSHIFT(y->cur->height, i ? y->csp->log2_chroma_h : 0),
                    y->cur->linesize[i],
                    (CUdeviceptr)dst->data[i],
                    AV_CEIL_RSHIFT(dst->width, i ? y->csp->log2_chroma_w : 0),
                    AV_CEIL_RSHIFT(dst->height, i ? y->csp->log2_chroma_h : 0),
                    dst->linesize[i] / comp->step,
                    parity, tff, clip_max);
    }

    if (y->current_field == YADIF_FIELD_END) {
        y->current_field = YADIF_FIELD_NORMAL;
    }

exit:
    CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    return;
}

static av_cold void deint_cuda_uninit(AVFilterContext *ctx)
{
    CUcontext dummy;
    DeintCUDAContext *s = ctx->priv;

    if (s->hwctx && s->cu_module) {
        CudaFunctions *cu = s->hwctx->internal->cuda_dl;
        CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
        CHECK_CU(cu->cuModuleUnload(s->cu_module));
        CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    }

    ff_yadif_uninit(ctx);

    av_buffer_unref(&s->device_ref);
    s->hwctx = NULL;
    av_buffer_unref(&s->input_frames_ref);
    s->input_frames = NULL;
}

static int config_input(AVFilterLink *inlink)
{
    FilterLink        *l = ff_filter_link(inlink);
    AVFilterContext *ctx = inlink->dst;
    DeintCUDAContext *s  = ctx->priv;

    if (!l->hw_frames_ctx) {
        av_log(ctx, AV_LOG_ERROR, "A hardware frames reference is "
               "required to associate the processing device.\n");
        return AVERROR(EINVAL);
    }

    s->input_frames_ref = av_buffer_ref(l->hw_frames_ctx);
    if (!s->input_frames_ref) {
        av_log(ctx, AV_LOG_ERROR, "A input frames reference create "
               "failed.\n");
        return AVERROR(ENOMEM);
    }
    s->input_frames = (AVHWFramesContext*)s->input_frames_ref->data;

    return 0;
}

static int config_output(AVFilterLink *link)
{
    FilterLink *l = ff_filter_link(link);
    AVHWFramesContext *output_frames;
    AVFilterContext *ctx = link->src;
    DeintCUDAContext *s = ctx->priv;
    YADIFContext *y = &s->yadif;
    CudaFunctions *cu;
    int ret = 0;
    CUcontext dummy;

    av_assert0(s->input_frames);
    s->device_ref = av_buffer_ref(s->input_frames->device_ref);
    if (!s->device_ref) {
        av_log(ctx, AV_LOG_ERROR, "A device reference create "
               "failed.\n");
        return AVERROR(ENOMEM);
    }
    s->hwctx = ((AVHWDeviceContext*)s->device_ref->data)->hwctx;
    cu = s->hwctx->internal->cuda_dl;

    l->hw_frames_ctx = av_hwframe_ctx_alloc(s->device_ref);
    if (!l->hw_frames_ctx) {
        av_log(ctx, AV_LOG_ERROR, "Failed to create HW frame context "
               "for output.\n");
        ret = AVERROR(ENOMEM);
        goto exit;
    }

    output_frames = (AVHWFramesContext*)l->hw_frames_ctx->data;

    output_frames->format    = AV_PIX_FMT_CUDA;
    output_frames->sw_format = s->input_frames->sw_format;
    output_frames->width     = ctx->inputs[0]->w;
    output_frames->height    = ctx->inputs[0]->h;

    output_frames->initial_pool_size = 4;

    ret = ff_filter_init_hw_frames(ctx, link, 10);
    if (ret < 0)
        goto exit;

    ret = av_hwframe_ctx_init(l->hw_frames_ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to initialise CUDA frame "
               "context for output: %d\n", ret);
        goto exit;
    }

    ret = ff_yadif_config_output_common(link);
    if (ret < 0)
        goto exit;

    y->csp = av_pix_fmt_desc_get(output_frames->sw_format);
    y->filter = filter;

    if (AV_CEIL_RSHIFT(link->w, y->csp->log2_chroma_w) < 3 || AV_CEIL_RSHIFT(link->h, y->csp->log2_chroma_h) < 3) {
        av_log(ctx, AV_LOG_ERROR, "Video with planes less than 3 columns or lines is not supported\n");
        ret = AVERROR(EINVAL);
        goto exit;
    }

    ret = CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
    if (ret < 0)
        goto exit;

    ret = ff_cuda_load_module(ctx, s->hwctx, &s->cu_module, ff_vf_bwdif_cuda_ptx_data, ff_vf_bwdif_cuda_ptx_len);
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_func_uchar, s->cu_module, "bwdif_uchar"));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_func_uchar2, s->cu_module, "bwdif_uchar2"));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_func_ushort, s->cu_module, "bwdif_ushort"));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_func_ushort2, s->cu_module, "bwdif_ushort2"));
    if (ret < 0)
        goto exit;

exit:
    CHECK_CU(cu->cuCtxPopCurrent(&dummy));

    return ret;
}

static const AVClass bwdif_cuda_class = {
    .class_name = "bwdif_cuda",
    .option     = ff_yadif_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_FILTER,
};

static const AVFilterPad deint_cuda_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = ff_yadif_filter_frame,
        .config_props  = config_input,
    },
};

static const AVFilterPad deint_cuda_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .request_frame = ff_yadif_request_frame,
        .config_props  = config_output,
    },
};

const FFFilter ff_vf_bwdif_cuda = {
    .p.name         = "bwdif_cuda",
    .p.description  = NULL_IF_CONFIG_SMALL("Deinterlace CUDA frames"),
    .p.priv_class   = &bwdif_cuda_class,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size      = sizeof(DeintCUDAContext),
    .uninit         = deint_cuda_uninit,
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_CUDA),
    FILTER_INPUTS(deint_cuda_inputs),
    FILTER_OUTPUTS(deint_cuda_outputs),
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
