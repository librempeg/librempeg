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

#include "libavutil/buffer.h"
#include "libavutil/hwcontext.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

typedef struct CudaUploadContext {
    const AVClass *class;
    int device_idx;

    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;
} CudaUploadContext;

static av_cold int cudaupload_init(AVFilterContext *ctx)
{
    CudaUploadContext *s = ctx->priv;
    char buf[64] = { 0 };

    snprintf(buf, sizeof(buf), "%d", s->device_idx);

    return av_hwdevice_ctx_create(&s->hwdevice, AV_HWDEVICE_TYPE_CUDA, buf, NULL, 0);
}

static av_cold void cudaupload_uninit(AVFilterContext *ctx)
{
    CudaUploadContext *s = ctx->priv;

    av_buffer_unref(&s->hwframe);
    av_buffer_unref(&s->hwdevice);
}

static int cudaupload_query_formats(const AVFilterContext *ctx,
                                    AVFilterFormatsConfig **cfg_in,
                                    AVFilterFormatsConfig **cfg_out)
{
    int ret;

    static const enum AVPixelFormat input_pix_fmts[] = {
        AV_PIX_FMT_NV12, AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_P010, AV_PIX_FMT_P016, AV_PIX_FMT_YUV444P16,
        AV_PIX_FMT_0RGB32, AV_PIX_FMT_0BGR32,
#if CONFIG_VULKAN
        AV_PIX_FMT_VULKAN,
#endif
        AV_PIX_FMT_NONE,
    };
    static const enum AVPixelFormat output_pix_fmts[] = {
        AV_PIX_FMT_CUDA, AV_PIX_FMT_NONE,
    };
    AVFilterFormats *in_fmts  = ff_make_format_list(input_pix_fmts);
    AVFilterFormats *out_fmts;

    ret = ff_formats_ref(in_fmts, &cfg_in[0]->formats);
    if (ret < 0)
        return ret;

    out_fmts = ff_make_format_list(output_pix_fmts);

    ret = ff_formats_ref(out_fmts, &cfg_out[0]->formats);
    if (ret < 0)
        return ret;

    return 0;
}

static int cudaupload_config_output(AVFilterLink *outlink)
{
    FilterLink     *outl = ff_filter_link(outlink);
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    FilterLink      *inl = ff_filter_link(inlink);
    CudaUploadContext *s = ctx->priv;

    AVHWFramesContext *hwframe_ctx;
    int ret;

    av_buffer_unref(&s->hwframe);
    s->hwframe = av_hwframe_ctx_alloc(s->hwdevice);
    if (!s->hwframe)
        return AVERROR(ENOMEM);

    hwframe_ctx            = (AVHWFramesContext*)s->hwframe->data;
    hwframe_ctx->format    = AV_PIX_FMT_CUDA;
    if (inl->hw_frames_ctx) {
        AVHWFramesContext *in_hwframe_ctx = (AVHWFramesContext*)inl->hw_frames_ctx->data;
        hwframe_ctx->sw_format = in_hwframe_ctx->sw_format;
    } else {
        hwframe_ctx->sw_format = inlink->format;
    }
    hwframe_ctx->width     = inlink->w;
    hwframe_ctx->height    = inlink->h;

    ret = av_hwframe_ctx_init(s->hwframe);
    if (ret < 0)
        return ret;

    outl->hw_frames_ctx = av_buffer_ref(s->hwframe);
    if (!outl->hw_frames_ctx)
        return AVERROR(ENOMEM);

    return 0;
}

static int cudaupload_filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext   *ctx = link->dst;
    AVFilterLink  *outlink = ctx->outputs[0];

    AVFrame *out = NULL;
    int ret;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    out->width  = in->width;
    out->height = in->height;

    ret = av_hwframe_transfer_data(out, in, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error transferring data to the GPU\n");
        goto fail;
    }

    ret = av_frame_copy_props(out, in);
    if (ret < 0)
        goto fail;

    av_frame_free(&in);

    return ff_filter_frame(ctx->outputs[0], out);
fail:
    av_frame_free(&in);
    av_frame_free(&out);
    return ret;
}

#define OFFSET(x) offsetof(CudaUploadContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption cudaupload_options[] = {
    { "device", "Number of the device to use", OFFSET(device_idx), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, FLAGS },
    { NULL },
};

AVFILTER_DEFINE_CLASS(cudaupload);

static const AVFilterPad cudaupload_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = cudaupload_filter_frame,
    },
};

static const AVFilterPad cudaupload_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = cudaupload_config_output,
    },
};

const FFFilter ff_vf_hwupload_cuda = {
    .p.name        = "hwupload_cuda",
    .p.description = NULL_IF_CONFIG_SMALL("Upload a system memory frame to a CUDA device."),

    .p.priv_class  = &cudaupload_class,

    .init      = cudaupload_init,
    .uninit    = cudaupload_uninit,

    .priv_size  = sizeof(CudaUploadContext),

    FILTER_INPUTS(cudaupload_inputs),
    FILTER_OUTPUTS(cudaupload_outputs),

    FILTER_QUERY_FUNC2(cudaupload_query_formats),

    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
