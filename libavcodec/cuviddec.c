/*
 * Nvidia CUVID decoder
 * Copyright (c) 2016 Timo Rothenpieler <timo@rothenpieler.org>
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

#include "config_components.h"

#include "compat/cuda/dynlink_loader.h"

#include "libavutil/buffer.h"
#include "libavutil/mathematics.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_cuda_internal.h"
#include "libavutil/cuda_check.h"
#include "libavutil/fifo.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "avcodec.h"
#include "bsf.h"
#include "codec_internal.h"
#include "decode.h"
#include "hwconfig.h"
#include "nvdec.h"
#include "internal.h"

#if !NVDECAPI_CHECK_VERSION(9, 0)
#define cudaVideoSurfaceFormat_YUV444 2
#define cudaVideoSurfaceFormat_YUV444_16Bit 3
#endif

#if NVDECAPI_CHECK_VERSION(11, 0)
#define CUVID_HAS_AV1_SUPPORT
#endif

typedef struct CuvidContext
{
    AVClass *avclass;

    CUvideodecoder cudecoder;
    CUvideoparser cuparser;

    /* This packet coincides with AVCodecInternal.in_pkt
     * and is not owned by us. */
    AVPacket *pkt;

    char *cu_gpu;
    int nb_surfaces;
    int drop_second_field;
    char *crop_expr;
    char *resize_expr;

    struct {
        int left;
        int top;
        int right;
        int bottom;
    } crop;

    struct {
        int width;
        int height;
    } resize;

    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;

    AVFifo      *frame_queue;

    int deint_mode;
    int deint_mode_current;
    int64_t prev_pts;
    int progressive_sequence;

    int internal_error;
    int decoder_flushing;

    int *key_frame;

    cudaVideoCodec codec_type;
    cudaVideoChromaFormat chroma_format;

    CUVIDDECODECAPS caps8, caps10, caps12;

    CUVIDPARSERPARAMS cuparseinfo;
    CUVIDEOFORMATEX *cuparse_ext;

    CudaFunctions *cudl;
    CuvidFunctions *cvdl;
} CuvidContext;

typedef struct CuvidParsedFrame
{
    CUVIDPARSERDISPINFO dispinfo;
    int second_field;
    int is_deinterlacing;
} CuvidParsedFrame;

#define CHECK_CU(x) FF_CUDA_CHECK_DL(avctx, ctx->cudl, x)

// NV recommends [2;4] range
#define CUVID_MAX_DISPLAY_DELAY (4)

// Actual pool size will be determined by parser.
#define CUVID_DEFAULT_NUM_SURFACES (CUVID_MAX_DISPLAY_DELAY + 1)

static int CUDAAPI cuvid_handle_video_sequence(void *opaque, CUVIDEOFORMAT* format)
{
    AVCodecContext *avctx = opaque;
    CuvidContext *ctx = avctx->priv_data;
    AVHWFramesContext *hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    CUVIDDECODECAPS *caps = NULL;
    CUVIDDECODECREATEINFO cuinfo;
    int surface_fmt;
    int chroma_444;
    int old_nb_surfaces, fifo_size_inc, fifo_size_mul = 1;

    int old_width = avctx->width;
    int old_height = avctx->height;

    enum AVPixelFormat pix_fmts[3] = { AV_PIX_FMT_CUDA,
                                       AV_PIX_FMT_NONE,  // Will be updated below
                                       AV_PIX_FMT_NONE };

    av_log(avctx, AV_LOG_TRACE, "pfnSequenceCallback, progressive_sequence=%d\n", format->progressive_sequence);

    memset(&cuinfo, 0, sizeof(cuinfo));

    ctx->internal_error = 0;

    avctx->coded_width = cuinfo.ulWidth = format->coded_width;
    avctx->coded_height = cuinfo.ulHeight = format->coded_height;

    // apply cropping
    cuinfo.display_area.left = format->display_area.left + ctx->crop.left;
    cuinfo.display_area.top = format->display_area.top + ctx->crop.top;
    cuinfo.display_area.right = format->display_area.right - ctx->crop.right;
    cuinfo.display_area.bottom = format->display_area.bottom - ctx->crop.bottom;

    // width and height need to be set before calling ff_get_format
    if (ctx->resize_expr) {
        avctx->width = ctx->resize.width;
        avctx->height = ctx->resize.height;
    } else {
        avctx->width = cuinfo.display_area.right - cuinfo.display_area.left;
        avctx->height = cuinfo.display_area.bottom - cuinfo.display_area.top;
    }

    // target width/height need to be multiples of two
    cuinfo.ulTargetWidth = avctx->width = (avctx->width + 1) & ~1;
    cuinfo.ulTargetHeight = avctx->height = (avctx->height + 1) & ~1;

    // aspect ratio conversion, 1:1, depends on scaled resolution
    cuinfo.target_rect.left = 0;
    cuinfo.target_rect.top = 0;
    cuinfo.target_rect.right = cuinfo.ulTargetWidth;
    cuinfo.target_rect.bottom = cuinfo.ulTargetHeight;

    chroma_444 = format->chroma_format == cudaVideoChromaFormat_444;

    switch (format->bit_depth_luma_minus8) {
    case 0: // 8-bit
        if (chroma_444) {
            pix_fmts[1] = AV_PIX_FMT_YUV444P;
#ifdef NVDEC_HAVE_422_SUPPORT
        } else if (format->chroma_format == cudaVideoChromaFormat_422) {
            pix_fmts[1] = AV_PIX_FMT_NV16;
#endif
        } else {
            pix_fmts[1] = AV_PIX_FMT_NV12;
        }
        caps = &ctx->caps8;
        break;
    case 2: // 10-bit
        if (chroma_444) {
#if FF_API_NVDEC_OLD_PIX_FMTS
            pix_fmts[1] = AV_PIX_FMT_YUV444P16;
#else
            pix_fmts[1] = AV_PIX_FMT_YUV444P10MSB;
#endif
#ifdef NVDEC_HAVE_422_SUPPORT
        } else if (format->chroma_format == cudaVideoChromaFormat_422) {
#if FF_API_NVDEC_OLD_PIX_FMTS
            pix_fmts[1] = AV_PIX_FMT_P216;
#else
            pix_fmts[1] = AV_PIX_FMT_P210;
#endif
#endif
        } else {
            pix_fmts[1] = AV_PIX_FMT_P010;
        }
        caps = &ctx->caps10;
        break;
    case 4: // 12-bit
        if (chroma_444) {
#if FF_API_NVDEC_OLD_PIX_FMTS
            pix_fmts[1] = AV_PIX_FMT_YUV444P16;
#else
            pix_fmts[1] = AV_PIX_FMT_YUV444P12MSB;
#endif
#ifdef NVDEC_HAVE_422_SUPPORT
        } else if (format->chroma_format == cudaVideoChromaFormat_422) {
#if FF_API_NVDEC_OLD_PIX_FMTS
            pix_fmts[1] = AV_PIX_FMT_P216;
#else
            pix_fmts[1] = AV_PIX_FMT_P212;
#endif
#endif
        } else {
#if FF_API_NVDEC_OLD_PIX_FMTS
            pix_fmts[1] = AV_PIX_FMT_P016;
#else
            pix_fmts[1] = AV_PIX_FMT_P012;
#endif
        }
        caps = &ctx->caps12;
        break;
    default:
        break;
    }

    if (!caps || !caps->bIsSupported) {
        av_log(avctx, AV_LOG_ERROR, "unsupported bit depth: %d\n",
               format->bit_depth_luma_minus8 + 8);
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    surface_fmt = ff_get_format(avctx, pix_fmts);
    if (surface_fmt < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed: %d\n", surface_fmt);
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    av_log(avctx, AV_LOG_VERBOSE, "Formats: Original: %s | HW: %s | SW: %s\n",
           av_get_pix_fmt_name(avctx->pix_fmt),
           av_get_pix_fmt_name(surface_fmt),
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    avctx->pix_fmt = surface_fmt;

    // Update our hwframe ctx, as the get_format callback might have refreshed it!
    if (avctx->hw_frames_ctx) {
        av_buffer_unref(&ctx->hwframe);

        ctx->hwframe = av_buffer_ref(avctx->hw_frames_ctx);
        if (!ctx->hwframe) {
            ctx->internal_error = AVERROR(ENOMEM);
            return 0;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    }

    ff_set_sar(avctx, av_div_q(
        (AVRational){ format->display_aspect_ratio.x, format->display_aspect_ratio.y },
        (AVRational){ avctx->width, avctx->height }));

    ctx->deint_mode_current = format->progressive_sequence
                              ? cudaVideoDeinterlaceMode_Weave
                              : ctx->deint_mode;

    ctx->progressive_sequence = format->progressive_sequence;

    if (!format->progressive_sequence && ctx->deint_mode_current == cudaVideoDeinterlaceMode_Weave)
        avctx->flags |= AV_CODEC_FLAG_INTERLACED_DCT;
    else
        avctx->flags &= ~AV_CODEC_FLAG_INTERLACED_DCT;

    if (format->video_signal_description.video_full_range_flag)
        avctx->color_range = AVCOL_RANGE_JPEG;
    else
        avctx->color_range = AVCOL_RANGE_MPEG;

    avctx->color_primaries = format->video_signal_description.color_primaries;
    avctx->color_trc = format->video_signal_description.transfer_characteristics;
    avctx->colorspace = format->video_signal_description.matrix_coefficients;

    if (format->bitrate)
        avctx->bit_rate = format->bitrate;

    if (format->frame_rate.numerator && format->frame_rate.denominator) {
        avctx->framerate.num = format->frame_rate.numerator;
        avctx->framerate.den = format->frame_rate.denominator;
    }

    if (ctx->cudecoder
            && avctx->coded_width == format->coded_width
            && avctx->coded_height == format->coded_height
            && avctx->width == old_width
            && avctx->height == old_height
            && ctx->chroma_format == format->chroma_format
            && ctx->codec_type == format->codec)
        return 1;

    if (ctx->cudecoder) {
        av_log(avctx, AV_LOG_TRACE, "Re-initializing decoder\n");
        ctx->internal_error = CHECK_CU(ctx->cvdl->cuvidDestroyDecoder(ctx->cudecoder));
        if (ctx->internal_error < 0)
            return 0;
        ctx->cudecoder = NULL;
    }

    if (hwframe_ctx->pool && (
            hwframe_ctx->width < avctx->width ||
            hwframe_ctx->height < avctx->height ||
            hwframe_ctx->format != AV_PIX_FMT_CUDA ||
            hwframe_ctx->sw_format != avctx->sw_pix_fmt)) {
        av_log(avctx, AV_LOG_ERROR, "AVHWFramesContext is already initialized with incompatible parameters\n");
        av_log(avctx, AV_LOG_DEBUG, "width: %d <-> %d\n", hwframe_ctx->width, avctx->width);
        av_log(avctx, AV_LOG_DEBUG, "height: %d <-> %d\n", hwframe_ctx->height, avctx->height);
        av_log(avctx, AV_LOG_DEBUG, "format: %s <-> cuda\n", av_get_pix_fmt_name(hwframe_ctx->format));
        av_log(avctx, AV_LOG_DEBUG, "sw_format: %s <-> %s\n",
               av_get_pix_fmt_name(hwframe_ctx->sw_format), av_get_pix_fmt_name(avctx->sw_pix_fmt));
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    ctx->chroma_format = format->chroma_format;

    cuinfo.CodecType = ctx->codec_type = format->codec;
    cuinfo.ChromaFormat = format->chroma_format;

    switch (avctx->sw_pix_fmt) {
    case AV_PIX_FMT_NV12:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_NV12;
        break;
    case AV_PIX_FMT_P010:
    case AV_PIX_FMT_P016:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_P016;
        break;
#ifdef NVDEC_HAVE_422_SUPPORT
    case AV_PIX_FMT_NV16:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_NV16;
        break;
    case AV_PIX_FMT_P216:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_P216;
        break;
#endif
    case AV_PIX_FMT_YUV444P:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_YUV444;
        break;
    case AV_PIX_FMT_YUV444P16:
        cuinfo.OutputFormat = cudaVideoSurfaceFormat_YUV444_16Bit;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported output format: %s\n",
               av_get_pix_fmt_name(avctx->sw_pix_fmt));
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    if (ctx->deint_mode_current != cudaVideoDeinterlaceMode_Weave && !ctx->drop_second_field) {
        avctx->framerate = av_mul_q(avctx->framerate, (AVRational){2, 1});
        fifo_size_mul = 2;
    }

    old_nb_surfaces = ctx->nb_surfaces;
    ctx->nb_surfaces = FFMAX(ctx->nb_surfaces, format->min_num_decode_surfaces + 3);
    if (avctx->extra_hw_frames > 0)
        ctx->nb_surfaces += avctx->extra_hw_frames;

    fifo_size_inc = ctx->nb_surfaces * fifo_size_mul - av_fifo_can_read(ctx->frame_queue) - av_fifo_can_write(ctx->frame_queue);
    if (fifo_size_inc > 0 && av_fifo_grow2(ctx->frame_queue, fifo_size_inc) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to grow frame queue on video sequence callback\n");
        ctx->internal_error = AVERROR(ENOMEM);
        return 0;
    }

    if (ctx->nb_surfaces > old_nb_surfaces && av_reallocp_array(&ctx->key_frame, ctx->nb_surfaces, sizeof(int)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to grow key frame array on video sequence callback\n");
        ctx->internal_error = AVERROR(ENOMEM);
        return 0;
    }

    cuinfo.ulNumDecodeSurfaces = ctx->nb_surfaces;
    cuinfo.ulNumOutputSurfaces = 1;
    cuinfo.ulCreationFlags = cudaVideoCreate_PreferCUVID;
    cuinfo.bitDepthMinus8 = format->bit_depth_luma_minus8;
    cuinfo.DeinterlaceMode = ctx->deint_mode_current;

    ctx->internal_error = CHECK_CU(ctx->cvdl->cuvidCreateDecoder(&ctx->cudecoder, &cuinfo));
    if (ctx->internal_error < 0)
        return 0;

    if (!hwframe_ctx->pool) {
        hwframe_ctx->format = AV_PIX_FMT_CUDA;
        hwframe_ctx->sw_format = avctx->sw_pix_fmt;
        hwframe_ctx->width = avctx->width;
        hwframe_ctx->height = avctx->height;

        if ((ctx->internal_error = av_hwframe_ctx_init(ctx->hwframe)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_ctx_init failed\n");
            return 0;
        }
    }

    if(ctx->cuparseinfo.ulMaxNumDecodeSurfaces != cuinfo.ulNumDecodeSurfaces) {
        ctx->cuparseinfo.ulMaxNumDecodeSurfaces = cuinfo.ulNumDecodeSurfaces;
        return cuinfo.ulNumDecodeSurfaces;
    }

    return 1;
}

static int CUDAAPI cuvid_handle_picture_decode(void *opaque, CUVIDPICPARAMS* picparams)
{
    AVCodecContext *avctx = opaque;
    CuvidContext *ctx = avctx->priv_data;

    av_log(avctx, AV_LOG_TRACE, "pfnDecodePicture\n");

    if(picparams->intra_pic_flag)
        ctx->key_frame[picparams->CurrPicIdx] = picparams->intra_pic_flag;

    ctx->internal_error = CHECK_CU(ctx->cvdl->cuvidDecodePicture(ctx->cudecoder, picparams));
    if (ctx->internal_error < 0)
        return 0;

    return 1;
}

static int CUDAAPI cuvid_handle_picture_display(void *opaque, CUVIDPARSERDISPINFO* dispinfo)
{
    AVCodecContext *avctx = opaque;
    CuvidContext *ctx = avctx->priv_data;
    CuvidParsedFrame parsed_frame = { { 0 } };
    int ret;

    parsed_frame.dispinfo = *dispinfo;
    ctx->internal_error = 0;

    // For some reason, dispinfo->progressive_frame is sometimes wrong.
    parsed_frame.dispinfo.progressive_frame = ctx->progressive_sequence;

    if (ctx->deint_mode_current == cudaVideoDeinterlaceMode_Weave) {
        ret = av_fifo_write(ctx->frame_queue, &parsed_frame, 1);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "Writing frame to fifo failed!\n");
    } else {
        parsed_frame.is_deinterlacing = 1;
        ret = av_fifo_write(ctx->frame_queue, &parsed_frame, 1);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "Writing first frame to fifo failed!\n");

        if (!ctx->drop_second_field) {
            parsed_frame.second_field = 1;
            ret = av_fifo_write(ctx->frame_queue, &parsed_frame, 1);
            if (ret < 0)
                av_log(avctx, AV_LOG_ERROR, "Writing second frame to fifo failed!\n");
        }
    }

    return 1;
}

static int cuvid_is_buffer_full(AVCodecContext *avctx)
{
    CuvidContext *ctx = avctx->priv_data;

    int shift = 0;
    if (ctx->deint_mode != cudaVideoDeinterlaceMode_Weave && !ctx->drop_second_field)
        shift = 1;

    // shift/divide frame count to ensure the buffer is still signalled full if one half-frame has already been returned when deinterlacing.
    return ((av_fifo_can_read(ctx->frame_queue) + shift) >> shift) + ctx->cuparseinfo.ulMaxDisplayDelay >= ctx->nb_surfaces;
}

static int cuvid_decode_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    CuvidContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVCUDADeviceContext *device_hwctx = device_ctx->hwctx;
    CUcontext dummy, cuda_ctx = device_hwctx->cuda_ctx;
    CUVIDSOURCEDATAPACKET cupkt;
    int ret = 0, eret = 0, is_flush = ctx->decoder_flushing;

    av_log(avctx, AV_LOG_TRACE, "cuvid_decode_packet\n");

    if (is_flush && avpkt && avpkt->size)
        return AVERROR_EOF;

    if (cuvid_is_buffer_full(avctx) && avpkt && avpkt->size)
        return AVERROR(EAGAIN);

    ret = CHECK_CU(ctx->cudl->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0) {
        return ret;
    }

    memset(&cupkt, 0, sizeof(cupkt));

    if (avpkt && avpkt->size) {
        cupkt.payload_size = avpkt->size;
        cupkt.payload = avpkt->data;

        if (avpkt->pts != AV_NOPTS_VALUE) {
            cupkt.flags = CUVID_PKT_TIMESTAMP;
            if (avctx->pkt_timebase.num && avctx->pkt_timebase.den)
                cupkt.timestamp = av_rescale_q(avpkt->pts, avctx->pkt_timebase, (AVRational){1, 10000000});
            else
                cupkt.timestamp = avpkt->pts;
        }
    } else {
        cupkt.flags = CUVID_PKT_ENDOFSTREAM;
        ctx->decoder_flushing = 1;
    }

    // When flushing, only actually flush cuvid when the output buffer has been fully emptied.
    // CUVID happily dumps out a ton of frames with no regard for its own available surfaces.
    if (!ctx->decoder_flushing || (ctx->decoder_flushing && !av_fifo_can_read(ctx->frame_queue)))
        ret = CHECK_CU(ctx->cvdl->cuvidParseVideoData(ctx->cuparser, &cupkt));
    else
        ret = 0;

    if (ret < 0)
        goto error;

    // cuvidParseVideoData doesn't return an error just because stuff failed...
    if (ctx->internal_error) {
        av_log(avctx, AV_LOG_ERROR, "cuvid decode callback error\n");
        ret = ctx->internal_error;
        goto error;
    }

error:
    eret = CHECK_CU(ctx->cudl->cuCtxPopCurrent(&dummy));

    if (eret < 0)
        return eret;
    else if (ret < 0)
        return ret;
    else if (is_flush)
        return AVERROR_EOF;
    else
        return 0;
}

static int cuvid_output_frame(AVCodecContext *avctx, AVFrame *frame)
{
    CuvidContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVCUDADeviceContext *device_hwctx = device_ctx->hwctx;
    CUcontext dummy, cuda_ctx = device_hwctx->cuda_ctx;
    CuvidParsedFrame parsed_frame;
    CUdeviceptr mapped_frame = 0;
    int ret = 0, eret = 0;

    av_log(avctx, AV_LOG_TRACE, "cuvid_output_frame\n");

    if (ctx->decoder_flushing) {
        ret = cuvid_decode_packet(avctx, NULL);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
    }

    if (!cuvid_is_buffer_full(avctx)) {
        AVPacket *const pkt = ctx->pkt;
        ret = ff_decode_get_packet(avctx, pkt);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
        ret = cuvid_decode_packet(avctx, pkt);
        av_packet_unref(pkt);
        // cuvid_is_buffer_full() should avoid this.
        if (ret == AVERROR(EAGAIN))
            ret = AVERROR_EXTERNAL;
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
    }

    ret = CHECK_CU(ctx->cudl->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        return ret;

    if (av_fifo_read(ctx->frame_queue, &parsed_frame, 1) >= 0) {
        const AVPixFmtDescriptor *pixdesc;
        CUVIDPROCPARAMS params;
        unsigned int pitch = 0;
        int offset = 0;
        int i;

        memset(&params, 0, sizeof(params));
        params.progressive_frame = parsed_frame.dispinfo.progressive_frame;
        params.second_field = parsed_frame.second_field;
        params.top_field_first = parsed_frame.dispinfo.top_field_first;

        ret = CHECK_CU(ctx->cvdl->cuvidMapVideoFrame(ctx->cudecoder, parsed_frame.dispinfo.picture_index, &mapped_frame, &pitch, &params));
        if (ret < 0)
            goto error;

        if (avctx->pix_fmt == AV_PIX_FMT_CUDA) {
            ret = av_hwframe_get_buffer(ctx->hwframe, frame, 0);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "av_hwframe_get_buffer failed\n");
                goto error;
            }

            ret = ff_decode_frame_props(avctx, frame);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "ff_decode_frame_props failed\n");
                goto error;
            }

            pixdesc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);

            for (i = 0; i < pixdesc->nb_components; i++) {
                int height = avctx->height >> (i ? pixdesc->log2_chroma_h : 0);
                CUDA_MEMCPY2D cpy = {
                    .srcMemoryType = CU_MEMORYTYPE_DEVICE,
                    .dstMemoryType = CU_MEMORYTYPE_DEVICE,
                    .srcDevice     = mapped_frame,
                    .dstDevice     = (CUdeviceptr)frame->data[i],
                    .srcPitch      = pitch,
                    .dstPitch      = frame->linesize[i],
                    .srcY          = offset,
                    .WidthInBytes  = FFMIN(pitch, frame->linesize[i]),
                    .Height        = height,
                };

                ret = CHECK_CU(ctx->cudl->cuMemcpy2DAsync(&cpy, device_hwctx->stream));
                if (ret < 0)
                    goto error;

                offset += height;
            }
        } else if (avctx->pix_fmt == AV_PIX_FMT_NV12      ||
                   avctx->pix_fmt == AV_PIX_FMT_P010      ||
                   avctx->pix_fmt == AV_PIX_FMT_P016      ||
#ifdef NVDEC_HAVE_422_SUPPORT
                   avctx->pix_fmt == AV_PIX_FMT_NV16      ||
                   avctx->pix_fmt == AV_PIX_FMT_P216      ||
#endif
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P   ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P16) {
            unsigned int offset = 0;
            AVFrame *tmp_frame = av_frame_alloc();
            if (!tmp_frame) {
                av_log(avctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
                ret = AVERROR(ENOMEM);
                goto error;
            }

            pixdesc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);

            tmp_frame->format        = AV_PIX_FMT_CUDA;
            tmp_frame->hw_frames_ctx = av_buffer_ref(ctx->hwframe);
            if (!tmp_frame->hw_frames_ctx) {
                ret = AVERROR(ENOMEM);
                av_frame_free(&tmp_frame);
                goto error;
            }

            tmp_frame->width         = avctx->width;
            tmp_frame->height        = avctx->height;

            /*
             * Note that the following logic would not work for three plane
             * YUV420 because the pitch value is different for the chroma
             * planes.
             */
            for (i = 0; i < pixdesc->nb_components; i++) {
                tmp_frame->data[i]     = (uint8_t*)mapped_frame + offset;
                tmp_frame->linesize[i] = pitch;
                offset += pitch * (avctx->height >> (i ? pixdesc->log2_chroma_h : 0));
            }

            ret = ff_get_buffer(avctx, frame, 0);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "ff_get_buffer failed\n");
                av_frame_free(&tmp_frame);
                goto error;
            }

            ret = av_hwframe_transfer_data(frame, tmp_frame, 0);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "av_hwframe_transfer_data failed\n");
                av_frame_free(&tmp_frame);
                goto error;
            }
            av_frame_free(&tmp_frame);
        } else {
            ret = AVERROR_BUG;
            goto error;
        }

        if (ctx->key_frame[parsed_frame.dispinfo.picture_index])
            frame->flags |= AV_FRAME_FLAG_KEY;
        else
            frame->flags &= ~AV_FRAME_FLAG_KEY;
        ctx->key_frame[parsed_frame.dispinfo.picture_index] = 0;

        frame->width = avctx->width;
        frame->height = avctx->height;
        if (avctx->pkt_timebase.num && avctx->pkt_timebase.den)
            frame->pts = av_rescale_q(parsed_frame.dispinfo.timestamp, (AVRational){1, 10000000}, avctx->pkt_timebase);
        else
            frame->pts = parsed_frame.dispinfo.timestamp;

        if (parsed_frame.second_field) {
            if (ctx->prev_pts == INT64_MIN) {
                ctx->prev_pts = frame->pts;
                frame->pts += (avctx->pkt_timebase.den * avctx->framerate.den) / (avctx->pkt_timebase.num * avctx->framerate.num);
            } else {
                int pts_diff = (frame->pts - ctx->prev_pts) / 2;
                ctx->prev_pts = frame->pts;
                frame->pts += pts_diff;
            }
        }

        /* CUVIDs opaque reordering breaks the internal pkt logic.
         * So set pkt_pts and clear all the other pkt_ fields.
         */
        frame->duration = 0;

        if (!parsed_frame.is_deinterlacing && !parsed_frame.dispinfo.progressive_frame)
            frame->flags |= AV_FRAME_FLAG_INTERLACED;

        if ((frame->flags & AV_FRAME_FLAG_INTERLACED) && parsed_frame.dispinfo.top_field_first)
            frame->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;
    } else if (ctx->decoder_flushing) {
        ret = AVERROR_EOF;
    } else {
        ret = AVERROR(EAGAIN);
    }

error:
    if (ret < 0)
        av_frame_unref(frame);

    if (mapped_frame)
        eret = CHECK_CU(ctx->cvdl->cuvidUnmapVideoFrame(ctx->cudecoder, mapped_frame));

    eret = CHECK_CU(ctx->cudl->cuCtxPopCurrent(&dummy));

    if (eret < 0)
        return eret;
    else
        return ret;
}

static av_cold int cuvid_decode_end(AVCodecContext *avctx)
{
    CuvidContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = ctx->hwdevice ? (AVHWDeviceContext *)ctx->hwdevice->data : NULL;
    AVCUDADeviceContext *device_hwctx = device_ctx ? device_ctx->hwctx : NULL;
    CUcontext dummy, cuda_ctx = device_hwctx ? device_hwctx->cuda_ctx : NULL;

    av_fifo_freep2(&ctx->frame_queue);

    if (cuda_ctx) {
        ctx->cudl->cuCtxPushCurrent(cuda_ctx);

        if (ctx->cuparser)
            ctx->cvdl->cuvidDestroyVideoParser(ctx->cuparser);

        if (ctx->cudecoder)
            ctx->cvdl->cuvidDestroyDecoder(ctx->cudecoder);

        ctx->cudl->cuCtxPopCurrent(&dummy);
    }

    ctx->cudl = NULL;

    av_buffer_unref(&ctx->hwframe);
    av_buffer_unref(&ctx->hwdevice);

    av_freep(&ctx->key_frame);
    av_freep(&ctx->cuparse_ext);

    cuvid_free_functions(&ctx->cvdl);

    return 0;
}

static int cuvid_test_capabilities(AVCodecContext *avctx,
                                   const CUVIDPARSERPARAMS *cuparseinfo,
                                   int probed_width,
                                   int probed_height,
                                   int bit_depth, int is_yuv422, int is_yuv444)
{
    CuvidContext *ctx = avctx->priv_data;
    CUVIDDECODECAPS *caps;
    int res8 = 0, res10 = 0, res12 = 0;

    if (!ctx->cvdl->cuvidGetDecoderCaps) {
        av_log(avctx, AV_LOG_WARNING, "Used Nvidia driver is too old to perform a capability check.\n");
        av_log(avctx, AV_LOG_WARNING, "The minimum required version is "
#if defined(_WIN32) || defined(__CYGWIN__)
            "378.66"
#else
            "378.13"
#endif
            ". Continuing blind.\n");
        ctx->caps8.bIsSupported = ctx->caps10.bIsSupported = 1;
        // 12 bit was not supported before the capability check was introduced, so disable it.
        ctx->caps12.bIsSupported = 0;
        return 0;
    }

    ctx->caps8.eCodecType = ctx->caps10.eCodecType = ctx->caps12.eCodecType
        = cuparseinfo->CodecType;

    ctx->caps8.eChromaFormat = ctx->caps10.eChromaFormat = ctx->caps12.eChromaFormat
        = is_yuv444 ? cudaVideoChromaFormat_444 :
#ifdef NVDEC_HAVE_422_SUPPORT
          (is_yuv422 ? cudaVideoChromaFormat_422 : cudaVideoChromaFormat_420);
#else
          cudaVideoChromaFormat_420;
#endif

    ctx->caps8.nBitDepthMinus8 = 0;
    ctx->caps10.nBitDepthMinus8 = 2;
    ctx->caps12.nBitDepthMinus8 = 4;

    res8 = CHECK_CU(ctx->cvdl->cuvidGetDecoderCaps(&ctx->caps8));
    res10 = CHECK_CU(ctx->cvdl->cuvidGetDecoderCaps(&ctx->caps10));
    res12 = CHECK_CU(ctx->cvdl->cuvidGetDecoderCaps(&ctx->caps12));

    av_log(avctx, AV_LOG_VERBOSE, "CUVID capabilities for %s:\n", avctx->codec->name);
    av_log(avctx, AV_LOG_VERBOSE, "8 bit: supported: %d, min_width: %d, max_width: %d, min_height: %d, max_height: %d\n",
           ctx->caps8.bIsSupported, ctx->caps8.nMinWidth, ctx->caps8.nMaxWidth, ctx->caps8.nMinHeight, ctx->caps8.nMaxHeight);
    av_log(avctx, AV_LOG_VERBOSE, "10 bit: supported: %d, min_width: %d, max_width: %d, min_height: %d, max_height: %d\n",
           ctx->caps10.bIsSupported, ctx->caps10.nMinWidth, ctx->caps10.nMaxWidth, ctx->caps10.nMinHeight, ctx->caps10.nMaxHeight);
    av_log(avctx, AV_LOG_VERBOSE, "12 bit: supported: %d, min_width: %d, max_width: %d, min_height: %d, max_height: %d\n",
           ctx->caps12.bIsSupported, ctx->caps12.nMinWidth, ctx->caps12.nMaxWidth, ctx->caps12.nMinHeight, ctx->caps12.nMaxHeight);

    switch (bit_depth) {
    case 10:
        caps = &ctx->caps10;
        if (res10 < 0)
            return res10;
        break;
    case 12:
        caps = &ctx->caps12;
        if (res12 < 0)
            return res12;
        break;
    default:
        caps = &ctx->caps8;
        if (res8 < 0)
            return res8;
    }

    if (!ctx->caps8.bIsSupported) {
        av_log(avctx, AV_LOG_ERROR, "Codec %s is not supported with this chroma format.\n", avctx->codec->name);
        return AVERROR(EINVAL);
    }

    if (!caps->bIsSupported) {
        av_log(avctx, AV_LOG_ERROR, "Bit depth %d with this chroma format is not supported.\n", bit_depth);
        return AVERROR(EINVAL);
    }

    if (probed_width > caps->nMaxWidth || probed_width < caps->nMinWidth) {
        av_log(avctx, AV_LOG_ERROR, "Video width %d not within range from %d to %d\n",
               probed_width, caps->nMinWidth, caps->nMaxWidth);
        return AVERROR(EINVAL);
    }

    if (probed_height > caps->nMaxHeight || probed_height < caps->nMinHeight) {
        av_log(avctx, AV_LOG_ERROR, "Video height %d not within range from %d to %d\n",
               probed_height, caps->nMinHeight, caps->nMaxHeight);
        return AVERROR(EINVAL);
    }

    if ((probed_width * probed_height) / 256 > caps->nMaxMBCount) {
        av_log(avctx, AV_LOG_ERROR, "Video macroblock count %d exceeds maximum of %d\n",
               (int)(probed_width * probed_height) / 256, caps->nMaxMBCount);
        return AVERROR(EINVAL);
    }

    return 0;
}

static av_cold int cuvid_decode_init(AVCodecContext *avctx)
{
    CuvidContext *ctx = avctx->priv_data;
    AVCUDADeviceContext *device_hwctx;
    AVHWDeviceContext *device_ctx;
    AVHWFramesContext *hwframe_ctx;
    CUVIDSOURCEDATAPACKET seq_pkt;
    CUcontext cuda_ctx = NULL;
    CUcontext dummy;
    uint8_t *extradata;
    int extradata_size;
    int ret = 0;

    enum AVPixelFormat pix_fmts[3] = { AV_PIX_FMT_CUDA,
                                       AV_PIX_FMT_NONE,
                                       AV_PIX_FMT_NONE };

    int probed_width = avctx->coded_width ? avctx->coded_width : 1280;
    int probed_height = avctx->coded_height ? avctx->coded_height : 720;
    int probed_bit_depth = 8, is_yuv444 = 0, is_yuv422 = 0;

    const AVPixFmtDescriptor *probe_desc = av_pix_fmt_desc_get(avctx->pix_fmt);
    if (probe_desc && probe_desc->nb_components)
        probed_bit_depth = probe_desc->comp[0].depth;

    if (probe_desc && !probe_desc->log2_chroma_w && !probe_desc->log2_chroma_h)
        is_yuv444 = 1;

#ifdef NVDEC_HAVE_422_SUPPORT
    if (probe_desc && probe_desc->log2_chroma_w && !probe_desc->log2_chroma_h)
        is_yuv422 = 1;
#endif

    // Pick pixel format based on bit depth and chroma sampling.
    switch (probed_bit_depth) {
    case 10:
#if FF_API_NVDEC_OLD_PIX_FMTS
        pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P16 : (is_yuv422 ? AV_PIX_FMT_P216 : AV_PIX_FMT_P010);
#else
        pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P10MSB : (is_yuv422 ? AV_PIX_FMT_P210 : AV_PIX_FMT_P010);
#endif
        break;
    case 12:
#if FF_API_NVDEC_OLD_PIX_FMTS
        pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P16 : (is_yuv422 ? AV_PIX_FMT_P216 : AV_PIX_FMT_P016);
#else
        pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P12MSB : (is_yuv422 ? AV_PIX_FMT_P212 : AV_PIX_FMT_P012);
#endif
        break;
    default:
        pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P : (is_yuv422 ? AV_PIX_FMT_NV16 : AV_PIX_FMT_NV12);
        break;
    }

    ctx->pkt = avctx->internal->in_pkt;
    // Accelerated transcoding scenarios with 'ffmpeg' require that the
    // pix_fmt be set to AV_PIX_FMT_CUDA early. The sw_pix_fmt, and the
    // pix_fmt for non-accelerated transcoding, do not need to be correct
    // but need to be set to something.
    ret = ff_get_format(avctx, pix_fmts);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed: %d\n", ret);
        return ret;
    }
    avctx->pix_fmt = ret;

    if (ctx->resize_expr && sscanf(ctx->resize_expr, "%dx%d",
                                   &ctx->resize.width, &ctx->resize.height) != 2) {
        av_log(avctx, AV_LOG_ERROR, "Invalid resize expressions\n");
        ret = AVERROR(EINVAL);
        goto error;
    }

    if (ctx->crop_expr && sscanf(ctx->crop_expr, "%dx%dx%dx%d",
                                 &ctx->crop.top, &ctx->crop.bottom,
                                 &ctx->crop.left, &ctx->crop.right) != 4) {
        av_log(avctx, AV_LOG_ERROR, "Invalid cropping expressions\n");
        ret = AVERROR(EINVAL);
        goto error;
    }

    ret = cuvid_load_functions(&ctx->cvdl, avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed loading nvcuvid.\n");
        goto error;
    }

    // respect the deprecated "surfaces" option if non-default value is given by user;
    if(ctx->nb_surfaces < 0)
        ctx->nb_surfaces = CUVID_DEFAULT_NUM_SURFACES;

    ctx->frame_queue = av_fifo_alloc2(ctx->nb_surfaces, sizeof(CuvidParsedFrame), 0);
    if (!ctx->frame_queue) {
        ret = AVERROR(ENOMEM);
        goto error;
    }

    if (avctx->hw_frames_ctx) {
        ctx->hwframe = av_buffer_ref(avctx->hw_frames_ctx);
        if (!ctx->hwframe) {
            ret = AVERROR(ENOMEM);
            goto error;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;

        ctx->hwdevice = av_buffer_ref(hwframe_ctx->device_ref);
        if (!ctx->hwdevice) {
            ret = AVERROR(ENOMEM);
            goto error;
        }
    } else {
        if (avctx->hw_device_ctx) {
            ctx->hwdevice = av_buffer_ref(avctx->hw_device_ctx);
            if (!ctx->hwdevice) {
                ret = AVERROR(ENOMEM);
                goto error;
            }
        } else {
            ret = av_hwdevice_ctx_create(&ctx->hwdevice, AV_HWDEVICE_TYPE_CUDA, ctx->cu_gpu, NULL, 0);
            if (ret < 0)
                goto error;
        }

        ctx->hwframe = av_hwframe_ctx_alloc(ctx->hwdevice);
        if (!ctx->hwframe) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_ctx_alloc failed\n");
            ret = AVERROR(ENOMEM);
            goto error;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    }

    device_ctx = hwframe_ctx->device_ctx;
    device_hwctx = device_ctx->hwctx;

    cuda_ctx = device_hwctx->cuda_ctx;
    ctx->cudl = device_hwctx->internal->cuda_dl;

    memset(&ctx->cuparseinfo, 0, sizeof(ctx->cuparseinfo));
    memset(&seq_pkt, 0, sizeof(seq_pkt));

    switch (avctx->codec->id) {
#if CONFIG_H264_CUVID_DECODER
    case AV_CODEC_ID_H264:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_H264;
        break;
#endif
#if CONFIG_HEVC_CUVID_DECODER
    case AV_CODEC_ID_HEVC:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_HEVC;
        break;
#endif
#if CONFIG_MJPEG_CUVID_DECODER
    case AV_CODEC_ID_MJPEG:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_JPEG;
        break;
#endif
#if CONFIG_MPEG1_CUVID_DECODER
    case AV_CODEC_ID_MPEG1VIDEO:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_MPEG1;
        break;
#endif
#if CONFIG_MPEG2_CUVID_DECODER
    case AV_CODEC_ID_MPEG2VIDEO:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_MPEG2;
        break;
#endif
#if CONFIG_MPEG4_CUVID_DECODER
    case AV_CODEC_ID_MPEG4:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_MPEG4;
        break;
#endif
#if CONFIG_VP8_CUVID_DECODER
    case AV_CODEC_ID_VP8:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_VP8;
        break;
#endif
#if CONFIG_VP9_CUVID_DECODER
    case AV_CODEC_ID_VP9:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_VP9;
        break;
#endif
#if CONFIG_VC1_CUVID_DECODER
    case AV_CODEC_ID_VC1:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_VC1;
        break;
#endif
#if CONFIG_AV1_CUVID_DECODER && defined(CUVID_HAS_AV1_SUPPORT)
    case AV_CODEC_ID_AV1:
        ctx->cuparseinfo.CodecType = cudaVideoCodec_AV1;
        break;
#endif
    default:
        av_log(avctx, AV_LOG_ERROR, "Invalid CUVID codec!\n");
        return AVERROR_BUG;
    }

    if (ffcodec(avctx->codec)->bsfs) {
        const AVCodecParameters *par = avctx->internal->bsf->par_out;
        extradata = par->extradata;
        extradata_size = par->extradata_size;
    } else {
        extradata = avctx->extradata;
        extradata_size = avctx->extradata_size;
    }

    // Check first bit to determine whether it's AV1CodecConfigurationRecord.
    // Skip first 4 bytes of AV1CodecConfigurationRecord to keep configOBUs
    // only, otherwise cuvidParseVideoData report unknown error.
    if (avctx->codec->id == AV_CODEC_ID_AV1 &&
            extradata_size > 4 &&
            extradata[0] & 0x80) {
        extradata += 4;
        extradata_size -= 4;
    }

    ctx->cuparse_ext = av_mallocz(sizeof(*ctx->cuparse_ext)
            + FFMAX(extradata_size - (int)sizeof(ctx->cuparse_ext->raw_seqhdr_data), 0));
    if (!ctx->cuparse_ext) {
        ret = AVERROR(ENOMEM);
        goto error;
    }

    if (extradata_size > 0)
        memcpy(ctx->cuparse_ext->raw_seqhdr_data, extradata, extradata_size);
    ctx->cuparse_ext->format.seqhdr_data_length = extradata_size;

    ctx->cuparseinfo.pExtVideoInfo = ctx->cuparse_ext;

    ctx->key_frame = av_mallocz(ctx->nb_surfaces * sizeof(int));
    if (!ctx->key_frame) {
        ret = AVERROR(ENOMEM);
        goto error;
    }

    ctx->cuparseinfo.ulMaxNumDecodeSurfaces = 1;
    ctx->cuparseinfo.ulMaxDisplayDelay = (avctx->flags & AV_CODEC_FLAG_LOW_DELAY) ? 0 : CUVID_MAX_DISPLAY_DELAY;
    ctx->cuparseinfo.pUserData = avctx;
    ctx->cuparseinfo.pfnSequenceCallback = cuvid_handle_video_sequence;
    ctx->cuparseinfo.pfnDecodePicture = cuvid_handle_picture_decode;
    ctx->cuparseinfo.pfnDisplayPicture = cuvid_handle_picture_display;

    ret = CHECK_CU(ctx->cudl->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        goto error;

    ret = cuvid_test_capabilities(avctx, &ctx->cuparseinfo,
                                  probed_width,
                                  probed_height,
                                  probed_bit_depth, is_yuv422, is_yuv444);
    if (ret < 0)
        goto error;

    ret = CHECK_CU(ctx->cvdl->cuvidCreateVideoParser(&ctx->cuparser, &ctx->cuparseinfo));
    if (ret < 0)
        goto error;

    seq_pkt.payload = ctx->cuparse_ext->raw_seqhdr_data;
    seq_pkt.payload_size = ctx->cuparse_ext->format.seqhdr_data_length;

    if (seq_pkt.payload && seq_pkt.payload_size) {
        ret = CHECK_CU(ctx->cvdl->cuvidParseVideoData(ctx->cuparser, &seq_pkt));
        if (ret < 0)
            goto error;
    }

    ret = CHECK_CU(ctx->cudl->cuCtxPopCurrent(&dummy));
    if (ret < 0)
        goto error;

    ctx->prev_pts = INT64_MIN;

    if (!avctx->pkt_timebase.num || !avctx->pkt_timebase.den)
        av_log(avctx, AV_LOG_WARNING, "Invalid pkt_timebase, passing timestamps as-is.\n");

    return 0;

error:
    cuvid_decode_end(avctx);
    return ret;
}

static void cuvid_flush(AVCodecContext *avctx)
{
    CuvidContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVCUDADeviceContext *device_hwctx = device_ctx->hwctx;
    CUcontext dummy, cuda_ctx = device_hwctx->cuda_ctx;
    CUVIDSOURCEDATAPACKET seq_pkt = { 0 };
    int ret;

    ret = CHECK_CU(ctx->cudl->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        goto error;

    av_fifo_reset2(ctx->frame_queue);

    if (ctx->cudecoder) {
        ctx->cvdl->cuvidDestroyDecoder(ctx->cudecoder);
        ctx->cudecoder = NULL;
    }

    if (ctx->cuparser) {
        ctx->cvdl->cuvidDestroyVideoParser(ctx->cuparser);
        ctx->cuparser = NULL;
    }

    ret = CHECK_CU(ctx->cvdl->cuvidCreateVideoParser(&ctx->cuparser, &ctx->cuparseinfo));
    if (ret < 0)
        goto error;

    seq_pkt.payload = ctx->cuparse_ext->raw_seqhdr_data;
    seq_pkt.payload_size = ctx->cuparse_ext->format.seqhdr_data_length;

    if (seq_pkt.payload && seq_pkt.payload_size) {
        ret = CHECK_CU(ctx->cvdl->cuvidParseVideoData(ctx->cuparser, &seq_pkt));
        if (ret < 0)
            goto error;
    }

    ret = CHECK_CU(ctx->cudl->cuCtxPopCurrent(&dummy));
    if (ret < 0)
        goto error;

    ctx->prev_pts = INT64_MIN;
    ctx->decoder_flushing = 0;

    return;
 error:
    av_log(avctx, AV_LOG_ERROR, "CUDA reinit on flush failed\n");
}

#define OFFSET(x) offsetof(CuvidContext, x)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "deint",    "Set deinterlacing mode", OFFSET(deint_mode), AV_OPT_TYPE_INT,   { .i64 = cudaVideoDeinterlaceMode_Weave    }, cudaVideoDeinterlaceMode_Weave, cudaVideoDeinterlaceMode_Adaptive, VD, .unit = "deint" },
    { "weave",    "Weave deinterlacing (do nothing)",        0, AV_OPT_TYPE_CONST, { .i64 = cudaVideoDeinterlaceMode_Weave    }, 0, 0, VD, .unit = "deint" },
    { "bob",      "Bob deinterlacing",                       0, AV_OPT_TYPE_CONST, { .i64 = cudaVideoDeinterlaceMode_Bob      }, 0, 0, VD, .unit = "deint" },
    { "adaptive", "Adaptive deinterlacing",                  0, AV_OPT_TYPE_CONST, { .i64 = cudaVideoDeinterlaceMode_Adaptive }, 0, 0, VD, .unit = "deint" },
    { "gpu",      "GPU to be used for decoding", OFFSET(cu_gpu), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { "surfaces", "Maximum surfaces to be used for decoding", OFFSET(nb_surfaces), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, INT_MAX, VD | AV_OPT_FLAG_DEPRECATED },
    { "drop_second_field", "Drop second field when deinterlacing", OFFSET(drop_second_field), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VD },
    { "crop",     "Crop (top)x(bottom)x(left)x(right)", OFFSET(crop_expr), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { "resize",   "Resize (width)x(height)", OFFSET(resize_expr), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { NULL }
};

static const AVCodecHWConfigInternal *const cuvid_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_CUDA,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_INTERNAL,
            .device_type = AV_HWDEVICE_TYPE_CUDA
        },
        .hwaccel = NULL,
    },
    NULL
};

#define DEFINE_CUVID_CODEC(x, X, bsf_name) \
    static const AVClass x##_cuvid_class = { \
        .class_name = #x "_cuvid", \
        .option = options, \
        .version = LIBAVUTIL_VERSION_INT, \
    }; \
    const FFCodec ff_##x##_cuvid_decoder = { \
        .p.name         = #x "_cuvid", \
        CODEC_LONG_NAME("Nvidia CUVID " #X " decoder"), \
        .p.type         = AVMEDIA_TYPE_VIDEO, \
        .p.id           = AV_CODEC_ID_##X, \
        .priv_data_size = sizeof(CuvidContext), \
        .p.priv_class   = &x##_cuvid_class, \
        .init           = cuvid_decode_init, \
        .close          = cuvid_decode_end, \
        FF_CODEC_RECEIVE_FRAME_CB(cuvid_output_frame), \
        .flush          = cuvid_flush, \
        .bsfs           = bsf_name, \
        .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE | \
                          FF_CODEC_CAP_SETS_FRAME_PROPS, \
        .hw_configs     = cuvid_hw_configs, \
        .p.wrapper_name = "cuvid", \
    };

#if CONFIG_AV1_CUVID_DECODER && defined(CUVID_HAS_AV1_SUPPORT)
DEFINE_CUVID_CODEC(av1, AV1, NULL)
#endif

#if CONFIG_HEVC_CUVID_DECODER
DEFINE_CUVID_CODEC(hevc, HEVC, "hevc_mp4toannexb")
#endif

#if CONFIG_H264_CUVID_DECODER
DEFINE_CUVID_CODEC(h264, H264, "h264_mp4toannexb")
#endif

#if CONFIG_MJPEG_CUVID_DECODER
DEFINE_CUVID_CODEC(mjpeg, MJPEG, NULL)
#endif

#if CONFIG_MPEG1_CUVID_DECODER
DEFINE_CUVID_CODEC(mpeg1, MPEG1VIDEO, NULL)
#endif

#if CONFIG_MPEG2_CUVID_DECODER
DEFINE_CUVID_CODEC(mpeg2, MPEG2VIDEO, NULL)
#endif

#if CONFIG_MPEG4_CUVID_DECODER
DEFINE_CUVID_CODEC(mpeg4, MPEG4, NULL)
#endif

#if CONFIG_VP8_CUVID_DECODER
DEFINE_CUVID_CODEC(vp8, VP8, NULL)
#endif

#if CONFIG_VP9_CUVID_DECODER
DEFINE_CUVID_CODEC(vp9, VP9, NULL)
#endif

#if CONFIG_VC1_CUVID_DECODER
DEFINE_CUVID_CODEC(vc1, VC1, NULL)
#endif
