/*
 * Copyright (c) 2010, Google, Inc.
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
 * AV1 decoder support via libaom
 */

#include <aom/aom_decoder.h>
#include <aom/aomdx.h>

#include "libavutil/common.h"
#include "libavutil/cpu.h"
#include "libavutil/hdr_dynamic_metadata.h"
#include "libavutil/imgutils.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "itut35.h"
#include "libaom.h"
#include "profiles.h"

typedef struct AV1DecodeContext {
    struct aom_codec_ctx decoder;
} AV1DecodeContext;

static av_cold int aom_init(AVCodecContext *avctx,
                            const struct aom_codec_iface *iface)
{
    AV1DecodeContext *ctx           = avctx->priv_data;
    struct aom_codec_dec_cfg deccfg = {
        .threads = FFMIN(avctx->thread_count ? avctx->thread_count : av_cpu_count(), 16)
    };

    av_log(avctx, AV_LOG_VERBOSE, "%s\n", aom_codec_version_str());
    av_log(avctx, AV_LOG_VERBOSE, "%s\n", aom_codec_build_config());

    if (aom_codec_dec_init(&ctx->decoder, iface, &deccfg, 0) != AOM_CODEC_OK) {
        const char *error = aom_codec_error(&ctx->decoder);
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize decoder: %s\n",
               error);
        return AVERROR(EINVAL);
    }

    return 0;
}

// returns 0 on success, AVERROR_INVALIDDATA otherwise
static int set_pix_fmt(AVCodecContext *avctx, struct aom_image *img)
{
    static const enum AVColorRange color_ranges[] = {
        AVCOL_RANGE_MPEG, AVCOL_RANGE_JPEG
    };
    avctx->color_range = color_ranges[img->range];
    avctx->color_primaries = img->cp;
    avctx->colorspace  = img->mc;
    avctx->color_trc   = img->tc;

    switch (img->fmt) {
    case AOM_IMG_FMT_I420:
    case AOM_IMG_FMT_I42016:
        if (img->bit_depth == 8) {
            avctx->pix_fmt = img->monochrome ?
                             AV_PIX_FMT_GRAY8 : AV_PIX_FMT_YUV420P;
            avctx->profile = AV_PROFILE_AV1_MAIN;
            return 0;
        } else if (img->bit_depth == 10) {
            avctx->pix_fmt = img->monochrome ?
                             AV_PIX_FMT_GRAY10 : AV_PIX_FMT_YUV420P10;
            avctx->profile = AV_PROFILE_AV1_MAIN;
            return 0;
        } else if (img->bit_depth == 12) {
            avctx->pix_fmt = img->monochrome ?
                             AV_PIX_FMT_GRAY12 : AV_PIX_FMT_YUV420P12;
            avctx->profile = AV_PROFILE_AV1_PROFESSIONAL;
            return 0;
        } else {
            return AVERROR_INVALIDDATA;
        }
    case AOM_IMG_FMT_I422:
    case AOM_IMG_FMT_I42216:
        if (img->bit_depth == 8) {
            avctx->pix_fmt = AV_PIX_FMT_YUV422P;
            avctx->profile = AV_PROFILE_AV1_PROFESSIONAL;
            return 0;
        } else if (img->bit_depth == 10) {
            avctx->pix_fmt = AV_PIX_FMT_YUV422P10;
            avctx->profile = AV_PROFILE_AV1_PROFESSIONAL;
            return 0;
        } else if (img->bit_depth == 12) {
            avctx->pix_fmt = AV_PIX_FMT_YUV422P12;
            avctx->profile = AV_PROFILE_AV1_PROFESSIONAL;
            return 0;
        } else {
            return AVERROR_INVALIDDATA;
        }
    case AOM_IMG_FMT_I444:
    case AOM_IMG_FMT_I44416:
        if (img->bit_depth == 8) {
            avctx->pix_fmt = avctx->colorspace == AVCOL_SPC_RGB ?
                             AV_PIX_FMT_GBRP : AV_PIX_FMT_YUV444P;
            avctx->profile = AV_PROFILE_AV1_HIGH;
            return 0;
        } else if (img->bit_depth == 10) {
            avctx->pix_fmt = AV_PIX_FMT_YUV444P10;
            avctx->pix_fmt = avctx->colorspace == AVCOL_SPC_RGB ?
                             AV_PIX_FMT_GBRP10 : AV_PIX_FMT_YUV444P10;
            avctx->profile = AV_PROFILE_AV1_HIGH;
            return 0;
        } else if (img->bit_depth == 12) {
            avctx->pix_fmt = avctx->colorspace == AVCOL_SPC_RGB ?
                             AV_PIX_FMT_GBRP12 : AV_PIX_FMT_YUV444P12;
            avctx->profile = AV_PROFILE_AV1_PROFESSIONAL;
            return 0;
        } else {
            return AVERROR_INVALIDDATA;
        }

    default:
        return AVERROR_INVALIDDATA;
    }
}

static int decode_metadata_itu_t_t35(AVFrame *frame,
                                     const uint8_t *buffer, size_t buffer_size)
{
    if (buffer_size < 6)
        return AVERROR(EINVAL);

    GetByteContext bc;
    bytestream2_init(&bc, buffer, buffer_size);

    const int country_code = bytestream2_get_byteu(&bc);
    const int provider_code = bytestream2_get_be16u(&bc);
    const int provider_oriented_code = bytestream2_get_be16u(&bc);
    const int application_identifier = bytestream2_get_byteu(&bc);

    // See "HDR10+ AV1 Metadata Handling Specification" v1.0.1, Section 2.1.
    if (country_code == ITU_T_T35_COUNTRY_CODE_US
        && provider_code == ITU_T_T35_PROVIDER_CODE_SAMSUNG
        && provider_oriented_code == 0x0001
        && application_identifier == 0x04) {
        // HDR10+
        AVDynamicHDRPlus *hdr_plus = av_dynamic_hdr_plus_create_side_data(frame);
        if (!hdr_plus)
            return AVERROR(ENOMEM);

        int res = av_dynamic_hdr_plus_from_t35(hdr_plus, bc.buffer,
                                               bytestream2_get_bytes_left(&bc));
        if (res < 0)
            return res;
    }

    return 0;
}

static int decode_metadata(AVFrame *frame, const struct aom_image *img)
{
    const size_t num_metadata = aom_img_num_metadata(img);
    for (size_t i = 0; i < num_metadata; ++i) {
        const aom_metadata_t *metadata = aom_img_get_metadata(img, i);
        if (!metadata)
            continue;

        switch (metadata->type) {
        case OBU_METADATA_TYPE_ITUT_T35: {
            int res = decode_metadata_itu_t_t35(frame, metadata->payload, metadata->sz);
            if (res < 0)
                return res;
            break;
        }
        default:
            break;
        }
    }
    return 0;
}

static int aom_decode(AVCodecContext *avctx, AVFrame *picture,
                      int *got_frame, AVPacket *avpkt)
{
    AV1DecodeContext *ctx = avctx->priv_data;
    const void *iter      = NULL;
    struct aom_image *img;
    int ret;

    if (aom_codec_decode(&ctx->decoder, avpkt->data, avpkt->size, NULL) !=
        AOM_CODEC_OK) {
        const char *error  = aom_codec_error(&ctx->decoder);
        const char *detail = aom_codec_error_detail(&ctx->decoder);

        av_log(avctx, AV_LOG_ERROR, "Failed to decode frame: %s\n", error);
        if (detail)
            av_log(avctx, AV_LOG_ERROR, "  Additional information: %s\n",
                   detail);
        return AVERROR_INVALIDDATA;
    }

    if ((img = aom_codec_get_frame(&ctx->decoder, &iter))) {
        if (img->d_w > img->w || img->d_h > img->h) {
            av_log(avctx, AV_LOG_ERROR, "Display dimensions %dx%d exceed storage %dx%d\n",
                   img->d_w, img->d_h, img->w, img->h);
            return AVERROR_EXTERNAL;
        }

        if ((ret = set_pix_fmt(avctx, img)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Unsupported output colorspace (%d) / bit_depth (%d)\n",
                   img->fmt, img->bit_depth);
            return ret;
        }

        if ((int)img->d_w != avctx->width || (int)img->d_h != avctx->height) {
            av_log(avctx, AV_LOG_INFO, "dimension change! %dx%d -> %dx%d\n",
                   avctx->width, avctx->height, img->d_w, img->d_h);
            ret = ff_set_dimensions(avctx, img->d_w, img->d_h);
            if (ret < 0)
                return ret;
        }
        if ((ret = ff_get_buffer(avctx, picture, 0)) < 0)
            return ret;

#ifdef AOM_CTRL_AOMD_GET_FRAME_FLAGS
        {
            aom_codec_frame_flags_t flags;
            ret = aom_codec_control(&ctx->decoder, AOMD_GET_FRAME_FLAGS, &flags);
            if (ret == AOM_CODEC_OK) {
                if (flags & AOM_FRAME_IS_KEY)
                    picture->flags |= AV_FRAME_FLAG_KEY;
                else
                    picture->flags &= ~AV_FRAME_FLAG_KEY;
                if (flags & (AOM_FRAME_IS_KEY | AOM_FRAME_IS_INTRAONLY))
                    picture->pict_type = AV_PICTURE_TYPE_I;
                else if (flags & AOM_FRAME_IS_SWITCH)
                    picture->pict_type = AV_PICTURE_TYPE_SP;
                else
                    picture->pict_type = AV_PICTURE_TYPE_P;
            }
        }
#endif

        av_reduce(&picture->sample_aspect_ratio.num,
                  &picture->sample_aspect_ratio.den,
                  picture->height * img->r_w,
                  picture->width * img->r_h,
                  INT_MAX);
        ff_set_sar(avctx, picture->sample_aspect_ratio);

        if ((img->fmt & AOM_IMG_FMT_HIGHBITDEPTH) && img->bit_depth == 8)
            ff_aom_image_copy_16_to_8(picture, img);
        else {
            const uint8_t *planes[4] = { img->planes[0], img->planes[1], img->planes[2] };
            const int      stride[4] = { img->stride[0], img->stride[1], img->stride[2] };

            av_image_copy(picture->data, picture->linesize, planes,
                          stride, avctx->pix_fmt, img->d_w, img->d_h);
        }
        ret = decode_metadata(picture, img);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to decode metadata\n");
            return ret;
        }
        *got_frame = 1;
    }
    return avpkt->size;
}

static av_cold int aom_free(AVCodecContext *avctx)
{
    AV1DecodeContext *ctx = avctx->priv_data;
    aom_codec_destroy(&ctx->decoder);
    return 0;
}

static av_cold int av1_init(AVCodecContext *avctx)
{
    return aom_init(avctx, aom_codec_av1_dx());
}

const FFCodec ff_libaom_av1_decoder = {
    .p.name         = "libaom-av1",
    CODEC_LONG_NAME("libaom AV1"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_AV1,
    .priv_data_size = sizeof(AV1DecodeContext),
    .init           = av1_init,
    .close          = aom_free,
    FF_CODEC_DECODE_CB(aom_decode),
    .p.capabilities = AV_CODEC_CAP_OTHER_THREADS | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE |
                      FF_CODEC_CAP_AUTO_THREADS,
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_av1_profiles),
    .p.wrapper_name = "libaom",
};
