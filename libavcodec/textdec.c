/*
 * Copyright (c) 2012 Clément Bœsch
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
 * Raw subtitles decoder
 */

#include "config_components.h"

#include "avcodec.h"
#include "ass.h"
#include "codec_internal.h"
#include "libavutil/attributes.h"
#include "libavutil/bprint.h"
#include "libavutil/opt.h"

typedef struct {
    AVClass *class;
    const char *linebreaks;
    int keep_ass_markup;
    int readorder;
} TextContext;

#define OFFSET(x) offsetof(TextContext, x)
#define SD AV_OPT_FLAG_SUBTITLE_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "keep_ass_markup", "Set if ASS tags must be escaped", OFFSET(keep_ass_markup), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, .flags=SD },
    { NULL }
};

static int text_decode_frame(AVCodecContext *avctx, AVSubtitle *sub,
                             int *got_sub_ptr, const AVPacket *avpkt)
{
    int ret = 0;
    AVBPrint buf;
    const char *ptr = avpkt->data;
    TextContext *text = avctx->priv_data;

    av_bprint_init(&buf, 0, AV_BPRINT_SIZE_UNLIMITED);
    if (ptr && avpkt->size > 0 && *ptr) {
        ff_ass_bprint_text_event(&buf, ptr, avpkt->size, text->linebreaks, text->keep_ass_markup);
        ret = ff_ass_add_rect(sub, buf.str, text->readorder++, 0, NULL, NULL);
    }
    av_bprint_finalize(&buf, NULL);
    if (ret < 0)
        return ret;
    *got_sub_ptr = sub->num_rects > 0;
    return avpkt->size;
}

static av_cold void text_flush(AVCodecContext *avctx)
{
    TextContext *text = avctx->priv_data;
    if (!(avctx->flags2 & AV_CODEC_FLAG2_RO_FLUSH_NOOP))
        text->readorder = 0;
}

static const AVClass textsub_decoder_class = {
    .class_name = "text/vplayer/stl/pjs/subviewer1 decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_TEXT_DECODER
const FFCodec ff_text_decoder = {
    .p.name         = "text",
    CODEC_LONG_NAME("Raw text subtitle"),
    .priv_data_size = sizeof(TextContext),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_TEXT,
    FF_CODEC_DECODE_SUB_CB(text_decode_frame),
    .init           = ff_ass_subtitle_header_default,
    .p.priv_class   = &textsub_decoder_class,
    .flush          = text_flush,
};
#endif

#if CONFIG_VPLAYER_DECODER || CONFIG_PJS_DECODER || CONFIG_SUBVIEWER1_DECODER || CONFIG_STL_DECODER

static av_cold int linebreak_init(AVCodecContext *avctx)
{
    TextContext *text = avctx->priv_data;
    text->linebreaks = "|";
    return ff_ass_subtitle_header_default(avctx);
}

#if CONFIG_VPLAYER_DECODER
const FFCodec ff_vplayer_decoder = {
    .p.name         = "vplayer",
    CODEC_LONG_NAME("VPlayer subtitle"),
    .priv_data_size = sizeof(TextContext),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_VPLAYER,
    FF_CODEC_DECODE_SUB_CB(text_decode_frame),
    .init           = linebreak_init,
    .p.priv_class   = &textsub_decoder_class,
    .flush          = text_flush,
};
#endif

#if CONFIG_STL_DECODER
const FFCodec ff_stl_decoder = {
    .p.name         = "stl",
    CODEC_LONG_NAME("Spruce subtitle format"),
    .priv_data_size = sizeof(TextContext),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_STL,
    FF_CODEC_DECODE_SUB_CB(text_decode_frame),
    .init           = linebreak_init,
    .p.priv_class   = &textsub_decoder_class,
    .flush          = text_flush,
};
#endif

#if CONFIG_PJS_DECODER
const FFCodec ff_pjs_decoder = {
    .p.name         = "pjs",
    CODEC_LONG_NAME("PJS subtitle"),
    .priv_data_size = sizeof(TextContext),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_PJS,
    FF_CODEC_DECODE_SUB_CB(text_decode_frame),
    .init           = linebreak_init,
    .p.priv_class   = &textsub_decoder_class,
    .flush          = text_flush,
};
#endif

#if CONFIG_SUBVIEWER1_DECODER
const FFCodec ff_subviewer1_decoder = {
    .p.name         = "subviewer1",
    CODEC_LONG_NAME("SubViewer1 subtitle"),
    .priv_data_size = sizeof(TextContext),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_SUBVIEWER1,
    FF_CODEC_DECODE_SUB_CB(text_decode_frame),
    .init           = linebreak_init,
    .p.priv_class   = &textsub_decoder_class,
    .flush          = text_flush,
};
#endif

#endif /* text subtitles with '|' line break */
