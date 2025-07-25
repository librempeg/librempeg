/*
 * TIFF image encoder
 * Copyright (c) 2007 Bartlomiej Wolowiec
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
 * TIFF image encoder
 * @author Bartlomiej Wolowiec
 */

#include "config.h"
#if CONFIG_ZLIB
#include <zlib.h>
#endif

#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"
#include "lzw.h"
#include "rle.h"
#include "tiff.h"
#include "tiff_common.h"
#include "version.h"

#define TIFF_MAX_ENTRY 32

/** sizes of various TIFF field types (string size = 1)*/
static const uint8_t type_sizes2[14] = {
    0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8, 4
};

typedef struct TiffEncoderContext {
    AVClass *class;                         ///< for private options
    AVCodecContext *avctx;

    int width;                              ///< picture width
    int height;                             ///< picture height
    unsigned int bpp;                       ///< bits per pixel
    int compr;                              ///< compression level
    int bpp_tab_size;                       ///< bpp_tab size
    enum TiffPhotometric photometric_interpretation;  ///< photometric interpretation
    int strips;                             ///< number of strips
    uint32_t *strip_sizes;
    unsigned int strip_sizes_size;
    uint32_t *strip_offsets;
    unsigned int strip_offsets_size;
    uint8_t *yuv_line;
    unsigned int yuv_line_size;
    int rps;                                ///< row per strip
    uint8_t entries[TIFF_MAX_ENTRY * 12];   ///< entries in header
    int num_entries;                        ///< number of entries
    uint8_t **buf;                          ///< actual position in buffer
    uint8_t *buf_start;                     ///< pointer to first byte in buffer
    int buf_size;                           ///< buffer size
    uint16_t subsampling[2];                ///< YUV subsampling factors
    struct LZWEncodeState *lzws;            ///< LZW encode state
    uint32_t dpi;                           ///< image resolution in DPI
} TiffEncoderContext;

/**
 * Check free space in buffer.
 *
 * @param s Tiff context
 * @param need Needed bytes
 * @return 0 - ok, 1 - no free space
 */
static inline int check_size(TiffEncoderContext *s, uint64_t need)
{
    if (s->buf_size < *s->buf - s->buf_start + need) {
        *s->buf = s->buf_start + s->buf_size + 1;
        av_log(s->avctx, AV_LOG_ERROR, "Buffer is too small\n");
        return 1;
    }
    return 0;
}

/**
 * Put n values to buffer.
 *
 * @param p pointer to pointer to output buffer
 * @param n number of values
 * @param val pointer to values
 * @param type type of values
 * @param flip = 0 - normal copy, >0 - flip
 */
static void tnput(uint8_t **p, int n, const uint8_t *val, enum TiffTypes type,
                  int flip)
{
    int i;
#if HAVE_BIGENDIAN
    flip ^= ((int[]) { 0, 0, 0, 1, 3, 3 })[type];
#endif
    for (i = 0; i < n * type_sizes2[type]; i++)
        *(*p)++ = val[i ^ flip];
}

/**
 * Add entry to directory in tiff header.
 *
 * @param s Tiff context
 * @param tag tag that identifies the entry
 * @param type entry type
 * @param count the number of values
 * @param ptr_val pointer to values
 */
static int add_entry(TiffEncoderContext *s, enum TiffTags tag,
                     enum TiffTypes type, int count, const void *ptr_val)
{
    uint8_t *entries_ptr = s->entries + 12 * s->num_entries;

    av_assert0(s->num_entries < TIFF_MAX_ENTRY);

    bytestream_put_le16(&entries_ptr, tag);
    bytestream_put_le16(&entries_ptr, type);
    bytestream_put_le32(&entries_ptr, count);

    if (type_sizes[type] * (int64_t)count <= 4) {
        tnput(&entries_ptr, count, ptr_val, type, 0);
    } else {
        bytestream_put_le32(&entries_ptr, *s->buf - s->buf_start);
        if (check_size(s, count * (int64_t)type_sizes2[type]))
            return AVERROR_INVALIDDATA;
        tnput(s->buf, count, ptr_val, type, 0);
    }

    s->num_entries++;
    return 0;
}

static int add_entry1(TiffEncoderContext *s,
                      enum TiffTags tag, enum TiffTypes type, int val)
{
    uint16_t w  = val;
    uint32_t dw = val;
    return add_entry(s, tag, type, 1,
                     type == TIFF_SHORT ? (void *)&w : (void *)&dw);
}

/**
 * Encode one strip in tiff file.
 *
 * @param s Tiff context
 * @param src input buffer
 * @param dst output buffer
 * @param n size of input buffer
 * @param compr compression method
 * @return number of output bytes. If an output error is encountered, a negative
 * value corresponding to an AVERROR error code is returned.
 */
static int encode_strip(TiffEncoderContext *s, const int8_t *src,
                        uint8_t *dst, int n, int compr)
{
    switch (compr) {
#if CONFIG_ZLIB
    case TIFF_DEFLATE:
    case TIFF_ADOBE_DEFLATE:
    {
        unsigned long zlen = s->buf_size - (*s->buf - s->buf_start);
        if (compress(dst, &zlen, src, n) != Z_OK) {
            av_log(s->avctx, AV_LOG_ERROR, "Compressing failed\n");
            return AVERROR_EXTERNAL;
        }
        return zlen;
    }
#endif
    case TIFF_RAW:
        if (check_size(s, n))
            return AVERROR(EINVAL);
        memcpy(dst, src, n);
        return n;
    case TIFF_PACKBITS:
        return ff_rle_encode(dst, s->buf_size - (*s->buf - s->buf_start),
                             src, 1, n, 2, 0xff, -1, 0);
    case TIFF_LZW:
        return ff_lzw_encode(s->lzws, src, n);
    default:
        av_log(s->avctx, AV_LOG_ERROR, "Unsupported compression method: %d\n",
               compr);
        return AVERROR(EINVAL);
    }
}

static void pack_yuv(TiffEncoderContext *s, const AVFrame *p,
                     uint8_t *dst, int lnum)
{
    int i, j, k;
    int w       = (s->width - 1) / s->subsampling[0] + 1;
    const uint8_t *pu = &p->data[1][lnum / s->subsampling[1] * p->linesize[1]];
    const uint8_t *pv = &p->data[2][lnum / s->subsampling[1] * p->linesize[2]];
    if (s->width % s->subsampling[0] || s->height % s->subsampling[1]) {
        for (i = 0; i < w; i++) {
            for (j = 0; j < s->subsampling[1]; j++)
                for (k = 0; k < s->subsampling[0]; k++)
                    *dst++ = p->data[0][FFMIN(lnum + j, s->height-1) * p->linesize[0] +
                                        FFMIN(i * s->subsampling[0] + k, s->width-1)];
            *dst++ = *pu++;
            *dst++ = *pv++;
        }
    }else{
        for (i = 0; i < w; i++) {
            for (j = 0; j < s->subsampling[1]; j++)
                for (k = 0; k < s->subsampling[0]; k++)
                    *dst++ = p->data[0][(lnum + j) * p->linesize[0] +
                                        i * s->subsampling[0] + k];
            *dst++ = *pu++;
            *dst++ = *pv++;
        }
    }
}

#define ADD_ENTRY(s, tag, type, count, ptr_val)         \
    do {                                                \
        ret = add_entry(s, tag, type, count, ptr_val);  \
        if (ret < 0)                                    \
            goto fail;                                  \
    } while (0)

#define ADD_ENTRY1(s, tag, type, val)           \
    do {                                        \
        ret = add_entry1(s, tag, type, val);    \
        if (ret < 0)                            \
            goto fail;                          \
    } while (0)

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *pict, int *got_packet)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(avctx->pix_fmt);
    TiffEncoderContext *s = avctx->priv_data;
    const AVFrame *const p = pict;
    int i;
    uint8_t *ptr;
    uint8_t *offset;
    uint32_t strips;
    int bytes_per_row;
    uint32_t res[2] = { s->dpi, 1 };    // image resolution (72/1)
    uint16_t bpp_tab[4];
    int ret = 0;
    int is_yuv = 0, alpha = 0;
    int shift_h, shift_v;
    int packet_size;

    s->width          = avctx->width;
    s->height         = avctx->height;
    s->subsampling[0] = 1;
    s->subsampling[1] = 1;

    if (!desc)
        return AVERROR(EINVAL);

    avctx->bits_per_coded_sample =
    s->bpp          = av_get_bits_per_pixel(desc);
    s->bpp_tab_size = desc->nb_components;

    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_RGBA64LE:
    case AV_PIX_FMT_RGBA:
        alpha = 1;
    case AV_PIX_FMT_RGB48LE:
    case AV_PIX_FMT_RGB24:
        s->photometric_interpretation = TIFF_PHOTOMETRIC_RGB;
        break;
    case AV_PIX_FMT_GRAY8:
        avctx->bits_per_coded_sample = 0x28;
    case AV_PIX_FMT_GRAY8A:
    case AV_PIX_FMT_YA16LE:
        alpha = avctx->pix_fmt == AV_PIX_FMT_GRAY8A || avctx->pix_fmt == AV_PIX_FMT_YA16LE;
    case AV_PIX_FMT_GRAY16LE:
    case AV_PIX_FMT_MONOBLACK:
        s->photometric_interpretation = TIFF_PHOTOMETRIC_BLACK_IS_ZERO;
        break;
    case AV_PIX_FMT_PAL8:
        s->photometric_interpretation = TIFF_PHOTOMETRIC_PALETTE;
        break;
    case AV_PIX_FMT_MONOWHITE:
        s->photometric_interpretation = TIFF_PHOTOMETRIC_WHITE_IS_ZERO;
        break;
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUV440P:
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUV410P:
    case AV_PIX_FMT_YUV411P:
        av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &shift_h, &shift_v);
        s->photometric_interpretation = TIFF_PHOTOMETRIC_YCBCR;
        s->subsampling[0]             = 1 << shift_h;
        s->subsampling[1]             = 1 << shift_v;
        is_yuv                        = 1;
        break;
    default:
        av_log(s->avctx, AV_LOG_ERROR,
               "This colors format is not supported\n");
        return AVERROR(EINVAL);
    }

    for (i = 0; i < s->bpp_tab_size; i++)
        bpp_tab[i] = desc->comp[i].depth;

    if (s->compr == TIFF_DEFLATE       ||
        s->compr == TIFF_ADOBE_DEFLATE ||
        s->compr == TIFF_LZW)
        // best choice for DEFLATE
        s->rps = s->height;
    else
        // suggest size of strip
        s->rps = FFMAX(8192 / (((s->width * s->bpp) >> 3) + 1), 1);
    // round rps up
    s->rps = ((s->rps - 1) / s->subsampling[1] + 1) * s->subsampling[1];

    strips = (s->height - 1) / s->rps + 1;

    bytes_per_row = (((s->width - 1) / s->subsampling[0] + 1) * s->bpp *
                     s->subsampling[0] * s->subsampling[1] + 7) >> 3;
    packet_size = avctx->height * bytes_per_row * 2 +
                  avctx->height * 4 + FF_INPUT_BUFFER_MIN_SIZE;

    if ((ret = ff_alloc_packet(avctx, pkt, packet_size)) < 0)
        return ret;
    ptr          = pkt->data;
    s->buf_start = pkt->data;
    s->buf       = &ptr;
    s->buf_size  = pkt->size;

    if (check_size(s, 8)) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // write header
    bytestream_put_le16(&ptr, 0x4949);
    bytestream_put_le16(&ptr, 42);

    offset = ptr;
    bytestream_put_le32(&ptr, 0);

    if (strips > INT_MAX / FFMAX(sizeof(s->strip_sizes[0]), sizeof(s->strip_offsets[0]))) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    av_fast_padded_mallocz(&s->strip_sizes  , &s->strip_sizes_size  , sizeof(s->strip_sizes  [0]) * strips);
    av_fast_padded_mallocz(&s->strip_offsets, &s->strip_offsets_size, sizeof(s->strip_offsets[0]) * strips);

    if (!s->strip_sizes || !s->strip_offsets) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    if (is_yuv) {
        av_fast_padded_malloc(&s->yuv_line, &s->yuv_line_size, bytes_per_row);
        if (s->yuv_line == NULL) {
            av_log(s->avctx, AV_LOG_ERROR, "Not enough memory\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

#if CONFIG_ZLIB
    if (s->compr == TIFF_DEFLATE || s->compr == TIFF_ADOBE_DEFLATE) {
        uint8_t *zbuf;
        int zlen, zn;
        int j;

        zlen = bytes_per_row * s->rps;
        zbuf = av_malloc(zlen);
        if (!zbuf) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        s->strip_offsets[0] = ptr - pkt->data;
        zn               = 0;
        for (j = 0; j < s->rps; j++) {
            if (is_yuv) {
                pack_yuv(s, p, s->yuv_line, j);
                memcpy(zbuf + zn, s->yuv_line, bytes_per_row);
                j += s->subsampling[1] - 1;
            } else
                memcpy(zbuf + j * bytes_per_row,
                       p->data[0] + j * p->linesize[0], bytes_per_row);
            zn += bytes_per_row;
        }
        ret = encode_strip(s, zbuf, ptr, zn, s->compr);
        av_free(zbuf);
        if (ret < 0) {
            av_log(s->avctx, AV_LOG_ERROR, "Encode strip failed\n");
            goto fail;
        }
        ptr           += ret;
        s->strip_sizes[0] = ptr - pkt->data - s->strip_offsets[0];
    } else
#endif
    {
    if (s->compr == TIFF_LZW) {
        s->lzws = av_malloc(ff_lzw_encode_state_size);
        if (!s->lzws) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }
    for (i = 0; i < s->height; i++) {
        if (s->strip_sizes[i / s->rps] == 0) {
            if (s->compr == TIFF_LZW) {
                ff_lzw_encode_init(s->lzws, ptr,
                                   s->buf_size - (*s->buf - s->buf_start),
                                   12, FF_LZW_TIFF, 0);
            }
            s->strip_offsets[i / s->rps] = ptr - pkt->data;
        }
        if (is_yuv) {
            pack_yuv(s, p, s->yuv_line, i);
            ret = encode_strip(s, s->yuv_line, ptr, bytes_per_row, s->compr);
            i  += s->subsampling[1] - 1;
        } else
            ret = encode_strip(s, p->data[0] + i * p->linesize[0],
                               ptr, bytes_per_row, s->compr);
        if (ret < 0) {
            av_log(s->avctx, AV_LOG_ERROR, "Encode strip failed\n");
            goto fail;
        }
        s->strip_sizes[i / s->rps] += ret;
        ptr                     += ret;
        if (s->compr == TIFF_LZW &&
            (i == s->height - 1 || i % s->rps == s->rps - 1)) {
            ret = ff_lzw_encode_flush(s->lzws);
            s->strip_sizes[(i / s->rps)] += ret;
            ptr                          += ret;
        }
    }
    if (s->compr == TIFF_LZW)
        av_freep(&s->lzws);
    }

    s->num_entries = 0;

    ADD_ENTRY1(s, TIFF_SUBFILE, TIFF_LONG, 0);
    ADD_ENTRY1(s, TIFF_WIDTH,   TIFF_LONG, s->width);
    ADD_ENTRY1(s, TIFF_HEIGHT,  TIFF_LONG, s->height);

    if (s->bpp_tab_size)
        ADD_ENTRY(s, TIFF_BPP, TIFF_SHORT, s->bpp_tab_size, bpp_tab);

    ADD_ENTRY1(s, TIFF_COMPR,       TIFF_SHORT, s->compr);
    ADD_ENTRY1(s, TIFF_PHOTOMETRIC, TIFF_SHORT, s->photometric_interpretation);
    ADD_ENTRY(s,  TIFF_STRIP_OFFS,  TIFF_LONG,  strips, s->strip_offsets);

    if (s->bpp_tab_size)
        ADD_ENTRY1(s, TIFF_SAMPLES_PER_PIXEL, TIFF_SHORT, s->bpp_tab_size);

    ADD_ENTRY1(s, TIFF_ROWSPERSTRIP, TIFF_LONG,     s->rps);
    ADD_ENTRY(s,  TIFF_STRIP_SIZE,   TIFF_LONG,     strips, s->strip_sizes);
    ADD_ENTRY(s,  TIFF_XRES,         TIFF_RATIONAL, 1,      res);
    if (avctx->sample_aspect_ratio.num > 0 &&
        avctx->sample_aspect_ratio.den > 0) {
        AVRational y = av_mul_q(av_make_q(s->dpi, 1),
                                avctx->sample_aspect_ratio);
        res[0] = y.num;
        res[1] = y.den;
    }
    ADD_ENTRY(s,  TIFF_YRES,         TIFF_RATIONAL, 1,      res);
    ADD_ENTRY1(s, TIFF_RES_UNIT,     TIFF_SHORT,    2);

    if (!(avctx->flags & AV_CODEC_FLAG_BITEXACT))
        ADD_ENTRY(s, TIFF_SOFTWARE_NAME, TIFF_STRING,
                  strlen(LIBAVCODEC_IDENT) + 1, LIBAVCODEC_IDENT);

    if (avctx->pix_fmt == AV_PIX_FMT_PAL8) {
        uint16_t pal[256 * 3];
        for (i = 0; i < 256; i++) {
            uint32_t rgb = *(uint32_t *) (p->data[1] + i * 4);
            pal[i]       = ((rgb >> 16) & 0xff) * 257;
            pal[i + 256] = ((rgb >>  8) & 0xff) * 257;
            pal[i + 512] =  (rgb        & 0xff) * 257;
        }
        ADD_ENTRY(s, TIFF_PAL, TIFF_SHORT, 256 * 3, pal);
    }
    if (alpha)
        ADD_ENTRY1(s,TIFF_EXTRASAMPLES,      TIFF_SHORT,            2);
    if (is_yuv) {
        /** according to CCIR Recommendation 601.1 */
        uint32_t refbw[12] = { 15, 1, 235, 1, 128, 1, 240, 1, 128, 1, 240, 1 };
        ADD_ENTRY(s, TIFF_YCBCR_SUBSAMPLING, TIFF_SHORT,    2, s->subsampling);
        if (avctx->chroma_sample_location == AVCHROMA_LOC_TOPLEFT)
            ADD_ENTRY1(s, TIFF_YCBCR_POSITIONING, TIFF_SHORT, 2);
        ADD_ENTRY(s, TIFF_REFERENCE_BW,      TIFF_RATIONAL, 6, refbw);
    }
    // write offset to dir
    bytestream_put_le32(&offset, ptr - pkt->data);

    if (check_size(s, 6 + s->num_entries * 12)) {
        ret = AVERROR(EINVAL);
        goto fail;
    }
    bytestream_put_le16(&ptr, s->num_entries);  // write tag count
    bytestream_put_buffer(&ptr, s->entries, s->num_entries * 12);
    bytestream_put_le32(&ptr, 0);

    pkt->size   = ptr - pkt->data;
    *got_packet = 1;

fail:
    return ret < 0 ? ret : 0;
}

static av_cold int encode_init(AVCodecContext *avctx)
{
    TiffEncoderContext *s = avctx->priv_data;

#if !CONFIG_ZLIB
    if (s->compr == TIFF_DEFLATE) {
        av_log(avctx, AV_LOG_ERROR,
               "Deflate compression needs zlib compiled in\n");
        return AVERROR(ENOSYS);
    }
#endif

    s->avctx = avctx;

    return 0;
}

static av_cold int encode_close(AVCodecContext *avctx)
{
    TiffEncoderContext *s = avctx->priv_data;

    av_freep(&s->strip_sizes);
    av_freep(&s->strip_offsets);
    av_freep(&s->yuv_line);

    return 0;
}

#define OFFSET(x) offsetof(TiffEncoderContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    {"dpi", "set the image resolution (in dpi)", OFFSET(dpi), AV_OPT_TYPE_INT, {.i64 = 72}, 1, 0x10000, AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_ENCODING_PARAM},
    { "compression_algo", NULL, OFFSET(compr), AV_OPT_TYPE_INT,   { .i64 = TIFF_PACKBITS }, TIFF_RAW, TIFF_DEFLATE, VE, .unit = "compression_algo" },
    { "packbits",         NULL, 0,             AV_OPT_TYPE_CONST, { .i64 = TIFF_PACKBITS }, 0,        0,            VE, .unit = "compression_algo" },
    { "raw",              NULL, 0,             AV_OPT_TYPE_CONST, { .i64 = TIFF_RAW      }, 0,        0,            VE, .unit = "compression_algo" },
    { "lzw",              NULL, 0,             AV_OPT_TYPE_CONST, { .i64 = TIFF_LZW      }, 0,        0,            VE, .unit = "compression_algo" },
    { "deflate",          NULL, 0,             AV_OPT_TYPE_CONST, { .i64 = TIFF_DEFLATE  }, 0,        0,            VE, .unit = "compression_algo" },
    { NULL },
};

static const AVClass tiffenc_class = {
    .class_name = "TIFF encoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_tiff_encoder = {
    .p.name         = "tiff",
    CODEC_LONG_NAME("TIFF image"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_TIFF,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(TiffEncoderContext),
    .init           = encode_init,
    .close          = encode_close,
    FF_CODEC_ENCODE_CB(encode_frame),
    CODEC_PIXFMTS(
        AV_PIX_FMT_RGB24, AV_PIX_FMT_RGB48LE, AV_PIX_FMT_PAL8,
        AV_PIX_FMT_RGBA, AV_PIX_FMT_RGBA64LE,
        AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY8A, AV_PIX_FMT_GRAY16LE, AV_PIX_FMT_YA16LE,
        AV_PIX_FMT_MONOBLACK, AV_PIX_FMT_MONOWHITE,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P),
    .color_ranges   = AVCOL_RANGE_MPEG,
    .p.priv_class   = &tiffenc_class,
};
