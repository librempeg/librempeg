/*
 * MJPEG decoder
 * Copyright (c) 2000, 2001 Fabrice Bellard
 * Copyright (c) 2003 Alex Beregszaszi
 * Copyright (c) 2003-2004 Michael Niedermayer
 *
 * Support for external huffman table, various fixes (AVID workaround),
 * aspecting, new decode_frame mechanism and apple mjpeg-b support
 *                                  by Alex Beregszaszi
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
 * MJPEG decoder.
 */

#include "config_components.h"

#include "libavutil/display.h"
#include "libavutil/emms.h"
#include "libavutil/imgutils.h"
#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "blockdsp.h"
#include "codec_internal.h"
#include "copy_block.h"
#include "decode.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "idctdsp.h"
#include "internal.h"
#include "jpegtables.h"
#include "mjpeg.h"
#include "mjpegdec.h"
#include "jpeglsdec.h"
#include "profiles.h"
#include "put_bits.h"
#include "thread.h"
#include "exif.h"
#include "bytestream.h"
#include "tiff_common.h"

static int init_default_huffman_tables(MJpegDecodeContext *s)
{
    static const struct {
        int class;
        int index;
        const uint8_t *bits;
        const uint8_t *values;
        int length;
    } ht[] = {
        { 0, 0, ff_mjpeg_bits_dc_luminance,
                ff_mjpeg_val_dc, 12 },
        { 0, 1, ff_mjpeg_bits_dc_chrominance,
                ff_mjpeg_val_dc, 12 },
        { 1, 0, ff_mjpeg_bits_ac_luminance,
                ff_mjpeg_val_ac_luminance,   162 },
        { 1, 1, ff_mjpeg_bits_ac_chrominance,
                ff_mjpeg_val_ac_chrominance, 162 },
        { 2, 0, ff_mjpeg_bits_ac_luminance,
                ff_mjpeg_val_ac_luminance,   162 },
        { 2, 1, ff_mjpeg_bits_ac_chrominance,
                ff_mjpeg_val_ac_chrominance, 162 },
    };
    int i, ret;

    for (i = 0; i < FF_ARRAY_ELEMS(ht); i++) {
        ff_vlc_free(&s->vlcs[ht[i].class][ht[i].index]);
        ret = ff_mjpeg_build_vlc(&s->vlcs[ht[i].class][ht[i].index],
                                 ht[i].bits, ht[i].values,
                                 ht[i].class == 1, s->avctx);
        if (ret < 0)
            return ret;

        if (ht[i].class < 2) {
            memcpy(s->raw_huffman_lengths[ht[i].class][ht[i].index],
                   ht[i].bits + 1, 16);
            memcpy(s->raw_huffman_values[ht[i].class][ht[i].index],
                   ht[i].values, ht[i].length);
        }
    }

    return 0;
}

static void parse_avid(MJpegDecodeContext *s, uint8_t *buf, int len)
{
    s->buggy_avid = 1;
    if (len > 14 && buf[12] == 1) /* 1 - NTSC */
        s->interlace_polarity = 1;
    if (len > 14 && buf[12] == 2) /* 2 - PAL */
        s->interlace_polarity = 0;
    if (s->avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(s->avctx, AV_LOG_INFO, "AVID: len:%d %d\n", len, len > 14 ? buf[12] : -1);
}

static void init_idct(AVCodecContext *avctx)
{
    MJpegDecodeContext *s = avctx->priv_data;

    ff_idctdsp_init(&s->idsp, avctx);
    ff_permute_scantable(s->permutated_scantable, ff_zigzag_direct,
                         s->idsp.idct_permutation);
}

av_cold int ff_mjpeg_decode_init(AVCodecContext *avctx)
{
    MJpegDecodeContext *s = avctx->priv_data;
    int ret;

    if (!s->picture_ptr) {
        s->picture = av_frame_alloc();
        if (!s->picture)
            return AVERROR(ENOMEM);
        s->picture_ptr = s->picture;
    }

    s->avctx = avctx;
    ff_blockdsp_init(&s->bdsp);
    ff_hpeldsp_init(&s->hdsp, avctx->flags);
    init_idct(avctx);
    s->buffer_size   = 0;
    s->buffer        = NULL;
    s->start_code    = -1;
    s->first_picture = 1;
    s->got_picture   = 0;
    s->orig_height    = avctx->coded_height;
    avctx->chroma_sample_location = AVCHROMA_LOC_CENTER;
    avctx->colorspace = AVCOL_SPC_BT470BG;
    s->hwaccel_pix_fmt = s->hwaccel_sw_pix_fmt = AV_PIX_FMT_NONE;

    if ((ret = init_default_huffman_tables(s)) < 0)
        return ret;

    if (s->extern_huff) {
        av_log(avctx, AV_LOG_INFO, "using external huffman table\n");
        if ((ret = init_get_bits(&s->gb, avctx->extradata, avctx->extradata_size * 8)) < 0)
            return ret;
        if (ff_mjpeg_decode_dht(s)) {
            av_log(avctx, AV_LOG_ERROR,
                   "error using external huffman table, switching back to internal\n");
            if ((ret = init_default_huffman_tables(s)) < 0)
                return ret;
        }
    }
    if (avctx->field_order == AV_FIELD_BB) { /* quicktime icefloe 019 */
        s->interlace_polarity = 1;           /* bottom field first */
        av_log(avctx, AV_LOG_DEBUG, "bottom field first\n");
    } else if (avctx->field_order == AV_FIELD_UNKNOWN) {
        if (avctx->codec_tag == AV_RL32("MJPG"))
            s->interlace_polarity = 1;
    }

    if (avctx->codec_id == AV_CODEC_ID_SMVJPEG) {
        if (avctx->extradata_size >= 4)
            s->smv_frames_per_jpeg = AV_RL32(avctx->extradata);

        if (s->smv_frames_per_jpeg <= 0) {
            av_log(avctx, AV_LOG_ERROR, "Invalid number of frames per jpeg.\n");
            return AVERROR_INVALIDDATA;
        }

        s->smv_frame = av_frame_alloc();
        if (!s->smv_frame)
            return AVERROR(ENOMEM);
    } else if (avctx->extradata_size > 8
        && AV_RL32(avctx->extradata) == 0x2C
        && AV_RL32(avctx->extradata+4) == 0x18) {
        parse_avid(s, avctx->extradata, avctx->extradata_size);
    }

    if (avctx->codec->id == AV_CODEC_ID_AMV)
        s->flipped = 1;

    return 0;
}


/* quantize tables */
int ff_mjpeg_decode_dqt(MJpegDecodeContext *s)
{
    int len, index, i;

    len = get_bits(&s->gb, 16) - 2;

    if (8*len > get_bits_left(&s->gb)) {
        av_log(s->avctx, AV_LOG_ERROR, "dqt: len %d is too large\n", len);
        return AVERROR_INVALIDDATA;
    }

    while (len >= 65) {
        int pr = get_bits(&s->gb, 4);
        if (pr > 1) {
            av_log(s->avctx, AV_LOG_ERROR, "dqt: invalid precision\n");
            return AVERROR_INVALIDDATA;
        }
        index = get_bits(&s->gb, 4);
        if (index >= 4)
            return -1;
        av_log(s->avctx, AV_LOG_DEBUG, "index=%d\n", index);
        /* read quant table */
        for (i = 0; i < 64; i++) {
            s->quant_matrixes[index][i] = get_bits(&s->gb, pr ? 16 : 8);
            if (s->quant_matrixes[index][i] == 0) {
                int log_level = s->avctx->err_recognition & AV_EF_EXPLODE ? AV_LOG_ERROR : AV_LOG_WARNING;
                av_log(s->avctx, log_level, "dqt: 0 quant value\n");
                if (s->avctx->err_recognition & AV_EF_EXPLODE)
                    return AVERROR_INVALIDDATA;
            }
        }

        // XXX FIXME fine-tune, and perhaps add dc too
        s->qscale[index] = FFMAX(s->quant_matrixes[index][1],
                                 s->quant_matrixes[index][8]) >> 1;
        av_log(s->avctx, AV_LOG_DEBUG, "qscale[%d]: %d\n",
               index, s->qscale[index]);
        len -= 1 + 64 * (1+pr);
    }
    return 0;
}

/* decode huffman tables and build VLC decoders */
int ff_mjpeg_decode_dht(MJpegDecodeContext *s)
{
    int len, index, i, class, n, v;
    uint8_t bits_table[17];
    uint8_t val_table[256];
    int ret = 0;

    len = get_bits(&s->gb, 16) - 2;

    if (8*len > get_bits_left(&s->gb)) {
        av_log(s->avctx, AV_LOG_ERROR, "dht: len %d is too large\n", len);
        return AVERROR_INVALIDDATA;
    }

    while (len > 0) {
        if (len < 17)
            return AVERROR_INVALIDDATA;
        class = get_bits(&s->gb, 4);
        if (class >= 2)
            return AVERROR_INVALIDDATA;
        index = get_bits(&s->gb, 4);
        if (index >= 4)
            return AVERROR_INVALIDDATA;
        n = 0;
        for (i = 1; i <= 16; i++) {
            bits_table[i] = get_bits(&s->gb, 8);
            n += bits_table[i];
        }
        len -= 17;
        if (len < n || n > 256)
            return AVERROR_INVALIDDATA;

        for (i = 0; i < n; i++) {
            v = get_bits(&s->gb, 8);
            val_table[i] = v;
        }
        len -= n;

        /* build VLC and flush previous vlc if present */
        ff_vlc_free(&s->vlcs[class][index]);
        av_log(s->avctx, AV_LOG_DEBUG, "class=%d index=%d nb_codes=%d\n",
               class, index, n);
        if ((ret = ff_mjpeg_build_vlc(&s->vlcs[class][index], bits_table,
                                      val_table, class > 0, s->avctx)) < 0)
            return ret;

        if (class > 0) {
            ff_vlc_free(&s->vlcs[2][index]);
            if ((ret = ff_mjpeg_build_vlc(&s->vlcs[2][index], bits_table,
                                          val_table, 0, s->avctx)) < 0)
                return ret;
        }

        for (i = 0; i < 16; i++)
            s->raw_huffman_lengths[class][index][i] = bits_table[i + 1];
        for (i = 0; i < 256; i++)
            s->raw_huffman_values[class][index][i] = val_table[i];
    }
    return 0;
}

int ff_mjpeg_decode_sof(MJpegDecodeContext *s)
{
    int len, nb_components, i, width, height, bits, ret, size_change;
    unsigned pix_fmt_id;
    int h_count[MAX_COMPONENTS] = { 0 };
    int v_count[MAX_COMPONENTS] = { 0 };

    s->cur_scan = 0;
    memset(s->upscale_h, 0, sizeof(s->upscale_h));
    memset(s->upscale_v, 0, sizeof(s->upscale_v));

    len     = get_bits(&s->gb, 16);
    bits    = get_bits(&s->gb, 8);

    if (bits > 16 || bits < 1) {
        av_log(s->avctx, AV_LOG_ERROR, "bits %d is invalid\n", bits);
        return AVERROR_INVALIDDATA;
    }

    if (s->avctx->bits_per_raw_sample != bits) {
        av_log(s->avctx, s->avctx->bits_per_raw_sample > 0 ? AV_LOG_INFO : AV_LOG_DEBUG, "Changing bps from %d to %d\n", s->avctx->bits_per_raw_sample, bits);
        s->avctx->bits_per_raw_sample = bits;
        init_idct(s->avctx);
    }
    if (s->pegasus_rct)
        bits = 9;
    if (bits == 9 && !s->pegasus_rct)
        s->rct  = 1;    // FIXME ugly

    if(s->lossless && s->avctx->lowres){
        av_log(s->avctx, AV_LOG_ERROR, "lowres is not possible with lossless jpeg\n");
        return -1;
    }

    height = get_bits(&s->gb, 16);
    width  = get_bits(&s->gb, 16);

    av_log(s->avctx, AV_LOG_DEBUG, "sof0: picture: %dx%d\n", width, height);
    if (av_image_check_size(width, height, 0, s->avctx) < 0)
        return AVERROR_INVALIDDATA;

    nb_components = get_bits(&s->gb, 8);
    if (nb_components <= 0 ||
        nb_components > MAX_COMPONENTS)
        return -1;
    if (s->ls && !(bits <= 8 || nb_components == 1)) {
        avpriv_report_missing_feature(s->avctx,
                                      "JPEG-LS that is not <= 8 "
                                      "bits/component or 16-bit gray");
        return AVERROR_PATCHWELCOME;
    }
    if (len != 8 + 3 * nb_components) {
        av_log(s->avctx, AV_LOG_ERROR, "decode_sof0: error, len(%d) mismatch %d components\n", len, nb_components);
        return AVERROR_INVALIDDATA;
    }

    s->nb_components = nb_components;
    s->h_max         = 1;
    s->v_max         = 1;
    for (i = 0; i < nb_components; i++) {
        /* component id */
        s->component_id[i] = get_bits(&s->gb, 8);
        h_count[i]         = get_bits(&s->gb, 4);
        v_count[i]         = get_bits(&s->gb, 4);
        /* compute hmax and vmax (only used in interleaved case) */
        if (h_count[i] > s->h_max)
            s->h_max = h_count[i];
        if (v_count[i] > s->v_max)
            s->v_max = v_count[i];
        s->quant_index[i] = get_bits(&s->gb, 8);
        if (s->quant_index[i] >= 4) {
            av_log(s->avctx, AV_LOG_ERROR, "quant_index is invalid\n");
            return AVERROR_INVALIDDATA;
        }
        if (!h_count[i] || !v_count[i]) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "Invalid sampling factor in component %d %d:%d\n",
                   i, h_count[i], v_count[i]);
            return AVERROR_INVALIDDATA;
        }

        av_log(s->avctx, AV_LOG_DEBUG, "component %d %d:%d id: %d quant:%d\n",
               i, h_count[i], v_count[i],
               s->component_id[i], s->quant_index[i]);
    }
    if (   nb_components == 4
        && s->component_id[0] == 'C'
        && s->component_id[1] == 'M'
        && s->component_id[2] == 'Y'
        && s->component_id[3] == 'K')
        s->adobe_transform = 0;

    if (s->ls && (s->h_max > 1 || s->v_max > 1)) {
        avpriv_report_missing_feature(s->avctx, "Subsampling in JPEG-LS");
        return AVERROR_PATCHWELCOME;
    }

    if (s->bayer) {
        if (nb_components == 2) {
            /* Bayer images embedded in DNGs can contain 2 interleaved components and the
               width stored in their SOF3 markers is the width of each one.  We only output
               a single component, therefore we need to adjust the output image width.  We
               handle the deinterleaving (but not the debayering) in this file. */
            width *= 2;
        }
        /* They can also contain 1 component, which is double the width and half the height
            of the final image (rows are interleaved).  We don't handle the decoding in this
            file, but leave that to the TIFF/DNG decoder. */
    }

    /* if different size, realloc/alloc picture */
    if (width != s->width || height != s->height || bits != s->bits ||
        memcmp(s->h_count, h_count, sizeof(h_count))                ||
        memcmp(s->v_count, v_count, sizeof(v_count))) {
        size_change = 1;

        s->width      = width;
        s->height     = height;
        s->bits       = bits;
        memcpy(s->h_count, h_count, sizeof(h_count));
        memcpy(s->v_count, v_count, sizeof(v_count));
        s->got_picture = 0;

        ret = ff_set_dimensions(s->avctx, width, height);
        if (ret < 0)
            return ret;

        if (s->avctx->codec_id != AV_CODEC_ID_SMVJPEG &&
            (s->avctx->codec_tag == MKTAG('A', 'V', 'R', 'n') ||
             s->avctx->codec_tag == MKTAG('A', 'V', 'D', 'J')) &&
            s->orig_height < height)
            s->avctx->height = AV_CEIL_RSHIFT(s->orig_height, s->avctx->lowres);

        s->first_picture = 0;
    } else {
        size_change = 0;
    }

    if (s->avctx->codec_id == AV_CODEC_ID_SMVJPEG) {
        s->avctx->height = s->avctx->coded_height / s->smv_frames_per_jpeg;
        if (s->avctx->height <= 0)
            return AVERROR_INVALIDDATA;
    }
    if (s->bayer && s->progressive)
        return AVERROR_INVALIDDATA;

    {
        if (s->v_max == 1 && s->h_max == 1 && s->lossless==1 && (nb_components==3 || nb_components==4))
            s->rgb = 1;
        else if (!s->lossless)
            s->rgb = 0;
        /* XXX: not complete test ! */
        pix_fmt_id = ((unsigned)s->h_count[0] << 28) | (s->v_count[0] << 24) |
                     (s->h_count[1] << 20) | (s->v_count[1] << 16) |
                     (s->h_count[2] << 12) | (s->v_count[2] <<  8) |
                     (s->h_count[3] <<  4) |  s->v_count[3];
        av_log(s->avctx, AV_LOG_DEBUG, "pix fmt id %x\n", pix_fmt_id);
        /* NOTE we do not allocate pictures large enough for the possible
         * padding of h/v_count being 4 */
        if (!(pix_fmt_id & 0xD0D0D0D0))
            pix_fmt_id -= (pix_fmt_id & 0xF0F0F0F0) >> 1;
        if (!(pix_fmt_id & 0x0D0D0D0D))
            pix_fmt_id -= (pix_fmt_id & 0x0F0F0F0F) >> 1;

        for (i = 0; i < 8; i++) {
            int j = 6 + (i&1) - (i&6);
            int is = (pix_fmt_id >> (4*i)) & 0xF;
            int js = (pix_fmt_id >> (4*j)) & 0xF;

            if (is == 1 && js != 2 && (i < 2 || i > 5))
                js = (pix_fmt_id >> ( 8 + 4*(i&1))) & 0xF;
            if (is == 1 && js != 2 && (i < 2 || i > 5))
                js = (pix_fmt_id >> (16 + 4*(i&1))) & 0xF;

            if (is == 1 && js == 2) {
                if (i & 1) s->upscale_h[j/2] = 1;
                else       s->upscale_v[j/2] = 1;
            }
        }

        if (s->bayer) {
            if (pix_fmt_id != 0x11110000 && pix_fmt_id != 0x11000000)
                goto unk_pixfmt;
        }

        switch (pix_fmt_id) {
        case 0x11110000: /* for bayer-encoded huffman lossless JPEGs embedded in DNGs */
            if (!s->bayer)
                goto unk_pixfmt;
            s->avctx->pix_fmt = AV_PIX_FMT_GRAY16LE;
            break;
        case 0x11111100:
            if (s->rgb)
                s->avctx->pix_fmt = s->bits <= 9 ? AV_PIX_FMT_BGR24 : AV_PIX_FMT_BGR48;
            else {
                if (   s->adobe_transform == 0
                    || s->component_id[0] == 'R' && s->component_id[1] == 'G' && s->component_id[2] == 'B') {
                    s->avctx->pix_fmt = s->bits <= 8 ? AV_PIX_FMT_GBRP : AV_PIX_FMT_GBRP16;
                } else {
                    if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUVJ444P;
                    else              s->avctx->pix_fmt = AV_PIX_FMT_YUV444P16;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
                }
            }
            av_assert0(s->nb_components == 3);
            break;
        case 0x11111111:
            if (s->rgb)
                s->avctx->pix_fmt = s->bits <= 9 ? AV_PIX_FMT_ABGR : AV_PIX_FMT_RGBA64;
            else {
                if (s->adobe_transform == 0 && s->bits <= 8) {
                    s->avctx->pix_fmt = AV_PIX_FMT_GBRAP;
                } else {
                    s->avctx->pix_fmt = s->bits <= 8 ? AV_PIX_FMT_YUVA444P : AV_PIX_FMT_YUVA444P16;
                    s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
                }
            }
            av_assert0(s->nb_components == 4);
            break;
        case 0x11412100:
            if (s->bits > 8)
                goto unk_pixfmt;
            if (s->component_id[0] == 'R' && s->component_id[1] == 'G' && s->component_id[2] == 'B') {
                s->avctx->pix_fmt = AV_PIX_FMT_GBRP;
                s->upscale_h[0] = 4;
                s->upscale_h[1] = 0;
                s->upscale_h[2] = 1;
            } else {
                goto unk_pixfmt;
            }
            break;
        case 0x22111122:
        case 0x22111111:
            if (s->adobe_transform == 0 && s->bits <= 8) {
                s->avctx->pix_fmt = AV_PIX_FMT_GBRAP;
                s->upscale_v[1] = s->upscale_v[2] = 1;
                s->upscale_h[1] = s->upscale_h[2] = 1;
            } else if (s->adobe_transform == 2 && s->bits <= 8) {
                s->avctx->pix_fmt = AV_PIX_FMT_YUVA444P;
                s->upscale_v[1] = s->upscale_v[2] = 1;
                s->upscale_h[1] = s->upscale_h[2] = 1;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            } else {
                if (s->bits <= 8) s->avctx->pix_fmt = AV_PIX_FMT_YUVA420P;
                else              s->avctx->pix_fmt = AV_PIX_FMT_YUVA420P16;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            }
            av_assert0(s->nb_components == 4);
            break;
        case 0x12121100:
        case 0x22122100:
        case 0x21211100:
        case 0x21112100:
        case 0x22211200:
        case 0x22221100:
        case 0x22112200:
        case 0x11222200:
            if (s->bits > 8)
                goto unk_pixfmt;
            if (s->adobe_transform == 0 || s->component_id[0] == 'R' &&
                    s->component_id[1] == 'G' && s->component_id[2] == 'B') {
                s->avctx->pix_fmt = AV_PIX_FMT_GBRP;
            } else {
                s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUVJ444P;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            }
            break;
        case 0x11000000:
        case 0x13000000:
        case 0x14000000:
        case 0x31000000:
        case 0x33000000:
        case 0x34000000:
        case 0x41000000:
        case 0x43000000:
        case 0x44000000:
            if(s->bits <= 8)
                s->avctx->pix_fmt = s->force_pal8 ? AV_PIX_FMT_PAL8 : AV_PIX_FMT_GRAY8;
            else
                s->avctx->pix_fmt = AV_PIX_FMT_GRAY16;
            break;
        case 0x12111100:
        case 0x14121200:
        case 0x14111100:
        case 0x22211100:
        case 0x22112100:
            if (s->component_id[0] == 'R' && s->component_id[1] == 'G' && s->component_id[2] == 'B') {
                if (s->bits <= 8) s->avctx->pix_fmt = AV_PIX_FMT_GBRP;
                else
                    goto unk_pixfmt;
                s->upscale_v[1] = s->upscale_v[2] = 1;
            } else {
                if (pix_fmt_id == 0x14111100)
                    s->upscale_v[1] = s->upscale_v[2] = 1;
                if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV440P : AV_PIX_FMT_YUVJ440P;
                else
                    goto unk_pixfmt;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            }
            break;
        case 0x21111100:
            if (s->component_id[0] == 'R' && s->component_id[1] == 'G' && s->component_id[2] == 'B') {
                if (s->bits <= 8) s->avctx->pix_fmt = AV_PIX_FMT_GBRP;
                else
                    goto unk_pixfmt;
                s->upscale_h[1] = s->upscale_h[2] = 1;
            } else {
                if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV422P : AV_PIX_FMT_YUVJ422P;
                else              s->avctx->pix_fmt = AV_PIX_FMT_YUV422P16;
                s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            }
            break;
        case 0x11311100:
            if (s->bits > 8)
                goto unk_pixfmt;
            if (s->component_id[0] == 'R' && s->component_id[1] == 'G' && s->component_id[2] == 'B')
                s->avctx->pix_fmt = AV_PIX_FMT_GBRP;
            else
                goto unk_pixfmt;
            s->upscale_h[0] = s->upscale_h[2] = 2;
            break;
        case 0x31111100:
            if (s->bits > 8)
                goto unk_pixfmt;
            s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUVJ444P;
            s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            s->upscale_h[1] = s->upscale_h[2] = 2;
            break;
        case 0x22121100:
        case 0x22111200:
        case 0x41211100:
            if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV422P : AV_PIX_FMT_YUVJ422P;
            else
                goto unk_pixfmt;
            s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            break;
        case 0x22111100:
        case 0x23111100:
        case 0x42111100:
        case 0x24111100:
            if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV420P : AV_PIX_FMT_YUVJ420P;
            else              s->avctx->pix_fmt = AV_PIX_FMT_YUV420P16;
            s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            if (pix_fmt_id == 0x42111100) {
                if (s->bits > 8)
                    goto unk_pixfmt;
                s->upscale_h[1] = s->upscale_h[2] = 1;
            } else if (pix_fmt_id == 0x24111100) {
                if (s->bits > 8)
                    goto unk_pixfmt;
                s->upscale_v[1] = s->upscale_v[2] = 1;
            } else if (pix_fmt_id == 0x23111100) {
                if (s->bits > 8)
                    goto unk_pixfmt;
                s->upscale_v[1] = s->upscale_v[2] = 2;
            }
            break;
        case 0x41111100:
            if (s->bits <= 8) s->avctx->pix_fmt = s->cs_itu601 ? AV_PIX_FMT_YUV411P : AV_PIX_FMT_YUVJ411P;
            else
                goto unk_pixfmt;
            s->avctx->color_range = s->cs_itu601 ? AVCOL_RANGE_MPEG : AVCOL_RANGE_JPEG;
            break;
        default:
    unk_pixfmt:
            avpriv_report_missing_feature(s->avctx, "Pixel format 0x%x bits:%d", pix_fmt_id, s->bits);
            memset(s->upscale_h, 0, sizeof(s->upscale_h));
            memset(s->upscale_v, 0, sizeof(s->upscale_v));
            return AVERROR_PATCHWELCOME;
        }
        if ((AV_RB32(s->upscale_h) || AV_RB32(s->upscale_v)) && s->avctx->lowres) {
            avpriv_report_missing_feature(s->avctx, "Lowres for weird subsampling");
            return AVERROR_PATCHWELCOME;
        }
        if (s->ls) {
            memset(s->upscale_h, 0, sizeof(s->upscale_h));
            memset(s->upscale_v, 0, sizeof(s->upscale_v));
            if (s->nb_components == 3) {
                s->avctx->pix_fmt = AV_PIX_FMT_RGB24;
            } else if (s->nb_components != 1) {
                av_log(s->avctx, AV_LOG_ERROR, "Unsupported number of components %d\n", s->nb_components);
                return AVERROR_PATCHWELCOME;
            } else if ((s->palette_index || s->force_pal8) && s->bits <= 8)
                s->avctx->pix_fmt = AV_PIX_FMT_PAL8;
            else if (s->bits <= 8)
                s->avctx->pix_fmt = AV_PIX_FMT_GRAY8;
            else
                s->avctx->pix_fmt = AV_PIX_FMT_GRAY16;
        }

        s->pix_desc = av_pix_fmt_desc_get(s->avctx->pix_fmt);
        if (!s->pix_desc) {
            av_log(s->avctx, AV_LOG_ERROR, "Could not get a pixel format descriptor.\n");
            return AVERROR_BUG;
        }

        if (s->avctx->pix_fmt == s->hwaccel_sw_pix_fmt && !size_change) {
            s->avctx->pix_fmt = s->hwaccel_pix_fmt;
        } else {
            enum AVPixelFormat pix_fmts[] = {
#if CONFIG_MJPEG_NVDEC_HWACCEL
                AV_PIX_FMT_CUDA,
#endif
#if CONFIG_MJPEG_VAAPI_HWACCEL
                AV_PIX_FMT_VAAPI,
#endif
                s->avctx->pix_fmt,
                AV_PIX_FMT_NONE,
            };
            s->hwaccel_pix_fmt = ff_get_format(s->avctx, pix_fmts);
            if (s->hwaccel_pix_fmt < 0)
                return AVERROR(EINVAL);

            s->hwaccel_sw_pix_fmt = s->avctx->pix_fmt;
            s->avctx->pix_fmt     = s->hwaccel_pix_fmt;
        }

        av_frame_unref(s->picture_ptr);
        ret = ff_thread_get_buffer(s->avctx, s->picture_ptr, 0);
        if (ret < 0)
            return ret;
        s->picture_ptr->pict_type = AV_PICTURE_TYPE_I;
        s->picture_ptr->flags |= AV_FRAME_FLAG_KEY;
        s->got_picture            = 1;

        if (s->avctx->skip_frame == AVDISCARD_ALL)
            return 0;

        // Lets clear the palette to avoid leaving uninitialized values in it
        if (s->avctx->pix_fmt == AV_PIX_FMT_PAL8)
            memset(s->picture_ptr->data[1], 0, 1024);

        for (i = 0; i < 4; i++)
            s->linesize[i] = s->picture_ptr->linesize[i];

        ff_dlog(s->avctx, "%d %d %d %d %d\n",
                s->width, s->height, s->linesize[0], s->linesize[1],
                s->avctx->height);
    }

    if ((s->rgb && !s->lossless && !s->ls) ||
        (!s->rgb && s->ls && s->nb_components > 1) ||
        (s->avctx->pix_fmt == AV_PIX_FMT_PAL8 && !s->ls)
    ) {
        av_log(s->avctx, AV_LOG_ERROR, "Unsupported coding and pixel format combination\n");
        return AVERROR_PATCHWELCOME;
    }

    /* totally blank picture as progressive JPEG will only add details to it */
    if (s->progressive) {
        int bw = (width  + s->h_max * 8 - 1) / (s->h_max * 8);
        int bh = (height + s->v_max * 8 - 1) / (s->v_max * 8);
        for (i = 0; i < s->nb_components; i++) {
            int size = bw * bh * s->h_count[i] * s->v_count[i];
            av_freep(&s->blocks[i]);
            av_freep(&s->last_nnz[i]);
            s->blocks[i]       = av_calloc(size, sizeof(**s->blocks));
            s->last_nnz[i]     = av_calloc(size, sizeof(**s->last_nnz));
            if (!s->blocks[i] || !s->last_nnz[i])
                return AVERROR(ENOMEM);
            s->block_stride[i] = bw * s->h_count[i];
        }
        memset(s->coefs_finished, 0, sizeof(s->coefs_finished));
    }

    if (s->avctx->hwaccel) {
        const FFHWAccel *hwaccel = ffhwaccel(s->avctx->hwaccel);
        s->hwaccel_picture_private =
            av_mallocz(hwaccel->frame_priv_data_size);
        if (!s->hwaccel_picture_private)
            return AVERROR(ENOMEM);

        ret = hwaccel->start_frame(s->avctx, NULL, s->raw_image_buffer,
                                   s->raw_image_buffer_size);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static inline int mjpeg_decode_dc(MJpegDecodeContext *s, int dc_index)
{
    int code;
    code = get_vlc2(&s->gb, s->vlcs[0][dc_index].table, 9, 2);
    if (code < 0 || code > 16) {
        av_log(s->avctx, AV_LOG_WARNING,
               "mjpeg_decode_dc: bad vlc: %d:%d (%p)\n",
               0, dc_index, &s->vlcs[0][dc_index]);
        return 0xfffff;
    }

    if (code)
        return get_xbits(&s->gb, code);
    else
        return 0;
}

/* decode block and dequantize */
static int decode_block(MJpegDecodeContext *s, int16_t *block, int component,
                        int dc_index, int ac_index, uint16_t *quant_matrix)
{
    int code, i, j, level, val;

    /* DC coef */
    val = mjpeg_decode_dc(s, dc_index);
    if (val == 0xfffff) {
        av_log(s->avctx, AV_LOG_ERROR, "error dc\n");
        return AVERROR_INVALIDDATA;
    }
    val = val * (unsigned)quant_matrix[0] + s->last_dc[component];
    s->last_dc[component] = val;
    block[0] = av_clip_int16(val);
    /* AC coefs */
    i = 0;
    {OPEN_READER(re, &s->gb);
    do {
        UPDATE_CACHE(re, &s->gb);
        GET_VLC(code, re, &s->gb, s->vlcs[1][ac_index].table, 9, 2);

        i += ((unsigned)code) >> 4;
            code &= 0xf;
        if (code) {
            if (code > MIN_CACHE_BITS - 16)
                UPDATE_CACHE(re, &s->gb);

            {
                int cache = GET_CACHE(re, &s->gb);
                int sign  = (~cache) >> 31;
                level     = (NEG_USR32(sign ^ cache,code) ^ sign) - sign;
            }

            LAST_SKIP_BITS(re, &s->gb, code);

            if (i > 63) {
                av_log(s->avctx, AV_LOG_ERROR, "error count: %d\n", i);
                return AVERROR_INVALIDDATA;
            }
            j        = s->permutated_scantable[i];
            block[j] = level * quant_matrix[i];
        }
    } while (i < 63);
    CLOSE_READER(re, &s->gb);}

    return 0;
}

static int decode_dc_progressive(MJpegDecodeContext *s, int16_t *block,
                                 int component, int dc_index,
                                 uint16_t *quant_matrix, int Al)
{
    unsigned val;
    s->bdsp.clear_block(block);
    val = mjpeg_decode_dc(s, dc_index);
    if (val == 0xfffff) {
        av_log(s->avctx, AV_LOG_ERROR, "error dc\n");
        return AVERROR_INVALIDDATA;
    }
    val = (val * (quant_matrix[0] << Al)) + s->last_dc[component];
    s->last_dc[component] = val;
    block[0] = val;
    return 0;
}

/* decode block and dequantize - progressive JPEG version */
static int decode_block_progressive(MJpegDecodeContext *s, int16_t *block,
                                    uint8_t *last_nnz, int ac_index,
                                    uint16_t *quant_matrix,
                                    int ss, int se, int Al, int *EOBRUN)
{
    int code, i, j, val, run;
    unsigned level;

    if (*EOBRUN) {
        (*EOBRUN)--;
        return 0;
    }

    {
        OPEN_READER(re, &s->gb);
        for (i = ss; ; i++) {
            UPDATE_CACHE(re, &s->gb);
            GET_VLC(code, re, &s->gb, s->vlcs[2][ac_index].table, 9, 2);

            run = ((unsigned) code) >> 4;
            code &= 0xF;
            if (code) {
                i += run;
                if (code > MIN_CACHE_BITS - 16)
                    UPDATE_CACHE(re, &s->gb);

                {
                    int cache = GET_CACHE(re, &s->gb);
                    int sign  = (~cache) >> 31;
                    level     = (NEG_USR32(sign ^ cache,code) ^ sign) - sign;
                }

                LAST_SKIP_BITS(re, &s->gb, code);

                if (i >= se) {
                    if (i == se) {
                        j = s->permutated_scantable[se];
                        block[j] = level * (quant_matrix[se] << Al);
                        break;
                    }
                    av_log(s->avctx, AV_LOG_ERROR, "error count: %d\n", i);
                    return AVERROR_INVALIDDATA;
                }
                j = s->permutated_scantable[i];
                block[j] = level * (quant_matrix[i] << Al);
            } else {
                if (run == 0xF) {// ZRL - skip 15 coefficients
                    i += 15;
                    if (i >= se) {
                        av_log(s->avctx, AV_LOG_ERROR, "ZRL overflow: %d\n", i);
                        return AVERROR_INVALIDDATA;
                    }
                } else {
                    val = (1 << run);
                    if (run) {
                        UPDATE_CACHE(re, &s->gb);
                        val += NEG_USR32(GET_CACHE(re, &s->gb), run);
                        LAST_SKIP_BITS(re, &s->gb, run);
                    }
                    *EOBRUN = val - 1;
                    break;
                }
            }
        }
        CLOSE_READER(re, &s->gb);
    }

    if (i > *last_nnz)
        *last_nnz = i;

    return 0;
}

#define REFINE_BIT(j) {                                             \
    UPDATE_CACHE(re, &s->gb);                                       \
    sign = block[j] >> 15;                                          \
    block[j] += SHOW_UBITS(re, &s->gb, 1) *                         \
                ((quant_matrix[i] ^ sign) - sign) << Al;            \
    LAST_SKIP_BITS(re, &s->gb, 1);                                  \
}

#define ZERO_RUN                                                    \
for (; ; i++) {                                                     \
    if (i > last) {                                                 \
        i += run;                                                   \
        if (i > se) {                                               \
            av_log(s->avctx, AV_LOG_ERROR, "error count: %d\n", i); \
            return -1;                                              \
        }                                                           \
        break;                                                      \
    }                                                               \
    j = s->permutated_scantable[i];                                 \
    if (block[j])                                                   \
        REFINE_BIT(j)                                               \
    else if (run-- == 0)                                            \
        break;                                                      \
}

/* decode block and dequantize - progressive JPEG refinement pass */
static int decode_block_refinement(MJpegDecodeContext *s, int16_t *block,
                                   uint8_t *last_nnz,
                                   int ac_index, uint16_t *quant_matrix,
                                   int ss, int se, int Al, int *EOBRUN)
{
    int code, i = ss, j, sign, val, run;
    int last    = FFMIN(se, *last_nnz);

    OPEN_READER(re, &s->gb);
    if (*EOBRUN) {
        (*EOBRUN)--;
    } else {
        for (; ; i++) {
            UPDATE_CACHE(re, &s->gb);
            GET_VLC(code, re, &s->gb, s->vlcs[2][ac_index].table, 9, 2);

            if (code & 0xF) {
                run = ((unsigned) code) >> 4;
                UPDATE_CACHE(re, &s->gb);
                val = SHOW_UBITS(re, &s->gb, 1);
                LAST_SKIP_BITS(re, &s->gb, 1);
                ZERO_RUN;
                j = s->permutated_scantable[i];
                val--;
                block[j] = ((quant_matrix[i] << Al) ^ val) - val;
                if (i == se) {
                    if (i > *last_nnz)
                        *last_nnz = i;
                    CLOSE_READER(re, &s->gb);
                    return 0;
                }
            } else {
                run = ((unsigned) code) >> 4;
                if (run == 0xF) {
                    ZERO_RUN;
                } else {
                    val = run;
                    run = (1 << run);
                    if (val) {
                        UPDATE_CACHE(re, &s->gb);
                        run += SHOW_UBITS(re, &s->gb, val);
                        LAST_SKIP_BITS(re, &s->gb, val);
                    }
                    *EOBRUN = run - 1;
                    break;
                }
            }
        }

        if (i > *last_nnz)
            *last_nnz = i;
    }

    for (; i <= last; i++) {
        j = s->permutated_scantable[i];
        if (block[j])
            REFINE_BIT(j)
    }
    CLOSE_READER(re, &s->gb);

    return 0;
}
#undef REFINE_BIT
#undef ZERO_RUN

static int handle_rstn(MJpegDecodeContext *s, int nb_components)
{
    int i;
    int reset = 0;

    if (s->restart_interval) {
        s->restart_count--;
        if(s->restart_count == 0 && s->avctx->codec_id == AV_CODEC_ID_THP){
            align_get_bits(&s->gb);
            for (i = 0; i < nb_components; i++) /* reset dc */
                s->last_dc[i] = (4 << s->bits);
        }

        i = 8 + ((-get_bits_count(&s->gb)) & 7);
        /* skip RSTn */
        if (s->restart_count == 0) {
            if(   show_bits(&s->gb, i) == (1 << i) - 1
               || show_bits(&s->gb, i) == 0xFF) {
                int pos = get_bits_count(&s->gb);
                align_get_bits(&s->gb);
                while (get_bits_left(&s->gb) >= 8 && show_bits(&s->gb, 8) == 0xFF)
                    skip_bits(&s->gb, 8);
                if (get_bits_left(&s->gb) >= 8 && (get_bits(&s->gb, 8) & 0xF8) == 0xD0) {
                    for (i = 0; i < nb_components; i++) /* reset dc */
                        s->last_dc[i] = (4 << s->bits);
                    reset = 1;
                } else
                    skip_bits_long(&s->gb, pos - get_bits_count(&s->gb));
            }
        }
    }
    return reset;
}

/* Handles 1 to 4 components */
static int ljpeg_decode_rgb_scan(MJpegDecodeContext *s, int nb_components, int predictor, int point_transform)
{
    int i, mb_x, mb_y;
    unsigned width;
    uint16_t (*buffer)[4];
    int left[4], top[4], topleft[4];
    const int linesize = s->linesize[0];
    const int mask     = ((1 << s->bits) - 1) << point_transform;
    int resync_mb_y = 0;
    int resync_mb_x = 0;
    int vpred[6];

    if (!s->bayer && s->nb_components < 3)
        return AVERROR_INVALIDDATA;
    if (s->bayer && s->nb_components > 2)
        return AVERROR_INVALIDDATA;
    if (s->nb_components <= 0 || s->nb_components > 4)
        return AVERROR_INVALIDDATA;
    if (s->v_max != 1 || s->h_max != 1 || !s->lossless)
        return AVERROR_INVALIDDATA;
    if (s->bayer) {
        if (s->rct || s->pegasus_rct)
            return AVERROR_INVALIDDATA;
    }


    s->restart_count = s->restart_interval;

    if (s->restart_interval == 0)
        s->restart_interval = INT_MAX;

    if (s->bayer)
        width = s->mb_width / nb_components; /* Interleaved, width stored is the total so need to divide */
    else
        width = s->mb_width;

    av_fast_malloc(&s->ljpeg_buffer, &s->ljpeg_buffer_size, width * 4 * sizeof(s->ljpeg_buffer[0][0]));
    if (!s->ljpeg_buffer)
        return AVERROR(ENOMEM);

    buffer = s->ljpeg_buffer;

    for (i = 0; i < 4; i++)
        buffer[0][i] = 1 << (s->bits - 1);

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        uint8_t *ptr = s->picture_ptr->data[0] + (linesize * mb_y);

        for (i = 0; i < 4; i++)
            top[i] = left[i] = topleft[i] = buffer[0][i];

        if ((mb_y * s->width) % s->restart_interval == 0) {
            for (i = 0; i < 6; i++)
                vpred[i] = 1 << (s->bits-1);
        }

        for (mb_x = 0; mb_x < width; mb_x++) {
            int modified_predictor = predictor;

            if (get_bits_left(&s->gb) < 1) {
                av_log(s->avctx, AV_LOG_ERROR, "bitstream end in rgb_scan\n");
                return AVERROR_INVALIDDATA;
            }

            if (s->restart_interval && !s->restart_count){
                s->restart_count = s->restart_interval;
                resync_mb_x = mb_x;
                resync_mb_y = mb_y;
                for(i=0; i<4; i++)
                    top[i] = left[i]= topleft[i]= 1 << (s->bits - 1);
            }
            if (mb_y == resync_mb_y || mb_y == resync_mb_y+1 && mb_x < resync_mb_x || !mb_x)
                modified_predictor = 1;

            for (i=0;i<nb_components;i++) {
                int pred, dc;

                topleft[i] = top[i];
                top[i]     = buffer[mb_x][i];

                dc = mjpeg_decode_dc(s, s->dc_index[i]);
                if(dc == 0xFFFFF)
                    return -1;

                if (!s->bayer || mb_x) {
                    pred = left[i];
                } else { /* This path runs only for the first line in bayer images */
                    vpred[i] += dc;
                    pred = vpred[i] - dc;
                }

                PREDICT(pred, topleft[i], top[i], pred, modified_predictor);

                left[i] = buffer[mb_x][i] =
                    mask & (pred + (unsigned)(dc * (1 << point_transform)));
            }

            if (s->restart_interval && !--s->restart_count) {
                align_get_bits(&s->gb);
                skip_bits(&s->gb, 16); /* skip RSTn */
            }
        }
        if (s->rct && s->nb_components == 4) {
            for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
                ptr[4*mb_x + 2] = buffer[mb_x][0] - ((buffer[mb_x][1] + buffer[mb_x][2] - 0x200) >> 2);
                ptr[4*mb_x + 1] = buffer[mb_x][1] + ptr[4*mb_x + 2];
                ptr[4*mb_x + 3] = buffer[mb_x][2] + ptr[4*mb_x + 2];
                ptr[4*mb_x + 0] = buffer[mb_x][3];
            }
        } else if (s->nb_components == 4) {
            for(i=0; i<nb_components; i++) {
                int c= s->comp_index[i];
                if (s->bits <= 8) {
                    for(mb_x = 0; mb_x < s->mb_width; mb_x++) {
                        ptr[4*mb_x+3-c] = buffer[mb_x][i];
                    }
                } else if(s->bits == 9) {
                    return AVERROR_PATCHWELCOME;
                } else {
                    for(mb_x = 0; mb_x < s->mb_width; mb_x++) {
                        ((uint16_t*)ptr)[4*mb_x+c] = buffer[mb_x][i];
                    }
                }
            }
        } else if (s->rct) {
            for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
                ptr[3*mb_x + 1] = buffer[mb_x][0] - ((buffer[mb_x][1] + buffer[mb_x][2] - 0x200) >> 2);
                ptr[3*mb_x + 0] = buffer[mb_x][1] + ptr[3*mb_x + 1];
                ptr[3*mb_x + 2] = buffer[mb_x][2] + ptr[3*mb_x + 1];
            }
        } else if (s->pegasus_rct) {
            for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
                ptr[3*mb_x + 1] = buffer[mb_x][0] - ((buffer[mb_x][1] + buffer[mb_x][2]) >> 2);
                ptr[3*mb_x + 0] = buffer[mb_x][1] + ptr[3*mb_x + 1];
                ptr[3*mb_x + 2] = buffer[mb_x][2] + ptr[3*mb_x + 1];
            }
        } else if (s->bayer) {
            if (s->bits <= 8)
                return AVERROR_PATCHWELCOME;
            if (nb_components == 1) {
                /* Leave decoding to the TIFF/DNG decoder (see comment in ff_mjpeg_decode_sof) */
                for (mb_x = 0; mb_x < width; mb_x++)
                    ((uint16_t*)ptr)[mb_x] = buffer[mb_x][0];
            } else if (nb_components == 2) {
                for (mb_x = 0; mb_x < width; mb_x++) {
                    ((uint16_t*)ptr)[2*mb_x + 0] = buffer[mb_x][0];
                    ((uint16_t*)ptr)[2*mb_x + 1] = buffer[mb_x][1];
                }
            }
        } else {
            for(i=0; i<nb_components; i++) {
                int c= s->comp_index[i];
                if (s->bits <= 8) {
                    for(mb_x = 0; mb_x < s->mb_width; mb_x++) {
                        ptr[3*mb_x+2-c] = buffer[mb_x][i];
                    }
                } else if(s->bits == 9) {
                    return AVERROR_PATCHWELCOME;
                } else {
                    for(mb_x = 0; mb_x < s->mb_width; mb_x++) {
                        ((uint16_t*)ptr)[3*mb_x+2-c] = buffer[mb_x][i];
                    }
                }
            }
        }
    }
    return 0;
}

static int ljpeg_decode_yuv_scan(MJpegDecodeContext *s, int predictor,
                                 int point_transform, int nb_components)
{
    int i, mb_x, mb_y, mask;
    int bits= (s->bits+7)&~7;
    int resync_mb_y = 0;
    int resync_mb_x = 0;

    point_transform += bits - s->bits;
    mask = ((1 << s->bits) - 1) << point_transform;

    av_assert0(nb_components>=1 && nb_components<=4);

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
            if (get_bits_left(&s->gb) < 1) {
                av_log(s->avctx, AV_LOG_ERROR, "bitstream end in yuv_scan\n");
                return AVERROR_INVALIDDATA;
            }
            if (s->restart_interval && !s->restart_count){
                s->restart_count = s->restart_interval;
                resync_mb_x = mb_x;
                resync_mb_y = mb_y;
            }

            if(!mb_x || mb_y == resync_mb_y || mb_y == resync_mb_y+1 && mb_x < resync_mb_x){
                int toprow  = mb_y == resync_mb_y || mb_y == resync_mb_y+1 && mb_x < resync_mb_x;
                int leftcol = !mb_x || mb_y == resync_mb_y && mb_x == resync_mb_x;
                for (i = 0; i < nb_components; i++) {
                    uint8_t *ptr;
                    uint16_t *ptr16;
                    int n, h, v, x, y, c, j, linesize;
                    n = s->nb_blocks[i];
                    c = s->comp_index[i];
                    h = s->h_scount[i];
                    v = s->v_scount[i];
                    x = 0;
                    y = 0;
                    linesize= s->linesize[c];

                    if(bits>8) linesize /= 2;

                    for(j=0; j<n; j++) {
                        int pred, dc;

                        dc = mjpeg_decode_dc(s, s->dc_index[i]);
                        if(dc == 0xFFFFF)
                            return -1;
                        if (   h * mb_x + x >= s->width
                            || v * mb_y + y >= s->height) {
                            // Nothing to do
                        } else if (bits<=8) {
                            ptr = s->picture_ptr->data[c] + (linesize * (v * mb_y + y)) + (h * mb_x + x); //FIXME optimize this crap
                            if(y==0 && toprow){
                                if(x==0 && leftcol){
                                    pred= 1 << (bits - 1);
                                }else{
                                    pred= ptr[-1];
                                }
                            }else{
                                if(x==0 && leftcol){
                                    pred= ptr[-linesize];
                                }else{
                                    PREDICT(pred, ptr[-linesize-1], ptr[-linesize], ptr[-1], predictor);
                                }
                            }

                            pred &= mask;
                            *ptr= pred + ((unsigned)dc << point_transform);
                        }else{
                            ptr16 = (uint16_t*)(s->picture_ptr->data[c] + 2*(linesize * (v * mb_y + y)) + 2*(h * mb_x + x)); //FIXME optimize this crap
                            if(y==0 && toprow){
                                if(x==0 && leftcol){
                                    pred= 1 << (bits - 1);
                                }else{
                                    pred= ptr16[-1];
                                }
                            }else{
                                if(x==0 && leftcol){
                                    pred= ptr16[-linesize];
                                }else{
                                    PREDICT(pred, ptr16[-linesize-1], ptr16[-linesize], ptr16[-1], predictor);
                                }
                            }

                            pred &= mask;
                            *ptr16= pred + ((unsigned)dc << point_transform);
                        }
                        if (++x == h) {
                            x = 0;
                            y++;
                        }
                    }
                }
            } else {
                for (i = 0; i < nb_components; i++) {
                    uint8_t *ptr;
                    uint16_t *ptr16;
                    int n, h, v, x, y, c, j, linesize, dc;
                    n        = s->nb_blocks[i];
                    c        = s->comp_index[i];
                    h        = s->h_scount[i];
                    v        = s->v_scount[i];
                    x        = 0;
                    y        = 0;
                    linesize = s->linesize[c];

                    if(bits>8) linesize /= 2;

                    for (j = 0; j < n; j++) {
                        int pred;

                        dc = mjpeg_decode_dc(s, s->dc_index[i]);
                        if(dc == 0xFFFFF)
                            return -1;
                        if (   h * mb_x + x >= s->width
                            || v * mb_y + y >= s->height) {
                            // Nothing to do
                        } else if (bits<=8) {
                            ptr = s->picture_ptr->data[c] +
                              (linesize * (v * mb_y + y)) +
                              (h * mb_x + x); //FIXME optimize this crap
                            PREDICT(pred, ptr[-linesize-1], ptr[-linesize], ptr[-1], predictor);

                            pred &= mask;
                            *ptr = pred + ((unsigned)dc << point_transform);
                        }else{
                            ptr16 = (uint16_t*)(s->picture_ptr->data[c] + 2*(linesize * (v * mb_y + y)) + 2*(h * mb_x + x)); //FIXME optimize this crap
                            PREDICT(pred, ptr16[-linesize-1], ptr16[-linesize], ptr16[-1], predictor);

                            pred &= mask;
                            *ptr16= pred + ((unsigned)dc << point_transform);
                        }

                        if (++x == h) {
                            x = 0;
                            y++;
                        }
                    }
                }
            }
            if (s->restart_interval && !--s->restart_count) {
                align_get_bits(&s->gb);
                skip_bits(&s->gb, 16); /* skip RSTn */
            }
        }
    }
    return 0;
}

static av_always_inline void mjpeg_copy_block(MJpegDecodeContext *s,
                                              uint8_t *dst, const uint8_t *src,
                                              int linesize, int lowres)
{
    switch (lowres) {
    case 0: s->hdsp.put_pixels_tab[1][0](dst, src, linesize, 8);
        break;
    case 1: copy_block4(dst, src, linesize, linesize, 4);
        break;
    case 2: copy_block2(dst, src, linesize, linesize, 2);
        break;
    case 3: *dst = *src;
        break;
    }
}

static void shift_output(MJpegDecodeContext *s, uint8_t *ptr, int linesize)
{
    int block_x, block_y;
    int size = 8 >> s->avctx->lowres;
    if (s->bits > 8) {
        for (block_y=0; block_y<size; block_y++)
            for (block_x=0; block_x<size; block_x++)
                *(uint16_t*)(ptr + 2*block_x + block_y*linesize) <<= 16 - s->bits;
    } else {
        for (block_y=0; block_y<size; block_y++)
            for (block_x=0; block_x<size; block_x++)
                *(ptr + block_x + block_y*linesize) <<= 8 - s->bits;
    }
}

static int mjpeg_decode_scan(MJpegDecodeContext *s, int nb_components, int Ah,
                             int Al, const uint8_t *mb_bitmask,
                             int mb_bitmask_size,
                             const AVFrame *reference)
{
    int i, mb_x, mb_y, chroma_h_shift, chroma_v_shift, chroma_width, chroma_height;
    uint8_t *data[MAX_COMPONENTS];
    const uint8_t *reference_data[MAX_COMPONENTS];
    int linesize[MAX_COMPONENTS];
    GetBitContext mb_bitmask_gb = {0}; // initialize to silence gcc warning
    int bytes_per_pixel = 1 + (s->bits > 8);

    if (mb_bitmask) {
        if (mb_bitmask_size != (s->mb_width * s->mb_height + 7)>>3) {
            av_log(s->avctx, AV_LOG_ERROR, "mb_bitmask_size mismatches\n");
            return AVERROR_INVALIDDATA;
        }
        init_get_bits(&mb_bitmask_gb, mb_bitmask, s->mb_width * s->mb_height);
    }

    s->restart_count = 0;

    av_pix_fmt_get_chroma_sub_sample(s->avctx->pix_fmt, &chroma_h_shift,
                                     &chroma_v_shift);
    chroma_width  = AV_CEIL_RSHIFT(s->width,  chroma_h_shift);
    chroma_height = AV_CEIL_RSHIFT(s->height, chroma_v_shift);

    for (i = 0; i < nb_components; i++) {
        int c   = s->comp_index[i];
        data[c] = s->picture_ptr->data[c];
        reference_data[c] = reference ? reference->data[c] : NULL;
        linesize[c] = s->linesize[c];
        s->coefs_finished[c] |= 1;
    }

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
            const int copy_mb = mb_bitmask && !get_bits1(&mb_bitmask_gb);

            if (s->restart_interval && !s->restart_count)
                s->restart_count = s->restart_interval;

            if (get_bits_left(&s->gb) < 0) {
                av_log(s->avctx, AV_LOG_ERROR, "overread %d\n",
                       -get_bits_left(&s->gb));
                return AVERROR_INVALIDDATA;
            }
            for (i = 0; i < nb_components; i++) {
                uint8_t *ptr;
                int n, h, v, x, y, c, j;
                int block_offset;
                n = s->nb_blocks[i];
                c = s->comp_index[i];
                h = s->h_scount[i];
                v = s->v_scount[i];
                x = 0;
                y = 0;
                for (j = 0; j < n; j++) {
                    block_offset = (((linesize[c] * (v * mb_y + y) * 8) +
                                     (h * mb_x + x) * 8 * bytes_per_pixel) >> s->avctx->lowres);

                    if (   8*(h * mb_x + x) < ((c == 1) || (c == 2) ? chroma_width  : s->width)
                        && 8*(v * mb_y + y) < ((c == 1) || (c == 2) ? chroma_height : s->height)) {
                        ptr = data[c] + block_offset;
                    } else
                        ptr = NULL;
                    if (!s->progressive) {
                        if (copy_mb) {
                            if (ptr)
                                mjpeg_copy_block(s, ptr, reference_data[c] + block_offset,
                                                linesize[c], s->avctx->lowres);

                        } else {
                            s->bdsp.clear_block(s->block);
                            if (decode_block(s, s->block, i,
                                             s->dc_index[i], s->ac_index[i],
                                             s->quant_matrixes[s->quant_sindex[i]]) < 0) {
                                av_log(s->avctx, AV_LOG_ERROR,
                                       "error y=%d x=%d\n", mb_y, mb_x);
                                return AVERROR_INVALIDDATA;
                            }
                            if (ptr && linesize[c]) {
                                s->idsp.idct_put(ptr, linesize[c], s->block);
                                if (s->bits & 7)
                                    shift_output(s, ptr, linesize[c]);
                            }
                        }
                    } else {
                        int block_idx  = s->block_stride[c] * (v * mb_y + y) +
                                         (h * mb_x + x);
                        int16_t *block = s->blocks[c][block_idx];
                        if (Ah)
                            block[0] += get_bits1(&s->gb) *
                                        s->quant_matrixes[s->quant_sindex[i]][0] << Al;
                        else if (decode_dc_progressive(s, block, i, s->dc_index[i],
                                                       s->quant_matrixes[s->quant_sindex[i]],
                                                       Al) < 0) {
                            av_log(s->avctx, AV_LOG_ERROR,
                                   "error y=%d x=%d\n", mb_y, mb_x);
                            return AVERROR_INVALIDDATA;
                        }
                    }
                    ff_dlog(s->avctx, "mb: %d %d processed\n", mb_y, mb_x);
                    ff_dlog(s->avctx, "%d %d %d %d %d %d %d \n",
                            mb_x, mb_y, x, y, c,
                            (v * mb_y + y) * 8, (h * mb_x + x) * 8);
                    if (++x == h) {
                        x = 0;
                        y++;
                    }
                }
            }

            handle_rstn(s, nb_components);
        }
    }
    return 0;
}

static int mjpeg_decode_scan_progressive_ac(MJpegDecodeContext *s, int ss,
                                            int se, int Ah, int Al)
{
    int mb_x, mb_y;
    int EOBRUN = 0;
    int c = s->comp_index[0];
    uint16_t *quant_matrix = s->quant_matrixes[s->quant_sindex[0]];

    av_assert0(ss>=0 && Ah>=0 && Al>=0);
    if (se < ss || se > 63) {
        av_log(s->avctx, AV_LOG_ERROR, "SS/SE %d/%d is invalid\n", ss, se);
        return AVERROR_INVALIDDATA;
    }

    // s->coefs_finished is a bitmask for coefficients coded
    // ss and se are parameters telling start and end coefficients
    s->coefs_finished[c] |= (2ULL << se) - (1ULL << ss);

    s->restart_count = 0;

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        int block_idx    = mb_y * s->block_stride[c];
        int16_t (*block)[64] = &s->blocks[c][block_idx];
        uint8_t *last_nnz    = &s->last_nnz[c][block_idx];
        if (get_bits_left(&s->gb) <= 0) {
            av_log(s->avctx, AV_LOG_ERROR, "bitstream truncated in mjpeg_decode_scan_progressive_ac\n");
            return AVERROR_INVALIDDATA;
        }
        for (mb_x = 0; mb_x < s->mb_width; mb_x++, block++, last_nnz++) {
                int ret;
                if (s->restart_interval && !s->restart_count)
                    s->restart_count = s->restart_interval;

                if (Ah)
                    ret = decode_block_refinement(s, *block, last_nnz, s->ac_index[0],
                                                  quant_matrix, ss, se, Al, &EOBRUN);
                else
                    ret = decode_block_progressive(s, *block, last_nnz, s->ac_index[0],
                                                   quant_matrix, ss, se, Al, &EOBRUN);

                if (ret >= 0 && get_bits_left(&s->gb) < 0)
                    ret = AVERROR_INVALIDDATA;
                if (ret < 0) {
                    av_log(s->avctx, AV_LOG_ERROR,
                           "error y=%d x=%d\n", mb_y, mb_x);
                    return AVERROR_INVALIDDATA;
                }

            if (handle_rstn(s, 0))
                EOBRUN = 0;
        }
    }
    return 0;
}

static void mjpeg_idct_scan_progressive_ac(MJpegDecodeContext *s)
{
    int mb_x, mb_y;
    int c;
    const int bytes_per_pixel = 1 + (s->bits > 8);
    const int block_size = s->lossless ? 1 : 8;

    for (c = 0; c < s->nb_components; c++) {
        uint8_t *data = s->picture_ptr->data[c];
        int linesize  = s->linesize[c];
        int h = s->h_max / s->h_count[c];
        int v = s->v_max / s->v_count[c];
        int mb_width     = (s->width  + h * block_size - 1) / (h * block_size);
        int mb_height    = (s->height + v * block_size - 1) / (v * block_size);

        if (~s->coefs_finished[c])
            av_log(s->avctx, AV_LOG_WARNING, "component %d is incomplete\n", c);

        for (mb_y = 0; mb_y < mb_height; mb_y++) {
            uint8_t *ptr     = data + (mb_y * linesize * 8 >> s->avctx->lowres);
            int block_idx    = mb_y * s->block_stride[c];
            int16_t (*block)[64] = &s->blocks[c][block_idx];
            for (mb_x = 0; mb_x < mb_width; mb_x++, block++) {
                s->idsp.idct_put(ptr, linesize, *block);
                if (s->bits & 7)
                    shift_output(s, ptr, linesize);
                ptr += bytes_per_pixel*8 >> s->avctx->lowres;
            }
        }
    }
}

int ff_mjpeg_decode_sos(MJpegDecodeContext *s, const uint8_t *mb_bitmask,
                        int mb_bitmask_size, const AVFrame *reference)
{
    int len, nb_components, i, h, v, predictor, point_transform;
    int index, id, ret;
    const int block_size = s->lossless ? 1 : 8;
    int ilv, prev_shift;

    if (!s->got_picture) {
        av_log(s->avctx, AV_LOG_WARNING,
                "Can not process SOS before SOF, skipping\n");
        return -1;
    }

    if (reference) {
        if (reference->width  != s->picture_ptr->width  ||
            reference->height != s->picture_ptr->height ||
            reference->format != s->picture_ptr->format) {
            av_log(s->avctx, AV_LOG_ERROR, "Reference mismatching\n");
            return AVERROR_INVALIDDATA;
        }
    }

    /* XXX: verify len field validity */
    len = get_bits(&s->gb, 16);
    nb_components = get_bits(&s->gb, 8);
    if (nb_components == 0 || nb_components > MAX_COMPONENTS) {
        avpriv_report_missing_feature(s->avctx,
                                      "decode_sos: nb_components (%d)",
                                      nb_components);
        return AVERROR_PATCHWELCOME;
    }
    if (len != 6 + 2 * nb_components) {
        av_log(s->avctx, AV_LOG_ERROR, "decode_sos: invalid len (%d)\n", len);
        return AVERROR_INVALIDDATA;
    }
    for (i = 0; i < nb_components; i++) {
        id = get_bits(&s->gb, 8);
        av_log(s->avctx, AV_LOG_DEBUG, "component: %d\n", id);
        /* find component index */
        for (index = 0; index < s->nb_components; index++)
            if (id == s->component_id[index])
                break;
        if (index == s->nb_components) {
            av_log(s->avctx, AV_LOG_ERROR,
                   "decode_sos: index(%d) out of components\n", index);
            return AVERROR_INVALIDDATA;
        }
        /* Metasoft MJPEG codec has Cb and Cr swapped */
        if (s->avctx->codec_tag == MKTAG('M', 'T', 'S', 'J')
            && nb_components == 3 && s->nb_components == 3 && i)
            index = 3 - i;

        s->quant_sindex[i] = s->quant_index[index];
        s->nb_blocks[i] = s->h_count[index] * s->v_count[index];
        s->h_scount[i]  = s->h_count[index];
        s->v_scount[i]  = s->v_count[index];

        s->comp_index[i] = index;

        s->dc_index[i] = get_bits(&s->gb, 4);
        s->ac_index[i] = get_bits(&s->gb, 4);

        if (s->dc_index[i] <  0 || s->ac_index[i] < 0 ||
            s->dc_index[i] >= 4 || s->ac_index[i] >= 4)
            goto out_of_range;
        if (!s->vlcs[0][s->dc_index[i]].table || !(s->progressive ? s->vlcs[2][s->ac_index[0]].table : s->vlcs[1][s->ac_index[i]].table))
            goto out_of_range;
    }

    predictor = get_bits(&s->gb, 8);       /* JPEG Ss / lossless JPEG predictor /JPEG-LS NEAR */
    ilv = get_bits(&s->gb, 8);             /* JPEG Se / JPEG-LS ILV */
    if(s->avctx->codec_tag != AV_RL32("CJPG")){
        prev_shift      = get_bits(&s->gb, 4); /* Ah */
        point_transform = get_bits(&s->gb, 4); /* Al */
    }else
        prev_shift = point_transform = 0;

    if (nb_components > 1) {
        /* interleaved stream */
        s->mb_width  = (s->width  + s->h_max * block_size - 1) / (s->h_max * block_size);
        s->mb_height = (s->height + s->v_max * block_size - 1) / (s->v_max * block_size);
    } else if (!s->ls) { /* skip this for JPEG-LS */
        h = s->h_max / s->h_scount[0];
        v = s->v_max / s->v_scount[0];
        s->mb_width     = (s->width  + h * block_size - 1) / (h * block_size);
        s->mb_height    = (s->height + v * block_size - 1) / (v * block_size);
        s->nb_blocks[0] = 1;
        s->h_scount[0]  = 1;
        s->v_scount[0]  = 1;
    }

    if (s->avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(s->avctx, AV_LOG_DEBUG, "%s %s p:%d >>:%d ilv:%d bits:%d skip:%d %s comp:%d\n",
               s->lossless ? "lossless" : "sequential DCT", s->rgb ? "RGB" : "",
               predictor, point_transform, ilv, s->bits, s->mjpb_skiptosod,
               s->pegasus_rct ? "PRCT" : (s->rct ? "RCT" : ""), nb_components);


    /* mjpeg-b can have padding bytes between sos and image data, skip them */
    for (i = s->mjpb_skiptosod; i > 0; i--)
        skip_bits(&s->gb, 8);

    for (i = 0; i < nb_components; i++)
        s->last_dc[i] = (4 << s->bits);

    if (s->avctx->hwaccel) {
        int bytes_to_start = get_bits_count(&s->gb) / 8;
        av_assert0(bytes_to_start >= 0 &&
                   s->raw_scan_buffer_size >= bytes_to_start);

        ret = FF_HW_CALL(s->avctx, decode_slice,
                         s->raw_scan_buffer      + bytes_to_start,
                         s->raw_scan_buffer_size - bytes_to_start);
        if (ret < 0)
            return ret;

    } else if (s->lossless) {
        av_assert0(s->picture_ptr == s->picture);
        if (CONFIG_JPEGLS_DECODER && s->ls) {
//            for () {
//            reset_ls_coding_parameters(s, 0);

            if ((ret = ff_jpegls_decode_picture(s, predictor,
                                                point_transform, ilv)) < 0)
                return ret;
        } else {
            if (s->rgb || s->bayer) {
                if ((ret = ljpeg_decode_rgb_scan(s, nb_components, predictor, point_transform)) < 0)
                    return ret;
            } else {
                if ((ret = ljpeg_decode_yuv_scan(s, predictor,
                                                 point_transform,
                                                 nb_components)) < 0)
                    return ret;
            }
        }
    } else {
        if (s->progressive && predictor) {
            av_assert0(s->picture_ptr == s->picture);
            if ((ret = mjpeg_decode_scan_progressive_ac(s, predictor,
                                                        ilv, prev_shift,
                                                        point_transform)) < 0)
                return ret;
        } else {
            if ((ret = mjpeg_decode_scan(s, nb_components,
                                         prev_shift, point_transform,
                                         mb_bitmask, mb_bitmask_size, reference)) < 0)
                return ret;
        }
    }

    emms_c();
    return 0;
 out_of_range:
    av_log(s->avctx, AV_LOG_ERROR, "decode_sos: ac/dc index out of range\n");
    return AVERROR_INVALIDDATA;
}

static int mjpeg_decode_dri(MJpegDecodeContext *s)
{
    if (get_bits(&s->gb, 16) != 4)
        return AVERROR_INVALIDDATA;
    s->restart_interval = get_bits(&s->gb, 16);
    s->restart_count    = 0;
    av_log(s->avctx, AV_LOG_DEBUG, "restart interval: %d\n",
           s->restart_interval);

    return 0;
}

static int mjpeg_decode_app(MJpegDecodeContext *s)
{
    int len, id, i;

    len = get_bits(&s->gb, 16);
    if (len < 6) {
        if (s->bayer) {
            // Pentax K-1 (digital camera) JPEG images embedded in DNG images contain unknown APP0 markers
            av_log(s->avctx, AV_LOG_WARNING, "skipping APPx (len=%"PRId32") for bayer-encoded image\n", len);
            skip_bits(&s->gb, len);
            return 0;
        } else
            return AVERROR_INVALIDDATA;
    }
    if (8 * len > get_bits_left(&s->gb))
        return AVERROR_INVALIDDATA;

    id   = get_bits_long(&s->gb, 32);
    len -= 6;

    if (s->avctx->debug & FF_DEBUG_STARTCODE)
        av_log(s->avctx, AV_LOG_DEBUG, "APPx (%s / %8X) len=%d\n",
               av_fourcc2str(av_bswap32(id)), id, len);

    /* Buggy AVID, it puts EOI only at every 10th frame. */
    /* Also, this fourcc is used by non-avid files too, it holds some
       information, but it's always present in AVID-created files. */
    if (id == AV_RB32("AVI1")) {
        /* structure:
            4bytes      AVI1
            1bytes      polarity
            1bytes      always zero
            4bytes      field_size
            4bytes      field_size_less_padding
        */
            s->buggy_avid = 1;
        i = get_bits(&s->gb, 8); len--;
        av_log(s->avctx, AV_LOG_DEBUG, "polarity %d\n", i);
        goto out;
    }

    if (id == AV_RB32("JFIF")) {
        int t_w, t_h, v1, v2;
        if (len < 8)
            goto out;
        skip_bits(&s->gb, 8); /* the trailing zero-byte */
        v1 = get_bits(&s->gb, 8);
        v2 = get_bits(&s->gb, 8);
        skip_bits(&s->gb, 8);

        s->avctx->sample_aspect_ratio.num = get_bits(&s->gb, 16);
        s->avctx->sample_aspect_ratio.den = get_bits(&s->gb, 16);
        if (   s->avctx->sample_aspect_ratio.num <= 0
            || s->avctx->sample_aspect_ratio.den <= 0) {
            s->avctx->sample_aspect_ratio.num = 0;
            s->avctx->sample_aspect_ratio.den = 1;
        }

        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO,
                   "mjpeg: JFIF header found (version: %x.%x) SAR=%d/%d\n",
                   v1, v2,
                   s->avctx->sample_aspect_ratio.num,
                   s->avctx->sample_aspect_ratio.den);

        len -= 8;
        if (len >= 2) {
            t_w = get_bits(&s->gb, 8);
            t_h = get_bits(&s->gb, 8);
            if (t_w && t_h) {
                /* skip thumbnail */
                if (len -10 - (t_w * t_h * 3) > 0)
                    len -= t_w * t_h * 3;
            }
            len -= 2;
        }
        goto out;
    }

    if (   id == AV_RB32("Adob")
        && len >= 7
        && show_bits(&s->gb, 8) == 'e'
        && show_bits_long(&s->gb, 32) != AV_RB32("e_CM")) {
        skip_bits(&s->gb,  8); /* 'e' */
        skip_bits(&s->gb, 16); /* version */
        skip_bits(&s->gb, 16); /* flags0 */
        skip_bits(&s->gb, 16); /* flags1 */
        s->adobe_transform = get_bits(&s->gb,  8);
        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO, "mjpeg: Adobe header found, transform=%d\n", s->adobe_transform);
        len -= 7;
        goto out;
    }

    if (id == AV_RB32("LJIF")) {
        int rgb = s->rgb;
        int pegasus_rct = s->pegasus_rct;
        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO,
                   "Pegasus lossless jpeg header found\n");
        skip_bits(&s->gb, 16); /* version ? */
        skip_bits(&s->gb, 16); /* unknown always 0? */
        skip_bits(&s->gb, 16); /* unknown always 0? */
        skip_bits(&s->gb, 16); /* unknown always 0? */
        switch (i=get_bits(&s->gb, 8)) {
        case 1:
            rgb         = 1;
            pegasus_rct = 0;
            break;
        case 2:
            rgb         = 1;
            pegasus_rct = 1;
            break;
        default:
            av_log(s->avctx, AV_LOG_ERROR, "unknown colorspace %d\n", i);
        }

        len -= 9;
        if (s->bayer)
            goto out;
        if (s->got_picture)
            if (rgb != s->rgb || pegasus_rct != s->pegasus_rct) {
                av_log(s->avctx, AV_LOG_WARNING, "Mismatching LJIF tag\n");
                goto out;
            }

        s->rgb = rgb;
        s->pegasus_rct = pegasus_rct;

        goto out;
    }
    if (id == AV_RL32("colr") && len > 0) {
        s->colr = get_bits(&s->gb, 8);
        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO, "COLR %d\n", s->colr);
        len --;
        goto out;
    }
    if (id == AV_RL32("xfrm") && len > 0) {
        s->xfrm = get_bits(&s->gb, 8);
        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO, "XFRM %d\n", s->xfrm);
        len --;
        goto out;
    }

    /* JPS extension by VRex */
    if (s->start_code == APP3 && id == AV_RB32("_JPS") && len >= 10) {
        int flags, layout, type;
        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO, "_JPSJPS_\n");

        skip_bits(&s->gb, 32); len -= 4;  /* JPS_ */
        skip_bits(&s->gb, 16); len -= 2;  /* block length */
        skip_bits(&s->gb, 8);             /* reserved */
        flags  = get_bits(&s->gb, 8);
        layout = get_bits(&s->gb, 8);
        type   = get_bits(&s->gb, 8);
        len -= 4;

        av_freep(&s->stereo3d);
        s->stereo3d = av_stereo3d_alloc();
        if (!s->stereo3d) {
            goto out;
        }
        if (type == 0) {
            s->stereo3d->type = AV_STEREO3D_2D;
        } else if (type == 1) {
            switch (layout) {
            case 0x01:
                s->stereo3d->type = AV_STEREO3D_LINES;
                break;
            case 0x02:
                s->stereo3d->type = AV_STEREO3D_SIDEBYSIDE;
                break;
            case 0x03:
                s->stereo3d->type = AV_STEREO3D_TOPBOTTOM;
                break;
            }
            if (!(flags & 0x04)) {
                s->stereo3d->flags = AV_STEREO3D_FLAG_INVERT;
            }
        }
        goto out;
    }

    /* EXIF metadata */
    if (s->start_code == APP1 && id == AV_RB32("Exif") && len >= 2) {
        GetByteContext gbytes;
        int ret, le, ifd_offset, bytes_read;
        const uint8_t *aligned;

        skip_bits(&s->gb, 16); // skip padding
        len -= 2;

        // init byte wise reading
        aligned = align_get_bits(&s->gb);
        bytestream2_init(&gbytes, aligned, len);

        // read TIFF header
        ret = ff_tdecode_header(&gbytes, &le, &ifd_offset);
        if (ret) {
            av_log(s->avctx, AV_LOG_ERROR, "mjpeg: invalid TIFF header in EXIF data\n");
        } else {
            bytestream2_seek(&gbytes, ifd_offset, SEEK_SET);

            // read 0th IFD and store the metadata
            // (return values > 0 indicate the presence of subimage metadata)
            ret = ff_exif_decode_ifd(s->avctx, &gbytes, le, 0, &s->exif_metadata);
            if (ret < 0) {
                av_log(s->avctx, AV_LOG_ERROR, "mjpeg: error decoding EXIF data\n");
            }
        }

        bytes_read = bytestream2_tell(&gbytes);
        skip_bits(&s->gb, bytes_read << 3);
        len -= bytes_read;

        goto out;
    }

    /* Apple MJPEG-A */
    if ((s->start_code == APP1) && (len > (0x28 - 8))) {
        id   = get_bits_long(&s->gb, 32);
        len -= 4;
        /* Apple MJPEG-A */
        if (id == AV_RB32("mjpg")) {
            /* structure:
                4bytes      field size
                4bytes      pad field size
                4bytes      next off
                4bytes      quant off
                4bytes      huff off
                4bytes      image off
                4bytes      scan off
                4bytes      data off
            */
            if (s->avctx->debug & FF_DEBUG_PICT_INFO)
                av_log(s->avctx, AV_LOG_INFO, "mjpeg: Apple MJPEG-A header found\n");
        }
    }

    if (s->start_code == APP2 && id == AV_RB32("ICC_") && len >= 10) {
        int id2;
        unsigned seqno;
        unsigned nummarkers;

        id   = get_bits_long(&s->gb, 32);
        id2  = get_bits(&s->gb, 24);
        len -= 7;
        if (id != AV_RB32("PROF") || id2 != AV_RB24("ILE")) {
            av_log(s->avctx, AV_LOG_WARNING, "Invalid ICC_PROFILE header in APP2\n");
            goto out;
        }

        skip_bits(&s->gb, 8);
        seqno  = get_bits(&s->gb, 8);
        len   -= 2;
        if (seqno == 0) {
            av_log(s->avctx, AV_LOG_WARNING, "Invalid sequence number in APP2\n");
            goto out;
        }

        nummarkers  = get_bits(&s->gb, 8);
        len        -= 1;
        if (nummarkers == 0) {
            av_log(s->avctx, AV_LOG_WARNING, "Invalid number of markers coded in APP2\n");
            goto out;
        } else if (s->iccnum != 0 && nummarkers != s->iccnum) {
            av_log(s->avctx, AV_LOG_WARNING, "Mistmatch in coded number of ICC markers between markers\n");
            goto out;
        } else if (seqno > nummarkers) {
            av_log(s->avctx, AV_LOG_WARNING, "Mismatching sequence number and coded number of ICC markers\n");
            goto out;
        }

        /* Allocate if this is the first APP2 we've seen. */
        if (s->iccnum == 0) {
            if (!FF_ALLOCZ_TYPED_ARRAY(s->iccentries, nummarkers)) {
                av_log(s->avctx, AV_LOG_ERROR, "Could not allocate ICC data arrays\n");
                return AVERROR(ENOMEM);
            }
            s->iccnum = nummarkers;
        }

        if (s->iccentries[seqno - 1].data) {
            av_log(s->avctx, AV_LOG_WARNING, "Duplicate ICC sequence number\n");
            goto out;
        }

        s->iccentries[seqno - 1].length = len;
        s->iccentries[seqno - 1].data   = av_malloc(len);
        if (!s->iccentries[seqno - 1].data) {
            av_log(s->avctx, AV_LOG_ERROR, "Could not allocate ICC data buffer\n");
            return AVERROR(ENOMEM);
        }

        memcpy(s->iccentries[seqno - 1].data, align_get_bits(&s->gb), len);
        skip_bits(&s->gb, len << 3);
        len = 0;
        s->iccread++;

        if (s->iccread > s->iccnum)
            av_log(s->avctx, AV_LOG_WARNING, "Read more ICC markers than are supposed to be coded\n");
    }

out:
    /* slow but needed for extreme adobe jpegs */
    if (len < 0)
        av_log(s->avctx, AV_LOG_ERROR,
               "mjpeg: error, decode_app parser read over the end\n");
    while (--len > 0)
        skip_bits(&s->gb, 8);

    return 0;
}

static int mjpeg_decode_com(MJpegDecodeContext *s)
{
    int len = get_bits(&s->gb, 16);
    if (len >= 2 && 8 * len - 16 <= get_bits_left(&s->gb)) {
        int i;
        char *cbuf = av_malloc(len - 1);
        if (!cbuf)
            return AVERROR(ENOMEM);

        for (i = 0; i < len - 2; i++)
            cbuf[i] = get_bits(&s->gb, 8);
        if (i > 0 && cbuf[i - 1] == '\n')
            cbuf[i - 1] = 0;
        else
            cbuf[i] = 0;

        if (s->avctx->debug & FF_DEBUG_PICT_INFO)
            av_log(s->avctx, AV_LOG_INFO, "comment: '%s'\n", cbuf);

        /* buggy avid, it puts EOI only at every 10th frame */
        if (!strncmp(cbuf, "AVID", 4)) {
            parse_avid(s, cbuf, len);
        } else if (!strcmp(cbuf, "CS=ITU601"))
            s->cs_itu601 = 1;
        else if ((!strncmp(cbuf, "Intel(R) JPEG Library, version 1", 32) && s->avctx->codec_tag) ||
                 (!strncmp(cbuf, "Metasoft MJPEG Codec", 20)))
            s->flipped = 1;
        else if (!strcmp(cbuf, "MULTISCOPE II")) {
            s->avctx->sample_aspect_ratio = (AVRational) { 1, 2 };
            s->multiscope = 2;
        }

        av_free(cbuf);
    }

    return 0;
}

/* return the 8 bit start code value and update the search
   state. Return -1 if no start code found */
static int find_marker(const uint8_t **pbuf_ptr, const uint8_t *buf_end)
{
    const uint8_t *buf_ptr;
    unsigned int v, v2;
    int val;
    int skipped = 0;

    buf_ptr = *pbuf_ptr;
    while (buf_end - buf_ptr > 1) {
        v  = *buf_ptr++;
        v2 = *buf_ptr;
        if ((v == 0xff) && (v2 >= SOF0) && (v2 <= COM) && buf_ptr < buf_end) {
            val = *buf_ptr++;
            goto found;
        }
        skipped++;
    }
    buf_ptr = buf_end;
    val = -1;
found:
    ff_dlog(NULL, "find_marker skipped %d bytes\n", skipped);
    *pbuf_ptr = buf_ptr;
    return val;
}

int ff_mjpeg_find_marker(MJpegDecodeContext *s,
                         const uint8_t **buf_ptr, const uint8_t *buf_end,
                         const uint8_t **unescaped_buf_ptr,
                         int *unescaped_buf_size)
{
    int start_code;
    start_code = find_marker(buf_ptr, buf_end);

    av_fast_padded_malloc(&s->buffer, &s->buffer_size, buf_end - *buf_ptr);
    if (!s->buffer)
        return AVERROR(ENOMEM);

    /* unescape buffer of SOS, use special treatment for JPEG-LS */
    if (start_code == SOS && !s->ls) {
        const uint8_t *src = *buf_ptr;
        const uint8_t *ptr = src;
        uint8_t *dst = s->buffer;

        #define copy_data_segment(skip) do {       \
            ptrdiff_t length = (ptr - src) - (skip);  \
            if (length > 0) {                         \
                memcpy(dst, src, length);             \
                dst += length;                        \
                src = ptr;                            \
            }                                         \
        } while (0)

        if (s->avctx->codec_id == AV_CODEC_ID_THP) {
            ptr = buf_end;
            copy_data_segment(0);
        } else {
            while (ptr < buf_end) {
                uint8_t x = *(ptr++);

                if (x == 0xff) {
                    ptrdiff_t skip = 0;
                    while (ptr < buf_end && x == 0xff) {
                        x = *(ptr++);
                        skip++;
                    }

                    /* 0xFF, 0xFF, ... */
                    if (skip > 1) {
                        copy_data_segment(skip);

                        /* decrement src as it is equal to ptr after the
                         * copy_data_segment macro and we might want to
                         * copy the current value of x later on */
                        src--;
                    }

                    if (x < RST0 || x > RST7) {
                        copy_data_segment(1);
                        if (x)
                            break;
                    }
                }
            }
            if (src < ptr)
                copy_data_segment(0);
        }
        #undef copy_data_segment

        *unescaped_buf_ptr  = s->buffer;
        *unescaped_buf_size = dst - s->buffer;
        memset(s->buffer + *unescaped_buf_size, 0,
               AV_INPUT_BUFFER_PADDING_SIZE);

        av_log(s->avctx, AV_LOG_DEBUG, "escaping removed %"PTRDIFF_SPECIFIER" bytes\n",
               (buf_end - *buf_ptr) - (dst - s->buffer));
    } else if (start_code == SOS && s->ls) {
        const uint8_t *src = *buf_ptr;
        uint8_t *dst  = s->buffer;
        int bit_count = 0;
        int t = 0, b = 0;
        PutBitContext pb;

        /* find marker */
        while (src + t < buf_end) {
            uint8_t x = src[t++];
            if (x == 0xff) {
                while ((src + t < buf_end) && x == 0xff)
                    x = src[t++];
                if (x & 0x80) {
                    t -= FFMIN(2, t);
                    break;
                }
            }
        }
        bit_count = t * 8;
        init_put_bits(&pb, dst, t);

        /* unescape bitstream */
        while (b < t) {
            uint8_t x = src[b++];
            put_bits(&pb, 8, x);
            if (x == 0xFF && b < t) {
                x = src[b++];
                if (x & 0x80) {
                    av_log(s->avctx, AV_LOG_WARNING, "Invalid escape sequence\n");
                    x &= 0x7f;
                }
                put_bits(&pb, 7, x);
                bit_count--;
            }
        }
        flush_put_bits(&pb);

        *unescaped_buf_ptr  = dst;
        *unescaped_buf_size = (bit_count + 7) >> 3;
        memset(s->buffer + *unescaped_buf_size, 0,
               AV_INPUT_BUFFER_PADDING_SIZE);
    } else {
        *unescaped_buf_ptr  = *buf_ptr;
        *unescaped_buf_size = buf_end - *buf_ptr;
    }

    return start_code;
}

static void reset_icc_profile(MJpegDecodeContext *s)
{
    int i;

    if (s->iccentries) {
        for (i = 0; i < s->iccnum; i++)
            av_freep(&s->iccentries[i].data);
        av_freep(&s->iccentries);
    }

    s->iccread = 0;
    s->iccnum  = 0;
}

int ff_mjpeg_decode_frame_from_buf(AVCodecContext *avctx, AVFrame *frame,
                                   int *got_frame, const AVPacket *avpkt,
                                   const uint8_t *buf, const int buf_size)
{
    MJpegDecodeContext *s = avctx->priv_data;
    const uint8_t *buf_end, *buf_ptr;
    const uint8_t *unescaped_buf_ptr;
    int hshift, vshift;
    int unescaped_buf_size;
    int start_code;
    int index;
    int ret = 0;
    int is16bit;
    AVDictionaryEntry *e = NULL;

    s->force_pal8 = 0;

    s->buf_size = buf_size;

    av_dict_free(&s->exif_metadata);
    av_freep(&s->stereo3d);
    s->adobe_transform = -1;

    if (s->iccnum != 0)
        reset_icc_profile(s);

redo_for_pal8:
    buf_ptr = buf;
    buf_end = buf + buf_size;
    while (buf_ptr < buf_end) {
        /* find start next marker */
        start_code = ff_mjpeg_find_marker(s, &buf_ptr, buf_end,
                                          &unescaped_buf_ptr,
                                          &unescaped_buf_size);
        /* EOF */
        if (start_code < 0) {
            break;
        } else if (unescaped_buf_size > INT_MAX / 8) {
            av_log(avctx, AV_LOG_ERROR,
                   "MJPEG packet 0x%x too big (%d/%d), corrupt data?\n",
                   start_code, unescaped_buf_size, buf_size);
            return AVERROR_INVALIDDATA;
        }
        av_log(avctx, AV_LOG_DEBUG, "marker=%x avail_size_in_buf=%"PTRDIFF_SPECIFIER"\n",
               start_code, buf_end - buf_ptr);

        ret = init_get_bits8(&s->gb, unescaped_buf_ptr, unescaped_buf_size);

        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "invalid buffer\n");
            goto fail;
        }

        s->start_code = start_code;
        if (avctx->debug & FF_DEBUG_STARTCODE)
            av_log(avctx, AV_LOG_DEBUG, "startcode: %X\n", start_code);

        /* process markers */
        if (start_code >= RST0 && start_code <= RST7) {
            av_log(avctx, AV_LOG_DEBUG,
                   "restart marker: %d\n", start_code & 0x0f);
            /* APP fields */
        } else if (start_code >= APP0 && start_code <= APP15) {
            if ((ret = mjpeg_decode_app(s)) < 0)
                av_log(avctx, AV_LOG_ERROR, "unable to decode APP fields: %s\n",
                       av_err2str(ret));
            /* Comment */
        } else if (start_code == COM) {
            ret = mjpeg_decode_com(s);
            if (ret < 0)
                return ret;
        } else if (start_code == DQT) {
            ret = ff_mjpeg_decode_dqt(s);
            if (ret < 0)
                return ret;
        }

        ret = -1;

        if (!CONFIG_JPEGLS_DECODER &&
            (start_code == SOF48 || start_code == LSE)) {
            av_log(avctx, AV_LOG_ERROR, "JPEG-LS support not enabled.\n");
            return AVERROR(ENOSYS);
        }

        if (avctx->skip_frame == AVDISCARD_ALL) {
            switch(start_code) {
            case SOF0:
            case SOF1:
            case SOF2:
            case SOF3:
            case SOF48:
            case SOI:
            case SOS:
            case EOI:
                break;
            default:
                goto skip;
            }
        }

        switch (start_code) {
        case SOI:
            s->restart_interval = 0;
            s->restart_count    = 0;
            s->raw_image_buffer      = buf_ptr;
            s->raw_image_buffer_size = buf_end - buf_ptr;
            /* nothing to do on SOI */
            break;
        case DHT:
            if ((ret = ff_mjpeg_decode_dht(s)) < 0) {
                av_log(avctx, AV_LOG_ERROR, "huffman table decode error\n");
                goto fail;
            }
            break;
        case SOF0:
        case SOF1:
            if (start_code == SOF0)
                avctx->profile = AV_PROFILE_MJPEG_HUFFMAN_BASELINE_DCT;
            else
                avctx->profile = AV_PROFILE_MJPEG_HUFFMAN_EXTENDED_SEQUENTIAL_DCT;
            s->lossless    = 0;
            s->ls          = 0;
            s->progressive = 0;
            if ((ret = ff_mjpeg_decode_sof(s)) < 0)
                goto fail;
            break;
        case SOF2:
            avctx->profile = AV_PROFILE_MJPEG_HUFFMAN_PROGRESSIVE_DCT;
            s->lossless    = 0;
            s->ls          = 0;
            s->progressive = 1;
            if ((ret = ff_mjpeg_decode_sof(s)) < 0)
                goto fail;
            break;
        case SOF3:
            avctx->profile     = AV_PROFILE_MJPEG_HUFFMAN_LOSSLESS;
#if FF_API_CODEC_PROPS
FF_DISABLE_DEPRECATION_WARNINGS
            avctx->properties |= FF_CODEC_PROPERTY_LOSSLESS;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
            s->lossless    = 1;
            s->ls          = 0;
            s->progressive = 0;
            if ((ret = ff_mjpeg_decode_sof(s)) < 0)
                goto fail;
            break;
        case SOF48:
            avctx->profile     = AV_PROFILE_MJPEG_JPEG_LS;
#if FF_API_CODEC_PROPS
FF_DISABLE_DEPRECATION_WARNINGS
            avctx->properties |= FF_CODEC_PROPERTY_LOSSLESS;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
            s->lossless    = 1;
            s->ls          = 1;
            s->progressive = 0;
            if ((ret = ff_mjpeg_decode_sof(s)) < 0)
                goto fail;
            break;
        case LSE:
            if (!CONFIG_JPEGLS_DECODER ||
                (ret = ff_jpegls_decode_lse(s)) < 0)
                goto fail;
            if (ret == 1)
                goto redo_for_pal8;
            break;
        case EOI:
eoi_parser:
            if (!avctx->hwaccel && avctx->skip_frame != AVDISCARD_ALL &&
                s->progressive && s->cur_scan && s->got_picture)
                mjpeg_idct_scan_progressive_ac(s);
            s->cur_scan = 0;
            if (!s->got_picture) {
                av_log(avctx, AV_LOG_WARNING,
                       "Found EOI before any SOF, ignoring\n");
                break;
            }

            if (avctx->skip_frame == AVDISCARD_ALL) {
                s->got_picture = 0;
                goto the_end_no_picture;
            }
            if (avctx->hwaccel) {
                ret = FF_HW_SIMPLE_CALL(avctx, end_frame);
                if (ret < 0)
                    return ret;

                av_freep(&s->hwaccel_picture_private);
            }

            av_frame_move_ref(frame, s->picture_ptr);

            if (s->lossless)
                frame->flags |= AV_FRAME_FLAG_LOSSLESS;
            *got_frame = 1;
            s->got_picture = 0;

            if (!s->lossless && avctx->debug & FF_DEBUG_QP) {
                int qp = FFMAX3(s->qscale[0],
                                s->qscale[1],
                                s->qscale[2]);

                av_log(avctx, AV_LOG_DEBUG, "QP: %d\n", qp);
            }

            goto the_end;
        case SOS:
            s->raw_scan_buffer      = buf_ptr;
            s->raw_scan_buffer_size = buf_end - buf_ptr;

            s->cur_scan++;
            if (avctx->skip_frame == AVDISCARD_ALL) {
                skip_bits(&s->gb, get_bits_left(&s->gb));
                break;
            }

            if ((ret = ff_mjpeg_decode_sos(s, NULL, 0, NULL)) < 0 &&
                (avctx->err_recognition & AV_EF_EXPLODE))
                goto fail;
            break;
        case DRI:
            if ((ret = mjpeg_decode_dri(s)) < 0)
                return ret;
            break;
        case SOF5:
        case SOF6:
        case SOF7:
        case SOF9:
        case SOF10:
        case SOF11:
        case SOF13:
        case SOF14:
        case SOF15:
        case JPG:
            av_log(avctx, AV_LOG_ERROR,
                   "mjpeg: unsupported coding type (%x)\n", start_code);
            break;
        }

skip:
        /* eof process start code */
        buf_ptr += (get_bits_count(&s->gb) + 7) / 8;
        av_log(avctx, AV_LOG_DEBUG,
               "marker parser used %d bytes (%d bits)\n",
               (get_bits_count(&s->gb) + 7) / 8, get_bits_count(&s->gb));
    }
    if (s->got_picture && s->cur_scan) {
        av_log(avctx, AV_LOG_WARNING, "EOI missing, emulating\n");
        goto eoi_parser;
    }
    av_log(avctx, AV_LOG_FATAL, "No JPEG data found in image\n");
    return AVERROR_INVALIDDATA;
fail:
    s->got_picture = 0;
    return ret;
the_end:

    is16bit = av_pix_fmt_desc_get(avctx->pix_fmt)->comp[0].step > 1;

    if (AV_RB32(s->upscale_h)) {
        int p;
        av_assert0(avctx->pix_fmt == AV_PIX_FMT_YUVJ444P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ440P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV440P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA444P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ422P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV422P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ420P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV420P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV420P16||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA420P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA420P16||
                   avctx->pix_fmt == AV_PIX_FMT_GBRP     ||
                   avctx->pix_fmt == AV_PIX_FMT_GBRAP
                  );
        ret = av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &hshift, &vshift);
        if (ret)
            return ret;

        av_assert0(s->nb_components == av_pix_fmt_count_planes(frame->format));
        for (p = 0; p<s->nb_components; p++) {
            uint8_t *line = frame->data[p];
            int w = s->width;
            int h = s->height;
            if (!s->upscale_h[p])
                continue;
            if (p==1 || p==2) {
                w = AV_CEIL_RSHIFT(w, hshift);
                h = AV_CEIL_RSHIFT(h, vshift);
            }
            if (s->upscale_v[p] == 1)
                h = (h+1)>>1;
            av_assert0(w > 0);
            for (int i = 0; i < h; i++) {
                if (s->upscale_h[p] == 1) {
                    if (is16bit) ((uint16_t*)line)[w - 1] = ((uint16_t*)line)[(w - 1) / 2];
                    else                      line[w - 1] = line[(w - 1) / 2];
                    for (index = w - 2; index > 0; index--) {
                        if (is16bit)
                            ((uint16_t*)line)[index] = (((uint16_t*)line)[index / 2] + ((uint16_t*)line)[(index + 1) / 2]) >> 1;
                        else
                            line[index] = (line[index / 2] + line[(index + 1) / 2]) >> 1;
                    }
                } else if (s->upscale_h[p] == 2) {
                    if (is16bit) {
                        ((uint16_t*)line)[w - 1] = ((uint16_t*)line)[(w - 1) / 3];
                        if (w > 1)
                            ((uint16_t*)line)[w - 2] = ((uint16_t*)line)[w - 1];
                    } else {
                        line[w - 1] = line[(w - 1) / 3];
                        if (w > 1)
                            line[w - 2] = line[w - 1];
                    }
                    for (index = w - 3; index > 0; index--) {
                        line[index] = (line[index / 3] + line[(index + 1) / 3] + line[(index + 2) / 3] + 1) / 3;
                    }
                } else if (s->upscale_h[p] == 4){
                    if (is16bit) {
                        uint16_t *line16 = (uint16_t *) line;
                        line16[w - 1] = line16[(w - 1) >> 2];
                        if (w > 1)
                            line16[w - 2] = (line16[(w - 1) >> 2] * 3 + line16[(w - 2) >> 2]) >> 2;
                        if (w > 2)
                            line16[w - 3] = (line16[(w - 1) >> 2] + line16[(w - 2) >> 2]) >> 1;
                    } else {
                        line[w - 1] = line[(w - 1) >> 2];
                        if (w > 1)
                            line[w - 2] = (line[(w - 1) >> 2] * 3 + line[(w - 2) >> 2]) >> 2;
                        if (w > 2)
                            line[w - 3] = (line[(w - 1) >> 2] + line[(w - 2) >> 2]) >> 1;
                    }
                    for (index = w - 4; index > 0; index--)
                        line[index] = (line[(index + 3) >> 2] + line[(index + 2) >> 2]
                            + line[(index + 1) >> 2] + line[index >> 2]) >> 2;
                }
                line += s->linesize[p];
            }
        }
    }
    if (AV_RB32(s->upscale_v)) {
        int p;
        av_assert0(avctx->pix_fmt == AV_PIX_FMT_YUVJ444P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ422P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV422P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ420P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV420P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV440P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVJ440P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA444P ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA420P  ||
                   avctx->pix_fmt == AV_PIX_FMT_YUVA420P16||
                   avctx->pix_fmt == AV_PIX_FMT_GBRP     ||
                   avctx->pix_fmt == AV_PIX_FMT_GBRAP
                   );
        ret = av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &hshift, &vshift);
        if (ret)
            return ret;

        av_assert0(s->nb_components == av_pix_fmt_count_planes(frame->format));
        for (p = 0; p < s->nb_components; p++) {
            uint8_t *dst;
            int w = s->width;
            int h = s->height;
            if (!s->upscale_v[p])
                continue;
            if (p==1 || p==2) {
                w = AV_CEIL_RSHIFT(w, hshift);
                h = AV_CEIL_RSHIFT(h, vshift);
            }
            dst = &((uint8_t *)frame->data[p])[(h - 1) * s->linesize[p]];
            for (int i = h - 1; i; i--) {
                uint8_t *src1 = &((uint8_t *)frame->data[p])[i * s->upscale_v[p] / (s->upscale_v[p] + 1) * s->linesize[p]];
                uint8_t *src2 = &((uint8_t *)frame->data[p])[(i + 1) * s->upscale_v[p] / (s->upscale_v[p] + 1) * s->linesize[p]];
                if (s->upscale_v[p] != 2 && (src1 == src2 || i == h - 1)) {
                    memcpy(dst, src1, w);
                } else {
                    for (index = 0; index < w; index++)
                        dst[index] = (src1[index] + src2[index]) >> 1;
                }
                dst -= s->linesize[p];
            }
        }
    }
    if (s->flipped && !s->rgb) {
        ret = av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &hshift, &vshift);
        if (ret)
            return ret;

        av_assert0(s->nb_components == av_pix_fmt_count_planes(frame->format));
        for (index=0; index<s->nb_components; index++) {
            int h = frame->height;
            if (index && index < 3)
                h = AV_CEIL_RSHIFT(h, vshift);
            if (frame->data[index]) {
                frame->data[index]     += (h - 1) * frame->linesize[index];
                frame->linesize[index] *= -1;
            }
        }
    }

    if (avctx->pix_fmt == AV_PIX_FMT_GBRP) {
        av_assert0(s->nb_components == 3);
        FFSWAP(uint8_t *, frame->data[0], frame->data[2]);
        FFSWAP(uint8_t *, frame->data[0], frame->data[1]);
        FFSWAP(int, frame->linesize[0], frame->linesize[2]);
        FFSWAP(int, frame->linesize[0], frame->linesize[1]);
    }

    if (s->adobe_transform == 0 && avctx->pix_fmt == AV_PIX_FMT_GBRAP) {
        int w = frame->width;
        int h = frame->height;
        av_assert0(s->nb_components == 4);
        for (int i = 0; i < h; i++) {
            int j;
            uint8_t *dst[4];
            for (index=0; index<4; index++) {
                dst[index] =   frame->data[index]
                             + frame->linesize[index]*i;
            }
            for (j=0; j<w; j++) {
                int k = dst[3][j];
                int r = dst[0][j] * k;
                int g = dst[1][j] * k;
                int b = dst[2][j] * k;
                dst[0][j] = g*257 >> 16;
                dst[1][j] = b*257 >> 16;
                dst[2][j] = r*257 >> 16;
            }
            memset(dst[3], 255, w);
        }
    }
    if (s->adobe_transform == 2 && avctx->pix_fmt == AV_PIX_FMT_YUVA444P) {
        int w = frame->width;
        int h = frame->height;
        av_assert0(s->nb_components == 4);
        for (int i = 0; i < h; i++) {
            int j;
            uint8_t *dst[4];
            for (index=0; index<4; index++) {
                dst[index] =   frame->data[index]
                             + frame->linesize[index]*i;
            }
            for (j=0; j<w; j++) {
                int k = dst[3][j];
                int r = (255 - dst[0][j]) * k;
                int g = (128 - dst[1][j]) * k;
                int b = (128 - dst[2][j]) * k;
                dst[0][j] = r*257 >> 16;
                dst[1][j] = (g*257 >> 16) + 128;
                dst[2][j] = (b*257 >> 16) + 128;
            }
            memset(dst[3], 255, w);
        }
    }

    if (s->stereo3d) {
        AVStereo3D *stereo = av_stereo3d_create_side_data(frame);
        if (stereo) {
            stereo->type  = s->stereo3d->type;
            stereo->flags = s->stereo3d->flags;
        }
        av_freep(&s->stereo3d);
    }

    if (s->iccnum != 0 && s->iccnum == s->iccread) {
        AVFrameSideData *sd;
        size_t offset = 0;
        int total_size = 0;

        /* Sum size of all parts. */
        for (int i = 0; i < s->iccnum; i++)
            total_size += s->iccentries[i].length;

        ret = ff_frame_new_side_data(avctx, frame, AV_FRAME_DATA_ICC_PROFILE, total_size, &sd);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Could not allocate frame side data\n");
            return ret;
        }

        if (sd) {
            /* Reassemble the parts, which are now in-order. */
            for (int i = 0; i < s->iccnum; i++) {
                memcpy(sd->data + offset, s->iccentries[i].data, s->iccentries[i].length);
                offset += s->iccentries[i].length;
            }
        }
    }

    if (e = av_dict_get(s->exif_metadata, "Orientation", e, AV_DICT_IGNORE_SUFFIX)) {
        char *value = e->value + strspn(e->value, " \n\t\r"), *endptr;
        int orientation = strtol(value, &endptr, 0);

        if (!*endptr) {
            AVFrameSideData *sd = NULL;

            if (orientation >= 2 && orientation <= 8) {
                int32_t *matrix;

                sd = av_frame_new_side_data(frame, AV_FRAME_DATA_DISPLAYMATRIX, sizeof(int32_t) * 9);
                if (!sd) {
                    av_log(avctx, AV_LOG_ERROR, "Could not allocate frame side data\n");
                    return AVERROR(ENOMEM);
                }

                matrix = (int32_t *)sd->data;

                switch (orientation) {
                case 2:
                    av_display_rotation_set(matrix, 0.0);
                    av_display_matrix_flip(matrix, 1, 0);
                    break;
                case 3:
                    av_display_rotation_set(matrix, 180.0);
                    break;
                case 4:
                    av_display_rotation_set(matrix, 180.0);
                    av_display_matrix_flip(matrix, 1, 0);
                    break;
                case 5:
                    av_display_rotation_set(matrix, 90.0);
                    av_display_matrix_flip(matrix, 1, 0);
                    break;
                case 6:
                    av_display_rotation_set(matrix, 90.0);
                    break;
                case 7:
                    av_display_rotation_set(matrix, -90.0);
                    av_display_matrix_flip(matrix, 1, 0);
                    break;
                case 8:
                    av_display_rotation_set(matrix, -90.0);
                    break;
                default:
                    av_assert0(0);
                }
            }
        }
    }

    av_dict_copy(&frame->metadata, s->exif_metadata, 0);
    av_dict_free(&s->exif_metadata);

    if (avctx->codec_id != AV_CODEC_ID_SMVJPEG &&
        (avctx->codec_tag == MKTAG('A', 'V', 'R', 'n') ||
         avctx->codec_tag == MKTAG('A', 'V', 'D', 'J')) &&
        avctx->coded_height > s->orig_height) {
        frame->height   = AV_CEIL_RSHIFT(avctx->coded_height, avctx->lowres);
        frame->crop_top = frame->height - avctx->height;
    }

the_end_no_picture:
    av_log(avctx, AV_LOG_DEBUG, "decode frame unused %"PTRDIFF_SPECIFIER" bytes\n",
           buf_end - buf_ptr);
    return buf_ptr - buf;
}

int ff_mjpeg_decode_frame(AVCodecContext *avctx, AVFrame *frame, int *got_frame,
                          AVPacket *avpkt)
{
    return ff_mjpeg_decode_frame_from_buf(avctx, frame, got_frame,
                                          avpkt, avpkt->data, avpkt->size);
}


/* mxpeg may call the following function (with a blank MJpegDecodeContext)
 * even without having called ff_mjpeg_decode_init(). */
av_cold int ff_mjpeg_decode_end(AVCodecContext *avctx)
{
    MJpegDecodeContext *s = avctx->priv_data;
    int i, j;

    av_frame_free(&s->picture);
    s->picture_ptr = NULL;
    av_frame_free(&s->smv_frame);

    av_freep(&s->buffer);
    av_freep(&s->stereo3d);
    av_freep(&s->ljpeg_buffer);
    s->ljpeg_buffer_size = 0;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++)
            ff_vlc_free(&s->vlcs[i][j]);
    }
    for (i = 0; i < MAX_COMPONENTS; i++) {
        av_freep(&s->blocks[i]);
        av_freep(&s->last_nnz[i]);
    }
    av_dict_free(&s->exif_metadata);

    reset_icc_profile(s);

    av_freep(&s->hwaccel_picture_private);
    av_freep(&s->jls_state);

    return 0;
}

static void decode_flush(AVCodecContext *avctx)
{
    MJpegDecodeContext *s = avctx->priv_data;
    s->got_picture = 0;

    s->smv_next_frame = 0;
    av_frame_unref(s->smv_frame);
}

#if CONFIG_MJPEG_DECODER
#define OFFSET(x) offsetof(MJpegDecodeContext, x)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "extern_huff", "Use external huffman table.",
      OFFSET(extern_huff), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VD },
    { NULL },
};

static const AVClass mjpegdec_class = {
    .class_name = "MJPEG decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_mjpeg_decoder = {
    .p.name         = "mjpeg",
    CODEC_LONG_NAME("MJPEG (Motion JPEG)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_MJPEG,
    .priv_data_size = sizeof(MJpegDecodeContext),
    .init           = ff_mjpeg_decode_init,
    .close          = ff_mjpeg_decode_end,
    FF_CODEC_DECODE_CB(ff_mjpeg_decode_frame),
    .flush          = decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS,
    .p.max_lowres   = 3,
    .p.priv_class   = &mjpegdec_class,
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_mjpeg_profiles),
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP |
                      FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM |
                      FF_CODEC_CAP_ICC_PROFILES,
    .hw_configs     = (const AVCodecHWConfigInternal *const []) {
#if CONFIG_MJPEG_NVDEC_HWACCEL
                        HWACCEL_NVDEC(mjpeg),
#endif
#if CONFIG_MJPEG_VAAPI_HWACCEL
                        HWACCEL_VAAPI(mjpeg),
#endif
                        NULL
                    },
};
#endif
#if CONFIG_THP_DECODER
const FFCodec ff_thp_decoder = {
    .p.name         = "thp",
    CODEC_LONG_NAME("Nintendo Gamecube THP video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_THP,
    .priv_data_size = sizeof(MJpegDecodeContext),
    .init           = ff_mjpeg_decode_init,
    .close          = ff_mjpeg_decode_end,
    FF_CODEC_DECODE_CB(ff_mjpeg_decode_frame),
    .flush          = decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .p.max_lowres   = 3,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
#endif

#if CONFIG_SMVJPEG_DECODER
// SMV JPEG just stacks several output frames into one JPEG picture
// we handle that by setting up the cropping parameters appropriately
static void smv_process_frame(AVCodecContext *avctx, AVFrame *frame)
{
    MJpegDecodeContext *s = avctx->priv_data;

    av_assert0((s->smv_next_frame + 1) * avctx->height <= avctx->coded_height);

    frame->width       = avctx->coded_width;
    frame->height      = avctx->coded_height;
    frame->crop_top    = FFMIN(s->smv_next_frame * avctx->height, frame->height);
    frame->crop_bottom = frame->height - (s->smv_next_frame + 1) * avctx->height;

    if (s->smv_frame->pts != AV_NOPTS_VALUE)
        s->smv_frame->pts += s->smv_frame->duration;
    s->smv_next_frame = (s->smv_next_frame + 1) % s->smv_frames_per_jpeg;

    if (s->smv_next_frame == 0)
        av_frame_unref(s->smv_frame);
}

static int smvjpeg_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    MJpegDecodeContext *s = avctx->priv_data;
    AVPacket *const pkt = avctx->internal->in_pkt;
    int got_frame = 0;
    int ret;

    if (s->smv_next_frame > 0)
        goto return_frame;

    ret = ff_decode_get_packet(avctx, pkt);
    if (ret < 0)
        return ret;

    av_frame_unref(s->smv_frame);

    ret = ff_mjpeg_decode_frame(avctx, s->smv_frame, &got_frame, pkt);
    s->smv_frame->pkt_dts = pkt->dts;
    av_packet_unref(pkt);
    if (ret < 0)
        return ret;

    if (!got_frame)
        return AVERROR(EAGAIN);

    // packet duration covers all the frames in the packet
    s->smv_frame->duration /= s->smv_frames_per_jpeg;

return_frame:
    av_assert0(s->smv_frame->buf[0]);
    ret = av_frame_ref(frame, s->smv_frame);
    if (ret < 0)
        return ret;

    smv_process_frame(avctx, frame);
    return 0;
}

const FFCodec ff_smvjpeg_decoder = {
    .p.name         = "smvjpeg",
    CODEC_LONG_NAME("SMV JPEG"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_SMVJPEG,
    .priv_data_size = sizeof(MJpegDecodeContext),
    .init           = ff_mjpeg_decode_init,
    .close          = ff_mjpeg_decode_end,
    FF_CODEC_RECEIVE_FRAME_CB(smvjpeg_receive_frame),
    .flush          = decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_EXPORTS_CROPPING |
                      FF_CODEC_CAP_INIT_CLEANUP,
};
#endif
