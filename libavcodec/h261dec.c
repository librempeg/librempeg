/*
 * H.261 decoder
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2004 Maarten Daniels
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
 * H.261 decoder.
 */

#include "libavutil/mem_internal.h"
#include "libavutil/thread.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "mpeg_er.h"
#include "mpegutils.h"
#include "mpegvideo.h"
#include "mpegvideodec.h"
#include "h261.h"

#define SLICE_OK         0
#define SLICE_ERROR     -1
#define SLICE_END       -2 ///<end marker found

#define H261_MBA_VLC_BITS 8
#define H261_MTYPE_VLC_BITS 6
#define H261_MV_VLC_BITS 7
#define H261_CBP_VLC_BITS 9
#define TCOEFF_VLC_BITS 9
#define MBA_STUFFING 33
#define MBA_STARTCODE 34

static VLCElem h261_mba_vlc[540];
static VLCElem h261_mtype_vlc[80];
static VLCElem h261_mv_vlc[144];
static VLCElem h261_cbp_vlc[512];

typedef struct H261DecContext {
    MpegEncContext s;

    GetBitContext gb;

    H261Context common;

    int current_mba;
    int mba_diff;
    int current_mv_x;
    int current_mv_y;
    int gob_number;
    int gob_start_code_skipped; // 1 if gob start code is already read before gob header is read

    DECLARE_ALIGNED_32(int16_t, block)[6][64];
} H261DecContext;

static av_cold void h261_decode_init_static(void)
{
    VLC_INIT_STATIC_TABLE(h261_mba_vlc, H261_MBA_VLC_BITS, 35,
                          ff_h261_mba_bits, 1, 1,
                          ff_h261_mba_code, 1, 1, 0);
    VLC_INIT_STATIC_SPARSE_TABLE(h261_mtype_vlc, H261_MTYPE_VLC_BITS, 10,
                                 ff_h261_mtype_bits, 1, 1,
                                 ff_h261_mtype_code, 1, 1,
                                 ff_h261_mtype_map,  2, 2, 0);
    VLC_INIT_STATIC_TABLE(h261_mv_vlc, H261_MV_VLC_BITS, 17,
                          &ff_h261_mv_tab[0][1], 2, 1,
                          &ff_h261_mv_tab[0][0], 2, 1, 0);
    VLC_INIT_STATIC_TABLE(h261_cbp_vlc, H261_CBP_VLC_BITS, 63,
                          &ff_h261_cbp_tab[0][1], 2, 1,
                          &ff_h261_cbp_tab[0][0], 2, 1, 0);
    INIT_FIRST_VLC_RL(ff_h261_rl_tcoeff, 552);
}

static av_cold int h261_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    H261DecContext *const h = avctx->priv_data;
    MpegEncContext *const s = &h->s;
    int ret;

    avctx->framerate = (AVRational) { 30000, 1001 };

    /* The H.261 analog of intra/key frames is setting the freeze picture release flag,
     * but this does not guarantee that the frame uses intra-only encoding,
     * so we still need to allocate dummy frames. So set pict_type to P here
     * for all frames and override it after having decoded the frame. */
    s->pict_type = AV_PICTURE_TYPE_P;

    s->private_ctx = &h->common;
    // set defaults
    ret = ff_mpv_decode_init(s, avctx);
    if (ret < 0)
        return ret;

    s->out_format  = FMT_H261;
    s->low_delay   = 1;
    avctx->pix_fmt = AV_PIX_FMT_YUV420P;

    ff_thread_once(&init_static_once, h261_decode_init_static);

    return 0;
}

static inline void h261_init_dest(MpegEncContext *s)
{
    const unsigned block_size = 8 >> s->avctx->lowres;
    ff_init_block_index(s);
    s->dest[0] += 2 * block_size;
    s->dest[1] += block_size;
    s->dest[2] += block_size;
}

/**
 * Decode the group of blocks header or slice header.
 * @return <0 if an error occurred
 */
static int h261_decode_gob_header(H261DecContext *h)
{
    unsigned int val;
    MpegEncContext *const s = &h->s;

    if (!h->gob_start_code_skipped) {
        /* Check for GOB Start Code */
        val = show_bits(&h->gb, 15);
        if (val)
            return -1;

        /* We have a GBSC */
        skip_bits(&h->gb, 16);
    }

    h->gob_start_code_skipped = 0;

    h->gob_number = get_bits(&h->gb, 4); /* GN */
    s->qscale     = get_bits(&h->gb, 5); /* GQUANT */

    /* Check if gob_number is valid */
    if (s->mb_height == 18) { // CIF
        if ((h->gob_number <= 0) || (h->gob_number > 12))
            return -1;
    } else { // QCIF
        if ((h->gob_number != 1) && (h->gob_number != 3) &&
            (h->gob_number != 5))
            return -1;
    }

    /* GEI */
    if (skip_1stop_8data_bits(&h->gb) < 0)
        return AVERROR_INVALIDDATA;

    if (s->qscale == 0) {
        av_log(s->avctx, AV_LOG_ERROR, "qscale has forbidden 0 value\n");
        if (s->avctx->err_recognition & (AV_EF_BITSTREAM | AV_EF_COMPLIANT))
            return -1;
        s->qscale = 1;
    }

    /* For the first transmitted macroblock in a GOB, MBA is the absolute
     * address. For subsequent macroblocks, MBA is the difference between
     * the absolute addresses of the macroblock and the last transmitted
     * macroblock. */
    h->current_mba = 0;
    h->mba_diff    = 0;

    return 0;
}

/**
 * Decode skipped macroblocks.
 * @return 0
 */
static int h261_decode_mb_skipped(H261DecContext *h, int mba1, int mba2)
{
    MpegEncContext *const s = &h->s;
    int i;

    s->mb_intra = 0;

    for (i = mba1; i < mba2; i++) {
        int j, xy;

        s->mb_x = ((h->gob_number - 1) % 2) * 11 + i % 11;
        s->mb_y = ((h->gob_number - 1) / 2) * 3 + i / 11;
        xy      = s->mb_x + s->mb_y * s->mb_stride;
        h261_init_dest(s);

        for (j = 0; j < 6; j++)
            s->block_last_index[j] = -1;

        s->mv_dir                      = MV_DIR_FORWARD;
        s->mv_type                     = MV_TYPE_16X16;
        s->cur_pic.mb_type[xy] = MB_TYPE_SKIP | MB_TYPE_16x16 | MB_TYPE_FORWARD_MV;
        s->mv[0][0][0]                 = 0;
        s->mv[0][0][1]                 = 0;
        s->mb_skipped                  = 1;
        h->common.mtype               &= ~MB_TYPE_H261_FIL;

        if (s->cur_pic.motion_val[0]) {
            int b_stride = 2*s->mb_width + 1;
            int b_xy     = 2 * s->mb_x + (2 * s->mb_y) * b_stride;
            s->cur_pic.motion_val[0][b_xy][0] = s->mv[0][0][0];
            s->cur_pic.motion_val[0][b_xy][1] = s->mv[0][0][1];
        }

        ff_mpv_reconstruct_mb(s, h->block);
    }

    return 0;
}

static int decode_mv_component(GetBitContext *gb, int v)
{
    int mv_diff = get_vlc2(gb, h261_mv_vlc, H261_MV_VLC_BITS, 2);

    /* check if mv_diff is valid */
    if (mv_diff < 0)
        return v;

    if (mv_diff && get_bits1(gb))
        mv_diff = -mv_diff;

    v += mv_diff;
    if (v <= -16)
        v += 32;
    else if (v >= 16)
        v -= 32;

    return v;
}

/**
 * Decode a macroblock.
 * @return <0 if an error occurred
 */
static int h261_decode_block(H261DecContext *h, int16_t *block, int n, int coded)
{
    MpegEncContext *const s = &h->s;
    int level, i, j, run;
    const RLTable *rl = &ff_h261_rl_tcoeff;
    const uint8_t *scan_table;
    const int qmul = s->qscale << 1, qadd = (s->qscale - 1) | 1;

    /* For the variable length encoding there are two code tables, one being
     * used for the first transmitted LEVEL in INTER, INTER + MC and
     * INTER + MC + FIL blocks, the second for all other LEVELs except the
     * first one in INTRA blocks which is fixed length coded with 8 bits.
     * NOTE: The two code tables only differ in one VLC so we handle that
     * manually. */
    scan_table = s->intra_scantable.permutated;
    if (s->mb_intra) {
        /* DC coef */
        level = get_bits(&h->gb, 8);
        // 0 (00000000b) and -128 (10000000b) are FORBIDDEN
        if ((level & 0x7F) == 0) {
            av_log(s->avctx, AV_LOG_ERROR, "illegal dc %d at %d %d\n",
                   level, s->mb_x, s->mb_y);
            return -1;
        }
        /* The code 1000 0000 is not used, the reconstruction level of 1024
         * being coded as 1111 1111. */
        if (level == 255)
            level = 128;
        block[0] = level * 8;
        i        = 1;
    } else if (coded) {
        // Run  Level   Code
        // EOB          Not possible for first level when cbp is available (that's why the table is different)
        // 0    1       1s
        // *    *       0*
        int check = show_bits(&h->gb, 2);
        i = 0;
        if (check & 0x2) {
            skip_bits(&h->gb, 2);
            block[0] = qmul + qadd;
            block[0] *= (check & 0x1) ? -1 : 1;
            i        = 1;
        }
    } else {
        i = 0;
    }
    if (!coded) {
        s->block_last_index[n] = i - 1;
        return 0;
    }
    {
    OPEN_READER(re, &h->gb);
    i--; // offset by -1 to allow direct indexing of scan_table
    for (;;) {
        UPDATE_CACHE(re, &h->gb);
        GET_RL_VLC(level, run, re, &h->gb, rl->rl_vlc[0], TCOEFF_VLC_BITS, 2, 0);
        if (run == 66) {
            if (level) {
                CLOSE_READER(re, &h->gb);
                av_log(s->avctx, AV_LOG_ERROR, "illegal ac vlc code at %dx%d\n",
                       s->mb_x, s->mb_y);
                return -1;
            }
            /* escape */
            /* The remaining combinations of (run, level) are encoded with a
             * 20-bit word consisting of 6 bits escape, 6 bits run and 8 bits
             * level. */
            run   = SHOW_UBITS(re, &h->gb, 6) + 1;
            SKIP_CACHE(re, &h->gb, 6);
            level = SHOW_SBITS(re, &h->gb, 8);
            if (level > 0)
                level = level * qmul + qadd;
            else if (level < 0)
                level = level * qmul - qadd;
            SKIP_COUNTER(re, &h->gb, 6 + 8);
        } else if (level == 0) {
            break;
        } else {
            level = level * qmul + qadd;
            if (SHOW_UBITS(re, &h->gb, 1))
                level = -level;
            SKIP_COUNTER(re, &h->gb, 1);
        }
        i += run;
        if (i >= 64) {
            CLOSE_READER(re, &h->gb);
            av_log(s->avctx, AV_LOG_ERROR, "run overflow at %dx%d\n",
                   s->mb_x, s->mb_y);
            return -1;
        }
        j        = scan_table[i];
        block[j] = level;
    }
    CLOSE_READER(re, &h->gb);
    }
    s->block_last_index[n] = i;
    return 0;
}

static int h261_decode_mb(H261DecContext *h)
{
    MpegEncContext *const s = &h->s;
    H261Context *const com = &h->common;
    int i, cbp, xy;

    cbp = 63;
    // Read mba
    do {
        h->mba_diff = get_vlc2(&h->gb, h261_mba_vlc,
                               H261_MBA_VLC_BITS, 2);

        /* Check for slice end */
        /* NOTE: GOB can be empty (no MB data) or exist only of MBA_stuffing */
        if (h->mba_diff == MBA_STARTCODE) { // start code
            h->gob_start_code_skipped = 1;
            return SLICE_END;
        }
    } while (h->mba_diff == MBA_STUFFING); // stuffing

    if (h->mba_diff < 0) {
        if (get_bits_left(&h->gb) <= 7)
            return SLICE_END;

        av_log(s->avctx, AV_LOG_ERROR, "illegal mba at %d %d\n", s->mb_x, s->mb_y);
        return SLICE_ERROR;
    }

    h->mba_diff    += 1;
    h->current_mba += h->mba_diff;

    if (h->current_mba > MBA_STUFFING)
        return SLICE_ERROR;

    s->mb_x = ((h->gob_number - 1) % 2) * 11 + ((h->current_mba - 1) % 11);
    s->mb_y = ((h->gob_number - 1) / 2) * 3 + ((h->current_mba - 1) / 11);
    xy      = s->mb_x + s->mb_y * s->mb_stride;
    h261_init_dest(s);

    // Read mtype
    com->mtype = get_vlc2(&h->gb, h261_mtype_vlc, H261_MTYPE_VLC_BITS, 2);
    if (com->mtype < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Invalid mtype index\n");
        return SLICE_ERROR;
    }

    // Read mquant
    if (IS_QUANT(com->mtype)) {
        s->qscale = get_bits(&h->gb, 5);
        if (!s->qscale)
            s->qscale = 1;
    }

    s->mb_intra = IS_INTRA4x4(com->mtype);

    // Read mv
    if (IS_16X16(com->mtype)) {
        /* Motion vector data is included for all MC macroblocks. MVD is
         * obtained from the macroblock vector by subtracting the vector
         * of the preceding macroblock. For this calculation the vector
         * of the preceding macroblock is regarded as zero in the
         * following three situations:
         * 1) evaluating MVD for macroblocks 1, 12 and 23;
         * 2) evaluating MVD for macroblocks in which MBA does not represent a difference of 1;
         * 3) MTYPE of the previous macroblock was not MC. */
        if ((h->current_mba ==  1) || (h->current_mba == 12) ||
            (h->current_mba == 23) || (h->mba_diff != 1)) {
            h->current_mv_x = 0;
            h->current_mv_y = 0;
        }

        h->current_mv_x = decode_mv_component(&h->gb, h->current_mv_x);
        h->current_mv_y = decode_mv_component(&h->gb, h->current_mv_y);
    } else {
        h->current_mv_x = 0;
        h->current_mv_y = 0;
    }

    // Read cbp
    if (HAS_CBP(com->mtype))
        cbp = get_vlc2(&h->gb, h261_cbp_vlc, H261_CBP_VLC_BITS, 1) + 1;

    if (s->mb_intra) {
        s->cur_pic.mb_type[xy] = MB_TYPE_INTRA;
        goto intra;
    }

    //set motion vectors
    s->mv_dir                      = MV_DIR_FORWARD;
    s->mv_type                     = MV_TYPE_16X16;
    s->cur_pic.mb_type[xy] = MB_TYPE_16x16 | MB_TYPE_FORWARD_MV;
    s->mv[0][0][0]                 = h->current_mv_x * 2; // gets divided by 2 in motion compensation
    s->mv[0][0][1]                 = h->current_mv_y * 2;

    if (s->cur_pic.motion_val[0]) {
        int b_stride = 2*s->mb_width + 1;
        int b_xy     = 2 * s->mb_x + (2 * s->mb_y) * b_stride;
        s->cur_pic.motion_val[0][b_xy][0] = s->mv[0][0][0];
        s->cur_pic.motion_val[0][b_xy][1] = s->mv[0][0][1];
    }

intra:
    /* decode each block */
    if (s->mb_intra || HAS_CBP(com->mtype)) {
        s->bdsp.clear_blocks(h->block[0]);
        for (i = 0; i < 6; i++) {
            if (h261_decode_block(h, h->block[i], i, cbp & 32) < 0)
                return SLICE_ERROR;
            cbp += cbp;
        }
    } else {
        for (i = 0; i < 6; i++)
            s->block_last_index[i] = -1;
    }

    ff_mpv_reconstruct_mb(s, h->block);

    return SLICE_OK;
}

/**
 * Decode the H.261 picture header.
 * @return <0 if no startcode found
 */
static int h261_decode_picture_header(H261DecContext *h, int *is_key)
{
    MpegEncContext *const s = &h->s;
    uint32_t startcode = 0;

    for (int i = get_bits_left(&h->gb); i > 24; i -= 1) {
        startcode = ((startcode << 1) | get_bits(&h->gb, 1)) & 0x000FFFFF;

        if (startcode == 0x10)
            break;
    }

    if (startcode != 0x10) {
        av_log(s->avctx, AV_LOG_ERROR, "Bad picture start code\n");
        return -1;
    }

    /* temporal reference */
    skip_bits(&h->gb, 5); /* picture timestamp */

    /* PTYPE starts here */
    skip_bits1(&h->gb); /* split screen off */
    skip_bits1(&h->gb); /* camera  off */
    *is_key = get_bits1(&h->gb); /* freeze picture release off */

    int format = get_bits1(&h->gb);

    // only 2 formats possible
    if (format == 0) { // QCIF
        s->width     = 176;
        s->height    = 144;
    } else { // CIF
        s->width     = 352;
        s->height    = 288;
    }

    skip_bits1(&h->gb); /* still image mode off */
    skip_bits1(&h->gb); /* Reserved */

    /* PEI */
    if (skip_1stop_8data_bits(&h->gb) < 0)
        return AVERROR_INVALIDDATA;

    h->gob_number = 0;
    return 0;
}

static int h261_decode_gob(H261DecContext *h)
{
    MpegEncContext *const s = &h->s;

    /* decode mb's */
    while (h->current_mba <= MBA_STUFFING) {
        int ret;
        /* DCT & quantize */
        ret = h261_decode_mb(h);
        if (ret < 0) {
            if (ret == SLICE_END) {
                h261_decode_mb_skipped(h, h->current_mba, 33);
                return 0;
            }
            av_log(s->avctx, AV_LOG_ERROR, "Error at MB: %d\n",
                   s->mb_x + s->mb_y * s->mb_stride);
            return -1;
        }

        h261_decode_mb_skipped(h,
                               h->current_mba - h->mba_diff,
                               h->current_mba - 1);
    }

    return -1;
}

static int h261_decode_frame(AVCodecContext *avctx, AVFrame *pict,
                             int *got_frame, AVPacket *avpkt)
{
    H261DecContext *const h = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    MpegEncContext *s  = &h->s;
    int ret, is_key;

    ff_dlog(avctx, "*****frame %"PRId64" size=%d\n", avctx->frame_num, buf_size);
    ff_dlog(avctx, "bytes=%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);

    h->gob_start_code_skipped = 0;

    init_get_bits(&h->gb, buf, buf_size * 8);

    ret = h261_decode_picture_header(h, &is_key);

    /* skip if the header was thrashed */
    if (ret < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "header damaged\n");
        return -1;
    }

    if (s->width != avctx->coded_width || s->height != avctx->coded_height) {
        ff_mpv_common_end(s);
    }

    if (!s->context_initialized) {
        if ((ret = ff_mpv_common_init(s)) < 0)
            return ret;

        ret = ff_set_dimensions(avctx, s->width, s->height);
        if (ret < 0)
            return ret;
    }

    if ((avctx->skip_frame >= AVDISCARD_NONINTRA && !is_key) ||
         avctx->skip_frame >= AVDISCARD_ALL)
        return buf_size;

    if (ff_mpv_frame_start(s, avctx) < 0)
        return -1;

    ff_mpeg_er_frame_start(s);

    /* decode each macroblock */
    s->mb_x = 0;
    s->mb_y = 0;

    while (h->gob_number < (s->mb_height == 18 ? 12 : 5)) {
        if (h261_decode_gob_header(h) < 0)
            break;
        h261_decode_gob(h);
    }
    ff_mpv_frame_end(s);

    if (is_key) {
        s->cur_pic.ptr->f->pict_type = AV_PICTURE_TYPE_I;
        s->cur_pic.ptr->f->flags    |= AV_FRAME_FLAG_KEY;
    }

    if ((ret = av_frame_ref(pict, s->cur_pic.ptr->f)) < 0)
        return ret;
    ff_print_debug_info(s, s->cur_pic.ptr, pict);

    *got_frame = 1;

    return buf_size;
}

const FFCodec ff_h261_decoder = {
    .p.name         = "h261",
    CODEC_LONG_NAME("H.261"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_H261,
    .priv_data_size = sizeof(H261DecContext),
    .init           = h261_decode_init,
    FF_CODEC_DECODE_CB(h261_decode_frame),
    .close          = ff_mpv_decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .p.max_lowres   = 3,
    .caps_internal  = FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM,
};
