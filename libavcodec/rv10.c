/*
 * RV10/RV20 decoder
 * Copyright (c) 2000,2001 Fabrice Bellard
 * Copyright (c) 2002-2004 Michael Niedermayer
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
 * RV10/RV20 decoder
 */

#include <inttypes.h>

#include "libavutil/imgutils.h"
#include "libavutil/thread.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "error_resilience.h"
#include "h263.h"
#include "h263data.h"
#include "h263dec.h"
#include "mpeg_er.h"
#include "mpegvideo.h"
#include "mpegvideodec.h"
#include "mpeg4video.h"
#include "mpegvideodata.h"
#include "rv10dec.h"

#define RV_GET_MAJOR_VER(x)  ((x) >> 28)
#define RV_GET_MINOR_VER(x) (((x) >> 20) & 0xFF)
#define RV_GET_MICRO_VER(x) (((x) >> 12) & 0xFF)

#define MAX_VLC_ENTRIES 1023 // Note: Does not include the skip entries.
#define DC_VLC_BITS        9

typedef struct RVDecContext {
    H263DecContext h;
    int sub_id;
    int orig_width, orig_height;
} RVDecContext;

/* (run, length) encoded value for the symbols table. The actual symbols
 * are run..run - length (mod 256).
 * The last two entries in the following table apply to luma only.
 * The skip values are not included in this list. */
static const uint8_t rv_sym_run_len[][2] = {
    {   0,   0 }, {   1,   0 }, { 255,   0 }, {   3,   1 }, { 254,  1 },
    {   7,   3 }, { 252,   3 }, {  15,   7 }, { 248,   7 }, {  31, 15 },
    { 240,  15 }, {  63,  31 }, { 224,  31 }, { 127,  63 }, { 192, 63 },
    { 255, 127 }, { 128, 127 }, { 127, 255 }, { 128, 255 },
};

/* entry[i] of the following tables gives
 * the number of VLC codes of length i + 2. */
static const uint16_t rv_lum_len_count[15] = {
    1,  0,  2,  4,  8, 16, 32,  0, 64,  0, 128,  0, 256,  0, 512,
};

static const uint16_t rv_chrom_len_count[15] = {
    1,  2,  4,  0,  8,  0, 16,  0, 32,  0,  64,  0, 128,  0, 256,
};

static VLCElem rv_dc_lum[1472], rv_dc_chrom[992];

int ff_rv_decode_dc(H263DecContext *const h, int n)
{
    int code;

    if (n < 4) {
        code = get_vlc2(&h->gb, rv_dc_lum, DC_VLC_BITS, 2);
    } else {
        code = get_vlc2(&h->gb, rv_dc_chrom, DC_VLC_BITS, 2);
        if (code < 0) {
            av_log(h->c.avctx, AV_LOG_ERROR, "chroma dc error\n");
            return -1;
        }
    }
    return code;
}

/* read RV 1.0 compatible frame header */
static int rv10_decode_picture_header(H263DecContext *const h)
{
    int mb_count, pb_frame, marker, mb_xy;

    marker = get_bits1(&h->gb);

    if (get_bits1(&h->gb))
        h->c.pict_type = AV_PICTURE_TYPE_P;
    else
        h->c.pict_type = AV_PICTURE_TYPE_I;

    if (!marker)
        av_log(h->c.avctx, AV_LOG_ERROR, "marker missing\n");

    pb_frame = get_bits1(&h->gb);

    ff_dlog(h->c.avctx, "pict_type=%d pb_frame=%d\n", h->c.pict_type, pb_frame);

    if (pb_frame) {
        avpriv_request_sample(h->c.avctx, "PB-frame");
        return AVERROR_PATCHWELCOME;
    }

    h->c.qscale = get_bits(&h->gb, 5);
    if (h->c.qscale == 0) {
        av_log(h->c.avctx, AV_LOG_ERROR, "Invalid qscale value: 0\n");
        return AVERROR_INVALIDDATA;
    }

    if (h->c.pict_type == AV_PICTURE_TYPE_I) {
        if (h->rv10_version == 3) {
            /* specific MPEG like DC coding not used */
            h->c.last_dc[0] = get_bits(&h->gb, 8);
            h->c.last_dc[1] = get_bits(&h->gb, 8);
            h->c.last_dc[2] = get_bits(&h->gb, 8);
            ff_dlog(h->c.avctx, "DC:%d %d %d\n", h->c.last_dc[0],
                    h->c.last_dc[1], h->c.last_dc[2]);
        }
    }
    /* if multiple packets per frame are sent, the position at which
     * to display the macroblocks is coded here */

    mb_xy = h->c.mb_x + h->c.mb_y * h->c.mb_width;
    if (show_bits(&h->gb, 12) == 0 || (mb_xy && mb_xy < h->c.mb_num)) {
        h->c.mb_x  = get_bits(&h->gb, 6); /* mb_x */
        h->c.mb_y  = get_bits(&h->gb, 6); /* mb_y */
        mb_count   = get_bits(&h->gb, 12);
    } else {
        h->c.mb_x  = 0;
        h->c.mb_y  = 0;
        mb_count = h->c.mb_width * h->c.mb_height;
    }
    skip_bits(&h->gb, 3);   /* ignored */

    return mb_count;
}

static int rv20_decode_picture_header(RVDecContext *rv, int whole_size)
{
    static const enum AVPictureType pict_types[] =
        { AV_PICTURE_TYPE_I, AV_PICTURE_TYPE_I /* hmm ... */,
          AV_PICTURE_TYPE_P, AV_PICTURE_TYPE_B };
    H263DecContext *const h = &rv->h;
    int seq, mb_pos, ret;
    int rpr_max;

    h->c.pict_type = pict_types[get_bits(&h->gb, 2)];

    if (h->c.low_delay && h->c.pict_type == AV_PICTURE_TYPE_B) {
        av_log(h->c.avctx, AV_LOG_ERROR, "low delay B\n");
        return -1;
    }
    if (!h->c.last_pic.ptr && h->c.pict_type == AV_PICTURE_TYPE_B) {
        av_log(h->c.avctx, AV_LOG_ERROR, "early B-frame\n");
        return AVERROR_INVALIDDATA;
    }

    if (get_bits1(&h->gb)) {
        av_log(h->c.avctx, AV_LOG_ERROR, "reserved bit set\n");
        return AVERROR_INVALIDDATA;
    }

    h->c.qscale = get_bits(&h->gb, 5);
    if (h->c.qscale == 0) {
        av_log(h->c.avctx, AV_LOG_ERROR, "Invalid qscale value: 0\n");
        return AVERROR_INVALIDDATA;
    }

    if (RV_GET_MINOR_VER(rv->sub_id) >= 2)
        h->loop_filter = get_bits1(&h->gb) && !h->c.avctx->lowres;

    if (RV_GET_MINOR_VER(rv->sub_id) <= 1)
        seq = get_bits(&h->gb, 8) << 7;
    else
        seq = get_bits(&h->gb, 13) << 2;

    rpr_max = h->c.avctx->extradata[1] & 7;
    if (rpr_max) {
        int f, new_w, new_h;
        int rpr_bits = av_log2(rpr_max) + 1;

        f = get_bits(&h->gb, rpr_bits);

        if (f) {
            if (h->c.avctx->extradata_size < 8 + 2 * f) {
                av_log(h->c.avctx, AV_LOG_ERROR, "Extradata too small.\n");
                return AVERROR_INVALIDDATA;
            }

            new_w = 4 * h->c.avctx->extradata[6 + 2 * f];
            new_h = 4 * h->c.avctx->extradata[7 + 2 * f];
        } else {
            new_w = rv->orig_width;
            new_h = rv->orig_height;
        }
        if (new_w != h->c.width || new_h != h->c.height || !h->c.context_initialized) {
            AVRational old_aspect = h->c.avctx->sample_aspect_ratio;
            av_log(h->c.avctx, AV_LOG_DEBUG,
                   "attempting to change resolution to %dx%d\n", new_w, new_h);
            if (av_image_check_size(new_w, new_h, 0, h->c.avctx) < 0)
                return AVERROR_INVALIDDATA;

            if (whole_size < (new_w + 15)/16 * ((new_h + 15)/16) / 8)
                return AVERROR_INVALIDDATA;

            ff_mpv_common_end(&h->c);

            // attempt to keep aspect during typical resolution switches
            if (!old_aspect.num)
                old_aspect = (AVRational){1, 1};
            if (2 * (int64_t)new_w * h->c.height == (int64_t)new_h * h->c.width)
                h->c.avctx->sample_aspect_ratio = av_mul_q(old_aspect, (AVRational){2, 1});
            if ((int64_t)new_w * h->c.height == 2 * (int64_t)new_h * h->c.width)
                h->c.avctx->sample_aspect_ratio = av_mul_q(old_aspect, (AVRational){1, 2});

            ret = ff_set_dimensions(h->c.avctx, new_w, new_h);
            if (ret < 0)
                return ret;

            h->c.width  = new_w;
            h->c.height = new_h;
            if ((ret = ff_mpv_common_init(&h->c)) < 0)
                return ret;
        }

        if (h->c.avctx->debug & FF_DEBUG_PICT_INFO) {
            av_log(h->c.avctx, AV_LOG_DEBUG, "F %d/%d/%d\n", f, rpr_bits, rpr_max);
        }
    }
    if (av_image_check_size(h->c.width, h->c.height, 0, h->c.avctx) < 0)
        return AVERROR_INVALIDDATA;

    mb_pos = ff_h263_decode_mba(h);

    seq |= h->c.time & ~0x7FFF;
    if (seq - h->c.time >  0x4000)
        seq -= 0x8000;
    if (seq - h->c.time < -0x4000)
        seq += 0x8000;

    if (seq != h->c.time) {
        if (h->c.pict_type != AV_PICTURE_TYPE_B) {
            h->c.time            = seq;
            h->c.pp_time         = h->c.time - h->c.last_non_b_time;
            h->c.last_non_b_time = h->c.time;
        } else {
            h->c.time    = seq;
            h->c.pb_time = h->c.pp_time - (h->c.last_non_b_time - h->c.time);
        }
    }
    if (h->c.pict_type == AV_PICTURE_TYPE_B) {
        if (h->c.pp_time <=h->c.pb_time || h->c.pp_time <= h->c.pp_time - h->c.pb_time || h->c.pp_time<=0) {
            av_log(h->c.avctx, AV_LOG_DEBUG,
                   "messed up order, possible from seeking? skipping current B-frame\n");
#define ERROR_SKIP_FRAME -123
            return ERROR_SKIP_FRAME;
        }
        ff_mpeg4_init_direct_mv(&h->c);
    }

    h->c.no_rounding = get_bits1(&h->gb);

    if (RV_GET_MINOR_VER(rv->sub_id) <= 1 && h->c.pict_type == AV_PICTURE_TYPE_B)
        // binary decoder reads 3+2 bits here but they don't seem to be used
        skip_bits(&h->gb, 5);

    h->c.h263_aic        = h->c.pict_type == AV_PICTURE_TYPE_I;
    if (h->c.h263_aic) {
        h->c.y_dc_scale_table =
        h->c.c_dc_scale_table = ff_aic_dc_scale_table;
    } else {
        h->c.y_dc_scale_table =
        h->c.c_dc_scale_table = ff_mpeg1_dc_scale_table;
    }
    if (!h->c.avctx->lowres)
        h->loop_filter = 1;

    if (h->c.avctx->debug & FF_DEBUG_PICT_INFO) {
        av_log(h->c.avctx, AV_LOG_INFO,
               "num:%5d x:%2d y:%2d type:%d qscale:%2d rnd:%d\n",
               seq, h->c.mb_x, h->c.mb_y, h->c.pict_type, h->c.qscale,
               h->c.no_rounding);
    }

    av_assert0(h->c.pict_type != AV_PICTURE_TYPE_B || !h->c.low_delay);

    return h->c.mb_width * h->c.mb_height - mb_pos;
}

static av_cold void rv10_build_vlc(VLCElem vlc[], int table_size,
                                   const uint16_t len_count[15],
                                   const uint8_t sym_rl[][2], int sym_rl_elems)
{
    uint16_t syms[MAX_VLC_ENTRIES];
    uint8_t  lens[MAX_VLC_ENTRIES];
    unsigned nb_syms = 0, nb_lens = 0;

    for (unsigned i = 0; i < sym_rl_elems; i++) {
        unsigned cur_sym = sym_rl[i][0];
        for (unsigned tmp = nb_syms + sym_rl[i][1]; nb_syms <= tmp; nb_syms++)
            syms[nb_syms] = 0xFF & cur_sym--;
    }

    for (unsigned i = 0; i < 15; i++)
        for (unsigned tmp = nb_lens + len_count[i]; nb_lens < tmp; nb_lens++)
            lens[nb_lens] = i + 2;
    av_assert1(nb_lens == nb_syms);
    ff_vlc_init_table_from_lengths(vlc, table_size, DC_VLC_BITS, nb_lens,
                                   lens, 1, syms, 2, 2, 0, 0);
}

static av_cold void rv10_init_static(void)
{
    rv10_build_vlc(rv_dc_lum, FF_ARRAY_ELEMS(rv_dc_lum), rv_lum_len_count,
                   rv_sym_run_len, FF_ARRAY_ELEMS(rv_sym_run_len));
    for (int i = 0; i < 1 << (DC_VLC_BITS - 7 /* Length of skip prefix */); i++) {
        /* All codes beginning with 0x7F have the same length and value.
         * Modifying the table directly saves us the useless subtables. */
        rv_dc_lum[(0x7F << (DC_VLC_BITS - 7)) + i].sym = 255;
        rv_dc_lum[(0x7F << (DC_VLC_BITS - 7)) + i].len = 18;
    }
    rv10_build_vlc(rv_dc_chrom, FF_ARRAY_ELEMS(rv_dc_chrom), rv_chrom_len_count,
                   rv_sym_run_len, FF_ARRAY_ELEMS(rv_sym_run_len) - 2);
    for (int i = 0; i < 1 << (DC_VLC_BITS - 9 /* Length of skip prefix */); i++) {
        /* Same as above. */
        rv_dc_chrom[(0x1FE << (DC_VLC_BITS - 9)) + i].sym = 255;
        rv_dc_chrom[(0x1FE << (DC_VLC_BITS - 9)) + i].len = 18;
    }
}

static av_cold int rv10_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    RVDecContext *rv = avctx->priv_data;
    H263DecContext *const h = &rv->h;
    int major_ver, minor_ver, micro_ver, ret;

    if (avctx->extradata_size < 8) {
        av_log(avctx, AV_LOG_ERROR, "Extradata is too small.\n");
        return AVERROR_INVALIDDATA;
    }
    if ((ret = av_image_check_size(avctx->coded_width,
                                   avctx->coded_height, 0, avctx)) < 0)
        return ret;

    ret = ff_h263_decode_init(avctx);
    if (ret < 0)
        return ret;

    rv->orig_width  = avctx->coded_width;
    rv->orig_height = avctx->coded_height;

    h->h263_long_vectors = avctx->extradata[3] & 1;
    rv->sub_id           = AV_RB32A(avctx->extradata + 4);
    if (avctx->codec_id == AV_CODEC_ID_RV20) {
        h->modified_quant        = 1;
        h->c.chroma_qscale_table = ff_h263_chroma_qscale_table;
    }

    major_ver = RV_GET_MAJOR_VER(rv->sub_id);
    minor_ver = RV_GET_MINOR_VER(rv->sub_id);
    micro_ver = RV_GET_MICRO_VER(rv->sub_id);

    switch (major_ver) {
    case 1:
        h->rv10_version = micro_ver ? 3 : 1;
        h->c.obmc         = micro_ver == 2;
        break;
    case 2:
        if (minor_ver >= 2) {
            h->c.low_delay      = 0;
            avctx->has_b_frames = 1;
        }
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "unknown header %X\n", rv->sub_id);
        avpriv_request_sample(avctx, "RV1/2 version");
        return AVERROR_PATCHWELCOME;
    }

    if (avctx->debug & FF_DEBUG_PICT_INFO) {
        av_log(avctx, AV_LOG_DEBUG, "ver:%X ver0:%"PRIX32"\n", rv->sub_id,
               AV_RL32A(avctx->extradata));
    }

    /* init static VLCs */
    ff_thread_once(&init_static_once, rv10_init_static);

    return 0;
}

static int rv10_decode_packet(AVCodecContext *avctx, const uint8_t *buf,
                              int buf_size, int buf_size2, int whole_size)
{
    RVDecContext *rv = avctx->priv_data;
    H263DecContext *const h = &rv->h;
    int mb_count, mb_pos, left, start_mb_x, active_bits_size, ret;

    active_bits_size = buf_size * 8;
    init_get_bits(&h->gb, buf, FFMAX(buf_size, buf_size2) * 8);
    if (h->c.codec_id == AV_CODEC_ID_RV10)
        mb_count = rv10_decode_picture_header(h);
    else
        mb_count = rv20_decode_picture_header(rv, whole_size);
    if (mb_count < 0) {
        if (mb_count != ERROR_SKIP_FRAME)
            av_log(h->c.avctx, AV_LOG_ERROR, "HEADER ERROR\n");
        return AVERROR_INVALIDDATA;
    }

    if (h->c.mb_x >= h->c.mb_width ||
        h->c.mb_y >= h->c.mb_height) {
        av_log(h->c.avctx, AV_LOG_ERROR, "POS ERROR %d %d\n", h->c.mb_x, h->c.mb_y);
        return AVERROR_INVALIDDATA;
    }
    mb_pos = h->c.mb_y * h->c.mb_width + h->c.mb_x;
    left   = h->c.mb_width * h->c.mb_height - mb_pos;
    if (mb_count > left) {
        av_log(h->c.avctx, AV_LOG_ERROR, "COUNT ERROR\n");
        return AVERROR_INVALIDDATA;
    }

    if (whole_size < h->c.mb_width * h->c.mb_height / 8)
        return AVERROR_INVALIDDATA;

    if ((h->c.mb_x == 0 && h->c.mb_y == 0) || !h->c.cur_pic.ptr) {
        // FIXME write parser so we always have complete frames?
        if (h->c.cur_pic.ptr) {
            ff_er_frame_end(&h->c.er, NULL);
            ff_mpv_frame_end(&h->c);
            h->c.mb_x = h->c.mb_y = h->c.resync_mb_x = h->c.resync_mb_y = 0;
        }
        if ((ret = ff_mpv_frame_start(&h->c, avctx)) < 0)
            return ret;
        ff_mpv_er_frame_start_ext(&h->c, 0, h->c.pp_time, h->c.pb_time);
    } else {
        if (h->c.cur_pic.ptr->f->pict_type != h->c.pict_type) {
            av_log(h->c.avctx, AV_LOG_ERROR, "Slice type mismatch\n");
            return AVERROR_INVALIDDATA;
        }
    }


    ff_dlog(avctx, "qscale=%d\n", h->c.qscale);

    /* default quantization values */
    if (h->c.codec_id == AV_CODEC_ID_RV10) {
        if (h->c.mb_y == 0)
            h->c.first_slice_line = 1;
    } else {
        h->c.first_slice_line = 1;
        h->c.resync_mb_x      = h->c.mb_x;
    }
    start_mb_x     = h->c.mb_x;
    h->c.resync_mb_y = h->c.mb_y;

    ff_set_qscale(&h->c, h->c.qscale);

    h->rv10_first_dc_coded[0] = 0;
    h->rv10_first_dc_coded[1] = 0;
    h->rv10_first_dc_coded[2] = 0;
    ff_init_block_index(&h->c);

    /* decode each macroblock */
    for (h->mb_num_left = mb_count; h->mb_num_left > 0; h->mb_num_left--) {
        int ret;
        ff_update_block_index(&h->c, 8, h->c.avctx->lowres, 1);
        ff_tlog(avctx, "**mb x=%d y=%d\n", h->c.mb_x, h->c.mb_y);

        h->c.mv_dir  = MV_DIR_FORWARD;
        h->c.mv_type = MV_TYPE_16X16;
        ret = ff_h263_decode_mb(h);

        // Repeat the slice end check from ff_h263_decode_mb with our active
        // bitstream size
        if (ret != SLICE_ERROR && active_bits_size >= get_bits_count(&h->gb)) {
            int v = show_bits(&h->gb, 16);

            if (get_bits_count(&h->gb) + 16 > active_bits_size)
                v >>= get_bits_count(&h->gb) + 16 - active_bits_size;

            if (!v)
                ret = SLICE_END;
        }
        if (ret != SLICE_ERROR && active_bits_size < get_bits_count(&h->gb) &&
            8 * buf_size2 >= get_bits_count(&h->gb)) {
            active_bits_size = buf_size2 * 8;
            av_log(avctx, AV_LOG_DEBUG, "update size from %d to %d\n",
                   8 * buf_size, active_bits_size);
            ret = SLICE_OK;
        }

        if (ret == SLICE_ERROR || active_bits_size < get_bits_count(&h->gb)) {
            av_log(h->c.avctx, AV_LOG_ERROR, "ERROR at MB %d %d\n", h->c.mb_x,
                   h->c.mb_y);
            return AVERROR_INVALIDDATA;
        }
        if (h->c.pict_type != AV_PICTURE_TYPE_B)
            ff_h263_update_motion_val(&h->c);
        ff_mpv_reconstruct_mb(&h->c, h->block);
        if (h->loop_filter)
            ff_h263_loop_filter(&h->c);

        if (++h->c.mb_x == h->c.mb_width) {
            h->c.mb_x = 0;
            h->c.mb_y++;
            ff_init_block_index(&h->c);
        }
        if (h->c.mb_x == h->c.resync_mb_x)
            h->c.first_slice_line = 0;
        if (ret == SLICE_END)
            break;
    }

    ff_er_add_slice(&h->c.er, start_mb_x, h->c.resync_mb_y, h->c.mb_x - 1, h->c.mb_y,
                    ER_MB_END);

    return active_bits_size;
}

static int get_slice_offset(AVCodecContext *avctx, const uint8_t *buf, int n)
{
    return AV_RL32(buf + n * 8);
}

static int rv10_decode_frame(AVCodecContext *avctx, AVFrame *pict,
                             int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    MpegEncContext *s = avctx->priv_data;
    int i, ret;
    int slice_count;
    const uint8_t *slices_hdr = NULL;

    ff_dlog(avctx, "*****frame %"PRId64" size=%d\n", avctx->frame_num, buf_size);

    /* no supplementary picture */
    if (buf_size == 0) {
        return 0;
    }

    slice_count = (*buf++) + 1;
    buf_size--;

    if (!slice_count || buf_size <= 8 * slice_count) {
        av_log(avctx, AV_LOG_ERROR, "Invalid slice count: %d.\n",
               slice_count);
        return AVERROR_INVALIDDATA;
    }

    slices_hdr = buf + 4;
    buf       += 8 * slice_count;
    buf_size  -= 8 * slice_count;

    for (i = 0; i < slice_count; i++) {
        unsigned offset = get_slice_offset(avctx, slices_hdr, i);
        int size, size2;

        if (offset >= buf_size)
            return AVERROR_INVALIDDATA;

        if (i + 1 == slice_count)
            size = buf_size - offset;
        else
            size = get_slice_offset(avctx, slices_hdr, i + 1) - offset;

        if (i + 2 >= slice_count)
            size2 = buf_size - offset;
        else
            size2 = get_slice_offset(avctx, slices_hdr, i + 2) - offset;

        if (size <= 0 || size2 <= 0 ||
            offset + FFMAX(size, size2) > buf_size)
            return AVERROR_INVALIDDATA;

        if ((ret = rv10_decode_packet(avctx, buf + offset, size, size2, buf_size)) < 0)
            return ret;

        if (ret > 8 * size)
            i++;
    }

    if (s->cur_pic.ptr && s->mb_y >= s->mb_height) {
        ff_er_frame_end(&s->er, NULL);
        ff_mpv_frame_end(s);

        if (s->pict_type == AV_PICTURE_TYPE_B || s->low_delay) {
            if ((ret = av_frame_ref(pict, s->cur_pic.ptr->f)) < 0)
                return ret;
            ff_print_debug_info(s, s->cur_pic.ptr, pict);
            ff_mpv_export_qp_table(s, pict, s->cur_pic.ptr, FF_MPV_QSCALE_TYPE_MPEG1);
        } else if (s->last_pic.ptr) {
            if ((ret = av_frame_ref(pict, s->last_pic.ptr->f)) < 0)
                return ret;
            ff_print_debug_info(s, s->last_pic.ptr, pict);
            ff_mpv_export_qp_table(s, pict,s->last_pic.ptr, FF_MPV_QSCALE_TYPE_MPEG1);
        }

        if (s->last_pic.ptr || s->low_delay) {
            *got_frame = 1;
        }

        // so we can detect if frame_end was not called (find some nicer solution...)
        ff_mpv_unref_picture(&s->cur_pic);
    }

    return avpkt->size;
}

const FFCodec ff_rv10_decoder = {
    .p.name         = "rv10",
    CODEC_LONG_NAME("RealVideo 1.0"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_RV10,
    .priv_data_size = sizeof(RVDecContext),
    .init           = rv10_decode_init,
    FF_CODEC_DECODE_CB(rv10_decode_frame),
    .close          = ff_mpv_decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .p.max_lowres   = 3,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};

const FFCodec ff_rv20_decoder = {
    .p.name         = "rv20",
    CODEC_LONG_NAME("RealVideo 2.0"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_RV20,
    .priv_data_size = sizeof(RVDecContext),
    .init           = rv10_decode_init,
    FF_CODEC_DECODE_CB(rv10_decode_frame),
    .close          = ff_mpv_decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
    .flush          = ff_mpeg_flush,
    .p.max_lowres   = 3,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
