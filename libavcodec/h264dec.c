/*
 * H.26L/H.264/AVC/JVT/14496-10/... decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
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
 * H.264 / AVC / MPEG-4 part10 codec.
 * @author Michael Niedermayer <michaelni@gmx.at>
 */

#define UNCHECKED_BITSTREAM_READER 1

#include "config_components.h"

#include "libavutil/avassert.h"
#include "libavutil/emms.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/thread.h"
#include "libavutil/video_enc_params.h"

#include "codec_internal.h"
#include "internal.h"
#include "error_resilience.h"
#include "avcodec.h"
#include "h264.h"
#include "h264dec.h"
#include "h2645_parse.h"
#include "h264data.h"
#include "h264_ps.h"
#include "golomb.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "mpegutils.h"
#include "profiles.h"
#include "rectangle.h"
#include "libavutil/refstruct.h"
#include "thread.h"
#include "threadframe.h"

const uint16_t ff_h264_mb_sizes[4] = { 256, 384, 512, 768 };

int avpriv_h264_has_num_reorder_frames(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    return h && h->ps.sps ? h->ps.sps->num_reorder_frames : 0;
}

static void h264_er_decode_mb(void *opaque, int ref, int mv_dir, int mv_type,
                              int (*mv)[2][4][2],
                              int mb_x, int mb_y, int mb_intra, int mb_skipped)
{
    const H264Context *h = opaque;
    H264SliceContext *sl = &h->slice_ctx[0];

    sl->mb_x = mb_x;
    sl->mb_y = mb_y;
    sl->mb_xy = mb_x + mb_y * h->mb_stride;
    memset(sl->non_zero_count_cache, 0, sizeof(sl->non_zero_count_cache));
    av_assert1(ref >= 0);
    /* FIXME: It is possible albeit uncommon that slice references
     * differ between slices. We take the easy approach and ignore
     * it for now. If this turns out to have any relevance in
     * practice then correct remapping should be added. */
    if (ref >= sl->ref_count[0])
        ref = 0;
    if (!sl->ref_list[0][ref].data[0]) {
        av_log(h->avctx, AV_LOG_DEBUG, "Reference not available for error concealing\n");
        ref = 0;
    }
    if ((sl->ref_list[0][ref].reference&3) != 3) {
        av_log(h->avctx, AV_LOG_DEBUG, "Reference invalid\n");
        return;
    }
    fill_rectangle(&h->cur_pic.ref_index[0][4 * sl->mb_xy],
                   2, 2, 2, ref, 1);
    fill_rectangle(&sl->ref_cache[0][scan8[0]], 4, 4, 8, ref, 1);
    fill_rectangle(sl->mv_cache[0][scan8[0]], 4, 4, 8,
                   pack16to32((*mv)[0][0][0], (*mv)[0][0][1]), 4);
    sl->mb_mbaff =
    sl->mb_field_decoding_flag = 0;
    ff_h264_hl_decode_mb(h, &h->slice_ctx[0]);
}

void ff_h264_draw_horiz_band(const H264Context *h, H264SliceContext *sl,
                             int y, int height)
{
    AVCodecContext *avctx = h->avctx;
    const AVFrame   *src  = h->cur_pic.f;
    const AVPixFmtDescriptor *desc;
    int offset[AV_NUM_DATA_POINTERS];
    int vshift;
    const int field_pic = h->picture_structure != PICT_FRAME;

    if (!avctx->draw_horiz_band)
        return;

    if (field_pic && h->first_field && !(avctx->slice_flags & SLICE_FLAG_ALLOW_FIELD))
        return;

    if (field_pic) {
        height <<= 1;
        y      <<= 1;
    }

    height = FFMIN(height, avctx->height - y);

    desc   = av_pix_fmt_desc_get(avctx->pix_fmt);
    vshift = desc->log2_chroma_h;

    offset[0] = y * src->linesize[0];
    offset[1] =
    offset[2] = (y >> vshift) * src->linesize[1];
    for (int i = 3; i < AV_NUM_DATA_POINTERS; i++)
        offset[i] = 0;

    emms_c();

    avctx->draw_horiz_band(avctx, src, offset,
                           y, h->picture_structure, height);
}

void ff_h264_free_tables(H264Context *h)
{
    int i;

    av_freep(&h->intra4x4_pred_mode);
    av_freep(&h->chroma_pred_mode_table);
    av_freep(&h->cbp_table);
    av_freep(&h->mvd_table[0]);
    av_freep(&h->mvd_table[1]);
    av_freep(&h->direct_table);
    av_freep(&h->non_zero_count);
    av_freep(&h->slice_table_base);
    h->slice_table = NULL;
    av_freep(&h->list_counts);

    av_freep(&h->mb2b_xy);
    av_freep(&h->mb2br_xy);

    av_refstruct_pool_uninit(&h->qscale_table_pool);
    av_refstruct_pool_uninit(&h->mb_type_pool);
    av_refstruct_pool_uninit(&h->motion_val_pool);
    av_refstruct_pool_uninit(&h->ref_index_pool);

#if CONFIG_ERROR_RESILIENCE
    av_freep(&h->er.mb_index2xy);
    av_freep(&h->er.error_status_table);
    av_freep(&h->er.er_temp_buffer);
    av_freep(&h->dc_val_base);
#endif

    for (i = 0; i < h->nb_slice_ctx; i++) {
        H264SliceContext *sl = &h->slice_ctx[i];

        av_freep(&sl->bipred_scratchpad);
        av_freep(&sl->edge_emu_buffer);
        av_freep(&sl->top_borders[0]);
        av_freep(&sl->top_borders[1]);

        sl->bipred_scratchpad_allocated = 0;
        sl->edge_emu_buffer_allocated   = 0;
        sl->top_borders_allocated[0]    = 0;
        sl->top_borders_allocated[1]    = 0;
    }
}

int ff_h264_alloc_tables(H264Context *h)
{
    ERContext *const er = &h->er;
    const int big_mb_num = h->mb_stride * (h->mb_height + 1);
    const int row_mb_num = 2*h->mb_stride*FFMAX(h->nb_slice_ctx, 1);
    const int st_size = big_mb_num + h->mb_stride;
    int x, y;

    if (!FF_ALLOCZ_TYPED_ARRAY(h->intra4x4_pred_mode,     row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->non_zero_count,         big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->slice_table_base,       st_size)         ||
        !FF_ALLOCZ_TYPED_ARRAY(h->cbp_table,              big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->chroma_pred_mode_table, big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mvd_table[0],           row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mvd_table[1],           row_mb_num * 8)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->direct_table,           big_mb_num * 4)  ||
        !FF_ALLOCZ_TYPED_ARRAY(h->list_counts,            big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mb2b_xy,                big_mb_num)      ||
        !FF_ALLOCZ_TYPED_ARRAY(h->mb2br_xy,               big_mb_num))
        return AVERROR(ENOMEM);
    h->slice_ctx[0].intra4x4_pred_mode = h->intra4x4_pred_mode;
    h->slice_ctx[0].mvd_table[0] = h->mvd_table[0];
    h->slice_ctx[0].mvd_table[1] = h->mvd_table[1];
    memset(h->slice_table_base, -1,
           st_size * sizeof(*h->slice_table_base));
    h->slice_table = h->slice_table_base + h->mb_stride * 2 + 1;
    for (y = 0; y < h->mb_height; y++)
        for (x = 0; x < h->mb_width; x++) {
            const int mb_xy = x + y * h->mb_stride;
            const int b_xy  = 4 * x + 4 * y * h->b_stride;

            h->mb2b_xy[mb_xy]  = b_xy;
            h->mb2br_xy[mb_xy] = 8 * (FMO ? mb_xy : (mb_xy % (2 * h->mb_stride)));
        }

    if (CONFIG_ERROR_RESILIENCE) {
        const int er_size = h->mb_height * h->mb_stride * (4*sizeof(int) + 1);
        int mb_array_size = h->mb_height * h->mb_stride;
        int y_size  = (2 * h->mb_width + 1) * (2 * h->mb_height + 1);
        int yc_size = y_size + 2 * big_mb_num;

        /* init ER */
        er->avctx          = h->avctx;
        er->decode_mb      = h264_er_decode_mb;
        er->opaque         = h;
        er->quarter_sample = 1;

        er->mb_num      = h->mb_num;
        er->mb_width    = h->mb_width;
        er->mb_height   = h->mb_height;
        er->mb_stride   = h->mb_stride;
        er->b8_stride   = h->mb_width * 2 + 1;

        // error resilience code looks cleaner with this
        if (!FF_ALLOCZ_TYPED_ARRAY(er->mb_index2xy,        h->mb_num + 1) ||
            !FF_ALLOCZ_TYPED_ARRAY(er->error_status_table, mb_array_size) ||
            !FF_ALLOCZ_TYPED_ARRAY(er->er_temp_buffer,     er_size)       ||
            !FF_ALLOCZ_TYPED_ARRAY(h->dc_val_base,         yc_size))
            return AVERROR(ENOMEM); // ff_h264_free_tables will clean up for us

        for (y = 0; y < h->mb_height; y++)
            for (x = 0; x < h->mb_width; x++)
                er->mb_index2xy[x + y * h->mb_width] = x + y * h->mb_stride;

        er->mb_index2xy[h->mb_height * h->mb_width] = (h->mb_height - 1) *
                                                      h->mb_stride + h->mb_width;
        er->dc_val[0] = h->dc_val_base + h->mb_width * 2 + 2;
        er->dc_val[1] = h->dc_val_base + y_size + h->mb_stride + 1;
        er->dc_val[2] = er->dc_val[1] + big_mb_num;
        for (int i = 0; i < yc_size; i++)
            h->dc_val_base[i] = 1024;
    }

    return 0;
}

/**
 * Init slice context
 */
void ff_h264_slice_context_init(H264Context *h, H264SliceContext *sl)
{
    sl->ref_cache[0][scan8[5]  + 1] =
    sl->ref_cache[0][scan8[7]  + 1] =
    sl->ref_cache[0][scan8[13] + 1] =
    sl->ref_cache[1][scan8[5]  + 1] =
    sl->ref_cache[1][scan8[7]  + 1] =
    sl->ref_cache[1][scan8[13] + 1] = PART_NOT_AVAILABLE;

    sl->er = &h->er;
}

static int h264_init_pic(H264Picture *pic)
{
    pic->f = av_frame_alloc();
    if (!pic->f)
        return AVERROR(ENOMEM);

    pic->f_grain = av_frame_alloc();
    if (!pic->f_grain)
        return AVERROR(ENOMEM);

    return 0;
}

static int h264_init_context(AVCodecContext *avctx, H264Context *h)
{
    int i, ret;

    h->avctx                 = avctx;
    h->cur_chroma_format_idc = -1;

    h->width_from_caller     = avctx->width;
    h->height_from_caller    = avctx->height;

    h->workaround_bugs       = avctx->workaround_bugs;
    h->flags                 = avctx->flags;
    h->poc.prev_poc_msb      = 1 << 16;
    h->recovery_frame        = -1;
    h->frame_recovered       = 0;
    h->poc.prev_frame_num    = -1;
    h->sei.common.frame_packing.arrangement_cancel_flag = -1;
    h->sei.common.unregistered.x264_build = -1;

    h->next_outputed_poc = INT_MIN;
    for (i = 0; i < FF_ARRAY_ELEMS(h->last_pocs); i++)
        h->last_pocs[i] = INT_MIN;

    ff_h264_sei_uninit(&h->sei);

    if (avctx->active_thread_type & FF_THREAD_FRAME) {
        h->decode_error_flags_pool = av_refstruct_pool_alloc(sizeof(atomic_int), 0);
        if (!h->decode_error_flags_pool)
            return AVERROR(ENOMEM);
    }

    h->nb_slice_ctx = (avctx->active_thread_type & FF_THREAD_SLICE) ? avctx->thread_count : 1;
    h->slice_ctx = av_calloc(h->nb_slice_ctx, sizeof(*h->slice_ctx));
    if (!h->slice_ctx) {
        h->nb_slice_ctx = 0;
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        if ((ret = h264_init_pic(&h->DPB[i])) < 0)
            return ret;
    }

    if ((ret = h264_init_pic(&h->cur_pic)) < 0)
        return ret;

    if ((ret = h264_init_pic(&h->last_pic_for_ec)) < 0)
        return ret;

    for (i = 0; i < h->nb_slice_ctx; i++)
        h->slice_ctx[i].h264 = h;

    return 0;
}

static void h264_free_pic(H264Context *h, H264Picture *pic)
{
    ff_h264_unref_picture(pic);
    av_frame_free(&pic->f);
    av_frame_free(&pic->f_grain);
}

static av_cold int h264_decode_end(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    ff_h264_remove_all_refs(h);
    ff_h264_free_tables(h);

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
        h264_free_pic(h, &h->DPB[i]);
    }
    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));

    h->cur_pic_ptr = NULL;

    av_refstruct_pool_uninit(&h->decode_error_flags_pool);

    av_freep(&h->slice_ctx);
    h->nb_slice_ctx = 0;

    ff_h264_sei_uninit(&h->sei);
    ff_h264_ps_uninit(&h->ps);

    ff_h2645_packet_uninit(&h->pkt);

    h264_free_pic(h, &h->cur_pic);
    h264_free_pic(h, &h->last_pic_for_ec);

    return 0;
}

static AVOnce h264_vlc_init = AV_ONCE_INIT;

static av_cold int h264_decode_init(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int ret;

    ret = h264_init_context(avctx, h);
    if (ret < 0)
        return ret;

    ret = ff_thread_once(&h264_vlc_init, ff_h264_decode_init_vlc);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "pthread_once has failed.");
        return AVERROR_UNKNOWN;
    }

    if (!avctx->internal->is_copy) {
        if (avctx->extradata_size > 0 && avctx->extradata) {
            ret = ff_h264_decode_extradata(avctx->extradata, avctx->extradata_size,
                                           &h->ps, &h->is_avc, &h->nal_length_size,
                                           avctx->err_recognition, avctx);
           if (ret < 0) {
               int explode = avctx->err_recognition & AV_EF_EXPLODE;
               av_log(avctx, explode ? AV_LOG_ERROR: AV_LOG_WARNING,
                      "Error decoding the extradata\n");
               if (explode) {
                   return ret;
               }
               ret = 0;
           }
        }
    }

    if (h->ps.sps && h->ps.sps->bitstream_restriction_flag &&
        h->avctx->has_b_frames < h->ps.sps->num_reorder_frames) {
        h->avctx->has_b_frames = h->ps.sps->num_reorder_frames;
    }

    ff_h264_flush_change(h);

    if (h->enable_er < 0 && (avctx->active_thread_type & FF_THREAD_SLICE))
        h->enable_er = 0;

    if (h->enable_er && (avctx->active_thread_type & FF_THREAD_SLICE)) {
        av_log(avctx, AV_LOG_WARNING,
               "Error resilience with slice threads is enabled. It is unsafe and unsupported and may crash. "
               "Use it at your own risk\n");
    }

    return 0;
}

/**
 * instantaneous decoder refresh.
 */
static void idr(H264Context *h)
{
    int i;
    ff_h264_remove_all_refs(h);
    h->poc.prev_frame_num        =
    h->poc.prev_frame_num_offset = 0;
    h->poc.prev_poc_msb          = 1<<16;
    h->poc.prev_poc_lsb          = -1;
    for (i = 0; i < FF_ARRAY_ELEMS(h->last_pocs); i++)
        h->last_pocs[i] = INT_MIN;
}

/* forget old pics after a seek */
void ff_h264_flush_change(H264Context *h)
{
    int i, j;

    h->next_outputed_poc = INT_MIN;
    h->prev_interlaced_frame = 1;
    idr(h);

    h->poc.prev_frame_num = -1;
    if (h->cur_pic_ptr) {
        h->cur_pic_ptr->reference = 0;
        for (j=i=0; h->delayed_pic[i]; i++)
            if (h->delayed_pic[i] != h->cur_pic_ptr)
                h->delayed_pic[j++] = h->delayed_pic[i];
        h->delayed_pic[j] = NULL;
    }
    ff_h264_unref_picture(&h->last_pic_for_ec);

    h->first_field = 0;
    h->recovery_frame = -1;
    h->frame_recovered = 0;
    h->current_slice = 0;
    h->mmco_reset = 1;
}

static void h264_decode_flush(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    int i;

    memset(h->delayed_pic, 0, sizeof(h->delayed_pic));

    ff_h264_flush_change(h);
    ff_h264_sei_uninit(&h->sei);

    for (i = 0; i < H264_MAX_PICTURE_COUNT; i++)
        ff_h264_unref_picture(&h->DPB[i]);
    h->cur_pic_ptr = NULL;
    ff_h264_unref_picture(&h->cur_pic);

    h->mb_y = 0;
    h->non_gray = 0;

    ff_h264_free_tables(h);
    h->context_initialized = 0;

    if (FF_HW_HAS_CB(avctx, flush))
        FF_HW_SIMPLE_CALL(avctx, flush);
}

static int get_last_needed_nal(H264Context *h)
{
    int nals_needed = 0;
    int slice_type = 0;
    int picture_intra_only = 1;
    int first_slice = 0;
    int i, ret;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        GetBitContext gb;

        /* packets can sometimes contain multiple PPS/SPS,
         * e.g. two PAFF field pictures in one packet, or a demuxer
         * which splits NALs strangely if so, when frame threading we
         * can't start the next thread until we've read all of them */
        switch (nal->type) {
        case H264_NAL_SPS:
        case H264_NAL_PPS:
            nals_needed = i;
            break;
        case H264_NAL_DPA:
        case H264_NAL_IDR_SLICE:
        case H264_NAL_SLICE:
            ret = init_get_bits8(&gb, nal->data + 1, nal->size - 1);
            if (ret < 0) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid zero-sized VCL NAL unit\n");
                if (h->avctx->err_recognition & AV_EF_EXPLODE)
                    return ret;

                break;
            }
            if (!get_ue_golomb_long(&gb) ||  // first_mb_in_slice
                !first_slice ||
                first_slice != nal->type)
                nals_needed = i;
            slice_type = get_ue_golomb_31(&gb);
            if (slice_type > 9)
                slice_type = 0;
            if (slice_type > 4)
                slice_type -= 5;

            slice_type = ff_h264_golomb_to_pict_type[slice_type];
            picture_intra_only &= (slice_type & 3) == AV_PICTURE_TYPE_I;
            if (!first_slice)
                first_slice = nal->type;
        }
    }

    h->picture_intra_only = picture_intra_only;

    return nals_needed;
}

static void debug_green_metadata(const H264SEIGreenMetaData *gm, void *logctx)
{
    av_log(logctx, AV_LOG_DEBUG, "Green Metadata Info SEI message\n");
    av_log(logctx, AV_LOG_DEBUG, "  green_metadata_type: %d\n", gm->green_metadata_type);

    if (gm->green_metadata_type == 0) {
        av_log(logctx, AV_LOG_DEBUG, "  green_metadata_period_type: %d\n", gm->period_type);

        if (gm->period_type == 2)
            av_log(logctx, AV_LOG_DEBUG, "  green_metadata_num_seconds: %d\n", gm->num_seconds);
        else if (gm->period_type == 3)
            av_log(logctx, AV_LOG_DEBUG, "  green_metadata_num_pictures: %d\n", gm->num_pictures);

        av_log(logctx, AV_LOG_DEBUG, "  SEI GREEN Complexity Metrics: %f %f %f %f\n",
               (float)gm->percent_non_zero_macroblocks/255,
               (float)gm->percent_intra_coded_macroblocks/255,
               (float)gm->percent_six_tap_filtering/255,
               (float)gm->percent_alpha_point_deblocking_instance/255);

    } else if (gm->green_metadata_type == 1) {
        av_log(logctx, AV_LOG_DEBUG, "  xsd_metric_type: %d\n", gm->xsd_metric_type);

        if (gm->xsd_metric_type == 0)
            av_log(logctx, AV_LOG_DEBUG, "  xsd_metric_value: %f\n",
                   (float)gm->xsd_metric_value/100);
    }
}

static int decode_nal_units(H264Context *h, AVBufferRef *buf_ref,
                            const uint8_t *buf, int buf_size)
{
    AVCodecContext *const avctx = h->avctx;
    int nals_needed = 0; ///< number of NALs that need decoding before the next frame thread starts
    int idr_cleared=0;
    int i, ret = 0;

    h->has_slice = 0;
    h->nal_unit_type= 0;

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS)) {
        h->current_slice = 0;
        if (!h->first_field) {
            h->cur_pic_ptr = NULL;
            ff_h264_sei_uninit(&h->sei);
        }
    }

    if (h->nal_length_size == 4) {
        if (buf_size > 8 && AV_RB32(buf) == 1 && AV_RB32(buf+5) > (unsigned)buf_size) {
            h->is_avc = 0;
        }else if(buf_size > 3 && AV_RB32(buf) > 1 && AV_RB32(buf) <= (unsigned)buf_size)
            h->is_avc = 1;
    }

    ret = ff_h2645_packet_split(&h->pkt, buf, buf_size, avctx, h->nal_length_size,
                                avctx->codec_id, !!h->is_avc * H2645_FLAG_IS_NALFF);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Error splitting the input into NAL units.\n");
        return ret;
    }

    if (avctx->active_thread_type & FF_THREAD_FRAME)
        nals_needed = get_last_needed_nal(h);
    if (nals_needed < 0)
        return nals_needed;

    for (i = 0; i < h->pkt.nb_nals; i++) {
        H2645NAL *nal = &h->pkt.nals[i];
        int max_slice_ctx, err;

        if (avctx->skip_frame >= AVDISCARD_NONREF &&
            nal->ref_idc == 0 && nal->type != H264_NAL_SEI)
            continue;

        // FIXME these should stop being context-global variables
        h->nal_ref_idc   = nal->ref_idc;
        h->nal_unit_type = nal->type;

        err = 0;
        switch (nal->type) {
        case H264_NAL_IDR_SLICE:
            if ((nal->data[1] & 0xFC) == 0x98) {
                av_log(h->avctx, AV_LOG_ERROR, "Invalid inter IDR frame\n");
                h->next_outputed_poc = INT_MIN;
                ret = -1;
                goto end;
            }
            if(!idr_cleared) {
                idr(h); // FIXME ensure we don't lose some frames if there is reordering
            }
            idr_cleared = 1;
            h->has_recovery_point = 1;
        case H264_NAL_SLICE:
            h->has_slice = 1;

            if ((err = ff_h264_queue_decode_slice(h, nal))) {
                H264SliceContext *sl = h->slice_ctx + h->nb_slice_ctx_queued;
                sl->ref_count[0] = sl->ref_count[1] = 0;
                break;
            }

            if (h->current_slice == 1) {
                if (avctx->active_thread_type & FF_THREAD_FRAME &&
                    i >= nals_needed && !h->setup_finished && h->cur_pic_ptr) {
                    ff_thread_finish_setup(avctx);
                    h->setup_finished = 1;
                }

                if (h->avctx->hwaccel &&
                    (ret = FF_HW_CALL(h->avctx, start_frame, buf_ref,
                                      buf, buf_size)) < 0)
                    goto end;
            }

            max_slice_ctx = avctx->hwaccel ? 1 : h->nb_slice_ctx;
            if (h->nb_slice_ctx_queued == max_slice_ctx) {
                if (h->avctx->hwaccel) {
                    ret = FF_HW_CALL(avctx, decode_slice, nal->raw_data, nal->raw_size);
                    h->nb_slice_ctx_queued = 0;
                } else
                    ret = ff_h264_execute_decode_slices(h);
                if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                    goto end;
            }
            break;
        case H264_NAL_DPA:
        case H264_NAL_DPB:
        case H264_NAL_DPC:
            avpriv_request_sample(avctx, "data partitioning");
            break;
        case H264_NAL_SEI:
            if (h->setup_finished) {
                avpriv_request_sample(avctx, "Late SEI");
                break;
            }
            ret = ff_h264_sei_decode(&h->sei, &nal->gb, &h->ps, avctx);
            h->has_recovery_point = h->has_recovery_point || h->sei.recovery_point.recovery_frame_cnt != -1;
            if (avctx->debug & FF_DEBUG_GREEN_MD)
                debug_green_metadata(&h->sei.green_metadata, h->avctx);
            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
            break;
        case H264_NAL_SPS: {
            GetBitContext tmp_gb = nal->gb;
            if (FF_HW_HAS_CB(avctx, decode_params)) {
                ret = FF_HW_CALL(avctx, decode_params,
                                 nal->type, nal->raw_data, nal->raw_size);
                if (ret < 0)
                    goto end;
            }
            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;
            av_log(h->avctx, AV_LOG_DEBUG,
                   "SPS decoding failure, trying again with the complete NAL\n");
            init_get_bits8(&tmp_gb, nal->raw_data + 1, nal->raw_size - 1);
            if (ff_h264_decode_seq_parameter_set(&tmp_gb, avctx, &h->ps, 0) >= 0)
                break;
            ff_h264_decode_seq_parameter_set(&nal->gb, avctx, &h->ps, 1);
            break;
        }
        case H264_NAL_PPS:
            if (FF_HW_HAS_CB(avctx, decode_params)) {
                ret = FF_HW_CALL(avctx, decode_params,
                                 nal->type, nal->raw_data, nal->raw_size);
                if (ret < 0)
                    goto end;
            }
            ret = ff_h264_decode_picture_parameter_set(&nal->gb, avctx, &h->ps,
                                                       nal->size_bits);
            if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
                goto end;
            break;
        case H264_NAL_AUD:
        case H264_NAL_END_SEQUENCE:
        case H264_NAL_END_STREAM:
        case H264_NAL_FILLER_DATA:
        case H264_NAL_SPS_EXT:
        case H264_NAL_AUXILIARY_SLICE:
            break;
        default:
            av_log(avctx, AV_LOG_DEBUG, "Unknown NAL code: %d (%d bits)\n",
                   nal->type, nal->size_bits);
        }

        if (err < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE)) {
            av_log(h->avctx, AV_LOG_ERROR, "decode_slice_header error\n");
            ret = err;
            goto end;
        }
    }

    ret = ff_h264_execute_decode_slices(h);
    if (ret < 0 && (h->avctx->err_recognition & AV_EF_EXPLODE))
        goto end;

    // set decode_error_flags to allow users to detect concealed decoding errors
    if ((ret < 0 || h->er.error_occurred) && h->cur_pic_ptr) {
        if (h->cur_pic_ptr->decode_error_flags) {
            /* Frame-threading in use */
            atomic_int *decode_error = h->cur_pic_ptr->decode_error_flags;
            /* Using atomics here is not supposed to provide syncronisation;
             * they are merely used to allow to set decode_error from both
             * decoding threads in case of coded slices. */
            atomic_fetch_or_explicit(decode_error, FF_DECODE_ERROR_DECODE_SLICES,
                                     memory_order_relaxed);
        } else
            h->cur_pic_ptr->f->decode_error_flags |= FF_DECODE_ERROR_DECODE_SLICES;
    }

    ret = 0;
end:

#if CONFIG_ERROR_RESILIENCE
    /*
     * FIXME: Error handling code does not seem to support interlaced
     * when slices span multiple rows
     * The ff_er_add_slice calls don't work right for bottom
     * fields; they cause massive erroneous error concealing
     * Error marking covers both fields (top and bottom).
     * This causes a mismatched s->error_count
     * and a bad error table. Further, the error count goes to
     * INT_MAX when called for bottom field, because mb_y is
     * past end by one (callers fault) and resync_mb_y != 0
     * causes problems for the first MB line, too.
     */
    if (!FIELD_PICTURE(h) && h->current_slice && h->enable_er) {

        H264SliceContext *sl = h->slice_ctx;
        int use_last_pic = h->last_pic_for_ec.f->buf[0] && !sl->ref_count[0];
        int decode_error_flags = 0;

        ff_h264_set_erpic(&h->er.cur_pic, h->cur_pic_ptr);

        if (use_last_pic) {
            ff_h264_set_erpic(&h->er.last_pic, &h->last_pic_for_ec);
            sl->ref_list[0][0].parent = &h->last_pic_for_ec;
            memcpy(sl->ref_list[0][0].data, h->last_pic_for_ec.f->data, sizeof(sl->ref_list[0][0].data));
            memcpy(sl->ref_list[0][0].linesize, h->last_pic_for_ec.f->linesize, sizeof(sl->ref_list[0][0].linesize));
            sl->ref_list[0][0].reference = h->last_pic_for_ec.reference;
        } else if (sl->ref_count[0]) {
            ff_h264_set_erpic(&h->er.last_pic, sl->ref_list[0][0].parent);
        } else
            ff_h264_set_erpic(&h->er.last_pic, NULL);

        if (sl->ref_count[1])
            ff_h264_set_erpic(&h->er.next_pic, sl->ref_list[1][0].parent);

        ff_er_frame_end(&h->er, &decode_error_flags);
        if (decode_error_flags) {
            if (h->cur_pic_ptr->decode_error_flags) {
                atomic_int *decode_error = h->cur_pic_ptr->decode_error_flags;
                atomic_fetch_or_explicit(decode_error, decode_error_flags,
                                         memory_order_relaxed);
            } else
                h->cur_pic_ptr->f->decode_error_flags |= decode_error_flags;
        }
        if (use_last_pic)
            memset(&sl->ref_list[0][0], 0, sizeof(sl->ref_list[0][0]));
    }
#endif /* CONFIG_ERROR_RESILIENCE */
    /* clean up */
    if (h->cur_pic_ptr && !h->droppable && h->has_slice) {
        ff_thread_report_progress(&h->cur_pic_ptr->tf, INT_MAX,
                                  h->picture_structure == PICT_BOTTOM_FIELD);
    }

    return (ret < 0) ? ret : buf_size;
}

static int h264_export_enc_params(AVFrame *f, const H264Picture *p)
{
    AVVideoEncParams *par;
    unsigned int nb_mb = p->mb_height * p->mb_width;
    unsigned int x, y;

    par = av_video_enc_params_create_side_data(f, AV_VIDEO_ENC_PARAMS_H264, nb_mb);
    if (!par)
        return AVERROR(ENOMEM);

    par->qp = p->pps->init_qp;

    par->delta_qp[1][0] = p->pps->chroma_qp_index_offset[0];
    par->delta_qp[1][1] = p->pps->chroma_qp_index_offset[0];
    par->delta_qp[2][0] = p->pps->chroma_qp_index_offset[1];
    par->delta_qp[2][1] = p->pps->chroma_qp_index_offset[1];

    for (y = 0; y < p->mb_height; y++)
        for (x = 0; x < p->mb_width; x++) {
            const unsigned int block_idx = y * p->mb_width + x;
            const unsigned int     mb_xy = y * p->mb_stride + x;
            AVVideoBlockParams *b = av_video_enc_params_block(par, block_idx);

            b->src_x = x * 16;
            b->src_y = y * 16;
            b->w     = 16;
            b->h     = 16;

            b->delta_qp = p->qscale_table[mb_xy] - par->qp;
        }

    return 0;
}

static int output_frame(H264Context *h, AVFrame *dst, H264Picture *srcp)
{
    int ret;

    ret = av_frame_ref(dst, srcp->needs_fg ? srcp->f_grain : srcp->f);
    if (ret < 0)
        return ret;

    if (srcp->needs_fg && (ret = av_frame_copy_props(dst, srcp->f)) < 0)
        return ret;

    if (srcp->decode_error_flags) {
        atomic_int *decode_error = srcp->decode_error_flags;
        /* The following is not supposed to provide synchronisation at all:
         * given that srcp has already finished decoding, decode_error
         * has already been set to its final value. */
        dst->decode_error_flags |= atomic_load_explicit(decode_error, memory_order_relaxed);
    }

    av_dict_set(&dst->metadata, "stereo_mode", ff_h264_sei_stereo_mode(&h->sei.common.frame_packing), 0);

    if (srcp->sei_recovery_frame_cnt == 0)
        dst->flags |= AV_FRAME_FLAG_KEY;

    if (h->avctx->export_side_data & AV_CODEC_EXPORT_DATA_VIDEO_ENC_PARAMS) {
        ret = h264_export_enc_params(dst, srcp);
        if (ret < 0)
            goto fail;
    }

    if (!(h->avctx->export_side_data & AV_CODEC_EXPORT_DATA_FILM_GRAIN))
        av_frame_remove_side_data(dst, AV_FRAME_DATA_FILM_GRAIN_PARAMS);

    return 0;
fail:
    av_frame_unref(dst);
    return ret;
}

static int is_avcc_extradata(const uint8_t *buf, int buf_size)
{
    int cnt= buf[5]&0x1f;
    const uint8_t *p= buf+6;
    if (!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 7)
            return 0;
        p += nalsize;
    }
    cnt = *(p++);
    if(!cnt)
        return 0;
    while(cnt--){
        int nalsize= AV_RB16(p) + 2;
        if(nalsize > buf_size - (p-buf) || (p[2] & 0x9F) != 8)
            return 0;
        p += nalsize;
    }
    return 1;
}

static int finalize_frame(H264Context *h, AVFrame *dst, H264Picture *out, int *got_frame)
{
    int ret;

    if (((h->avctx->flags & AV_CODEC_FLAG_OUTPUT_CORRUPT) ||
         (h->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL) ||
         out->recovered)) {

        if (h->skip_gray > 0 &&
            h->non_gray && out->gray &&
            !(h->avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL)
        )
            return 0;

        if (!h->avctx->hwaccel &&
            (out->field_poc[0] == INT_MAX ||
             out->field_poc[1] == INT_MAX)
           ) {
            int p;
            AVFrame *f = out->f;
            int field = out->field_poc[0] == INT_MAX;
            uint8_t *dst_data[4];
            int linesizes[4];
            const uint8_t *src_data[4];

            av_log(h->avctx, AV_LOG_DEBUG, "Duplicating field %d to fill missing\n", field);

            for (p = 0; p<4; p++) {
                dst_data[p] = f->data[p] + (field^1)*f->linesize[p];
                src_data[p] = f->data[p] +  field   *f->linesize[p];
                linesizes[p] = 2*f->linesize[p];
            }

            av_image_copy(dst_data, linesizes, src_data, linesizes,
                          f->format, f->width, f->height>>1);
        }

        ret = output_frame(h, dst, out);
        if (ret < 0)
            return ret;

        *got_frame = 1;

        if (CONFIG_MPEGVIDEODEC) {
            ff_print_debug_info2(h->avctx, dst,
                                 out->mb_type,
                                 out->qscale_table,
                                 out->motion_val,
                                 out->mb_width, out->mb_height, out->mb_stride, 1);
        }
    }

    return 0;
}

static int send_next_delayed_frame(H264Context *h, AVFrame *dst_frame,
                                   int *got_frame, int buf_index)
{
    int ret, i, out_idx;
    H264Picture *out;

    h->cur_pic_ptr = NULL;
    h->first_field = 0;

    while (h->delayed_pic[0]) {
        out = h->delayed_pic[0];
        out_idx = 0;
        for (i = 1;
             h->delayed_pic[i] &&
             !(h->delayed_pic[i]->f->flags & AV_FRAME_FLAG_KEY) &&
             !h->delayed_pic[i]->mmco_reset;
             i++)
            if (h->delayed_pic[i]->poc < out->poc) {
                out     = h->delayed_pic[i];
                out_idx = i;
            }

        for (i = out_idx; h->delayed_pic[i]; i++)
            h->delayed_pic[i] = h->delayed_pic[i + 1];

        if (out) {
            h->frame_recovered |= out->recovered;
            out->recovered |= h->frame_recovered & FRAME_RECOVERED_SEI;

            out->reference &= ~DELAYED_PIC_REF;
            ret = finalize_frame(h, dst_frame, out, got_frame);
            if (ret < 0)
                return ret;
            if (*got_frame)
                break;
        }
    }

    return buf_index;
}

static int h264_decode_frame(AVCodecContext *avctx, AVFrame *pict,
                             int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    H264Context *h     = avctx->priv_data;
    int buf_index;
    int ret;

    h->flags = avctx->flags;
    h->setup_finished = 0;
    h->nb_slice_ctx_queued = 0;

    ff_h264_unref_picture(&h->last_pic_for_ec);

    /* end of stream, output what is still in the buffers */
    if (buf_size == 0)
        return send_next_delayed_frame(h, pict, got_frame, 0);

    if (av_packet_get_side_data(avpkt, AV_PKT_DATA_NEW_EXTRADATA, NULL)) {
        size_t side_size;
        uint8_t *side = av_packet_get_side_data(avpkt, AV_PKT_DATA_NEW_EXTRADATA, &side_size);
        ff_h264_decode_extradata(side, side_size,
                                 &h->ps, &h->is_avc, &h->nal_length_size,
                                 avctx->err_recognition, avctx);
    }
    if (h->is_avc && buf_size >= 9 && buf[0]==1 && buf[2]==0 && (buf[4]&0xFC)==0xFC) {
        if (is_avcc_extradata(buf, buf_size))
            return ff_h264_decode_extradata(buf, buf_size,
                                            &h->ps, &h->is_avc, &h->nal_length_size,
                                            avctx->err_recognition, avctx);
    }

    buf_index = decode_nal_units(h, avpkt->buf, buf, buf_size);
    if (buf_index < 0)
        return AVERROR_INVALIDDATA;

    if (!h->cur_pic_ptr && h->nal_unit_type == H264_NAL_END_SEQUENCE) {
        av_assert0(buf_index <= buf_size);
        return send_next_delayed_frame(h, pict, got_frame, buf_index);
    }

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS) && (!h->cur_pic_ptr || !h->has_slice)) {
        if (avctx->skip_frame >= AVDISCARD_NONREF ||
            buf_size >= 4 && !memcmp("Q264", buf, 4))
            return buf_size;
        av_log(avctx, AV_LOG_ERROR, "no frame!\n");
        return AVERROR_INVALIDDATA;
    }

    if (!(avctx->flags2 & AV_CODEC_FLAG2_CHUNKS) ||
        (h->mb_y >= h->mb_height && h->mb_height)) {
        if ((ret = ff_h264_field_end(h, &h->slice_ctx[0], 0)) < 0)
            return ret;

        /* Wait for second field. */
        if (h->next_output_pic) {
            ret = finalize_frame(h, pict, h->next_output_pic, got_frame);
            if (ret < 0)
                return ret;
        }
    }

    av_assert0(pict->buf[0] || !*got_frame);

    ff_h264_unref_picture(&h->last_pic_for_ec);

    return buf_size;
}

#define OFFSET(x) offsetof(H264Context, x)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
#define VDX VD | AV_OPT_FLAG_EXPORT
static const AVOption h264_options[] = {
    { "is_avc", "is avc", OFFSET(is_avc), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VDX },
    { "nal_length_size", "nal_length_size", OFFSET(nal_length_size), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 4, VDX },
    { "enable_er", "Enable error resilience on damaged frames (unsafe)", OFFSET(enable_er), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, VD },
    { "x264_build", "Assume this x264 version if no x264 version found in any SEI", OFFSET(x264_build), AV_OPT_TYPE_INT, {.i64 = -1}, -1, INT_MAX, VD },
    { "skip_gray", "Do not return gray gap frames", OFFSET(skip_gray), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VD },
    { "noref_gray", "Avoid using gray gap frames as references", OFFSET(noref_gray), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, VD },
    { NULL },
};

static const AVClass h264_class = {
    .class_name = "H264 Decoder",
    .option     = h264_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_h264_decoder = {
    .p.name                = "h264",
    CODEC_LONG_NAME("H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_H264,
    .priv_data_size        = sizeof(H264Context),
    .init                  = h264_decode_init,
    .close                 = h264_decode_end,
    FF_CODEC_DECODE_CB(h264_decode_frame),
    .p.capabilities        = AV_CODEC_CAP_DR1 |
                             AV_CODEC_CAP_DELAY | AV_CODEC_CAP_SLICE_THREADS |
                             AV_CODEC_CAP_FRAME_THREADS,
    .hw_configs            = (const AVCodecHWConfigInternal *const []) {
#if CONFIG_H264_DXVA2_HWACCEL
                               HWACCEL_DXVA2(h264),
#endif
#if CONFIG_H264_D3D11VA_HWACCEL
                               HWACCEL_D3D11VA(h264),
#endif
#if CONFIG_H264_D3D11VA2_HWACCEL
                               HWACCEL_D3D11VA2(h264),
#endif
#if CONFIG_H264_D3D12VA_HWACCEL
                               HWACCEL_D3D12VA(h264),
#endif
#if CONFIG_H264_NVDEC_HWACCEL
                               HWACCEL_NVDEC(h264),
#endif
#if CONFIG_H264_VAAPI_HWACCEL
                               HWACCEL_VAAPI(h264),
#endif
#if CONFIG_H264_VDPAU_HWACCEL
                               HWACCEL_VDPAU(h264),
#endif
#if CONFIG_H264_VIDEOTOOLBOX_HWACCEL
                               HWACCEL_VIDEOTOOLBOX(h264),
#endif
#if CONFIG_H264_VULKAN_HWACCEL
                               HWACCEL_VULKAN(h264),
#endif
                               NULL
                           },
    .caps_internal         = FF_CODEC_CAP_EXPORTS_CROPPING |
                             FF_CODEC_CAP_INIT_CLEANUP,
    .flush                 = h264_decode_flush,
    UPDATE_THREAD_CONTEXT(ff_h264_update_thread_context),
    UPDATE_THREAD_CONTEXT_FOR_USER(ff_h264_update_thread_context_for_user),
    .p.profiles            = NULL_IF_CONFIG_SMALL(ff_h264_profiles),
    .p.priv_class          = &h264_class,
};
