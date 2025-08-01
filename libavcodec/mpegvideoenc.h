/*
 * Generic DCT based hybrid video encoder
 * Copyright (c) 2000, 2001, 2002 Fabrice Bellard
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
 * mpegvideo header.
 */

#ifndef AVCODEC_MPEGVIDEOENC_H
#define AVCODEC_MPEGVIDEOENC_H

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/mem_internal.h"
#include "libavutil/opt.h"
#include "fdctdsp.h"
#include "motion_est.h"
#include "mpegvideo.h"
#include "mpegvideoencdsp.h"
#include "pixblockdsp.h"
#include "put_bits.h"
#include "ratecontrol.h"

#define MPVENC_MAX_B_FRAMES 16

typedef struct MPVEncContext {
    MpegEncContext c;           ///< the common base context

    /** bit output */
    PutBitContext pb;

    unsigned int lambda;        ///< Lagrange multiplier used in rate distortion
    unsigned int lambda2;       ///< (lambda*lambda) >> FF_LAMBDA_SHIFT
    int *lambda_table;
    int adaptive_quant;         ///< use adaptive quantization
    int dquant;                 ///< qscale difference to prev qscale
    int skipdct;                ///< skip dct and code zero residual

    int quantizer_noise_shaping;

    int luma_elim_threshold;
    int chroma_elim_threshold;

    int mpv_flags;              ///< flags set by private options
    /// Bitfield containing information which frames to reconstruct.
    int frame_reconstruction_bitfield;

    /**
     * Reference to the source picture.
     */
    AVFrame *new_pic;

    struct MPVMainEncContext *parent;

    FDCTDSPContext fdsp;
    MpegvideoEncDSPContext mpvencdsp;
    PixblockDSPContext pdsp;
    MotionEstContext me;

    int f_code;                 ///< forward MV resolution
    int b_code;                 ///< backward MV resolution for B-frames

    int16_t (*p_mv_table)[2];            ///< MV table (1MV per MB) P-frame
    int16_t (*b_forw_mv_table)[2];       ///< MV table (1MV per MB) forward mode B-frame
    int16_t (*b_back_mv_table)[2];       ///< MV table (1MV per MB) backward mode B-frame
    int16_t (*b_bidir_forw_mv_table)[2]; ///< MV table (1MV per MB) bidir mode B-frame
    int16_t (*b_bidir_back_mv_table)[2]; ///< MV table (1MV per MB) bidir mode B-frame
    int16_t (*b_direct_mv_table)[2];     ///< MV table (1MV per MB) direct mode B-frame
    int16_t (*b_field_mv_table[2][2][2])[2];///< MV table (4MV per MB) interlaced B-frame
    uint8_t (*p_field_select_table[2]);  ///< Only the first element is allocated
    uint8_t (*b_field_select_table[2][2]); ///< allocated jointly with p_field_select_table

    uint16_t *mb_type;          ///< Table for candidate MB types
    uint16_t *mb_var;           ///< Table for MB variances
    uint16_t *mc_mb_var;        ///< Table for motion compensated MB variances
    uint8_t  *mb_mean;          ///< Table for MB luminance
    uint64_t encoding_error[MPV_MAX_PLANES];

    int intra_quant_bias;    ///< bias for the quantizer
    int inter_quant_bias;    ///< bias for the quantizer
    int min_qcoeff;          ///< minimum encodable coefficient
    int max_qcoeff;          ///< maximum encodable coefficient
    int ac_esc_length;       ///< num of bits needed to encode the longest esc
    const uint8_t *intra_ac_vlc_length;
    const uint8_t *intra_ac_vlc_last_length;
    const uint8_t *intra_chroma_ac_vlc_length;
    const uint8_t *intra_chroma_ac_vlc_last_length;
    const uint8_t *inter_ac_vlc_length;
    const uint8_t *inter_ac_vlc_last_length;
    const uint8_t *luma_dc_vlc_length;

    int coded_score[12];

    int16_t (*block)[64];       ///< points into blocks below

    /** precomputed matrix (combine qscale and DCT renorm) */
    int (*q_intra_matrix)[64];
    int (*q_chroma_intra_matrix)[64];
    int (*q_inter_matrix)[64];
    /** identical to the above but for MMX & these are not permutated, second 64 entries are bias*/
    uint16_t (*q_intra_matrix16)[2][64];
    uint16_t (*q_chroma_intra_matrix16)[2][64];
    uint16_t (*q_inter_matrix16)[2][64];

    /* noise reduction */
    void (*denoise_dct)(struct MPVEncContext *s, int16_t *block);
    int (*dct_error_sum)[64];
    int dct_count[2];
    uint16_t (*dct_offset)[64];

    int picture_number;

    /* statistics, used for 2-pass encoding */
    int mv_bits;
    int i_tex_bits;
    int p_tex_bits;
    int i_count;
    int misc_bits; ///< cbp, mb_type
    int last_bits; ///< temp var used for calculating the above vars

    int mb_skip_run;

    /* H.263 specific */
    int gob_index;
    int mb_info;                   ///< interval for outputting info about mb offsets as side data
    int prev_mb_info, last_mb_info;
    int mb_info_size;
    uint8_t *mb_info_ptr;

    /* H.263+ specific */
    int umvplus;                   ///< == H.263+ && unrestricted_mv
    int h263_slice_structured;
    int alt_inter_vlc;             ///< alternative inter vlc
    int modified_quant;
    int loop_filter;

    /* MJPEG specific */
    struct MJpegContext *mjpeg_ctx;
    int esc_pos;

    /* MPEG-1 specific */
    int last_mv_dir;               ///< last mv_dir, used for B-frame encoding

    /* MPEG-4 specific */
    int data_partitioning;         ///< data partitioning flag, set via option
    int partitioned_frame;         ///< is current frame partitioned
    int mpeg_quant;
    PutBitContext tex_pb;          ///< used for data partitioned VOPs
    PutBitContext pb2;             ///< used for data partitioned VOPs

    /* MSMPEG4 specific */
    int slice_height;              ///< in macroblocks
    int flipflop_rounding;         ///< also used for MPEG-4, H.263+
    int esc3_level_length;

    /* RTP specific */
    int rtp_mode;
    int rtp_payload_size;
    int error_rate;

    uint8_t *ptr_lastgob;

    void (*encode_mb)(struct MPVEncContext *s, int16_t block[][64],
                      int motion_x, int motion_y);

    int (*dct_quantize)(struct MPVEncContext *s, int16_t *block/*align 16*/, int n, int qscale, int *overflow);

    me_cmp_func ildct_cmp[2]; ///< 0 = intra, 1 = non-intra
    me_cmp_func n_sse_cmp[2]; ///< either SSE or NSSE cmp func
    me_cmp_func sad_cmp[2];
    me_cmp_func sse_cmp[2];
    int (*sum_abs_dctelem)(const int16_t *block);

    int intra_penalty;

    DECLARE_ALIGNED_32(int16_t, blocks)[2][12][64]; // for HQ mode we need to keep the best block
} MPVEncContext;

typedef struct MPVMainEncContext {
    MPVEncContext s;               ///< The main slicecontext

    int intra_only;                ///< if true, only intra pictures are generated
    int gop_size;
    int max_b_frames;              ///< max number of B-frames
    int picture_in_gop_number;     ///< 0-> first pic in gop, ...
    int input_picture_number;      ///< used to set pic->display_picture_number
    int coded_picture_number;      ///< used to set pic->coded_picture_number

    MPVPicture *input_picture[MPVENC_MAX_B_FRAMES + 1]; ///< next pictures in display order
    MPVPicture *reordered_input_picture[MPVENC_MAX_B_FRAMES + 1]; ///< next pictures in coded order

    int64_t user_specified_pts;    ///< last non-zero pts from user-supplied AVFrame
    /**
     * pts difference between the first and second input frame, used for
     * calculating dts of the first frame when there's a delay */
    int64_t dts_delta;
    /**
     * reordered pts to be used as dts for the next output frame when there's
     * a delay */
    int64_t reordered_pts;

    /// temporary frames used by b_frame_strategy = 2
    AVFrame *tmp_frames[MPVENC_MAX_B_FRAMES + 2];
    int b_frame_strategy;
    int b_sensitivity;
    int brd_scale;

    int scenechange_threshold;

    int noise_reduction;

    float border_masking;
    int lmin, lmax;
    int vbv_ignore_qmax;

    /* MPEG-1/2 specific */
    int vbv_delay_pos;             ///< offset of vbv_delay in the bitstream

    const uint8_t *fcode_tab;      ///< smallest fcode needed for each MV

    /* frame skip options */
    int frame_skip_threshold;
    int frame_skip_factor;
    int frame_skip_exp;
    int frame_skip_cmp;
    me_cmp_func frame_skip_cmp_fn;

    int (*encode_picture_header)(struct MPVMainEncContext *m);

    /* bit rate control */
    int64_t bit_rate;
    int64_t total_bits;
    int frame_bits;                ///< bits used for the current frame
    int header_bits;
    int stuffing_bits;             ///< bits used for stuffing
    int next_lambda;               ///< next lambda used for retrying to encode a frame
    int fixed_qscale;              ///< fixed qscale if non zero
    int last_lambda_for[5];        ///< last lambda for a specific pict type
    int last_pict_type;            //FIXME removes
    int last_non_b_pict_type;      ///< used for MPEG-4 gmc B-frames & ratecontrol
    RateControlContext rc_context; ///< contains stuff only accessed in ratecontrol.c

    int me_penalty_compensation;
    int me_pre;                          ///< prepass for motion estimation

    int64_t mb_var_sum;            ///< sum of MB variance for current frame
    int64_t mc_mb_var_sum;         ///< motion compensated MB variance for current frame

    char *me_map_base;             ///< backs MotionEstContext.(map|score_map)
    char *dct_error_sum_base;      ///< backs dct_error_sum
    int16_t (*mv_table_base)[2];
} MPVMainEncContext;

static inline const MPVMainEncContext *slice_to_mainenc(const MPVEncContext *s)
{
#ifdef NO_SLICE_THREADING_HERE
    av_assert2(s->c.slice_context_count <= 1 &&
               !(s->c.avctx->codec->capabilities & AV_CODEC_CAP_SLICE_THREADS));
    return (const MPVMainEncContext*)s;
#else
    return s->parent;
#endif
}

#define MAX_FCODE        7
#define UNI_AC_ENC_INDEX(run,level) ((run)*128 + (level))
#define INPLACE_OFFSET 16

/* MB types for encoding */
#define CANDIDATE_MB_TYPE_INTRA      (1 <<  0)
#define CANDIDATE_MB_TYPE_INTER      (1 <<  1)
#define CANDIDATE_MB_TYPE_INTER4V    (1 <<  2)
#define CANDIDATE_MB_TYPE_SKIPPED    (1 <<  3)

#define CANDIDATE_MB_TYPE_DIRECT     (1 <<  4)
#define CANDIDATE_MB_TYPE_FORWARD    (1 <<  5)
#define CANDIDATE_MB_TYPE_BACKWARD   (1 <<  6)
#define CANDIDATE_MB_TYPE_BIDIR      (1 <<  7)

#define CANDIDATE_MB_TYPE_INTER_I    (1 <<  8)
#define CANDIDATE_MB_TYPE_FORWARD_I  (1 <<  9)
#define CANDIDATE_MB_TYPE_BACKWARD_I (1 << 10)
#define CANDIDATE_MB_TYPE_BIDIR_I    (1 << 11)

#define CANDIDATE_MB_TYPE_DIRECT0    (1 << 12)

/* mpegvideo_enc common options */
#define FF_MPV_FLAG_SKIP_RD      0x0001
#define FF_MPV_FLAG_STRICT_GOP   0x0002
#define FF_MPV_FLAG_QP_RD        0x0004
#define FF_MPV_FLAG_CBP_RD       0x0008
#define FF_MPV_FLAG_NAQ          0x0010
#define FF_MPV_FLAG_MV0          0x0020

#define FF_MPV_OPT_CMP_FUNC \
{ "sad",    "Sum of absolute differences, fast", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_SAD }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "sse",    "Sum of squared errors", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_SSE }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "satd",   "Sum of absolute Hadamard transformed differences", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_SATD }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "dct",    "Sum of absolute DCT transformed differences", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_DCT }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "psnr",   "Sum of squared quantization errors, low quality", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_PSNR }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "bit",    "Number of bits needed for the block", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_BIT }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "rd",     "Rate distortion optimal, slow", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_RD }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "zero",   "Zero", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_ZERO }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "vsad",   "Sum of absolute vertical differences", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_VSAD }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "vsse",   "Sum of squared vertical differences", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_VSSE }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "nsse",   "Noise preserving sum of squared differences", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_NSSE }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "dct264", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_DCT264 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "dctmax", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_DCTMAX }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "chroma", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_CHROMA }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{ "msad",   "Sum of absolute differences, median predicted", 0, AV_OPT_TYPE_CONST, {.i64 = FF_CMP_MEDIAN_SAD }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }

#define FF_MPV_OFFSET(x) offsetof(MPVEncContext, x)
#define FF_MPV_MAIN_OFFSET(x) offsetof(MPVMainEncContext, x)
#define FF_RC_OFFSET(x)  offsetof(MPVMainEncContext, rc_context.x)
#define FF_MPV_OPT_FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM)
#define FF_MPV_COMMON_OPTS \
FF_MPV_OPT_CMP_FUNC, \
{ "mpv_flags",      "Flags common for all mpegvideo-based encoders.", FF_MPV_OFFSET(mpv_flags), AV_OPT_TYPE_FLAGS, { .i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "skip_rd",        "RD optimal MB level residual skipping", 0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_SKIP_RD },    0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "strict_gop",     "Strictly enforce gop size",             0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_STRICT_GOP }, 0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "qp_rd",          "Use rate distortion optimization for qp selection", 0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_QP_RD },  0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "cbp_rd",         "use rate distortion optimization for CBP",          0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_CBP_RD }, 0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "naq",            "normalize adaptive quantization",                   0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_NAQ },    0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{ "luma_elim_threshold",   "single coefficient elimination threshold for luminance (negative values also consider dc coefficient)",\
                                                                      FF_MPV_OFFSET(luma_elim_threshold), AV_OPT_TYPE_INT, { .i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS },\
{ "chroma_elim_threshold", "single coefficient elimination threshold for chrominance (negative values also consider dc coefficient)",\
                                                                      FF_MPV_OFFSET(chroma_elim_threshold), AV_OPT_TYPE_INT, { .i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS },\
{ "quantizer_noise_shaping", NULL,                                  FF_MPV_OFFSET(quantizer_noise_shaping), AV_OPT_TYPE_INT, { .i64 = 0 },       0, INT_MAX, FF_MPV_OPT_FLAGS },\
{ "error_rate", "Simulate errors in the bitstream to test error concealment.",                                                                                                  \
                                                                    FF_MPV_OFFSET(error_rate),              AV_OPT_TYPE_INT, { .i64 = 0 },       0, INT_MAX, FF_MPV_OPT_FLAGS },\
{"qsquish", "how to keep quantizer between qmin and qmax (0 = clip, 1 = use differentiable function)",                                                                          \
                                                                    FF_RC_OFFSET(qsquish), AV_OPT_TYPE_FLOAT, {.dbl = 0 }, 0, 99, FF_MPV_OPT_FLAGS},                        \
{"rc_qmod_amp", "experimental quantizer modulation",                FF_RC_OFFSET(qmod_amp), AV_OPT_TYPE_FLOAT, {.dbl = 0 }, -FLT_MAX, FLT_MAX, FF_MPV_OPT_FLAGS},           \
{"rc_qmod_freq", "experimental quantizer modulation",               FF_RC_OFFSET(qmod_freq), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS},             \
{"rc_eq", "Set rate control equation. When computing the expression, besides the standard functions "                                                                           \
          "defined in the section 'Expression Evaluation', the following functions are available: "                                                                             \
          "bits2qp(bits), qp2bits(qp). Also the following constants are available: iTex pTex tex mv "                                                                           \
          "fCode iCount mcVar var isI isP isB avgQP qComp avgIITex avgPITex avgPPTex avgBPTex avgTex.",                                                                         \
                                                                    FF_RC_OFFSET(rc_eq), AV_OPT_TYPE_STRING,                           .flags = FF_MPV_OPT_FLAGS },            \
{"rc_init_cplx", "initial complexity for 1-pass encoding",          FF_RC_OFFSET(initial_cplx), AV_OPT_TYPE_FLOAT, {.dbl = 0 }, -FLT_MAX, FLT_MAX, FF_MPV_OPT_FLAGS},       \
{"rc_buf_aggressivity", "currently useless",                        FF_RC_OFFSET(buffer_aggressivity), AV_OPT_TYPE_FLOAT, {.dbl = 1.0 }, -FLT_MAX, FLT_MAX, FF_MPV_OPT_FLAGS}, \
{"border_mask", "increase the quantizer for macroblocks close to borders", FF_MPV_MAIN_OFFSET(border_masking), AV_OPT_TYPE_FLOAT, {.dbl = 0 }, -FLT_MAX, FLT_MAX, FF_MPV_OPT_FLAGS},    \
{"lmin", "minimum Lagrange factor (VBR)",                           FF_MPV_MAIN_OFFSET(lmin), AV_OPT_TYPE_INT, {.i64 =  2*FF_QP2LAMBDA }, 0, INT_MAX, FF_MPV_OPT_FLAGS },            \
{"lmax", "maximum Lagrange factor (VBR)",                           FF_MPV_MAIN_OFFSET(lmax), AV_OPT_TYPE_INT, {.i64 = 31*FF_QP2LAMBDA }, 0, INT_MAX, FF_MPV_OPT_FLAGS },            \
{"skip_threshold", "Frame skip threshold",                          FF_MPV_MAIN_OFFSET(frame_skip_threshold), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"skip_factor", "Frame skip factor",                                FF_MPV_MAIN_OFFSET(frame_skip_factor), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"skip_exp", "Frame skip exponent",                                 FF_MPV_MAIN_OFFSET(frame_skip_exp), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"skip_cmp", "Frame skip compare function",                         FF_MPV_MAIN_OFFSET(frame_skip_cmp), AV_OPT_TYPE_INT, {.i64 = FF_CMP_DCTMAX }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS, .unit = "cmp_func" }, \
{"noise_reduction", "Noise reduction",                              FF_MPV_MAIN_OFFSET(noise_reduction), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"ps", "RTP payload size in bytes",                             FF_MPV_OFFSET(rtp_payload_size), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \

#define FF_MPV_COMMON_BFRAME_OPTS \
{"b_strategy", "Strategy to choose between I/P/B-frames",      FF_MPV_MAIN_OFFSET(b_frame_strategy), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 2, FF_MPV_OPT_FLAGS }, \
{"b_sensitivity", "Adjust sensitivity of b_frame_strategy 1",  FF_MPV_MAIN_OFFSET(b_sensitivity), AV_OPT_TYPE_INT, {.i64 = 40 }, 1, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"brd_scale", "Downscale frames for dynamic B-frame decision", FF_MPV_MAIN_OFFSET(brd_scale), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 3, FF_MPV_OPT_FLAGS },

#define FF_MPV_COMMON_MOTION_EST_OPTS \
{ "mv0",            "always try a mb with mv=<0,0>",                     0, AV_OPT_TYPE_CONST, { .i64 = FF_MPV_FLAG_MV0 },    0, 0, FF_MPV_OPT_FLAGS, .unit = "mpv_flags" },\
{"motion_est", "motion estimation algorithm",                       FF_MPV_OFFSET(me.motion_est), AV_OPT_TYPE_INT, {.i64 = FF_ME_EPZS }, FF_ME_ZERO, FF_ME_XONE, FF_MPV_OPT_FLAGS, .unit = "motion_est" },   \
{ "zero", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ZERO }, 0, 0, FF_MPV_OPT_FLAGS, .unit = "motion_est" }, \
{ "epzs", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_EPZS }, 0, 0, FF_MPV_OPT_FLAGS, .unit = "motion_est" }, \
{ "xone", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_XONE }, 0, 0, FF_MPV_OPT_FLAGS, .unit = "motion_est" }, \
{"mepc", "Motion estimation bitrate penalty compensation (1.0 = 256)", FF_MPV_MAIN_OFFSET(me_penalty_compensation), AV_OPT_TYPE_INT, {.i64 = 256 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"mepre", "pre motion estimation", FF_MPV_MAIN_OFFSET(me_pre), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \
{"intra_penalty", "Penalty for intra blocks in block decision", FF_MPV_OFFSET(intra_penalty), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, INT_MAX/2, FF_MPV_OPT_FLAGS }, \
{"sc_threshold", "Scene change threshold",                          FF_MPV_MAIN_OFFSET(scenechange_threshold), AV_OPT_TYPE_INT, {.i64 = 0 }, INT_MIN, INT_MAX, FF_MPV_OPT_FLAGS }, \

extern const AVClass ff_mpv_enc_class;

int ff_mpv_encode_init(AVCodecContext *avctx);

int ff_mpv_encode_end(AVCodecContext *avctx);
int ff_mpv_encode_picture(AVCodecContext *avctx, AVPacket *pkt,
                          const AVFrame *frame, int *got_packet);
int ff_mpv_reallocate_putbitbuffer(MPVEncContext *s, size_t threshold, size_t size_increase);

void ff_write_quant_matrix(PutBitContext *pb, uint16_t *matrix);

void ff_dct_encode_init(MPVEncContext *s);
void ff_mpvenc_dct_init_mips(MPVEncContext *s);
void ff_dct_encode_init_x86(MPVEncContext *s);

void ff_convert_matrix(MPVEncContext *s, int (*qmat)[64], uint16_t (*qmat16)[2][64],
                       const uint16_t *quant_matrix, int bias, int qmin, int qmax, int intra);

void ff_block_permute(int16_t *block, const uint8_t *permutation,
                      const uint8_t *scantable, int last);

static inline int get_bits_diff(MPVEncContext *s)
{
    const int bits = put_bits_count(&s->pb);
    const int last = s->last_bits;

    s->last_bits = bits;

    return bits - last;
}

#endif
