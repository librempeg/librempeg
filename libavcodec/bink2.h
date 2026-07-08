/*
 * Bink video 2 decoder
 * Copyright (c) 2014 Konstantin Shishkov
 * Copyright (c) 2019 Paul B Mahol
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

#ifndef AVCODEC_BINK2_H
#define AVCODEC_BINK2_H

#include <stdint.h>

#include "libavutil/mem_internal.h"
#include "avcodec.h"
#include "blockdsp.h"
#include "copy_block.h"
#include "unary.h"
#include "get_bits.h"

static VLC bink2f_quant_vlc;
static VLC bink2f_ac_val0_vlc;
static VLC bink2f_ac_val1_vlc;
static VLC bink2f_ac_skip0_vlc;
static VLC bink2f_ac_skip1_vlc;
static VLC bink2g_ac_skip0_vlc;
static VLC bink2g_ac_skip1_vlc;
static VLC bink2g_mv_vlc;

static const uint8_t kb2h_num_slices[] = {
    2, 3, 4, 8,
};

static const uint8_t luma_repos[] = {
    0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15,
};

static const uint8_t dq_patterns[8] = { 8, 0, 1, 0, 2, 0, 1, 0 };

static const uint8_t bink2_next_skips[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0,
};

#define NUM_AC_SKIPS 14
#define BINK_FLAG_ALPHA 0x00100000
#define DC_MPRED(A, B, C) FFMIN(FFMAX((C) + (B) - (A), FFMIN3(A, B, C)), FFMAX3(A, B, C))
#define DC_MPRED2(A, B) FFMIN(FFMAX((A), (B)), FFMAX(FFMIN((A), (B)), 2 * (A) - (B)))
#define LHFILTER(src)    (((((src)[0]+(src)[1])*19 >> 1)-((src)[-1]+(src)[2  ])*2+(((src)[-2  ]+(src)[3  ])>>1)+8)>>4)
#define LVFILTER(src, i) (((((src)[0]+(src)[i])*19 >> 1)-((src)[-i]+(src)[2*i])*2+(((src)[-2*i]+(src)[3*i])>>1)+8)>>4)

typedef struct QuantPredict {
    int8_t intra_q;
    int8_t inter_q;
} QuantPredict;

typedef struct DCPredict {
    float dc[4][16];
    int   block_type;
} DCPredict;

typedef struct DCIPredict {
    int dc[4][16];
    int block_type;
} DCIPredict;

typedef struct MVectors {
    int v[4][2];
    int nb_vectors;
} MVectors;

typedef struct MVPredict {
    MVectors mv;
} MVPredict;

/*
 * Decoder context
 */
typedef struct Bink2Context {
    AVCodecContext  *avctx;
    GetBitContext   gb;
    BlockDSPContext dsp;
    AVFrame         *last;
    int             version;              ///< internal Bink file version
    int             has_alpha;

    DECLARE_ALIGNED(16, float, block[4][64]);
    DECLARE_ALIGNED(16, int16_t, iblock[4][64]);

    QuantPredict    *current_q;
    QuantPredict    *prev_q;

    DCPredict       *current_dc;
    DCPredict       *prev_dc;

    DCIPredict      *current_idc;
    DCIPredict      *prev_idc;

    MVPredict       *current_mv;
    MVPredict       *prev_mv;

    uint8_t         *col_cbp;
    uint8_t         *row_cbp;

    int             num_slices;
    int             slice_height[4];

    int             comp;
    int             mb_pos;
    unsigned        flags;
    unsigned        frame_flags;
} Bink2Context;

/**
 * Bink2 video block types
 */
enum BlockTypes {
    INTRA_BLOCK = 0, ///< intra DCT block
    SKIP_BLOCK,      ///< skipped block
    MOTION_BLOCK,    ///< block is copied from previous frame with some offset
    RESIDUE_BLOCK,   ///< motion block with some difference added
};

#endif /* AVCODEC_BINK2_H */
