/*
 * RoQ Video Encoder.
 *
 * Copyright (C) 2007 Vitor Sessak <vitor1001@gmail.com>
 * Copyright (C) 2004-2007 Eric Lasota
 *    Based on RoQ specs (C) 2001 Tim Ferguson
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
 * id RoQ encoder by Vitor. Based on the Switchblade3 library and the
 * Switchblade3 FFmpeg glue by Eric Lasota.
 */

/*
 * COSTS:
 * Level 1:
 *  SKIP - 2 bits
 *  MOTION - 2 + 8 bits
 *  CODEBOOK - 2 + 8 bits
 *  SUBDIVIDE - 2 + combined subcel cost
 *
 * Level 2:
 *  SKIP - 2 bits
 *  MOTION - 2 + 8 bits
 *  CODEBOOK - 2 + 8 bits
 *  SUBDIVIDE - 2 + 4*8 bits
 *
 * Maximum cost: 138 bits per cel
 *
 * Proper evaluation requires LCD fraction comparison, which requires
 * Squared Error (SE) loss * savings increase
 *
 * Maximum savings increase: 136 bits
 * Maximum SE loss without overflow: 31580641
 * Components in 8x8 supercel: 192
 * Maximum SE precision per component: 164482
 *    >65025, so no truncation is needed (phew)
 */

#include <string.h>

#include "libavutil/attributes.h"
#include "libavutil/lfg.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "roqvideo.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "elbg.h"
#include "encode.h"
#include "mathops.h"

#define CHROMA_BIAS 1

/**
 * Maximum number of generated 4x4 codebooks. Can't be 256 to workaround a
 * Quake 3 bug.
 */
#define MAX_CBS_4x4 256

#define MAX_CBS_2x2 256 ///< Maximum number of 2x2 codebooks.

/* The cast is useful when multiplying it by INT_MAX */
#define ROQ_LAMBDA_SCALE ((uint64_t) FF_LAMBDA_SCALE)

typedef struct RoqCodebooks {
    int numCB4;
    int numCB2;
    int usedCB2[MAX_CBS_2x2];
    int usedCB4[MAX_CBS_4x4];
    uint8_t unpacked_cb2[MAX_CBS_2x2*2*2*3];
    uint8_t unpacked_cb4[MAX_CBS_4x4*4*4*3];
    uint8_t unpacked_cb4_enlarged[MAX_CBS_4x4*8*8*3];
} RoqCodebooks;

/**
 * Temporary vars
 */
typedef struct RoqTempData
{
    int f2i4[MAX_CBS_4x4];
    int i2f4[MAX_CBS_4x4];
    int f2i2[MAX_CBS_2x2];
    int i2f2[MAX_CBS_2x2];

    int mainChunkSize;

    int numCB4;
    int numCB2;

    RoqCodebooks codebooks;

    int used_option[4];
} RoqTempData;

typedef struct SubcelEvaluation {
    int eval_dist[4];
    int best_bit_use;
    int best_coding;

    int subCels[4];
    motion_vect motion;
    int cbEntry;
} SubcelEvaluation;

typedef struct CelEvaluation {
    int eval_dist[4];
    int best_coding;

    SubcelEvaluation subCels[4];

    motion_vect motion;
    int cbEntry;

    int sourceX, sourceY;
} CelEvaluation;

typedef struct RoqEncContext {
    RoqContext common;
    struct ELBGContext *elbg;
    AVLFG randctx;
    uint64_t lambda;

    motion_vect *this_motion4;
    motion_vect *last_motion4;

    motion_vect *this_motion8;
    motion_vect *last_motion8;

    unsigned int framesSinceKeyframe;

    const AVFrame *frame_to_enc;
    uint8_t *out_buf;
    RoqTempData tmp_data;
    roq_cell results4[4 * MAX_CBS_4x4];
    int tmp_codebook_buf[FFMAX(24 * MAX_CBS_4x4, 6 * MAX_CBS_2x2)];

    CelEvaluation *cel_evals;
    int *closest_cb;
    int *points;  // Allocated together with closest_cb

    int first_frame;
    int quake3_compat; // Quake 3 compatibility option
} RoqEncContext;

/* Macroblock support functions */
static void unpack_roq_cell(roq_cell *cell, uint8_t u[4*3])
{
    memcpy(u  , cell->y, 4);
    memset(u+4, cell->u, 4);
    memset(u+8, cell->v, 4);
}

static void unpack_roq_qcell(uint8_t cb2[], roq_qcell *qcell, uint8_t u[4*4*3])
{
    int i,cp;
    static const int offsets[4] = {0, 2, 8, 10};

    for (cp=0; cp<3; cp++)
        for (i=0; i<4; i++) {
            u[4*4*cp + offsets[i]  ] = cb2[qcell->idx[i]*2*2*3 + 4*cp  ];
            u[4*4*cp + offsets[i]+1] = cb2[qcell->idx[i]*2*2*3 + 4*cp+1];
            u[4*4*cp + offsets[i]+4] = cb2[qcell->idx[i]*2*2*3 + 4*cp+2];
            u[4*4*cp + offsets[i]+5] = cb2[qcell->idx[i]*2*2*3 + 4*cp+3];
        }
}


static void enlarge_roq_mb4(uint8_t base[3*16], uint8_t u[3*64])
{
    int x,y,cp;

    for(cp=0; cp<3; cp++)
        for(y=0; y<8; y++)
            for(x=0; x<8; x++)
                *u++ = base[(y/2)*4 + (x/2) + 16*cp];
}

static inline int square(int x)
{
    return x*x;
}

static inline int eval_sse(const uint8_t *a, const uint8_t *b, int count)
{
    int diff=0;

    while(count--)
        diff += square(*b++ - *a++);

    return diff;
}

// FIXME Could use DSPContext.sse, but it is not so speed critical (used
// just for motion estimation).
static int block_sse(uint8_t * const *buf1, uint8_t * const *buf2, int x1, int y1,
                     int x2, int y2, const int *stride1, const int *stride2, int size)
{
    int i, k;
    int sse=0;

    for (k=0; k<3; k++) {
        int bias = (k ? CHROMA_BIAS : 4);
        for (i=0; i<size; i++)
            sse += bias*eval_sse(buf1[k] + (y1+i)*stride1[k] + x1,
                                 buf2[k] + (y2+i)*stride2[k] + x2, size);
    }

    return sse;
}

static int eval_motion_dist(RoqEncContext *enc, int x, int y, motion_vect vect,
                             int size)
{
    RoqContext *const roq = &enc->common;
    int mx=vect.d[0];
    int my=vect.d[1];

    if (mx < -7 || mx > 7)
        return INT_MAX;

    if (my < -7 || my > 7)
        return INT_MAX;

    mx += x;
    my += y;

    if ((unsigned) mx > roq->width-size || (unsigned) my > roq->height-size)
        return INT_MAX;

    return block_sse(enc->frame_to_enc->data, roq->last_frame->data, x, y,
                     mx, my,
                     enc->frame_to_enc->linesize, roq->last_frame->linesize,
                     size);
}

/**
 * @return distortion between two macroblocks
 */
static inline int squared_diff_macroblock(uint8_t a[], uint8_t b[], int size)
{
    int cp, sdiff=0;

    for(cp=0;cp<3;cp++) {
        int bias = (cp ? CHROMA_BIAS : 4);
        sdiff += bias*eval_sse(a, b, size*size);
        a += size*size;
        b += size*size;
    }

    return sdiff;
}

/**
 * Initialize cel evaluators and set their source coordinates
 */
static int create_cel_evals(RoqEncContext *enc)
{
    RoqContext *const roq = &enc->common;

    enc->cel_evals = av_malloc_array(roq->width * roq->height / 64, sizeof(CelEvaluation));
    if (!enc->cel_evals)
        return AVERROR(ENOMEM);

    /* Map to the ROQ quadtree order */
    for (int y = 0, n = 0; y < roq->height; y += 16)
        for (int x = 0; x < roq->width; x += 16)
            for(int i = 0; i < 4; i++) {
                enc->cel_evals[n  ].sourceX = x + (i&1)*8;
                enc->cel_evals[n++].sourceY = y + (i&2)*4;
            }

    return 0;
}

/**
 * Get macroblocks from parts of the image
 */
static void get_frame_mb(const AVFrame *frame, int x, int y, uint8_t mb[], int dim)
{
    int i, j, cp;

    for (cp=0; cp<3; cp++) {
        int stride = frame->linesize[cp];
        for (i=0; i<dim; i++)
            for (j=0; j<dim; j++)
                *mb++ = frame->data[cp][(y+i)*stride + x + j];
    }
}

/**
 * Find the codebook with the lowest distortion from an image
 */
static int index_mb(uint8_t cluster[], uint8_t cb[], int numCB,
                    int *outIndex, int dim)
{
    int i, lDiff = INT_MAX, pick=0;

    /* Diff against the others */
    for (i=0; i<numCB; i++) {
        int diff = squared_diff_macroblock(cluster, cb + i*dim*dim*3, dim);
        if (diff < lDiff) {
            lDiff = diff;
            pick = i;
        }
    }

    *outIndex = pick;
    return lDiff;
}

#define EVAL_MOTION(MOTION) \
    do { \
        diff = eval_motion_dist(enc, j, i, MOTION, blocksize); \
            \
        if (diff < lowestdiff) { \
            lowestdiff = diff; \
            bestpick = MOTION; \
        } \
    } while(0)

static void motion_search(RoqEncContext *enc, int blocksize)
{
    static const motion_vect offsets[8] = {
        {{ 0,-1}},
        {{ 0, 1}},
        {{-1, 0}},
        {{ 1, 0}},
        {{-1, 1}},
        {{ 1,-1}},
        {{-1,-1}},
        {{ 1, 1}},
    };

    RoqContext *const roq = &enc->common;
    int diff, lowestdiff, oldbest;
    int off[3];
    motion_vect bestpick = {{0,0}};
    int i, j, k, offset;

    motion_vect *last_motion;
    motion_vect *this_motion;
    motion_vect vect, vect2;
    const int max = (roq->width / blocksize) * roq->height / blocksize;

    if (blocksize == 4) {
        last_motion = enc->last_motion4;
        this_motion = enc->this_motion4;
    } else {
        last_motion = enc->last_motion8;
        this_motion = enc->this_motion8;
    }

    for (i = 0; i< roq->height; i += blocksize)
        for (j = 0; j < roq->width; j += blocksize) {
            lowestdiff = eval_motion_dist(enc, j, i, (motion_vect) {{0,0}},
                                          blocksize);
            bestpick.d[0] = 0;
            bestpick.d[1] = 0;

            if (blocksize == 4)
                EVAL_MOTION(enc->this_motion8[(i/8) * (roq->width/8) + j/8]);

            offset = (i/blocksize) * roq->width / blocksize + j / blocksize;
            if (offset < max && offset >= 0)
                EVAL_MOTION(last_motion[offset]);

            offset++;
            if (offset < max && offset >= 0)
                EVAL_MOTION(last_motion[offset]);

            offset = (i/blocksize + 1) * roq->width / blocksize + j / blocksize;
            if (offset < max && offset >= 0)
                EVAL_MOTION(last_motion[offset]);

            off[0]= (i/blocksize) * roq->width / blocksize + j/blocksize - 1;
            off[1]= off[0] - roq->width / blocksize + 1;
            off[2]= off[1] + 1;

            if (i) {

                for(k=0; k<2; k++)
                    vect.d[k]= mid_pred(this_motion[off[0]].d[k],
                                        this_motion[off[1]].d[k],
                                        this_motion[off[2]].d[k]);

                EVAL_MOTION(vect);
                for(k=0; k<3; k++)
                    EVAL_MOTION(this_motion[off[k]]);
            } else if(j)
                EVAL_MOTION(this_motion[off[0]]);

            vect = bestpick;

            oldbest = -1;
            while (oldbest != lowestdiff) {
                oldbest = lowestdiff;
                for (k=0; k<8; k++) {
                    vect2 = vect;
                    vect2.d[0] += offsets[k].d[0];
                    vect2.d[1] += offsets[k].d[1];
                    EVAL_MOTION(vect2);
                }
                vect = bestpick;
            }
            offset = (i/blocksize) * roq->width / blocksize + j/blocksize;
            this_motion[offset] = bestpick;
        }
}

/**
 * Get distortion for all options available to a subcel
 */
static void gather_data_for_subcel(SubcelEvaluation *subcel, int x,
                                   int y, RoqEncContext *enc)
{
    RoqContext *const roq = &enc->common;
    RoqTempData *const tempData = &enc->tmp_data;
    uint8_t mb4[4*4*3];
    uint8_t mb2[2*2*3];
    int cluster_index;
    int i, best_dist;

    static const int bitsUsed[4] = {2, 10, 10, 34};

    if (enc->framesSinceKeyframe >= 1) {
        subcel->motion = enc->this_motion4[y * roq->width / 16 + x / 4];

        subcel->eval_dist[RoQ_ID_FCC] =
            eval_motion_dist(enc, x, y,
                             enc->this_motion4[y * roq->width / 16 + x / 4], 4);
    } else
        subcel->eval_dist[RoQ_ID_FCC] = INT_MAX;

    if (enc->framesSinceKeyframe >= 2)
        subcel->eval_dist[RoQ_ID_MOT] = block_sse(enc->frame_to_enc->data,
                                                  roq->current_frame->data, x,
                                                  y, x, y,
                                                  enc->frame_to_enc->linesize,
                                                  roq->current_frame->linesize,
                                                  4);
    else
        subcel->eval_dist[RoQ_ID_MOT] = INT_MAX;

    cluster_index = y * roq->width / 16 + x / 4;

    get_frame_mb(enc->frame_to_enc, x, y, mb4, 4);

    subcel->eval_dist[RoQ_ID_SLD] = index_mb(mb4,
                                             tempData->codebooks.unpacked_cb4,
                                             tempData->codebooks.numCB4,
                                             &subcel->cbEntry, 4);

    subcel->eval_dist[RoQ_ID_CCC] = 0;

    for(i=0;i<4;i++) {
        subcel->subCels[i] = enc->closest_cb[cluster_index*4+i];

        get_frame_mb(enc->frame_to_enc, x+2*(i&1),
                     y+(i&2), mb2, 2);

        subcel->eval_dist[RoQ_ID_CCC] +=
            squared_diff_macroblock(tempData->codebooks.unpacked_cb2 + subcel->subCels[i]*2*2*3, mb2, 2);
    }

    best_dist = INT_MAX;
    for (i=0; i<4; i++)
        if (ROQ_LAMBDA_SCALE*subcel->eval_dist[i] + enc->lambda*bitsUsed[i] <
            best_dist) {
            subcel->best_coding = i;
            subcel->best_bit_use = bitsUsed[i];
            best_dist = ROQ_LAMBDA_SCALE*subcel->eval_dist[i] +
                enc->lambda*bitsUsed[i];
        }
}

/**
 * Get distortion for all options available to a cel
 */
static void gather_data_for_cel(CelEvaluation *cel, RoqEncContext *enc)
{
    RoqContext *const roq = &enc->common;
    RoqTempData *const tempData = &enc->tmp_data;
    uint8_t mb8[8*8*3];
    int index = cel->sourceY * roq->width / 64 + cel->sourceX/8;
    int i, j, best_dist, divide_bit_use;

    int bitsUsed[4] = {2, 10, 10, 0};

    if (enc->framesSinceKeyframe >= 1) {
        cel->motion = enc->this_motion8[index];

        cel->eval_dist[RoQ_ID_FCC] =
            eval_motion_dist(enc, cel->sourceX, cel->sourceY,
                             enc->this_motion8[index], 8);
    } else
        cel->eval_dist[RoQ_ID_FCC] = INT_MAX;

    if (enc->framesSinceKeyframe >= 2)
        cel->eval_dist[RoQ_ID_MOT] = block_sse(enc->frame_to_enc->data,
                                               roq->current_frame->data,
                                               cel->sourceX, cel->sourceY,
                                               cel->sourceX, cel->sourceY,
                                               enc->frame_to_enc->linesize,
                                               roq->current_frame->linesize,8);
    else
        cel->eval_dist[RoQ_ID_MOT] = INT_MAX;

    get_frame_mb(enc->frame_to_enc, cel->sourceX, cel->sourceY, mb8, 8);

    cel->eval_dist[RoQ_ID_SLD] =
        index_mb(mb8, tempData->codebooks.unpacked_cb4_enlarged,
                 tempData->codebooks.numCB4, &cel->cbEntry, 8);

    gather_data_for_subcel(cel->subCels + 0, cel->sourceX+0, cel->sourceY+0, enc);
    gather_data_for_subcel(cel->subCels + 1, cel->sourceX+4, cel->sourceY+0, enc);
    gather_data_for_subcel(cel->subCels + 2, cel->sourceX+0, cel->sourceY+4, enc);
    gather_data_for_subcel(cel->subCels + 3, cel->sourceX+4, cel->sourceY+4, enc);

    cel->eval_dist[RoQ_ID_CCC] = 0;
    divide_bit_use = 0;
    for (i=0; i<4; i++) {
        cel->eval_dist[RoQ_ID_CCC] +=
            cel->subCels[i].eval_dist[cel->subCels[i].best_coding];
        divide_bit_use += cel->subCels[i].best_bit_use;
    }

    best_dist = INT_MAX;
    bitsUsed[3] = 2 + divide_bit_use;

    for (i=0; i<4; i++)
        if (ROQ_LAMBDA_SCALE*cel->eval_dist[i] + enc->lambda*bitsUsed[i] <
            best_dist) {
            cel->best_coding = i;
            best_dist = ROQ_LAMBDA_SCALE*cel->eval_dist[i] +
                enc->lambda*bitsUsed[i];
        }

    tempData->used_option[cel->best_coding]++;
    tempData->mainChunkSize += bitsUsed[cel->best_coding];

    if (cel->best_coding == RoQ_ID_SLD)
        tempData->codebooks.usedCB4[cel->cbEntry]++;

    if (cel->best_coding == RoQ_ID_CCC)
        for (i=0; i<4; i++) {
            if (cel->subCels[i].best_coding == RoQ_ID_SLD)
                tempData->codebooks.usedCB4[cel->subCels[i].cbEntry]++;
            else if (cel->subCels[i].best_coding == RoQ_ID_CCC)
                for (j=0; j<4; j++)
                    tempData->codebooks.usedCB2[cel->subCels[i].subCels[j]]++;
        }
}

static void remap_codebooks(RoqEncContext *enc)
{
    RoqContext *const roq = &enc->common;
    RoqTempData *const tempData = &enc->tmp_data;
    int i, j, idx=0;

    /* Make remaps for the final codebook usage */
    for (i=0; i<(enc->quake3_compat ? MAX_CBS_4x4-1 : MAX_CBS_4x4); i++) {
        if (tempData->codebooks.usedCB4[i]) {
            tempData->i2f4[i] = idx;
            tempData->f2i4[idx] = i;
            for (j=0; j<4; j++)
                tempData->codebooks.usedCB2[roq->cb4x4[i].idx[j]]++;
            idx++;
        }
    }

    tempData->numCB4 = idx;

    idx = 0;
    for (i=0; i<MAX_CBS_2x2; i++) {
        if (tempData->codebooks.usedCB2[i]) {
            tempData->i2f2[i] = idx;
            tempData->f2i2[idx] = i;
            idx++;
        }
    }
    tempData->numCB2 = idx;

}

/**
 * Write codebook chunk
 */
static void write_codebooks(RoqEncContext *enc)
{
    RoqContext *const roq = &enc->common;
    RoqTempData *const tempData = &enc->tmp_data;
    int i, j;
    uint8_t **outp= &enc->out_buf;

    if (tempData->numCB2) {
        bytestream_put_le16(outp, RoQ_QUAD_CODEBOOK);
        bytestream_put_le32(outp, tempData->numCB2*6 + tempData->numCB4*4);
        bytestream_put_byte(outp, tempData->numCB4);
        bytestream_put_byte(outp, tempData->numCB2);

        for (i=0; i<tempData->numCB2; i++) {
            bytestream_put_buffer(outp, roq->cb2x2[tempData->f2i2[i]].y, 4);
            bytestream_put_byte(outp, roq->cb2x2[tempData->f2i2[i]].u);
            bytestream_put_byte(outp, roq->cb2x2[tempData->f2i2[i]].v);
        }

        for (i=0; i<tempData->numCB4; i++)
            for (j=0; j<4; j++)
                bytestream_put_byte(outp, tempData->i2f2[roq->cb4x4[tempData->f2i4[i]].idx[j]]);

    }
}

static inline uint8_t motion_arg(motion_vect mot)
{
    uint8_t ax = 8 - ((uint8_t) mot.d[0]);
    uint8_t ay = 8 - ((uint8_t) mot.d[1]);
    return ((ax&15)<<4) | (ay&15);
}

typedef struct CodingSpool {
    int typeSpool;
    int typeSpoolLength;
    uint8_t argumentSpool[64];
    uint8_t *args;
    uint8_t **pout;
} CodingSpool;

/* NOTE: Typecodes must be spooled AFTER arguments!! */
static void write_typecode(CodingSpool *s, uint8_t type)
{
    s->typeSpool |= (type & 3) << (14 - s->typeSpoolLength);
    s->typeSpoolLength += 2;
    if (s->typeSpoolLength == 16) {
        bytestream_put_le16(s->pout, s->typeSpool);
        bytestream_put_buffer(s->pout, s->argumentSpool,
                              s->args - s->argumentSpool);
        s->typeSpoolLength = 0;
        s->typeSpool = 0;
        s->args = s->argumentSpool;
    }
}

static void reconstruct_and_encode_image(RoqEncContext *enc,
                                         int w, int h, int numBlocks)
{
    RoqContext *const roq = &enc->common;
    RoqTempData *const tempData = &enc->tmp_data;
    int i, j, k;
    int x, y;
    int subX, subY;

    roq_qcell *qcell;
    CelEvaluation *eval;

    CodingSpool spool;

    spool.typeSpool=0;
    spool.typeSpoolLength=0;
    spool.args = spool.argumentSpool;
    spool.pout = &enc->out_buf;

    if (tempData->used_option[RoQ_ID_CCC]%2)
        tempData->mainChunkSize+=8; //FIXME

    /* Write the video chunk header */
    bytestream_put_le16(&enc->out_buf, RoQ_QUAD_VQ);
    bytestream_put_le32(&enc->out_buf, tempData->mainChunkSize/8);
    bytestream_put_byte(&enc->out_buf, 0x0);
    bytestream_put_byte(&enc->out_buf, 0x0);

    for (i=0; i<numBlocks; i++) {
        eval = enc->cel_evals + i;

        x = eval->sourceX;
        y = eval->sourceY;

        switch (eval->best_coding) {
        case RoQ_ID_MOT:
            write_typecode(&spool, RoQ_ID_MOT);
            break;

        case RoQ_ID_FCC:
            bytestream_put_byte(&spool.args, motion_arg(eval->motion));

            write_typecode(&spool, RoQ_ID_FCC);
            ff_apply_motion_8x8(roq, x, y,
                                eval->motion.d[0], eval->motion.d[1]);
            break;

        case RoQ_ID_SLD:
            bytestream_put_byte(&spool.args, tempData->i2f4[eval->cbEntry]);
            write_typecode(&spool, RoQ_ID_SLD);

            qcell = roq->cb4x4 + eval->cbEntry;
            ff_apply_vector_4x4(roq, x  , y  , roq->cb2x2 + qcell->idx[0]);
            ff_apply_vector_4x4(roq, x+4, y  , roq->cb2x2 + qcell->idx[1]);
            ff_apply_vector_4x4(roq, x  , y+4, roq->cb2x2 + qcell->idx[2]);
            ff_apply_vector_4x4(roq, x+4, y+4, roq->cb2x2 + qcell->idx[3]);
            break;

        case RoQ_ID_CCC:
            write_typecode(&spool, RoQ_ID_CCC);

            for (j=0; j<4; j++) {
                subX = x + 4*(j&1);
                subY = y + 2*(j&2);

                switch(eval->subCels[j].best_coding) {
                case RoQ_ID_MOT:
                    break;

                case RoQ_ID_FCC:
                    bytestream_put_byte(&spool.args,
                                        motion_arg(eval->subCels[j].motion));

                    ff_apply_motion_4x4(roq, subX, subY,
                                        eval->subCels[j].motion.d[0],
                                        eval->subCels[j].motion.d[1]);
                    break;

                case RoQ_ID_SLD:
                    bytestream_put_byte(&spool.args,
                                        tempData->i2f4[eval->subCels[j].cbEntry]);

                    qcell = roq->cb4x4 + eval->subCels[j].cbEntry;

                    ff_apply_vector_2x2(roq, subX  , subY  ,
                                        roq->cb2x2 + qcell->idx[0]);
                    ff_apply_vector_2x2(roq, subX+2, subY  ,
                                        roq->cb2x2 + qcell->idx[1]);
                    ff_apply_vector_2x2(roq, subX  , subY+2,
                                        roq->cb2x2 + qcell->idx[2]);
                    ff_apply_vector_2x2(roq, subX+2, subY+2,
                                        roq->cb2x2 + qcell->idx[3]);
                    break;

                case RoQ_ID_CCC:
                    for (k=0; k<4; k++) {
                        int cb_idx = eval->subCels[j].subCels[k];
                        bytestream_put_byte(&spool.args,
                                            tempData->i2f2[cb_idx]);

                        ff_apply_vector_2x2(roq, subX + 2*(k&1), subY + (k&2),
                                            roq->cb2x2 + cb_idx);
                    }
                    break;
                }
                write_typecode(&spool, eval->subCels[j].best_coding);
            }
            break;
        }
    }

    /* Flush the remainder of the argument/type spool */
    while (spool.typeSpoolLength)
        write_typecode(&spool, 0x0);
}


/**
 * Create a single YUV cell from a 2x2 section of the image
 */
static inline void frame_block_to_cell(int *block, uint8_t * const *data,
                                       int top, int left, const int *stride)
{
    int i, j, u=0, v=0;

    for (i=0; i<2; i++)
        for (j=0; j<2; j++) {
            int x = (top+i)*stride[0] + left + j;
            *block++ = data[0][x];
            x = (top+i)*stride[1] + left + j;
            u       += data[1][x];
            v       += data[2][x];
        }

    *block++ = (u + 2) / 4 * CHROMA_BIAS;
    *block++ = (v + 2) / 4 * CHROMA_BIAS;
}

/**
 * Create YUV clusters for the entire image
 */
static void create_clusters(const AVFrame *frame, int w, int h, int *points)
{
    int i, j, k, l;

    for (i=0; i<h; i+=4)
        for (j=0; j<w; j+=4) {
            for (k=0; k < 2; k++)
                for (l=0; l < 2; l++)
                    frame_block_to_cell(points + (l + 2*k)*6, frame->data,
                                        i+2*k, j+2*l, frame->linesize);
            points += 24;
        }
}

static int generate_codebook(RoqEncContext *enc,
                             int *points, int inputCount, roq_cell *results,
                             int size, int cbsize)
{
    int i, j, k, ret = 0;
    int c_size = size*size/4;
    int *buf;
    int *codebook = enc->tmp_codebook_buf;
    int *closest_cb = enc->closest_cb;

    ret = avpriv_elbg_do(&enc->elbg, points, 6 * c_size, inputCount, codebook,
                         cbsize, 1, closest_cb, &enc->randctx, 0);
    if (ret < 0)
        return ret;

    buf = codebook;
    for (i=0; i<cbsize; i++)
        for (k=0; k<c_size; k++) {
            for(j=0; j<4; j++)
                results->y[j] = *buf++;

            results->u =    (*buf++ + CHROMA_BIAS/2)/CHROMA_BIAS;
            results->v =    (*buf++ + CHROMA_BIAS/2)/CHROMA_BIAS;
            results++;
        }
    return 0;
}

static int generate_new_codebooks(RoqEncContext *enc)
{
    int i, j, ret = 0;
    RoqCodebooks *codebooks = &enc->tmp_data.codebooks;
    RoqContext *const roq = &enc->common;
    int max = roq->width * roq->height / 16;
    uint8_t mb2[3*4];
    int *points = enc->points;

    /* Subsample YUV data */
    create_clusters(enc->frame_to_enc, roq->width, roq->height, points);

    codebooks->numCB4 = (enc->quake3_compat ? MAX_CBS_4x4-1 : MAX_CBS_4x4);

    /* Create 4x4 codebooks */
    if ((ret = generate_codebook(enc, points, max, enc->results4,
                                 4, codebooks->numCB4)) < 0)
        return ret;

    /* Create 2x2 codebooks */
    if ((ret = generate_codebook(enc, points, max * 4,
                                 roq->cb2x2, 2, MAX_CBS_2x2)) < 0)
        return ret;

    codebooks->numCB2 = MAX_CBS_2x2;

    /* Unpack 2x2 codebook clusters */
    for (i=0; i<codebooks->numCB2; i++)
        unpack_roq_cell(roq->cb2x2 + i, codebooks->unpacked_cb2 + i*2*2*3);

    /* Index all 4x4 entries to the 2x2 entries, unpack, and enlarge */
    for (i=0; i<codebooks->numCB4; i++) {
        for (j=0; j<4; j++) {
            unpack_roq_cell(&enc->results4[4*i + j], mb2);
            index_mb(mb2, codebooks->unpacked_cb2, codebooks->numCB2,
                     &roq->cb4x4[i].idx[j], 2);
        }
        unpack_roq_qcell(codebooks->unpacked_cb2, roq->cb4x4 + i,
                         codebooks->unpacked_cb4 + i*4*4*3);
        enlarge_roq_mb4(codebooks->unpacked_cb4 + i*4*4*3,
                        codebooks->unpacked_cb4_enlarged + i*8*8*3);
    }

    return 0;
}

static int roq_encode_video(RoqEncContext *enc)
{
    RoqTempData *const tempData = &enc->tmp_data;
    RoqContext *const roq = &enc->common;
    int ret;

    memset(tempData, 0, sizeof(*tempData));

    ret = generate_new_codebooks(enc);
    if (ret < 0)
        return ret;

    if (enc->framesSinceKeyframe >= 1) {
        motion_search(enc, 8);
        motion_search(enc, 4);
    }

 retry_encode:
    for (int i = 0; i < roq->width * roq->height / 64; i++)
        gather_data_for_cel(enc->cel_evals + i, enc);

    /* Quake 3 can't handle chunks bigger than 65535 bytes */
    if (tempData->mainChunkSize/8 > 65535 && enc->quake3_compat) {
        if (enc->lambda > 100000) {
            av_log(roq->logctx, AV_LOG_ERROR, "Cannot encode video in Quake compatible form\n");
            return AVERROR(EINVAL);
        }
        av_log(roq->logctx, AV_LOG_ERROR,
               "Warning, generated a frame too big for Quake (%d > 65535), "
               "now switching to a bigger qscale value.\n",
               tempData->mainChunkSize/8);
        enc->lambda *= 1.5;
        tempData->mainChunkSize = 0;
        memset(tempData->used_option, 0, sizeof(tempData->used_option));
        memset(tempData->codebooks.usedCB4, 0,
               sizeof(tempData->codebooks.usedCB4));
        memset(tempData->codebooks.usedCB2, 0,
               sizeof(tempData->codebooks.usedCB2));

        goto retry_encode;
    }

    remap_codebooks(enc);

    write_codebooks(enc);

    reconstruct_and_encode_image(enc, roq->width, roq->height,
                                 roq->width * roq->height / 64);

    /* Rotate frame history */
    FFSWAP(AVFrame *,    roq->current_frame,   roq->last_frame);
    FFSWAP(motion_vect *, enc->last_motion4, enc->this_motion4);
    FFSWAP(motion_vect *, enc->last_motion8, enc->this_motion8);

    enc->framesSinceKeyframe++;

    return 0;
}

static av_cold int roq_encode_end(AVCodecContext *avctx)
{
    RoqEncContext *const enc = avctx->priv_data;

    av_frame_free(&enc->common.current_frame);
    av_frame_free(&enc->common.last_frame);

    av_freep(&enc->cel_evals);
    av_freep(&enc->closest_cb);
    av_freep(&enc->this_motion4);
    av_freep(&enc->last_motion4);
    av_freep(&enc->this_motion8);
    av_freep(&enc->last_motion8);

    avpriv_elbg_free(&enc->elbg);

    return 0;
}

static av_cold int roq_encode_init(AVCodecContext *avctx)
{
    RoqEncContext *const enc = avctx->priv_data;
    RoqContext    *const roq = &enc->common;

    av_lfg_init(&enc->randctx, 1);

    roq->logctx = avctx;

    enc->framesSinceKeyframe = 0;
    if ((avctx->width & 0xf) || (avctx->height & 0xf)) {
        av_log(avctx, AV_LOG_ERROR, "Dimensions must be divisible by 16\n");
        return AVERROR(EINVAL);
    }

    if (avctx->width > 65535 || avctx->height > 65535) {
        av_log(avctx, AV_LOG_ERROR, "Dimensions are max %d\n", enc->quake3_compat ? 32768 : 65535);
        return AVERROR(EINVAL);
    }

    if (((avctx->width)&(avctx->width-1))||((avctx->height)&(avctx->height-1)))
        av_log(avctx, AV_LOG_ERROR, "Warning: dimensions not power of two, this is not supported by quake\n");

    roq->width  = avctx->width;
    roq->height = avctx->height;

    enc->framesSinceKeyframe = 0;
    enc->first_frame = 1;

    roq->last_frame    = av_frame_alloc();
    roq->current_frame = av_frame_alloc();
    if (!roq->last_frame || !roq->current_frame)
        return AVERROR(ENOMEM);

    enc->this_motion4 =
        av_calloc(roq->width * roq->height / 16, sizeof(*enc->this_motion4));

    enc->last_motion4 =
        av_malloc_array (roq->width * roq->height / 16, sizeof(motion_vect));

    enc->this_motion8 =
        av_calloc(roq->width * roq->height / 64, sizeof(*enc->this_motion8));

    enc->last_motion8 =
        av_malloc_array (roq->width * roq->height / 64, sizeof(motion_vect));

    /* 4x4 codebook needs 6 * 4 * 4 / 4 * width * height / 16 * sizeof(int);
     * and so does the points buffer. */
    enc->closest_cb   =
        av_malloc_array(roq->width * roq->height, 3 * sizeof(int));

    if (!enc->this_motion4 || !enc->last_motion4 ||
        !enc->this_motion8 || !enc->last_motion8 || !enc->closest_cb)
        return AVERROR(ENOMEM);

    enc->points = enc->closest_cb + roq->width * roq->height * 3 / 2;

    return create_cel_evals(enc);
}

static void roq_write_video_info_chunk(RoqEncContext *enc)
{
    /* ROQ info chunk */
    bytestream_put_le16(&enc->out_buf, RoQ_INFO);

    /* Size: 8 bytes */
    bytestream_put_le32(&enc->out_buf, 8);

    /* Unused argument */
    bytestream_put_byte(&enc->out_buf, 0x00);
    bytestream_put_byte(&enc->out_buf, 0x00);

    /* Width */
    bytestream_put_le16(&enc->out_buf, enc->common.width);

    /* Height */
    bytestream_put_le16(&enc->out_buf, enc->common.height);

    /* Unused in Quake 3, mimics the output of the real encoder */
    bytestream_put_byte(&enc->out_buf, 0x08);
    bytestream_put_byte(&enc->out_buf, 0x00);
    bytestream_put_byte(&enc->out_buf, 0x04);
    bytestream_put_byte(&enc->out_buf, 0x00);
}

static int roq_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                            const AVFrame *frame, int *got_packet)
{
    RoqEncContext *const enc = avctx->priv_data;
    RoqContext    *const roq = &enc->common;
    int size, ret;

    enc->frame_to_enc = frame;

    if (frame->quality)
        enc->lambda = frame->quality - 1;
    else
        enc->lambda = 2*ROQ_LAMBDA_SCALE;

    /* 138 bits max per 8x8 block +
     *     256 codebooks*(6 bytes 2x2 + 4 bytes 4x4) + 8 bytes frame header */
    size = ((roq->width * roq->height / 64) * 138 + 7) / 8 + 256 * (6 + 4) + 8;
    if ((ret = ff_alloc_packet(avctx, pkt, size)) < 0)
        return ret;
    enc->out_buf = pkt->data;

    /* Check for I-frame */
    if (enc->framesSinceKeyframe == avctx->gop_size)
        enc->framesSinceKeyframe = 0;

    if (enc->first_frame) {
        /* Alloc memory for the reconstruction data (we must know the stride
         for that) */
        if ((ret = ff_encode_alloc_frame(avctx, roq->current_frame)) < 0 ||
            (ret = ff_encode_alloc_frame(avctx, roq->last_frame   )) < 0)
            return ret;

        /* Before the first video frame, write a "video info" chunk */
        roq_write_video_info_chunk(enc);

        enc->first_frame = 0;
    }

    /* Encode the actual frame */
    ret = roq_encode_video(enc);
    if (ret < 0)
        return ret;

    pkt->size   = enc->out_buf - pkt->data;
    if (enc->framesSinceKeyframe == 1)
        pkt->flags |= AV_PKT_FLAG_KEY;
    *got_packet = 1;

    return 0;
}

#define OFFSET(x) offsetof(RoqEncContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "quake3_compat", "Whether to respect known limitations in Quake 3 decoder", OFFSET(quake3_compat), AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, VE },
    { NULL },
};

static const AVClass roq_class = {
    .class_name = "RoQ",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_roq_encoder = {
    .p.name               = "roqvideo",
    CODEC_LONG_NAME("id RoQ video"),
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_ROQ,
    .p.capabilities       = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size       = sizeof(RoqEncContext),
    .init                 = roq_encode_init,
    FF_CODEC_ENCODE_CB(roq_encode_frame),
    .close                = roq_encode_end,
    CODEC_PIXFMTS(AV_PIX_FMT_YUVJ444P),
    .color_ranges         = AVCOL_RANGE_JPEG,
    .p.priv_class   = &roq_class,
    .caps_internal        = FF_CODEC_CAP_INIT_CLEANUP,
};
