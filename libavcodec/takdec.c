/*
 * TAK decoder
 * Copyright (c) 2012 Paul B Mahol
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
 * TAK (Tom's lossless Audio Kompressor) decoder
 * @author Paul B Mahol
 */

#include "libavutil/attributes.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/samplefmt.h"

#define CACHED_BITSTREAM_READER !ARCH_X86_32
#define BITSTREAM_READER_LE
#include "audiodsp.h"
#include "thread.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "unary.h"
#include "tak.h"
#include "takdsp.h"

#define MAX_SUBFRAMES     8                         ///< max number of subframes per channel
#define MAX_PREDICTORS  256

typedef struct MCDParam {
    int8_t present;                                 ///< decorrelation parameter availability for this channel
    int8_t index;                                   ///< index into array of decorrelation types
    int8_t chan1;
    int8_t chan2;
} MCDParam;

typedef struct ACoeffs {
    int fnum;
    int sample_shift;
    int fshift_num;

    int coeffs[256];
} ACoeffs;

typedef struct FramePositions {
    int frame_size;                                 ///< maximum frame size in samples
    int resolution;
    int block_size_min;
    int nb_blocks;
    int positions[129];
} FramePositions;

typedef struct SubFramePositions {
    const FramePositions *frame;
    int frame_size;                                 ///< current (sub)frame size in samples
    int res_shift_num;
    int nb_blocks;
} SubFramePositions;

enum TAKJointModes {
    TAK_JOINT_OFF,
    TAK_JOINT_LEFT,
    TAK_JOINT_RIGHT,
    TAK_JOINT_LEFT_SIDE,
    TAK_JOINT_RIGHT_SIDE,
    TAK_JOINT_MID_SIDE,
};

typedef struct TAKDecContext {
    AVCodecContext *avctx;                          ///< parent AVCodecContext
    AudioDSPContext adsp;
    TAKDSPContext   tdsp;
    TAKStreamInfo   ti;
    GetBitContext   gb;                             ///< bitstream reader initialized to start at the current frame

    FramePositions    fpos;                       ///< frame partition positions
    SubFramePositions spos;
    FramePositions    bcpos;                      ///< bit coder partition positions
    SubFramePositions bc_spos;
    int               bc_resolution_max;
    int               pos_frame_size;             ///< frame size the position tables were built for
    int               pos_sample_rate;            ///< sample rate the position tables were built for
    int32_t           lpc_work[512];

    int             uval;
    int             nb_samples;                     ///< number of samples in the current frame
    uint8_t        *decode_buffer;
    unsigned int    decode_buffer_size;
    int32_t        *decoded[TAK_MAX_CHANNELS];      ///< decoded samples for each channel

    int8_t          lpc_mode[TAK_MAX_CHANNELS];
    int8_t          sample_shift[TAK_MAX_CHANNELS]; ///< shift applied to every sample in the channel
    int16_t         predictors[MAX_PREDICTORS];
    int             nb_subframes;                   ///< number of subframes in the current frame
    int16_t         subframe_len[MAX_SUBFRAMES];    ///< subframe length in samples
    int             subframe_scale;

    int             wasted_bits[2];
    int             prefilter[2];
    ACoeffs         pcoeffs[2];                     ///< prefilter coefficients
    ACoeffs         jcoeffs;                        ///< joint stereo predictor coefficients
    int             jsub_start;
    int             jsub_end;
    int             jshift;

    int8_t          dmode;                          ///< channel decorrelation type in the current frame

    MCDParam        mcdparams[TAK_MAX_CHANNELS];    ///< multichannel decorrelation parameters

    int8_t          coding_mode[128];
    DECLARE_ALIGNED(16, int16_t, filter)[MAX_PREDICTORS];
    DECLARE_ALIGNED(16, int16_t, residues)[544];
} TAKDecContext;

static const int8_t mc_dmodes[] = { 1, 3, 4, 6, };

static const uint16_t predictor_sizes[] = {
    4, 8, 12, 16, 24, 32, 48, 64, 80, 96, 128, 160, 192, 224, 256, 0,
};

static const uint16_t pred_types[] = {
    4,  8,  12,  16,  24,  32,  48,  64,
    80, 96, 112, 128, 160, 192, 224, 256
};

static const struct CParam0 {
    int c1_bit_num;
    int c2b_min_crit;
    int c2a_min_val;
    int c2b_min_val;
    int c2b_size;
    int c3_min_val;
} ycodes[47] = {
    { 3, 6, 4, 8, 4, 44 },
    { 4, 14, 10, 18, 4, 54 },
    { 4, 12, 4, 20, 8, 92 },
    { 5, 28, 22, 34, 8, 106 },
    { 5, 26, 12, 40, 12, 148 },
    { 6, 56, 44, 68, 16, 212 },
    { 6, 52, 24, 80, 24, 296 },
    { 7, 112, 88, 136, 32, 424 },
    { 7, 104, 48, 160, 48, 592 },
    { 8, 224, 176, 272, 64, 848 },
    { 8, 208, 96, 320, 96, 1184 },
    { 9, 448, 352, 544, 128, 1696 },
    { 9, 416, 192, 640, 192, 2368 },
    { 10, 896, 704, 1088, 256, 3392 },
    { 10, 832, 384, 1280, 384, 4736 },
    { 11, 1792, 1408, 2176, 512, 6784 },
    { 11, 1664, 768, 2560, 768, 9472 },
    { 12, 3584, 2816, 4352, 1024, 13568 },
    { 12, 3328, 1536, 5120, 1536, 18944 },
    { 13, 7168, 5632, 8704, 2048, 27136 },
    { 13, 6656, 3072, 10240, 3072, 37888 },
    { 14, 14336, 11264, 17408, 4096, 54272 },
    { 14, 13312, 6144, 20480, 6144, 75776 },
    { 15, 28672, 22528, 34816, 8192, 108544 },
    { 15, 26624, 12288, 40960, 12288, 151552 },
    { 16, 57344, 45056, 69632, 16384, 217088 },
    { 16, 53248, 24576, 81920, 24576, 303104 },
    { 17, 114688, 90112, 139264, 32768, 434176 },
    { 17, 106496, 49152, 163840, 49152, 606208 },
    { 18, 229376, 180224, 278528, 65536, 868352 },
    { 18, 212992, 98304, 327680, 98304, 1212416 },
    { 19, 458752, 360448, 557056, 131072, 1736704 },
    { 19, 425984, 196608, 655360, 196608, 2424832 },
    { 20, 917504, 720896, 1114112, 262144, 3473408 },
    { 20, 851968, 393216, 1310720, 393216, 4849664 },
    { 21, 1835008, 1441792, 2228224, 524288, 6946816 },
    { 21, 1703936, 786432, 2621440, 786432, 9699328 },
    { 22, 3670016, 2883584, 4456448, 1048576, 13893632 },
    { 22, 3407872, 1572864, 5242880, 1572864, 19398656 },
    { 23, 7340032, 5767168, 8912896, 2097152, 27787264 },
    { 23, 6815744, 3145728, 10485760, 3145728, 38797312 },
    { 24, 14680064, 11534336, 17825792, 4194304, 55574528 },
    { 24, 13631488, 6291456, 20971520, 6291456, 77594624 },
    { 25, 29360128, 23068672, 35651584, 8388608, 111149056 },
    { 25, 27262976, 12582912, 41943040, 12582912, 155189248 },
    { 26, 58720256, 46137344, 71303168, 16777216, 222298112 },
    { 26, 54525952, 25165824, 83886080, 25165824, 310378496 },
};

static const struct CParam1 {
    int8_t c1_bit_num;
    int8_t c2_min_val;
    int8_t c2_max_val;
} zcodes[3] = {
    { 1, 1, 4 },
    { 2, 2, 5 },
    { 3, 4, 7 },
};

static const struct CParam {
    int init;
    int escape;
    int scale;
    int aescape;
    int bias;
} xcodes[50] = {
    { 0x01, 0x0000001, 0x0000001, 0x0000003, 0x0000008 },
    { 0x02, 0x0000003, 0x0000001, 0x0000007, 0x0000006 },
    { 0x03, 0x0000005, 0x0000002, 0x000000E, 0x000000D },
    { 0x03, 0x0000003, 0x0000003, 0x000000D, 0x0000018 },
    { 0x04, 0x000000B, 0x0000004, 0x000001C, 0x0000019 },
    { 0x04, 0x0000006, 0x0000006, 0x000001A, 0x0000030 },
    { 0x05, 0x0000016, 0x0000008, 0x0000038, 0x0000032 },
    { 0x05, 0x000000C, 0x000000C, 0x0000034, 0x0000060 },
    { 0x06, 0x000002C, 0x0000010, 0x0000070, 0x0000064 },
    { 0x06, 0x0000018, 0x0000018, 0x0000068, 0x00000C0 },
    { 0x07, 0x0000058, 0x0000020, 0x00000E0, 0x00000C8 },
    { 0x07, 0x0000030, 0x0000030, 0x00000D0, 0x0000180 },
    { 0x08, 0x00000B0, 0x0000040, 0x00001C0, 0x0000190 },
    { 0x08, 0x0000060, 0x0000060, 0x00001A0, 0x0000300 },
    { 0x09, 0x0000160, 0x0000080, 0x0000380, 0x0000320 },
    { 0x09, 0x00000C0, 0x00000C0, 0x0000340, 0x0000600 },
    { 0x0A, 0x00002C0, 0x0000100, 0x0000700, 0x0000640 },
    { 0x0A, 0x0000180, 0x0000180, 0x0000680, 0x0000C00 },
    { 0x0B, 0x0000580, 0x0000200, 0x0000E00, 0x0000C80 },
    { 0x0B, 0x0000300, 0x0000300, 0x0000D00, 0x0001800 },
    { 0x0C, 0x0000B00, 0x0000400, 0x0001C00, 0x0001900 },
    { 0x0C, 0x0000600, 0x0000600, 0x0001A00, 0x0003000 },
    { 0x0D, 0x0001600, 0x0000800, 0x0003800, 0x0003200 },
    { 0x0D, 0x0000C00, 0x0000C00, 0x0003400, 0x0006000 },
    { 0x0E, 0x0002C00, 0x0001000, 0x0007000, 0x0006400 },
    { 0x0E, 0x0001800, 0x0001800, 0x0006800, 0x000C000 },
    { 0x0F, 0x0005800, 0x0002000, 0x000E000, 0x000C800 },
    { 0x0F, 0x0003000, 0x0003000, 0x000D000, 0x0018000 },
    { 0x10, 0x000B000, 0x0004000, 0x001C000, 0x0019000 },
    { 0x10, 0x0006000, 0x0006000, 0x001A000, 0x0030000 },
    { 0x11, 0x0016000, 0x0008000, 0x0038000, 0x0032000 },
    { 0x11, 0x000C000, 0x000C000, 0x0034000, 0x0060000 },
    { 0x12, 0x002C000, 0x0010000, 0x0070000, 0x0064000 },
    { 0x12, 0x0018000, 0x0018000, 0x0068000, 0x00C0000 },
    { 0x13, 0x0058000, 0x0020000, 0x00E0000, 0x00C8000 },
    { 0x13, 0x0030000, 0x0030000, 0x00D0000, 0x0180000 },
    { 0x14, 0x00B0000, 0x0040000, 0x01C0000, 0x0190000 },
    { 0x14, 0x0060000, 0x0060000, 0x01A0000, 0x0300000 },
    { 0x15, 0x0160000, 0x0080000, 0x0380000, 0x0320000 },
    { 0x15, 0x00C0000, 0x00C0000, 0x0340000, 0x0600000 },
    { 0x16, 0x02C0000, 0x0100000, 0x0700000, 0x0640000 },
    { 0x16, 0x0180000, 0x0180000, 0x0680000, 0x0C00000 },
    { 0x17, 0x0580000, 0x0200000, 0x0E00000, 0x0C80000 },
    { 0x17, 0x0300000, 0x0300000, 0x0D00000, 0x1800000 },
    { 0x18, 0x0B00000, 0x0400000, 0x1C00000, 0x1900000 },
    { 0x18, 0x0600000, 0x0600000, 0x1A00000, 0x3000000 },
    { 0x19, 0x1600000, 0x0800000, 0x3800000, 0x3200000 },
    { 0x19, 0x0C00000, 0x0C00000, 0x3400000, 0x6000000 },
    { 0x1A, 0x2C00000, 0x1000000, 0x7000000, 0x6400000 },
    { 0x1A, 0x1800000, 0x1800000, 0x6800000, 0xC000000 },
};

static int set_bps_params(AVCodecContext *avctx)
{
    switch (avctx->bits_per_raw_sample) {
    case 8:
        avctx->sample_fmt = AV_SAMPLE_FMT_U8P;
        break;
    case 16:
        avctx->sample_fmt = AV_SAMPLE_FMT_S16P;
        break;
    case 24:
        avctx->sample_fmt = AV_SAMPLE_FMT_S32P;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "invalid/unsupported bits per sample: %d\n",
               avctx->bits_per_raw_sample);
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int calc_max_resolution(int sample_rate, int block_size_min)
{
    int res = 3;

    while (res > 0 && ((sample_rate / (64 << res)) < block_size_min))
        res--;

    return res;
}

static void frame_positions_create(FramePositions *p, int sample_rate,
                                   int frame_size, int resolution,
                                   int block_num_max, int block_align,
                                   int block_size_min)
{
    uint32_t block_align_mask = ~(block_align - 1);
    int block_round_add = block_align / 2;
    int blocks_per_second, block_pos;

    memset(p, 0, sizeof(*p));

    block_size_min    = FFMAX3(block_size_min, block_align, 4);
    p->frame_size     = frame_size;
    p->block_size_min = block_size_min;
    p->resolution     = FFMIN(resolution, calc_max_resolution(sample_rate, block_size_min));
    blocks_per_second = 64 << p->resolution;

    p->positions[0] = 0;
    block_pos       = 0;
    do {
        block_pos++;
        p->positions[block_pos] = ((sample_rate * block_pos) / blocks_per_second +
                                   block_round_add) & block_align_mask;
    } while (p->positions[block_pos] < frame_size && block_pos < block_num_max);

    if (p->positions[block_pos] < frame_size) {
        p->positions[block_pos] = frame_size;
    } else if (p->positions[block_pos] > frame_size) {
        p->positions[block_pos] = frame_size;
        if (p->positions[block_pos] - p->positions[block_pos - 1] < block_size_min) {
            block_pos--;
            p->positions[block_pos] = frame_size;
        }
    }

    p->nb_blocks = block_pos;
}

static int sample_to_block_pos(const FramePositions *p, int sample_pos,
                               int sample_rate)
{
    int ret = av_rescale_rnd(sample_pos, 64 << p->resolution, sample_rate, AV_ROUND_NEAR_INF);

    if (ret > p->nb_blocks) {
        ret = p->nb_blocks;
    } else {
        while (ret < p->nb_blocks && p->positions[ret + 1] <= sample_pos)
            ret++;
    }

    while (p->positions[ret] > sample_pos && ret > 0)
        ret--;

    return ret;
}

static void sub_frame_positions_setup(SubFramePositions *s,
                                      const FramePositions *const p,
                                      int frame_size, int resolution,
                                      int sample_rate)
{
    memset(s, 0, sizeof(*s));

    s->frame         = p;
    s->frame_size    = frame_size;
    s->res_shift_num = p->resolution - resolution;

    s->nb_blocks = sample_to_block_pos(p, frame_size, sample_rate) >> s->res_shift_num;
    if ((s->nb_blocks == 0) ||
        (frame_size - p->positions[s->nb_blocks << s->res_shift_num] >= p->block_size_min))
        s->nb_blocks++;
}

static int block_pos_to_sample_pos(const SubFramePositions *s, int block_pos)
{
    if (block_pos < s->nb_blocks)
        return s->frame->positions[block_pos << s->res_shift_num];
    else
        return s->frame_size;
}

static int get_block_size(const SubFramePositions *s, int block_pos)
{
    if (block_pos < s->nb_blocks - 1)
        return s->frame->positions[(block_pos + 1) << s->res_shift_num] -
               s->frame->positions[ block_pos      << s->res_shift_num];
    else
        return s->frame_size - s->frame->positions[block_pos << s->res_shift_num];
}

static int bco_opt_partition_res(int sample_rate, int sample_bits)
{
    if (sample_bits == 8) {
        if (sample_rate >= 48000)
            return 3;
        if (sample_rate >= 32000)
            return 2;
        if (sample_rate >= 16000)
            return 1;
        return 0;
    } else if (sample_bits >= 16 && sample_bits <= 24) {
        if (sample_rate >= 44100)
            return 3;
        if (sample_rate >= 22050)
            return 2;
        if (sample_rate >= 11025)
            return 1;
        return 0;
    }

    return 0;
}

static void set_sample_rate_params(AVCodecContext *avctx)
{
    TAKDecContext *s  = avctx->priv_data;
    int shift;

    if (avctx->sample_rate < 11025) {
        shift = 3;
    } else if (avctx->sample_rate < 22050) {
        shift = 2;
    } else if (avctx->sample_rate < 44100) {
        shift = 1;
    } else {
        shift = 0;
    }
    s->uval           = FFALIGN(avctx->sample_rate + 511LL >> 9, 4) << shift;
    s->subframe_scale = FFALIGN(avctx->sample_rate + 511LL >> 9, 4) << 1;
}

static av_cold int tak_decode_init(AVCodecContext *avctx)
{
    TAKDecContext *s = avctx->priv_data;

    ff_audiodsp_init(&s->adsp);
    ff_takdsp_init(&s->tdsp);

    s->avctx = avctx;
    avctx->bits_per_raw_sample = avctx->bits_per_coded_sample;

    set_sample_rate_params(avctx);

    return set_bps_params(avctx);
}

static void decode_lpc(int32_t *coeffs, int mode, int length)
{
    int i;

    if (length < 2)
        return;

    if (mode == 1) {
        unsigned a1 = *coeffs++;
        for (i = 0; i < length - 1 >> 1; i++) {
            *coeffs   += a1;
            coeffs[1] += (unsigned)*coeffs;
            a1         = coeffs[1];
            coeffs    += 2;
        }
        if (length - 1 & 1)
            *coeffs += a1;
    } else if (mode == 2) {
        unsigned a1    = coeffs[1];
        unsigned a2    = a1 + *coeffs;
        coeffs[1] = a2;
        if (length > 2) {
            coeffs += 2;
            for (i = 0; i < length - 2 >> 1; i++) {
                unsigned a3    = *coeffs + a1;
                unsigned a4    = a3 + a2;
                *coeffs   = a4;
                a1        = coeffs[1] + a3;
                a2        = a1 + a4;
                coeffs[1] = a2;
                coeffs   += 2;
            }
            if (length & 1)
                *coeffs += a1 + a2;
        }
    } else if (mode == 3) {
        unsigned a1    = coeffs[1];
        unsigned a2    = a1 + *coeffs;
        coeffs[1] = a2;
        if (length > 2) {
            unsigned a3  = coeffs[2];
            unsigned a4  = a3 + a1;
            unsigned a5  = a4 + a2;
            coeffs[2] = a5;
            coeffs += 3;
            for (i = 0; i < length - 3; i++) {
                a3     += *coeffs;
                a4     += a3;
                a5     += a4;
                *coeffs = a5;
                coeffs++;
            }
        }
    }
}

static int decode_segment(TAKDecContext *s, int8_t mode, int32_t *decoded, int len)
{
    struct CParam code;
    GetBitContext *gb = &s->gb;
    int i;

    if (!mode) {
        memset(decoded, 0, len * sizeof(*decoded));
        return 0;
    }

    if (mode > FF_ARRAY_ELEMS(xcodes))
        return AVERROR_INVALIDDATA;
    code = xcodes[mode - 1];

    for (i = 0; i < len; i++) {
        unsigned x = get_bits_long(gb, code.init);
        if (x >= code.escape && get_bits1(gb)) {
            x |= 1 << code.init;
            if (x >= code.aescape) {
                unsigned scale = get_unary(gb, 1, 9);
                if (scale == 9) {
                    int scale_bits = get_bits(gb, 3);
                    if (scale_bits > 0) {
                        if (scale_bits == 7) {
                            scale_bits += get_bits(gb, 5);
                            if (scale_bits > 29)
                                return AVERROR_INVALIDDATA;
                        }
                        scale = get_bits_long(gb, scale_bits) + 1;
                        x    += code.scale * scale;
                    }
                    x += code.bias;
                } else
                    x += code.scale * scale - code.escape;
            } else
                x -= code.escape;
        }
        decoded[i] = (x >> 1) ^ -(x & 1);
    }

    return 0;
}

static int decode_residues(TAKDecContext *s, int32_t *decoded, int length)
{
    GetBitContext *gb = &s->gb;
    int i, mode, ret;

    if (length > s->nb_samples)
        return AVERROR_INVALIDDATA;

    if (get_bits1(gb)) {
        int wlength, rval;

        wlength = length / s->uval;

        rval = length - (wlength * s->uval);

        if (rval < s->uval / 2)
            rval += s->uval;
        else
            wlength++;

        if (wlength <= 1 || wlength > 128)
            return AVERROR_INVALIDDATA;

        s->coding_mode[0] = mode = get_bits(gb, 6);

        for (i = 1; i < wlength; i++) {
            int c = get_unary(gb, 1, 6);

            switch (c) {
            case 6:
                mode = get_bits(gb, 6);
                break;
            case 5:
            case 4:
            case 3: {
                /* mode += sign ? (1 - c) : (c - 1) */
                int sign = get_bits1(gb);
                mode    += (-sign ^ (c - 1)) + sign;
                break;
            }
            case 2:
                mode++;
                break;
            case 1:
                mode--;
                break;
            }
            s->coding_mode[i] = mode;
        }

        i = 0;
        while (i < wlength) {
            int len = 0;

            mode = s->coding_mode[i];
            do {
                if (i >= wlength - 1)
                    len += rval;
                else
                    len += s->uval;
                i++;

                if (i == wlength)
                    break;
            } while (s->coding_mode[i] == mode);

            if ((ret = decode_segment(s, mode, decoded, len)) < 0)
                return ret;
            decoded += len;
        }
    } else {
        mode = get_bits(gb, 6);
        if ((ret = decode_segment(s, mode, decoded, length)) < 0)
            return ret;
    }

    return 0;
}

static int get_bits_esc4(GetBitContext *gb)
{
    if (get_bits1(gb))
        return get_bits(gb, 4) + 1;
    else
        return 0;
}

static int decode_subframe(TAKDecContext *s, int32_t *decoded,
                           int subframe_size, int prev_subframe_size)
{
    GetBitContext *gb = &s->gb;
    int x, y, i, j, ret = 0;
    int dshift, size, filter_quant, filter_order;
    int tfilter[MAX_PREDICTORS];

    if (!get_bits1(gb))
        return decode_residues(s, decoded, subframe_size);

    filter_order = predictor_sizes[get_bits(gb, 4)];

    if (prev_subframe_size > 0 && get_bits1(gb)) {
        if (filter_order > prev_subframe_size)
            return AVERROR_INVALIDDATA;

        decoded       -= filter_order;
        subframe_size += filter_order;

        if (filter_order > subframe_size)
            return AVERROR_INVALIDDATA;
    } else {
        int lpc_mode;

        if (filter_order > subframe_size)
            return AVERROR_INVALIDDATA;

        lpc_mode = get_bits(gb, 2);
        if (lpc_mode > 2)
            return AVERROR_INVALIDDATA;

        if ((ret = decode_residues(s, decoded, filter_order)) < 0)
            return ret;

        if (lpc_mode)
            decode_lpc(decoded, lpc_mode, filter_order);
    }

    dshift = get_bits_esc4(gb);
    size   = get_bits1(gb) + 6;

    filter_quant = 10;
    if (get_bits1(gb)) {
        filter_quant -= get_bits(gb, 3) + 1;
        if (filter_quant < 3)
            return AVERROR_INVALIDDATA;
    }

    if (get_bits_left(gb) < 2*10 + 2*size)
        return AVERROR_INVALIDDATA;

    s->predictors[0] = get_sbits(gb, 10);
    s->predictors[1] = get_sbits(gb, 10);
    s->predictors[2] = get_sbits(gb, size) * (1 << (10 - size));
    s->predictors[3] = get_sbits(gb, size) * (1 << (10 - size));
    if (filter_order > 4) {
        int tmp = size - get_bits1(gb);

        for (i = 4; i < filter_order; i++) {
            if (!(i & 3))
                x = tmp - get_bits(gb, 2);
            s->predictors[i] = get_sbits(gb, x) * (1 << (10 - size));
        }
    }

    tfilter[0] = s->predictors[0] * 64;
    for (i = 1; i < filter_order; i++) {
        uint32_t *p1 = &tfilter[0];
        uint32_t *p2 = &tfilter[i - 1];

        for (j = 0; j < (i + 1) / 2; j++) {
            x     = *p1 + ((int32_t)(s->predictors[i] * *p2 + 256) >> 9);
            *p2  += (int32_t)(s->predictors[i] * *p1 + 256) >> 9;
            *p1++ = x;
            p2--;
        }

        tfilter[i] = s->predictors[i] * 64;
    }

    x = 1 << (32 - (15 - filter_quant));
    y = 1 << ((15 - filter_quant) - 1);
    for (i = 0, j = filter_order - 1; i < filter_order / 2; i++, j--) {
        s->filter[j] = x - ((tfilter[i] + y) >> (15 - filter_quant));
        s->filter[i] = x - ((tfilter[j] + y) >> (15 - filter_quant));
    }

    if ((ret = decode_residues(s, &decoded[filter_order],
                               subframe_size - filter_order)) < 0)
        return ret;

    for (i = 0; i < filter_order; i++)
        s->residues[i] = *decoded++ >> dshift;

    y    = FF_ARRAY_ELEMS(s->residues) - filter_order;
    x    = subframe_size - filter_order;
    while (x > 0) {
        int tmp = FFMIN(y, x);

        for (i = 0; i < tmp; i++) {
            int v = 1 << (filter_quant - 1);

            if (filter_order & -16)
                v += (unsigned)s->adsp.scalarproduct_int16(&s->residues[i], s->filter,
                                                 filter_order & -16);
            for (j = filter_order & -16; j < filter_order; j += 4) {
                v += s->residues[i + j + 3] * (unsigned)s->filter[j + 3] +
                     s->residues[i + j + 2] * (unsigned)s->filter[j + 2] +
                     s->residues[i + j + 1] * (unsigned)s->filter[j + 1] +
                     s->residues[i + j    ] * (unsigned)s->filter[j    ];
            }
            v = (av_clip_intp2(v >> filter_quant, 13) * (1 << dshift)) - (unsigned)*decoded;
            *decoded++ = v;
            s->residues[filter_order + i] = v >> dshift;
        }

        x -= tmp;
        if (x > 0)
            memcpy(s->residues, &s->residues[y], 2 * filter_order);
    }

    return 0;
}

static int load_small(TAKDecContext *s, int *coeffs, const int fnum)
{
    GetBitContext *gb = &s->gb;

    for (int block_no = 0, i = 0; block_no < fnum / 4; block_no++) {
        int res = get_bits(gb, 4);

        if (res <= 0)
            return AVERROR_INVALIDDATA;

        for (int j = 0; j < 4; j++, i++)
             coeffs[i] = get_sbits_long(gb, res);
    }

    return 0;
}

static int load_big(TAKDecContext *s, int *coeffs, const int fnum)
{
    GetBitContext *gb = &s->gb;
    int max_res, res_bits;

    max_res = get_bits(gb, 4);
    res_bits = 2 + get_bits1(gb);

    for (int block_no = 0, i = 0; block_no < fnum / 4; block_no++) {
        int res = max_res - get_bits_long(gb, res_bits);

        if (res <= 0)
            return AVERROR_INVALIDDATA;

        for (int j = 0; j < 4; j++, i++)
             coeffs[i] = get_sbits_long(gb, res);
    }

    return 0;
}

static int diff_coeffs(TAKDecContext *s, int *coeffs, const int fnum)
{
    int max = FFABS(coeffs[0]);

    for (int idx = 1; idx < fnum; idx++) {
        coeffs[idx] = coeffs[idx] - coeffs[idx - 1];
        max = FFMAX(max, FFABS(coeffs[idx]));
    }

    if (max >= (1 << 13))
        return AVERROR_INVALIDDATA;

    return 0;
}

static int load_lpc_coeffs(TAKDecContext *s, ACoeffs *pf)
{
    GetBitContext *gb = &s->gb;
    int diff_flag, ret;

    pf->fshift_num = get_bits(gb, 3);

    if (pf->fnum >= 12) {
        diff_flag = get_bits1(gb);
    } else {
        diff_flag = 0;
    }

    memset(pf->coeffs, 0, sizeof(pf->coeffs));

    if (pf->fnum <= 16) {
        ret = load_small(s, pf->coeffs, pf->fnum);
    } else {
        ret = load_big(s, pf->coeffs, pf->fnum);
    }

    if (ret >= 0 && (diff_flag != 0))
        ret = diff_coeffs(s, pf->coeffs, pf->fnum);

    return ret;
}

static void decode_delta1(int32_t *decoded, const int size)
{
    int32_t sum = decoded[0];

    for (int n = 1; n < size; n++) {
        sum += decoded[n];
        decoded[n] = sum;
    }
}

static void decode_delta2(int32_t *decoded, const int size)
{
    int32_t sum0, sum1;

    if (size > 1) {
        sum0 = decoded[0];
        sum1 = decoded[1];
        sum0 += sum1;
        decoded[1] = sum0;

        for (int n = 2; n < size; n++) {
            sum1 += decoded[n];
            sum0 += sum1;
            decoded[n] = sum0;
        }
    }
}

static void shift_down(const int32_t *src, int32_t *dst,
                       const int N, const int shift)
{
    if (shift == 0) {
        memcpy(dst, src, N * sizeof(*dst));
    } else {
        for (int n = 0; n < N; n++) {
            int sign = src[n] >> 31;

            dst[n] = ((((src[n] + sign) ^ sign) >> shift) ^ sign) - sign;
        }
    }
}

static void lpc_decode_block(TAKDecContext *s, const ACoeffs *pf,
                             int32_t *decoded, const int size)
{
    const int pred_num = pf->fnum;
    const int sample_shift = pf->sample_shift;
    const int pred_shift = 10 - pf->fshift_num;
    const int pred_add = 1 << (pred_shift - 1);
    const int block_size_max = FF_ARRAY_ELEMS(s->lpc_work) - pred_num;
    const int *coeffs = pf->coeffs;
    int32_t *work = s->lpc_work;
    int32_t *samples = decoded + pred_num;
    int rest = size - pred_num;

    shift_down(decoded, work, pred_num, sample_shift);

    while (rest > 0) {
        const int block_size = FFMIN(rest, block_size_max);
        int32_t *pred = work;

        rest -= block_size;

        for (int n = 0; n < block_size; n++) {
            int32_t sum = 0;
            int val, sign;

            for (int i = 0; i < pred_num; i++)
                sum += pred[i] * coeffs[i];

            val  = sum;
            sign = val >> 31;
            val  = (((val + sign) ^ sign) + pred_add) >> pred_shift;
            val  = FFMIN(val, 8191);
            val  = (val ^ sign) - sign;

            samples[0] = val * (1 << sample_shift) - samples[0];

            sign            = samples[0] >> 31;
            pred[pred_num]  = ((((samples[0] + sign) ^ sign) >> sample_shift) ^ sign) - sign;

            pred++;
            samples++;
        }

        if (rest > 0)
            memmove(work, work + block_size_max, pred_num * sizeof(*work));
    }
}

static int bcb_decompress_samples(TAKDecContext *s, int32_t *decoded, const int size,
                                  uint8_t idx, int max_bits)
{
    GetBitContext *gb = &s->gb;
    int c1_bit_num_l = ycodes[idx].c1_bit_num;
    int c2a_min_val_l = ycodes[idx].c2a_min_val;
    int c2a_add = (c2a_min_val_l >> 1) - c2a_min_val_l;
    int c2b_min_crit_l = ycodes[idx].c2b_min_crit;
    int c2b_add = (ycodes[idx].c2b_min_val >> 1) - c2b_min_crit_l;
    int c3_add = (ycodes[idx].c3_min_val >> 1) - c2b_min_crit_l;
    int c2b_mul = ycodes[idx].c2b_size >> 1;
    int end_code_bits = max_bits - 10;

    for (int i = 0; i < size; i++) {
        int32_t abs_val = get_bits_long(gb, c1_bit_num_l);

        if (abs_val >= c2a_min_val_l) {
            int sign = get_bits1(gb);

            if (abs_val >= c2b_min_crit_l) {
                int stop_num = get_unary(gb, 1, 9);

                if (stop_num < 9) {
                    abs_val += c2b_add + stop_num * c2b_mul;
                } else {
                    abs_val += c3_add;
                    if (end_code_bits > 0)
                        abs_val += get_bits_long(gb, end_code_bits) * (unsigned)c2b_mul;
                }
            } else {
                abs_val += c2a_add;
            }

            decoded[i] = abs_val ^ -sign;
        } else {
            decoded[i] = (abs_val >> 1) ^ -(abs_val & 1);
        }
    }

    return 0;
}

static int bca_decompress_samples(TAKDecContext *s, int32_t *decoded, const int size,
                                  uint8_t idx, int max_bits)
{
    GetBitContext *gb = &s->gb;
    int end_code_neg_ofs = 1 << (max_bits-1);
    int c1_bits, c2_min_val;

    c1_bits = zcodes[idx].c1_bit_num;
    c2_min_val = zcodes[idx].c2_min_val;

    for (int i = 0; i < size; i++) {
        int bits = get_bits_long(gb, c1_bits);

        if (bits) {
            if (bits <= c2_min_val)
                decoded[i] = bits - 1;
            else
                decoded[i] = -(bits - c2_min_val);
        } else {
            bits = get_unary(gb, 1, 4);
            if (bits < 4) {
                if (get_bits1(gb))
                    decoded[i] = -(bits + c2_min_val);
                else
                    decoded[i] = bits + c2_min_val;
            } else {
                bits = get_bits_long(gb, max_bits);
                if (bits < end_code_neg_ofs) {
                    decoded[i] = c2_min_val + 4 + bits;
                } else {
                    decoded[i] = -(c2_min_val + 4 + bits - end_code_neg_ofs);
                }
            }
        }
    }

    return 0;
}

static int bco_decompress_samples(TAKDecContext *s, int32_t *decoded, const int size,
                                  const int code, int max_bits)
{
    int ret;

    if (code >= BCOB_FIRST && code <= BCOB_LAST) {
        ret = bcb_decompress_samples(s, decoded, size, code-BCOB_FIRST, max_bits);
    } else if (code >= BCOA_FIRST && code <= BCOA_LAST) {
        ret = bca_decompress_samples(s, decoded, size, code-BCOA_FIRST, max_bits);
    } else if (code == BCO_SILENCE) {
        for (int i = 0; i < size; i++)
            decoded[i] = 0;
        ret = 0;
    } else {
        ret = AVERROR_INVALIDDATA;
    }

    return ret;
}


static int decompress_code_diff_code_bits_10(TAKDecContext *s, int nb_codes, uint8_t *codes)
{
    GetBitContext *gb = &s->gb;
    int bits, code_bits;

    bits = get_bits(gb, 6);
    if (bits < BCO_SILENCE || bits > BCO_LAST)
        return AVERROR_INVALIDDATA;
    code_bits = get_bits(gb, 3) + 1;

    for (int i = 0; i < nb_codes; i++) {
        int next_code = bits + get_bits_long(gb, code_bits);

        if (next_code < BCO_SILENCE || next_code > BCO_LAST)
            return AVERROR_INVALIDDATA;

        codes[i] = next_code;
    }

    return 0;
}

static int decompress_code_diff_code_bits_11(TAKDecContext *s, int nb_codes, uint8_t *codes)
{
    GetBitContext *gb = &s->gb;
    int last_code;

    last_code = get_bits(gb, 6);
    if (last_code < BCO_SILENCE || last_code > BCO_LAST)
        return AVERROR_INVALIDDATA;

    codes[0] = last_code;

    for (int i = 1; i < nb_codes; i++) {
        int stop_pos = get_unary(gb, 1, 9);

        if (stop_pos > 0) {
            if (stop_pos < 9) {
                if (get_bits1(gb))
                    last_code -= stop_pos;
                else
                    last_code += stop_pos;
            } else {
                last_code = get_bits(gb, 6);
            }

            if (last_code < BCO_SILENCE || last_code > BCO_LAST)
                return AVERROR_INVALIDDATA;
        }

        codes[i] = last_code;
    }

    return 0;
}

static int decompress_code_diff_code_bits_21(TAKDecContext *s, int nb_codes, uint8_t *codes)
{
    GetBitContext *gb = &s->gb;
    int last_code;

    last_code = get_bits(gb, 6);
    if (last_code < BCO_SILENCE || last_code > BCO_LAST)
        return AVERROR_INVALIDDATA;

    codes[0] = last_code;

    for (int i = 1; i < nb_codes; i++) {
        int bits = get_bits(gb, 2);

        if (bits == 0) {
            int stop_pos = get_unary(gb, 1, 7);

            if (stop_pos < 7) {
                if (get_bits1(gb)) {
                    last_code -= stop_pos + 2;
                } else {
                    last_code += stop_pos + 2;
                }
            } else {
                last_code = get_bits(gb, 6);
            }
        } else if (bits == 2) {
            last_code--;
        } else if (bits == 3) {
            last_code++;
        }

        if (last_code < BCO_SILENCE || last_code > BCO_LAST)
            return AVERROR_INVALIDDATA;

        codes[i] = last_code;
    }

    return 0;
}

static int decompress_code_diff_code_bits(TAKDecContext *s, int nb_codes, uint8_t *codes)
{
    GetBitContext *gb = &s->gb;
    int diff_code, ret;

    diff_code = get_bits(gb, 2);
    switch (diff_code) {
    case BC_CDC_10:
        ret = decompress_code_diff_code_bits_10(s, nb_codes, codes);
        break;
    case BC_CDC_11:
        ret = decompress_code_diff_code_bits_11(s, nb_codes, codes);
        break;
    case BC_CDC_21:
        ret = decompress_code_diff_code_bits_21(s, nb_codes, codes);
        break;
    default:
        ret = AVERROR_INVALIDDATA;
        break;
    }

    return ret;
}

/**
 * Number of bits needed to hold nb_values distinct values.
 */
static int bit_need_for_value_num(int nb_values)
{
    if (nb_values <= 2)
        return 1;

    return av_ceil_log2(nb_values);
}

static int bco_calc_max_bits(int max_diff, int bit_code)
{
    if (bit_code >= BCOB_FIRST && bit_code <= BCOB_LAST) {
        const struct CParam0 *p = &ycodes[bit_code - BCOB_FIRST];

        max_diff *= 2;
        if (max_diff >= (p->c3_min_val + p->c2b_size))
            return bit_need_for_value_num((max_diff - p->c3_min_val) / p->c2b_size + 1) + 10;
        return 10;
    } else if (bit_code >= BCOA_FIRST && bit_code <= BCOA_LAST) {
        const struct CParam1 *p = &zcodes[bit_code - BCOA_FIRST];

        if (max_diff > p->c2_max_val)
            return 1 + bit_need_for_value_num(max_diff - p->c2_max_val);
        return 1;
    }

    return 1;
}

static int decompress_samples(TAKDecContext *s, int32_t *decoded, const int size)
{
    GetBitContext *gb = &s->gb;
    int ret, bits, max_bits;

    if (get_bits_left(gb) < 0)
        return AVERROR_INVALIDDATA;

    if (get_bits1(gb) == 0) {
        bits = get_bits(gb, 6);
        if (bits > BCO_LAST)
            return AVERROR_INVALIDDATA;

        if (bits == BCO_SILENCE) {
            memset(decoded, 0, size * sizeof(*decoded));
            return 0;
        }

        max_bits = get_bits(gb, 5);
        if (max_bits < 0 || max_bits > 28)
            return AVERROR_INVALIDDATA;

        return bco_decompress_samples(s, decoded, size, bits, max_bits);
    } else {
        uint8_t block_codes[128] = { 0 };
        int32_t *block = decoded;
        int nb_blocks, max_diff, left = size, i = 0;

        sub_frame_positions_setup(&s->bc_spos, &s->bcpos, size, s->bc_resolution_max,
                                  s->avctx->sample_rate);

        nb_blocks = s->bc_spos.nb_blocks;
        if (nb_blocks <= 1 || nb_blocks > FF_ARRAY_ELEMS(block_codes))
            return AVERROR_INVALIDDATA;

        ret = decompress_code_diff_code_bits(s, nb_blocks, block_codes);
        if (ret < 0)
            return ret;

        max_bits = get_bits(gb, 5);
        if (max_bits < 0 || max_bits > 28)
            return AVERROR_INVALIDDATA;

        if (max_bits > 0)
            max_diff = 1 << max_bits;
        else
            max_diff = 0;

        while (i < nb_blocks) {
            uint8_t act_code = block_codes[i];
            int block_size = 0;

            do {
                block_size += get_block_size(&s->bc_spos, i);
                i++;
            } while (i < nb_blocks && block_codes[i] == act_code);

            if (block_size <= 0 || block_size > left)
                return AVERROR_INVALIDDATA;

            ret = bco_decompress_samples(s, block, block_size, act_code,
                                         bco_calc_max_bits(max_diff, act_code));
            if (ret < 0)
                return ret;

            block += block_size;
            left  -= block_size;
        }
    }

    return 0;
}

static int decode_delta(TAKDecContext *s, int32_t *decoded, const int size)
{
    GetBitContext *gb = &s->gb;
    int ret, delta_type;

    delta_type = get_bits(gb, 2);
    if (delta_type > 2)
        return AVERROR_INVALIDDATA;

    ret = decompress_samples(s, decoded, size);
    if (ret < 0)
        return ret;

    switch (delta_type) {
    case 1:
        decode_delta1(decoded, size);
        break;
    case 2:
        decode_delta2(decoded, size);
        break;
    }

    return 0;
}

static int decode_lpc0(TAKDecContext *s, int32_t *decoded, const int size)
{
    GetBitContext *gb = &s->gb;
    ACoeffs pf = { 0 };
    int ret;

    pf.sample_shift = get_bits(gb, 4);
    pf.fnum         = pred_types[get_bits(gb, 4)];
    if (pf.fnum > size)
        return AVERROR_INVALIDDATA;

    ret = decode_delta(s, decoded, pf.fnum);
    if (ret < 0)
        return ret;

    ret = load_lpc_coeffs(s, &pf);
    if (ret < 0)
        return ret;

    ret = decompress_samples(s, decoded + pf.fnum, size - pf.fnum);
    if (ret < 0)
        return ret;

    lpc_decode_block(s, &pf, decoded, size);

    return 0;
}

static int decode_subframe0(TAKDecContext *s, int32_t *decoded, const int size)
{
    if (get_bits1(&s->gb))
        return decode_lpc0(s, decoded, size);
    else
        return decode_delta(s, decoded, size);
}

static void decode_channel_filters(TAKDecContext *s, const int chan)
{
    const int wasted_bits = s->wasted_bits[chan];
    int32_t *decoded = s->decoded[chan];
    const int size = s->nb_samples;

    if (s->prefilter[chan])
        lpc_decode_block(s, &s->pcoeffs[chan], decoded + 1, size - 1);

    if (wasted_bits > 0) {
        for (int n = 1; n < size; n++)
            decoded[n] = decoded[n] * (1 << wasted_bits);
    }

    decode_delta1(decoded, size);
}

static int decode_channel0(TAKDecContext *s, const int chan)
{
    AVCodecContext *avctx = s->avctx;
    GetBitContext *gb     = &s->gb;
    ACoeffs *pf           = &s->pcoeffs[chan];
    int32_t *decoded      = s->decoded[chan];
    const int sample_num  = s->nb_samples - 1;
    int prev_bpos = 0, prev_spos = 0, i, ret;

    s->wasted_bits[chan] = get_bits_esc4(gb);
    if (s->wasted_bits[chan] >= avctx->bits_per_raw_sample)
        return AVERROR_INVALIDDATA;

    *decoded++ = get_sbits(gb, avctx->bits_per_raw_sample - s->wasted_bits[chan]) * (1 << s->wasted_bits[chan]);

    if (get_bits1(gb)) {
        s->prefilter[chan] = 1;
        pf->fnum = 4;
        pf->sample_shift = get_bits(gb, 4);

        if (get_bits(gb, 2) != 0)
            return AVERROR_INVALIDDATA;

        ret = load_lpc_coeffs(s, pf);
        if (ret < 0)
            return ret;
    } else {
        s->prefilter[chan] = 0;
    }

    s->nb_subframes = get_bits(gb, 3) + 1;
    if (s->nb_subframes > 5)
        return AVERROR_INVALIDDATA;

    if (get_bits_left(gb) < (s->nb_subframes - 1) * 6)
        return AVERROR_INVALIDDATA;

    for (i = 0; i < s->nb_subframes - 1; i++) {
        int bpos = get_bits(gb, 6), spos;

        if (bpos <= prev_bpos || bpos >= s->spos.nb_blocks)
            return AVERROR_INVALIDDATA;

        spos = block_pos_to_sample_pos(&s->spos, bpos);

        s->subframe_len[i] = spos - prev_spos;
        if (s->subframe_len[i] <= 0)
            return AVERROR_INVALIDDATA;

        prev_bpos = bpos;
        prev_spos = spos;
    }
    s->subframe_len[i] = sample_num - prev_spos;
    if (s->subframe_len[i] <= 0)
        return AVERROR_INVALIDDATA;

    for (i = 0; i < s->nb_subframes; i++) {
        if ((ret = decode_subframe0(s, decoded, s->subframe_len[i])) < 0)
            return ret;
        decoded += s->subframe_len[i];
    }

    return 0;
}

static int load_joint_info(TAKDecContext *s)
{
    GetBitContext *gb = &s->gb;

    s->jcoeffs.fnum = 8;
    s->jsub_start   = get_bits1(gb);
    s->jsub_end     = get_bits1(gb);
    s->jshift       = get_bits(gb, 4);
    if (get_bits(gb, 2) != 1)
        return AVERROR_INVALIDDATA;

    return load_lpc_coeffs(s, &s->jcoeffs);
}

static int decode_joint_lpc(TAKDecContext *s, const int32_t *ca, int32_t *cb,
                            const int size)
{
    const int block_size_max = FF_ARRAY_ELEMS(s->lpc_work) - 8;
    const int *coeffs = s->jcoeffs.coeffs;
    const int wshift = s->jshift;
    int32_t *work = s->lpc_work;
    int32_t *samples = cb + 4;
    int predict_num, block_start = 0;

    if (size < 256)
        return AVERROR_INVALIDDATA;

    predict_num = size - 7;

    if (s->jsub_start) {
        for (int n = 0; n < 4; n++)
            cb[n] += ca[n];
    }

    if (s->jsub_end) {
        for (int n = 4 + predict_num; n < size; n++)
            cb[n] += ca[n];
    }

    shift_down(ca, work, 8, wshift);

    while (predict_num > 0) {
        const int block_size = FFMIN(predict_num, block_size_max);
        int32_t *pred = work;

        predict_num -= block_size;

        shift_down(ca + 8 + block_start, work + 8, block_size, wshift);

        for (int n = 0; n < block_size; n++) {
            unsigned sum = 0;
            int val, sign;

            for (int i = 0; i < 8; i++)
                sum += (unsigned)pred[i] * coeffs[i];

            val  = sum;
            sign = val >> 31;
            val  = (((val + sign) ^ sign) + 512) >> 10;
            val  = FFMIN(val, 8191);
            val  = (val ^ sign) - sign;

            samples[0] = val * (1 << wshift) - samples[0];

            pred++;
            samples++;
        }

        memmove(work, work + block_size, 8 * sizeof(*work));

        block_start += block_size;
    }

    return 0;
}

static int decode_joint_channels0(TAKDecContext *s, const int mode)
{
    int32_t *ca = s->decoded[0] + 1;
    int32_t *cb = s->decoded[1] + 1;
    const int size = s->nb_samples - 1;

    switch (mode) {
    case TAK_JOINT_OFF:
        break;
    case TAK_JOINT_LEFT:
        return decode_joint_lpc(s, ca, cb, size);
    case TAK_JOINT_RIGHT:
        return decode_joint_lpc(s, cb, ca, size);
    case TAK_JOINT_LEFT_SIDE:
        for (int n = 0; n < size; n++)
            cb[n] += ca[n];
        break;
    case TAK_JOINT_RIGHT_SIDE:
        for (int n = 0; n < size; n++)
            ca[n] = cb[n] - ca[n];
        break;
    case TAK_JOINT_MID_SIDE:
        for (int n = 0; n < size; n++) {
            int side = cb[n];
            int mid  = 2 * ca[n] + (side & 1);

            ca[n] = (mid - side) >> 1;
            cb[n] = (mid + side) >> 1;
        }
        break;
    }

    return 0;
}

static int decode_stereo0(TAKDecContext *s)
{
    GetBitContext *gb = &s->gb;
    int mode, first, ret;

    mode = get_bits(gb, 3);
    if (mode > TAK_JOINT_MID_SIDE)
        return AVERROR_INVALIDDATA;

    first = mode == TAK_JOINT_RIGHT || mode == TAK_JOINT_RIGHT_SIDE;

    if ((ret = decode_channel0(s, first)) < 0)
        return ret;

    if (mode == TAK_JOINT_LEFT || mode == TAK_JOINT_RIGHT) {
        if ((ret = load_joint_info(s)) < 0)
            return ret;
    }

    if ((ret = decode_channel0(s, !first)) < 0)
        return ret;

    if (mode >= TAK_JOINT_LEFT_SIDE && (s->prefilter[0] || s->prefilter[1]))
        return AVERROR_INVALIDDATA;

    return decode_joint_channels0(s, mode);
}

static int decode_channel(TAKDecContext *s, int chan)
{
    AVCodecContext *avctx = s->avctx;
    GetBitContext *gb     = &s->gb;
    int32_t *decoded      = s->decoded[chan];
    int left              = s->nb_samples - 1;
    int i = 0, ret, prev = 0;

    s->sample_shift[chan] = get_bits_esc4(gb);
    if (s->sample_shift[chan] >= avctx->bits_per_raw_sample)
        return AVERROR_INVALIDDATA;

    *decoded++ = get_sbits(gb, avctx->bits_per_raw_sample - s->sample_shift[chan]);
    s->lpc_mode[chan] = get_bits(gb, 2);
    s->nb_subframes   = get_bits(gb, 3) + 1;

    if (s->nb_subframes > 1) {
        if (get_bits_left(gb) < (s->nb_subframes - 1) * 6)
            return AVERROR_INVALIDDATA;

        for (; i < s->nb_subframes - 1; i++) {
            int v = get_bits(gb, 6);

            s->subframe_len[i] = (v - prev) * s->subframe_scale;
            if (s->subframe_len[i] <= 0)
                return AVERROR_INVALIDDATA;

            left -= s->subframe_len[i];
            prev  = v;
        }

        if (left <= 0)
            return AVERROR_INVALIDDATA;
    }
    s->subframe_len[i] = left;

    prev = 0;
    for (i = 0; i < s->nb_subframes; i++) {
        if ((ret = decode_subframe(s, decoded, s->subframe_len[i], prev)) < 0)
            return ret;
        decoded += s->subframe_len[i];
        prev     = s->subframe_len[i];
    }

    return 0;
}

static int decorrelate(TAKDecContext *s, int c1, int c2, int length)
{
    GetBitContext *gb = &s->gb;
    int32_t *p1       = s->decoded[c1] + (s->dmode > 5);
    int32_t *p2       = s->decoded[c2] + (s->dmode > 5);
    int32_t bp1       = p1[0];
    int32_t bp2       = p2[0];
    int i;
    int dshift, dfactor;

    length += s->dmode < 6;

    switch (s->dmode) {
    case 1: /* left/side */
        s->tdsp.decorrelate_ls(p1, p2, length);
        break;
    case 2: /* side/right */
        s->tdsp.decorrelate_sr(p1, p2, length);
        break;
    case 3: /* side/mid */
        s->tdsp.decorrelate_sm(p1, p2, length);
        break;
    case 4: /* side/left with scale factor */
        FFSWAP(int32_t*, p1, p2);
        FFSWAP(int32_t, bp1, bp2);
        av_fallthrough;
    case 5: /* side/right with scale factor */
        dshift  = get_bits_esc4(gb);
        dfactor = get_sbits(gb, 10);
        s->tdsp.decorrelate_sf(p1, p2, length, dshift, dfactor);
        break;
    case 6:
        FFSWAP(int32_t*, p1, p2);
        av_fallthrough;
    case 7: {
        int length2, order_half, filter_order, dval1, dval2;
        int tmp, x, code_size;

        if (length < 256)
            return AVERROR_INVALIDDATA;

        dshift       = get_bits_esc4(gb);
        filter_order = 8 << get_bits1(gb);
        dval1        = get_bits1(gb);
        dval2        = get_bits1(gb);

        for (i = 0; i < filter_order; i++) {
            if (!(i & 3))
                code_size = 14 - get_bits(gb, 3);
            s->filter[i] = get_sbits(gb, code_size);
        }

        order_half = filter_order / 2;
        length2    = length - (filter_order - 1);

        /* decorrelate beginning samples */
        if (dval1) {
            for (i = 0; i < order_half; i++) {
                int32_t a = p1[i];
                int32_t b = p2[i];
                p1[i]     = a + b;
            }
        }

        /* decorrelate ending samples */
        if (dval2) {
            for (i = length2 + order_half; i < length; i++) {
                int32_t a = p1[i];
                int32_t b = p2[i];
                p1[i]     = a + b;
            }
        }


        for (i = 0; i < filter_order; i++)
            s->residues[i] = *p2++ >> dshift;

        p1 += order_half;
        x = FF_ARRAY_ELEMS(s->residues) - filter_order;
        for (; length2 > 0; length2 -= tmp) {
            tmp = FFMIN(length2, x);

            for (i = 0; i < tmp - (tmp == length2); i++)
                s->residues[filter_order + i] = *p2++ >> dshift;

            for (i = 0; i < tmp; i++) {
                int v = 1 << 9;

                if (filter_order == 16) {
                    v += s->adsp.scalarproduct_int16(&s->residues[i], s->filter,
                                                     filter_order);
                } else {
                    v += s->residues[i + 7] * s->filter[7] +
                         s->residues[i + 6] * s->filter[6] +
                         s->residues[i + 5] * s->filter[5] +
                         s->residues[i + 4] * s->filter[4] +
                         s->residues[i + 3] * s->filter[3] +
                         s->residues[i + 2] * s->filter[2] +
                         s->residues[i + 1] * s->filter[1] +
                         s->residues[i    ] * s->filter[0];
                }

                v = av_clip_intp2(v >> 10, 13) * (1U << dshift) - *p1;
                *p1++ = v;
            }

            memmove(s->residues, &s->residues[tmp], 2 * filter_order);
        }
        break;
    }
    }

    if (s->dmode > 0 && s->dmode < 6) {
        p1[0] = bp1;
        p2[0] = bp2;
    }

    return 0;
}

static int tak_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *pkt)
{
    TAKDecContext *s  = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int chan, i, ret, hsize;

    if (pkt->size < TAK_MIN_FRAME_HEADER_BYTES)
        return AVERROR_INVALIDDATA;

    if ((ret = init_get_bits8(gb, pkt->data, pkt->size)) < 0)
        return ret;

    if ((ret = ff_tak_decode_frame_header(avctx, gb, &s->ti, 0)) < 0)
        return ret;

    hsize = get_bits_count(gb) / 8;
    if (avctx->err_recognition & (AV_EF_CRCCHECK|AV_EF_COMPLIANT)) {
        if (ff_tak_check_crc(pkt->data, hsize)) {
            av_log(avctx, AV_LOG_ERROR, "CRC error\n");
            if (avctx->err_recognition & AV_EF_EXPLODE)
                return AVERROR_INVALIDDATA;
        }
    }

    if (s->ti.codec != TAK_CODEC_MONO_STEREO_OLD &&
        s->ti.codec != TAK_CODEC_MONO_STEREO &&
        s->ti.codec != TAK_CODEC_MULTICHANNEL) {
        avpriv_report_missing_feature(avctx, "TAK codec type %d", s->ti.codec);
        return AVERROR_PATCHWELCOME;
    }
    if (s->ti.data_type) {
        av_log(avctx, AV_LOG_ERROR,
               "unsupported data type: %d\n", s->ti.data_type);
        return AVERROR_INVALIDDATA;
    }
    if ((s->ti.codec == TAK_CODEC_MONO_STEREO_OLD || s->ti.codec == TAK_CODEC_MONO_STEREO) && s->ti.channels > 2) {
        av_log(avctx, AV_LOG_ERROR,
               "invalid number of channels: %d\n", s->ti.channels);
        return AVERROR_INVALIDDATA;
    }
    if (s->ti.channels > 6) {
        av_log(avctx, AV_LOG_ERROR,
               "unsupported number of channels: %d\n", s->ti.channels);
        return AVERROR_INVALIDDATA;
    }

    if (s->ti.frame_samples <= 0) {
        av_log(avctx, AV_LOG_ERROR, "unsupported/invalid number of samples\n");
        return AVERROR_INVALIDDATA;
    }

    avctx->bits_per_raw_sample = s->ti.bps;
    if ((ret = set_bps_params(avctx)) < 0)
        return ret;
    if (s->ti.sample_rate != avctx->sample_rate) {
        avctx->sample_rate = s->ti.sample_rate;
        set_sample_rate_params(avctx);
    }

    av_channel_layout_uninit(&avctx->ch_layout);
    if (s->ti.ch_layout) {
        av_channel_layout_from_mask(&avctx->ch_layout, s->ti.ch_layout);
    } else {
        avctx->ch_layout.order       = AV_CHANNEL_ORDER_UNSPEC;
        avctx->ch_layout.nb_channels = s->ti.channels;
    }

    s->nb_samples = s->ti.last_frame_samples ? s->ti.last_frame_samples
                                             : s->ti.frame_samples;

    frame->nb_samples = s->nb_samples;
    if ((ret = ff_thread_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    ff_thread_finish_setup(avctx);

    if (avctx->bits_per_raw_sample <= 16) {
        int buf_size = av_samples_get_buffer_size(NULL, avctx->ch_layout.nb_channels,
                                                  s->nb_samples,
                                                  AV_SAMPLE_FMT_S32P, 0);
        if (buf_size < 0)
            return buf_size;
        av_fast_malloc(&s->decode_buffer, &s->decode_buffer_size, buf_size);
        if (!s->decode_buffer)
            return AVERROR(ENOMEM);
        ret = av_samples_fill_arrays((uint8_t **)s->decoded, NULL,
                                     s->decode_buffer, avctx->ch_layout.nb_channels,
                                     s->nb_samples, AV_SAMPLE_FMT_S32P, 0);
        if (ret < 0)
            return ret;
    } else {
        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++)
            s->decoded[chan] = (int32_t *)frame->extended_data[chan];
    }

    if (s->nb_samples < 16) {
        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++) {
            int32_t *decoded = s->decoded[chan];
            for (i = 0; i < s->nb_samples; i++)
                decoded[i] = get_sbits(gb, avctx->bits_per_raw_sample);
        }
    } else {
        if (s->ti.codec == TAK_CODEC_MONO_STEREO_OLD) {
            const int frame_size_max = FFMAX(s->ti.frame_samples, s->nb_samples);

            if (frame_size_max  != s->pos_frame_size ||
                avctx->sample_rate != s->pos_sample_rate) {
                frame_positions_create(&s->fpos, avctx->sample_rate,
                                       frame_size_max, 2, 64, 8, 8);
                frame_positions_create(&s->bcpos, avctx->sample_rate,
                                       frame_size_max, 3, 128, 4, 4);
                s->bc_resolution_max = FFMIN(bco_opt_partition_res(avctx->sample_rate,
                                                                   avctx->bits_per_raw_sample),
                                             s->bcpos.resolution);
                s->pos_frame_size  = frame_size_max;
                s->pos_sample_rate = avctx->sample_rate;
            }

            sub_frame_positions_setup(&s->spos, &s->fpos, s->nb_samples,
                                      s->fpos.resolution, avctx->sample_rate);

            if (avctx->ch_layout.nb_channels == 2) {
                if ((ret = decode_stereo0(s)) < 0)
                    return ret;
            } else {
                if ((ret = decode_channel0(s, 0)) < 0)
                    return ret;
            }

            for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++)
                decode_channel_filters(s, chan);
        } else if (s->ti.codec == TAK_CODEC_MONO_STEREO) {
            for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++)
                if (ret = decode_channel(s, chan))
                    return ret;

            if (avctx->ch_layout.nb_channels == 2) {
                s->nb_subframes = get_bits1(gb) + 1;
                if (s->nb_subframes > 1) {
                    s->subframe_len[1] = get_bits(gb, 6);
                }

                s->dmode = get_bits(gb, 3);
                if (ret = decorrelate(s, 0, 1, s->nb_samples - 1))
                    return ret;
            }
        } else if (s->ti.codec == TAK_CODEC_MULTICHANNEL) {
            if (get_bits1(gb)) {
                int ch_mask = 0;

                chan = get_bits(gb, 4) + 1;
                if (chan > avctx->ch_layout.nb_channels)
                    return AVERROR_INVALIDDATA;

                for (i = 0; i < chan; i++) {
                    int nbit = get_bits(gb, 4);

                    if (nbit >= avctx->ch_layout.nb_channels)
                        return AVERROR_INVALIDDATA;

                    if (ch_mask & 1 << nbit)
                        return AVERROR_INVALIDDATA;

                    s->mcdparams[i].present = get_bits1(gb);
                    if (s->mcdparams[i].present) {
                        s->mcdparams[i].index = get_bits(gb, 2);
                        s->mcdparams[i].chan2 = get_bits(gb, 4);
                        if (s->mcdparams[i].chan2 >= avctx->ch_layout.nb_channels) {
                            av_log(avctx, AV_LOG_ERROR,
                                   "invalid channel 2 (%d) for %d channel(s)\n",
                                   s->mcdparams[i].chan2, avctx->ch_layout.nb_channels);
                            return AVERROR_INVALIDDATA;
                        }
                        if (s->mcdparams[i].index == 1) {
                            if ((nbit == s->mcdparams[i].chan2) ||
                                (ch_mask & 1 << s->mcdparams[i].chan2))
                                return AVERROR_INVALIDDATA;

                            ch_mask |= 1 << s->mcdparams[i].chan2;
                        } else if (!(ch_mask & 1 << s->mcdparams[i].chan2)) {
                            return AVERROR_INVALIDDATA;
                        }
                    }
                    s->mcdparams[i].chan1 = nbit;

                    ch_mask |= 1 << nbit;
                }
            } else {
                chan = avctx->ch_layout.nb_channels;
                for (i = 0; i < chan; i++) {
                    s->mcdparams[i].present = 0;
                    s->mcdparams[i].chan1   = i;
                }
            }

            for (i = 0; i < chan; i++) {
                if (s->mcdparams[i].present && s->mcdparams[i].index == 1)
                    if (ret = decode_channel(s, s->mcdparams[i].chan2))
                        return ret;

                if (ret = decode_channel(s, s->mcdparams[i].chan1))
                    return ret;

                if (s->mcdparams[i].present) {
                    s->dmode = mc_dmodes[s->mcdparams[i].index];
                    if (ret = decorrelate(s,
                                          s->mcdparams[i].chan2,
                                          s->mcdparams[i].chan1,
                                          s->nb_samples - 1))
                        return ret;
                }
            }
        }

        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++) {
            int32_t *decoded = s->decoded[chan];

            if (s->lpc_mode[chan])
                decode_lpc(decoded, s->lpc_mode[chan], s->nb_samples);

            if (s->sample_shift[chan] > 0)
                for (i = 0; i < s->nb_samples; i++)
                    decoded[i] *= 1U << s->sample_shift[chan];
        }
    }

    align_get_bits(gb);
    skip_bits(gb, 24);
    if (get_bits_left(gb) < 0)
        av_log(avctx, AV_LOG_DEBUG, "overread\n");
    else if (get_bits_left(gb) > 0)
        av_log(avctx, AV_LOG_DEBUG, "underread\n");

    if (avctx->err_recognition & (AV_EF_CRCCHECK | AV_EF_COMPLIANT)) {
        if (ff_tak_check_crc(pkt->data + hsize,
                             get_bits_count(gb) / 8 - hsize)) {
            av_log(avctx, AV_LOG_ERROR, "CRC error\n");
            if (avctx->err_recognition & AV_EF_EXPLODE)
                return AVERROR_INVALIDDATA;
        }
    }

    /* convert to output buffer */
    switch (avctx->sample_fmt) {
    case AV_SAMPLE_FMT_U8P:
        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++) {
            uint8_t *samples = (uint8_t *)frame->extended_data[chan];
            int32_t *decoded = s->decoded[chan];
            for (i = 0; i < s->nb_samples; i++)
                samples[i] = decoded[i] + 0x80U;
        }
        break;
    case AV_SAMPLE_FMT_S16P:
        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++) {
            int16_t *samples = (int16_t *)frame->extended_data[chan];
            int32_t *decoded = s->decoded[chan];
            for (i = 0; i < s->nb_samples; i++)
                samples[i] = decoded[i];
        }
        break;
    case AV_SAMPLE_FMT_S32P:
        for (chan = 0; chan < avctx->ch_layout.nb_channels; chan++) {
            int32_t *samples = (int32_t *)frame->extended_data[chan];
            for (i = 0; i < s->nb_samples; i++)
                samples[i] *= 1U << 8;
        }
        break;
    }

    *got_frame_ptr = 1;

    return pkt->size;
}

#if HAVE_THREADS
static int update_thread_context(AVCodecContext *dst,
                                 const AVCodecContext *src)
{
    TAKDecContext *tsrc = src->priv_data;
    TAKDecContext *tdst = dst->priv_data;

    if (dst == src)
        return 0;
    memcpy(&tdst->ti, &tsrc->ti, sizeof(TAKStreamInfo));
    return 0;
}
#endif

static av_cold int tak_decode_close(AVCodecContext *avctx)
{
    TAKDecContext *s = avctx->priv_data;

    av_freep(&s->decode_buffer);

    return 0;
}

const FFCodec ff_tak_decoder = {
    .p.name           = "tak",
    CODEC_LONG_NAME("TAK (Tom's lossless Audio Kompressor)"),
    .p.type           = AVMEDIA_TYPE_AUDIO,
    .p.id             = AV_CODEC_ID_TAK,
    .priv_data_size   = sizeof(TAKDecContext),
    .init             = tak_decode_init,
    .close            = tak_decode_close,
    FF_CODEC_DECODE_CB(tak_decode_frame),
    UPDATE_THREAD_CONTEXT(update_thread_context),
    .p.capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS | AV_CODEC_CAP_CHANNEL_CONF,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_U8P, AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P),
};
