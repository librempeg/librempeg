/*
 * Monkey's Audio lossless audio decoder
 * Copyright (c) 2007 Benjamin Zores <ben@geexbox.org>
 *  based upon libdemac from Dave Chapman.
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

#include <inttypes.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/crc.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "lossless_audiodsp.h"
#include "avcodec.h"
#include "bswapdsp.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "unary.h"

/**
 * @file
 * Monkey's Audio lossless audio decoder
 */

#define MAX_CHANNELS        2
#define MAX_BYTESPERSAMPLE  3

#define APE_FRAMECODE_MONO_SILENCE    1
#define APE_FRAMECODE_STEREO_SILENCE  3
#define APE_FRAMECODE_PSEUDO_STEREO   4

#define HISTORY_SIZE 512
#define PREDICTOR_ORDER 8
/** Total size of all predictor histories */
#define PREDICTOR_SIZE 50

#define YDELAYA (18 + PREDICTOR_ORDER*4)
#define YDELAYB (18 + PREDICTOR_ORDER*3)
#define XDELAYA (18 + PREDICTOR_ORDER*2)
#define XDELAYB (18 + PREDICTOR_ORDER)

#define YADAPTCOEFFSA 18
#define XADAPTCOEFFSA 14
#define YADAPTCOEFFSB 10
#define XADAPTCOEFFSB 5

/**
 * Possible compression levels
 * @{
 */
enum APECompressionLevel {
    COMPRESSION_LEVEL_FAST       = 1000,
    COMPRESSION_LEVEL_NORMAL     = 2000,
    COMPRESSION_LEVEL_HIGH       = 3000,
    COMPRESSION_LEVEL_EXTRA_HIGH = 4000,
    COMPRESSION_LEVEL_INSANE     = 5000
};
/** @} */

#define APE_FILTER_LEVELS 3

/** Filter orders depending on compression level */
static const uint16_t ape_filter_orders[5][APE_FILTER_LEVELS] = {
    {  0,   0,    0 },
    { 16,   0,    0 },
    { 64,   0,    0 },
    { 32, 256,    0 },
    { 16, 256, 1280 }
};

/** Filter fraction bits depending on compression level */
static const uint8_t ape_filter_fracbits[5][APE_FILTER_LEVELS] = {
    {  0,  0,  0 },
    { 11,  0,  0 },
    { 11,  0,  0 },
    { 10, 13,  0 },
    { 11, 13, 15 }
};


/** Filters applied to the decoded data */
typedef struct APEFilter {
    int16_t *coeffs;        ///< actual coefficients used in filtering
    int16_t *adaptcoeffs;   ///< adaptive filter coefficients used for correcting of actual filter coefficients
    int16_t *historybuffer; ///< filter memory
    int16_t *delay;         ///< filtered values

    uint32_t avg;
} APEFilter;

typedef struct APERice {
    uint32_t k;
    uint32_t ksum;
} APERice;

typedef struct APERangecoder {
    uint32_t low;           ///< low end of interval
    uint32_t range;         ///< length of interval
    uint32_t help;          ///< bytes_to_follow resp. intermediate value
    unsigned int buffer;    ///< buffer for input/output
} APERangecoder;

/** Filter histories */
typedef struct APEPredictor {
    int32_t *buf;

    int32_t lastA[2];

    int32_t filterA[2];
    int32_t filterB[2];

    uint32_t coeffsA[2][4];  ///< adaption coefficients
    uint32_t coeffsB[2][5];  ///< adaption coefficients
    int32_t historybuffer[HISTORY_SIZE + PREDICTOR_SIZE];

    unsigned int sample_pos;
} APEPredictor;

typedef struct APEPredictor64 {
    int64_t *buf;

    int64_t lastA[2];

    int64_t filterA[2];
    int64_t filterB[2];

    uint64_t coeffsA[2][4];  ///< adaption coefficients
    uint64_t coeffsB[2][5];  ///< adaption coefficients
    int64_t historybuffer[HISTORY_SIZE + PREDICTOR_SIZE];
} APEPredictor64;

/** Decoder context */
typedef struct APEContext {
    AVClass *class;                          ///< class for AVOptions
    AVCodecContext *avctx;
    BswapDSPContext bdsp;
    LLAudDSPContext adsp;
    int channels;
    int samples;                             ///< samples left to decode in current frame
    int bps;

    int fileversion;                         ///< codec version, very important in decoding process
    int compression_level;                   ///< compression levels
    int fset;                                ///< which filter set to use (calculated from compression level)
    int flags;                               ///< global decoder flags

    uint32_t CRC;                            ///< signalled frame CRC
    uint32_t CRC_state;                      ///< accumulated CRC
    int frameflags;                          ///< frame flags
    APEPredictor predictor;                  ///< predictor used for final reconstruction
    APEPredictor64 predictor64;              ///< 64bit predictor used for final reconstruction

    int32_t *decoded_buffer;
    int decoded_size;
    int32_t *decoded[MAX_CHANNELS];          ///< decoded data for each channel
    int32_t *interim_buffer;
    int interim_size;
    int32_t *interim[MAX_CHANNELS];          ///< decoded data for each channel
    int blocks_per_loop;                     ///< maximum number of samples to decode for each call

    int16_t* filterbuf[APE_FILTER_LEVELS];   ///< filter memory

    APERangecoder rc;                        ///< rangecoder used to decode actual values
    APERice riceX;                           ///< rice code parameters for the second channel
    APERice riceY;                           ///< rice code parameters for the first channel
    APEFilter filters[APE_FILTER_LEVELS][2]; ///< filters used for reconstruction
    GetBitContext gb;

    uint8_t *data;                           ///< current frame data
    uint8_t *data_end;                       ///< frame data end
    int data_size;                           ///< frame data allocated size
    const uint8_t *ptr;                      ///< current position in frame data

    int error;
    int interim_mode;

    void (*entropy_decode_mono)(struct APEContext *ctx, int blockstodecode);
    void (*entropy_decode_stereo)(struct APEContext *ctx, int blockstodecode);
    void (*predictor_decode_mono)(struct APEContext *ctx, int count);
    void (*predictor_decode_stereo)(struct APEContext *ctx, int count);
} APEContext;

static void ape_apply_filters(APEContext *ctx, int32_t *decoded0,
                              int32_t *decoded1, int count);

static void entropy_decode_mono_0000(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_0000(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3860(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3860(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3900(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3900(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3930(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3990(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3990(APEContext *ctx, int blockstodecode);

static void predictor_decode_mono_3800(APEContext *ctx, int count);
static void predictor_decode_stereo_3800(APEContext *ctx, int count);
static void predictor_decode_mono_3930(APEContext *ctx, int count);
static void predictor_decode_stereo_3930(APEContext *ctx, int count);
static void predictor_decode_mono_3950(APEContext *ctx, int count);
static void predictor_decode_stereo_3950(APEContext *ctx, int count);

static av_cold int ape_decode_close(AVCodecContext *avctx)
{
    APEContext *s = avctx->priv_data;
    int i;

    for (i = 0; i < APE_FILTER_LEVELS; i++)
        av_freep(&s->filterbuf[i]);

    av_freep(&s->decoded_buffer);
    av_freep(&s->interim_buffer);
    av_freep(&s->data);
    s->decoded_size = s->data_size = 0;

    return 0;
}

static av_cold int ape_decode_init(AVCodecContext *avctx)
{
    APEContext *s = avctx->priv_data;
    int channels = avctx->ch_layout.nb_channels;
    int i;

    if (avctx->extradata_size != 6) {
        av_log(avctx, AV_LOG_ERROR, "Incorrect extradata\n");
        return AVERROR(EINVAL);
    }
    if (channels > 2) {
        av_log(avctx, AV_LOG_ERROR, "Only mono and stereo is supported\n");
        return AVERROR(EINVAL);
    }
    avctx->bits_per_raw_sample =
    s->bps = avctx->bits_per_coded_sample;
    switch (s->bps) {
    case 8:
        avctx->sample_fmt = AV_SAMPLE_FMT_U8P;
        s->interim_mode = 0;
        break;
    case 16:
        avctx->sample_fmt = AV_SAMPLE_FMT_S16P;
        s->interim_mode = 0;
        break;
    case 24:
        avctx->sample_fmt = AV_SAMPLE_FMT_S32P;
        s->interim_mode = -1;
        break;
    default:
        avpriv_request_sample(avctx,
                              "%d bits per coded sample", s->bps);
        return AVERROR_PATCHWELCOME;
    }
    s->avctx             = avctx;
    s->channels          = channels;
    s->fileversion       = AV_RL16(avctx->extradata);
    s->compression_level = AV_RL16(avctx->extradata + 2);
    s->flags             = AV_RL16(avctx->extradata + 4);

    av_log(avctx, AV_LOG_VERBOSE, "Compression Level: %d - Flags: %d\n",
           s->compression_level, s->flags);
    if (s->compression_level % 1000 || s->compression_level > COMPRESSION_LEVEL_INSANE ||
        !s->compression_level ||
        (s->fileversion < 3930 && s->compression_level == COMPRESSION_LEVEL_INSANE)) {
        av_log(avctx, AV_LOG_ERROR, "Incorrect compression level %d\n",
               s->compression_level);
        return AVERROR_INVALIDDATA;
    }
    s->fset = s->compression_level / 1000 - 1;
    for (i = 0; i < APE_FILTER_LEVELS; i++) {
        if (!ape_filter_orders[s->fset][i])
            break;
        if (!(s->filterbuf[i] = av_malloc((ape_filter_orders[s->fset][i] * 3 + HISTORY_SIZE) * 4)))
            return AVERROR(ENOMEM);
    }

    if (s->fileversion < 3860) {
        s->entropy_decode_mono   = entropy_decode_mono_0000;
        s->entropy_decode_stereo = entropy_decode_stereo_0000;
    } else if (s->fileversion < 3900) {
        s->entropy_decode_mono   = entropy_decode_mono_3860;
        s->entropy_decode_stereo = entropy_decode_stereo_3860;
    } else if (s->fileversion < 3930) {
        s->entropy_decode_mono   = entropy_decode_mono_3900;
        s->entropy_decode_stereo = entropy_decode_stereo_3900;
    } else if (s->fileversion < 3990) {
        s->entropy_decode_mono   = entropy_decode_mono_3900;
        s->entropy_decode_stereo = entropy_decode_stereo_3930;
    } else {
        s->entropy_decode_mono   = entropy_decode_mono_3990;
        s->entropy_decode_stereo = entropy_decode_stereo_3990;
    }

    if (s->fileversion < 3930) {
        s->predictor_decode_mono   = predictor_decode_mono_3800;
        s->predictor_decode_stereo = predictor_decode_stereo_3800;
    } else if (s->fileversion < 3950) {
        s->predictor_decode_mono   = predictor_decode_mono_3930;
        s->predictor_decode_stereo = predictor_decode_stereo_3930;
    } else {
        s->predictor_decode_mono   = predictor_decode_mono_3950;
        s->predictor_decode_stereo = predictor_decode_stereo_3950;
    }

    ff_bswapdsp_init(&s->bdsp);
    ff_llauddsp_init(&s->adsp);
    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout = (channels == 2) ? (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO
                                       : (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    return 0;
}

/**
 * @name APE range decoding functions
 * @{
 */

#define CODE_BITS    32
#define TOP_VALUE    ((unsigned int)1 << (CODE_BITS-1))
#define SHIFT_BITS   (CODE_BITS - 9)
#define EXTRA_BITS   ((CODE_BITS-2) % 8 + 1)
#define BOTTOM_VALUE (TOP_VALUE >> 8)

/** Start the decoder */
static inline void range_start_decoding(APEContext *ctx)
{
    ctx->rc.buffer = bytestream_get_byte(&ctx->ptr);
    ctx->rc.low    = ctx->rc.buffer >> (8 - EXTRA_BITS);
    ctx->rc.range  = (uint32_t) 1 << EXTRA_BITS;
}

/** Perform normalization */
static inline void range_dec_normalize(APEContext *ctx)
{
    while (ctx->rc.range <= BOTTOM_VALUE) {
        ctx->rc.buffer <<= 8;
        if(ctx->ptr < ctx->data_end) {
            ctx->rc.buffer += *ctx->ptr;
            ctx->ptr++;
        } else {
            ctx->error = 1;
        }
        ctx->rc.low    = (ctx->rc.low << 8)    | ((ctx->rc.buffer >> 1) & 0xFF);
        ctx->rc.range  <<= 8;
    }
}

/**
 * Calculate cumulative frequency for next symbol. Does NO update!
 * @param ctx decoder context
 * @param tot_f is the total frequency or (code_value)1<<shift
 * @return the cumulative frequency
 */
static inline int range_decode_culfreq(APEContext *ctx, int tot_f)
{
    range_dec_normalize(ctx);
    ctx->rc.help = ctx->rc.range / tot_f;
    return ctx->rc.low / ctx->rc.help;
}

/**
 * Decode value with given size in bits
 * @param ctx decoder context
 * @param shift number of bits to decode
 */
static inline int range_decode_culshift(APEContext *ctx, int shift)
{
    range_dec_normalize(ctx);
    ctx->rc.help = ctx->rc.range >> shift;
    return ctx->rc.low / ctx->rc.help;
}


/**
 * Update decoding state
 * @param ctx decoder context
 * @param sy_f the interval length (frequency of the symbol)
 * @param lt_f the lower end (frequency sum of < symbols)
 */
static inline void range_decode_update(APEContext *ctx, int sy_f, int lt_f)
{
    ctx->rc.low  -= ctx->rc.help * lt_f;
    ctx->rc.range = ctx->rc.help * sy_f;
}

/** Decode n bits (n <= 16) without modelling */
static inline int range_decode_bits(APEContext *ctx, int n)
{
    int sym = range_decode_culshift(ctx, n);
    range_decode_update(ctx, 1, sym);
    return sym;
}


#define MODEL_ELEMENTS 64

/**
 * Fixed probabilities for symbols in Monkey Audio version 3.97
 */
static const uint16_t counts_3970[22] = {
        0, 14824, 28224, 39348, 47855, 53994, 58171, 60926,
    62682, 63786, 64463, 64878, 65126, 65276, 65365, 65419,
    65450, 65469, 65480, 65487, 65491, 65493,
};

/**
 * Probability ranges for symbols in Monkey Audio version 3.97
 */
static const uint16_t counts_diff_3970[21] = {
    14824, 13400, 11124, 8507, 6139, 4177, 2755, 1756,
    1104, 677, 415, 248, 150, 89, 54, 31,
    19, 11, 7, 4, 2,
};

/**
 * Fixed probabilities for symbols in Monkey Audio version 3.98
 */
static const uint16_t counts_3980[22] = {
        0, 19578, 36160, 48417, 56323, 60899, 63265, 64435,
    64971, 65232, 65351, 65416, 65447, 65466, 65476, 65482,
    65485, 65488, 65490, 65491, 65492, 65493,
};

/**
 * Probability ranges for symbols in Monkey Audio version 3.98
 */
static const uint16_t counts_diff_3980[21] = {
    19578, 16582, 12257, 7906, 4576, 2366, 1170, 536,
    261, 119, 65, 31, 19, 10, 6, 3,
    3, 2, 1, 1, 1,
};

/**
 * Decode symbol
 * @param ctx decoder context
 * @param counts probability range start position
 * @param counts_diff probability range widths
 */
static inline int range_get_symbol(APEContext *ctx,
                                   const uint16_t counts[],
                                   const uint16_t counts_diff[])
{
    int symbol, cf;

    cf = range_decode_culshift(ctx, 16);

    if(cf > 65492){
        symbol= cf - 65535 + 63;
        range_decode_update(ctx, 1, cf);
        if(cf > 65535)
            ctx->error=1;
        return symbol;
    }
    /* figure out the symbol inefficiently; a binary search would be much better */
    for (symbol = 0; counts[symbol + 1] <= cf; symbol++);

    range_decode_update(ctx, counts_diff[symbol], counts[symbol]);

    return symbol;
}
/** @} */ // group rangecoder

static inline void update_rice(APERice *rice, unsigned int x)
{
    int lim = rice->k ? (1 << (rice->k + 4)) : 0;
    rice->ksum += ((x + 1) / 2) - ((rice->ksum + 16) >> 5);

    if (rice->ksum < lim)
        rice->k--;
    else if (rice->ksum >= (1 << (rice->k + 5)) && rice->k < 24)
        rice->k++;
}

static inline int get_rice_ook(GetBitContext *gb, int k)
{
    unsigned int x;

    x = get_unary(gb, 1, get_bits_left(gb));

    if (k)
        x = (x << k) | get_bits(gb, k);

    return x;
}

static inline int ape_decode_value_3860(APEContext *ctx, GetBitContext *gb,
                                        APERice *rice)
{
    unsigned int x, overflow;

    overflow = get_unary(gb, 1, get_bits_left(gb));

    if (ctx->fileversion > 3880) {
        while (overflow >= 16) {
            overflow -= 16;
            rice->k  += 4;
        }
    }

    if (!rice->k)
        x = overflow;
    else if(rice->k <= MIN_CACHE_BITS) {
        x = (overflow << rice->k) + get_bits(gb, rice->k);
    } else {
        av_log(ctx->avctx, AV_LOG_ERROR, "Too many bits: %"PRIu32"\n", rice->k);
        ctx->error = 1;
        return AVERROR_INVALIDDATA;
    }
    rice->ksum += x - (rice->ksum + 8 >> 4);
    if (rice->ksum < (rice->k ? 1 << (rice->k + 4) : 0))
        rice->k--;
    else if (rice->ksum >= (1 << (rice->k + 5)) && rice->k < 24)
        rice->k++;

    /* Convert to signed */
    return ((x >> 1) ^ ((x & 1) - 1)) + 1;
}

static inline int ape_decode_value_3900(APEContext *ctx, APERice *rice)
{
    unsigned int x, overflow;
    int tmpk;

    overflow = range_get_symbol(ctx, counts_3970, counts_diff_3970);

    if (overflow == (MODEL_ELEMENTS - 1)) {
        tmpk = range_decode_bits(ctx, 5);
        overflow = 0;
    } else
        tmpk = (rice->k < 1) ? 0 : rice->k - 1;

    if (tmpk <= 16 || ctx->fileversion < 3910) {
        if (tmpk > 23) {
            av_log(ctx->avctx, AV_LOG_ERROR, "Too many bits: %d\n", tmpk);
            return AVERROR_INVALIDDATA;
        }
        x = range_decode_bits(ctx, tmpk);
    } else if (tmpk <= 31) {
        x = range_decode_bits(ctx, 16);
        x |= (range_decode_bits(ctx, tmpk - 16) << 16);
    } else {
        av_log(ctx->avctx, AV_LOG_ERROR, "Too many bits: %d\n", tmpk);
        return AVERROR_INVALIDDATA;
    }
    x += overflow << tmpk;

    update_rice(rice, x);

    /* Convert to signed */
    return ((x >> 1) ^ ((x & 1) - 1)) + 1;
}

static inline int ape_decode_value_3990(APEContext *ctx, APERice *rice)
{
    unsigned int x, overflow, pivot;
    int base;

    pivot = FFMAX(rice->ksum >> 5, 1);

    overflow = range_get_symbol(ctx, counts_3980, counts_diff_3980);

    if (overflow == (MODEL_ELEMENTS - 1)) {
        overflow  = (unsigned)range_decode_bits(ctx, 16) << 16;
        overflow |= range_decode_bits(ctx, 16);
    }

    if (pivot < 0x10000) {
        base = range_decode_culfreq(ctx, pivot);
        range_decode_update(ctx, 1, base);
    } else {
        int base_hi = pivot, base_lo;
        int bbits = 0;

        while (base_hi & ~0xFFFF) {
            base_hi >>= 1;
            bbits++;
        }
        base_hi = range_decode_culfreq(ctx, base_hi + 1);
        range_decode_update(ctx, 1, base_hi);
        base_lo = range_decode_culfreq(ctx, 1 << bbits);
        range_decode_update(ctx, 1, base_lo);

        base = (base_hi << bbits) + base_lo;
    }

    x = base + overflow * pivot;

    update_rice(rice, x);

    /* Convert to signed */
    return ((x >> 1) ^ ((x & 1) - 1)) + 1;
}

static int get_k(int ksum)
{
    return av_log2(ksum) + !!ksum;
}

static void decode_array_0000(APEContext *ctx, GetBitContext *gb,
                              int32_t *out, APERice *rice, int blockstodecode)
{
    int i;
    unsigned ksummax, ksummin;

    rice->ksum = 0;
    for (i = 0; i < FFMIN(blockstodecode, 5); i++) {
        out[i] = get_rice_ook(&ctx->gb, 10);
        rice->ksum += out[i];
    }

    if (blockstodecode <= 5)
        goto end;

    rice->k = get_k(rice->ksum / 10);
    if (rice->k >= 24)
        return;
    for (; i < FFMIN(blockstodecode, 64); i++) {
        out[i] = get_rice_ook(&ctx->gb, rice->k);
        rice->ksum += out[i];
        rice->k = get_k(rice->ksum / ((i + 1) * 2));
        if (rice->k >= 24)
            return;
    }

    if (blockstodecode <= 64)
        goto end;

    rice->k = get_k(rice->ksum >> 7);
    ksummax = 1 << rice->k + 7;
    ksummin = rice->k ? (1 << rice->k + 6) : 0;
    for (; i < blockstodecode; i++) {
        if (get_bits_left(&ctx->gb) < 1) {
            ctx->error = 1;
            return;
        }
        out[i] = get_rice_ook(&ctx->gb, rice->k);
        rice->ksum += out[i] - (unsigned)out[i - 64];
        while (rice->ksum < ksummin) {
            rice->k--;
            ksummin = rice->k ? ksummin >> 1 : 0;
            ksummax >>= 1;
        }
        while (rice->ksum >= ksummax) {
            rice->k++;
            if (rice->k > 24)
                return;
            ksummax <<= 1;
            ksummin = ksummin ? ksummin << 1 : 128;
        }
    }

end:
    for (i = 0; i < blockstodecode; i++)
        out[i] = ((out[i] >> 1) ^ ((out[i] & 1) - 1)) + 1;
}

static void entropy_decode_mono_0000(APEContext *ctx, int blockstodecode)
{
    decode_array_0000(ctx, &ctx->gb, ctx->decoded[0], &ctx->riceY,
                      blockstodecode);
}

static void entropy_decode_stereo_0000(APEContext *ctx, int blockstodecode)
{
    decode_array_0000(ctx, &ctx->gb, ctx->decoded[0], &ctx->riceY,
                      blockstodecode);
    decode_array_0000(ctx, &ctx->gb, ctx->decoded[1], &ctx->riceX,
                      blockstodecode);
}

static void entropy_decode_mono_3860(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceY);
}

static void entropy_decode_stereo_3860(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];
    int blocks = blockstodecode;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceY);
    while (blocks--)
        *decoded1++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceX);
}

static void entropy_decode_mono_3900(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
}

static void entropy_decode_stereo_3900(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];
    int blocks = blockstodecode;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
    range_dec_normalize(ctx);
    // because of some implementation peculiarities we need to backpedal here
    ctx->ptr -= 1;
    range_start_decoding(ctx);
    while (blocks--)
        *decoded1++ = ape_decode_value_3900(ctx, &ctx->riceX);
}

static void entropy_decode_stereo_3930(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];

    while (blockstodecode--) {
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
        *decoded1++ = ape_decode_value_3900(ctx, &ctx->riceX);
    }
}

static void entropy_decode_mono_3990(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3990(ctx, &ctx->riceY);
}

static void entropy_decode_stereo_3990(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];

    while (blockstodecode--) {
        *decoded0++ = ape_decode_value_3990(ctx, &ctx->riceY);
        *decoded1++ = ape_decode_value_3990(ctx, &ctx->riceX);
    }
}

static int init_entropy_decoder(APEContext *ctx)
{
    /* Read the CRC */
    if (ctx->fileversion >= 3900) {
        if (ctx->data_end - ctx->ptr < 6)
            return AVERROR_INVALIDDATA;
        ctx->CRC = bytestream_get_be32(&ctx->ptr);
    } else {
        ctx->CRC = get_bits_long(&ctx->gb, 32);
    }

    /* Read the frame flags if they exist */
    ctx->frameflags = 0;
    ctx->CRC_state = UINT32_MAX;
    if ((ctx->fileversion > 3820) && (ctx->CRC & 0x80000000)) {
        ctx->CRC &= ~0x80000000;

        if (ctx->data_end - ctx->ptr < 6)
            return AVERROR_INVALIDDATA;
        ctx->frameflags = bytestream_get_be32(&ctx->ptr);
    }

    /* Initialize the rice structs */
    ctx->riceX.k = 10;
    ctx->riceX.ksum = (1 << ctx->riceX.k) * 16;
    ctx->riceY.k = 10;
    ctx->riceY.ksum = (1 << ctx->riceY.k) * 16;

    if (ctx->fileversion >= 3900) {
        /* The first 8 bits of input are ignored. */
        ctx->ptr++;

        range_start_decoding(ctx);
    }

    return 0;
}

static const int32_t initial_coeffs_fast_3320[1] = {
    375,
};

static const int32_t initial_coeffs_a_3800[3] = {
    64, 115, 64,
};

static const int32_t initial_coeffs_b_3800[2] = {
    740, 0
};

static const int32_t initial_coeffs_3930[4] = {
    360, 317, -109, 98
};

static const int64_t initial_coeffs_3930_64bit[4] = {
    360, 317, -109, 98
};

static void init_predictor_decoder(APEContext *ctx)
{
    APEPredictor *p = &ctx->predictor;
    APEPredictor64 *p64 = &ctx->predictor64;

    /* Zero the history buffers */
    memset(p->historybuffer, 0, PREDICTOR_SIZE * sizeof(*p->historybuffer));
    memset(p64->historybuffer, 0, PREDICTOR_SIZE * sizeof(*p64->historybuffer));
    p->buf = p->historybuffer;
    p64->buf = p64->historybuffer;

    /* Initialize and zero the coefficients */
    if (ctx->fileversion < 3930) {
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            memcpy(p->coeffsA[0], initial_coeffs_fast_3320,
                   sizeof(initial_coeffs_fast_3320));
            memcpy(p->coeffsA[1], initial_coeffs_fast_3320,
                   sizeof(initial_coeffs_fast_3320));
        } else {
            memcpy(p->coeffsA[0], initial_coeffs_a_3800,
                   sizeof(initial_coeffs_a_3800));
            memcpy(p->coeffsA[1], initial_coeffs_a_3800,
                   sizeof(initial_coeffs_a_3800));
        }
    } else {
        memcpy(p->coeffsA[0], initial_coeffs_3930, sizeof(initial_coeffs_3930));
        memcpy(p->coeffsA[1], initial_coeffs_3930, sizeof(initial_coeffs_3930));
        memcpy(p64->coeffsA[0], initial_coeffs_3930_64bit, sizeof(initial_coeffs_3930_64bit));
        memcpy(p64->coeffsA[1], initial_coeffs_3930_64bit, sizeof(initial_coeffs_3930_64bit));
    }
    memset(p->coeffsB, 0, sizeof(p->coeffsB));
    memset(p64->coeffsB, 0, sizeof(p64->coeffsB));
    if (ctx->fileversion < 3930) {
        memcpy(p->coeffsB[0], initial_coeffs_b_3800,
               sizeof(initial_coeffs_b_3800));
        memcpy(p->coeffsB[1], initial_coeffs_b_3800,
               sizeof(initial_coeffs_b_3800));
    }

    p->filterA[0] = p->filterA[1] = 0;
    p->filterB[0] = p->filterB[1] = 0;
    p->lastA[0]   = p->lastA[1]   = 0;

    p64->filterA[0] = p64->filterA[1] = 0;
    p64->filterB[0] = p64->filterB[1] = 0;
    p64->lastA[0]   = p64->lastA[1]   = 0;

    p->sample_pos = 0;
}

/** Get inverse sign of integer (-1 for positive, 1 for negative and 0 for zero) */
static inline int APESIGN(int32_t x) {
    return (x < 0) - (x > 0);
}

static av_always_inline int filter_fast_3320(APEPredictor *p,
                                             const int decoded, const int filter,
                                             const int delayA)
{
    int32_t predictionA;

    p->buf[delayA] = p->lastA[filter];
    if (p->sample_pos < 3) {
        p->lastA[filter]   = decoded;
        p->filterA[filter] = decoded;
        return decoded;
    }

    predictionA = p->buf[delayA] * 2U - p->buf[delayA - 1];
    p->lastA[filter] = decoded + (unsigned)((int32_t)(predictionA  * p->coeffsA[filter][0]) >> 9);

    if ((decoded ^ predictionA) > 0)
        p->coeffsA[filter][0]++;
    else
        p->coeffsA[filter][0]--;

    p->filterA[filter] += (unsigned)p->lastA[filter];

    return p->filterA[filter];
}

static av_always_inline int filter_3800(APEPredictor *p,
                                        const unsigned decoded, const int filter,
                                        const int delayA,  const int delayB,
                                        const int start,   const int shift)
{
    int32_t predictionA, predictionB, sign;
    int32_t d0, d1, d2, d3, d4;

    p->buf[delayA] = p->lastA[filter];
    p->buf[delayB] = p->filterB[filter];
    if (p->sample_pos < start) {
        predictionA = decoded + p->filterA[filter];
        p->lastA[filter]   = decoded;
        p->filterB[filter] = decoded;
        p->filterA[filter] = predictionA;
        return predictionA;
    }
    d2 =  p->buf[delayA];
    d1 = (p->buf[delayA] - (unsigned)p->buf[delayA - 1]) * 2;
    d0 =  p->buf[delayA] + ((p->buf[delayA - 2] - (unsigned)p->buf[delayA - 1]) * 8);
    d3 =  p->buf[delayB] * 2U - p->buf[delayB - 1];
    d4 =  p->buf[delayB];

    predictionA = d0 * p->coeffsA[filter][0] +
                  d1 * p->coeffsA[filter][1] +
                  d2 * p->coeffsA[filter][2];

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += (((d0 >> 30) & 2) - 1) * sign;
    p->coeffsA[filter][1] += (((d1 >> 28) & 8) - 4) * sign;
    p->coeffsA[filter][2] += (((d2 >> 28) & 8) - 4) * sign;

    predictionB = d3 * p->coeffsB[filter][0] -
                  d4 * p->coeffsB[filter][1];
    p->lastA[filter] = decoded + (predictionA >> 11);
    sign = APESIGN(p->lastA[filter]);
    p->coeffsB[filter][0] += (((d3 >> 29) & 4) - 2) * sign;
    p->coeffsB[filter][1] -= (((d4 >> 30) & 2) - 1) * sign;

    p->filterB[filter] = p->lastA[filter] + (unsigned)(predictionB >> shift);
    p->filterA[filter] = p->filterB[filter] + (unsigned)((int)(p->filterA[filter] * 31U) >> 5);

    return p->filterA[filter];
}

static void long_filter_high_3800(int32_t *buffer, int order, int shift, int length)
{
    int i, j;
    int32_t dotprod, sign;
    int32_t coeffs[256], delay[256+256], *delayp = delay;

    if (order >= length)
        return;

    memset(coeffs, 0, order * sizeof(*coeffs));
    for (i = 0; i < order; i++)
        delay[i] = buffer[i];
    for (i = order; i < length; i++) {
        dotprod = 0;
        sign = APESIGN(buffer[i]);
        if (sign == 1) {
            for (j = 0; j < order; j++) {
                dotprod += delayp[j] * (unsigned)coeffs[j];
                coeffs[j] += (delayp[j] >> 31) | 1;
            }
        } else if (sign == -1) {
            for (j = 0; j < order; j++) {
                dotprod += delayp[j] * (unsigned)coeffs[j];
                coeffs[j] -= (delayp[j] >> 31) | 1;
            }
        } else {
            for (j = 0; j < order; j++) {
                dotprod += delayp[j] * (unsigned)coeffs[j];
            }
        }
        buffer[i] -= (unsigned)(dotprod >> shift);
        delayp ++;
        delayp[order - 1] = buffer[i];
        if (delayp - delay == 256) {
            memcpy(delay, delayp, sizeof(*delay)*256);
            delayp = delay;
        }
    }
}

static void long_filter_ehigh_3830(int32_t *buffer, int length)
{
    int i, j;
    int32_t dotprod, sign;
    int32_t delay[8] = { 0 };
    uint32_t coeffs[8] = { 0 };

    for (i = 0; i < length; i++) {
        dotprod = 0;
        sign = APESIGN(buffer[i]);
        for (j = 7; j >= 0; j--) {
            dotprod += delay[j] * coeffs[j];
            coeffs[j] += ((delay[j] >> 31) | 1) * sign;
        }
        for (j = 7; j > 0; j--)
            delay[j] = delay[j - 1];
        delay[0] = buffer[i];
        buffer[i] -= (unsigned)(dotprod >> 9);
    }
}

static void predictor_decode_stereo_3800(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];
    int start = 4, shift = 10;

    if (ctx->compression_level == COMPRESSION_LEVEL_HIGH) {
        start = 16;
        long_filter_high_3800(decoded0, 16, 9, count);
        long_filter_high_3800(decoded1, 16, 9, count);
    } else if (ctx->compression_level == COMPRESSION_LEVEL_EXTRA_HIGH) {
        int order = 128, shift2 = 11;

        if (ctx->fileversion >= 3830) {
            order <<= 1;
            shift++;
            shift2++;
            long_filter_ehigh_3830(decoded0 + order, count - order);
            long_filter_ehigh_3830(decoded1 + order, count - order);
        }
        start = order;
        long_filter_high_3800(decoded0, order, shift2, count);
        long_filter_high_3800(decoded1, order, shift2, count);
    }

    while (count--) {
        int X = *decoded0, Y = *decoded1;
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            *decoded0 = filter_fast_3320(p, Y, 0, YDELAYA);
            decoded0++;
            *decoded1 = filter_fast_3320(p, X, 1, XDELAYA);
            decoded1++;
        } else {
            *decoded0 = filter_3800(p, Y, 0, YDELAYA, YDELAYB,
                                    start, shift);
            decoded0++;
            *decoded1 = filter_3800(p, X, 1, XDELAYA, XDELAYB,
                                    start, shift);
            decoded1++;
        }

        /* Combined */
        p->buf++;
        p->sample_pos++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static void predictor_decode_mono_3800(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded[0];
    int start = 4, shift = 10;

    if (ctx->compression_level == COMPRESSION_LEVEL_HIGH) {
        start = 16;
        long_filter_high_3800(decoded0, 16, 9, count);
    } else if (ctx->compression_level == COMPRESSION_LEVEL_EXTRA_HIGH) {
        int order = 128, shift2 = 11;

        if (ctx->fileversion >= 3830) {
            order <<= 1;
            shift++;
            shift2++;
            long_filter_ehigh_3830(decoded0 + order, count - order);
        }
        start = order;
        long_filter_high_3800(decoded0, order, shift2, count);
    }

    while (count--) {
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            *decoded0 = filter_fast_3320(p, *decoded0, 0, YDELAYA);
            decoded0++;
        } else {
            *decoded0 = filter_3800(p, *decoded0, 0, YDELAYA, YDELAYB,
                                    start, shift);
            decoded0++;
        }

        /* Combined */
        p->buf++;
        p->sample_pos++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static av_always_inline int predictor_update_3930(APEPredictor *p,
                                                  const int decoded, const int filter,
                                                  const int delayA)
{
    int32_t predictionA, sign;
    uint32_t d0, d1, d2, d3;

    p->buf[delayA]     = p->lastA[filter];
    d0 = p->buf[delayA    ];
    d1 = p->buf[delayA    ] - (unsigned)p->buf[delayA - 1];
    d2 = p->buf[delayA - 1] - (unsigned)p->buf[delayA - 2];
    d3 = p->buf[delayA - 2] - (unsigned)p->buf[delayA - 3];

    predictionA = d0 * p->coeffsA[filter][0] +
                  d1 * p->coeffsA[filter][1] +
                  d2 * p->coeffsA[filter][2] +
                  d3 * p->coeffsA[filter][3];

    p->lastA[filter] = decoded + (predictionA >> 9);
    p->filterA[filter] = p->lastA[filter] + ((int)(p->filterA[filter] * 31U) >> 5);

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += (((int32_t)d0 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][1] += (((int32_t)d1 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][2] += (((int32_t)d2 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][3] += (((int32_t)d3 < 0) * 2 - 1) * sign;

    return p->filterA[filter];
}

static void predictor_decode_stereo_3930(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];

    ape_apply_filters(ctx, ctx->decoded[0], ctx->decoded[1], count);

    while (count--) {
        /* Predictor Y */
        int Y = *decoded1, X = *decoded0;
        *decoded0 = predictor_update_3930(p, Y, 0, YDELAYA);
        decoded0++;
        *decoded1 = predictor_update_3930(p, X, 1, XDELAYA);
        decoded1++;

        /* Combined */
        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static void predictor_decode_mono_3930(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded[0];

    ape_apply_filters(ctx, ctx->decoded[0], NULL, count);

    while (count--) {
        *decoded0 = predictor_update_3930(p, *decoded0, 0, YDELAYA);
        decoded0++;

        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static av_always_inline int predictor_update_filter(APEPredictor64 *p,
                                                    const int decoded, const int filter,
                                                    const int delayA,  const int delayB,
                                                    const int adaptA,  const int adaptB,
                                                    int interim_mode)
{
    int64_t predictionA, predictionB;
    int32_t sign;

    p->buf[delayA]     = p->lastA[filter];
    p->buf[adaptA]     = APESIGN(p->buf[delayA]);
    p->buf[delayA - 1] = p->buf[delayA] - (uint64_t)p->buf[delayA - 1];
    p->buf[adaptA - 1] = APESIGN(p->buf[delayA - 1]);

    predictionA = p->buf[delayA    ] * p->coeffsA[filter][0] +
                  p->buf[delayA - 1] * p->coeffsA[filter][1] +
                  p->buf[delayA - 2] * p->coeffsA[filter][2] +
                  p->buf[delayA - 3] * p->coeffsA[filter][3];

    /*  Apply a scaled first-order filter compression */
    p->buf[delayB]     = p->filterA[filter ^ 1] - ((int64_t)(p->filterB[filter] * 31ULL) >> 5);
    p->buf[adaptB]     = APESIGN(p->buf[delayB]);
    p->buf[delayB - 1] = p->buf[delayB] - (uint64_t)p->buf[delayB - 1];
    p->buf[adaptB - 1] = APESIGN(p->buf[delayB - 1]);
    p->filterB[filter] = p->filterA[filter ^ 1];

    predictionB = p->buf[delayB    ] * p->coeffsB[filter][0] +
                  p->buf[delayB - 1] * p->coeffsB[filter][1] +
                  p->buf[delayB - 2] * p->coeffsB[filter][2] +
                  p->buf[delayB - 3] * p->coeffsB[filter][3] +
                  p->buf[delayB - 4] * p->coeffsB[filter][4];

    if (interim_mode < 1) {
        predictionA = (int32_t)predictionA;
        predictionB = (int32_t)predictionB;
        p->lastA[filter] = (int32_t)(decoded + (unsigned)((int32_t)(predictionA + (predictionB >> 1)) >> 10));
    } else {
        p->lastA[filter] = decoded + ((int64_t)((uint64_t)predictionA + (predictionB >> 1)) >> 10);
    }
    p->filterA[filter] = p->lastA[filter] + ((int64_t)(p->filterA[filter] * 31ULL) >> 5);

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += p->buf[adaptA    ] * sign;
    p->coeffsA[filter][1] += p->buf[adaptA - 1] * sign;
    p->coeffsA[filter][2] += p->buf[adaptA - 2] * sign;
    p->coeffsA[filter][3] += p->buf[adaptA - 3] * sign;
    p->coeffsB[filter][0] += p->buf[adaptB    ] * sign;
    p->coeffsB[filter][1] += p->buf[adaptB - 1] * sign;
    p->coeffsB[filter][2] += p->buf[adaptB - 2] * sign;
    p->coeffsB[filter][3] += p->buf[adaptB - 3] * sign;
    p->coeffsB[filter][4] += p->buf[adaptB - 4] * sign;

    return p->filterA[filter];
}

static void predictor_decode_stereo_3950(APEContext *ctx, int count)
{
    APEPredictor64 *p_default = &ctx->predictor64;
    APEPredictor64 p_interim;
    int lcount = count;
    int num_passes = 1;

    ape_apply_filters(ctx, ctx->decoded[0], ctx->decoded[1], count);
    if (ctx->interim_mode == -1) {
        p_interim = *p_default;
        num_passes ++;
        memcpy(ctx->interim[0], ctx->decoded[0], sizeof(*ctx->interim[0])*count);
        memcpy(ctx->interim[1], ctx->decoded[1], sizeof(*ctx->interim[1])*count);
    }

    for (int pass = 0; pass < num_passes; pass++) {
        int32_t *decoded0, *decoded1;
        int interim_mode = ctx->interim_mode > 0 || pass;
        APEPredictor64 *p;

        if (pass) {
            p        = &p_interim;
            decoded0 = ctx->interim[0];
            decoded1 = ctx->interim[1];
        } else {
            p        = p_default;
            decoded0 = ctx->decoded[0];
            decoded1 = ctx->decoded[1];
        }
        p->buf = p->historybuffer;

        count = lcount;
        while (count--) {
            /* Predictor Y */
            int32_t a0 = predictor_update_filter(p, *decoded0, 0, YDELAYA, YDELAYB,
                                                YADAPTCOEFFSA, YADAPTCOEFFSB,
                                                interim_mode);
            int32_t a1 = predictor_update_filter(p, *decoded1, 1, XDELAYA, XDELAYB,
                                                XADAPTCOEFFSA, XADAPTCOEFFSB,
                                                interim_mode);
            *decoded0++ = a0;
            *decoded1++ = a1;
            if (num_passes > 1) {
                int32_t left  = a1 - (unsigned)(a0 / 2);
                int32_t right = left + (unsigned)a0;

                if (FFMIN(FFNABS(left), FFNABS(right)) < -(1<<23)) {
                    ctx->interim_mode = !interim_mode;
                    av_log(ctx->avctx, AV_LOG_VERBOSE, "Interim mode: %d\n", ctx->interim_mode);
                    break;
                }
            }

            /* Combined */
            p->buf++;

            /* Have we filled the history buffer? */
            if (p->buf == p->historybuffer + HISTORY_SIZE) {
                memmove(p->historybuffer, p->buf,
                        PREDICTOR_SIZE * sizeof(*p->historybuffer));
                p->buf = p->historybuffer;
            }
        }
    }
    if (num_passes > 1 && ctx->interim_mode > 0) {
        memcpy(ctx->decoded[0], ctx->interim[0], sizeof(*ctx->interim[0])*lcount);
        memcpy(ctx->decoded[1], ctx->interim[1], sizeof(*ctx->interim[1])*lcount);
        *p_default = p_interim;
        p_default->buf = p_default->historybuffer;
    }
}

static void predictor_decode_mono_3950(APEContext *ctx, int count)
{
    APEPredictor64 *p = &ctx->predictor64;
    int32_t *decoded0 = ctx->decoded[0];
    int32_t predictionA, currentA, A, sign;

    ape_apply_filters(ctx, ctx->decoded[0], NULL, count);

    currentA = p->lastA[0];

    while (count--) {
        A = *decoded0;

        p->buf[YDELAYA] = currentA;
        p->buf[YDELAYA - 1] = p->buf[YDELAYA] - (uint64_t)p->buf[YDELAYA - 1];

        predictionA = p->buf[YDELAYA    ] * p->coeffsA[0][0] +
                      p->buf[YDELAYA - 1] * p->coeffsA[0][1] +
                      p->buf[YDELAYA - 2] * p->coeffsA[0][2] +
                      p->buf[YDELAYA - 3] * p->coeffsA[0][3];

        currentA = A + (uint64_t)(predictionA >> 10);

        p->buf[YADAPTCOEFFSA]     = APESIGN(p->buf[YDELAYA    ]);
        p->buf[YADAPTCOEFFSA - 1] = APESIGN(p->buf[YDELAYA - 1]);

        sign = APESIGN(A);
        p->coeffsA[0][0] += p->buf[YADAPTCOEFFSA    ] * sign;
        p->coeffsA[0][1] += p->buf[YADAPTCOEFFSA - 1] * sign;
        p->coeffsA[0][2] += p->buf[YADAPTCOEFFSA - 2] * sign;
        p->coeffsA[0][3] += p->buf[YADAPTCOEFFSA - 3] * sign;

        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }

        p->filterA[0] = currentA + (uint64_t)((int64_t)(p->filterA[0] * 31U) >> 5);
        *(decoded0++) = p->filterA[0];
    }

    p->lastA[0] = currentA;
}

static void do_init_filter(APEFilter *f, int16_t *buf, int order)
{
    f->coeffs = buf;
    f->historybuffer = buf + order;
    f->delay       = f->historybuffer + order * 2;
    f->adaptcoeffs = f->historybuffer + order;

    memset(f->historybuffer, 0, (order * 2) * sizeof(*f->historybuffer));
    memset(f->coeffs, 0, order * sizeof(*f->coeffs));
    f->avg = 0;
}

static void init_filter(APEContext *ctx, APEFilter *f, int16_t *buf, int order)
{
    do_init_filter(&f[0], buf, order);
    do_init_filter(&f[1], buf + order * 3 + HISTORY_SIZE, order);
}

static void do_apply_filter(APEContext *ctx, int version, APEFilter *f,
                            int32_t *data, int count, int order, int fracbits)
{
    int res;
    unsigned absres;

    while (count--) {
        /* round fixedpoint scalar product */
        res = ctx->adsp.scalarproduct_and_madd_int16(f->coeffs,
                                                     f->delay - order,
                                                     f->adaptcoeffs - order,
                                                     order, APESIGN(*data));
        res = (int64_t)(res + (1LL << (fracbits - 1))) >> fracbits;
        res += (unsigned)*data;
        *data++ = res;

        /* Update the output history */
        *f->delay++ = av_clip_int16(res);

        if (version < 3980) {
            /* Version ??? to < 3.98 files (untested) */
            f->adaptcoeffs[0]  = (res == 0) ? 0 : ((res >> 28) & 8) - 4;
            f->adaptcoeffs[-4] >>= 1;
            f->adaptcoeffs[-8] >>= 1;
        } else {
            /* Version 3.98 and later files */

            /* Update the adaption coefficients */
            absres = FFABSU(res);
            if (absres)
                *f->adaptcoeffs = APESIGN(res) *
                                  (8 << ((absres > f->avg * 3LL) + (absres > (f->avg + f->avg / 3))));
                /* equivalent to the following code
                    if (absres <= f->avg * 4 / 3)
                        *f->adaptcoeffs = APESIGN(res) * 8;
                    else if (absres <= f->avg * 3)
                        *f->adaptcoeffs = APESIGN(res) * 16;
                    else
                        *f->adaptcoeffs = APESIGN(res) * 32;
                */
            else
                *f->adaptcoeffs = 0;

            f->avg += (int)(absres - (unsigned)f->avg) / 16;

            f->adaptcoeffs[-1] >>= 1;
            f->adaptcoeffs[-2] >>= 1;
            f->adaptcoeffs[-8] >>= 1;
        }

        f->adaptcoeffs++;

        /* Have we filled the history buffer? */
        if (f->delay == f->historybuffer + HISTORY_SIZE + (order * 2)) {
            memmove(f->historybuffer, f->delay - (order * 2),
                    (order * 2) * sizeof(*f->historybuffer));
            f->delay = f->historybuffer + order * 2;
            f->adaptcoeffs = f->historybuffer + order;
        }
    }
}

static void apply_filter(APEContext *ctx, APEFilter *f,
                         int32_t *data0, int32_t *data1,
                         int count, int order, int fracbits)
{
    do_apply_filter(ctx, ctx->fileversion, &f[0], data0, count, order, fracbits);
    if (data1)
        do_apply_filter(ctx, ctx->fileversion, &f[1], data1, count, order, fracbits);
}

static void ape_apply_filters(APEContext *ctx, int32_t *decoded0,
                              int32_t *decoded1, int count)
{
    int i;

    for (i = 0; i < APE_FILTER_LEVELS; i++) {
        if (!ape_filter_orders[ctx->fset][i])
            break;
        apply_filter(ctx, ctx->filters[i], decoded0, decoded1, count,
                     ape_filter_orders[ctx->fset][i],
                     ape_filter_fracbits[ctx->fset][i]);
    }
}

static int init_frame_decoder(APEContext *ctx)
{
    int i, ret;
    if ((ret = init_entropy_decoder(ctx)) < 0)
        return ret;
    init_predictor_decoder(ctx);

    for (i = 0; i < APE_FILTER_LEVELS; i++) {
        if (!ape_filter_orders[ctx->fset][i])
            break;
        init_filter(ctx, ctx->filters[i], ctx->filterbuf[i],
                    ape_filter_orders[ctx->fset][i]);
    }
    return 0;
}

static void ape_unpack_mono(APEContext *ctx, int count)
{
    if (ctx->frameflags & APE_FRAMECODE_STEREO_SILENCE) {
        /* We are pure silence, so we're done. */
        av_log(ctx->avctx, AV_LOG_DEBUG, "pure silence mono\n");
        return;
    }

    ctx->entropy_decode_mono(ctx, count);
    if (ctx->error)
        return;

    /* Now apply the predictor decoding */
    ctx->predictor_decode_mono(ctx, count);

    /* Pseudo-stereo - just copy left channel to right channel */
    if (ctx->channels == 2) {
        memcpy(ctx->decoded[1], ctx->decoded[0], count * sizeof(*ctx->decoded[1]));
    }
}

static void ape_unpack_stereo(APEContext *ctx, int count)
{
    unsigned left, right;
    int32_t *decoded0 = ctx->decoded[0];
    int32_t *decoded1 = ctx->decoded[1];

    if ((ctx->frameflags & APE_FRAMECODE_STEREO_SILENCE) == APE_FRAMECODE_STEREO_SILENCE) {
        /* We are pure silence, so we're done. */
        av_log(ctx->avctx, AV_LOG_DEBUG, "pure silence stereo\n");
        return;
    }

    ctx->entropy_decode_stereo(ctx, count);
    if (ctx->error)
        return;

    /* Now apply the predictor decoding */
    ctx->predictor_decode_stereo(ctx, count);

    /* Decorrelate and scale to output depth */
    while (count--) {
        left = *decoded1 - (unsigned)(*decoded0 / 2);
        right = left + *decoded0;

        *(decoded0++) = left;
        *(decoded1++) = right;
    }
}

static int ape_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    APEContext *s = avctx->priv_data;
    uint8_t *sample8;
    int16_t *sample16;
    int32_t *sample24;
    int i, ch, ret;
    int blockstodecode;
    uint64_t decoded_buffer_size;

    /* this should never be negative, but bad things will happen if it is, so
       check it just to make sure. */
    av_assert0(s->samples >= 0);

    if(!s->samples){
        uint32_t nblocks, offset;
        int buf_size;

        if (!avpkt->size) {
            *got_frame_ptr = 0;
            return 0;
        }
        if (avpkt->size < 8) {
            av_log(avctx, AV_LOG_ERROR, "Packet is too small\n");
            return AVERROR_INVALIDDATA;
        }
        buf_size = avpkt->size & ~3;
        if (buf_size != avpkt->size) {
            av_log(avctx, AV_LOG_WARNING, "packet size is not a multiple of 4. "
                   "extra bytes at the end will be skipped.\n");
        }
        if (s->fileversion < 3950) // previous versions overread two bytes
            buf_size += 2;
        av_fast_padded_malloc(&s->data, &s->data_size, buf_size);
        if (!s->data)
            return AVERROR(ENOMEM);
        s->bdsp.bswap_buf((uint32_t *) s->data, (const uint32_t *) buf,
                          buf_size >> 2);
        memset(s->data + (buf_size & ~3), 0, buf_size & 3);
        s->ptr = s->data;
        s->data_end = s->data + buf_size;

        nblocks = bytestream_get_be32(&s->ptr);
        offset  = bytestream_get_be32(&s->ptr);
        if (s->fileversion >= 3900) {
            if (offset > 3) {
                av_log(avctx, AV_LOG_ERROR, "Incorrect offset passed\n");
                av_freep(&s->data);
                s->data_size = 0;
                return AVERROR_INVALIDDATA;
            }
            if (s->data_end - s->ptr < offset) {
                av_log(avctx, AV_LOG_ERROR, "Packet is too small\n");
                return AVERROR_INVALIDDATA;
            }
            s->ptr += offset;
        } else {
            if ((ret = init_get_bits8(&s->gb, s->ptr, s->data_end - s->ptr)) < 0)
                return ret;
            if (s->fileversion > 3800)
                skip_bits_long(&s->gb, offset * 8);
            else
                skip_bits_long(&s->gb, offset);
        }

        if (!nblocks || nblocks > INT_MAX / 2 / sizeof(*s->decoded_buffer) - 8) {
            av_log(avctx, AV_LOG_ERROR, "Invalid sample count: %"PRIu32".\n",
                   nblocks);
            return AVERROR_INVALIDDATA;
        }

        /* Initialize the frame decoder */
        if (init_frame_decoder(s) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error reading frame header\n");
            return AVERROR_INVALIDDATA;
        }
        s->samples = nblocks;
    }

    if (!s->data) {
        *got_frame_ptr = 0;
        return avpkt->size;
    }

    blockstodecode = FFMIN(s->blocks_per_loop, s->samples);
    // for old files coefficients were not interleaved,
    // so we need to decode all of them at once
    if (s->fileversion < 3930)
        blockstodecode = s->samples;

    /* reallocate decoded sample buffer if needed */
    decoded_buffer_size = 2LL * FFALIGN(blockstodecode, 8) * sizeof(*s->decoded_buffer);
    av_assert0(decoded_buffer_size <= INT_MAX);

    /* get output buffer */
    frame->nb_samples = blockstodecode;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0) {
        s->samples=0;
        return ret;
    }

    av_fast_malloc(&s->decoded_buffer, &s->decoded_size, decoded_buffer_size);
    if (!s->decoded_buffer)
        return AVERROR(ENOMEM);
    memset(s->decoded_buffer, 0, decoded_buffer_size);
    s->decoded[0] = s->decoded_buffer;
    s->decoded[1] = s->decoded_buffer + FFALIGN(blockstodecode, 8);

    if (s->interim_mode < 0) {
        av_fast_malloc(&s->interim_buffer, &s->interim_size, decoded_buffer_size);
        if (!s->interim_buffer)
            return AVERROR(ENOMEM);
        memset(s->interim_buffer, 0, decoded_buffer_size);
        s->interim[0] = s->interim_buffer;
        s->interim[1] = s->interim_buffer + FFALIGN(blockstodecode, 8);
    } else {
        av_freep(&s->interim_buffer);
        s->interim_size = 0;
        memset(s->interim, 0, sizeof(s->interim));
    }

    s->error=0;

    if ((s->channels == 1) || (s->frameflags & APE_FRAMECODE_PSEUDO_STEREO))
        ape_unpack_mono(s, blockstodecode);
    else
        ape_unpack_stereo(s, blockstodecode);

    if (s->error) {
        s->samples=0;
        av_log(avctx, AV_LOG_ERROR, "Error decoding frame\n");
        return AVERROR_INVALIDDATA;
    }

    switch (s->bps) {
    case 8:
        for (ch = 0; ch < s->channels; ch++) {
            sample8 = (uint8_t *)frame->data[ch];
            for (i = 0; i < blockstodecode; i++)
                *sample8++ = (s->decoded[ch][i] + 0x80U) & 0xff;
        }
        break;
    case 16:
        for (ch = 0; ch < s->channels; ch++) {
            sample16 = (int16_t *)frame->data[ch];
            for (i = 0; i < blockstodecode; i++)
                *sample16++ = s->decoded[ch][i];
        }
        break;
    case 24:
        for (ch = 0; ch < s->channels; ch++) {
            sample24 = (int32_t *)frame->data[ch];
            for (i = 0; i < blockstodecode; i++)
                *sample24++ = s->decoded[ch][i] * 256U;
        }
        break;
    }

    s->samples -= blockstodecode;

    if (avctx->err_recognition & AV_EF_CRCCHECK &&
        s->fileversion >= 3900) {
        uint32_t crc = s->CRC_state;
        const AVCRC *crc_tab = av_crc_get_table(AV_CRC_32_IEEE_LE);
        int stride = s->bps == 24 ? 4 : (s->bps>>3);
        int offset = s->bps == 24;
        int bytes  = s->bps >> 3;

        for (i = 0; i < blockstodecode; i++) {
            for (ch = 0; ch < s->channels; ch++) {
#if HAVE_BIGENDIAN
                uint8_t *smp_native = frame->data[ch] + i*stride;
                uint8_t smp[4];
                for(int j = 0; j<stride; j++)
                    smp[j] = smp_native[stride-j-1];
#else
                uint8_t *smp = frame->data[ch] + i*stride;
#endif
                crc = av_crc(crc_tab, crc, smp+offset, bytes);
            }
        }

        if (!s->samples && (~crc >> 1) ^ s->CRC) {
            av_log(avctx, AV_LOG_ERROR, "CRC mismatch! Previously decoded "
                   "frames may have been affected as well.\n");
            if (avctx->err_recognition & AV_EF_EXPLODE)
                return AVERROR_INVALIDDATA;
        }

        s->CRC_state = crc;
    }

    *got_frame_ptr = 1;

    return !s->samples ? avpkt->size : 0;
}

static void ape_flush(AVCodecContext *avctx)
{
    APEContext *s = avctx->priv_data;
    s->samples= 0;
}

#define OFFSET(x) offsetof(APEContext, x)
#define PAR (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_AUDIO_PARAM)
static const AVOption options[] = {
    { "max_samples", "maximum number of samples decoded per call",             OFFSET(blocks_per_loop), AV_OPT_TYPE_INT,   { .i64 = 4608 },    1,       INT_MAX, PAR, .unit = "max_samples" },
    { "all",         "no maximum. decode all samples for each packet at once", 0,                       AV_OPT_TYPE_CONST, { .i64 = INT_MAX }, INT_MIN, INT_MAX, PAR, .unit = "max_samples" },
    { NULL},
};

static const AVClass ape_decoder_class = {
    .class_name = "APE decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_ape_decoder = {
    .p.name         = "ape",
    CODEC_LONG_NAME("Monkey's Audio"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_APE,
    .priv_data_size = sizeof(APEContext),
    .init           = ape_decode_init,
    .close          = ape_decode_close,
    FF_CODEC_DECODE_CB(ape_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DELAY |
                      AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .flush          = ape_flush,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_U8P, AV_SAMPLE_FMT_S16P, AV_SAMPLE_FMT_S32P),
    .p.priv_class   = &ape_decoder_class,
};
