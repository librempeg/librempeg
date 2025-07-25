/*
 * MLP decoder
 * Copyright (c) 2007-2008 Ian Caulfield
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
 * MLP decoder
 */

#include "config_components.h"

#include <stdint.h>

#include "avcodec.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem_internal.h"
#include "libavutil/thread.h"
#include "libavutil/opt.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "mlp_parse.h"
#include "mlpdsp.h"
#include "mlp.h"
#include "config.h"
#include "profiles.h"

/** number of bits used for VLC lookup - longest Huffman code is 9 */
#if ARCH_ARM
#define VLC_BITS            5
#define VLC_STATIC_SIZE     64
#else
#define VLC_BITS            9
#define VLC_STATIC_SIZE     512
#endif

typedef struct SubStream {
    /// Set if a valid restart header has been read. Otherwise the substream cannot be decoded.
    uint8_t     restart_seen;
    /// Set if end of stream is encountered
    uint8_t     end_of_stream;

    //@{
    /** restart header data */
    /// The type of substream given by the restart header sync word.
    uint16_t    substream_type;

    /// The index of the first channel coded in this substream.
    uint8_t     min_channel;
    /// The index of the last channel coded in this substream.
    uint8_t     max_channel;
    /// The coded channels mask in this substream.
    uint64_t    coded_channels;
    /// The number of channels input into the rematrix stage.
    uint8_t     max_matrix_channel;
    /// For each channel output by the matrix, the output channel to map it to
    uint8_t     ch_assign[MAX_CHANNELS];
    /// The channel layout for this substream
    uint64_t    mask;
    /// The matrix encoding mode for this substream
    enum AVMatrixEncoding matrix_encoding;
    enum AVMatrixEncoding prev_matrix_encoding;

    /// Channel coding parameters for channels in the substream
    ChannelParams channel_params[MAX_CHANNELS];

    /// The left shift applied to random noise in 0x31ea substreams.
    uint8_t     noise_shift;
    /// The current seed value for the pseudorandom noise generator(s).
    uint32_t    noisegen_seed;

    /// Maximum output_shift value.
    int8_t      max_shift;
    /// Maximum size of coded audio samples LSBs part.
    int8_t      max_lsbs;
    /// Maximum bit-depth of output audio samples.
    int8_t      max_bits;

    /// Set if the substream contains extra info to check the size of VLC blocks.
    uint8_t     data_check_present;

    /// Bitmask of which parameter sets are conveyed in a decoding parameter block.
    uint8_t     param_presence_flags;
    //@}

    //@{
    /** matrix data */

    /// Number of matrices to be applied.
    uint8_t     num_primitive_matrices;

    /// matrix output channel
    uint8_t     matrix_out_ch[MAX_MATRICES];

    /// Size of the LSBs of the matrix output encoded in the bitstream.
    uint8_t     lsb_bypass[MAX_MATRICES];
    /// Matrix coefficients fractional part size in bits.
    uint8_t     matrix_coeff_frac_bits[MAX_MATRICES];
    /// Matrix coefficients shift amount.
    int8_t      matrix_coeff_shift[MAX_MATRICES];
    /// Matrix coefficients presence mask.
    uint16_t    matrix_coeff_mask[MAX_MATRICES];
    /// Matrix coefficients, stored as 2.18 fixed point.
    DECLARE_ALIGNED(32, int32_t, matrix_coeff)[MAX_MATRICES][MAX_CHANNELS];
    /// Delta matrix coefficients size in bits for 0x31ec substreams.
    uint8_t     delta_matrix_coeff_bits[MAX_MATRICES];
    /// Delta matrix coefficients precision.
    uint8_t     delta_matrix_coeff_prec[MAX_MATRICES];
    /// Delta matrix coefficients, stored as 2.18 fixed point.
    DECLARE_ALIGNED(32, int32_t, delta_matrix_coeff)[MAX_MATRICES][MAX_CHANNELS];
    /// Left shift to apply to noise values in 0x31eb and 0x31ec substreams.
    uint8_t     matrix_noise_shift[MAX_MATRICES];
    //@}

    /// Left shift to apply to Huffman-decoded residuals.
    uint8_t     quant_step_size[MAX_CHANNELS];

    /// number of PCM samples in current audio block
    uint16_t    blocksize;
    /// Number of PCM samples decoded so far in this frame.
    uint16_t    blockpos;

    /// Left shift to apply to decoded PCM values to get final 24-bit output.
    int8_t      output_shift[MAX_CHANNELS];

    /// Running XOR of all output samples.
    int32_t     lossless_check_data;

} SubStream;

typedef struct MLPDecodeContext {
    const AVClass  *class;
    AVCodecContext *avctx;

    AVChannelLayout downmix_layout;
    /// Set to enable decoding of non-loudspeaker feed (objects) audio channels
    int extract_objects;

    /// Current access unit being read has a major sync.
    int         is_major_sync_unit;

    /// Size of the major sync unit, in bytes
    int         major_sync_header_size;

    /// Set if a valid major sync block has been read. Otherwise no decoding is possible.
    uint8_t     params_valid;

    /// Number of substreams contained within this stream.
    uint8_t     num_substreams;

    /// Which substream of substreams carry 16-channel presentation
    uint8_t     extended_substream_info;

    /// Which substream of substreams carry 2/6/8-channel presentation
    uint8_t     substream_info;

    /// Index of the last substream to decode - further substreams are skipped.
    uint8_t     max_decoded_substream;

    /// Stream needs channel reordering to comply with FFmpeg's channel order
    uint8_t     needs_reordering;

    /// number of PCM samples contained in each frame
    int         access_unit_size;
    /// next power of two above the number of samples in each frame
    int         access_unit_size_pow2;

    SubStream   substream[MAX_SUBSTREAMS];

    int         matrix_changed;
    int         filter_changed[MAX_CHANNELS][NUM_FILTERS];

    int8_t      noise_buffer[MAX_BLOCKSIZE_POW2];
    int8_t      bypassed_lsbs[MAX_BLOCKSIZE][MAX_CHANNELS];
    DECLARE_ALIGNED(32, int32_t, sample_buffer)[MAX_BLOCKSIZE][MAX_CHANNELS];

    MLPDSPContext dsp;
    int32_t (*pack_output)(int32_t lossless_check_data,
                           uint16_t blockpos,
                           int32_t (*sample_buffer)[MAX_CHANNELS],
                           void *data,
                           uint8_t *ch_assign,
                           int8_t *output_shift,
                           uint8_t max_matrix_channel,
                           int is32);
} MLPDecodeContext;

static const enum AVChannel thd_channel_order[] = {
    AV_CHAN_FRONT_LEFT, AV_CHAN_FRONT_RIGHT,                     // LR
    AV_CHAN_FRONT_CENTER,                                        // C
    AV_CHAN_LOW_FREQUENCY,                                       // LFE
    AV_CHAN_SIDE_LEFT, AV_CHAN_SIDE_RIGHT,                       // LRs
    AV_CHAN_TOP_FRONT_LEFT, AV_CHAN_TOP_FRONT_RIGHT,             // LRvh
    AV_CHAN_FRONT_LEFT_OF_CENTER, AV_CHAN_FRONT_RIGHT_OF_CENTER, // LRc
    AV_CHAN_BACK_LEFT, AV_CHAN_BACK_RIGHT,                       // LRrs
    AV_CHAN_BACK_CENTER,                                         // Cs
    AV_CHAN_TOP_CENTER,                                          // Ts
    AV_CHAN_SURROUND_DIRECT_LEFT, AV_CHAN_SURROUND_DIRECT_RIGHT, // LRsd
    AV_CHAN_WIDE_LEFT, AV_CHAN_WIDE_RIGHT,                       // LRw
    AV_CHAN_TOP_FRONT_CENTER,                                    // Cvh
    AV_CHAN_LOW_FREQUENCY_2,                                     // LFE2
};

static int mlp_channel_layout_subset(AVChannelLayout *layout, uint64_t mask)
{
    return av_channel_layout_check(layout) &&
           av_channel_layout_subset(layout, mask) ==
           av_channel_layout_subset(layout, UINT64_MAX);
}

static enum AVChannel thd_channel_layout_extract_channel(uint64_t channel_layout,
                                                         int index)
{
    int i;

    if (av_popcount64(channel_layout) <= index)
        return AV_CHAN_NONE;

    for (i = 0; i < FF_ARRAY_ELEMS(thd_channel_order); i++)
        if (channel_layout & (1ULL << thd_channel_order[i]) && !index--)
            return thd_channel_order[i];
    return AV_CHAN_NONE;
}

static VLC huff_vlc[3];

/** Initialize static data, constant between all invocations of the codec. */

static av_cold void init_static(void)
{
    for (int i = 0; i < 3; i++) {
        static VLCElem vlc_buf[3 * VLC_STATIC_SIZE];
        huff_vlc[i].table           = &vlc_buf[i * VLC_STATIC_SIZE];
        huff_vlc[i].table_allocated = VLC_STATIC_SIZE;
        vlc_init(&huff_vlc[i], VLC_BITS, 18,
                 &ff_mlp_huffman_tables[i][0][1], 2, 1,
                 &ff_mlp_huffman_tables[i][0][0], 2, 1, VLC_INIT_USE_STATIC);
    }

    ff_mlp_init_crc();
}

static inline int32_t calculate_sign_huff(MLPDecodeContext *m,
                                          unsigned int substr, unsigned int ch)
{
    SubStream *s = &m->substream[substr];
    ChannelParams *cp = &s->channel_params[ch];
    int lsb_bits = cp->huff_lsbs - s->quant_step_size[ch];
    int sign_shift = lsb_bits + (cp->codebook ? 2 - cp->codebook : -1);
    int32_t sign_huff_offset = cp->huff_offset;

    if (cp->codebook > 0)
        sign_huff_offset -= 7 << lsb_bits;

    if (sign_shift >= 0)
        sign_huff_offset -= 1 << sign_shift;

    return sign_huff_offset;
}

/** Read a sample, consisting of either, both or neither of entropy-coded MSBs
 *  and plain LSBs. */

static inline int read_huff_channels(MLPDecodeContext *m, GetBitContext *gbp,
                                     unsigned int substr, unsigned int pos)
{
    SubStream *s = &m->substream[substr];
    unsigned int mat, channel;

    for (mat = 0; mat < s->num_primitive_matrices; mat++)
        if (s->lsb_bypass[mat])
            m->bypassed_lsbs[pos + s->blockpos][mat] = get_bits(gbp, s->lsb_bypass[mat]);

    for (channel = s->min_channel; channel <= s->max_channel; channel++) {
        ChannelParams *cp = &s->channel_params[channel];
        int codebook = cp->codebook;
        int quant_step_size = s->quant_step_size[channel];
        int lsb_bits = cp->huff_lsbs - quant_step_size;
        int result = 0;

        if (codebook > 0)
            result = get_vlc2(gbp, huff_vlc[codebook-1].table,
                            VLC_BITS, (9 + VLC_BITS - 1) / VLC_BITS);

        if (result < 0)
            return AVERROR_INVALIDDATA;

        if (lsb_bits > 0)
            result = (result << lsb_bits) + get_bits_long(gbp, lsb_bits);

        result  += cp->sign_huff_offset;
        result *= 1 << quant_step_size;

        m->sample_buffer[pos + s->blockpos][channel] = result;
    }

    return 0;
}

static av_cold int mlp_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    MLPDecodeContext *m = avctx->priv_data;
    int substr;

    m->avctx = avctx;
    for (substr = 0; substr < MAX_SUBSTREAMS; substr++)
        m->substream[substr].lossless_check_data = 0xffffffff;
    ff_mlpdsp_init(&m->dsp);

    if (m->downmix_layout.nb_channels) {
        if (!av_channel_layout_compare(&m->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO) ||
            !av_channel_layout_compare(&m->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO_DOWNMIX)) {
            av_channel_layout_uninit(&avctx->ch_layout);
            avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
        } else if (!av_channel_layout_compare(&m->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0)) {
            av_channel_layout_uninit(&avctx->ch_layout);
            avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0;
        } else if (!av_channel_layout_compare(&m->downmix_layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1)) {
            av_channel_layout_uninit(&avctx->ch_layout);
            avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1;
        }
        else
          av_log(avctx, AV_LOG_WARNING, "Invalid downmix layout\n");
    }

    ff_thread_once(&init_static_once, init_static);

    return 0;
}

/** Read a major sync info header - contains high level information about
 *  the stream - sample rate, channel arrangement etc. Most of this
 *  information is not actually necessary for decoding, only for playback.
 */

static int read_major_sync(MLPDecodeContext *m, GetBitContext *gb)
{
    MLPHeaderInfo mh;
    int substr, ret;

    if ((ret = ff_mlp_read_major_sync(m->avctx, &mh, gb)) != 0)
        return ret;

    if (mh.group1_bits == 0) {
        av_log(m->avctx, AV_LOG_ERROR, "invalid/unknown bits per sample\n");
        return AVERROR_INVALIDDATA;
    }
    if (mh.group2_bits > mh.group1_bits) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Channel group 2 cannot have more bits per sample than group 1.\n");
        return AVERROR_INVALIDDATA;
    }

    if (mh.group2_samplerate && mh.group2_samplerate != mh.group1_samplerate) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Channel groups with differing sample rates are not currently supported.\n");
        return AVERROR_INVALIDDATA;
    }

    if (mh.group1_samplerate == 0) {
        av_log(m->avctx, AV_LOG_ERROR, "invalid/unknown sampling rate\n");
        return AVERROR_INVALIDDATA;
    }
    if (mh.group1_samplerate > MAX_SAMPLERATE) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Sampling rate %d is greater than the supported maximum (%d).\n",
               mh.group1_samplerate, MAX_SAMPLERATE);
        return AVERROR_INVALIDDATA;
    }
    if (mh.access_unit_size > MAX_BLOCKSIZE) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Block size %d is greater than the supported maximum (%d).\n",
               mh.access_unit_size, MAX_BLOCKSIZE);
        return AVERROR_INVALIDDATA;
    }
    if (mh.access_unit_size_pow2 > MAX_BLOCKSIZE_POW2) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Block size pow2 %d is greater than the supported maximum (%d).\n",
               mh.access_unit_size_pow2, MAX_BLOCKSIZE_POW2);
        return AVERROR_INVALIDDATA;
    }

    if (mh.num_substreams == 0)
        return AVERROR_INVALIDDATA;
    if (m->avctx->codec_id == AV_CODEC_ID_MLP && mh.num_substreams > 2) {
        av_log(m->avctx, AV_LOG_ERROR, "MLP only supports up to 2 substreams.\n");
        return AVERROR_INVALIDDATA;
    }
    if (mh.num_substreams > MAX_SUBSTREAMS) {
        avpriv_request_sample(m->avctx,
                              "%d substreams (more than the "
                              "maximum supported by the decoder)",
                              mh.num_substreams);
        return AVERROR_PATCHWELCOME;
    }

    m->major_sync_header_size = mh.header_size;

    m->access_unit_size      = mh.access_unit_size;
    m->access_unit_size_pow2 = mh.access_unit_size_pow2;

    m->num_substreams        = mh.num_substreams;
    m->extended_substream_info = mh.extended_substream_info;
    m->substream_info        = mh.substream_info;

    /*  If there is a 4th substream and the MSB of substream_info is set,
     *  there is a 16-channel spatial presentation (Atmos in TrueHD).
     */
    if (m->avctx->codec_id == AV_CODEC_ID_TRUEHD
            && m->num_substreams == 4 && m->substream_info >> 7 == 1) {
        m->avctx->profile     = AV_PROFILE_TRUEHD_ATMOS;
    }

    /* Limit to decoding the first 3 substreams (or allow the 4th for objects) */
    m->max_decoded_substream = FFMIN(m->num_substreams - 1,
        m->extract_objects ? 3 : 2);

    av_log(m->avctx, AV_LOG_DEBUG, "decoding up to substream %" PRIu8 "\n",
        m->max_decoded_substream);

    m->avctx->sample_rate    = mh.group1_samplerate;
    m->avctx->frame_size     = mh.access_unit_size;

    m->avctx->bits_per_raw_sample = mh.group1_bits;
    if (mh.group1_bits > 16)
        m->avctx->sample_fmt = AV_SAMPLE_FMT_S32;
    else
        m->avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    m->pack_output = m->dsp.mlp_select_pack_output(m->substream[m->max_decoded_substream].ch_assign,
                                                   m->substream[m->max_decoded_substream].output_shift,
                                                   m->substream[m->max_decoded_substream].max_matrix_channel,
                                                   m->avctx->sample_fmt == AV_SAMPLE_FMT_S32);

    m->params_valid = 1;
    for (substr = 0; substr < MAX_SUBSTREAMS; substr++)
        m->substream[substr].restart_seen = 0;

    /* Set the layout for each substream. When there's more than one, the first
     * substream is Stereo. Subsequent substreams' layouts are indicated in the
     * major sync. */
    if (m->avctx->codec_id == AV_CODEC_ID_MLP) {
        if (mh.stream_type != SYNC_MLP) {
            avpriv_request_sample(m->avctx,
                        "unexpected stream_type %X in MLP",
                        mh.stream_type);
            return AVERROR_PATCHWELCOME;
        }
        if ((substr = (mh.num_substreams > 1)))
            m->substream[0].mask = AV_CH_LAYOUT_STEREO;
        m->substream[substr].mask = mh.channel_layout_mlp;
    } else {
        if (mh.stream_type != SYNC_TRUEHD) {
            avpriv_request_sample(m->avctx,
                        "unexpected stream_type %X in !MLP",
                        mh.stream_type);
            return AVERROR_PATCHWELCOME;
        }
        m->substream[1].mask = mh.channel_layout_thd_stream1;
        if (mh.channels_thd_stream1 == 2 &&
            mh.channels_thd_stream2 == 2 &&
            m->avctx->ch_layout.nb_channels == 2)
            m->substream[0].mask = AV_CH_LAYOUT_STEREO;
        if ((substr = (mh.num_substreams > 1)))
            m->substream[0].mask = AV_CH_LAYOUT_STEREO;
        if (mh.num_substreams == 1 &&
            mh.channels_thd_stream1 == 1 &&
            mh.channels_thd_stream2 == 1 &&
            m->avctx->ch_layout.nb_channels == 1)
            m->substream[0].mask = AV_CH_LAYOUT_MONO;
        if (mh.num_substreams > 2)
            if (mh.channel_layout_thd_stream2)
                m->substream[2].mask = mh.channel_layout_thd_stream2;
            else
                m->substream[2].mask = mh.channel_layout_thd_stream1;
        if (m->avctx->ch_layout.nb_channels > 2)
            if (mh.num_substreams > 2)
                m->substream[1].mask = mh.channel_layout_thd_stream1;
            else
                m->substream[mh.num_substreams > 1].mask = mh.channel_layout_thd_stream2;
    }

    m->needs_reordering = mh.channel_arrangement >= 18 && mh.channel_arrangement <= 20;

    /* Parse the TrueHD decoder channel modifiers and set each substream's
     * AVMatrixEncoding accordingly.
     *
     * The meaning of the modifiers depends on the channel layout:
     *
     * - THD_CH_MODIFIER_LTRT, THD_CH_MODIFIER_LBINRBIN only apply to 2-channel
     *
     * - THD_CH_MODIFIER_MONO applies to 1-channel or 2-channel (dual mono)
     *
     * - THD_CH_MODIFIER_SURROUNDEX, THD_CH_MODIFIER_NOTSURROUNDEX only apply to
     *   layouts with an Ls/Rs channel pair
     */
    for (substr = 0; substr < MAX_SUBSTREAMS; substr++)
        m->substream[substr].matrix_encoding = AV_MATRIX_ENCODING_NONE;
    if (m->avctx->codec_id == AV_CODEC_ID_TRUEHD) {
        if (mh.num_substreams > 2 &&
            mh.channel_layout_thd_stream2 & AV_CH_SIDE_LEFT &&
            mh.channel_layout_thd_stream2 & AV_CH_SIDE_RIGHT &&
            mh.channel_modifier_thd_stream2 == THD_CH_MODIFIER_SURROUNDEX)
            m->substream[2].matrix_encoding = AV_MATRIX_ENCODING_DOLBYEX;

        if (mh.num_substreams > 1 &&
            mh.channel_layout_thd_stream1 & AV_CH_SIDE_LEFT &&
            mh.channel_layout_thd_stream1 & AV_CH_SIDE_RIGHT &&
            mh.channel_modifier_thd_stream1 == THD_CH_MODIFIER_SURROUNDEX)
            m->substream[1].matrix_encoding = AV_MATRIX_ENCODING_DOLBYEX;

        if (mh.num_substreams > 0)
            switch (mh.channel_modifier_thd_stream0) {
            case THD_CH_MODIFIER_LTRT:
                m->substream[0].matrix_encoding = AV_MATRIX_ENCODING_DOLBY;
                break;
            case THD_CH_MODIFIER_LBINRBIN:
                m->substream[0].matrix_encoding = AV_MATRIX_ENCODING_DOLBYHEADPHONE;
                break;
            default:
                break;
            }
    }

    return 0;
}

/** Read a restart header from a block in a substream. This contains parameters
 *  required to decode the audio that do not change very often. Generally
 *  (always) present only in blocks following a major sync. */

static int read_restart_header(MLPDecodeContext *m, GetBitContext *gbp,
                               const uint8_t *buf, unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int ch;
    int sync_word, tmp;
    uint8_t checksum;
    uint8_t lossless_check;
    uint8_t max_bit_depth;
    int start_count = get_bits_count(gbp);
    int min_channel, max_channel, max_matrix_channel;
    const int std_max_matrix_channel = m->avctx->codec_id == AV_CODEC_ID_MLP
                                     ? MAX_MATRIX_CHANNEL_MLP
                                     : MAX_MATRIX_CHANNEL_TRUEHD;

    sync_word = get_bits(gbp, 14);

    if (sync_word < 0x31ea || 0x31ec < sync_word) {
        av_log(m->avctx, AV_LOG_ERROR,
               "restart header sync incorrect (got 0x%04x)\n", sync_word);
        return AVERROR_INVALIDDATA;
    }

    if (m->avctx->codec_id == AV_CODEC_ID_MLP && 0x31ea != sync_word) {
        av_log(m->avctx, AV_LOG_ERROR, "MLP must have 0x31ea sync word.\n");
        return AVERROR_INVALIDDATA;
    }

    skip_bits(gbp, 16); /* Output timestamp */

    min_channel        = get_bits(gbp, 4);
    max_channel        = get_bits(gbp, 4);
    max_matrix_channel = get_bits(gbp, 4);

    if (max_matrix_channel > std_max_matrix_channel) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Max matrix channel cannot be greater than %d.\n",
               std_max_matrix_channel);
        return AVERROR_INVALIDDATA;
    }

    /* This should happen for TrueHD streams with >6 channels and MLP's noise
     * type. It is not yet known if this is allowed. */
    if (max_matrix_channel > MAX_MATRIX_CHANNEL_MLP && 0x31ea == sync_word) {
        avpriv_request_sample(m->avctx,
                              "%d channels (more than the "
                              "maximum supported by the decoder)",
                              max_channel + 2);
        return AVERROR_PATCHWELCOME;
    }

    if (max_channel + 1 > MAX_CHANNELS || max_channel + 1 < min_channel)
        return AVERROR_INVALIDDATA;

    s->min_channel        = min_channel;
    s->max_channel        = max_channel;
    s->coded_channels     = ((1LL << (max_channel - min_channel + 1)) - 1) << min_channel;
    s->max_matrix_channel = max_matrix_channel;
    s->substream_type     = sync_word;

    if (mlp_channel_layout_subset(&m->downmix_layout, s->mask) &&
        m->max_decoded_substream > substr) {
        av_log(m->avctx, AV_LOG_DEBUG,
               "Extracting %d-channel downmix (0x%"PRIx64") from substream %d. "
               "Further substreams will be skipped.\n",
               s->max_channel + 1, s->mask, substr);
        m->max_decoded_substream = substr;
    }

    s->noise_shift   = get_bits(gbp,  4);
    s->noisegen_seed = get_bits(gbp, 23);
    s->max_shift     = get_bits(gbp,  4);
    s->max_lsbs      = get_bits(gbp,  5);
    s->max_bits      = get_bits(gbp,  5);

    max_bit_depth = (0x31ec == sync_word) ? 31 : 24;
    if (max_bit_depth < s->max_lsbs) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Max LSB size %" PRIu8 " for substream %u exceeds "
               "%" PRIu8 " bits.\n",
               s->max_lsbs, substr, max_bit_depth);
        return AVERROR_INVALIDDATA;
    }

    if (max_bit_depth < s->max_bits) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Max output bit-depth %" PRIu8 " for substream %u exceeds "
               "%" PRIu8 " bits.\n",
               s->max_bits, substr, max_bit_depth);
        return AVERROR_INVALIDDATA;
    }

    skip_bits(gbp, 5);

    s->data_check_present = get_bits1(gbp);
    lossless_check = get_bits(gbp, 8);
    if (substr == m->max_decoded_substream
        && s->lossless_check_data != 0xffffffff) {
        tmp = xor_32_to_8(s->lossless_check_data);
        if (tmp != lossless_check)
            av_log(m->avctx, AV_LOG_WARNING,
                   "Lossless check failed - expected %02x, calculated %02x.\n",
                   lossless_check, tmp);
    }

    skip_bits(gbp, 16);

    memset(s->ch_assign, 0, sizeof(s->ch_assign));

    for (ch = 0; ch <= s->max_matrix_channel; ch++) {
        int ch_assign = get_bits(gbp, 6);
        if (m->avctx->codec_id == AV_CODEC_ID_TRUEHD && s->mask) {
            AVChannelLayout l;
            enum AVChannel channel = thd_channel_layout_extract_channel(s->mask, ch_assign);

            av_channel_layout_from_mask(&l, s->mask);
            ch_assign = av_channel_layout_index_from_channel(&l, channel);
        }
        if (ch_assign < 0 || ch_assign > s->max_matrix_channel) {
            avpriv_request_sample(m->avctx,
                                  "Assignment of matrix channel %d to invalid output channel %d",
                                  ch, ch_assign);
            return AVERROR_PATCHWELCOME;
        }
        s->ch_assign[ch_assign] = ch;
    }

    checksum = ff_mlp_restart_checksum(buf, get_bits_count(gbp) - start_count);

    if (checksum != get_bits(gbp, 8))
        av_log(m->avctx, AV_LOG_ERROR, "restart header checksum error\n");

    /* Set default decoding parameters. */
    s->param_presence_flags   = 0xff;
    s->num_primitive_matrices = 0;
    s->blocksize              = 8;
    s->lossless_check_data    = 0;

    memset(s->output_shift   , 0, sizeof(s->output_shift   ));
    memset(s->quant_step_size, 0, sizeof(s->quant_step_size));

    for (ch = s->min_channel; ch <= s->max_channel; ch++) {
        ChannelParams *cp = &s->channel_params[ch];
        cp->filter_params[FIR].order = 0;
        cp->filter_params[IIR].order = 0;
        cp->filter_params[FIR].shift = 0;
        cp->filter_params[IIR].shift = 0;

        /* Default audio coding is 24-bit raw PCM. */
        cp->huff_offset      = 0;
        cp->sign_huff_offset = -(1 << 23);
        cp->codebook         = 0;
        cp->huff_lsbs        = (3 == substr) ? 31 : 24;
    }

    if (substr == m->max_decoded_substream) {
        av_channel_layout_uninit(&m->avctx->ch_layout);
        if (substr < 3) /* Loudspeaker feed channels */
            av_channel_layout_from_mask(&m->avctx->ch_layout, s->mask);
        else /* Object channels */
            m->avctx->ch_layout = (AVChannelLayout) {
                AV_CHANNEL_ORDER_UNSPEC,
                s->max_channel+1
            };

        m->pack_output = m->dsp.mlp_select_pack_output(s->ch_assign,
                                                       s->output_shift,
                                                       s->max_matrix_channel,
                                                       m->avctx->sample_fmt == AV_SAMPLE_FMT_S32);

        if (m->avctx->codec_id == AV_CODEC_ID_MLP && m->needs_reordering) {
            if (s->mask == (AV_CH_LAYOUT_QUAD|AV_CH_LOW_FREQUENCY) ||
                s->mask == AV_CH_LAYOUT_5POINT0_BACK) {
                int i = s->ch_assign[4];
                s->ch_assign[4] = s->ch_assign[3];
                s->ch_assign[3] = s->ch_assign[2];
                s->ch_assign[2] = i;
            } else if (s->mask == AV_CH_LAYOUT_5POINT1_BACK) {
                FFSWAP(int, s->ch_assign[2], s->ch_assign[4]);
                FFSWAP(int, s->ch_assign[3], s->ch_assign[5]);
            }
        }

    }

    return 0;
}

/** Read parameters for one of the prediction filters. */

static int read_filter_params(MLPDecodeContext *m, GetBitContext *gbp,
                              unsigned int substr, unsigned int channel,
                              unsigned int filter)
{
    SubStream *s = &m->substream[substr];
    FilterParams *fp = &s->channel_params[channel].filter_params[filter];
    const int max_order = filter ? MAX_IIR_ORDER : MAX_FIR_ORDER;
    const char fchar = filter ? 'I' : 'F';
    int i, order;

    // Filter is 0 for FIR, 1 for IIR.
    av_assert0(filter < 2);

    if (m->filter_changed[channel][filter]++ > 1) {
        av_log(m->avctx, AV_LOG_ERROR, "Filters may change only once per access unit.\n");
        return AVERROR_INVALIDDATA;
    }

    order = get_bits(gbp, 4);
    if (order > max_order) {
        av_log(m->avctx, AV_LOG_ERROR,
               "%cIR filter order %d is greater than maximum %d.\n",
               fchar, order, max_order);
        return AVERROR_INVALIDDATA;
    }
    fp->order = order;

    if (order > 0) {
        int32_t *fcoeff = s->channel_params[channel].coeff[filter];
        int coeff_bits, coeff_shift;

        fp->shift = get_bits(gbp, 4);

        coeff_bits  = get_bits(gbp, 5);
        coeff_shift = get_bits(gbp, 3);
        if (coeff_bits < 1 || coeff_bits > 16) {
            av_log(m->avctx, AV_LOG_ERROR,
                   "%cIR filter coeff_bits must be between 1 and 16.\n",
                   fchar);
            return AVERROR_INVALIDDATA;
        }
        if (coeff_bits + coeff_shift > 16) {
            av_log(m->avctx, AV_LOG_ERROR,
                   "Sum of coeff_bits and coeff_shift for %cIR filter must be 16 or less.\n",
                   fchar);
            return AVERROR_INVALIDDATA;
        }

        for (i = 0; i < order; i++)
            fcoeff[i] = get_sbits(gbp, coeff_bits) * (1 << coeff_shift);

        if (get_bits1(gbp)) {
            int state_bits, state_shift;

            if (filter == FIR) {
                av_log(m->avctx, AV_LOG_ERROR,
                       "FIR filter has state data specified.\n");
                return AVERROR_INVALIDDATA;
            }

            state_bits  = get_bits(gbp, 4);
            state_shift = get_bits(gbp, 4);

            /* TODO: Check validity of state data. */

            for (i = 0; i < order; i++)
                fp->state[i] = state_bits ? get_sbits(gbp, state_bits) * (1 << state_shift) : 0;
        }
    }

    return 0;
}

/** Get the maximum number of primitive matrices allowed. */

static int get_max_nb_primitive_matrices(MLPDecodeContext *m, unsigned int substr)
{
    switch (substr) {
    case 0: // substream 0 (up to 2 matrix channels)
        return 2;
    case 1: // substream 1
        if (m->substream_info & 0x8) // 6-ch pres carried
            return 6;
        if (m->substream_info & 0x20) // 8-ch pres carried
            return 8;
        break;
    case 2: // substream 2
        if (m->substream_info & 0x40) // 8-ch pres carried
            return 8;
        break;
    case 3: // substream 3
        if (m->substream_info & 80) // 16-ch pres carried
            return 16;
        break;
    }

    return MAX_MATRICES_TRUEHD;
}

/** Read parameters for primitive matrices (0x31ea and 0x31eb substreams). */

static int read_31ea_31eb_matrix_params(MLPDecodeContext *m, unsigned int substr, GetBitContext *gbp)
{
    SubStream *s = &m->substream[substr];
    unsigned int mat, ch;
    const int max_primitive_matrices = m->avctx->codec_id == AV_CODEC_ID_MLP
                                     ? MAX_MATRICES_MLP
                                     : get_max_nb_primitive_matrices(m, substr);

    if (++m->matrix_changed > 1) {
        av_log(m->avctx, AV_LOG_ERROR, "Matrices may change only once per access unit.\n");
        return AVERROR_INVALIDDATA;
    }

    s->num_primitive_matrices = get_bits(gbp, 4);

    if (s->num_primitive_matrices > max_primitive_matrices) {
        av_log(m->avctx, AV_LOG_ERROR,
               "Number of primitive matrices cannot be greater than %d "
               "for substream %u of type 0x%04x.\n",
               max_primitive_matrices, substr, s->substream_type);
        goto error;
    }

    for (mat = 0; mat < s->num_primitive_matrices; mat++) {
        int frac_bits, max_chan;
        s->matrix_out_ch[mat] = get_bits(gbp, 4);
        frac_bits             = get_bits(gbp, 4);
        s->lsb_bypass[mat]    = get_bits1(gbp);

        if (s->matrix_out_ch[mat] > s->max_matrix_channel) {
            av_log(m->avctx, AV_LOG_ERROR,
                    "Invalid channel %d specified as output from matrix.\n",
                    s->matrix_out_ch[mat]);
            goto error;
        }
        if (frac_bits > 14) {
            av_log(m->avctx, AV_LOG_ERROR,
                    "Too many fractional bits specified.\n");
            goto error;
        }

        max_chan = s->max_matrix_channel;
        if (0x31ea == s->substream_type)
            max_chan += 2;

        for (ch = 0; ch <= max_chan; ch++) {
            int coeff_val = 0;
            if (get_bits1(gbp))
                coeff_val = get_sbits(gbp, frac_bits + 2);

            s->matrix_coeff[mat][ch] = coeff_val * (1 << ((14 + 4) - frac_bits));
        }

        if (0x31eb == s->substream_type)
            s->matrix_noise_shift[mat] = get_bits(gbp, 4);
        else
            s->matrix_noise_shift[mat] = 0;
    }

    return 0;

error:
    s->num_primitive_matrices = 0;
    memset(s->matrix_out_ch, 0, sizeof(s->matrix_out_ch));

    return AVERROR_INVALIDDATA;
}

/** Read parameters for primitive matrices (0x31ec substreams). */

static int read_31ec_matrix_params(MLPDecodeContext *m, unsigned int substr, GetBitContext *gbp)
{
    SubStream *s = &m->substream[substr];
    unsigned int mat, ch;

    const int max_primitive_matrices = get_max_nb_primitive_matrices(m, substr);

    if (++m->matrix_changed > 1) {
        av_log(m->avctx, AV_LOG_ERROR, "Matrices may change only once per access unit.\n");
        return AVERROR_INVALIDDATA;
    }

    /* Seed primitive matrices */

    if (get_bits1(gbp)) {
        /* New seed primitive matrices */

        if (get_bits1(gbp)) {
            /* New seed matrices parameters */
            s->num_primitive_matrices = get_bits(gbp, 4) + 1;

            if (s->num_primitive_matrices > max_primitive_matrices) {
                av_log(m->avctx, AV_LOG_ERROR,
                    "Number of primitive matrices cannot be greater than %d "
                    "in substream %u of type 0x%04x.\n",
                    max_primitive_matrices, substr, s->substream_type);
                goto error;
            }

            for (mat = 0; mat < s->num_primitive_matrices; mat++) {
                s->matrix_out_ch         [mat] = get_bits(gbp, 4);
                s->matrix_coeff_frac_bits[mat] = get_bits(gbp, 4);
                s->matrix_coeff_shift    [mat] = ((int) get_bits(gbp, 3)) - 1;
                s->lsb_bypass            [mat] = get_bits(gbp, 2);
                s->matrix_noise_shift    [mat] = get_bits(gbp, 4);
                s->matrix_coeff_mask     [mat] = get_bits(gbp, s->max_matrix_channel + 1);

                if (s->matrix_out_ch[mat] > s->max_matrix_channel) {
                    av_log(m->avctx, AV_LOG_ERROR,
                            "Invalid channel %d specified as output from matrix.\n",
                            s->matrix_out_ch[mat]);
                    goto error;
                }
                if (s->matrix_coeff_frac_bits[mat] > 14) {
                    av_log(m->avctx, AV_LOG_ERROR,
                            "Too many fractional bits specified.\n");
                    goto error;
                }
            }
        }

        /* Seed matrices coefficients */
        for (mat = 0; mat < s->num_primitive_matrices; mat++) {
            const int coeff_shift = s->matrix_coeff_shift[mat] -
                                    s->matrix_coeff_frac_bits[mat];

            memset(s->matrix_coeff[mat], 0, sizeof(s->matrix_coeff[mat]));

            for (ch = 0; ch <= s->max_matrix_channel; ch++) {
                int64_t coeff_val;

                if (!((s->matrix_coeff_mask[mat] >> ch) & 0x1))
                    continue; // skip channel

                coeff_val = get_sbits(gbp, s->matrix_coeff_frac_bits[mat] + 2);
                s->matrix_coeff[mat][ch] = coeff_val * (1 << (18 + coeff_shift));
            }
        }
    }

    if (!get_bits1(gbp)) {
        /* No primitive matrices interpolation */
        memset(s->delta_matrix_coeff, 0, sizeof(s->delta_matrix_coeff));
    }
    else if (get_bits1(gbp)) {
        /* New delta primitive matrices */

        if (get_bits1(gbp)) {
            /* New delta primitive matrices parameters */

            for (mat = 0; mat < s->num_primitive_matrices; mat++) {
                s->delta_matrix_coeff_bits[mat] = get_bits(gbp, 4) + 1;
                s->delta_matrix_coeff_prec[mat] = get_bits(gbp, 2);
            }
        }

        for (mat = 0; mat < s->num_primitive_matrices; mat++) {
            const int coeff_shift = s->matrix_coeff_shift[mat]
                - s->delta_matrix_coeff_prec[mat]
                - s->matrix_coeff_frac_bits[mat];

            memset(s->delta_matrix_coeff[mat], 0, sizeof(s->delta_matrix_coeff[mat]));

            if (s->delta_matrix_coeff_bits[mat] <= 1)
                continue; // skip matrice

            for (ch = 0; ch <= s->max_matrix_channel; ch++) {
                int64_t coeff_val;

                if (!((s->matrix_coeff_mask[mat] >> ch) & 0x1))
                    continue; // skip channel

                coeff_val = get_sbits(gbp, s->delta_matrix_coeff_bits[mat]);
                s->delta_matrix_coeff[mat][ch] = coeff_val * (1 << (18 + coeff_shift));
            }
        }
    }

    return 0;
error:
    s->num_primitive_matrices = 0;
    memset(s->matrix_out_ch, 0, sizeof(s->matrix_out_ch));

    return AVERROR_INVALIDDATA;
}

/** Read channel parameters. */

static int read_channel_params(MLPDecodeContext *m, unsigned int substr,
                               GetBitContext *gbp, unsigned int ch)
{
    SubStream *s = &m->substream[substr];
    ChannelParams *cp = &s->channel_params[ch];
    FilterParams *fir = &cp->filter_params[FIR];
    FilterParams *iir = &cp->filter_params[IIR];
    int ret;

    if (s->param_presence_flags & PARAM_FIR)
        if (get_bits1(gbp))
            if ((ret = read_filter_params(m, gbp, substr, ch, FIR)) < 0)
                return ret;

    if (s->param_presence_flags & PARAM_IIR)
        if (get_bits1(gbp))
            if ((ret = read_filter_params(m, gbp, substr, ch, IIR)) < 0)
                return ret;

    if (fir->order + iir->order > 8) {
        av_log(m->avctx, AV_LOG_ERROR, "Total filter orders too high.\n");
        return AVERROR_INVALIDDATA;
    }

    if (fir->order && iir->order &&
        fir->shift != iir->shift) {
        av_log(m->avctx, AV_LOG_ERROR,
                "FIR and IIR filters must use the same precision.\n");
        return AVERROR_INVALIDDATA;
    }
    /* The FIR and IIR filters must have the same precision.
     * To simplify the filtering code, only the precision of the
     * FIR filter is considered. If only the IIR filter is employed,
     * the FIR filter precision is set to that of the IIR filter, so
     * that the filtering code can use it. */
    if (!fir->order && iir->order)
        fir->shift = iir->shift;

    if (s->param_presence_flags & PARAM_HUFFOFFSET)
        if (get_bits1(gbp))
            cp->huff_offset = get_sbits(gbp, 15);

    cp->codebook  = get_bits(gbp, 2);
    cp->huff_lsbs = get_bits(gbp, 5);

    if (cp->codebook > 0 && cp->huff_lsbs > s->max_lsbs) {
        av_log(m->avctx, AV_LOG_ERROR, "Invalid huff_lsbs=%" PRIu8 ", "
               "exceeds max_lsbs=%" PRIu8 ".\n",
               cp->huff_lsbs, s->max_lsbs);
        cp->huff_lsbs = 0;
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

/** Read decoding parameters that change more often than those in the restart
 *  header. */

static int read_decoding_params(MLPDecodeContext *m, GetBitContext *gbp,
                                unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int ch;
    int ret = 0;
    unsigned recompute_sho = 0;

    if (s->param_presence_flags & PARAM_PRESENCE)
        if (get_bits1(gbp))
            s->param_presence_flags = get_bits(gbp, 8);

    if (s->param_presence_flags & PARAM_BLOCKSIZE)
        if (get_bits1(gbp)) {
            s->blocksize = get_bits(gbp, 9);
            if (s->blocksize < 8 || s->blocksize > m->access_unit_size) {
                av_log(m->avctx, AV_LOG_ERROR, "Invalid blocksize.\n");
                s->blocksize = 0;
                return AVERROR_INVALIDDATA;
            }
        }

    if (s->param_presence_flags & PARAM_MATRIX)
        if (get_bits1(gbp)) {
            if (0x31ec == s->substream_type)
                ret = read_31ec_matrix_params(m, substr, gbp);
            else
                ret = read_31ea_31eb_matrix_params(m, substr, gbp);
            if (ret < 0)
                return ret;
        }

    if (s->param_presence_flags & PARAM_OUTSHIFT)
        if (get_bits1(gbp)) {
            for (ch = 0; ch <= s->max_matrix_channel; ch++) {
                s->output_shift[ch] = get_sbits(gbp, 4);
                if (s->output_shift[ch] < 0) {
                    avpriv_request_sample(m->avctx, "Negative output_shift");
                    s->output_shift[ch] = 0;
                }
                if (s->max_shift < s->output_shift[ch])
                    av_log(m->avctx, AV_LOG_WARNING,
                           "output_shift=%d exceeds max_shift=%d\n",
                           s->output_shift[ch], s->max_shift);
            }
            if (substr == m->max_decoded_substream)
                m->pack_output = m->dsp.mlp_select_pack_output(s->ch_assign,
                                                               s->output_shift,
                                                               s->max_matrix_channel,
                                                               m->avctx->sample_fmt == AV_SAMPLE_FMT_S32);
        }

    if (s->param_presence_flags & PARAM_QUANTSTEP)
        if (get_bits1(gbp))
            for (ch = 0; ch <= s->max_channel; ch++) {
                s->quant_step_size[ch] = get_bits(gbp, 4);

                recompute_sho |= 1<<ch;
            }

    for (ch = s->min_channel; ch <= s->max_channel; ch++)
        if (get_bits1(gbp)) {
            recompute_sho |= 1<<ch;
            if ((ret = read_channel_params(m, substr, gbp, ch)) < 0)
                goto fail;
        }


fail:
    for (ch = 0; ch <= s->max_channel; ch++) {
        if (recompute_sho & (1<<ch)) {
            ChannelParams *cp = &s->channel_params[ch];

            if (cp->codebook > 0 && cp->huff_lsbs < s->quant_step_size[ch]) {
                if (ret >= 0) {
                    av_log(m->avctx, AV_LOG_ERROR, "quant_step_size larger than huff_lsbs\n");
                    ret = AVERROR_INVALIDDATA;
                }
                s->quant_step_size[ch] = 0;
            }

            cp->sign_huff_offset = calculate_sign_huff(m, substr, ch);
        }
    }
    return ret;
}

#define MSB_MASK(bits)  (-(1 << (bits)))

/** Generate PCM samples using the prediction filters and residual values
 *  read from the data stream, and update the filter state. */

static void filter_channel(MLPDecodeContext *m, unsigned int substr,
                           unsigned int channel)
{
    SubStream *s = &m->substream[substr];
    const int32_t *fircoeff = s->channel_params[channel].coeff[FIR];
    int32_t state_buffer[NUM_FILTERS][MAX_BLOCKSIZE + MAX_FIR_ORDER];
    int32_t *firbuf = state_buffer[FIR] + MAX_BLOCKSIZE;
    int32_t *iirbuf = state_buffer[IIR] + MAX_BLOCKSIZE;
    FilterParams *fir = &s->channel_params[channel].filter_params[FIR];
    FilterParams *iir = &s->channel_params[channel].filter_params[IIR];
    unsigned int filter_shift = fir->shift;
    int32_t mask = MSB_MASK(s->quant_step_size[channel]);

    memcpy(firbuf, fir->state, MAX_FIR_ORDER * sizeof(int32_t));
    memcpy(iirbuf, iir->state, MAX_IIR_ORDER * sizeof(int32_t));

    m->dsp.mlp_filter_channel(firbuf, fircoeff,
                              fir->order, iir->order,
                              filter_shift, mask, s->blocksize,
                              &m->sample_buffer[s->blockpos][channel]);

    memcpy(fir->state, firbuf - s->blocksize, MAX_FIR_ORDER * sizeof(int32_t));
    memcpy(iir->state, iirbuf - s->blocksize, MAX_IIR_ORDER * sizeof(int32_t));
}

/** Read a block of PCM residual data (or actual if no filtering active). */

static int read_block_data(MLPDecodeContext *m, GetBitContext *gbp,
                           unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int i, ch, expected_stream_pos = 0;
    int ret;

    if (s->data_check_present) {
        expected_stream_pos  = get_bits_count(gbp);
        expected_stream_pos += get_bits(gbp, 16);
        avpriv_request_sample(m->avctx,
                              "Substreams with VLC block size check info");
    }

    if (s->blockpos + s->blocksize > m->access_unit_size) {
        av_log(m->avctx, AV_LOG_ERROR, "too many audio samples in frame\n");
        return AVERROR_INVALIDDATA;
    }

    memset(&m->bypassed_lsbs[s->blockpos][0], 0,
           s->blocksize * sizeof(m->bypassed_lsbs[0]));

    for (i = 0; i < s->blocksize; i++)
        if ((ret = read_huff_channels(m, gbp, substr, i)) < 0)
            return ret;

    for (ch = s->min_channel; ch <= s->max_channel; ch++)
        filter_channel(m, substr, ch);

    s->blockpos += s->blocksize;

    if (s->data_check_present) {
        if (get_bits_count(gbp) != expected_stream_pos)
            av_log(m->avctx, AV_LOG_ERROR, "block data length mismatch\n");
        skip_bits(gbp, 8);
    }

    return 0;
}

/** Data table used for TrueHD noise generation function. */

static const int8_t noise_table[256] = {
     30,  51,  22,  54,   3,   7,  -4,  38,  14,  55,  46,  81,  22,  58,  -3,   2,
     52,  31,  -7,  51,  15,  44,  74,  30,  85, -17,  10,  33,  18,  80,  28,  62,
     10,  32,  23,  69,  72,  26,  35,  17,  73,  60,   8,  56,   2,   6,  -2,  -5,
     51,   4,  11,  50,  66,  76,  21,  44,  33,  47,   1,  26,  64,  48,  57,  40,
     38,  16, -10, -28,  92,  22, -18,  29, -10,   5, -13,  49,  19,  24,  70,  34,
     61,  48,  30,  14,  -6,  25,  58,  33,  42,  60,  67,  17,  54,  17,  22,  30,
     67,  44,  -9,  50, -11,  43,  40,  32,  59,  82,  13,  49, -14,  55,  60,  36,
     48,  49,  31,  47,  15,  12,   4,  65,   1,  23,  29,  39,  45,  -2,  84,  69,
      0,  72,  37,  57,  27,  41, -15, -16,  35,  31,  14,  61,  24,   0,  27,  24,
     16,  41,  55,  34,  53,   9,  56,  12,  25,  29,  53,   5,  20, -20,  -8,  20,
     13,  28,  -3,  78,  38,  16,  11,  62,  46,  29,  21,  24,  46,  65,  43, -23,
     89,  18,  74,  21,  38, -12,  19,  12, -19,   8,  15,  33,   4,  57,   9,  -8,
     36,  35,  26,  28,   7,  83,  63,  79,  75,  11,   3,  87,  37,  47,  34,  40,
     39,  19,  20,  42,  27,  34,  39,  77,  13,  42,  59,  64,  45,  -1,  32,  37,
     45,  -5,  53,  -6,   7,  36,  50,  23,   6,  32,   9, -21,  18,  71,  27,  52,
    -25,  31,  35,  42,  -1,  68,  63,  52,  26,  43,  66,  37,  41,  25,  40,  70,
};

/** Noise generation functions.
 *  I'm not sure what these are for - they seem to be some kind of pseudorandom
 *  sequence generators, used to generate noise data which is used when the
 *  channels are rematrixed. I'm not sure if they provide a practical benefit
 *  to compression, or just obfuscate the decoder. Are they for some kind of
 *  dithering? */

/** Generate two channels of noise, used in the matrix when
 *  restart sync word == 0x31ea. */

static void generate_2_noise_channels(MLPDecodeContext *m, unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int i;
    uint32_t seed = s->noisegen_seed;
    unsigned int maxchan = s->max_matrix_channel;

    for (i = 0; i < s->blockpos; i++) {
        uint16_t seed_shr7 = seed >> 7;
        m->sample_buffer[i][maxchan+1] = ((int8_t)(seed >> 15)) * (1 << s->noise_shift);
        m->sample_buffer[i][maxchan+2] = ((int8_t) seed_shr7)   * (1 << s->noise_shift);

        seed = (seed << 16) ^ seed_shr7 ^ (seed_shr7 << 5);
    }

    s->noisegen_seed = seed;
}

/** Generate a block of noise, used when restart sync word == 0x31eb. */

static void fill_noise_buffer(MLPDecodeContext *m, unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int i;
    uint32_t seed = s->noisegen_seed;

    for (i = 0; i < m->access_unit_size_pow2; i++) {
        uint8_t seed_shr15 = seed >> 15;
        m->noise_buffer[i] = noise_table[seed_shr15];
        seed = (seed << 8) ^ seed_shr15 ^ (seed_shr15 << 5);
    }

    s->noisegen_seed = seed;
}

#if defined(ASSERT_LEVEL) && ASSERT_LEVEL >= 2

/** Check matrices-based channel remapping output for saturation. */

static void check_rematrix_output(MLPDecodeContext *m, unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    unsigned int mat, sample;

    for (mat = 0; mat < s->num_primitive_matrices; mat++) {
        unsigned int dest_ch = s->matrix_out_ch[mat];

        const uint8_t shift = (3 <= substr && s->min_channel <= dest_ch) ? 31 : 23;
        const int32_t min_value = -(1u << shift);
        const int32_t max_value =  (1u << shift) - 1;

        for (sample = 0; sample < s->blockpos; sample++) {
            if (m->sample_buffer[sample][dest_ch] < min_value)
                av_log(m->avctx, AV_LOG_WARNING,
                    "rematrix negative saturation substr=%u mat=%u sample=%d "
                    "value=%" PRId32 "\n",
                    substr, mat, sample, m->sample_buffer[sample][dest_ch]);
            if (m->sample_buffer[sample][dest_ch] > max_value)
                av_log(m->avctx, AV_LOG_WARNING,
                    "rematrix positive saturation substr=%u mat=%u sample=%d "
                    "value=%" PRId32 "\n",
                    substr, mat, sample, m->sample_buffer[sample][dest_ch]);
        }
    }
}

/** Check output audio bit-depth. */

static void check_output_bit_depth(MLPDecodeContext *m, unsigned int substr)
{
    SubStream *s = &m->substream[substr];
    uint32_t cumul_mask = 0;
    unsigned int chan, sample;

    for (chan = 0; chan <= s->max_matrix_channel; chan++)
        for (sample = 0; sample < s->blockpos; sample++)
            cumul_mask |= FFABS(m->sample_buffer[sample][chan]);

    if ((1u << s->max_bits) <= cumul_mask)
        av_log(m->avctx, AV_LOG_WARNING, "output audio bit-depth exceeds "
               "expected %u bits.\n",
               s->max_bits);
}
#endif

/** Write the audio data into the output buffer. */

static int output_data(MLPDecodeContext *m, unsigned int substr,
                       AVFrame *frame, int *got_frame_ptr)
{
    AVCodecContext *avctx = m->avctx;
    SubStream *s = &m->substream[substr];
    unsigned int mat, chan, maxchan;
    int ret;
    int is32 = (m->avctx->sample_fmt == AV_SAMPLE_FMT_S32);

    if (m->avctx->ch_layout.nb_channels != s->max_matrix_channel + 1) {
        av_log(m->avctx, AV_LOG_ERROR, "channel count mismatch\n");
        return AVERROR_INVALIDDATA;
    }

    if (!s->blockpos) {
        av_log(avctx, AV_LOG_ERROR, "No samples to output.\n");
        return AVERROR_INVALIDDATA;
    }

    maxchan = s->max_matrix_channel;
    if (0x31ea == s->substream_type) {
        generate_2_noise_channels(m, substr);
        maxchan += 2;
    } else {
        fill_noise_buffer(m, substr);
    }

    /* Apply the channel matrices in turn to reconstruct the original audio
     * samples. */
    for (mat = 0; mat < s->num_primitive_matrices; mat++) {
        unsigned int dest_ch = s->matrix_out_ch[mat];

        if (substr < 3) {
            /* Single primitive matrices */
            m->dsp.mlp_rematrix_channel(&m->sample_buffer[0][0],
                                        s->matrix_coeff[mat],
                                        &m->bypassed_lsbs[0][mat],
                                        m->noise_buffer,
                                        s->num_primitive_matrices - mat,
                                        dest_ch,
                                        s->blockpos,
                                        maxchan,
                                        s->matrix_noise_shift[mat],
                                        m->access_unit_size_pow2,
                                        MSB_MASK(s->quant_step_size[dest_ch]));
        }
        else {
            /* Interpolated primitive matrices */
            m->dsp.mlp_rematrix_interp_channel(&m->sample_buffer[0][0],
                                               s->matrix_coeff[mat],
                                               s->delta_matrix_coeff[mat],
                                               &m->bypassed_lsbs[0][mat],
                                               m->noise_buffer,
                                               s->num_primitive_matrices - mat,
                                               dest_ch,
                                               s->blockpos,
                                               maxchan,
                                               s->matrix_noise_shift[mat],
                                               m->access_unit_size_pow2,
                                               MSB_MASK(s->quant_step_size[dest_ch]));

            for (chan = 0; chan <= maxchan; chan++)
                s->matrix_coeff[mat][chan] += s->delta_matrix_coeff[mat][chan];
        }
    }

#if defined(ASSERT_LEVEL) && ASSERT_LEVEL >= 2
    check_rematrix_output(m, substr);
#endif

    /* get output buffer */
    frame->nb_samples = s->blockpos;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    s->lossless_check_data = m->pack_output(s->lossless_check_data,
                                            s->blockpos,
                                            m->sample_buffer,
                                            frame->data[0],
                                            s->ch_assign,
                                            s->output_shift,
                                            s->max_matrix_channel,
                                            is32);

#if defined(ASSERT_LEVEL) && ASSERT_LEVEL >= 2
    check_output_bit_depth(m, substr);
#endif

    /* Update matrix encoding side data */
    if (s->matrix_encoding != s->prev_matrix_encoding) {
        if ((ret = ff_side_data_update_matrix_encoding(frame, s->matrix_encoding)) < 0)
            return ret;

        s->prev_matrix_encoding = s->matrix_encoding;
    }

    *got_frame_ptr = 1;

    return 0;
}

/** Read an access unit from the stream.
 *  @return negative on error, 0 if not enough data is present in the input stream,
 *  otherwise the number of bytes consumed. */

static int read_access_unit(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame_ptr, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    MLPDecodeContext *m = avctx->priv_data;
    GetBitContext gb;
    unsigned int length, substr;
    unsigned int substream_start;
    unsigned int header_size = 4;
    unsigned int substr_header_size = 0;
    uint8_t substream_parity_present[MAX_SUBSTREAMS];
    uint16_t substream_data_len[MAX_SUBSTREAMS];
    uint8_t parity_bits;
    int ret;

    if (buf_size < 4)
        return AVERROR_INVALIDDATA;

    length = (AV_RB16(buf) & 0xfff) * 2;

    if (length < 4 || length > buf_size)
        return AVERROR_INVALIDDATA;

    init_get_bits(&gb, (buf + 4), (length - 4) * 8);

    m->is_major_sync_unit = 0;
    if (show_bits_long(&gb, 31) == (0xf8726fba >> 1)) {
        if (read_major_sync(m, &gb) < 0)
            goto error;
        m->is_major_sync_unit = 1;
        header_size += m->major_sync_header_size;
        frame->flags |= AV_FRAME_FLAG_KEY;
    }

    if (!m->params_valid) {
        av_log(m->avctx, AV_LOG_WARNING,
               "Stream parameters not seen; skipping frame.\n");
        *got_frame_ptr = 0;
        return length;
    }

    substream_start = 0;

    for (substr = 0; substr < m->num_substreams; substr++) {
        int extraword_present, checkdata_present, end, nonrestart_substr;

        extraword_present = get_bits1(&gb);
        nonrestart_substr = get_bits1(&gb);
        checkdata_present = get_bits1(&gb);
        skip_bits1(&gb);

        end = get_bits(&gb, 12) * 2;

        substr_header_size += 2;

        if (extraword_present) {
            if (m->avctx->codec_id == AV_CODEC_ID_MLP) {
                av_log(m->avctx, AV_LOG_ERROR, "There must be no extraword for MLP.\n");
                goto error;
            }
            skip_bits(&gb, 16);
            substr_header_size += 2;
        }

        if (length < header_size + substr_header_size) {
            av_log(m->avctx, AV_LOG_ERROR, "Insufficient data for headers\n");
            goto error;
        }

        if (!(nonrestart_substr ^ m->is_major_sync_unit)) {
            av_log(m->avctx, AV_LOG_ERROR, "Invalid nonrestart_substr.\n");
            goto error;
        }

        if (end + header_size + substr_header_size > length) {
            av_log(m->avctx, AV_LOG_ERROR,
                   "Indicated length of substream %d data goes off end of "
                   "packet.\n", substr);
            end = length - header_size - substr_header_size;
        }

        if (end < substream_start) {
            av_log(avctx, AV_LOG_ERROR,
                   "Indicated end offset of substream %d data "
                   "is smaller than calculated start offset.\n",
                   substr);
            goto error;
        }

        if (substr > m->max_decoded_substream)
            continue;

        substream_parity_present[substr] = checkdata_present;
        substream_data_len[substr] = end - substream_start;
        substream_start = end;
    }

    parity_bits  = ff_mlp_calculate_parity(buf, 4);
    parity_bits ^= ff_mlp_calculate_parity(buf + header_size, substr_header_size);

    if ((((parity_bits >> 4) ^ parity_bits) & 0xF) != 0xF) {
        av_log(avctx, AV_LOG_ERROR, "Parity check failed.\n");
        goto error;
    }

    buf += header_size + substr_header_size;

    for (substr = 0; substr <= m->max_decoded_substream; substr++) {
        SubStream *s = &m->substream[substr];

        init_get_bits(&gb, buf, substream_data_len[substr] * 8);

        m->matrix_changed = 0;
        memset(m->filter_changed, 0, sizeof(m->filter_changed));

        s->blockpos = 0;
        do {
            if (get_bits1(&gb)) {
                if (get_bits1(&gb)) {
                    /* A restart header should be present. */
                    if (read_restart_header(m, &gb, buf, substr) < 0)
                        goto next_substr;
                    s->restart_seen = 1;
                }

                if (!s->restart_seen)
                    goto next_substr;
                if (read_decoding_params(m, &gb, substr) < 0)
                    goto next_substr;
            }

            if (!s->restart_seen)
                goto next_substr;

            if (((avctx->ch_layout.nb_channels == 6 &&
                  ((m->substream_info >> 2) & 0x3) != 0x3) ||
                 (avctx->ch_layout.nb_channels == 8 &&
                  ((m->substream_info >> 4) & 0x7) != 0x7 &&
                  ((m->substream_info >> 4) & 0x7) != 0x6 &&
                  ((m->substream_info >> 4) & 0x7) != 0x4 &&
                  ((m->substream_info >> 4) & 0x7) != 0x3)) &&
                substr > 0 && substr < m->max_decoded_substream &&
                (s->min_channel <= m->substream[substr - 1].max_channel)) {
                av_log(avctx, AV_LOG_DEBUG,
                       "Previous substream(%d) channels overlaps current substream(%d) channels, skipping.\n",
                       substr - 1, substr);
                goto next_substr;
            }

            if (substr != m->max_decoded_substream &&
                ((s->coded_channels & m->substream[m->max_decoded_substream].coded_channels) != 0)) {
                av_log(avctx, AV_LOG_DEBUG,
                       "Current substream(%d) channels overlaps final substream(%d) channels, skipping.\n",
                       substr, m->max_decoded_substream);
                goto next_substr;
            }

            if ((ret = read_block_data(m, &gb, substr)) < 0)
                return ret;

            if (get_bits_count(&gb) >= substream_data_len[substr] * 8)
                goto substream_length_mismatch;

        } while (!get_bits1(&gb));

        skip_bits(&gb, (-get_bits_count(&gb)) & 15);

        if (substream_data_len[substr] * 8 - get_bits_count(&gb) >= 32) {
            int shorten_by;

            if (get_bits(&gb, 16) != 0xD234)
                return AVERROR_INVALIDDATA;

            shorten_by = get_bits(&gb, 16);
            if      (m->avctx->codec_id == AV_CODEC_ID_TRUEHD && shorten_by  & 0x2000)
                s->blockpos -= FFMIN(shorten_by & 0x1FFF, s->blockpos);
            else if (m->avctx->codec_id == AV_CODEC_ID_MLP    && shorten_by != 0xD234)
                return AVERROR_INVALIDDATA;

            av_log(m->avctx, AV_LOG_DEBUG, "End of stream indicated.\n");
            s->end_of_stream = 1;
        }

        if (substream_parity_present[substr]) {
            uint8_t parity, checksum;

            if (substream_data_len[substr] * 8 - get_bits_count(&gb) != 16)
                goto substream_length_mismatch;

            parity   = ff_mlp_calculate_parity(buf, substream_data_len[substr] - 2);
            checksum = ff_mlp_checksum8       (buf, substream_data_len[substr] - 2);

            if ((get_bits(&gb, 8) ^ parity) != 0xa9    )
                av_log(m->avctx, AV_LOG_ERROR, "Substream %d parity check failed.\n", substr);
            if ( get_bits(&gb, 8)           != checksum)
                av_log(m->avctx, AV_LOG_ERROR, "Substream %d checksum failed.\n"    , substr);
        }

        if (substream_data_len[substr] * 8 != get_bits_count(&gb))
            goto substream_length_mismatch;

next_substr:
        if (!s->restart_seen)
            av_log(m->avctx, AV_LOG_ERROR,
                   "No restart header present in substream %d.\n", substr);

        buf += substream_data_len[substr];
    }

    if ((ret = output_data(m, m->max_decoded_substream, frame, got_frame_ptr)) < 0)
        return ret;

    for (substr = 0; substr <= m->max_decoded_substream; substr++){
        SubStream *s = &m->substream[substr];

        if (s->end_of_stream) {
            s->lossless_check_data = 0xffffffff;
            s->end_of_stream = 0;
            m->params_valid = 0;
        }
    }

    return length;

substream_length_mismatch:
    av_log(m->avctx, AV_LOG_ERROR, "substream %d length mismatch\n", substr);
    return AVERROR_INVALIDDATA;

error:
    m->params_valid = 0;
    return AVERROR_INVALIDDATA;
}

static void mlp_decode_flush(AVCodecContext *avctx)
{
    MLPDecodeContext *m = avctx->priv_data;

    m->params_valid = 0;
    for (int substr = 0; substr <= m->max_decoded_substream; substr++){
        SubStream *s = &m->substream[substr];

        s->lossless_check_data = 0xffffffff;
        s->prev_matrix_encoding = 0;
    }
}

#define OFFSET(x) offsetof(MLPDecodeContext, x)
#define FLAGS (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_AUDIO_PARAM)
static const AVOption options[] = {
    { "downmix", "Request a specific channel layout from the decoder",
        OFFSET(downmix_layout), AV_OPT_TYPE_CHLAYOUT, {.str = NULL}, .flags = FLAGS },
    { "extract_objects", "Enable extraction of audio object channels",
        OFFSET(extract_objects), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, .flags = FLAGS },
    { NULL },
};

static const AVClass mlp_decoder_class = {
    .class_name = "MLP decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVClass truehd_decoder_class = {
    .class_name = "TrueHD decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_MLP_DECODER
const FFCodec ff_mlp_decoder = {
    .p.name         = "mlp",
    CODEC_LONG_NAME("MLP (Meridian Lossless Packing)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_MLP,
    .priv_data_size = sizeof(MLPDecodeContext),
    .p.priv_class   = &mlp_decoder_class,
    .init           = mlp_decode_init,
    FF_CODEC_DECODE_CB(read_access_unit),
    .flush          = mlp_decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
};
#endif
#if CONFIG_TRUEHD_DECODER
const FFCodec ff_truehd_decoder = {
    .p.name         = "truehd",
    CODEC_LONG_NAME("TrueHD"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_TRUEHD,
    .priv_data_size = sizeof(MLPDecodeContext),
    .p.priv_class   = &truehd_decoder_class,
    .init           = mlp_decode_init,
    FF_CODEC_DECODE_CB(read_access_unit),
    .flush          = mlp_decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_truehd_profiles),
};
#endif /* CONFIG_TRUEHD_DECODER */
