/*
 * E-AC-3 encoder
 * Copyright (c) 2011 Justin Ruggles <justin.ruggles@gmail.com>
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
 * E-AC-3 encoder
 */

#define AC3ENC_FLOAT 1

#include "libavutil/attributes.h"
#include "libavutil/thread.h"
#include "ac3enc.h"
#include "codec_internal.h"
#include "eac3enc.h"
#include "eac3_data.h"
#include "put_bits.h"


static const AVClass eac3enc_class = {
    .class_name = "E-AC-3 Encoder",
    .option     = &ff_ac3_enc_options[2], /* First two options are AC-3 only. */
    .version    = LIBAVUTIL_VERSION_INT,
};

/**
 * LUT for finding a matching frame exponent strategy index from a set of
 * exponent strategies for a single channel across all 6 blocks.
 */
static int8_t eac3_frame_expstr_index_tab[3][4][4][4][4][4];


/**
 * Initialize E-AC-3 exponent tables.
 */
static av_cold void eac3_exponent_init(void)
{
    int i;

    memset(eac3_frame_expstr_index_tab, -1, sizeof(eac3_frame_expstr_index_tab));
    for (i = 0; i < 32; i++) {
        eac3_frame_expstr_index_tab[ff_eac3_frm_expstr[i][0]-1]
                                   [ff_eac3_frm_expstr[i][1]]
                                   [ff_eac3_frm_expstr[i][2]]
                                   [ff_eac3_frm_expstr[i][3]]
                                   [ff_eac3_frm_expstr[i][4]]
                                   [ff_eac3_frm_expstr[i][5]] = i;
    }
}


void ff_eac3_get_frame_exp_strategy(AC3EncodeContext *s)
{
    int ch;

    if (s->num_blocks < 6) {
        s->use_frame_exp_strategy = 0;
        return;
    }

    s->use_frame_exp_strategy = 1;
    for (ch = !s->cpl_on; ch <= s->fbw_channels; ch++) {
        int expstr = eac3_frame_expstr_index_tab[s->exp_strategy[ch][0]-1]
                                                [s->exp_strategy[ch][1]]
                                                [s->exp_strategy[ch][2]]
                                                [s->exp_strategy[ch][3]]
                                                [s->exp_strategy[ch][4]]
                                                [s->exp_strategy[ch][5]];
        if (expstr < 0) {
            s->use_frame_exp_strategy = 0;
            break;
        }
        s->frame_exp_strategy[ch] = expstr;
    }
}



void ff_eac3_set_cpl_states(AC3EncodeContext *s)
{
    int ch, blk;
    int first_cpl_coords[AC3_MAX_CHANNELS];

    /* set first cpl coords */
    for (ch = 1; ch <= s->fbw_channels; ch++)
        first_cpl_coords[ch] = 1;
    for (blk = 0; blk < s->num_blocks; blk++) {
        AC3Block *block = &s->blocks[blk];
        for (ch = 1; ch <= s->fbw_channels; ch++) {
            if (block->channel_in_cpl[ch]) {
                if (first_cpl_coords[ch]) {
                    block->new_cpl_coords[ch] = 2;
                    first_cpl_coords[ch]  = 0;
                }
            } else {
                first_cpl_coords[ch] = 1;
            }
        }
    }

    /* set first cpl leak */
    for (blk = 0; blk < s->num_blocks; blk++) {
        AC3Block *block = &s->blocks[blk];
        if (block->cpl_in_use) {
            block->new_cpl_leak = 2;
            break;
        }
    }
}

/**
 * Write the E-AC-3 frame header to the output bitstream.
 */
static void eac3_output_frame_header(AC3EncodeContext *s, PutBitContext *pb)
{
    int blk, ch;
    AC3EncOptions *opt = &s->options;

    put_bits_assume_flushed(pb);

    put_bits(pb, 16, 0x0b77);                   /* sync word */

    /* BSI header */
    put_bits(pb,  2, 0);                        /* stream type = independent */
    put_bits(pb,  3, 0);                        /* substream id = 0 */
    put_bits(pb, 11, (s->frame_size / 2) - 1);  /* frame size */
    put_bits(pb, 2, s->bit_alloc.sr_code);      /* sample rate code */
    put_bits(pb, 2, s->num_blks_code);          /* number of blocks */
    put_bits(pb, 3, s->channel_mode);           /* audio coding mode */
    put_bits(pb, 1, s->lfe_on);                 /* LFE channel indicator */
    put_bits(pb, 5, s->bitstream_id);           /* bitstream id (EAC3=16) */
    put_bits(pb, 5, -opt->dialogue_level);      /* dialogue normalization level */
    put_bits(pb, 1, 0);                         /* no compression gain */
    /* mixing metadata*/
    put_bits(pb, 1, opt->eac3_mixing_metadata);
    if (opt->eac3_mixing_metadata) {
        if (s->channel_mode > AC3_CHMODE_STEREO)
            put_bits(pb, 2, opt->preferred_stereo_downmix);
        if (s->has_center) {
            put_bits(pb, 3, s->ltrt_center_mix_level);
            put_bits(pb, 3, s->loro_center_mix_level);
        }
        if (s->has_surround) {
            put_bits(pb, 3, s->ltrt_surround_mix_level);
            put_bits(pb, 3, s->loro_surround_mix_level);
        }
        if (s->lfe_on)
            put_bits(pb, 1, 0);
        put_bits(pb, 1, 0);                     /* no program scale */
        put_bits(pb, 1, 0);                     /* no ext program scale */
        put_bits(pb, 2, 0);                     /* no mixing parameters */
        if (s->channel_mode < AC3_CHMODE_STEREO)
            put_bits(pb, 1, 0);                 /* no pan info */
        put_bits(pb, 1, 0);                     /* no frame mix config info */
    }
    /* info metadata*/
    put_bits(pb, 1, opt->eac3_info_metadata);
    if (opt->eac3_info_metadata) {
        put_bits(pb, 3, s->bitstream_mode);
        put_bits(pb, 1, opt->copyright);
        put_bits(pb, 1, opt->original);
        if (s->channel_mode == AC3_CHMODE_STEREO) {
            put_bits(pb, 2, opt->dolby_surround_mode);
            put_bits(pb, 2, opt->dolby_headphone_mode);
        }
        if (s->channel_mode >= AC3_CHMODE_2F2R)
            put_bits(pb, 2, opt->dolby_surround_ex_mode);
        put_bits(pb, 1, opt->audio_production_info);
        if (opt->audio_production_info) {
            put_bits(pb, 5, opt->mixing_level - 80);
            put_bits(pb, 2, opt->room_type);
            put_bits(pb, 1, opt->ad_converter_type);
        }
        put_bits(pb, 1, 0);
    }
    if (s->num_blocks != 6)
        put_bits(pb, 1, !(s->avctx->frame_num % 6)); /* converter sync flag */
    put_bits(pb, 1, 0);                         /* no additional bit stream info */

    /* frame header */
    if (s->num_blocks == 6) {
        put_bits(pb, 1, !s->use_frame_exp_strategy); /* exponent strategy syntax */
        put_bits(pb, 1, 0);                     /* aht enabled = no */
    }
    put_bits(pb, 2, 0);                         /* snr offset strategy = 1 */
    put_bits(pb, 1, 0);                         /* transient pre-noise processing enabled = no */
    put_bits(pb, 1, 0);                         /* block switch syntax enabled = no */
    put_bits(pb, 1, 0);                         /* dither flag syntax enabled = no */
    put_bits(pb, 1, 0);                         /* bit allocation model syntax enabled = no */
    put_bits(pb, 1, 0);                         /* fast gain codes enabled = no */
    put_bits(pb, 1, 0);                         /* dba syntax enabled = no */
    put_bits(pb, 1, 0);                         /* skip field syntax enabled = no */
    put_bits(pb, 1, 0);                         /* spx enabled = no */
    /* coupling strategy use flags */
    if (s->channel_mode > AC3_CHMODE_MONO) {
        put_bits(pb, 1, s->blocks[0].cpl_in_use);
        for (blk = 1; blk < s->num_blocks; blk++) {
            AC3Block *block = &s->blocks[blk];
            put_bits(pb, 1, block->new_cpl_strategy);
            if (block->new_cpl_strategy)
                put_bits(pb, 1, block->cpl_in_use);
        }
    }
    /* exponent strategy */
    if (s->use_frame_exp_strategy) {
        for (ch = !s->cpl_on; ch <= s->fbw_channels; ch++)
            put_bits(pb, 5, s->frame_exp_strategy[ch]);
    } else {
        for (blk = 0; blk < s->num_blocks; blk++)
            for (ch = !s->blocks[blk].cpl_in_use; ch <= s->fbw_channels; ch++)
                put_bits(pb, 2, s->exp_strategy[ch][blk]);
    }
    if (s->lfe_on) {
        for (blk = 0; blk < s->num_blocks; blk++)
            put_bits(pb, 1, s->exp_strategy[s->lfe_channel][blk]);
    }
    /* E-AC-3 to AC-3 converter exponent strategy (not optional when num blocks == 6) */
    if (s->num_blocks != 6) {
        put_bits(pb, 1, 0);
    } else {
        for (ch = 1; ch <= s->fbw_channels; ch++) {
            if (s->use_frame_exp_strategy)
                put_bits(pb, 5, s->frame_exp_strategy[ch]);
            else
                put_bits(pb, 5, 0);
        }
    }
    /* snr offsets */
    put_bits(pb, 6, s->coarse_snr_offset);
    put_bits(pb, 4, s->fine_snr_offset[1]);
    /* block start info */
    if (s->num_blocks > 1)
        put_bits(pb, 1, 0);
}

static av_cold int eac3_encode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    AC3EncodeContext *s = avctx->priv_data;

    s->eac3 = 1;
    s->output_frame_header = eac3_output_frame_header;

    ff_thread_once(&init_static_once, eac3_exponent_init);

    return ff_ac3_float_encode_init(avctx);
}

const FFCodec ff_eac3_encoder = {
    .p.name          = "eac3",
    CODEC_LONG_NAME("ATSC A/52 E-AC-3"),
    .p.type          = AVMEDIA_TYPE_AUDIO,
    .p.id            = AV_CODEC_ID_EAC3,
    .p.capabilities  = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size  = sizeof(AC3EncodeContext),
    .init            = eac3_encode_init,
    FF_CODEC_ENCODE_CB(ff_ac3_encode_frame),
    .close           = ff_ac3_encode_close,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
    .p.priv_class    = &eac3enc_class,
    CODEC_SAMPLERATES_ARRAY(ff_ac3_sample_rate_tab),
    CODEC_CH_LAYOUTS_ARRAY(ff_ac3_ch_layouts),
    .defaults        = ff_ac3_enc_defaults,
    .caps_internal   = FF_CODEC_CAP_INIT_CLEANUP,
};
