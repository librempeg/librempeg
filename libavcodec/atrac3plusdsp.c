/*
 * ATRAC3+ compatible decoder
 *
 * Copyright (c) 2010-2013 Maxim Poliakovski
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
 *  @file
 *  DSP functions for ATRAC3+ decoder.
 */

#include <math.h>

#include "libavutil/float_dsp.h"
#include "libavutil/libm.h"
#include "libavutil/mem_internal.h"

#include "sinewin.h"
#include "atrac3plus.h"

/**
 *  Map quant unit number to its position in the spectrum.
 *  To get the number of spectral lines in each quant unit do the following:
 *  num_specs = qu_to_spec_pos[i+1] - qu_to_spec_pos[i]
 */
const uint16_t ff_atrac3p_qu_to_spec_pos[33] = {
      0,    16,   32,   48,   64,   80,   96,  112,
    128,   160,  192,  224,  256,  288,  320,  352,
    384,   448,  512,  576,  640,  704,  768,  896,
    1024, 1152, 1280, 1408, 1536, 1664, 1792, 1920,
    2048
};

/* Scalefactors table. */
/* Approx. Equ: pow(2.0, (i - 16.0 + 0.501783948) / 3.0) */
const float ff_atrac3p_sf_tab[64] = {
    0.027852058,  0.0350914, 0.044212341, 0.055704117,  0.0701828,
    0.088424683, 0.11140823,   0.1403656,  0.17684937, 0.22281647, 0.2807312, 0.35369873,
    0.44563293,   0.5614624,  0.70739746,  0.89126587,  1.1229248, 1.4147949,  1.7825317,
    2.2458496,    2.8295898,   3.5650635,   4.4916992,  5.6591797,  7.130127,  8.9833984,
    11.318359,    14.260254,   17.966797,   22.636719,  28.520508, 35.933594,  45.273438,
    57.041016,    71.867188,   90.546875,   114.08203,  143.73438, 181.09375,  228.16406,
    287.46875,     362.1875,   456.32812,    574.9375,    724.375, 912.65625,   1149.875,
    1448.75,      1825.3125,     2299.75,      2897.5,   3650.625,    4599.5,     5795.0,
    7301.25,         9199.0,     11590.0,     14602.5,    18398.0,   23180.0,    29205.0,
    36796.0,        46360.0,     58410.0
};

/* Mantissa table. */
/* pow(10, x * log10(2) + 0.05) / 2 / ([1,2,3,5,7,15,31] + 0.5) */
const float ff_atrac3p_mant_tab[8] = {
    0.0,
    0.74801636,
    0.44882202,
    0.32058716,
    0.20400238,
    0.1496048,
    0.07239151,
    0.035619736
};

#define ATRAC3P_MDCT_SIZE (ATRAC3P_SUBBAND_SAMPLES * 2)

#define TWOPI (2 * M_PI)

#define DEQUANT_PHASE(ph) (((ph) & 0x1F) << 6)

static DECLARE_ALIGNED(32, float, sine_table)[2048]; ///< wave table
static DECLARE_ALIGNED(32, float, hann_window)[256]; ///< Hann windowing function
static float amp_sf_tab[64];   ///< scalefactors for quantized amplitudes

av_cold void ff_atrac3p_init_dsp_static(void)
{
    int i;

    /* generate sine wave table */
    for (i = 0; i < 2048; i++)
        sine_table[i] = sin(TWOPI * i / 2048);

    /* generate Hann window */
    for (i = 0; i < 256; i++)
        hann_window[i] = (1.0f - cos(TWOPI * i / 256.0f)) * 0.5f;

    /* generate amplitude scalefactors table */
    for (i = 0; i < 64; i++)
        amp_sf_tab[i] = exp2f((i - 3) / 4.0f);

    ff_init_ff_sine_windows(7);
    ff_init_ff_sine_windows(6);
}

/**
 *  Synthesize sine waves according to given parameters.
 *
 *  @param[in]    synth_param   ptr to common synthesis parameters
 *  @param[in]    waves_info    parameters for each sine wave
 *  @param[in]    envelope      envelope data for all waves in a group
 *  @param[in]    fdsp          ptr to floating-point DSP context
 *  @param[in]    invert_phase  flag indicating 180° phase shift
 *  @param[in]    reg_offset    region offset for trimming envelope data
 *  @param[out]   out           receives sythesized data
 */
static void waves_synth(Atrac3pWaveSynthParams *synth_param,
                        Atrac3pWavesData *waves_info,
                        Atrac3pWaveEnvelope *envelope,
                        AVFloatDSPContext *fdsp,
                        int invert_phase, int reg_offset, float *out)
{
    int i, wn, inc, pos;
    double amp;
    Atrac3pWaveParam *wave_param = &synth_param->waves[waves_info->start_index];

    for (wn = 0; wn < waves_info->num_wavs; wn++, wave_param++) {
        /* amplitude dequantization */
        amp = amp_sf_tab[wave_param->amp_sf] *
              (!synth_param->amplitude_mode
               ? (wave_param->amp_index + 1) / 15.13f
               : 1.0f);

        inc = wave_param->freq_index;
        pos = DEQUANT_PHASE(wave_param->phase_index) - (reg_offset ^ 128) * inc & 2047;

        /* waveform generation */
        for (i = 0; i < 128; i++) {
            out[i] += sine_table[pos] * amp;
            pos     = (pos + inc) & 2047;
        }
    }

    /* invert phase if requested */
    if (invert_phase)
        fdsp->vector_fmul_scalar(out, out, -1.0f, 128);

    /* fade in with steep Hann window if requested */
    if (envelope->has_start_point) {
        pos = (envelope->start_pos << 2) - reg_offset;
        if (pos > 0 && pos <= 128) {
            memset(out, 0, pos * sizeof(*out));
            if (!envelope->has_stop_point ||
                envelope->start_pos != envelope->stop_pos) {
                out[pos + 0] *= hann_window[0];
                out[pos + 1] *= hann_window[32];
                out[pos + 2] *= hann_window[64];
                out[pos + 3] *= hann_window[96];
            }
        }
    }

    /* fade out with steep Hann window if requested */
    if (envelope->has_stop_point) {
        pos = (envelope->stop_pos + 1 << 2) - reg_offset;
        if (pos > 0 && pos <= 128) {
            out[pos - 4] *= hann_window[96];
            out[pos - 3] *= hann_window[64];
            out[pos - 2] *= hann_window[32];
            out[pos - 1] *= hann_window[0];
            memset(&out[pos], 0, (128 - pos) * sizeof(out[pos]));
        }
    }
}

void ff_atrac3p_generate_tones(Atrac3pChanUnitCtx *ch_unit, AVFloatDSPContext *fdsp,
                               int ch_num, int sb, float *out)
{
    DECLARE_ALIGNED(32, float, wavreg1)[128] = { 0 };
    DECLARE_ALIGNED(32, float, wavreg2)[128] = { 0 };
    int i, reg1_env_nonzero, reg2_env_nonzero;
    Atrac3pWavesData *tones_now  = &ch_unit->channels[ch_num].tones_info_prev[sb];
    Atrac3pWavesData *tones_next = &ch_unit->channels[ch_num].tones_info[sb];

    /* reconstruct full envelopes for both overlapping regions
     * from truncated bitstream data */
    if (tones_next->pend_env.has_start_point &&
        tones_next->pend_env.start_pos < tones_next->pend_env.stop_pos) {
        tones_next->curr_env.has_start_point = 1;
        tones_next->curr_env.start_pos       = tones_next->pend_env.start_pos + 32;
    } else if (tones_now->pend_env.has_start_point) {
        tones_next->curr_env.has_start_point = 1;
        tones_next->curr_env.start_pos       = tones_now->pend_env.start_pos;
    } else {
        tones_next->curr_env.has_start_point = 0;
        tones_next->curr_env.start_pos       = 0;
    }

    if (tones_now->pend_env.has_stop_point &&
        tones_now->pend_env.stop_pos >= tones_next->curr_env.start_pos) {
        tones_next->curr_env.has_stop_point = 1;
        tones_next->curr_env.stop_pos       = tones_now->pend_env.stop_pos;
    } else if (tones_next->pend_env.has_stop_point) {
        tones_next->curr_env.has_stop_point = 1;
        tones_next->curr_env.stop_pos       = tones_next->pend_env.stop_pos + 32;
    } else {
        tones_next->curr_env.has_stop_point = 0;
        tones_next->curr_env.stop_pos       = 64;
    }

    /* is the visible part of the envelope non-zero? */
    reg1_env_nonzero = (tones_now->curr_env.stop_pos    < 32) ? 0 : 1;
    reg2_env_nonzero = (tones_next->curr_env.start_pos >= 32) ? 0 : 1;

    /* synthesize waves for both overlapping regions */
    if (tones_now->num_wavs && reg1_env_nonzero)
        waves_synth(ch_unit->waves_info_prev, tones_now, &tones_now->curr_env,
                    fdsp, ch_unit->waves_info_prev->invert_phase[sb] & ch_num,
                    128, wavreg1);

    if (tones_next->num_wavs && reg2_env_nonzero)
        waves_synth(ch_unit->waves_info, tones_next, &tones_next->curr_env, fdsp,
                    ch_unit->waves_info->invert_phase[sb] & ch_num, 0, wavreg2);

    /* Hann windowing for non-faded wave signals */
    if (tones_now->num_wavs && tones_next->num_wavs &&
        reg1_env_nonzero && reg2_env_nonzero) {
        fdsp->vector_fmul(wavreg1, wavreg1, &hann_window[128], 128);
        fdsp->vector_fmul(wavreg2, wavreg2,  hann_window,      128);
    } else {
        if (tones_now->num_wavs && !tones_now->curr_env.has_stop_point)
            fdsp->vector_fmul(wavreg1, wavreg1, &hann_window[128], 128);

        if (tones_next->num_wavs && !tones_next->curr_env.has_start_point)
            fdsp->vector_fmul(wavreg2, wavreg2, hann_window, 128);
    }

    /* Overlap and add to residual */
    for (i = 0; i < 128; i++)
        out[i] += wavreg1[i] + wavreg2[i];
}

static const uint8_t subband_to_powgrp[ATRAC3P_SUBBANDS] = {
    0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4
};

/* noise table for power compensation */
static const float noise_tab[1024] = {
    -0.01358032,  -0.05593872,   0.01696777,  -0.14871216,  -0.26412964,  -0.09893799,   0.25723267,
     0.02008057,  -0.72235107,  -0.44351196,  -0.22985840,   0.16833496,   0.46902466,   0.05917358,
    -0.15179443,   0.41299438,  -0.01287842,   0.13360596,   0.43557739,  -0.09530640,  -0.58422852,
     0.39266968,  -0.08343506,  -0.25604248,   0.22848511,   0.26013184,  -0.65588379,   0.17288208,
    -0.08673096,  -0.05203247,   0.07299805,  -0.28665161,  -0.35806274,   0.06552124,  -0.09387207,
     0.21099854,  -0.28347778,  -0.72402954,   0.05050659,  -0.10635376,  -0.18853760,   0.29724121,
     0.20703125,  -0.29791260,  -0.37634277,   0.47970581,  -0.09976196,   0.32641602,  -0.29248047,
    -0.28237915,   0.26028442,  -0.36157227,   0.22042847,  -0.03222656,  -0.37268066,  -0.03759766,
     0.09909058,   0.23284912,   0.19320679,   0.14453125,  -0.02139282,  -0.19702148,   0.31533813,
    -0.16741943,   0.35031128,  -0.35656738,  -0.66128540,  -0.00701904,   0.20898438,   0.26837158,
    -0.33706665,  -0.04568481,   0.12600708,   0.10284424,   0.07321167,  -0.18280029,   0.38101196,
     0.21301270,   0.04541016,   0.01156616,  -0.26391602,  -0.02346802,  -0.22125244,   0.29760742,
    -0.36233521,  -0.31314087,  -0.13967896,  -0.11276245,  -0.19433594,   0.34490967,   0.02343750,
     0.21963501,  -0.02777100,  -0.67678833,  -0.08999634,   0.14233398,  -0.27697754,   0.51422119,
    -0.05047607,   0.48327637,   0.37167358,  -0.60806274,   0.18728638,  -0.15191650,   0.00637817,
     0.02832031,  -0.15618896,   0.60644531,   0.21826172,   0.06384277,  -0.31863403,   0.08816528,
     0.15447998,  -0.07015991,  -0.08154297,  -0.40966797,  -0.39785767,  -0.11709595,   0.22052002,
     0.18466187,  -0.17257690,   0.03759766,  -0.06195068,   0.00433350,   0.12176514,   0.34011841,
     0.25610352,  -0.05294800,   0.41033936,   0.16854858,  -0.76187134,   0.13845825,  -0.19418335,
    -0.21524048,  -0.44412231,  -0.08160400,  -0.28195190,  -0.01873779,   0.15524292,  -0.37438965,
    -0.44860840,   0.43096924,  -0.24746704,   0.49856567,   0.14859009,   0.38159180,   0.20541382,
    -0.39175415,  -0.65850830,  -0.43716431,   0.13037109,  -0.05111694,   0.39956665,   0.21447754,
    -0.04861450,   0.33654785,   0.10589600,  -0.88085938,  -0.30822754,   0.38577271,   0.30047607,
     0.38836670,   0.09118652,  -0.36477661,  -0.01641846,  -0.23031616,   0.26058960,   0.18859863,
    -0.21868896,  -0.17861938,  -0.29754639,   0.09777832,   0.10806274,  -0.51605225,   0.00076294,
     0.13259888,   0.11090088,  -0.24084473,   0.24957275,   0.01379395,  -0.04141235,  -0.04937744,
     0.57394409,   0.27410889,   0.27587891,   0.45013428,  -0.32592773,   0.11160278,  -0.00970459,
     0.29092407,   0.03356934,  -0.70925903,   0.04882812,   0.43499756,   0.07720947,  -0.27554321,
    -0.01742554,  -0.08413696,  -0.04028320,  -0.52850342,  -0.07330322,   0.05181885,   0.21362305,
    -0.18765259,   0.07058716,  -0.03009033,   0.32662964,   0.27023315,  -0.28002930,   0.17568970,
     0.03338623,   0.30242920,  -0.03921509,   0.32174683,  -0.23733521,   0.08575439,  -0.38269043,
     0.09194946,  -0.07238770,   0.17941284,  -0.51278687,  -0.25146484,   0.19790649,  -0.19195557,
     0.16549683,   0.42456055,   0.39129639,  -0.02868652,   0.17980957,   0.24902344,  -0.76583862,
    -0.20959473,   0.61013794,   0.37011719,   0.36859131,  -0.04486084,   0.10678101,  -0.15994263,
    -0.05328369,   0.28463745,  -0.06420898,  -0.36987305,  -0.28009033,  -0.11764526,   0.04312134,
    -0.08038330,   0.04885864,  -0.03067017,  -0.00042725,   0.34289551,  -0.00988770,   0.34838867,
     0.32516479,  -0.16271973,   0.38269043,   0.03240967,   0.12417603,  -0.14331055,  -0.34902954,
    -0.18325806,   0.29421997,   0.44284058,   0.75170898,  -0.67245483,  -0.12176514,   0.27914429,
    -0.29806519,   0.19863892,   0.30087280,   0.22680664,  -0.36633301,  -0.32534790,  -0.57553101,
    -0.16641235,   0.43811035,   0.08331299,   0.15942383,   0.26516724,  -0.24240112,  -0.11761475,
    -0.16827393,  -0.14260864,   0.46343994,   0.11804199,  -0.55514526,  -0.02520752,  -0.14309692,
     0.00448608,   0.02749634,  -0.30545044,   0.70965576,   0.45108032,   0.66439819,  -0.68255615,
    -0.12496948,   0.09146118,  -0.21109009,  -0.23791504,   0.79943848,  -0.35205078,  -0.24963379,
     0.18719482,  -0.19079590,   0.07458496,   0.07623291,  -0.28781128,  -0.37121582,  -0.19580078,
    -0.01773071,  -0.16717529,   0.13040161,   0.14672852,   0.42379761,   0.03582764,   0.11431885,
     0.05145264,   0.44702148,   0.08963013,   0.01367188,  -0.54519653,  -0.12692261,   0.21176147,
     0.04925537,   0.30670166,  -0.11029053,   0.19555664,  -0.27740479,   0.23043823,   0.15554810,
    -0.19299316,  -0.25729370,   0.17800903,  -0.03579712,  -0.05065918,  -0.06933594,  -0.09500122,
    -0.07821655,   0.23889160,  -0.31900024,   0.03073120,  -0.00415039,   0.61315918,   0.37176514,
    -0.13442993,  -0.15536499,  -0.19216919,  -0.37899780,   0.19992065,   0.02630615,  -0.12573242,
     0.25927734,  -0.02447510,   0.29629517,  -0.40731812,  -0.17333984,   0.24310303,  -0.10607910,
     0.14828491,   0.08792114,  -0.18743896,  -0.05572510,  -0.04833984,   0.10473633,  -0.29028320,
    -0.67687988,  -0.28170776,  -0.41687012,   0.05413818,  -0.23284912,   0.09555054,  -0.08969116,
    -0.15112305,   0.12738037,   0.35986328,   0.28948975,   0.30691528,   0.23956299,   0.06973267,
    -0.31198120,  -0.18450928,   0.22280884,  -0.21600342,   0.23522949,  -0.61840820,  -0.13012695,
     0.26412964,   0.47320557,  -0.26440430,   0.38757324,   0.17352295,  -0.26104736,  -0.25866699,
    -0.12274170,  -0.29733276,   0.07687378,   0.18588257,  -0.08880615,   0.31185913,   0.05313110,
    -0.10885620,  -0.14901733,  -0.22323608,  -0.08538818,   0.19812012,   0.19732666,  -0.18927002,
     0.29058838,   0.25555420,  -0.48599243,   0.18768311,   0.01345825,   0.34887695,   0.21530151,
     0.19857788,   0.18661499,  -0.01394653,  -0.09063721,  -0.38781738,   0.27160645,  -0.20379639,
    -0.32119751,  -0.23889160,   0.27096558,   0.24951172,   0.07922363,   0.07479858,  -0.50946045,
     0.10220337,   0.58364868,  -0.19503784,  -0.18560791,  -0.01165771,   0.47195435,   0.22430420,
    -0.38635254,  -0.03732300,  -0.09179688,   0.06991577,   0.15106201,   0.20605469,  -0.05969238,
    -0.41821289,   0.12231445,  -0.04672241,  -0.05117798,  -0.11523438,  -0.51849365,  -0.04077148,
     0.44284058,  -0.64086914,   0.17019653,   0.02236938,   0.22848511,  -0.23214722,  -0.32354736,
    -0.14068604,  -0.29690552,  -0.19891357,   0.02774048,  -0.20965576,  -0.52191162,  -0.19299316,
    -0.07290649,   0.49053955,  -0.22302246,   0.05642700,   0.13122559,  -0.20819092,  -0.83590698,
    -0.08181763,   0.26797485,  -0.00091553,  -0.09457397,   0.17089844,  -0.27020264,   0.30270386,
     0.05496216,   0.09564209,  -0.08590698,   0.02130127,   0.35931396,   0.21728516,  -0.15396118,
    -0.05053711,   0.02719116,   0.16302490,   0.43212891,   0.10229492,  -0.40820312,   0.21646118,
     0.08435059,  -0.11145020,  -0.39962769,  -0.05618286,  -0.10223389,  -0.60839844,   0.33724976,
    -0.06341553,  -0.47369385,  -0.32852173,   0.05242920,   0.19635010,  -0.19137573,  -0.67901611,
     0.16180420,   0.05133057,  -0.22283936,   0.09646606,   0.24288940,  -0.45007324,   0.08804321,
     0.14053345,   0.22619629,  -0.01000977,   0.36355591,  -0.19863892,  -0.30364990,  -0.24118042,
    -0.57461548,   0.26498413,   0.04345703,  -0.09796143,  -0.47714233,  -0.23739624,   0.18737793,
     0.08926392,  -0.02795410,   0.00305176,  -0.08700562,  -0.38711548,   0.03222656,   0.10940552,
    -0.41906738,  -0.01620483,  -0.47061157,   0.37985229,  -0.21624756,   0.47976685,  -0.20046997,
    -0.62533569,  -0.26907349,  -0.02877808,   0.00671387,  -0.29071045,  -0.24685669,  -0.15722656,
    -0.26055908,   0.29968262,   0.28225708,  -0.08990479,  -0.16748047,  -0.46759033,  -0.25067139,
    -0.25183105,  -0.45932007,   0.05828857,   0.29006958,   0.23840332,  -0.17974854,   0.26931763,
     0.10696411,  -0.06848145,  -0.17126465,  -0.10522461,  -0.55386353,  -0.42306519,  -0.07608032,
     0.24380493,   0.38586426,   0.16882324,   0.26751709,   0.17303467,   0.35809326,  -0.22094727,
    -0.30703735,  -0.28497314,  -0.04321289,   0.15219116,  -0.17071533,  -0.39334106,   0.03439331,
    -0.10809326,  -0.30590820,   0.26449585,  -0.07412720,   0.13638306,  -0.01062012,   0.27996826,
     0.04397583,  -0.05557251,  -0.56933594,   0.03363037,  -0.00949097,   0.52642822,  -0.44329834,
     0.28308105,  -0.05499268,  -0.23312378,  -0.29870605,  -0.05123901,   0.26831055,  -0.35238647,
    -0.30993652,   0.34646606,  -0.19775391,   0.44595337,   0.13769531,   0.45358276,   0.19961548,
     0.42681885,   0.15722656,   0.00128174,   0.23757935,   0.40988159,   0.25164795,  -0.00732422,
    -0.12405396,  -0.43420410,  -0.00402832,   0.34243774,   0.36264038,   0.18807983,  -0.09301758,
    -0.10296631,   0.05532837,  -0.31652832,   0.14337158,   0.35040283,   0.32540894,   0.05728149,
    -0.12030029,  -0.25942993,  -0.20312500,  -0.16491699,  -0.46051025,  -0.08004761,   0.50772095,
     0.16168213,   0.28439331,   0.08105469,  -0.19104004,   0.38589478,  -0.16400146,  -0.25454712,
     0.20281982,  -0.20730591,  -0.06311035,   0.32937622,   0.15032959,  -0.05340576,   0.30487061,
    -0.11648560,   0.38009644,  -0.20062256,   0.43466187,   0.01150513,   0.35754395,  -0.13146973,
     0.67489624,   0.05212402,   0.27914429,  -0.39431763,   0.75308228,  -0.13366699,   0.24453735,
     0.42248535,  -0.65905762,  -0.00546265,  -0.03491211,  -0.13659668,  -0.08294678,  -0.45666504,
     0.27188110,   0.12731934,   0.61148071,   0.10449219,  -0.28836060,   0.00091553,   0.24618530,
     0.13119507,   0.05685425,   0.17355347,   0.42034912,   0.08514404,   0.24536133,   0.18951416,
    -0.19107056,  -0.15036011,   0.02334595,   0.54986572,   0.32321167,  -0.16104126,  -0.03054810,
     0.43594360,   0.17309570,   0.61053467,   0.24731445,   0.33334351,   0.15240479,   0.15588379,
     0.36425781,  -0.30407715,  -0.13302612,   0.00427246,   0.04171753,  -0.33178711,   0.34216309,
    -0.12463379,  -0.02764893,   0.05905151,  -0.31436157,   0.16531372,   0.34542847,  -0.03292847,
     0.12527466,  -0.12313843,  -0.13171387,   0.04757690,  -0.45095825,  -0.19085693,   0.35342407,
    -0.23239136,  -0.34387207,   0.11264038,  -0.15740967,   0.05273438,   0.74942017,   0.21505737,
     0.08514404,  -0.42391968,  -0.19531250,   0.35293579,   0.25305176,   0.15731812,  -0.70324707,
    -0.21591187,   0.35604858,   0.14132690,   0.11724854,   0.15853882,  -0.24597168,   0.07019043,
     0.02127075,   0.12658691,   0.06390381,  -0.12292480,   0.15441895,  -0.47640991,   0.06195068,
     0.58981323,  -0.15151978,  -0.03604126,  -0.45059204,  -0.01672363,  -0.46997070,   0.25750732,
     0.18084717,   0.06661987,   0.13253784,   0.67828369,   0.11370850,   0.11325073,  -0.04611206,
    -0.07791138,  -0.36544800,  -0.06747437,  -0.31594849,   0.16131592,   0.41983032,   0.11071777,
    -0.36889648,   0.30963135,  -0.37875366,   0.58508301,   0.00393677,   0.12338257,   0.03424072,
    -0.21728516,  -0.12838745,  -0.46981812,   0.05868530,  -0.25015259,   0.27407837,   0.65240479,
    -0.34429932,  -0.15179443,   0.14056396,   0.33505249,   0.28826904,   0.09921265,   0.34390259,
     0.13656616,  -0.23608398,   0.00863647,   0.02627563,  -0.19119263,   0.19775391,  -0.07214355,
     0.07809448,   0.03454590,  -0.03417969,   0.00033569,  -0.23095703,   0.18673706,   0.05798340,
     0.03814697,  -0.04318237,   0.05487061,   0.08633423,   0.55950928,  -0.06347656,   0.10333252,
     0.25305176,   0.05853271,   0.12246704,  -0.25543213,  -0.34262085,  -0.36437988,  -0.21304321,
    -0.05093384,   0.02777100,   0.07620239,  -0.21215820,  -0.09326172,   0.19021606,  -0.40579224,
    -0.01193237,   0.19845581,  -0.35336304,  -0.07397461,   0.20104980,   0.08615112,  -0.44375610,
     0.11419678,   0.24453735,  -0.16555786,  -0.05081177,  -0.01406860,   0.27893066,  -0.18692017,
     0.07473755,   0.03451538,  -0.39733887,   0.21548462,  -0.22534180,  -0.39651489,  -0.04989624,
    -0.57662964,   0.06390381,   0.62020874,  -0.13470459,   0.04345703,  -0.21862793,  -0.02789307,
     0.51696777,  -0.27587891,   0.39004517,   0.09857178,  -0.00738525,   0.31317139,   0.00048828,
    -0.46572876,   0.29531860,  -0.10009766,  -0.27856445,   0.03594971,   0.25048828,  -0.74584961,
    -0.25350952,  -0.03302002,   0.31188965,   0.01571655,   0.46710205,   0.21591187,   0.07260132,
    -0.42132568,  -0.53900146,  -0.13674927,  -0.16571045,  -0.34454346,   0.12359619,  -0.11184692,
     0.00967407,   0.34576416,  -0.05761719,   0.34848022,   0.17645264,  -0.39395142,   0.10339355,
     0.18215942,   0.20697021,   0.59109497,  -0.11560059,  -0.07385254,   0.10397339,   0.35437012,
    -0.22863770,   0.01794434,   0.17559814,  -0.17495728,   0.12142944,   0.10928345,  -1.00000000,
    -0.01379395,   0.21237183,  -0.27035522,   0.27319336,  -0.37066650,   0.41354370,  -0.40054321,
     0.00689697,   0.26321411,   0.39266968,   0.65298462,   0.41625977,  -0.13909912,   0.78375244,
    -0.30941772,   0.20169067,  -0.39367676,   0.94021606,  -0.24066162,   0.05557251,  -0.24533081,
    -0.05444336,  -0.76754761,  -0.19375610,  -0.11041260,  -0.17532349,   0.16006470,   0.02188110,
     0.17465210,  -0.04342651,  -0.56777954,  -0.40988159,   0.26687622,   0.11700439,  -0.00344849,
    -0.05395508,   0.37426758,  -0.40719604,  -0.15032959,  -0.01660156,   0.04196167,  -0.04559326,
    -0.12969971,   0.12011719,   0.08419800,  -0.11199951,   0.35174561,   0.10275269,  -0.25686646,
     0.48446655,   0.03225708,   0.28408813,  -0.18701172,   0.36282349,  -0.03280640,   0.32302856,
     0.17233276,   0.48269653,   0.31112671,  -0.04946899,   0.12774658,   0.52685547,   0.10211182,
     0.05953979,   0.05999756,   0.20144653,   0.00744629,   0.27316284,   0.24377441,   0.39672852,
     0.01702881,  -0.35513306,   0.11364746,  -0.13555908,   0.48880005,  -0.15417480,  -0.09149170,
    -0.02615356,   0.46246338,  -0.72250366,   0.22332764,   0.23849487,  -0.25686646,  -0.08514404,
    -0.02062988,  -0.34494019,  -0.02297974,  -0.80386353,  -0.08074951,  -0.12689209,  -0.06896973,
     0.24099731,  -0.35650635,  -0.09558105,   0.29254150,   0.23132324,  -0.16726685,   0.00000000,
    -0.24237061,   0.30899048,   0.29504395,  -0.20898438,   0.17059326,  -0.07672119,  -0.14395142,
     0.05572510,   0.20602417,  -0.51550293,  -0.03167725,  -0.48840332,  -0.20425415,   0.14144897,
     0.07275391,  -0.76669312,  -0.22488403,   0.20651245,   0.03259277,   0.00085449,   0.03039551,
     0.47555542,   0.38351440
};

/** Noise level table for power compensation.
 *  Equ: pow(2.0f, (double)(6 - i) / 3.0f) where i = 0...15 */
static const float pwc_levs[16] = {
    3.96875, 3.15625,     2.5,    2.0, 1.59375,   1.25,     1.0, 0.78125,
    0.625,       0.5, 0.40625, 0.3125,    0.25, 0.1875, 0.15625, 0.0
};

/** Map subband number to quant unit number. */
static const uint8_t subband_to_qu[17] = {
    0, 8, 12, 16, 18, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
};

void ff_atrac3p_power_compensation(Atrac3pChanUnitCtx *ctx, AVFloatDSPContext *fdsp,
                                   int ch_index, float *sp, int rng_index, int sb)
{
    AtracGainInfo *g1, *g2;
    LOCAL_ALIGNED_32(float, pwcsp, [ATRAC3P_SUBBAND_SAMPLES]);
    float *dst, grp_lev, qu_lev;
    int i, gain_lev, gcv = 0, qu, nsp;
    int swap_ch = (ctx->unit_type == CH_UNIT_STEREO && ctx->swap_channels[sb]) ? 1 : 0;

    if (ctx->channels[ch_index ^ swap_ch].power_levs[subband_to_powgrp[sb]] == ATRAC3P_POWER_COMP_OFF)
        return;

    /* generate initial noise spectrum */
    for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES; i++, rng_index++)
        pwcsp[i] = noise_tab[rng_index & 0x3FF];

    /* check gain control information */
    g1 = &ctx->channels[ch_index ^ swap_ch].gain_data[sb];
    g2 = &ctx->channels[ch_index ^ swap_ch].gain_data_prev[sb];

    gain_lev = (g1->num_points > 0) ? (6 - g1->lev_code[0]) : 0;

    for (i = 0; i < g2->num_points; i++)
        gcv = FFMAX(gcv, gain_lev - (g2->lev_code[i] - 6));

    for (i = 0; i < g1->num_points; i++)
        gcv = FFMAX(gcv, 6 - g1->lev_code[i]);

    grp_lev = pwc_levs[ctx->channels[ch_index ^ swap_ch].power_levs[subband_to_powgrp[sb]]] / (1 << gcv);

    /* skip the lowest two quant units (frequencies 0...351 Hz) for subband 0 */
    for (qu = subband_to_qu[sb] + (!sb ? 2 : 0); qu < subband_to_qu[sb + 1]; qu++) {
        if (ctx->channels[ch_index].qu_wordlen[qu] <= 0)
            continue;

        qu_lev = ff_atrac3p_sf_tab[ctx->channels[ch_index].qu_sf_idx[qu]] *
                 ff_atrac3p_mant_tab[ctx->channels[ch_index].qu_wordlen[qu]] /
                 (1 << ctx->channels[ch_index].qu_wordlen[qu]) * grp_lev;

        dst = &sp[ff_atrac3p_qu_to_spec_pos[qu]];
        nsp = ff_atrac3p_qu_to_spec_pos[qu + 1] - ff_atrac3p_qu_to_spec_pos[qu];

        fdsp->vector_fmac_scalar(dst, pwcsp, qu_lev, nsp);
    }
}

void ff_atrac3p_imdct(AVFloatDSPContext *fdsp, AVTXContext *mdct_ctx,
                      av_tx_fn mdct_fn, float *pIn, float *pOut,
                      int wind_id, int sb)
{
    int i;

    if (sb & 1)
        for (i = 0; i < ATRAC3P_SUBBAND_SAMPLES / 2; i++)
            FFSWAP(float, pIn[i], pIn[ATRAC3P_SUBBAND_SAMPLES - 1 - i]);

    mdct_fn(mdct_ctx, pOut, pIn, sizeof(float));

    /* Perform windowing on the output.
     * ATRAC3+ uses two different MDCT windows:
     * - The first one is just the plain sine window of size 256
     * - The 2nd one is the plain sine window of size 128
     *   wrapped into zero (at the start) and one (at the end) regions.
     *   Both regions are 32 samples long. */
    if (wind_id & 2) { /* 1st half: steep window */
        memset(pOut, 0, sizeof(float) * 32);
        fdsp->vector_fmul(&pOut[32], &pOut[32], ff_sine_64, 64);
    } else /* 1st half: simple sine window */
        fdsp->vector_fmul(pOut, pOut, ff_sine_128, ATRAC3P_MDCT_SIZE / 2);

    if (wind_id & 1) { /* 2nd half: steep window */
        fdsp->vector_fmul_reverse(&pOut[160], &pOut[160], ff_sine_64, 64);
        memset(&pOut[224], 0, sizeof(float) * 32);
    } else /* 2nd half: simple sine window */
        fdsp->vector_fmul_reverse(&pOut[128], &pOut[128], ff_sine_128,
                                  ATRAC3P_MDCT_SIZE / 2);
}

/* lookup table for fast modulo 23 op required for cyclic buffers of the IPQF */
static const int mod23_lut[26] = {
    23,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 0
};

/* First half of the 384-tap IPQF filtering coefficients. */
static const float ipqf_coeffs1[ATRAC3P_PQF_FIR_LEN][16] = {
    { -5.8336207e-7,    -8.0604229e-7,    -4.2005411e-7,    -4.4400572e-8,
       3.226247e-8,      3.530856e-8,      1.2660377e-8,     0.000010516783,
      -0.000011838618,   6.005389e-7,      0.0000014333754,  0.0000023108685,
       0.0000032569742,  0.0000046192422,  0.0000063894258,  0.0000070302972 },
    { -0.0000091622824, -0.000010502935,  -0.0000079212787, -0.0000041712024,
      -0.0000026336629, -0.0000015432918, -5.7168614e-7,     0.0000018111954,
       0.000023530851,   0.00002780562,    0.000032302323,   0.000036968919,
       0.000041575615,   0.000045337845,   0.000046043948,   0.000048585582 },
    { -0.000064464548,  -0.000068306952,  -0.000073081472,  -0.00007612785,
      -0.000074850752,  -0.000070208509,  -0.000062285151,  -0.000058270442,
      -0.000056296329,  -0.000049888811,  -0.000035615325,  -0.000018532943,
       0.0000016657353,  0.00002610587,    0.000053397067,   0.00008079566 },
    { -0.00054488552,   -0.00052537228,   -0.00049731287,   -0.00045778,
      -0.00040612387,   -0.00034301577,   -0.00026866337,   -0.00018248901,
      -0.000084307925,   0.000025081157,   0.00014135583,    0.00026649953,
       0.00039945057,    0.00053928449,    0.00068422867,    0.00083093712 },
    { -0.0014771431,    -0.001283227,     -0.0010566821,    -0.00079780724,
      -0.00050782406,   -0.00018855913,    0.00015771533,    0.00052769453,
       0.00091862219,    0.001326357,      0.0017469483,     0.0021754825,
       0.0026067684,     0.0030352892,     0.0034549395,     0.0038591374 },
    { -0.0022995141,    -0.001443546,     -0.00049266568,    0.00055068987,
       0.001682895,      0.0028992873,     0.0041943151,     0.0055614738,
       0.0069935122,     0.0084823566,     0.010018963,      0.011593862,
       0.013196872,      0.014817309,      0.016444042,      0.018065533 },
    { -0.034426283,     -0.034281436,     -0.033992987,     -0.033563249,
      -0.032995768,     -0.032295227,     -0.031467363,     -0.030518902,
      -0.02945766,      -0.028291954,     -0.027031265,     -0.025685543,
      -0.024265358,     -0.022781773,     -0.021246184,     -0.019670162 },
    { -0.0030586775,    -0.0037203205,    -0.0042847847,    -0.0047529764,
      -0.0051268316,    -0.0054091476,    -0.0056034233,    -0.005714261,
      -0.0057445862,    -0.0057025906,    -0.0055920109,    -0.0054194843,
      -0.0051914565,    -0.0049146507,    -0.0045959447,    -0.0042418269 },
    { -0.0016376863,    -0.0017651899,    -0.0018608454,    -0.0019252141,
      -0.0019593791,    -0.0019653172,    -0.0019450618,    -0.0018990048,
      -0.00183808,      -0.0017501717,    -0.0016481078,    -0.0015320742,
      -0.0014046903,    -0.0012685474,    -0.001125814,     -0.00097943726 },
    { -0.00055432378,   -0.00055472925,   -0.00054783461,   -0.00053276919,
      -0.00051135791,   -0.00048466062,   -0.00045358928,   -0.00042499689,
      -0.00036942671,   -0.0003392619,    -0.00030001783,   -0.00025986304,
      -0.0002197204,    -0.00018116167,   -0.00014691355,   -0.00011279432 },
    { -0.000064147389,  -0.00006174868,   -0.000054267788,  -0.000047133824,
      -0.000042927582,  -0.000039477309,  -0.000036340745,  -0.000029687517,
      -0.000049787737,  -0.000041577889,  -0.000033864744,  -0.000026534748,
      -0.000019841305,  -0.000014789486,  -0.000013131184,  -0.0000099198869 },
    { -0.0000062990207, -0.0000072701259, -0.000011984052,  -0.000017348082,
      -0.000019907106,  -0.000021348773,  -0.000021961965,  -0.000012203576,
      -0.000010840992,   4.6299544e-7,     5.2588763e-7,     2.7792686e-7,
      -2.3649704e-7,    -0.0000010897784, -9.171448e-7,     -5.22682e-7 }
};

/* Second half of the 384-tap IPQF filtering coefficients. */
static const float ipqf_coeffs2[ATRAC3P_PQF_FIR_LEN][16] = {
    {  5.22682e-7,       9.171448e-7,      0.0000010897784,  2.3649704e-7,
      -2.7792686e-7,    -5.2588763e-7,    -4.6299544e-7,     0.000010840992,
      -0.000012203576,  -0.000021961965,  -0.000021348773,  -0.000019907106,
      -0.000017348082,  -0.000011984052,  -0.0000072701259, -0.0000062990207 },
    {  0.0000099198869,  0.000013131184,   0.000014789486,   0.000019841305,
       0.000026534748,   0.000033864744,   0.000041577889,   0.000049787737,
      -0.000029687517,  -0.000036340745,  -0.000039477309,  -0.000042927582,
      -0.000047133824,  -0.000054267788,  -0.00006174868,   -0.000064147389 },
    {  0.00011279432,    0.00014691355,    0.00018116167,    0.0002197204,
       0.00025986304,    0.00030001783,    0.0003392619,     0.00036942671,
      -0.00042499689,   -0.00045358928,   -0.00048466062,   -0.00051135791,
      -0.00053276919,   -0.00054783461,   -0.00055472925,   -0.00055432378 },
    {  0.00097943726,    0.001125814,      0.0012685474,     0.0014046903,
       0.0015320742,     0.0016481078,     0.0017501717,     0.00183808,
      -0.0018990048,    -0.0019450618,    -0.0019653172,    -0.0019593791,
      -0.0019252141,    -0.0018608454,    -0.0017651899,    -0.0016376863 },
    {  0.0042418269,     0.0045959447,     0.0049146507,     0.0051914565,
       0.0054194843,     0.0055920109,     0.0057025906,     0.0057445862,
      -0.005714261,     -0.0056034233,    -0.0054091476,    -0.0051268316,
      -0.0047529764,    -0.0042847847,    -0.0037203205,    -0.0030586775 },
    {  0.019670162,      0.021246184,      0.022781773,      0.024265358,
       0.025685543,      0.027031265,      0.028291954,      0.02945766,
      -0.030518902,     -0.031467363,     -0.032295227,     -0.032995768,
      -0.033563249,     -0.033992987,     -0.034281436,     -0.034426283 },
    { -0.018065533,     -0.016444042,     -0.014817309,     -0.013196872,
      -0.011593862,     -0.010018963,     -0.0084823566,    -0.0069935122,
       0.0055614738,     0.0041943151,     0.0028992873,     0.001682895,
       0.00055068987,   -0.00049266568,   -0.001443546,     -0.0022995141 },
    { -0.0038591374,    -0.0034549395,    -0.0030352892,    -0.0026067684,
      -0.0021754825,    -0.0017469483,    -0.001326357,     -0.00091862219,
       0.00052769453,    0.00015771533,   -0.00018855913,   -0.00050782406,
      -0.00079780724,   -0.0010566821,    -0.001283227,     -0.0014771431 },
    { -0.00083093712,   -0.00068422867,   -0.00053928449,   -0.00039945057,
      -0.00026649953,   -0.00014135583,   -0.000025081157,   0.000084307925,
      -0.00018248901,   -0.00026866337,   -0.00034301577,   -0.00040612387,
      -0.00045778,      -0.00049731287,   -0.00052537228,   -0.00054488552 },
    { -0.00008079566,   -0.000053397067,  -0.00002610587,   -0.0000016657353,
       0.000018532943,   0.000035615325,   0.000049888811,   0.000056296329,
      -0.000058270442,  -0.000062285151,  -0.000070208509,  -0.000074850752,
      -0.00007612785,   -0.000073081472,  -0.000068306952,  -0.000064464548 },
    { -0.000048585582,  -0.000046043948,  -0.000045337845,  -0.000041575615,
      -0.000036968919,  -0.000032302323,  -0.00002780562,   -0.000023530851,
       0.0000018111954, -5.7168614e-7,    -0.0000015432918, -0.0000026336629,
      -0.0000041712024, -0.0000079212787, -0.000010502935,  -0.0000091622824 },
    { -0.0000070302972, -0.0000063894258, -0.0000046192422, -0.0000032569742,
      -0.0000023108685, -0.0000014333754, -6.005389e-7,      0.000011838618,
       0.000010516783,   1.2660377e-8,     3.530856e-8,      3.226247e-8,
      -4.4400572e-8,    -4.2005411e-7,    -8.0604229e-7,    -5.8336207e-7 }
};

void ff_atrac3p_ipqf(AVTXContext *dct_ctx, av_tx_fn dct_fn,
                     Atrac3pIPQFChannelCtx *hist, const float *in, float *out)
{
    int i, s, sb, t, pos_now, pos_next;
    LOCAL_ALIGNED(32, float, idct_in, [ATRAC3P_SUBBANDS]);
    LOCAL_ALIGNED(32, float, idct_out, [ATRAC3P_SUBBANDS]);

    memset(out, 0, ATRAC3P_FRAME_SAMPLES * sizeof(*out));

    for (s = 0; s < ATRAC3P_SUBBAND_SAMPLES; s++) {
        /* pick up one sample from each subband */
        for (sb = 0; sb < ATRAC3P_SUBBANDS; sb++)
            idct_in[sb] = in[sb * ATRAC3P_SUBBAND_SAMPLES + s];

        /* Calculate the sine and cosine part of the PQF using IDCT-IV */
        dct_fn(dct_ctx, idct_out, idct_in, sizeof(float));

        /* append the result to the history */
        for (i = 0; i < 8; i++) {
            hist->buf1[hist->pos][i] = idct_out[i + 8];
            hist->buf2[hist->pos][i] = idct_out[7 - i];
        }

        pos_now  = hist->pos;
        pos_next = mod23_lut[pos_now + 2]; // pos_next = (pos_now + 1) % 23;

        for (t = 0; t < ATRAC3P_PQF_FIR_LEN; t++) {
            for (i = 0; i < 8; i++) {
                out[s * 16 + i + 0] += hist->buf1[pos_now][i]  * ipqf_coeffs1[t][i] +
                                       hist->buf2[pos_next][i] * ipqf_coeffs2[t][i];
                out[s * 16 + i + 8] += hist->buf1[pos_now][7 - i]  * ipqf_coeffs1[t][i + 8] +
                                       hist->buf2[pos_next][7 - i] * ipqf_coeffs2[t][i + 8];
            }

            pos_now  = mod23_lut[pos_next + 2]; // pos_now  = (pos_now  + 2) % 23;
            pos_next = mod23_lut[pos_now + 2];  // pos_next = (pos_next + 2) % 23;
        }

        hist->pos = mod23_lut[hist->pos]; // hist->pos = (hist->pos - 1) % 23;
    }
}
