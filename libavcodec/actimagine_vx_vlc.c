/*
 * H.264 cavlc tables
 * Copyright (c) 2003 Michael Niedermayer <michaelni at gmx.at>
 *
 * This file is part of Librempeg.
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

#include "internal.h"
#include "libavutil/thread.h"
#include "libavutil/avassert.h"
#include "actimagine_vx_vlc.h"

static const uint8_t coeff_token_len[4][4 * 17] = {
    {
         1, 0, 0, 0,
         6, 2, 0, 0,     8, 6, 3, 0,     9, 8, 7, 5,    10, 9, 8, 6,
        11,10, 9, 7,    13,11,10, 8,    13,13,11, 9,    13,13,13,10,
        14,14,13,11,    14,14,14,13,    15,15,14,14,    15,15,15,14,
        16,15,15,15,    16,16,16,15,    16,16,16,16,    16,16,16,16
    },
    {
         2, 0, 0, 0,
         6, 2, 0, 0,     6, 5, 3, 0,     7, 6, 6, 4,     8, 6, 6, 4,
         8, 7, 7, 5,     9, 8, 8, 6,    11, 9, 9, 6,    11,11,11, 7,
        12,11,11, 9,    12,12,12,11,    12,12,12,11,    13,13,13,12,
        13,13,13,13,    13,14,13,13,    14,14,14,13,    14,14,14,14,
    },
    {
         4, 0, 0, 0,
         6, 4, 0, 0,     6, 5, 4, 0,     6, 5, 5, 4,     7, 5, 5, 4,
         7, 5, 5, 4,     7, 6, 6, 4,     7, 6, 6, 4,     8, 7, 7, 5,
         8, 8, 7, 6,     9, 8, 8, 7,     9, 9, 8, 8,     9, 9, 9, 8,
        10, 9, 9, 9,    10,10,10,10,    10,10,10,10,    10,10,10,10,
    },
    {
         6, 0, 0, 0,
         6, 6, 0, 0,     6, 6, 6, 0,     6, 6, 6, 6,     6, 6, 6, 6,
         6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
         6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
         6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
    }
};

static const uint8_t coeff_token_bits[4][4 * 17] = {
    {
         1, 0, 0, 0,
         5, 1, 0, 0,     7, 4, 1, 0,     7, 6, 5, 3,     7, 6, 5, 3,
         7, 6, 5, 4,    15, 6, 5, 4,    11,14, 5, 4,     8,10,13, 4,
        15,14, 9, 4,    11,10,13,12,    15,14, 9,12,    11,10,13, 8,
        15, 1, 9,12,    11,14,13, 8,     7,10, 9,12,     4, 6, 5, 8,
    },
    {
         3, 0, 0, 0,
        11, 2, 0, 0,     7, 7, 3, 0,     7,10, 9, 5,     7, 6, 5, 4,
         4, 6, 5, 6,     7, 6, 5, 8,    15, 6, 5, 4,    11,14,13, 4,
        15,10, 9, 4,    11,14,13,12,     8,10, 9, 8,    15,14,13,12,
        11,10, 9,12,     7,11, 6, 8,     9, 8,10, 1,     7, 6, 5, 4,
    },
    {
        15, 0, 0, 0,
        15,14, 0, 0,    11,15,13, 0,     8,12,14,12,    15,10,11,11,
        11, 8, 9,10,     9,14,13, 9,     8,10, 9, 8,    15,14,13,13,
        11,14,10,12,    15,10,13,12,    11,14, 9,12,     8,10,13, 8,
        13, 7, 9,12,     9,12,11,10,     5, 8, 7, 6,     1, 4, 3, 2,
    },
    {
         3, 0, 0, 0,
         0, 1, 0, 0,     4, 5, 6, 0,     8, 9,10,11,    12,13,14,15,
        16,17,18,19,    20,21,22,23,    24,25,26,27,    28,29,30,31,
        32,33,34,35,    36,37,38,39,    40,41,42,43,    44,45,46,47,
        48,49,50,51,    52,53,54,55,    56,57,58,59,    60,61,62,63,
    }
};

const uint8_t ff_h264_cavlc_coeff_token_table_index[17] = {
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3
};

static const uint8_t total_zeros_len[16][16] = {
    {1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},
    {3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},
    {4,3,3,3,4,4,3,3,4,5,5,6,5,6},
    {5,3,4,4,3,3,3,4,3,4,5,5,5},
    {4,4,4,3,3,3,3,3,4,5,4,5},
    {6,5,3,3,3,3,3,3,4,3,6},
    {6,5,3,3,3,2,3,4,3,6},
    {6,4,5,3,2,2,3,3,6},
    {6,6,4,2,2,3,2,5},
    {5,5,3,2,2,2,4},
    {4,4,3,3,1,3},
    {4,4,2,1,3},
    {3,3,1,2},
    {2,2,1},
    {1,1},
};

static const uint8_t total_zeros_bits[16][16] = {
    {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1},
    {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
    {5,7,6,5,4,3,4,3,2,3,2,1,1,0},
    {3,7,5,4,6,5,4,3,3,2,2,1,0},
    {5,4,3,7,6,5,4,3,2,1,1,0},
    {1,1,7,6,5,4,3,2,1,1,0},
    {1,1,5,4,3,3,2,1,1,0},
    {1,1,1,3,3,2,2,1,0},
    {1,0,1,3,2,1,1,1},
    {1,0,1,3,2,1,1},
    {0,1,1,2,1,3},
    {0,1,1,1,1},
    {0,1,1,1},
    {0,1,1},
    {0,1},
};

static const uint8_t run_len[7][16] = {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
};

static const uint8_t run_bits[7][16] = {
    {1,0},
    {1,1,0},
    {3,2,1,0},
    {3,2,1,1,0},
    {3,2,3,2,1,0},
    {3,0,1,3,2,5,4},
    {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1},
};

const unsigned int ff_h264_cavlc_suffix_limit[7] = {
    0, 3, 6, 12, 24, 48, INT_MAX
};

VLC ff_h264_cavlc_coeff_token_vlc[4];
static VLCElem coeff_token_vlc_tables[520 + 332 + 280 + 256];
static const int coeff_token_vlc_tables_size[4] = { 520, 332, 280, 256 };

VLC ff_h264_cavlc_total_zeros_vlc[15];
static VLCElem total_zeros_vlc_tables[15][512];
static const int total_zeros_vlc_tables_size = 512;

VLC ff_h264_cavlc_run_vlc[6 + 1];
static VLCElem run_vlc_tables[6][8];
static const int run_vlc_tables_size = 8;

VLC ff_h264_cavlc_run7_vlc;
static VLCElem run7_vlc_table[96];
static const int run7_vlc_table_size = 96;

static av_cold void cavlc_init_vlc(void)
{
    int offset = 0;

    for (int i = 0; i < 4; i++) {
        ff_h264_cavlc_coeff_token_vlc[i].table = coeff_token_vlc_tables + offset;
        ff_h264_cavlc_coeff_token_vlc[i].table_allocated = coeff_token_vlc_tables_size[i];
        vlc_init(&ff_h264_cavlc_coeff_token_vlc[i], FF_H264_CAVLC_COEFF_TOKEN_VLC_BITS, 4 * 17,
                 &coeff_token_len [i][0], 1, 1,
                 &coeff_token_bits[i][0], 1, 1,
                 VLC_INIT_USE_STATIC);
        offset += coeff_token_vlc_tables_size[i];
    }

    av_assert0(offset == FF_ARRAY_ELEMS(coeff_token_vlc_tables));

    for (int i = 0; i < 15; i++) {
        ff_h264_cavlc_total_zeros_vlc[i].table = total_zeros_vlc_tables[i];
        ff_h264_cavlc_total_zeros_vlc[i].table_allocated = total_zeros_vlc_tables_size;
        vlc_init(&ff_h264_cavlc_total_zeros_vlc[i],
                 FF_H264_CAVLC_TOTAL_ZEROS_VLC_BITS, 16,
                 &total_zeros_len [i][0], 1, 1,
                 &total_zeros_bits[i][0], 1, 1,
                 VLC_INIT_USE_STATIC);
    }

    for (int i = 0; i < 6; i++) {
        ff_h264_cavlc_run_vlc[i + 1].table = run_vlc_tables[i];
        ff_h264_cavlc_run_vlc[i + 1].table_allocated = run_vlc_tables_size;
        vlc_init(&ff_h264_cavlc_run_vlc[i + 1],
                 FF_H264_CAVLC_RUN_VLC_BITS, 7,
                 &run_len [i][0], 1, 1,
                 &run_bits[i][0], 1, 1,
                 VLC_INIT_USE_STATIC);
    }

    ff_h264_cavlc_run7_vlc.table = run7_vlc_table,
    ff_h264_cavlc_run7_vlc.table_allocated = run7_vlc_table_size;
    vlc_init(&ff_h264_cavlc_run7_vlc, FF_H264_CAVLC_RUN7_VLC_BITS, 16,
             &run_len [6][0], 1, 1,
             &run_bits[6][0], 1, 1,
             VLC_INIT_USE_STATIC);
}

int ff_h264_cavlc_data_init_vlc(void)
{
    static AVOnce vlc_init = AV_ONCE_INIT;
    return ff_thread_once(&vlc_init, cavlc_init_vlc);
}
