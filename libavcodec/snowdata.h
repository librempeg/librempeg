/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (C) 2006 Robert Edele <yartrebo@earthlink.net>
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

#ifndef AVCODEC_SNOWDATA_H
#define AVCODEC_SNOWDATA_H

#include "snow.h"

static const uint8_t obmc32[1024]={
  0,  0,  0,  0,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,  8,  4,  4,  4,  4,  4,  4,  4,  4,  0,  0,  0,  0,
  0,  4,  4,  4,  8,  8,  8, 12, 12, 16, 16, 16, 20, 20, 20, 24, 24, 20, 20, 20, 16, 16, 16, 12, 12,  8,  8,  8,  4,  4,  4,  0,
  0,  4,  8,  8, 12, 12, 16, 20, 20, 24, 28, 28, 32, 32, 36, 40, 40, 36, 32, 32, 28, 28, 24, 20, 20, 16, 12, 12,  8,  8,  4,  0,
  0,  4,  8, 12, 16, 20, 24, 28, 28, 32, 36, 40, 44, 48, 52, 56, 56, 52, 48, 44, 40, 36, 32, 28, 28, 24, 20, 16, 12,  8,  4,  0,
  4,  8, 12, 16, 20, 24, 28, 32, 40, 44, 48, 52, 56, 60, 64, 68, 68, 64, 60, 56, 52, 48, 44, 40, 32, 28, 24, 20, 16, 12,  8,  4,
  4,  8, 12, 20, 24, 32, 36, 40, 48, 52, 56, 64, 68, 76, 80, 84, 84, 80, 76, 68, 64, 56, 52, 48, 40, 36, 32, 24, 20, 12,  8,  4,
  4,  8, 16, 24, 28, 36, 44, 48, 56, 60, 68, 76, 80, 88, 96,100,100, 96, 88, 80, 76, 68, 60, 56, 48, 44, 36, 28, 24, 16,  8,  4,
  4, 12, 20, 28, 32, 40, 48, 56, 64, 72, 80, 88, 92,100,108,116,116,108,100, 92, 88, 80, 72, 64, 56, 48, 40, 32, 28, 20, 12,  4,
  4, 12, 20, 28, 40, 48, 56, 64, 72, 80, 88, 96,108,116,124,132,132,124,116,108, 96, 88, 80, 72, 64, 56, 48, 40, 28, 20, 12,  4,
  4, 16, 24, 32, 44, 52, 60, 72, 80, 92,100,108,120,128,136,148,148,136,128,120,108,100, 92, 80, 72, 60, 52, 44, 32, 24, 16,  4,
  4, 16, 28, 36, 48, 56, 68, 80, 88,100,112,120,132,140,152,164,164,152,140,132,120,112,100, 88, 80, 68, 56, 48, 36, 28, 16,  4,
  4, 16, 28, 40, 52, 64, 76, 88, 96,108,120,132,144,156,168,180,180,168,156,144,132,120,108, 96, 88, 76, 64, 52, 40, 28, 16,  4,
  8, 20, 32, 44, 56, 68, 80, 92,108,120,132,144,156,168,180,192,192,180,168,156,144,132,120,108, 92, 80, 68, 56, 44, 32, 20,  8,
  8, 20, 32, 48, 60, 76, 88,100,116,128,140,156,168,184,196,208,208,196,184,168,156,140,128,116,100, 88, 76, 60, 48, 32, 20,  8,
  8, 20, 36, 52, 64, 80, 96,108,124,136,152,168,180,196,212,224,224,212,196,180,168,152,136,124,108, 96, 80, 64, 52, 36, 20,  8,
  8, 24, 40, 56, 68, 84,100,116,132,148,164,180,192,208,224,240,240,224,208,192,180,164,148,132,116,100, 84, 68, 56, 40, 24,  8,
  8, 24, 40, 56, 68, 84,100,116,132,148,164,180,192,208,224,240,240,224,208,192,180,164,148,132,116,100, 84, 68, 56, 40, 24,  8,
  8, 20, 36, 52, 64, 80, 96,108,124,136,152,168,180,196,212,224,224,212,196,180,168,152,136,124,108, 96, 80, 64, 52, 36, 20,  8,
  8, 20, 32, 48, 60, 76, 88,100,116,128,140,156,168,184,196,208,208,196,184,168,156,140,128,116,100, 88, 76, 60, 48, 32, 20,  8,
  8, 20, 32, 44, 56, 68, 80, 92,108,120,132,144,156,168,180,192,192,180,168,156,144,132,120,108, 92, 80, 68, 56, 44, 32, 20,  8,
  4, 16, 28, 40, 52, 64, 76, 88, 96,108,120,132,144,156,168,180,180,168,156,144,132,120,108, 96, 88, 76, 64, 52, 40, 28, 16,  4,
  4, 16, 28, 36, 48, 56, 68, 80, 88,100,112,120,132,140,152,164,164,152,140,132,120,112,100, 88, 80, 68, 56, 48, 36, 28, 16,  4,
  4, 16, 24, 32, 44, 52, 60, 72, 80, 92,100,108,120,128,136,148,148,136,128,120,108,100, 92, 80, 72, 60, 52, 44, 32, 24, 16,  4,
  4, 12, 20, 28, 40, 48, 56, 64, 72, 80, 88, 96,108,116,124,132,132,124,116,108, 96, 88, 80, 72, 64, 56, 48, 40, 28, 20, 12,  4,
  4, 12, 20, 28, 32, 40, 48, 56, 64, 72, 80, 88, 92,100,108,116,116,108,100, 92, 88, 80, 72, 64, 56, 48, 40, 32, 28, 20, 12,  4,
  4,  8, 16, 24, 28, 36, 44, 48, 56, 60, 68, 76, 80, 88, 96,100,100, 96, 88, 80, 76, 68, 60, 56, 48, 44, 36, 28, 24, 16,  8,  4,
  4,  8, 12, 20, 24, 32, 36, 40, 48, 52, 56, 64, 68, 76, 80, 84, 84, 80, 76, 68, 64, 56, 52, 48, 40, 36, 32, 24, 20, 12,  8,  4,
  4,  8, 12, 16, 20, 24, 28, 32, 40, 44, 48, 52, 56, 60, 64, 68, 68, 64, 60, 56, 52, 48, 44, 40, 32, 28, 24, 20, 16, 12,  8,  4,
  0,  4,  8, 12, 16, 20, 24, 28, 28, 32, 36, 40, 44, 48, 52, 56, 56, 52, 48, 44, 40, 36, 32, 28, 28, 24, 20, 16, 12,  8,  4,  0,
  0,  4,  8,  8, 12, 12, 16, 20, 20, 24, 28, 28, 32, 32, 36, 40, 40, 36, 32, 32, 28, 28, 24, 20, 20, 16, 12, 12,  8,  8,  4,  0,
  0,  4,  4,  4,  8,  8,  8, 12, 12, 16, 16, 16, 20, 20, 20, 24, 24, 20, 20, 20, 16, 16, 16, 12, 12,  8,  8,  8,  4,  4,  4,  0,
  0,  0,  0,  0,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,  8,  4,  4,  4,  4,  4,  4,  4,  4,  0,  0,  0,  0,
 //error:0.000020
};
static const uint8_t obmc16[256]={
  0,  4,  4,  8,  8, 12, 12, 16, 16, 12, 12,  8,  8,  4,  4,  0,
  4,  8, 16, 20, 28, 32, 40, 44, 44, 40, 32, 28, 20, 16,  8,  4,
  4, 16, 24, 36, 44, 56, 64, 76, 76, 64, 56, 44, 36, 24, 16,  4,
  8, 20, 36, 48, 64, 76, 92,104,104, 92, 76, 64, 48, 36, 20,  8,
  8, 28, 44, 64, 80,100,116,136,136,116,100, 80, 64, 44, 28,  8,
 12, 32, 56, 76,100,120,144,164,164,144,120,100, 76, 56, 32, 12,
 12, 40, 64, 92,116,144,168,196,196,168,144,116, 92, 64, 40, 12,
 16, 44, 76,104,136,164,196,224,224,196,164,136,104, 76, 44, 16,
 16, 44, 76,104,136,164,196,224,224,196,164,136,104, 76, 44, 16,
 12, 40, 64, 92,116,144,168,196,196,168,144,116, 92, 64, 40, 12,
 12, 32, 56, 76,100,120,144,164,164,144,120,100, 76, 56, 32, 12,
  8, 28, 44, 64, 80,100,116,136,136,116,100, 80, 64, 44, 28,  8,
  8, 20, 36, 48, 64, 76, 92,104,104, 92, 76, 64, 48, 36, 20,  8,
  4, 16, 24, 36, 44, 56, 64, 76, 76, 64, 56, 44, 36, 24, 16,  4,
  4,  8, 16, 20, 28, 32, 40, 44, 44, 40, 32, 28, 20, 16,  8,  4,
  0,  4,  4,  8,  8, 12, 12, 16, 16, 12, 12,  8,  8,  4,  4,  0,
//error:0.000015
};

//linear *64
static const uint8_t obmc8[64]={
  4, 12, 20, 28, 28, 20, 12,  4,
 12, 36, 60, 84, 84, 60, 36, 12,
 20, 60,100,140,140,100, 60, 20,
 28, 84,140,196,196,140, 84, 28,
 28, 84,140,196,196,140, 84, 28,
 20, 60,100,140,140,100, 60, 20,
 12, 36, 60, 84, 84, 60, 36, 12,
  4, 12, 20, 28, 28, 20, 12,  4,
//error:0.000000
};

//linear *64
static const uint8_t obmc4[16]={
 16, 48, 48, 16,
 48,144,144, 48,
 48,144,144, 48,
 16, 48, 48, 16,
//error:0.000000
};

const int8_t ff_quant3bA[256]={
 0, 0, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,
};

const uint8_t * const ff_obmc_tab[4]= {
    obmc32, obmc16, obmc8, obmc4
};

/* ff_qexp[i] = lrintf(128 * 2^(i / QROOT)) with QROOT = 32 */
const uint8_t ff_qexp[QROOT] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 156, 159,
    162, 166, 170, 173, 177, 181, 185, 189, 193, 197, 202,
    206, 211, 215, 220, 225, 230, 235, 240, 245, 251,
};

/* table generated at runtime */
int ff_scale_mv_ref[MAX_REF_FRAMES][MAX_REF_FRAMES];


#endif /* AVCODEC_SNOW_H */
