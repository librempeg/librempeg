/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avfilter.h"
#include "video.h"

#undef SRC_F
#undef SRP
#if SRC_DEPTH == 8
#define SRC_F 0
#define SRP(x) ((x)[0])
#elif SRC_DEPTH == 16
#define SRC_F 1
#if SRC_E == 0
#define SRP(x) AV_RL16((x))
#elif SRC_E == 1
#define SRP(x) AV_RB16((x))
#endif
#endif

#include "pf2pf_template.c"
