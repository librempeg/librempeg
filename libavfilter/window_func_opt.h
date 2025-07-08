/*
 * Copyright (c) 2015 Paul B Mahol
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

#ifndef AVFILTER_WINDOW_FUNC_OPT_H
#define AVFILTER_WINDOW_FUNC_OPT_H

enum WindowFunc     { WFUNC_RECT, WFUNC_HANNING, WFUNC_HAMMING, WFUNC_BLACKMAN,
                      WFUNC_BARTLETT, WFUNC_WELCH, WFUNC_FLATTOP,
                      WFUNC_BHARRIS, WFUNC_BNUTTALL, WFUNC_SINE, WFUNC_NUTTALL,
                      WFUNC_BHANN, WFUNC_LANCZOS, WFUNC_GAUSS, WFUNC_TUKEY,
                      WFUNC_DOLPH, WFUNC_CAUCHY, WFUNC_PARZEN, WFUNC_POISSON,
                      WFUNC_BOHMAN, WFUNC_KAISER,
                      NB_WFUNC };

#define WIN_FUNC_OPTION(win_func_opt_name, win_func_offset, flag, default_window_func)                              \
    { win_func_opt_name, "set window function", win_func_offset, AV_OPT_TYPE_INT, {.i64 = default_window_func}, 0, NB_WFUNC-1, flag, .unit = "win_func" }, \
        { "rect",     "Rectangular",      0, AV_OPT_TYPE_CONST, {.i64=WFUNC_RECT},     0, 0, flag, .unit = "win_func" }, \
        { "bartlett", "Bartlett",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BARTLETT}, 0, 0, flag, .unit = "win_func" }, \
        { "hann",     "Hann",             0, AV_OPT_TYPE_CONST, {.i64=WFUNC_HANNING},  0, 0, flag, .unit = "win_func" }, \
        { "hanning",  "Hanning",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_HANNING},  0, 0, flag, .unit = "win_func" }, \
        { "hamming",  "Hamming",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_HAMMING},  0, 0, flag, .unit = "win_func" }, \
        { "blackman", "Blackman",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BLACKMAN}, 0, 0, flag, .unit = "win_func" }, \
        { "welch",    "Welch",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_WELCH},    0, 0, flag, .unit = "win_func" }, \
        { "flattop",  "Flat-top",         0, AV_OPT_TYPE_CONST, {.i64=WFUNC_FLATTOP},  0, 0, flag, .unit = "win_func" }, \
        { "bharris",  "Blackman-Harris",  0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BHARRIS},  0, 0, flag, .unit = "win_func" }, \
        { "bnuttall", "Blackman-Nuttall", 0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BNUTTALL}, 0, 0, flag, .unit = "win_func" }, \
        { "bhann",    "Bartlett-Hann",    0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BHANN},    0, 0, flag, .unit = "win_func" }, \
        { "sine",     "Sine",             0, AV_OPT_TYPE_CONST, {.i64=WFUNC_SINE},     0, 0, flag, .unit = "win_func" }, \
        { "nuttall",  "Nuttall",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_NUTTALL},  0, 0, flag, .unit = "win_func" }, \
        { "lanczos",  "Lanczos",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_LANCZOS},  0, 0, flag, .unit = "win_func" }, \
        { "gauss",    "Gauss",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_GAUSS},    0, 0, flag, .unit = "win_func" }, \
        { "tukey",    "Tukey",            0, AV_OPT_TYPE_CONST, {.i64=WFUNC_TUKEY},    0, 0, flag, .unit = "win_func" }, \
        { "dolph",    "Dolph-Chebyshev",  0, AV_OPT_TYPE_CONST, {.i64=WFUNC_DOLPH},    0, 0, flag, .unit = "win_func" }, \
        { "cauchy",   "Cauchy",           0, AV_OPT_TYPE_CONST, {.i64=WFUNC_CAUCHY},   0, 0, flag, .unit = "win_func" }, \
        { "parzen",   "Parzen",           0, AV_OPT_TYPE_CONST, {.i64=WFUNC_PARZEN},   0, 0, flag, .unit = "win_func" }, \
        { "poisson",  "Poisson",          0, AV_OPT_TYPE_CONST, {.i64=WFUNC_POISSON},  0, 0, flag, .unit = "win_func" }, \
        { "bohman",   "Bohman",           0, AV_OPT_TYPE_CONST, {.i64=WFUNC_BOHMAN},   0, 0, flag, .unit = "win_func" }, \
        { "kaiser",   "Kaiser",           0, AV_OPT_TYPE_CONST, {.i64=WFUNC_KAISER},   0, 0, flag, .unit = "win_func" }

#endif /* AVFILTER_WINDOW_FUNC_OPT_H */

