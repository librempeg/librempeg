/*
 * CCITT Fax Group 3 and 4 decompression
 * Copyright (c) 2008 Konstantin Shishkov
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
 * CCITT Fax Group 3 and 4 decompression
 * @author Konstantin Shishkov
 */
#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "avcodec.h"
#include "get_bits.h"
#include "put_bits.h"
#include "faxcompr.h"

#define CCITT_SYMS 104

static const uint16_t ccitt_syms[CCITT_SYMS] = {
    0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,
   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,   24,   25,
   26,   27,   28,   29,   30,   31,   32,   33,   34,   35,   36,   37,   38,
   39,   40,   41,   42,   43,   44,   45,   46,   47,   48,   49,   50,   51,
   52,   53,   54,   55,   56,   57,   58,   59,   60,   61,   62,   63,   64,
  128,  192,  256,  320,  384,  448,  512,  576,  640,  704,  768,  832,  896,
  960, 1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728,
 1792, 1856, 1920, 1984, 2048, 2112, 2176, 2240, 2304, 2368, 2432, 2496, 2560
};

static const uint8_t ccitt_codes_bits[2][CCITT_SYMS] =
{
  {
    0x35, 0x07, 0x07, 0x08, 0x0B, 0x0C, 0x0E, 0x0F, 0x13, 0x14, 0x07, 0x08, 0x08,
    0x03, 0x34, 0x35, 0x2A, 0x2B, 0x27, 0x0C, 0x08, 0x17, 0x03, 0x04, 0x28, 0x2B,
    0x13, 0x24, 0x18, 0x02, 0x03, 0x1A, 0x1B, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x04, 0x05, 0x0A, 0x0B, 0x52, 0x53, 0x54,
    0x55, 0x24, 0x25, 0x58, 0x59, 0x5A, 0x5B, 0x4A, 0x4B, 0x32, 0x33, 0x34, 0x1B,
    0x12, 0x17, 0x37, 0x36, 0x37, 0x64, 0x65, 0x68, 0x67, 0xCC, 0xCD, 0xD2, 0xD3,
    0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0x98, 0x99, 0x9A, 0x18, 0x9B,
    0x08, 0x0C, 0x0D, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1C, 0x1D, 0x1E, 0x1F
  },
  {
    0x37, 0x02, 0x03, 0x02, 0x03, 0x03, 0x02, 0x03, 0x05, 0x04, 0x04, 0x05, 0x07,
    0x04, 0x07, 0x18, 0x17, 0x18, 0x08, 0x67, 0x68, 0x6C, 0x37, 0x28, 0x17, 0x18,
    0xCA, 0xCB, 0xCC, 0xCD, 0x68, 0x69, 0x6A, 0x6B, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6,
    0xD7, 0x6C, 0x6D, 0xDA, 0xDB, 0x54, 0x55, 0x56, 0x57, 0x64, 0x65, 0x52, 0x53,
    0x24, 0x37, 0x38, 0x27, 0x28, 0x58, 0x59, 0x2B, 0x2C, 0x5A, 0x66, 0x67, 0x0F,
    0xC8, 0xC9, 0x5B, 0x33, 0x34, 0x35, 0x6C, 0x6D, 0x4A, 0x4B, 0x4C, 0x4D, 0x72,
    0x73, 0x74, 0x75, 0x76, 0x77, 0x52, 0x53, 0x54, 0x55, 0x5A, 0x5B, 0x64, 0x65,
    0x08, 0x0C, 0x0D, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1C, 0x1D, 0x1E, 0x1F
  }
};

static const uint8_t ccitt_codes_lens[2][CCITT_SYMS] =
{
  {
     8,  6,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,
     7,  7,  7,  7,  7,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,
     8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,
     8,  8,  8,  8,  5,  5,  6,  7,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,
     9,  9,  9,  9,  9,  9,  9,  9,  9,  6,  9, 11, 11, 11, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12
  },
  {
    10,  3,  2,  2,  3,  4,  4,  5,  6,  6,  7,  7,  7,  8,  8,  9, 10, 10, 10, 11,
    11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 10, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 11, 11, 11, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12
  }
};

static const uint8_t ccitt_group3_2d_bits[11] = {
    1, 1, 2, 2, 2, 1, 3, 3, 3, 1, 1
};

static const uint8_t ccitt_group3_2d_lens[11] = {
    4, 3, 7, 6, 3, 1, 3, 6, 7, 7, 9
};

// Also contains the other VLC tables pointed to by ccitt_vlc
static VLCElem ccitt_group3_2d_vlc[512 + 528 + 648];
static const VLCElem *ccitt_vlc[2];

static av_cold void ccitt_unpack_init(void)
{
    VLCInitState state = VLC_INIT_STATE(ccitt_group3_2d_vlc);
    int i;

    ff_vlc_init_tables(&state, 9, 11,
                       ccitt_group3_2d_lens, 1, 1,
                       ccitt_group3_2d_bits, 1, 1, 0);
    for (i = 0; i < 2; i++) {
        ccitt_vlc[i] = ff_vlc_init_tables_sparse(&state, 9, CCITT_SYMS,
                                                 ccitt_codes_lens[i], 1, 1,
                                                 ccitt_codes_bits[i], 1, 1,
                                                 ccitt_syms, 2, 2, 0);
    }
}

av_cold void ff_ccitt_unpack_init(void)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    ff_thread_once(&init_static_once, ccitt_unpack_init);
}

static int decode_uncompressed(AVCodecContext *avctx, GetBitContext *gb,
                               unsigned int *pix_left, int **runs,
                               const int *runend, int *mode)
{
    int eob = 0;
    int newmode;
    int saved_run = 0;

    do {
        int cwi, k;
        int cw = 0;
        int codes[2];
        do {
            cwi = show_bits(gb, 11);
            if (!cwi) {
                av_log(avctx, AV_LOG_ERROR, "Invalid uncompressed codeword\n");
                return AVERROR_INVALIDDATA;
            }
            cwi = 10 - av_log2(cwi);
            if (get_bits_left(gb) < cwi + 1)
                return AVERROR_INVALIDDATA;
            skip_bits(gb, cwi + 1);
            if (cwi > 5) {
                newmode = get_bits1(gb);
                eob = 1;
                cwi -= 6;
            }
            cw += cwi;
        } while(cwi == 5);

        codes[0] = cw;
        codes[1] = !eob;

        for (k = 0; k < 2; k++) {
            if (codes[k]) {
                if (*mode == !k) {
                    *(*runs)++ = saved_run;
                    if (*runs >= runend) {
                        av_log(avctx, AV_LOG_ERROR, "uncompressed run overrun\n");
                        return AVERROR_INVALIDDATA;
                    }
                    if (*pix_left <= saved_run) {
                        av_log(avctx, AV_LOG_ERROR, "uncompressed run went out of bounds\n");
                        return AVERROR_INVALIDDATA;
                    }
                    *pix_left -= saved_run;
                    saved_run = 0;
                    *mode = !*mode;
                }
                saved_run += codes[k];
            }
        }
    } while (!eob);
    *(*runs)++ = saved_run;
    if (*runs >= runend) {
        av_log(avctx, AV_LOG_ERROR, "uncompressed run overrun\n");
        return AVERROR_INVALIDDATA;
    }
    if (*pix_left <= saved_run) {
        if (*pix_left == saved_run)
            return 1;
        av_log(avctx, AV_LOG_ERROR, "uncompressed run went out of boundsE\n");
        return AVERROR_INVALIDDATA;
    }
    *pix_left -= saved_run;
    saved_run = 0;
    *mode = !*mode;
    if (newmode != *mode) { //FIXME CHECK
        *(*runs)++ = 0;
        if (*runs >= runend) {
            av_log(avctx, AV_LOG_ERROR, "uncompressed run overrun\n");
            return AVERROR_INVALIDDATA;
        }
        *mode = newmode;
    }
    return 0;
}

static int decode_group3_1d_line(AVCodecContext *avctx, GetBitContext *gb,
                                 unsigned int pix_left, int *runs,
                                 const int *runend)
{
    int mode         = 0;
    unsigned int run = 0;
    unsigned int t;
    for (;;) {
        if (get_bits_left(gb) <= 0)
            return AVERROR_INVALIDDATA;
        t    = get_vlc2(gb, ccitt_vlc[mode], 9, 2);
        run += t;
        if (t < 64) {
            *runs++ = run;
            if (runs >= runend) {
                av_log(avctx, AV_LOG_ERROR, "Run overrun\n");
                return AVERROR_INVALIDDATA;
            }
            if (pix_left <= run) {
                if (pix_left == run)
                    break;
                av_log(avctx, AV_LOG_ERROR, "Run went out of bounds\n");
                return AVERROR_INVALIDDATA;
            }
            pix_left -= run;
            run       = 0;
            mode      = !mode;
        } else if ((int)t == -1) {
            if (get_bits_left(gb) > 12 && show_bits(gb, 12) == 15) {
                int ret;
                skip_bits(gb, 12);
                ret = decode_uncompressed(avctx, gb, &pix_left, &runs, runend, &mode);
                if (ret < 0) {
                    return ret;
                } else if (ret)
                    break;
            } else {
                av_log(avctx, AV_LOG_ERROR, "Incorrect code\n");
                return AVERROR_INVALIDDATA;
            }
        }
    }
    *runs++ = 0;
    return 0;
}

static int decode_group3_2d_line(AVCodecContext *avctx, GetBitContext *gb,
                                 unsigned int width, int *runs,
                                 const int *runend, const int *ref)
{
    int mode          = 0, saved_run = 0, t;
    int run_off       = *ref++;
    unsigned int offs = 0, run = 0;

    while (offs < width) {
        int cmode;
        if (get_bits_left(gb) <= 0)
            return AVERROR_INVALIDDATA;
        cmode = get_vlc2(gb, ccitt_group3_2d_vlc, 9, 1);
        if (cmode == -1) {
            av_log(avctx, AV_LOG_ERROR, "Incorrect mode VLC\n");
            return AVERROR_INVALIDDATA;
        }
        if (!cmode) { //pass mode
            if (run_off < width)
                run_off += *ref++;
            run      = run_off - offs;
            offs     = run_off;
            if (run_off < width)
                run_off += *ref++;
            if (offs > width) {
                av_log(avctx, AV_LOG_ERROR, "Run went out of bounds\n");
                return AVERROR_INVALIDDATA;
            }
            saved_run += run;
        } else if (cmode == 1) { //horizontal mode
            int k;
            for (k = 0; k < 2; k++) {
                run = 0;
                for (;;) {
                    if (get_bits_left(gb) <= 0)
                        return AVERROR_INVALIDDATA;
                    t = get_vlc2(gb, ccitt_vlc[mode], 9, 2);
                    if (t == -1) {
                        av_log(avctx, AV_LOG_ERROR, "Incorrect code\n");
                        return AVERROR_INVALIDDATA;
                    }
                    run += t;
                    if (t < 64)
                        break;
                }
                *runs++ = run + saved_run;
                if (runs >= runend) {
                    av_log(avctx, AV_LOG_ERROR, "Run overrun\n");
                    return AVERROR_INVALIDDATA;
                }
                saved_run = 0;
                offs     += run;
                if (offs > width || run > width) {
                    av_log(avctx, AV_LOG_ERROR, "Run went out of bounds\n");
                    return AVERROR_INVALIDDATA;
                }
                mode = !mode;
            }
        } else if (cmode == 9 || cmode == 10) {
            int xxx;
            if (get_bits_left(gb) < 3)
                return AVERROR_INVALIDDATA;
            xxx = get_bits(gb, 3);
            if (cmode == 9 && xxx == 7) {
                int ret;
                int pix_left = width - offs;

                if (saved_run) {
                    av_log(avctx, AV_LOG_ERROR, "saved run %d on entering uncompressed mode\n", saved_run);
                    return AVERROR_INVALIDDATA;
                }
                ret = decode_uncompressed(avctx, gb, &pix_left, &runs, runend, &mode);
                offs = width - pix_left;
                if (ret < 0) {
                    return ret;
                } else if (ret)
                    break;
            } else {
                avpriv_report_missing_feature(avctx, "Special mode %d xxx=%d support", cmode, xxx);
                return AVERROR_PATCHWELCOME;
            }
        } else { //vertical mode
            run      = run_off - offs + (cmode - 5);
            run_off -= *--ref;
            offs    += run;
            if (offs > width || run > width) {
                av_log(avctx, AV_LOG_ERROR, "Run went out of bounds\n");
                return AVERROR_INVALIDDATA;
            }
            *runs++ = run + saved_run;
            if (runs >= runend) {
                av_log(avctx, AV_LOG_ERROR, "Run overrun\n");
                return AVERROR_INVALIDDATA;
            }
            saved_run = 0;
            mode      = !mode;
        }
        //sync line pointers
        while (offs < width && run_off <= offs) {
            run_off += *ref++;
            run_off += *ref++;
        }
    }
    *runs++ = saved_run;
    if (saved_run) {
        if (runs >= runend) {
            av_log(avctx, AV_LOG_ERROR, "Run overrun\n");
            return -1;
        }
        *runs++ = 0;
    }
    return 0;
}

static void put_line(uint8_t *dst, int size, int width, const int *runs)
{
    PutBitContext pb;
    int run, mode = ~0, pix_left = width, run_idx = 0;

    init_put_bits(&pb, dst, size);
    while (pix_left > 0) {
        run       = runs[run_idx++];
        mode      = ~mode;
        pix_left -= run;
        for (; run > 16; run -= 16)
            put_sbits(&pb, 16, mode);
        if (run)
            put_sbits(&pb, run, mode);
    }
    flush_put_bits(&pb);
}

static int find_group3_syncmarker(GetBitContext *gb, int srcsize)
{
    unsigned int state = -1;
    srcsize -= get_bits_count(gb);
    while (srcsize-- > 0) {
        state += state + get_bits1(gb);
        if ((state & 0xFFF) == 1)
            return 0;
    }
    return -1;
}

int ff_ccitt_unpack(AVCodecContext *avctx, const uint8_t *src, int srcsize,
                    uint8_t *dst, int height, int stride,
                    enum TiffCompr compr, int opts)
{
    int j;
    GetBitContext gb;
    int *runs, *ref = NULL, *runend;
    int ret;
    int runsize = avctx->width + 2;
    int has_eol;

    runs = av_malloc_array(runsize, sizeof(runs[0]));
    ref  = av_malloc_array(runsize, sizeof(ref[0]));
    if (!runs || !ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ref[0] = avctx->width;
    ref[1] = 0;
    ref[2] = 0;
    if ((ret = init_get_bits8(&gb, src, srcsize)) < 0)
        goto fail;
    has_eol = show_bits(&gb, 12) == 1 || show_bits(&gb, 16) == 1;

    for (j = 0; j < height; j++) {
        runend = runs + runsize;
        if (compr == TIFF_G4) {
            ret = decode_group3_2d_line(avctx, &gb, avctx->width, runs, runend,
                                        ref);
            if (ret < 0)
                goto fail;
        } else {
            int g3d1 = (compr == TIFF_G3) && !(opts & 1);
            if (compr != TIFF_CCITT_RLE &&
                has_eol &&
                find_group3_syncmarker(&gb, srcsize * 8) < 0)
                break;
            if (compr == TIFF_CCITT_RLE || g3d1 || get_bits1(&gb))
                ret = decode_group3_1d_line(avctx, &gb, avctx->width, runs,
                                            runend);
            else
                ret = decode_group3_2d_line(avctx, &gb, avctx->width, runs,
                                            runend, ref);
            if (compr == TIFF_CCITT_RLE)
                align_get_bits(&gb);
        }
        if (avctx->err_recognition & AV_EF_EXPLODE && ret < 0)
            goto fail;

        if (ret < 0) {
            put_line(dst, stride, avctx->width, ref);
        } else {
            put_line(dst, stride, avctx->width, runs);
            FFSWAP(int *, runs, ref);
        }
        dst += stride;
    }
    ret = 0;
fail:
    av_free(runs);
    av_free(ref);
    return ret;
}
