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

#include <stdint.h>

#define BITSTREAM_READER_LE
#include "get_bits.h"

#include "inflate.h"

static void build_fixed_trees(InflateTree *lt, InflateTree *dt)
{
    for (int i = 0; i < 16; i++)
        lt->counts[i] = 0;

    lt->counts[7] = 24;
    lt->counts[8] = 152;
    lt->counts[9] = 112;

    for (int i = 0; i < 24; i++)
        lt->symbols[i] = 256 + i;
    for (int i = 0; i < 144; i++)
        lt->symbols[24 + i] = i;
    for (int i = 0; i < 8; i++)
        lt->symbols[24 + 144 + i] = 280 + i;
    for (int i = 0; i < 112; i++)
        lt->symbols[24 + 144 + 8 + i] = 144 + i;

    lt->max_sym = 285;

    for (int i = 0; i < 16; i++)
        dt->counts[i] = 0;

    dt->counts[5] = 32;

    for (int i = 0; i < 32; i++)
        dt->symbols[i] = i;

    dt->max_sym = 29;
}

static int decode_symbol(InflateContext *s, const InflateTree *t)
{
    GetBitContext *gb = &s->gb;
    int base = 0, offs = 0;

    for (int len = 1; ; len++) {
        offs = 2 * offs + get_bits1(gb);

        if (offs < t->counts[len])
            break;

        base += t->counts[len];
        offs -= t->counts[len];
    }

    return t->symbols[base + offs];
}

static uint32_t get_bits_base(GetBitContext *gb, int bits, int base)
{
    return base + get_bitsz(gb, bits);
}

static int inflate_block_data(InflateContext *s, InflateTree *lt, InflateTree *dt)
{
    static const uint8_t length_bits[30] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
        4, 4, 4, 4, 5, 5, 5, 5, 0, 127
    };
    static const uint16_t length_base[30] = {
         3,  4,  5,   6,   7,   8,   9,  10,  11,  13,
        15, 17, 19,  23,  27,  31,  35,  43,  51,  59,
        67, 83, 99, 115, 131, 163, 195, 227, 258,   0
    };
    static const uint8_t dist_bits[30] = {
        0, 0,  0,  0,  1,  1,  2,  2,  3,  3,
        4, 4,  5,  5,  6,  6,  7,  7,  8,  8,
        9, 9, 10, 10, 11, 11, 12, 12, 13, 13
    };
    static const uint16_t dist_base[30] = {
        1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
        33,   49,   65,   97,  129,  193,  257,   385,   513,   769,
        1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
    };
    const ptrdiff_t linesize = s->linesize;
    GetBitContext *gb = &s->gb;
    const int height = s->height;
    const int width = s->width;
    int ret, x = s->x, y = s->y;

    for (;;) {
        int sym = decode_symbol(s, lt);

        if (get_bits_left(gb) < 0) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        if (sym < 256) {
            if (y >= height) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            s->dst[linesize * y + x] = sym;

            x++;
            if (x >= width) {
                x = 0;
                y++;
            }
        } else {
            int len, dist, offs, offs_y, offs_x;

            if (sym == 256) {
                s->x = x;
                s->y = y;

                return 0;
            }

            if (sym > lt->max_sym || sym - 257 > 28 || dt->max_sym == -1) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            sym -= 257;

            len = get_bits_base(gb, length_bits[sym],
                                length_base[sym]);

            dist = decode_symbol(s, dt);

            if (dist > dt->max_sym || dist > 29) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            offs = get_bits_base(gb, dist_bits[dist], dist_base[dist]);
            offs = y * width + x - offs;
            if (offs < 0) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            offs_y = offs / width;
            offs_x = offs % width;

            while (len > 0) {
                const int ilen = FFMIN(FFMIN3(width - x, width - offs_x, len), FFABS(offs_x - x) + FFABS(offs_y - y) * width);

                if (y >= height) {
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }

                memmove(s->dst + linesize * y + x, s->dst + linesize * offs_y + offs_x, ilen);

                x += ilen;
                if (x >= width) {
                    x = 0;
                    y++;
                }

                offs_x += ilen;
                if (offs_x >= width) {
                    offs_x = 0;
                    offs_y++;
                }

                len -= ilen;
            }
        }
    }
fail:
    s->x = x;
    s->y = y;

    return ret;
}

static int build_tree(InflateTree *t, const uint8_t *lengths, unsigned num)
{
    unsigned num_codes, available;
    uint16_t offs[16];

    for (int i = 0; i < 16; i++)
        t->counts[i] = 0;

    t->max_sym = -1;

    for (int i = 0; i < num; i++) {
        if (lengths[i]) {
            t->max_sym = i;
            t->counts[lengths[i]]++;
        }
    }

    available = 1;
    num_codes = 0;
    for (int i = 0; i < 16; i++) {
        unsigned int used = t->counts[i];

        if (used > available)
            return AVERROR_INVALIDDATA;

        available = 2 * (available - used);

        offs[i] = num_codes;
        num_codes += used;
    }

    if ((num_codes > 1 && available > 0) ||
        (num_codes == 1 && t->counts[1] != 1))
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < num; i++) {
        if (lengths[i])
            t->symbols[offs[lengths[i]]++] = i;
    }

    if (num_codes == 1) {
        t->counts[1] = 2;
        t->symbols[1] = t->max_sym + 1;
    }

    return 0;
}

static int decode_trees(InflateContext *s, InflateTree *lt, InflateTree *dt)
{
    GetBitContext *gb = &s->gb;
    uint8_t lengths[288 + 32] = { 0 };
    static const uint8_t len_recode[19] = {
        16, 17, 18, 0,  8, 7,  9, 6, 10, 5,
        11,  4, 12, 3, 13, 2, 14, 1, 15
    };
    unsigned hlit, hdist, hclen;
    int ret;

    memset(lt, 0, sizeof(*lt));
    memset(dt, 0, sizeof(*dt));

    hlit = get_bits_base(gb, 5, 257);
    hdist = get_bits_base(gb, 5, 1);
    hclen = get_bits_base(gb, 4, 4);
    if (hlit > 286 || hdist > 30)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < 19; i++)
        lengths[i] = 0;

    for (int i = 0; i < hclen; i++) {
        unsigned clen = get_bits(gb, 3);

        lengths[len_recode[i]] = clen;
    }

    ret = build_tree(lt, lengths, 19);
    if (ret < 0)
        return ret;

    if (lt->max_sym == -1)
        return AVERROR_INVALIDDATA;

    for (int num = 0; num < hlit + hdist;) {
        int len, sym = decode_symbol(s, lt);

        if (sym > lt->max_sym)
            return AVERROR_INVALIDDATA;

        switch (sym) {
        case 16:
            if (num == 0)
                return AVERROR_INVALIDDATA;
            sym = lengths[num - 1];
            len = get_bits_base(gb, 2, 3);
            break;
        case 17:
            sym = 0;
            len = get_bits_base(gb, 3, 3);
            break;
        case 18:
            sym = 0;
            len = get_bits_base(gb, 7, 11);
            break;
        default:
            len = 1;
            break;
        }

        if (len > hlit + hdist - num)
            return AVERROR_INVALIDDATA;

        memset(lengths + num, sym, len);
        num += len;
    }

    if (lengths[256] == 0)
        return AVERROR_INVALIDDATA;

    memset(lt, 0, sizeof(*lt));
    ret = build_tree(lt, lengths, hlit);
    if (ret < 0)
        return ret;

    memset(dt, 0, sizeof(*dt));
    return build_tree(dt, lengths + hlit, hdist);
}

static int inflate_fixed_block(InflateContext *s)
{
    if (!s->fixed_cb_initialized) {
        build_fixed_trees(&s->fixed_ltree, &s->fixed_dtree);
        s->fixed_cb_initialized = 1;
    }

    return inflate_block_data(s, &s->fixed_ltree, &s->fixed_dtree);
}

static int inflate_dynamic_block(InflateContext *s)
{
    int ret = decode_trees(s, &s->dynamic_ltree, &s->dynamic_dtree);

    if (ret < 0)
        return ret;

    return inflate_block_data(s, &s->dynamic_ltree, &s->dynamic_dtree);
}

int ff_inflate(InflateContext *s,
               const uint8_t *src, int src_len,
               uint8_t *dst, int height,
               int width, ptrdiff_t linesize)
{
    GetBitContext *gb = &s->gb;
    int ret, cm, cinfo;
    int bfinal, bmode;
    uint16_t hdr;

    s->x = 0;
    s->y = 0;
    s->dst = dst;
    s->height = height;
    s->width = width;
    s->linesize = linesize;

    ret = init_get_bits8(gb, src, src_len);
    if (ret < 0)
        return ret;

    hdr = show_bits(gb, 16);
    cm = hdr & 0xF;
    cinfo = (hdr >> 4) & 7;

    if (cm == 8 && cinfo <= 7 && ((av_bswap16(hdr) % 31) == 0))
        skip_bits(gb, 16);

    do {
        int len, inv_len;

        bfinal = get_bits1(gb);
        bmode = get_bits(gb, 2);

        switch (bmode) {
        case 0:
            align_get_bits(gb);
            len = get_bits(gb, 16);
            inv_len = get_bits(gb, 16);
            if ((len ^ inv_len) != 0xFFFF)
                return AVERROR_INVALIDDATA;

            while (len > 0) {
                const int ilen = FFMIN(width - s->x, len);

                memcpy(dst + linesize * s->y + s->x, src + (get_bits_count(gb) >> 3), ilen);

                s->x += ilen;
                if (s->x >= width) {
                    s->x = 0;
                    s->y++;
                }

                len -= ilen;
                skip_bits_long(gb, ilen * 8);
            }
            break;
        case 1:
            ret = inflate_fixed_block(s);
            break;
        case 2:
            ret = inflate_dynamic_block(s);
            break;
        case 3:
            return AVERROR_INVALIDDATA;
        }

        if (ret < 0)
            break;
    } while (!bfinal);

    if (ret < 0)
        return ret;

    align_get_bits(gb);
    skip_bits_long(gb, 32);

    return (get_bits_count(gb) >> 3);
}
