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

#include "libavutil/mem.h"
#define CACHED_BITSTREAM_READER !ARCH_X86_32
#define BITSTREAM_READER_LE
#include "get_bits.h"

#include "inflate.h"

static const uint8_t length_bits[30] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 4, 4, 5, 5, 5, 5, 0, 127
};

static const uint16_t length_base[30] = {
    3,  4,  5,   6,   7,   8,   9,   10,  11, 13,
    15, 17, 19,  23,  27,  31,  35,  43,  51, 59,
    67, 83, 99, 115, 131, 163, 195, 227, 258,  0
};

static const uint8_t dist_bits[30] = {
    0, 0,  0,  0,  1,  1,  2,  2,  3,  3,
    4, 4,  5,  5,  6,  6,  7,  7,  8,  8,
    9, 9, 10, 10, 11, 11, 12, 12, 13, 13
};

static const uint16_t dist_base[30] = {
    1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
    33,   49,   65,   97,   129,  193,  257,  385,   513,   769,
    1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
};

static int build_vlc(InflateTree *t, const int nb,
                     const uint8_t *lens, const uint16_t *symbols)
{
    return ff_vlc_init_from_lengths(&t->vlc, 10, nb, lens, 1,
                                    symbols, 2, 2, 0, VLC_INIT_OUTPUT_LE, NULL);
}

static int build_fixed_trees(InflateTree *lt, InflateTree *dt)
{
    uint16_t symbols[288] = { 0 };
    uint8_t lens[288] = { 0 };
    int ret;

    ff_vlc_free(&lt->vlc);
    ff_vlc_free(&dt->vlc);

    for (int i = 0; i < 24; i++) {
        symbols[i] = 256 + i;
        lens[i] = 7;
    }
    for (int i = 0; i < 144; i++) {
        symbols[24 + i] = i;
        lens[24 + i] = 8;
    }
    for (int i = 0; i < 8; i++) {
        symbols[24 + 144 + i] = 280 + i;
        lens[24 + 144 + i] = 8;
    }
    for (int i = 0; i < 112; i++) {
        symbols[24 + 144 + 8 + i] = 144 + i;
        lens[24 + 144 + 8 + i] = 9;
    }

    lt->max_sym = 285;

    ret = build_vlc(lt, 288, lens, symbols);
    if (ret < 0)
        return ret;

    for (int i = 0; i < 32; i++) {
        symbols[i] = i;
        lens[i] = 5;
    }

    dt->max_sym = 29;

    return build_vlc(dt, 32, lens, symbols);
}

static int decode_symbol(GetBitContext *gb, const VLCElem *tab)
{
    return get_vlc2(gb, tab, 10, 2);
}

static av_always_inline uint32_t get_bits_base(GetBitContext *gb, int bits, int base)
{
    return base + get_bitsz(gb, bits);
}

static int inflate_block_datax(InflateContext *s, InflateTree *lt, InflateTree *dt)
{
    const ptrdiff_t linesize = s->linesize;
    GetBitContext *gb = &s->gb;
    const int height = s->height;
    int width = s->width;
    int ret = 0, x = s->x, y = s->y;
    uint8_t *dst = s->tmp;
    const int dt_max_sym = dt->max_sym;
    const int lt_max_sym = lt->max_sym;
    const VLCElem *dt_tab = dt->vlc.table;
    const VLCElem *lt_tab = lt->vlc.table;
    const unsigned history_size = FF_ARRAY_ELEMS(s->history);
    const unsigned history_mask = history_size-1;
    unsigned history_pos = s->history_pos;
    uint8_t *history = s->history;

    for (;;) {
        int sym = decode_symbol(gb, lt_tab);

        if (sym < 256) {
            dst[x] = sym;
            history[history_pos++] = sym;
            history_pos &= history_mask;

            x++;
            if (x >= width) {
                s->row_fun(s->priv_data, s->dst, linesize, s->tmp, &y, &width, height);
                dst = s->tmp;
                x = 0;
                if (y >= height)
                    break;
            }
        } else if (sym == 256) {
            s->x = x;
            s->y = y;
            s->width = width;
            s->history_pos = history_pos;

            return 0;
        } else {
            int len, dist, offs;

            if (sym > lt_max_sym || sym > 285) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            sym -= 257;

            len = get_bits_base(gb, length_bits[sym],
                                length_base[sym]);

            if (len > ((height - y) * width - x)) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            dist = decode_symbol(gb, dt_tab);

            if (dist < 0 || dist > dt_max_sym || dist > 29) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            offs = get_bits_base(gb, dist_bits[dist], dist_base[dist]);
            offs = ((int)history_pos - offs) & history_mask;

            while (len > 0) {
                const int hmin = FFMIN(history_pos, offs);
                const int hmax = FFMAX(history_pos, offs);
                const int left = FFMIN3(len, history_size - offs, history_size - history_pos);
                const int ix = FFMIN3(width - x, hmax - hmin, left);

                memcpy(dst + x, history + offs, ix);
                memcpy(history + history_pos, dst + x, ix);

                offs += ix;
                offs &= history_mask;

                history_pos += ix;
                history_pos &= history_mask;

                len -= ix;
                x += ix;
                if (x >= width) {
                    s->row_fun(s->priv_data, s->dst, linesize, s->tmp, &y, &width, height);
                    dst = s->tmp;
                    if (y >= height)
                        break;
                    x = 0;
                }
            }
        }
    }
fail:
    s->x = x;
    s->y = y;
    s->width = width;
    s->history_pos = history_pos;

    return ret;
}

static int build_tree(InflateTree *t, const uint8_t *lengths, unsigned num)
{
    uint16_t symbols[288] = { 0 }, counts[16] = { 0 }, offs[16] = { 0 };
    unsigned num_codes, available;
    uint8_t lens[288] = { 0 };

    ff_vlc_free(&t->vlc);

    t->max_sym = -1;
    for (int i = 0; i < num; i++) {
        if (lengths[i]) {
            t->max_sym = i;
            counts[lengths[i]]++;
        }
    }

    if (t->max_sym < 0)
        return AVERROR_INVALIDDATA;

    available = 1;
    num_codes = 0;
    for (int i = 0; i < 16; i++) {
        unsigned used = counts[i];

        if (used > available)
            return AVERROR_INVALIDDATA;

        available = 2 * (available - used);

        offs[i] = num_codes;
        num_codes += used;
    }

    if ((num_codes > 1 && available > 0) ||
        (num_codes == 1 && counts[1] != 1))
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < num; i++) {
        if (lengths[i]) {
            unsigned idx = offs[lengths[i]];

            symbols[idx] = i;
            lens[idx] = lengths[i];
            offs[lengths[i]]++;
        }
    }

    if (num_codes == 1)
        symbols[1] = t->max_sym + 1;

    return build_vlc(t, num_codes, lens, symbols);
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

    ff_vlc_free(&lt->vlc);
    ff_vlc_free(&dt->vlc);

    hlit = get_bits_base(gb, 5, 257);
    hdist = get_bits_base(gb, 5, 1);
    hclen = get_bits_base(gb, 4, 4);
    if (hlit > 286 || hdist > 30)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < hclen; i++) {
        unsigned clen = get_bits(gb, 3);

        lengths[len_recode[i]] = clen;
    }

    ret = build_tree(lt, lengths, 19);
    if (ret < 0)
        return ret;

    for (int num = 0; num < hlit + hdist;) {
        int len, sym = decode_symbol(gb, lt->vlc.table);

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

    ff_vlc_free(&lt->vlc);

    if (lengths[256] == 0)
        return AVERROR_INVALIDDATA;

    ret = build_tree(lt, lengths, hlit);
    if (ret < 0)
        return ret;

    return build_tree(dt, lengths + hlit, hdist);
}

static int inflate_fixed_blockx(InflateContext *s)
{
    if (!s->fixed_cb_initialized) {
        int ret = build_fixed_trees(&s->fixed_ltree, &s->fixed_dtree);

        if (ret < 0)
            return ret;

        s->fixed_cb_initialized = 1;
    }

    return inflate_block_datax(s, &s->fixed_ltree, &s->fixed_dtree);
}

static int inflate_dynamic_blockx(InflateContext *s)
{
    int ret = decode_trees(s, &s->dynamic_ltree, &s->dynamic_dtree);

    if (ret < 0)
        return ret;

    return inflate_block_datax(s, &s->dynamic_ltree, &s->dynamic_dtree);
}

static int inflate_raw_blockx(InflateContext *s)
{
    const ptrdiff_t linesize = s->linesize;
    GetBitContext *gb = &s->gb;
    const int height = s->height;
    int width = s->width;
    int x = s->x, y = s->y, len, inv_len;
    uint8_t *dst = s->tmp;
    const unsigned history_size = FF_ARRAY_ELEMS(s->history);
    const unsigned history_mask = history_size-1;
    unsigned history_pos = s->history_pos;
    uint8_t *history = s->history;

    align_get_bits(gb);
    len = get_bits(gb, 16);
    inv_len = get_bits(gb, 16);
    if ((len ^ inv_len) != 0xFFFF)
        return AVERROR_INVALIDDATA;

    if (get_bits_left(gb) < len * 8)
        return AVERROR_INVALIDDATA;

    if (len > ((height - y) * width - x))
        return AVERROR_INVALIDDATA;

    while (len > 0) {
        const int ilen = FFMIN(width - x, len);

        memcpy(dst + x, gb->buffer + (get_bits_count(gb) >> 3), ilen);
        for (int i = 0; i < ilen; i++) {
            history[history_pos++] = dst[i];
            history_pos &= history_mask;
        }

        x += ilen;
        if (x >= width) {
            x = 0;
            s->row_fun(s->priv_data, s->dst, linesize, s->tmp, &y, &width, height);
            dst = s->tmp;
        }

        len -= ilen;
        skip_bits_long(gb, ilen * 8);
    }

    s->x = x;
    s->y = y;
    s->width = width;
    s->history_pos = history_pos;

    return 0;
}

static int inflate_raw_block(InflateContext *s)
{
    const ptrdiff_t linesize = s->linesize;
    GetBitContext *gb = &s->gb;
    const int height = s->height;
    const int width = s->width;
    int x = s->x, y = s->y, len, inv_len;
    uint8_t *dst = s->dst;

    align_get_bits(gb);
    len = get_bits(gb, 16);
    inv_len = get_bits(gb, 16);
    if ((len ^ inv_len) != 0xFFFF)
        return AVERROR_INVALIDDATA;

    if (get_bits_left(gb) < len * 8)
        return AVERROR_INVALIDDATA;

    if (len > ((height - y) * width - x))
        return AVERROR_INVALIDDATA;

    while (len > 0) {
        const int ilen = FFMIN(width - x, len);

        memcpy(dst + linesize * y + x, gb->buffer + (get_bits_count(gb) >> 3), ilen);

        x += ilen;
        if (x >= width) {
            x = 0;
            y++;
        }

        len -= ilen;
        skip_bits_long(gb, ilen * 8);
    }

    s->x = x;
    s->y = y;

    return 0;
}

static int inflate_block_data(InflateContext *s, InflateTree *lt, InflateTree *dt)
{
    const ptrdiff_t linesize = s->linesize;
    GetBitContext *gb = &s->gb;
    const int height = s->height;
    const int width = s->width;
    int ret = 0, x = s->x, y = s->y;
    uint8_t *dst = s->dst + y * linesize;
    const int dt_max_sym = dt->max_sym;
    const int lt_max_sym = lt->max_sym;
    const VLCElem *dt_tab = dt->vlc.table;
    const VLCElem *lt_tab = lt->vlc.table;

    for (;;) {
        int sym = decode_symbol(gb, lt_tab);

        if (sym < 256) {
            dst[x] = sym;

            x++;
            if (x >= width) {
                dst += linesize;
                x = 0;
                y++;
                if (y >= height)
                    break;
            }
        } else if (sym == 256) {
            s->x = x;
            s->y = y;

            return 0;
        } else {
            int len, dist, offs, offs_y, offs_x;
            uint8_t *odst;

            if (sym > lt_max_sym || sym > 285) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            sym -= 257;

            len = get_bits_base(gb, length_bits[sym],
                                length_base[sym]);

            if (len > ((height - y) * width - x)) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            dist = decode_symbol(gb, dt_tab);

            if (dist < 0 || dist > dt_max_sym || dist > 29) {
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
            offs_x = offs - offs_y * width;
            odst = s->dst + offs_y * linesize;

            while (len > 0) {
                const unsigned ix = width - FFMAX(x, offs_x);
                const unsigned ilen = FFMIN(ix, len);

                if ((offs_y != y) || (x >= offs_x + ilen)) {
                    memcpy(dst + x, odst + offs_x, ilen);
                } else if (x > offs_x) {
                    const int overlap = x - offs_x;

                    av_memcpy_backptr(dst + x, overlap, ilen);
                } else {
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }

                x += ilen;
                if (x >= width) {
                    dst += linesize;
                    x = 0;
                    y++;
                }

                offs_x += ilen;
                if (offs_x >= width) {
                    odst += linesize;
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

static int inflate_fixed_block(InflateContext *s)
{
    if (!s->fixed_cb_initialized) {
        int ret = build_fixed_trees(&s->fixed_ltree, &s->fixed_dtree);

        if (ret < 0)
            return ret;

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

    if (!src && !dst) {
        if (s->fixed_cb_initialized) {
            ff_vlc_free(&s->fixed_ltree.vlc);
            ff_vlc_free(&s->fixed_dtree.vlc);
        }

        return 0;
    }

    s->x = 0;
    s->y = 0;
    s->history_pos = 0;
    s->dst = dst;
    s->height = height;
    s->width = width;
    s->linesize = linesize;
    s->priv_data = NULL;
    s->row_fun = NULL;
    s->tmp = NULL;

    ret = init_get_bits8(gb, src, src_len);
    if (ret < 0)
        goto end;

    hdr = show_bits(gb, 16);
    cm = hdr & 0xF;
    cinfo = (hdr >> 4) & 7;

    if (cm == 8 && cinfo <= 7 && ((av_bswap16(hdr) % 31) == 0))
        skip_bits(gb, 16);

    do {
        bfinal = get_bits1(gb);
        bmode = get_bits(gb, 2);

        switch (bmode) {
        case 0:
            ret = inflate_raw_block(s);
            break;
        case 1:
            ret = inflate_fixed_block(s);
            break;
        case 2:
            ret = inflate_dynamic_block(s);
            break;
        case 3:
            ret = AVERROR_INVALIDDATA;
            break;
        }

        if (ret < 0)
            break;

        if (get_bits_left(gb) < 0) {
            ret = AVERROR_INVALIDDATA;
            break;
        }
    } while (!bfinal);

end:
    ff_vlc_free(&s->dynamic_ltree.vlc);
    ff_vlc_free(&s->dynamic_dtree.vlc);

    if (ret < 0)
        return ret;

    align_get_bits(gb);
    skip_bits_long(gb, 32);

    return (get_bits_count(gb) >> 3);
}

int ff_inflatex(InflateContext *s,
                const uint8_t *src, int src_len,
                uint8_t *dst, int height,
                int width, ptrdiff_t linesize,
                void *priv_data, uint8_t *tmp, void (*row_fun)(void *priv_data, uint8_t *row, ptrdiff_t linesize,
                                                               uint8_t *tmp, int *y, int *width, const int height))
{
    GetBitContext *gb = &s->gb;
    int ret, cm, cinfo;
    int bfinal, bmode;
    uint16_t hdr;

    if (!src && !dst) {
        if (s->fixed_cb_initialized) {
            ff_vlc_free(&s->fixed_ltree.vlc);
            ff_vlc_free(&s->fixed_dtree.vlc);
        }

        return 0;
    }

    s->x = 0;
    s->y = 0;
    s->history_pos = 0;
    s->dst = dst;
    s->height = height;
    s->width = width;
    s->linesize = linesize;
    s->priv_data = priv_data;
    s->row_fun = row_fun;
    s->tmp = tmp;

    ret = init_get_bits8(gb, src, src_len);
    if (ret < 0)
        goto end;

    hdr = show_bits(gb, 16);
    cm = hdr & 0xF;
    cinfo = (hdr >> 4) & 7;

    if (cm == 8 && cinfo <= 7 && ((av_bswap16(hdr) % 31) == 0))
        skip_bits(gb, 16);

    do {
        bfinal = get_bits1(gb);
        bmode = get_bits(gb, 2);

        switch (bmode) {
        case 0:
            ret = inflate_raw_blockx(s);
            break;
        case 1:
            ret = inflate_fixed_blockx(s);
            break;
        case 2:
            ret = inflate_dynamic_blockx(s);
            break;
        case 3:
            ret = AVERROR_INVALIDDATA;
            break;
        }

        if (ret < 0)
            break;

        if (s->y >= s->height)
            break;

        if (get_bits_left(gb) < 0) {
            ret = AVERROR_INVALIDDATA;
            break;
        }
    } while (!bfinal);

end:
    ff_vlc_free(&s->dynamic_ltree.vlc);
    ff_vlc_free(&s->dynamic_dtree.vlc);

    if (ret < 0)
        return ret;

    align_get_bits(gb);
    skip_bits_long(gb, 32);

    return (get_bits_count(gb) >> 3);
}
