/*
 * CyberFlix DreamFactory v5/D5 (cfdf_d5video) video decoder
 * Copyright (c) 2026
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

/*
 * Decoder for the STEP/MFRM paletted (PAL8) frames carried by the
 * DreamFactory 5 .move/.trak container. The frame payload ("this" structure)
 * is laid out as: dimensions at +0x20 (rows, s16) and +0x22 (rowBytes, s16),
 * a 256-entry BGRX palette at +0x28, and the compressed scanline stream at
 * +0x428.
 *
 * The codec is a per-scanline state machine: a control byte selects a row mode
 * (raw, skip, vertical copy, or prediction from a row +/-1..+/-4 away); a
 * predicted row is then produced by an LSB-first prefix code with 8 operations
 * (two delta runs, two RLE runs, skip, literal, reference copy, back-reference)
 * and a bsr-driven sign/magnitude delta sub-decoder. Frames are inter-coded:
 * predictions read both already-updated rows (this frame) and not-yet-updated
 * rows (previous frame), so a persistent reference canvas is maintained.
 *
 * The decoder runs on a private contiguous canvas whose row stride equals the
 * image width (as the original assumes), with margins so the +/-4 row
 * predictors and 16-bit back-references stay in bounds; the image region is
 * then blitted into the output PAL8 frame.
 */

#include <stdint.h>
#include <string.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

/* delta magnitude table */
static const uint8_t df_dtab[16] = {8,8,8,8,8,8,8,7,6,5,4,3,2,1,0,0};

/* back-reference offsets are 16-bit, so the canvas needs that much room before
 * the image origin; the +/-4 row predictors need four rows on each side. */
#define DF_FRONT_MARGIN 0x10000
#define DF_STREAM_PAD   0x4000

typedef struct CFDFD5VideoContext {
    uint8_t *canvas;        /* persistent reference, contiguous stride == width */
    uint8_t *origin;        /* image top-left within canvas */
    int      width, height, stride;
    uint8_t *sbuf;          /* zero-padded copy of the compressed stream */
    int      sbuf_size;
    int      have_ref;      /* a reference frame has been built (P-frames follow) */
    uint32_t pal[256];
} CFDFD5VideoContext;

static int df_bsr16(unsigned v) /* highest set bit, v != 0 */
{
    int i = 15;
    while (!(v & 0x8000)) { v <<= 1; i--; }
    return i;
}

/* ax = (ax << n) | (top n bits of dwin), 16-bit, 1 <= n <= 16 */
static inline uint32_t df_shld16(uint32_t a, uint32_t dwin, int n)
{
    return ((a << n) | ((dwin & 0xffff) >> (16 - n))) & 0xffff;
}

/* faithful forward movs: movsb(len&1) + movsw(len&2) + (len>>2) movsd.
 * For overlapping src < dst (back-reference) the dword copies propagate as
 * rep movsd does, which differs from memmove. */
static inline void df_movs(uint8_t *d, const uint8_t *sp, int len)
{
    if (len & 1) { *d++ = *sp++; }
    if (len & 2) { d[0] = sp[0]; d[1] = sp[1]; d += 2; sp += 2; }
    for (int i = len >> 2; i > 0; i--) {
        d[0] = sp[0]; d[1] = sp[1]; d[2] = sp[2]; d[3] = sp[3];
        d += 4; sp += 4;
    }
}

/* bsr-driven sign/magnitude delta sub-decoder.
 *  refp : reference copy source (DELTA_A: output run start; DELTA_B: predictor)
 *  cnt  : pixel counter (DELTA_A: len-1; DELTA_B: len)
 *  *pedi: dst write pointer (advanced)
 *  returns the resynced stream pointer. */
static const uint8_t *df_delta(uint8_t **pedi, const uint8_t *refp,
                               int cnt, const uint8_t *s, const uint8_t *s_end)
{
    uint8_t *edi = *pedi;
    const uint8_t *sp;
    uint32_t a, dwin;
    int bitcnt, ecx = 0;

    if (s + 4 > s_end) { *pedi = edi; return s; }
    a    = (uint32_t)(s[0] << 8) | s[1]; s += 2;   /* w1 (big-endian) */
    dwin = (uint32_t)(s[0] << 8) | s[1]; s += 2;   /* w2 (big-endian) */
    sp = s;
    bitcnt = 16;

    for (;;) {
        unsigned av = a & 0xffff;
        if (av == 0) goto path_C;
        ecx = df_bsr16(av);
        if (ecx != 15) goto path_B;
        /* path A: copy reference pixel, 1-bit code */
        *edi++ = *refp++;
        a = df_shld16(a, dwin, 1);
        if (--bitcnt != 0) {
            if (--cnt == 0) break;
            dwin = (dwin << 1) & 0xffff;
            continue;
        } else {
            if (--cnt == 0) break;
            if (sp + 2 > s_end) break;
            dwin = (uint32_t)(sp[0] << 8) | sp[1]; sp += 2;
            bitcnt = 16;
            continue;
        }
    path_B:
        if (ecx >= 8) goto path_B1;
    path_C:
        /* cx < 8 or ax == 0: additive literal, consume 16 bits */
        *edi = (uint8_t)((a & 0xff) + *refp);
        edi++; refp++;
        ecx = 16;
        goto consume;
    path_B1:
        ecx = ecx - 1;                     /* cx-1 */
        {
            int signbit = (a >> ecx) & 1;  /* bt ax, cx-1 */
            uint8_t delta = df_dtab[ecx];  /* table load also leaves ecx = delta */
            *edi++ = *refp++;
            if (signbit) edi[-1] += delta;
            else         edi[-1] -= delta;
            ecx = delta + 2;               /* bits to consume = delta + 2 */
        }
        if (bitcnt >= ecx) goto c05;
    consume:
        {
            int needed = ecx, avail = bitcnt;
            a = df_shld16(a, dwin, avail);
            needed -= avail;
            if (sp + 2 > s_end) break;
            dwin = (uint32_t)(sp[0] << 8) | sp[1]; sp += 2;
            bitcnt = 16;
            ecx = needed;
        }
    c05:
        a = df_shld16(a, dwin, ecx);
        bitcnt -= ecx;
        if (bitcnt == 0) {
            if (sp + 2 > s_end) break;
            dwin = (uint32_t)(sp[0] << 8) | sp[1]; sp += 2;
            bitcnt = 16;
            if (--cnt == 0) break;
            continue;
        }
        dwin = (dwin << ecx) & 0xffff;
        if (--cnt == 0) break;
    }

    /* resync the stream pointer: undo the look-ahead word(s) (0x492c2f) */
    {
        int rem = bitcnt;
        const uint8_t *b = sp - 2;
        if (rem != 0) {
            if (rem >= 8)  b--;
            if (rem >= 16) b--;
        }
        *pedi = edi;
        return b;
    }
}

/* Decode one STEP frame into the persistent canvas (origin = image top-left). */
static void df_decode_step(CFDFD5VideoContext *c,
                           const uint8_t *src, const uint8_t *src_end)
{
    const int stride = c->stride, width = c->width;
    const int r1 = stride, r2 = 2*stride, r3 = 3*stride, r4 = 4*stride;
    const uint8_t *s = src;
    /* Each row starts at its anchored origin. For a valid stream the codes of a
     * row consume exactly `width` pixels, so the write pointer naturally lands
     * on the next row; anchoring per row makes the decoder robust against
     * malformed streams whose rows over-run (no cumulative pointer drift). */
    for (int row = 0; row < c->height; row++) {
        uint8_t *edi = c->origin + (size_t)row * stride;
        const uint8_t *refp;
        uint8_t B;
        int rem;

        if (s >= src_end) break;
        B = *s++;

        if (B == 4) {                              /* raw scanline */
            if (s + width > src_end) break;
            memcpy(edi, s, width);
            s += width;
            continue;
        }
        if (B == 40)
            continue;                                /* skip row */
        if (B >= 44 && B <= 72) {                   /* vertical copy */
            static const int voff[8] = {-4,-3,-2,-1,1,2,3,4};
            memcpy(edi, edi + voff[(B - 44) >> 2] * stride, width);
            continue;
        }
        switch (B) {                                /* predicted rows */
        case 8:  refp = edi - r4; break;
        case 12: refp = edi - r3; break;
        case 16: refp = edi - r2; break;
        case 20: refp = edi - r1; break;
        case 24: refp = edi + r1; break;
        case 28: refp = edi + r2; break;
        case 32: refp = edi + r3; break;
        case 36: refp = edi + r4; break;
        default: return;                            /* invalid control byte */
        }

        rem = width;
        while (rem > 0) {
            uint8_t ctrl;
            int len;

            if (s >= src_end) goto row_done;
            ctrl = *s++;
            len = ctrl >> 3;
            if (len == 0) {
                if (s >= src_end) goto row_done;
                len = *s++ + 0x20;
            }
            switch (ctrl & 7) {
            case 0: {  /* DELTA_A: reference is the output (left neighbour) */
                uint8_t *runref;
                rem -= len; refp += len;
                runref = edi;
                if (s >= src_end) goto row_done;
                *edi++ = *s++;
                s = df_delta(&edi, runref, len - 1, s, src_end);
                break; }
            case 1: {  /* DELTA_B: reference is the predictor row */
                rem -= len;
                s = df_delta(&edi, refp, len, s, src_end);
                refp += len;
                break; }
            case 2:    /* skip */
                edi += len; refp += len; rem -= len;
                break;
            case 3:    /* copy from reference row (never overlaps) */
                rem -= len;
                df_movs(edi, refp, len); edi += len; refp += len;
                break;
            case 4: {  /* RLE of previous pixel */
                uint8_t v = edi[-1];
                refp += len; rem -= len;
                memset(edi, v, len); edi += len;
                break; }
            case 5:    /* literal run from stream */
                refp += len; rem -= len;
                if (s + len > src_end) goto row_done;
                df_movs(edi, s, len); edi += len; s += len;
                break;
            case 6: {  /* RLE of new literal pixel */
                uint8_t v;
                refp += len; rem -= len;
                if (s >= src_end) goto row_done;
                v = *s++;
                memset(edi, v, len); edi += len;
                break; }
            case 7: {  /* back-reference (may overlap) */
                unsigned off;
                refp += len; rem -= len;
                if (s + 2 > src_end) goto row_done;
                off = (unsigned)s[0] | (s[1] << 8); s += 2;
                df_movs(edi, edi - off, len); edi += len;
                break; }
            }
        }
    row_done:;
    }
}

static int cfdf_d5_video_alloc(AVCodecContext *avctx, int width, int height)
{
    CFDFD5VideoContext *c = avctx->priv_data;
    size_t size;

    if (width <= 0 || height <= 0 || width > 16384 || height > 16384)
        return AVERROR_INVALIDDATA;

    av_freep(&c->canvas);
    c->width    = width;
    c->height   = height;
    c->stride   = width;
    c->have_ref = 0;
    size = DF_FRONT_MARGIN + (size_t)(height + 8) * width;
    c->canvas = av_mallocz(size);
    if (!c->canvas)
        return AVERROR(ENOMEM);
    c->origin = c->canvas + DF_FRONT_MARGIN + 4 * width;
    return 0;
}

static int cfdf_d5_video_decode(AVCodecContext *avctx, AVFrame *frame,
                                int *got_frame, AVPacket *avpkt)
{
    CFDFD5VideoContext *c = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int width, height, slen, ret;
    const uint8_t *stream, *stream_end;

    if (avpkt->size < 0x428)
        return AVERROR_INVALIDDATA;

    width  = (int16_t)AV_RL16(buf + 0x22);
    height = (int16_t)AV_RL16(buf + 0x20);

    if (!c->canvas || width != c->width || height != c->height) {
        if ((ret = cfdf_d5_video_alloc(avctx, width, height)) < 0)
            return ret;
        if ((ret = ff_set_dimensions(avctx, width, height)) < 0)
            return ret;
    }

    /* zero-padded copy of the compressed stream (the original reads past the
     * end into zero-initialised memory) */
    slen = avpkt->size - 0x428;
    if (slen + DF_STREAM_PAD > c->sbuf_size) {
        av_freep(&c->sbuf);
        c->sbuf_size = slen + DF_STREAM_PAD;
        c->sbuf = av_malloc(c->sbuf_size);
        if (!c->sbuf) { c->sbuf_size = 0; return AVERROR(ENOMEM); }
    }
    memcpy(c->sbuf, buf + 0x428, slen);
    memset(c->sbuf + slen, 0, DF_STREAM_PAD);
    stream     = c->sbuf;
    stream_end = c->sbuf + slen + DF_STREAM_PAD;

    df_decode_step(c, stream, stream_end);

    /* palette: 256 BGRX dwords (0x00RRGGBB) at +0x28 */
    for (int i = 0; i < 256; i++)
        c->pal[i] = 0xff000000u | AV_RL32(buf + 0x28 + i * 4);

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int y = 0; y < c->height; y++)
        memcpy(frame->data[0] + y * frame->linesize[0],
               c->origin + (size_t)y * c->stride, c->width);
    memcpy(frame->data[1], c->pal, AVPALETTE_SIZE);

    if (!c->have_ref) {
        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->flags    |= AV_FRAME_FLAG_KEY;
        c->have_ref      = 1;
    } else {
        frame->pict_type = AV_PICTURE_TYPE_P;
        frame->flags    &= ~AV_FRAME_FLAG_KEY;
    }

    *got_frame = 1;
    return avpkt->size;
}

static av_cold int cfdf_d5_video_init(AVCodecContext *avctx)
{
    avctx->pix_fmt = AV_PIX_FMT_PAL8;
    return 0;
}

static av_cold int cfdf_d5_video_close(AVCodecContext *avctx)
{
    CFDFD5VideoContext *c = avctx->priv_data;
    av_freep(&c->canvas);
    av_freep(&c->sbuf);
    return 0;
}

const FFCodec ff_cfdf_d5_video_decoder = {
    .p.name         = "cfdf_d5_video",
    CODEC_LONG_NAME("CFDF D5 (Cyberflix DreamFactory v5) video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CFDF_D5_VIDEO,
    .priv_data_size = sizeof(CFDFD5VideoContext),
    .init           = cfdf_d5_video_init,
    .close          = cfdf_d5_video_close,
    FF_CODEC_DECODE_CB(cfdf_d5_video_decode),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
