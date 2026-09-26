/*
 * CyberFlix DreamFactory video decoder
 *
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
 *
 * The v4 variant uses the identical bitstream with a simpler
 * frame payload: rows (s16) at +0, width (s16) at +2, scanline stream at +4.
 * There is one global palette per movie segment instead of one per frame; the
 * demuxer delivers it as stream extradata (AVPALETTE layout) and as packet
 * side data on segment changes. Each packet carries a 16-byte prefix: the
 * frame's dirty rect (y0,x0,y1,x1), the segment decode-canvas size (w,h) and
 * the segment window placement (x,y) relative to segment 0. The decode canvas
 * is the segment's movie window (chained segments may change geometry;
 * only the dirty rect is blitted at the window offset onto the persistent
 * visible screen.
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

typedef struct CFDFVideoContext {
    uint8_t *canvas;        /* persistent reference, contiguous stride == width */
    uint8_t *origin;        /* image top-left within canvas */
    size_t   canvas_size;   /* full allocation (margins included) */
    int      width, height, stride;
    uint8_t *sbuf;          /* zero-padded copy of the compressed stream */
    int      sbuf_size;
    uint8_t *visible;       /* v4: persistent rect-blit screen (screen_w*screen_h) */
    int      screen_w, screen_h; /* v4: fixed output screen (segment 0 window) */
    int      have_ref;      /* a reference frame has been built (P-frames follow) */
    int      packet_mode;
    uint32_t pal[256];
} CFDFVideoContext;

enum CFDFVideoPacketMode {
    CFDF_VIDEO_PACKET_D5,
    CFDF_VIDEO_PACKET_EARLY,
};

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

    if (s_end - s < 4)
        return NULL;
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
            if (s_end - sp < 2)
                return NULL;
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
            if (s_end - sp < 2)
                return NULL;
            dwin = (uint32_t)(sp[0] << 8) | sp[1]; sp += 2;
            bitcnt = 16;
            ecx = needed;
        }
    c05:
        a = df_shld16(a, dwin, ecx);
        bitcnt -= ecx;
        if (bitcnt == 0) {
            if (s_end - sp < 2)
                return NULL;
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

/* Decode one frame into the persistent canvas. */
static int df_decode_step(CFDFVideoContext *c, int height, int width,
                          const uint8_t *src, const uint8_t *src_end)
{
    const int stride = c->stride;
    const int r1 = stride, r2 = 2 * stride, r3 = 3 * stride, r4 = 4 * stride;
    const uint8_t *s = src;

    for (int row = 0; row < height; row++) {
        uint8_t *edi = c->origin + (size_t)row * stride;
        const uint8_t *refp;
        uint8_t b;
        int rem = width;

        if (s >= src_end)
            return AVERROR_INVALIDDATA;
        b = *s++;

        if (b == 4) {
            if (src_end - s < width)
                return AVERROR_INVALIDDATA;
            memcpy(edi, s, width);
            s += width;
            continue;
        }
        if (b == 40)
            continue;
        if (b >= 44 && b <= 72) {
            static const int8_t voff[8] = { -4, -3, -2, -1, 1, 2, 3, 4 };
            memcpy(edi, edi + voff[(b - 44) >> 2] * stride, width);
            continue;
        }
        switch (b) {
        case 8:  refp = edi - r4; break;
        case 12: refp = edi - r3; break;
        case 16: refp = edi - r2; break;
        case 20: refp = edi - r1; break;
        case 24: refp = edi + r1; break;
        case 28: refp = edi + r2; break;
        case 32: refp = edi + r3; break;
        case 36: refp = edi + r4; break;
        default:
            return AVERROR_INVALIDDATA;
        }

        while (rem > 0) {
            uint8_t ctrl;
            int len;

            if (s >= src_end)
                return AVERROR_INVALIDDATA;
            ctrl = *s++;
            len = ctrl >> 3;
            if (!len) {
                if (s >= src_end)
                    return AVERROR_INVALIDDATA;
                len = *s++ + 0x20;
            }
            if (len > rem)
                return AVERROR_INVALIDDATA;

            switch (ctrl & 7) {
            case 0: {
                uint8_t *runref = edi;

                refp += len;
                rem  -= len;
                if (s >= src_end)
                    return AVERROR_INVALIDDATA;
                *edi++ = *s++;
                if (len > 1) {
                    s = df_delta(&edi, runref, len - 1, s, src_end);
                    if (!s)
                        return AVERROR_INVALIDDATA;
                }
                break;
            }
            case 1:
                rem -= len;
                s = df_delta(&edi, refp, len, s, src_end);
                if (!s)
                    return AVERROR_INVALIDDATA;
                refp += len;
                break;
            case 2:
                edi += len;
                refp += len;
                rem -= len;
                break;
            case 3:
                rem -= len;
                df_movs(edi, refp, len);
                edi += len;
                refp += len;
                break;
            case 4: {
                uint8_t v;

                v = edi[-1];
                refp += len;
                rem -= len;
                memset(edi, v, len);
                edi += len;
                break;
            }
            case 5:
                if (src_end - s < len)
                    return AVERROR_INVALIDDATA;
                refp += len;
                rem -= len;
                df_movs(edi, s, len);
                edi += len;
                s += len;
                break;
            case 6: {
                uint8_t v;

                if (s >= src_end)
                    return AVERROR_INVALIDDATA;
                v = *s++;
                refp += len;
                rem -= len;
                memset(edi, v, len);
                edi += len;
                break;
            }
            case 7: {
                unsigned off;

                if (src_end - s < 2)
                    return AVERROR_INVALIDDATA;
                off = (unsigned)s[0] | (s[1] << 8);
                s += 2;
                refp += len;
                rem -= len;
                df_movs(edi, edi - off, len);
                edi += len;
                break;
            }
            }
        }
    }

    return 0;
}

static int cfdf_video_alloc(AVCodecContext *avctx, int width, int height)
{
    CFDFVideoContext *c = avctx->priv_data;
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
    c->canvas_size = size;
    c->origin = c->canvas + DF_FRONT_MARGIN + 4 * width;
    return 0;
}

static void cfdf_video_reset(CFDFVideoContext *c)
{
    size_t size;

    if (c->canvas) {
        size = DF_FRONT_MARGIN +
               (size_t)(c->height + 8) * c->stride;
        memset(c->canvas, 0, size);
    }
    c->have_ref = 0;
}

static int cfdf_d5_video_decode(AVCodecContext *avctx, AVFrame *frame,
                                int *got_frame, AVPacket *avpkt)
{
    CFDFVideoContext *c = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int width, height, slen, ret;
    const uint8_t *stream, *stream_end;

    if (avpkt->size < 0x428)
        return AVERROR_INVALIDDATA;

    width  = (int16_t)AV_RL16(buf + 0x22);
    height = (int16_t)AV_RL16(buf + 0x20);

    if (!c->canvas || width != c->width || height != c->height) {
        if ((ret = cfdf_video_alloc(avctx, width, height)) < 0)
            return ret;
        if ((ret = ff_set_dimensions(avctx, width, height)) < 0)
            return ret;
    }

    /* Replaying the key packet must not retain a later reference canvas. */
    if ((avpkt->flags & AV_PKT_FLAG_KEY) && c->have_ref)
        cfdf_video_reset(c);

    /* zero-padded copy of the compressed stream (the original reads past the
     * end into zero-initialised memory) */
    slen = avpkt->size - 0x428;
    if (slen > INT_MAX - DF_STREAM_PAD)
        return AVERROR_INVALIDDATA;
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

    if ((ret = df_decode_step(c, c->height, c->width,
                              stream, stream_end)) < 0)
        return ret;

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

static av_cold int cfdf_video_close(AVCodecContext *avctx)
{
    CFDFVideoContext *c = avctx->priv_data;
    av_freep(&c->canvas);
    av_freep(&c->sbuf);
    av_freep(&c->visible);
    return 0;
}

/* (Re)allocate the segment decode canvas only; the visible screen and the
 * I/P bookkeeping are stream-global and survive chained-segment geometry
 * changes (the engine builds a fresh movie buffer per segment while the
 * screen keeps its contents). */
static int cfdf_early_video_canvas_alloc(CFDFVideoContext *c, int width, int height)
{
    size_t size;

    if (width <= 0 || height <= 0 || width > 2048 || height > 2048)
        return AVERROR_INVALIDDATA;

    av_freep(&c->canvas);
    c->width  = width;
    c->height = height;
    c->stride = width;
    size = DF_FRONT_MARGIN + (size_t)(height + 8) * width;
    c->canvas = av_mallocz(size);
    if (!c->canvas)
        return AVERROR(ENOMEM);
    c->canvas_size = size;
    c->origin = c->canvas + DF_FRONT_MARGIN + 4 * width;
    return 0;
}

static av_cold int cfdf_early_video_init(AVCodecContext *avctx)
{
    CFDFVideoContext *c = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    /* opaque black until the demuxer provides the movie palette */
    for (int i = 0; i < 256; i++)
        c->pal[i] = 0xff000000u;
    if (avctx->extradata && avctx->extradata_size >= AVPALETTE_SIZE)
        memcpy(c->pal, avctx->extradata, AVPALETTE_SIZE);
    /* The two palette endpoints are the Windows GDI static-system-palette
     * reserved colours: index 0 = black, index 255 = white, fixed by the OS.
     * The v4 .MOV stores them reversed (0 = white, 255 = black) as
     * placeholders that are never realised on screen, so override both:
     * index 0 is the clear/blackframe color */
    c->pal[0]   = 0xff000000u;
    c->pal[255] = 0xffffffffu;
    return 0;
}

static int cfdf_early_video_decode(AVCodecContext *avctx, AVFrame *frame,
                                   int *got_frame, AVPacket *avpkt)
{
    CFDFVideoContext *c = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    const uint8_t *stream, *stream_end, *sd;
    int rows, width, cw, ch, px, py, slen, ret;
    int ry0, rx0, ry1, rx1;
    size_t sd_size;

    /* 16-byte prefix (from the demuxer): dirty rect (y0,x0,y1,x1) in
     * segment-canvas coordinates, the segment decode-canvas size (w,h), and
     * the window placement (x,y) relative to segment 0; then the payload */
    if (avpkt->size < 16 + 4)
        return AVERROR_INVALIDDATA;

    ry0 = (int16_t)AV_RL16(buf +  0);
    rx0 = (int16_t)AV_RL16(buf +  2);
    ry1 = (int16_t)AV_RL16(buf +  4);
    rx1 = (int16_t)AV_RL16(buf +  6);
    cw  = (int16_t)AV_RL16(buf +  8);
    ch  = (int16_t)AV_RL16(buf + 10);
    px  = (int16_t)AV_RL16(buf + 12);
    py  = (int16_t)AV_RL16(buf + 14);
    buf += 16;

    rows  = (int16_t)AV_RL16(buf + 0);
    width = (int16_t)AV_RL16(buf + 2);
    if (rows <= 0 || width <= 0)
        return AVERROR_INVALIDDATA;

    /* The decode canvas is the coded frame itself: a DIB whose rows are
     * DWORD-aligned, so the coded width is the movie window width rounded up
     * to a multiple of 4. The window (cw,ch from the demuxer) is what the engine blits
     * to the screen; the padding columns are never shown. Most movies are
     * already 4-aligned, where window and coded width coincide. */
    if (cw <= 0)
        cw = width;
    if (ch <= 0)
        ch = rows;
    if (cw > width || ch > rows)
        return AVERROR_INVALIDDATA;

    /* the output screen is fixed for the stream: segment 0's window (set as
     * stream dimensions by the demuxer) */
    if (!c->visible) {
        c->screen_w = avctx->width  > 0 ? avctx->width  : cw + px;
        c->screen_h = avctx->height > 0 ? avctx->height : ch + py;
        if (c->screen_w <= 0 || c->screen_h <= 0 ||
            c->screen_w > 2048 || c->screen_h > 2048)
            return AVERROR_INVALIDDATA;
        if ((ret = ff_set_dimensions(avctx, c->screen_w, c->screen_h)) < 0)
            return ret;
        c->visible = av_mallocz((size_t)c->screen_w * c->screen_h);
        if (!c->visible)
            return AVERROR(ENOMEM);
    }

    /* a chained segment with new geometry gets a fresh decode canvas — sized
     * by the coded frame, not the window. The screen persists; same-geometry
     * chains keep and reuse the movie buffer. */
    if (!c->canvas || width != c->width || rows != c->height) {
        if ((ret = cfdf_early_video_canvas_alloc(c, width, rows)) < 0)
            return ret;
    }

    /* zero-padded copy of the compressed stream (the original reads past the
     * end into zero-initialised memory) */
    slen = avpkt->size - 16 - 4;
    if (slen > INT_MAX - DF_STREAM_PAD)
        return AVERROR_INVALIDDATA;
    if (slen + DF_STREAM_PAD > c->sbuf_size) {
        av_freep(&c->sbuf);
        c->sbuf_size = slen + DF_STREAM_PAD;
        c->sbuf = av_malloc(c->sbuf_size);
        if (!c->sbuf) { c->sbuf_size = 0; return AVERROR(ENOMEM); }
    }
    memcpy(c->sbuf, buf + 4, slen);
    memset(c->sbuf + slen, 0, DF_STREAM_PAD);
    stream     = c->sbuf;
    stream_end = c->sbuf + slen + DF_STREAM_PAD;

    /* A keyframe restarts the movie: the engine clears the offscreen buffer and
     * the visible screen. (Rare — once per stream; also covers seek replay.) */
    if (avpkt->flags & AV_PKT_FLAG_KEY) {
        cfdf_video_reset(c);
        memset(c->visible, 0, (size_t)c->screen_w * c->screen_h);
    }

    if ((ret = df_decode_step(c, rows, width,
                              stream, stream_end)) < 0)
        return ret;

    /* Rect-bounded blit: the decoder produces a full offscreen frame (kept
     * byte-exact for prediction), but the engine copies only the frame's dirty
     * rect onto the persistent visible screen, at the segment's window
     * placement; pixels outside the rect keep prior screen content. This is
     * what makes overlay frames composite over a held background while frame
     * updates cleanly. The blit is opaque: index 0 is a real black pixel. */
    {
        /* the rect lives in window coordinates: clip to the window */
        int y0 = av_clip(ry0, 0, ch), y1 = av_clip(ry1, 0, ch);
        int x0 = av_clip(rx0, 0, cw), x1 = av_clip(rx1, 0, cw);

        /* clip the placed rect to the screen */
        x0 = FFMAX(x0, -px);
        x1 = FFMIN(x1, c->screen_w - px);
        for (int y = y0; y < y1; y++) {
            int sy = y + py;
            if (sy < 0 || sy >= c->screen_h)
                continue;
            if (x1 > x0)
                memcpy(c->visible + (size_t)sy * c->screen_w + px + x0,
                       c->origin  + (size_t)y  * c->stride   + x0, x1 - x0);
        }
    }

    /* palette changes (chained movie segments) arrive as packet side data */
    sd = av_packet_get_side_data(avpkt, AV_PKT_DATA_PALETTE, &sd_size);
    if (sd && sd_size >= AVPALETTE_SIZE) {
        memcpy(c->pal, sd, AVPALETTE_SIZE);
        c->pal[0]   = 0xff000000u; /* reserved black across segments */
        c->pal[255] = 0xffffffffu; /* reserved white across segments */
    }

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int y = 0; y < c->screen_h; y++)
        memcpy(frame->data[0] + y * frame->linesize[0],
               c->visible + (size_t)y * c->screen_w, c->screen_w);
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

static av_cold int cfdf_video_init(AVCodecContext *avctx)
{
    CFDFVideoContext *c = avctx->priv_data;

    /* D5 packets carry dimensions and palette in-band. Earlier MOV packets
     * use the stream palette as their explicit packet-layout contract. */
    if (!avctx->extradata_size) {
        c->packet_mode = CFDF_VIDEO_PACKET_D5;
        avctx->pix_fmt = AV_PIX_FMT_PAL8;
        return 0;
    }
    if (avctx->extradata_size != AVPALETTE_SIZE)
        return AVERROR_INVALIDDATA;

    c->packet_mode = CFDF_VIDEO_PACKET_EARLY;
    return cfdf_early_video_init(avctx);
}

static int cfdf_video_decode(AVCodecContext *avctx, AVFrame *frame,
                             int *got_frame, AVPacket *avpkt)
{
    CFDFVideoContext *c = avctx->priv_data;

    if (c->packet_mode == CFDF_VIDEO_PACKET_EARLY)
        return cfdf_early_video_decode(avctx, frame, got_frame, avpkt);
    return cfdf_d5_video_decode(avctx, frame, got_frame, avpkt);
}

static void cfdf_video_flush(AVCodecContext *avctx)
{
    CFDFVideoContext *c = avctx->priv_data;

    cfdf_video_reset(c);
    if (c->packet_mode == CFDF_VIDEO_PACKET_EARLY && c->visible)
        memset(c->visible, 0, (size_t)c->screen_w * c->screen_h);
}

const FFCodec ff_cfdf_video_decoder = {
    .p.name         = "cfdf_video",
    CODEC_LONG_NAME("CFDF (Cyberflix DreamFactory) video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CFDF_VIDEO,
    .priv_data_size = sizeof(CFDFVideoContext),
    .init           = cfdf_video_init,
    .close          = cfdf_video_close,
    .flush          = cfdf_video_flush,
    FF_CODEC_DECODE_CB(cfdf_video_decode),
    .p.capabilities = AV_CODEC_CAP_DR1,
};

const FFCodec ff_cfdf_d5_video_decoder = {
    .p.name         = "cfdf_d5_video",
    CODEC_LONG_NAME("CFDF (Cyberflix DreamFactory) video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CFDF_VIDEO,
    .priv_data_size = sizeof(CFDFVideoContext),
    .init           = cfdf_video_init,
    .close          = cfdf_video_close,
    .flush          = cfdf_video_flush,
    FF_CODEC_DECODE_CB(cfdf_video_decode),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
