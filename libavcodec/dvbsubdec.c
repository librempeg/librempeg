/*
 * DVB subtitle decoding
 * Copyright (c) 2005 Ian Caulfield
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

#include "avcodec.h"
#include "get_bits.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "libavutil/colorspace.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/thread.h"

#define DVBSUB_PAGE_SEGMENT     0x10
#define DVBSUB_REGION_SEGMENT   0x11
#define DVBSUB_CLUT_SEGMENT     0x12
#define DVBSUB_OBJECT_SEGMENT   0x13
#define DVBSUB_DISPLAYDEFINITION_SEGMENT 0x14
#define DVBSUB_END_DISPLAY_SEGMENT  0x80

#define cm (ff_crop_tab + MAX_NEG_CROP)

#define RGBA(r,g,b,a) (((unsigned)(a) << 24) | ((r) << 16) | ((g) << 8) | (b))

typedef struct DVBSubCLUT {
    int id;
    int version;

    uint32_t clut4[4];
    uint32_t clut16[16];
    uint32_t clut256[256];

    struct DVBSubCLUT *next;
} DVBSubCLUT;

static DVBSubCLUT default_clut;

typedef struct DVBSubObjectDisplay {
    int object_id;
    int region_id;

    int x_pos;
    int y_pos;

    int fgcolor;
    int bgcolor;

    struct DVBSubObjectDisplay *region_list_next;
    struct DVBSubObjectDisplay *object_list_next;
} DVBSubObjectDisplay;

typedef struct DVBSubObject {
    int id;
    int version;

    int type;

    DVBSubObjectDisplay *display_list;

    struct DVBSubObject *next;
} DVBSubObject;

typedef struct DVBSubRegionDisplay {
    int region_id;

    int x_pos;
    int y_pos;

    struct DVBSubRegionDisplay *next;
} DVBSubRegionDisplay;

typedef struct DVBSubRegion {
    int id;
    int version;

    int width;
    int height;
    int depth;

    int clut;
    int bgcolor;

    uint8_t computed_clut[4*256];
    int has_computed_clut;

    uint8_t *pbuf;
    int buf_size;
    int dirty;

    DVBSubObjectDisplay *display_list;

    struct DVBSubRegion *next;
} DVBSubRegion;

typedef struct DVBSubDisplayDefinition {
    int version;

    int x;
    int y;
    int width;
    int height;
} DVBSubDisplayDefinition;

typedef struct DVBSubContext {
    AVClass *class;
    int composition_id;
    int ancillary_id;

    int version;
    int time_out;
    int compute_edt; /**< if 1 end display time calculated using pts
                          if 0 (Default) calculated using time out */
    int compute_clut;
    int clut_count2[257][256];
    int substream;
    int64_t prev_start;
    DVBSubRegion *region_list;
    DVBSubCLUT   *clut_list;
    DVBSubObject *object_list;

    DVBSubRegionDisplay *display_list;
    DVBSubDisplayDefinition *display_definition;
} DVBSubContext;


static DVBSubObject* get_object(DVBSubContext *ctx, int object_id)
{
    DVBSubObject *ptr = ctx->object_list;

    while (ptr && ptr->id != object_id) {
        ptr = ptr->next;
    }

    return ptr;
}

static DVBSubCLUT* get_clut(DVBSubContext *ctx, int clut_id)
{
    DVBSubCLUT *ptr = ctx->clut_list;

    while (ptr && ptr->id != clut_id) {
        ptr = ptr->next;
    }

    return ptr;
}

static DVBSubRegion* get_region(DVBSubContext *ctx, int region_id)
{
    DVBSubRegion *ptr = ctx->region_list;

    while (ptr && ptr->id != region_id) {
        ptr = ptr->next;
    }

    return ptr;
}

static void delete_region_display_list(DVBSubContext *ctx, DVBSubRegion *region)
{
    DVBSubObject *object, *obj2, **obj2_ptr;
    DVBSubObjectDisplay *display, *obj_disp, **obj_disp_ptr;

    while (region->display_list) {
        display = region->display_list;

        object = get_object(ctx, display->object_id);

        if (object) {
            obj_disp_ptr = &object->display_list;
            obj_disp = *obj_disp_ptr;

            while (obj_disp && obj_disp != display) {
                obj_disp_ptr = &obj_disp->object_list_next;
                obj_disp = *obj_disp_ptr;
            }

            if (obj_disp) {
                *obj_disp_ptr = obj_disp->object_list_next;

                if (!object->display_list) {
                    obj2_ptr = &ctx->object_list;
                    obj2 = *obj2_ptr;

                    while (obj2 != object) {
                        av_assert0(obj2);
                        obj2_ptr = &obj2->next;
                        obj2 = *obj2_ptr;
                    }

                    *obj2_ptr = obj2->next;

                    av_freep(&obj2);
                }
            }
        }

        region->display_list = display->region_list_next;

        av_freep(&display);
    }

}

static void delete_cluts(DVBSubContext *ctx)
{
    while (ctx->clut_list) {
        DVBSubCLUT *clut = ctx->clut_list;

        ctx->clut_list = clut->next;

        av_freep(&clut);
    }
}

static void delete_objects(DVBSubContext *ctx)
{
    while (ctx->object_list) {
        DVBSubObject *object = ctx->object_list;

        ctx->object_list = object->next;

        av_freep(&object);
    }
}

static void delete_regions(DVBSubContext *ctx)
{
    while (ctx->region_list) {
        DVBSubRegion *region = ctx->region_list;

        ctx->region_list = region->next;

        delete_region_display_list(ctx, region);

        av_freep(&region->pbuf);
        av_freep(&region);
    }
}

static av_cold void init_default_clut(void)
{
    int i, r, g, b, a = 0;

    default_clut.id = -1;
    default_clut.next = NULL;

    default_clut.clut4[0] = RGBA(  0,   0,   0,   0);
    default_clut.clut4[1] = RGBA(255, 255, 255, 255);
    default_clut.clut4[2] = RGBA(  0,   0,   0, 255);
    default_clut.clut4[3] = RGBA(127, 127, 127, 255);

    default_clut.clut16[0] = RGBA(  0,   0,   0,   0);
    for (i = 1; i < 16; i++) {
        if (i < 8) {
            r = (i & 1) ? 255 : 0;
            g = (i & 2) ? 255 : 0;
            b = (i & 4) ? 255 : 0;
        } else {
            r = (i & 1) ? 127 : 0;
            g = (i & 2) ? 127 : 0;
            b = (i & 4) ? 127 : 0;
        }
        default_clut.clut16[i] = RGBA(r, g, b, 255);
    }

    default_clut.clut256[0] = RGBA(  0,   0,   0,   0);
    for (i = 1; i < 256; i++) {
        if (i < 8) {
            r = (i & 1) ? 255 : 0;
            g = (i & 2) ? 255 : 0;
            b = (i & 4) ? 255 : 0;
            a = 63;
        } else {
            switch (i & 0x88) {
            case 0x00:
                r = ((i & 1) ? 85 : 0) + ((i & 0x10) ? 170 : 0);
                g = ((i & 2) ? 85 : 0) + ((i & 0x20) ? 170 : 0);
                b = ((i & 4) ? 85 : 0) + ((i & 0x40) ? 170 : 0);
                a = 255;
                break;
            case 0x08:
                r = ((i & 1) ? 85 : 0) + ((i & 0x10) ? 170 : 0);
                g = ((i & 2) ? 85 : 0) + ((i & 0x20) ? 170 : 0);
                b = ((i & 4) ? 85 : 0) + ((i & 0x40) ? 170 : 0);
                a = 127;
                break;
            case 0x80:
                r = 127 + ((i & 1) ? 43 : 0) + ((i & 0x10) ? 85 : 0);
                g = 127 + ((i & 2) ? 43 : 0) + ((i & 0x20) ? 85 : 0);
                b = 127 + ((i & 4) ? 43 : 0) + ((i & 0x40) ? 85 : 0);
                a = 255;
                break;
            case 0x88:
                r = ((i & 1) ? 43 : 0) + ((i & 0x10) ? 85 : 0);
                g = ((i & 2) ? 43 : 0) + ((i & 0x20) ? 85 : 0);
                b = ((i & 4) ? 43 : 0) + ((i & 0x40) ? 85 : 0);
                a = 255;
                break;
            }
        }
        default_clut.clut256[i] = RGBA(r, g, b, a);
    }
}

static av_cold int dvbsub_init_decoder(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    DVBSubContext *ctx = avctx->priv_data;

    if (ctx->substream < 0) {
        ctx->composition_id = -1;
        ctx->ancillary_id   = -1;
    } else if (!avctx->extradata || (avctx->extradata_size < 4) || ((avctx->extradata_size % 5 != 0) && (avctx->extradata_size != 4))) {
        av_log(avctx, AV_LOG_WARNING, "Invalid DVB subtitles stream extradata!\n");
        ctx->composition_id = -1;
        ctx->ancillary_id   = -1;
    } else {
        if (avctx->extradata_size > 5*ctx->substream + 2) {
            ctx->composition_id = AV_RB16(avctx->extradata + 5*ctx->substream);
            ctx->ancillary_id   = AV_RB16(avctx->extradata + 5*ctx->substream + 2);
        } else {
            av_log(avctx, AV_LOG_WARNING, "Selected DVB subtitles sub-stream %d is not available\n", ctx->substream);
            ctx->composition_id = AV_RB16(avctx->extradata);
            ctx->ancillary_id   = AV_RB16(avctx->extradata + 2);
        }
    }

    ctx->version = -1;
    ctx->prev_start = AV_NOPTS_VALUE;

    ff_thread_once(&init_static_once, init_default_clut);

    return 0;
}

static av_cold int dvbsub_close_decoder(AVCodecContext *avctx)
{
    DVBSubContext *ctx = avctx->priv_data;
    DVBSubRegionDisplay *display;

    delete_regions(ctx);

    delete_objects(ctx);

    delete_cluts(ctx);

    av_freep(&ctx->display_definition);

    while (ctx->display_list) {
        display = ctx->display_list;
        ctx->display_list = display->next;

        av_freep(&display);
    }

    return 0;
}

static int dvbsub_read_2bit_string(AVCodecContext *avctx,
                                   uint8_t *destbuf, int dbuf_len,
                                   const uint8_t **srcbuf, int buf_size,
                                   int non_mod, uint8_t *map_table, int x_pos)
{
    GetBitContext gb;

    int bits;
    int run_length;
    int pixels_read = x_pos;

    init_get_bits(&gb, *srcbuf, buf_size << 3);

    destbuf += x_pos;

    while (get_bits_count(&gb) < buf_size << 3 && pixels_read < dbuf_len) {
        bits = get_bits(&gb, 2);

        if (bits) {
            if (non_mod != 1 || bits != 1) {
                if (map_table)
                    *destbuf++ = map_table[bits];
                else
                    *destbuf++ = bits;
            }
            pixels_read++;
        } else {
            bits = get_bits1(&gb);
            if (bits == 1) {
                run_length = get_bits(&gb, 3) + 3;
                bits = get_bits(&gb, 2);

                if (non_mod == 1 && bits == 1)
                    pixels_read += run_length;
                else {
                    if (map_table)
                        bits = map_table[bits];
                    while (run_length-- > 0 && pixels_read < dbuf_len) {
                        *destbuf++ = bits;
                        pixels_read++;
                    }
                }
            } else {
                bits = get_bits1(&gb);
                if (bits == 0) {
                    bits = get_bits(&gb, 2);
                    if (bits == 2) {
                        run_length = get_bits(&gb, 4) + 12;
                        bits = get_bits(&gb, 2);

                        if (non_mod == 1 && bits == 1)
                            pixels_read += run_length;
                        else {
                            if (map_table)
                                bits = map_table[bits];
                            while (run_length-- > 0 && pixels_read < dbuf_len) {
                                *destbuf++ = bits;
                                pixels_read++;
                            }
                        }
                    } else if (bits == 3) {
                        run_length = get_bits(&gb, 8) + 29;
                        bits = get_bits(&gb, 2);

                        if (non_mod == 1 && bits == 1)
                            pixels_read += run_length;
                        else {
                            if (map_table)
                                bits = map_table[bits];
                            while (run_length-- > 0 && pixels_read < dbuf_len) {
                                *destbuf++ = bits;
                                pixels_read++;
                            }
                        }
                    } else if (bits == 1) {
                        if (map_table)
                            bits = map_table[0];
                        else
                            bits = 0;
                        run_length = 2;
                        while (run_length-- > 0 && pixels_read < dbuf_len) {
                            *destbuf++ = bits;
                            pixels_read++;
                        }
                    } else {
                        (*srcbuf) += (get_bits_count(&gb) + 7) >> 3;
                        return pixels_read;
                    }
                } else {
                    if (map_table)
                        bits = map_table[0];
                    else
                        bits = 0;
                    *destbuf++ = bits;
                    pixels_read++;
                }
            }
        }
    }

    if (get_bits(&gb, 6))
        av_log(avctx, AV_LOG_ERROR, "line overflow\n");

    (*srcbuf) += (get_bits_count(&gb) + 7) >> 3;

    return pixels_read;
}

static int dvbsub_read_4bit_string(AVCodecContext *avctx, uint8_t *destbuf, int dbuf_len,
                                   const uint8_t **srcbuf, int buf_size,
                                   int non_mod, uint8_t *map_table, int x_pos)
{
    GetBitContext gb;

    int bits;
    int run_length;
    int pixels_read = x_pos;

    init_get_bits(&gb, *srcbuf, buf_size << 3);

    destbuf += x_pos;

    while (get_bits_count(&gb) < buf_size << 3 && pixels_read < dbuf_len) {
        bits = get_bits(&gb, 4);

        if (bits) {
            if (non_mod != 1 || bits != 1) {
                if (map_table)
                    *destbuf++ = map_table[bits];
                else
                    *destbuf++ = bits;
            }
            pixels_read++;
        } else {
            bits = get_bits1(&gb);
            if (bits == 0) {
                run_length = get_bits(&gb, 3);

                if (run_length == 0) {
                    (*srcbuf) += (get_bits_count(&gb) + 7) >> 3;
                    return pixels_read;
                }

                run_length += 2;

                if (map_table)
                    bits = map_table[0];
                else
                    bits = 0;

                while (run_length-- > 0 && pixels_read < dbuf_len) {
                    *destbuf++ = bits;
                    pixels_read++;
                }
            } else {
                bits = get_bits1(&gb);
                if (bits == 0) {
                    run_length = get_bits(&gb, 2) + 4;
                    bits = get_bits(&gb, 4);

                    if (non_mod == 1 && bits == 1)
                        pixels_read += run_length;
                    else {
                        if (map_table)
                            bits = map_table[bits];
                        while (run_length-- > 0 && pixels_read < dbuf_len) {
                            *destbuf++ = bits;
                            pixels_read++;
                        }
                    }
                } else {
                    bits = get_bits(&gb, 2);
                    if (bits == 2) {
                        run_length = get_bits(&gb, 4) + 9;
                        bits = get_bits(&gb, 4);

                        if (non_mod == 1 && bits == 1)
                            pixels_read += run_length;
                        else {
                            if (map_table)
                                bits = map_table[bits];
                            while (run_length-- > 0 && pixels_read < dbuf_len) {
                                *destbuf++ = bits;
                                pixels_read++;
                            }
                        }
                    } else if (bits == 3) {
                        run_length = get_bits(&gb, 8) + 25;
                        bits = get_bits(&gb, 4);

                        if (non_mod == 1 && bits == 1)
                            pixels_read += run_length;
                        else {
                            if (map_table)
                                bits = map_table[bits];
                            while (run_length-- > 0 && pixels_read < dbuf_len) {
                                *destbuf++ = bits;
                                pixels_read++;
                            }
                        }
                    } else if (bits == 1) {
                        if (map_table)
                            bits = map_table[0];
                        else
                            bits = 0;
                        run_length = 2;
                        while (run_length-- > 0 && pixels_read < dbuf_len) {
                            *destbuf++ = bits;
                            pixels_read++;
                        }
                    } else {
                        if (map_table)
                            bits = map_table[0];
                        else
                            bits = 0;
                        *destbuf++ = bits;
                        pixels_read ++;
                    }
                }
            }
        }
    }

    if (get_bits(&gb, 8))
        av_log(avctx, AV_LOG_ERROR, "line overflow\n");

    (*srcbuf) += (get_bits_count(&gb) + 7) >> 3;

    return pixels_read;
}

static int dvbsub_read_8bit_string(AVCodecContext *avctx,
                                   uint8_t *destbuf, int dbuf_len,
                                    const uint8_t **srcbuf, int buf_size,
                                    int non_mod, uint8_t *map_table, int x_pos)
{
    const uint8_t *sbuf_end = (*srcbuf) + buf_size;
    int bits;
    int run_length;
    int pixels_read = x_pos;

    destbuf += x_pos;

    while (*srcbuf < sbuf_end && pixels_read < dbuf_len) {
        bits = *(*srcbuf)++;

        if (bits) {
            if (non_mod != 1 || bits != 1) {
                if (map_table)
                    *destbuf++ = map_table[bits];
                else
                    *destbuf++ = bits;
            }
            pixels_read++;
        } else {
            bits = *(*srcbuf)++;
            run_length = bits & 0x7f;
            if ((bits & 0x80) == 0) {
                if (run_length == 0) {
                    return pixels_read;
                }

                bits = 0;
            } else {
                bits = *(*srcbuf)++;
            }
            if (non_mod == 1 && bits == 1)
                pixels_read += run_length;
            else {
                if (map_table)
                    bits = map_table[bits];
                while (run_length-- > 0 && pixels_read < dbuf_len) {
                    *destbuf++ = bits;
                    pixels_read++;
                }
            }
        }
    }

    if (*(*srcbuf)++)
        av_log(avctx, AV_LOG_ERROR, "line overflow\n");

    return pixels_read;
}

static void compute_default_clut(DVBSubContext *ctx, uint8_t *clut, AVSubtitleRect *rect, int w, int h)
{
    uint8_t list[256] = {0};
    uint8_t list_inv[256];
    int counttab[256] = {0};
    int (*counttab2)[256] = ctx->clut_count2;
    int count, i, x, y;
    ptrdiff_t stride = rect->linesize[0];

    memset(ctx->clut_count2, 0 , sizeof(ctx->clut_count2));

#define V(x,y) rect->data[0][(x) + (y)*stride]
    for (y = 0; y<h; y++) {
        for (x = 0; x<w; x++) {
            int v = V(x,y) + 1;
            int vl = x     ? V(x-1,y) + 1 : 0;
            int vr = x+1<w ? V(x+1,y) + 1 : 0;
            int vt = y     ? V(x,y-1) + 1 : 0;
            int vb = y+1<h ? V(x,y+1) + 1 : 0;
            counttab[v-1] += !!((v!=vl) + (v!=vr) + (v!=vt) + (v!=vb));
            counttab2[vl][v-1] ++;
            counttab2[vr][v-1] ++;
            counttab2[vt][v-1] ++;
            counttab2[vb][v-1] ++;
        }
    }
#define L(x,y) list[d[(x) + (y)*stride]]

    for (i = 0; i<256; i++) {
        counttab2[i+1][i] = 0;
    }
    for (i = 0; i<256; i++) {
        int bestscore = 0;
        int bestv = 0;

        for (x = 0; x < 256; x++) {
            int scorev = 0;
            if (list[x])
                continue;
            scorev += counttab2[0][x];
            for (y = 0; y < 256; y++) {
                scorev += list[y] * counttab2[y+1][x];
            }

            if (scorev) {
                int score = 1024LL*scorev / counttab[x];
                if (score > bestscore) {
                    bestscore = score;
                    bestv = x;
                }
            }
        }
        if (!bestscore)
            break;
        list    [ bestv ] = 1;
        list_inv[     i ] = bestv;
    }

    count = FFMAX(i - 1, 1);
    for (i--; i >= 0; i--) {
        int v = i * 255 / count;
        AV_WN32(clut + 4*list_inv[i], RGBA(v/2,v,v/2,v));
    }
}


static int save_subtitle_set(AVCodecContext *avctx, AVSubtitle *sub, int *got_output)
{
    DVBSubContext *ctx = avctx->priv_data;
    DVBSubRegionDisplay *display;
    DVBSubDisplayDefinition *display_def = ctx->display_definition;
    DVBSubRegion *region;
    AVSubtitleRect *rect;
    const DVBSubCLUT *clut;
    const uint32_t *clut_table;
    int i;
    int offset_x=0, offset_y=0;
    int ret = 0;


    if (display_def) {
        offset_x = display_def->x;
        offset_y = display_def->y;
    }

    /* Not touching AVSubtitles again*/
    if (sub->num_rects) {
        avpriv_request_sample(ctx, "Different Version of Segment asked Twice");
        return AVERROR_PATCHWELCOME;
    }
    for (display = ctx->display_list; display; display = display->next) {
        region = get_region(ctx, display->region_id);
        if (region && region->dirty)
            sub->num_rects++;
    }

    if (ctx->compute_edt == 0) {
        sub->end_display_time = ctx->time_out * 1000;
        *got_output = 1;
    } else if (ctx->prev_start != AV_NOPTS_VALUE) {
        sub->end_display_time = av_rescale_q((sub->pts - ctx->prev_start ), AV_TIME_BASE_Q, (AVRational){ 1, 1000 }) - 1;
        *got_output = 1;
    }
    if (sub->num_rects > 0) {

        sub->rects = av_calloc(sub->num_rects, sizeof(*sub->rects));
        if (!sub->rects) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        for (i = 0; i < sub->num_rects; i++) {
            sub->rects[i] = av_mallocz(sizeof(*sub->rects[i]));
            if (!sub->rects[i]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
        }

        i = 0;

        for (display = ctx->display_list; display; display = display->next) {
            region = get_region(ctx, display->region_id);

            if (!region)
                continue;

            if (!region->dirty)
                continue;

            rect = sub->rects[i];
            rect->x = display->x_pos + offset_x;
            rect->y = display->y_pos + offset_y;
            rect->w = region->width;
            rect->h = region->height;
            rect->nb_colors = (1 << region->depth);
            rect->type      = SUBTITLE_BITMAP;
            rect->linesize[0] = region->width;

            clut = get_clut(ctx, region->clut);

            if (!clut)
                clut = &default_clut;

            switch (region->depth) {
            case 2:
                clut_table = clut->clut4;
                break;
            case 8:
                clut_table = clut->clut256;
                break;
            case 4:
            default:
                clut_table = clut->clut16;
                break;
            }

            rect->data[1] = av_mallocz(AVPALETTE_SIZE);
            if (!rect->data[1]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            memcpy(rect->data[1], clut_table, (1 << region->depth) * sizeof(*clut_table));

            rect->data[0] = av_memdup(region->pbuf, region->buf_size);
            if (!rect->data[0]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            if ((clut == &default_clut && ctx->compute_clut < 0) || ctx->compute_clut == 1) {
                if (!region->has_computed_clut) {
                    compute_default_clut(ctx, region->computed_clut, rect, rect->w, rect->h);
                    region->has_computed_clut = 1;
                }

                memcpy(rect->data[1], region->computed_clut, sizeof(region->computed_clut));
            }

            i++;
        }
    }

    return 0;
fail:
    if (sub->rects) {
        for (i=0; i < sub->num_rects; i++) {
            rect = sub->rects[i];
            if (rect) {
                av_freep(&rect->data[0]);
                av_freep(&rect->data[1]);
            }
            av_freep(&sub->rects[i]);
        }
        av_freep(&sub->rects);
    }
    sub->num_rects = 0;
    return ret;
}

static void dvbsub_parse_pixel_data_block(AVCodecContext *avctx, DVBSubObjectDisplay *display,
                                          const uint8_t *buf, int buf_size, int top_bottom, int non_mod)
{
    DVBSubContext *ctx = avctx->priv_data;

    DVBSubRegion *region = get_region(ctx, display->region_id);
    const uint8_t *buf_end = buf + buf_size;
    uint8_t *pbuf;
    int x_pos, y_pos;
    int i;

    uint8_t map2to4[] = { 0x0,  0x7,  0x8,  0xf};
    uint8_t map2to8[] = {0x00, 0x77, 0x88, 0xff};
    uint8_t map4to8[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                         0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    uint8_t *map_table;

#if 0
    ff_dlog(avctx, "DVB pixel block size %d, %s field:\n", buf_size,
            top_bottom ? "bottom" : "top");

    for (i = 0; i < buf_size; i++) {
        if (i % 16 == 0)
            ff_dlog(avctx, "0x%8p: ", buf+i);

        ff_dlog(avctx, "%02x ", buf[i]);
        if (i % 16 == 15)
            ff_dlog(avctx, "\n");
    }

    if (i % 16)
        ff_dlog(avctx, "\n");
#endif

    if (!region)
        return;

    pbuf = region->pbuf;
    region->dirty = 1;

    x_pos = display->x_pos;
    y_pos = display->y_pos;

    y_pos += top_bottom;

    while (buf < buf_end) {
        if ((*buf!=0xf0 && x_pos >= region->width) || y_pos >= region->height) {
            av_log(avctx, AV_LOG_ERROR, "Invalid object location! %d-%d %d-%d %02x\n", x_pos, region->width, y_pos, region->height, *buf);
            return;
        }

        switch (*buf++) {
        case 0x10:
            if (region->depth == 8)
                map_table = map2to8;
            else if (region->depth == 4)
                map_table = map2to4;
            else
                map_table = NULL;

            x_pos = dvbsub_read_2bit_string(avctx, pbuf + (y_pos * region->width),
                                            region->width, &buf, buf_end - buf,
                                            non_mod, map_table, x_pos);
            break;
        case 0x11:
            if (region->depth < 4) {
                av_log(avctx, AV_LOG_ERROR, "4-bit pixel string in %d-bit region!\n", region->depth);
                return;
            }

            if (region->depth == 8)
                map_table = map4to8;
            else
                map_table = NULL;

            x_pos = dvbsub_read_4bit_string(avctx, pbuf + (y_pos * region->width),
                                            region->width, &buf, buf_end - buf,
                                            non_mod, map_table, x_pos);
            break;
        case 0x12:
            if (region->depth < 8) {
                av_log(avctx, AV_LOG_ERROR, "8-bit pixel string in %d-bit region!\n", region->depth);
                return;
            }

            x_pos = dvbsub_read_8bit_string(avctx, pbuf + (y_pos * region->width),
                                            region->width, &buf, buf_end - buf,
                                            non_mod, NULL, x_pos);
            break;

        case 0x20:
            map2to4[0] = (*buf) >> 4;
            map2to4[1] = (*buf++) & 0xf;
            map2to4[2] = (*buf) >> 4;
            map2to4[3] = (*buf++) & 0xf;
            break;
        case 0x21:
            for (i = 0; i < 4; i++)
                map2to8[i] = *buf++;
            break;
        case 0x22:
            for (i = 0; i < 16; i++)
                map4to8[i] = *buf++;
            break;

        case 0xf0:
            x_pos = display->x_pos;
            y_pos += 2;
            break;
        default:
            av_log(avctx, AV_LOG_INFO, "Unknown/unsupported pixel block 0x%x\n", *(buf-1));
        }
    }

    if (ctx->compute_clut != -2)
        region->has_computed_clut = 0;
}

static int dvbsub_parse_object_segment(AVCodecContext *avctx,
                                       const uint8_t *buf, int buf_size)
{
    DVBSubContext *ctx = avctx->priv_data;

    const uint8_t *buf_end = buf + buf_size;
    int object_id;
    DVBSubObject *object;
    DVBSubObjectDisplay *display;
    int top_field_len, bottom_field_len;

    int coding_method, non_modifying_color;

    object_id = AV_RB16(buf);
    buf += 2;

    object = get_object(ctx, object_id);

    if (!object)
        return AVERROR_INVALIDDATA;

    coding_method = ((*buf) >> 2) & 3;
    non_modifying_color = ((*buf++) >> 1) & 1;

    if (coding_method == 0) {
        top_field_len = AV_RB16(buf);
        buf += 2;
        bottom_field_len = AV_RB16(buf);
        buf += 2;

        if (buf + top_field_len + bottom_field_len > buf_end) {
            av_log(avctx, AV_LOG_ERROR, "Field data size %d+%d too large\n", top_field_len, bottom_field_len);
            return AVERROR_INVALIDDATA;
        }

        for (display = object->display_list; display; display = display->object_list_next) {
            const uint8_t *block = buf;
            int bfl = bottom_field_len;

            dvbsub_parse_pixel_data_block(avctx, display, block, top_field_len, 0,
                                            non_modifying_color);

            if (bottom_field_len > 0)
                block = buf + top_field_len;
            else
                bfl = top_field_len;

            dvbsub_parse_pixel_data_block(avctx, display, block, bfl, 1,
                                            non_modifying_color);
        }
    } else if (coding_method == 1) {
        avpriv_report_missing_feature(avctx, "coded as a string of characters");
        return AVERROR_PATCHWELCOME;
    } else if (coding_method == 2) {
        avpriv_report_missing_feature(avctx, "progressive coding of pixels");
        return AVERROR_PATCHWELCOME;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Unknown object coding %d\n", coding_method);
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int dvbsub_parse_clut_segment(AVCodecContext *avctx,
                                     const uint8_t *buf, int buf_size)
{
    DVBSubContext *ctx = avctx->priv_data;

    const uint8_t *buf_end = buf + buf_size;
    int i, clut_id;
    int version;
    DVBSubCLUT *clut;
    int entry_id, depth , full_range;
    int y, cr, cb, alpha;
    int r, g, b, r_add, g_add, b_add;

    ff_dlog(avctx, "DVB clut packet:\n");

    for (i=0; i < buf_size; i++) {
        ff_dlog(avctx, "%02x ", buf[i]);
        if (i % 16 == 15)
            ff_dlog(avctx, "\n");
    }

    if (i % 16)
        ff_dlog(avctx, "\n");

    clut_id = *buf++;
    version = ((*buf)>>4)&15;
    buf += 1;

    clut = get_clut(ctx, clut_id);

    if (!clut) {
        clut = av_memdup(&default_clut, sizeof(*clut));
        if (!clut)
            return AVERROR(ENOMEM);

        clut->id = clut_id;
        clut->version = -1;

        clut->next = ctx->clut_list;
        ctx->clut_list = clut;
    }

    if (clut->version != version) {

        clut->version = version;

        while (buf + 4 < buf_end) {
            entry_id = *buf++;

            depth = (*buf) & 0xe0;

            if (depth == 0) {
                av_log(avctx, AV_LOG_ERROR, "Invalid clut depth 0x%x!\n", *buf);
            }

            full_range = (*buf++) & 1;

            if (full_range) {
                y     = *buf++;
                cr    = *buf++;
                cb    = *buf++;
                alpha = *buf++;
            } else {
                y     = buf[0] & 0xfc;
                cr    = (((buf[0] & 3) << 2) | ((buf[1] >> 6) & 3)) << 4;
                cb    = (buf[1] << 2) & 0xf0;
                alpha = (buf[1] << 6) & 0xc0;

                buf += 2;
            }

            if (y == 0)
                alpha = 0xff;

            YUV_TO_RGB1_CCIR(cb, cr);
            YUV_TO_RGB2_CCIR(r, g, b, y);

            ff_dlog(avctx, "clut %d := (%d,%d,%d,%d)\n", entry_id, r, g, b, alpha);
            if (!!(depth & 0x80) + !!(depth & 0x40) + !!(depth & 0x20) > 1) {
                ff_dlog(avctx, "More than one bit level marked: %x\n", depth);
                if (avctx->strict_std_compliance > FF_COMPLIANCE_NORMAL)
                    return AVERROR_INVALIDDATA;
            }

            if (depth & 0x80 && entry_id < 4)
                clut->clut4[entry_id] = RGBA(r,g,b,255 - alpha);
            else if (depth & 0x40 && entry_id < 16)
                clut->clut16[entry_id] = RGBA(r,g,b,255 - alpha);
            else if (depth & 0x20)
                clut->clut256[entry_id] = RGBA(r,g,b,255 - alpha);
        }
    }

    return 0;
}


static int dvbsub_parse_region_segment(AVCodecContext *avctx,
                                       const uint8_t *buf, int buf_size)
{
    DVBSubContext *ctx = avctx->priv_data;

    const uint8_t *buf_end = buf + buf_size;
    int region_id, object_id;
    int av_unused version;
    DVBSubRegion *region;
    DVBSubObject *object;
    DVBSubObjectDisplay *display;
    int fill;
    int ret;

    if (buf_size < 10)
        return AVERROR_INVALIDDATA;

    region_id = *buf++;

    region = get_region(ctx, region_id);

    if (!region) {
        region = av_mallocz(sizeof(*region));
        if (!region)
            return AVERROR(ENOMEM);

        region->id = region_id;
        region->version = -1;

        region->next = ctx->region_list;
        ctx->region_list = region;
    }

    version = ((*buf)>>4) & 15;
    fill = ((*buf++) >> 3) & 1;

    region->width = AV_RB16(buf);
    buf += 2;
    region->height = AV_RB16(buf);
    buf += 2;

    ret = av_image_check_size2(region->width, region->height, avctx->max_pixels, AV_PIX_FMT_PAL8, 0, avctx);
    if (ret >= 0 && region->width * region->height * 2 > 320 * 1024 * 8) {
        ret = AVERROR_INVALIDDATA;
        av_log(avctx, AV_LOG_ERROR, "Pixel buffer memory constraint violated\n");
    }
    if (ret < 0) {
        region->width= region->height= 0;
        return ret;
    }

    if (region->width * region->height != region->buf_size) {
        av_free(region->pbuf);

        region->buf_size = region->width * region->height;

        region->pbuf = av_malloc(region->buf_size);
        if (!region->pbuf) {
            region->buf_size =
            region->width =
            region->height = 0;
            return AVERROR(ENOMEM);
        }

        fill = 1;
        region->dirty = 0;
    }

    region->depth = 1 << (((*buf++) >> 2) & 7);
    if (region->depth < 2 || region->depth > 8) {
        av_log(avctx, AV_LOG_ERROR, "region depth %d is invalid\n", region->depth);
        region->depth= 4;
    }
    region->clut = *buf++;

    if (region->depth == 8) {
        region->bgcolor = *buf++;
        buf += 1;
    } else {
        buf += 1;

        if (region->depth == 4)
            region->bgcolor = (((*buf++) >> 4) & 15);
        else
            region->bgcolor = (((*buf++) >> 2) & 3);
    }

    ff_dlog(avctx, "Region %d, (%dx%d)\n", region_id, region->width, region->height);

    if (fill) {
        memset(region->pbuf, region->bgcolor, region->buf_size);
        ff_dlog(avctx, "Fill region (%d)\n", region->bgcolor);
    }

    delete_region_display_list(ctx, region);

    while (buf + 5 < buf_end) {
        object_id = AV_RB16(buf);
        buf += 2;

        object = get_object(ctx, object_id);

        if (!object) {
            object = av_mallocz(sizeof(*object));
            if (!object)
                return AVERROR(ENOMEM);

            object->id = object_id;
            object->next = ctx->object_list;
            ctx->object_list = object;
        }

        object->type = (*buf) >> 6;

        display = av_mallocz(sizeof(*display));
        if (!display)
            return AVERROR(ENOMEM);

        display->object_id = object_id;
        display->region_id = region_id;

        display->x_pos = AV_RB16(buf) & 0xfff;
        buf += 2;
        display->y_pos = AV_RB16(buf) & 0xfff;
        buf += 2;

        if (display->x_pos >= region->width ||
            display->y_pos >= region->height) {
            av_log(avctx, AV_LOG_ERROR, "Object outside region\n");
            av_free(display);
            return AVERROR_INVALIDDATA;
        }

        if ((object->type == 1 || object->type == 2) && buf+1 < buf_end) {
            display->fgcolor = *buf++;
            display->bgcolor = *buf++;
        }

        display->region_list_next = region->display_list;
        region->display_list = display;

        display->object_list_next = object->display_list;
        object->display_list = display;
    }

    return 0;
}

static int dvbsub_parse_page_segment(AVCodecContext *avctx,
                                     const uint8_t *buf, int buf_size, AVSubtitle *sub, int *got_output)
{
    DVBSubContext *ctx = avctx->priv_data;
    DVBSubRegionDisplay *display;
    DVBSubRegionDisplay *tmp_display_list, **tmp_ptr;

    const uint8_t *buf_end = buf + buf_size;
    int region_id;
    int page_state;
    int timeout;
    int version;

    if (buf_size < 1)
        return AVERROR_INVALIDDATA;

    timeout = *buf++;
    version = ((*buf)>>4) & 15;
    page_state = ((*buf++) >> 2) & 3;

    if (ctx->version == version) {
        return 0;
    }

    ctx->time_out = timeout;
    ctx->version = version;

    ff_dlog(avctx, "Page time out %ds, state %d\n", ctx->time_out, page_state);

    if (ctx->compute_edt == 1)
        save_subtitle_set(avctx, sub, got_output);

    if (page_state == 1 || page_state == 2) {
        delete_regions(ctx);
        delete_objects(ctx);
        delete_cluts(ctx);
    }

    tmp_display_list = ctx->display_list;
    ctx->display_list = NULL;

    while (buf + 5 < buf_end) {
        region_id = *buf++;
        buf += 1;

        display = ctx->display_list;
        while (display && display->region_id != region_id) {
            display = display->next;
        }
        if (display) {
            av_log(avctx, AV_LOG_ERROR, "duplicate region\n");
            break;
        }

        display = tmp_display_list;
        tmp_ptr = &tmp_display_list;

        while (display && display->region_id != region_id) {
            tmp_ptr = &display->next;
            display = display->next;
        }

        if (!display) {
            display = av_mallocz(sizeof(*display));
            if (!display)
                return AVERROR(ENOMEM);
        }

        display->region_id = region_id;

        display->x_pos = AV_RB16(buf);
        buf += 2;
        display->y_pos = AV_RB16(buf);
        buf += 2;

        *tmp_ptr = display->next;

        display->next = ctx->display_list;
        ctx->display_list = display;

        ff_dlog(avctx, "Region %d, (%d,%d)\n", region_id, display->x_pos, display->y_pos);
    }

    while (tmp_display_list) {
        display = tmp_display_list;

        tmp_display_list = display->next;

        av_freep(&display);
    }

    return 0;
}

static int dvbsub_parse_display_definition_segment(AVCodecContext *avctx,
                                                   const uint8_t *buf,
                                                   int buf_size)
{
    DVBSubContext *ctx = avctx->priv_data;
    DVBSubDisplayDefinition *display_def = ctx->display_definition;
    int dds_version, info_byte;

    if (buf_size < 5)
        return AVERROR_INVALIDDATA;

    info_byte   = bytestream_get_byte(&buf);
    dds_version = info_byte >> 4;
    if (display_def && display_def->version == dds_version)
        return 0; // already have this display definition version

    if (!display_def) {
        display_def             = av_mallocz(sizeof(*display_def));
        if (!display_def)
            return AVERROR(ENOMEM);
        ctx->display_definition = display_def;
    }

    display_def->version = dds_version;
    display_def->x       = 0;
    display_def->y       = 0;
    display_def->width   = bytestream_get_be16(&buf) + 1;
    display_def->height  = bytestream_get_be16(&buf) + 1;
    if (!avctx->width || !avctx->height) {
        int ret = ff_set_dimensions(avctx, display_def->width, display_def->height);
        if (ret < 0)
            return ret;
    }

    if (info_byte & 1<<3) { // display_window_flag
        if (buf_size < 13)
            return AVERROR_INVALIDDATA;

        display_def->x = bytestream_get_be16(&buf);
        display_def->width  = bytestream_get_be16(&buf) - display_def->x + 1;
        display_def->y = bytestream_get_be16(&buf);
        display_def->height = bytestream_get_be16(&buf) - display_def->y + 1;
    }

    return 0;
}

static int dvbsub_display_end_segment(AVCodecContext *avctx, const uint8_t *buf,
                                      int buf_size, AVSubtitle *sub,int *got_output)
{
    DVBSubContext *ctx = avctx->priv_data;

    if (ctx->compute_edt == 0)
        save_subtitle_set(avctx, sub, got_output);
    return 0;
}

static int dvbsub_decode(AVCodecContext *avctx, AVSubtitle *sub,
                         int *got_sub_ptr, const AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    DVBSubContext *ctx = avctx->priv_data;
    const uint8_t *p, *p_end;
    int segment_type;
    int page_id;
    int segment_length;
    int i;
    int ret = 0;
    int got_page = 0;
    int got_region = 0;
    int got_object = 0;
    int got_end_display = 0;
    int got_displaydef = 0;

    ff_dlog(avctx, "DVB sub packet:\n");

    for (i=0; i < buf_size; i++) {
        ff_dlog(avctx, "%02x ", buf[i]);
        if (i % 16 == 15)
            ff_dlog(avctx, "\n");
    }

    if (i % 16)
        ff_dlog(avctx, "\n");

    if (buf_size <= 6 || *buf != 0x0f) {
        ff_dlog(avctx, "incomplete or broken packet");
        return AVERROR_INVALIDDATA;
    }

    p = buf;
    p_end = buf + buf_size;

    while (p_end - p >= 6 && *p == 0x0f) {
        p += 1;
        segment_type = *p++;
        page_id = AV_RB16(p);
        p += 2;
        segment_length = AV_RB16(p);
        p += 2;

        if (avctx->debug & FF_DEBUG_STARTCODE) {
            av_log(avctx, AV_LOG_DEBUG, "segment_type:%d page_id:%d segment_length:%d\n", segment_type, page_id, segment_length);
        }

        if (p_end - p < segment_length) {
            ff_dlog(avctx, "incomplete or broken packet");
            ret = -1;
            goto end;
        }

        if (page_id == ctx->composition_id || page_id == ctx->ancillary_id ||
            ctx->composition_id == -1 || ctx->ancillary_id == -1) {
            int ret = 0;
            switch (segment_type) {
            case DVBSUB_PAGE_SEGMENT:
                ret = dvbsub_parse_page_segment(avctx, p, segment_length, sub, got_sub_ptr);
                got_page = 1;
                break;
            case DVBSUB_REGION_SEGMENT:
                ret = dvbsub_parse_region_segment(avctx, p, segment_length);
                got_region = 1;
                break;
            case DVBSUB_CLUT_SEGMENT:
                ret = dvbsub_parse_clut_segment(avctx, p, segment_length);
                if (ret < 0) goto end;
                break;
            case DVBSUB_OBJECT_SEGMENT:
                ret = dvbsub_parse_object_segment(avctx, p, segment_length);
                got_object = 1;
                break;
            case DVBSUB_DISPLAYDEFINITION_SEGMENT:
                ret = dvbsub_parse_display_definition_segment(avctx, p,
                                                              segment_length);
                got_displaydef = 1;
                break;
            case DVBSUB_END_DISPLAY_SEGMENT:
                ret = dvbsub_display_end_segment(avctx, p, segment_length, sub, got_sub_ptr);
                got_end_display = 1;
                break;
            default:
                ff_dlog(avctx, "Subtitling segment type 0x%x, page id %d, length %d\n",
                        segment_type, page_id, segment_length);
                break;
            }
            if (ret < 0)
                goto end;
        }

        p += segment_length;
    }

    // Even though not mandated by the spec, we're imposing a minimum requirement
    // for a useful packet to have at least one page, region and object segment.
    if (got_page && got_region && got_object) {

        if (!got_displaydef && !avctx->width && !avctx->height) {
            // Default from ETSI EN 300 743 V1.3.1 (7.2.1)
            avctx->width  = 720;
            avctx->height = 576;
        }

        // Some streams do not send an end-of-display segment but if we have all the other
        // segments then we need no further data.
        if (!got_end_display) {
            av_log(avctx, AV_LOG_DEBUG, "Missing display_end_segment, emulating\n");
            dvbsub_display_end_segment(avctx, p, 0, sub, got_sub_ptr);
        }
    }
end:
    if (ret < 0) {
        return ret;
    } else {
        if (ctx->compute_edt == 1)
            FFSWAP(int64_t, ctx->prev_start, sub->pts);
    }

    return p - buf;
}

#define DS AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_SUBTITLE_PARAM
#define OFFSET(x) offsetof(DVBSubContext, x)
static const AVOption options[] = {
    {"compute_edt", "compute end of time using pts or timeout", OFFSET(compute_edt), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, DS},
    {"compute_clut", "compute clut when not available(-1) or only once (-2) or always(1) or never(0)", OFFSET(compute_clut), AV_OPT_TYPE_BOOL, {.i64 = -1}, -2, 1, DS},
    {"dvb_substream", "", OFFSET(substream), AV_OPT_TYPE_INT, {.i64 = -1}, -1, 63, DS},
    {NULL}
};
static const AVClass dvbsubdec_class = {
    .class_name = "DVB Sub Decoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_dvbsub_decoder = {
    .p.name         = "dvbsub",
    CODEC_LONG_NAME("DVB subtitles"),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_DVB_SUBTITLE,
    .priv_data_size = sizeof(DVBSubContext),
    .init           = dvbsub_init_decoder,
    .close          = dvbsub_close_decoder,
    FF_CODEC_DECODE_SUB_CB(dvbsub_decode),
    .p.priv_class   = &dvbsubdec_class,
};
