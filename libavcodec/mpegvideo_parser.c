/*
 * MPEG-1 / MPEG-2 video parser
 * Copyright (c) 2000,2001 Fabrice Bellard
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
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

#include "libavutil/avassert.h"
#include "decode.h"
#include "parser.h"
#include "mpeg12.h"
#include "mpeg12data.h"
#include "startcode.h"

struct MpvParseContext {
    ParseContext pc;
    AVRational frame_rate;
    int progressive_sequence;
    int width, height;
};

/**
 * Find the end of the current frame in the bitstream.
 * @return the position of the first byte of the next frame, or -1
 */
static int mpeg1_find_frame_end(ParseContext *pc, const uint8_t *buf,
                                int buf_size, AVCodecParserContext *s)
{
    int i;
    uint32_t state = pc->state;

    /* EOF considered as end of frame */
    if (buf_size == 0)
        return 0;

/*
 0  frame start         -> 1/4
 1  first_SEQEXT        -> 0/2
 2  first field start   -> 3/0
 3  second_SEQEXT       -> 2/0
 4  searching end
*/

    for (i = 0; i < buf_size; i++) {
        av_assert1(pc->frame_start_found >= 0 && pc->frame_start_found <= 4);
        if (pc->frame_start_found & 1) {
            if (state == EXT_START_CODE && (buf[i] & 0xF0) != 0x80)
                pc->frame_start_found--;
            else if (state == EXT_START_CODE + 2) {
                if ((buf[i] & 3) == 3)
                    pc->frame_start_found = 0;
                else
                    pc->frame_start_found = (pc->frame_start_found + 1) & 3;
            }
            state++;
        } else {
            i = avpriv_find_start_code(buf + i, buf + buf_size, &state) - buf - 1;
            if (pc->frame_start_found == 0 && state >= SLICE_MIN_START_CODE && state <= SLICE_MAX_START_CODE) {
                i++;
                pc->frame_start_found = 4;
            }
            if (state == SEQ_END_CODE) {
                pc->frame_start_found = 0;
                pc->state = -1;
                return i + 1;
            }
            if (pc->frame_start_found == 2 && state == SEQ_START_CODE)
                pc->frame_start_found = 0;
            if (pc->frame_start_found  < 4 && state == EXT_START_CODE)
                pc->frame_start_found++;
            if (pc->frame_start_found == 4 && (state & 0xFFFFFF00) == 0x100) {
                if (state < SLICE_MIN_START_CODE || state > SLICE_MAX_START_CODE) {
                    pc->frame_start_found = 0;
                    pc->state             = -1;
                    return i - 3;
                }
            }
            if (pc->frame_start_found == 0 && s && state == PICTURE_START_CODE) {
                ff_fetch_timestamp(s, i - 3, 1, i > 3);
            }
        }
    }
    pc->state = state;
    return END_NOT_FOUND;
}

static void mpegvideo_extract_headers(AVCodecParserContext *s,
                                      AVCodecContext *avctx,
                                      const uint8_t *buf, int buf_size)
{
    struct MpvParseContext *pc = s->priv_data;
    const uint8_t *buf_end = buf + buf_size;
    int bytes_left;
    int did_set_size=0;
    int set_dim_ret = 0;
    int bit_rate = 0;
    int vbv_delay = 0;
    enum AVPixelFormat pix_fmt = AV_PIX_FMT_NONE;

    // number of picture coding extensions (i.e. MPEG2 pictures)
    // in this packet - should be 1 or 2
    int nb_pic_ext = 0;
    // when there are two pictures in the packet this indicates
    // which field is in the first of them
    int first_field = AV_FIELD_UNKNOWN;

//FIXME replace the crap with get_bits()
    while (buf < buf_end) {
        uint32_t start_code = -1;
        buf= avpriv_find_start_code(buf, buf_end, &start_code);
        bytes_left = buf_end - buf;
        switch(start_code) {
        case PICTURE_START_CODE:
            if (bytes_left >= 2) {
                s->pict_type = (buf[1] >> 3) & 7;
                if (bytes_left >= 4)
                    vbv_delay = ((buf[1] & 0x07) << 13) | (buf[2] << 5) | (buf[3] >> 3);
            }
            break;
        case SEQ_START_CODE:
            if (bytes_left >= 7) {
                int frame_rate_index;

                pc->width  = (buf[0] << 4) | (buf[1] >> 4);
                pc->height = ((buf[1] & 0x0f) << 8) | buf[2];
                if(!avctx->width || !avctx->height || !avctx->coded_width || !avctx->coded_height){
                    set_dim_ret = ff_set_dimensions(avctx, pc->width, pc->height);
                    did_set_size=1;
                }
                pix_fmt = AV_PIX_FMT_YUV420P;
                frame_rate_index = buf[3] & 0xf;
                pc->frame_rate = avctx->framerate = ff_mpeg12_frame_rate_tab[frame_rate_index];
                bit_rate = (buf[4]<<10) | (buf[5]<<2) | (buf[6]>>6);
                avctx->codec_id = AV_CODEC_ID_MPEG1VIDEO;
            }
            break;
        case EXT_START_CODE:
            if (bytes_left >= 1) {
                switch (buf[0] >> 4) { // ext_type
                case 0x1: /* sequence extension */
                    if (bytes_left >= 6) {
                        int horiz_size_ext   = ((buf[1] & 1) << 1) | (buf[2] >> 7);
                        int  vert_size_ext   = (buf[2] >> 5) & 3;
                        int   bit_rate_ext   = ((buf[2] & 0x1F)<<7) | (buf[3]>>1);
                        int frame_rate_ext_n = (buf[5] >> 5) & 3;
                        int frame_rate_ext_d = (buf[5] & 0x1f);
                        pc->progressive_sequence = buf[1] & (1 << 3);
                        avctx->has_b_frames= !(buf[5] >> 7);

                        switch ((buf[1] >> 1) & 3) { // chroma_format
                        case 1: pix_fmt = AV_PIX_FMT_YUV420P; break;
                        case 2: pix_fmt = AV_PIX_FMT_YUV422P; break;
                        case 3: pix_fmt = AV_PIX_FMT_YUV444P; break;
                        }

                        pc->width  = (pc->width & 0xFFF) | (horiz_size_ext << 12);
                        pc->height = (pc->height& 0xFFF) | ( vert_size_ext << 12);
                        bit_rate = (bit_rate&0x3FFFF) | (bit_rate_ext << 18);
                        if(did_set_size)
                            set_dim_ret = ff_set_dimensions(avctx, pc->width, pc->height);
                        avctx->framerate.num = pc->frame_rate.num * (frame_rate_ext_n + 1);
                        avctx->framerate.den = pc->frame_rate.den * (frame_rate_ext_d + 1);
                        avctx->codec_id = AV_CODEC_ID_MPEG2VIDEO;
                    }
                    break;
                case 0x8: /* picture coding extension */
                    if (bytes_left >= 5) {
                        int    top_field_first = buf[3] & (1 << 7);
                        int repeat_first_field = buf[3] & (1 << 1);
                        int  progressive_frame = buf[4] & (1 << 7);

                        /* check if we must repeat the frame */
                        s->repeat_pict = 1;
                        if (repeat_first_field) {
                            if (pc->progressive_sequence) {
                                if (top_field_first)
                                    s->repeat_pict = 5;
                                else
                                    s->repeat_pict = 3;
                            } else if (progressive_frame) {
                                s->repeat_pict = 2;
                            }
                        }

                        if (!pc->progressive_sequence && !progressive_frame) {
                            if (top_field_first)
                                s->field_order = AV_FIELD_TT;
                            else
                                s->field_order = AV_FIELD_BB;
                        } else
                            s->field_order = AV_FIELD_PROGRESSIVE;

                        s->picture_structure = buf[2] & 3;

                        if (!nb_pic_ext) {
                            // remember parity of the first field for the case
                            // when there are 2 fields in packet
                            switch (s->picture_structure) {
                            case AV_PICTURE_STRUCTURE_BOTTOM_FIELD: first_field = AV_FIELD_BB; break;
                            case AV_PICTURE_STRUCTURE_TOP_FIELD:    first_field = AV_FIELD_TT; break;
                            }
                        }

                        nb_pic_ext++;
                    }
                    break;
                }
            }
            break;
        case -1:
            goto the_end;
        default:
            /* we stop parsing when we encounter a slice. It ensures
               that this function takes a negligible amount of time */
            if (start_code >= SLICE_MIN_START_CODE &&
                start_code <= SLICE_MAX_START_CODE)
                goto the_end;
            break;
        }
    }
 the_end:
    if (set_dim_ret < 0)
        av_log(avctx, AV_LOG_ERROR, "Failed to set dimensions\n");

    if (avctx->codec_id == AV_CODEC_ID_MPEG2VIDEO && bit_rate && bit_rate != 0x3FFFF) {
        avctx->rc_max_rate = 400LL*bit_rate;
    }
    if (bit_rate &&
        ((avctx->codec_id == AV_CODEC_ID_MPEG1VIDEO && bit_rate != 0x3FFFF) || vbv_delay != 0xFFFF)) {
        avctx->bit_rate = 400LL*bit_rate;
    }

    if (pix_fmt != AV_PIX_FMT_NONE) {
        s->format = pix_fmt;
        s->width  = pc->width;
        s->height = pc->height;
        s->coded_width  = FFALIGN(pc->width,  16);
        s->coded_height = FFALIGN(pc->height, 16);
    }

    if (avctx->codec_id == AV_CODEC_ID_MPEG1VIDEO || nb_pic_ext > 1) {
        s->repeat_pict       = 1;
        s->picture_structure = AV_PICTURE_STRUCTURE_FRAME;
        s->field_order       = nb_pic_ext > 1 ? first_field : AV_FIELD_PROGRESSIVE;
    }
}

static int mpegvideo_parse(AVCodecParserContext *s,
                           AVCodecContext *avctx,
                           const uint8_t **poutbuf, int *poutbuf_size,
                           const uint8_t *buf, int buf_size)
{
    struct MpvParseContext *pc1 = s->priv_data;
    ParseContext *pc= &pc1->pc;
    int next;

    if(s->flags & PARSER_FLAG_COMPLETE_FRAMES){
        next= buf_size;
    }else{
        next = mpeg1_find_frame_end(pc, buf, buf_size, s);

        if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }

    }
    /* we have a full frame : we just parse the first few MPEG headers
       to have the full timing information. The time take by this
       function should be negligible for uncorrupted streams */
    mpegvideo_extract_headers(s, avctx, buf, buf_size);
    ff_dlog(NULL, "pict_type=%d frame_rate=%0.3f repeat_pict=%d\n",
            s->pict_type, av_q2d(avctx->framerate), s->repeat_pict);

    *poutbuf = buf;
    *poutbuf_size = buf_size;
    return next;
}

static int mpegvideo_parse_init(AVCodecParserContext *s)
{
    s->pict_type = AV_PICTURE_TYPE_NONE; // first frame might be partial
    return 0;
}

const AVCodecParser ff_mpegvideo_parser = {
    .codec_ids      = { AV_CODEC_ID_MPEG1VIDEO, AV_CODEC_ID_MPEG2VIDEO },
    .priv_data_size = sizeof(struct MpvParseContext),
    .parser_init    = mpegvideo_parse_init,
    .parser_parse   = mpegvideo_parse,
    .parser_close   = ff_parse_close,
};
