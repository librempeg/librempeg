/*
 * SDR Demuxer / Demodulator Vissualization
 * Copyright (c) 2023 Michael Niedermayer
 *
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

/**
 * @file
 *
 *
 */


#include "sdr.h"

#include <float.h>
#include "libavutil/avassert.h"
#include "libavutil/ffmath.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/avstring.h"
#include "libavutil/xga_font_data.h"
#include "libavformat/avformat.h"

static inline void draw_point_component(uint8_t *frame_buffer, ptrdiff_t stride, int x, int y, int r, int g, int b, int w, int h)
{
    uint8_t *p;

    if (x<0 || y<0 || x>=w || y>=h)
        return;
    p = frame_buffer + 4*x + stride*y;

    p[0] = av_clip_uint8(p[0] + (b>>16));
    p[1] = av_clip_uint8(p[1] + (g>>16));
    p[2] = av_clip_uint8(p[2] + (r>>16));
}

// Draw a point with subpixel precission, (it looked bad otherwise)
static void draw_point(uint8_t *frame_buffer, ptrdiff_t stride, int x, int y, int r, int g, int b, int w, int h)
{
    int px = x>>8;
    int py = y>>8;
    int sx = x&255;
    int sy = y&255;
    int s;

    s = (256 - sx) * (256 - sy);
    draw_point_component(frame_buffer, stride, px  , py  , r*s, g*s, b*s, w, h);
    s = sx * (256 - sy);
    draw_point_component(frame_buffer, stride, px+1, py  , r*s, g*s, b*s, w, h);
    s = (256 - sx) * sy;
    draw_point_component(frame_buffer, stride, px  , py+1, r*s, g*s, b*s, w, h);
    s = sx * sy;
    draw_point_component(frame_buffer, stride, px+1, py+1, r*s, g*s, b*s, w, h);
}

static void draw_char(uint8_t *frame_buffer, ptrdiff_t stride, char ch, int x0, int y0, int xd, int yd, int r, int g, int b, int w, int h)
{
    for(int y = 0; y < 16; y++) {
        int mask = avpriv_vga16_font[16*ch + y];
        for(int x = 0; x < 8; x++) {
            if (mask&0x80)
                draw_point(frame_buffer, stride, x0 + xd*x - yd*y, y0 + yd*x + xd*y, r, g, b, w, h);
            mask<<=1;
        }
    }
}

static void draw_string(uint8_t *frame_buffer, ptrdiff_t stride, char *str, int x0, int y0, int xd, int yd, int r, int g, int b, int w, int h)
{
    while(*str) {
        draw_char(frame_buffer, stride, *str++, x0, y0, xd, yd, r, g, b, w, h);
        x0 += xd*9;
        y0 += yd*9;
    }
}

static void advance_waterfall(SDRStream *sst, int h) {
    if (!sst->frame_buffer_line) {
        memcpy(sst->frame_buffer + sst->frame_size, sst->frame_buffer, sst->frame_size);
        sst->frame_buffer_line = h-1;
    } else
        sst->frame_buffer_line--;
}

int ff_sdr_vissualization(SDRContext *sdr, AVStream *st, AVPacket *pkt)
{
    SDRStream *sst = st->priv_data;
    int w = st->codecpar->width;
    int h = st->codecpar->height;
    int h2 = FFMIN(64, h / 4);
    int frame_index = av_rescale(sdr->pts,        sdr->fps.num, sdr->fps.den * TIMEBASE);
    int  last_index = av_rescale(sdr->last_pts,   sdr->fps.num, sdr->fps.den * TIMEBASE);
    int skip = frame_index == last_index || sdr->missing_streams;
    av_assert0(sdr->missing_streams >= 0);

    if (sdr->block_center_freq) {
        if (sst->last_block_center_freq) {
            int last_center = lrint((F2INDEX(sst->last_block_center_freq) - sdr->block_size) * w / (2*sdr->block_size));

            last_center %= w;
            if (last_center < 0)
                last_center += w;
            av_assert0(last_center >= 0 && last_center < w);

            for(int y= 0; y<h - 1; y++) {
                uint8_t *dst = sst->frame_buffer + 4*w*(sst->frame_buffer_line + y);
                uint8_t *src = dst + 4*w;

                memcpy(dst + 4*last_center, src,                       4*(w - last_center));
                memcpy(dst                , src + 4*(w - last_center), 4*     last_center );
            }
            advance_waterfall(sst, h);
        }
        sst->last_block_center_freq = sdr->block_center_freq;
    }

    for(int x= 0; x<w; x++) {
        int color;
        int idx = 4*(x + sst->frame_buffer_line*w);
        int bindex  =  x    * 2ll * sdr->block_size / w;
        int bindex2 = (x+1) * 2ll * sdr->block_size / w;
        float a = 0;
        av_assert0(bindex2 <= 2 * sdr->block_size);
        for (int i = bindex; i < bindex2; i++) {
            AVComplexFloat sample = sdr->block[i];
            a += len2(sample);
        }
        color = lrintf(log(a)*8 + 32);

        sst->frame_buffer[idx + 0] = color;
        sst->frame_buffer[idx + 1] = color;
        sst->frame_buffer[idx + 2] = color;
        sst->frame_buffer[idx + 3] = 255;
    }

    // Display locations of all vissible stations
//     for(int station_index = 0; station_index<sdr->nb_stations; station_index++) {
//         Station *s = sdr->station[station_index];
//         double f = s->frequency;
// //                     int bw = s->bandwidth;
// //                     int xleft = 256*((f-bw) - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
// //                     int xright= 256*((f+bw) - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
//         int xmid  = 256*( f     - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
//         int g = s->modulation == AM ? 50 : 0;
//         int b = s->modulation == AM ? 0 : 70;
//         int r = s->stream ? 50 : 0;
//
//         draw_point(sst->frame_buffer, 4*w, xmid, 256*(sst->frame_buffer_line+1), r, g, b, w, h);
//     }

    if (!skip) {
        int ret = av_new_packet(pkt, sst->frame_size);
        if (ret < 0)
            return ret;

        for(int y= 0; y<h2; y++) {
            for(int x= 0; x<w; x++) {
                int color;
                int idx = x + y*w;
                int idx_t = (idx / h2) + (idx % h2)*w;
                int bindex  =  idx    * 2ll * sdr->block_size / (w * h2);
                int bindex2 = (idx+1) * 2ll * sdr->block_size / (w * h2);
                float a = 0;
                av_assert0(bindex2 <= 2 * sdr->block_size);
                for (int i = bindex; i < bindex2; i++) {
                    AVComplexFloat sample = sdr->block[i];
                    a += len2(sample);
                }
                color = lrintf(log(a)*9 + 64);

                idx_t *= 4;

                pkt->data[idx_t+0] = color;
                pkt->data[idx_t+1] = color;
                pkt->data[idx_t+2] = color;
                pkt->data[idx_t+3] = 255;
            }
        }

        for (int y= h2; y<h; y++)
            memcpy(pkt->data + 4*y*w, sst->frame_buffer + 4*(y + sst->frame_buffer_line - h2)*w, 4*w);

        Station *station_list[1000];
        int nb_stations = ff_sdr_find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.6, station_list, FF_ARRAY_ELEMS(station_list));
        for(int station_index = 0; station_index<nb_stations; station_index++) {
            Station *s = station_list[station_index];
            double f = s->frequency;
            int xmid  = 256*( f     - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
            char text[100];
            int color = s->stream ? 64 : 32;
            int size = s->stream ? 181 : 128;
            int xd = size, yd = size;

            if (!s->in_station_list)
                continue;

            if (s->name[0]) {
                snprintf(text, sizeof(text), "%s ", s->name);
            } else {
                snprintf(text, sizeof(text), "%s ", ff_sdr_modulation_descs[s->modulation].shortname);
            }
            av_strlcatf(text, sizeof(text), "%f Mhz %d %d %d",
                     f/1000000, (int)s->score, ff_sdr_histogram_score(s), s->timeout);
            draw_string(pkt->data, 4*w, text, xmid + 8*yd, 320*h2, xd, yd, color, color, color, w, h);
            if (s->radiotext[0]) {
                draw_string(pkt->data, 4*w, s->radiotext, xmid + 8*yd, 320*h2 + 24*yd, xd, yd, color, color, color, w, h);
            }
        }
    }

    advance_waterfall(sst, h);

//TODO
//                 draw RDS*
    return skip;
}
