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

#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include "libavutil/frame.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/video_hint.h"

int main(void)
{
    AVVideoHint *hint;
    AVVideoRect *rect;
    AVFrame *frame;
    size_t size;

    static const size_t alloc_counts[] = { 0, 1, 4 };

    /* av_video_hint_alloc - various counts */
    printf("Testing av_video_hint_alloc()\n");
    for (int i = 0; i < FF_ARRAY_ELEMS(alloc_counts); i++) {
        size_t nb = alloc_counts[i];
        hint = av_video_hint_alloc(nb, &size);
        if (hint) {
            printf("alloc %zu: OK, nb_rects=%zu, size>0=%s\n",
                   nb, hint->nb_rects, size > 0 ? "yes" : "no");
            av_free(hint);
        } else {
            printf("alloc %zu: FAIL\n", nb);
        }
    }

    /* pointer consistency and write/read back */
    printf("\nTesting av_video_hint_get_rect()\n");
    hint = av_video_hint_alloc(3, &size);
    if (hint) {
        /* verify av_video_hint_rects points to first rect */
        rect = av_video_hint_rects(hint);
        if ((uint8_t *)rect != (uint8_t *)hint + hint->rect_offset)
            printf("rects: pointer inconsistent with rect_offset\n");

        for (int i = 0; i < 3; i++) {
            rect = av_video_hint_get_rect(hint, i);
            if ((uint8_t *)rect != (uint8_t *)hint + hint->rect_offset +
                                   (size_t)i * hint->rect_size)
                printf("rect %d: pointer inconsistent with rect_offset/rect_size\n", i);
            rect->x = i * 100;
            rect->y = i * 200;
            rect->width = 64 + i;
            rect->height = 48 + i;
        }
        for (int i = 0; i < 3; i++) {
            rect = av_video_hint_get_rect(hint, i);
            printf("rect %d: x=%u y=%u w=%u h=%u\n",
                   i, rect->x, rect->y, rect->width, rect->height);
        }

        av_free(hint);
    }

    /* type field */
    printf("\nTesting type field\n");
    hint = av_video_hint_alloc(1, &size);
    if (hint) {
        hint->type = AV_VIDEO_HINT_TYPE_CONSTANT;
        printf("constant: type=%d\n", hint->type);
        hint->type = AV_VIDEO_HINT_TYPE_CHANGED;
        printf("changed: type=%d\n", hint->type);
        av_free(hint);
    }

    /* av_video_hint_create_side_data */
    printf("\nTesting av_video_hint_create_side_data()\n");
    frame = av_frame_alloc();
    if (frame) {
        hint = av_video_hint_create_side_data(frame, 2);
        if (hint) {
            printf("side_data: OK, nb_rects=%zu\n", hint->nb_rects);
            rect = av_video_hint_get_rect(hint, 0);
            rect->x = 10;
            rect->y = 20;
            rect->width = 320;
            rect->height = 240;
            rect = av_video_hint_get_rect(hint, 0);
            printf("side_data rect 0: x=%u y=%u\n", rect->x, rect->y);
        } else {
            printf("side_data: FAIL\n");
        }
        av_frame_free(&frame);
    }

    /* OOM paths via av_max_alloc */
    printf("\nTesting OOM paths\n");
    av_max_alloc(1);
    hint = av_video_hint_alloc(1, &size);
    printf("alloc OOM: %s\n", hint ? "FAIL" : "OK");
    av_free(hint);
    av_max_alloc(INT_MAX);

    frame = av_frame_alloc();
    if (frame) {
        av_max_alloc(1);
        hint = av_video_hint_create_side_data(frame, 1);
        printf("side_data OOM: %s\n", hint ? "FAIL" : "OK");
        av_max_alloc(INT_MAX);
        av_frame_free(&frame);
    }

    return 0;
}
