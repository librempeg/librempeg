/*
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

#include <stdio.h>

#include "libavutil/pixdesc.h"
#include "libavfilter/drawutils.h"

int main(void)
{
    enum AVPixelFormat f;
    const AVPixFmtDescriptor *desc;
    FFDrawContext draw;
    FFDrawColor color;
    int r, i;

    for (f = 0; av_pix_fmt_desc_get(f); f++) {
        desc = av_pix_fmt_desc_get(f);
        if (!desc->name)
            continue;
        printf("Testing %s...%*s", desc->name,
               (int)(16 - strlen(desc->name)), "");
        r = ff_draw_init(&draw, f, 0);
        if (r < 0) {
            printf("no: %s\n", av_err2str(r));
            continue;
        }
        ff_draw_color(&draw, &color, (uint8_t[]) { 1, 0, 0, 1 });
        for (i = 0; i < sizeof(color); i++)
            if (((uint8_t *)&color)[i] != 128)
                break;
        if (i == sizeof(color)) {
            printf("fallback color\n");
            continue;
        }
        printf("ok\n");
    }
    return 0;
}
