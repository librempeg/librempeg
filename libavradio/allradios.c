/*
 * Register all the grabbing radios.
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

#include "libavformat/internal.h"
#include "libavformat/mux.h"
#include "avradio.h"

/* radios */
extern const AVInputFormat  ff_sdr_demuxer;

#include "libavradio/inradio_list.c"

void avradio_register_all(void)
{
    avpriv_register_radios(NULL, inradio_list);
}

static const void *next_input(const AVInputFormat *prev, AVClassCategory c2)
{
    const AVClass *pc;
    const AVClassCategory c1 = AV_CLASS_CATEGORY_RADIO_INPUT;
    AVClassCategory category = AV_CLASS_CATEGORY_NA;
    const AVInputFormat *fmt = NULL;
    int i = 0;

    while (prev && (fmt = inradio_list[i])) {
        i++;
        if (prev == fmt)
            break;
    }

    do {
        fmt = inradio_list[i++];
        if (!fmt)
            break;
        pc = fmt->priv_class;
        if (!pc)
            continue;
        category = pc->category;
    } while (category != c1 && category != c2);
    return fmt;
}

const AVInputFormat *av_input_radio_next(const AVInputFormat  *d)
{
    return next_input(d, AV_CLASS_CATEGORY_RADIO_INPUT);
}

