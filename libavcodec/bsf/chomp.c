/*
 * Chomp bitstream filter
 * Copyright (c) 2010 Alex Converse <alex.converse@gmail.com>
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

#include "bsf.h"
#include "bsf_internal.h"

static int chomp_filter(AVBSFContext *ctx, AVPacket *pkt)
{
    int ret;

    ret = ff_bsf_get_packet_ref(ctx, pkt);
    if (ret < 0)
        return ret;

    while (pkt->size > 0 && !pkt->data[pkt->size - 1])
        pkt->size--;

    return 0;
}

/**
 * This filter removes a string of NULL bytes from the end of a packet.
 */
const FFBitStreamFilter ff_chomp_bsf = {
    .p.name = "chomp",
    .filter = chomp_filter,
};
