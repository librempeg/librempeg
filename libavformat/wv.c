/*
 * WavPack shared functions
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

#include <stdint.h>
#include <string.h>

#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/macros.h"

#include "wv.h"

int ff_wv_parse_header(WvHeader *wv, const uint8_t *data)
{
    int is_v4 = 0;

    memset(wv, 0, sizeof(*wv));

    if (AV_RL32(data) != MKTAG('w', 'v', 'p', 'k'))
        return AVERROR_INVALIDDATA;

    wv->blocksize     = AV_RL32(data + 4);
    if (wv->blocksize < 2 || wv->blocksize > WV_BLOCK_LIMIT)
        return AVERROR_INVALIDDATA;

    if (!is_v4)
        wv->lapsed += (4 + 4);

    wv->version       = AV_RL16(data + 8);
    if ((wv->version >= 0x402) && (wv->version <= 0x410)) {
        wv->total_samples = AV_RL32(data + 12);
        wv->block_idx     = AV_RL32(data + 16);
        wv->samples       = AV_RL32(data + 20);
        if (wv->samples >= 0x30000)
            return AVERROR_INVALIDDATA;
        wv->flags         = AV_RL32(data + 24);
        wv->crc           = AV_RL32(data + 28);
        is_v4 = 1;
    } else if ((wv->version >= 2) && (wv->version <= 3)) {
        wv->v2_info = AV_RL16(data + 10); /* may be minimum bit depth (if present) */
        if (wv->version == 3) {
            wv->v3_info = AV_RL16(data + 12); /* has mono flag, precedes (and differs from) v4 flags. */
            if ((wv->v3_info & 0xfff0) != 0)
                return AVERROR_INVALIDDATA;
            /* (todo) rest of the version 3 fields */
            if (AV_RL32(data + 24))
                return AVERROR_INVALIDDATA;
        }
    } else {
        if (wv->version != 1)
            return AVERROR_INVALIDDATA;
        /* (todo) report unknown WavPack version back to the terminal.
         * can only be done at the presence of an avctx context. */
    }

    if (!is_v4)
        wv->lapsed += wv->blocksize;

    wv->blocksize = (is_v4) ? wv->blocksize - 24 : wv->blocksize - wv->blocksize;
    wv->initial = (wv->version >= 4) ? !!(wv->flags & WV_FLAG_INITIAL_BLOCK) : 0;
    wv->final   = (wv->version >= 4) ? !!(wv->flags & WV_FLAG_FINAL_BLOCK) : 0;

    return 0;
}
