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

#ifndef AVFORMAT_PSX_PROBE_H
#define AVFORMAT_PSX_PROBE_H

static inline int ff_psx_probe(uint8_t *buf, int offset, int size)
{
    int score = 0;

    for (int i = offset; i < size - 2; i += 16) {
        int predictor = (buf[i+0] >> 4) & 15;
        int flags = buf[i+1];

        if (predictor > 5 || flags > 7)
            return 0;

        for (int j = i + 2; j < FFMIN(i + 16, size); j++) {
            if (buf[j]) {
                score++;
                break;
            }
        }
    }

    return score;
}

#endif /* AVFORMAT_PSX_PROBE_H */
