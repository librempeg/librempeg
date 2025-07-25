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

#ifndef AVCODEC_OPUS_DSP_H
#define AVCODEC_OPUS_DSP_H

typedef struct OpusDSP {
    void (*postfilter)(float *data, int period, float *gains, int len);
    float (*deemphasis)(float *out, float *in, float coeff, const float *weights, int len);
} OpusDSP;

void ff_opus_dsp_init(OpusDSP *ctx);

void ff_opus_dsp_init_x86(OpusDSP *ctx);
void ff_opus_dsp_init_aarch64(OpusDSP *ctx);
void ff_opus_dsp_init_riscv(OpusDSP *ctx);

#endif /* AVCODEC_OPUS_DSP_H */
