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

#include "config.h"

#include "libavutil/attributes.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/opus/dsp.h"

void ff_opus_postfilter_fma3(float *data, int period, float *gains, int len);
float ff_opus_deemphasis_fma3(float *out, float *in, float coeff, const float *weights, int len);

av_cold void ff_opus_dsp_init_x86(OpusDSP *ctx)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_FMA3(cpu_flags)) {
        ctx->postfilter = ff_opus_postfilter_fma3;
        ctx->deemphasis = ff_opus_deemphasis_fma3;
    }
}
