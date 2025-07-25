/*
 * Copyright (c) 2010 Mans Rullgard <mans@mansr.com>
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

#include "config.h"

#include "libavutil/aarch64/cpu.h"
#include "libavutil/attributes.h"
#include "libavutil/internal.h"
#include "libavcodec/synth_filter.h"

void ff_synth_filter_float_neon(AVTXContext *imdct,
                                float *synth_buf_ptr, int *synth_buf_offset,
                                float synth_buf2[32], const float window[512],
                                float out[32], float in[32],
                                float scale, av_tx_fn imdct_fn);

av_cold void ff_synth_filter_init_aarch64(SynthFilterContext *s)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_neon(cpu_flags))
        s->synth_filter_float = ff_synth_filter_float_neon;
}
