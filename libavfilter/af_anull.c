/*
 * Copyright (c) 2010 S.N. Hemanth Meenakshisundaram <smeenaks@ucsd.edu>
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

/**
 * @file
 * null audio filter
 */

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "libavutil/internal.h"

static const AVFilterPad anull_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_AUDIO,
        .get_buffer.audio = ff_null_get_audio_buffer,
    },
};

const FFFilter ff_af_anull = {
    .p.name        = "anull",
    .p.description = NULL_IF_CONFIG_SMALL("Pass the source unchanged to the output."),
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(anull_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
};
