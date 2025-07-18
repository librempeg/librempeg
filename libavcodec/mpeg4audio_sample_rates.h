/*
 * MPEG-4 Audio sample rates
 * Copyright (c) 2008 Baptiste Coudurier <baptiste.coudurier@free.fr>
 * Copyright (c) 2009 Alex Converse <alex.converse@gmail.com>
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

#ifndef AVCODEC_MPEG4AUDIO_SAMPLE_RATES_H
#define AVCODEC_MPEG4AUDIO_SAMPLE_RATES_H

// This table contains only 13 real elements and is padded with zeroes.
// It is used by the AAC encoder as sample rate table, so the encoder
// needs to actually support all of these rates and it needs to have
// a trailing zero.
const int ff_mpeg4audio_sample_rates[16] = {
    96000, 88200, 64000, 48000, 44100, 32000,
    24000, 22050, 16000, 12000, 11025, 8000, 7350
};
#endif
