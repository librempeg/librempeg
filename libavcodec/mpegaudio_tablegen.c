/*
 * Generate a header file for hardcoded mpegaudiodec tables
 *
 * Copyright (c) 2009 Reimar Döffinger <Reimar.Doeffinger@gmx.de>
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

#include <stdlib.h>
#define CONFIG_HARDCODED_TABLES 0
#define BUILD_TABLES
#include "libavutil/tablegen.h"
#include "mpegaudio_tablegen.h"
#include "tableprint.h"

int main(void)
{
    mpegaudio_tableinit();

    write_fileheader();

    WRITE_ARRAY("static const", uint32_t, exp_table_fixed);
    WRITE_ARRAY("static const", float, exp_table_float);
    WRITE_2D_ARRAY("static const", uint32_t, expval_table_fixed);
    WRITE_2D_ARRAY("static const", float, expval_table_float);

    return 0;
}
