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

#include "avfilter.h"
#include "audio.h"

#undef itype
#undef ftype
#undef SAMPLE_FORMAT
#undef SPOT
#if DEPTH == 16
#define SPOT 0
#define SAMPLE_FORMAT s16p
#elif DEPTH == 32
#define SPOT 1
#define SAMPLE_FORMAT s32p
#elif DEPTH == 33
#define SPOT 2
#define SAMPLE_FORMAT fltp
#elif DEPTH == 65
#define SPOT 3
#define SAMPLE_FORMAT dblp
#endif

#undef NBR
#define NBR 1
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 2
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 3
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 4
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 5
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 6
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 7
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 8
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 9
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 10
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 11
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 12
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 13
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 14
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 15
#include "aiirsrc_template_entry_nbr.c"

#undef NBR
#define NBR 16
#include "aiirsrc_template_entry_nbr.c"
