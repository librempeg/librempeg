/*
 * CyberFlix DreamFactory audio bank parsing
 *
 * This implementation follows the format documented by vgmstream
 * cfdf_Pre_V4-rc10-gold. See COPYING.vgmstream for its permission notice.
 *
 * This file is part of Librempeg.
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Librempeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef AVFORMAT_CFDF_BANK_H
#define AVFORMAT_CFDF_BANK_H
#include "avformat.h"
typedef struct CFDFBank CFDFBank;
int ff_cfdf_bank_open(AVFormatContext *s, CFDFBank **bank, int big_endian);
int ff_cfdf_bank_packet(AVFormatContext *s, CFDFBank *bank, AVPacket *pkt);
int ff_cfdf_bank_seek(AVFormatContext *s, CFDFBank *bank, int stream, int64_t ts);
void ff_cfdf_bank_close(CFDFBank **bank);
#endif
