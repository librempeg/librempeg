/*
 * RTP network protocol
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

#ifndef AVFORMAT_RTPPROTO_H
#define AVFORMAT_RTPPROTO_H

#include "url.h"

int ff_rtp_set_remote_url(URLContext *h, const char *uri);

int ff_rtp_get_local_rtp_port(URLContext *h);

#endif /* AVFORMAT_RTPPROTO_H */
