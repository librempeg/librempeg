/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVRADIO_AVRADIO_H
#define AVRADIO_AVRADIO_H

#include "version_major.h"
#ifndef HAVE_AV_CONFIG_H
/* When included as part of the ffmpeg build, only include the major version
 * to avoid unnecessary rebuilds. When included externally, keep including
 * the full version information. */
#include "version.h"
#endif

/**
 * @file
 * @ingroup lavr
 * Main libavradio API header
 */

/**
 * @defgroup lavr libavradio
 * Special radio devices muxing/demuxing library.
 *
 * Libavradio is a complementary library to @ref libavf "libavformat". It
 * provides various "special" platform-specific muxers and demuxers, e.g. for
 * Software defined radios. As a consequence, many
 * (de)muxers in libavradio are of the AVFMT_NOFILE type (they use their own
 * I/O functions). The filename passed to avformat_open_input() often does not
 * refer to an actually existing file, but has some special device-specific
 * meaning - e.g. for xcbgrab it is the display name.
 *
 * To use libavradio, simply call avradio_register_all() to register all
 * compiled muxers and demuxers. They all use standard libavformat API.
 *
 * @{
 */

#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/dict.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"

/**
 * Return the LIBAVRADIO_VERSION_INT constant.
 */
unsigned avradio_version(void);

/**
 * Return the libavradio build-time configuration.
 */
const char *avradio_configuration(void);

/**
 * Return the libavradio license.
 */
const char *avradio_license(void);

/**
 * Initialize libavradio and register all the input and output radios.
 */
void avradio_register_all(void);

/**
 * Input radios iterator.
 *
 * If d is NULL, returns the first registered input radio,
 * if d is non-NULL, returns the next registered input radio after d
 * or NULL if d is the last one.
 */
const AVInputFormat *av_input_radio_next(const AVInputFormat  *d);

/**
 * Send control message from application to radio.
 *
 * @param s         radio context.
 * @param type      message type.
 * @param data      message data. Exact type depends on message type.
 * @param data_size size of message data.
 * @return >= 0 on success, negative on error.
 *         AVERROR(ENOSYS) when radio doesn't implement handler of the message.
 */
int avradio_app_to_dev_control_message(struct AVFormatContext *s,
                                        enum AVAppToDevMessageType type,
                                        void *data, size_t data_size);

/**
 * Send control message from radio to application.
 *
 * @param s         radio context.
 * @param type      message type.
 * @param data      message data. Can be NULL.
 * @param data_size size of message data.
 * @return >= 0 on success, negative on error.
 *         AVERROR(ENOSYS) when application doesn't implement handler of the message.
 */
int avradio_dev_to_app_control_message(struct AVFormatContext *s,
                                       enum AVDevToAppMessageType type,
                                       void *data, size_t data_size);

/**
 * @}
 */

#endif /* AVRADIO_AVRADIO_H */
