/*
 * Linux audio play and grab interface
 * Copyright (c) 2000, 2001 Fabrice Bellard
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

#include <string.h>

#if HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>

#include "libavutil/file_open.h"
#include "libavutil/log.h"

#include "avdevice.h"

#include "oss.h"

int ff_oss_audio_open(AVFormatContext *s1, int is_output,
                      const char *audio_device)
{
    OSSAudioData *s = s1->priv_data;
    int audio_fd;
    int tmp, err;
    char *flip = getenv("AUDIO_FLIP_LEFT");

    if (is_output)
        audio_fd = avpriv_open(audio_device, O_WRONLY);
    else
        audio_fd = avpriv_open(audio_device, O_RDONLY);
    if (audio_fd < 0) {
        av_log(s1, AV_LOG_ERROR, "%s: %s\n", audio_device, av_err2str(AVERROR(errno)));
        return AVERROR(EIO);
    }

    if (flip && *flip == '1') {
        s->flip_left = 1;
    }

    /* non blocking mode */
    if (!is_output) {
        if (fcntl(audio_fd, F_SETFL, O_NONBLOCK) < 0) {
            av_log(s1, AV_LOG_WARNING, "%s: Could not enable non block mode (%s)\n", audio_device, av_err2str(AVERROR(errno)));
        }
    }

    s->frame_size = OSS_AUDIO_BLOCK_SIZE;

#define CHECK_IOCTL_ERROR(event)                                              \
    if (err < 0) {                                                            \
        av_log(s1, AV_LOG_ERROR, #event ": %s\n", av_err2str(AVERROR(errno)));\
        goto fail;                                                            \
    }

    /* select format : favour native format
     * We don't CHECK_IOCTL_ERROR here because even if failed OSS still may be
     * usable. If OSS is not usable the SNDCTL_DSP_SETFMTS later is going to
     * fail anyway. */
    err = ioctl(audio_fd, SNDCTL_DSP_GETFMTS, &tmp);
    if (err < 0) {
        av_log(s1, AV_LOG_WARNING, "SNDCTL_DSP_GETFMTS: %s\n", av_err2str(AVERROR(errno)));
    }

#if HAVE_BIGENDIAN
    if (tmp & AFMT_S16_BE) {
        tmp = AFMT_S16_BE;
    } else if (tmp & AFMT_S16_LE) {
        tmp = AFMT_S16_LE;
    } else {
        tmp = 0;
    }
#else
    if (tmp & AFMT_S16_LE) {
        tmp = AFMT_S16_LE;
    } else if (tmp & AFMT_S16_BE) {
        tmp = AFMT_S16_BE;
    } else {
        tmp = 0;
    }
#endif

    switch(tmp) {
    case AFMT_S16_LE:
        s->codec_id = AV_CODEC_ID_PCM_S16LE;
        s->sample_size = 2;
        break;
    case AFMT_S16_BE:
        s->codec_id = AV_CODEC_ID_PCM_S16BE;
        s->sample_size = 2;
        break;
    default:
        av_log(s1, AV_LOG_ERROR, "Soundcard does not support 16 bit sample format\n");
        close(audio_fd);
        return AVERROR(EIO);
    }
    err=ioctl(audio_fd, SNDCTL_DSP_SETFMT, &tmp);
    CHECK_IOCTL_ERROR(SNDCTL_DSP_SETFMT)

    tmp = (s->channels == 2);
    err = ioctl(audio_fd, SNDCTL_DSP_STEREO, &tmp);
    CHECK_IOCTL_ERROR(SNDCTL_DSP_STEREO)

    tmp = s->sample_rate;
    err = ioctl(audio_fd, SNDCTL_DSP_SPEED, &tmp);
    CHECK_IOCTL_ERROR(SNDCTL_DSP_SPEED)
    s->sample_rate = tmp; /* store real sample rate */
    s->fd = audio_fd;

    return 0;
 fail:
    close(audio_fd);
    return AVERROR(EIO);
#undef CHECK_IOCTL_ERROR
}

int ff_oss_audio_close(OSSAudioData *s)
{
    close(s->fd);
    return 0;
}
