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

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "avdevice.h"
#include "internal.h"
#include "libavformat/demux.h"
#include "libavformat/mux.h"

int avdevice_app_to_dev_control_message(struct AVFormatContext *s, enum AVAppToDevMessageType type,
                                        void *data, size_t data_size)
{
    if (!s->oformat || !ffofmt(s->oformat)->control_message)
        return AVERROR(ENOSYS);
    return ffofmt(s->oformat)->control_message(s, type, data, data_size);
}

int avdevice_dev_to_app_control_message(struct AVFormatContext *s, enum AVDevToAppMessageType type,
                                        void *data, size_t data_size)
{
    if (!s->control_message_cb)
        return AVERROR(ENOSYS);
    return s->control_message_cb(s, type, data, data_size);
}

int avdevice_list_devices(AVFormatContext *s, AVDeviceInfoList **device_list)
{
    int ret;
    av_assert0(s);
    av_assert0(device_list);
    av_assert0(s->oformat || s->iformat);
    if ((s->oformat && !ffofmt(s->oformat)->get_device_list) ||
        (s->iformat && !ffifmt(s->iformat)->get_device_list)) {
        *device_list = NULL;
        return AVERROR(ENOSYS);
    }
    *device_list = av_mallocz(sizeof(AVDeviceInfoList));
    if (!(*device_list))
        return AVERROR(ENOMEM);
    /* no default device by default */
    (*device_list)->default_device = -1;
    if (s->oformat)
        ret = ffofmt(s->oformat)->get_device_list(s, *device_list);
    else
        ret = ffifmt(s->iformat)->get_device_list(s, *device_list);
    if (ret < 0) {
        avdevice_free_list_devices(device_list);
        return ret;
    }
    return (*device_list)->nb_devices;
}

static int list_devices_for_context(AVFormatContext *s, AVDictionary *options,
                                    AVDeviceInfoList **device_list)
{
    AVDictionary *tmp = NULL;
    int ret;

    av_dict_copy(&tmp, options, 0);
    if ((ret = av_opt_set_dict2(s, &tmp, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;
    ret = avdevice_list_devices(s, device_list);
  fail:
    av_dict_free(&tmp);
    avformat_free_context(s);
    return ret;
}

int avdevice_list_input_sources(const AVInputFormat *device, const char *device_name,
                                AVDictionary *device_options, AVDeviceInfoList **device_list)
{
    AVFormatContext *s = NULL;
    int ret;

    if ((ret = ff_alloc_input_device_context(&s, device, device_name)) < 0)
        return ret;
    return list_devices_for_context(s, device_options, device_list);
}

int avdevice_list_output_sinks(const AVOutputFormat *device, const char *device_name,
                               AVDictionary *device_options, AVDeviceInfoList **device_list)
{
    AVFormatContext *s = NULL;
    int ret;

    if ((ret = avformat_alloc_output_context2(&s, device, device_name, NULL)) < 0)
        return ret;
    return list_devices_for_context(s, device_options, device_list);
}

void avdevice_free_list_devices(AVDeviceInfoList **device_list)
{
    AVDeviceInfoList *list;
    AVDeviceInfo *dev;
    int i;

    av_assert0(device_list);
    list = *device_list;
    if (!list)
        return;

    for (i = 0; i < list->nb_devices; i++) {
        dev = list->devices[i];
        if (dev) {
            av_freep(&dev->device_name);
            av_freep(&dev->device_description);
            av_freep(&dev->media_types);
            av_free(dev);
        }
    }
    av_freep(&list->devices);
    av_freep(device_list);
}
