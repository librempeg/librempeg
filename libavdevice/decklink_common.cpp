/*
 * Blackmagic DeckLink output
 * Copyright (c) 2013-2014 Ramiro Polla, Luca Barbato, Deti Fliegl
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

/* Include internal.h first to avoid conflict between winsock.h (used by
 * DeckLink headers) and winsock2.h (used by libavformat) in MSVC++ builds */
extern "C" {
#include "libavformat/internal.h"
}

#include <DeckLinkAPI.h>
#ifdef _WIN32
#include <DeckLinkAPI_i.c>
#else
/* The file provided by the SDK is known to be missing prototypes, which doesn't
   cause issues with GCC since the warning doesn't apply to C++ files.  However
   Clang does complain (and warnings are treated as errors), so suppress the
   warning just for this one file */
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#endif
#include <DeckLinkAPIDispatch.cpp>
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#endif

extern "C" {
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/bswap.h"
#include "avdevice.h"
}

#include "decklink_common.h"

static IDeckLinkIterator *decklink_create_iterator(AVFormatContext *avctx)
{
    IDeckLinkIterator *iter;

#ifdef _WIN32
    if (CoInitialize(NULL) < 0) {
        av_log(avctx, AV_LOG_ERROR, "COM initialization failed.\n");
        return NULL;
    }

    if (CoCreateInstance(CLSID_CDeckLinkIterator, NULL, CLSCTX_ALL,
                         IID_IDeckLinkIterator, (void**) &iter) != S_OK) {
        iter = NULL;
    }
#else
    iter = CreateDeckLinkIteratorInstance();
#endif
    if (!iter) {
        av_log(avctx, AV_LOG_ERROR, "Could not create DeckLink iterator. "
                                    "Make sure you have DeckLink drivers " BLACKMAGIC_DECKLINK_API_VERSION_STRING " or newer installed.\n");
    } else {
        IDeckLinkAPIInformation *api;
        int64_t version;
#ifdef _WIN32
        if (CoCreateInstance(CLSID_CDeckLinkAPIInformation, NULL, CLSCTX_ALL,
                             IID_IDeckLinkAPIInformation, (void**) &api) != S_OK) {
            api = NULL;
        }
#else
        api = CreateDeckLinkAPIInformationInstance();
#endif
        if (api && api->GetInt(BMDDeckLinkAPIVersion, &version) == S_OK) {
            if (version < BLACKMAGIC_DECKLINK_API_VERSION)
                av_log(avctx, AV_LOG_WARNING, "Installed DeckLink drivers are too old and may be incompatible with the SDK this module was built against. "
                                              "Make sure you have DeckLink drivers " BLACKMAGIC_DECKLINK_API_VERSION_STRING " or newer installed.\n");
        } else {
            av_log(avctx, AV_LOG_ERROR, "Failed to check installed DeckLink API version.\n");
        }
        if (api)
            api->Release();
    }

    return iter;
}

static int decklink_get_attr_string(IDeckLink *dl, BMDDeckLinkAttributeID cfg_id, const char **s)
{
    DECKLINK_STR tmp;
    HRESULT hr;
    IDeckLinkProfileAttributes *attr;
    *s = NULL;
    if (dl->QueryInterface(IID_IDeckLinkProfileAttributes, (void **)&attr) != S_OK)
        return AVERROR_EXTERNAL;
    hr = attr->GetString(cfg_id, &tmp);
    attr->Release();
    if (hr == S_OK) {
        *s = DECKLINK_STRDUP(tmp);
        DECKLINK_FREE(tmp);
        if (!*s)
            return AVERROR(ENOMEM);
    } else if (hr == E_FAIL) {
        return AVERROR_EXTERNAL;
    }
    return 0;
}

static int decklink_select_input(AVFormatContext *avctx, BMDDeckLinkConfigurationID cfg_id)
{
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;
    BMDDeckLinkAttributeID attr_id = (cfg_id == bmdDeckLinkConfigAudioInputConnection) ? BMDDeckLinkAudioInputConnections : BMDDeckLinkVideoInputConnections;
    int64_t bmd_input              = (cfg_id == bmdDeckLinkConfigAudioInputConnection) ? (int64_t)ctx->audio_input : (int64_t)ctx->video_input;
    const char *type_name          = (cfg_id == bmdDeckLinkConfigAudioInputConnection) ? "audio" : "video";
    int64_t supported_connections = 0;
    HRESULT res;

    if (bmd_input) {
        res = ctx->attr->GetInt(attr_id, &supported_connections);
        if (res != S_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to query supported %s inputs.\n", type_name);
            return AVERROR_EXTERNAL;
        }
        if ((supported_connections & bmd_input) != bmd_input) {
            av_log(avctx, AV_LOG_ERROR, "Device does not support selected %s input.\n", type_name);
            return AVERROR(ENOSYS);
        }
        res = ctx->cfg->SetInt(cfg_id, bmd_input);
        if (res != S_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to select %s input.\n", type_name);
            return AVERROR_EXTERNAL;
        }
    }
    return 0;
}

static DECKLINK_BOOL field_order_eq(enum AVFieldOrder field_order, BMDFieldDominance bmd_field_order)
{
    if (field_order == AV_FIELD_UNKNOWN)
        return true;
    if ((field_order == AV_FIELD_TT || field_order == AV_FIELD_TB) && bmd_field_order == bmdUpperFieldFirst)
        return true;
    if ((field_order == AV_FIELD_BB || field_order == AV_FIELD_BT) && bmd_field_order == bmdLowerFieldFirst)
        return true;
    if (field_order == AV_FIELD_PROGRESSIVE && (bmd_field_order == bmdProgressiveFrame || bmd_field_order == bmdProgressiveSegmentedFrame))
        return true;
    return false;
}

int ff_decklink_set_configs(AVFormatContext *avctx,
                            decklink_direction_t direction) {
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;
    HRESULT res;

    if (ctx->duplex_mode) {
        DECKLINK_BOOL duplex_supported = false;

#if BLACKMAGIC_DECKLINK_API_VERSION >= 0x0b000000
        IDeckLinkProfileManager *manager = NULL;
        if (ctx->dl->QueryInterface(IID_IDeckLinkProfileManager, (void **)&manager) == S_OK)
            duplex_supported = true;
#else
        if (ctx->attr->GetFlag(BMDDeckLinkSupportsDuplexModeConfiguration, &duplex_supported) != S_OK)
            duplex_supported = false;
#endif

        if (duplex_supported) {
#if BLACKMAGIC_DECKLINK_API_VERSION >= 0x0b000000
            IDeckLinkProfile *profile = NULL;
            BMDProfileID bmd_profile_id;

            if (ctx->duplex_mode < 0 || ctx->duplex_mode >= FF_ARRAY_ELEMS(decklink_profile_id_map))
                return EINVAL;
            bmd_profile_id = decklink_profile_id_map[ctx->duplex_mode];
            res = manager->GetProfile(bmd_profile_id, &profile);
            if (res == S_OK) {
                res = profile->SetActive();
                profile->Release();
            }
            manager->Release();
#else
            res = ctx->cfg->SetInt(bmdDeckLinkConfigDuplexMode, ctx->duplex_mode == 2 ? bmdDuplexModeFull : bmdDuplexModeHalf);
#endif
            if (res != S_OK)
                av_log(avctx, AV_LOG_WARNING, "Setting duplex mode failed.\n");
            else
                av_log(avctx, AV_LOG_VERBOSE, "Successfully set duplex mode to %s duplex.\n", ctx->duplex_mode == 2 || ctx->duplex_mode == 4 ? "full" : "half");
        } else {
            av_log(avctx, AV_LOG_WARNING, "Unable to set duplex mode, because it is not supported.\n");
        }
    }
    if (direction == DIRECTION_IN) {
        int ret;
        ret = decklink_select_input(avctx, bmdDeckLinkConfigAudioInputConnection);
        if (ret < 0)
            return ret;
        ret = decklink_select_input(avctx, bmdDeckLinkConfigVideoInputConnection);
        if (ret < 0)
            return ret;
    }
    if (direction == DIRECTION_OUT && cctx->timing_offset != INT_MIN) {
        res = ctx->cfg->SetInt(bmdDeckLinkConfigReferenceInputTimingOffset, cctx->timing_offset);
        if (res != S_OK)
            av_log(avctx, AV_LOG_WARNING, "Setting timing offset failed.\n");
    }

    if (direction == DIRECTION_OUT && ctx->link > 0) {
        res = ctx->cfg->SetInt(bmdDeckLinkConfigSDIOutputLinkConfiguration, ctx->link);
        if (res != S_OK)
            av_log(avctx, AV_LOG_WARNING, "Setting link configuration failed.\n");
        else
            av_log(avctx, AV_LOG_VERBOSE, "Successfully set link configuration: 0x%x.\n", ctx->link);
        if (ctx->link == bmdLinkConfigurationQuadLink && cctx->sqd >= 0) {
            res = ctx->cfg->SetFlag(bmdDeckLinkConfigQuadLinkSDIVideoOutputSquareDivisionSplit, cctx->sqd);
            if (res != S_OK)
                av_log(avctx, AV_LOG_WARNING, "Setting SquareDivisionSplit failed.\n");
            else
                av_log(avctx, AV_LOG_VERBOSE, "Successfully set SquareDivisionSplit.\n");
        }
    }

    if (direction == DIRECTION_OUT && cctx->level_a >= 0) {
        DECKLINK_BOOL level_a_supported = false;

        if (ctx->attr->GetFlag(BMDDeckLinkSupportsSMPTELevelAOutput, &level_a_supported) != S_OK)
            level_a_supported = false;

        if (level_a_supported) {
            res = ctx->cfg->SetFlag(bmdDeckLinkConfigSMPTELevelAOutput, cctx->level_a);
            if (res != S_OK)
                av_log(avctx, AV_LOG_WARNING, "Setting SMPTE levelA failed.\n");
            else
                av_log(avctx, AV_LOG_VERBOSE, "Successfully set SMPTE levelA.\n");
        } else {
            av_log(avctx, AV_LOG_WARNING, "Unable to set SMPTE levelA mode, because it is not supported.\n");
        }
    }

    return 0;
}

int ff_decklink_set_format(AVFormatContext *avctx,
                               int width, int height,
                               int tb_num, int tb_den,
                               enum AVFieldOrder field_order,
                               decklink_direction_t direction)
{
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;
#if BLACKMAGIC_DECKLINK_API_VERSION >= 0x0b000000
    DECKLINK_BOOL support;
#else
    BMDDisplayModeSupport support;
#endif
    IDeckLinkDisplayModeIterator *itermode;
    IDeckLinkDisplayMode *mode;
    int i = 1;
    HRESULT res;

    av_log(avctx, AV_LOG_DEBUG, "Trying to find mode for frame size %dx%d, frame timing %d/%d, field order %d, direction %d, format code %s\n",
        width, height, tb_num, tb_den, field_order, direction, cctx->format_code ? cctx->format_code : "(unset)");

    if (direction == DIRECTION_IN) {
        res = ctx->dli->GetDisplayModeIterator (&itermode);
    } else {
        res = ctx->dlo->GetDisplayModeIterator (&itermode);
    }

    if (res!= S_OK) {
            av_log(avctx, AV_LOG_ERROR, "Could not get Display Mode Iterator\n");
            return AVERROR(EIO);
    }

    char format_buf[] = "    ";
    if (cctx->format_code)
        memcpy(format_buf, cctx->format_code, FFMIN(strlen(cctx->format_code), sizeof(format_buf)));
    BMDDisplayMode target_mode = (BMDDisplayMode)AV_RB32(format_buf);
    AVRational target_tb = av_make_q(tb_num, tb_den);
    ctx->bmd_mode = bmdModeUnknown;
    while ((ctx->bmd_mode == bmdModeUnknown) && itermode->Next(&mode) == S_OK) {
        BMDTimeValue bmd_tb_num, bmd_tb_den;
        int bmd_width  = mode->GetWidth();
        int bmd_height = mode->GetHeight();
        BMDDisplayMode bmd_mode = mode->GetDisplayMode();
        BMDFieldDominance bmd_field_dominance = mode->GetFieldDominance();

        mode->GetFrameRate(&bmd_tb_num, &bmd_tb_den);
        AVRational mode_tb = av_make_q(bmd_tb_num, bmd_tb_den);

        if ((bmd_width == width &&
             bmd_height == height &&
             !av_cmp_q(mode_tb, target_tb) &&
             field_order_eq(field_order, bmd_field_dominance))
             || target_mode == bmd_mode) {
            ctx->bmd_mode   = bmd_mode;
            ctx->bmd_width  = bmd_width;
            ctx->bmd_height = bmd_height;
            ctx->bmd_tb_den = bmd_tb_den;
            ctx->bmd_tb_num = bmd_tb_num;
            ctx->bmd_field_dominance = bmd_field_dominance;
            av_log(avctx, AV_LOG_INFO, "Found Decklink mode %d x %d with rate %.2f%s\n",
                bmd_width, bmd_height, 1/av_q2d(mode_tb),
                (ctx->bmd_field_dominance==bmdLowerFieldFirst || ctx->bmd_field_dominance==bmdUpperFieldFirst)?"(i)":"");
        }

        mode->Release();
        i++;
    }

    itermode->Release();

    if (ctx->bmd_mode == bmdModeUnknown)
        return -1;

#if BLACKMAGIC_DECKLINK_API_VERSION >= 0x0b050000
    if (direction == DIRECTION_IN) {
        BMDDisplayMode actualMode = ctx->bmd_mode;
        if (ctx->dli->DoesSupportVideoMode(ctx->video_input, ctx->bmd_mode, ctx->raw_format,
                                           bmdNoVideoInputConversion, bmdSupportedVideoModeDefault,
                                           &actualMode, &support) != S_OK || !support || ctx->bmd_mode != actualMode)
            return -1;
    } else {
        BMDDisplayMode actualMode = ctx->bmd_mode;
        if (ctx->dlo->DoesSupportVideoMode(bmdVideoConnectionUnspecified, ctx->bmd_mode, ctx->raw_format,
                                           bmdNoVideoOutputConversion, bmdSupportedVideoModeDefault,
                                           &actualMode, &support) != S_OK || !support || ctx->bmd_mode != actualMode)
            return -1;
    }
    return 0;
#elif BLACKMAGIC_DECKLINK_API_VERSION >= 0x0b000000
    if (direction == DIRECTION_IN) {
        if (ctx->dli->DoesSupportVideoMode(ctx->video_input, ctx->bmd_mode, ctx->raw_format,
                                           bmdSupportedVideoModeDefault,
                                           &support) != S_OK)
            return -1;
    } else {
        BMDDisplayMode actualMode = ctx->bmd_mode;
        if (ctx->dlo->DoesSupportVideoMode(bmdVideoConnectionUnspecified, ctx->bmd_mode, ctx->raw_format,
                                           bmdSupportedVideoModeDefault,
                                           &actualMode, &support) != S_OK || !support || ctx->bmd_mode != actualMode) {
            return -1;
        }

    }
    if (support)
        return 0;
#else
    if (direction == DIRECTION_IN) {
        if (ctx->dli->DoesSupportVideoMode(ctx->bmd_mode, ctx->raw_format,
                                           bmdVideoOutputFlagDefault,
                                           &support, NULL) != S_OK)
            return -1;
    } else {
        if (!ctx->supports_vanc || ctx->dlo->DoesSupportVideoMode(ctx->bmd_mode, ctx->raw_format,
                                                                  bmdVideoOutputVANC,
                                                                  &support, NULL) != S_OK || support != bmdDisplayModeSupported) {
            /* Try without VANC enabled */
            if (ctx->dlo->DoesSupportVideoMode(ctx->bmd_mode, ctx->raw_format,
                                               bmdVideoOutputFlagDefault,
                                               &support, NULL) != S_OK) {
                return -1;
            }
            ctx->supports_vanc = 0;
        }

    }
    if (support == bmdDisplayModeSupported)
        return 0;
#endif

    return -1;
}

int ff_decklink_set_format(AVFormatContext *avctx, decklink_direction_t direction) {
    return ff_decklink_set_format(avctx, 0, 0, 0, 0, AV_FIELD_UNKNOWN, direction);
}

void ff_decklink_packet_queue_init(AVFormatContext *avctx, DecklinkPacketQueue *q, int64_t queue_size)
{
    memset(q, 0, sizeof(DecklinkPacketQueue));
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
    q->avctx = avctx;
    q->max_q_size = queue_size;
}

void ff_decklink_packet_queue_flush(DecklinkPacketQueue *q)
{
    AVPacket pkt;

    pthread_mutex_lock(&q->mutex);
    while (avpriv_packet_list_get(&q->pkt_list, &pkt) == 0) {
        av_packet_unref(&pkt);
    }
    q->nb_packets = 0;
    q->size       = 0;
    pthread_mutex_unlock(&q->mutex);
}

void ff_decklink_packet_queue_end(DecklinkPacketQueue *q)
{
    ff_decklink_packet_queue_flush(q);
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

unsigned long long ff_decklink_packet_queue_size(DecklinkPacketQueue *q)
{
    unsigned long long size;
    pthread_mutex_lock(&q->mutex);
    size = q->size;
    pthread_mutex_unlock(&q->mutex);
    return size;
}

int ff_decklink_packet_queue_put(DecklinkPacketQueue *q, AVPacket *pkt)
{
    int pkt_size = pkt->size;
    int ret;

    // Drop Packet if queue size is > maximum queue size
    if (ff_decklink_packet_queue_size(q) > (uint64_t)q->max_q_size) {
        av_packet_unref(pkt);
        av_log(q->avctx, AV_LOG_WARNING,  "Decklink input buffer overrun!\n");
        return -1;
    }
    /* ensure the packet is reference counted */
    if (av_packet_make_refcounted(pkt) < 0) {
        av_packet_unref(pkt);
        return -1;
    }

    pthread_mutex_lock(&q->mutex);

    ret = avpriv_packet_list_put(&q->pkt_list, pkt, NULL, 0);
    if (ret == 0) {
        q->nb_packets++;
        q->size += pkt_size + sizeof(AVPacket);
        pthread_cond_signal(&q->cond);
    } else {
        av_packet_unref(pkt);
    }

    pthread_mutex_unlock(&q->mutex);
    return ret;
}

int ff_decklink_packet_queue_get(DecklinkPacketQueue *q, AVPacket *pkt, int block)
{
    int ret;

    pthread_mutex_lock(&q->mutex);

    for (;; ) {
        ret = avpriv_packet_list_get(&q->pkt_list, pkt);
        if (ret == 0) {
            q->nb_packets--;
            q->size -= pkt->size + sizeof(AVPacket);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            pthread_cond_wait(&q->cond, &q->mutex);
        }
    }
    pthread_mutex_unlock(&q->mutex);
    return ret;
}

int64_t ff_decklink_packet_queue_peekpts(DecklinkPacketQueue *q)
{
    PacketListEntry *pkt1;
    int64_t pts = -1;

    pthread_mutex_lock(&q->mutex);
    pkt1 = q->pkt_list.head;
    if (pkt1) {
        pts = pkt1->pkt.pts;
    }
    pthread_mutex_unlock(&q->mutex);

    return pts;
}


int ff_decklink_list_devices(AVFormatContext *avctx,
                             struct AVDeviceInfoList *device_list,
                             int show_inputs, int show_outputs)
{
    IDeckLink *dl = NULL;
    IDeckLinkIterator *iter = decklink_create_iterator(avctx);
    int ret = 0;

    if (!iter)
        return AVERROR(EIO);

    while (ret == 0 && iter->Next(&dl) == S_OK) {
        IDeckLinkOutput *output_config;
        IDeckLinkInput *input_config;
        const char *display_name = NULL;
        const char *unique_name = NULL;
        AVDeviceInfo *new_device = NULL;
        int add = 0;

        ret = decklink_get_attr_string(dl, BMDDeckLinkDisplayName, &display_name);
        if (ret < 0)
            goto next;
        ret = decklink_get_attr_string(dl, BMDDeckLinkDeviceHandle, &unique_name);
        if (ret < 0)
            goto next;

        if (show_outputs) {
            if (dl->QueryInterface(IID_IDeckLinkOutput, (void **)&output_config) == S_OK) {
                output_config->Release();
                add = 1;
            }
        }

        if (show_inputs) {
            if (dl->QueryInterface(IID_IDeckLinkInput, (void **)&input_config) == S_OK) {
                input_config->Release();
                add = 1;
            }
        }

        if (add == 1) {
            new_device = (AVDeviceInfo *) av_mallocz(sizeof(AVDeviceInfo));
            if (!new_device) {
                ret = AVERROR(ENOMEM);
                goto next;
            }

            new_device->device_name = av_strdup(unique_name ? unique_name : display_name);
            new_device->device_description = av_strdup(display_name);

            if (!new_device->device_name ||
                !new_device->device_description ||
                av_dynarray_add_nofree(&device_list->devices, &device_list->nb_devices, new_device) < 0) {
                ret = AVERROR(ENOMEM);
                av_freep(&new_device->device_name);
                av_freep(&new_device->device_description);
                av_freep(&new_device);
                goto next;
            }
        }

    next:
        av_freep(&display_name);
        av_freep(&unique_name);
        dl->Release();
    }
    iter->Release();
    return ret;
}

/* This is a wrapper around the ff_decklink_list_devices() which dumps the
   output to av_log() and exits (for backward compatibility with the
   "-list_devices" argument). */
void ff_decklink_list_devices_legacy(AVFormatContext *avctx,
                                     int show_inputs, int show_outputs)
{
    struct AVDeviceInfoList *device_list = NULL;
    int ret;

    device_list = (struct AVDeviceInfoList *) av_mallocz(sizeof(AVDeviceInfoList));
    if (!device_list)
        return;

    ret = ff_decklink_list_devices(avctx, device_list, show_inputs, show_outputs);
    if (ret == 0) {
        av_log(avctx, AV_LOG_INFO, "Blackmagic DeckLink %s devices:\n",
               show_inputs ? "input" : "output");
        for (int i = 0; i < device_list->nb_devices; i++) {
            av_log(avctx, AV_LOG_INFO, "\t'%s'\n", device_list->devices[i]->device_description);
        }
    }
    avdevice_free_list_devices(&device_list);
}

int ff_decklink_list_formats(AVFormatContext *avctx, decklink_direction_t direction)
{
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;
    IDeckLinkDisplayModeIterator *itermode;
    IDeckLinkDisplayMode *mode;
    uint32_t format_code;
    HRESULT res;

    if (direction == DIRECTION_IN) {
        int ret;
        ret = decklink_select_input(avctx, bmdDeckLinkConfigAudioInputConnection);
        if (ret < 0)
            return ret;
        ret = decklink_select_input(avctx, bmdDeckLinkConfigVideoInputConnection);
        if (ret < 0)
            return ret;
        res = ctx->dli->GetDisplayModeIterator (&itermode);
    } else {
        res = ctx->dlo->GetDisplayModeIterator (&itermode);
    }

    if (res!= S_OK) {
            av_log(avctx, AV_LOG_ERROR, "Could not get Display Mode Iterator\n");
            return AVERROR(EIO);
    }

    av_log(avctx, AV_LOG_INFO, "Supported formats for '%s':\n\tformat_code\tdescription",
               avctx->url);
    while (itermode->Next(&mode) == S_OK) {
        BMDTimeValue tb_num, tb_den;
        mode->GetFrameRate(&tb_num, &tb_den);
        format_code = av_bswap32(mode->GetDisplayMode());
        av_log(avctx, AV_LOG_INFO, "\n\t%.4s\t\t%ldx%ld at %d/%d fps",
                (char*) &format_code, mode->GetWidth(), mode->GetHeight(),
                (int) tb_den, (int) tb_num);
        switch (mode->GetFieldDominance()) {
        case bmdLowerFieldFirst:
        av_log(avctx, AV_LOG_INFO, " (interlaced, lower field first)"); break;
        case bmdUpperFieldFirst:
        av_log(avctx, AV_LOG_INFO, " (interlaced, upper field first)"); break;
        }
        mode->Release();
    }
    av_log(avctx, AV_LOG_INFO, "\n");

    itermode->Release();

    return 0;
}

void ff_decklink_cleanup(AVFormatContext *avctx)
{
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;

    if (ctx->dli)
        ctx->dli->Release();
    if (ctx->dlo)
        ctx->dlo->Release();
    if (ctx->attr)
        ctx->attr->Release();
    if (ctx->cfg)
        ctx->cfg->Release();
    if (ctx->dl)
        ctx->dl->Release();
}

int ff_decklink_init_device(AVFormatContext *avctx, const char* name)
{
    struct decklink_cctx *cctx = (struct decklink_cctx *)avctx->priv_data;
    struct decklink_ctx *ctx = (struct decklink_ctx *)cctx->ctx;
    IDeckLink *dl = NULL;
    IDeckLinkIterator *iter = decklink_create_iterator(avctx);
    if (!iter)
        return AVERROR_EXTERNAL;

    while (iter->Next(&dl) == S_OK) {
        const char *display_name = NULL;
        const char *unique_name = NULL;
        decklink_get_attr_string(dl, BMDDeckLinkDisplayName, &display_name);
        decklink_get_attr_string(dl, BMDDeckLinkDeviceHandle, &unique_name);
        if (display_name && !strcmp(name, display_name) || unique_name && !strcmp(name, unique_name)) {
            av_free((void *)unique_name);
            av_free((void *)display_name);
            ctx->dl = dl;
            break;
        }
        av_free((void *)display_name);
        av_free((void *)unique_name);
        dl->Release();
    }
    iter->Release();
    if (!ctx->dl)
        return AVERROR(ENXIO);

    if (ctx->dl->QueryInterface(IID_IDeckLinkConfiguration, (void **)&ctx->cfg) != S_OK) {
        av_log(avctx, AV_LOG_ERROR, "Could not get configuration interface for '%s'\n", name);
        ff_decklink_cleanup(avctx);
        return AVERROR_EXTERNAL;
    }

    if (ctx->dl->QueryInterface(IID_IDeckLinkProfileAttributes, (void **)&ctx->attr) != S_OK) {
        av_log(avctx, AV_LOG_ERROR, "Could not get attributes interface for '%s'\n", name);
        ff_decklink_cleanup(avctx);
        return AVERROR_EXTERNAL;
    }

    return 0;
}
