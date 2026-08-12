/*
 * ALSA input and output
 * Copyright (c) 2007 Luca Abeni ( lucabe72 email it )
 * Copyright (c) 2007 Benoit Fouet ( benoit fouet free fr )
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

/**
 * @file
 * ALSA input and output: common code
 * @author Luca Abeni ( lucabe72 email it )
 * @author Benoit Fouet ( benoit fouet free fr )
 * @author Nicolas George ( nicolas george normalesup org )
 */

#include "config_components.h"

#include <alsa/asoundlib.h>
#include "avdevice.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"

#include "alsa.h"

static av_cold snd_pcm_format_t codec_id_to_pcm_format(int codec_id)
{
    switch(codec_id) {
        case AV_CODEC_ID_PCM_F64LE: return SND_PCM_FORMAT_FLOAT64_LE;
        case AV_CODEC_ID_PCM_F64BE: return SND_PCM_FORMAT_FLOAT64_BE;
        case AV_CODEC_ID_PCM_F32LE: return SND_PCM_FORMAT_FLOAT_LE;
        case AV_CODEC_ID_PCM_F32BE: return SND_PCM_FORMAT_FLOAT_BE;
        case AV_CODEC_ID_PCM_S32LE: return SND_PCM_FORMAT_S32_LE;
        case AV_CODEC_ID_PCM_S32BE: return SND_PCM_FORMAT_S32_BE;
        case AV_CODEC_ID_PCM_U32LE: return SND_PCM_FORMAT_U32_LE;
        case AV_CODEC_ID_PCM_U32BE: return SND_PCM_FORMAT_U32_BE;
        case AV_CODEC_ID_PCM_S24LE: return SND_PCM_FORMAT_S24_3LE;
        case AV_CODEC_ID_PCM_S24BE: return SND_PCM_FORMAT_S24_3BE;
        case AV_CODEC_ID_PCM_U24LE: return SND_PCM_FORMAT_U24_3LE;
        case AV_CODEC_ID_PCM_U24BE: return SND_PCM_FORMAT_U24_3BE;
        case AV_CODEC_ID_PCM_S16LE: return SND_PCM_FORMAT_S16_LE;
        case AV_CODEC_ID_PCM_S16BE: return SND_PCM_FORMAT_S16_BE;
        case AV_CODEC_ID_PCM_U16LE: return SND_PCM_FORMAT_U16_LE;
        case AV_CODEC_ID_PCM_U16BE: return SND_PCM_FORMAT_U16_BE;
        case AV_CODEC_ID_PCM_S8:    return SND_PCM_FORMAT_S8;
        case AV_CODEC_ID_PCM_U8:    return SND_PCM_FORMAT_U8;
        case AV_CODEC_ID_PCM_MULAW: return SND_PCM_FORMAT_MU_LAW;
        case AV_CODEC_ID_PCM_ALAW:  return SND_PCM_FORMAT_A_LAW;
        default:                 return SND_PCM_FORMAT_UNKNOWN;
    }
}

#define MAKE_REORDER_FUNC(NAME, TYPE, CHANNELS, LAYOUT, MAP)                \
static void alsa_reorder_ ## NAME ## _ ## LAYOUT(const void *in_v,          \
                                                 void *out_v,               \
                                                 int n)                     \
{                                                                           \
    const TYPE *in = in_v;                                                  \
    TYPE      *out = out_v;                                                 \
                                                                            \
    while (n-- > 0) {                                                       \
        MAP                                                                 \
        in  += CHANNELS;                                                    \
        out += CHANNELS;                                                    \
    }                                                                       \
}

#define MAKE_REORDER_FUNCS(CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int8,  int8_t,  CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int16, int16_t, CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int32, int32_t, CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(f32,   float,   CHANNELS, LAYOUT, MAP)

MAKE_REORDER_FUNCS(5, out_50, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[3]; \
        out[3] = in[4]; \
        out[4] = in[2]; \
        )

MAKE_REORDER_FUNCS(6, out_51, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[4]; \
        out[3] = in[5]; \
        out[4] = in[2]; \
        out[5] = in[3]; \
        )

MAKE_REORDER_FUNCS(8, out_71, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[4]; \
        out[3] = in[5]; \
        out[4] = in[2]; \
        out[5] = in[3]; \
        out[6] = in[6]; \
        out[7] = in[7]; \
        )

#define FORMAT_I8  0
#define FORMAT_I16 1
#define FORMAT_I32 2
#define FORMAT_F32 3

#define PICK_REORDER(layout)\
switch(format) {\
    case FORMAT_I8:  s->reorder_func = alsa_reorder_int8_out_ ##layout;  break;\
    case FORMAT_I16: s->reorder_func = alsa_reorder_int16_out_ ##layout; break;\
    case FORMAT_I32: s->reorder_func = alsa_reorder_int32_out_ ##layout; break;\
    case FORMAT_F32: s->reorder_func = alsa_reorder_f32_out_ ##layout;   break;\
}

static const struct {
    unsigned int   pos;
    enum AVChannel ch;
} alsa_chmap_table[] = {
    { SND_CHMAP_MONO,    AV_CHAN_FRONT_CENTER         },
    { SND_CHMAP_FL,      AV_CHAN_FRONT_LEFT           },
    { SND_CHMAP_FR,      AV_CHAN_FRONT_RIGHT          },
    { SND_CHMAP_RL,      AV_CHAN_BACK_LEFT            },
    { SND_CHMAP_RR,      AV_CHAN_BACK_RIGHT           },
    { SND_CHMAP_FC,      AV_CHAN_FRONT_CENTER         },
    { SND_CHMAP_LFE,     AV_CHAN_LOW_FREQUENCY        },
    { SND_CHMAP_SL,      AV_CHAN_SIDE_LEFT            },
    { SND_CHMAP_SR,      AV_CHAN_SIDE_RIGHT           },
    { SND_CHMAP_RC,      AV_CHAN_BACK_CENTER          },
    { SND_CHMAP_FLC,     AV_CHAN_FRONT_LEFT_OF_CENTER },
    { SND_CHMAP_FRC,     AV_CHAN_FRONT_RIGHT_OF_CENTER},
    { SND_CHMAP_FLW,     AV_CHAN_WIDE_LEFT            },
    { SND_CHMAP_FRW,     AV_CHAN_WIDE_RIGHT           },
    { SND_CHMAP_TC,      AV_CHAN_TOP_CENTER           },
    { SND_CHMAP_TFL,     AV_CHAN_TOP_FRONT_LEFT       },
    { SND_CHMAP_TFR,     AV_CHAN_TOP_FRONT_RIGHT      },
    { SND_CHMAP_TFC,     AV_CHAN_TOP_FRONT_CENTER     },
    { SND_CHMAP_TRL,     AV_CHAN_TOP_BACK_LEFT        },
    { SND_CHMAP_TRR,     AV_CHAN_TOP_BACK_RIGHT       },
    { SND_CHMAP_TRC,     AV_CHAN_TOP_BACK_CENTER      },
    { SND_CHMAP_TSL,     AV_CHAN_TOP_SIDE_LEFT        },
    { SND_CHMAP_TSR,     AV_CHAN_TOP_SIDE_RIGHT       },
    { SND_CHMAP_LLFE,    AV_CHAN_LOW_FREQUENCY        },
    { SND_CHMAP_RLFE,    AV_CHAN_LOW_FREQUENCY_2      },
    { SND_CHMAP_BC,      AV_CHAN_BOTTOM_FRONT_CENTER  },
    { SND_CHMAP_BLC,     AV_CHAN_BOTTOM_FRONT_LEFT    },
    { SND_CHMAP_BRC,     AV_CHAN_BOTTOM_FRONT_RIGHT   },
    { SND_CHMAP_UNKNOWN, AV_CHAN_UNKNOWN              },
    { SND_CHMAP_NA,      AV_CHAN_UNUSED               },
};

static enum AVChannel alsa_chmap_pos_to_av_chan(unsigned int pos)
{
    for (unsigned i = 0; i < FF_ARRAY_ELEMS(alsa_chmap_table); i++)
        if (alsa_chmap_table[i].pos == pos)
            return alsa_chmap_table[i].ch;
    return AV_CHAN_NONE;
}

static int alsa_get_chmap(AVFormatContext *ctx, snd_pcm_t *h, AVChannelLayout *layout)
{
    snd_pcm_chmap_t *chmap = snd_pcm_get_chmap(h);
    int ret = 0;

    if (!chmap)
        return 0;

    if (layout->nb_channels != chmap->channels) {
        av_log(ctx, AV_LOG_ERROR, "Inconsistent channel layout requested!\n");
        ret = AVERROR(EINVAL);
        goto out;
    }

    av_channel_layout_uninit(layout);
    ret = av_channel_layout_custom_init(layout, chmap->channels);
    if (ret < 0)
        goto out;

    for (unsigned i = 0; i < chmap->channels; i++) {
        enum AVChannel ch = alsa_chmap_pos_to_av_chan(chmap->pos[i] &
                                                      SND_CHMAP_POSITION_MASK);
        if (ch == AV_CHAN_NONE) {
            av_log(ctx, AV_LOG_WARNING,
                   "unknown ALSA channel position %u.\n",
                   chmap->pos[i] & SND_CHMAP_POSITION_MASK);
            continue;
        }
        layout->u.map[i].id = ch;
    }
    ret = av_channel_layout_retype(layout, 0,
                                   AV_CHANNEL_LAYOUT_RETYPE_FLAG_CANONICAL);

out:
    free(chmap);
    return ret;
}

static av_cold int find_reorder_func(AlsaData *s, int codec_id,
                                     const AVChannelLayout *layout, int out)
{
    int format;

    /* reordering input is not currently supported */
    if (!out)
        return AVERROR(ENOSYS);

    /* reordering is not needed for QUAD or 2_2 layout */
    if (!av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_QUAD) ||
        !av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_2_2))
        return 0;

    switch (codec_id) {
    case AV_CODEC_ID_PCM_S8:
    case AV_CODEC_ID_PCM_U8:
    case AV_CODEC_ID_PCM_ALAW:
    case AV_CODEC_ID_PCM_MULAW: format = FORMAT_I8;  break;
    case AV_CODEC_ID_PCM_S16LE:
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_U16LE:
    case AV_CODEC_ID_PCM_U16BE: format = FORMAT_I16; break;
    case AV_CODEC_ID_PCM_S32LE:
    case AV_CODEC_ID_PCM_S32BE:
    case AV_CODEC_ID_PCM_U32LE:
    case AV_CODEC_ID_PCM_U32BE: format = FORMAT_I32; break;
    case AV_CODEC_ID_PCM_F32LE:
    case AV_CODEC_ID_PCM_F32BE: format = FORMAT_F32; break;
    default:                 return AVERROR(ENOSYS);
    }

    if (!av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0_BACK) ||
        !av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0))
        PICK_REORDER(50)
    else if (!av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1_BACK) ||
             !av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1))
        PICK_REORDER(51)
    else if (!av_channel_layout_compare(layout, &(AVChannelLayout)AV_CHANNEL_LAYOUT_7POINT1))
        PICK_REORDER(71)

    return s->reorder_func ? 0 : AVERROR(ENOSYS);
}

av_cold int ff_alsa_open(AVFormatContext *ctx, snd_pcm_stream_t mode,
                         unsigned int *sample_rate,
                         AVChannelLayout *layout, enum AVCodecID *codec_id)
{
    AlsaData *s = ctx->priv_data;
    const char *audio_device;
    int res, flags = 0;
    snd_pcm_format_t format;
    snd_pcm_t *h;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_uframes_t buffer_size, period_size;

    if (ctx->url[0] == 0) audio_device = "default";
    else                  audio_device = ctx->url;

    if (*codec_id == AV_CODEC_ID_NONE)
        *codec_id = DEFAULT_CODEC_ID;
    format = codec_id_to_pcm_format(*codec_id);
    if (format == SND_PCM_FORMAT_UNKNOWN) {
        av_log(ctx, AV_LOG_ERROR, "sample format 0x%04x is not supported\n", *codec_id);
        return AVERROR(ENOSYS);
    }
    if (ctx->flags & AVFMT_FLAG_NONBLOCK) {
        flags = SND_PCM_NONBLOCK;
    }
    res = snd_pcm_open(&h, audio_device, mode, flags);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot open audio device %s (%s)\n",
               audio_device, snd_strerror(res));
        return AVERROR(EIO);
    }

    res = snd_pcm_hw_params_malloc(&hw_params);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate hardware parameter structure (%s)\n",
               snd_strerror(res));
        goto fail1;
    }

    res = snd_pcm_hw_params_any(h, hw_params);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot initialize hardware parameter structure (%s)\n",
               snd_strerror(res));
        goto fail;
    }

    res = snd_pcm_hw_params_set_access(h, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set access type (%s)\n",
               snd_strerror(res));
        goto fail;
    }

    res = snd_pcm_hw_params_set_format(h, hw_params, format);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set sample format 0x%04x %d (%s)\n",
               *codec_id, format, snd_strerror(res));
        goto fail;
    }

    res = snd_pcm_hw_params_set_rate_near(h, hw_params, sample_rate, 0);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set sample rate (%s)\n",
               snd_strerror(res));
        goto fail;
    }

    // For output streams, the stream should already have a layout.
    // Adding this check for clarity.
    if (mode == SND_PCM_STREAM_CAPTURE && layout->nb_channels == 0) {
        unsigned int channels = 2;
        if (snd_pcm_hw_params_test_channels(h, hw_params, channels) < 0) {
            res = snd_pcm_hw_params_get_channels_min(hw_params, &channels);
            if (res < 0) {
                av_log(ctx, AV_LOG_ERROR, "cannot get minimum channel count (%s)\n",
                       snd_strerror(res));
                goto fail;
            }
            av_log(ctx, AV_LOG_VERBOSE,
                   "stereo not supported, falling back to %u channel(s)\n", channels);
        }
        layout->order       = AV_CHANNEL_ORDER_UNSPEC;
        layout->nb_channels = channels;
    }

    res = snd_pcm_hw_params_set_channels(h, hw_params, layout->nb_channels);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set channel count to %d (%s)\n",
               layout->nb_channels, snd_strerror(res));
        goto fail;
    }

    s->frame_size = av_get_bits_per_sample(*codec_id) / 8 * layout->nb_channels;

    snd_pcm_hw_params_get_buffer_size_max(hw_params, &buffer_size);
    buffer_size = FFMIN(buffer_size, ALSA_BUFFER_SIZE_MAX);
    /* TODO: maybe use ctx->max_picture_buffer somehow */
    res = snd_pcm_hw_params_set_buffer_size_near(h, hw_params, &buffer_size);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set ALSA buffer size (%s)\n",
               snd_strerror(res));
        goto fail;
    }

    snd_pcm_hw_params_get_period_size_min(hw_params, &period_size, NULL);
    if (!period_size)
        period_size = buffer_size / 4;
    res = snd_pcm_hw_params_set_period_size_near(h, hw_params, &period_size, NULL);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set ALSA period size (%s)\n",
               snd_strerror(res));
        goto fail;
    }
    s->period_size = period_size;

    res = snd_pcm_hw_params(h, hw_params);
    if (res < 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot set parameters (%s)\n",
               snd_strerror(res));
        goto fail;
    }

    snd_pcm_hw_params_free(hw_params);

    // TODO: when support for channel layout gets more widespread in alsa
    // drivers, this might be used to supersede the re-ordering function
    // below.
    if (mode == SND_PCM_STREAM_CAPTURE &&
        layout->order == AV_CHANNEL_ORDER_UNSPEC) {
        res = alsa_get_chmap(ctx, h, layout);
        if (res < 0)
            goto fail1;
    }

    if (layout->nb_channels > 2 && layout->order != AV_CHANNEL_ORDER_UNSPEC) {
        // See comment above. find_reorder_func is currently only implemented in
        // playback mode.
        if (find_reorder_func(s, *codec_id, layout, mode == SND_PCM_STREAM_PLAYBACK) < 0) {
            char name[128];
            av_channel_layout_describe(layout, name, sizeof(name));
            av_log(ctx, AV_LOG_WARNING, "ALSA channel layout unknown or unimplemented for %s %s.\n",
                   name, mode == SND_PCM_STREAM_PLAYBACK ? "playback" : "capture");
        }
        if (s->reorder_func) {
            s->reorder_buf_size = buffer_size;
            s->reorder_buf = av_malloc_array(s->reorder_buf_size, s->frame_size);
            if (!s->reorder_buf)
                goto fail1;
        }
    }

    s->pkt = av_packet_alloc();
    if (!s->pkt)
        goto fail1;

    s->h = h;
    return 0;

fail:
    snd_pcm_hw_params_free(hw_params);
fail1:
    snd_pcm_close(h);
    return AVERROR(EIO);
}

av_cold int ff_alsa_close(AVFormatContext *s1)
{
    AlsaData *s = s1->priv_data;

    if (snd_pcm_stream(s->h) == SND_PCM_STREAM_PLAYBACK) {
        snd_pcm_nonblock(s->h, 0);
        snd_pcm_drain(s->h);
    }
    av_freep(&s->reorder_buf);
    if (CONFIG_ALSA_INDEV)
        ff_timefilter_destroy(s->timefilter);
    snd_pcm_close(s->h);
    av_packet_free(&s->pkt);
    return 0;
}

int ff_alsa_xrun_recover(AVFormatContext *s1, int err)
{
    AlsaData *s = s1->priv_data;
    snd_pcm_t *handle = s->h;

    av_log(s1, AV_LOG_WARNING, "ALSA buffer xrun.\n");
    if (err == -EPIPE) {
        err = snd_pcm_prepare(handle);
        if (err < 0) {
            av_log(s1, AV_LOG_ERROR, "cannot recover from underrun (snd_pcm_prepare failed: %s)\n", snd_strerror(err));

            return AVERROR(EIO);
        }
#ifdef ESTRPIPE
    } else if (err == -ESTRPIPE) {
        av_log(s1, AV_LOG_ERROR, "-ESTRPIPE... Unsupported!\n");

        return -1;
#endif
    }
    return err;
}

int ff_alsa_extend_reorder_buf(AlsaData *s, int min_size)
{
    int size = s->reorder_buf_size;
    void *r;

    av_assert0(size != 0);
    while (size < min_size)
        size *= 2;
    r = av_realloc_array(s->reorder_buf, size, s->frame_size);
    if (!r)
        return AVERROR(ENOMEM);
    s->reorder_buf = r;
    s->reorder_buf_size = size;
    return 0;
}

/* ported from alsa-utils/aplay.c */
int ff_alsa_get_device_list(AVDeviceInfoList *device_list, snd_pcm_stream_t stream_type)
{
    int ret = 0;
    void **hints, **n;
    char *name = NULL, *descr = NULL, *io = NULL, *tmp;
    AVDeviceInfo *new_device = NULL;
    const char *filter = stream_type == SND_PCM_STREAM_PLAYBACK ? "Output" : "Input";

    if (snd_device_name_hint(-1, "pcm", &hints) < 0)
        return AVERROR_EXTERNAL;
    n = hints;
    while (*n && !ret) {
        name = snd_device_name_get_hint(*n, "NAME");
        descr = snd_device_name_get_hint(*n, "DESC");
        io = snd_device_name_get_hint(*n, "IOID");
        if (!io || !strcmp(io, filter)) {
            new_device = av_mallocz(sizeof(AVDeviceInfo));
            if (!new_device) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            new_device->device_name = av_strdup(name ? name : "");
            if (!descr)
                new_device->device_description = av_strdup("");
            else if ((tmp = strrchr(descr, '\n')) && tmp[1])
                new_device->device_description = av_strdup(&tmp[1]);
            else
                new_device->device_description = av_strdup(descr);
            if (!new_device->device_description || !new_device->device_name) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            if ((ret = av_dynarray_add_nofree(&device_list->devices,
                                              &device_list->nb_devices, new_device)) < 0) {
                goto fail;
            }
            if (!strcmp(new_device->device_name, "default"))
                device_list->default_device = device_list->nb_devices - 1;
            new_device = NULL;
        }
      fail:
        free(io);
        free(name);
        free(descr);
        n++;
    }
    if (new_device) {
        av_free(new_device->device_description);
        av_free(new_device->device_name);
        av_free(new_device);
    }
    snd_device_name_free_hint(hints);
    return ret;
}
