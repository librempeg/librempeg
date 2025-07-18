/*
 * Audio Toolbox system codecs
 *
 * copyright (c) 2016 rcombs
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

#include <AudioToolbox/AudioToolbox.h>

#define FF_BUFQUEUE_SIZE 256
#include "libavfilter/bufferqueue.h"

#include "config.h"
#include "audio_frame_queue.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"
#include "internal.h"
#include "libavformat/isom.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/log.h"

typedef struct ATDecodeContext {
    AVClass *av_class;
    int mode;
    int quality;

    AudioConverterRef converter;
    struct FFBufQueue frame_queue;
    struct FFBufQueue used_frame_queue;

    unsigned pkt_size;
    AudioFrameQueue afq;
    int eof;
    int frame_size;

    AVFrame* encoding_frame;
} ATDecodeContext;

static UInt32 ffat_get_format_id(enum AVCodecID codec, int profile)
{
    switch (codec) {
    case AV_CODEC_ID_AAC:
        switch (profile) {
        case AV_PROFILE_AAC_LOW:
        default:
            return kAudioFormatMPEG4AAC;
        case AV_PROFILE_AAC_HE:
            return kAudioFormatMPEG4AAC_HE;
        case AV_PROFILE_AAC_HE_V2:
            return kAudioFormatMPEG4AAC_HE_V2;
        case AV_PROFILE_AAC_LD:
            return kAudioFormatMPEG4AAC_LD;
#if MAC_OS_X_VERSION_MIN_REQUIRED >= 1060
        case AV_PROFILE_AAC_ELD:
            return kAudioFormatMPEG4AAC_ELD;
#endif
        }
    case AV_CODEC_ID_ADPCM_IMA_QT:
        return kAudioFormatAppleIMA4;
    case AV_CODEC_ID_ALAC:
        return kAudioFormatAppleLossless;
#if MAC_OS_X_VERSION_MIN_REQUIRED >= 1060
    case AV_CODEC_ID_ILBC:
        return kAudioFormatiLBC;
#endif
    case AV_CODEC_ID_PCM_ALAW:
        return kAudioFormatALaw;
    case AV_CODEC_ID_PCM_MULAW:
        return kAudioFormatULaw;
    default:
        av_assert0(!"Invalid codec ID!");
        return 0;
    }
}

static int ffat_update_ctx(AVCodecContext *avctx)
{
    ATDecodeContext *at = avctx->priv_data;
    UInt32 size = sizeof(unsigned);
    AudioConverterPrimeInfo prime_info;
    AudioStreamBasicDescription out_format;

    AudioConverterGetProperty(at->converter,
                              kAudioConverterPropertyMaximumOutputPacketSize,
                              &size, &at->pkt_size);

    if (at->pkt_size <= 0)
        at->pkt_size = 1024 * 50;

    size = sizeof(prime_info);

    if (!AudioConverterGetProperty(at->converter,
                                   kAudioConverterPrimeInfo,
                                   &size, &prime_info)) {
        avctx->initial_padding = prime_info.leadingFrames;
    }

    size = sizeof(out_format);
    if (!AudioConverterGetProperty(at->converter,
                                   kAudioConverterCurrentOutputStreamDescription,
                                   &size, &out_format)) {
        if (out_format.mFramesPerPacket) {
            avctx->frame_size = out_format.mFramesPerPacket;
        } else {
            /* The doc on mFramesPerPacket says:
             *   For formats with a variable number of frames per packet, such as
             *   Ogg Vorbis, set this field to 0.
             * Looks like it means for decoding. There is no known case that
             * mFramesPerPacket is zero for encoding. Use a default value for safety.
             */
            avctx->frame_size = 1024;
            av_log(avctx, AV_LOG_WARNING, "Missing mFramesPerPacket\n");
        }
        if (out_format.mBytesPerPacket && avctx->codec_id == AV_CODEC_ID_ILBC)
            avctx->block_align = out_format.mBytesPerPacket;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Get OutputStreamDescription failed\n");
        return AVERROR_EXTERNAL;
    }

    at->frame_size = avctx->frame_size;
    if (avctx->codec_id == AV_CODEC_ID_PCM_MULAW ||
        avctx->codec_id == AV_CODEC_ID_PCM_ALAW) {
        at->pkt_size *= 1024;
        avctx->frame_size *= 1024;
    }

    return 0;
}

static int read_descr(GetByteContext *gb, int *tag)
{
    int len = 0;
    int count = 4;
    *tag = bytestream2_get_byte(gb);
    while (count--) {
        int c = bytestream2_get_byte(gb);
        len = (len << 7) | (c & 0x7f);
        if (!(c & 0x80))
            break;
    }
    return len;
}

static int get_ilbc_mode(AVCodecContext *avctx)
{
    if (avctx->block_align == 38)
        return 20;
    else if (avctx->block_align == 50)
        return 30;
    else if (avctx->bit_rate > 0)
        return avctx->bit_rate <= 14000 ? 30 : 20;
    else
        return 30;
}

static av_cold int get_channel_label(int channel)
{
    uint64_t map = 1 << channel;
    if (map <= AV_CH_LOW_FREQUENCY)
        return channel + 1;
    else if (map <= AV_CH_BACK_RIGHT)
        return channel + 29;
    else if (map <= AV_CH_BACK_CENTER)
        return channel - 1;
    else if (map <= AV_CH_SIDE_RIGHT)
        return channel - 4;
    else if (map <= AV_CH_TOP_BACK_RIGHT)
        return channel + 1;
    else if (map <= AV_CH_STEREO_RIGHT)
        return -1;
    else if (map <= AV_CH_WIDE_RIGHT)
        return channel + 4;
    else if (map <= AV_CH_SURROUND_DIRECT_RIGHT)
        return channel - 23;
    else if (map == AV_CH_LOW_FREQUENCY_2)
        return kAudioChannelLabel_LFE2;
    else
        return -1;
}

static int remap_layout(AudioChannelLayout *layout, const AVChannelLayout *in_layout)
{
    int i;
    layout->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
    layout->mNumberChannelDescriptions = in_layout->nb_channels;
    for (i = 0; i < in_layout->nb_channels; i++) {
        int c, label;

        c = av_channel_layout_channel_from_index(in_layout, i);
        if (c < 0 || c >= 64)
            return AVERROR(EINVAL);
        label = get_channel_label(c);
        layout->mChannelDescriptions[i].mChannelLabel = label;
        if (label < 0)
            return AVERROR(EINVAL);
        c++;
    }
    return 0;
}

static int get_aac_tag(const AVChannelLayout *in_layout)
{
    static const struct {
        AVChannelLayout chl;
        int tag;
    } map[] = {
        { AV_CHANNEL_LAYOUT_MONO,              kAudioChannelLayoutTag_Mono },
        { AV_CHANNEL_LAYOUT_STEREO,            kAudioChannelLayoutTag_Stereo },
        { AV_CHANNEL_LAYOUT_QUAD,              kAudioChannelLayoutTag_AAC_Quadraphonic },
        { AV_CHANNEL_LAYOUT_OCTAGONAL,         kAudioChannelLayoutTag_AAC_Octagonal },
        { AV_CHANNEL_LAYOUT_SURROUND,          kAudioChannelLayoutTag_AAC_3_0 },
        { AV_CHANNEL_LAYOUT_4POINT0,           kAudioChannelLayoutTag_AAC_4_0 },
        { AV_CHANNEL_LAYOUT_5POINT0,           kAudioChannelLayoutTag_AAC_5_0 },
        { AV_CHANNEL_LAYOUT_5POINT1,           kAudioChannelLayoutTag_AAC_5_1 },
        { AV_CHANNEL_LAYOUT_6POINT0,           kAudioChannelLayoutTag_AAC_6_0 },
        { AV_CHANNEL_LAYOUT_6POINT1,           kAudioChannelLayoutTag_AAC_6_1 },
        { AV_CHANNEL_LAYOUT_7POINT0,           kAudioChannelLayoutTag_AAC_7_0 },
        { AV_CHANNEL_LAYOUT_7POINT1_WIDE_BACK, kAudioChannelLayoutTag_AAC_7_1 },
        { AV_CHANNEL_LAYOUT_7POINT1,           kAudioChannelLayoutTag_MPEG_7_1_C },
    };
    int i;

    for (i = 0; i < FF_ARRAY_ELEMS(map); i++)
        if (!av_channel_layout_compare(in_layout, &map[i].chl))
            return map[i].tag;

    return 0;
}

static av_cold int ffat_init_encoder(AVCodecContext *avctx)
{
    ATDecodeContext *at = avctx->priv_data;
    OSStatus status;
    int ret;

    AudioStreamBasicDescription in_format = {
        .mSampleRate = avctx->sample_rate,
        .mFormatID = kAudioFormatLinearPCM,
        .mFormatFlags = ((avctx->sample_fmt == AV_SAMPLE_FMT_FLT ||
                          avctx->sample_fmt == AV_SAMPLE_FMT_DBL) ? kAudioFormatFlagIsFloat
                        : avctx->sample_fmt == AV_SAMPLE_FMT_U8 ? 0
                        : kAudioFormatFlagIsSignedInteger)
                        | kAudioFormatFlagIsPacked,
        .mBytesPerPacket = av_get_bytes_per_sample(avctx->sample_fmt) * avctx->ch_layout.nb_channels,
        .mFramesPerPacket = 1,
        .mBytesPerFrame = av_get_bytes_per_sample(avctx->sample_fmt) * avctx->ch_layout.nb_channels,
        .mChannelsPerFrame = avctx->ch_layout.nb_channels,
        .mBitsPerChannel = av_get_bytes_per_sample(avctx->sample_fmt) * 8,
    };
    AudioStreamBasicDescription out_format = {
        .mSampleRate = avctx->sample_rate,
        .mFormatID = ffat_get_format_id(avctx->codec_id, avctx->profile),
        .mChannelsPerFrame = in_format.mChannelsPerFrame,
    };
    UInt32 layout_size = sizeof(AudioChannelLayout) +
                         sizeof(AudioChannelDescription) * avctx->ch_layout.nb_channels;
    AudioChannelLayout *channel_layout = av_malloc(layout_size);

    if (!channel_layout)
        return AVERROR(ENOMEM);

    if (avctx->codec_id == AV_CODEC_ID_ILBC) {
        int mode = get_ilbc_mode(avctx);
        out_format.mFramesPerPacket  = 8000 * mode / 1000;
        out_format.mBytesPerPacket   = (mode == 20 ? 38 : 50);
    }

    status = AudioConverterNew(&in_format, &out_format, &at->converter);

    if (status != 0) {
        av_log(avctx, AV_LOG_ERROR, "AudioToolbox init error: %i\n", (int)status);
        av_free(channel_layout);
        return AVERROR_UNKNOWN;
    }

    if (avctx->ch_layout.order == AV_CHANNEL_ORDER_UNSPEC)
        av_channel_layout_default(&avctx->ch_layout, avctx->ch_layout.nb_channels);

    if ((status = remap_layout(channel_layout, &avctx->ch_layout)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Invalid channel layout\n");
        av_free(channel_layout);
        return status;
    }

    if (AudioConverterSetProperty(at->converter, kAudioConverterInputChannelLayout,
                                  layout_size, channel_layout)) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported input channel layout\n");
        av_free(channel_layout);
        return AVERROR(EINVAL);
    }
    if (avctx->codec_id == AV_CODEC_ID_AAC) {
        int tag = get_aac_tag(&avctx->ch_layout);
        if (tag) {
            channel_layout->mChannelLayoutTag = tag;
            channel_layout->mNumberChannelDescriptions = 0;
        }
    }
    if (AudioConverterSetProperty(at->converter, kAudioConverterOutputChannelLayout,
                                  layout_size, channel_layout)) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported output channel layout\n");
        av_free(channel_layout);
        return AVERROR(EINVAL);
    }
    av_free(channel_layout);

    if (avctx->bits_per_raw_sample)
        AudioConverterSetProperty(at->converter,
                                  kAudioConverterPropertyBitDepthHint,
                                  sizeof(avctx->bits_per_raw_sample),
                                  &avctx->bits_per_raw_sample);

#if !TARGET_OS_IPHONE
    if (at->mode == -1)
        at->mode = (avctx->flags & AV_CODEC_FLAG_QSCALE) ?
                   kAudioCodecBitRateControlMode_Variable :
                   kAudioCodecBitRateControlMode_Constant;

    AudioConverterSetProperty(at->converter, kAudioCodecPropertyBitRateControlMode,
                              sizeof(at->mode), &at->mode);

    if (at->mode == kAudioCodecBitRateControlMode_Variable) {
        int q = avctx->global_quality / FF_QP2LAMBDA;
        if (q < 0 || q > 14) {
            av_log(avctx, AV_LOG_WARNING,
                   "VBR quality %d out of range, should be 0-14\n", q);
            q = av_clip(q, 0, 14);
        }
        q = 127 - q * 9;
        AudioConverterSetProperty(at->converter, kAudioCodecPropertySoundQualityForVBR,
                                  sizeof(q), &q);
    } else
#endif
    if (avctx->bit_rate > 0) {
        UInt32 rate = avctx->bit_rate;
        UInt32 size;
        status = AudioConverterGetPropertyInfo(at->converter,
                                               kAudioConverterApplicableEncodeBitRates,
                                               &size, NULL);
        if (!status && size) {
            UInt32 new_rate = rate;
            int count;
            int i;
            AudioValueRange *ranges = av_malloc(size);
            if (!ranges)
                return AVERROR(ENOMEM);
            AudioConverterGetProperty(at->converter,
                                      kAudioConverterApplicableEncodeBitRates,
                                      &size, ranges);
            count = size / sizeof(AudioValueRange);
            for (i = 0; i < count; i++) {
                AudioValueRange *range = &ranges[i];
                if (rate >= range->mMinimum && rate <= range->mMaximum) {
                    new_rate = rate;
                    break;
                } else if (rate > range->mMaximum) {
                    new_rate = range->mMaximum;
                } else {
                    new_rate = range->mMinimum;
                    break;
                }
            }
            if (new_rate != rate) {
                av_log(avctx, AV_LOG_WARNING,
                       "Bitrate %u not allowed; changing to %u\n", rate, new_rate);
                rate = new_rate;
            }
            av_free(ranges);
        }
        AudioConverterSetProperty(at->converter, kAudioConverterEncodeBitRate,
                                  sizeof(rate), &rate);
    }

    at->quality = 96 - at->quality * 32;
    AudioConverterSetProperty(at->converter, kAudioConverterCodecQuality,
                              sizeof(at->quality), &at->quality);

    if (!AudioConverterGetPropertyInfo(at->converter, kAudioConverterCompressionMagicCookie,
                                       &avctx->extradata_size, NULL) &&
        avctx->extradata_size) {
        int extradata_size = avctx->extradata_size;
        uint8_t *extradata;
        if (!(avctx->extradata = av_mallocz(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE)))
            return AVERROR(ENOMEM);
        if (avctx->codec_id == AV_CODEC_ID_ALAC) {
            avctx->extradata_size = 0x24;
            AV_WB32(avctx->extradata,     0x24);
            AV_WB32(avctx->extradata + 4, MKBETAG('a','l','a','c'));
            extradata = avctx->extradata + 12;
            avctx->extradata_size = 0x24;
        } else {
            extradata = avctx->extradata;
        }
        status = AudioConverterGetProperty(at->converter,
                                           kAudioConverterCompressionMagicCookie,
                                           &extradata_size, extradata);
        if (status != 0) {
            av_log(avctx, AV_LOG_ERROR, "AudioToolbox cookie error: %i\n", (int)status);
            return AVERROR_UNKNOWN;
        } else if (avctx->codec_id == AV_CODEC_ID_AAC) {
            GetByteContext gb;
            int tag, len;
            bytestream2_init(&gb, extradata, extradata_size);
            do {
                len = read_descr(&gb, &tag);
                if (tag == MP4DecConfigDescrTag) {
                    bytestream2_skip(&gb, 13);
                    len = read_descr(&gb, &tag);
                    if (tag == MP4DecSpecificDescrTag) {
                        len = FFMIN(gb.buffer_end - gb.buffer, len);
                        memmove(extradata, gb.buffer, len);
                        avctx->extradata_size = len;
                        break;
                    }
                } else if (tag == MP4ESDescrTag) {
                    int flags;
                    bytestream2_skip(&gb, 2);
                    flags = bytestream2_get_byte(&gb);
                    if (flags & 0x80) //streamDependenceFlag
                        bytestream2_skip(&gb, 2);
                    if (flags & 0x40) //URL_Flag
                        bytestream2_skip(&gb, bytestream2_get_byte(&gb));
                    if (flags & 0x20) //OCRstreamFlag
                        bytestream2_skip(&gb, 2);
                }
            } while (bytestream2_get_bytes_left(&gb));
        } else if (avctx->codec_id != AV_CODEC_ID_ALAC) {
            avctx->extradata_size = extradata_size;
        }
    }

    ret = ffat_update_ctx(avctx);
    if (ret < 0)
        return ret;

#if !TARGET_OS_IPHONE && defined(__MAC_10_9)
    if (at->mode == kAudioCodecBitRateControlMode_Variable && avctx->rc_max_rate) {
        UInt32 max_size = avctx->rc_max_rate * avctx->frame_size / avctx->sample_rate;
        if (max_size)
            AudioConverterSetProperty(at->converter, kAudioCodecPropertyPacketSizeLimitForVBR,
                                      sizeof(max_size), &max_size);
    }
#endif

    ff_af_queue_init(avctx, &at->afq);

    at->encoding_frame = av_frame_alloc();
    if (!at->encoding_frame)
        return AVERROR(ENOMEM);

    return 0;
}

static OSStatus ffat_encode_callback(AudioConverterRef converter, UInt32 *nb_packets,
                                     AudioBufferList *data,
                                     AudioStreamPacketDescription **packets,
                                     void *inctx)
{
    AVCodecContext *avctx = inctx;
    ATDecodeContext *at = avctx->priv_data;
    AVFrame *frame;
    int ret;

    if (!at->frame_queue.available) {
        if (at->eof) {
            *nb_packets = 0;
            return 0;
        } else {
            *nb_packets = 0;
            return 1;
        }
    }

    frame = ff_bufqueue_get(&at->frame_queue);

    data->mNumberBuffers              = 1;
    data->mBuffers[0].mNumberChannels = avctx->ch_layout.nb_channels;
    data->mBuffers[0].mDataByteSize   = frame->nb_samples *
                                        av_get_bytes_per_sample(avctx->sample_fmt) *
                                        avctx->ch_layout.nb_channels;
    data->mBuffers[0].mData           = frame->data[0];
    if (*nb_packets > frame->nb_samples)
        *nb_packets = frame->nb_samples;

    ret = av_frame_replace(at->encoding_frame, frame);
    if (ret < 0) {
        *nb_packets = 0;
        return ret;
    }

    ff_bufqueue_add(avctx, &at->used_frame_queue, frame);

    return 0;
}

static int ffat_encode(AVCodecContext *avctx, AVPacket *avpkt,
                       const AVFrame *frame, int *got_packet_ptr)
{
    ATDecodeContext *at = avctx->priv_data;
    OSStatus ret;

    AudioBufferList out_buffers = {
        .mNumberBuffers = 1,
        .mBuffers = {
            {
                .mNumberChannels = avctx->ch_layout.nb_channels,
                .mDataByteSize = at->pkt_size,
            }
        }
    };
    AudioStreamPacketDescription out_pkt_desc = {0};

    if (frame) {
        AVFrame *in_frame;

        if (ff_bufqueue_is_full(&at->frame_queue)) {
            /*
             * The frame queue is significantly larger than needed in practice,
             * but no clear way to determine the minimum number of samples to
             * get output from AudioConverterFillComplexBuffer().
             */
            av_log(avctx, AV_LOG_ERROR, "Bug: frame queue is too small.\n");
            return AVERROR_BUG;
        }

        if ((ret = ff_af_queue_add(&at->afq, frame)) < 0)
            return ret;

        in_frame = av_frame_clone(frame);
        if (!in_frame)
            return AVERROR(ENOMEM);

        ff_bufqueue_add(avctx, &at->frame_queue, in_frame);
    } else {
        at->eof = 1;
    }

    if ((ret = ff_alloc_packet(avctx, avpkt, at->pkt_size)) < 0)
        return ret;


    out_buffers.mBuffers[0].mData = avpkt->data;

    *got_packet_ptr = avctx->frame_size / at->frame_size;

    ret = AudioConverterFillComplexBuffer(at->converter, ffat_encode_callback, avctx,
                                          got_packet_ptr, &out_buffers,
                                          (avctx->frame_size > at->frame_size) ? NULL : &out_pkt_desc);

    ff_bufqueue_discard_all(&at->used_frame_queue);

    if ((!ret || ret == 1) && *got_packet_ptr) {
        avpkt->size = out_buffers.mBuffers[0].mDataByteSize;
        ff_af_queue_remove(&at->afq, out_pkt_desc.mVariableFramesInPacket ?
                                     out_pkt_desc.mVariableFramesInPacket :
                                     avctx->frame_size,
                           &avpkt->pts,
                           &avpkt->duration);
        avpkt->flags |= AV_PKT_FLAG_KEY;
    } else if (ret && ret != 1) {
        av_log(avctx, AV_LOG_ERROR, "Encode error: %i\n", ret);
        return AVERROR_EXTERNAL;
    }

    return 0;
}

static av_cold void ffat_encode_flush(AVCodecContext *avctx)
{
    ATDecodeContext *at = avctx->priv_data;
    AudioConverterReset(at->converter);
    ff_bufqueue_discard_all(&at->frame_queue);
    ff_bufqueue_discard_all(&at->used_frame_queue);
}

static av_cold int ffat_close_encoder(AVCodecContext *avctx)
{
    ATDecodeContext *at = avctx->priv_data;
    AudioConverterDispose(at->converter);
    ff_bufqueue_discard_all(&at->frame_queue);
    ff_bufqueue_discard_all(&at->used_frame_queue);
    ff_af_queue_close(&at->afq);
    av_frame_free(&at->encoding_frame);
    return 0;
}

static const AVProfile aac_profiles[] = {
    { AV_PROFILE_AAC_LOW,   "LC"       },
    { AV_PROFILE_AAC_HE,    "HE-AAC"   },
    { AV_PROFILE_AAC_HE_V2, "HE-AACv2" },
    { AV_PROFILE_AAC_LD,    "LD"       },
    { AV_PROFILE_AAC_ELD,   "ELD"      },
    { AV_PROFILE_UNKNOWN },
};

#define AE AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
#if !TARGET_OS_IPHONE
    {"aac_at_mode", "ratecontrol mode", offsetof(ATDecodeContext, mode), AV_OPT_TYPE_INT, {.i64 = -1}, -1, kAudioCodecBitRateControlMode_Variable, AE, .unit = "mode"},
        {"auto", "VBR if global quality is given; CBR otherwise", 0, AV_OPT_TYPE_CONST, {.i64 = -1}, INT_MIN, INT_MAX, AE, .unit = "mode"},
        {"cbr",  "constant bitrate", 0, AV_OPT_TYPE_CONST, {.i64 = kAudioCodecBitRateControlMode_Constant}, INT_MIN, INT_MAX, AE, .unit = "mode"},
        {"abr",  "long-term average bitrate", 0, AV_OPT_TYPE_CONST, {.i64 = kAudioCodecBitRateControlMode_LongTermAverage}, INT_MIN, INT_MAX, AE, .unit = "mode"},
        {"cvbr", "constrained variable bitrate", 0, AV_OPT_TYPE_CONST, {.i64 = kAudioCodecBitRateControlMode_VariableConstrained}, INT_MIN, INT_MAX, AE, .unit = "mode"},
        {"vbr" , "variable bitrate", 0, AV_OPT_TYPE_CONST, {.i64 = kAudioCodecBitRateControlMode_Variable}, INT_MIN, INT_MAX, AE, .unit = "mode"},
#endif
    {"aac_at_quality", "quality vs speed control", offsetof(ATDecodeContext, quality), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 2, AE},
    { NULL },
};

#define FFAT_ENC_CLASS(NAME) \
    static const AVClass ffat_##NAME##_enc_class = { \
        .class_name = "at_" #NAME "_enc", \
        .option     = options, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define FFAT_ENC(NAME, ID, PROFILES, CAPS, CHANNEL_LAYOUTS, CH_LAYOUTS) \
    FFAT_ENC_CLASS(NAME) \
    const FFCodec ff_##NAME##_at_encoder = { \
        .p.name         = #NAME "_at", \
        CODEC_LONG_NAME(#NAME " (AudioToolbox)"), \
        .p.type         = AVMEDIA_TYPE_AUDIO, \
        .p.id           = ID, \
        .priv_data_size = sizeof(ATDecodeContext), \
        .init           = ffat_init_encoder, \
        .close          = ffat_close_encoder, \
        FF_CODEC_ENCODE_CB(ffat_encode), \
        .flush          = ffat_encode_flush, \
        .p.priv_class   = &ffat_##NAME##_enc_class, \
        .p.capabilities = AV_CODEC_CAP_DELAY | \
                          AV_CODEC_CAP_ENCODER_FLUSH CAPS, \
        .p.profiles     = PROFILES, \
        .p.wrapper_name = "at", \
        CODEC_CH_LAYOUTS_ARRAY(CH_LAYOUTS), \
        CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_U8), \
    };

static const AVChannelLayout aac_at_ch_layouts[] = {
    AV_CHANNEL_LAYOUT_MONO,
    AV_CHANNEL_LAYOUT_STEREO,
    AV_CHANNEL_LAYOUT_SURROUND,
    AV_CHANNEL_LAYOUT_4POINT0,
    AV_CHANNEL_LAYOUT_5POINT0,
    AV_CHANNEL_LAYOUT_5POINT1,
    AV_CHANNEL_LAYOUT_6POINT0,
    AV_CHANNEL_LAYOUT_6POINT1,
    AV_CHANNEL_LAYOUT_7POINT0,
    AV_CHANNEL_LAYOUT_7POINT1_WIDE_BACK,
    AV_CHANNEL_LAYOUT_QUAD,
    AV_CHANNEL_LAYOUT_OCTAGONAL,
    { 0 },
};

FFAT_ENC(aac,          AV_CODEC_ID_AAC,          aac_profiles, , aac_at_channel_layouts, aac_at_ch_layouts)
//FFAT_ENC(adpcm_ima_qt, AV_CODEC_ID_ADPCM_IMA_QT, NULL)
FFAT_ENC(alac,         AV_CODEC_ID_ALAC,         NULL, , NULL, NULL)
FFAT_ENC(ilbc,         AV_CODEC_ID_ILBC,         NULL, , NULL, NULL)
FFAT_ENC(pcm_alaw,     AV_CODEC_ID_PCM_ALAW,     NULL, , NULL, NULL)
FFAT_ENC(pcm_mulaw,    AV_CODEC_ID_PCM_MULAW,    NULL, , NULL, NULL)
