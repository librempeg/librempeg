/*
 * SDR Input device / Demodulator
 * Copyright (c) 2023 Michael Niedermayer
 *
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

#include <SoapySDR/Version.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include "sdr.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/thread.h"
#include "libavutil/tx.h"
#include "libavcodec/kbdwin.h"
#include "libavformat/avformat.h"
#include "libavformat/demux.h"
#include "libavformat/internal.h"
#include "libavdevice/avdevice.h"

#define MAX_CHANNELS 4

static int sdrindev_read_callback(SDRContext *sdr, FIFOElement *fifo_element, int remaining)
{
    AVFormatContext *avfmt = sdr->avfmt;
    void *soapy_buffers[MAX_CHANNELS] = {(uint8_t*)fifo_element->halfblock + (sdr->block_size - remaining) * sdr->sample_size, NULL};
    long long soapy_ts;
    int soapy_flags;

    int ret = SoapySDRDevice_readStream(sdr->soapy, sdr->soapyRxStream,
                                        soapy_buffers,
                                        remaining,
                                        &soapy_flags,
                                        &soapy_ts,
                                        10*1000);

    if (ret == SOAPY_SDR_TIMEOUT) {
        return AVERROR(EAGAIN);
    } else if (ret == SOAPY_SDR_OVERFLOW) {
        av_log(avfmt, AV_LOG_WARNING, "SOAPY OVERFLOW\n");
        return AVERROR(EAGAIN);
    } else if (ret < 0) {
        av_log(avfmt, AV_LOG_ERROR, "SoapySDRDevice_readStream() Failed with (%d) %s\n", ret, SoapySDRDevice_lastError());
        av_usleep(10*1000);
        return AVERROR(EAGAIN);
    }

    return ret;
}

static int sdrindev_set_gain_callback(SDRContext *sdr, float gain)
{
    AVFormatContext *avfmt = sdr->avfmt;
    SoapySDRDevice *soapy = sdr->soapy;

    if (sdr->sdr_gain == GAIN_DEFAULT)
        return 0;

    //sdrplay has a inverted gain range, not using max_gain as this is a user parameter
    if (sdr->sdrplay_fixes > 0) {
        gain = FFMIN(48 - gain, 45);
    }

    if (soapy) {
        int ret = SoapySDRDevice_setGainMode(soapy, SOAPY_SDR_RX, 0, sdr->sdr_gain == GAIN_SDR_AGC);
        if (ret) {
            av_log(avfmt, AV_LOG_WARNING, "Failed to set gain mode %d (%s)\n", sdr->sdr_gain == GAIN_SDR_AGC, SoapySDRDevice_lastError());
        }

        if (sdr->sdr_gain != GAIN_SDR_AGC) {
            ret = SoapySDRDevice_setGain(soapy, SOAPY_SDR_RX, 0, gain);
            if (ret) {
                av_log(avfmt, AV_LOG_WARNING, "Failed to set gain to %f (%s)\n", gain, SoapySDRDevice_lastError());
            }
        }
    }
    return 0;
}

static int64_t sdrindev_set_frequency_callback(SDRContext *sdr, int64_t freq)
{
    AVFormatContext *avfmt = sdr->avfmt;
    SoapySDRDevice *soapy = sdr->soapy;
    if (soapy) {
        if (sdr->rtlsdr_fixes>0) {
            // The official unmodified RTLSDR needs to be put in Direct sampling Q-ADC mode below 24Mhz and non direct sampling above
            const char *value = freq < 24000000 ? "2" : "0";
            if (!sdr->current_direct_samp || strcmp(value, sdr->current_direct_samp)) {
                int ret = SoapySDRDevice_writeSetting(soapy, "direct_samp", value);
                if (ret) {
                    av_log(avfmt, AV_LOG_WARNING, "SoapySDRDevice_writeSetting fail while adjusting direct sampling mode for RTLSDR\n");
                } else
                    sdr->current_direct_samp = value;
            }
            //The R820T has a 16 bit fractional PLL which can do only multiplies of 439.45
            //Its more complex but this approximation works
            //It has to be noted that SOAPY does not tell us about this, instead saopy
            //pretends whatever we ask for we get exactly, but we dont
            //For more details see: michelebavaro.blogspot.com/2014/05/gnss-carrier-phase-rtlsdr-and.html
            freq = lrint(freq / 439.45) * 439.45;
        }

        if (SoapySDRDevice_setFrequency(soapy, SOAPY_SDR_RX, 0, freq, NULL) != 0) {
            av_log(avfmt, AV_LOG_ERROR, "setFrequency fail: %s\n", SoapySDRDevice_lastError());
            return AVERROR_EXTERNAL;
        }
    }
    return freq;
}

static void print_and_free_list(AVFormatContext *s, char** names, size_t length, const char *title)
{
    if (length) {
        av_log(s, AV_LOG_INFO, "%s:", title);
        for (int i = 0; i < length; i++)
            av_log(s, AV_LOG_INFO, "%c%s", ", "[!i], names[i]);
        av_log(s, AV_LOG_INFO, "\n");
    }
    SoapySDRStrings_clear(&names, length);
}

/**
 * Initial setup of SDR HW through soapy.
 * This will go over available settings and match them up with what the user requested
 * That is List available + check user settings + autoselect if the user made no choice
 * It will also set callbacks and run the common init.
 */
static int sdrindev_initial_hw_setup(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;
    int ret, i;

    SoapySDRRange *ranges, range;
    SoapySDRKwargs *results = NULL;
    SoapySDRKwargs args = {};
    int has_agc, has_adcc;
    SoapySDRArgInfo *arg_info = NULL;
    size_t length;
    char** names;
    int64_t max_sample_rate;
    SoapySDRDevice *soapy = NULL;
    SoapySDRStream *soapyRxStream = NULL;
    const char * soapy_format;

    sdr->read_callback          = sdrindev_read_callback;
    sdr->set_frequency_callback = sdrindev_set_frequency_callback;
    sdr->set_gain_callback      = sdrindev_set_gain_callback;

    // Go over all available soapy devices
    // Print the usable ones, and choose one unless the user has choosen one
    results = SoapySDRDevice_enumerate(NULL, &length);
    for (i = 0; i < length; i++) {
        int usable = 1;
        for (int j = 0; j < results[i].size; j++) {
            if (!strcmp("driver", results[i].keys[j])) {
                if (!strcmp("audio", results[i].vals[j])) {
                    usable = 0;
                } else if (!sdr->driver_name) {
                    sdr->driver_name = av_strdup(results[i].vals[j]);
                    if (!sdr->driver_name)
                        return AVERROR(ENOMEM);
                }
            }
        }
        if (!usable)
            continue;
        av_log(s, AV_LOG_INFO, "Soapy enumeration %d\n", i);
        for (int j = 0; j < results[i].size; j++) {
            av_log(s, AV_LOG_INFO, "    results %s = %s\n", results[i].keys[j], results[i].vals[j]);
        }
    }
    SoapySDRKwargsList_clear(results, length);

    av_log(s, AV_LOG_INFO, "Opening %s\n", sdr->driver_name);
    if (!sdr->driver_name)
        return AVERROR(EINVAL); //No driver specified and none found

    if (sdr->rtlsdr_fixes < 0)
        sdr->rtlsdr_fixes = !strcmp(sdr->driver_name, "rtlsdr");
    if (sdr->sdrplay_fixes < 0)
        sdr->sdrplay_fixes = !strcmp(sdr->driver_name, "sdrplay");

    SoapySDRKwargs_set(&args, "driver", sdr->driver_name);
    sdr->soapy = soapy = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (soapy == NULL) {
        av_log(s, AV_LOG_ERROR, "SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return AVERROR_EXTERNAL;
    }

    names = SoapySDRDevice_listTimeSources(soapy, &length);
    print_and_free_list(s, names, length, "Clocks");

    //Go over all Antennas and print them
    names = SoapySDRDevice_listAntennas(soapy, SOAPY_SDR_RX, 0, &length);
    print_and_free_list(s, names, length, "Antennas");

    //Go over all tunable elements
    names = SoapySDRDevice_listFrequencies(soapy, SOAPY_SDR_RX, 0, &length);
    print_and_free_list(s, names, length, "Tunables");

    //Go over all Gain Elements and print them
    names = SoapySDRDevice_listGains(soapy, SOAPY_SDR_RX, 0, &length);
    print_and_free_list(s, names, length, "Rx Gain Elements");

    //Inform the user if AGC is supported and setup AGC as requested by the user
    has_agc = SoapySDRDevice_hasGainMode(soapy, SOAPY_SDR_RX, 0);
    av_log(s, AV_LOG_INFO, "RX AGC Supported: %s\n", has_agc ? "yes" : "no");
    if (!has_agc &&  sdr->sdr_gain == GAIN_SDR_AGC) {
        av_log(s, AV_LOG_WARNING, "hardware AGC unsupported switching to software AGC\n");
        sdr->sdr_gain = GAIN_SW_AGC;
    }

    //Inform the user if automatic DC correction is supported and setup DC correction as requested by the user
    has_adcc = SoapySDRDevice_hasDCOffsetMode(soapy, SOAPY_SDR_RX, 0);
    av_log(s, AV_LOG_INFO, "RX Automatic DC Correction Supported: %s\n", has_adcc ? "yes" : "no");
    if (has_adcc && sdr->sdr_adcc >= 0)
        SoapySDRDevice_setDCOffsetMode(soapy, SOAPY_SDR_RX, 0, sdr->sdr_adcc);

    //Inform the user about the Gain range available
    range = SoapySDRDevice_getGainRange(soapy, SOAPY_SDR_RX, 0);
    av_log(s, AV_LOG_INFO, "Rx Gain range: %f dB - %f dB\n", range.minimum, range.maximum);

    if (!sdr->min_gain)
        sdr->min_gain = range.minimum;

    if (!sdr->max_gain)
        sdr->max_gain = range.maximum;
    if (sdr->min_gain > sdr->max_gain) {
        av_log(s, AV_LOG_ERROR, "Invalid gain range\n");
        return AVERROR(EINVAL);
    }

    //Inform the user about the Frequency ranges available, verify the range requested by the user and set the range if the user has not specified one
    ranges = SoapySDRDevice_getFrequencyRange(soapy, SOAPY_SDR_RX, 0, &length);
    av_log(s, AV_LOG_INFO, "Rx freq ranges: ");
    for (i = 0; i < length; i++) {
        double minimum = ranges[i].minimum;
        av_log(s, AV_LOG_INFO, "[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);

        if (sdr->rtlsdr_fixes>0) {
            //rtlsdr supports direct sampling down to 500khz, frequency range returned is wrong
            minimum = FFMIN(minimum, 500000);
            if (!sdr->min_freq)
                sdr->min_freq = minimum;
        }

        if (sdr->max_freq > ranges[i].maximum)
            continue;
        if (sdr->min_freq && sdr->min_freq < minimum)
            continue;
        break;
    }
    av_log(s, AV_LOG_INFO, "\n");
    if (i == length) {
        av_log(s, AV_LOG_ERROR, "Invalid frequency range\n");
        return AVERROR(EINVAL);
    }
    if (!sdr->max_freq)
        sdr->max_freq = ranges[i].maximum;
    if (!sdr->min_freq)
        sdr->min_freq = ranges[i].minimum;
    free(ranges); ranges = NULL;
    av_log(s, AV_LOG_INFO, "frequency range: %"PRId64" - %"PRId64"\n", sdr->min_freq, sdr->max_freq);
    //TODO do any drivers support multiple distinct frequency ranges ? if so we pick just one, thats not ideal

    //Inform the user about the Sample rate ranges available, verify the rate requested by the user and choose the range if the user has not specified one
    ranges = SoapySDRDevice_getSampleRateRange(soapy, SOAPY_SDR_RX, 0, &length);
    max_sample_rate = 0;
    av_log(s, AV_LOG_INFO, "SampleRate ranges: ");
    for (i = 0; i < length; i++) {
        av_log(s, AV_LOG_INFO, "[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
        if (sdr->sdr_sample_rate &&
            (sdr->sdr_sample_rate < ranges[i].minimum || sdr->sdr_sample_rate > ranges[i].maximum))
            continue;
        if (sdr->rtlsdr_fixes)
            // 2.56, 2.88 and 3.2 Mhz do not work reliable here so lets not automatically choose them, the user can override this
            if (ranges[i].maximum > (2560000 + 2160000)/2)
                continue;
        max_sample_rate = FFMAX(max_sample_rate, ranges[i].maximum);
    }
    av_log(s, AV_LOG_INFO, "\n");
    free(ranges); ranges = NULL;
    if (!sdr->sdr_sample_rate) {
        sdr->sdr_sample_rate = max_sample_rate;
    }

    // We disallow odd sample rates as they result in either center or endpoint frequencies to be non integer. No big deal but simpler is better
    if (!max_sample_rate || sdr->sdr_sample_rate%2) {
        av_log(s, AV_LOG_ERROR, "Invalid sdr sample rate\n");
        return AVERROR(EINVAL);
    }

    //List the bandwidth ranges available
    ranges = SoapySDRDevice_getBandwidthRange(soapy, SOAPY_SDR_RX, 0, &length);
    av_log(s, AV_LOG_INFO, "Bandwidth ranges: ");
    for (i = 0; i < length; i++) {
        av_log(s, AV_LOG_INFO, "[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
    }
    av_log(s, AV_LOG_INFO, "\n");
    free(ranges); ranges = NULL;

    //apply settings
    if (SoapySDRDevice_setSampleRate(soapy, SOAPY_SDR_RX, 0, sdr->sdr_sample_rate) != 0) {
        av_log(s, AV_LOG_ERROR, "setSampleRate fail: %s\n", SoapySDRDevice_lastError());
        return AVERROR_EXTERNAL;
    }
    ret = ff_sdr_set_freq(sdr, sdr->wanted_freq);
    if (ret < 0)
        return ret;

    //List the available custom options for this specific driver to the user
    arg_info = SoapySDRDevice_getSettingInfo(soapy, &length);
    av_log(s, AV_LOG_INFO, "settings: ");
    for (i=0; i<length; i++) {
        SoapySDRArgInfo *info = arg_info + i;
        const char *def = SoapySDRDevice_readSetting(soapy, info->key);
        av_log(s, AV_LOG_INFO, "key:%s desc:%s def:%s ", info->key, info->description, def);
        if (info->units && *info->units)
            av_log(s, AV_LOG_INFO, "units: %s ", info->units);
        if (info->type == SOAPY_SDR_ARG_INFO_INT || info->type == SOAPY_SDR_ARG_INFO_FLOAT) {
            av_log(s, AV_LOG_INFO, "range: %g - %g ", info->range.minimum, info->range.maximum);
        }
        if (info->numOptions) {
            av_log(s, AV_LOG_INFO, "Options: ");
            for (int o= 0; o<info->numOptions; o++){
                av_log(s, AV_LOG_INFO, "%s (%s) ", info->options[o], info->optionNames[o]);
            }
        }
    }
    av_log(s, AV_LOG_INFO, "\n");

    soapy_format = SoapySDRDevice_getNativeStreamFormat(soapy, SOAPY_SDR_RX, 0, &sdr->sample_scale);
    av_log(s, AV_LOG_INFO, "native format: %s, full scale: %f\n", soapy_format, sdr->sample_scale);
    if (!strcmp(soapy_format, SOAPY_SDR_CS8)) {
        sdr->sample_size = 2;
    } else if (!strcmp(soapy_format, SOAPY_SDR_CS16)) {
        sdr->sample_size = 4;
    } else {
        sdr->sample_size = 8;
        sdr->sample_scale = 1.0;
        soapy_format = SOAPY_SDR_CF32;
    }

    //setup a stream (complex 32bit floats)
#if SOAPY_SDR_API_VERSION < 0x00080000 // The old version is still widely used so we must support it
   if (SoapySDRDevice_setupStream(soapy, &soapyRxStream, SOAPY_SDR_RX, soapy_format, NULL, 0, NULL))
       soapyRxStream = NULL;
#else
    soapyRxStream = SoapySDRDevice_setupStream(soapy, SOAPY_SDR_RX, soapy_format, NULL, 0, NULL);
#endif
    if (!soapyRxStream) {
        av_log(s, AV_LOG_ERROR, "setupStream fail: %s\n", SoapySDRDevice_lastError());
        return AVERROR_EXTERNAL;
    }
    sdr->soapyRxStream = soapyRxStream;

    sdr->bandwidth = SoapySDRDevice_getBandwidth(soapy, SOAPY_SDR_RX, 0);

    // rtlsdr doesnt return a valid value
    if (!sdr->bandwidth)
        sdr->bandwidth = sdr->sdr_sample_rate * 4 / 5;
    av_log(s, AV_LOG_INFO, "bandwidth %"PRId64"\n", sdr->bandwidth);

    SoapySDRDevice_activateStream(soapy, soapyRxStream, 0, 0, 0);

    return ff_sdr_common_init(s);
}

static int sdrindev_read_close(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;
    SoapySDRDevice *soapy = sdr->soapy;

    ff_sdr_stop_threading(s);

    if (soapy) {
        if (sdr->soapyRxStream) {
            SoapySDRDevice_deactivateStream(soapy, sdr->soapyRxStream, 0, 0);
            SoapySDRDevice_closeStream(soapy, sdr->soapyRxStream);
            sdr->soapyRxStream = NULL;
        }

        SoapySDRDevice_unmake(soapy);
        sdr->soapy = NULL;
    }

    return ff_sdr_read_close(s);
}

static int sdr_get_device_list(AVFormatContext *ctx, AVDeviceInfoList *device_list)
{
    SoapySDRKwargs *results = NULL;
    size_t length;
    int ret = 0;
    int i;

    if (!device_list)
        return AVERROR(EINVAL);

    results = SoapySDRDevice_enumerate(NULL, &length);
    for (i = 0; i < length; i++) {
        AVDeviceInfo *device = NULL;
        device = av_mallocz(sizeof(AVDeviceInfo));
        if (!device) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        for (int j = 0; j < results[i].size; j++) {
            if (!strcmp("driver", results[i].keys[j]) && !device->device_name)
                device->device_name = av_strdup(results[i].vals[j]);
            if (!strcmp("label", results[i].keys[j]) && !device->device_description)
                device->device_description = av_strdup(results[i].vals[j]);
        }

        if (device->device_name) {
            if ((ret = av_dynarray_add_nofree(&device_list->devices,
                                              &device_list->nb_devices, device)) < 0)
                goto fail;
        }
        continue;
        fail:
        if (device) {
            av_freep(&device->device_name);
            av_freep(&device->device_description);
            av_freep(&device);
        }
        break;
    }
    SoapySDRKwargsList_clear(results, length);

    return ret;
}

static const AVClass sdr_demuxer_class = {
    .class_name = "sdr",
    .item_name  = av_default_item_name,
    .option     = ff_sdr_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_RADIO_INPUT,
};

const AVInputFormat ff_sdr_demuxer = {
    .name           = "sdr",
    .long_name      = NULL_IF_CONFIG_SMALL("Software Defined Radio Demodulator"),
    .priv_data_size = sizeof(SDRContext),
    .read_header    = sdrindev_initial_hw_setup,
    .read_packet    = ff_sdr_read_packet,
    .read_close     = sdrindev_read_close,
    .read_seek      = ff_sdr_read_seek,
    .get_device_list= sdr_get_device_list,
    .flags          = AVFMT_NOFILE,
    .flags_internal = FF_FMT_INIT_CLEANUP,
    .priv_class = &sdr_demuxer_class,
};
