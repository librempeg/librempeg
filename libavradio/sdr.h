/*
 * SDR Input device / Demuxer / Demodulator
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

#ifndef AVRADIO_SDR_H
#define AVRADIO_SDR_H

#include <stdatomic.h>
#include <float.h>

#include "libavutil/fifo.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/thread.h"
#include "libavutil/tree.h"
#include "libavutil/tx.h"
#include "libavformat/avformat.h"

#define FREQ_BITS 22
#define TIMEBASE ((48000ll / 128) << FREQ_BITS)

#define INDEX2F(INDEX) (((INDEX) - sdr->block_size  + 0.5) * 0.5 * sdr->sdr_sample_rate / sdr->block_size      + sdr->block_center_freq)
#define F2INDEX(F)     (((    F) - sdr->block_center_freq) * 2   * sdr->block_size      / sdr->sdr_sample_rate + sdr->block_size  - 0.5)

typedef enum Mode {
    SingleStationMode, //< demodulate 1 station for Radio like usage
    AllStationMode,    //< demodulate all stations in current input
    ModeNB,
} Mode;

typedef enum AMMode {
    AMMidSide,
    AMLeftRight,
    AMInPhase,
    AMEnvelope,
    AMModeNB,
} AMMode;

typedef enum FMEmphasisMode {
    EMPHASIS_50us,
    EMPHASIS_75us,
    EMPHASIS_NONE,
    EMPHASISNB,
} FMEmphasisMode;

typedef enum Modulation {
    AM,
    FM,
    OFDM_DQPSK, //DAB
    //QAM, PSK, ...
} Modulation;

typedef enum SDR_GAIN {
    GAIN_DEFAULT = -3,
    GAIN_SW_AGC = -2,
    GAIN_SDR_AGC = -1,
} SDR_GAIN;

#define HISTOGRAMM_SIZE 9

typedef struct Station {
    char name[9];

    char radiotext[65];
    char artist[65];
    char title[65];
    char album[65];
    char programm_type_name[9];
    int program_id[2];
    uint8_t rt_ab_flag;
    uint8_t rtp_toggle_bit;
    uint8_t rtp_running_bit;
    uint16_t rtp_appgroup;

    enum Modulation modulation;
    double frequency;
    int nb_frequency;       ///< number of detections which are used to compute the frequency
    int64_t bandwidth;
    float score;
    int in_station_list;    ///< non zero if this station is in the station list
    int timeout;            //since how many blocks was this detectable but not detected
    int multiplex_index;    //DAB can have multiple stations on one frequency

    int detection_per_mix_frequency[HISTOGRAMM_SIZE];
    int non_detection_per_mix_frequency[HISTOGRAMM_SIZE];

    float (*rds_ring)[2];
    int rds_ring_pos;

    struct SDRStream *stream;
    int processing_index;
} Station;

typedef struct FIFOElement {
    int64_t center_frequency;
    float gain;
    void *halfblock;
} FIFOElement;

typedef struct SDRStream {
    int processing_index;
    float *out_buf;

    Station *station;
    float am_amplitude;

    int frame_size;
    int frame_buffer_line;
    uint8_t *frame_buffer;
    int64_t last_block_center_freq;
} SDRStream;

typedef struct SDRContext {
    const AVClass *class;
    AVFormatContext *avfmt;
    void *soapy;
    void *soapyRxStream;
    const char *current_direct_samp;
    AVTXContext *fft_ctx;
    av_tx_fn fft;
    Mode mode;
    AVRational fps;
    char *driver_name;
    AVDictionary *driver_dict;
    char *dump_url;
    int fileheader_size;
    AVIOContext *dump_avio;
    struct AVTreeNode *station_root;
    int width, height;
    int single_ch_audio_st_index;
    int waterfall_st_index;
    int demodulate_all_fm;
    int64_t freq;
    int64_t min_freq;
    int64_t max_freq;
    int64_t min_center_freq;
    int64_t max_center_freq;
    int sdr_sample_rate;
    float min_gain;
    float max_gain;
    int sdr_gain;
    float agc_min_headroom;
    float agc_max_headroom;
    float agc_max_headroom_time;
    int agc_low_time;
    float agc_gain;                         ///< current gain, should be accessed only by buffer thread after init
    atomic_int wanted_gain;
    int sdr_adcc;
    int64_t bandwidth;
    int64_t last_pts;
    int64_t pts;
    int block_size;
    double block_time;
    int kbd_alpha;
    AVComplexFloat *windowed_block;
    int64_t block_center_freq;              ///< center frequency the current block contains
    int wraparound;
    int64_t station_freq;
    int64_t user_wanted_freq;
    int sample_size;
    double sample_scale;

    int64_t am_bandwidth;
    int64_t fm_bandwidth;
    int64_t fm_bandwidth_p2;
    int am_block_size;
    int fm_block_size;
    int fm_block_size_p2;
    int rds_ring_size;
    AVComplexFloat *am_block;
    AVComplexFloat *am_iblock;
    AVComplexFloat *am_icarrier;
    float *am_window;
    AVComplexFloat *fm_iside;
    AVComplexFloat *fm_block;
    AVComplexFloat *fm_iblock;
    AVComplexFloat *fm_icarrier;
    float *fm_window;
    float *fm_window_p2;

    AVTXContext *am_ifft_ctx;
    AVTXContext *am_fft_ctx;
    AVTXContext *fm_ifft_ctx;
    AVTXContext *fm_fft_ctx;
    AVTXContext *fm_ifft_p2_ctx;
    av_tx_fn am_ifft;
    av_tx_fn am_fft;
    av_tx_fn fm_ifft;
    av_tx_fn fm_fft;
    av_tx_fn fm_ifft_p2;

    int am_mode;                            ///< AMMode but using int for generic option access
    int emphasis_mode;
    int am_fft_ref;

    float am_threshold;
    float fm_threshold;
    float am_multiple;
    float fm_multiple;
    float am_multiple_tolerance;

    pthread_t hw_thread;
    int thread_started;
    pthread_mutex_t mutex;                  ///< Mutex to protect common variable between mainthread and hw_thread, and also to protect soapy from concurrent calls
    AVFifo *empty_block_fifo;
    AVFifo *full_block_fifo;
    atomic_int close_requested;
    atomic_int_least64_t wanted_freq;       ///< center frequency we want the hw to provide next, only written to by main thread
    atomic_int seek_direction;              ///< if a seek is requested this is -1 or 1 otherwise 0, only written to by main thread
    int skip_probe;

    /**
     * Setup the hardware for the requested frequency
     */
    int64_t (*set_frequency_callback)(struct SDRContext *sdr, int64_t frequency);

    /**
     * Setup the hardware for the requested gain
     * This must only be called from the buffer thread after setup (or more mutex calls are needed)
     */
    int (*set_gain_callback)(struct SDRContext *sdr, float gain);

    /**
     * Read from the hardware, block if nothing available with a reasonable timeout
     *
     * @param remaining space remaining in fifo_element
     *
     * @return AVERROR(EAGAIN) in case of timeout or the number of elements read
     */
    int (*read_callback)(struct SDRContext *sdr, FIFOElement *fifo_element, int remaining);

    /**
     * space remaining in teh current block in the file.
     *
     * This is needed as the sdrdemux code can be setup to use a different block size
     * than what was used when capturing the data
     */
    int remaining_file_block_size;

    AVComplexFloat *block;
    float *len2block;
    float *window;

    int missing_streams;

    int rtlsdr_fixes;
    int sdrplay_fixes;
    int dc_fix;
} SDRContext;

typedef struct ModulationDescriptor {
    const char *name;
    const char *shortname;
    enum Modulation modulation;
    enum AVMediaType media_type;

    /**
     * Scan all of the current sdr->block and call create_station() for each found station
     */
    int (*probe)(SDRContext *sdr);

    /**
     * Demodulate given station into packet
     */
    int (*demodulate)(SDRContext *sdr, Station *station, AVStream *st, AVPacket *pkt);
} ModulationDescriptor;

typedef struct BandDescriptor {
    const char *name;
    const char *shortname;
    int64_t freq_min;
    int64_t freq_max;
} BandDescriptor ;

extern const AVOption ff_sdr_options[];

extern ModulationDescriptor ff_sdr_modulation_descs[];

/**
 * Detect hw bug specific workarounds.
 */
void ff_sdr_autodetect_workarounds(SDRContext *sdr);

/**
 * Set the center frequency of the hardware
 * this will check the argument and call set_frequency_callback()
 * It can be called before the thread is started or from within the thread,
 */
int ff_sdr_set_freq(SDRContext *sdr, int64_t freq);

int ff_sdr_common_init(AVFormatContext *s);

int ff_sdr_read_packet(AVFormatContext *s, AVPacket *pkt);

int ff_sdr_read_seek(AVFormatContext *s, int stream_index, int64_t target, int flags);

/**
 * shuts down threads, destroys mutex
 * Safe to call if no thread was started or after it was shutdown
 */
void ff_sdr_stop_threading(AVFormatContext *s);

int ff_sdr_read_close(AVFormatContext *s);

int ff_sdr_vissualization(SDRContext *sdr, AVStream *st, AVPacket *pkt);

/**
 * Find stations within the given parameters.
 * @param[out] station_list array to return stations in
 * @param nb_stations size of station array
 * @returns number of stations found
 */
int ff_sdr_find_stations(SDRContext *sdr, double freq, double range, Station **station_list, int station_list_size);

int ff_sdr_histogram_score(Station *s);

/**
 * Decode RDS
 * @param signal the time domain RDS signal
 */
int ff_sdr_decode_rds(SDRContext *sdr, Station *station, AVComplexFloat *signal);

static inline float len2(AVComplexFloat c)
{
    return c.re*c.re + c.im*c.im;
}

#endif /* AVRADIO_SDR_H */
