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

#include <pthread.h>
#include <stdatomic.h>
#include <float.h>

#include "libavutil/fifo.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/thread.h"
#include "libavutil/tx.h"
#include "libavformat/avformat.h"


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

typedef struct Station {
    char *name;
    enum Modulation modulation;
    double frequency;
    int64_t bandwidth;
    int64_t bandwidth_p2;
    float score;
    int timeout;            //since how many blocks was this detectable but not detected
    int multiplex_index;    //DAB can have multiple stations on one frequency

    struct SDRStream *stream;
} Station;

typedef struct FIFOElement {
    int64_t center_frequency;
    void *halfblock;
} FIFOElement;

typedef struct SDRStream {
    AVTXContext *ifft_ctx;
    AVTXContext *fft_p2_ctx;
    AVTXContext *ifft_p2_ctx;
    av_tx_fn ifft;
    av_tx_fn fft_p2;
    av_tx_fn ifft_p2;
    int block_size;
    int block_size_p2;
    int processing_index;
    float *out_buf;
    AVComplexFloat *block;
    AVComplexFloat *iblock;
    AVComplexFloat *icarrier;
    AVComplexFloat *iside;
    float *window;
    float *window_p2;
    Station *station;
    float am_amplitude;

    int frame_size;
    int frame_buffer_line;
    uint8_t *frame_buffer;
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
    char *dump_url;
    int fileheader_size;
    AVIOContext *dump_avio;
    /**
     * Current list of unambigously detected stations
     */
    Station **station;
    int nb_stations;
    /**
     * Current list of detected stations, these can be overlapping and low quality detections.
     * Used for probing. Stations are not removed from this when added to station.
     */
    Station **candidate_station;
    int nb_candidate_stations;
    int width, height;
    int single_ch_audio_st_index;
    int waterfall_st_index;
    int64_t freq;
    int64_t min_freq;
    int64_t max_freq;
    int64_t min_center_freq;
    int64_t max_center_freq;
    int sdr_sample_rate;
    int sdr_agc;
    int sdr_adcc;
    int64_t bandwidth;
    int64_t last_pts;
    int64_t pts;
    int block_size;
    int kbd_alpha;
    AVComplexFloat *windowed_block;
    int64_t block_center_freq;              ///< center frequency the current block contains
    int64_t station_freq;
    int sample_size;
    double sample_scale;

    int am_mode;                            ///< AMMode but using int for generic option access
    int emphasis_mode;
    int am_fft_ref;

    pthread_t hw_thread;
    int thread_started;
    pthread_mutex_t mutex;                  ///< Mutex to protect common variable between mainthread and hw_thread, and also to protect soapy from concurrent calls
    AVFifo *empty_block_fifo;
    AVFifo *full_block_fifo;
    atomic_int close_requested;
    int64_t wanted_freq;                    ///< center frequency we want the hw to provide next
    int seek_direction;                     ///< if a seek is requested this is -1 or 1 otherwise 0
    int skip_probe;

    /**
     * Setup the hardware for the requested frequency
     */
    int (*set_frequency_callback)(struct SDRContext *sdr, int64_t frequency);

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
    float rtlsdr_dc_offset;
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
    int (*demodulate)(SDRContext *sdr, int stream_index, AVPacket *pkt);
} ModulationDescriptor;

typedef struct BandDescriptor {
    const char *name;
    const char *shortname;
    int64_t freq_min;
    int64_t freq_max;
} BandDescriptor ;

extern const AVOption avpriv_sdr_options[];

/**
 * Set the center frequency of the hardware
 * this will check the argument and call set_frequency_callback()
 * It can be called before the thread is started or from within the thread,
 */
int avpriv_sdr_set_freq(SDRContext *sdr, int64_t freq);

int avpriv_sdr_common_init(AVFormatContext *s);

int avpriv_sdr_read_packet(AVFormatContext *s, AVPacket *pkt);

int avpriv_sdr_read_seek(AVFormatContext *s, int stream_index, int64_t target, int flags);

/**
 * shuts down threads, destroys mutex
 * Safe to call if no thread was started or after it was shutdown
 */
void avpriv_sdr_stop_threading(AVFormatContext *s);

int avpriv_sdr_read_close(AVFormatContext *s);

#endif /* AVRADIO_SDR_H */
