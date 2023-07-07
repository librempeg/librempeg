/*
 * SDR Demuxer / Demodulator
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

/**
 * @file
 *
 *
 */

/**
 * TODO
 * * DAB
 * * DVB
 * * Improve probing using multiple detections and differential detection
 *
 */

#include "sdr.h"

#include <pthread.h>
#include <stdatomic.h>
#include <float.h>
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/thread.h"
#include "libavutil/tree.h"
#include "libavutil/tx.h"
#include "libavutil/xga_font_data.h"
#include "libavcodec/kbdwin.h"
#include "libavformat/avformat.h"
#include "libavformat/demux.h"
#include "libavformat/internal.h"

#ifdef SYN_TEST
#include "libavutil/lfg.h"
#endif

#define FREQ_BITS 22
#define TIMEBASE ((48000ll / 128) << FREQ_BITS)

#define AM_FREQ_TOLERANCE 5
#define FM_FREQ_TOLERANCE 500

#define STATION_TIMEOUT 100 ///< The number of frames after which a station is removed if it was not detected
#define CANDIDATE_STATION_TIMEOUT 4

#define AM_MAX23 0.06     //smaller causes failure on synthetic signals
#define AM_MAX4  0.02

//Least squares fit at 1khz points of frequency response shown by Frank McClatchie, FM SYSTEMS, INC. 800-235-6960
static double emphasis75us(int f)
{
    return ((((((((- 4.79546E-9 * f + 5.32101E-7) * f - 0.0000254577) * f + 0.000687225) * f
                   - 0.0114925) * f + 0.122781  ) * f - 0.827885    ) * f + 3.25025    ) * f - 4.6049) * f + 2.06937;
}

/**
 * Apply emphasis filter in frequency domain
 */
static void apply_deemphasis(SDRContext *sdr, AVComplexFloat *data, int len, int sample_rate, int dir)
{
    if (sdr->emphasis_mode == EMPHASIS_NONE)
        return;

    len = FFMIN(len, len * 19000 / sample_rate);

    for (int i = 1; i < len; i++) {
        double index = 1.0 + i / 1000;
        double scale;

        if (sdr->emphasis_mode == EMPHASIS_50us)
            index = FFMIN(0, index - 1061);

        scale = ff_exp10(-emphasis75us(index) / 20.0);

        data[i*dir].re *= scale;
        data[i*dir].im *= scale;
    }
}

static float len2(AVComplexFloat c)
{
    return c.re*c.re + c.im*c.im;
}

static void free_station(Station *station)
{
    av_freep(&station->name);
    if (station->stream)
        station->stream->station = NULL;
    av_free(station);
}

static inline int histogram_index(SDRContext *sdr, double f)
{
    f = HISTOGRAMM_SIZE*((f - sdr->block_center_freq) / sdr->sdr_sample_rate + 0.5);
    return av_clip((int)f, 0, HISTOGRAMM_SIZE-1);
}

static int histogram_score(Station *s)
{
    int score = 0;
    for(int i = 0; i<HISTOGRAMM_SIZE; i++) {
        score +=
             (5*s->detection_per_mix_frequency[i] > s->non_detection_per_mix_frequency[i])
            -(5*s->detection_per_mix_frequency[i] < s->non_detection_per_mix_frequency[i]);
    }
    return score;
}

typedef struct FindStationContext {
    double freq;
    double range;
    Station **station_list;
    int station_list_size;
    int nb_stations;
} FindStationContext;

static int find_station_cmp(void *opaque, void *elem)
{
    FindStationContext *c = opaque;
    Station *station = elem;
    double distance = station->frequency - c->freq;
    if (distance < -c->range)
        return -1;
    if (distance >  c->range)
        return  1;
    return 0;
}

static int find_station_enu(void *opaque, void *elem)
{
    FindStationContext *c = opaque;
    if (c->nb_stations < c->station_list_size) {
        c->station_list[c->nb_stations++] = elem;
    } else
        av_log(NULL, AV_LOG_WARNING, "find station reached list size of %d\n", c->station_list_size);

    return 0;
}

static int free_station_enu(void *opaque, void *elem)
{
    free_station(elem);
    return 0;
}

/**
 * Find stations within the given parameters.
 * @param[out] station_list array to return stations in
 * @param nb_stations size of station array
 * @returns number of stations found
 */
static int find_stations(SDRContext *sdr, double freq, double range, Station **station_list, int station_list_size)
{
    FindStationContext find_station_context;
    find_station_context.freq = freq;
    find_station_context.range = range;
    find_station_context.station_list = station_list;
    find_station_context.station_list_size = station_list_size;
    find_station_context.nb_stations = 0;
    av_tree_enumerate(sdr->station_root, &find_station_context, find_station_cmp, find_station_enu);
    return find_station_context.nb_stations;
}

static int create_station(SDRContext *sdr, Station *candidate_station) {
    enum Modulation modulation  = candidate_station->modulation;
    double freq                 = candidate_station->frequency;
    int64_t bandwidth           = candidate_station->bandwidth;
    int64_t bandwidth_p2        = candidate_station->bandwidth_p2;
    float score                 = candidate_station->score;
    void *tmp;
    int i;
    Station *best_station = NULL;
    float drift = bandwidth/3.0;
    double best_distance = drift;
    int conflict = INT_MAX;
    int nb_candidate_conflict = 0;
    int nb_candidate_match = 0;

    if (candidate_station->in_station_list)
        return 0;

    // suspect looking histogram
    if (histogram_score(candidate_station) <= 0)
        return 0;

    Station *station_list[1000];
    int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.5, station_list, FF_ARRAY_ELEMS(station_list));
    for (i=0; i<nb_stations; i++) {
        Station *s = station_list[i];
        double delta = fabs(s->frequency - freq);

        if (!s->in_station_list)
            continue;

        // Station already added, or we have 2 rather close stations
        //FIXME we want to make sure that the stronger station is not skiped but we also dont want to add a station twice
        if (modulation == s->modulation && delta < best_distance) {
            best_distance = delta;
            best_station = s;
        }
        if (modulation !=s->modulation && delta < (bandwidth + s->bandwidth)/2.1) {
            conflict = FFMIN(conflict, s->timeout);
            // special case, lets not remove an actively listen to station, this can be done too but that needs more thought
            if (s->stream)
                conflict = 0;
        }
    }
    if (best_station) {
        if (score > best_station->score && conflict == INT_MAX) {
            int log_level = fabs(best_station->frequency - freq) < 3.0 ? AV_LOG_DEBUG : AV_LOG_WARNING;
            av_log(sdr->avfmt, log_level, "Update station score:%f -> %f freq: %f %f -> %f\n",
                    best_station->score, score,
                    best_station->frequency - freq, best_station->frequency, freq
            );

            if (best_station->stream) {
                candidate_station->stream = best_station->stream;
                best_station->stream = NULL;
                candidate_station->stream->station = candidate_station;
            }
            candidate_station->in_station_list = 1;
            best_station->in_station_list = 0;
        }
        return 2;
    }

    nb_candidate_match += candidate_station->nb_frequency - 1;
    for (i=0; i<nb_stations; i++) {
        int freq_precission = modulation == AM ? AM_FREQ_TOLERANCE : FM_FREQ_TOLERANCE;
        Station *s = station_list[i];
        double delta = fabs(s->frequency - freq);

        // Station already added, or we have 2 rather close stations
        if (modulation == s->modulation && delta < freq_precission && s != candidate_station) {
            nb_candidate_match += s->nb_frequency;
        }
        if (modulation != s->modulation && delta < (bandwidth + s->bandwidth)/2.1)
            nb_candidate_conflict += s->nb_frequency;
    }
    //if we have a recent conflict with an established station, skip this one
    if (conflict < CANDIDATE_STATION_TIMEOUT)
        return -1;
    if (conflict < candidate_station->timeout)
        return -1;

    //AM detection is less reliable ATM so we dont want it to override FM stations
    if (modulation == AM && conflict < INT_MAX)
        return -1;

    // if we just found a new station or have some conflicting candidates then lets also probe the next frame
    sdr->skip_probe = 0;

    if (nb_candidate_match < nb_candidate_conflict + 1)
        return -1;

    if (conflict < INT_MAX) {
        for (i=0; i<nb_stations; i++) {
            Station *s = station_list[i];
            double delta = fabs(s->frequency - freq);
            // We recheck that the stations we remove are not active because floating point could round differently
            if (s->stream == NULL &&
                modulation != s->modulation && delta < (bandwidth + s->bandwidth)/2.1) {

                s->in_station_list = 0;
            }
        }
    }

    candidate_station->in_station_list = 1;

    av_log(sdr, AV_LOG_INFO, "create_station %d f:%f bw:%"PRId64"/%"PRId64" score: %f\n", modulation, freq, bandwidth, bandwidth_p2, score);

    return 1;
}

static void create_stations(SDRContext *sdr)
{
    Station *station_list[1000];

    if (!sdr->block_center_freq)
        return;

    int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.5, station_list, FF_ARRAY_ELEMS(station_list));

    for(int i = 0; i<nb_stations; i++) {
        create_station(sdr, station_list[i]);
    }
}

static int station_cmp(const void *key, const void *b)
{
    const Station *sa = key;
    const Station *sb = b;
    return 2*((sa->frequency  > sb->frequency ) - (sa->frequency  < sb-> frequency))
             +(sa->modulation > sb->modulation) - (sa->modulation < sb->modulation);
}

static void *tree_insert(struct AVTreeNode **rootp, void *key,
                  int (*cmp)(const void *key, const void *b),
                  struct AVTreeNode **next)
{
    if (!*next)
        *next = av_mallocz(av_tree_node_size); //FIXME check ENOMEM
    return av_tree_insert(rootp, key, cmp, next);
}

static void *tree_remove(struct AVTreeNode **rootp, void *key,
                  int (*cmp)(const void *key, const void *b), struct AVTreeNode **next)
{
    av_freep(next);
    return av_tree_insert(rootp, key, cmp, next);
}

/**
 * remove stations which we no longer receive well
 * Especially with AM and weather conditions stations disapear, this keeps things a bit more tidy
 */
static void decay_stations(SDRContext *sdr)
{
    Station *station_list[1000];
    int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->bandwidth*0.5, station_list, FF_ARRAY_ELEMS(station_list));

    for (int i=0; i<nb_stations; i++) {
        Station *station = station_list[i];

        if (station->frequency - station->bandwidth/2 < sdr->block_center_freq - sdr->bandwidth/2 ||
            station->frequency + station->bandwidth/2 > sdr->block_center_freq + sdr->bandwidth/2)
            continue;

        if (station->timeout)
            station->non_detection_per_mix_frequency[histogram_index(sdr, station->frequency)] ++;

        if (station->in_station_list) {
            int station_timeout = STATION_TIMEOUT;
            int hs = histogram_score(station);

            if (hs == 0) {
                station_timeout = 5; //give the station a moment to be properly detected and then discard it
            } else if(hs < 0) {
                station_timeout = 0; //probably not a station
            }

            if (station->timeout++ > station_timeout) {
                if (!station->stream)
                    station->in_station_list = 0;
            }
        } else {
            if (station->timeout++ > CANDIDATE_STATION_TIMEOUT) {
                struct AVTreeNode *next = NULL;
                tree_remove(&sdr->station_root, station, station_cmp, &next);
                av_freep(&next);

                free_station(station);
            }
        }
    }
}

static int create_candidate_station(SDRContext *sdr, enum Modulation modulation, double freq, int64_t bandwidth, int64_t bandwidth_p2, float score) {
    Station *station;
    void *tmp;
    struct AVTreeNode *next = NULL;
    Station *station_list[1000];
    double snapdistance = modulation == AM ? AM_FREQ_TOLERANCE : FM_FREQ_TOLERANCE;
    int nb_stations = find_stations(sdr, freq, snapdistance, station_list, FF_ARRAY_ELEMS(station_list));

    if (nb_stations) {
        for(int i = 1; i<nb_stations; i++)
            if (station_list[0]->modulation != modulation ||
                (station_list[i]->modulation == modulation &&
                 fabs(station_list[0]->frequency - freq) > fabs(station_list[i]->frequency - freq)))
                station_list[0] = station_list[i];
        nb_stations = station_list[0]->modulation == modulation;
    }

    if (!nb_stations) {
        station = av_mallocz(sizeof(*station));
        if (!station)
            return AVERROR(ENOMEM);
        station->frequency = freq;
    } else {
        station = station_list[0];
        // We will update the frequency so we need to reinsert
        tree_remove(&sdr->station_root, station, station_cmp, &next);
        station->frequency = station->nb_frequency * station->frequency + freq;
        station->timeout   = 0;
    }
    station->frequency /= ++station->nb_frequency;

    station->detection_per_mix_frequency[histogram_index(sdr, freq)] ++;
    station->modulation   = modulation;
    station->bandwidth    = bandwidth;
    station->bandwidth_p2 = bandwidth_p2;
    station->score        = score;

    tmp = tree_insert(&sdr->station_root, station, station_cmp, &next);
    if (tmp && tmp != station) {
        //This will not happen in real C implementations but floats allow odd things in theory
        av_freep(&station);
    }
    av_freep(&next);

    return 1;
}

static void probe_common(SDRContext *sdr)
{
    for(int i = 0; i < 2*sdr->block_size; i++) {
        sdr->len2block[i] = len2(sdr->block[i]);
    }
}

// simple and dumb implementation for max we could do it faster but it doesnzt matter ATM
static float max_in_range(SDRContext *sdr, unsigned start, unsigned end)
{
    float max = sdr->len2block[start];
    for (; start <= end; start++)
        max = fmax(max,  sdr->len2block[start]);
    return max;
}

static float countbelow(SDRContext *sdr, unsigned start, unsigned end, float level)
{
    int count = 0;
    for (; start <= end; start++)
        count += sdr->len2block[start] < level;
    return count;
}

static double find_peak(SDRContext *sdr, const float *data, int index, int len)
{
    double y[3], b, a;

    if (index == 0) {
        index = 1;
    } else if (index >= len - 1)
        index = len - 2;

    //We use a simple quadratic to find the maximum at higher precission than the available samples
    //ax^2 + bx + c
    //dy/dx = 2ax + b = 0
    y[0] = data[index-1];
    y[1] = data[index];
    y[2] = data[index+1];

    b = (y[2] - y[0]) * 0.5;
    a = (y[0] + y[2]) * 0.5 - y[1];

    //This should not happen
    if (a >= 0.0)
        return INT_MIN;

    return index -0.5 * b / a;
    //TODO theres some simplification possible above but the fm_probe() using this needs to be tuned first so we dont optimize the wrong algorithm
}

static double find_peak_macleod(const SDRContext *sdr, const AVComplexFloat *data, int index, int len, float *phase) {
    AVComplexFloat ref;
    double r0, r1, r2, rd, g;
    double ab[16][2] = {
        {1.525084, 3.376388}, {1.773260, 3.129280}, {1.970200, 2.952300}, {2.122700, 2.825800},
        {2.243600, 2.731620}, {2.341880, 2.658740}, {2.423310, 2.600750}, {2.492110, 2.553360},
        {2.551000, 2.513930}, {2.602300, 2.480400}, {2.646880, 2.451880}, {2.686200, 2.427200},
        {2.721880, 2.405120}, {2.753600, 2.385800}, {2.781800, 2.368900}, {2.808580, 2.352920},
    };

    double a = ab[sdr->kbd_alpha-1][0];
    double b = ab[sdr->kbd_alpha-1][1];

    if (index == 0) {
        index = 1;
    } else if (index >= len - 1)
        index = len - 2;

    /* Baed on Macleod, M.D., "Fast Nearly ML Estimation of the Parameters
     * of Real or Complex Single Tones or Resolved Multiple Tones,"
     * IEEE Trans. Sig. Proc. Vol 46 No 1,
     * January 1998, pp141-148.
     *
     * We use the 3 point estimator without corrections, the corrections
     * provide insignificant performance increase. We add compensation due to
     * the window used, this performs substantially better than the original
     * with a rectangular window.
     */
    ref = data[index];
    r0 = data[index-1].re * ref.re + data[index-1].im * ref.im;
    r1 =           ref.re * ref.re +           ref.im * ref.im;
    r2 = data[index+1].re * ref.re + data[index+1].im * ref.im;
    rd = 2*r1 + r0 + r2;
    if (r2 > r1 || r0 > r1 || rd <=0) // rounding and float numeric issues
        return -1;
    g = (r0-r2) / rd + DBL_MIN;
    g = (sqrt(a*a + 8*g*g)-a)/(b*g);

    if (phase) {
        AVComplexFloat t;
        if (g < 0){
            t.re = ref.re * (1+g) + data[index-1].re * g;
            t.im = ref.im * (1+g) + data[index-1].im * g;
        } else {
            t.re = ref.re * (1-g) - data[index+1].re * g;
            t.im = ref.im * (1-g) - data[index+1].im * g;
        }
        *phase = atan2(t.im, t.re) - M_PI*g;
    }

    return index + g;
}

static int probe_am(SDRContext *sdr)
{
    int i;
    int bandwidth_f = 6000;
    int half_bw_i = bandwidth_f * (int64_t)sdr->block_size / sdr->sdr_sample_rate;
    int border_i = (sdr->sdr_sample_rate - sdr->bandwidth) * sdr->block_size / sdr->sdr_sample_rate;
    double avg = 0;

    if (2*half_bw_i > 2*sdr->block_size)
        return 0;

    for (i = 0; i<2*half_bw_i; i++)
        avg += sdr->len2block[i];

    for (i = half_bw_i; i<2*sdr->block_size - half_bw_i; i++) {
        float mid = sdr->len2block[i];
        double score;
        avg += sdr->len2block[i + half_bw_i];
        score = half_bw_i * mid / (avg - mid);
        avg -= sdr->len2block[i - half_bw_i];

        if (i < border_i || i > 2*sdr->block_size - border_i)
            continue;

        //TODO also check for symmetry in the spectrum
        if (mid > 0 && score > sdr->am_threshold &&
            sdr->len2block[i - 1] <  mid          && sdr->len2block[i + 1] <= mid &&
            sdr->len2block[i - 2] <  mid*AM_MAX23 && sdr->len2block[i + 2] <  mid*AM_MAX23 &&
            sdr->len2block[i - 3] <  mid*AM_MAX23 && sdr->len2block[i + 3] <  mid*AM_MAX23
        ){
            if (max_in_range(sdr, i-half_bw_i, i-4) < mid*AM_MAX4 &&
                max_in_range(sdr, i+4, i+half_bw_i) < mid*AM_MAX4) {
                double peak_i = find_peak_macleod(sdr, sdr->block, i, 2*sdr->block_size, NULL);
                if (peak_i < 0)
                    continue;
                if (fabs(peak_i-i) > 1.0) {
                    av_log(sdr->avfmt, AV_LOG_WARNING, "peak detection failure\n");
                    continue;
                }

                create_candidate_station(sdr, AM, INDEX2F(peak_i), bandwidth_f, 0, score);
            }
        }
    }

    return 0;
}

/**
 *
 * @param data_len      length of the data and len2block arrays
 * @param index         center of search
 * @param search_range  range on both sides to search for carrier
 * @param len           AM signal if any surrounding the carrier
 */
static double find_am_carrier(SDRContext *sdr, const AVComplexFloat *data, int data_len, float *len2block, int index, int search_range, int len)
{
    int i_max, i;
    double mid, score;
    double avg = 0;

    if (index + len + search_range  >= data_len ||
        index - len - search_range  < 0)
        return AVERROR(ERANGE);

    for (i = index - len - search_range; i < index + len + search_range; i++)
        len2block[i] = len2(data[i]);

    i_max = index;
    for (i = index-search_range; i<index+search_range; i++) {
        if (len2block[i] > len2block[i_max])
            i_max = i;
    }
    mid = len2block[i_max];

    for (i = -len; i < len+1; i++)
        avg += len2block[i + index];
    score = len * mid / (avg - mid);
    //find optimal frequency for this block if we have a carrier
    if (score > sdr->am_threshold / 4) {
        double peak_i = find_peak_macleod(sdr, data, i_max, data_len, NULL);
        if (peak_i < 0)
            return peak_i;
        if (fabs(peak_i-i_max) > 1.0)
            return -1;
        return peak_i;
    }
    return -1;
}

/**
 * Demodulate with a carrier that is in the middle of the signal like in AM
 * This will normalize the signal based on the carrier amplitude and subtract the carrier
 */
static void synchronous_am_demodulation(AVComplexFloat *iblock, AVComplexFloat *icarrier, float *window, int len)
{
    for (int i = 0; i<len; i++) {
        AVComplexFloat c = icarrier[i];
        AVComplexFloat s = iblock[i];
        float          w = window[i];
        float        den = w / (c.re*c.re + c.im*c.im);
        av_assert0(c.re*c.re + c.im*c.im > 0);

        iblock[i].re = ( s.im*c.im + s.re*c.re) * den;
        iblock[i].im = ( s.im*c.re - s.re*c.im) * den;
        iblock[i].re -= w;
    }
}

/**
 * Demodulate with a carrier that is half the frequency and reduced amplitude
 * This will not normalize the signal based on the carrier amplitude
 */
static void synchronous_am_demodulation2(AVComplexFloat *iblock, AVComplexFloat *icarrier, float *window, int len)
{
    for (int i = 0; i<len; i++) {
        AVComplexFloat c = icarrier[i];
        AVComplexFloat s = iblock[i];
        AVComplexFloat c2;
        float          w = window[i];
        float        den = w / (c.re*c.re + c.im*c.im);
        av_assert0(c.re*c.re + c.im*c.im > 0);

        c2.re = c.re*c.re - c.im*c.im;
        c2.im = c.re*c.im + c.im*c.re;

        iblock[i].re = ( s.im*c2.im + s.re*c2.re) * den;
        iblock[i].im = ( s.im*c2.re - s.re*c2.im) * den;
    }
}

static int demodulate_am(SDRContext *sdr, int stream_index, AVPacket *pkt)
{
    AVStream *st   = sdr->avfmt->streams[stream_index];
    SDRStream *sst = st->priv_data;
    double freq    = sst->station->frequency;
    int64_t bandwidth = sst->station->bandwidth;
    int index = lrint(F2INDEX(freq));
    int len   = (bandwidth * 2ll * sdr->block_size + sdr->sdr_sample_rate/2) / sdr->sdr_sample_rate;
    float *newbuf;
    float scale;
    int sample_rate = sdr->sdr_sample_rate * (int64_t)sst->block_size / sdr->block_size;
    int ret, i;
    double current_station_i;
    float limits[2] = {-0.0, 0.0};
    float clip = 1.0;
    enum AMMode am_mode = sdr->am_mode;

#define CARRIER_SEARCH 2
    if (index + len + CARRIER_SEARCH>= 2*sdr->block_size ||
        index - len - CARRIER_SEARCH < 0 ||
        2*len + 1 > 2*sst->block_size)
        return AVERROR(ERANGE);

    current_station_i = find_am_carrier(sdr, sdr->block, 2*sdr->block_size, sdr->len2block, index, CARRIER_SEARCH, len);

    if (current_station_i >= 0) {
        av_log(sdr->avfmt, AV_LOG_DEBUG, "adjusting frequency %f, max index: %ld\n", INDEX2F(current_station_i) - freq, lrint(F2INDEX(freq)) - index);
        freq = INDEX2F(current_station_i);
        index = lrint(F2INDEX(freq));
    } else if (sdr->am_fft_ref) {
        // We have no carrier so we cannot decode Synchronously to a carrier
        am_mode = AMEnvelope;
    }

    newbuf = av_malloc(sizeof(*sst->out_buf) * 2 * sst->block_size);
    if (!newbuf)
        return AVERROR(ENOMEM);
#define SEPC 4

    i = 2*len+1;
    memcpy(sst->block, sdr->block + index - len, sizeof(*sst->block) * i);
    memset(sst->block + i, 0, sizeof(*sst->block) * (2 * sst->block_size - i));

    sst->ifft(sst->ifft_ctx, sst->iblock  , sst->block, sizeof(AVComplexFloat));

    if (am_mode == AMEnvelope) {
        double vdotw = 0;
        double wdot = 0; // could be precalculated
        for (i = 0; i<2*sst->block_size; i++) {
            float w = sst->window[i];
            float v = sqrt(len2(sst->iblock[i]));
            sst->iblock[i].re = v;
            sst->iblock[i].im = 0;

            vdotw += w*v;
            wdot += w*w;
        }

        vdotw /= wdot ;
        for (i = 0; i<2*sst->block_size; i++) {
            float w = sst->window[i];
            sst->iblock[i].re -= w*vdotw;
        }

        scale = 0.9/vdotw;
    } else if (sdr->am_fft_ref) {
        // Synchronous demodulation using FFT
        memset(sst->block, 0, sizeof(*sst->block) * i);
        for (i = len-SEPC+1; i<len+SEPC; i++)
            sst->block[i] = sdr->block[index + i - len];
        sst->ifft(sst->ifft_ctx, sst->icarrier, sst->block, sizeof(AVComplexFloat));

        synchronous_am_demodulation(sst->iblock, sst->icarrier, sst->window, 2*sst->block_size);
        scale = 0.9;
    } else {
        // Synchronous demodulation using Macleod based systhesized carrier
        double fcorr = F2INDEX(freq) - index + len;
        double theta = -M_PI*fcorr / sst->block_size;
        AVComplexDouble mdelta = {cos(theta), sin(theta)};
        AVComplexDouble m = {1,0};
        AVComplexDouble dc1 = {0,0};
        AVComplexFloat mm;
        double s2 = 0;
        double dcw = 0;
        float amp, amp2;
        for(i = 0; i<2*sst->block_size; i++) {
            double tmp;
            AVComplexFloat v = sst->iblock[i];
            sst->iblock[i].re = v.re*m.re - v.im*m.im;
            sst->iblock[i].im = v.re*m.im + v.im*m.re;
            tmp  = m.re*mdelta.im + m.im*mdelta.re;
            m.re = m.re*mdelta.re - m.im*mdelta.im;
            m.im = tmp;
            dc1.re += sst->iblock[i].re * sst->window[i];
            dc1.im += sst->iblock[i].im * sst->window[i];
            s2     += len2(sst->iblock[i]);
            dcw    += sst->window[i] * sst->window[i];
        }

        amp = dcw / (dc1.re*dc1.re + dc1.im*dc1.im);
        amp2= dcw / s2;
        amp = FFMIN(amp, amp2 * 0.1);
        if (sst->am_amplitude)
            amp = 0.9*sst->am_amplitude + 0.1*amp;
        sst->am_amplitude = amp;

        mm = (AVComplexFloat){dc1.re * amp, -dc1.im * amp};
        for(i = 0; i<2*sst->block_size; i++) {
            AVComplexFloat v = sst->iblock[i];
            sst->iblock[i].re = v.re*mm.re - v.im*mm.im - sst->window[i];
            sst->iblock[i].im = v.re*mm.im + v.im*mm.re;
        }

        scale = 0.9;
    }

    for(i = 0; i<2*sst->block_size; i++) {
        av_assert0(isfinite(sst->iblock[i].re));
        av_assert0(isfinite(sst->iblock[i].im));
        limits[0] = FFMIN(limits[0], FFMIN(sst->iblock[i].re - sst->iblock[i].im,  sst->iblock[i].re + sst->iblock[i].im));
        limits[1] = FFMAX(limits[1], FFMAX(sst->iblock[i].re - sst->iblock[i].im,  sst->iblock[i].re + sst->iblock[i].im));
    }
    av_assert1(FFMAX(limits[1], -limits[0]) >= 0);
    scale = FFMIN(scale, 0.98 / FFMAX(limits[1], -limits[0]));

    for(i = 0; i<sst->block_size; i++) {
        float m, q;

        m = sst->out_buf[2*i+0] + (sst->iblock[i                  ].re) * sst->window[i                  ] * scale;
        newbuf[2*i+0]           = (sst->iblock[i + sst->block_size].re) * sst->window[i + sst->block_size] * scale;

        switch(am_mode) {
        case AMMidSide:
        case AMLeftRight:
            q = sst->out_buf[2*i+1] +  sst->iblock[i                  ].im * sst->window[i                  ] * scale;
            newbuf[2*i+1]           =  sst->iblock[i + sst->block_size].im * sst->window[i + sst->block_size] * scale;
            switch(am_mode) {
            case AMMidSide:
                q *= 0.5;
                sst->out_buf[2*i+0] = m + q;
                sst->out_buf[2*i+1] = m - q;
                break;
            case AMLeftRight:
                sst->out_buf[2*i+0] = m;
                sst->out_buf[2*i+1] = q;
                break;
            }
            break;

        case AMEnvelope:
        case AMInPhase:
            sst->out_buf[2*i+0] =
            sst->out_buf[2*i+1] = m;
            break;
        }

        if (fabs(sst->out_buf[i]) > clip) {
            av_log(sdr->avfmt, AV_LOG_WARNING, "CLIP %f\n", sst->out_buf[i]);
            clip = fabs(sst->out_buf[i]) * 1.1;
        }
    }

    ret = av_packet_from_data(pkt, (void*)sst->out_buf, sizeof(*sst->out_buf) * 2 * sst->block_size);
    if (ret < 0)
        av_free(sst->out_buf);
    sst->out_buf = newbuf;

    if (st->codecpar->ch_layout.nb_channels != 2 ||
        st->codecpar->sample_rate != sample_rate
    ) {
        av_log(sdr->avfmt, AV_LOG_INFO, "set channel parameters %d %d %d\n", st->codecpar->ch_layout.nb_channels, st->codecpar->sample_rate, sample_rate);
        if (st->codecpar->sample_rate == 0)
            sdr->missing_streams--;
        ff_add_param_change(pkt, 2, 0, sample_rate, 0, 0);
        st->codecpar->sample_rate = sample_rate;
    }

    return ret;
}

static int probe_fm(SDRContext *sdr)
{
    int i;
    int bandwidth_f  = 180*1000;
    int bandwidth_p2 =  38*1000; //phase 2 bandwidth
    int half_bw_i = bandwidth_f * (int64_t)sdr->block_size / sdr->sdr_sample_rate;
    float last_score[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    int border_i = (sdr->sdr_sample_rate - sdr->bandwidth) * sdr->block_size / sdr->sdr_sample_rate;
    double noise_floor = FLT_MAX;

    if (2*half_bw_i > 2*sdr->block_size)
        return 0;

    for (int pass = 0; pass < 2; pass ++) {
        double avg[2] = {0}, tri = 0;
        double mean = 0;
        double center = 0;
        for (i = 0; i<half_bw_i; i++) {
            avg[0] += sdr->len2block[i];
            tri    += i*sdr->len2block[i];
        }
        mean = tri;
        for (; i<2*half_bw_i; i++) {
            avg[1] += sdr->len2block[i];
            tri    += (2*half_bw_i-i)*sdr->len2block[i];
            mean   += i*sdr->len2block[i];
        }

        for(i = half_bw_i; i<2*sdr->block_size - half_bw_i; i++) {
            double b = avg[0] + sdr->len2block[i];
            avg[0] += sdr->len2block[i] - sdr->len2block[i - half_bw_i];
            avg[1] -= sdr->len2block[i] - sdr->len2block[i + half_bw_i];
            b += avg[1];
            tri += avg[1] - avg[0];

            mean += (i+half_bw_i)*sdr->len2block[i+half_bw_i];
            center = mean / b;
            mean -= (i-half_bw_i)*sdr->len2block[i-half_bw_i];

            if (i < border_i || i > 2*sdr->block_size - border_i)
                continue;

            if (pass == 0) {
                noise_floor = FFMIN(noise_floor, tri);
            } else {
                last_score[2] = last_score[1];
                last_score[1] = last_score[0];
                last_score[0] = tri / (noise_floor);

                if (last_score[1] >= last_score[0] &&
                    last_score[1] > last_score[2] &&
                    last_score[1] > sdr->fm_threshold) {
                    double score = last_score[1];

                    float rmax   = max_in_range(sdr, i-half_bw_i/4, i+half_bw_i/4);
                    int lowcount = countbelow(sdr, i-half_bw_i/4, i+half_bw_i/4, rmax / 100);
                    double peak_i;

                    if (lowcount / (half_bw_i*0.5) > 0.99)
                        continue;

                    // as secondary check, we could check that without the center 3 samples we are still having a strong signal FIXME

                    peak_i = find_peak(sdr, last_score, 1, 3) + i - 1;
                    if (peak_i < 0)
                        continue;
                    av_assert0(fabs(peak_i-i) < 2);
                    double f = peak_i * 0.5 * sdr->sdr_sample_rate / sdr->block_size + sdr->block_center_freq - sdr->sdr_sample_rate/2;
                    double f2 = center * 0.5 * sdr->sdr_sample_rate / sdr->block_size + sdr->block_center_freq - sdr->sdr_sample_rate/2;

                    if (fabs(f2 - f) > 1000)
                        continue;
                    create_candidate_station(sdr, FM, f2, bandwidth_f, bandwidth_p2, score);
                }
            }
        }
    }

    return 0;
}

static int demodulate_fm(SDRContext *sdr, int stream_index, AVPacket *pkt)
{
    AVStream *st   = sdr->avfmt->streams[stream_index];
    SDRStream *sst = st->priv_data;
    double freq    = sst->station->frequency;
    int64_t bandwidth = sst->station->bandwidth;
    int index = lrint(F2INDEX(freq));
    int len   = (bandwidth * 2ll * sdr->block_size + sdr->sdr_sample_rate/2) / sdr->sdr_sample_rate;
    float *newbuf;
    float scale;
    int sample_rate    = sdr->sdr_sample_rate * (int64_t)sst->block_size    / sdr->block_size;
    int sample_rate_p2 = sdr->sdr_sample_rate * (int64_t)sst->block_size_p2 / sdr->block_size;
    int ret, i;
    float clip = 1.0;
    int carrier19_i = 2L*sst->block_size*19000 / sample_rate;
    int len17_i     = 2L*sst->block_size*17000 / sample_rate;
    double carrier19_i_exact;
    int W= 5;

    //If only some of the bandwidth is available, just try with less
    int len2 = FFMIN(index, 2*sdr->block_size - index);
    if (len2 < len && len2 > len/2)
        len = len2;

    if (index + len >= 2*sdr->block_size ||
        index - len < 0 ||
        2*len + 1   > 2*sst->block_size)
    return AVERROR(ERANGE);

    newbuf = av_malloc(sizeof(*sst->out_buf) * 2 * sst->block_size);
    if (!newbuf)
        return AVERROR(ENOMEM);

    i = 2*len+1;
    memcpy(sst->block, sdr->block + index, sizeof(*sst->block) * (len + 1));
    memcpy(sst->block + 2 * sst->block_size - len, sdr->block + index - len, sizeof(*sst->block) * len);
    memset(sst->block + len + 1, 0, sizeof(*sst->block) * (2 * sst->block_size - i));

    sst->ifft(sst->ifft_ctx, sst->iblock, sst->block, sizeof(AVComplexFloat));

    for (i = 0; i<2*sst->block_size - 1; i++) {
        AVComplexFloat x = sst->iblock[i];
        AVComplexFloat y = sst->iblock[i+1];
        sst->iblock[i].re = atan2(x.im * y.re - x.re * y.im,
                                  x.re * y.re + x.im * y.im) * sst->window[i];
        sst->iblock[i].im = 0;
    }
    sst->iblock[i].re = 0;
    sst->iblock[i].im = 0;

    av_assert0(sst->block_size_p2 * 2 < sst->block_size);
    //FIXME this only needs to be a RDFT
    //CONSIDER, this and in fact alot can be done with bandpass and lowpass filters instead of FFTs, find out which is better
    //CONSIDER synthesizing the carrier instead of IFFT, we have all parameters for that
    sst->fft_p2(sst->fft_p2_ctx, sst->block, sst->iblock, sizeof(AVComplexFloat));
    // Only the low N/2+1 are used the upper is just a reflection

    carrier19_i_exact = find_am_carrier(sdr, sst->block, 2*sst->block_size, (void*)(sst->block + 1 + sst->block_size), carrier19_i, 10, 10);
    carrier19_i = lrint(carrier19_i_exact);

    if (carrier19_i >= 0) {
        int shift = carrier19_i/2;
        i = sst->block_size;
        memset(sst->block + i, 0, 2*sst->block_size_p2 * sizeof(AVComplexFloat));
        memcpy(sst->block + i + carrier19_i - W - shift , sst->block + carrier19_i - W, sizeof(AVComplexFloat)*(2*W+1));
        sst->ifft_p2(sst->ifft_p2_ctx, sst->icarrier, sst->block + i, sizeof(AVComplexFloat));

        memcpy(sst->block + i + 2*carrier19_i - 2*shift - len17_i, sst->block + 2*carrier19_i - len17_i, sizeof(AVComplexFloat)*2*len17_i);
        apply_deemphasis(sdr, sst->block + i + 2*carrier19_i - 2*shift, sst->block_size_p2, sample_rate_p2, + 1);
        apply_deemphasis(sdr, sst->block + i + 2*carrier19_i - 2*shift, sst->block_size_p2, sample_rate_p2, - 1);
        sst->ifft_p2(sst->ifft_p2_ctx, sst->iside   , sst->block + i, sizeof(AVComplexFloat));
        synchronous_am_demodulation2(sst->iside, sst->icarrier, sst->window_p2, 2*sst->block_size_p2);
    }
    memset(sst->block + len17_i, 0, (2*sst->block_size_p2 - len17_i) * sizeof(AVComplexFloat));
    apply_deemphasis(sdr, sst->block, 2*sst->block_size_p2, sample_rate_p2, + 1);
    sst->ifft_p2(sst->ifft_p2_ctx, sst->iblock  , sst->block, sizeof(AVComplexFloat));
    memset(sst->iblock + 2*sst->block_size_p2, 0 ,(2*sst->block_size -2*sst->block_size_p2) * sizeof(AVComplexFloat));

    scale      = 5 / (M_PI * 2*sst->block_size);
    for(i = 0; i<sst->block_size_p2; i++) {
        float m, q;

        m = sst->out_buf[2*i+0] + (sst->iblock[i                     ].re) * sst->window_p2[i                     ] * scale;
        newbuf[2*i+0]           = (sst->iblock[i + sst->block_size_p2].re) * sst->window_p2[i + sst->block_size_p2] * scale;

        if (carrier19_i >= 0) {
            q = sst->out_buf[2*i+1] +  sst->iside[i                     ].im * sst->window_p2[i                     ] * scale;
            newbuf[2*i+1]           =  sst->iside[i + sst->block_size_p2].im * sst->window_p2[i + sst->block_size_p2] * scale;

            sst->out_buf[2*i+0] = m + q;
            sst->out_buf[2*i+1] = m - q;
        } else {
            sst->out_buf[2*i+0] =
            sst->out_buf[2*i+1] = m;
        }

        if (fabs(sst->out_buf[i]) > clip) {
            av_log(sdr->avfmt, AV_LOG_WARNING, "CLIP %f\n", sst->out_buf[i]);
            clip = fabs(sst->out_buf[i]) * 1.1;
        }
    }

    ret = av_packet_from_data(pkt, (void*)sst->out_buf, sizeof(*sst->out_buf) * 2 * sst->block_size_p2);
    if (ret < 0)
        av_free(sst->out_buf);
    sst->out_buf = newbuf;

    if (st->codecpar->ch_layout.nb_channels != 2 ||
        st->codecpar->sample_rate != sample_rate_p2
    ) {
        av_log(sdr->avfmt, AV_LOG_INFO, "set channel parameters %d %d %d\n", st->codecpar->ch_layout.nb_channels, st->codecpar->sample_rate, sample_rate);
        if (st->codecpar->sample_rate == 0)
            sdr->missing_streams--;
        ff_add_param_change(pkt, 2, 0, sample_rate_p2, 0, 0);
        st->codecpar->sample_rate = sample_rate_p2;
    }

    return ret;
}


BandDescriptor band_descs[] = {
    {"Shortwave band"   , "SW",  6000000,  18000000},
    {"FM broadcast band", "FM", 88000000, 108000000},
};

ModulationDescriptor modulation_descs[] = {
    {"Amplitude Modulation", "AM", AM, AVMEDIA_TYPE_AUDIO, probe_am, demodulate_am},
    {"Frequency Modulation", "FM", FM, AVMEDIA_TYPE_AUDIO, probe_fm, demodulate_fm},
};

int avpriv_sdr_set_freq(SDRContext *sdr, int64_t freq)
{
    freq = av_clip64(freq, sdr->min_center_freq, sdr->max_center_freq);

    if (sdr->set_frequency_callback) {
        int ret = sdr->set_frequency_callback(sdr, freq);
        if (ret < 0)
            return ret;
    }

    sdr->freq = freq;

    return 0;
}

static void free_stream(SDRContext *sdr, int stream_index)
{
    AVFormatContext *s = sdr->avfmt;
    AVStream *st = s->streams[stream_index];
    SDRStream *sst = st->priv_data;

    av_tx_uninit(&sst->ifft_ctx);
    av_tx_uninit(&sst->fft_p2_ctx);
    av_tx_uninit(&sst->ifft_p2_ctx);
    sst->ifft = NULL;
    sst->fft_p2 = NULL;
    sst->ifft_p2 = NULL;
    sst->block_size = 0;

    av_freep(&sst->out_buf);
    av_freep(&sst->block);
    av_freep(&sst->iblock);
    av_freep(&sst->icarrier);
    av_freep(&sst->iside);
    av_freep(&sst->window);
    av_freep(&sst->window_p2);

}

static int setup_stream(SDRContext *sdr, int stream_index, Station *station)
{
    AVFormatContext *s = sdr->avfmt;
    AVStream *st = s->streams[stream_index];
    SDRStream *sst = st->priv_data;
    int ret;

    //For now we expect each station to be only demodulated once, nothing should break though if its done more often
    av_assert0(station->stream == NULL || station->stream == sst);

    if (!station->stream)
        av_log(sdr->avfmt, AV_LOG_INFO, "setup stream_index: %d to Station: %f Mhz\n", stream_index, station->frequency / 1000000);

    if (sst->station)
        sst->station->stream = NULL;

    sst->station = station;
    station->stream = sst;

    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
        free_stream(sdr, stream_index);

        for (sst->block_size = 4; 2ll *sst->station->bandwidth * sdr->block_size / sdr->sdr_sample_rate > sst->block_size; sst->block_size <<= 1)
            ;
        sst->block_size = FFMIN(sdr->block_size,  sst->block_size);

        ret = av_tx_init(&sst->ifft_ctx, &sst->ifft, AV_TX_FLOAT_FFT, 1, 2*sst->block_size, NULL, 0);
        if (ret < 0)
            return ret;

        if (sst->station->bandwidth_p2) {
            //Allocate 2nd stage demodulation fields if needed
            ret = av_tx_init(&sst-> fft_p2_ctx, &sst-> fft_p2, AV_TX_FLOAT_FFT, 0, 2*sst->block_size   , NULL, 0);
            if (ret < 0)
                return ret;

            for (sst->block_size_p2 = 4; 2ll *sst->station->bandwidth_p2 * sdr->block_size / sdr->sdr_sample_rate > sst->block_size_p2; sst->block_size_p2 <<= 1)
                ;
            ret = av_tx_init(&sst->ifft_p2_ctx, &sst->ifft_p2, AV_TX_FLOAT_FFT, 1, 2*sst->block_size_p2, NULL, 0);
            if (ret < 0)
                return ret;

        }

        sst->out_buf   = av_malloc(sizeof(*sst->out_buf)  * 2 * sst->block_size);
        sst->block     = av_malloc(sizeof(*sst-> block)   * 2 * sst->block_size);
        sst->iblock    = av_malloc(sizeof(*sst->iblock)   * 2 * sst->block_size);
        sst->icarrier  = av_malloc(sizeof(*sst->icarrier) * 2 * sst->block_size);
        sst->iside     = av_malloc(sizeof(*sst->iside)    * 2 * sst->block_size_p2);
        sst->window    = av_malloc(sizeof(*sst->window)   * 2 * sst->block_size);
        sst->window_p2 = av_malloc(sizeof(*sst->window_p2)* 2 * sst->block_size_p2);
        if (!sst->out_buf || !sst->block || !sst->iblock || !sst->icarrier || !sst->iside || !sst->window || !sst->window_p2)
            return AVERROR(ENOMEM);

        avpriv_kbd_window_init(sst->window, sdr->kbd_alpha, sst->block_size);
        for(int i = sst->block_size; i < 2 * sst->block_size; i++) {
            sst->window[i] = sst->window[2*sst->block_size - i - 1];
        }

        avpriv_kbd_window_init(sst->window_p2, sdr->kbd_alpha, sst->block_size_p2);
        for(int i = sst->block_size_p2; i < 2 * sst->block_size_p2; i++) {
            sst->window_p2[i] = sst->window_p2[2*sst->block_size_p2 - i - 1];
        }
        sst->am_amplitude = 0;
    }

    return 0;
}

static void inject_block_into_fifo(SDRContext *sdr, AVFifo *fifo, FIFOElement *fifo_element, const char *error_message)
{
    int ret;

    pthread_mutex_lock(&sdr->mutex);
    ret = av_fifo_write(fifo, fifo_element, 1);
    pthread_mutex_unlock(&sdr->mutex);
    if (ret < 0) {
        av_log(sdr->avfmt, AV_LOG_DEBUG, "%s", error_message);
        av_freep(&fifo_element->halfblock);
    }
}

static void flush_fifo(SDRContext *sdr, AVFifo *fifo)
{
    FIFOElement fifo_element;

    pthread_mutex_lock(&sdr->mutex);
    while(fifo) {
        int ret = av_fifo_read(fifo, &fifo_element, 1);
        if (ret < 0)
            break;
        av_freep(&fifo_element.halfblock);
    }
    pthread_mutex_unlock(&sdr->mutex);
}

static int sdrfile_read_callback(SDRContext *sdr, FIFOElement *fifo_element, int remaining)
{
    AVFormatContext *avfmt = sdr->avfmt;
    int64_t size;
    void *buffer = (uint8_t*)fifo_element->halfblock + (sdr->block_size - remaining) * sdr->sample_size;
    int ret;

    if (!sdr->remaining_file_block_size) {
        int block_size;
        avio_skip(avfmt->pb, 16 + 4);        //FFSDR00XintYYBE
        block_size = avio_rb32(avfmt->pb);
        fifo_element->center_frequency = av_int2double(avio_rb64(avfmt->pb));
        avio_rb64(avfmt->pb); //pts
        avio_skip(avfmt->pb, sdr->fileheader_size - 40);
        sdr->remaining_file_block_size = block_size;
    }
    pthread_mutex_lock(&sdr->mutex);

    //We have no FIFO API to check how much we can write
    size = sdr->sdr_sample_rate / sdr->block_size - av_fifo_can_read(sdr->full_block_fifo);
    pthread_mutex_unlock(&sdr->mutex);
    if (size <= 0) {
        av_usleep(10*1000);
        return AVERROR(EAGAIN);
    }
    size = FFMIN(remaining, sdr->remaining_file_block_size) * sdr->sample_size;
    ret = avio_read(avfmt->pb, buffer, size);
    if (ret == AVERROR_EOF || (ret > 0 && ret % sdr->sample_size)) {
        fifo_element->center_frequency = AVERROR_EOF;
        ret = remaining * sdr->sample_size;
        sdr->remaining_file_block_size = remaining;
        avio_seek(avfmt->pb, SEEK_SET, 0);
        av_log(avfmt, AV_LOG_INFO, "EOF\n");
    } else if (ret == AVERROR(EAGAIN)) {
        av_log(avfmt, AV_LOG_DEBUG, "read EAGAIN\n");
        return AVERROR(EAGAIN);
    } else if (ret < 0) {
        av_log(avfmt, AV_LOG_ERROR, "read Failed with (%d)\n", ret);
        return AVERROR(EAGAIN);
    }

    ret /= sdr->sample_size;
    sdr->remaining_file_block_size -= ret;
    return ret;
}

static int snap2station(SDRContext *sdr, int *seek_direction) {
    AVFormatContext *avfmt = sdr->avfmt;
    AVStream *st = avfmt->streams[sdr->single_ch_audio_st_index];
    SDRStream *sst = st->priv_data;
    double current_freq;
    double best_distance = INT64_MAX;
    Station *best_station = NULL;
    Station *station_list[1000];
    int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.5, station_list, FF_ARRAY_ELEMS(station_list));

    if (sst->station) {
        current_freq = sst->station->frequency;
    } else if (sdr->station_freq) {
        current_freq = sdr->station_freq;
    } else
        current_freq = sdr->block_center_freq;

    for(int i = 0; i<nb_stations; i++) {
        Station *station = station_list[i];
        double distance = station->frequency - current_freq;

        if (!station->in_station_list)
            continue;

        if (distance * *seek_direction < 0 || station == sst->station)
            continue;
        distance = fabs(distance);
        if (distance < best_distance) {
            best_distance = distance;
            best_station = station;
        }
    }
    av_assert0(!best_station || best_station != sst->station);

    if (best_station) {
        int ret = setup_stream(sdr, sdr->single_ch_audio_st_index, best_station);
        if (ret < 0) {
            av_log(avfmt, AV_LOG_DEBUG, "setup_stream failed\n");
            return ret;
        }

        pthread_mutex_lock(&sdr->mutex);
        *seek_direction     =
        sdr->seek_direction = 0;
        sdr->wanted_freq = lrint(best_station->frequency + 213*1000); // We target a bit off teh exact frequency to avoid artifacts
        //200*1000 had artifacts

        av_log(avfmt, AV_LOG_DEBUG, "request f = %"PRId64"\n", sdr->wanted_freq);
        pthread_mutex_unlock(&sdr->mutex);
        return 1;
    }

    return 0;
}

/**
 * move frequency in the delta direction within a band
 * This can be called from the thread so it must only access runtime constants from sdr->
 */
static int64_t snap2band(SDRContext *sdr, int64_t wanted_freq, int64_t delta) {
    int64_t best_distance = INT64_MAX;
    int64_t best_frequency = wanted_freq;
    int64_t wrap_frequency = delta > 0 ? 0 : INT64_MAX;

    for(int i = 0; i<FF_ARRAY_ELEMS(band_descs); i++) {
        int64_t distance, new_freq;
        BandDescriptor *desc = &band_descs[i];
        int64_t min_center_freq = av_clip64(desc->freq_min, sdr->min_freq, sdr->max_freq);
        int64_t max_center_freq = av_clip64(desc->freq_max, sdr->min_freq, sdr->max_freq);

        // Is the band within the frequency range ?
        if (min_center_freq == max_center_freq)
            continue;

        if (max_center_freq - min_center_freq <= sdr->bandwidth) {
            min_center_freq =
            max_center_freq = (min_center_freq + max_center_freq) / 2;
        } else {
            min_center_freq += sdr->bandwidth / 2;
            max_center_freq -= sdr->bandwidth / 2;
        }

        // Is the band in the direction we want to seek to ?
        if (FFSIGN(min_center_freq - wanted_freq) == FFSIGN(max_center_freq - wanted_freq))
            if (FFSIGN(delta) != FFSIGN(min_center_freq - wanted_freq))
                continue;

        new_freq = av_clip64(wanted_freq + delta, min_center_freq, max_center_freq);
        distance = FFABS(new_freq - (wanted_freq + delta));
        if (distance < best_distance && new_freq != wanted_freq) {
            best_distance = distance;
            best_frequency = new_freq;
        }
    }

    //If we cant move (that is hit the end) then wrap around unless we already wraped
    if (best_distance == INT64_MAX && wanted_freq != wrap_frequency)
        return snap2band(sdr, wrap_frequency, delta);

    return best_frequency;
}

/**
 * Grab data from soapy and put it in a bigger buffer.
 * This thread would not be needed if libsoapy internal buffering was not restricted
 * to a few milli seconds. libavformat cannot guarntee that it will get called from the user
 * application every 10-20ms or something like that so we need a thread to pull data from soapy
 * and put it in a larger buffer that we can then read from at the rate the code is called
 * by the user
 */
static void *soapy_needs_bigger_buffers_worker(SDRContext *sdr)
{
    AVFormatContext *avfmt = sdr->avfmt;
    unsigned block_counter = 0;

    sdr->remaining_file_block_size = 0;

    ff_thread_setname("sdrdemux - soapy rx");

    while(!atomic_load(&sdr->close_requested)) {
        FIFOElement fifo_element;
        int remaining, ret;
        int empty_blocks, full_blocks;

        //i wish av_fifo was thread safe
        pthread_mutex_lock(&sdr->mutex);
        ret = av_fifo_read(sdr->empty_block_fifo, &fifo_element, 1);
        empty_blocks = av_fifo_can_read(sdr->empty_block_fifo);
        full_blocks = av_fifo_can_read(sdr->full_block_fifo);
        pthread_mutex_unlock(&sdr->mutex);

        if (ret < 0) {
            av_log(avfmt, AV_LOG_WARNING, "Allocating new block due to lack of space full:%d empty:%d\n", full_blocks, empty_blocks);
            fifo_element.halfblock = av_malloc(sdr->block_size * sdr->sample_size);
            if (!fifo_element.halfblock) {
                av_log(avfmt, AV_LOG_ERROR, "Allocation failed, waiting for free space\n");
                // we wait 10ms here, tests have shown soapy to loose data when it is not serviced for 40-50ms
                av_usleep(10*1000);
                continue;
            }
        }

        block_counter ++;
        pthread_mutex_lock(&sdr->mutex);
        // we try to get 2 clean blocks after windowing, to improve chances scanning doesnt miss too much
        // First block after parameter change is not reliable, we do not assign it any frequency
        // 2 blocks are needed with windowing to get a clean FFT output
        // Thus > 3 is the minimum for the next frequency update if we want to do something reliable with the data
        if (sdr->seek_direction && block_counter > 5) {
            sdr->wanted_freq = snap2band(sdr, sdr->wanted_freq, sdr->seek_direction*sdr->bandwidth*0.5);
        }
        if (sdr->wanted_freq != sdr->freq) {
            //We could use a seperate MUTEX for the FIFO and for soapy
            avpriv_sdr_set_freq(sdr, sdr->wanted_freq);
            //This shouldnt really cause any problem if we just continue on error except that we continue returning data with the previous target frequency range
            //And theres not much else we can do, an error message was already printed by avpriv_sdr_set_freq() in that case
            block_counter = 0; // we just changed the frequency, do not trust the next blocks content
        }
        pthread_mutex_unlock(&sdr->mutex);

        fifo_element.center_frequency = block_counter > 0 ? sdr->freq : 0;

        remaining = sdr->block_size;
        while (remaining && !atomic_load(&sdr->close_requested)) {
            ret = sdr->read_callback(sdr, &fifo_element, remaining);
            if (ret == AVERROR(EAGAIN))
                continue;

            av_assert0(ret <= remaining);
            remaining -= ret;
        }

        inject_block_into_fifo(sdr, sdr->full_block_fifo, &fifo_element, "block fifo overflow, discarding block\n");
    }
    av_assert0(atomic_load(&sdr->close_requested) == 1);

    return NULL;
}

int avpriv_sdr_common_init(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;
    AVStream *st;
    SDRStream *sst;
    int ret;
    float scale = 1.0 / sdr->sample_scale;

    sdr->avfmt = s;

    if (sdr->width>1 && sdr->height>1) {
        /* video stream */
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        sst = av_mallocz(sizeof(SDRStream));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
        st->codecpar->codec_tag = 0;  /* no fourcc */
        st->codecpar->format = AV_PIX_FMT_BGRA;
        st->codecpar->width  = sdr->width;
        st->codecpar->height = sdr->height;
        avpriv_set_pts_info(st, 64, 1, (48000/128) << FREQ_BITS);
        sdr->waterfall_st_index = st->index;
    } else
        sdr->waterfall_st_index = -1;

    if (sdr->mode == SingleStationMode) {
        /* audio stream */
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        sst = av_mallocz(sizeof(SDRStream));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_tag = 0; /* no fourcc */
        st->codecpar->codec_id = HAVE_BIGENDIAN ? AV_CODEC_ID_PCM_F32BE : AV_CODEC_ID_PCM_F32LE;
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
        st->codecpar->sample_rate = 0; //will be set later
        avpriv_set_pts_info(st, 64, 1, (48000/128) << FREQ_BITS);
        sdr->single_ch_audio_st_index = st->index;
        sdr->missing_streams++;
    }

    sdr->min_center_freq = sdr->min_freq + sdr->sdr_sample_rate / 2;
    sdr->max_center_freq = sdr->max_freq + sdr->sdr_sample_rate / 2;


    if(!sdr->block_size) {
        // The user has not set a block_size, we choose one automatically based on the sample rate
        for(sdr->block_size = 1; 25*sdr->block_size < sdr->sdr_sample_rate; sdr->block_size <<=1)
            ;
    } else if(sdr->block_size & (sdr->block_size - 1)) {
        av_log(s, AV_LOG_ERROR, "Block size must be a power of 2\n");
        return AVERROR(EINVAL);
    }
    av_log(s, AV_LOG_INFO, "Block size %d\n", sdr->block_size);


    sdr->windowed_block = av_malloc(sizeof(*sdr->windowed_block) * 2 * sdr->block_size);
    sdr->block     = av_malloc(sizeof(*sdr->block    ) * 2 * sdr->block_size);
    sdr->len2block = av_malloc(sizeof(*sdr->len2block) * 2 * sdr->block_size);
    sdr->window    = av_malloc(sizeof(*sdr->window   ) * 2 * sdr->block_size);
    if (!sdr->windowed_block || !sdr->len2block || !sdr->block || !sdr->window)
        return AVERROR(ENOMEM);

    ret = av_tx_init(&sdr->fft_ctx, &sdr->fft, AV_TX_FLOAT_FFT, 0, 2*sdr->block_size, NULL, 0);
    if (ret < 0)
        return ret;

    avpriv_kbd_window_init(sdr->window, sdr->kbd_alpha, sdr->block_size);

    for(int i = sdr->block_size; i < 2 * sdr->block_size; i++) {
        sdr->window[i] = sdr->window[2*sdr->block_size - i - 1];
    }
    for (int i = 0; i < 2 * sdr->block_size; i++)
        sdr->window[i] *= ((i&1) ? 1:-1) * scale;

    if (sdr->waterfall_st_index >= 0) {
        AVStream *st = s->streams[sdr->waterfall_st_index];
        SDRStream *sst = st->priv_data;
        av_assert0(st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO);
        sst->frame_size   = sdr->width * sdr->height * 4;
        sst->frame_buffer = av_mallocz(sst->frame_size * 2);
        if (!sst->frame_buffer)
            return AVERROR(ENOMEM);
        for(int i = 3; i<sst->frame_size * 2; i+=4)
            sst->frame_buffer[i] = 255;
    }

    sdr->pts = 0;

    sdr->empty_block_fifo = av_fifo_alloc2(1, sizeof(FIFOElement), AV_FIFO_FLAG_AUTO_GROW);
    sdr-> full_block_fifo = av_fifo_alloc2(1, sizeof(FIFOElement), AV_FIFO_FLAG_AUTO_GROW);
    if (!sdr->empty_block_fifo || !sdr-> full_block_fifo)
        return AVERROR(ENOMEM);
    //Limit fifo to 1second to avoid OOM
    av_fifo_auto_grow_limit(sdr->empty_block_fifo, sdr->sdr_sample_rate / sdr->block_size);
    av_fifo_auto_grow_limit(sdr-> full_block_fifo, sdr->sdr_sample_rate / sdr->block_size);

    atomic_init(&sdr->close_requested, 0);
    ret = pthread_mutex_init(&sdr->mutex, NULL);
    if (ret) {
        av_log(s, AV_LOG_ERROR, "pthread_mutex_init failed: %s\n", strerror(ret));
        return AVERROR(ret);
    }
    sdr->thread_started = 1;

    ret = pthread_create(&sdr->hw_thread, NULL, (void*)soapy_needs_bigger_buffers_worker, sdr);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR, "pthread_create failed : %s\n", strerror(ret));
        pthread_mutex_destroy(&sdr->mutex);
        return AVERROR(ret);
    }
    sdr->thread_started = 2;

    if(sdr->dump_url)  {
        ret = avio_open2(&sdr->dump_avio, sdr->dump_url, AVIO_FLAG_WRITE, NULL, NULL);
        if (ret < 0) {
            fprintf(stderr, "Unable to open %s\n", sdr->dump_url);
            return ret;
        }
    }

    return 0;
}

/**
 * This matches sdrindev_initial_hw_setup() but using a file and no hardware
 */
static int sdrfile_initial_setup(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;

    int version, ret, bits;

    avio_skip(s->pb, 5);        //FFSDR
    version = avio_rb24(s->pb); //000
    avio_skip(s->pb, 3);        //int
    bits = avio_rb16(s->pb);
    if (bits == AV_RB16("16")) {
        sdr->sample_size = 4;
        sdr->sample_scale = 32768;
    } else if (bits == AV_RB16("08")) {
        sdr->sample_size = 2;
        sdr->sample_scale = 128;
    } else
        return AVERROR_INVALIDDATA;

    avio_skip(s->pb, 3);        //BE
    sdr->sdr_sample_rate = avio_rb32(s->pb);
                           avio_rb32(s->pb); //block_size
    sdr->wanted_freq     = av_int2double(avio_rb64(s->pb));
    sdr->pts             = avio_rb64(s->pb);
    if (version > AV_RB24("000")) {
        sdr->bandwidth       = avio_rb32(s->pb);
        sdr->fileheader_size = avio_rb32(s->pb);
    } else {
        sdr->bandwidth       = sdr->sdr_sample_rate;
        sdr->fileheader_size = 40;
    }
    if (sdr->bandwidth >= sdr->sdr_sample_rate)
        sdr->bandwidth = sdr->sdr_sample_rate * 4 / 5;

    //After reading the first packet header we return to the begin so the packet can be read whole
    avio_seek(s->pb, 0, SEEK_SET);

    ret = avpriv_sdr_set_freq(sdr, sdr->wanted_freq);
    if (ret < 0)
        return ret;

    sdr->read_callback = sdrfile_read_callback;

    return avpriv_sdr_common_init(s);
}

static inline void draw_point_component(uint8_t *frame_buffer, ptrdiff_t stride, int x, int y, int r, int g, int b, int w, int h)
{
    uint8_t *p;

    if (x<0 || y<0 || x>=w || y>=h)
        return;
    p = frame_buffer + 4*x + stride*y;

    p[0] = av_clip_uint8(p[0] + (b>>16));
    p[1] = av_clip_uint8(p[1] + (g>>16));
    p[2] = av_clip_uint8(p[2] + (r>>16));
}

// Draw a point with subpixel precission, (it looked bad otherwise)
static void draw_point(uint8_t *frame_buffer, ptrdiff_t stride, int x, int y, int r, int g, int b, int w, int h)
{
    int px = x>>8;
    int py = y>>8;
    int sx = x&255;
    int sy = y&255;
    int s;

    s = (256 - sx) * (256 - sy);
    draw_point_component(frame_buffer, stride, px  , py  , r*s, g*s, b*s, w, h);
    s = sx * (256 - sy);
    draw_point_component(frame_buffer, stride, px+1, py  , r*s, g*s, b*s, w, h);
    s = (256 - sx) * sy;
    draw_point_component(frame_buffer, stride, px  , py+1, r*s, g*s, b*s, w, h);
    s = sx * sy;
    draw_point_component(frame_buffer, stride, px+1, py+1, r*s, g*s, b*s, w, h);
}

static void draw_char(uint8_t *frame_buffer, ptrdiff_t stride, char ch, int x0, int y0, int xd, int yd, int r, int g, int b, int w, int h)
{
    for(int y = 0; y < 16; y++) {
        int mask = avpriv_vga16_font[16*ch + y];
        for(int x = 0; x < 8; x++) {
            if (mask&0x80)
                draw_point(frame_buffer, stride, x0 + xd*x - yd*y, y0 + yd*x + xd*y, r, g, b, w, h);
            mask<<=1;
        }
    }
}

static void draw_string(uint8_t *frame_buffer, ptrdiff_t stride, char *str, int x0, int y0, int xd, int yd, int r, int g, int b, int w, int h)
{
    while(*str) {
        draw_char(frame_buffer, stride, *str++, x0, y0, xd, yd, r, g, b, w, h);
        x0 += xd*9;
        y0 += yd*9;
    }
}

static int vissualization(SDRContext *sdr, AVStream *st, AVPacket *pkt)
{
    SDRStream *sst = st->priv_data;
    int w = st->codecpar->width;
    int h = st->codecpar->height;
    int h2 = FFMIN(64, h / 4);
    int frame_index = av_rescale(sdr->pts,        sdr->fps.num, sdr->fps.den * TIMEBASE);
    int  last_index = av_rescale(sdr->last_pts,   sdr->fps.num, sdr->fps.den * TIMEBASE);
    int skip = frame_index == last_index || sdr->missing_streams;
    av_assert0(sdr->missing_streams >= 0);

    for(int x= 0; x<w; x++) {
        int color;
        int idx = 4*(x + sst->frame_buffer_line*w);
        int bindex  =  x    * 2ll * sdr->block_size / w;
        int bindex2 = (x+1) * 2ll * sdr->block_size / w;
        float a = 0;
        av_assert0(bindex2 <= 2 * sdr->block_size);
        for (int i = bindex; i < bindex2; i++) {
            AVComplexFloat sample = sdr->block[i];
            a += len2(sample);
        }
        color = lrintf(log(a)*8 + 32);

        sst->frame_buffer[idx + 0] = color;
        sst->frame_buffer[idx + 1] = color;
        sst->frame_buffer[idx + 2] = color;
        sst->frame_buffer[idx + 3] = 255;
    }

    // Display locations of all vissible stations
//     for(int station_index = 0; station_index<sdr->nb_stations; station_index++) {
//         Station *s = sdr->station[station_index];
//         double f = s->frequency;
// //                     int bw = s->bandwidth;
// //                     int xleft = 256*((f-bw) - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
// //                     int xright= 256*((f+bw) - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
//         int xmid  = 256*( f     - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
//         int g = s->modulation == AM ? 50 : 0;
//         int b = s->modulation == AM ? 0 : 70;
//         int r = s->stream ? 50 : 0;
//
//         draw_point(sst->frame_buffer, 4*w, xmid, 256*(sst->frame_buffer_line+1), r, g, b, w, h);
//     }

    if (!skip) {
        int ret = av_new_packet(pkt, sst->frame_size);
        if (ret < 0)
            return ret;

        for(int y= 0; y<h2; y++) {
            for(int x= 0; x<w; x++) {
                int color;
                int idx = x + y*w;
                int idx_t = (idx / h2) + (idx % h2)*w;
                int bindex  =  idx    * 2ll * sdr->block_size / (w * h2);
                int bindex2 = (idx+1) * 2ll * sdr->block_size / (w * h2);
                float a = 0;
                av_assert0(bindex2 <= 2 * sdr->block_size);
                for (int i = bindex; i < bindex2; i++) {
                    AVComplexFloat sample = sdr->block[i];
                    a += len2(sample);
                }
                color = lrintf(log(a)*9 + 64);

                idx_t *= 4;

                pkt->data[idx_t+0] = color;
                pkt->data[idx_t+1] = color;
                pkt->data[idx_t+2] = color;
                pkt->data[idx_t+3] = 255;
            }
        }

        for (int y= h2; y<h; y++)
            memcpy(pkt->data + 4*y*w, sst->frame_buffer + 4*(y + sst->frame_buffer_line - h2)*w, 4*w);

        Station *station_list[1000];
        int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.6, station_list, FF_ARRAY_ELEMS(station_list));
        for(int station_index = 0; station_index<nb_stations; station_index++) {
            Station *s = station_list[station_index];
            double f = s->frequency;
            int xmid  = 256*( f     - sdr->block_center_freq + sdr->sdr_sample_rate/2) * w / sdr->sdr_sample_rate;
            char text[20];
            int color = s->stream ? 64 : 32;
            int size = s->stream ? 181 : 128;
            int xd = size, yd = size;

            if (!s->in_station_list)
                continue;

            snprintf(text, sizeof(text), "%s %f Mhz",
                     modulation_descs[s->modulation].shortname,
                     f/1000000);
            draw_string(pkt->data, 4*w, text, xmid + 8*yd, 320*h2, xd, yd, color, color, color, w, h);
        }
    }

    if (!sst->frame_buffer_line) {
        memcpy(sst->frame_buffer + sst->frame_size, sst->frame_buffer, sst->frame_size);
        sst->frame_buffer_line = h-1;
    } else
        sst->frame_buffer_line--;

//TODO
//                 draw RDS*
    return skip;
}

int avpriv_sdr_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SDRContext *sdr = s->priv_data;
    int ret, i, full_blocks, seek_direction;
    FIFOElement fifo_element[2];

process_next_block:

    for (int stream_index = 0; stream_index < s->nb_streams; stream_index++) {
        AVStream *st = s->streams[stream_index];
        SDRStream *sst = st->priv_data;

        if (sst->processing_index) {
            int skip = 1;
            if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                skip = vissualization(sdr, st, pkt);
                if (skip < 0)
                    return skip;
            } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
                if (sst->station) {
                    skip = 0;
                    ret = modulation_descs[ sst->station->modulation ].demodulate(sdr, stream_index, pkt);
                    if (ret < 0) {
                        av_log(s, AV_LOG_ERROR, "demodulation failed ret = %d\n", ret);
                    }
                }
            } else
                av_assert0(0);
            sst->processing_index = 0;
            if (pkt && !skip) {
                pkt->stream_index = stream_index;
                pkt->dts = (sdr->pts & (-1<<FREQ_BITS));
                if (sst->station)
                   pkt->dts += lrint(sst->station->frequency/1000);
                pkt->pts = pkt->dts;

                return 0;
            }
        }
    }

    pthread_mutex_lock(&sdr->mutex);
    full_blocks = av_fifo_can_read(sdr->full_block_fifo) - 1;
    ret = av_fifo_peek(sdr->full_block_fifo, &fifo_element, 2, 0);
    if (ret >= 0)
        av_fifo_drain2(sdr->full_block_fifo, 1);
    seek_direction = sdr->seek_direction; //This doesnt need a mutex here at all but tools might complain
    pthread_mutex_unlock(&sdr->mutex);

    if (ret < 0) {
        av_log(s, AV_LOG_DEBUG, "EAGAIN on not enough data\n");
        return AVERROR(EAGAIN);
    }
    if (fifo_element[0].center_frequency == AVERROR_EOF || fifo_element[1].center_frequency == AVERROR_EOF)
        return AVERROR_EOF;
    if (fifo_element[0].center_frequency != fifo_element[1].center_frequency) {
        av_log(s, AV_LOG_DEBUG, "Mismatching frequency blocks\n");
//         fifo_element[0].center_frequency = 0;
//         inject_block_into_fifo(sdr, sdr->empty_block_fifo, &fifo_element, "Cannot pass next buffer, freeing it\n");
        //Its simpler to just continue than to discard this, but we must make sure that on seeking we have matching blocks for each block we want to scan
        sdr->block_center_freq = 0;
        sdr->skip_probe = 1; // we want to probe the next block as it will have new frequencies
    } else
        sdr->block_center_freq = fifo_element[0].center_frequency;

    if (sdr->dump_avio) {
        uint8_t header[48] = "FFSDR001int16BE";
        uint8_t *tmp = (void*)sdr->windowed_block; //We use an unused array as temporary here

        if (sdr->sample_size == 2)
            memcpy(header + 11, "08", 2);

        AV_WB32(header+16, sdr->sdr_sample_rate);
        AV_WB32(header+20, sdr->block_size);
        AV_WB64(header+24, av_double2int(fifo_element[0].center_frequency));
        AV_WB64(header+32, sdr->pts);
        AV_WB32(header+40, sdr->bandwidth);
        AV_WB32(header+44, sizeof(header));

        avio_write(sdr->dump_avio, header, sizeof(header));

        if (sdr->sample_size == 2) {
            avio_write(sdr->dump_avio, fifo_element[0].halfblock, sdr->block_size * sdr->sample_size);
        } else if (sdr->sample_size == 4){
            const int16_t *halfblock = fifo_element[0].halfblock;
            for(int i=0; i<sdr->block_size; i++) {
                AV_WB16(tmp + 4*i + 0, halfblock[2*i+0]);
                AV_WB16(tmp + 4*i + 2, halfblock[2*i+1]);
            }
            avio_write(sdr->dump_avio, tmp, sdr->block_size * sdr->sample_size);
        } else {
            //We convert to 16bit to safe space, i have no hardware which can do more than 14bit
            const AVComplexFloat *halfblock = fifo_element[0].halfblock;
            for(int i=0; i<sdr->block_size; i++) {
                int re = lrint(halfblock[i].re * 32768);
                int im = lrint(halfblock[i].im * 32768);
                AV_WB16(tmp + 4*i + 0, re);
                AV_WB16(tmp + 4*i + 2, im);
            }
            avio_write(sdr->dump_avio, tmp, sdr->block_size * 4);
        }

    }
//TODO merge scale into window
    if (sdr->sample_size == 2) {
        const int8_t *halfblock0 = fifo_element[0].halfblock;
        const int8_t *halfblock1 = fifo_element[1].halfblock;
        if (sdr->rtlsdr_fixes>0) {
            if (!sdr->rtlsdr_dc_offset || !sdr->block_center_freq) {
                int sum = 0;
                for (i = 0; i<2*sdr->block_size; i++)
                    sum += halfblock0[i]
                        +  halfblock1[i];
                sdr->rtlsdr_dc_offset = -sum / (4.0*sdr->block_size);
                av_log(s, AV_LOG_DEBUG, "Compensating DC offset %f (this should be around -0.6)\n", sdr->rtlsdr_dc_offset);
            }
            for (i = 0; i<sdr->block_size; i++) {
                sdr->windowed_block[i].re = (halfblock0[2*i+0] + sdr->rtlsdr_dc_offset) * sdr->window[i];
                sdr->windowed_block[i].im = (halfblock0[2*i+1] + sdr->rtlsdr_dc_offset) * sdr->window[i];
            }
            for (i = sdr->block_size; i<2*sdr->block_size; i++) {
                sdr->windowed_block[i].re = (halfblock1[2*(i - sdr->block_size)+0] + sdr->rtlsdr_dc_offset) * sdr->window[i];
                sdr->windowed_block[i].im = (halfblock1[2*(i - sdr->block_size)+1] + sdr->rtlsdr_dc_offset) * sdr->window[i];
            }
        } else {
            for (i = 0; i<sdr->block_size; i++) {
                sdr->windowed_block[i].re = halfblock0[2*i+0] * sdr->window[i];
                sdr->windowed_block[i].im = halfblock0[2*i+1] * sdr->window[i];
            }
            for (i = sdr->block_size; i<2*sdr->block_size; i++) {
                sdr->windowed_block[i].re = halfblock1[2*(i - sdr->block_size)+0] * sdr->window[i];
                sdr->windowed_block[i].im = halfblock1[2*(i - sdr->block_size)+1] * sdr->window[i];
            }
        }
    } else if (sdr->sample_size == 4){
        const int16_t *halfblock0 = fifo_element[0].halfblock;
        const int16_t *halfblock1 = fifo_element[1].halfblock;
        for (i = 0; i<sdr->block_size; i++) {
            sdr->windowed_block[i].re = halfblock0[2*i+0] * sdr->window[i];
            sdr->windowed_block[i].im = halfblock0[2*i+1] * sdr->window[i];
        }
        for (i = sdr->block_size; i<2*sdr->block_size; i++) {
            sdr->windowed_block[i].re = halfblock1[2*(i - sdr->block_size)+0] * sdr->window[i];
            sdr->windowed_block[i].im = halfblock1[2*(i - sdr->block_size)+1] * sdr->window[i];
        }
    } else {
        const AVComplexFloat *halfblock0 = fifo_element[0].halfblock;
        const AVComplexFloat *halfblock1 = fifo_element[1].halfblock;
        for (i = 0; i<sdr->block_size; i++) {
            sdr->windowed_block[i].re = halfblock0[i].re * sdr->window[i];
            sdr->windowed_block[i].im = halfblock0[i].im * sdr->window[i];
        }
        for (i = sdr->block_size; i<2*sdr->block_size; i++) {
            sdr->windowed_block[i].re = halfblock1[i - sdr->block_size].re * sdr->window[i];
            sdr->windowed_block[i].im = halfblock1[i - sdr->block_size].im * sdr->window[i];
        }
    }

    inject_block_into_fifo(sdr, sdr->empty_block_fifo, &fifo_element[0], "Cannot pass next buffer, freeing it\n");
#ifdef SYN_TEST //synthetic test signal
    static int64_t synp=0;
    AVLFG avlfg;
    if(!synp)
        av_lfg_init(&avlfg, 0);
    for(int i = 0; i<2*sdr->block_size; i++) {
        double f = F2INDEX(7123456.78901234567);
        int64_t synp2 = synp % (40*sdr->block_size);
        double fsig0 = 0.00002;
        double fsig2= 123;

        if (!i)
            av_log(0,0, "i= %f %f\n", f, INDEX2F(f));
        double noise[2] = {0,0};
        double sig0 = 1.0 + 0.25*sin(synp2*fsig0*synp2*M_PI / sdr->block_size);
        double sig2 = 0.1 + 0.03*cos(synp*fsig2*M_PI / sdr->block_size);
        if (i & 256)
            av_bmg_get(&avlfg, noise);
        sdr->windowed_block[i].re = (noise[0]*0.0000001 + 0.00001*sig0*sin(synp*M_PI*(f)/sdr->block_size)
                                      + 0.00001*sig2*cos(synp*M_PI*(f)/sdr->block_size)
                                    ) * fabs(sdr->window[i]);
        sdr->windowed_block[i].im = noise[1]*0.0000001 * sdr->window[i];
        synp++;
    }
    synp -= sdr->block_size;
#endif


    sdr->fft(sdr->fft_ctx, sdr->block, sdr->windowed_block, sizeof(AVComplexFloat));
    // windowed_block is unused now, we can fill it with the next blocks data

    if (sdr->block_center_freq) {
        if (sdr->skip_probe-- <= 0) {
            //Probing takes a bit of time, lets not do it every time
            sdr->skip_probe = 5;
            probe_common(sdr);

            for(int i = 0; i < FF_ARRAY_ELEMS(modulation_descs); i++) {
                ModulationDescriptor *md = &modulation_descs[i];
                md->probe(sdr);
                av_assert0(i == md->modulation);
            }

            decay_stations(sdr);
            create_stations(sdr);
        }

        if (sdr->mode == SingleStationMode) {
            AVStream *st = s->streams[sdr->single_ch_audio_st_index];
            SDRStream *sst = st->priv_data;

            if (!sst->station || seek_direction) {
                ret = snap2station(sdr, &seek_direction);
                if (ret < 0)
                    return ret;
            }
        } else {
            av_assert0(sdr->mode == AllStationMode);
            Station *station_list[1000];
            int nb_stations = find_stations(sdr, sdr->block_center_freq, sdr->sdr_sample_rate*0.5, station_list, FF_ARRAY_ELEMS(station_list));
            for(int i = 0; i<nb_stations; i++) {
                Station *station = station_list[i];
                if (!station->stream) {
                    /* audio stream */
                    AVStream *st = avformat_new_stream(s, NULL);
                    SDRStream *sst;
                    if (!st)
                        return AVERROR(ENOMEM);
                    sst = av_mallocz(sizeof(*sst));
                    if (!sst)
                        return AVERROR(ENOMEM);
                    st->priv_data = sst;
                    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                    st->codecpar->codec_tag = 0; /* no fourcc */
                    st->codecpar->codec_id = HAVE_BIGENDIAN ? AV_CODEC_ID_PCM_F32BE : AV_CODEC_ID_PCM_F32LE;
                    st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
                    st->codecpar->sample_rate = 0; // will be set later
                    avpriv_set_pts_info(st, 64, 1, (48000/128) << FREQ_BITS);
                    ret = setup_stream(sdr, st->index, station);
                    if (ret < 0)
                        return ret;
                    sdr->missing_streams++;
                }
            }
        }

        // new data is available for all streams, lets tell them
        //TODO with mixed blocks during scanning theres more than one block_center_freq, we may want to try to support them as they likely have demodulatable data
        for (int stream_index = 0; stream_index < s->nb_streams; stream_index++) {
            AVStream *st = s->streams[stream_index];
            SDRStream *sst = st->priv_data;
            sst->processing_index += sdr->block_size;
        }
    }

    sdr->last_pts = sdr->pts;
    sdr->pts += av_rescale(sdr->block_size, TIMEBASE, sdr->sdr_sample_rate);

    //some user apps force a delay on EAGAIN so we cannot always return EAGAIN or we overflow the fifos
    if (full_blocks >= 2)
        goto process_next_block;

    return AVERROR(EAGAIN);
}

int avpriv_sdr_read_seek(AVFormatContext *s, int stream_index,
                         int64_t target, int flags)
{
    SDRContext *sdr = s->priv_data;
    int64_t step;
    int dir = (flags & AVSEEK_FLAG_BACKWARD) ? -1 : 1;
    AVStream *st = s->streams[stream_index];
    SDRStream *sst = st->priv_data;
    int ret;

    if (sdr->mode != SingleStationMode) {
        return AVERROR(ENOTSUP);
    }

    av_assert0(stream_index >= 0);

    step = FFABS(target - sdr->pts);
    if (step < 35 * TIMEBASE) {
        target = (sdr->pts & (-1<<FREQ_BITS)) + dir;
        if (sst->station)
            target += lrint(sst->station->frequency/1000);
    }else if (step < 330 * TIMEBASE)
        return AVERROR(ENOTSUP); // Seek to next/prev band
    else
        return AVERROR(ENOTSUP); // Reserved for future ideas

    ret = snap2station(sdr, &dir);
    if (ret < 0)
        return ret;
    //snap2station found no station lets command the thread to seek
    if (!ret) {
        pthread_mutex_lock(&sdr->mutex);
        sdr->seek_direction = dir;
        pthread_mutex_unlock(&sdr->mutex);
        flush_fifo(sdr, sdr->full_block_fifo);
    }

    return 0;
}

void avpriv_sdr_stop_threading(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;

    atomic_store(&sdr->close_requested, 1);

    if(sdr->thread_started >= 2)
        pthread_join(sdr->hw_thread, NULL);

    if(sdr->thread_started >= 1) {
        flush_fifo(sdr, sdr->empty_block_fifo);
        flush_fifo(sdr, sdr-> full_block_fifo);

        pthread_mutex_destroy(&sdr->mutex);
    }

    sdr->thread_started = 0;
}

int avpriv_sdr_read_close(AVFormatContext *s)
{
    SDRContext *sdr = s->priv_data;
    int i;

    avpriv_sdr_stop_threading(s);

    av_fifo_freep2(&sdr->empty_block_fifo);
    av_fifo_freep2(&sdr->full_block_fifo);

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st   = s->streams[i];
        SDRStream *sst = st->priv_data;

        free_stream(sdr, i);

        av_freep(&sst->frame_buffer);
        sst->frame_size = 0;
    }

    av_tree_enumerate(sdr->station_root, NULL, NULL, free_station_enu);
    av_tree_destroy(sdr->station_root);
    sdr->station_root = NULL;

    av_freep(&sdr->windowed_block);
    av_freep(&sdr->block);
    av_freep(&sdr->len2block);
    av_freep(&sdr->window);

    av_tx_uninit(&sdr->fft_ctx);
    sdr->fft = NULL;

    avio_close(sdr->dump_avio);

    return 0;
}

static int sdrfile_probe(const AVProbeData *p)
{
    if (memcmp(p->buf  , "FFSDR00", 7))
        return 0;
    if (memcmp(p->buf+8,         "int16BE", 7) &&
        memcmp(p->buf+8,         "int08BE", 7))
        return 0;
    return AVPROBE_SCORE_MAX;
}

#define OFFSET(x) offsetof(SDRContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

const AVOption avpriv_sdr_options[] = {
    { "video_size", "set frame size", OFFSET(width), AV_OPT_TYPE_IMAGE_SIZE, {.str = "0x0"}, 0, 0, DEC },
    { "framerate" , "set frame rate", OFFSET(fps), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX,DEC },
    { "block_size", "FFT block size", OFFSET(block_size), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, DEC},
    { "mode",       ""              , OFFSET(mode), AV_OPT_TYPE_INT,       {.i64 = SingleStationMode}, 0, ModeNB-1, DEC, "mode"},
        { "single_mode", "Demodulate 1 station",    0, AV_OPT_TYPE_CONST,  {.i64 = SingleStationMode}, 0, 0, DEC, "mode"},
        { "all_mode"   , "Demodulate all station",  0, AV_OPT_TYPE_CONST,  {.i64 =    AllStationMode}, 0, 0, DEC, "mode"},

    { "station_freq", "current station/channel/stream frequency", OFFSET(station_freq), AV_OPT_TYPE_INT64, {.i64 = 88000000}, 0, INT64_MAX, DEC},

    { "driver"  , "sdr driver name"  , OFFSET(driver_name), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, DEC},
    { "rtlsdr_fixes" , "workaround rtlsdr issues", OFFSET(rtlsdr_fixes), AV_OPT_TYPE_INT , {.i64 = -1}, -1, 1, DEC},
    { "sdr_sr"  , "sdr sample rate"  , OFFSET(sdr_sample_rate ), AV_OPT_TYPE_INT , {.i64 = 0}, 0, INT_MAX, DEC},
    { "sdr_freq", "sdr frequency"    , OFFSET(wanted_freq), AV_OPT_TYPE_INT64 , {.i64 = 9000000}, 0, INT64_MAX, DEC},
    { "sdr_agc" , "sdr automatic gain control",  OFFSET(sdr_agc),  AV_OPT_TYPE_BOOL , {.i64 =  1}, -1, 1, DEC},
    { "sdr_adcc" ,"sdr automatic dc correction", OFFSET(sdr_adcc), AV_OPT_TYPE_BOOL , {.i64 = -1}, -1, 1, DEC},
    { "min_freq", "minimum frequency", OFFSET(min_freq   ), AV_OPT_TYPE_INT64 , {.i64 = 0}, 0, INT64_MAX, DEC},
    { "max_freq", "maximum frequency", OFFSET(max_freq   ), AV_OPT_TYPE_INT64 , {.i64 = 0}, 0, INT64_MAX, DEC},

    { "dumpurl", "url to dump soapy output to"  , OFFSET(dump_url), AV_OPT_TYPE_STRING , {.str = NULL}, 0, 0, DEC},
    { "kbd_alpha", "Kaiser Bessel window parameter"  , OFFSET(kbd_alpha), AV_OPT_TYPE_INT , {.i64 = 8}, 1, 16, DEC},


    { "am_mode", "AM Demodulation Mode", OFFSET(am_mode  ), AV_OPT_TYPE_INT , {.i64 = AMMidSide}, 0, AMModeNB-1, DEC, "am_mode"},
        { "am_leftright", "AM Demodulation Left Right", 0, AV_OPT_TYPE_CONST, {.i64 = AMLeftRight}, 0, 0, DEC, "am_mode"},
        { "am_midside", "AM Demodulation Mid Side", 0, AV_OPT_TYPE_CONST,   {.i64 = AMMidSide}, 0, 0, DEC, "am_mode"},
        { "am_inphase", "AM Demodulation In Phase", 0, AV_OPT_TYPE_CONST,   {.i64 = AMInPhase}, 0, 0, DEC, "am_mode"},
        { "am_envelope","AM Demodulation EnvelopeDC", 0, AV_OPT_TYPE_CONST, {.i64 = AMEnvelope}, 0, 0, DEC, "am_mode"},

    { "am_fft_ref", "Use FFT Based carrier for AM demodulation", OFFSET(am_fft_ref), AV_OPT_TYPE_INT , {.i64 = 0}, 0, 1, DEC},

    { "fm_emphasis"     , "FM De-Emphasis", OFFSET(emphasis_mode), AV_OPT_TYPE_INT, {.i64 = EMPHASIS_75us}, 0, EMPHASISNB - 1, DEC, "fm_emphasis"},
        { "emphasis75us", "FM De-Emphasis 75us", 0, AV_OPT_TYPE_CONST, {.i64 = EMPHASIS_75us}, 0, 0, DEC, "fm_emphasis"},
        { "emphasis50us", "FM De-Emphasis 50us", 0, AV_OPT_TYPE_CONST, {.i64 = EMPHASIS_50us}, 0, 0, DEC, "fm_emphasis"},
        { "none"        , "No FM De-Emphasis"  , 0, AV_OPT_TYPE_CONST, {.i64 = EMPHASIS_NONE}, 0, 0, DEC, "fm_emphasis"},

    { "am_threshold"     , "AM detection threshold", OFFSET(am_threshold), AV_OPT_TYPE_FLOAT, {.dbl = 20}, 0, FLT_MAX, DEC},
    { "fm_threshold"     , "FM detection threshold", OFFSET(fm_threshold), AV_OPT_TYPE_FLOAT, {.dbl = 50}, 0, FLT_MAX, DEC},

    { NULL },
};

static const AVClass sdrfile_demuxer_class = {
    .class_name = "sdrfile",
    .item_name  = av_default_item_name,
    .option     = avpriv_sdr_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEMUXER,
};

const AVInputFormat ff_sdrfile_demuxer = {
    .name           = "sdrfile",
    .long_name      = NULL_IF_CONFIG_SMALL("Software Defined Radio Demodulator (Using a file for testing)"),
    .priv_data_size = sizeof(SDRContext),
    .read_probe     = sdrfile_probe,
    .read_header    = sdrfile_initial_setup,
    .read_packet    = avpriv_sdr_read_packet,
    .read_close     = avpriv_sdr_read_close,
    .read_seek      = avpriv_sdr_read_seek,
    .flags_internal = FF_FMT_INIT_CLEANUP,
    .priv_class = &sdrfile_demuxer_class,
};
