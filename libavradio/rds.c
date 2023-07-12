/*
 * Radio Data System
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

#include "sdr.h"

#include <float.h>
#include "libavutil/avassert.h"
#include "libavutil/ffmath.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavformat/avformat.h"
#include "libavformat/demux.h"

#define MAX_BURST 5 // Tradeoff between undetected errors and correction capacity values from 2 to 5 are reasonable

static int burst_len(unsigned u)
{
    if (!u)
        return 0;

    while(!(u&1))
        u>>=1;
    return 1+av_log2(u);
}

/**
 * Check and correct RDS block
 * @param[out] group the data bits are returned here
 * @param block block nu,ber (0 to 3)
 * @return 1 if correctable single bit error, 0 if no error, >99 if non correctable errors
 */
static int check_rds_block(Station *station, uint16_t group[4], const float diff[104], const int block)
{
#define RDS_G 0x5B9 //101 1011 1001
    static const uint16_t offset[4] = {0x0FC, 0x198, 0x168, 0x1B4};
    unsigned codeword = 0;
    unsigned syndrom  = 0;
    const float *blockdiff = diff + block*26;

    //FIXME we could do this more efficiently but does it matter?
    for(int i=0; i<26; i++) {
        int bit = blockdiff[i] < 0;

        codeword += codeword + bit;
        syndrom += syndrom + bit;
        if (syndrom & (1<<10))
            syndrom ^= RDS_G;
    }

    if (block==2 && (group[1]&0x800)) {
        syndrom ^= 0x350;
    }else
        syndrom ^= offset[block];
    //FIXME the spec talks about a special case of a 0 offset used in the USA

    group[block] = codeword >> 10;

    // try correcting the most common error patterns
    for (int i=0; i<27-MAX_BURST; i++) {
        if (!(syndrom>>MAX_BURST)) {
            int ret = burst_len(syndrom);
            group[block] ^= (syndrom << i) >> 10;

            if (block == 0 && station->program_id[0]) {
                if (group[0] == station->program_id[0]) {
                    return ret;
                } else if (group[0] == station->program_id[1]) {
                    return ret;
                } else if (ret) {
                    return 20; //PI change is uncommon, so dont accept this in a damaged block, PI is repeated alot so we can wait for a clean block
                }
            }

            return ret;
        }
        if (syndrom&1)
            syndrom ^= RDS_G;
        syndrom >>= 1;
    }

    return 20;
}

static int decode_rds_group(SDRContext *sdr, Station *station, uint16_t group[4])
{
    int pi = group[0];
    int a  = group[1] >> 12;
    int b  = group[1] & 0x800;
    int tp = group[1] & 0x400;
    int pty= (group[1] >> 5) & 0x1F;

    if (station->program_id[0] && station->program_id[0] != pi)
        av_log(sdr->avfmt, AV_LOG_INFO, "PI changed to %X\n", pi);

    if (station->program_id[1] == pi) {
        FFSWAP(int, station->program_id[1], station->program_id[0]);
    } else if (station->program_id[0] != pi) {
        station->program_id[1] = pi;
        return 0; // skip first packet with new PI, likely its just damaged
    }

    //sucessfully decoding a RDS group implies that the station has been detected correctly
    station->timeout = 0;

    switch(a) {
    case 0:
        AV_WB16(station->name + 2*(group[1]&3), group[3]);
    break;
    case 2:{
        int new_ab_flag = group[1] & 16;
        if (new_ab_flag != station->rt_ab_flag) {
            memset(station->radiotext, 0, sizeof(station->radiotext));
            station->rt_ab_flag = new_ab_flag;
        }
        if (b) {
            AV_WB16(station->radiotext + 2*(group[1]&15)    , group[3]);
        } else {
            AV_WB16(station->radiotext + 4*(group[1]&15)    , group[2]);
            AV_WB16(station->radiotext + 4*(group[1]&15) + 2, group[3]);
        }
    break;}
    case 3:
        if (!b) {
            int application_id = group[3];
            switch (application_id) {
            case 0x4BD7:
                //RadioText Plus / RT+ for group 2A RT - IEC 62106-6:2023
                station->rtp_appgroup = group[1] & 31;
                break;
            }
        }
    break;
    case 10:
        if (b==0) {
            AV_WB16(station->programm_type_name + 4*(group[1]&1)    , group[2]);
            AV_WB16(station->programm_type_name + 4*(group[1]&1) + 2, group[3]);
        }
    break;
//     case 14:
//     break;
    default:
        if (2*a + b == station->rtp_appgroup) {
            int toggle_bit  = group[1]&16;
            int running_bit = group[1]&8;
            uint64_t v = ((group[1]&7LL)<<32) + ((uint64_t)group[2]<<16) + group[3];
            int tag[2][3];
            if (toggle_bit != station->rtp_toggle_bit) {
                station->artist[0] =
                station->title[0] =
                station->album[0] = 0;
            }
            station->rtp_toggle_bit = toggle_bit;
            for(int i = 0; i<6; i++) {
                tag[i/3][i%3] = (v>>29) & 63;
                v <<= 6;
            }
            tag[1][2] >>= 1;
            av_log(0,0, "\n");
            for(int i = 0; i<2; i++) {
                char *target= NULL;
                switch(tag[i][0]) {
                case 1: target = station->title; break;
                case 2: target = station->album; break;
                case 4: target = station->artist; break;
                default:
                    av_log(sdr->avfmt, AV_LOG_DEBUG, "Unhandled RT+ code %d\n", tag[i][0]);
                }
                if (target) {
                    memcpy(target, station->radiotext + tag[i][1], 1 + tag[i][2]);
                    target[1+tag[i][2]] = 0;
                }
            }
        }
    }

    return 0;
}

int ff_sdr_decode_rds(SDRContext *sdr, Station *station, AVComplexFloat *signal)
{
    int i, phase;
    float (*ring)[2] = station->rds_ring;
    float diff[2*104 - 1];
    uint16_t group[4];
    int64_t num_step_in_p2 = sdr->sdr_sample_rate * (int64_t)sdr->fm_block_size_p2;
    int64_t den_step_on_p2 = sdr->block_size * 2375LL;
#define IDX(I) ((I)*num_step_in_p2/den_step_on_p2)
    av_assert0(station->rds_ring_pos <= sdr->rds_ring_size - 2*sdr->fm_block_size_p2);

    //For reasons that are beyond me, RDS spec allows inphase and quadrature so we have to compute and check both
    for (int i=0; i < sdr->fm_block_size_p2; i++) {
        ring[ station->rds_ring_pos + i                         ][0] += signal[i].re * sdr->fm_window_p2[i];
        ring[ station->rds_ring_pos + i + sdr->fm_block_size_p2 ][0]  = signal[i + sdr->fm_block_size_p2].re * sdr->fm_window_p2[i + sdr->fm_block_size_p2];
        ring[ station->rds_ring_pos + i                         ][1] += signal[i].im * sdr->fm_window_p2[i];
        ring[ station->rds_ring_pos + i + sdr->fm_block_size_p2 ][1]  = signal[i + sdr->fm_block_size_p2].im * sdr->fm_window_p2[i + sdr->fm_block_size_p2];
    }
    station->rds_ring_pos += sdr->fm_block_size_p2;

    while (station->rds_ring_pos > IDX(2) + IDX(4*104-1)) {
        int best_phase, step;
        float best_amplitude = -1;
        float last_bpsk = 0;
        int best_errors = INT_MAX;
        for (phase = 0; phase < 2*IDX(2); phase++) {
            double a = 0;
            for (i = 0; i<2*104; i++) {
                a += fabs(ring[IDX(2*i+1)][phase] - ring[IDX(2*i)][phase]);
            }
            if (a > best_amplitude) {
                best_amplitude = a;
                best_phase = phase;
            }
        }

        phase = best_phase;
        for (i = 0; i<2*104; i++) {
            float bpsk = ring[IDX(2*i+1)][phase] - ring[IDX(2*i)][phase];
            if (i)
                diff[i-1] = bpsk * last_bpsk;
            last_bpsk = bpsk;
        }

        for (phase = 0; phase < 104; phase++) {
            int error = 0;
            for (int block = 0; block < 4; block++) {
                error += check_rds_block(station, group, diff + phase, block);
            }
            if (error < best_errors) {
                best_errors = error;
                best_phase = phase;
            }
        }
        av_log(sdr->avfmt, AV_LOG_DEBUG, "RDS ERR:%d\n", best_errors);

        // are we having no errors or correctable errors
        if (best_errors < 10) {
            int error = 0;
            for (int block = 0; block < 4; block++) {
                error += check_rds_block(station, group, diff + best_phase, block);
            }
            //have to recheck because of floats
            if (error < 10) {
                decode_rds_group(sdr, station, group);
            }
        }
        step = IDX(2*(best_phase + 103));

        av_assert0(station->rds_ring_pos >= step);
        memmove(ring, ring + step, (station->rds_ring_pos + sdr->fm_block_size_p2 - step) * sizeof(*station->rds_ring));
        station->rds_ring_pos -= step;
    }
    av_assert0 (station->rds_ring_pos + 2*sdr->fm_block_size_p2 <= sdr->rds_ring_size);

    return 0;
}
