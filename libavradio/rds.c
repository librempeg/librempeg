/*
 * RDS
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

/**
 * Check and correct RDS block
 * @param[out] group the data bits are returned here
 * @param block block nu,ber (0 to 3)
 * @return 1 if correctable single bit error, 0 if no error, >99 if non correctable errors
 */
static int check_rds_block(uint16_t group[4], const float diff[104], int block)
{
#define RDS_G 0x5B9 //101 1011 1001
    static const uint16_t offset[4] = {0x0FC, 0x198, 0x168, 0x1B4};
    unsigned codeword = 0;
    unsigned syndrom = 0;
    //we carry floats through to here so we can do a soft decission decoder
    //ATM lets just do hard decission decoding that should be more than good enough

    //FIXME we could do this more efficiently but does it matter?
    for(int i=0; i<26; i++) {
        int bit = (diff[i + block*26]<0);
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

    // try correcting some basic errors
    if (syndrom) {
        for (unsigned e = 1; e <= 2; e ++) {
            unsigned mask = 255 >> (8-e);
            unsigned syndrom1 = mask;
            for(int i=0; i<27-e; i++) {
                if (syndrom == syndrom1) {
                    group[block] ^= (mask<<i) >> 10;
                    return e;
                }
                syndrom1 += syndrom1;
                if (syndrom1 & (1<<10))
                    syndrom1 ^= RDS_G;
            }
        }
        return 100; // this is a good place do a 2nd pass with a soft decssion multi bit decoder
    }

    return 0;
}

static int decode_rds_group(SDRContext *sdr, SDRStream *sst, uint16_t group[4])
{
    Station *station = sst->station;
    int pi = group[0];
    int a  = group[1] >> 12;
    int b  = group[1] & 0x800;
    int tp = group[1] & 0x400;
    int pty= (group[1] >> 5) & 0x1F;

    switch(a) {
    case 0:
        AV_WB16(station->name + 2*(group[1]&3), group[3]);
    break;
    case 2:
        if (b) {
            AV_WB16(station->radiotext + 2*(group[1]&15)    , group[3]);
        } else {
            AV_WB16(station->radiotext + 4*(group[1]&15)    , group[2]);
            AV_WB16(station->radiotext + 4*(group[1]&15) + 2, group[3]);
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
        av_log(sdr->avfmt, AV_LOG_DEBUG, "RDS: PI %X, A %X B %X PTY %X\n", pi,a,b,pty);
    }

    return 0;
}

int ff_sdr_decode_rds(SDRContext *sdr, SDRStream *sst, AVComplexFloat *signal)
{
    int i, phase;
    float (*ring)[2] = sst->rds_ring;
    float diff[2*104 - 1];
    uint16_t group[4];
    int64_t num_step_in_p2 = sdr->sdr_sample_rate * (int64_t)sst->block_size_p2;
    int64_t den_step_on_p2 = sdr->block_size * 2375LL;
#define IDX(I) ((I)*num_step_in_p2/den_step_on_p2)

    av_assert0(sst->rds_ring_pos <= sst->rds_ring_size - 2*sst->block_size_p2);

    //For reasons that are beyond me, RDS spec allows inphase and quadrature so we have to compute and check both
    for (int i=0; i < sst->block_size_p2; i++) {
        ring[ sst->rds_ring_pos + i                      ][0] += signal[i].re * sst->window_p2[i];
        ring[ sst->rds_ring_pos + i + sst->block_size_p2 ][0]  = signal[i + sst->block_size_p2].re * sst->window_p2[i + sst->block_size_p2];
        ring[ sst->rds_ring_pos + i                      ][1] += signal[i].im * sst->window_p2[i];
        ring[ sst->rds_ring_pos + i + sst->block_size_p2 ][1]  = signal[i + sst->block_size_p2].im * sst->window_p2[i + sst->block_size_p2];
    }
    sst->rds_ring_pos += sst->block_size_p2;

    while (sst->rds_ring_pos > IDX(2) + IDX(4*104-1)) {
        int best_phase;
        float best_amplitude = -1;
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
        float last_bpsk = 0;
        for (i = 0; i<2*104; i++) {
            float bpsk = ring[IDX(2*i+1)][phase] - ring[IDX(2*i)][phase];
            if (i)
                diff[i-1] = bpsk * last_bpsk;
            last_bpsk = bpsk;
        }

        int best_errors = INT_MAX;
        for (phase = 0; phase < 104; phase++) {
            int error = 0;
            for (int block = 0; block < 4; block++) {
                error += check_rds_block(group, diff + phase, block);
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
                error += check_rds_block(group, diff + best_phase, block);
            }
            //have to recheck because of floats
            if (error < 10) {
                decode_rds_group(sdr, sst, group);
            }
        }
        int step = IDX(2*(best_phase + 103));

        av_assert0(sst->rds_ring_pos >= step);
        memmove(ring, ring + step, (sst->rds_ring_pos + sst->block_size_p2 - step) * sizeof(*sst->rds_ring));
        sst->rds_ring_pos -= step;
    }
    av_assert0 (sst->rds_ring_pos + 2*sst->block_size_p2 <= sst->rds_ring_size);

    return 0;
}
