/*
 * copyright (c) 2009 Michael Niedermayer <michaelni@gmx.at>
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

#include <stdlib.h>

#include "libavformat/avformat.h"
#include "libavformat/demux.h"
#include "libavcodec/put_bits.h"
#include "libavutil/lfg.h"
#include "libavutil/mem.h"
#include "libavutil/timer.h"

#define MAX_FORMATS 1000 //this must be larger than the number of formats
static int score_array[MAX_FORMATS];
static int64_t time_array[MAX_FORMATS];
static int failures = 0;
static const char *single_format;

#ifndef AV_READ_TIME
#define AV_READ_TIME(x) 0
#endif

static void probe(AVProbeData *pd, int type, int p, int size)
{
    int i = 0;
    const AVInputFormat *fmt = NULL;
    void *fmt_opaque = NULL;

    while ((fmt = av_demuxer_iterate(&fmt_opaque))) {
        if (fmt->flags & AVFMT_NOFILE)
            continue;
        if (ffifmt(fmt)->read_probe &&
            (!single_format || !strcmp(single_format, fmt->name))
        ) {
            int score;
            int64_t start = AV_READ_TIME();
            score = ffifmt(fmt)->read_probe(pd);
            time_array[i] += AV_READ_TIME() - start;
            if (score > score_array[i] && score > AVPROBE_SCORE_MAX / 4) {
                score_array[i] = score;
                fprintf(stderr,
                        "Failure of %s probing code with score=%d type=%d p=%X size=%d\n",
                        fmt->name, score, type, p, size);
                failures++;
            }
        }
        i++;
    }
}

static void print_times(void)
{
    int i = 0;
    const AVInputFormat *fmt = NULL;
    void *fmt_opaque = NULL;

    while ((fmt = av_demuxer_iterate(&fmt_opaque))) {
        if (fmt->flags & AVFMT_NOFILE)
            continue;
        if (time_array[i] > 1000000) {
            fprintf(stderr, "%12"PRIu64" cycles, %12s\n",
                    time_array[i], fmt->name);
        }
        i++;
    }
}

static int read_int(char *arg) {
    int ret;

    if (!arg || !*arg)
        return -1;
    ret = strtol(arg, &arg, 0);
    if (*arg)
        return -1;
    return ret;
}

int main(int argc, char **argv)
{
    unsigned int p, i, type, size, retry;
    AVProbeData pd = { 0 };
    AVLFG state;
    PutBitContext pb;
    int retry_count= 4097;
    int max_size = 65537;
    int j;

    for (j = i = 1; i<argc; i++) {
        if (!strcmp(argv[i], "-f") && i+1<argc && !single_format) {
            single_format = argv[++i];
        } else if (read_int(argv[i])>0 && j == 1) {
            retry_count = read_int(argv[i]);
            j++;
        } else if (read_int(argv[i])>0 && j == 2) {
            max_size = read_int(argv[i]);
            j++;
        } else {
            fprintf(stderr, "probetest [-f <input format>] [<retry_count> [<max_size>]]\n");
            return 1;
        }
    }

    if (max_size > 1000000000U/8) {
        fprintf(stderr, "max_size out of bounds\n");
        return 1;
    }

    if (retry_count > 1000000000U) {
        fprintf(stderr, "retry_count out of bounds\n");
        return 1;
    }

    av_lfg_init(&state, 0xdeadbeef);

    pd.buf = NULL;
    for (size = 1; size < max_size; size *= 2) {
        pd.buf_size = size;
        pd.buf      = av_realloc(pd.buf, size + AVPROBE_PADDING_SIZE);
        pd.filename = "";

        if (!pd.buf) {
            fprintf(stderr, "out of memory\n");
            return 1;
        }

        memset(pd.buf, 0, size + AVPROBE_PADDING_SIZE);

        fprintf(stderr, "testing size=%d\n", size);

        for (retry = 0; retry < retry_count; retry += FFMAX(size, 32)) {
            for (type = 0; type < 4; type++) {
                for (p = 0; p < 4096; p++) {
                    unsigned hist = 0;
                    init_put_bits(&pb, pd.buf, size);
                    switch (type) {
                    case 0:
                        for (i = 0; i < size * 8; i++)
                            put_bits(&pb, 1, (av_lfg_get(&state) & 0xFFFFFFFF) > p << 20);
                        break;
                    case 1:
                        for (i = 0; i < size * 8; i++) {
                            unsigned int p2 = hist ? p & 0x3F : (p >> 6);
                            unsigned int v  = (av_lfg_get(&state) & 0xFFFFFFFF) > p2 << 26;
                            put_bits(&pb, 1, v);
                            hist = v;
                        }
                        break;
                    case 2:
                        for (i = 0; i < size * 8; i++) {
                            unsigned int p2 = (p >> (hist * 3)) & 7;
                            unsigned int v  = (av_lfg_get(&state) & 0xFFFFFFFF) > p2 << 29;
                            put_bits(&pb, 1, v);
                            hist = (2 * hist + v) & 3;
                        }
                        break;
                    case 3:
                        for (i = 0; i < size; i++) {
                            int c = 0;
                            while (p & 63) {
                                c = (av_lfg_get(&state) & 0xFFFFFFFF) >> 24;
                                if (c >= 'a' && c <= 'z' && (p & 1))
                                    break;
                                else if (c >= 'A' && c <= 'Z' && (p & 2))
                                    break;
                                else if (c >= '0' && c <= '9' && (p & 4))
                                    break;
                                else if (c == ' ' && (p & 8))
                                    break;
                                else if (c == 0 && (p & 16))
                                    break;
                                else if (c == 1 && (p & 32))
                                    break;
                            }
                            pd.buf[i] = c;
                        }
                    }
                    flush_put_bits(&pb);
                    probe(&pd, type, p, size);
                }
            }
        }
    }
    if(AV_READ_TIME())
        print_times();
    return failures;
}
