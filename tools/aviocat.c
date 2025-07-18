/*
 * Copyright (c) 2012 Martin Storsjo
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

#include <stdio.h>
#include <stdlib.h>

#include "libavutil/time.h"
#include "libavformat/avformat.h"

static int usage(const char *argv0, int ret)
{
    fprintf(stderr, "%s [-b bytespersec] [-d duration] [-oi <options>] [-oo <options>] [-v] input_url output_url\n", argv0);
    fprintf(stderr, "<options>: AVOptions expressed as key=value, :-separated\n");
    return ret;
}

int main(int argc, char **argv)
{
    int bps = 0, duration = 0, verbose = 0, ret, i;
    const char *input_url = NULL, *output_url = NULL;
    int64_t stream_pos = 0;
    int64_t start_time;
    AVIOContext *input, *output;
    AVDictionary *in_opts = NULL;
    AVDictionary *out_opts = NULL;

    avformat_network_init();

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-b") && i + 1 < argc) {
            bps = atoi(argv[i + 1]);
            i++;
        } else if (!strcmp(argv[i], "-d") && i + 1 < argc) {
            duration = atoi(argv[i + 1]);
            i++;
        } else if (!strcmp(argv[i], "-oi") && i + 1 < argc) {
            if (av_dict_parse_string(&in_opts, argv[i + 1], "=", ":", 0) < 0) {
                fprintf(stderr, "Cannot parse option string %s\n",
                        argv[i + 1]);
                return usage(argv[0], 1);
            }
            i++;
        } else if (!strcmp(argv[i], "-oo") && i + 1 < argc) {
            if (av_dict_parse_string(&out_opts, argv[i + 1], "=", ":", 0) < 0) {
                fprintf(stderr, "Cannot parse option string %s\n",
                        argv[i + 1]);
                return usage(argv[0], 1);
            }
            i++;
        } else if (!strcmp(argv[i], "-v")) {
            verbose = 1;
        } else if (!input_url) {
            input_url = argv[i];
        } else if (!output_url) {
            output_url = argv[i];
        } else {
            return usage(argv[0], 1);
        }
    }
    if (!output_url)
        return usage(argv[0], 1);

    ret = avio_open2(&input, input_url, AVIO_FLAG_READ, NULL, &in_opts);
    if (ret) {
        fprintf(stderr, "Unable to open %s: %s\n", input_url, av_err2str(ret));
        return 1;
    }
    if (verbose) {
        int64_t size = avio_seek(input, 0, AVSEEK_SIZE);
        if (size >= 0) {
            fprintf(stderr, "aviocat: input size: %"PRId64"\n", size);
        } else {
            fprintf(stderr, "aviocat: input size: unknown\n");
        }
    }
    if (duration && !bps) {
        int64_t size = avio_size(input);
        if (size < 0) {
            fprintf(stderr, "Unable to get size of %s: %s\n", input_url, av_err2str(ret));
            goto fail;
        }
        bps = size / duration;
    }
    ret = avio_open2(&output, output_url, AVIO_FLAG_WRITE, NULL, &out_opts);
    if (ret) {
        fprintf(stderr, "Unable to open %s: %s\n", output_url, av_err2str(ret));
        goto fail;
    }

    start_time = av_gettime_relative();
    while (1) {
        uint8_t buf[1024];
        int n;
        n = avio_read(input, buf, sizeof(buf));
        if (n <= 0)
            break;
        avio_write(output, buf, n);
        if (output->error) {
            fprintf(stderr, "Unable to write %s: %s\n", output_url, av_err2str(output->error));
            break;
        }
        stream_pos += n;
        if (bps) {
            avio_flush(output);
            while ((av_gettime_relative() - start_time) * bps / AV_TIME_BASE < stream_pos)
                av_usleep(50 * 1000);
        }
    }

    avio_flush(output);
    avio_close(output);

fail:
    av_dict_free(&in_opts);
    av_dict_free(&out_opts);
    avio_close(input);
    avformat_network_deinit();
    return ret ? 1 : 0;
}
