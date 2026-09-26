/*
 * Common finite DreamFactory movie timeline support
 *
 * This file is part of Librempeg.
 */

#ifndef AVFORMAT_CFDF_MOV_H
#define AVFORMAT_CFDF_MOV_H

#include <stdint.h>

#include "avformat.h"

#define CFDF_MOV_NAME_SIZE 16

typedef struct CFDFMovBlock {
    int64_t offset;
    int32_t size;
    int32_t duration;
    int32_t palette;
    int16_t ry0, rx0, ry1, rx1;
    int16_t cw, ch;
    int16_t px, py;
} CFDFMovBlock;

typedef struct CFDFMovFrameInfo {
    int block;
    uint16_t action;
    uint8_t sfx;
} CFDFMovFrameInfo;

typedef struct CFDFMovStream {
    CFDFMovBlock *blocks;
    int nb_blocks;
    int block_idx;
    int64_t start_pts;
    int64_t pts;
} CFDFMovStream;

typedef struct CFDFMovSound {
    int64_t data;
    int32_t size;
    int32_t nb_samples;
    int codec;
    int rate;
    int silent;
} CFDFMovSound;

typedef struct CFDFMovSFX {
    CFDFMovSound sound;
    int64_t start_ticks;
    int loop;
    char name[CFDF_MOV_NAME_SIZE + 1];
} CFDFMovSFX;

typedef struct CFDFMovPlaylist {
    CFDFMovSound *seq;
    int nseq;
    int loop_start;
    int finite;
    int disk;
    int64_t start_ticks;
} CFDFMovPlaylist;

int ff_cfdf_mov_append_video_loop(CFDFMovBlock **blocks, int *nb_blocks,
                                  int *blocks_alloc, int loop_start,
                                  int loop_end, int64_t *ticks,
                                  int64_t end_ticks);
int ff_cfdf_mov_add_audio_stream(AVFormatContext *s,
                                 const CFDFMovSound *snds, int count,
                                 int64_t start_ns, const char *title,
                                 const char *timeline,
                                 int64_t *max_audio_end);
int ff_cfdf_mov_add_sfx_stream(AVFormatContext *s, const CFDFMovSFX *sfx,
                               int64_t end_ticks, int64_t *max_audio_end);
int64_t ff_cfdf_mov_playlist_audible_end(const CFDFMovSound *seq, int nseq,
                                         int loop_start, int finite, int disk,
                                         int last_playlist,
                                         int64_t start_ticks,
                                         int64_t end_ticks);
int ff_cfdf_mov_schedule_playlists(AVFormatContext *s, CFDFMovPlaylist *pls,
                                   int npls, int64_t chain_end,
                                   int64_t *max_audio_end);

#endif /* AVFORMAT_CFDF_MOV_H */
