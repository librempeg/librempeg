/*
 * Common finite DreamFactory movie timeline support
 *
 * This file is part of Librempeg.
 */

#include <limits.h>
#include <string.h>

#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavcodec/cfdf_audio.h"
#include "avformat.h"
#include "cfdf_mov.h"
#include "internal.h"

int ff_cfdf_mov_append_video_loop(CFDFMovBlock **blocks, int *nb_blocks,
                                  int *blocks_alloc, int loop_start,
                                  int loop_end, int64_t *ticks,
                                  int64_t end_ticks)
{
    int64_t loop_ticks = 0;
    int loop_blocks;

    if (loop_start < 0 || loop_end < loop_start || loop_end >= *nb_blocks)
        return AVERROR_INVALIDDATA;

    for (int i = loop_start; i <= loop_end; i++)
        loop_ticks += (*blocks)[i].duration;
    if (loop_ticks <= 0)
        return AVERROR_INVALIDDATA;

    loop_blocks = loop_end - loop_start + 1;
    if (end_ticks > *ticks) {
        int64_t remaining = end_ticks - *ticks;
        int64_t cycles = remaining / loop_ticks +
                         (remaining % loop_ticks != 0);

        if (cycles > (INT_MAX - *nb_blocks) / loop_blocks)
            return AVERROR_INVALIDDATA;
    }

    while (*ticks < end_ticks) {
        for (int i = loop_start; i <= loop_end && *ticks < end_ticks; i++) {
            CFDFMovBlock block = (*blocks)[i];
            int64_t remaining = end_ticks - *ticks;

            block.duration = FFMIN(block.duration, remaining);
            if (block.duration <= 0)
                return AVERROR_INVALIDDATA;

            if (*nb_blocks == *blocks_alloc) {
                CFDFMovBlock *new_blocks;
                int new_alloc;

                if (*blocks_alloc > INT_MAX / 2)
                    return AVERROR(ENOMEM);
                new_alloc = *blocks_alloc ? *blocks_alloc * 2 : 256;
                new_blocks = av_realloc_array(*blocks, new_alloc,
                                              sizeof(**blocks));
                if (!new_blocks)
                    return AVERROR(ENOMEM);
                *blocks = new_blocks;
                *blocks_alloc = new_alloc;
            }
            (*blocks)[(*nb_blocks)++] = block;
            *ticks += block.duration;
        }
    }

    return 0;
}

static int mov_trim_sound(AVFormatContext *s, CFDFMovSound *snd,
                          int64_t max_samples)
{
    if (max_samples <= 0) {
        snd->nb_samples = 0;
        snd->size       = 0;
        return 0;
    }
    if (snd->nb_samples <= max_samples)
        return 0;
    if (snd->codec == 2) {
        snd->size = snd->nb_samples = (int32_t)max_samples;
    } else {
        uint8_t *buf = av_malloc(snd->size);
        int64_t got = 0;

        if (!buf)
            return AVERROR(ENOMEM);
        avio_seek(s->pb, snd->data, SEEK_SET);
        if (avio_read(s->pb, buf, snd->size) != snd->size) {
            av_free(buf);
            return AVERROR_INVALIDDATA;
        }
        snd->size = ff_cfdf_v40_movie_prefix(buf, snd->size,
                                              max_samples, &got);
        av_free(buf);
        if (snd->size < 0)
            return snd->size;
        snd->nb_samples = (int32_t)got;
    }
    return 0;
}

int ff_cfdf_mov_add_audio_stream(AVFormatContext *s,
                                 const CFDFMovSound *snds, int count,
                                 int64_t start_ns, const char *title,
                                 const char *timeline,
                                 int64_t *max_audio_end)
{
    CFDFMovStream *cs;
    AVStream *st;
    int64_t total = 0;
    int variant = snds[0].codec == 2;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    av_dict_set(&st->metadata, "timeline", timeline, 0);
    cs = av_mallocz(sizeof(*cs));
    if (!cs)
        return AVERROR(ENOMEM);
    st->priv_data = cs;

    cs->blocks = av_calloc(count, sizeof(*cs->blocks));
    if (!cs->blocks)
        return AVERROR(ENOMEM);
    for (int i = 0; i < count; i++) {
        cs->blocks[i].offset   = snds[i].data;
        cs->blocks[i].size     = snds[i].size;
        cs->blocks[i].duration = snds[i].nb_samples;
        cs->blocks[i].palette  = -1;
        total += snds[i].nb_samples;
    }
    cs->nb_blocks = count;
    cs->start_pts = av_rescale(start_ns, snds[0].rate, 1000000000);
    cs->pts       = cs->start_pts;

    if (max_audio_end && strcmp(timeline, "untimed")) {
        int64_t end_ns = start_ns + av_rescale(total, 1000000000,
                                               snds[0].rate);
        int64_t end_ticks = av_rescale_rnd(end_ns, 60, 1000000000,
                                           AV_ROUND_UP);
        if (end_ticks > *max_audio_end)
            *max_audio_end = end_ticks;
    }

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = variant ? AV_CODEC_ID_CFDF_DPCM : AV_CODEC_ID_ADPCM_CFDF;
    st->codecpar->sample_rate = snds[0].rate;
    st->codecpar->ch_layout   = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    st->start_time            = cs->start_pts;
    st->duration              = total;

    st->codecpar->extradata = av_mallocz(1 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!st->codecpar->extradata)
        return AVERROR(ENOMEM);
    st->codecpar->extradata[0]   = 1;
    st->codecpar->extradata_size = 1;

    if (title && title[0])
        av_dict_set(&st->metadata, "title", title, 0);

    avpriv_set_pts_info(st, 64, 1, snds[0].rate);
    return 0;
}

int64_t ff_cfdf_mov_playlist_audible_end(const CFDFMovSound *seq, int nseq,
                                         int loop_start, int finite, int disk,
                                         int last_playlist,
                                         int64_t start_ticks,
                                         int64_t end_ticks)
{
    int64_t start_ns, end_ns, rel_ns, pass_ns = 0, cycle_ns = 0;
    int64_t occurrence_ns, first_ns = -1, last_ns = -1, off_ns = 0;
    int first, loop;

    if (nseq <= 0 || end_ticks <= start_ticks)
        return end_ticks;

    start_ns = av_rescale(start_ticks, 1000000000, 60);
    end_ns   = av_rescale(end_ticks,   1000000000, 60);
    rel_ns   = end_ns - start_ns;

    for (int i = 0; i < nseq; i++)
        pass_ns += av_rescale(seq[i].nb_samples, 1000000000, seq[i].rate);
    if (pass_ns <= 0)
        return end_ticks;

    loop = !disk && loop_start >= 0 && !(finite && last_playlist);
    first = 0;
    occurrence_ns = start_ns;

    if (rel_ns >= pass_ns) {
        int64_t after_first, cycle_index;

        if (!loop)
            return end_ticks;
        loop_start = FFMIN(loop_start, nseq - 1);
        for (int i = loop_start; i < nseq; i++)
            cycle_ns += av_rescale(seq[i].nb_samples, 1000000000,
                                   seq[i].rate);
        if (cycle_ns <= 0)
            return end_ticks;

        after_first   = rel_ns - pass_ns;
        cycle_index   = after_first / cycle_ns;
        occurrence_ns = start_ns + pass_ns + cycle_index * cycle_ns;
        first         = loop_start;
    }

    for (int i = first; i < nseq; i++) {
        int64_t dur_ns = av_rescale(seq[i].nb_samples, 1000000000,
                                    seq[i].rate);

        if (!seq[i].silent) {
            if (first_ns < 0)
                first_ns = off_ns;
            last_ns = off_ns + dur_ns;
        }
        off_ns += dur_ns;
    }

    if (first_ns < 0 || end_ns <= occurrence_ns + first_ns ||
        end_ns >= occurrence_ns + last_ns)
        return end_ticks;
    if (loop && last_ns >= off_ns)
        return end_ticks;

    return av_rescale_rnd(occurrence_ns + last_ns, 60, 1000000000,
                          AV_ROUND_UP);
}

int ff_cfdf_mov_add_sfx_stream(AVFormatContext *s, const CFDFMovSFX *sfx,
                               int64_t end_ticks, int64_t *max_audio_end)
{
    CFDFMovSound *seq;
    int64_t max_samples, remaining;
    int count, nseq = 0, ret;

    max_samples = av_rescale_rnd(end_ticks - sfx->start_ticks,
                                 sfx->sound.rate, 60, AV_ROUND_DOWN);
    if (max_samples <= 0)
        return 0;

    count = sfx->loop ? FFMIN((max_samples + sfx->sound.nb_samples - 1) /
                              sfx->sound.nb_samples, 8192) : 1;
    seq = av_calloc(count, sizeof(*seq));
    if (!seq)
        return AVERROR(ENOMEM);

    remaining = max_samples;
    for (int i = 0; i < count && remaining > 0; i++) {
        seq[nseq] = sfx->sound;
        ret = mov_trim_sound(s, &seq[nseq], remaining);
        if (ret < 0)
            goto fail;
        if (seq[nseq].nb_samples <= 0)
            break;
        remaining -= seq[nseq].nb_samples;
        nseq++;
    }

    ret = nseq ? ff_cfdf_mov_add_audio_stream(s, seq, nseq,
                                      av_rescale(sfx->start_ticks,
                                                 1000000000, 60),
                                      sfx->name, "sfx", max_audio_end) : 0;
fail:
    av_free(seq);
    return ret;
}

int ff_cfdf_mov_schedule_playlists(AVFormatContext *s, CFDFMovPlaylist *pls,
                                   int npls, int64_t chain_end,
                                   int64_t *max_audio_end)
{
    CFDFMovSound *run = NULL;
    int ret = 0;

    for (int j = 0; j < npls; j++) {
        CFDFMovSound *seq = pls[j].seq;
        int nseq = pls[j].nseq;
        int64_t win_end = j + 1 < npls ? pls[j + 1].start_ticks : chain_end;
        int64_t start_ns = av_rescale(pls[j].start_ticks, 1000000000, 60);
        int64_t win_end_ns = av_rescale(win_end, 1000000000, 60);
        int64_t pass_ns = 0, sched_ns, run_start_ns;
        int lo = 0, hi, nrun = 0, run_alloc = 0, loop, loop_start;

        run = NULL;

        for (int k = 0; k < nseq; k++)
            pass_ns += av_rescale(seq[k].nb_samples, 1000000000,
                                  seq[k].rate);

        loop = pls[j].loop_start >= 0 &&
               !(pls[j].finite && j + 1 == npls);
        loop_start = loop ? FFMIN(pls[j].loop_start, nseq - 1) : 0;

        if (loop && pass_ns > 0 && nseq > 0) {
            int64_t cycle_ns = 0;
            int tail = nseq - loop_start;
            int extra = 0;

            for (int k = loop_start; k < nseq; k++)
                cycle_ns += av_rescale(seq[k].nb_samples, 1000000000,
                                       seq[k].rate);
            while (cycle_ns > 0 &&
                   pass_ns + (int64_t)extra * cycle_ns <
                       win_end_ns - start_ns &&
                   nseq + (int64_t)(extra + 1) * tail <= 8192)
                extra++;
            if (extra > 0) {
                CFDFMovSound *ext =
                    av_realloc_array(seq, nseq + (size_t)extra * tail,
                                     sizeof(*seq));
                if (!ext) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                for (int r = 0; r < extra; r++)
                    memcpy(ext + nseq + (size_t)r * tail,
                           ext + loop_start, tail * sizeof(*ext));
                pls[j].seq = seq = ext;
                nseq += extra * tail;
                pls[j].nseq = nseq;
            }
        }

        lo = nseq;
        hi = -1;
        for (int k = 0; k < nseq; k++) {
            if (!seq[k].silent) {
                if (k < lo)
                    lo = k;
                if (k > hi)
                    hi = k;
            }
        }
        if (lo > hi) {
            lo = 0;
            hi = nseq - 1;
        }

        sched_ns = run_start_ns = start_ns;
        for (int k = 0; k < nseq; k++) {
            int64_t room, dur_ns;

            if (sched_ns >= win_end_ns)
                break;
            room = av_rescale(win_end_ns - sched_ns, seq[k].rate,
                              1000000000);
            if ((ret = mov_trim_sound(s, &seq[k], room)) < 0)
                goto fail;
            if (!seq[k].nb_samples)
                break;
            dur_ns = av_rescale(seq[k].nb_samples, 1000000000, seq[k].rate);
            if (k >= lo && k <= hi) {
                if (nrun > 0 && (seq[k].codec != run[0].codec ||
                                 seq[k].rate  != run[0].rate)) {
                    ret = ff_cfdf_mov_add_audio_stream(s, run, nrun,
                                                       run_start_ns,
                                                       av_basename(s->url),
                                                       "background",
                                                       max_audio_end);
                    if (ret < 0)
                        goto fail;
                    nrun = 0;
                    run_start_ns = sched_ns;
                }
                if (nrun == run_alloc) {
                    CFDFMovSound *nr = av_realloc_array(run,
                                            run_alloc + 64, sizeof(*run));
                    if (!nr) {
                        ret = AVERROR(ENOMEM);
                        goto fail;
                    }
                    run = nr;
                    run_alloc += 64;
                }
                run[nrun++] = seq[k];
            }
            sched_ns += dur_ns;
            if (nrun == 0)
                run_start_ns = sched_ns;
        }
        if (nrun > 0) {
            ret = ff_cfdf_mov_add_audio_stream(s, run, nrun, run_start_ns,
                                               av_basename(s->url),
                                               "background", max_audio_end);
            if (ret < 0)
                goto fail;
        }
        av_freep(&run);
    }
    return 0;

fail:
    av_freep(&run);
    return ret;
}
