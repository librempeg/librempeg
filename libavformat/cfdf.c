/*
 * CFDF demuxer
 * Copyright (c) 2025 Paul B Mahol
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

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/pixfmt.h"
#include "libavcodec/cfdf_audio.h"
#include "avformat.h"
#include "cfdf_mov.h"
#include "cfdf_bank.h"
#include "demux.h"
#include "internal.h"

typedef struct CFDFDemuxContext {
    CFDFBank *bank;
    int big_endian;
    int movie;              /* container 0 is a DreamFactory movie header */
    uint8_t *palettes;      /* movie mode: nb_palettes x AVPALETTE_SIZE */
    int nb_palettes;
    int64_t max_audio_end;  /* latest audio end on the timeline, in 1/60 s ticks */
} CFDFDemuxContext;

static unsigned cfdf_r16(AVFormatContext *s)
{
    CFDFDemuxContext *ctx = s->priv_data;

    return ctx->big_endian ? avio_rb16(s->pb) : avio_rl16(s->pb);
}

static unsigned cfdf_r32(AVFormatContext *s)
{
    CFDFDemuxContext *ctx = s->priv_data;

    return ctx->big_endian ? avio_rb32(s->pb) : avio_rl32(s->pb);
}

static int read_probe(const AVProbeData *p)
{
    unsigned count, size;
    int be, tagged;

    if (p->buf_size < 0x408)
        return 0;
    tagged = !memcmp(p->buf + 0x20, "LPPALPPA", 8) ||
             !memcmp(p->buf + 0x20, "MOVEDFME", 8) ||
             !memcmp(p->buf + 0x20, "SONGDFST", 8);
    be = !memcmp(p->buf + 0x20, "MOVEDFME", 8) ||
         !memcmp(p->buf + 0x20, "SONGDFST", 8) ||
         (AV_RB32(p->buf) == 0x10000 && AV_RL32(p->buf) != 0x10000);
    if (!tagged && (be ? AV_RB32(p->buf) : AV_RL32(p->buf)) != 0x10000)
        return 0;
    count = be ? AV_RB32(p->buf + 0x14) : AV_RL32(p->buf + 0x14);
    size = be ? AV_RB32(p->buf + 4) : AV_RL32(p->buf + 4);
    if (count < 2 || count > INT16_MAX || 0x400LL + count * 4 > size)
        return 0;
    for (int i = 0; i < FFMIN(count, (p->buf_size - 0x400) / 4); i++) {
        unsigned off = be ? AV_RB32(p->buf + 0x400 + i * 4) :
                            AV_RL32(p->buf + 0x400 + i * 4);
        if (off && (off < 0x400LL + count * 4 || off > size - 8))
            return 0;
    }
    return tagged ? AVPROBE_SCORE_MAX : AVPROBE_SCORE_EXTENSION + 1;
}

#define MOV_MAX_SEGMENTS 64

enum CFDFMovLayout {
    CFDF_MOV_NONE,
    CFDF_MOV_PROTO,
    CFDF_MOV_V1,
    CFDF_MOV_V4,
};

static void mov_read_pstring(AVIOContext *pb, int64_t off, int64_t fsize,
                             char *dst, int dst_size)
{
    int len, i;

    dst[0] = '\0';
    if (off < 0 || off + 1 > fsize)
        return;
    avio_seek(pb, off, SEEK_SET);
    len = avio_r8(pb);
    if (len > dst_size - 1)
        len = dst_size - 1;
    if (off + 1 + len > fsize)
        len = (int)(fsize - (off + 1));
    for (i = 0; i < len; i++)
        dst[i] = avio_r8(pb);
    dst[i] = '\0';
}

static int mov_valid_cont(int containers, const int64_t *coffs, int64_t fsize,
                          int64_t idx, int64_t need)
{
    return idx >= 0 && idx < containers && coffs[idx] > 0 &&
           coffs[idx] + 8 + need <= fsize;
}


/* Parse a movie sound container: codec u16 at payload +0x1a (1 = v4.0 ADPCM,
 * 2 = v4.1 DPCM), rate at +0x1c, data offset at +0x2c (relative to the
 * payload); the chunk runs to the container end. */
static int mov_parse_sound(AVFormatContext *s, int containers,
                           const int64_t *coffs, int64_t fsize,
                           int64_t idx, CFDFMovSound *snd)
{
    AVIOContext *pb = s->pb;
    int64_t hp;
    uint32_t csize, data_off;
    int codec, rate;

    if (!mov_valid_cont(containers, coffs, fsize, idx, 0x30))
        return AVERROR_INVALIDDATA;
    hp = coffs[idx] + 8;

    avio_seek(pb, coffs[idx] + 0x04, SEEK_SET);
    csize = cfdf_r32(s);
    avio_seek(pb, hp + 0x1a, SEEK_SET);
    codec = cfdf_r16(s);
    rate  = cfdf_r32(s);
    avio_seek(pb, hp + 0x2c, SEEK_SET);
    data_off = cfdf_r32(s);

    if (codec != 1 && codec != 2)
        return AVERROR_INVALIDDATA;
    if (rate != 11025 && rate != 22050 && rate != 44100)
        return AVERROR_INVALIDDATA;
    if (data_off >= csize || csize > INT32_MAX || csize > fsize - hp)
        return AVERROR_INVALIDDATA;

    snd->data  = hp + data_off;
    snd->size  = (int32_t)(csize - data_off);
    snd->codec = codec;
    snd->rate  = rate;

    /* Silence detection follows the v5 demuxer / vgmstream rules:
     * v4.0 blocks are silent when every control byte is 0x40 (DC absolute) or
     * >= 0xC0 (Mode III repeats of the DC level); v4.1 when every byte is
     * 0x00 or 0x80. Used to trim leading/trailing silence-filler chunks
     * (typically low-rate) from background playlists. */
    {
        uint8_t *buf = av_malloc(snd->size);
        if (!buf)
            return AVERROR(ENOMEM);
        avio_seek(pb, snd->data, SEEK_SET);
        if (avio_read(pb, buf, snd->size) != snd->size) {
            av_free(buf);
            return AVERROR_INVALIDDATA;
        }
        snd->silent = 1;
        if (codec == 2) {
            snd->nb_samples = snd->size;
            for (int i = 0; i < snd->size; i++)
                if (buf[i] != 0x00 && buf[i] != 0x80) {
                    snd->silent = 0;
                    break;
                }
        } else {
            snd->nb_samples = ff_cfdf_v40_movie_count(buf, snd->size);
            if (snd->nb_samples < 0) {
                av_free(buf);
                return snd->nb_samples;
            }
            snd->silent = snd->size > 0 && buf[0] == 0x40;
            for (int i = 1; snd->silent && i < snd->size; i++)
                if (buf[i] != 0x40 && buf[i] < 0xC0) {
                    snd->silent = 0;
                    break;
                }
        }
        av_free(buf);
    }
    if (!snd->nb_samples)
        return AVERROR_INVALIDDATA;

    return 0;
}

/* Proto-era movie sounds contain a native-endian two-byte duration followed
 * by the same v4.0 ADPCM control stream used by later movies. The original
 * assets use 370 decoded samples per authored tick and carry no sample rate.
 * The players supply 22050 Hz on Windows and 0x56EE8BA3 (about 22254.545 Hz)
 * on Macintosh, represented here by the nearest integer rate. */
static int mov_parse_proto_sound(AVFormatContext *s, int containers,
                                 const int64_t *coffs, int64_t fsize,
                                 int64_t idx, CFDFMovSound *snd)
{
    AVIOContext *pb = s->pb;
    uint8_t *buf;
    uint32_t csize;
    int samples, ticks, duration;
    CFDFDemuxContext *ctx = s->priv_data;

    if (!mov_valid_cont(containers, coffs, fsize, idx, 3))
        return AVERROR_INVALIDDATA;

    avio_seek(pb, coffs[idx] + 4, SEEK_SET);
    csize = cfdf_r32(s);
    if (csize <= 2 || csize > INT32_MAX || csize > fsize - coffs[idx] - 8)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, coffs[idx] + 8, SEEK_SET);
    duration = cfdf_r16(s);

    buf = av_malloc(csize - 2);
    if (!buf)
        return AVERROR(ENOMEM);
    if (avio_read(pb, buf, csize - 2) != csize - 2) {
        av_free(buf);
        return AVERROR_INVALIDDATA;
    }

    samples = ff_cfdf_v40_movie_count(buf, csize - 2);
    if (samples <= 0 || samples % 370) {
        av_free(buf);
        return AVERROR_INVALIDDATA;
    }
    ticks = samples / 370;
    /* Some resources retain the opposite byte order in this field. Accept
     * it only when the complete bitstream independently verifies the value. */
    if (ticks != duration && ticks != ((duration >> 8) | ((duration & 255) << 8))) {
        av_free(buf);
        return AVERROR_INVALIDDATA;
    }

    snd->data       = coffs[idx] + 10;
    snd->size       = csize - 2;
    snd->nb_samples = samples;
    snd->codec      = 1;
    snd->rate       = ctx->big_endian ? 22255 : 22050;
    snd->silent     = snd->size > 0 && buf[0] == 0x40;
    for (int i = 1; snd->silent && i < snd->size; i++)
        if (buf[i] != 0x40 && buf[i] < 0xc0)
            snd->silent = 0;

    av_free(buf);
    return 0;
}


/* Proto and revision-1 movies use the same STEP frames and 60 Hz scheduler
 * as v4, but keep an 0x50-byte frame table in a smaller control record.
 * Normalize those records into the v4 packet/audio timeline contract. */
static int mov_read_early_header(AVFormatContext *s, int containers,
                                 const int64_t *coffs, int64_t fsize,
                                 enum CFDFMovLayout layout)
{
    CFDFDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    CFDFMovBlock *vblocks = NULL;
    CFDFMovSFX *sfx = NULL;
    CFDFMovPlaylist pls[MOV_MAX_SEGMENTS] = { 0 };
    uint8_t *used = NULL;
    struct {
        int64_t base;
        int audio_a;
        int audio_b;
        int proto;
    } dirs[MOV_MAX_SEGMENTS];
    int64_t visited[MOV_MAX_SEGMENTS];
    int64_t ticks = 0, base = 0, active_sfx_end = 0;
    int nvb = 0, vblocks_alloc = 0, nb_sfx = 0, sfx_alloc = 0;
    int npls = 0, ndirs = 0, nvis = 0, W0 = 0, H0 = 0, ret = 0;

    ctx->movie = 1;
    used = av_calloc(containers, 1);
    if (!used)
        return AVERROR(ENOMEM);

    for (;;) {
        int64_t c0, hdr, frame_off, palette_off, seg_ticks0;
        uint32_t csize0, count, next = 0;
        int default_ticks, W, H, audio_a = 0, audio_b = 0;
        int playlist_words = 0, pal_idx, seg_first = 1, dup = 0;
        uint8_t *pal;

        for (int i = 0; i < nvis; i++)
            if (visited[i] == base)
                dup = 1;
        if (dup || nvis >= MOV_MAX_SEGMENTS) {
            av_log(s, AV_LOG_WARNING,
                   "early movie chain truncated (loop or too long)\n");
            break;
        }
        visited[nvis++] = base;

        if (!mov_valid_cont(containers, coffs, fsize, base, 0x8a2)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        c0  = coffs[base];
        hdr = c0 + 8;
        avio_seek(pb, c0 + 4, SEEK_SET);
        csize0 = cfdf_r32(s);

        if (layout == CFDF_MOV_PROTO) {
            avio_seek(pb, hdr, SEEK_SET);
            count = cfdf_r16(s);
            audio_a = cfdf_r16(s);
            audio_b = cfdf_r16(s);
            avio_seek(pb, hdr + 0x0a, SEEK_SET);
            H = cfdf_r16(s);
            W = cfdf_r16(s);
            default_ticks = cfdf_r32(s);
            avio_seek(pb, hdr + 0x1c, SEEK_SET);
            playlist_words = cfdf_r16(s);
            palette_off = 0x22;
            frame_off   = 0x8a2;
        } else {
            avio_seek(pb, hdr + 2, SEEK_SET);
            if (cfdf_r16(s) != 1) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            avio_seek(pb, hdr + 0x18, SEEK_SET);
            count   = cfdf_r16(s);
            audio_a = cfdf_r16(s);
            audio_b = cfdf_r16(s);
            avio_seek(pb, hdr + 0x22, SEEK_SET);
            H = cfdf_r16(s);
            W = cfdf_r16(s);
            default_ticks = cfdf_r32(s);
            avio_seek(pb, hdr + 0x34, SEEK_SET);
            playlist_words = cfdf_r16(s);
            next = cfdf_r32(s);
            palette_off = 0x3e;
            frame_off   = 0x8c2;

        }

        if (base + 1 + audio_a + audio_b > containers ||
            playlist_words > 64) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        dirs[ndirs].base    = base;
        dirs[ndirs].audio_a = audio_a;
        dirs[ndirs].audio_b = audio_b;
        dirs[ndirs].proto   = layout == CFDF_MOV_PROTO;
        ndirs++;

        if (default_ticks < 0 || default_ticks > 0xffff)
            default_ticks = 0;
        if (W <= 0 || H <= 0 || W > 2048 || H > 2048 ||
            count == 0 || count > INT16_MAX ||
            frame_off + (int64_t)count * 0x50 > csize0 ||
            hdr + frame_off + (int64_t)count * 0x50 > fsize) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        W0 = FFMAX(W0, W);
        H0 = FFMAX(H0, H);

        pal = av_realloc(ctx->palettes,
                         (ctx->nb_palettes + 1) * (size_t)AVPALETTE_SIZE);
        if (!pal) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        ctx->palettes = pal;
        pal += (size_t)ctx->nb_palettes * AVPALETTE_SIZE;
        avio_seek(pb, hdr + palette_off, SEEK_SET);
        for (int i = 0; i < 256; i++) {
            uint32_t r, g, b;

            avio_skip(pb, 2);
            r = cfdf_r16(s) >> 8;
            g = cfdf_r16(s) >> 8;
            b = cfdf_r16(s) >> 8;
            AV_WL32(pal + i * 4,
                    0xff000000u | (r << 16) | (g << 8) | b);
        }
        pal_idx = ctx->nb_palettes++;
        seg_ticks0 = ticks;

        for (uint32_t i = 0; i < count; i++) {
            int64_t e = hdr + frame_off + (int64_t)i * 0x50;
            int64_t ii, ci = -1;
            CFDFMovBlock *nvbk;
            CFDFMovSFX *nsfx;
            CFDFMovSound snd;
            int32_t img, sound;
            int flags, per, dur;
            uint32_t isz;

            avio_seek(pb, e + 2, SEEK_SET);
            per = cfdf_r32(s);
            if (per < 0 || per > 0xffff)
                per = 0;
            dur = FFMAX(default_ticks, per);
            if (dur < 1)
                dur = 1;
            avio_seek(pb, e + 0x1a, SEEK_SET);
            flags = cfdf_r16(s);
            img   = cfdf_r32(s);
            sound = cfdf_r32(s);

            ii = base + img;
            if (!mov_valid_cont(containers, coffs, fsize, ii, 4)) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            avio_seek(pb, coffs[ii] + 4, SEEK_SET);
            isz = cfdf_r32(s);
            if (isz < 4 || isz > INT32_MAX ||
                isz > fsize - coffs[ii] - 8) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }

            if (nvb == vblocks_alloc) {
                int new_alloc = vblocks_alloc ? vblocks_alloc * 2 : 256;

                if (vblocks_alloc > INT_MAX / 2) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                nvbk = av_realloc_array(vblocks, new_alloc,
                                         sizeof(*vblocks));
                if (!nvbk) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                vblocks = nvbk;
                vblocks_alloc = new_alloc;
            }
            vblocks[nvb].offset   = coffs[ii] + 8;
            vblocks[nvb].size     = isz;
            vblocks[nvb].duration = dur;
            vblocks[nvb].palette  = seg_first ? pal_idx : -1;
            vblocks[nvb].ry0      = 0;
            vblocks[nvb].rx0      = 0;
            vblocks[nvb].ry1      = H;
            vblocks[nvb].rx1      = W;
            vblocks[nvb].cw       = W;
            vblocks[nvb].ch       = H;
            vblocks[nvb].px       = 0;
            vblocks[nvb].py       = 0;
            nvb++;
            seg_first = 0;
            used[ii] = 1;

            if (sound > 0 && sound <= audio_a)
                ci = base + sound;
            else if (sound < 0)
                ci = (layout == CFDF_MOV_V1 ? base : 0) - (int64_t)sound;

            if (ci >= 0 && ci < containers &&
                !(layout == CFDF_MOV_PROTO ?
                  mov_parse_proto_sound(s, containers, coffs, fsize, ci, &snd) :
                  mov_parse_sound(s, containers, coffs, fsize, ci, &snd))) {
                if (nb_sfx == sfx_alloc) {
                    int new_alloc = sfx_alloc ? sfx_alloc * 2 : 64;

                    if (sfx_alloc > INT_MAX / 2) {
                        ret = AVERROR(ENOMEM);
                        goto fail;
                    }
                    nsfx = av_realloc_array(sfx, new_alloc, sizeof(*sfx));
                    if (!nsfx) {
                        ret = AVERROR(ENOMEM);
                        goto fail;
                    }
                    sfx = nsfx;
                    sfx_alloc = new_alloc;
                }
                sfx[nb_sfx].sound       = snd;
                sfx[nb_sfx].start_ticks = ticks;
                sfx[nb_sfx].loop        = 0;
                snprintf(sfx[nb_sfx].name, sizeof(sfx[nb_sfx].name),
                         "sound_%"PRId64, ci);
                active_sfx_end = ticks +
                    av_rescale_rnd(snd.nb_samples, 60, snd.rate, AV_ROUND_UP);
                nb_sfx++;
                used[ci] = 1;
            } else if (sound) {
                av_log(s, AV_LOG_WARNING,
                       "unresolved early movie sound selector %d at frame %u\n",
                       sound, i);
            }
            ticks += dur;
            if ((flags & 1) && active_sfx_end > ticks) {
                int64_t wait = active_sfx_end - ticks;

                if (wait > INT_MAX - vblocks[nvb - 1].duration) {
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }
                vblocks[nvb - 1].duration += wait;
                ticks = active_sfx_end;
            }
        }

        /* Proto stores only 1-based group-B selectors and loops to entry zero.
         * Revision 1 stores the same selector array plus a separate u32 loop
         * node at +0x8be. An empty record leaves the preceding playlist active. */
        if (playlist_words > 0 && audio_b > 0 &&
            npls < MOV_MAX_SEGMENTS) {
            int loop_start, nseq = 0, entries = playlist_words;
            int64_t playlist_off = hdr +
                (layout == CFDF_MOV_PROTO ? 0x822 : 0x83e);
            CFDFMovSound *seq = av_calloc(entries, sizeof(*seq));

            if (!seq) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            for (int i = 0; i < entries; i++) {
                int v;
                int64_t si;

                avio_seek(pb, playlist_off + (int64_t)i * 2, SEEK_SET);
                v  = cfdf_r16(s);
                si = base + audio_a + v;

                if (v < 1 || v > audio_b ||
                    (layout == CFDF_MOV_PROTO ?
                     mov_parse_proto_sound(s, containers, coffs, fsize, si,
                                           &seq[nseq]) :
                     mov_parse_sound(s, containers, coffs, fsize, si,
                                     &seq[nseq])))
                    continue;
                used[si] = 1;
                nseq++;
            }
            if (layout == CFDF_MOV_PROTO) {
                loop_start = 0;
            } else {
                uint32_t loop_node;

                avio_seek(pb, hdr + 0x8be, SEEK_SET);
                loop_node = cfdf_r32(s);
                loop_start = loop_node > INT_MAX ? INT_MAX : loop_node;
            }
            if (nseq > 0) {
                int finite = loop_start >= nseq;

                if (loop_start < 0)
                    loop_start = 0;
                if (loop_start >= nseq)
                    loop_start = nseq - 1;
                pls[npls].seq         = seq;
                pls[npls].nseq        = nseq;
                pls[npls].loop_start  = loop_start;
                pls[npls].finite      = finite;
                pls[npls].disk        = 0;
                pls[npls].start_ticks = seg_ticks0;
                npls++;
            } else {
                av_free(seq);
            }
        }

        if (layout == CFDF_MOV_PROTO || !next)
            break;
        if (next >= containers) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        base = next;
    }

    /* Early frame sounds occupy one replacement channel for the complete
     * authored movie. A control-chain boundary does not itself stop a sound;
     * a following trigger replaces it, while the final finite sound is allowed
     * to finish naturally after the picture timeline ends. */
    for (int i = 0; i < nb_sfx; i++) {
        int64_t end_ticks = i + 1 < nb_sfx ?
                                sfx[i + 1].start_ticks :
                                sfx[i].start_ticks +
                                av_rescale_rnd(sfx[i].sound.nb_samples, 60,
                                               sfx[i].sound.rate, AV_ROUND_UP);

        ret = ff_cfdf_mov_add_sfx_stream(s, &sfx[i], end_ticks,
                                         &ctx->max_audio_end);
        if (ret < 0)
            goto fail;
    }
    av_freep(&sfx);
    nb_sfx = sfx_alloc = 0;

    if (nvb > 0) {
        AVStream *st = avformat_new_stream(s, NULL);
        CFDFMovStream *cs;
        int64_t total = 0;

        if (!st) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        cs = av_mallocz(sizeof(*cs));
        if (!cs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        st->priv_data = cs;
        cs->blocks    = vblocks;
        cs->nb_blocks = nvb;
        vblocks       = NULL;
        for (int i = 0; i < nvb; i++)
            total += cs->blocks[i].duration;

        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id   = AV_CODEC_ID_CFDF_VIDEO;
        st->codecpar->format     = AV_PIX_FMT_PAL8;
        st->codecpar->width      = W0;
        st->codecpar->height     = H0;
        st->duration             = total;
        st->nb_frames            = nvb;
        st->codecpar->extradata =
            av_mallocz(AVPALETTE_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!st->codecpar->extradata) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        memcpy(st->codecpar->extradata, ctx->palettes, AVPALETTE_SIZE);
        st->codecpar->extradata_size = AVPALETTE_SIZE;
        avpriv_set_pts_info(st, 64, 1, 60);
        if (total > 0)
            av_reduce(&st->avg_frame_rate.num, &st->avg_frame_rate.den,
                      (int64_t)nvb * 60, total, INT_MAX);
    }

    ret = ff_cfdf_mov_schedule_playlists(s, pls, npls, ticks,
                                         &ctx->max_audio_end);
    if (ret < 0)
        goto fail;

    /* Preserve unreferenced grouped audio as optional director material without
     * placing it on the linear render timeline. */
    for (int d = 0; d < ndirs; d++) {
        int count = dirs[d].audio_a + dirs[d].audio_b;

        for (int i = 0; i < count; i++) {
            int64_t ci = dirs[d].base + 1 + i;
            CFDFMovSound snd;

            if (ci < 0 || ci >= containers || used[ci] ||
                (dirs[d].proto ?
                 mov_parse_proto_sound(s, containers, coffs, fsize, ci, &snd) :
                 mov_parse_sound(s, containers, coffs, fsize, ci, &snd)))
                continue;
            ret = ff_cfdf_mov_add_audio_stream(s, &snd, 1, 0, NULL,
                                               "untimed",
                                               &ctx->max_audio_end);
            if (ret < 0)
                goto fail;
            used[ci] = 1;
        }
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *best = NULL;

        if (s->streams[i]->codecpar->codec_type != AVMEDIA_TYPE_AUDIO)
            continue;
        for (int j = 0; j < s->nb_streams; j++) {
            AVStream *st = s->streams[j];
            const AVDictionaryEntry *t =
                av_dict_get(st->metadata, "timeline", NULL, 0);

            if (!t || strcmp(t->value, "background"))
                continue;
            if (!best || st->codecpar->sample_rate > best->codecpar->sample_rate ||
                (st->codecpar->sample_rate == best->codecpar->sample_rate &&
                 st->duration > best->duration))
                best = st;
        }
        if (best)
            best->disposition |= AV_DISPOSITION_DEFAULT;
        break;
    }

    av_dict_set(&s->metadata, "cfdf_movie_layout",
                layout == CFDF_MOV_PROTO ? "proto" : "revision_1", 0);
    if (s->nb_streams == 0)
        ret = AVERROR_INVALIDDATA;

fail:
    for (int i = 0; i < npls; i++)
        av_freep(&pls[i].seq);
    av_freep(&vblocks);
    av_freep(&sfx);
    av_freep(&used);
    return ret;
}

static int mov_read_header(AVFormatContext *s, int containers,
                           const int64_t *coffs, int64_t fsize)
{
    CFDFDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    CFDFMovBlock *vblocks = NULL;
    CFDFMovSFX *sfx = NULL;
    CFDFMovFrameInfo *frame_info = NULL;
    uint8_t *used = NULL;
    int nvb = 0, vblocks_alloc = 0, W0 = 0, H0 = 0;
    int scr_x0 = INT_MAX, scr_y0 = INT_MAX, scr_x1 = INT_MIN, scr_y1 = INT_MIN;
    int cue_events = 0, has_input_gate = 0, terminal_input_gate = 0, last_hold = 0;
    int64_t ticks = 0, vstart_ticks = 0, movie_end_ticks = 0;
    int64_t visited[MOV_MAX_SEGMENTS];
    struct { int64_t base, snd, names; } dirs[MOV_MAX_SEGMENTS];
    CFDFMovPlaylist pls[MOV_MAX_SEGMENTS];
    int nvis = 0, ndirs = 0, npls = 0, nb_sfx = 0, sfx_alloc = 0, ret = 0;
    int64_t base = 0;

    ctx->movie = 1;

    used = av_calloc(containers, 1);
    if (!used)
        return AVERROR(ENOMEM);

    for (;;) {
        int64_t c0, hdr, snd_i, names_i, cue_i, seg_ticks0, sfx_group_end = 0;
        int64_t gate_ticks = 0, gate_sfx_end = 0;
        int64_t loop_end_ticks = 0, loop_sfx_end = 0;
        uint32_t csize0, count, next;
        int default_ticks, W, H, wx, wy, pal_idx, seg_first, dup = 0;
        int gate_block = -1, sfx_group_loop = 0;
        int loop_start_block = -1, loop_end_block = -1, loop_sfx_count = 0;
        uint8_t *pal;

        for (int k = 0; k < nvis; k++)
            if (visited[k] == base)
                dup = 1;
        if (dup || nvis >= MOV_MAX_SEGMENTS) {
            av_log(s, AV_LOG_WARNING, "movie chain truncated (loop or too long)\n");
            break;
        }
        visited[nvis++] = base;

        if (!mov_valid_cont(containers, coffs, fsize, base, 0x884))
            break;
        c0  = coffs[base];
        hdr = c0 + 8;

        avio_seek(pb, hdr, SEEK_SET);
        if (cfdf_r32(s) != 0x00040000)
            break;
        avio_seek(pb, c0 + 0x04, SEEK_SET);
        csize0 = cfdf_r32(s);
        avio_seek(pb, hdr + 0x1c, SEEK_SET);
        default_ticks = cfdf_r32(s);
        if (default_ticks < 0 || default_ticks > 0xffff)
            default_ticks = 0;

        avio_seek(pb, hdr + 0x870, SEEK_SET);
        H = (int16_t)cfdf_r16(s);
        W = (int16_t)cfdf_r16(s);
        avio_seek(pb, hdr + 0x878, SEEK_SET);
        count = cfdf_r32(s);
        if (W <= 0 || H <= 0 || W > 2048 || H > 2048 ||
            count == 0 || count > INT16_MAX ||
            0x87c + (int64_t)count * 42 > csize0 ||
            hdr + 0x87c + (int64_t)count * 42 > fsize)
            break;

        frame_info = av_calloc(count, sizeof(*frame_info));
        if (!frame_info) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        for (uint32_t i = 0; i < count; i++)
            frame_info[i].block = -1;

        /* Window placement on the engine screen (header word at +0x24): the
         * chain's segments may have different geometry and sit at different
         * offsets. The output screen is the bounding box of all the windows;
         * blit offsets are resolved against it.
         * Absolute placements are stored per block for now. */
        avio_seek(pb, hdr + 0x24, SEEK_SET);
        wx = cfdf_r16(s);
        wy = cfdf_r16(s);
        scr_x0 = FFMIN(scr_x0, wx);
        scr_y0 = FFMIN(scr_y0, wy);
        scr_x1 = FFMAX(scr_x1, wx + W);
        scr_y1 = FFMAX(scr_y1, wy + H);

        /* per-segment global palette -> AVPALETTE (used as extradata and as
         * palette side data on the segment's first frame) */
        pal = av_realloc(ctx->palettes, (ctx->nb_palettes + 1) * (size_t)AVPALETTE_SIZE);
        if (!pal) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        ctx->palettes = pal;
        pal += (size_t)ctx->nb_palettes * AVPALETTE_SIZE;
        avio_seek(pb, hdr + 0x6c, SEEK_SET);
        for (int i = 0; i < 256; i++) {
            uint32_t r, g, b;
            avio_skip(pb, 2);
            r = cfdf_r16(s) >> 8;
            g = cfdf_r16(s) >> 8;
            b = cfdf_r16(s) >> 8;
            AV_WL32(pal + i * 4, 0xff000000u | (r << 16) | (g << 8) | b);
        }
        pal_idx = ctx->nb_palettes++;

        avio_seek(pb, hdr + 0x60, SEEK_SET);
        names_i = base + (int32_t)cfdf_r32(s);
        avio_seek(pb, hdr + 0x64, SEEK_SET);
        snd_i   = base + (int32_t)cfdf_r32(s);
        avio_seek(pb, hdr + 0x68, SEEK_SET);
        cue_i   = base + (int32_t)cfdf_r32(s);

        if (mov_valid_cont(containers, coffs, fsize, cue_i, 4)) {
            avio_seek(pb, coffs[cue_i] + 8, SEEK_SET);
            if ((int)cfdf_r32(s) > 0)
                cue_events = 1;
        }

        dirs[ndirs].base  = base;
        dirs[ndirs].snd   = snd_i;
        dirs[ndirs].names = names_i;
        ndirs++;

        seg_ticks0 = ticks;
        seg_first  = 1;

        for (uint32_t i = 0; i < count; i++) {
            int64_t e = hdr + 0x87c + (int64_t)i * 42, di, ii;
            char name[CFDF_MOV_NAME_SIZE + 1], jump_name[CFDF_MOV_NAME_SIZE + 1];
            int32_t cond, img, desc;
            int y0, x0, y1, x1, per = 0, dflags = 0, dtype = 0, dur;
            int input_gate, desc_valid = 0, jump_loop = 0, hotspots = -1;
            uint32_t dsize = 0;

            avio_seek(pb, e, SEEK_SET);
            cond = cfdf_r32(s);
            y0 = (int16_t)cfdf_r16(s);   /* dirty rect: top    (+4) */
            x0 = (int16_t)cfdf_r16(s);   /*             left   (+6) */
            y1 = (int16_t)cfdf_r16(s);   /*             bottom (+8) */
            x1 = (int16_t)cfdf_r16(s);   /*             right  (+10) */
            img = cfdf_r32(s);
            desc = cfdf_r32(s);

            name[0] = jump_name[0] = '\0';
            di = base + desc;
            if (mov_valid_cont(containers, coffs, fsize, di, 0x24)) {
                int64_t dp = coffs[di] + 8;

                desc_valid = 1;
                avio_seek(pb, coffs[di] + 4, SEEK_SET);
                dsize = cfdf_r32(s);
                avio_seek(pb, dp, SEEK_SET);
                dtype = cfdf_r16(s);
                avio_seek(pb, dp + 2, SEEK_SET);
                per = cfdf_r32(s);
                if (per < 0 || per > 0xffff)
                    per = 0;
                dflags = avio_r8(pb) & 0xff; /* dp + 6 */
                mov_read_pstring(pb, dp + 0x12, fsize, name, sizeof(name));
                if (dtype == 2 && dsize >= 0x33 &&
                    coffs[di] + 8 + dsize <= fsize) {
                    int jump_len;

                    avio_seek(pb, dp + 0x32, SEEK_SET);
                    jump_len = avio_r8(pb);
                    if (0x33 + jump_len <= dsize)
                        mov_read_pstring(pb, dp + 0x32, fsize, jump_name,
                                         sizeof(jump_name));
                }
                if (dsize >= 0x446 && coffs[di] + 8 + dsize <= fsize) {
                    avio_seek(pb, dp + 0x442, SEEK_SET);
                    hotspots = cfdf_r32(s);
                }
            }
            frame_info[i].action = dtype;
            frame_info[i].sfx = name[0] != '\0';

            /* The original runtime dispatches condition 2 only when
             * descriptor bit 0x04 is clear; when set, it bypasses condition
             * dispatch and returns to scheduler processing. */
            input_gate = desc_valid && cond == 2 && !(dflags & 0x04);
            if (gate_block >= 0 && cond != 1)
                gate_block = -1;

            dur = FFMAX(per, default_ticks);
            if (dur < 1)
                dur = 1;

            /* Named SFX trigger: fires when this frame is displayed. All
             * such triggers use engine group 0, replacing its prior sound. */
            if (name[0] &&
                mov_valid_cont(containers, coffs, fsize, names_i, 8)) {
                int64_t nd = coffs[names_i] + 8;
                uint32_t ndsize, nent;

                avio_seek(pb, coffs[names_i] + 0x04, SEEK_SET);
                ndsize = cfdf_r32(s);
                avio_seek(pb, nd + 4, SEEK_SET);
                nent = cfdf_r32(s);
                if (nent > 0 && nent <= INT16_MAX &&
                    8 + (int64_t)nent * 42 <= ndsize) {
                    for (uint32_t j = 0; j < nent; j++) {
                        int64_t ne = nd + 8 + (int64_t)j * 42, ci;
                        char sname[CFDF_MOV_NAME_SIZE + 1];
                        CFDFMovSound snd;
                        CFDFMovSFX *nsfx;
                        int loop;

                        mov_read_pstring(pb, ne + 10, fsize, sname, sizeof(sname));
                        if (!sname[0] || av_strcasecmp(sname, name))
                            continue;
                        avio_seek(pb, ne, SEEK_SET);
                        loop = avio_r8(pb) & 2;
                        avio_seek(pb, ne + 4, SEEK_SET);
                        ci = base + (int32_t)cfdf_r32(s);
                        if (!mov_parse_sound(s, containers, coffs, fsize, ci, &snd)) {
                            /* The replacement-channel RE is from the v4.0
                             * engine. Keep v4.1 scheduling unchanged until
                             * its corresponding call path is established. */
                            if (snd.codec == 2) {
                                ret = ff_cfdf_mov_add_audio_stream(s, &snd, 1,
                                        av_rescale(ticks, 1000000000, 60),
                                        sname, "sfx", &ctx->max_audio_end);
                                if (ret < 0)
                                    goto fail;
                                used[ci] = 1;
                                break;
                            }
                            if (nb_sfx == sfx_alloc) {
                                int new_alloc;

                                if (sfx_alloc > INT_MAX / 2) {
                                    ret = AVERROR(ENOMEM);
                                    goto fail;
                                }
                                new_alloc = sfx_alloc ? sfx_alloc * 2 : 64;
                                nsfx = av_realloc_array(sfx, new_alloc,
                                                       sizeof(*sfx));
                                if (!nsfx) {
                                    ret = AVERROR(ENOMEM);
                                    goto fail;
                                }
                                sfx = nsfx;
                                sfx_alloc = new_alloc;
                            }
                            sfx[nb_sfx].sound       = snd;
                            sfx[nb_sfx].start_ticks = ticks;
                            sfx[nb_sfx].loop        = loop;
                            av_strlcpy(sfx[nb_sfx].name, sname,
                                       sizeof(sfx[nb_sfx].name));
                            nb_sfx++;
                            sfx_group_end = ticks +
                                av_rescale_rnd(snd.nb_samples, 60, snd.rate,
                                               AV_ROUND_UP);
                            sfx_group_loop = loop != 0;
                            gate_block = -1;
                            used[ci] = 1;
                        }
                        break;
                    }
                }
            }

            /* image frame vs marker entry (markers have a 0x0 dirty rect and
             * display nothing; their ticks extend the previous frame) */
            ii = base + img;
            if ((y1 || x1) && mov_valid_cont(containers, coffs, fsize, ii, 4)) {
                uint32_t isz;
                avio_seek(pb, coffs[ii] + 0x04, SEEK_SET);
                isz = cfdf_r32(s);
                if (isz >= 4 && isz <= INT32_MAX && coffs[ii] + 8 + isz <= fsize) {
                    if (nvb == vblocks_alloc) {
                        int new_alloc;
                        CFDFMovBlock *nvbk;

                        if (vblocks_alloc > INT_MAX / 2) {
                            ret = AVERROR(ENOMEM);
                            goto fail;
                        }
                        new_alloc = vblocks_alloc ? vblocks_alloc * 2 : 256;
                        nvbk = av_realloc_array(vblocks, new_alloc,
                                                sizeof(*vblocks));
                        if (!nvbk) {
                            ret = AVERROR(ENOMEM);
                            goto fail;
                        }
                        vblocks = nvbk;
                        vblocks_alloc = new_alloc;
                    }
                    vblocks[nvb].offset   = coffs[ii] + 8;
                    vblocks[nvb].size     = isz;
                    vblocks[nvb].duration = dur;
                    vblocks[nvb].palette  = seg_first ? pal_idx : -1;
                    vblocks[nvb].ry0      = y0;
                    vblocks[nvb].rx0      = x0;
                    vblocks[nvb].ry1      = y1;
                    vblocks[nvb].rx1      = x1;
                    vblocks[nvb].cw       = W;
                    vblocks[nvb].ch       = H;
                    vblocks[nvb].px       = wx;
                    vblocks[nvb].py       = wy;
                    frame_info[i].block   = nvb;
                    nvb++;
                    seg_first = 0;
                    used[ii]  = 1;
                    ticks += dur;

                    /* Descriptor type 2 is the engine's named-frame jump.
                     * A backward jump with no interactive hotspots forms an
                     * authored animation loop. For finite linear output,
                     * repeat a side-effect-free loop while the finite group-0
                     * sound that entered it remains active. */
                    if (input_gate && dtype == 2 && !(dflags & 0x11) &&
                        !name[0] && jump_name[0] &&
                        hotspots == 0 && !sfx_group_loop &&
                        sfx_group_end > ticks && loop_start_block < 0) {
                        int target = -1, body_ok = 1;

                        for (uint32_t j = 0; j < i; j++) {
                            char frame_name[CFDF_MOV_NAME_SIZE + 1];

                            mov_read_pstring(pb,
                                             hdr + 0x87c + (int64_t)j * 42 + 26,
                                             fsize, frame_name,
                                             sizeof(frame_name));
                            if (frame_name[0] &&
                                !av_strcasecmp(frame_name, jump_name)) {
                                target = j;
                                break;
                            }
                        }
                        if (target >= 0 && frame_info[target].block >= 0) {
                            int64_t te = hdr + 0x87c + (int64_t)target * 42;
                            int ty0, tx0, ty1, tx1;

                            avio_seek(pb, te + 4, SEEK_SET);
                            ty0 = (int16_t)cfdf_r16(s);
                            tx0 = (int16_t)cfdf_r16(s);
                            ty1 = (int16_t)cfdf_r16(s);
                            tx1 = (int16_t)cfdf_r16(s);
                            if (ty0 || tx0 || ty1 != H || tx1 != W)
                                body_ok = 0;
                            for (uint32_t j = target; body_ok && j < i; j++)
                                if (frame_info[j].action != 6 ||
                                    frame_info[j].sfx)
                                    body_ok = 0;
                        } else {
                            body_ok = 0;
                        }
                        if (body_ok) {
                            loop_start_block = frame_info[target].block;
                            loop_end_block   = nvb - 1;
                            loop_end_ticks   = ticks;
                            loop_sfx_end     = sfx_group_end;
                            loop_sfx_count   = nb_sfx;
                            jump_loop        = 1;
                        }
                    }
                    /* Descriptor bit0/bit4 = pause the movie clock on this
                     * frame until the sounds triggered so far finish, then
                     * resume.
                     */
                    if (dflags & 0x11) {
                        int64_t hold_end = FFMAX(ctx->max_audio_end,
                                                 sfx_group_end);
                        if (hold_end > ticks) {
                            vblocks[nvb - 1].duration += hold_end - ticks;
                            ticks = hold_end;
                        }
                    }
                    if (input_gate && !jump_loop && !sfx_group_loop &&
                        sfx_group_end > ticks) {
                        gate_block   = nvb - 1;
                        gate_ticks   = ticks;
                        gate_sfx_end = sfx_group_end;
                    }
                    has_input_gate |= input_gate && !jump_loop;
                    last_hold = (dflags & 0x11) != 0;
                    terminal_input_gate = 0;
                    continue;
                }
            }
            if (input_gate) {
                has_input_gate = 1;
                terminal_input_gate = 1;
            }
            if (nvb > 0)
                vblocks[nvb - 1].duration += dur;
            else
                vstart_ticks += dur;
            ticks += dur;
            /* marker entries carry hold flags too: S001's final marker holds
             * the cut-to-black frame while the closing voice (triggered on
             * the previous frame) plays out; RFdemo holds on two markers */
            if (dflags & 0x11) {
                int64_t hold_end = FFMAX(ctx->max_audio_end, sfx_group_end);
                int64_t ext = hold_end - ticks;

                if (ext > 0) {
                    if (nvb > 0)
                        vblocks[nvb - 1].duration += ext;
                    else
                        vstart_ticks += ext;
                    ticks = hold_end;
                }
            }
            if (input_gate && nvb > 0 && !sfx_group_loop &&
                sfx_group_end > ticks) {
                gate_block   = nvb - 1;
                gate_ticks   = ticks;
                gate_sfx_end = sfx_group_end;
            }
        }

        if (loop_start_block >= 0) {
            nvb = loop_end_block + 1;
            nb_sfx = loop_sfx_count;
            ticks = loop_end_ticks;
            ret = ff_cfdf_mov_append_video_loop(&vblocks, &nvb,
                                        &vblocks_alloc, loop_start_block,
                                        loop_end_block, &ticks, loop_sfx_end);
            if (ret < 0)
                goto fail;
            gate_block = -1;
            last_hold = 0;
            terminal_input_gate = 0;
        } else if (gate_block >= 0 && gate_sfx_end > gate_ticks) {
            /* A terminal input gate may be followed by condition-1 teardown
             * frames. Let its active finite group-0 occurrence finish while
             * the gate remains open before those frames resume. */
            int64_t ext = gate_sfx_end - gate_ticks;

            if (ext > INT_MAX - vblocks[gate_block].duration) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            vblocks[gate_block].duration += ext;
            ticks += ext;
        }

        av_freep(&frame_info);

        /* Group 0 is stopped when a movie segment ends. Until then each
         * trigger is bounded by the following trigger; repeating entries fill
         * that interval, while ordinary entries remain one-shots. */
        for (int i = 0; i < nb_sfx; i++) {
            int64_t end_ticks = i + 1 < nb_sfx ?
                                    sfx[i + 1].start_ticks : ticks;

            ret = ff_cfdf_mov_add_sfx_stream(s, &sfx[i], end_ticks,
                                             &ctx->max_audio_end);
            if (ret < 0)
                goto fail;
        }
        av_freep(&sfx);
        nb_sfx = 0;
        sfx_alloc = 0;

        /* background-music playlist: 1-based indices at +6 into the record
         * list at +0x10e (stride 0x1a, chunk container index at +4). Chunks
         * play back to back from the segment start; the signed dword at +0 is
         * the zero-based entry where the playlist cycles after its first pass.
         * Collected here and scheduled after the chain walk, because a
         * playlist plays until the *next* non-empty playlist replaces it (an
         * empty playlist leaves the previous one running
         */
        if (mov_valid_cont(containers, coffs, fsize, snd_i, 0x10e) &&
            npls < MOV_MAX_SEGMENTS) {
            int64_t p1 = coffs[snd_i] + 8;
            uint32_t s1size;
            int plc, recc, loop_start;

            avio_seek(pb, coffs[snd_i] + 0x04, SEEK_SET);
            s1size = cfdf_r32(s);
            avio_seek(pb, p1, SEEK_SET);
            loop_start = (int32_t)cfdf_r32(s);
            avio_seek(pb, p1 + 4, SEEK_SET);
            plc = cfdf_r16(s);
            avio_seek(pb, p1 + 0x10a, SEEK_SET);
            recc = cfdf_r32(s);

            /* The playlist field holds at most 128 entries; when the record
             * list is longer the track is disk-streamed and the whole record
             * array plays in order (the same rule v5 uses for MTHM themes) */
            if (plc >= 0 && plc <= 128 && recc > 0 && recc <= INT16_MAX &&
                0x10e + (int64_t)recc * 0x1a <= s1size) {
                int disk = recc > plc;
                int nsrc = disk ? recc : plc;
                CFDFMovSound *seq = av_calloc(nsrc, sizeof(*seq));
                int nseq = 0;

                if (!seq) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                for (int k = 0; k < nsrc; k++) {
                    int64_t ci;
                    int v;

                    if (disk) {
                        v = k + 1;
                    } else {
                        avio_seek(pb, p1 + 6 + (int64_t)k * 2, SEEK_SET);
                        v = (int16_t)cfdf_r16(s);
                    }
                    if (v < 1 || v > recc)
                        continue;
                    avio_seek(pb, p1 + 0x10e + (int64_t)(v - 1) * 0x1a + 4, SEEK_SET);
                    ci = base + (int32_t)cfdf_r32(s);
                    if (mov_parse_sound(s, containers, coffs, fsize, ci, &seq[nseq]))
                        continue;
                    used[ci] = 1;
                    nseq++;
                }
                if (nseq > 0) {
                    int finite = loop_start >= recc;

                    if (loop_start < 0)
                        loop_start = 0;
                    if (loop_start >= recc)
                        loop_start = recc - 1;
                    pls[npls].seq         = seq;
                    pls[npls].nseq        = nseq;
                    pls[npls].start_ticks = seg_ticks0;
                    pls[npls].loop_start  = disk ? -1 :
                                              FFMIN(loop_start, nseq - 1);
                    pls[npls].finite      = !disk && finite;
                    pls[npls].disk        = disk;
                    npls++;
                } else {
                    av_freep(&seq);
                }
            }
        }

        if (loop_start_block >= 0)
            break;
        avio_seek(pb, hdr + 0x2c, SEEK_SET);
        next = cfdf_r32(s);
        if (!next)
            break;
        base = next;
    }

    /* Resolve the output screen: the bounding box of all segment windows.
     * Each block's placement was stored absolute; rebase it to the box
     * origin. Single-segment (and same-window chained) movies are unchanged
     * — the box is that one window. */
    if (nvb > 0 && scr_x1 > scr_x0 && scr_y1 > scr_y0) {
        W0 = FFMIN(scr_x1 - scr_x0, 2048);
        H0 = FFMIN(scr_y1 - scr_y0, 2048);
        for (int i = 0; i < nvb; i++) {
            vblocks[i].px -= scr_x0;
            vblocks[i].py -= scr_y0;
        }
    }

    /* remaining director sounds (never scheduled or triggered): untimed */
    for (int d = 0; d < ndirs; d++) {
        int64_t nd, p1;
        uint32_t ndsize, nent;
        int recc;

        if (mov_valid_cont(containers, coffs, fsize, dirs[d].names, 8)) {
            nd = coffs[dirs[d].names] + 8;
            avio_seek(pb, coffs[dirs[d].names] + 0x04, SEEK_SET);
            ndsize = cfdf_r32(s);
            avio_seek(pb, nd + 4, SEEK_SET);
            nent = cfdf_r32(s);
            if (nent > 0 && nent <= INT16_MAX &&
                8 + (int64_t)nent * 42 <= ndsize) {
                for (uint32_t j = 0; j < nent; j++) {
                    int64_t ne = nd + 8 + (int64_t)j * 42, ci;
                    char sname[CFDF_MOV_NAME_SIZE + 1];
                    CFDFMovSound snd;

                    avio_seek(pb, ne + 4, SEEK_SET);
                    ci = dirs[d].base + (int32_t)cfdf_r32(s);
                    if (ci < 0 || ci >= containers || used[ci])
                        continue;
                    mov_read_pstring(pb, ne + 10, fsize, sname, sizeof(sname));
                    if (mov_parse_sound(s, containers, coffs, fsize, ci, &snd))
                        continue;
                    ret = ff_cfdf_mov_add_audio_stream(s, &snd, 1, 0,
                                               sname[0] ? sname : NULL,
                                               "untimed",
                                               &ctx->max_audio_end);
                    if (ret < 0)
                        goto fail;
                    used[ci] = 1;
                }
            }
        }

        if (mov_valid_cont(containers, coffs, fsize, dirs[d].snd, 0x10e)) {
            uint32_t s1size;
            p1 = coffs[dirs[d].snd] + 8;
            avio_seek(pb, coffs[dirs[d].snd] + 0x04, SEEK_SET);
            s1size = cfdf_r32(s);
            avio_seek(pb, p1 + 0x10a, SEEK_SET);
            recc = cfdf_r32(s);
            if (recc > 0 && recc <= INT16_MAX &&
                0x10e + (int64_t)recc * 0x1a <= s1size) {
                for (int j = 0; j < recc; j++) {
                    int64_t ci;
                    CFDFMovSound snd;

                    avio_seek(pb, p1 + 0x10e + (int64_t)j * 0x1a + 4, SEEK_SET);
                    ci = dirs[d].base + (int32_t)cfdf_r32(s);
                    if (ci < 0 || ci >= containers || used[ci])
                        continue;
                    if (mov_parse_sound(s, containers, coffs, fsize, ci, &snd))
                        continue;
                    ret = ff_cfdf_mov_add_audio_stream(s, &snd, 1, 0, NULL,
                                                       "untimed",
                                                       &ctx->max_audio_end);
                    if (ret < 0)
                        goto fail;
                    used[ci] = 1;
                }
            }
        }
    }

    if (nvb > 0) {
        CFDFMovStream *cs;
        AVStream *st;
        int64_t total = 0;

        st = avformat_new_stream(s, NULL);
        if (!st) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        cs = av_mallocz(sizeof(*cs));
        if (!cs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        st->priv_data = cs;
        cs->blocks    = vblocks;
        cs->nb_blocks = nvb;
        cs->start_pts = vstart_ticks;
        cs->pts       = vstart_ticks;
        vblocks       = NULL;

        for (int i = 0; i < cs->nb_blocks; i++)
            total += cs->blocks[i].duration;

        /* Preserve the deterministic fallback for interactive credits: play
         * condition-2 slides at their authored rate, then hold the final one
         * through a lone soundtrack, or honor a final hold for a short
         * multi-chunk playlist. Disk-streamed tracks and unflagged multi-chunk
         * padding remain bounded by the picture. */
        {
            int64_t audio_hold = 0;

            for (int j = 0; j < npls; j++) {
                int64_t dur_t = 0;
                int allsilent = 1;

                if (pls[j].disk)
                    continue;
                for (int k = 0; k < pls[j].nseq; k++) {
                    dur_t += av_rescale(pls[j].seq[k].nb_samples, 60,
                                        pls[j].seq[k].rate);
                    if (!pls[j].seq[k].silent)
                        allsilent = 0;
                }
                if (allsilent)
                    continue;
                if ((pls[j].nseq == 1 && has_input_gate) || last_hold)
                    audio_hold = FFMAX(audio_hold, pls[j].start_ticks + dur_t);
            }
            if (audio_hold > vstart_ticks + total) {
                cs->blocks[cs->nb_blocks - 1].duration +=
                    audio_hold - (vstart_ticks + total);
                total = audio_hold - vstart_ticks;
            }
        }

        /* A terminal marker input gate leaves the last displayed image alive
         * while the original player waits for external input. A demuxer cannot
         * wait indefinitely, so for finite linear output finish only the
         * currently audible portion of the active background occurrence. */
        if (terminal_input_gate) {
            int64_t end_ticks = vstart_ticks + total;

            for (int j = 0; j < npls; j++) {
                int64_t next_ticks = j + 1 < npls ? pls[j + 1].start_ticks
                                                   : INT64_MAX;
                int64_t audible_end;

                if (end_ticks <= pls[j].start_ticks || end_ticks >= next_ticks)
                    continue;
                audible_end = ff_cfdf_mov_playlist_audible_end(pls[j].seq,
                                                       pls[j].nseq,
                                                       pls[j].loop_start,
                                                       pls[j].finite, pls[j].disk,
                                                       j + 1 == npls,
                                                       pls[j].start_ticks, end_ticks);
                if (audible_end > end_ticks) {
                    cs->blocks[cs->nb_blocks - 1].duration +=
                        audible_end - end_ticks;
                    total += audible_end - end_ticks;
                    end_ticks = audible_end;
                }
            }
        }

        movie_end_ticks = vstart_ticks + total;

        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id   = AV_CODEC_ID_CFDF_VIDEO;
        st->codecpar->format     = AV_PIX_FMT_PAL8;
        st->codecpar->width      = W0;
        st->codecpar->height     = H0;
        st->start_time           = vstart_ticks;
        st->duration             = total;
        st->nb_frames            = cs->nb_blocks;

        st->codecpar->extradata = av_mallocz(AVPALETTE_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!st->codecpar->extradata) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        memcpy(st->codecpar->extradata, ctx->palettes, AVPALETTE_SIZE);
        st->codecpar->extradata_size = AVPALETTE_SIZE;

        avpriv_set_pts_info(st, 64, 1, 60);
        if (total > 0) {
            AVRational fr;
            av_reduce(&fr.num, &fr.den, (int64_t)cs->nb_blocks * 60, total, INT_MAX);
            st->avg_frame_rate = fr;
        }
    }

    ret = ff_cfdf_mov_schedule_playlists(s, pls, npls,
                                 movie_end_ticks > 0 ? movie_end_ticks : ticks,
                                 &ctx->max_audio_end);
    if (ret < 0)
        goto fail;

    /* Sounds live inside the movie: unless a hold (descriptor bit0/bit4,
     * applied in the walk above) keeps the picture up for them, the original program
     * tears the movie down when the last frame's ticks elapse and everything
     * still playing stops. Background playlists are authored longer than the
     * picture as safety padding, so scheduled audio is clamped at the movie end;
     * sample-accurate for v4.1 (byte = sample), whole opcodes for v4.0. */
    if (movie_end_ticks > 0) {
        for (int i = 0; i < s->nb_streams; i++) {
            AVStream *st = s->streams[i];
            CFDFMovStream *cs = st->priv_data;
            const AVDictionaryEntry *t = av_dict_get(st->metadata, "timeline", NULL, 0);
            int64_t clamp_end, acc;
            int v41, k;

            if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO || !cs ||
                !t || !strcmp(t->value, "untimed"))
                continue;
            clamp_end = av_rescale(movie_end_ticks, st->codecpar->sample_rate, 60);
            if (cs->start_pts + st->duration <= clamp_end)
                continue;

            {
                const AVDictionaryEntry *title =
                    av_dict_get(st->metadata, "title", NULL, 0);
                double over = (cs->start_pts + st->duration - clamp_end) /
                              (double)st->codecpar->sample_rate;

                /* a substantially cut *triggered* sound is the signature of
                 * an interactive movie: the engine holds the picture for
                 * user input while the line plays out, pacing a demuxer
                 * cannot know; disclose that instead of trimming silently */
                if (!strcmp(t->value, "sfx") && over > 0.5)
                    av_log(s, AV_LOG_WARNING,
                           "sound '%s' runs %.2fs past the movie end (%.2fs); "
                           "trimming at the picture's end. If this movie is "
                           "interactive the engine holds for input while the "
                           "full sound plays - user-driven pacing is not "
                           "represented\n",
                           title ? title->value : "", over,
                           movie_end_ticks / 60.0);
                else
                    av_log(s, AV_LOG_INFO,
                           "audio '%s' runs %.2fs past the movie end (%.2fs) "
                           "and the final frame does not wait for sound; "
                           "trimming as the engine's teardown does\n",
                           title ? title->value : "", over,
                           movie_end_ticks / 60.0);
            }

            v41 = st->codecpar->codec_id == AV_CODEC_ID_CFDF_DPCM;
            acc = cs->start_pts;
            for (k = 0; k < cs->nb_blocks; k++) {
                CFDFMovBlock *b = &cs->blocks[k];
                int64_t room = clamp_end - acc;

                if (room <= 0)
                    break;
                if (b->duration > room) {          /* boundary block: trim it */
                    if (v41) {
                        b->size     = (int32_t)room;
                        b->duration = (int32_t)room;
                    } else {
                        uint8_t *buf = av_malloc(b->size);
                        int64_t got = 0;

                        if (!buf) {
                            ret = AVERROR(ENOMEM);
                            goto fail;
                        }
                        avio_seek(pb, b->offset, SEEK_SET);
                        ret = avio_read(pb, buf, b->size);
                        if (ret != b->size) {
                            av_free(buf);
                            ret = ret < 0 ? ret : AVERROR_INVALIDDATA;
                            goto fail;
                        }
                        ret = ff_cfdf_v40_movie_prefix(buf, b->size, room, &got);
                        av_free(buf);
                        if (ret < 0)
                            goto fail;
                        b->size = ret;
                        b->duration = (int32_t)got;
                    }
                    if (b->duration > 0) {
                        acc += b->duration;
                        k++;
                    }
                    break;
                }
                acc += b->duration;
            }
            cs->nb_blocks = k;
            st->duration  = acc - cs->start_pts;
        }
    }

    /* default audio = the best background track: highest sample rate first
     * (silence fillers ride at low rates), then the longest one */
    {
        AVStream *best = NULL;

        for (int i = 0; i < s->nb_streams; i++) {
            AVStream *st = s->streams[i];
            const AVDictionaryEntry *t = av_dict_get(st->metadata, "timeline", NULL, 0);

            if (!t || strcmp(t->value, "background"))
                continue;
            if (!best ||
                st->codecpar->sample_rate > best->codecpar->sample_rate ||
                (st->codecpar->sample_rate == best->codecpar->sample_rate &&
                 st->duration > best->duration))
                best = st;
        }
        if (best)
            best->disposition |= AV_DISPOSITION_DEFAULT;
    }

    if (cue_events)
        av_log(s, AV_LOG_WARNING, "timed cue events present; "
               "they are not represented in the demuxed timeline\n");

    if (s->nb_streams == 0) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }
    ret = 0;

fail:
    for (int j = 0; j < npls; j++)
        av_freep(&pls[j].seq);
    av_freep(&vblocks);
    av_freep(&sfx);
    av_freep(&frame_info);
    av_freep(&used);
    return ret;
}

static int read_header(AVFormatContext *s)
{
    CFDFDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    uint8_t header[0x400];
    int containers;

    if (avio_read(pb, header, sizeof(header)) != sizeof(header))
        return AVERROR_INVALIDDATA;
    ctx->big_endian = !memcmp(header + 0x20, "MOVEDFME", 8) ||
                      !memcmp(header + 0x20, "SONGDFST", 8) ||
                      (AV_RB32(header) == 0x10000 && AV_RL32(header) != 0x10000);
    avio_seek(pb, 0, SEEK_SET);
    avio_skip(pb, 0x14);
    containers = cfdf_r32(s);
    if (containers <= 0 || containers > INT16_MAX)
        return AVERROR_INVALIDDATA;

    /* Movie revisions share the container table but use three control layouts:
     * proto (no LPPALPPA), revision 1, and the later revision 4 structure. */
    {
        int64_t fsize = avio_size(pb);
        int64_t *coffs = av_calloc(containers, sizeof(*coffs));
        enum CFDFMovLayout movie = CFDF_MOV_NONE;
        int lppalppa;

        if (!coffs)
            return AVERROR(ENOMEM);
        if (fsize <= 0)
            fsize = INT64_MAX;

        avio_seek(pb, 0x400, SEEK_SET);
        for (int i = 0; i < containers; i++)
            coffs[i] = cfdf_r32(s);

        avio_seek(pb, 0x20, SEEK_SET);
        lppalppa = avio_rb32(pb) == MKBETAG('L','P','P','A') &&
                    avio_rb32(pb) == MKBETAG('L','P','P','A');

        if (mov_valid_cont(containers, coffs, fsize, 0, 0x884)) {
            int64_t c0 = coffs[0];
            uint32_t ver, csize0, count;
            int mw, mh;

            avio_seek(pb, c0 + 0x04, SEEK_SET);
            csize0 = cfdf_r32(s);
            avio_seek(pb, c0 + 8, SEEK_SET);
            ver = cfdf_r32(s);
            avio_seek(pb, c0 + 8 + 0x870, SEEK_SET);
            mh = (int16_t)cfdf_r16(s);
            mw = (int16_t)cfdf_r16(s);
            avio_seek(pb, c0 + 8 + 0x878, SEEK_SET);
            count = cfdf_r32(s);
            /* .trk container 0 can also start with 0x00040000 (it is a small
             * theme container there), so demand the full movie-header shape */
            if (lppalppa && ver == 0x00040000 &&
                count > 0 && count <= INT16_MAX &&
                0x87c + (int64_t)count * 42 <= csize0 &&
                mw > 0 && mh > 0 && mw <= 2048 && mh <= 2048)
                movie = CFDF_MOV_V4;
        }

        if (movie == CFDF_MOV_NONE && (lppalppa || ctx->big_endian) &&
            mov_valid_cont(containers, coffs, fsize, 0, 0x8c2)) {
            int64_t c0 = coffs[0], hdr = c0 + 8;
            uint32_t csize0, count;
            int revision, mw, mh;

            avio_seek(pb, c0 + 4, SEEK_SET);
            csize0 = cfdf_r32(s);
            avio_seek(pb, hdr + 2, SEEK_SET);
            revision = cfdf_r16(s);
            avio_seek(pb, hdr + 0x18, SEEK_SET);
            count = cfdf_r16(s);
            avio_seek(pb, hdr + 0x22, SEEK_SET);
            mh = cfdf_r16(s);
            mw = cfdf_r16(s);
            if (revision == 1 && count > 0 && count <= INT16_MAX &&
                0x8c2 + (int64_t)count * 0x50 <= csize0 &&
                mw > 0 && mh > 0 && mw <= 2048 && mh <= 2048)
                movie = CFDF_MOV_V1;
        }

        if (movie == CFDF_MOV_NONE && !lppalppa &&
            mov_valid_cont(containers, coffs, fsize, 0, 0x8a2)) {
            int64_t c0 = coffs[0], hdr = c0 + 8;
            uint32_t csize0, count, declared_size;
            int mw, mh;

            avio_seek(pb, 4, SEEK_SET);
            declared_size = cfdf_r32(s);
            avio_seek(pb, c0 + 4, SEEK_SET);
            csize0 = cfdf_r32(s);
            avio_seek(pb, hdr, SEEK_SET);
            count = cfdf_r16(s);
            avio_seek(pb, hdr + 0x0a, SEEK_SET);
            mh = cfdf_r16(s);
            mw = cfdf_r16(s);
            if ((fsize == INT64_MAX || declared_size == fsize) &&
                count > 0 && count <= INT16_MAX &&
                0x8a2 + (int64_t)count * 0x50 <= csize0 &&
                mw > 0 && mh > 0 && mw <= 2048 && mh <= 2048)
                movie = CFDF_MOV_PROTO;
        }

        if (movie != CFDF_MOV_NONE) {
            int ret = movie == CFDF_MOV_V4 ?
                          mov_read_header(s, containers, coffs, fsize) :
                          mov_read_early_header(s, containers, coffs, fsize,
                                                movie);
            av_freep(&coffs);
            return ret;
        }
        av_freep(&coffs);
    }

    return ff_cfdf_bank_open(s, &ctx->bank, ctx->big_endian);
}

static int mov_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    CFDFDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    CFDFMovStream *cs;
    CFDFMovBlock *blk;
    AVStream *st;
    int best = -1, ret;
    double best_t = 0;

    for (int i = 0; i < s->nb_streams; i++) {
        CFDFMovStream *c = s->streams[i]->priv_data;
        double t;

        if (!c || c->block_idx >= c->nb_blocks)
            continue;
        t = c->pts * av_q2d(s->streams[i]->time_base);
        if (best < 0 || t < best_t) {
            best   = i;
            best_t = t;
        }
    }
    if (best < 0)
        return AVERROR_EOF;

    st  = s->streams[best];
    cs  = st->priv_data;
    blk = &cs->blocks[cs->block_idx];

    avio_seek(pb, blk->offset, SEEK_SET);
    if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        /* 16-byte prefix on each video frame: the dirty rect (y0,x0,y1,x1, in
         * segment-canvas coordinates), the segment decode-canvas size (w,h)
         * and the window placement (x,y) relative to segment 0; the decoder
         * decodes on the segment canvas and blits only the rect onto the
         * persistent screen at the window offset */
        if (blk->size < 0 || blk->size > INT_MAX - 16)
            return AVERROR_INVALIDDATA;
        if ((ret = av_new_packet(pkt, blk->size + 16)) < 0)
            return ret;
        AV_WL16(pkt->data +  0, blk->ry0);
        AV_WL16(pkt->data +  2, blk->rx0);
        AV_WL16(pkt->data +  4, blk->ry1);
        AV_WL16(pkt->data +  6, blk->rx1);
        AV_WL16(pkt->data +  8, blk->cw);
        AV_WL16(pkt->data + 10, blk->ch);
        AV_WL16(pkt->data + 12, blk->px);
        AV_WL16(pkt->data + 14, blk->py);
        ret = avio_read(pb, pkt->data + 16, blk->size);
        if (ret != blk->size) {
            av_packet_unref(pkt);
            return ret < 0 ? ret : AVERROR_INVALIDDATA;
        }
        if (ctx->big_endian && blk->size >= 4) {
            unsigned rows = AV_RB16(pkt->data + 16);
            unsigned width = AV_RB16(pkt->data + 18);
            AV_WL16(pkt->data + 16, rows);
            AV_WL16(pkt->data + 18, width);
        }
    } else {
        ret = av_get_packet(pb, pkt, blk->size);
        if (ret < 0)
            return ret;
    }

    pkt->stream_index = st->index;
    pkt->pos          = blk->offset;
    pkt->pts          = cs->pts;
    pkt->duration     = blk->duration;
    /* audio chunks reset the codec state (all keyframes); the video is
     * inter-coded with the first frame as its only keyframe */
    if (st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO || cs->block_idx == 0)
        pkt->flags |= AV_PKT_FLAG_KEY;

    if (blk->palette >= 0 && blk->palette < ctx->nb_palettes) {
        uint8_t *sd = av_packet_new_side_data(pkt, AV_PKT_DATA_PALETTE,
                                              AVPALETTE_SIZE);
        if (!sd)
            return AVERROR(ENOMEM);
        memcpy(sd, ctx->palettes + (size_t)blk->palette * AVPALETTE_SIZE,
               AVPALETTE_SIZE);
    }

    cs->pts += blk->duration;
    cs->block_idx++;
    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    CFDFDemuxContext *ctx = s->priv_data;

    return ctx->movie ? mov_read_packet(s, pkt) : ff_cfdf_bank_packet(s, ctx->bank, pkt);
}

/* Movie mode: rewind the video to its single keyframe (frame 0); audio blocks
 * are independent, so audio streams jump to the block containing the target. */
static int mov_read_seek(AVFormatContext *s, int stream_index, int64_t ts, int flags)
{
    double target;

    if (stream_index < 0)
        target = ts / (double)AV_TIME_BASE;
    else
        target = ts * av_q2d(s->streams[stream_index]->time_base);
    if (target < 0)
        target = 0;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        CFDFMovStream *cs = st->priv_data;

        if (!cs)
            continue;

        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            cs->block_idx = 0;
            cs->pts       = cs->start_pts;
        } else {
            double tb = av_q2d(st->time_base);
            int64_t acc = cs->start_pts;
            int k = 0;

            while (k < cs->nb_blocks &&
                   (acc + cs->blocks[k].duration) * tb <= target) {
                acc += cs->blocks[k].duration;
                k++;
            }
            cs->block_idx = k;
            cs->pts       = acc;
        }
    }
    return 0;
}

static int read_seek(AVFormatContext *s, int stream_index, int64_t ts, int flags)
{
    CFDFDemuxContext *ctx = s->priv_data;

    if (stream_index >= s->nb_streams)
        return AVERROR(EINVAL);
    return ctx->movie ? mov_read_seek(s, stream_index, ts, flags) :
                        ff_cfdf_bank_seek(s, ctx->bank, stream_index, ts);
}

static int read_close(AVFormatContext *s)
{
    CFDFDemuxContext *ctx = s->priv_data;

    if (ctx->movie) {
        for (int i = 0; i < s->nb_streams; i++) {
            CFDFMovStream *cs = s->streams[i]->priv_data;
            if (cs)
                av_freep(&cs->blocks);
        }
    }
    av_freep(&ctx->palettes);
    ff_cfdf_bank_close(&ctx->bank);
    return 0;
}

const FFInputFormat ff_cfdf_demuxer = {
    .p.name         = "cfdf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CFDF (Cyberflix DreamFactory)"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "trk,snd,sfx,11k,mov,move",
    .priv_data_size = sizeof(CFDFDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
