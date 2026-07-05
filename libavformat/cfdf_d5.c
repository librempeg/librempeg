/*
 * CyberFlix DreamFactory v5/D5 (cfdf_d5) demuxer
 * Copyright (c) 2026
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

/*
 * DreamFactory 5 (.move / .trak) movies. Reuses the v4 (cfdf) container
 * envelope: file size @0x04, container count @0x14, u32 container-offset table
 * @0x400. The magic @0x20 is 'MOVE''D5ME' (.move) or 'TRAK''D5ST' (.trak),
 * with each 4CC stored little-endian (so a big-endian read gives the natural
 * tag). Containers are self-describing via a 4CC at +0x0C: SOUN (audio), and
 * playlist/director containers MTHM/MSND (.move) or STHM/SSND (.trak).
 *
 * Audio: SOUN payload base H = container + 0x08. Codec selected by H+0x1a
 * (==1 -> v4.0 ADPCM), else H+0x18 (==0 -> v4.1 DPCM), else IMA. Rate @H+0x1c.
 * Block count @H+0x28; block-offset table @H+0x2c has block_count+1 entries
 * (input byte offsets relative to H); block k spans [table[k], table[k+1]).
 * Codec state resets every block, so each block becomes one packet.
 *
 * Streams exposed (FFmpeg has no subsong concept, so these are parallel
 * streams; stream 0 is the best default):
 * - A background track, when the theme provides one: the SOUN sequence
 *   stitched into one continuous stream (titled with the file name).
 *     .move: MTHM playlist. Disk-streamed (segment_count > order_count) plays
 *            the whole segment array and the fragments are not listed again;
 *            otherwise it follows the 1-based order list and every SOUN is also
 *            listed. Leading/trailing silent SOUNs are trimmed from the track.
 *     .trak: STHM order list (> 1 segment); every SOUN is also listed.
 * - Each individual SOUN, titled from the sound director / theme label
 *   (MSND/MTHM for .move, SSND/STHM for .trak) when available.
 *
 * Note: vgmstream marks the .trak background track as an end-to-end loop; the
 * FFmpeg demuxer model has no first-class loop points, so that is not carried.
 */

#include <limits.h>

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/pixfmt.h"
#include "libavutil/rational.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define DF_HEADER_SIZE  0x400
#define DF_MAX_CHUNKS   0x4000
#define DF_NAME_SIZE    256

enum {
    CFDF_D5_V40 = 0,
    CFDF_D5_V41 = 1,
    CFDF_D5_IMA = 2,
};

typedef struct CFDFD5Block {
    int64_t offset;
    int     size;
    int     nb_samples;
} CFDFD5Block;

typedef struct CFDFD5Stream {
    CFDFD5Block *blocks;
    int          nb_blocks;
    int          block_idx;
    int64_t      pts;
} CFDFD5Stream;

typedef struct CFDFD5DemuxContext {
    int cur_stream;
} CFDFD5DemuxContext;

static int read_probe(const AVProbeData *p)
{
    uint32_t m20, m24;

    if (p->buf_size < DF_HEADER_SIZE)
        return 0;

    m20 = AV_RB32(p->buf + 0x20);
    m24 = AV_RB32(p->buf + 0x24);
    if (!((m20 == MKTAG('M','O','V','E') && m24 == MKTAG('D','5','M','E')) ||
          (m20 == MKTAG('T','R','A','K') && m24 == MKTAG('D','5','S','T'))))
        return 0;

    if ((int)AV_RL32(p->buf + 0x14) <= 0 ||
        AV_RL32(p->buf + 0x14) > INT16_MAX)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int is_id_at(AVIOContext *pb, int64_t off, uint32_t id)
{
    avio_seek(pb, off, SEEK_SET);
    return avio_rb32(pb) == id;
}

/* Read a Pascal string (u8 length + chars) at off into dst, bounded by file size. */
static void read_pstring(AVIOContext *pb, int64_t off, int64_t fsize,
                         char *dst, int dst_size)
{
    int len, i;

    dst[0] = '\0';
    if (off < 0 || off >= fsize)
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

/* Count output samples of one block (sizes the packet pts/duration). */
static int count_block_samples(AVIOContext *pb, int variant, int64_t off, int size)
{
    int n, p;
    uint8_t *buf;

    if (size <= 0)
        return 0;
    if (variant == CFDF_D5_V41)
        return size;
    if (variant == CFDF_D5_IMA) {
        int step;
        if (size < 3)
            return 0;
        avio_seek(pb, off + 2, SEEK_SET);
        step = avio_r8(pb);
        return (step > 0x58) ? 0 : (1 + 2 * (size - 3));
    }

    /* v4.0: walk the control stream */
    buf = av_malloc(size);
    if (!buf)
        return AVERROR(ENOMEM);
    avio_seek(pb, off, SEEK_SET);
    if (avio_read(pb, buf, size) != size) {
        av_free(buf);
        return AVERROR_INVALIDDATA;
    }

    n = 0;
    p = 1; /* skip seed */
    while (p < size) {
        uint8_t c = buf[p++];
        if (!(c & 0x80))
            n += 1;
        else if (!(c & 0x40)) {
            int cnt = (c & 0x3f) + 1;
            n += 2 * cnt;
            p += cnt;
        } else
            n += (c & 0x3f) + 1;
    }
    av_free(buf);

    return n;
}

/* Parse a SOUN container's block table, appending blocks to *blocks. On the
 * first SOUN parsed for a stream, variant and rate are filled; later ones must
 * match (so a stitched track stays a single coherent stream). */
static int parse_soun(AVFormatContext *s, int64_t coff,
                      int *variant, int *rate,
                      CFDFD5Block **blocks, int *nb_blocks)
{
    AVIOContext *pb = s->pb;
    int64_t H = coff + 0x08;
    int sel18, sel1a, r, count, var;
    uint32_t cont_size;
    int64_t table;

    if (!is_id_at(pb, coff + 0x0C, MKTAG('S','O','U','N')))
        return AVERROR_INVALIDDATA;

    avio_seek(pb, coff + 0x04, SEEK_SET);
    cont_size = avio_rl32(pb);

    avio_seek(pb, H + 0x18, SEEK_SET);
    sel18 = avio_rl16(pb);
    avio_seek(pb, H + 0x1a, SEEK_SET);
    sel1a = avio_rl16(pb);
    avio_seek(pb, H + 0x1c, SEEK_SET);
    r = avio_rl32(pb);
    avio_seek(pb, H + 0x28, SEEK_SET);
    count = avio_rl32(pb);
    table = H + 0x2c;

    if (r != 11025 && r != 22050 && r != 44100)
        return AVERROR_INVALIDDATA;
    if (count <= 0 || count > DF_MAX_CHUNKS)
        return AVERROR_INVALIDDATA;
    /* the block-offset table has count+1 entries; the terminal entry marks the
     * true end of the last block (cont_size includes alignment padding) */
    if (table + (int64_t)(count + 1) * 4 > H + cont_size)
        return AVERROR_INVALIDDATA;

    var = (sel1a == 1) ? CFDF_D5_V40 :
          (sel18 == 0) ? CFDF_D5_V41 : CFDF_D5_IMA;

    if (*nb_blocks == 0) {
        *variant = var;
        *rate    = r;
    } else if (var != *variant || r != *rate) {
        return AVERROR_INVALIDDATA;
    }

    for (int k = 0; k < count; k++) {
        uint32_t rel, next_rel;
        CFDFD5Block *nb;
        int bsize, ns;

        avio_seek(pb, table + (int64_t)k * 4, SEEK_SET);
        rel = avio_rl32(pb);
        next_rel = avio_rl32(pb);
        if (next_rel < rel || next_rel > cont_size)
            return AVERROR_INVALIDDATA;

        bsize = (int)(next_rel - rel);
        if (bsize <= 0)
            continue;

        ns = count_block_samples(pb, var, H + rel, bsize);
        if (ns < 0)
            return ns;

        nb = av_realloc_array(*blocks, *nb_blocks + 1, sizeof(**blocks));
        if (!nb)
            return AVERROR(ENOMEM);
        *blocks = nb;
        nb[*nb_blocks].offset     = H + rel;
        nb[*nb_blocks].size       = bsize;
        nb[*nb_blocks].nb_samples = ns;
        (*nb_blocks)++;
    }

    return 0;
}

/* True if every block of a SOUN is silent. v4.1: all bytes 0x00/0x80; v4.0: all
 * bytes 0x40 or >= 0xC0 (Mode III repeats of the DC level). IMA: pattern
 * unknown, treated as non-silent so it is never trimmed. */
static int soun_is_silent(AVIOContext *pb, int64_t coff)
{
    int64_t H = coff + 0x08, table;
    int sel18, sel1a, var, count;
    uint32_t cont_size;

    if (!is_id_at(pb, coff + 0x0C, MKTAG('S','O','U','N')))
        return 0;
    avio_seek(pb, coff + 0x04, SEEK_SET);
    cont_size = avio_rl32(pb);
    avio_seek(pb, H + 0x18, SEEK_SET);
    sel18 = avio_rl16(pb);
    avio_seek(pb, H + 0x1a, SEEK_SET);
    sel1a = avio_rl16(pb);
    avio_seek(pb, H + 0x28, SEEK_SET);
    count = avio_rl32(pb);
    table = H + 0x2c;

    var = (sel1a == 1) ? CFDF_D5_V40 :
          (sel18 == 0) ? CFDF_D5_V41 : CFDF_D5_IMA;
    if (count <= 0 || count > DF_MAX_CHUNKS)
        return 0;

    for (int k = 0; k < count; k++) {
        uint32_t bs, be;
        int left;

        avio_seek(pb, table + (int64_t)k * 4, SEEK_SET);
        bs = avio_rl32(pb);
        be = avio_rl32(pb);
        if (be < bs || be > cont_size)
            return 0;
        left = (int)(be - bs);
        avio_seek(pb, H + bs, SEEK_SET);
        while (left-- > 0) {
            uint8_t b = avio_r8(pb);
            if (var == CFDF_D5_V41) {
                if (b != 0x00 && b != 0x80)
                    return 0;
            } else if (var == CFDF_D5_IMA) {
                if (b != 0x00) /* IMA silence: all-zero block */
                    return 0;
            } else { /* v4.0 */
                if (b != 0x40 && b < 0xC0)
                    return 0;
            }
        }
    }
    return 1;
}

/* Resolve a .move SOUN display name: (1) MTHM segment label, else (2) MSND
 * sound director (scene-relative: MHED resets the scene base). */
static void lookup_name_move(AVIOContext *pb, int64_t fsize, int containers,
                             const int64_t *coffs, int soun_id,
                             char *dst, int dst_size)
{
    int scene_base = 0;

    dst[0] = '\0';

    for (int i = 0; i < containers; i++) {
        uint32_t seg_count;
        if (coffs[i] <= 0 || !is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','T','H','M')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x222, SEEK_SET);
        seg_count = avio_rl32(pb);
        if (seg_count == 0 || seg_count > DF_MAX_CHUNKS)
            continue;
        for (uint32_t sg = 0; sg < seg_count; sg++) {
            int64_t seg = coffs[i] + 0x08 + 0x226 + (int64_t)sg * 0x22;
            avio_seek(pb, seg + 0x0c, SEEK_SET);
            if ((int)avio_rl32(pb) != soun_id)
                continue;
            read_pstring(pb, seg + 0x12, fsize, dst, dst_size);
            if (dst[0])
                return;
        }
    }

    for (int i = 0; i < containers; i++) {
        int entry_count;
        if (coffs[i] <= 0)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','H','E','D')))
            scene_base = i;
        if (!is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','S','N','D')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x18, SEEK_SET);
        entry_count = avio_rl32(pb);
        if (entry_count <= 0 || entry_count > DF_MAX_CHUNKS)
            continue;
        for (int k = 0; k < entry_count; k++) {
            int64_t e = coffs[i] + 0x08 + 0x28 + (int64_t)k * 0x30;
            avio_seek(pb, e + 0x02, SEEK_SET);
            if (scene_base + avio_rl16(pb) != soun_id)
                continue;
            read_pstring(pb, e + 0x08, fsize, dst, dst_size);
            return;
        }
    }
}

/* Resolve a .trak SOUN display name: (1) SSND sound director, else (2) STHM
 * theme segment label. Both use absolute SOUN ids. */
static void lookup_name_trak(AVIOContext *pb, int64_t fsize, int containers,
                             const int64_t *coffs, int soun_id,
                             char *dst, int dst_size)
{
    dst[0] = '\0';

    for (int i = 0; i < containers; i++) {
        int entry_count;
        if (coffs[i] <= 0 || !is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','S','N','D')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x1c, SEEK_SET);
        entry_count = avio_rl32(pb);
        if (entry_count <= 0 || entry_count > DF_MAX_CHUNKS)
            continue;
        for (int k = 0; k < entry_count; k++) {
            int64_t e = coffs[i] + 0x08 + 0x20 + (int64_t)k * 0x1a;
            avio_seek(pb, e + 0x04, SEEK_SET);
            if ((int)avio_rl32(pb) != soun_id)
                continue;
            read_pstring(pb, e + 0x0a, fsize, dst, dst_size);
            if (dst[0])
                return;
        }
    }

    for (int i = 0; i < containers; i++) {
        uint32_t seg_count;
        if (coffs[i] <= 0 || !is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','T','H','M')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x222, SEEK_SET);
        seg_count = avio_rl32(pb);
        if (seg_count == 0 || seg_count > DF_MAX_CHUNKS)
            continue;
        for (uint32_t sg = 0; sg < seg_count; sg++) {
            int64_t seg = coffs[i] + 0x08 + 0x22a + (int64_t)sg * 0x1a;
            avio_seek(pb, seg + 0x00, SEEK_SET);
            if ((int)avio_rl32(pb) != soun_id)
                continue;
            read_pstring(pb, seg + 0x06, fsize, dst, dst_size);
            if (dst[0])
                return;
        }
    }
}

static int valid_soun(AVIOContext *pb, int64_t fsize, int containers,
                      const int64_t *coffs, int soun_id)
{
    return soun_id >= 0 && soun_id < containers && coffs[soun_id] > 0 &&
           coffs[soun_id] + 0x30 <= fsize &&
           is_id_at(pb, coffs[soun_id] + 0x0C, MKTAG('S','O','U','N'));
}

/* .move MTHM playlist -> seg_souns[] (every segment), seq[] (playback order),
 * and disk flag. Returns 1 on a usable theme, 0 otherwise. */
static int read_theme_move(AVFormatContext *s, int containers, const int64_t *coffs,
                           int theme_id, int **seg_souns, int *seg_count,
                           int **seq, int *seq_count, int *disk)
{
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    int64_t theme_head = coffs[theme_id] + 0x08;
    int order_count, *segs = NULL, *sq = NULL;
    uint32_t sc;

    *seg_souns = NULL; *seg_count = 0; *seq = NULL; *seq_count = 0; *disk = 0;

    avio_seek(pb, theme_head + 0x1c, SEEK_SET);
    order_count = avio_rl16(pb);
    avio_seek(pb, theme_head + 0x222, SEEK_SET);
    sc = avio_rl32(pb);
    if (sc == 0 || sc > DF_MAX_CHUNKS || order_count < 0 || order_count > DF_MAX_CHUNKS)
        return 0;

    segs = av_malloc_array(sc, sizeof(*segs));
    if (!segs)
        return 0;
    for (uint32_t i = 0; i < sc; i++) {
        int sid;
        avio_seek(pb, theme_head + 0x226 + (int64_t)i * 0x22 + 0x0c, SEEK_SET);
        sid = avio_rl32(pb);
        if (!valid_soun(pb, fsize, containers, coffs, sid)) {
            av_free(segs);
            return 0;
        }
        segs[i] = sid;
    }

    *disk = ((uint32_t)order_count < sc);
    if (*disk) {
        sq = av_malloc_array(sc, sizeof(*sq));
        if (!sq) { av_free(segs); return 0; }
        for (uint32_t i = 0; i < sc; i++)
            sq[i] = segs[i];
        *seq_count = sc;
    } else {
        if (order_count <= 0) { av_free(segs); return 0; }
        sq = av_malloc_array(order_count, sizeof(*sq));
        if (!sq) { av_free(segs); return 0; }
        for (int i = 0; i < order_count; i++) {
            int e;
            avio_seek(pb, theme_head + 0x1e + (int64_t)i * 2, SEEK_SET);
            e = avio_rl16(pb);
            if (e < 1 || (uint32_t)e > sc) { av_free(sq); av_free(segs); return 0; }
            sq[i] = segs[e - 1];
        }
        *seq_count = order_count;
    }

    *seg_souns = segs;
    *seg_count = sc;
    *seq       = sq;
    return 1;
}

/* .trak STHM order list -> seq[] of absolute SOUN ids. Returns 1 on success. */
static int read_theme_trak(AVFormatContext *s, int containers, const int64_t *coffs,
                           int **seq, int *seq_count)
{
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);

    *seq = NULL; *seq_count = 0;

    for (int i = 0; i < containers; i++) {
        int64_t theme_head;
        int order_count, *sq;
        uint32_t sc;

        if (coffs[i] <= 0 || !is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','T','H','M')))
            continue;
        theme_head = coffs[i] + 0x08;
        avio_seek(pb, theme_head + 0x1c, SEEK_SET);
        order_count = avio_rl16(pb);
        avio_seek(pb, theme_head + 0x222, SEEK_SET);
        sc = avio_rl32(pb);
        if (order_count <= 0 || order_count > DF_MAX_CHUNKS || sc == 0 || sc > DF_MAX_CHUNKS)
            return 0;

        sq = av_malloc_array(order_count, sizeof(*sq));
        if (!sq)
            return 0;
        for (int o = 0; o < order_count; o++) {
            int e, sid;
            avio_seek(pb, theme_head + 0x1e + (int64_t)o * 2, SEEK_SET);
            e = avio_rl16(pb);
            if (e < 1 || (uint32_t)e > sc) { av_free(sq); return 0; }
            avio_seek(pb, theme_head + 0x22a + (int64_t)(e - 1) * 0x1a + 0x00, SEEK_SET);
            sid = avio_rl32(pb);
            if (!valid_soun(pb, fsize, containers, coffs, sid)) { av_free(sq); return 0; }
            sq[o] = sid;
        }
        *seq = sq;
        *seq_count = order_count;
        return 1;
    }
    return 0;
}

/* Scan the container table for STEP video frames and, if any, add a PAL8 video
 * stream emitting one packet per frame (payload base = container + 0x08, which
 * is the "this" structure the decoder parses). Returns 1 if a stream was added,
 * 0 if there is no video, <0 on fatal error. */
static int build_video_stream(AVFormatContext *s, int containers,
                              const int64_t *coffs, int64_t fsize)
{
    AVIOContext *pb = s->pb;
    CFDFD5Block *blocks = NULL;
    CFDFD5Stream *cs;
    AVStream *st;
    int nb = 0, width = 0, height = 0, default_ticks = 0;
    int64_t total = 0, last_mfrm = -1;

    /* VFR based timing. The engine paces frames against a 60 Hz clock
     * advancing by max(per-frame, default) ticks each frame.
     * The movie default is MHED payload +0x1c; the per-frame
     * override is the paired MFRM payload +0x1a (MFRM and STEP alternate in the
     * container table, so the MFRM right before a STEP is its descriptor). */
    for (int i = 0; i < containers; i++) {
        if (coffs[i] <= 0 || coffs[i] + 0x20 > fsize)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','H','E','D'))) {
            avio_seek(pb, coffs[i] + 0x08 + 0x1c, SEEK_SET);
            default_ticks = (int)avio_rl32(pb);
            break;
        }
    }
    if (default_ticks <= 0 || default_ticks > 0xffff)
        default_ticks = 4;   /* ~15 fps fallback when absent/implausible */

    for (int i = 0; i < containers; i++) {
        uint32_t csize;
        CFDFD5Block *nbk;
        int64_t base;
        int dur, mfrm_dur = 0;

        if (coffs[i] <= 0 || coffs[i] + 0x10 > fsize)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','F','R','M'))) {
            last_mfrm = coffs[i] + 0x08;   /* descriptor for the next STEP */
            continue;
        }
        if (coffs[i] + 0x428 > fsize ||
            !is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','T','E','P')))
            continue;

        avio_seek(pb, coffs[i] + 0x04, SEEK_SET);
        csize = avio_rl32(pb);
        base  = coffs[i] + 0x08;
        if (csize < 0x428 || base + csize > fsize)
            continue;

        if (nb == 0) {
            avio_seek(pb, base + 0x20, SEEK_SET);
            height = (int16_t)avio_rl16(pb);
            width  = (int16_t)avio_rl16(pb);
            if (width <= 0 || height <= 0) {
                av_freep(&blocks);
                return 0;
            }
        }

        if (last_mfrm > 0 && last_mfrm + 0x1e <= fsize) {
            avio_seek(pb, last_mfrm + 0x1a, SEEK_SET);
            mfrm_dur = (int)avio_rl32(pb);
            if (mfrm_dur < 0 || mfrm_dur > 0xffff)
                mfrm_dur = 0;   /* implausible -> fall back to the default */
        }
        dur = mfrm_dur > default_ticks ? mfrm_dur : default_ticks;

        nbk = av_realloc_array(blocks, nb + 1, sizeof(*blocks));
        if (!nbk) {
            av_freep(&blocks);
            return AVERROR(ENOMEM);
        }
        blocks = nbk;
        blocks[nb].offset     = base;
        blocks[nb].size       = csize;
        blocks[nb].nb_samples = dur;   /* frame duration in 1/60 s ticks */
        total += dur;
        nb++;
    }

    if (nb == 0)
        return 0;

    st = avformat_new_stream(s, NULL);
    if (!st) { av_freep(&blocks); return AVERROR(ENOMEM); }

    cs = av_mallocz(sizeof(*cs));
    if (!cs) { av_freep(&blocks); return AVERROR(ENOMEM); }
    st->priv_data = cs;
    cs->blocks    = blocks;
    cs->nb_blocks = nb;

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_CFDF_D5_VIDEO;
    st->codecpar->format     = AV_PIX_FMT_PAL8;
    st->codecpar->width      = width;
    st->codecpar->height     = height;
    st->start_time           = 0;
    st->duration             = total;
    st->nb_frames            = nb;

    /* 60 Hz tick: durations above are in 1/60 s units */
    avpriv_set_pts_info(st, 64, 1, 60);

    /* nominal average frame rate (VFR) = frames / seconds = nb*60 / total */
    if (total > 0) {
        AVRational fr;
        av_reduce(&fr.num, &fr.den, (int64_t)nb * 60, total, INT_MAX);
        st->avg_frame_rate = fr;
    }

    return 1;
}

static int add_stream(AVFormatContext *s, int variant, int rate,
                      CFDFD5Block *blocks, int nb_blocks, const char *title)
{
    CFDFD5Stream *cs;
    AVStream *st;
    int64_t total = 0;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    cs = av_mallocz(sizeof(*cs));
    if (!cs)
        return AVERROR(ENOMEM);
    st->priv_data = cs;
    cs->blocks    = blocks;
    cs->nb_blocks = nb_blocks;

    for (int i = 0; i < nb_blocks; i++)
        total += blocks[i].nb_samples;

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_CFDF_D5;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout   = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    st->start_time            = 0;
    st->duration              = total;

    st->codecpar->extradata = av_mallocz(1 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!st->codecpar->extradata)
        return AVERROR(ENOMEM);
    st->codecpar->extradata[0]   = variant;
    st->codecpar->extradata_size = 1;

    if (title && title[0])
        av_dict_set(&st->metadata, "title", title, 0);

    avpriv_set_pts_info(st, 64, 1, rate);

    return 0;
}

/* Add one stream for a single SOUN. Returns 1 if added, 0 if skipped (bad
 * data), <0 on fatal error. */
static int build_soun_stream(AVFormatContext *s, int64_t coff, const char *title)
{
    CFDFD5Block *blocks = NULL;
    int nb = 0, var = 0, rate = 0, ret;

    ret = parse_soun(s, coff, &var, &rate, &blocks, &nb);
    if (ret == AVERROR(ENOMEM)) {
        av_freep(&blocks);
        return ret;
    }
    if (ret < 0 || nb <= 0) {
        av_freep(&blocks);
        return 0;
    }
    ret = add_stream(s, var, rate, blocks, nb, title);
    if (ret < 0) {
        av_freep(&blocks);
        return ret;
    }
    return 1;
}

/* Add one stream stitching the SOUN sequence seq[0..count) in order. Returns 1
 * if added, 0 if no usable track, <0 on fatal error. */
static int build_track_stream(AVFormatContext *s, const int64_t *coffs,
                              const int *seq, int count, const char *title)
{
    CFDFD5Block *blocks = NULL;
    int nb = 0, var = 0, rate = 0, ret;

    for (int i = 0; i < count; i++) {
        ret = parse_soun(s, coffs[seq[i]], &var, &rate, &blocks, &nb);
        if (ret == AVERROR(ENOMEM)) {
            av_freep(&blocks);
            return ret;
        }
        if (ret < 0) {
            av_freep(&blocks);
            return 0;
        }
    }
    if (nb <= 0) {
        av_freep(&blocks);
        return 0;
    }
    ret = add_stream(s, var, rate, blocks, nb, title);
    if (ret < 0) {
        av_freep(&blocks);
        return ret;
    }
    return 1;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    int64_t *coffs = NULL;
    int *souns = NULL, soun_count = 0;
    int containers, is_trak, ret = 0;
    const char *basename;

    if (fsize <= 0)
        fsize = INT64_MAX;
    basename = av_basename(s->url);

    avio_seek(pb, 0x14, SEEK_SET);
    containers = avio_rl32(pb);
    if (containers <= 0 || containers > INT16_MAX)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x20, SEEK_SET);
    is_trak = (avio_rb32(pb) == MKTAG('T','R','A','K'));

    coffs = av_calloc(containers, sizeof(*coffs));
    souns = av_calloc(containers, sizeof(*souns));
    if (!coffs || !souns) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    avio_seek(pb, DF_HEADER_SIZE, SEEK_SET);
    for (int i = 0; i < containers; i++)
        coffs[i] = avio_rl32(pb);
    for (int i = 0; i < containers; i++) {
        if (coffs[i] <= 0 || coffs[i] + 0x30 > fsize)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','O','U','N')))
            souns[soun_count++] = i;
    }
    if (soun_count == 0)
        goto video;

    if (is_trak) {
        int *seq = NULL, seq_count = 0, has_track;

        has_track = read_theme_trak(s, containers, coffs, &seq, &seq_count) && seq_count > 1;
        if (has_track) {
            int lo = 0, hi = seq_count - 1;
            while (lo <= hi && soun_is_silent(pb, coffs[seq[lo]]))
                lo++;
            while (hi >= lo && soun_is_silent(pb, coffs[seq[hi]]))
                hi--;
            if (lo > hi) { lo = 0; hi = seq_count - 1; }
            ret = build_track_stream(s, coffs, seq + lo, hi - lo + 1, basename);
            if (ret < 0) { av_freep(&seq); goto end; }
        }
        av_freep(&seq);

        /* every SOUN is also listed individually (keep stitched chunks) */
        for (int j = 0; j < soun_count; j++) {
            char name[DF_NAME_SIZE];
            lookup_name_trak(pb, fsize, containers, coffs, souns[j], name, sizeof(name));
            ret = build_soun_stream(s, coffs[souns[j]], name[0] ? name : NULL);
            if (ret < 0)
                goto end;
        }
    } else {
        int *seg_souns = NULL, *seq = NULL, seg_count = 0, seq_count = 0, disk = 0;
        int theme_id = -1, has_track = 0;

        for (int i = 0; i < containers && theme_id < 0; i++) {
            uint32_t sc;
            if (coffs[i] <= 0 || !is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','T','H','M')))
                continue;
            avio_seek(pb, coffs[i] + 0x08 + 0x222, SEEK_SET);
            sc = avio_rl32(pb);
            if (sc > 0 && sc <= DF_MAX_CHUNKS)
                theme_id = i;
        }
        if (theme_id >= 0)
            has_track = read_theme_move(s, containers, coffs, theme_id,
                                        &seg_souns, &seg_count, &seq, &seq_count, &disk);

        if (has_track) {
            int lo = 0, hi = seq_count - 1;
            if (!disk) {
                while (lo <= hi && soun_is_silent(pb, coffs[seq[lo]]))
                    lo++;
                while (hi >= lo && soun_is_silent(pb, coffs[seq[hi]]))
                    hi--;
                if (lo > hi) { lo = 0; hi = seq_count - 1; }
            }
            ret = build_track_stream(s, coffs, seq + lo, hi - lo + 1, basename);
            if (ret < 0) { av_freep(&seg_souns); av_freep(&seq); goto end; }
        }

        for (int j = 0; j < soun_count; j++) {
            int sid = souns[j];
            char name[DF_NAME_SIZE];

            if (has_track && disk) { /* disk fragments are folded into the track */
                int frag = 0;
                for (int k = 0; k < seg_count; k++)
                    if (seg_souns[k] == sid) { frag = 1; break; }
                if (frag)
                    continue;
            }
            lookup_name_move(pb, fsize, containers, coffs, sid, name, sizeof(name));
            ret = build_soun_stream(s, coffs[sid], name[0] ? name : NULL);
            if (ret < 0) { av_freep(&seg_souns); av_freep(&seq); goto end; }
        }

        av_freep(&seg_souns);
        av_freep(&seq);
    }

video:
    ret = build_video_stream(s, containers, coffs, fsize);
    if (ret < 0)
        goto end;

    if (s->nb_streams == 0) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }
    ret = 0;

end:
    av_freep(&coffs);
    av_freep(&souns);
    return ret;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    CFDFD5Stream *cs;
    CFDFD5Block *blk;
    AVStream *st;
    int best = -1, ret;
    double best_t = 0;

    /* Interleave: emit the stream whose next block has the smallest presentation
     * time (compared across the streams' differing timebases). This keeps audio
     * and video roughly PTS-ordered so players stay in sync and seeking works. */
    for (int i = 0; i < s->nb_streams; i++) {
        CFDFD5Stream *c = s->streams[i]->priv_data;
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
    ret = av_get_packet(pb, pkt, blk->size);
    if (ret < 0)
        return ret;

    pkt->stream_index = st->index;
    pkt->pos          = blk->offset;
    pkt->pts          = cs->pts;
    pkt->duration     = blk->nb_samples;
    /* Audio blocks reset their codec state each block, so all are keyframes;
     * STEP video is inter-coded, so only the first frame is a keyframe. */
    if (st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO || cs->block_idx == 0)
        pkt->flags |= AV_PKT_FLAG_KEY;

    cs->pts += blk->nb_samples;
    cs->block_idx++;

    return 0;
}

static int read_seek(AVFormatContext *s, int stream_index, int64_t ts, int flags)
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
        CFDFD5Stream *cs = st->priv_data;

        if (!cs)
            continue;

        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            /* STEP video is inter-coded with a single keyframe (frame 0). Any
             * seek rewinds to it; the decoder rebuilds the reference frame
             * forward and the generic seek discards output until the target. */
            cs->block_idx = 0;
            cs->pts       = 0;
        } else {
            /* Audio blocks are independent (each a keyframe): jump to the block
             * containing the target time. */
            double tb = av_q2d(st->time_base);
            int64_t acc = 0;
            int k = 0;

            while (k < cs->nb_blocks &&
                   (acc + cs->blocks[k].nb_samples) * tb <= target) {
                acc += cs->blocks[k].nb_samples;
                k++;
            }
            cs->block_idx = k;
            cs->pts       = acc;
        }
    }

    return 0;
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        CFDFD5Stream *cs = s->streams[i]->priv_data;
        if (cs)
            av_freep(&cs->blocks);
    }
    return 0;
}

const FFInputFormat ff_cfdf_d5_demuxer = {
    .p.name         = "cfdf_d5",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CFDF D5 (Cyberflix DreamFactory v5)"),
    .p.extensions   = "move,trak",
    .priv_data_size = sizeof(CFDFD5DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};

