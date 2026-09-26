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
 * Streams expose timed themes, frame-triggered sounds, and untimed catalog
 * entries. Track playlists remain one scheduled background.
 * A separately tagged compatibility stream supports direct playback.
 */

#include <errno.h>
#include <limits.h>
#include <string.h>

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
    int64_t      start_pts;
    int64_t      pts;
} CFDFD5Stream;

typedef struct CFDFD5DemuxContext {
    int cur_stream;
} CFDFD5DemuxContext;

typedef struct CFDFD5MoveSound {
    int soun_id;
    int index;
    int flags;
    int pan;
} CFDFD5MoveSound;

typedef struct CFDFD5MoveFrame {
    int      scene_base;
    int      scene;
    int      mfrm_id;
    int      step_id;
    int64_t  scene_start;
    int64_t  scene_end;
    int64_t  pts;
    int      duration;
} CFDFD5MoveFrame;

typedef struct CFDFD5MoveTimeline {
    int64_t *container_pts;
    CFDFD5MoveFrame *frames;
    int      nb_frames;
    int64_t  total_ticks;
    int      hold_frames;
    int      max_hold_ticks;
} CFDFD5MoveTimeline;

#define CFDF_D5_MFRM_HOLD 0x01
#define CFDF_D5_MFRM_POOL_HOLD 0x10

static int read_probe(const AVProbeData *p)
{
    uint32_t m20, m24;

    if (p->buf_size < DF_HEADER_SIZE)
        return 0;

    m20 = AV_RB32(p->buf + 0x20);
    m24 = AV_RB32(p->buf + 0x24);
    if (!((m20 == MKTAG('M','O','V','E') &&
           (m24 == MKTAG('D','5','M','E') ||
            m24 == MKTAG('D','F','M','E'))) ||
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

static int is_d5_id_at(AVIOContext *pb, int64_t off, uint32_t id)
{
    avio_seek(pb, off + 0x08, SEEK_SET);
    return avio_rl32(pb) == 0x00050000 && avio_rb32(pb) == id;
}

/* All authored timing uses this frame duration. */
static int mfrm_duration_ticks(AVIOContext *pb, int64_t coff, int64_t fsize,
                               int default_ticks)
{
    int64_t H = coff + 0x08;
    uint32_t size, override = 0;

    if (default_ticks <= 0 || default_ticks > 0xffff)
        default_ticks = 4;
    if (coff <= 0 || coff + 0x10 > fsize)
        return default_ticks;

    avio_seek(pb, coff + 0x04, SEEK_SET);
    size = avio_rl32(pb);
    if (size >= 0x1e && H + size <= fsize) {
        avio_seek(pb, H + 0x1a, SEEK_SET);
        override = avio_rl32(pb);
        if (override > 0xffff)
            override = 0;
    }

    return FFMAX(default_ticks, (int)override);
}

/* Hold flags select completion of the fixed sound or both pooled sounds. */
static int mfrm_flags(AVIOContext *pb, int64_t coff, int64_t fsize)
{
    int64_t H = coff + 0x08;
    uint32_t size;

    if (coff <= 0 || coff + 0x10 > fsize)
        return 0;

    avio_seek(pb, coff + 0x04, SEEK_SET);
    size = avio_rl32(pb);
    if (size < 0x1f || H + size > fsize)
        return 0;

    avio_seek(pb, H + 0x1e, SEEK_SET);
    return avio_r8(pb);
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
        if (size < 4)
            return 0;
        avio_seek(pb, off + 2, SEEK_SET);
        step = avio_r8(pb);
        return (step > 0x58) ? 0 : 2 * (size - 3);
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

        if (coffs[i] <= 0)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','H','E','D'))) {
            scene_base = i;
            continue;
        }
        if (!is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','T','H','M')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x222, SEEK_SET);
        seg_count = avio_rl32(pb);
        if (seg_count == 0 || seg_count > DF_MAX_CHUNKS)
            continue;
        for (uint32_t sg = 0; sg < seg_count; sg++) {
            int64_t seg = coffs[i] + 0x08 + 0x226 + (int64_t)sg * 0x22;
            avio_seek(pb, seg + 0x0c, SEEK_SET);
            if ((int64_t)scene_base + avio_rl32(pb) != soun_id)
                continue;
            read_pstring(pb, seg + 0x12, fsize, dst, dst_size);
            if (dst[0])
                return;
        }
    }

    scene_base = 0;
    for (int i = 0; i < containers; i++) {
        int entry_count;
        if (coffs[i] <= 0)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','H','E','D')))
            scene_base = i;
        if (!is_id_at(pb, coffs[i] + 0x0C, MKTAG('M','S','N','D')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x1c, SEEK_SET);
        entry_count = avio_rl32(pb);
        if (entry_count <= 0 || entry_count > DF_MAX_CHUNKS)
            continue;
        for (int k = 0; k < entry_count; k++) {
            int64_t e = coffs[i] + 0x08 + 0x20 + (int64_t)k * 0x30;
            avio_seek(pb, e + 0x0a, SEEK_SET);
            if ((int64_t)scene_base + avio_rl32(pb) != soun_id)
                continue;
            read_pstring(pb, e + 0x10, fsize, dst, dst_size);
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

/* Scene resources are stored as directory ids relative to the MHED entry. */
static int move_scene_resource(AVIOContext *pb, int64_t fsize, int containers,
                               const int64_t *coffs, int scene_base,
                               int field, uint32_t id)
{
    int64_t resource;

    if (scene_base < 0 || scene_base >= containers ||
        coffs[scene_base] <= 0 ||
        coffs[scene_base] + 0x08 + field + 4 > fsize ||
        !is_id_at(pb, coffs[scene_base] + 0x0c,
                  MKTAG('M','H','E','D')))
        return -1;

    avio_seek(pb, coffs[scene_base] + 0x08 + field, SEEK_SET);
    resource = (int64_t)scene_base + avio_rl32(pb);
    if (resource < 0 || resource >= containers || coffs[resource] <= 0 ||
        coffs[resource] + 0x10 > fsize ||
        !is_id_at(pb, coffs[resource] + 0x0c, id))
        return -1;

    return resource;
}

static int move_scene_frame_count(AVIOContext *pb, int64_t fsize,
                                  const int64_t *coffs, int scene_base)
{
    int64_t H = coffs[scene_base] + 0x08;
    uint32_t count, size;

    avio_seek(pb, coffs[scene_base] + 0x04, SEEK_SET);
    size = avio_rl32(pb);
    if (size < 0x7c || H + size > fsize)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, H + 0x78, SEEK_SET);
    count = avio_rl32(pb);
    if (count > DF_MAX_CHUNKS ||
        0x7c + (int64_t)count * 0x2e > size)
        return AVERROR_INVALIDDATA;

    return count;
}

static int move_frame_resource(AVIOContext *pb, int64_t fsize, int containers,
                               const int64_t *coffs, int scene_base,
                               int frame, int field, uint32_t id)
{
    int64_t H = coffs[scene_base] + 0x08;
    int64_t resource;

    avio_seek(pb, H + 0x7c + (int64_t)frame * 0x2e + field, SEEK_SET);
    resource = (int64_t)scene_base + avio_rl32(pb);
    if (resource < 0 || resource >= containers || coffs[resource] <= 0 ||
        coffs[resource] + 0x10 > fsize ||
        !is_d5_id_at(pb, coffs[resource], id))
        return AVERROR_INVALIDDATA;

    return resource;
}

/* Parse the authored theme order and optional tail loop. */
static int read_theme_move(AVFormatContext *s, int containers, const int64_t *coffs,
                           int theme_id, int scene_base,
                           int **seq, int *seq_count,
                           int *loop_start, int *loop)
{
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    int64_t theme_head = coffs[theme_id] + 0x08;
    int order_count, source_count, *segs = NULL, *sq = NULL;
    uint32_t flags, sc, ls;

    *seq = NULL;
    *seq_count = 0;
    *loop_start = 0;
    *loop = 0;

    avio_seek(pb, theme_head + 0x14, SEEK_SET);
    flags = avio_rl32(pb);
    avio_seek(pb, theme_head + 0x18, SEEK_SET);
    ls = avio_rl32(pb);
    avio_seek(pb, theme_head + 0x1c, SEEK_SET);
    order_count = (int16_t)avio_rl16(pb);
    avio_seek(pb, theme_head + 0x222, SEEK_SET);
    sc = avio_rl32(pb);
    if (sc == 0 || sc > DF_MAX_CHUNKS || order_count <= 0 ||
        order_count > DF_MAX_CHUNKS)
        return 0;

    source_count = sc;

    segs = av_malloc_array(source_count, sizeof(*segs));
    if (!segs)
        return 0;
    for (int i = 0; i < source_count; i++) {
        int64_t sid;

        avio_seek(pb, theme_head + 0x226 + (int64_t)i * 0x22 + 0x0c,
                  SEEK_SET);
        /* Theme sound references are relative to the current scene. */
        sid = (int64_t)scene_base + avio_rl32(pb);
        if (sid < 0 || sid > INT_MAX ||
            !valid_soun(pb, fsize, containers, coffs, (int)sid)) {
            av_free(segs);
            return 0;
        }
        segs[i] = (int)sid;
    }

    /* Disk themes continue through the segment directory. Their order table
     * describes only the initial resident segments. */
    if (flags & 1) {
        *seq = segs;
        *seq_count = source_count;
        return 1;
    }

    sq = av_malloc_array(order_count, sizeof(*sq));
    if (!sq) {
        av_free(segs);
        return 0;
    }
    for (int i = 0; i < order_count; i++) {
        int e;
        avio_seek(pb, theme_head + 0x1e + (int64_t)i * 2, SEEK_SET);
        e = (int16_t)avio_rl16(pb);
        e = av_clip(e, 1, source_count);
        sq[i] = segs[e - 1];
    }

    av_free(segs);
    *seq = sq;
    *seq_count = order_count;
    *loop_start = av_clip64(ls, 0, order_count - 1);
    *loop = !(flags & 1);
    return 1;
}

static void lookup_theme_name_move(AVIOContext *pb, int64_t fsize,
                                   const int64_t *coffs, int theme_id,
                                   int scene_base, int soun_id,
                                   char *dst, int dst_size)
{
    int64_t H = coffs[theme_id] + 0x08;
    uint32_t sc;

    dst[0] = '\0';
    avio_seek(pb, H + 0x222, SEEK_SET);
    sc = avio_rl32(pb);
    if (sc == 0 || sc > DF_MAX_CHUNKS)
        return;

    for (uint32_t i = 0; i < sc; i++) {
        int64_t seg = H + 0x226 + (int64_t)i * 0x22;

        avio_seek(pb, seg + 0x0c, SEEK_SET);
        if ((int64_t)scene_base + avio_rl32(pb) != soun_id)
            continue;
        read_pstring(pb, seg + 0x12, fsize, dst, dst_size);
        return;
    }
}

/* Parse the alternate segment/order theme representation. */
static int read_background_theme_move(AVFormatContext *s, int containers,
                                      const int64_t *coffs, int theme_id,
                                      int scene_base,
                                      int **seg_souns, int *seg_count,
                                      int **seq, int *seq_count, int *disk)
{
    AVIOContext *pb = s->pb;
    int64_t fsize = avio_size(pb);
    int64_t theme_head = coffs[theme_id] + 0x08;
    int order_count, *segs = NULL, *sq = NULL;
    uint32_t sc;

    *seg_souns = NULL;
    *seg_count = 0;
    *seq = NULL;
    *seq_count = 0;
    *disk = 0;

    avio_seek(pb, theme_head + 0x1c, SEEK_SET);
    order_count = avio_rl16(pb);
    avio_seek(pb, theme_head + 0x222, SEEK_SET);
    sc = avio_rl32(pb);
    if (sc == 0 || sc > DF_MAX_CHUNKS ||
        order_count > DF_MAX_CHUNKS)
        return 0;

    segs = av_malloc_array(sc, sizeof(*segs));
    if (!segs)
        return AVERROR(ENOMEM);
    for (uint32_t i = 0; i < sc; i++) {
        int64_t sid;

        avio_seek(pb, theme_head + 0x226 + (int64_t)i * 0x22 + 0x0c,
                  SEEK_SET);
        sid = (int64_t)scene_base + avio_rl32(pb);
        if (sid < 0 || sid > INT_MAX ||
            !valid_soun(pb, fsize, containers, coffs, (int)sid)) {
            av_free(segs);
            return 0;
        }
        segs[i] = (int)sid;
    }

    *disk = ((uint32_t)order_count < sc);
    if (*disk) {
        sq = av_malloc_array(sc, sizeof(*sq));
        if (!sq) {
            av_free(segs);
            return AVERROR(ENOMEM);
        }
        for (uint32_t i = 0; i < sc; i++)
            sq[i] = segs[i];
        *seq_count = sc;
    } else {
        if (order_count <= 0) {
            av_free(segs);
            return 0;
        }
        sq = av_malloc_array(order_count, sizeof(*sq));
        if (!sq) {
            av_free(segs);
            return AVERROR(ENOMEM);
        }
        for (int i = 0; i < order_count; i++) {
            int entry;
            avio_seek(pb, theme_head + 0x1e + (int64_t)i * 2, SEEK_SET);
            entry = avio_rl16(pb);
            if (entry < 1 || (uint32_t)entry > sc) {
                av_free(sq);
                av_free(segs);
                return 0;
            }
            sq[i] = segs[entry - 1];
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

/* Add the STEP resources selected by the authored MHED frame directory. */
static int build_video_stream(AVFormatContext *s, const int64_t *coffs,
                              int64_t fsize,
                              const CFDFD5MoveTimeline *timeline)
{
    AVIOContext *pb = s->pb;
    CFDFD5Block *blocks = NULL;
    CFDFD5Stream *cs;
    AVStream *st;
    int nb = 0, width = 0, height = 0;
    int64_t total = 0;

    if (!timeline)
        return 0;

    for (int frame = 0; frame < timeline->nb_frames; frame++) {
        int i = timeline->frames[frame].step_id;
        uint32_t csize;
        CFDFD5Block *nbk;
        int64_t base;

        if (coffs[i] + 0x428 > fsize ||
            !is_d5_id_at(pb, coffs[i], MKTAG('S','T','E','P'))) {
            av_freep(&blocks);
            return AVERROR_INVALIDDATA;
        }

        avio_seek(pb, coffs[i] + 0x04, SEEK_SET);
        csize = avio_rl32(pb);
        base  = coffs[i] + 0x08;
        if (csize < 0x428 || base + csize > fsize) {
            av_freep(&blocks);
            return AVERROR_INVALIDDATA;
        }

        if (nb == 0) {
            avio_seek(pb, base + 0x20, SEEK_SET);
            height = (int16_t)avio_rl16(pb);
            width  = (int16_t)avio_rl16(pb);
            if (width <= 0 || height <= 0) {
                av_freep(&blocks);
                return 0;
            }
        }

        nbk = av_realloc_array(blocks, nb + 1, sizeof(*blocks));
        if (!nbk) {
            av_freep(&blocks);
            return AVERROR(ENOMEM);
        }
        blocks = nbk;
        blocks[nb].offset     = base;
        blocks[nb].size       = csize;
        blocks[nb].nb_samples = timeline->frames[frame].duration;
        total += timeline->frames[frame].duration;
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

    avpriv_set_pts_info(st, 64, 1, 60);
    av_dict_set_int(&st->metadata, "cfdf_d5_hold_frames",
                    timeline->hold_frames, 0);
    av_dict_set_int(&st->metadata, "cfdf_d5_max_hold_ticks",
                    timeline->max_hold_ticks, 0);

    if (total > 0) {
        AVRational fr;
        av_reduce(&fr.num, &fr.den, (int64_t)nb * 60, total, INT_MAX);
        st->avg_frame_rate = fr;
    }

    return 1;
}

/* Catalogued sounds remain untimed until a frame command schedules them. */
static int add_stream_at(AVFormatContext *s, int variant, int rate,
                         CFDFD5Block *blocks, int nb_blocks, const char *title,
                         const char *timeline, int64_t start,
                         AVRational start_time_base)
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
    cs->start_pts = av_rescale_q_rnd(start, start_time_base,
                                     (AVRational){ 1, rate },
                                     (enum AVRounding)(AV_ROUND_NEAR_INF |
                                                       AV_ROUND_PASS_MINMAX));
    cs->pts       = cs->start_pts;

    av_dict_set(&st->metadata, "timeline", timeline, 0);
    if (!strcmp(timeline, "background")) {
        int have_default = 0;
        for (int i = 0; i + 1 < s->nb_streams; i++)
            have_default |= !!(s->streams[i]->disposition & AV_DISPOSITION_DEFAULT);
        if (!have_default)
            st->disposition |= AV_DISPOSITION_DEFAULT;
    }

    for (int i = 0; i < nb_blocks; i++)
        total += blocks[i].nb_samples;

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_CFDF_D5;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout   = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    st->start_time            = cs->start_pts;
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

static int build_soun_stream_at(AVFormatContext *s, int64_t coff,
                                const char *title, const char *timeline,
                                int64_t start, AVRational start_time_base)
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
    ret = add_stream_at(s, var, rate, blocks, nb, title, timeline,
                        start, start_time_base);
    if (ret < 0) {
        av_freep(&blocks);
        return ret;
    }
    return 1;
}

static int build_soun_stream(AVFormatContext *s, int64_t coff, const char *title,
                             const char *timeline, int64_t start_ticks)
{
    return build_soun_stream_at(s, coff, title, timeline, start_ticks,
                                (AVRational){ 1, 60 });
}

/* Add one stream stitching the SOUN sequence seq[0..count) in order. Returns 1
 * if added, 0 if no usable track, <0 on fatal error. */
static int build_track_stream(AVFormatContext *s, const int64_t *coffs,
                              const int *seq, int count, const char *title,
                              int64_t start_ticks, int loop_start, int loop,
                              int scene)
{
    CFDFD5Block *blocks = NULL;
    AVStream *st;
    int nb = 0, var = 0, rate = 0, ret;
    int64_t loop_start_samples = 0;

    for (int i = 0; i < count; i++) {
        if (i == loop_start)
            for (int k = 0; k < nb; k++)
                loop_start_samples += blocks[k].nb_samples;
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
    ret = add_stream_at(s, var, rate, blocks, nb, title, "background",
                        start_ticks, (AVRational){ 1, 60 });
    if (ret < 0) {
        av_freep(&blocks);
        return ret;
    }
    if (scene >= 0) {
        st = s->streams[s->nb_streams - 1];
        av_dict_set(&st->metadata, "cfdf_d5_slot", "2", 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_scene", scene, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_loop", loop, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_loop_start", loop_start_samples, 0);
    }
    return 1;
}

/* Keep the compatibility stream separate from timed theme nodes. */
static int build_move_background_stream(AVFormatContext *s, int containers,
                                        const int64_t *coffs,
                                        const char *title, int compatibility)
{
    AVIOContext *pb = s->pb;
    int *seg_souns = NULL, *seq = NULL;
    int seg_count = 0, seq_count = 0, disk = 0;
    int theme_id = -1, theme_scene_base = 0, scene_base = 0;
    int has_track = 0, ret = 0;

    for (int i = 0; i < containers && theme_id < 0; i++) {
        uint32_t sc;

        if (coffs[i] <= 0)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0c, MKTAG('M','H','E','D'))) {
            scene_base = i;
            continue;
        }
        if (!is_id_at(pb, coffs[i] + 0x0c, MKTAG('M','T','H','M')))
            continue;
        avio_seek(pb, coffs[i] + 0x08 + 0x222, SEEK_SET);
        sc = avio_rl32(pb);
        if (sc > 0 && sc <= DF_MAX_CHUNKS) {
            theme_id = i;
            theme_scene_base = scene_base;
        }
    }

    if (theme_id >= 0) {
        has_track = read_background_theme_move(s, containers, coffs, theme_id,
                                               theme_scene_base,
                                               &seg_souns, &seg_count,
                                               &seq, &seq_count, &disk);
        if (has_track < 0) {
            ret = has_track;
            goto end;
        }
    }

    if (has_track) {
        int lo = 0, hi = seq_count - 1;

        if (!disk) {
            while (lo <= hi && soun_is_silent(pb, coffs[seq[lo]]))
                lo++;
            while (hi >= lo && soun_is_silent(pb, coffs[seq[hi]]))
                hi--;
            if (lo > hi) {
                lo = 0;
                hi = seq_count - 1;
            }
        }
        ret = build_track_stream(s, coffs, seq + lo, hi - lo + 1,
                                 title, 0, 0, 0, -1);
        if (ret > 0) {
            AVStream *st = s->streams[s->nb_streams - 1];

            av_dict_set(&st->metadata, "cfdf_d5_slot", "2", 0);
            if (compatibility) {
                for (unsigned i = 0; i < s->nb_streams; i++) {
                    AVStream *other = s->streams[i];

                    if (other->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
                        other->disposition &= ~AV_DISPOSITION_DEFAULT;
                }
                st->disposition |= AV_DISPOSITION_DEFAULT;
                av_dict_set(&st->metadata, "timeline",
                            "background_compat", 0);
                av_dict_set_int(&st->metadata, "cfdf_d5_compat", 1, 0);
            }
        }
    }

end:
    av_freep(&seg_souns);
    av_freep(&seq);
    return ret;
}

static int find_move_sound(AVFormatContext *s, int containers,
                           const int64_t *coffs, int msnd_id, int scene_base,
                           int64_t fsize, const char *name,
                           CFDFD5MoveSound *sound)
{
    AVIOContext *pb = s->pb;
    int count;

    if (msnd_id < 0 || msnd_id >= containers ||
        coffs[msnd_id] <= 0 || coffs[msnd_id] + 0x44 > fsize)
        return -1;

    avio_seek(pb, coffs[msnd_id] + 0x08 + 0x1c, SEEK_SET);
    count = avio_rl32(pb);
    if (count <= 0 || count > DF_MAX_CHUNKS)
        return -1;

    for (int i = 0; i < count; i++) {
        int64_t entry = coffs[msnd_id] + 0x08 + 0x20 + (int64_t)i * 0x30;
        char entry_name[DF_NAME_SIZE];
        int soun_id;

        if (entry + 0x30 > fsize)
            return -1;
        read_pstring(pb, entry + 0x10, fsize, entry_name, sizeof(entry_name));
        if (strcmp(entry_name, name))
            continue;

        avio_seek(pb, entry + 0x0a, SEEK_SET);
        soun_id = scene_base + avio_rl32(pb);
        if (!valid_soun(pb, fsize, containers, coffs, soun_id))
            return -1;
        avio_seek(pb, entry + 0x04, SEEK_SET);
        sound->pan = (int16_t)avio_rl16(pb);
        sound->flags = avio_r8(pb);
        sound->soun_id = soun_id;
        sound->index = i;
        return 0;
    }

    return -1;
}


static void free_move_timeline(CFDFD5MoveTimeline *timeline)
{
    av_freep(&timeline->container_pts);
    av_freep(&timeline->frames);
    memset(timeline, 0, sizeof(*timeline));
}

static int soun_duration_ticks(AVFormatContext *s, int64_t coff,
                               int64_t *duration_ticks)
{
    CFDFD5Block *blocks = NULL;
    int nb = 0, variant = 0, rate = 0, ret;
    int64_t samples = 0;

    *duration_ticks = 0;
    ret = parse_soun(s, coff, &variant, &rate, &blocks, &nb);
    if (ret < 0) {
        av_freep(&blocks);
        return ret;
    }

    for (int i = 0; i < nb; i++)
        samples += blocks[i].nb_samples;
    av_freep(&blocks);

    if (samples > 0 && rate > 0)
        *duration_ticks = av_rescale_rnd(samples, 60, rate, AV_ROUND_UP);
    return 0;
}

typedef struct CFDFD5SoundSlot {
    int key;
    int64_t end;
} CFDFD5SoundSlot;

/* Matching keys restart first; otherwise prefer a free slot, then replace
 * the lower key only when the incoming key is greater. Ties choose slot 1. */
static int move_pool_slot(CFDFD5SoundSlot *pool, int key, int64_t pts)
{
    int slot;

    for (int i = 0; i < 2; i++)
        if (pool[i].end <= pts)
            pool[i].key = -1;
    for (int i = 0; i < 2; i++)
        if (pool[i].key == key)
            return i;
    for (int i = 0; i < 2; i++)
        if (pool[i].key < 0)
            return i;
    slot = pool[0].key < pool[1].key ? 0 : 1;
    return key > pool[slot].key ? slot : -1;
}

/* Resolve a nominal clock shared by video and audio. Device-buffer latency
 * is not part of the authored timeline. */
static int build_move_timeline(AVFormatContext *s, int containers,
                               const int64_t *coffs, int64_t fsize,
                               CFDFD5MoveTimeline *timeline)
{
    AVIOContext *pb = s->pb;
    int scene = -1;
    int64_t pts = 0;

    timeline->container_pts = av_calloc(containers,
                                        sizeof(*timeline->container_pts));
    if (!timeline->container_pts) {
        free_move_timeline(timeline);
        return AVERROR(ENOMEM);
    }

    for (int scene_base = 0; scene_base < containers; scene_base++) {
        CFDFD5MoveFrame *frames;
        CFDFD5SoundSlot pool[2] = { { -1, 0 }, { -1, 0 } };
        int64_t fixed_end = 0, scene_start;
        int first_frame = timeline->nb_frames;
        int default_ticks, frame_count, msnd_id, next_scene;
        int64_t H;

        timeline->container_pts[scene_base] = pts;
        if (coffs[scene_base] <= 0 || coffs[scene_base] + 0x10 > fsize ||
            !is_d5_id_at(pb, coffs[scene_base], MKTAG('M','H','E','D')))
            continue;
        H = coffs[scene_base] + 0x08;
        scene++;
        scene_start = pts;
        msnd_id = move_scene_resource(pb, fsize, containers, coffs,
                                      scene_base, 0x60,
                                      MKTAG('M','S','N','D'));
        avio_seek(pb, H + 0x1c, SEEK_SET);
        default_ticks = avio_rl32(pb);
        if (default_ticks <= 0 || default_ticks > 0xffff)
            default_ticks = 4;

        frame_count = move_scene_frame_count(pb, fsize, coffs, scene_base);
        if (frame_count < 0) {
            free_move_timeline(timeline);
            return frame_count;
        }
        if (timeline->nb_frames > DF_MAX_CHUNKS - frame_count) {
            free_move_timeline(timeline);
            return AVERROR_INVALIDDATA;
        }
        frames = av_realloc_array(timeline->frames,
                                  timeline->nb_frames + frame_count,
                                  sizeof(*timeline->frames));
        if (!frames && frame_count > 0) {
            free_move_timeline(timeline);
            return AVERROR(ENOMEM);
        }
        timeline->frames = frames;

        next_scene = containers;
        for (int i = scene_base + 1; i < containers; i++) {
            if (coffs[i] > 0 && coffs[i] + 0x10 <= fsize &&
                is_d5_id_at(pb, coffs[i], MKTAG('M','H','E','D'))) {
                next_scene = i;
                break;
            }
        }
        for (int i = scene_base; i < next_scene; i++)
            timeline->container_pts[i] = scene_start;

        for (int index = 0; index < frame_count; index++) {
            CFDFD5MoveFrame *frame;
            CFDFD5MoveSound sound;
            char name[DF_NAME_SIZE];
            uint32_t size;
            int mfrm_id, step_id;
            int flags, nominal;
            int64_t duration, frame_end;

            step_id = move_frame_resource(pb, fsize, containers, coffs,
                                          scene_base, index, 0x10,
                                          MKTAG('S','T','E','P'));
            mfrm_id = move_frame_resource(pb, fsize, containers, coffs,
                                          scene_base, index, 0x14,
                                          MKTAG('M','F','R','M'));
            if (step_id < 0 || mfrm_id < 0) {
                free_move_timeline(timeline);
                return AVERROR_INVALIDDATA;
            }

            nominal = mfrm_duration_ticks(pb, coffs[mfrm_id], fsize,
                                          default_ticks);
            flags = mfrm_flags(pb, coffs[mfrm_id], fsize);
            duration = nominal;

            avio_seek(pb, coffs[mfrm_id] + 0x04, SEEK_SET);
            size = avio_rl32(pb);
            name[0] = '\0';
            if (size >= 0x2b && coffs[mfrm_id] + 0x08 + size <= fsize)
                read_pstring(pb, coffs[mfrm_id] + 0x08 + 0x2a,
                             coffs[mfrm_id] + 0x08 + size,
                             name, sizeof(name));

            if (name[0] && msnd_id >= 0 &&
                find_move_sound(s, containers, coffs, msnd_id,
                                scene_base, fsize, name, &sound) >= 0) {
                int64_t end = INT64_MAX;
                int slot = sound.flags & 8 ?
                           move_pool_slot(pool, sound.index, pts) : 2;

                if (!(sound.flags & 2)) {
                    int64_t sound_ticks = 0;
                    int ret = soun_duration_ticks(s, coffs[sound.soun_id],
                                                  &sound_ticks);
                    if (ret < 0) {
                        free_move_timeline(timeline);
                        return ret;
                    }
                    end = pts + sound_ticks;
                }
                if (slot == 2)
                    fixed_end = end;
                else if (slot >= 0) {
                    pool[slot].key = sound.index;
                    pool[slot].end = end;
                }
            }

            frame_end = pts + nominal;
            if (flags & (CFDF_D5_MFRM_HOLD | CFDF_D5_MFRM_POOL_HOLD)) {
                int64_t wait_end = frame_end;
                int64_t extension;

                if (flags & CFDF_D5_MFRM_HOLD)
                    wait_end = FFMAX(wait_end, fixed_end);
                if (flags & CFDF_D5_MFRM_POOL_HOLD)
                    wait_end = FFMAX(wait_end, FFMAX(pool[0].end, pool[1].end));
                if (wait_end == INT64_MAX) {
                    av_log(s, AV_LOG_ERROR,
                           "Cannot linearize a hold on a looping sound\n");
                    free_move_timeline(timeline);
                    return AVERROR_PATCHWELCOME;
                }
                extension = wait_end - frame_end;

                timeline->hold_frames++;
                timeline->max_hold_ticks =
                    FFMAX(timeline->max_hold_ticks,
                          extension > INT_MAX ? INT_MAX : (int)extension);
                duration += extension;
            }
            if (duration <= 0 || duration > INT_MAX) {
                free_move_timeline(timeline);
                return AVERROR_INVALIDDATA;
            }

            frame = &timeline->frames[timeline->nb_frames++];
            frame->scene_base = scene_base;
            frame->scene = scene;
            frame->mfrm_id = mfrm_id;
            frame->step_id = step_id;
            frame->scene_start = scene_start;
            frame->pts = pts;
            frame->duration = (int)duration;
            timeline->container_pts[mfrm_id] = pts;
            timeline->container_pts[step_id] = pts;
            pts += duration;
        }

        for (int i = first_frame; i < timeline->nb_frames; i++)
            timeline->frames[i].scene_end = pts;
        scene_base = next_scene - 1;
    }

    timeline->total_ticks = pts;
    return 0;
}

/* Frame commands resolve named sounds through the active sound directory. */
static int build_move_sfx_streams(AVFormatContext *s, int containers,
                                  const int64_t *coffs, int64_t fsize,
                                  const CFDFD5MoveTimeline *timeline)
{
    AVIOContext *pb = s->pb;
    int scene_base = -1, msnd_id = -1;

    for (int index = 0; index < timeline->nb_frames; index++) {
        const CFDFD5MoveFrame *frame = &timeline->frames[index];
        int i = frame->mfrm_id;
        int64_t H;

        H = coffs[i] + 0x08;

        if (scene_base != frame->scene_base) {
            scene_base = frame->scene_base;
            msnd_id = move_scene_resource(pb, fsize, containers, coffs,
                                          scene_base, 0x60,
                                          MKTAG('M','S','N','D'));
        }
        {
            uint32_t size;
            char name[DF_NAME_SIZE];
            CFDFD5MoveSound sound;
            AVStream *st;
            int ret;

            avio_seek(pb, coffs[i] + 0x04, SEEK_SET);
            size = avio_rl32(pb);
            if (size < 0x2b || H + size > fsize)
                continue;

            read_pstring(pb, H + 0x2a, H + size, name, sizeof(name));
            if (name[0] && msnd_id >= 0) {
                ret = find_move_sound(s, containers, coffs, msnd_id,
                                      scene_base, fsize, name, &sound);
                if (ret >= 0) {
                    ret = build_soun_stream(s, coffs[sound.soun_id], name,
                                            "sfx", frame->pts);
                    if (ret < 0)
                        return ret;
                    if (ret > 0) {
                        st = s->streams[s->nb_streams - 1];
                        av_dict_set(&st->metadata, "cfdf_d5_slot",
                                    sound.flags & 8 ? "pool" : "3", 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_key",
                                        0x08000000 + sound.index, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_msnd_index",
                                        sound.index, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_scene",
                                        frame->scene, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_scene_start",
                                        frame->scene_start, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_scene_end",
                                        frame->scene_end, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_start_ticks",
                                        frame->pts, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_pan",
                                        sound.pan, 0);
                        av_dict_set_int(&st->metadata, "cfdf_d5_loop",
                                        !!(sound.flags & 2), 0);
                    }
                }
            }
        }
    }

    return 0;
}

static int build_move_theme_segments(AVFormatContext *s, int containers,
                                     const int64_t *coffs, int64_t fsize,
                                     int theme_id, int scene_base, int scene,
                                     int64_t scene_start_ticks,
                                     int64_t scene_end_ticks,
                                     const char *fallback_title)
{
    AVIOContext *pb = s->pb;
    int *seq = NULL;
    int count = 0, loop_start = 0, loop = 0;
    int pos = 0, pass = 0, emitted = 0, ret;
    int64_t cursor_us, scene_end_us;

    if (scene_end_ticks <= scene_start_ticks)
        return 0;
    if (!read_theme_move(s, containers, coffs, theme_id, scene_base,
                         &seq, &count, &loop_start, &loop))
        return 0;

    cursor_us = av_rescale_q(scene_start_ticks, (AVRational){ 1, 60 },
                            AV_TIME_BASE_Q);
    scene_end_us = av_rescale_q(scene_end_ticks, (AVRational){ 1, 60 },
                               AV_TIME_BASE_Q);

    /* A finite disk theme is one continuous track, not one stream per buffer.
     * Keep separate nodes if their codecs or sample rates cannot be joined. */
    if (!loop) {
        ret = build_track_stream(s, coffs, seq, count, fallback_title,
                                 scene_start_ticks, 0, 0, scene);
        if (ret < 0) {
            av_freep(&seq);
            return ret;
        }
        if (ret > 0) {
            AVStream *st = s->streams[s->nb_streams - 1];

            av_dict_set_int(&st->metadata, "cfdf_d5_scene_start",
                            scene_start_ticks, 0);
            av_dict_set_int(&st->metadata, "cfdf_d5_end",
                            scene_end_ticks, 0);
            av_dict_set_int(&st->metadata, "cfdf_d5_end_us", scene_end_us, 0);
            av_dict_set_int(&st->metadata, "cfdf_d5_start_us", cursor_us, 0);
            av_freep(&seq);
            return ret;
        }
    }

    /* Adjacent theme nodes may change codec or sample rate. */
    while (cursor_us < scene_end_us && emitted < DF_MAX_CHUNKS) {
        AVStream *st;
        char name[DF_NAME_SIZE];
        int64_t duration_us;
        int soun_id;

        if (pos >= count) {
            if (!loop)
                break;
            pos = loop_start;
            pass++;
        }

        soun_id = seq[pos];
        lookup_theme_name_move(pb, fsize, coffs, theme_id, scene_base,
                               soun_id, name, sizeof(name));
        ret = build_soun_stream_at(s, coffs[soun_id],
                                   name[0] ? name : fallback_title,
                                   "background", cursor_us, AV_TIME_BASE_Q);
        if (ret < 0) {
            av_freep(&seq);
            return ret;
        }
        if (ret == 0)
            break;

        st = s->streams[s->nb_streams - 1];
        duration_us = av_rescale_q(st->duration, st->time_base,
                                  AV_TIME_BASE_Q);
        if (duration_us <= 0) {
            av_freep(&seq);
            return AVERROR_INVALIDDATA;
        }

        av_dict_set(&st->metadata, "cfdf_d5_slot", "2", 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_scene", scene, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_scene_start",
                        scene_start_ticks, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_end",
                        scene_end_ticks, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_end_us",
                        scene_end_us, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_playlist_entry", pos, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_playlist_pass", pass, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_playlist_loop", loop, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_playlist_loop_start",
                        loop_start, 0);
        av_dict_set_int(&st->metadata, "cfdf_d5_start_us", cursor_us, 0);

        /* Nodes are expanded here; looping one node would skip later nodes. */
        av_dict_set_int(&st->metadata, "cfdf_d5_loop", 0, 0);

        cursor_us += duration_us;
        pos++;
        emitted++;
    }

    av_freep(&seq);
    return emitted;
}

/* An empty theme leaves the active slot-2 playlist unchanged. */
static int move_theme_has_sequence(AVFormatContext *s, int64_t coff,
                                   int64_t fsize)
{
    AVIOContext *pb = s->pb;
    int64_t H = coff + 0x08;
    int order_count;
    uint32_t size, source_count;

    if (coff <= 0 || coff + 0x10 > fsize ||
        !is_id_at(pb, coff + 0x0c, MKTAG('M','T','H','M')))
        return 0;

    avio_seek(pb, coff + 0x04, SEEK_SET);
    size = avio_rl32(pb);
    if (size < 0x226 || H + size > fsize)
        return 0;

    avio_seek(pb, H + 0x1c, SEEK_SET);
    order_count = (int16_t)avio_rl16(pb);
    avio_seek(pb, H + 0x222, SEEK_SET);
    source_count = avio_rl32(pb);

    return order_count > 0 && order_count <= DF_MAX_CHUNKS &&
           source_count > 0 && source_count <= DF_MAX_CHUNKS;
}

static int build_move_theme_streams(AVFormatContext *s, int containers,
                                    const int64_t *coffs, int64_t fsize,
                                    const char *title,
                                    const CFDFD5MoveTimeline *timeline)
{
    AVIOContext *pb = s->pb;
    int built = 0, scene = -1, scene_base = 0;
    int64_t scene_start = 0;

    for (int i = 0; i < containers; i++) {
        if (coffs[i] <= 0 || coffs[i] + 0x10 > fsize)
            continue;

        if (is_id_at(pb, coffs[i] + 0x0c, MKTAG('M','H','E','D'))) {
            scene_base = i;
            scene++;
            scene_start = timeline->container_pts[i];
            continue;
        }

        if (is_id_at(pb, coffs[i] + 0x0c, MKTAG('M','T','H','M'))) {
            int64_t theme_end = timeline->total_ticks;
            int ret;

            if (!move_theme_has_sequence(s, coffs[i], fsize))
                continue;

            /* Only a later non-empty theme replaces slot 2. */
            for (int j = i + 1; j < containers; j++) {
                if (coffs[j] <= 0 || coffs[j] + 0x10 > fsize)
                    continue;
                if (is_id_at(pb, coffs[j] + 0x0c,
                             MKTAG('M','T','H','M')) &&
                    move_theme_has_sequence(s, coffs[j], fsize)) {
                    theme_end = timeline->container_pts[j];
                    break;
                }
            }

            ret = build_move_theme_segments(s, containers, coffs, fsize, i,
                                            scene_base, FFMAX(scene, 0),
                                            scene_start, theme_end, title);
            if (ret < 0)
                return ret;
            built += ret;
        }
    }

    return built;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    CFDFD5MoveTimeline timeline = { 0 };
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

    if (!is_trak &&
        (coffs[0] <= 0 || coffs[0] + 0x10 > fsize ||
         !is_d5_id_at(pb, coffs[0], MKTAG('M','H','E','D')))) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    for (int i = 0; i < containers; i++) {
        if (coffs[i] <= 0 || coffs[i] + 0x30 > fsize)
            continue;
        if (is_id_at(pb, coffs[i] + 0x0C, MKTAG('S','O','U','N')))
            souns[soun_count++] = i;
    }

    if (!is_trak) {
        ret = build_move_timeline(s, containers, coffs, fsize, &timeline);
        if (ret < 0)
            goto end;
        av_dict_set(&s->metadata, "cfdf_d5_timeline", "2", 0);
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
            ret = build_track_stream(s, coffs, seq + lo, hi - lo + 1,
                                     basename, 0, 0, 0, -1);
            if (ret < 0) { av_freep(&seq); goto end; }
        }
        av_freep(&seq);

        /* every SOUN is also listed individually (keep stitched chunks) */
        for (int j = 0; j < soun_count; j++) {
            char name[DF_NAME_SIZE];
            lookup_name_trak(pb, fsize, containers, coffs, souns[j], name, sizeof(name));
            ret = build_soun_stream(s, coffs[souns[j]], name[0] ? name : NULL,
                                    "untimed", 0);
            if (ret < 0)
                goto end;
        }
    } else {
        int theme_streams;

        theme_streams = build_move_theme_streams(s, containers, coffs, fsize,
                                                 basename, &timeline);
        if (theme_streams < 0) {
            ret = theme_streams;
            goto end;
        }
        ret = build_move_background_stream(s, containers, coffs, basename,
                                           theme_streams > 0);
        if (ret < 0)
            goto end;

        ret = build_move_sfx_streams(s, containers, coffs, fsize, &timeline);
        if (ret < 0)
            goto end;

        for (int j = 0; j < soun_count; j++) {
            int sid = souns[j];
            char name[DF_NAME_SIZE];

            lookup_name_move(pb, fsize, containers, coffs, sid, name, sizeof(name));
            ret = build_soun_stream(s, coffs[sid], name[0] ? name : NULL,
                                    "untimed", 0);
            if (ret < 0)
                goto end;
        }
    }

video:
    ret = build_video_stream(s, coffs, fsize,
                             is_trak ? NULL : &timeline);
    if (ret < 0)
        goto end;

    if (s->nb_streams == 0) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }
    ret = 0;

end:
    free_move_timeline(&timeline);
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
    AVRational target_tb;
    int64_t target_ts;

    /* Authored blocks support timestamp seeking only. */
    if (flags & (AVSEEK_FLAG_BYTE | AVSEEK_FLAG_FRAME))
        return AVERROR(ENOSYS);

    if (stream_index < 0) {
        target_ts = ts;
        target_tb = AV_TIME_BASE_Q;
    } else {
        if ((unsigned)stream_index >= s->nb_streams)
            return AVERROR(EINVAL);
        target_ts = ts;
        target_tb = s->streams[stream_index]->time_base;
    }
    if (target_ts < 0)
        target_ts = 0;

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
            int64_t acc = cs->start_pts;
            int k = 0;

            while (k < cs->nb_blocks &&
                   av_compare_ts(acc + cs->blocks[k].nb_samples,
                                 st->time_base, target_ts, target_tb) <= 0) {
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
    .p.flags        = AVFMT_NO_BYTE_SEEK,
    .priv_data_size = sizeof(CFDFD5DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
