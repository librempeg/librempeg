/*
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

#include "libavutil/intreadwrite.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"

#include "libavcodec/get_bits.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

#define SACD_SECTOR      2048
#define MTOC_LSN         510
#define MAX_TRACKS       255

#define PT_AUDIO         2
#define PT_SUPPL         3
#define PT_PADDING       7

enum { FMT_DST = 0, FMT_DSD_3IN14 = 2, FMT_DSD_3IN16 = 3 };

typedef struct SACDSector {
    int frame_start, type, len;
} SACDSector;

typedef struct SACDArea {
    int       present, eof;
    AVStream *st;
    uint32_t  toc_lsn;
    uint32_t  start_lsn, end_lsn, cur_lsn;
    int       channels, frame_format, track_count;

    /* sector parse state */
    uint8_t   sec[SACD_SECTOR];
    int       sec_valid;
    int       n_pkt, cur_pkt, data_off;
    SACDSector pi[8];

    /* frame reassembly */
    uint8_t  *buf;
    unsigned  buf_cap;
    int       buf_len;
    int64_t   cur_pos;
} SACDArea;

typedef struct SACDContext {
    SACDArea area[2];            /* 0 = stereo, 1 = multichannel */
} SACDContext;

static int read_probe(const AVProbeData *p)
{
    int64_t off = (int64_t)MTOC_LSN * SACD_SECTOR;

    if (p->buf_size >= off + 8 && !memcmp(p->buf + off, "SACDMTOC", 8))
        return AVPROBE_SCORE_MAX;

    return 0;
}

static int read_sectors(AVFormatContext *s, int64_t *pos, uint32_t lsn, int n, uint8_t *dst)
{
    int64_t ret = avio_seek(s->pb, (int64_t)lsn * SACD_SECTOR, SEEK_SET);

    if (ret < 0)
        return ret;

    if (pos)
        pos[0] = avio_tell(s->pb);

    ret = avio_read(s->pb, dst, n * SACD_SECTOR);

    return ret == n * SACD_SECTOR ? 0 : (ret < 0 ? ret : AVERROR_EOF);
}

static int64_t tc2ts(const uint8_t *t)   /* min, sec, frame */
{
    return ((int64_t)t[0] * 60 + t[1]) * AV_TIME_BASE + t[2] * 13333;
}

static int init_area(AVFormatContext *s, SACDArea *a, int idx)
{
    AVIOContext *pb = s->pb;
    int ret, trl2_off = 2;
    uint16_t ttxt_lsn;
    uint8_t *toc;
    AVStream *st;
    int sr_code;

    toc = av_calloc(8, SACD_SECTOR);   /* Area TOC is a few sectors */
    if (!toc)
        return AVERROR(ENOMEM);

    ret = read_sectors(s, &a->cur_pos, a->toc_lsn, 8, toc);
    if (ret < 0)
        goto end;

    if (memcmp(toc, idx ? "MULCHTOC" : "TWOCHTOC", 8)) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    sr_code         = toc[20];
    a->frame_format = toc[21] & 0x0f;
    a->channels     = toc[32];
    a->track_count  = toc[69];
    a->start_lsn    = AV_RB32(toc + 72);
    a->end_lsn      = AV_RB32(toc + 76);
    ttxt_lsn        = AV_RB16(toc + 128);
    a->cur_lsn      = a->start_lsn;

    if (sr_code != 0x04) {
        avpriv_request_sample(s, "sample_rate code 0x%02X", sr_code);
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    a->st = st;
    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id   = a->frame_format == FMT_DST ? AV_CODEC_ID_DST
                                                          : AV_CODEC_ID_DSD_MSBF;
    st->codecpar->sample_rate = 2822400 / 8;
    av_channel_layout_default(&st->codecpar->ch_layout, a->channels);

    if (a->frame_format != FMT_DST)
        st->codecpar->block_align = a->channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    st->id = idx;
    av_dict_set(&st->metadata, "title", idx ? "Multichannel" : "Stereo", 0);

    /* Chapters: only from the first area that has a track list */
    if (!s->nb_chapters && a->track_count > 0) {
        const uint8_t *trl2 = toc + trl2_off * SACD_SECTOR;
        uint16_t offsets[MAX_TRACKS] = { 0 };
        int64_t ttxt_pos = 0;

        if (ttxt_lsn) {
            uint8_t hdr[8] = { 0 };

            avio_seek(pb, (int64_t)(a->toc_lsn + ttxt_lsn) * SACD_SECTOR, SEEK_SET);

            ttxt_pos = avio_tell(pb);

            avio_read(pb, hdr, sizeof(hdr));
            if (!memcmp(hdr, "SACDTTxt", 8)) {
                for (int i = 0; i < a->track_count; i++)
                    offsets[i] = avio_rb16(pb);
            }
        }

        if (!memcmp(trl2, "SACDTRL2", 8)) {
            for (int i = 0; i < a->track_count && i < MAX_TRACKS; i++) {
                int64_t start = tc2ts(trl2 + 8 + i * 4);
                int64_t duration = tc2ts(trl2 + 8 + MAX_TRACKS * 4 + i * 4);
                uint8_t title[256] = { 0 };

                if (ttxt_pos) {
                    int entries;

                    avio_seek(pb, ttxt_pos + offsets[i], SEEK_SET);
                    entries = avio_r8(pb);
                    if (entries == 0x01) {
                        avio_skip(pb, 5);
                        avio_get_str(pb, UINT16_MAX, title, sizeof(title));
                    } else if (entries == 0x02) {
                        avio_skip(pb, 5);
                        avio_get_str(pb, UINT16_MAX, title, sizeof(title));
                    }
                }

                if (!avpriv_new_chapter(s, i + 1, AV_TIME_BASE_Q,
                                        start, start + duration, title[0] ? title : NULL)) {
                    ret = AVERROR(ENOMEM);
                    goto end;
                }
            }
        }
    }

    ret = 0;

end:
    av_free(toc);

    return ret;
}

static int read_header(AVFormatContext *s)
{
    uint8_t mtoc[SACD_SECTOR] = { 0 };
    SACDContext *c = s->priv_data;
    int ret;

    if ((ret = read_sectors(s, NULL, MTOC_LSN, 1, mtoc)) < 0)
        return ret;

    if (memcmp(mtoc, "SACDMTOC", 8))
        return AVERROR_INVALIDDATA;

    c->area[0].toc_lsn = AV_RB32(mtoc + 64);
    c->area[1].toc_lsn = AV_RB32(mtoc + 72);

    for (int i = 0; i < 2; i++) {
        c->area[i].present = c->area[i].toc_lsn != 0;

        if (!c->area[i].present)
            continue;

        if ((ret = init_area(s, &c->area[i], i)) < 0)
            return ret;
    }

    if (!c->area[0].present && !c->area[1].present)
        return AVERROR_INVALIDDATA;

    return 0;
}

static int parse_sector(SACDArea *a)
{
    GetBitContext gbc, *gb = &gbc;
    int ret, pic, fic, dst;

    ret = init_get_bits8(gb, a->sec, sizeof(a->sec));
    if (ret < 0)
        return ret;

    pic = get_bits(gb, 3);
    fic = get_bits(gb, 3);
    skip_bits(gb, 1);
    dst = get_bits1(gb);

    for (int i = 0; i < pic; i++) {
        a->pi[i].frame_start = get_bits1(gb);
        skip_bits(gb, 1);
        a->pi[i].type = get_bits(gb, 3);
        a->pi[i].len = get_bits(gb, 11);
    }

    skip_bits_long(gb, (3 + dst) * 8 * fic);

    a->n_pkt = pic;
    a->cur_pkt = 0;
    a->data_off = (get_bits_count(gb) + 7) >> 3;
    a->sec_valid = 1;

    return 0;
}

static int emit(SACDArea *a, AVPacket *pkt)
{
    int ret = av_new_packet(pkt, a->buf_len);

    if (ret < 0)
        return ret;

    memcpy(pkt->data, a->buf, a->buf_len);

    pkt->pos = a->cur_pos;
    pkt->stream_index = a->st->index;
    pkt->duration = 4704;
    pkt->flags |= AV_PKT_FLAG_KEY;
    a->buf_len = 0;

    return 0;
}

static int read_frame(AVFormatContext *s, SACDArea *a, AVPacket *pkt)
{
    for (;;) {
        if (!a->sec_valid) {
            int ret;

            if (a->cur_lsn >= a->end_lsn) {
                a->eof = 1;
                return a->buf_len ? emit(a, pkt) : AVERROR_EOF;
            }

            if ((ret = read_sectors(s, &a->cur_pos, a->cur_lsn++, 1, a->sec)) < 0)
                return ret;

            if ((ret = parse_sector(a)) < 0)
                return ret;
        }

        while (a->cur_pkt < a->n_pkt) {
            int i = a->cur_pkt;
            int len = a->pi[i].len;

            if (a->pi[i].type == PT_AUDIO) {
                if (a->pi[i].frame_start) {
                    a->pi[i].frame_start = 0;   /* handle once */
                    if (a->buf_len)
                        return emit(a, pkt);
                }

                if (a->data_off + len > SACD_SECTOR)
                    return AVERROR_INVALIDDATA;

                if (a->buf_len + len > a->buf_cap) {
                    a->buf = av_realloc_f(a->buf, (a->buf_len + len) * 2, sizeof(*a->buf));
                    if (!a->buf)
                        return AVERROR(ENOMEM);

                    a->buf_cap = (a->buf_len + len) * 2;
                }

                memcpy(a->buf + a->buf_len, a->sec + a->data_off, len);
                a->buf_len += len;
            }

            a->data_off += len;
            a->cur_pkt++;
        }
        a->sec_valid = 0;
    }
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SACDContext *c = s->priv_data;
    int best = -1;

    for (int i = 0; i < 2; i++) {
        if (!c->area[i].present || c->area[i].eof)
            continue;

        if (best < 0 || c->area[i].cur_pos < c->area[best].cur_pos)
            best = i;
    }

    if (best < 0)
        return AVERROR_EOF;

    int ret = read_frame(s, &c->area[best], pkt);
    if (ret == AVERROR_EOF)
        return read_packet(s, pkt);   /* try the other area */

    return ret;
}

static int read_close(AVFormatContext *s)
{
    SACDContext *c = s->priv_data;

    av_freep(&c->area[0].buf);
    av_freep(&c->area[1].buf);

    return 0;
}

const FFInputFormat ff_sacd_demuxer = {
    .p.name         = "sacd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SACD (Super Audio CD)"),
    .p.extensions   = "iso",
    .p.flags        = AVFMT_GENERIC_INDEX | AVFMT_NO_BYTE_SEEK,
    .priv_data_size = sizeof(SACDContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
};
