/*
 * CyberFlix DreamFactory audio bank parsing
 *
 * This implementation follows the format documented by vgmstream
 * cfdf_Pre_V4-rc10-gold. See COPYING.vgmstream for its permission notice.
 *
 * This file is part of Librempeg.
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Librempeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/avstring.h"
#include "avformat.h"
#include "cfdf_bank.h"
#include "internal.h"

typedef struct CFDFBankChunk {
    int64_t offset;
    int size, samples, padding, rate, codec, silent;
    char name[33];
} CFDFBankChunk;

typedef struct CFDFBankTrack {
    int *order;
    int count, cursor;
    int64_t pts;
} CFDFBankTrack;

struct CFDFBank {
    CFDFBankChunk *chunks;
    CFDFBankTrack *tracks;
    int count, nb_tracks;
};

static unsigned word(const uint8_t *p, int be)
{
    return be ? AV_RB16(p) : AV_RL16(p);
}

static uint32_t dword(const uint8_t *p, int be)
{
    return be ? AV_RB32(p) : AV_RL32(p);
}

static int read_at(AVIOContext *pb, int64_t pos, uint8_t *buf, int size)
{
    int ret;

    if ((ret = avio_seek(pb, pos, SEEK_SET)) < 0)
        return ret;
    return avio_read(pb, buf, size) == size ? 0 : AVERROR_INVALIDDATA;
}

static void name_from(const uint8_t *p, int left, char *dst, int max)
{
    int len;

    if (left <= 0)
        return;
    len = p[0];
    if (len > max || len >= left)
        return;
    memcpy(dst, p + 1, len);
    dst[len] = 0;
}

/* Count the seed, absolute samples, signed nibble pairs and repeated samples. */
static int sample_count(const uint8_t *p, int size)
{
    int64_t n = size > 0;

    for (int i = 1; i < size;) {
        unsigned code = p[i++];
        int run = (code & 63) + 1;

        if (code < 128)
            n++;
        else if (code < 192) {
            if (run > size - i)
                return 0;
            i += run;
            n += 2 * run;
        } else
            n += run;
        if (n > INT_MAX)
            return 0;
    }
    return n;
}

static int add_track(AVFormatContext *s, CFDFBank *bank, const int *order,
                     int count, const char *name, int loop, int continuous)
{
    CFDFBankTrack *track = &bank->tracks[bank->nb_tracks];
    CFDFBankChunk *first = &bank->chunks[order[0]];
    AVStream *st;
    int64_t duration = 0;

    for (int i = 0; i < count; i++) {
        CFDFBankChunk *chunk = &bank->chunks[order[i]];

        if (chunk->codec != first->codec || chunk->rate != first->rate)
            return AVERROR_PATCHWELCOME;
        duration += chunk->samples;
    }
    track->order = av_memdup(order, count * sizeof(*order));
    if (!track->order)
        return AVERROR(ENOMEM);
    track->count = count;
    bank->nb_tracks++;
    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = first->codec == 1 ? AV_CODEC_ID_ADPCM_CFDF :
                                               AV_CODEC_ID_CFDF_DPCM;
    st->codecpar->sample_rate = first->rate;
    st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    st->codecpar->extradata = av_mallocz(1 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!st->codecpar->extradata)
        return AVERROR(ENOMEM);
    st->codecpar->extradata[0] = !continuous;
    st->codecpar->extradata_size = 1;
    st->start_time = 0;
    st->duration = duration;
    avpriv_set_pts_info(st, 64, 1, first->rate);
    if (name && *name)
        av_dict_set(&st->metadata, "title", name, 0);
    if (!st->index)
        st->disposition |= AV_DISPOSITION_DEFAULT;
    if (loop) {
        av_dict_set(&st->metadata, "loop_start", "0", 0);
        av_dict_set_int(&st->metadata, "loop_end", duration, 0);
    }
    return 0;
}

int ff_cfdf_bank_open(AVFormatContext *s, CFDFBank **out, int be)
{
    CFDFBank *bank;
    uint8_t header[0x400], **payload = NULL;
    uint32_t *sizes = NULL, *offsets = NULL;
    int *order = NULL, order_count = 0, loop = 0, continuous = 0, disk = 0;
    int ret = AVERROR_INVALIDDATA, proto, v1 = 0;
    int64_t fsize = avio_size(s->pb);
    char title[33] = { 0 };

    if (read_at(s->pb, 0, header, sizeof(header)) < 0 ||
        fsize != dword(header + 4, be))
        return AVERROR_INVALIDDATA;
    bank = av_mallocz(sizeof(*bank));
    if (!bank)
        return AVERROR(ENOMEM);
    *out = bank;
    bank->count = dword(header + 0x14, be);
    if (bank->count < 2 || bank->count > INT16_MAX ||
        0x400LL + bank->count * 4 > fsize)
        return AVERROR_INVALIDDATA;
    proto = memcmp(header + 0x20, "LPPALPPA", 8) &&
            memcmp(header + 0x20, "SONGDFST", 8);
    if (proto && dword(header, be) != 0x10000)
        return AVERROR_INVALIDDATA;
    bank->chunks = av_calloc(bank->count, sizeof(*bank->chunks));
    bank->tracks = av_calloc(bank->count + 1, sizeof(*bank->tracks));
    payload = av_calloc(bank->count, sizeof(*payload));
    sizes = av_calloc(bank->count, sizeof(*sizes));
    offsets = av_calloc(bank->count, sizeof(*offsets));
    order = av_calloc(bank->count + 128, sizeof(*order));
    if (!bank->chunks || !bank->tracks || !payload || !sizes || !offsets || !order) {
        ret = AVERROR(ENOMEM);
        goto done;
    }
    for (int i = 0; i < bank->count; i++) {
        uint8_t raw[8];
        CFDFBankChunk *chunk = &bank->chunks[i];
        unsigned skip, declared;

        if (read_at(s->pb, 0x400LL + 4 * i, raw, 4) < 0)
            goto done;
        offsets[i] = dword(raw, be);
        if (!offsets[i])
            continue;
        if (offsets[i] < 0x400LL + bank->count * 4 ||
            offsets[i] > fsize - 8 || read_at(s->pb, offsets[i], raw, 8) < 0)
            goto done;
        sizes[i] = dword(raw + 4, be);
        if (dword(raw, be) != i || sizes[i] > fsize - offsets[i] - 8 ||
            sizes[i] > INT_MAX)
            goto done;
        payload[i] = av_malloc(sizes[i] + (size_t)AV_INPUT_BUFFER_PADDING_SIZE);
        if (!payload[i]) {
            ret = AVERROR(ENOMEM);
            goto done;
        }
        if (read_at(s->pb, offsets[i] + 8LL, payload[i], sizes[i]) < 0)
            goto done;
        if (proto) {
            if (!i || sizes[i] <= 2)
                continue;
            skip = 2;
            chunk->codec = 1;
            chunk->rate = be ? 22255 : 22050;
            declared = word(payload[i], be) * 370;
        } else {
            if (sizes[i] < 0x30)
                continue;
            chunk->codec = word(payload[i] + 0x1a, be);
            chunk->rate = dword(payload[i] + 0x1c, be);
            skip = dword(payload[i] + 0x2c, be);
            declared = dword(payload[i] + 0x24, be);
            if ((chunk->codec != 1 && chunk->codec != 2) ||
                (chunk->rate != 11025 && chunk->rate != 22050 && chunk->rate != 44100) ||
                skip < 0x30 || skip >= sizes[i])
                continue;
            if (chunk->codec == 2)
                declared /= 2;
        }
        chunk->size = sizes[i] - skip;
        chunk->samples = chunk->codec == 1 ? sample_count(payload[i] + skip, chunk->size) : chunk->size;
        if (!declared || declared > INT_MAX || chunk->samples < declared ||
            (proto && chunk->samples != declared)) {
            chunk->samples = 0;
            continue;
        }
        chunk->padding = chunk->samples - declared;
        chunk->samples = declared;
        chunk->offset = offsets[i] + 8LL + skip;
        chunk->silent = 1;
        for (int j = skip; j < sizes[i]; j++) {
            unsigned b = payload[i][j];
            if (chunk->codec == 1 ? (b != 64 && b < 192) : (b != 0 && b != 128))
                chunk->silent = 0;
        }
    }
    if (payload[0]) {
        unsigned base = 0, groups = 0, count = 0, at = 0;
        uint8_t *p = payload[0];

        if (proto && sizes[0] >= 0x86) {
            base = word(p, be);
            groups = word(p + 2, be);
            count = word(p + 4, be);
            at = 6;
        } else if (!proto && sizes[0] >= 0xba) {
            base = word(p + 0x18, be);
            groups = word(p + 0x1a, be);
            count = word(p + 0x1c, be);
            at = 0x1e;
        }
        if (base && groups && base + groups < bank->count && count && count <= 64) {
            v1 = !proto;
            for (int j = 0; j < count; j++) {
                unsigned selector = word(p + at + j * 2, be);
                unsigned id = base + selector;

                if (!selector || selector > groups || !bank->chunks[id].samples)
                    goto done;
                order[order_count++] = id;
            }
            loop = v1;
            if (v1) {
                name_from(p + 0x9e, sizes[0] - 0x9e, title, 32);
                for (int id = 1; id < bank->count; id++) {
                    unsigned at = 0xba + (id - 1) * 0x18;
                    if (at < sizes[0])
                        name_from(p + at, sizes[0] - at, bank->chunks[id].name, 15);
                }
            }
        }
    }
    if (!proto && !v1 && payload[0]) {
        unsigned block = 1, singles = 0;
        uint8_t *p = payload[0];

        if (sizes[0] >= 0x24) {
            if (av_match_ext(s->url, "snd") && word(p + 2, be) == 4)
                block = dword(p + 0x1c, be);
            singles = dword(p + 0x20, be);
        }
        if (sizes[0] > 0x24)
            name_from(p + 0x24, sizes[0] - 0x24, title, 32);
        if (block && block < bank->count && sizes[block] >= 0x10e) {
            uint8_t *list = payload[block];
            unsigned count = word(list + 4, be);
            unsigned chunks = word(list + 0x10a, be);

            if (count > 130 || chunks > bank->count || 0x10eLL + chunks * 26 > sizes[block])
                goto done;
            disk = continuous = chunks > count;
            order_count = continuous || !count ? chunks : count;
            for (int j = 0; j < order_count; j++) {
                int index = continuous || !count ? j : (int)word(list + 6 + j * 2, be) - 1;
                unsigned id;

                if (index < 0 || index >= chunks)
                    goto done;
                id = word(list + 0x10e + index * 26 + 4, be);
                if (id >= bank->count || !bank->chunks[id].samples)
                    goto done;
                order[j] = id;
                name_from(list + 0x10e + index * 26 + 10, 16, bank->chunks[id].name, 15);
            }
            continuous = continuous && chunks && bank->chunks[order[0]].codec == 2;
            loop = !disk;
        }
        if (singles && singles < bank->count && sizes[singles] >= 8) {
            uint8_t *list = payload[singles];
            unsigned count = word(list + 4, be);

            if (8LL + count * 26 > sizes[singles])
                goto done;
            for (int j = 0; j < count; j++) {
                unsigned id = dword(list + 8 + j * 26 + 4, be);
                if (id < bank->count)
                    name_from(list + 8 + j * 26 + 10, 16, bank->chunks[id].name, 15);
            }
        }
    }
    if (order_count) {
        int first = 0, end = order_count;

        while (first < end && bank->chunks[order[first]].silent)
            first++;
        while (end > first && bank->chunks[order[end - 1]].silent)
            end--;
        if (first == end) {
            first = 0;
            end = order_count;
        }
        ret = add_track(s, bank, order + first, end - first, title, loop, continuous);
        if (ret < 0)
            goto done;
    }
    for (int i = 0; i < bank->count; i++) {
        if (!bank->chunks[i].samples)
            continue;
        if (disk) {
            int in_playlist = 0;
            for (int j = 0; j < order_count; j++)
                in_playlist |= order[j] == i;
            if (in_playlist)
                continue;
        }
        ret = add_track(s, bank, &i, 1, bank->chunks[i].name, 0, 0);
        if (ret < 0)
            goto done;
    }
    ret = bank->nb_tracks ? 0 : AVERROR_INVALIDDATA;
done:
    if (payload)
        for (int i = 0; i < bank->count; i++)
            av_free(payload[i]);
    av_free(payload);
    av_free(sizes);
    av_free(offsets);
    av_free(order);
    return ret;
}

int ff_cfdf_bank_packet(AVFormatContext *s, CFDFBank *bank, AVPacket *pkt)
{
    int selected = -1, ret;
    CFDFBankTrack *track;
    CFDFBankChunk *chunk;

    for (int i = 0; i < bank->nb_tracks; i++) {
        if (bank->tracks[i].cursor >= bank->tracks[i].count)
            continue;
        if (selected < 0 || av_compare_ts(bank->tracks[i].pts, s->streams[i]->time_base,
                                         bank->tracks[selected].pts, s->streams[selected]->time_base) < 0)
            selected = i;
    }
    if (selected < 0)
        return AVERROR_EOF;
    track = &bank->tracks[selected];
    chunk = &bank->chunks[track->order[track->cursor]];
    if (avio_seek(s->pb, chunk->offset, SEEK_SET) < 0)
        return AVERROR_INVALIDDATA;
    ret = av_get_packet(s->pb, pkt, chunk->size);
    if (ret != chunk->size) {
        av_packet_unref(pkt);
        return ret < 0 ? ret : AVERROR_INVALIDDATA;
    }
    if (chunk->padding) {
        uint8_t *skip = av_packet_new_side_data(pkt, AV_PKT_DATA_SKIP_SAMPLES, 10);
        if (!skip) {
            av_packet_unref(pkt);
            return AVERROR(ENOMEM);
        }
        memset(skip, 0, 10);
        AV_WL32(skip + 4, chunk->padding);
    }
    pkt->stream_index = selected;
    pkt->pts = pkt->dts = track->pts;
    pkt->duration = chunk->samples;
    if (s->streams[selected]->codecpar->extradata[0] || !track->cursor)
        pkt->flags |= AV_PKT_FLAG_KEY;
    track->pts += chunk->samples;
    track->cursor++;
    return 0;
}

int ff_cfdf_bank_seek(AVFormatContext *s, CFDFBank *bank, int stream, int64_t ts)
{
    AVRational tb = stream < 0 ? AV_TIME_BASE_Q : s->streams[stream]->time_base;

    for (int i = 0; i < bank->nb_tracks; i++) {
        CFDFBankTrack *track = &bank->tracks[i];
        int64_t target = av_rescale_q(ts, tb, s->streams[i]->time_base);

        track->cursor = 0;
        track->pts = 0;
        if (!s->streams[i]->codecpar->extradata[0])
            continue; /* continuous DPCM must replay its history */
        while (track->cursor < track->count) {
            int samples = bank->chunks[track->order[track->cursor]].samples;
            if (track->pts + samples > target)
                break;
            track->pts += samples;
            track->cursor++;
        }
    }
    return 0;
}

void ff_cfdf_bank_close(CFDFBank **out)
{
    CFDFBank *bank = *out;

    if (!bank)
        return;
    for (int i = 0; i < bank->nb_tracks; i++)
        av_free(bank->tracks[i].order);
    av_free(bank->tracks);
    av_free(bank->chunks);
    av_freep(out);
}
