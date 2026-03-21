/*
 * Sega FILM abridged format demuxer
 * Copyright (C) 2026 Librempeg
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

#include "libavutil/intreadwrite.h"
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

/* only enable this when better check against 'AF'/'FA' field is thought up. */
#define PROBE_AF_FIELD 0

typedef struct FilmAbridgedTabEntryContext
{
    unsigned char tab_entry_type;
    int64_t tab_entry_offset;
    int64_t tab_entry_size;
    int64_t tab_entry_duration;
    int64_t starting_duration;
    int64_t overall_duration;
    int current_stream_index;
    int ignore_this_chunk;
} FilmAbridgedTabEntryContext;

typedef struct FilmAbridgedTabContext
{
    int64_t base_offset;
    int64_t tab_header_size;
    int64_t tab_data_size;
    int16_t next_sector;
    int16_t tab_entries;
    int tab_entry;
    int is_fully_allocated;
    FilmAbridgedTabEntryContext *tab_entry_data;
} FilmAbridgedTabContext;

typedef struct FilmAbridgedDemuxContext
{
    int tabs;
    int tab;
    int is_fully_allocated;
    FilmAbridgedTabContext *tab_data;
    int video_stream_index;
    int64_t video_duration;
    int audio_stream_index;
    int64_t audio_duration;
    int duration_fully_obtained;
} FilmAbridgedDemuxContext;

static int film_abridged_check_title_string_header(const int title_string_pos, unsigned char *buffer)
{
    int title_string_dynamic_pos = title_string_pos;
    int valid_chars = 0;

    if (title_string_pos != 6)
        return 0;

    for (int i = 0; i < 6; i++)
    {
        if ((buffer[title_string_dynamic_pos] >= 'A')
            &&
            (buffer[title_string_dynamic_pos] <= 'Z'))
            valid_chars++;
        else if ((buffer[title_string_dynamic_pos] >= '0')
                 &&
                 (buffer[title_string_dynamic_pos] <= '9'))
            valid_chars++;
        else
            break;
    }

    if (valid_chars < 6)
        return 0;

    return 1;
}

static int read_probe(const AVProbeData *p)
{
    #if PROBE_AF_FIELD
    signed short buf_0x04 = 0;
    #endif
    signed short buf_0x0c = 0;

    if (AV_RB32(&p->buf[0]) != MKBETAG('F', 'I', 'L', 'M'))
        return 0;

    #if PROBE_AF_FIELD
    /* (todo) needs better checking than this, current one almost doesn't work.
     * (stops at 'AF'(little-endian)/'FA'(big-endian), then reports invalid file altogether)
     */
    buf_0x04 = AV_RL16(&p->buf[4]);
    if ((buf_0x04 != 0x4146)
        /* 'AF'(little-endian)/'FA'(big-endian)
         * 'AF'(little-endian) is what you're seeing above.
         */
        |
        (buf_0x04 <= 0)
        |
        (buf_0x04 >= 0x770))
        return 0;
    #endif

    if (!film_abridged_check_title_string_header(6,p->buf))
        return 0;

    buf_0x0c = AV_RB16(&p->buf[12]);
    if ((buf_0x0c <= 0) && (buf_0x0c > 0x14a))
        return 0;

    if (AV_RB16(&p->buf[14]) != 0x8140)
        return 0;

    if (AV_RB8(&p->buf[16]))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    FilmAbridgedDemuxContext *ctx = s->priv_data;

    ctx->is_fully_allocated = 0;
    ctx->tabs = -1;
    ctx->tab = -1;
    ctx->tab_data = NULL;
    ctx->video_stream_index = -1;
    ctx->video_duration = 0;
    ctx->audio_stream_index = -1;
    ctx->audio_duration = 0;
    ctx->duration_fully_obtained = -1;

    s->ctx_flags |= AVFMTCTX_NOHEADER;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    FilmAbridgedDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int i, j, i1, j1;
    FilmAbridgedTabContext *curr_tab_data = NULL;
    FilmAbridgedTabContext *prev_tab_data = NULL;
    FilmAbridgedTabContext *temp_tab_data = NULL;
    FilmAbridgedTabEntryContext *tab_entry_data = NULL;
    FilmAbridgedTabEntryContext *prev_tab_entry = NULL;
    AVStream *st;
    int stream_index;
    int ret = 0;

    if (!ctx->is_fully_allocated)
    {
        if (ctx->tabs == -1)
            ctx->tabs = 1;

        for (i = 0; i < ctx->tabs; i++)
        {
            int i_progress = ctx->tabs - i;

            pos = avio_tell(pb);

            if (!ctx->tab_data)
            {
                ctx->tab_data = av_malloc_array(ctx->tabs, sizeof(FilmAbridgedTabContext));
                if (!ctx->tab_data)
                    return AVERROR(ENOMEM);
            }

            if ((ctx->tabs > 1) && (i_progress == 1))
            {
                temp_tab_data = av_realloc_array(ctx->tab_data, ctx->tabs, sizeof(FilmAbridgedTabContext));
                if (!temp_tab_data)
                    return AVERROR(ENOMEM);

                ctx->tab_data = temp_tab_data;

                curr_tab_data = &ctx->tab_data[i];
                prev_tab_data = &ctx->tab_data[i-1];
                curr_tab_data->base_offset = prev_tab_data->base_offset + (prev_tab_data->next_sector * 0x800);
            } else {
                curr_tab_data = &ctx->tab_data[0];
                curr_tab_data->base_offset = pos;
            }

            avio_seek(pb, curr_tab_data->base_offset, SEEK_SET);
            if (avio_feof(pb))
                return AVERROR_EOF;

            curr_tab_data->tab_header_size = 0x800;
            curr_tab_data->tab_data_size = 0;
            curr_tab_data->tab_entry_data = NULL;
            curr_tab_data->tab_entry = -1;
            curr_tab_data->is_fully_allocated = 0;

            avio_skip(pb, 4);
            if (avio_rb16(pb) != 0x4641)
            /* 'AF'(little-endian)/'FA'(big-endian)
             * 'FA'(big-endian) is what you're seeing above.
             */
            {
                avio_seek(pb, -2, SEEK_CUR);
                curr_tab_data->next_sector = avio_rl16(pb);
                if (curr_tab_data->next_sector)
                    ctx->tabs++;
            } else
                curr_tab_data->next_sector = 0;
            avio_skip(pb, 6);
            curr_tab_data->tab_entries = avio_rb16(pb) + 1;

            if (curr_tab_data->tab_entries)
            {
                if (!curr_tab_data->tab_entry_data)
                {
                    curr_tab_data->tab_entry_data = av_malloc_array(curr_tab_data->tab_entries, sizeof(FilmAbridgedTabEntryContext));
                    if (!curr_tab_data->tab_entry_data)
                        return AVERROR(ENOMEM);
                }

                for (j = 0; j < curr_tab_data->tab_entries; j++)
                {
                    tab_entry_data = &curr_tab_data->tab_entry_data[j];

                    tab_entry_data->tab_entry_type = avio_r8(pb);
                    tab_entry_data->tab_entry_size = avio_rb16(pb);
                    tab_entry_data->tab_entry_offset =
                        curr_tab_data->base_offset +
                        curr_tab_data->tab_header_size +
                        curr_tab_data->tab_data_size;
                    tab_entry_data->tab_entry_duration = 0;
                    tab_entry_data->starting_duration = 0;
                    tab_entry_data->overall_duration = 0;
                    tab_entry_data->current_stream_index = -1;
                    tab_entry_data->ignore_this_chunk = 0;

                    curr_tab_data->tab_data_size += tab_entry_data->tab_entry_size;

                    /* denotes "start of tab TOC", literally where the first tab entry is. */
                    if (tab_entry_data->tab_entry_type & 0x80)
                        tab_entry_data->tab_entry_type ^= 0x80;

                    if ((i > 0) && (j == 0) && (tab_entry_data->tab_entry_type == 1))
                        tab_entry_data->ignore_this_chunk = 1;

                    switch (tab_entry_data->tab_entry_type)
                    {
                        case 0:
                            tab_entry_data->tab_entry_duration = 1;
                            tab_entry_data->overall_duration = ctx->video_duration += tab_entry_data->tab_entry_duration;

                            if (ctx->video_stream_index == -1)
                            {
                                st = avformat_new_stream(s, NULL);
                                if (!st)
                                    return AVERROR(ENOMEM);

                                ctx->video_stream_index = st->index;
                                st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
                                st->codecpar->codec_id = AV_CODEC_ID_CINEPAK;
                                st->codecpar->codec_tag = 0;
                                st->codecpar->width = 320;
                                st->codecpar->height = 200;
                                st->start_time = 0;
                                avpriv_set_pts_info(st, 64, 1, 10);
                            }

                            if (tab_entry_data->current_stream_index == -1)
                                tab_entry_data->current_stream_index = ctx->video_stream_index;

                            break;
                        case 1:
                            if (!tab_entry_data->ignore_this_chunk)
                            {
                                tab_entry_data->tab_entry_duration = tab_entry_data->tab_entry_size;
                                tab_entry_data->overall_duration = ctx->audio_duration += tab_entry_data->tab_entry_duration;
                            }

                            if (ctx->audio_stream_index == -1)
                            {
                                st = avformat_new_stream(s, NULL);
                                if (!st)
                                    return AVERROR(ENOMEM);

                                ctx->audio_stream_index = st->index;
                                st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                                st->codecpar->codec_id = AV_CODEC_ID_PCM_SGA;
                                st->codecpar->codec_tag = 1;
                                av_channel_layout_default(&st->codecpar->ch_layout, 1);
                                st->codecpar->sample_rate = 16384;
                                st->start_time = 0;
                                avpriv_set_pts_info(st, 64, 1, 16384);
                            }

                            if (tab_entry_data->current_stream_index == -1)
                                tab_entry_data->current_stream_index = ctx->audio_stream_index;

                            break;
                        default:
                            avpriv_request_sample(s, "codec %X", tab_entry_data->tab_entry_type);
                            return AVERROR_PATCHWELCOME;
                    }

                    if (j > 0)
                    {
                        for (j1 = (j - 1); j1 >= 0; j1--)
                        {
                            if (!j1)
                            {
                                i1 = i - 1;
                                if (i1 >= 0)
                                {
                                    prev_tab_data = &ctx->tab_data[i1];
                                    j1 = prev_tab_data->tab_entries - 1;
                                    curr_tab_data = prev_tab_data;
                                }
                            }

                            prev_tab_entry = &curr_tab_data->tab_entry_data[j1];
                            if (prev_tab_entry->tab_entry_type == tab_entry_data->tab_entry_type)
                            {
                                if (!tab_entry_data->ignore_this_chunk)
                                {
                                    tab_entry_data->starting_duration = prev_tab_entry->overall_duration;
                                    if (i != i1)
                                        curr_tab_data = &ctx->tab_data[i];
                                    break;
                                }
                            }
                        }
                    }
                }

                curr_tab_data->is_fully_allocated = 1;

                if (curr_tab_data->is_fully_allocated)
                    curr_tab_data->tab_entry = 0;
            }
        }

        ctx->is_fully_allocated = 1;

        if (ctx->is_fully_allocated)
        {
            ctx->tab = 0;
            avio_seek(pb, 0, SEEK_SET);
        }
    }

    if (ctx->is_fully_allocated)
    {
        i = ctx->tab;
        if ((i >= ctx->tabs)
            ||
            (i < 0))
            return AVERROR_EOF;

        if ((ctx->duration_fully_obtained == -1)
            &&
            ((ctx->video_stream_index >= 0)
            &&
            (ctx->audio_stream_index >= 0)))
        {
            st = s->streams[ctx->video_stream_index];
            st->nb_frames = st->duration = ctx->video_duration;
            st = s->streams[ctx->audio_stream_index];
            st->duration = ctx->audio_duration;
            ctx->duration_fully_obtained = 1;
        }

        if ((i >= 0) && (i < ctx->tabs))
        {
            curr_tab_data = &ctx->tab_data[i];

            j = curr_tab_data->tab_entry;
            if (j >= curr_tab_data->tab_entries)
            {
                ctx->tab++;
                return ret;
            } else if (j < 0)
                return AVERROR_EOF;

            if ((j >= 0) && (j < curr_tab_data->tab_entries))
            {
                tab_entry_data = &curr_tab_data->tab_entry_data[j];

                if (!tab_entry_data->ignore_this_chunk)
                {
                    switch (tab_entry_data->tab_entry_type)
                    {
                        case 0:
                            stream_index = ctx->video_stream_index;
                            break;
                        case 1:
                            stream_index = ctx->audio_stream_index;
                            break;
                        default:
                            avpriv_request_sample(s, "codec %X", tab_entry_data->tab_entry_type);
                            return AVERROR_PATCHWELCOME;
                    }

                    ret = ffio_ensure_seekback(pb, 2);
                    if (ret < 0)
                        return 0;

                    pos = avio_seek(pb, tab_entry_data->tab_entry_offset, SEEK_SET);
                    if (pos < 0) {
                        return pos;
                    } else if (avio_feof(pb))
                        return AVERROR_EOF;

                    av_log(s, AV_LOG_DEBUG, "TAB %03d; TAB ENTRY CODEC 0x%02x\n", i, tab_entry_data->tab_entry_type);
                    av_log(s, AV_LOG_DEBUG, "TAB %03d; TAB ENTRY OFFSET 0x%08llx\n", i, tab_entry_data->tab_entry_offset);

                    ret = av_get_packet(pb, pkt, tab_entry_data->tab_entry_size);
                    pkt->pos = tab_entry_data->tab_entry_offset;
                    pkt->stream_index = stream_index;
                    pkt->duration = tab_entry_data->tab_entry_duration;
                } else
                    av_log(s, AV_LOG_DEBUG, "TAB %03d; TAB ENTRY DUPE\n", i);

                curr_tab_data->tab_entry++;

                return ret;
            }
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index, int64_t timestamp, int flags)
{
    FilmAbridgedDemuxContext *ctx = s->priv_data;
    FilmAbridgedTabContext *curr_tab_data = NULL;
    FilmAbridgedTabEntryContext *tab_entry_data = NULL;
    int ret = 0;
    int exact_pos_found = 0;
    int64_t target_duration = 0;

    if (stream_index == ctx->video_stream_index) {
        target_duration = ctx->video_duration;
    } else if (stream_index == ctx->audio_stream_index)
        target_duration = ctx->audio_duration;

    if (
        ((timestamp >= 0)
        &&
        (timestamp < target_duration))
       )
    {
        for (int i = 0; i < ctx->tabs; i++)
        {
            curr_tab_data = &ctx->tab_data[i];

            for (int j = 0; j < curr_tab_data->tab_entries; j++)
            {
                tab_entry_data = &curr_tab_data->tab_entry_data[j];

                if (stream_index == tab_entry_data->current_stream_index)
                {
                    if (
                        (timestamp >= tab_entry_data->starting_duration)
                        &&
                        (timestamp < tab_entry_data->overall_duration)
                       )
                    {
                        ctx->tab = i;
                        curr_tab_data->tab_entry = j;
                        exact_pos_found = 1;
                    }
                }

                if (exact_pos_found)
                    break;
            }

            if (exact_pos_found)
                break;
        }
    }

    return ret;
}

static int read_close(AVFormatContext *s)
{
    FilmAbridgedDemuxContext *ctx = s->priv_data;

    if (ctx->tab_data)
    {
        for (int i = 0; i < ctx->tabs; i++)
            if (ctx->tab_data[i].tab_entry_data)
                av_freep(&ctx->tab_data[i].tab_entry_data);
        av_freep(&ctx->tab_data);
    }

    return 0;
}

const FFInputFormat ff_segafilm_abridged_demuxer = {
    .p.name         = "film_abridged",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega FILM (abridged variant)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(FilmAbridgedDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
