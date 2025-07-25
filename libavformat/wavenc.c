/*
 * WAV muxer
 * Copyright (c) 2001, 2002 Fabrice Bellard
 *
 * Sony Wave64 muxer
 * Copyright (c) 2012 Paul B Mahol
 *
 * WAV muxer RF64 support
 * Copyright (c) 2013 Daniel Verkamp <daniel@drv.nu>
 *
 * EBU Tech 3285 - Supplement 3 - Peak Envelope Chunk encoder
 * Copyright (c) 2014 Georg Lippitsch <georg.lippitsch@gmx.at>
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

#include "config_components.h"

#include <stdint.h>
#include <string.h>
#include <time.h>

#include "libavutil/avstring.h"
#include "libavutil/dict.h"
#include "libavutil/common.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/time_internal.h"

#include "avformat.h"
#include "avio.h"
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"
#include "riff.h"

#define RF64_AUTO   (-1)
#define RF64_NEVER  0
#define RF64_ALWAYS 1

typedef enum {
    PEAK_OFF = 0,
    PEAK_ON,
    PEAK_ONLY
} PeakType;

typedef enum {
    PEAK_FORMAT_UINT8 = 1,
    PEAK_FORMAT_UINT16
} PeakFormat;

typedef struct WAVMuxContext {
    const AVClass *class;
    int64_t data;
    int64_t fact_pos;
    int64_t ds64;
    int64_t minpts;
    int64_t maxpts;
    int16_t *peak_maxpos, *peak_maxneg;
    uint32_t peak_num_frames;
    unsigned peak_outbuf_size;
    uint32_t peak_outbuf_bytes;
    unsigned size_increment;
    uint8_t *peak_output;
    int last_duration;
    int write_bext;
    int write_peak;
    int rf64;
    int peak_block_size;
    int peak_format;
    int peak_block_pos;
    int peak_ppv;
    int peak_bps;
} WAVMuxContext;

#if CONFIG_WAV_MUXER
static inline void bwf_write_bext_string(AVFormatContext *s, const char *key, int maxlen)
{
    AVDictionaryEntry *tag;
    size_t len = 0;

    if (tag = av_dict_get(s->metadata, key, NULL, 0)) {
        len = strlen(tag->value);
        len = FFMIN(len, maxlen);
        avio_write(s->pb, tag->value, len);
    }

    ffio_fill(s->pb, 0, maxlen - len);
}

static void bwf_write_bext_chunk(AVFormatContext *s)
{
    AVDictionaryEntry *tmp_tag;
    uint64_t time_reference = 0;
    int64_t bext = ff_start_tag(s->pb, "bext");

    bwf_write_bext_string(s, "description", 256);
    bwf_write_bext_string(s, "originator", 32);
    bwf_write_bext_string(s, "originator_reference", 32);
    bwf_write_bext_string(s, "origination_date", 10);
    bwf_write_bext_string(s, "origination_time", 8);

    if (tmp_tag = av_dict_get(s->metadata, "time_reference", NULL, 0))
        time_reference = strtoll(tmp_tag->value, NULL, 10);
    avio_wl64(s->pb, time_reference);
    avio_wl16(s->pb, 1);  // set version to 1

    if ((tmp_tag = av_dict_get(s->metadata, "umid", NULL, 0)) && strlen(tmp_tag->value) > 2) {
        unsigned char umidpart_str[17] = {0};
        int64_t i;
        uint64_t umidpart;
        size_t len = strlen(tmp_tag->value+2);

        for (i = 0; i < len/16; i++) {
            memcpy(umidpart_str, tmp_tag->value + 2 + (i*16), 16);
            umidpart = strtoull(umidpart_str, NULL, 16);
            avio_wb64(s->pb, umidpart);
        }
        ffio_fill(s->pb, 0, 64 - i*8);
    } else
        ffio_fill(s->pb, 0, 64); // zero UMID

    ffio_fill(s->pb, 0, 190); // Reserved

    if (tmp_tag = av_dict_get(s->metadata, "coding_history", NULL, 0))
        avio_put_str(s->pb, tmp_tag->value);

    ff_end_tag(s->pb, bext);
}

static av_cold void wav_deinit(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;

    av_freep(&wav->peak_maxpos);
    av_freep(&wav->peak_maxneg);
    av_freep(&wav->peak_output);
}

static av_cold int peak_init_writer(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;
    AVCodecParameters *par = s->streams[0]->codecpar;

    if (par->codec_id != AV_CODEC_ID_PCM_S8 &&
        par->codec_id != AV_CODEC_ID_PCM_S16LE &&
        par->codec_id != AV_CODEC_ID_PCM_U8 &&
        par->codec_id != AV_CODEC_ID_PCM_U16LE) {
        av_log(s, AV_LOG_ERROR, "Codec %s not supported for Peak Chunk\n",
               avcodec_get_name(par->codec_id));
        return -1;
    }

    wav->peak_bps = av_get_bits_per_sample(par->codec_id) / 8;

    if (wav->peak_bps == 1 && wav->peak_format == PEAK_FORMAT_UINT16) {
        av_log(s, AV_LOG_ERROR,
               "Writing 16 bit peak for 8 bit audio does not make sense\n");
        return AVERROR(EINVAL);
    }
    if (par->ch_layout.nb_channels > INT_MAX / (wav->peak_bps * wav->peak_ppv))
        return AVERROR(ERANGE);
    wav->size_increment = par->ch_layout.nb_channels * wav->peak_bps * wav->peak_ppv;

    wav->peak_maxpos = av_calloc(par->ch_layout.nb_channels, sizeof(*wav->peak_maxpos));
    wav->peak_maxneg = av_calloc(par->ch_layout.nb_channels, sizeof(*wav->peak_maxneg));
    if (!wav->peak_maxpos || !wav->peak_maxneg)
        goto nomem;

    return 0;

nomem:
    return AVERROR(ENOMEM);
}

static int peak_write_frame(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;
    AVCodecParameters *par = s->streams[0]->codecpar;
    unsigned new_size = wav->peak_outbuf_bytes + wav->size_increment;
    uint8_t *tmp;
    int c;

    if (new_size > INT_MAX) {
        wav->write_peak = PEAK_OFF;
        return AVERROR(ERANGE);
    }
    tmp = av_fast_realloc(wav->peak_output, &wav->peak_outbuf_size, new_size);
    if (!tmp) {
        wav->write_peak = PEAK_OFF;
        return AVERROR(ENOMEM);
    }
    wav->peak_output = tmp;

    for (c = 0; c < par->ch_layout.nb_channels; c++) {
        wav->peak_maxneg[c] = -wav->peak_maxneg[c];

        if (wav->peak_bps == 2 && wav->peak_format == PEAK_FORMAT_UINT8) {
            wav->peak_maxpos[c] = wav->peak_maxpos[c] / 256;
            wav->peak_maxneg[c] = wav->peak_maxneg[c] / 256;
        }

        if (wav->peak_ppv == 1)
            wav->peak_maxpos[c] =
                FFMAX(wav->peak_maxpos[c], wav->peak_maxneg[c]);

        if (wav->peak_format == PEAK_FORMAT_UINT8) {
            wav->peak_output[wav->peak_outbuf_bytes++] =
                wav->peak_maxpos[c];
            if (wav->peak_ppv == 2) {
                wav->peak_output[wav->peak_outbuf_bytes++] =
                    wav->peak_maxneg[c];
            }
        } else {
            AV_WL16(wav->peak_output + wav->peak_outbuf_bytes,
                    wav->peak_maxpos[c]);
            wav->peak_outbuf_bytes += 2;
            if (wav->peak_ppv == 2) {
                AV_WL16(wav->peak_output + wav->peak_outbuf_bytes,
                        wav->peak_maxneg[c]);
                wav->peak_outbuf_bytes += 2;
            }
        }
        wav->peak_maxpos[c] = 0;
        wav->peak_maxneg[c] = 0;
    }
    wav->peak_num_frames++;

    return 0;
}

static int peak_write_chunk(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par = s->streams[0]->codecpar;
    int64_t peak = ff_start_tag(s->pb, "levl");
    int64_t now0;
    time_t now_secs;
    char timestamp[28];

    /* Peak frame of incomplete block at end */
    if (wav->peak_block_pos) {
        int ret = peak_write_frame(s);
        if (ret < 0)
            return ret;
    }

    memset(timestamp, 0, sizeof(timestamp));
    if (!(s->flags & AVFMT_FLAG_BITEXACT)) {
        struct tm tmpbuf;
        av_log(s, AV_LOG_INFO, "Writing local time and date to Peak Envelope Chunk\n");
        now0 = av_gettime();
        now_secs = now0 / 1000000;
        if (strftime(timestamp, sizeof(timestamp), "%Y:%m:%d:%H:%M:%S:", localtime_r(&now_secs, &tmpbuf))) {
            av_strlcatf(timestamp, sizeof(timestamp), "%03d", (int)((now0 / 1000) % 1000));
        } else {
            av_log(s, AV_LOG_ERROR, "Failed to write timestamp\n");
            return -1;
        }
    }

    avio_wl32(pb, 1);                           /* version */
    avio_wl32(pb, wav->peak_format);            /* 8 or 16 bit */
    avio_wl32(pb, wav->peak_ppv);               /* positive and negative */
    avio_wl32(pb, wav->peak_block_size);        /* frames per value */
    avio_wl32(pb, par->ch_layout.nb_channels);  /* number of channels */
    avio_wl32(pb, wav->peak_num_frames);        /* number of peak frames */
    avio_wl32(pb, -1);                          /* audio sample frame position (not implemented) */
    avio_wl32(pb, 128);                         /* equal to size of header */
    avio_write(pb, timestamp, 28);              /* ASCII time stamp */
    ffio_fill(pb, 0, 60);

    avio_write(pb, wav->peak_output, wav->peak_outbuf_bytes);

    ff_end_tag(pb, peak);

    if (!wav->data)
        wav->data = peak;

    return 0;
}

static int wav_write_header(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t fmt;

    if (wav->rf64 == RF64_ALWAYS) {
        ffio_wfourcc(pb, "RF64");
        avio_wl32(pb, -1); /* RF64 chunk size: use size in ds64 */
    } else {
        ffio_wfourcc(pb, "RIFF");
        avio_wl32(pb, -1); /* file length */
    }

    ffio_wfourcc(pb, "WAVE");

    if (wav->rf64 != RF64_NEVER) {
        /* write empty ds64 chunk or JUNK chunk to reserve space for ds64 */
        ffio_wfourcc(pb, wav->rf64 == RF64_ALWAYS ? "ds64" : "JUNK");
        avio_wl32(pb, 28); /* chunk size */
        wav->ds64 = avio_tell(pb);
        ffio_fill(pb, 0, 28);
    }

    if (wav->write_peak != PEAK_ONLY) {
        /* format header */
        fmt = ff_start_tag(pb, "fmt ");
        if (ff_put_wav_header(s, pb, s->streams[0]->codecpar, 0) < 0) {
            av_log(s, AV_LOG_ERROR, "Codec %s not supported in WAVE format\n",
                   avcodec_get_name(s->streams[0]->codecpar->codec_id));
            return AVERROR(ENOSYS);
        }
        ff_end_tag(pb, fmt);
    }

    if (s->streams[0]->codecpar->codec_tag != 0x01 /* hence for all other than PCM */
        && (s->pb->seekable & AVIO_SEEKABLE_NORMAL)) {
        wav->fact_pos = ff_start_tag(pb, "fact");
        avio_wl32(pb, 0);
        ff_end_tag(pb, wav->fact_pos);
    }

    if (wav->write_bext)
        bwf_write_bext_chunk(s);

    if (wav->write_peak) {
        int ret;
        if ((ret = peak_init_writer(s)) < 0)
            return ret;
    }

    avpriv_set_pts_info(s->streams[0], 64, 1, s->streams[0]->codecpar->sample_rate);
    wav->maxpts = wav->last_duration = 0;
    wav->minpts = INT64_MAX;

    if (wav->write_peak != PEAK_ONLY) {
        /* info header */
        ff_riff_write_info(s);

        /* data header */
        wav->data = ff_start_tag(pb, "data");
    }

    return 0;
}

static int wav_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb  = s->pb;
    WAVMuxContext    *wav = s->priv_data;

    if (wav->write_peak != PEAK_ONLY)
        avio_write(pb, pkt->data, pkt->size);

    if (wav->write_peak) {
        int c = 0;
        int i;
        for (i = 0; i < pkt->size; i += wav->peak_bps) {
            if (wav->peak_bps == 1) {
                wav->peak_maxpos[c] = FFMAX(wav->peak_maxpos[c], *(int8_t*)(pkt->data + i));
                wav->peak_maxneg[c] = FFMIN(wav->peak_maxneg[c], *(int8_t*)(pkt->data + i));
            } else {
                wav->peak_maxpos[c] = FFMAX(wav->peak_maxpos[c], (int16_t)AV_RL16(pkt->data + i));
                wav->peak_maxneg[c] = FFMIN(wav->peak_maxneg[c], (int16_t)AV_RL16(pkt->data + i));
            }
            if (++c == s->streams[0]->codecpar->ch_layout.nb_channels) {
                c = 0;
                if (++wav->peak_block_pos == wav->peak_block_size) {
                    int ret = peak_write_frame(s);
                    if (ret < 0)
                        return ret;
                    wav->peak_block_pos = 0;
                }
            }
        }
    }

    if(pkt->pts != AV_NOPTS_VALUE) {
        wav->minpts        = FFMIN(wav->minpts, pkt->pts);
        wav->maxpts        = FFMAX(wav->maxpts, pkt->pts);
        wav->last_duration = pkt->duration;
    } else
        av_log(s, AV_LOG_ERROR, "wav_write_packet: NOPTS\n");
    return 0;
}

static int wav_write_trailer(AVFormatContext *s)
{
    AVIOContext *pb  = s->pb;
    WAVMuxContext    *wav = s->priv_data;
    int64_t file_size, data_size;
    int64_t number_of_samples = 0;
    int rf64 = 0;
    int ret = 0;

    if (s->pb->seekable & AVIO_SEEKABLE_NORMAL) {
        if (wav->write_peak != PEAK_ONLY && avio_tell(pb) - wav->data < UINT32_MAX) {
            ff_end_tag(pb, wav->data);
        }

        if (wav->write_peak && wav->peak_output) {
            ret = peak_write_chunk(s);
        }

        /* update file size */
        file_size = avio_tell(pb);
        data_size = file_size - wav->data;
        if (wav->rf64 == RF64_ALWAYS || (wav->rf64 == RF64_AUTO && file_size - 8 > UINT32_MAX)) {
            rf64 = 1;
        } else if (file_size - 8 <= UINT32_MAX) {
            avio_seek(pb, 4, SEEK_SET);
            avio_wl32(pb, (uint32_t)(file_size - 8));
            avio_seek(pb, file_size, SEEK_SET);
        } else {
            av_log(s, AV_LOG_ERROR,
                   "Filesize %"PRId64" invalid for wav, output file will be broken\n",
                   file_size);
        }
        number_of_samples = av_rescale_q(wav->maxpts - wav->minpts + wav->last_duration,
                                       s->streams[0]->time_base,
                                       av_make_q(1, s->streams[0]->codecpar->sample_rate));

        if(s->streams[0]->codecpar->codec_tag != 0x01) {
            /* Update num_samps in fact chunk */
            avio_seek(pb, wav->fact_pos, SEEK_SET);
            if (rf64 || (wav->rf64 == RF64_AUTO && number_of_samples > UINT32_MAX)) {
                rf64 = 1;
                avio_wl32(pb, -1);
            } else {
                avio_wl32(pb, number_of_samples);
                avio_seek(pb, file_size, SEEK_SET);
            }
        }

        if (rf64) {
            /* overwrite RIFF with RF64 */
            avio_seek(pb, 0, SEEK_SET);
            ffio_wfourcc(pb, "RF64");
            avio_wl32(pb, -1);

            /* write ds64 chunk (overwrite JUNK if rf64 == RF64_AUTO) */
            avio_seek(pb, wav->ds64 - 8, SEEK_SET);
            ffio_wfourcc(pb, "ds64");
            avio_wl32(pb, 28);                  /* ds64 chunk size */
            avio_wl64(pb, file_size - 8);       /* RF64 chunk size */
            avio_wl64(pb, data_size);           /* data chunk size */
            avio_wl64(pb, number_of_samples);   /* fact chunk number of samples */
            avio_wl32(pb, 0);                   /* number of table entries for non-'data' chunks */

            /* write -1 in data chunk size */
            avio_seek(pb, wav->data - 4, SEEK_SET);
            avio_wl32(pb, -1);

            avio_seek(pb, file_size, SEEK_SET);
        }
    }

    return ret;
}

#define OFFSET(x) offsetof(WAVMuxContext, x)
#define ENC AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "write_bext", "Write BEXT chunk.", OFFSET(write_bext), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { "write_peak", "Write Peak Envelope chunk.",            OFFSET(write_peak), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, ENC, .unit = "peak" },
    { "off",        "Do not write peak chunk.",              0,                  AV_OPT_TYPE_CONST, { .i64 = PEAK_OFF  }, 0, 0, ENC, .unit = "peak" },
    { "on",         "Append peak chunk after wav data.",     0,                  AV_OPT_TYPE_CONST, { .i64 = PEAK_ON   }, 0, 0, ENC, .unit = "peak" },
    { "only",       "Write only peak chunk, omit wav data.", 0,                  AV_OPT_TYPE_CONST, { .i64 = PEAK_ONLY }, 0, 0, ENC, .unit = "peak" },
    { "rf64",       "Use RF64 header rather than RIFF for large files.",    OFFSET(rf64), AV_OPT_TYPE_INT,   { .i64 = RF64_NEVER  },-1, 1, ENC, .unit = "rf64" },
    { "auto",       "Write RF64 header if file grows large enough.",        0,            AV_OPT_TYPE_CONST, { .i64 = RF64_AUTO   }, 0, 0, ENC, .unit = "rf64" },
    { "always",     "Always write RF64 header regardless of file size.",    0,            AV_OPT_TYPE_CONST, { .i64 = RF64_ALWAYS }, 0, 0, ENC, .unit = "rf64" },
    { "never",      "Never write RF64 header regardless of file size.",     0,            AV_OPT_TYPE_CONST, { .i64 = RF64_NEVER  }, 0, 0, ENC, .unit = "rf64" },
    { "peak_block_size", "Number of audio samples used to generate each peak frame.",   OFFSET(peak_block_size), AV_OPT_TYPE_INT, { .i64 = 256 }, 0, 65536, ENC },
    { "peak_format",     "The format of the peak envelope data (1: uint8, 2: uint16).", OFFSET(peak_format), AV_OPT_TYPE_INT,     { .i64 = PEAK_FORMAT_UINT16 }, PEAK_FORMAT_UINT8, PEAK_FORMAT_UINT16, ENC },
    { "peak_ppv",        "Number of peak points per peak value (1 or 2).",              OFFSET(peak_ppv), AV_OPT_TYPE_INT, { .i64 = 2 }, 1, 2, ENC },
    { NULL },
};

static const AVClass wav_muxer_class = {
    .class_name = "WAV muxer",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_wav_muxer = {
    .p.name            = "wav",
    .p.long_name       = NULL_IF_CONFIG_SMALL("WAV / WAVE (Waveform Audio)"),
    .p.mime_type       = "audio/x-wav",
    .p.extensions      = "wav",
    .priv_data_size    = sizeof(WAVMuxContext),
    .p.audio_codec     = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec     = AV_CODEC_ID_NONE,
    .p.subtitle_codec  = AV_CODEC_ID_NONE,
    .write_header      = wav_write_header,
    .write_packet      = wav_write_packet,
    .write_trailer     = wav_write_trailer,
    .deinit            = wav_deinit,
    .p.flags           = AVFMT_TS_NONSTRICT,
    .p.codec_tag       = ff_wav_codec_tags_list,
    .p.priv_class      = &wav_muxer_class,
    .flags_internal    = FF_OFMT_FLAG_MAX_ONE_OF_EACH,
};
#endif /* CONFIG_WAV_MUXER */

#if CONFIG_W64_MUXER
#include "w64.h"

static void start_guid(AVIOContext *pb, const uint8_t *guid, int64_t *pos)
{
    *pos = avio_tell(pb);

    avio_write(pb, guid, 16);
    avio_wl64(pb, INT64_MAX);
}

static void end_guid(AVIOContext *pb, int64_t start)
{
    int64_t end, pos = avio_tell(pb);

    end = FFALIGN(pos, 8);
    ffio_fill(pb, 0, end - pos);
    avio_seek(pb, start + 16, SEEK_SET);
    avio_wl64(pb, end - start);
    avio_seek(pb, end, SEEK_SET);
}

static int w64_write_header(AVFormatContext *s)
{
    WAVMuxContext *wav = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t start;
    int ret;

    avio_write(pb, ff_w64_guid_riff, sizeof(ff_w64_guid_riff));
    avio_wl64(pb, -1);
    avio_write(pb, ff_w64_guid_wave, sizeof(ff_w64_guid_wave));
    start_guid(pb, ff_w64_guid_fmt, &start);
    if ((ret = ff_put_wav_header(s, pb, s->streams[0]->codecpar, 0)) < 0) {
        av_log(s, AV_LOG_ERROR, "Codec %s not supported\n",
               avcodec_get_name(s->streams[0]->codecpar->codec_id));
        return ret;
    }
    end_guid(pb, start);

    if (s->streams[0]->codecpar->codec_tag != 0x01 /* hence for all other than PCM */
        && (s->pb->seekable & AVIO_SEEKABLE_NORMAL)) {
        start_guid(pb, ff_w64_guid_fact, &wav->fact_pos);
        avio_wl64(pb, 0);
        end_guid(pb, wav->fact_pos);
    }

    start_guid(pb, ff_w64_guid_data, &wav->data);

    return 0;
}

static int w64_write_trailer(AVFormatContext *s)
{
    AVIOContext    *pb = s->pb;
    WAVMuxContext *wav = s->priv_data;
    int64_t file_size;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        end_guid(pb, wav->data);

        file_size = avio_tell(pb);
        avio_seek(pb, 16, SEEK_SET);
        avio_wl64(pb, file_size);

        if (s->streams[0]->codecpar->codec_tag != 0x01) {
            int64_t number_of_samples;

            number_of_samples = av_rescale(wav->maxpts - wav->minpts + wav->last_duration,
                                           s->streams[0]->codecpar->sample_rate * (int64_t)s->streams[0]->time_base.num,
                                           s->streams[0]->time_base.den);
            avio_seek(pb, wav->fact_pos + 24, SEEK_SET);
            avio_wl64(pb, number_of_samples);
        }

        avio_seek(pb, file_size, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_w64_muxer = {
    .p.name            = "w64",
    .p.long_name       = NULL_IF_CONFIG_SMALL("Sony Wave64"),
    .p.extensions      = "w64",
    .priv_data_size    = sizeof(WAVMuxContext),
    .p.audio_codec     = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec     = AV_CODEC_ID_NONE,
    .p.subtitle_codec  = AV_CODEC_ID_NONE,
    .write_header      = w64_write_header,
    .write_packet      = wav_write_packet,
    .write_trailer     = w64_write_trailer,
    .p.flags           = AVFMT_TS_NONSTRICT,
    .p.codec_tag       = ff_wav_codec_tags_list,
    .flags_internal    = FF_OFMT_FLAG_MAX_ONE_OF_EACH,
};
#endif /* CONFIG_W64_MUXER */
