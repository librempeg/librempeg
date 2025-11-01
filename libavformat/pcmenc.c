/*
 * RAW PCM muxers
 * Copyright (c) 2002 Fabrice Bellard
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

#include "avformat.h"
#include "mux.h"
#include "rawenc.h"

#define PCMDEF_0(name_, long_name_, ext, codec)
#define PCMDEF_1(name_, long_name_, ext, codec)             \
const FFOutputFormat ff_pcm_ ## name_ ## _muxer = {         \
    .p.name        = #name_,                                \
    .p.long_name   = NULL_IF_CONFIG_SMALL(long_name_),      \
    .p.extensions  = (ext) ? #name_ "," #ext : #name_,      \
    .p.audio_codec = codec,                                 \
    .p.video_codec = AV_CODEC_ID_NONE,                      \
    .p.subtitle_codec = AV_CODEC_ID_NONE,                   \
    .p.flags       = AVFMT_NOTIMESTAMPS,                    \
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |      \
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,   \
    .write_packet = ff_raw_write_packet,                    \
};
#define PCMDEF_2(name, long_name, ext, codec, enabled)      \
    PCMDEF_ ## enabled(name, long_name, ext, codec)
#define PCMDEF_3(name, long_name, ext, codec, config)       \
    PCMDEF_2(name, long_name, ext, codec, config)
#define PCMDEF(name, long_name, ext, uppercase)             \
    PCMDEF_3(name, long_name, ext, AV_CODEC_ID_PCM_ ## uppercase, \
             CONFIG_PCM_ ## uppercase ## _MUXER)

PCMDEF(f64be, "PCM 64-bit floating-point big-endian",    AV_NE("f64", NULL), F64BE)
PCMDEF(f64le, "PCM 64-bit floating-point little-endian", AV_NE(NULL, "f64"), F64LE)
PCMDEF(f32be, "PCM 32-bit floating-point big-endian",    AV_NE("f32", NULL), F32BE)
PCMDEF(f32le, "PCM 32-bit floating-point little-endian", AV_NE(NULL, "f32"), F32LE)
PCMDEF(s32be, "PCM signed 32-bit big-endian",      AV_NE("s32", NULL),S32BE)
PCMDEF(s32le, "PCM signed 32-bit little-endian",   AV_NE(NULL, "s32"),S32LE)
PCMDEF(s24be, "PCM signed 24-bit big-endian",      AV_NE("s24", NULL),S24BE)
PCMDEF(s24le, "PCM signed 24-bit little-endian",   AV_NE(NULL, "s24"),S24LE)
PCMDEF(s16be, "PCM signed 16-bit big-endian",      AV_NE("sw,s16", NULL),S16BE)
PCMDEF(s16le, "PCM signed 16-bit little-endian",   AV_NE(NULL," sw,s16"),S16LE)
PCMDEF(s8,    "PCM signed 8-bit",                              "sb,s8",  S8)
PCMDEF(u32be, "PCM unsigned 32-bit big-endian",    AV_NE("u32", NULL),U32BE)
PCMDEF(u32le, "PCM unsigned 32-bit little-endian", AV_NE(NULL, "u32"),U32LE)
PCMDEF(u24be, "PCM unsigned 24-bit big-endian",    AV_NE("u24", NULL),U24BE)
PCMDEF(u24le, "PCM unsigned 24-bit little-endian", AV_NE(NULL, "u24"),U24LE)
PCMDEF(u16be, "PCM unsigned 16-bit big-endian",    AV_NE("uw,u16", NULL),U16BE)
PCMDEF(u16le, "PCM unsigned 16-bit little-endian", AV_NE(NULL, "uw,u16"),U16LE)
PCMDEF(u8,    "PCM unsigned 8-bit",                             "ub,u8", U8)
PCMDEF(alaw,  "PCM A-law",                                      "al",  ALAW)
PCMDEF(mulaw, "PCM mu-law",                                     "ul", MULAW)
PCMDEF(vidc,  "PCM Archimedes VIDC",                            NULL,  VIDC)
