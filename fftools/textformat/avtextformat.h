/*
 * Copyright (c) The FFmpeg developers
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

#ifndef FFTOOLS_TEXTFORMAT_AVTEXTFORMAT_H
#define FFTOOLS_TEXTFORMAT_AVTEXTFORMAT_H

#include <stdint.h>
#include "libavutil/dict.h"
#include "libavformat/avio.h"
#include "libavutil/bprint.h"
#include "libavutil/rational.h"
#include "libavutil/hash.h"
#include "avtextwriters.h"

#define SECTION_MAX_NB_CHILDREN 11


typedef struct AVTextFormatSection {
    int id;             ///< unique id identifying a section
    const char *name;

#define AV_TEXTFORMAT_SECTION_FLAG_IS_WRAPPER      1 ///< the section only contains other sections, but has no data at its own level
#define AV_TEXTFORMAT_SECTION_FLAG_IS_ARRAY        2 ///< the section contains an array of elements of the same type
#define AV_TEXTFORMAT_SECTION_FLAG_HAS_VARIABLE_FIELDS 4 ///< the section may contain a variable number of fields with variable keys.
                                           ///  For these sections the element_name field is mandatory.
#define AV_TEXTFORMAT_SECTION_FLAG_HAS_TYPE        8 ///< the section contains a type to distinguish multiple nested elements
#define AV_TEXTFORMAT_SECTION_FLAG_NUMBERING_BY_TYPE 16 ///< the items in this array section should be numbered individually by type

    int flags;
    const int children_ids[SECTION_MAX_NB_CHILDREN + 1]; ///< list of children section IDS, terminated by -1
    const char *element_name; ///< name of the contained element, if provided
    const char *unique_name;  ///< unique section name, in case the name is ambiguous
    AVDictionary *entries_to_show;
    const char *(*get_type)(const void *data); ///< function returning a type if defined, must be defined when SECTION_FLAG_HAS_TYPE is defined
    int show_all_entries;
} AVTextFormatSection;

typedef struct AVTextFormatContext AVTextFormatContext;

#define AV_TEXTFORMAT_FLAG_SUPPORTS_OPTIONAL_FIELDS 1
#define AV_TEXTFORMAT_FLAG_SUPPORTS_MIXED_ARRAY_CONTENT 2

typedef enum {
    AV_TEXTFORMAT_STRING_VALIDATION_FAIL,
    AV_TEXTFORMAT_STRING_VALIDATION_REPLACE,
    AV_TEXTFORMAT_STRING_VALIDATION_IGNORE,
    AV_TEXTFORMAT_STRING_VALIDATION_NB
} StringValidation;

typedef struct AVTextFormatter {
    const AVClass *priv_class;      ///< private class of the formatter, if any
    int priv_size;                  ///< private size for the formatter context
    const char *name;

    int  (*init)  (AVTextFormatContext *tctx);
    void (*uninit)(AVTextFormatContext *tctx);

    void (*print_section_header)(AVTextFormatContext *tctx, const void *data);
    void (*print_section_footer)(AVTextFormatContext *tctx);
    void (*print_integer)       (AVTextFormatContext *tctx, const char *, int64_t);
    void (*print_string)        (AVTextFormatContext *tctx, const char *, const char *);
    int flags;                  ///< a combination or AV_TEXTFORMAT__FLAG_*
} AVTextFormatter;

#define SECTION_MAX_NB_LEVELS    12
#define SECTION_MAX_NB_SECTIONS 100

struct AVTextFormatContext {
    const AVClass *class;              ///< class of the formatter
    const AVTextFormatter *formatter;  ///< the AVTextFormatter of which this is an instance
    AVTextWriterContext *writer;       ///< the AVTextWriterContext

    char *name;                        ///< name of this formatter instance
    void *priv;                        ///< private data for use by the filter

    const AVTextFormatSection *sections; ///< array containing all sections
    int nb_sections;                   ///< number of sections

    int level;                         ///< current level, starting from 0

    /** number of the item printed in the given section, starting from 0 */
    unsigned int nb_item[SECTION_MAX_NB_LEVELS];
    unsigned int nb_item_type[SECTION_MAX_NB_LEVELS][SECTION_MAX_NB_SECTIONS];

    /** section per each level */
    const AVTextFormatSection *section[SECTION_MAX_NB_LEVELS];
    AVBPrint section_pbuf[SECTION_MAX_NB_LEVELS]; ///< generic print buffer dedicated to each section,
                                                  ///  used by various formatters

    int show_optional_fields;
    int show_value_unit;
    int use_value_prefix;
    int use_byte_value_binary_prefix;
    int use_value_sexagesimal_format;

    struct AVHashContext *hash;

    int string_validation;
    char *string_validation_replacement;
    unsigned int string_validation_utf8_flags;
};

typedef struct AVTextFormatOptions {
    int show_optional_fields;
    int show_value_unit;
    int use_value_prefix;
    int use_byte_value_binary_prefix;
    int use_value_sexagesimal_format;
} AVTextFormatOptions;

#define AV_TEXTFORMAT_PRINT_STRING_OPTIONAL 1
#define AV_TEXTFORMAT_PRINT_STRING_VALIDATE 2

int avtext_context_open(AVTextFormatContext **ptctx, const AVTextFormatter *formatter, AVTextWriterContext *writer_context, const char *args,
                        const AVTextFormatSection *sections, int nb_sections, AVTextFormatOptions options, char *show_data_hash);

void avtext_context_close(AVTextFormatContext **tctx);


void avtext_print_section_header(AVTextFormatContext *tctx, const void *data, int section_id);

void avtext_print_section_footer(AVTextFormatContext *tctx);

void avtext_print_integer(AVTextFormatContext *tctx, const char *key, int64_t val, int flags);

int avtext_print_string(AVTextFormatContext *tctx, const char *key, const char *val, int flags);

void avtext_print_unit_int(AVTextFormatContext *tctx, const char *key, int64_t value, const char *unit);

void avtext_print_rational(AVTextFormatContext *tctx, const char *key, AVRational q, char sep);

void avtext_print_time(AVTextFormatContext *tctx, const char *key, int64_t ts, const AVRational *time_base, int is_duration);

void avtext_print_ts(AVTextFormatContext *tctx, const char *key, int64_t ts, int is_duration);

void avtext_print_data(AVTextFormatContext *tctx, const char *key, const uint8_t *data, int size);

void avtext_print_data_hash(AVTextFormatContext *tctx, const char *key, const uint8_t *data, int size);

void avtext_print_integers(AVTextFormatContext *tctx, const char *key, uint8_t *data, int size,
                           const char *format, int columns, int bytes, int offset_add);

const AVTextFormatter *avtext_get_formatter_by_name(const char *name);

extern const AVTextFormatter avtextformatter_default;
extern const AVTextFormatter avtextformatter_compact;
extern const AVTextFormatter avtextformatter_csv;
extern const AVTextFormatter avtextformatter_flat;
extern const AVTextFormatter avtextformatter_ini;
extern const AVTextFormatter avtextformatter_json;
extern const AVTextFormatter avtextformatter_xml;

#endif /* FFTOOLS_TEXTFORMAT_AVTEXTFORMAT_H */
