/*
 * TLS/SSL Protocol
 * Copyright (c) 2011 Martin Storsjo
 * Copyright (c) 2025 Jack Lau
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

#ifndef AVFORMAT_TLS_H
#define AVFORMAT_TLS_H

#include "libavutil/bprint.h"
#include "libavutil/opt.h"
#include "version.h"

#include "url.h"

/**
 * Maximum size limit of a certificate and private key size.
 */
#define MAX_CERTIFICATE_SIZE 8192

typedef struct TLSShared {
    const AVClass *class;
    char *ca_file;
    int verify;
    char *cert_file;
    char *key_file;
    int listen;

    char *host;
    char *http_proxy;

    char underlying_host[200];
    int numerichost;

    int external_sock;
    URLContext *udp;
    URLContext *tcp;

    int is_dtls;
    int use_srtp;

    /* The certificate and private key content used for DTLS handshake */
    char* cert_buf;
    char* key_buf;

    /**
     * The size of RTP packet, should generally be set to MTU.
     * Note that pion requires a smaller value, for example, 1200.
     */
    int mtu;
} TLSShared;

#define TLS_OPTFL (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_ENCODING_PARAM)

#if FF_API_NO_DEFAULT_TLS_VERIFY
#define TLS_VERIFY_DEFAULT 0
#else
#define TLS_VERIFY_DEFAULT 1
#endif

#define FF_TLS_CLIENT_OPTIONS(pstruct, options_field) \
    {"ca_file",    "Certificate Authority database file", offsetof(pstruct, options_field . ca_file),   AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"cafile",     "Certificate Authority database file", offsetof(pstruct, options_field . ca_file),   AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"tls_verify", "Verify the peer certificate",         offsetof(pstruct, options_field . verify),    AV_OPT_TYPE_BOOL, { .i64 = TLS_VERIFY_DEFAULT }, 0, 1, .flags = TLS_OPTFL }, \
    {"verify",     "Verify the peer certificate",         offsetof(pstruct, options_field . verify),    AV_OPT_TYPE_BOOL, { .i64 = TLS_VERIFY_DEFAULT }, 0, 1, .flags = TLS_OPTFL }, \
    {"cert_file",  "Certificate file",                    offsetof(pstruct, options_field . cert_file), AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"cert",       "Certificate file",                    offsetof(pstruct, options_field . cert_file), AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"key_file",   "Private key file",                    offsetof(pstruct, options_field . key_file),  AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"key",        "Private key file",                    offsetof(pstruct, options_field . key_file),  AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"verifyhost", "Verify against a specific hostname",  offsetof(pstruct, options_field . host),      AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }

#define TLS_COMMON_OPTIONS(pstruct, options_field) \
    {"listen",     "Listen for incoming connections",     offsetof(pstruct, options_field . listen),    AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1, .flags = TLS_OPTFL }, \
    {"http_proxy", "Set proxy to tunnel through",         offsetof(pstruct, options_field . http_proxy), AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"external_sock", "Use external socket",              offsetof(pstruct, options_field . external_sock), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, .flags = TLS_OPTFL }, \
    {"use_srtp",   "Enable use_srtp DTLS extension",      offsetof(pstruct, options_field . use_srtp),  AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, .flags = TLS_OPTFL }, \
    {"mtu", "Maximum Transmission Unit", offsetof(pstruct, options_field . mtu), AV_OPT_TYPE_INT,  { .i64 = 0 }, 0, INT_MAX, .flags = TLS_OPTFL}, \
    {"cert_pem",   "Certificate PEM string",              offsetof(pstruct, options_field . cert_buf),  AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    {"key_pem",    "Private key PEM string",              offsetof(pstruct, options_field . key_buf),   AV_OPT_TYPE_STRING, .flags = TLS_OPTFL }, \
    FF_TLS_CLIENT_OPTIONS(pstruct, options_field)

int ff_tls_open_underlying(TLSShared *c, URLContext *parent, const char *uri, AVDictionary **options);

int ff_url_read_all(const char *url, AVBPrint *bp);

int ff_tls_set_external_socket(URLContext *h, URLContext *sock);

int ff_dtls_export_materials(URLContext *h, char *dtls_srtp_materials, size_t materials_sz);

int ff_ssl_read_key_cert(char *key_url, char *cert_url, char *key_buf, size_t key_sz, char *cert_buf, size_t cert_sz, char **fingerprint);

int ff_ssl_gen_key_cert(char *key_buf, size_t key_sz, char *cert_buf, size_t cert_sz, char **fingerprint);

void ff_gnutls_init(void);
void ff_gnutls_deinit(void);

int ff_openssl_init(void);
void ff_openssl_deinit(void);

#endif /* AVFORMAT_TLS_H */
