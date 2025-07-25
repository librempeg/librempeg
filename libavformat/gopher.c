/*
 * Gopher protocol
 *
 * Copyright (c) 2009 Toshimitsu Kimura
 * Copyright (c) 2021 parazyd <parazyd@dyne.org>
 *
 * based on libavformat/http.c, Copyright (c) 2000, 2001 Fabrice Bellard
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

#include "config.h"
#include "config_components.h"

#include "libavutil/avstring.h"
#include "avformat.h"
#include "internal.h"
#include "network.h"
#include "url.h"

typedef struct GopherContext {
    URLContext *hd;
} GopherContext;

static int gopher_write(URLContext *h, const uint8_t *buf, int size)
{
    GopherContext *s = h->priv_data;
    return ffurl_write(s->hd, buf, size);
}

static int gopher_connect(URLContext *h, const char *path)
{
    char buffer[1024];

    if (!*path) return AVERROR(EINVAL);
    switch (*++path) {
        case ';':
        case '<':
        case '5':
        case '9':
        case 's':
            path = strchr(path, '/');
            if (!path) return AVERROR(EINVAL);
            break;
        default:
            av_log(h, AV_LOG_WARNING,
                   "Gopher protocol type '%c' not supported yet!\n",
                   *path);
            return AVERROR(EINVAL);
    }

    /* send gopher sector */
    snprintf(buffer, sizeof(buffer), "%s\r\n", path);

    if (gopher_write(h, buffer, strlen(buffer)) < 0)
        return AVERROR(EIO);

    return 0;
}

static int gopher_close(URLContext *h)
{
    GopherContext *s = h->priv_data;
    ffurl_closep(&s->hd);
    return 0;
}

static int gopher_open(URLContext *h, const char *uri, int flags)
{
    GopherContext *s = h->priv_data;
    char proto[10], hostname[1024], auth[1024], path[1024], buf[1024];
    int port, err;
    const char *lower_proto = "tcp";

    h->is_streamed = 1;

    /* needed in any case to build the host string */
    av_url_split(proto, sizeof(proto), auth, sizeof(auth),
                 hostname, sizeof(hostname), &port, path, sizeof(path), uri);

    if (port < 0)
        port = 70;

    if (!strcmp(proto, "gophers"))
        lower_proto = "tls";

    ff_url_join(buf, sizeof(buf), lower_proto, NULL, hostname, port, NULL);

    s->hd = NULL;
    err = ffurl_open_whitelist(&s->hd, buf, AVIO_FLAG_READ_WRITE,
                               &h->interrupt_callback, NULL, h->protocol_whitelist, h->protocol_blacklist, h);
    if (err < 0)
        goto fail;

    if ((err = gopher_connect(h, path)) < 0)
        goto fail;
    return 0;
 fail:
    gopher_close(h);
    return err;
}

static int gopher_read(URLContext *h, uint8_t *buf, int size)
{
    GopherContext *s = h->priv_data;
    int len = ffurl_read(s->hd, buf, size);
    return len;
}

#if CONFIG_GOPHER_PROTOCOL
const URLProtocol ff_gopher_protocol = {
    .name              = "gopher",
    .url_open          = gopher_open,
    .url_read          = gopher_read,
    .url_write         = gopher_write,
    .url_close         = gopher_close,
    .priv_data_size    = sizeof(GopherContext),
    .flags             = URL_PROTOCOL_FLAG_NETWORK,
    .default_whitelist = "gopher,tcp"
};
#endif /* CONFIG_GOPHER_PROTOCOL */

#if CONFIG_GOPHERS_PROTOCOL
const URLProtocol ff_gophers_protocol = {
    .name              = "gophers",
    .url_open          = gopher_open,
    .url_read          = gopher_read,
    .url_write         = gopher_write,
    .url_close         = gopher_close,
    .priv_data_size    = sizeof(GopherContext),
    .flags             = URL_PROTOCOL_FLAG_NETWORK,
    .default_whitelist = "gopher,gophers,tcp,tls"
};
#endif /* CONFIG_GOPHERS_PROTOCOL */
