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

#undef map_type
#undef INAME
#undef pixel_type
#if MAP_TYPE == 32
#define map_type float
#define INAME linear
# if SOURCE_TYPE == 8
# define pixel_type uint8_t
# else
# define pixel_type uint16_t
# endif
#else
#define map_type uint16_t
#define INAME nearest
# if SOURCE_TYPE == 8
# define pixel_type uint8_t
# else
# define pixel_type uint16_t
# endif
#endif

#define fn3(a,b,c) a##b##_##c##_slice
#define fn2(a,b,c) fn3(a,b,c)
#define fn(a)      fn2(a, SOURCE_TYPE, INAME)

/**
 * remap_planar algorithm expects planes of same size
 * pixels are copied from source to target using :
 * Target_frame[y][x] = Source_frame[ ymap[y][x] ][ [xmap[y][x] ];
 */
static int fn(remap_planar)(AVFilterContext *ctx, void *arg,
                            int jobnr, int nb_jobs)
{
    RemapContext *s = ctx->priv;
    const ThreadData *td = arg;
    const AVFrame *in  = td->in;
    const AVFrame *xin = td->xin;
    const AVFrame *yin = td->yin;
    const AVFrame *out = td->out;
    const int slice_start = (out->height *  jobnr   ) / nb_jobs;
    const int slice_end   = (out->height * (jobnr+1)) / nb_jobs;
    const int xlinesize = xin->linesize[0] / sizeof(map_type);
    const int ylinesize = yin->linesize[0] / sizeof(map_type);
    const int out_width = out->width;
    const int in_height = in->height;
    const int in_width = in->width;

    for (int plane = 0; plane < td->nb_planes ; plane++) {
        const int dlinesize  = out->linesize[plane] / sizeof(pixel_type);
        const pixel_type *src = (const pixel_type *)in->data[plane];
        pixel_type *dst = (pixel_type *)out->data[plane] + slice_start * dlinesize;
        const int slinesize  = in->linesize[plane] / sizeof(pixel_type);
        const map_type *xmap = (const map_type *)xin->data[0] + slice_start * xlinesize;
        const map_type *ymap = (const map_type *)yin->data[0] + slice_start * ylinesize;
        const int color = s->fill_color[plane];

        for (int y = slice_start; y < slice_end; y++) {
            for (int x = 0; x < out_width; x++) {
#if MAP_TYPE == 32
                const int xm = floorf(xmap[x]);
                const int ym = floorf(ymap[x]);

                if (ym >= 0 && ym < in_height && xm >= 0 && xm < in_width) {
                    const int xn = FFMIN(xm + 1, in_width -1);
                    const int yn = FFMIN(ym + 1, in_height-1);
                    const float du = xmap[x] - xm;
                    const float dv = ymap[x] - ym;
                    float p0 = src[ym * slinesize + xm];
                    float p1 = src[ym * slinesize + xn];
                    float p2 = src[yn * slinesize + xm];
                    float p3 = src[yn * slinesize + xn];
                    float sum = 0.f;

                    sum += (1.f - du) * (1.f - dv) * p0;
                    sum += (      du) * (1.f - dv) * p1;
                    sum += (1.f - du) * (      dv) * p2;
                    sum += (      du) * (      dv) * p3;

                    dst[x] = lrintf(sum);
                } else {
                    dst[x] = color;
                }
#else
                if (ymap[x] < in_height && xmap[x] < in_width) {
                    dst[x] = src[ymap[x] * slinesize + xmap[x]];
                } else {
                    dst[x] = color;
                }
#endif
            }

            dst  += dlinesize;
            xmap += xlinesize;
            ymap += ylinesize;
        }
    }

    return 0;
}

/**
 * remap_packed algorithm expects pixels with both padded bits (step) and
 * number of components correctly set.
 * pixels are copied from source to target using :
 * Target_frame[y][x] = Source_frame[ ymap[y][x] ][ [xmap[y][x] ];
 */
static int fn(remap_packed)(AVFilterContext *ctx, void *arg,
                            int jobnr, int nb_jobs)
{
    RemapContext *s = ctx->priv;
    const ThreadData *td = arg;
    const AVFrame *in  = td->in;
    const AVFrame *xin = td->xin;
    const AVFrame *yin = td->yin;
    const AVFrame *out = td->out;
    const int slice_start = (out->height *  jobnr   ) / nb_jobs;
    const int slice_end   = (out->height * (jobnr+1)) / nb_jobs;
    const int dlinesize  = out->linesize[0] / sizeof(pixel_type);
    const int slinesize  = in->linesize[0] / sizeof(pixel_type);
    const int xlinesize  = xin->linesize[0] / sizeof(map_type);
    const int ylinesize  = yin->linesize[0] / sizeof(map_type);
    const pixel_type *src = (const pixel_type *)in->data[0];
    pixel_type *dst = (pixel_type *)out->data[0] + slice_start * dlinesize;
    const map_type *xmap = (const map_type *)xin->data[0] + slice_start * xlinesize;
    const map_type *ymap = (const map_type *)yin->data[0] + slice_start * ylinesize;
    const int nb_components = td->nb_components;
    const int in_height = in->height;
    const int in_width = in->width;
    const int step = td->step / sizeof(pixel_type);

    for (int y = slice_start; y < slice_end; y++) {
        for (int x = 0; x < out->width; x++) {
#if MAP_TYPE == 32
            const int xm = floorf(xmap[x]);
            const int ym = floorf(ymap[x]);

            if (ym >= 0 && ym < in_height && xm >= 0 && xm < in_width) {
                const int xn = FFMIN(xm + 1, in_width -1);
                const int yn = FFMIN(ym + 1, in_height-1);
                const float du = xmap[x] - xm;
                const float dv = ymap[x] - ym;

                for (int c = 0; c < nb_components; c++) {
                    float p0 = src[ym * slinesize + xm * step + c];
                    float p1 = src[ym * slinesize + xn * step + c];
                    float p2 = src[yn * slinesize + xm * step + c];
                    float p3 = src[yn * slinesize + xn * step + c];
                    float sum = 0.f;

                    sum += (1.f - du) * (1.f - dv) * p0;
                    sum += (      du) * (1.f - dv) * p1;
                    sum += (1.f - du) * (      dv) * p2;
                    sum += (      du) * (      dv) * p3;

                    dst[x * step + c] = lrintf(sum);
                }
            } else {
                for (int c = 0; c < nb_components; c++) {
                    dst[x * step + c] = s->fill_color[c];
                }
            }
#else
            if (ymap[x] < in_height && xmap[x] < in_width) {
                for (int c = 0; c < nb_components; c++) {
                    dst[x * step + c] = src[ymap[x] * slinesize + xmap[x] * step + c];
                }
            } else {
                for (int c = 0; c < nb_components; c++) {
                    dst[x * step + c] = s->fill_color[c];
                }
            }
#endif
        }

        dst  += dlinesize;
        xmap += xlinesize;
        ymap += ylinesize;
    }

    return 0;
}
