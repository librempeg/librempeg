/*
 * Copyright (c) 2015 Paul B Mahol
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

#ifndef AVFILTER_WINDOW_FUNC_H
#define AVFILTER_WINDOW_FUNC_H

#include <math.h>
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "window_func_opt.h"

#ifdef DEPTH

#undef ftype
#undef fun

#if DEPTH == 64
#define ftype double
#else
#define ftype float
#endif

#define fun3(a,b)  a##_##b
#define fun2(a,b)  fun3(a,b)
#define fun(a)     fun2(a, DEPTH)

#else

#undef ftype
#undef fun

#define fun(a)      (a)
#define ftype float

#endif

static inline void fun(generate_window_func)(ftype *lut, int N, int win_func,
                                             ftype *overlap)
{
    switch (win_func) {
    case WFUNC_RECT:
        for (int n = 0; n < N; n++)
            lut[n] = 1.;
        *overlap = 0.;
        break;
    case WFUNC_BARTLETT:
        for (int n = 0; n < N; n++)
            lut[n] = 1.-fabs((n-(N-1)/2.)/((N-1)/2.));
        *overlap = 0.5;
        break;
    case WFUNC_HANNING:
        for (int n = 0; n < N; n++)
            lut[n] = .5*(1-cos(2*M_PI*n/(N-1)));
        *overlap = 0.5;
        break;
    case WFUNC_HAMMING:
        for (int n = 0; n < N; n++)
            lut[n] = .54-.46*cos(2*M_PI*n/(N-1));
        *overlap = 0.75;
        break;
    case WFUNC_BLACKMAN:
        for (int n = 0; n < N; n++)
            lut[n] = .42659-.49656*cos(2*M_PI*n/(N-1))+.076849*cos(4*M_PI*n/(N-1));
        *overlap = 0.661;
        break;
    case WFUNC_WELCH:
        for (int n = 0; n < N; n++)
            lut[n] = 1.-(n-(N-1)/2.)/((N-1)/2.)*(n-(N-1)/2.)/((N-1)/2.);
        *overlap = 0.293;
        break;
    case WFUNC_FLATTOP:
        for (int n = 0; n < N; n++)
            lut[n] = 1.-1.985844164102*cos( 2*M_PI*n/(N-1))+1.791176438506*cos( 4*M_PI*n/(N-1))-
            1.282075284005*cos( 6*M_PI*n/(N-1))+0.667777530266*cos( 8*M_PI*n/(N-1))-
            0.240160796576*cos(10*M_PI*n/(N-1))+0.056656381764*cos(12*M_PI*n/(N-1))-
            0.008134974479*cos(14*M_PI*n/(N-1))+0.000624544650*cos(16*M_PI*n/(N-1))-
            0.000019808998*cos(18*M_PI*n/(N-1))+0.000000132974*cos(20*M_PI*n/(N-1));
        *overlap = 0.841;
        break;
    case WFUNC_BHARRIS:
        for (int n = 0; n < N; n++)
            lut[n] = 0.35875-0.48829*cos(2*M_PI*n/(N-1))+0.14128*cos(4*M_PI*n/(N-1))-0.01168*cos(6*M_PI*n/(N-1));
        *overlap = 0.661;
        break;
    case WFUNC_BNUTTALL:
        for (int n = 0; n < N; n++)
            lut[n] = 0.3635819-0.4891775*cos(2*M_PI*n/(N-1))+0.1365995*cos(4*M_PI*n/(N-1))-0.0106411*cos(6*M_PI*n/(N-1));
        *overlap = 0.75;
        break;
    case WFUNC_BHANN:
        for (int n = 0; n < N; n++)
            lut[n] = 0.62-0.48*fabs(n/(double)(N-1)-.5)-0.38*cos(2*M_PI*n/(N-1));
        *overlap = 0.5;
        break;
    case WFUNC_SINE:
        for (int n = 0; n < N; n++)
            lut[n] = sin(M_PI*n/(N-1));
        *overlap = 0.75;
        break;
    case WFUNC_NUTTALL:
        for (int n = 0; n < N; n++)
            lut[n] = 0.355768-0.487396*cos(2*M_PI*n/(N-1))+0.144232*cos(4*M_PI*n/(N-1))-0.012604*cos(6*M_PI*n/(N-1));
        *overlap = 0.75;
        break;
    case WFUNC_LANCZOS:
        #define SINC(x) (!(x)) ? 1 : sin(M_PI * (x))/(M_PI * (x));
        for (int n = 0; n < N; n++)
            lut[n] = SINC((2.*n)/(N-1)-1);
        *overlap = 0.75;
        break;
    case WFUNC_GAUSS:
        #define SQR(x) ((x)*(x))
        for (int n = 0; n < N; n++)
            lut[n] = exp(-0.5 * SQR((n-(N-1)/2)/(0.4*(N-1)/2.f)));
        *overlap = 0.75;
        break;
    case WFUNC_TUKEY:
        for (int n = 0; n < N; n++) {
            double M = (N-1)/2.;

            if (FFABS(n - M) >= 0.3 * M) {
                lut[n] = 0.5 * (1 + cos((M_PI*(FFABS(n - M) - 0.3 * M))/((1 - 0.3) * M)));
            } else {
                lut[n] = 1;
            }
        }
        *overlap = 0.33;
        break;
    case WFUNC_DOLPH: {
        double b = cosh(acosh(pow(10.0, 5.0)) / (N-1)), sum, t, norm = 0;
        int j;
        for (int c = 1 - 1 / (b*b), n = (N-1) / 2; n >= 0; --n) {
            for (sum = !n, b = t = j = 1; j <= n && sum != t; b *= (n-j) * (1./j), ++j)
                t = sum, sum += (b *= c * (N - n - j) * (1./j));
            sum /= (N - 1 - n), norm = norm ? norm : sum, sum /= norm;
            lut[n] = sum;
            lut[N - 1 - n] = sum;
        }
        *overlap = 0.75;}
        break;
    case WFUNC_CAUCHY:
        for (int n = 0; n < N; n++) {
            double x = 2 * ((n / (double)(N - 1)) - .5);

            if (x <= -.5 || x >= .5) {
                lut[n] = 0;
            } else {
                lut[n] = FFMIN(1, fabs(1/(1+4*16*x*x)));
            }
        }
        *overlap = 0.75;
        break;
    case WFUNC_PARZEN:
        for (int n = 0; n < N; n++) {
            double x = 2 * ((n / (double)(N - 1)) - .5);

            if (x > 0.25 && x <= 0.5) {
                lut[n] = -2 * powf(-1 + 2 * x, 3);
            } else if (x >= -.5 && x < -.25) {
                lut[n] = 2 * powf(1 + 2 * x, 3);
            } else if (x >= -.25 && x < 0) {
                lut[n] = 1 - 24 * x * x - 48 * x * x * x;
            } else if (x >= 0 && x <= .25) {
                lut[n] = 1 - 24 * x * x + 48 * x * x * x;
            } else {
                lut[n] = 0;
            }
        }
        *overlap = 0.75;
        break;
    case WFUNC_POISSON:
        for (int n = 0; n < N; n++) {
            double x = 2 * ((n / (double)(N - 1)) - .5);

            if (x >= 0 && x <= .5) {
                lut[n] = exp(-6*x);
            } else if (x < 0 && x >= -.5) {
                lut[n] = exp(6*x);
            } else {
                lut[n] = 0;
            }
        }
        *overlap = 0.75;
        break;
    case WFUNC_BOHMAN:
        for (int n = 0; n < N; n++) {
            double x = 2 * ((n / (double)(N - 1))) - 1.;

            lut[n] = (1 - fabs(x)) * cos(M_PI*fabs(x)) + 1./M_PI*sin(M_PI*fabs(x));
        }
        *overlap = 0.75;
        break;
    case WFUNC_KAISER:
    {
        double scale = 1.0 / av_bessel_i0(12.);
        for (int n = 0; n < N; n++) {
            double x = 2.0 / (double)(N - 1);
            lut[n] = av_bessel_i0(12. * sqrt(1. - SQR(n * x - 1.))) * scale;
        }
        *overlap = 0.75;
        break;
    }
    default:
        av_assert0(0);
    }
}

#endif /* AVFILTER_WINDOW_FUNC_H */
