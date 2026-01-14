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

#undef FCOS
#undef FSIN
#undef FACOS
#undef FASIN
#undef FSQRT
#undef FCBRT
#undef CLIP
#undef ftype
#undef stype
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define FCBRT cbrtf
#define FSQRT sqrtf
#define FASIN asinf
#define FACOS acosf
#define FSIN sinf
#define FCOS cosf
#define CLIP av_clipf
#define ftype float
#define stype int16_t
#define SAMPLE_FORMAT s16
#elif DEPTH == 31
#define FCBRT cbrt
#define FSQRT sqrt
#define FASIN asin
#define FACOS acos
#define FSIN sin
#define FCOS cos
#define CLIP av_clipd
#define ftype double
#define stype int32_t
#define SAMPLE_FORMAT s32
#elif DEPTH == 32
#define FCBRT cbrtf
#define FSQRT sqrtf
#define FASIN asinf
#define FACOS acosf
#define FSIN sinf
#define FCOS cosf
#define CLIP av_clipf
#define ftype float
#define stype float
#define SAMPLE_FORMAT flt
#else
#define FCBRT cbrt
#define FSQRT sqrt
#define FASIN asin
#define FACOS acos
#define FSIN sin
#define FCOS cos
#define CLIP av_clipd
#define ftype double
#define stype double
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(fade_gain)(int curve, int64_t index, int64_t range, ftype silence, ftype unity)
{
#define SQR(a) ((a)*(a))
#define CUBE(a) ((a)*(a)*(a))
    ftype gain;

    gain = CLIP(F(1.0) * index / range, F(0.0), F(1.0));

    switch (curve) {
    case QSIN:
        gain = FSIN(gain * F(M_PI) / F(2.0));
        break;
    case IQSIN:
        /* 0.6... = 2 / M_PI */
        gain = F(0.6366197723675814) * FASIN(gain);
        break;
    case ESIN:
        gain = F(1.0) - FCOS(F(M_PI) / F(4.0) * (CUBE(F(2.0)*gain - F(1.0)) + F(1.0)));
        break;
    case HSIN:
        gain = (F(1.0) - FCOS(gain * F(M_PI))) / F(2.0);
        break;
    case IHSIN:
        /* 0.3... = 1 / M_PI */
        gain = F(0.3183098861837907) * FACOS(F(1.0) - F(2.0) * gain);
        break;
    case EXP:
        /* -11.5... = 5*ln(0.1) */
        gain = exp(F(-11.512925464970227) * (F(1.0) - gain));
        break;
    case LOG:
        gain = CLIP(F(1.0) + F(0.2) * log10(gain), F(0.0), F(1.0));
        break;
    case PAR:
        gain = F(1.0) - FSQRT(F(1.0) - gain);
        break;
    case IPAR:
        gain = (F(1.0) - (F(1.0) - gain) * (F(1.0) - gain));
        break;
    case QUA:
        gain = SQR(gain);
        break;
    case CUB:
        gain = CUBE(gain);
        break;
    case SQU:
        gain = FSQRT(gain);
        break;
    case CBR:
        gain = FCBRT(gain);
        break;
    case DESE:
        gain = gain <= F(0.5) ? FCBRT(F(2.0) * gain) / 2: F(1.0) - FCBRT(F(2.0) * (F(1.0) - gain)) / 2;
        break;
    case DESI:
        gain = gain <= F(0.5) ? CUBE(F(2.0) * gain) / 2: F(1.0) - CUBE(F(2.0) * (F(1.0) - gain)) / 2;
        break;
    case LOSI: {
                   const ftype a = F(1.0) / (F(1.0) - F(0.787)) - F(1.0);
                   ftype A = F(1.0) / (F(1.0) + exp(0 -((gain-F(0.5)) * a * F(2.0))));
                   ftype B = F(1.0) / (F(1.0) + exp(a));
                   ftype C = F(1.0) / (F(1.0) + exp(0-a));
                   gain = (A - B) / (C - B);
               }
        break;
    case SINC:
        gain = gain >= F(1.0) ? F(1.0) : FSIN(F(M_PI) * (F(1.0) - gain)) / (F(M_PI) * (F(1.0) - gain));
        break;
    case ISINC:
        gain = gain <= F(0.0) ? F(0.0) : F(1.0) - FSIN(F(M_PI) * gain) / (F(M_PI) * gain);
        break;
    case QUAT:
        gain = gain * gain * gain * gain;
        break;
    case QUATR:
        gain = pow(gain, F(0.25));
        break;
    case QSIN2:
        gain = SQR(FSIN(gain * F(M_PI) / F(2.0)));
        break;
    case HSIN2:
        gain = pow((F(1.0) - FCOS(gain * F(M_PI))) / F(2.0), F(2.0));
        break;
    case NONE:
        gain = F(1.0);
        break;
    }

    return silence + (unity - silence) * gain;
}

static void fn(fade_samplesp)(uint8_t **dst, uint8_t *const *src,
                              int nb_samples, int channels, int dir,
                              int64_t start, int64_t range,int curve,
                              double dsilence, double dunity)
{
    const ftype silence = dsilence;
    const ftype unity = dunity;

    for (int i = 0; i < nb_samples; i++) {
        const ftype gain = fn(fade_gain)(curve,start+i*dir,range,silence,unity);
        for (int c = 0; c < channels; c++) {
            stype *d = (stype *)dst[c];
            const stype *s = (stype *)src[c];

            d[i] = s[i] * gain;
        }
    }
}

static void fn(fade_samples)(uint8_t **dst, uint8_t *const *src,
                             int nb_samples, int channels, int dir,
                             int64_t start, int64_t range, int curve,
                             double dsilence, double dunity)
{
    const ftype silence = dsilence;
    const ftype unity = dunity;
    const stype *s = (stype *)src[0];
    stype *d = (stype *)dst[0];

    for (int i = 0, k = 0; i < nb_samples; i++) {
        const ftype gain = fn(fade_gain)(curve,start+i*dir,range,silence,unity);
        for (int c = 0; c < channels; c++, k++)
            d[k] = s[k] * gain;
    }
}

static void fn(scale_samplesp)(uint8_t **dst, uint8_t *const *src,
                               int nb_samples, int channels,
                               double dgain)
{
    const ftype gain = dgain;

    for (int i = 0; i < nb_samples; i++) {
        for (int c = 0; c < channels; c++) {
            stype *d = (stype *)dst[c];
            const stype *s = (stype *)src[c];

            d[i] = s[i] * gain;
        }
    }
}

static void fn(scale_samples)(uint8_t **dst, uint8_t *const *src,
                              int nb_samples, int channels, double dgain)
{
    const ftype gain = dgain;
    stype *d = (stype *)dst[0];
    const stype *s = (stype *)src[0];

    for (int i = 0, k = 0; i < nb_samples; i++) {
        for (int c = 0; c < channels; c++, k++)
            d[k] = s[k] * gain;
    }
}

static void fn(crossfade_samplesp)(uint8_t **dst, uint8_t *const *cf0,
                                   uint8_t *const *cf1,
                                   int nb_samples, int64_t total_samples,
                                   int64_t offset, int channels,
                                   int curve0, int curve1)
{
    for (int i = 0; i < nb_samples; i++) {
        const ftype gain0 = fn(fade_gain)(curve0, total_samples-1-(offset+i), total_samples,F(0.0),F(1.0));
        const ftype gain1 = fn(fade_gain)(curve1, offset+i, total_samples, F(0.0), F(1.0));
        for (int c = 0; c < channels; c++) {
            const stype *s0 = (stype *)cf0[c];
            const stype *s1 = (stype *)cf1[c];
            stype *d = (stype *)dst[c];

            d[i] = s0[i] * gain0 + s1[i] * gain1;
        }
    }
}

static void fn(crossfade_samples)(uint8_t **dst, uint8_t *const *cf0,
                                  uint8_t *const *cf1,
                                  int nb_samples, int64_t total_samples,
                                  int64_t offset, int channels,
                                  int curve0, int curve1)
{
    stype *d = (stype *)dst[0];
    const stype *s0 = (stype *)cf0[0];
    const stype *s1 = (stype *)cf1[0];

    for (int i = 0, k = 0; i < nb_samples; i++) {
        const ftype gain0 = fn(fade_gain)(curve0, total_samples-1-(offset+i), total_samples,F(0.0),F(1.0));
        const ftype gain1 = fn(fade_gain)(curve1, offset+i, total_samples, F(0.0), F(1.0));
        for (int c = 0; c < channels; c++, k++)
            d[k] = s0[k] * gain0 + s1[k] * gain1;
    }
}
