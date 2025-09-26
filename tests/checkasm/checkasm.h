/*
 * Assembly testing and benchmarking tool
 * Copyright (c) 2015 Henrik Gramner
 * Copyright (c) 2008 Loren Merritt
 *
 * This file is part of FFmpeg.
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef TESTS_CHECKASM_CHECKASM_H
#define TESTS_CHECKASM_CHECKASM_H

#include <stdint.h>
#include "config.h"

#if CONFIG_LINUX_PERF
#include <unistd.h> // read(3)
#include <sys/ioctl.h>
#include <asm/unistd.h>
#include <linux/perf_event.h>
#elif CONFIG_MACOS_KPERF
#include "libavutil/macos_kperf.h"
#endif

#include "libavutil/avstring.h"
#include "libavutil/cpu.h"
#include "libavutil/emms.h"
#include "libavutil/internal.h"
#include "libavutil/lfg.h"
#include "libavutil/timer.h"

#ifdef _WIN32
#include <windows.h>
#if ARCH_X86_32
#include <setjmp.h>
typedef jmp_buf checkasm_context;
#define checkasm_save_context() checkasm_handle_signal(setjmp(checkasm_context_buf))
#define checkasm_load_context(s) longjmp(checkasm_context_buf, s)
#elif WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
/* setjmp/longjmp on Windows on architectures using SEH (all except x86_32)
 * will try to use SEH to unwind the stack, which doesn't work for assembly
 * functions without unwind information. */
typedef struct { CONTEXT c; int status; } checkasm_context;
#define checkasm_save_context() \
    (checkasm_context_buf.status = 0, \
     RtlCaptureContext(&checkasm_context_buf.c), \
     checkasm_handle_signal(checkasm_context_buf.status))
#define checkasm_load_context(s) \
    (checkasm_context_buf.status = s, \
     RtlRestoreContext(&checkasm_context_buf.c, NULL))
#else
#define checkasm_context void*
#define checkasm_save_context() 0
#define checkasm_load_context() do {} while (0)
#endif
#elif defined(_WASI_EMULATED_SIGNAL)
#define checkasm_context void*
#define checkasm_save_context() 0
#define checkasm_load_context() do {} while (0)
#else
#include <setjmp.h>
typedef sigjmp_buf checkasm_context;
#define checkasm_save_context() checkasm_handle_signal(sigsetjmp(checkasm_context_buf, 1))
#define checkasm_load_context(s) siglongjmp(checkasm_context_buf, s)
#endif

void checkasm_check_aacencdsp(void);
void checkasm_check_aacpsdsp(void);
void checkasm_check_ac3dsp(void);
void checkasm_check_aes(void);
void checkasm_check_afir(void);
void checkasm_check_alacdsp(void);
void checkasm_check_apv_dsp(void);
void checkasm_check_audiodsp(void);
void checkasm_check_av_tx(void);
void checkasm_check_blackdetect(void);
void checkasm_check_blend(void);
void checkasm_check_blockdsp(void);
void checkasm_check_bswapdsp(void);
void checkasm_check_colordetect(void);
void checkasm_check_colorspace(void);
void checkasm_check_diracdsp(void);
void checkasm_check_exrdsp(void);
void checkasm_check_fdctdsp(void);
void checkasm_check_fixed_dsp(void);
void checkasm_check_flacdsp(void);
void checkasm_check_float_dsp(void);
void checkasm_check_fmtconvert(void);
void checkasm_check_g722dsp(void);
void checkasm_check_h263dsp(void);
void checkasm_check_h264chroma(void);
void checkasm_check_h264dsp(void);
void checkasm_check_h264pred(void);
void checkasm_check_h264qpel(void);
void checkasm_check_hevc_add_res(void);
void checkasm_check_hevc_deblock(void);
void checkasm_check_hevc_idct(void);
void checkasm_check_hevc_pel(void);
void checkasm_check_hevc_sao(void);
void checkasm_check_hpeldsp(void);
void checkasm_check_huffyuvdsp(void);
void checkasm_check_idctdsp(void);
void checkasm_check_idet(void);
void checkasm_check_jpeg2000dsp(void);
void checkasm_check_llauddsp(void);
void checkasm_check_lls(void);
void checkasm_check_llviddsp(void);
void checkasm_check_llviddspenc(void);
void checkasm_check_lpc(void);
void checkasm_check_motion(void);
void checkasm_check_mpegvideoencdsp(void);
void checkasm_check_nlmeans(void);
void checkasm_check_opusdsp(void);
void checkasm_check_pixblockdsp(void);
void checkasm_check_sbrdsp(void);
void checkasm_check_rv34dsp(void);
void checkasm_check_rv40dsp(void);
void checkasm_check_scene_sad(void);
void checkasm_check_svq1enc(void);
void checkasm_check_synth_filter(void);
void checkasm_check_sw_gbrp(void);
void checkasm_check_sw_range_convert(void);
void checkasm_check_sw_rgb(void);
void checkasm_check_sw_scale(void);
void checkasm_check_sw_yuv2rgb(void);
void checkasm_check_sw_yuv2yuv(void);
void checkasm_check_sw_ops(void);
void checkasm_check_takdsp(void);
void checkasm_check_utvideodsp(void);
void checkasm_check_v210dec(void);
void checkasm_check_v210enc(void);
void checkasm_check_vc1dsp(void);
void checkasm_check_vf_bwdif(void);
void checkasm_check_vf_eq(void);
void checkasm_check_vf_gblur(void);
void checkasm_check_vf_hflip(void);
void checkasm_check_vf_threshold(void);
void checkasm_check_vf_sobel(void);
void checkasm_check_vp8dsp(void);
void checkasm_check_vp9dsp(void);
void checkasm_check_videodsp(void);
void checkasm_check_vorbisdsp(void);
void checkasm_check_vvc_alf(void);
void checkasm_check_vvc_mc(void);
void checkasm_check_vvc_sao(void);

struct CheckasmPerf;

void *checkasm_check_func(void *func, const char *name, ...) av_printf_format(2, 3);
int checkasm_bench_func(void);
int checkasm_fail_func(const char *msg, ...) av_printf_format(1, 2);
struct CheckasmPerf *checkasm_get_perf_context(void);
void checkasm_report(const char *name, ...) av_printf_format(1, 2);
void checkasm_set_signal_handler_state(int enabled);
int checkasm_handle_signal(int s);
extern checkasm_context checkasm_context_buf;

/* float compare utilities */
int float_near_ulp(float a, float b, unsigned max_ulp);
int float_near_abs_eps(float a, float b, float eps);
int float_near_abs_eps_ulp(float a, float b, float eps, unsigned max_ulp);
int float_near_ulp_array(const float *a, const float *b, unsigned max_ulp,
                         unsigned len);
int float_near_abs_eps_array(const float *a, const float *b, float eps,
                             unsigned len);
int float_near_abs_eps_array_ulp(const float *a, const float *b, float eps,
                                 unsigned max_ulp, unsigned len);
int double_near_abs_eps(double a, double b, double eps);
int double_near_abs_eps_array(const double *a, const double *b, double eps,
                              unsigned len);

extern AVLFG checkasm_lfg;
#define rnd() av_lfg_get(&checkasm_lfg)

static av_unused void *func_ref, *func_new;

extern uint64_t bench_runs;

/* Decide whether or not the specified function needs to be tested */
#define check_func(func, ...) (checkasm_save_context(), func_ref = checkasm_check_func((func_new = func), __VA_ARGS__))

/* Declare the function prototype. The first argument is the return value, the remaining
 * arguments are the function parameters. Naming parameters is optional. */
#define declare_func(ret, ...) declare_new(ret, __VA_ARGS__) typedef ret func_type(__VA_ARGS__)
#define declare_func_float(ret, ...) declare_new_float(ret, __VA_ARGS__) typedef ret func_type(__VA_ARGS__)
#define declare_func_emms(cpu_flags, ret, ...) declare_new_emms(cpu_flags, ret, __VA_ARGS__) typedef ret func_type(__VA_ARGS__)

/* Indicate that the current test has failed */
#define fail() checkasm_fail_func("%s:%d", av_basename(__FILE__), __LINE__)

/* Print the test outcome */
#define report checkasm_report

/* Call the reference function */
#define call_ref(...)\
    (checkasm_set_signal_handler_state(1),\
     ((func_type *)func_ref)(__VA_ARGS__));\
    checkasm_set_signal_handler_state(0)

#if ARCH_X86 && HAVE_X86ASM
/* Verifies that clobbered callee-saved registers are properly saved and restored
 * and that either no MMX registers are touched or emms is issued */
void checkasm_checked_call(void *func, ...);
/* Verifies that clobbered callee-saved registers are properly saved and restored
 * and issues emms for asm functions which are not required to do so */
void checkasm_checked_call_emms(void *func, ...);
/* Verifies that clobbered callee-saved registers are properly saved and restored
 * but doesn't issue emms. Meant for dsp functions returning float or double */
void checkasm_checked_call_float(void *func, ...);

#if ARCH_X86_64
/* Evil hack: detect incorrect assumptions that 32-bit ints are zero-extended to 64-bit.
 * This is done by clobbering the stack with junk around the stack pointer and calling the
 * assembly function through checked_call() with added dummy arguments which forces all
 * real arguments to be passed on the stack and not in registers. For 32-bit arguments the
 * upper half of the 64-bit register locations on the stack will now contain junk which will
 * cause misbehaving functions to either produce incorrect output or segfault. Note that
 * even though this works extremely well in practice, it's technically not guaranteed
 * and false negatives is theoretically possible, but there can never be any false positives.
 */
void checkasm_stack_clobber(uint64_t clobber, ...);
#define declare_new(ret, ...) ret (*checked_call)(void *, int, int, int, int, int, __VA_ARGS__)\
                              = (void *)checkasm_checked_call;
#define declare_new_float(ret, ...) ret (*checked_call)(void *, int, int, int, int, int, __VA_ARGS__)\
                                    = (void *)checkasm_checked_call_float;
#define declare_new_emms(cpu_flags, ret, ...) \
    ret (*checked_call)(void *, int, int, int, int, int, __VA_ARGS__) = \
        ((cpu_flags) & av_get_cpu_flags()) ? (void *)checkasm_checked_call_emms : \
                                             (void *)checkasm_checked_call;
#define CLOB (UINT64_C(0xdeadbeefdeadbeef))
#define call_new(...) (checkasm_set_signal_handler_state(1),\
                       checkasm_stack_clobber(CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,\
                                              CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB),\
                       checked_call(func_new, 0, 0, 0, 0, 0, __VA_ARGS__));\
                      checkasm_set_signal_handler_state(0)
#elif ARCH_X86_32
#define declare_new(ret, ...) ret (*checked_call)(void *, __VA_ARGS__) = (void *)checkasm_checked_call;
#define declare_new_float(ret, ...) ret (*checked_call)(void *, __VA_ARGS__) = (void *)checkasm_checked_call_float;
#define declare_new_emms(cpu_flags, ret, ...) ret (*checked_call)(void *, __VA_ARGS__) = \
        ((cpu_flags) & av_get_cpu_flags()) ? (void *)checkasm_checked_call_emms :        \
                                             (void *)checkasm_checked_call;
#define call_new(...)\
    (checkasm_set_signal_handler_state(1),\
     checked_call(func_new, __VA_ARGS__));\
    checkasm_set_signal_handler_state(0)
#endif
#elif ARCH_ARM && HAVE_ARMV5TE_EXTERNAL
/* Use a dummy argument, to offset the real parameters by 2, not only 1.
 * This makes sure that potential 8-byte-alignment of parameters is kept the same
 * even when the extra parameters have been removed. */
void checkasm_checked_call_vfp(void *func, int dummy, ...);
void checkasm_checked_call_novfp(void *func, int dummy, ...);
extern void (*checkasm_checked_call)(void *func, int dummy, ...);
#define declare_new(ret, ...) ret (*checked_call)(void *, int dummy, __VA_ARGS__, \
                                                  int, int, int, int, int, int, int, int, \
                                                  int, int, int, int, int, int, int) = (void *)checkasm_checked_call;
#define call_new(...) \
    (checkasm_set_signal_handler_state(1),\
     checked_call(func_new, 0, __VA_ARGS__, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0));\
    checkasm_set_signal_handler_state(0)
#elif ARCH_AARCH64 && !defined(__APPLE__)
void checkasm_stack_clobber(uint64_t clobber, ...);
void checkasm_checked_call(void *func, ...);
#define declare_new(ret, ...) ret (*checked_call)(void *, int, int, int, int, int, int, int, __VA_ARGS__,\
                                                  int, int, int, int, int, int, int, int,\
                                                  int, int, int, int, int, int, int)\
                              = (void *)checkasm_checked_call;
#define CLOB (UINT64_C(0xdeadbeefdeadbeef))
#define call_new(...) (checkasm_set_signal_handler_state(1),\
                       checkasm_stack_clobber(CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,\
                                              CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB,CLOB),\
                      checked_call(func_new, 0, 0, 0, 0, 0, 0, 0, __VA_ARGS__,\
                                   7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0));\
                     checkasm_set_signal_handler_state(0)
#elif ARCH_RISCV
void checkasm_set_function(void *);
void *checkasm_get_wrapper(void);

#if HAVE_RV && (__riscv_xlen == 64) && defined (__riscv_d)
#define declare_new(ret, ...) \
    ret (*checked_call)(__VA_ARGS__) = checkasm_get_wrapper();
#define call_new(...) \
    (checkasm_set_signal_handler_state(1),\
     checkasm_set_function(func_new), checked_call(__VA_ARGS__));\
    checkasm_set_signal_handler_state(0)
#else
#define declare_new(ret, ...)
#define call_new(...)\
    (checkasm_set_signal_handler_state(1),\
     ((func_type *)func_new)(__VA_ARGS__));\
    checkasm_set_signal_handler_state(0)
#endif
#else
#define declare_new(ret, ...)
#define declare_new_float(ret, ...)
#define declare_new_emms(cpu_flags, ret, ...)
/* Call the function */
#define call_new(...)\
    (checkasm_set_signal_handler_state(1),\
     ((func_type *)func_new)(__VA_ARGS__));\
    checkasm_set_signal_handler_state(0)
#endif

#ifndef declare_new_emms
#define declare_new_emms(cpu_flags, ret, ...) declare_new(ret, __VA_ARGS__)
#endif
#ifndef declare_new_float
#define declare_new_float(ret, ...) declare_new(ret, __VA_ARGS__)
#endif

typedef struct CheckasmPerf {
    int sysfd;
    uint64_t cycles;
    int iterations;
} CheckasmPerf;

#if defined(AV_READ_TIME) || CONFIG_LINUX_PERF || CONFIG_MACOS_KPERF

#if CONFIG_LINUX_PERF
#define PERF_START(t) do {                              \
    ioctl(sysfd, PERF_EVENT_IOC_RESET, 0);              \
    ioctl(sysfd, PERF_EVENT_IOC_ENABLE, 0);             \
} while (0)
#define PERF_STOP(t) do {                               \
    int ret;                                            \
    ioctl(sysfd, PERF_EVENT_IOC_DISABLE, 0);            \
    ret = read(sysfd, &t, sizeof(t));                   \
    (void)ret;                                          \
} while (0)
#elif CONFIG_MACOS_KPERF
#define PERF_START(t) t = ff_kperf_cycles()
#define PERF_STOP(t)  t = ff_kperf_cycles() - t
#else
#define PERF_START(t) t = AV_READ_TIME()
#define PERF_STOP(t)  t = AV_READ_TIME() - t
#endif

#define CALL4(...)\
    do {\
        tfunc(__VA_ARGS__); \
        tfunc(__VA_ARGS__); \
        tfunc(__VA_ARGS__); \
        tfunc(__VA_ARGS__); \
    } while (0)

#define CALL16(...)\
    do {\
        CALL4(__VA_ARGS__); \
        CALL4(__VA_ARGS__); \
        CALL4(__VA_ARGS__); \
        CALL4(__VA_ARGS__); \
    } while (0)

/* Benchmark the function */
#define bench_new(...)\
    do {\
        if (checkasm_bench_func()) {\
            struct CheckasmPerf *perf = checkasm_get_perf_context();\
            av_unused const int sysfd = perf->sysfd;\
            func_type *tfunc = func_new;\
            uint64_t tsum = 0;\
            uint64_t ti, tcount = 0;\
            uint64_t t = 0; \
            const uint64_t truns = FFMAX(bench_runs >> 3, 1);\
            checkasm_set_signal_handler_state(1);\
            for (ti = 0; ti < truns; ti++) {\
                PERF_START(t);\
                CALL16(__VA_ARGS__);\
                CALL16(__VA_ARGS__);\
                PERF_STOP(t);\
                if (t*tcount <= tsum*4 && ti > 0) {\
                    tsum += t;\
                    tcount++;\
                }\
            }\
            emms_c();\
            perf->cycles += tsum;\
            perf->iterations += tcount;\
            checkasm_set_signal_handler_state(0);\
        }\
    } while (0)
#else
#define bench_new(...) while(0)
#define PERF_START(t)  while(0)
#define PERF_STOP(t)   while(0)
#endif

#define BUF_RECT(type, name, w, h) \
    LOCAL_ALIGNED_32(type, name##_buf, [((h)+32)*(FFALIGN(w,64)+64) + 64]); \
    av_unused ptrdiff_t name##_stride = sizeof(type)*(FFALIGN(w,64)+64); \
    av_unused int name##_buf_h = (h)+32; \
    type *name = name##_buf + (FFALIGN(w,64)+64)*16 + 64

#define PIXEL_RECT(name, w, h) \
    LOCAL_ALIGNED_32(uint8_t, name##_buf, [sizeof(uint16_t) * (((h)+32)*(FFALIGN(w,64)+64) + 64)],); \
    av_unused ptrdiff_t name##_stride = sizeof(uint16_t) * (FFALIGN(w,64)+64); \
    av_unused int name##_buf_h = (h)+32; \
    uint8_t *name = name##_buf + (FFALIGN(w,64)+64)*16 + 64

#define CLEAR_BUF_RECT(name) \
    memset(name##_buf, 0x99, name##_stride * name##_buf_h + 64)
#define CLEAR_PIXEL_RECT(name) \
    CLEAR_BUF_RECT(name)

#define DECL_CHECKASM_CHECK_FUNC(type) \
int checkasm_check_##type(const char *file, int line, \
                          const type *buf1, ptrdiff_t stride1, \
                          const type *buf2, ptrdiff_t stride2, \
                          int w, int h, const char *name, \
                          int align_w, int align_h, \
                          int padding)

DECL_CHECKASM_CHECK_FUNC(uint8_t);
DECL_CHECKASM_CHECK_FUNC(uint16_t);
DECL_CHECKASM_CHECK_FUNC(uint32_t);
DECL_CHECKASM_CHECK_FUNC(int16_t);
DECL_CHECKASM_CHECK_FUNC(int32_t);

int checkasm_check_float_ulp(const char *file, int line,
                             const float *buf1, ptrdiff_t stride1,
                             const float *buf2, ptrdiff_t stride2,
                             int w, int h, const char *name,
                             unsigned max_ulp, int align_w, int align_h,
                             int padding);

#define PASTE(a,b) a ## b
#define CONCAT(a,b) PASTE(a,b)

#define checkasm_check2(prefix, ...) CONCAT(checkasm_check_, prefix)(__FILE__, __LINE__, __VA_ARGS__)
#define checkasm_check(prefix, ...) checkasm_check2(prefix, __VA_ARGS__, 0, 0, 0)
/* Check a pointer from BUF_RECT, checking whether there have been
 * writes outside of the designated area. */
#define checkasm_check_padded(...) \
    checkasm_check2(__VA_ARGS__, 1, 1, 8)
/* Check a pointer from BUF_RECT, checking whether there have been
 * writes outside of the designated area. Allow writing slightly past the
 * end of the buffer, by aligning w/h to align_w/align_h, and checking
 * for overwrites outside of that. */
#define checkasm_check_padded_align(...) \
    checkasm_check2(__VA_ARGS__, 8)

/* This assumes that there is a local variable named "bit_depth".
 * For tests that don't have that and only operate on a single
 * bitdepth, just call checkasm_check(uint8_t, ...) directly. */
#define checkasm_check_pixel2(buf1, stride1, buf2, stride2, ...) \
    ((bit_depth > 8) ?                                          \
     checkasm_check2(uint16_t, (const uint16_t*)buf1, stride1,   \
                               (const uint16_t*)buf2, stride2,   \
                               __VA_ARGS__) :                    \
     checkasm_check2(uint8_t,  (const uint8_t*) buf1, stride1,   \
                               (const uint8_t*) buf2, stride2,   \
                               __VA_ARGS__))
#define checkasm_check_pixel(...) \
    checkasm_check_pixel2(__VA_ARGS__, 0, 0, 0)
#define checkasm_check_pixel_padded(...) \
    checkasm_check_pixel2(__VA_ARGS__, 1, 1, 8)
#define checkasm_check_pixel_padded_align(...) \
    checkasm_check_pixel2(__VA_ARGS__, 8)

/* This assumes that there is a local variable named "bit_depth"
 * and that the type-specific buffers obey the name ## _BITDEPTH
 * convention.
 * For tests that don't have that and only operate on a single
 * bitdepth, just call checkasm_check(uint8_t, ...) directly. */
#define checkasm_check_dctcoef(buf1, stride1, buf2, stride2, ...) \
    ((bit_depth > 8) ?                                        \
     checkasm_check(int32_t, buf1 ## _32, stride1,            \
                             buf2 ## _32, stride2,            \
                             __VA_ARGS__) :                   \
     checkasm_check(int16_t, buf1 ## _16, stride1,            \
                             buf2 ## _16, stride2,            \
                             __VA_ARGS__))

#endif /* TESTS_CHECKASM_CHECKASM_H */
