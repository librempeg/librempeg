# License

Most files in Librempeg are under the GNU General Public License version 3
or later (GPL v3+). Read the file `COPYING.GPLv3` for details. Some other
files have MIT/X11/BSD-style licenses. In combination the GPL v3+ applies to
Librempeg.

Some optional parts of Librempeg are licensed under the GNU General Public License
version 2 or later (GPL v2+). See the file `COPYING.GPLv2` for details. None of
these parts are used by default, you have to explicitly pass `--enable-gpl` to
configure to activate them. In this case, Librempeg's license changes to GPL v2+.

Specifically, the GPL parts of Librempeg are:

- optional x86 optimization in the files
    - `libavcodec/x86/flac_dsp_gpl.asm`
    - `libavcodec/x86/idct_mmx.c`
    - `libavfilter/x86/vf_removegrain.asm`
- the following building and testing tools
    - `compat/solaris/make_sunver.pl`
    - `doc/t2h.pm`
    - `doc/texi2pod.pl`
    - `libswresample/tests/swresample.c`
    - `tests/checkasm/*`
    - `tests/tiny_ssim.c`
- the following filters in libavfilter:
    - `signature_lookup.c`
    - `vf_blackframe.c`
    - `vf_boxblur.c`
    - `vf_colormatrix.c`
    - `vf_cover_rect.c`
    - `vf_cropdetect.c`
    - `vf_delogo.c`
    - `vf_eq.c`
    - `vf_find_rect.c`
    - `vf_fspp.c`
    - `vf_histeq.c`
    - `vf_hqdn3d.c`
    - `vf_kerndeint.c`
    - `vf_lensfun.c` (GPL version 3 or later)
    - `vf_mcdeint.c`
    - `vf_mpdecimate.c`
    - `vf_nnedi.c`
    - `vf_owdenoise.c`
    - `vf_perspective.c`
    - `vf_phase.c`
    - `vf_pp.c`
    - `vf_pp7.c`
    - `vf_pullup.c`
    - `vf_repeatfields.c`
    - `vf_sab.c`
    - `vf_signature.c`
    - `vf_smartblur.c`
    - `vf_spp.c`
    - `vf_stereo3d.c`
    - `vf_super2xsai.c`
    - `vf_tinterlace.c`
    - `vf_uspp.c`
    - `vf_vaguedenoiser.c`
    - `vsrc_mptestsrc.c`

Some optional parts of Librempeg are also licensed under the GNU Affero General
Public License version 3 or later (AGPL v3+). See the file `COPYING.AGPLv3` for
 details. None of these parts are used by default, you have to explicitly pass
`--enable-agpl` to configure to activate them. In this case, Librempeg's license
changes to AGPL v3+.

Specifically, the AGPL parts of Librempeg are:

- avfilter frame threading


There are a handful of files under other licensing terms, namely:

* The files `libavcodec/jfdctfst.c`, `libavcodec/jfdctint_template.c` and
  `libavcodec/jrevdct.c` are taken from libjpeg, see the top of the files for
  licensing details. Specifically note that you must credit the IJG in the
  documentation accompanying your program if you only distribute executables.
  You must also indicate any changes including additions and deletions to
  those three files in the documentation.
* `tests/reference.pnm` is under the expat license.


## External libraries

Librempeg can be combined with a number of external libraries, which sometimes
affect the licensing of binaries resulting from the combination.

### Compatible libraries

The following libraries are under GPL version 2:
- avisynth
- frei0r
- libcdio
- libdavs2
- librubberband
- libvidstab
- libx264
- libx265
- libxavs
- libxavs2
- libxvid

When combining them with Librempeg, Librempeg needs to be licensed as GPL as well by
passing `--enable-gpl` to configure.

The following libraries are under LGPL version 3:
- gmp
- libaribb24
- liblensfun

The VMAF, mbedTLS, RK MPI, OpenCORE and VisualOn libraries are under the Apache License
2.0. That license is incompatible with the LGPL v2.1 and the GPL v2, but not with
version 3 of those licenses. So to combine these libraries with Librempeg, the
license version needs to be upgraded by passing `--enable-version3` to configure.

The smbclient library is under the GPL v3, to combine it with Librempeg,
the options `--enable-gpl` and `--enable-version3` have to be passed to
configure to upgrade Librempeg to the GPL v3.

### Incompatible libraries

There are certain libraries you can combine with Librempeg whose licenses are not
compatible with the GPL. If you wish to enable these
libraries, even in circumstances that their license may be incompatible, pass
`--enable-nonfree` to configure. This will cause the resulting binary to be
unredistributable.

The Fraunhofer FDK AAC and OpenSSL libraries are under licenses which are
incompatible with the GPLv2 and v3.
