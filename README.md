Librempeg README
=============

Librempeg is a collection of libraries and tools to process multimedia content
such as audio, video, subtitles and related metadata.

## Libraries

* `libavcodec` provides implementation of a wider range of codecs.
* `libavformat` implements streaming protocols, container formats and basic I/O access.
* `libavutil` includes hashers, decompressors and miscellaneous utility functions.
* `libavfilter` provides means to alter decoded audio and video through a directed graph of connected filters.
* `libavdevice` provides an abstraction to access capture and playback devices.
* `libswresample` implements audio mixing and resampling routines.
* `libswscale` implements color conversion and scaling routines.

## Tools

* `ffmpeg` is a command line toolbox to manipulate, convert and stream multimedia content.
* `ffplay` is a minimalistic multimedia player.
* `ffprobe` is a simple analysis tool to inspect multimedia content.
* Additional small tools such as `aviocat`, `ismindex` and `qt-faststart`.

## Documentation

The offline documentation is available in the **doc/** directory.

### Examples

Coding examples are available in the **doc/examples** directory.

## License

Librempeg codebase is mainly GPLv3-licensed with optional components licensed under
AGPL. Please refer to the LICENSE file for detailed information.

## Contributing

Contributing can be done via Github pull requests.
