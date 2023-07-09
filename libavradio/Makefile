NAME    = avradio
DESC    = FFmpeg softwarde defined radio handling library

HEADERS = avradio.h                                                     \
          version.h                                                     \
          version_major.h                                               \

OBJS    = allradios.o                                                   \
          avradio.o                                                     \
          version.o                                                     \


# input/output radios
OBJS-$(CONFIG_SDR_INRADIO)            		+= sdrinradio.o vissualize.o rds.o
OBJS-$(CONFIG_SDRFILE_INRADIO)                  += sdrdemux.o vissualize.o rds.o
