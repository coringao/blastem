LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := main

SDL_PATH := android/jni/SDL

LOCAL_C_INCLUDES := $(LOCAL_PATH)/$(SDL_PATH)/include

LOCAL_CFLAGS += -std=gnu99 -DUSE_GLES

# Add your application source files here...
LOCAL_SRC_FILES := $(SDL_PATH)/src/main/android/SDL_android_main.c \
	68kinst.c psg.c z80.c backend.c io.c render_sdl.c \
	tern.c m68k_core.c musashi/m68kops.c musashi/m68kcpu.c \
	util.c wave.c blastem.c gen.c mem.c vdp.c ym2612.c config.c gen_x86.c \
	terminal.c z80inst.c menu.c arena.c zlib/adler32.c zlib/compress.c \
	zlib/crc32.c zlib/deflate.c zlib/gzclose.c zlib/gzlib.c zlib/gzread.c \
	zlib/gzwrite.c zlib/infback.c zlib/inffast.c zlib/inflate.c \
	zlib/inftrees.c zlib/trees.c zlib/uncompr.c zlib/zutil.c \
	nuklear_ui/font_android.c nuklear_ui/blastem_nuklear.c nuklear_ui/sfnt.c \
	ppm.c controller_info.c png.c system.c genesis.c sms.c serialize.c \
	saves.c hash.c xband.c zip.c bindings.c jcart.c paths.c megawifi.c \
	nor.c i2c.c sega_mapper.c realtec.c multi_game.c net.c romdb.c 

LOCAL_SHARED_LIBRARIES := SDL2

LOCAL_LDLIBS := -lGLESv1_CM -lGLESv2 -llog

include $(BUILD_SHARED_LIBRARY)
