#include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) := leds.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include

ifeq ($(CONFIG_WTK_LEDS_AW2015),y)
obj-y	+=  aw2015/
endif

ifeq ($(CONFIG_WTK_LEDS_AW2013),y)
obj-y	+=  aw2013/
endif
