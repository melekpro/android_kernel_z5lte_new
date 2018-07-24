# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

ifeq ($(CONFIG_WTK_MHALL),y)
obj-y   +=  sense_level/
endif

