# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/focal_touch/firmware_config/$(CONFIG_TS_FIRMWARE)

obj-y    +=  focaltech_ctl.o
obj-y    +=  focaltech_ex_fun.o
obj-y    +=  focaltech_flash.o
obj-y    +=  focaltech_gesture.o
obj-y    +=  focaltech_core.o
obj-y    +=  focaltech_esdcheck.o
obj-y    +=  focaltech_sensor.o
obj-y    +=  focaltech_i2c.o

obj-y +=     focaltech_test/
obj-y +=     focaltech_global/
