# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/focal_touch/firmware_config/$(CONFIG_TS_FIRMWARE)

obj-y += focaltech_test.o
obj-y += focaltech_test_main.o
obj-y += focaltech_test_ini.o
obj-y += focaltech_test_global.o
obj-y += focaltech_test_detail_threshold.o
