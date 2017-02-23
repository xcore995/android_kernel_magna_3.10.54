#
# Makefile for the FC8150 drivers.
#

# Each configuration option enables a list of files.
# LGE

include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-y += broadcast_fc8300_drv_if.o
obj-y += broadcast_fc8300.o
obj-y += fc8300Driver/
EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/broadcast/oneseg
EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/broadcast/oneseg/fc8300/fc8300Driver/inc