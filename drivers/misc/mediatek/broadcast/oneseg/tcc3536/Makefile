#
# Makefile for the ISDB-Tmm TCC3530 drivers.
#

# Each configuration option enables a list of files.
# LGE
include $(srctree)/drivers/misc/mediatek/Makefile.custom

CFLAGS_KERNEL+=-D_MODEL_TCC3536_
CFLAGS_KERNEL+=-D_TCSPI_ONLY_

obj-y += broadcast_tcc3530_drv_if.o
obj-y += broadcast_tcc353x.o
obj-y += tcc353x_hal.o
obj-y += tcc353x_irq.o
obj-y += Tcc353xDriver/
obj-y += tcc353x_ringbuffer.o

#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcast/oneseg
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/common
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/api/inc
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/PAL
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/sample/monitoring
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/sample/interruptProcess
#EXTRA_CFLAGS += -Idrivers/misc/mediatek/broadcastoneseg/tcc3536/Tcc353xDriver/sample/main
