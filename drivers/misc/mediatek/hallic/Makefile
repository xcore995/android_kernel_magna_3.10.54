include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y	+=  hallic_dummy.o

ifeq ($(CONFIG_BU52061NVX),y)
obj-y	+=  bu52061nvx/
endif
