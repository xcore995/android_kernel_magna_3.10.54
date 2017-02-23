include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
# LGE_CHANGE_S: [2014-09-04] boyoung1.kim@lge.com
# Comment: This sensor driver do not used
# obj-y   +=  mag.o
# obj-y   +=  mag_factory.o
# LGE_CHANGE_E: [2014-09-04] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_AKM8963_NEW),y)
obj-y	+=  akm8963-new/
endif

# LGE_CHANGE_S: [2014-09-24] boyoung1.kim@lge.com
# Comment: Add MTK Magnetometer Sensor driver
ifeq ($(CONFIG_BOSCH_BMM050),y)
obj-y   +=  bmm050/
endif
# LGE_CHANGE_E: [2014-09-24] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_BMM056_NEW),y)
obj-y	+=  bmm056-new/
endif
ifeq ($(CONFIG_MTK_AKM8963),y)
obj-y	+=  akm8963/
endif

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y   +=  yamaha532/
endif

ifeq ($(CONFIG_MTK_AKM09911),y)
obj-y	+=  akm09911/
endif


ifeq ($(CONFIG_MTK_BMM050),y)
obj-y   +=  bmm050/
endif

ifeq ($(CONFIG_BOSCH_BMM050),y)
obj-y   +=  bmm050/
endif
