include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

#obj-y   +=  alsps.o
#obj-y   +=  alsps_factory.o
#obj-y   +=  aal_control.o

ifeq ($(CONFIG_MTK_AUTO_DETECT_ALSPS),y)
obj-y	+=  alsps_device_check.o
endif

# LGE_CHANGE_S: [2014-10-20] boyoung1.kim@lge.com
# Comment: Add Proximity sensor driver
ifeq ($(CONFIG_MTK_APDS9130),y)
obj-y	+=  apds9130/
endif

ifeq ($(CONFIG_ROHM_RPR0521),y)
obj-y	+=  rpr0521/
endif
# LGE_CHANGE_E: [2014-10-20] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_GP2AP002S00F),y)
obj-y	+=  gp2ap002s00f/
endif

ifeq ($(CONFIG_MTK_CM36283),y)
obj-y	+=  cm36283/
endif

ifeq ($(CONFIG_MTK_APM_16D),y)
obj-y	+=  apm_16d/
endif

ifeq ($(CONFIG_MTK_EPL2182),y)
obj-y	+=  epl2182/
endif

ifeq ($(CONFIG_MTK_EPL2182_NEW),y)
obj-y	+=  EPL2182-new/
endif
ifeq ($(CONFIG_MTK_STK3X1X),y)
obj-y	+=  stk3x1x/
endif
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif

ifeq ($(CONFIG_MTK_APDS9930),y)
obj-y	+=  apds9930/
endif

ifeq ($(CONFIG_MTK_CM3232_NEW),y)
obj-y	+=  cm3232-new/
endif

ifeq ($(CONFIG_MTK_APDS9930_NEW),y)
obj-y	+=  APDS9930-new/
endif
