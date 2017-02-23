include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

# LGE_CHANGE_S: [2014-09-04] boyoung1.kim@lge.com
# Comment: This sensor dirver do not used
#obj-y   +=  accel.o accel_factory.o
# LGE_CHANGE_E: [2014-09-04] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_K2DH),y)
obj-y	+=  k2dh/
endif
ifeq ($(CONFIG_MTK_BMA050),y)
obj-y   +=  bma050/
endif
ifeq ($(CONFIG_MTK_BMA050_NEW),y)
obj-y   +=  bma050-new/
endif
ifeq ($(CONFIG_MTK_BMA222E),y)
obj-y   +=  bma222E/
endif
ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y   +=  bma222E-new/
endif
ifeq ($(CONFIG_MTK_MPU6050G),y)
obj-y   +=  mpu6050g/
endif
ifeq ($(CONFIG_MTK_MPU6050G_NEW),y)
obj-y   +=  mpu6050g-new/
endif
ifeq ($(CONFIG_MTK_BMA250),y)
obj-y   +=  bma250/
endif
ifeq ($(CONFIG_BOSCH_BMA255),y)
obj-y   +=  bma255/
endif
ifeq ($(CONFIG_MTK_KXTIK1004),y)
obj-y   +=  kxtik1004/
endif

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y   +=  mpu6515/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  KXTJ2_1009-new/
endif

# LGE_CHANGE_S: [2014-09-03] boyoung1.kim@lge.com
# Comment: Add Accelerometer Sensor driver
ifeq ($(CONFIG_BOSCH_BMA150),y)
obj-y   +=  bma150/
endif
# LGE_CHANGE_E: [2014-09-03] boyoung1.kim@lge.com

