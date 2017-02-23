include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

# LGE_CHANGE_S: [2014-09-04] boyoung1.kim@lge.com/
# Comment: This sensor driver do not used
#obj-y   +=  gyroscope.o gyro_factory.o
# LGE_CHANGE_E: [2014-09-04] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_MPU3050C),y)
obj-y	+=  mpu3050c/
endif

ifeq ($(CONFIG_MTK_MPU3000),y)
obj-y   +=  mpu3000/
endif

ifeq ($(CONFIG_MTK_MPU3000_NEW),y)
obj-y   +=  mpu3000-new/
endif
ifeq ($(CONFIG_MTK_MPU6050GY_NEW),y)
obj-y   +=  mpu6050gy-new/
endif

ifeq ($(CONFIG_MTK_BMG160),y)
obj-y   +=  bmg160/
endif

ifeq ($(CONFIG_MTK_MPU6515G),y)
obj-y   +=  mpu6515/
endif

ifeq ($(CONFIG_MTK_MPU3050C_NEW),y)
obj-y   +=  mpu3050c-new/
endif
