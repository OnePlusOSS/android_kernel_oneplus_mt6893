ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB)	+= sensorHub/
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
ifeq ($(CONFIG_CUSTOM_KERNEL_ALS),y)
obj-$(CONFIG_CUSTOM_KERNEL_ALS) += alsps/
else
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
endif
ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELGYRO),y)
obj-y += accelgyro/
else
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
endif
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
obj-$(CONFIG_CUSTOM_KERNEL_STEP_COUNTER) += step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION_SENSOR) += step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SITUATION) += situation/
obj-$(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION) += sensorfusion/
#ifdef OPLUS_FEATURE_SENSOR
obj-y += oplus_sensor_devinfo/
#endif //OPLUS_FEATURE_SENSOR
#ifdef OPLUS_FEATURE_SENSORR_ALGORITHM
#ifneq ($(OPLUS_FEATURE_SENSOR_ALGORITHM),)
obj-y += oplus_virtual_sensor/
#endif
#endif //OPLUS_FEATURE_SENSOR_ALGORITHM
else
obj-y += dummy.o
endif
