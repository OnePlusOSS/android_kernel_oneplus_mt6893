# SPDX-License-Identifier: GPL-2.0

obj-y += core/

obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB) += mtk_nanohub/
obj-$(CONFIG_MTK_SENSORHUB) += sensorhub/
obj-$(CONFIG_MTK_HF_LSM6DSM_SECONDARY) += lsm6dsm_secondary/
obj-$(CONFIG_MTK_HF_TEST_CASE) += test/
#ifdef OPLUS_FEATURE_SENSOR
#/*ZengChao, 2020/08/06, Add for oplus sensor type to send some info to scp*/
obj-y += oplus_sensor_devinfo/
#ifneq ($(OPLUS_FEATURE_SENSOR_ALGORITHM),)
obj-y += oplus_virtual_sensor/
obj-y += oplus_consumer_ir/
#endif
#endif //OPLUS_FEATURE_SENSOR
obj-$(CONFIG_MTK_AP_REAR_SENSOR) += mn29005/
