# SPDX-License-Identifier: GPL-2.0
$(warning "tee config----- CONFIG_TEE: $(CONFIG_TEE)")
$(warning "tee config----- CONFIG_TRUSTONIC_TEE_SUPPORT: $(CONFIG_TRUSTONIC_TEE_SUPPORT)")
$(warning "tee config----- CONFIG_TRUSTONIC_TEE_VERSION: $(CONFIG_TRUSTONIC_TEE_VERSION)")
obj-$(CONFIG_TEE) += tee.o
tee-objs += tee_core.o
tee-objs += tee_shm.o
tee-objs += tee_shm_pool.o
obj-$(CONFIG_OPTEE) += optee/
