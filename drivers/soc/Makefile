# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-y				+= mediatek/
obj-$(CONFIG_ARCH_MXC)		+= imx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-y						+= mediatek/
obj-$(CONFIG_ARCH_MESON)	+= amlogic/
obj-$(CONFIG_ARCH_QCOM)		+= qcom/
obj-y				+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)	+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_ARCH_SUNXI)	+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-$(CONFIG_SOC_TI)		+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-$(CONFIG_ARCH_ZX)		+= zte/

#ifdef OPLUS_FEATURE_LOWMEM_DBG
obj-$(CONFIG_OPLUS_FEATURE_LOWMEM_DBG)	+=  oplus/lowmem_dbg/
#endif /* OPLUS_FEATURE_LOWMEM_DBG */

#ifdef OPLUS_FEATURE_IOMONITOR
obj-$(CONFIG_IOMONITOR)    +=  oplus/iomonitor/
#endif /*OPLUS_FEATURE_IOMONITOR*/

#ifdef OPLUS_FEATURE_MEMLEAK_DETECT
#Add for svelte
obj-$(CONFIG_SVELTE)     +=  oplus/svelte/
#endif /* OPLUS_FEATURE_MEMLEAK_DETECT */

#ifdef OPLUS_BUG_STABILITY
obj-y += oplus/system/
#endif /* OPLUS_BUG_STABILITY */

#ifdef OPLUS_FEATURE_TP_BASIC
#Qicai.gu  12:00, 2020/06/03  devi info
obj-y +=   oplus/device_info/
##endif /* OPLUS_FEATURE_TP_BASIC */

#ifdef OPLUS_FEATURE_HEALTHINFO
obj-y  +=  oplus/healthinfo/
#endif /* OPLUS_FEATURE_HEALTHINFO */

#ifdef OPLUS_FEATURE_MIDAS
obj-$(CONFIG_OPLUS_FEATURE_MIDAS)    += oplus/midas/
#endif

#ifdef OPLUS_BUG_STABILITY
obj-y +=  oplus/oplus_custom/
#endif /* OPLUS_BUG_STABILITY */

#Add for NandSwap
obj-$(CONFIG_NANDSWAP)		+= oplus/oplus_nandswap/

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
obj-y += oplus/oplus_criticallog/
#endif OPLUS_FEATURE_MODEM_MINIDUMP

# ifdef OPLUS_FEATURE_SENSOR 
obj-$(CONFIG_OPLUS_ALS_CALI)   +=  oplus/oplus_als_cali_value/
# endif /* OPLUS_FEATURE_SENSOR */

