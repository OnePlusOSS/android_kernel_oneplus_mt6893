#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/fp5513e4af/
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/fp5513e4af_s5kjn103/
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/fp5513af/
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9718taf
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9718taf_evenc_s5kjn103
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/ak7314af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/lc898217af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/fp5516af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/lc898122af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9800waf
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9800af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9827af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/ak7316af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/b0954p65af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/bu64253af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/gt9778af_s5kjn103
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/lc898212xdaf/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/jd5516we4
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/jd5516we4af_23689_imx890
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/aw8601_chanel_ov064b
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9827af_23231_imx882
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9827af_23687_imx882
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9827af_23231_imx882pd
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9827af_23687_imx882pd
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9800saf
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/dw9800safpd
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/bu63165af/inc
else
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/bu63169af/inc
endif
ifeq ($(CONFIG_MACH_MT6779), y)
  ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/
endif

obj-$(CONFIG_MTK_LENS)    += mainaf.o

mainaf-y  += main_lens.o
mainaf-y  += common/aw8601_chanel_ov064b/AW8601xAF_CHANEL_OV64B.o
mainaf-y  += common/dw9827af_23231_imx882/DW9827AF_23231_IMX882.o
mainaf-y  += common/dw9827af_23687_imx882/DW9827AF_23687_IMX882.o
mainaf-y  += common/dw9827af_23231_imx882pd/DW9827AF_23231_IMX882PD.o
mainaf-y  += common/dw9827af_23687_imx882pd/DW9827AF_23687_IMX882PD.o
mainaf-y  += common/dw9800saf/DW9800SAF.o
mainaf-y  += common/dw9800safpd/DW9800SAFPD.o
mainaf-y  += common/fp5510e2af/FP5510E2AF.o
mainaf-y  += common/bu6424af/BU6424AF.o
mainaf-y  += common/bu6429af/BU6429AF.o
mainaf-y  += common/bu64748af/bu64748_function.o
mainaf-y  += common/bu64748af/bu64748af.o
mainaf-y  += common/dw9714af/DW9714AF.o
mainaf-y  += common/dw9800waf/DW9800WAF.o
mainaf-y  += common/dw9827af/DW9827AF.o
mainaf-y  += common/ak7316af/AK7316AF.o
mainaf-y  += common/dw9814af/DW9814AF.o
mainaf-y  += common/dw9718af/DW9718AF.o
mainaf-y  += common/dw9718saf/DW9718SAF.o
mainaf-y  += common/dw9718taf/DW9718TAF.o
mainaf-y  += common/ak7314af/AK7314AF.o
mainaf-y  += common/jd5516we4/JD5516WE4.o
mainaf-y  += common/jd5516we4af_23689_imx890/JD5516WE4AF_23689_IMX890.o
mainaf-y  += common/dw9718taf_parkera_ov13b10/DW9718TAF_PARKERA_OV13B10.o
mainaf-y  += common/dw9718taf_parkera_ov13b2a/DW9718TAF_PARKERA_OV13B2A.o
mainaf-y  += common/dw9718taf_parkera_s5kjn103/DW9718TAF_PARKERA_S5KJN103.o
mainaf-y  += common/dw9718taf_parkerb_s5kjn103/DW9718TAF_PARKERB_S5KJN103.o
mainaf-y  += common/dw9719taf/DW9719TAF.o
mainaf-y  += common/dw9763af/DW9763AF.o
mainaf-y  += common/dw9839af/DW9839AF.o
mainaf-y  += common/gt9764af/GT9764AF.o
mainaf-y  += common/gt9764afi/GT9764AFI.o
mainaf-y  += common/gt9764afii/GT9764AFII.o
mainaf-y  += common/lc898212af/LC898212AF.o
mainaf-y  += common/lc898212xdaf/LC898212XDAF.o
mainaf-y  += common/lc898212xdaf/AfInter.o
mainaf-y  += common/lc898212xdaf/AfInit.o
mainaf-y  += common/lc898212xdaf/AfSTMV.o
mainaf-y  += common/lc898214af/LC898214AF.o
mainaf-y  += common/lc898217af/LC898217AF.o
mainaf-y  += common/lc898217afa/LC898217AFA.o
mainaf-y  += common/lc898217afb/LC898217AFB.o
mainaf-y  += common/lc898217afc/LC898217AFC.o
mainaf-y  += common/lc898229af/LC898229AF.o
mainaf-y  += common/lc898122af/LC898122AF.o
mainaf-y  += common/lc898122af/OisCmd.o
mainaf-y  += common/lc898122af/OisIni.o
mainaf-y  += common/wv511aaf/WV511AAF.o
mainaf-y  += common/ak7371af/AK7371AF.o
mainaf-y  += common/bu63169af/BU63169AF.o
mainaf-y  += common/bu63169af/OIS_func.o
mainaf-y  += common/bu63169af/OIS_main.o
mainaf-y  += common/bu63169af/OIS_user.o
mainaf-y  += common/bu64253gwzaf/BU64253GWZAF.o
#ifdef OPLUS_FEATURE_CAMERA_COMMON
mainaf-y  += common/fp5516af/FP5516AF.o
mainaf-y  += common/bu64253af/BU64253AF.o
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
mainaf-y  += common/dw9718taf_zhaoyun/DW9718TAF_ZHAOYUN.o
mainaf-y  += common/gt9772af_limu_imx355/GT9772AF_LIMU_IMX355.o
mainaf-y  += common/dw9718taf_limu/DW9718TAF_LIMU.o
mainaf-y  += common/fp5513af_limu/FP5513AF_LIMU.o
#endif
mainaf-y  += common/gt9768af/GT9768AF.o
mainaf-y  += common/gt9772af/GT9772AF.o
mainaf-y  += common/gt9778af/GT9778AF.o
mainaf-y  += common/fp5513e4af/FP5513E4AF.o
mainaf-y  += common/gt9778af_s5kjn103/GT9778AF_S5KJN103.o
mainaf-y  += common/fp5513e4af_s5kjn103/fp5513e4af_s5kjn103.o
mainaf-y  += common/fp5513af/FP5513AF.o
mainaf-y  += common/dw9718taf_evenc_s5kjn103/DW9718TAF_EVENC_S5KJN103.o
mainaf-y  += common/fp5516af/FP5516AF.o
mainaf-y  += common/dw9800af/DW9800AF.o
mainaf-y  += common/b0954p65af/B0954P65AF.o
mainaf-y  += common/bu64253gwzaf/BU64253GWZAF.o
mainaf-y  += common/fp5510e2af/FP5510E2AF.o
mainaf-y  += common/bu6424af/BU6424AF.o
mainaf-y  += common/bu6429af/BU6429AF.o
mainaf-y  += common/bu64253af/BU64253AF.o
mainaf-y  += common/dw9714af/DW9714AF.o
mainaf-y  += common/dw9800waf/DW9800WAF.o
mainaf-y  += common/dw9827af/DW9827AF.o
mainaf-y  += common/ak7316af/AK7316AF.o
mainaf-y  += common/dw9814af/DW9814AF.o
mainaf-y  += common/dw9718af/DW9718AF.o
mainaf-y  += common/dw9718saf/DW9718SAF.o
mainaf-y  += common/dw9719taf/DW9719TAF.o
mainaf-y  += common/dw9763af/DW9763AF.o
mainaf-y  += common/dw9839af/DW9839AF.o
mainaf-y  += common/lc898212af/LC898212AF.o
mainaf-y  += common/lc898212xdaf/LC898212XDAF.o
mainaf-y  += common/lc898212xdaf/AfSTMV.o
mainaf-y  += common/lc898212xdaf/AfInter.o
mainaf-y  += common/lc898212xdaf/AfInit.o
mainaf-y  += common/lc898214af/LC898214AF.o
mainaf-y  += common/lc898217af/LC898217AF.o
mainaf-y  += common/lc898217afa/LC898217AFA.o
mainaf-y  += common/lc898217afb/LC898217AFB.o
mainaf-y  += common/lc898217afc/LC898217AFC.o
mainaf-y  += common/lc898122af/LC898122AF.o
mainaf-y  += common/lc898122af/OisCmd.o
mainaf-y  += common/lc898122af/OisIni.o
mainaf-y  += common/ad5820af/AD5820AF.o
mainaf-y  += common/wv511aaf/WV511AAF.o
mainaf-y  += common/ak7371af/AK7371AF.o
mainaf-y  += common/fp5516af/FP5516AF.o
mainaf-y  += common/ak7377af/AK7377AF.o
mainaf-y  += common/ak7375caf/AK7375CAF.o
mainaf-y  += common/gt9772af_aladdin/GT9772AF_ALADDIN.o
mainaf-y  += common/gt9772af_atom/GT9772AF_ATOM.o
mainaf-y  += common/gt9772af_atom_imx355/GT9772AF_ATOM_IMX355.o
