/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
******************************************************************/
#ifndef _OPLUS_DISPLAY_PRIVATE_API_H_
#define _OPLUS_DISPLAY_PRIVATE_API_H_

#include <linux/err.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <drm/drm_device.h>
#include <drm/drmP.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/mailbox/mtk-cmdq-mailbox.h>
#include <drm/drm_mipi_dsi.h>
#include <video/videomode.h>

#include "mtk_drm_drv.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_plane.h"
#include "mtk_writeback.h"
#include "mtk_fence.h"
#include "mtk_sync.h"
#include "mtk_drm_session.h"
#include "mtk_dump.h"
#include "mtk_drm_fb.h"
#include "mtk_rect.h"
#include "mtk_drm_ddp_addon.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_lowpower.h"
#include "mtk_drm_fbdev.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_disp_recovery.h"
#include "mtk_drm_arr.h"
#include "mtk_drm_trace.h"

#define OPLUS_SILKY_MAX_BRIGHTNESS 8191
#define OPLUS_MAX_BRIGHTNESS 4095
#define OPLUS_NORMAL_MAX_BRIGHTNESS 2047
#define OPLUS_MIN_BRIGHTNESS 0

struct t_condition_wq {
	wait_queue_head_t wq;
	atomic_t condition;
};

struct mtk_dsi {
	struct mtk_ddp_comp ddp_comp;
	struct device *dev;
	struct mipi_dsi_host host;
	struct drm_encoder encoder;
	struct drm_connector conn;
	struct drm_panel *panel;
	struct mtk_panel_ext *ext;
	struct cmdq_pkt_buffer cmdq_buf;
	struct drm_bridge *bridge;
	struct phy *phy;

	void __iomem *regs;

	struct clk *engine_clk;
	struct clk *digital_clk;
	struct clk *hs_clk;

	u32 data_rate;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	struct videomode vm;
	int clk_refcnt;
	bool output_en;
	bool doze_enabled;
	u32 irq_data;
	wait_queue_head_t irq_wait_queue;
	struct mtk_dsi_driver_data *driver_data;

	struct t_condition_wq enter_ulps_done;
	struct t_condition_wq exit_ulps_done;
	unsigned int hs_trail;
	unsigned int hs_prpr;
	unsigned int hs_zero;
	unsigned int lpx;
	unsigned int ta_get;
	unsigned int ta_sure;
	unsigned int ta_go;
	unsigned int da_hs_exit;
	unsigned int cont_det;
	unsigned int clk_zero;
	unsigned int clk_hs_prpr;
	unsigned int clk_hs_exit;
	unsigned int clk_hs_post;

	unsigned int vsa;
	unsigned int vbp;
	unsigned int vfp;
	unsigned int hsa_byte;
	unsigned int hbp_byte;
	unsigned int hfp_byte;

	bool mipi_hopping_sta;
	bool panel_osc_hopping_sta;
};


/* aod_area begin */
struct aod_area {
	bool enable;
	int x;
	int y;
	int w;
	int h;
	int color;
	int bitdepth;
	int mono;
	int gray;
};


#ifdef CONFIG_OPLUS_CRC_CHECK_SUPPORT
/* store reg value returned on crc check */
struct crc_check_value {
	char reg_14_1st_0;
	char reg_14_1st_1;
	char reg_14_2nd_0;
	char reg_14_2nd_1;
	char reg_15_1st_0;
	char reg_15_1st_1;
	char reg_15_2nd_0;
	char reg_15_2nd_1;
};
#endif

#define RAMLESS_AOD_AREA_NUM		6
#define RAMLESS_AOD_PAYLOAD_SIZE	100

#endif /* _OPLUS_DISPLAY_PRIVATE_API_H_ */
