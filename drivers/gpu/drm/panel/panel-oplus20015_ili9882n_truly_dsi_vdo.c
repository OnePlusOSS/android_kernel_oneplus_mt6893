/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/of_graph.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"

#define AVDD_REG 0x00
#define AVDD_REG 0x01

/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
static char bl_tb0[] = { 0x51, 0xff };
extern int __attribute((weak)) tp_gesture_enable_flag(void) { return 0; };
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);

//extern unsigned long esd_flag;
static int esd_brightness;
extern void __attribute((weak)) lcd_queue_load_tp_fw(void) { return; };
/* #ifdef OPLUS_BUG_STABILITY */
static int cabc_lastlevel = 0;
static int last_brightness = 0;
static void cabc_switch(void *dsi, dcs_write_gce cb,
                void *handle, unsigned int cabc_mode);
/* #endif */
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;
};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{
	    .compatible = "mediatek,I2C_LCD_BIAS",
	},
	{},
};

static const struct i2c_device_id _lcm_i2c_id[] = { { LCM_I2C_ID_NAME, 0 },
						    {} };

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect		   = _lcm_i2c_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = LCM_I2C_ID_NAME,
		.of_match_table = _lcm_i2c_of_match,
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	pr_debug("[LCM][I2C] NT: info==>name=%s addr=0x%x\n", client->name,
		 client->addr);
	_lcm_i2c_client = client;
	return 0;
}

static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_debug("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	return ret;
}

/*
 * module load/unload record keeping
 */
static int __init _lcm_i2c_init(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_add_driver(&_lcm_i2c_driver);
	pr_debug("[LCM][I2C] %s success\n", __func__);
	return 0;
}

static void __exit _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);
/***********************************/

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

//	struct gpio_desc *bias_en;
	bool prepared;
	bool enabled;

	int error;

    bool is_normal_mode;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("SYQ %s+\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x01);  //3H 
	lcm_dcs_write_seq_static(ctx, 0x00, 0x47);  //STVA    
	lcm_dcs_write_seq_static(ctx, 0x01, 0x32);  //STVA Duty 4H
	lcm_dcs_write_seq_static(ctx, 0x02, 0x00);  //45%   CLK duty 
	lcm_dcs_write_seq_static(ctx, 0x03, 0x00);  //45%   CLK duty
	lcm_dcs_write_seq_static(ctx, 0x04, 0x04);  //STVB
	lcm_dcs_write_seq_static(ctx, 0x05, 0x32);  //STV Duty 4H
	lcm_dcs_write_seq_static(ctx, 0x06, 0x00);  //45%   CLK duty
	lcm_dcs_write_seq_static(ctx, 0x07, 0x00);  //45%   CLK duty
	lcm_dcs_write_seq_static(ctx, 0x08, 0x85);  //CLK RISE    
	lcm_dcs_write_seq_static(ctx, 0x09, 0x04);  //CLK FALL   
	lcm_dcs_write_seq_static(ctx, 0x0a, 0x72); //CLK Duty 4H 
	lcm_dcs_write_seq_static(ctx, 0x0b, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x0c, 0x00);  //45%   CLK duty
	lcm_dcs_write_seq_static(ctx, 0x0d, 0x00); //45%   CLK duty   
	lcm_dcs_write_seq_static(ctx, 0x0e, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x0f, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x28, 0x48);  //STCH1   
	lcm_dcs_write_seq_static(ctx, 0x29, 0x88);
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x48);   //STCH2 
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x88);
								
	lcm_dcs_write_seq_static(ctx, 0x31, 0x0C);     // RST_L
	lcm_dcs_write_seq_static(ctx, 0x32, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x33, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x34, 0x23);     // GLV
	lcm_dcs_write_seq_static(ctx, 0x35, 0x02);     // VGL_L
	lcm_dcs_write_seq_static(ctx, 0x36, 0x08);     // STV1_L
	lcm_dcs_write_seq_static(ctx, 0x37, 0x0A);     // STV2_L
	lcm_dcs_write_seq_static(ctx, 0x38, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x39, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x3A, 0x10);     // CLK1_L
	lcm_dcs_write_seq_static(ctx, 0x3B, 0x10);     // CLK1_L
	lcm_dcs_write_seq_static(ctx, 0x3C, 0x12);     // CLK2_L
	lcm_dcs_write_seq_static(ctx, 0x3D, 0x12);     // CLK2_L
	lcm_dcs_write_seq_static(ctx, 0x3E, 0x14);     // CK1B_L
	lcm_dcs_write_seq_static(ctx, 0x3F, 0x14);     // CK1B_L
	lcm_dcs_write_seq_static(ctx, 0x40, 0x16);     // CK2B_L
	lcm_dcs_write_seq_static(ctx, 0x41, 0x16);     // CK2B_L
	lcm_dcs_write_seq_static(ctx, 0x42, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x43, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x44, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x45, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x46, 0x07);     // 
								
	lcm_dcs_write_seq_static(ctx, 0x47, 0x0D);     // RST_R
	lcm_dcs_write_seq_static(ctx, 0x48, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x49, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x4A, 0x23);     // GLV
	lcm_dcs_write_seq_static(ctx, 0x4B, 0x02);     // VGL_R
	lcm_dcs_write_seq_static(ctx, 0x4C, 0x09);     // STV1_R
	lcm_dcs_write_seq_static(ctx, 0x4D, 0x0B);     // STV2_R
	lcm_dcs_write_seq_static(ctx, 0x4E, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x4F, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x50, 0x11);     // CLK1_R
	lcm_dcs_write_seq_static(ctx, 0x51, 0x11);     // CLK1_R
	lcm_dcs_write_seq_static(ctx, 0x52, 0x13);     // CLK2_R
	lcm_dcs_write_seq_static(ctx, 0x53, 0x13);     // CLK2_R
	lcm_dcs_write_seq_static(ctx, 0x54, 0x15);     // CK1B_R
	lcm_dcs_write_seq_static(ctx, 0x55, 0x15);     // CK1B_R
	lcm_dcs_write_seq_static(ctx, 0x56, 0x17);     // CK2B_R
	lcm_dcs_write_seq_static(ctx, 0x57, 0x17);     // CK2B_R
	lcm_dcs_write_seq_static(ctx, 0x58, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x59, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x5A, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x5B, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x5C, 0x07);     // 
								
	lcm_dcs_write_seq_static(ctx, 0x61, 0x0C);     // RST_L
	lcm_dcs_write_seq_static(ctx, 0x62, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x63, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x64, 0x23);     // GLV
	lcm_dcs_write_seq_static(ctx, 0x65, 0x02);     // VGL_L
	lcm_dcs_write_seq_static(ctx, 0x66, 0x08);     // STV1_L
	lcm_dcs_write_seq_static(ctx, 0x67, 0x0A);     // STV2_L
	lcm_dcs_write_seq_static(ctx, 0x68, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x69, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x6A, 0x10);     // CLK1_L
	lcm_dcs_write_seq_static(ctx, 0x6B, 0x10);     // CLK1_L
	lcm_dcs_write_seq_static(ctx, 0x6C, 0x12);     // CLK2_L
	lcm_dcs_write_seq_static(ctx, 0x6D, 0x12);     // CLK2_L
	lcm_dcs_write_seq_static(ctx, 0x6E, 0x14);     // CK1B_L
	lcm_dcs_write_seq_static(ctx, 0x6F, 0x14);     // CK1B_L
	lcm_dcs_write_seq_static(ctx, 0x70, 0x16);     // CK2B_L
	lcm_dcs_write_seq_static(ctx, 0x71, 0x16);     // CK2B_L
	lcm_dcs_write_seq_static(ctx, 0x72, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x73, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x74, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x75, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x76, 0x07);     // 
								
	lcm_dcs_write_seq_static(ctx, 0x77, 0x0D);     // RST_R
	lcm_dcs_write_seq_static(ctx, 0x78, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x79, 0x02);     // VGL
	lcm_dcs_write_seq_static(ctx, 0x7A, 0x23);     // GLV
	lcm_dcs_write_seq_static(ctx, 0x7B, 0x02);     // VGL_R
	lcm_dcs_write_seq_static(ctx, 0x7C, 0x09);     // STV1_R
	lcm_dcs_write_seq_static(ctx, 0x7D, 0x0B);     // STV2_R
	lcm_dcs_write_seq_static(ctx, 0x7E, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x7F, 0x06);     // VDD
	lcm_dcs_write_seq_static(ctx, 0x80, 0x11);     // CLK1_R
	lcm_dcs_write_seq_static(ctx, 0x81, 0x11);     // CLK1_R
	lcm_dcs_write_seq_static(ctx, 0x82, 0x13);     // CLK2_R
	lcm_dcs_write_seq_static(ctx, 0x83, 0x13);     // CLK2_R
	lcm_dcs_write_seq_static(ctx, 0x84, 0x15);     // CK1B_R
	lcm_dcs_write_seq_static(ctx, 0x85, 0x15);     // CK1B_R
	lcm_dcs_write_seq_static(ctx, 0x86, 0x17);     // CK2B_R
	lcm_dcs_write_seq_static(ctx, 0x87, 0x17);     // CK2B_R
	lcm_dcs_write_seq_static(ctx, 0x88, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x89, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x8A, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x8B, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0x8C, 0x07);     // 
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x33);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x33);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xE2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xE6, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xE7, 0x54);
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xF1, 0x1C);    // Tcon ESD option
	lcm_dcs_write_seq_static(ctx, 0x4B, 0x5A);    // line_chopper
	lcm_dcs_write_seq_static(ctx, 0x50, 0xCA);    // line_chopper
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00);     // line_chopper
	lcm_dcs_write_seq_static(ctx, 0x06, 0x8F);     // Internal Line Time (RTN)
	lcm_dcs_write_seq_static(ctx, 0x0B, 0xA0);     // Internal VFP[9]
	lcm_dcs_write_seq_static(ctx, 0x0C, 0x00);     // Internal VFP[8]
	lcm_dcs_write_seq_static(ctx, 0x0D, 0x14);     // Internal VBP
	lcm_dcs_write_seq_static(ctx, 0x0E, 0xE6);     // Internal VFP
	lcm_dcs_write_seq_static(ctx, 0x4E, 0x11);     // SRC BIAS
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x05);
	lcm_dcs_write_seq_static(ctx, 0x03, 0x00);   // Vcom
	lcm_dcs_write_seq_static(ctx, 0x04, 0xD7);   // Vcom
	lcm_dcs_write_seq_static(ctx, 0x58, 0x61);   // VGL 2x
	lcm_dcs_write_seq_static(ctx, 0x63, 0x8D);    // GVDDN = -5.3V
	lcm_dcs_write_seq_static(ctx, 0x64, 0x8D);    // GVDDP = 5.3V
	lcm_dcs_write_seq_static(ctx, 0x68, 0xA1);    // VGHO = 15V
	lcm_dcs_write_seq_static(ctx, 0x69, 0xA7);    // VGH = 16V
	lcm_dcs_write_seq_static(ctx, 0x6A, 0x79);    // VGLO = -10V
	lcm_dcs_write_seq_static(ctx, 0x6B, 0x6B);    // VGL = -11V
	lcm_dcs_write_seq_static(ctx, 0x85, 0x37);     // HW RESET option
	lcm_dcs_write_seq_static(ctx, 0x46, 0x00);     // LVD HVREG option
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x06);
	lcm_dcs_write_seq_static(ctx, 0xD9, 0x10);     // 3Lane
	lcm_dcs_write_seq_static(ctx, 0xC0, 0x40);     // NL = 1600
	lcm_dcs_write_seq_static(ctx, 0xC1, 0x16);     // NL = 1600
	lcm_dcs_write_seq_static(ctx, 0x06, 0xA4); 
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x08);								
	lcm_dcs_write_seq_static(ctx, 0xE0, 0x00,0x24,0x76,0xAB,0xEB,0x55,0x20,0x49,0x79,0xA0,0xA9,0xDE,0x11,0x3D,0x68,0xEA,0x94,0xC6,0xE6,0x0D,0xFF,0x2C,0x55,0x85,0xB5,0x03,0xEC);								
	lcm_dcs_write_seq_static(ctx, 0xE1, 0x00,0x24,0x76,0xAB,0xEB,0x55,0x20,0x49,0x79,0xA0,0xA9,0xDE,0x11,0x3D,0x68,0xEA,0x94,0xC6,0xE6,0x0D,0xFF,0x2C,0x55,0x85,0xB5,0x03,0xEC);								
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0x44);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0x81);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x70);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x70);
	lcm_dcs_write_seq_static(ctx, 0xAB, 0xE0);     // AutoTrimType
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x0E);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x10);     // TSVD Rise position
	lcm_dcs_write_seq_static(ctx, 0x13, 0x10);     // LV mode TSHD Rise position
	lcm_dcs_write_seq_static(ctx, 0x00, 0xA0);      // LV mode
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x80, 0x35); 
	lcm_dcs_write_seq_static(ctx, 0x81, 0x35); 
	lcm_dcs_write_seq_static(ctx, 0x82, 0x35); 
	lcm_dcs_write_seq_static(ctx, 0x83, 0x30);     
	lcm_dcs_write_seq_static(ctx, 0x84, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x85, 0x40);
	lcm_dcs_write_seq_static(ctx, 0x86, 0x10);
	lcm_dcs_write_seq_static(ctx, 0x87, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xAF, 0x18); 
	lcm_dcs_write_seq_static(ctx, 0x88, 0xCC); 
	lcm_dcs_write_seq_static(ctx, 0x89, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x8A, 0xED);
	lcm_dcs_write_seq_static(ctx, 0x8B, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x8C, 0x9B); 
	lcm_dcs_write_seq_static(ctx, 0x8D, 0xBA);
	lcm_dcs_write_seq_static(ctx, 0x8E, 0x99);
	lcm_dcs_write_seq_static(ctx, 0x8F, 0xA7);
	lcm_dcs_write_seq_static(ctx, 0x90, 0xB2); 
	lcm_dcs_write_seq_static(ctx, 0x91, 0xC5);
	lcm_dcs_write_seq_static(ctx, 0x92, 0xD5);
	lcm_dcs_write_seq_static(ctx, 0x93, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x94, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0x95, 0xF6); 
	lcm_dcs_write_seq_static(ctx, 0x96, 0x70);
	lcm_dcs_write_seq_static(ctx, 0x97, 0xAD);
	lcm_dcs_write_seq_static(ctx, 0x98, 0x93); 
	lcm_dcs_write_seq_static(ctx, 0x99, 0xA0);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0xA3);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0xBD);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0xCD); 
	lcm_dcs_write_seq_static(ctx, 0x9D, 0xDD);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0xAC, 0xFA); //UI mode white
	lcm_dcs_write_seq_static(ctx, 0xAD, 0xE0); //Still mode white
	lcm_dcs_write_seq_static(ctx, 0xAE, 0xD3); //Moving mode white   
								
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x82, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x68, 0x04, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00); 
	msleep(20);
	pr_info("%s-\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(20);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);

	ctx->error = 0;
	ctx->prepared = false;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	pr_info("%s: tp_gesture_enable_flag = %d \n", __func__, tp_gesture_enable_flag());
	if ((0 == tp_gesture_enable_flag())) {
	pr_info("%s:  \n", __func__);
	/*ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	*/

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
/*        usleep_range(2 * 1000, 2 * 1000);
        ctx->bias_en = devm_gpiod_get(ctx->dev, "ldo", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_en, 0);
        devm_gpiod_put(ctx->dev, ctx->bias_en);*/
	}
#endif
	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s+\n", __func__);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0x14);
	_lcm_i2c_write_bytes(0x1, 0x14);
	
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        usleep_range(5 * 1000, 5 * 1000);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(10 * 1000, 10 * 1000);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	pr_info("%s-\n", __func__);
	/* #ifdef OPLUS_BUG_STABILITY */
//  	lcd_queue_load_tp_fw();
  	/* #endif */ /*OPLUS_BUG_STABILITY*/
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
	pr_info("%s-\n", __func__);
/*	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);*/
/*	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_lcm_i2c_write_bytes(0x0, 0xf);
	_lcm_i2c_write_bytes(0x1, 0xf);
    msleep(10);*/
#endif

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(10 * 1000, 10 * 1000);
	lcd_queue_load_tp_fw();
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define VAC (1600)
#define HAC (720)

static struct drm_display_mode default_mode = {
	.clock = 91191,
	.hdisplay = HAC,
	.hsync_start = HAC + 40,//HFP
	.hsync_end = HAC + 40 + 8,//HSA
	.htotal = HAC + 40 + 8 + 50,//HBP1289
	.vdisplay = VAC,
	.vsync_start = VAC + 240,//VFP
	.vsync_end = VAC + 240 + 2,//VSA
	.vtotal = VAC + 240 + 2 + 16,//VBP4948
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	//.pll_clk = 553,
	//.vfp_low_power = 2528,//45hz
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	/*.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9c, .mask_list[0] = 0x9c,
	},*/
	.data_rate = 733,
	.tp_lcd_suspend = 1,
	/*.dyn = {
		.pll_clk = 362,
		.switch_en = 1,
		.data_rate = 724,
		.hbp = 36,
	},*/
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	/*char bl_tb0[] = {0x51, 0x07, 0xff};
	if (level > 2047)
		level = 2047;
	pr_err("%s backlight = %d\n", __func__, level);
	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));*/

	char bl_tb0[] = {0x51, 0xFF, 0xFF};
        char bl_tb1[] = {0x55, 0x00};
        char bl_tb3[] = {0x53, 0x24};
	pr_err("%s backlight = %d\n", __func__, level);
	if (level > 4095)
                level = 4095;
        if ((level > 0) && (level < 10))
                level = 10;
        bl_tb0[1] = level >> 8;
        bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;

	pr_err("%s SYQ bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	//return 0;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
/* #ifdef OPLUS_BUG_STABILITY */
	if ((last_brightness == 0) && (cabc_lastlevel != 0)) {
		bl_tb1[1] = cabc_lastlevel;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}
	last_brightness = level;
/* #endif */
	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext->params = &ext_params;
	else
		ret = 1;

	return ret;
}

static int mtk_panel_ext_param_get(struct mtk_panel_params *ext_para,
			 unsigned int mode)
{
	int ret = 0;
	pr_err("%s ,%d\n", __func__,mode);
	if (mode == 0)
		ext_para = &ext_params;
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x2C};
	return;
	pr_err("%s cabc = %d\n", __func__, cabc_mode);
	bl_tb0[1] = (u8)cabc_mode;
    msleep(5);
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
/* #ifdef OPLUS_BUG_STABILITY */
	cabc_lastlevel = cabc_mode;
/* #endif */
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	//.esd_backlight_check = oplus_esd_backlight_check,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.mode_switch = mode_switch,
	//.cabc_switch = cabc_switch,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 151;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			pr_info("device_node name:%s\n", remote_node->name);
                   }
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm.\n", __func__);
		return 0;
	}	

	pr_info("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}
	
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}
static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "truly,ili9882n,dphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "oplus20015_ili9882n_truly_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
