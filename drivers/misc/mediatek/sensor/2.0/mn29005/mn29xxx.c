// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include "hf_manager.h"
#include "mn29xxx.h"
#include "../oplus_sensor_devinfo/sensor_devinfo.h"

#define CHECK_CHIP_ID_TIME_MAX          0x05
#define C_I2C_FIFO_SIZE                 0x08
#define LIGHT_SENSOR_NAME               "mn29xxx"
#define SEN_TYPE                        SENSOR_TYPE_REAR_ALS
#define MN29xxx_ID                      0x88
#define ALS_POLLING_MODE                1
#define REG_MONITOR                     1
#define DBG_ENABLE                      0
#define ALS_MAX_LUX_PATCH               1
#define ALS_MAX_LUX_LINEARLY            1

#if ALS_MAX_LUX_LINEARLY
#define MAX_LUX_H_THD       50000
#define MAX_LUX_RATIO       1200
#endif
static mn29_optical_sensor mn29_sensor;
#define LOG_TAG "[MN29xxx]"
#define APS_LOGE(fmt, ...) do { \
		pr_err( LOG_TAG " " fmt,  ##__VA_ARGS__);  \
	} while (0);
#define APS_LOGI(fmt, ...) do { \
		if (DBG_ENABLE) {  \
			pr_info( LOG_TAG " " fmt,  ##__VA_ARGS__);  \
		} \
	} while (0);

typedef enum
{
	CMC_BIT_RAW             = 0x0,
	CMC_BIT_PRE_COUNT       = 0x1,
	CMC_BIT_DYN_INT         = 0x2,
} CMC_ALS_REPORT_TYPE;
/******************************************************************************
 *  ALS_DYN_INTT
 ******************************************************************************/
//Dynamic INTT
uint16_t dynamic_intt_idx;
uint16_t dynamic_intt_init_idx = 1; //initial dynamic_intt_idx
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint8_t als_dynamic_intt_intt[3]; //initial_global_variable
uint16_t als_dynamic_intt_value[3];
uint8_t als_dynamic_intt_gain[3];
uint8_t als_dynamic_intt_cycle[3];
uint8_t als_dynamic_intt_enh[3];
uint16_t als_dynamic_intt_high_thr[3];
uint16_t als_dynamic_intt_low_thr[3];
uint8_t als_dynamic_intt_rs[3]; //2^n
uint16_t als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_intt)/sizeof(uint8_t);
uint16_t mn_als_rs_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
uint16_t mn_rs_num = sizeof(mn_als_rs_value)/sizeof(uint16_t);
/******************************************************************************
 *  ALS Light source
 ******************************************************************************/
int offset_gain;
int scale_gain;
uint32_t lsrc_als_offset = 0;
uint16_t lsrc_raw = 0;
uint32_t lsrc_lux = 0;
uint32_t lsrc_ratio = 0;
static uint16_t debug_cnt = 0;

#if ALS_MAX_LUX_PATCH
uint8_t als_max_intt;
uint8_t als_max_gain;
uint8_t als_max_cycle;
uint16_t lpc_gain;
uint32_t als_l2ll_lux_thd;
uint32_t als_ll2l_lux_thd;
bool als_max_lux_flag;
#endif

static struct sensor_info mn29xxx_sensor_info[] = {
	{
		.sensor_type = SEN_TYPE,
		.gain = 100,
	},
};

struct mn29xxx_device {
	struct hf_device hf_dev;
	uint32_t i2c_num;
	uint32_t i2c_addr;
	struct i2c_client *client;
	atomic_t raw_enable;
	struct mutex para_lock;
};
struct mn29xxx_device *gdriver_dev = NULL;
struct cali_data* g_rear_cali_data = NULL;


static int sensor_read_oplus_custom_rear(struct cali_data *data)
{
	get_sensor_parameter_rear_als(data);
	return 0;
}

/* I2C operation functions */
static int mn29xxx_i2c_read_block(struct i2c_client *client,
				uint8_t addr, uint8_t *data, uint8_t len)
{
	int err = 0;
	uint8_t beg = addr;
	struct i2c_msg msgs[2] = {
		{/*.addr = client->addr,*/
			.flags = 0,
			.len = 1,
			.buf = &beg
		},
		{/*.addr = client->addr*/
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};
	if (!client)
		return -EINVAL;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		pr_err_ratelimited("mn29xxx i2c_trans err:%x %x (%d %p %d) %d\n",
				   msgs[0].addr, client->addr, addr, data, len,
				   err);
		err = -EIO;
	} else {
		err = 0; /*no error*/
	}
	return err;
}

static int mn29xxx_i2c_write_block(struct i2c_client *client,
				uint8_t addr, uint8_t *data, uint8_t len)
{
	/* because address also occupies one byte,
	 * the maximum length for write is 7 bytes
	 */
	int err = 0, idx = 0, num = 0;
	char buf[32];

	if (!client) {
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		pr_err_ratelimited("mn29xxx len %d fi %d\n", len,
				   C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		pr_err_ratelimited("mn29xxx send command error!!\n");
		return -EFAULT;
	}

	return 0;
}
/****************************************************************************/
static int mn29xxx_set_lsensor_intr_threshold(struct i2c_client *client, uint16_t ailt, uint16_t aiht)
{
	int rc = 0;
	uint8_t thd[4];

	thd[0] = (uint8_t) (ailt & 0x00ff);
	thd[1] = (uint8_t) (ailt >> 8);
	thd[2] = (uint8_t) (aiht & 0x00ff);
	thd[3] = (uint8_t) (aiht >> 8);
	rc = mn29xxx_i2c_write_block( client, DEVREG_ALS_ILTL, thd, 4);
	APS_LOGI("[mn29xxx_set_lsensor_intr_threshold] - rc=%d, low_thd = %d, high_thd = %d \n", rc, ailt, aiht);
	return rc;
}

static void mn29xxx_als_dyn_cfg(struct i2c_client *client, uint8_t cycle)
{
	uint8_t buf_block[3], als_persist;
#if 0
	if( (dynamic_intt_idx==0) && (als_first_flag==false) && (mn29_sensor.als.polling_mode==0))
		als_persist = MN_PERIST_4;
	else
#endif
		als_persist = mn29_sensor.als.persist;

	buf_block[0] = (MN_POWER_OFF | MN_RESETN_RESET);
	mn29xxx_i2c_write_block( client, DEVREG_RESET, buf_block, 1 );
	buf_block[0] = mn29_sensor.als.als_std | mn29_sensor.als.integration_time | mn29_sensor.als.gain;
	buf_block[1] = mn29_sensor.als.enh_mode | mn29_sensor.als.interrupt_channel_select | cycle;
	buf_block[2] = mn29_sensor.als.als_rs | als_persist | mn29_sensor.als.interrupt_type;
	mn29xxx_i2c_write_block( client, DEVREG_ALS_CONFIG, buf_block, 3 );
	buf_block[0] = (MN_POWER_ON | MN_RESETN_RUN);
	mn29xxx_i2c_write_block( client, DEVREG_RESET, buf_block, 1 );
}
//write_global_variable
static int  mn29xxx_write_global_variable(struct i2c_client *client)
{
	uint8_t buf_block[7];
	int ret = 0;
	buf_block[0] = (MN_POWER_ON | MN_RESETN_RUN);
	ret = mn29xxx_i2c_write_block( client, DEVREG_RESET, buf_block, 1);
	buf_block[0] = (MN_CMP_RESET | MN_LOCK);
	ret = mn29xxx_i2c_write_block( client, DEVREG_PS_STATUS, buf_block, 1 );
	buf_block[0] = (MN_CMP_RUN | MN_UN_LOCK);
	ret = mn29xxx_i2c_write_block( client, DEVREG_PS_STATUS, buf_block, 1 );
	buf_block[0] = (MN_CMP_RESET | MN_LOCK);
	ret = mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, buf_block, 1 );
	buf_block[0] = (MN_CMP_RUN | MN_UN_LOCK);
	ret = mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, buf_block, 1 );
	buf_block[0] = 0xFF;
	ret = mn29xxx_i2c_write_block( client, 0x27, buf_block, 1 );
	buf_block[0] = 0x0F;
	ret = mn29xxx_i2c_write_block( client, 0x28, buf_block, 1 );

	buf_block[0] = (MN_POWER_OFF | MN_RESETN_RESET);
	ret = mn29xxx_i2c_write_block( client, DEVREG_RESET, buf_block, 1 );
	mn29xxx_set_lsensor_intr_threshold(client, mn29_sensor.als.low_threshold, mn29_sensor.als.high_threshold);
	buf_block[0] = (mn29_sensor.interrupt_control | mn29_sensor.als.als_dark_pixel);
	ret = mn29xxx_i2c_write_block( client, DEVREG_INT_CTRL, buf_block, 1 ); //ALS_DARK_OFF, ALS_PIXEL_NOR
	//als config
	buf_block[0] = mn29_sensor.als.als_std | mn29_sensor.als.integration_time |mn29_sensor.als.gain;
	buf_block[1] = mn29_sensor.als.enh_mode | mn29_sensor.als.interrupt_channel_select | mn29_sensor.als.cycle;
	buf_block[2] = mn29_sensor.als.als_rs | mn29_sensor.als.persist | mn29_sensor.als.interrupt_type;
	ret = mn29xxx_i2c_write_block( client, DEVREG_ALS_CONFIG, buf_block, 3 );
	//set mode and wait
	buf_block[0] = (mn29_sensor.wait | mn29_sensor.mode);
	ret = mn29xxx_i2c_write_block( client, DEVREG_ENABLE, buf_block, 1 );
	APS_LOGI( "[mn29xxx_write_global_variable] - ret=%d \n", ret);
	return ret;
}
static void mn29xxx_als_ag_calc_new_thd(void)
{
	int idx=0, i=0, j=0, gain_value=0, intt_value=0, enh_value=1, enh_temp=0, total_value=0;

	als_dynamic_intt_intt[0] = mn29_sensor.als.als_aintt_h;
	als_dynamic_intt_intt[1] = mn29_sensor.als.als_aintt_m;
	als_dynamic_intt_intt[2] = mn29_sensor.als.als_aintt_l;
	als_dynamic_intt_value[0] = mn_als_intt_value[mn29_sensor.als.als_aintt_h>>2];
	als_dynamic_intt_value[1] = mn_als_intt_value[mn29_sensor.als.als_aintt_m>>2];
	als_dynamic_intt_value[2] = mn_als_intt_value[mn29_sensor.als.als_aintt_l>>2];
	als_dynamic_intt_gain[0] =  mn29_sensor.als.als_ag_h;
	als_dynamic_intt_gain[1] =  mn29_sensor.als.als_ag_m;
	als_dynamic_intt_gain[2] =  mn29_sensor.als.als_ag_l;
	als_dynamic_intt_enh[0] = mn29_sensor.als.als_aenh_h;
	als_dynamic_intt_enh[1] = mn29_sensor.als.als_aenh_m;
	als_dynamic_intt_enh[2] = mn29_sensor.als.als_aenh_l;
	als_dynamic_intt_cycle[0] = mn29_sensor.als.als_acycle_h;
	als_dynamic_intt_cycle[1] = mn29_sensor.als.als_acycle_m;
	als_dynamic_intt_cycle[2] = mn29_sensor.als.als_acycle_l;
	als_dynamic_intt_high_thr[0] = mn29_sensor.als.als_ag_h2m_thd;
	als_dynamic_intt_high_thr[1] = mn29_sensor.als.als_ag_m2l_thd;
	als_dynamic_intt_high_thr[2] = 65535;
	als_dynamic_intt_low_thr[0] = 0;
	als_dynamic_intt_low_thr[1] = mn29_sensor.als.als_ag_m2h_thd;
	als_dynamic_intt_low_thr[2] = mn29_sensor.als.als_ag_l2m_thd;
	for(i = 0; i < (als_dynamic_intt_intt_num-1); i++)
	{
		if (als_dynamic_intt_gain[i] == MN_GAIN_HIGH) {
			gain_value = 64;
		} else if (als_dynamic_intt_gain[i] == MN_GAIN_MID) {
			gain_value = 8;
		} else {
			gain_value = 1;
		}

		if (als_dynamic_intt_gain[als_dynamic_intt_intt_num-1] == MN_GAIN_HIGH) {
			gain_value /= 64;
		} else if(als_dynamic_intt_gain[als_dynamic_intt_intt_num-1] == MN_GAIN_MID) {
			gain_value /= 8;
		}

		enh_value = 1;
		for(j=0; j<(als_dynamic_intt_enh[i]  >> 6) ; j++)
			enh_value *= 2;
		enh_temp = 1;
		for(j=0; j<(als_dynamic_intt_enh[als_dynamic_intt_intt_num-1] >> 6) ; j++)
			enh_temp *= 2;
		enh_value = enh_value/enh_temp;

		intt_value = als_dynamic_intt_value[i] / als_dynamic_intt_value[(als_dynamic_intt_intt_num-1)];
		total_value = gain_value * intt_value * enh_value;
		for(idx = 0; idx < mn_rs_num;  idx++)
		{
		    if (total_value < mn_als_rs_value[idx]) {
		        break;
		    }
		}
		APS_LOGI( "[mn29xxx_als_ag_calc_new_thd]: idx=%d, mn_als_rs_value=%d, total_value=%d, enh_value=%d \r\n", idx-1, mn_als_rs_value[idx-1], total_value, enh_value);
		als_dynamic_intt_rs[i] = ((idx-1)<<4);
		als_dynamic_intt_high_thr[i] = als_dynamic_intt_high_thr[i]/total_value;
		als_dynamic_intt_low_thr[i] = als_dynamic_intt_low_thr[i]/total_value;
		APS_LOGI( "[mn29xxx_als_ag_calc_new_thd]: als_dynamic_intt_low_thr[%d]=%d, als_dynamic_intt_high_thr[%d]=%d \r\n", i, als_dynamic_intt_low_thr[i], i, als_dynamic_intt_high_thr[i]);
	}
}

static int mn29xxx_initial_global_variable(struct i2c_client *client)
{
	int ret = 0;
	uint8_t rx_buf[2] = {0};

	mn29xxx_i2c_read_block(client, DEVREG_REV_ID, rx_buf, 2);
	mn29_sensor.revno = (uint16_t) rx_buf[0] | (rx_buf[1]<<8);
	APS_LOGI("[mn29xxx_initial_global_variable]: renvo = 0x%x \n", mn29_sensor.revno);
	//general setting
	mn29_sensor.mode = MN_MODE_IDLE;
	mn29_sensor.wait = MN_WAIT_0_MS;
	//als setting
	mn29_sensor.als.polling_mode = ALS_POLLING_MODE;
	mn29_sensor.interrupt_control = MN_INT_CTRL_ALS_OR_PS;
	mn29_sensor.als.als_dark_pixel = ALS_DARK_OFF;
	if (mn29_sensor.als.polling_mode) {
		mn29_sensor.als.interrupt_type = MN_INTTY_DISABLE;
	} else {
		mn29_sensor.als.interrupt_type = MN_INTTY_ACTIVE;
	}
	mn29_sensor.als.factory.lux_per_count = 250;  //coarse
	mn29_sensor.als.factory.lux_per_lux = 1000;   //fine
	mn29_sensor.als.integration_time = MN_ALS_INTT_128;
	mn29_sensor.als.gain = MN_GAIN_LOW;
	mn29_sensor.als.cycle = MN_CYCLE_16;
	mn29_sensor.als.report_type = CMC_BIT_DYN_INT; //CMC_BIT_DYN_INT //CMC_BIT_RAW; //CMC_BIT_PRE_COUNT
	mn29_sensor.als.als_intr_percent = 5; //20%
	mn29_sensor.als.enh_mode = MN_ALS_ENH_MODE_1;
	mn29_sensor.als.als_rs = MN_RS_0;
	mn29_sensor.als.als_std = MN_ALS_PRE;
	mn29_sensor.als.persist = MN_PERIST_1;
	mn29_sensor.als.interrupt_channel_select = MN_ALS_INT_CHSEL_1;
	//ALS AG CONFIG
	mn29_sensor.als.als_aintt_h = MN_ALS_INTT_1024;
	mn29_sensor.als.als_aintt_m = MN_ALS_INTT_512;
	mn29_sensor.als.als_aintt_l = MN_ALS_INTT_128;
	mn29_sensor.als.als_aenh_h = MN_ALS_ENH_MODE_1;
	mn29_sensor.als.als_aenh_m = MN_ALS_ENH_MODE_1;
	mn29_sensor.als.als_aenh_l = MN_ALS_ENH_MODE_1;
	mn29_sensor.als.als_ag_h =  MN_GAIN_HIGH;
	mn29_sensor.als.als_ag_m =  MN_GAIN_MID;
	mn29_sensor.als.als_ag_l =  MN_GAIN_LOW;
	mn29_sensor.als.als_acycle_h = MN_CYCLE_2;
	mn29_sensor.als.als_acycle_m = MN_CYCLE_4;
	mn29_sensor.als.als_acycle_l = MN_CYCLE_16;
	mn29_sensor.als.als_ag_h2m_thd = 60000;
	mn29_sensor.als.als_ag_m2h_thd = 3000;
	mn29_sensor.als.als_ag_m2l_thd = 60000;
	mn29_sensor.als.als_ag_l2m_thd = 1000;

	mn29_sensor.als.als_factor = g_rear_cali_data->rear_als_factor;
	//ALS AG THD
	mn29xxx_als_ag_calc_new_thd();
#if ALS_MAX_LUX_PATCH
	als_max_intt = MN_ALS_INTT_16;
	als_max_cycle = MN_CYCLE_64;
	als_max_gain = MN_GAIN_LOW;
	als_l2ll_lux_thd = 14500; //65535*0.3=19660, so 19xxx
	als_ll2l_lux_thd = 12000;
	if (mn29_sensor.als.als_ag_l == MN_GAIN_LOW) {
	   lpc_gain = mn_als_intt_value[mn29_sensor.als.als_aintt_l>>2] / mn_als_intt_value[als_max_intt>>2];
	} else if (mn29_sensor.als.als_ag_l == MN_GAIN_MID) {
	   lpc_gain = (8*mn_als_intt_value[mn29_sensor.als.als_aintt_l>>2]) / mn_als_intt_value[als_max_intt>>2];
	} else if (mn29_sensor.als.als_ag_l == MN_GAIN_HIGH) {
	   lpc_gain = (64*mn_als_intt_value[mn29_sensor.als.als_aintt_l>>2]) / mn_als_intt_value[als_max_intt>>2];
	} else {
	   lpc_gain = 1;
	}
	lpc_gain = lpc_gain/2; //20210427
	APS_LOGI("[mn29xxx_initial_global_variable]: new_lpc_gain=%d \n", lpc_gain);//20210427
#endif //ALS_MAX_LUX_PATCH-
	offset_gain = 1800;
	scale_gain = 800;
	ret = mn29xxx_write_global_variable(client);
	return ret;
}

static int mn29xxx_update_mode(struct i2c_client *client)
{
	int ret = -1;
	uint8_t tx_buf[1] = {0};
	//APS_LOGI("mn29xxx_update_mode: enable_als=%d\n", mn29_sensor.als.als_enable);
	//**** mode selection ****
	if(mn29_sensor.als.als_enable)
		mn29_sensor.mode = MN_MODE_ALS;
	else
		mn29_sensor.mode = MN_MODE_IDLE;
	tx_buf[0] = (MN_POWER_OFF | MN_RESETN_RESET);
	mn29xxx_i2c_write_block( client, DEVREG_RESET, tx_buf, 1);
	tx_buf[0] = (mn29_sensor.wait | mn29_sensor.mode);
	ret = mn29xxx_i2c_write_block( client, DEVREG_ENABLE, tx_buf, 1);
	if(mn29_sensor.mode != MN_MODE_IDLE)
	{
		tx_buf[0] = (MN_POWER_ON | MN_RESETN_RUN);
		mn29xxx_i2c_write_block( client, DEVREG_RESET, tx_buf, 1);
	}
	return ret;
}

static void lsrc_raw_convert_to_adc(uint32_t ch0, uint32_t ch1, uint16_t *new_raw)
{
	uint32_t als_offset=0, als_scale=0;
	uint16_t nor_raw=0;

	if(ch1 > 0)
	{
		lsrc_ratio = ch0*1000 / ch1;
		als_offset = (ch0 * ch0) / (ch0+ch1) * offset_gain / 1000;
		als_scale = (1000-scale_gain)*ch1 / 1000;
		///APS_LOGI("[lsrc_raw_convert_to_adc]: als_offset=%lu , als_scale=%lu \r\n", als_offset, als_scale);
		if( als_offset < als_scale )
			nor_raw = ch1 - als_offset;
		else
			nor_raw = ch1 * scale_gain / 1000;
	}
	else
	{
		nor_raw = ch1;
	}
	lsrc_raw = nor_raw;
	lsrc_als_offset = als_offset;

	*new_raw = nor_raw;

	///APS_LOGI("[lsrc_raw_convert_to_adc]: ch0=%lu, ch1=%lu, nor_raw=%u \r\n", ch0, ch1, nor_raw);
}

static void lux_convert_to_lux(uint32_t raw, uint32_t lpc, uint32_t *new_lux)
{
	uint32_t lux = 0, new_lpc;

	new_lpc = mn29_sensor.als.factory.lux_per_lux * lpc / 1000;
	lux =  new_lpc * raw;
	*new_lux = lux;

	///APS_LOGI("[lux_convert_to_lux]: lux_per_lux=%d, new_lpc=%lu, lux=%lu \n", mn29_sensor.als.factory.lux_per_lux, new_lpc, lux/1000);
}

static bool mn29xxx_als_dyn_detect(struct i2c_client *client, uint16_t als_ch1, uint16_t *new_ch1)
{
	bool change_flag=false;
	uint8_t tx_buf[1] = {0};
	///APS_LOGI("[mn29xxx_als_dyn_detect]: dyn_intt_idx=%d, dyn_intt_value=%d, dyn_intt_gain=0x%x \n", dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx], als_dynamic_intt_gain[dynamic_intt_idx]);
	///APS_LOGI("[mn29xxx_als_dyn_detect]: dyn_intt_cycle=0x%x, dyn_intt_enh=0x%x \n", als_dynamic_intt_cycle[dynamic_intt_idx], (als_dynamic_intt_enh[dynamic_intt_idx]>>6));

	if (mn29_sensor.als.compare_high != mn29_sensor.als.compare_low) {
		tx_buf[0] = (MN_CMP_RESET | MN_LOCK);
		mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, tx_buf, 1 );
		tx_buf[0] = (MN_CMP_RUN | MN_UN_LOCK);
		mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, tx_buf, 1 );
	}
	if (als_ch1 > dynamic_intt_high_thr || (dynamic_intt_idx!=(als_dynamic_intt_intt_num-1) && (mn29_sensor.als.sat_ctia || mn29_sensor.als.sat || mn29_sensor.als.sat_1)))	{
		if (dynamic_intt_idx != (als_dynamic_intt_intt_num - 1)) {
			change_flag = true;
			*new_ch1 = dynamic_intt_high_thr;
			dynamic_intt_idx++;
			if (dynamic_intt_idx >= (als_dynamic_intt_intt_num - 1)){
				dynamic_intt_idx = (als_dynamic_intt_intt_num - 1);
			}
			APS_LOGI(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \n", dynamic_intt_idx, als_ch1);
		}
	} else if(als_ch1 < dynamic_intt_low_thr) {
		if (dynamic_intt_idx != 0) {
			change_flag = true;
			*new_ch1 = dynamic_intt_low_thr;
			dynamic_intt_idx--;
			if (dynamic_intt_idx <= 0) {
				dynamic_intt_idx = 0;
			}
			APS_LOGI(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \n", dynamic_intt_idx, als_ch1);
		}
	} else {
		*new_ch1 = als_ch1;
	}

	if (change_flag == true) {
		APS_LOGI("[mn29xxx_als_dyn_detect]: ALS_DYN_INTT:Change Setting \n");
		mn29_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
		mn29_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
		mn29_sensor.als.als_rs = als_dynamic_intt_rs[dynamic_intt_idx];
		mn29_sensor.als.enh_mode = als_dynamic_intt_enh[dynamic_intt_idx];
		mn29_sensor.als.cycle = als_dynamic_intt_cycle[dynamic_intt_idx];
		dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
		dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
		mn29xxx_als_dyn_cfg(client, mn29_sensor.als.cycle);
	}

	return change_flag;
}
/*----------------------------------------------------------------------------*/
#if ALS_MAX_LUX_PATCH //ALS_MAX_LUX_PATCH+
static void mn29xxx_max_lux_change_cfg(struct i2c_client *client, bool change_max_flag)
{
	//APS_LOGI("[mn29xxx_max_lux_change_cfg]: change_max_flag=%d \r\n", change_max_flag);
	if (change_max_flag) {
		mn29_sensor.als.integration_time = als_max_intt;
		mn29_sensor.als.gain = als_max_gain;
		mn29_sensor.als.cycle = als_max_cycle;
	} else {
		//recover normal
		mn29_sensor.als.integration_time = mn29_sensor.als.als_aintt_l;
		mn29_sensor.als.gain = mn29_sensor.als.als_ag_l;
		mn29_sensor.als.cycle = mn29_sensor.als.als_acycle_l;
	}
	mn29xxx_als_dyn_cfg(client, mn29_sensor.als.cycle);
}
#endif //ALS_MAX_LUX_PATCH-
#if ALS_MAX_LUX_LINEARLY
static void mn29xxx_max_lux_linearly_algr(uint32_t orig_lux, uint32_t *new_lux)
{
	uint32_t low_ratio=0, lux;

	if (orig_lux > als_l2ll_lux_thd) {
		if (orig_lux > MAX_LUX_H_THD) {
			low_ratio = MAX_LUX_RATIO;
		} else {
			low_ratio = 1000 + ( (MAX_LUX_RATIO-1000) * (orig_lux-als_l2ll_lux_thd) / (MAX_LUX_H_THD-als_l2ll_lux_thd) );
		}

	} else {
		low_ratio = 1000;
	}
	lux = orig_lux * low_ratio / 1000;
	*new_lux = lux;
	APS_LOGI("[mn29xxx_max_lux_linearly_algr]: low_ratio=%lu, lux=%lu \r\n", low_ratio, lux);
}
#endif

static uint32_t mn29xxx_get_als_value(struct i2c_client *client, uint16_t als_ch0, uint16_t als_ch1)
{
	uint32_t lux = 0;
	uint16_t ag_ch1=0;
#if ALS_MAX_LUX_LINEARLY
	uint32_t new_lux = 0;
#endif
	switch(mn29_sensor.als.report_type)
	{
		case CMC_BIT_RAW:
			return als_ch1;
		break;
		case CMC_BIT_PRE_COUNT:
			lsrc_raw_convert_to_adc(als_ch0, als_ch1, &lsrc_raw);
			lsrc_lux = mn29_sensor.als.factory.lux_per_count * lsrc_raw / 1000;
			lux_convert_to_lux(lsrc_raw, mn29_sensor.als.factory.lux_per_count, &lux);
			return (lux / 1000);
		break;
		case CMC_BIT_DYN_INT:
#if ALS_MAX_LUX_PATCH //ALS_MAX_LUX_PATCH+
			if (als_max_lux_flag == false) {
#endif //ALS_MAX_LUX_PATCH-
				mn29_sensor.als.als_ag_change_flag = mn29xxx_als_dyn_detect(client, als_ch1, &ag_ch1);
				lsrc_raw_convert_to_adc(als_ch0, ag_ch1, &lsrc_raw);
				lsrc_lux = mn29_sensor.als.factory.lux_per_count * lsrc_raw / 1000;
				lux_convert_to_lux(lsrc_raw, mn29_sensor.als.factory.lux_per_count, &lux);
#if ALS_MAX_LUX_PATCH //ALS_MAX_LUX_PATCH+
				if ((lux/1000) > als_l2ll_lux_thd) {
					als_max_lux_flag = true;
					mn29_sensor.als.als_ag_change_flag = true;
					lux = als_l2ll_lux_thd*1000;
					mn29xxx_max_lux_change_cfg(client, true);
				} else {
					als_max_lux_flag = false;
				}
			} else {
				uint32_t als_new_lpc = 0;
				als_new_lpc = mn29_sensor.als.factory.lux_per_count * lpc_gain;
				lsrc_raw_convert_to_adc(als_ch0, als_ch1, &lsrc_raw);
				lsrc_lux = als_new_lpc * lsrc_raw / 1000;
				lux_convert_to_lux(lsrc_raw, als_new_lpc, &lux);
				if ((lux/1000) <= als_ll2l_lux_thd) {
					als_max_lux_flag = false;
					mn29_sensor.als.als_ag_change_flag = true;
					lux = als_ll2l_lux_thd*1000;
					mn29xxx_max_lux_change_cfg(client, false);
				} else {
					als_max_lux_flag = true;
					mn29_sensor.als.als_ag_change_flag = false;
				}
			}
#endif //ALS_MAX_LUX_PATCH-
#if ALS_MAX_LUX_LINEARLY
			mn29xxx_max_lux_linearly_algr( (lux/1000), &new_lux );
			return new_lux;
#else
			return (lux / 1000);
#endif
		break;
	}

	return 0;
}

static int32_t mn29xxx_read_als(struct i2c_client *client)
{
	uint8_t buf[7];
	uint16_t real_ch0;

	mn29xxx_i2c_read_block(client, DEVREG_ALS_STATUS, buf, 7);
	mn29_sensor.als.sat_ctia = (buf[0] & 0x80);
	mn29_sensor.als.sat_1 = (buf[0] & 0x40);
	mn29_sensor.als.sat = (buf[0] & 0x20);
	mn29_sensor.als.compare_high = (buf[0] & 0x10);
	mn29_sensor.als.compare_low = (buf[0] & 0x08);
	mn29_sensor.als.interrupt_flag = (buf[0] & 0x04);
	mn29_sensor.als.compare_reset = (buf[0] & 0x02);
	mn29_sensor.als.lock = (buf[0] & 0x01);
	real_ch0 = (buf[2]<<8) | buf[1];
	mn29_sensor.als.data.channel[1] = (buf[4]<<8) | buf[3];
	mn29_sensor.als.data.channel[2] = (buf[6]<<8) | buf[5];
	if (real_ch0 > mn29_sensor.als.data.channel[2]) {
		mn29_sensor.als.data.channel[0] = real_ch0 - mn29_sensor.als.data.channel[2];
	} else {
		mn29_sensor.als.data.channel[0] = 0;
	}

	debug_cnt++;
	if (debug_cnt > 10000) {
		debug_cnt = 0;
		APS_LOGI("mn29xxx_read_als: als_status=0x%x, ch0=%d, ch1=%d, ch2=%d \n", buf[0], mn29_sensor.als.data.channel[0], mn29_sensor.als.data.channel[1], mn29_sensor.als.data.channel[2]);
	}

	if (mn29_sensor.als.compare_high == mn29_sensor.als.compare_low) {
		APS_LOGI("[mn29xxx_read_als]: als don't ready \n");
		return -1;
	}

	return mn29_sensor.als.data.channel[1];
}
#if 0
static void mn29xxx_als_dyn_first_report(struct i2c_client *client)
{
	uint32_t i=0, lux;
	uint8_t buf[1];

	mn29xxx_als_dyn_cfg( client, MN_CYCLE_1 );
	buf[0] = (MN_POWER_OFF | MN_RESETN_RESET);
	mn29xxx_i2c_write_block( client, DEVREG_RESET, buf, 1 );
	buf[0] = MN_MODE_ALS;
	mn29xxx_i2c_write_block( client, DEVREG_ENABLE, buf, 1 );
	buf[0] = (MN_POWER_ON | MN_RESETN_RUN);
	mn29xxx_i2c_write_block( client, DEVREG_RESET, buf, 1 );
	do {
		if(dynamic_intt_idx == 0)
			mdelay(60);
		else
			mdelay(30);
		mn29xxx_read_als(client);
		lux = mn29xxx_get_als_value(client, mn29_sensor.als.data.channel[0], mn29_sensor.als.data.channel[1]);
		if(mn29_sensor.als.als_ag_change_flag)
		    mn29xxx_als_dyn_cfg( client, MN_CYCLE_1 );
		APS_LOGI("[mn_sensor_als_dyn_first_report]: i=%d, als_ch1=%u, lux=%u ", i, mn29_sensor.als.data.channel[1], lux);
		i++;
	} while (mn29_sensor.als.als_ag_change_flag && (i < als_dynamic_intt_intt_num));
}
#endif
/****************************************************************************/
static int mn29xxx_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	uint8_t chip_id = 0;
	uint8_t read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		mn29xxx_i2c_read_block(client, DEVREG_REV_ID,
				   &chip_id, 1);

		if (chip_id != MN29xxx_ID) {
			mdelay(1);
			pr_err("%s fail(0x%2x).\n",
				__func__, chip_id);
		} else {
			err = 0;
			pr_info("%s success(0x%2x).\n",
				__func__, chip_id);
			break;
		}
	}
	return err;
}

static int mn29xxx_init_device(struct i2c_client *client)
{
	int err = -1;

	err = mn29xxx_initial_global_variable(client);
	if (err < 0)
	    pr_err("%s fail\n", __func__);
	return err;
}

#if REG_MONITOR
static bool mn29_sensor_check_reg(struct i2c_client *client)
{
	uint8_t buf[2];
	mn29xxx_i2c_read_block(client, DEVREG_INT_CTRL, buf, 1);
	//APS_LOGI("REG0x%x=0x%x \r\n", DEVREG_INT_CTRL, buf[0] );
	if (buf[0] != (mn29_sensor.interrupt_control | mn29_sensor.als.als_dark_pixel)) {
		APS_LOGI("reset all setting(0x%x)\r\n", (mn29_sensor.interrupt_control | mn29_sensor.als.als_dark_pixel) );
		mn29xxx_write_global_variable(client);
		buf[0] = (MN_POWER_ON | MN_RESETN_RUN);
		mn29xxx_i2c_write_block( client, DEVREG_RESET, buf, 1);
		APS_LOGI("PS CMP RESET/RUN \r\n");
		return false;
	}
	return true;
}
#endif

static int mn29xxx_sample(struct hf_device *hfdev)
{
	struct i2c_client *client;
	struct mn29xxx_device *driver_dev;
	struct hf_manager *manager;
	struct hf_manager_event event;
	int64_t current_time;
	s32 value = 0;
	int err = 0;

	if (!hfdev) {
		APS_LOGE("mn29xxx sample failed:invalid hfdev\n");
		return -1;
	}
	client = hf_device_get_private_data(hfdev);
	driver_dev = i2c_get_clientdata(client);
	manager = driver_dev->hf_dev.manager;

	err = mn29xxx_read_als(client);
#if REG_MONITOR
	if (mn29_sensor_check_reg(client) == false) {
		APS_LOGI("recover settting .................... \n");
		return -1;
	}
#endif
	if (err < 0) {
		APS_LOGE(" mn29xxx sample failed\n");
		return err;
	}
	value = (s32)mn29xxx_get_als_value(client, mn29_sensor.als.data.channel[0], mn29_sensor.als.data.channel[1]); //calculate lux
	if ((mn29_sensor.als.als_first_flag==true) && (mn29_sensor.als.als_ag_change_flag==true)) {
		APS_LOGI("[mn29xxx_sample]: als_ag don't ready als_first_flag=%d,als_ag_change_flag=%d \r\n",
				mn29_sensor.als.als_first_flag,mn29_sensor.als.als_ag_change_flag);
		return -1;
	}
	mn29_sensor.als.als_first_flag = false;
	///APS_LOGE("before cal mn29xxx_read_als: als_value=%d,als_factor= %d\n", value,mn29_sensor.als.als_factor);

	current_time = ktime_get_boot_ns();
	if (atomic_read(&driver_dev->raw_enable)) {
		memset(&event, 0, sizeof(struct hf_manager_event));
		event.timestamp = current_time;
		event.sensor_type = SEN_TYPE;
		event.accurancy = SENSOR_ACCURANCY_HIGH;
		event.action = RAW_ACTION;
		event.word[0] = value*100;
		manager->report(manager, &event);
	}

	if (mn29_sensor.als.als_factor && mn29_sensor.als.als_factor != 1000) {
		value = value * mn29_sensor.als.als_factor/10;
	} else {
		mutex_lock(&gdriver_dev->para_lock);
		sensor_read_oplus_custom_rear(g_rear_cali_data);
		if (g_rear_cali_data->rear_als_factor == 0) {
			mn29_sensor.als.als_factor = 1000;
		} else {
			mn29_sensor.als.als_factor = g_rear_cali_data->rear_als_factor;
		}
		value = value * mn29_sensor.als.als_factor/10;
		mutex_unlock(&gdriver_dev->para_lock);
	}

	memset(&event, 0, sizeof(struct hf_manager_event));
	event.timestamp = current_time;
	event.sensor_type = SEN_TYPE;
	event.accurancy = SENSOR_ACCURANCY_HIGH;
	event.action = DATA_ACTION;
	event.word[0] = value;

	manager->report(manager, &event);
	manager->complete(manager);
	APS_LOGE("final report mn29xxx_read_als: als_value=%d, als_factor= %d\n", value, mn29_sensor.als.als_factor);

	return 0;
}

static int mn29xxx_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	struct i2c_client *client;
	uint8_t tx_buf[1] = {0};
	int ret = -1;
	client = hf_device_get_private_data(hfdev);

	APS_LOGE("[mn29xxx_enable]: als enable=%d \n", en);
	if (en) {
#if REG_MONITOR
		mn29xxx_write_global_variable(client);
#endif
		mn29_sensor.als.als_enable = true;
		mn29_sensor.als.als_first_flag = true;
		mn29xxx_set_lsensor_intr_threshold( client, L_SENSOR_LTHD_TRIGER, L_SENSOR_HTHD );
		tx_buf[0] = (MN_POWER_ON | MN_RESETN_RUN);
		mn29xxx_i2c_write_block( client, DEVREG_RESET, tx_buf, 1 );
		tx_buf[0] = (MN_CMP_RESET | MN_LOCK);
		mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, tx_buf, 1 );
		tx_buf[0] = (MN_CMP_RUN | MN_UN_LOCK);
		mn29xxx_i2c_write_block( client, DEVREG_ALS_STATUS, tx_buf, 1 );

		if (mn29_sensor.als.report_type == CMC_BIT_DYN_INT) {
#if ALS_MAX_LUX_PATCH
			als_max_lux_flag = false;
#endif
			dynamic_intt_idx = dynamic_intt_init_idx;
			mn29_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
			mn29_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
			mn29_sensor.als.als_rs = als_dynamic_intt_rs[dynamic_intt_idx];
			mn29_sensor.als.cycle = als_dynamic_intt_cycle[dynamic_intt_idx];
			mn29_sensor.als.enh_mode = als_dynamic_intt_enh[dynamic_intt_idx];
			dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
			dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
#if 0
			if(mn29_sensor.als.polling_mode)
				mn29xxx_als_dyn_first_report(client);
#endif
			mn29xxx_als_dyn_cfg(client, mn29_sensor.als.cycle);
		}

	} else {
		mn29_sensor.als.als_enable = false;
		mn29_sensor.als.als_first_flag = false;
	}
	ret = mn29xxx_update_mode(client);

	APS_LOGE("[mn29xxx_enable]: als ret=%d \n", ret);
	return ret;
}

static int mn29xxx_batch(struct hf_device *hfdev, int sensor_type,
		int64_t delay, int64_t latency)
{
	APS_LOGE("%s id:%d delay:%lld latency:%lld\n", __func__, sensor_type,
			delay, latency);
	return 0;
}

static int mn29xxx_flush(struct hf_device *hfdev, int sensor_type)
{
	struct i2c_client *client;
	struct mn29xxx_device *driver_dev;
	struct hf_manager *manager;
	struct hf_manager_event event;
	int64_t current_time;

	if (!hfdev) {
		APS_LOGE("mn29xxx sample failed:invalid hfdev\n");
		return -1;
	}
	client = hf_device_get_private_data(hfdev);
	driver_dev = i2c_get_clientdata(client);
	manager = driver_dev->hf_dev.manager;
	current_time = ktime_get_boot_ns();

	memset(&event, 0, sizeof(struct hf_manager_event));
	event.timestamp = current_time;
	event.sensor_type = SEN_TYPE;
	event.accurancy = SENSOR_ACCURANCY_HIGH;
	event.action = FLUSH_ACTION;
	event.word[0] = 0;
	if (mn29_sensor.als.als_enable) {
		manager->report(manager, &event);
		manager->complete(manager);
		APS_LOGE("%s flush data id:%d\n", __func__, sensor_type);
		return 0;
	}
	return 0;
}

#if 0
#include <linux/kthread.h>
#include <linux/sched.h>

static int stopthread  = 0;

int threadTask(void *arg)
{
	while (1) {
		if (stopthread) {
			printk("threadTask: kthread_should_stop\n");
			break;
		}
		mdelay(1000);
		mn29xxx_sample(&gdriver_dev->hf_dev);
	}
	return 0;
}

static int  init_kernel_Thread(void)
{
	static struct task_struct *test_TaskStruct;
	test_TaskStruct = kthread_create(threadTask, NULL, "KernelThead");
	if (IS_ERR(test_TaskStruct)) {
		printk("kthread_create error\n");
	} else {
		wake_up_process(test_TaskStruct);
	}
	return 0;
}
#endif
int mn29xxx_raw_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	struct i2c_client *client = hf_device_get_private_data(hfdev);
	struct mn29xxx_device *driver_dev = i2c_get_clientdata(client);
	APS_LOGE("[mn29xxx_raw_enable]: als enable=%d \n", en);

	atomic_set(&driver_dev->raw_enable, en);
#if 0
	mn29xxx_enable(hfdev,sensor_type,en);
	if (en) {
		init_kernel_Thread();
	} else {
		stopthread = 1;
	}
#endif
	return 0;
}

int mn29xxx_calibration(struct hf_device *hfdev, int sensor_type){
	APS_LOGE("[mn29xxx_calibration] \n");
	return 0;
}
int mn29xxx_config_cali(struct hf_device *hfdev,int sensor_type, void *data, uint8_t length){
	mutex_lock(&gdriver_dev->para_lock);
	printk(" ---- %s[%d] data = %d----\n", __FUNCTION__, __LINE__,*(int*)data);
	mn29_sensor.als.als_factor = *(int*)data;
	mutex_unlock(&gdriver_dev->para_lock);
	return 0;
}

static int mn29xxx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct mn29xxx_device *driver_dev = NULL;

	err = mn29xxx_check_chip_id(client);
	if (err < 0) {
		pr_err("mn29xxx chip id mismatch,exit probe!\n");
		goto findHW_fail;
	}

	driver_dev = kzalloc(sizeof(*driver_dev), GFP_KERNEL);
	if (!driver_dev) {
		err = -ENOMEM;
		goto malloc_fail;
	}

	driver_dev->hf_dev.dev_name = LIGHT_SENSOR_NAME;
	driver_dev->hf_dev.device_poll = HF_DEVICE_IO_POLLING;
	driver_dev->hf_dev.device_bus = HF_DEVICE_IO_SYNC;
	driver_dev->hf_dev.support_list = mn29xxx_sensor_info;
	driver_dev->hf_dev.support_size = ARRAY_SIZE(mn29xxx_sensor_info);
	driver_dev->hf_dev.sample = mn29xxx_sample;
	driver_dev->hf_dev.enable = mn29xxx_enable;
	driver_dev->hf_dev.batch = mn29xxx_batch;
	driver_dev->hf_dev.rawdata = mn29xxx_raw_enable;
	driver_dev->hf_dev.calibration = mn29xxx_calibration;
	driver_dev->hf_dev.config_cali = mn29xxx_config_cali;
	driver_dev->hf_dev.flush = mn29xxx_flush;
	gdriver_dev = driver_dev;

	err = hf_manager_create(&driver_dev->hf_dev);

	mutex_init(&gdriver_dev->para_lock);
	if (err < 0) {
		pr_err("%s hf_manager_create fail\n", __func__);
		err = -1;
		goto create_manager_fail;
	}

	i2c_set_clientdata(client, driver_dev);
	hf_device_set_private_data(&driver_dev->hf_dev, client);

	g_rear_cali_data = kzalloc(sizeof(struct cali_data), GFP_KERNEL);
	sensor_read_oplus_custom_rear(g_rear_cali_data);
	pr_info("rear_als_factor=%d\n", g_rear_cali_data->rear_als_factor);

	err = mn29xxx_init_device(client);
	if (err < 0) {
		pr_err("%s fail\n", __func__);
		goto init_fail;
	}

	pr_info("%s success!\n", __func__);
	return 0;

init_fail:
create_manager_fail:
	kfree(driver_dev);
malloc_fail:
findHW_fail:
	pr_err("%s fail!\n", __func__);
	return err;
}

static int mn29xxx_remove(struct i2c_client *client)
{
	struct mn29xxx_device *driver_dev = i2c_get_clientdata(client);

	hf_manager_destroy(driver_dev->hf_dev.manager);
	kfree(driver_dev);
	return 0;
}

static const struct i2c_device_id mn29xxx_id[] = {
	{LIGHT_SENSOR_NAME, 0},
	{},
};

static const struct of_device_id mn29xxx_of_match[] = {
	{.compatible = "mediatek,mn29xxxals"},
	{},
};

static struct i2c_driver mn29xxx_driver = {
	.driver = {
		.name = LIGHT_SENSOR_NAME,
		.bus = &i2c_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = mn29xxx_of_match,
	},
	.probe = mn29xxx_probe,
	.remove = mn29xxx_remove,
	.id_table = mn29xxx_id
};

#ifdef OPLUS_FEATURE_SENSOR
static int __init mn29xxx_driver_init(void)
{
	return i2c_add_driver(&mn29xxx_driver);
}

static void __exit mn29xxx_driver_exit(void)
{
}
late_initcall(mn29xxx_driver_init);
module_exit(mn29xxx_driver_exit);
#endif


MODULE_DESCRIPTION("mnx29xxx driver");
MODULE_AUTHOR("Mediatek");
MODULE_LICENSE("GPL");
