/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _OPLUS_WRAP_H_
#define _OPLUS_WRAP_H_

#include <linux/workqueue.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
#include <linux/wakelock.h>
#endif
#include <linux/timer.h>
#include <linux/slab.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/firmware.h>

#define OPLUS_WRAP_MCU_HWID_UNKNOW   -1
#define OPLUS_WRAP_MCU_HWID_STM8S	0
#define OPLUS_WRAP_MCU_HWID_N76E		1
#define OPLUS_WRAP_ASIC_HWID_RK826	2
#define OPLUS_WRAP_ASIC_HWID_OP10	3
#define OPLUS_WRAP_ASIC_HWID_RT5125   4
#define OPLUS_WRAP_ASIC_HWID_NON_EXIST 5

enum {
	WRAP_CHARGER_MODE,
	HEADPHONE_MODE,
	NORMAL_CHARGER_MODE,
};

enum {
	FW_ERROR_DATA_MODE,
	FW_NO_CHECK_MODE,
	FW_CHECK_MODE,
};

enum {
	WRAP_MAX_CURRENT_NO_LIMIT,
	WRAP_MAX_CURRENT_LIMIT_2A,
	WRAP_MAX_CURRENT_LIMIT_OTHER,
};
enum {
	FASTCHG_CHARGER_TYPE_UNKOWN,
	PORTABLE_PIKAQIU_1 = 0x31,
	PORTABLE_PIKAQIU_2 = 0x32,
	PORTABLE_50W = 0x33,
	PORTABLE_20W_1 = 0X34,
	PORTABLE_20W_2 = 0x35,
	PORTABLE_20W_3 = 0x36,
};

enum e_fastchg_power{
	FASTCHG_POWER_UNKOWN,
	FASTCHG_POWER_5V4A_5V6A_WRAP,
	FASTCHG_POWER_11V3A_FLASHCHARGER,
	FASTCHG_POWER_10V5A_SINGLE_BAT_SWRAP,
	FASTCHG_POWER_10V5A_TWO_BAT_SWRAP,
	FASTCHG_POWER_10V6P5A_TWO_BAT_SWRAP,
	FASTCHG_POWER_OTHER,
};

enum {
	BAT_TEMP_NATURAL = 0,
	BAT_TEMP_HIGH0,
	BAT_TEMP_HIGH1,
	BAT_TEMP_HIGH2,
	BAT_TEMP_HIGH3,
	BAT_TEMP_HIGH4,
	BAT_TEMP_HIGH5,
	BAT_TEMP_LOW0,
	BAT_TEMP_LOW1,
	BAT_TEMP_LOW2,
	BAT_TEMP_LITTLE_COOL,
	BAT_TEMP_COOL,
	BAT_TEMP_NORMAL_LOW,
	BAT_TEMP_NORMAL_HIGH,
	BAT_TEMP_LITTLE_COLD,
	BAT_TEMP_EXIT,
};

enum {
	FASTCHG_TEMP_RANGE_INIT = 0,
	FASTCHG_TEMP_RANGE_LITTLE_COLD,/*0 ~ 5*/
	FASTCHG_TEMP_RANGE_COOL,/*5 ~ 12*/
	FASTCHG_TEMP_RANGE_LITTLE_COOL, /*12 `16*/
	FASTCHG_TEMP_RANGE_NORMAL_LOW, /*16-25*/
	FASTCHG_TEMP_RANGE_NORMAL_HIGH, /*25-43*/
};


struct wrap_gpio_control {
	int switch1_gpio;
	int switch1_ctr1_gpio;
	int switch2_gpio;
	int switch3_gpio;
	int reset_gpio;
	int clock_gpio;
	int data_gpio;
	int wrap_mcu_id_gpio;
	int wrap_asic_id_gpio;
	int data_irq;
	struct pinctrl *pinctrl;

	struct pinctrl_state *gpio_switch1_act_switch2_act;
	struct pinctrl_state *gpio_switch1_sleep_switch2_sleep;
	struct pinctrl_state *gpio_switch1_act_switch2_sleep;
	struct pinctrl_state *gpio_switch1_sleep_switch2_act;
	struct pinctrl_state *gpio_switch1_ctr1_act;
	struct pinctrl_state *gpio_switch1_ctr1_sleep;

	struct pinctrl_state *gpio_clock_active;
	struct pinctrl_state *gpio_clock_sleep;
	struct pinctrl_state *gpio_data_active;
	struct pinctrl_state *gpio_data_sleep;
	struct pinctrl_state *gpio_reset_active;
	struct pinctrl_state *gpio_reset_sleep;
	struct pinctrl_state *gpio_wrap_mcu_id_default;
	struct pinctrl_state *gpio_wrap_asic_id_active;
	struct pinctrl_state *gpio_wrap_asic_id_sleep;
};

struct oplus_wrap_chip {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_wrap_operations *vops;
	struct wrap_gpio_control wrap_gpio;
	struct delayed_work fw_update_work;
	struct delayed_work fw_update_work_fix;
	struct delayed_work fastchg_work;
	struct delayed_work delay_reset_mcu_work;
	struct delayed_work check_charger_out_work;
	struct work_struct wrap_watchdog_work;
	struct timer_list watchdog;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct wake_lock wrap_wake_lock;
#else
	struct wakeup_source *wrap_ws;
#endif

	struct power_supply *batt_psy;
	int pcb_version;
	bool allow_reading;
	bool fastchg_started;
	bool fastchg_ing;
	bool fastchg_allow;
	bool fastchg_to_normal;
	bool fastchg_to_warm;
	bool fastchg_low_temp_full;
	bool btb_temp_over;
	bool fastchg_dummy_started;
	bool need_to_up;
	bool have_updated;
	bool mcu_update_ing;
	bool mcu_boot_by_gpio;
	const unsigned char *firmware_data;
	unsigned int fw_data_count;
	int fw_mcu_version;
	int fw_data_version;
	int adapter_update_real;
	int adapter_update_report;
	int dpdm_switch_mode;
	bool support_wrap_by_normal_charger_path;
/* Add for wrap batt 4.40*/
	bool batt_type_4400mv;
	bool wrap_fw_check;
	bool support_single_batt_swrap;
	bool wrap_is_platform_gauge;
	int wrap_fw_type;
	int fw_update_flag;
	struct manufacture_info manufacture_info;
	bool wrap_fw_update_newmethod;
	char *fw_path;
	struct mutex pinctrl_mutex;
	int wrap_temp_cur_range;
	int wrap_little_cool_temp;
	int wrap_cool_temp;
	int wrap_little_cold_temp;
	int wrap_normal_low_temp;
	int wrap_little_cool_temp_default;
	int wrap_cool_temp_default;
	int wrap_little_cold_temp_default;
	int wrap_normal_low_temp_default;
	int wrap_low_temp;
	int wrap_high_temp;
	int wrap_low_soc;
	int wrap_high_soc;
	int wrap_cool_bat_volt;
	int wrap_little_cool_bat_volt;
	int wrap_normal_bat_volt;
	int wrap_warm_bat_volt;
	int wrap_cool_bat_suspend_volt;
	int wrap_little_cool_bat_suspend_volt;
	int wrap_normal_bat_suspend_volt;
	int wrap_warm_bat_suspend_volt;
	int wrap_chg_current_now;
	int fast_chg_type;
	bool disable_adapter_output;// 0--wrap adapter output normal,  1--disable wrap adapter output
	int set_wrap_current_limit;///0--no limit;  1--max current limit 2A
	bool wrap_multistep_adjust_current_support;
	int wrap_reply_mcu_bits;
	int wrap_multistep_initial_batt_temp;
	int wrap_strategy_normal_current;
	int wrap_strategy1_batt_high_temp0;
	int wrap_strategy1_batt_high_temp1;
	int wrap_strategy1_batt_high_temp2;
	int wrap_strategy1_batt_low_temp2;
	int wrap_strategy1_batt_low_temp1;
	int wrap_strategy1_batt_low_temp0;
	int wrap_strategy1_high_current0;
	int wrap_strategy1_high_current1;
	int wrap_strategy1_high_current2;
	int wrap_strategy1_low_current2;
	int wrap_strategy1_low_current1;
	int wrap_strategy1_low_current0;
	int wrap_strategy2_batt_up_temp1;
	int wrap_strategy2_batt_up_down_temp2;
	int wrap_strategy2_batt_up_temp3;
	int wrap_strategy2_batt_up_down_temp4;
	int wrap_strategy2_batt_up_temp5;
	int wrap_strategy2_batt_up_temp6;
	int wrap_strategy2_high0_current;
	int wrap_strategy2_high1_current;
	int wrap_strategy2_high2_current;
	int wrap_strategy2_high3_current;
	int fastchg_batt_temp_status;
	int wrap_batt_over_high_temp;
	int wrap_batt_over_low_temp;
	int wrap_over_high_or_low_current;
	int wrap_strategy_change_count;
	int *wrap_current_lvl;
	int wrap_current_lvl_cnt;
	int detach_unexpectly;
	bool disable_real_fast_chg;
	bool reset_adapter;
	bool suspend_charger;
	bool temp_range_init;
	bool w_soc_temp_to_mcu;
	int soc_range;
	bool parse_fw_from_dt;
	int *abnormal_adapter_current;
	int abnormal_adapter_current_cnt;
	int allowed_current_max;
};

#define MAX_FW_NAME_LENGTH	60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    60
struct oplus_wrap_operations {
	int (*fw_update)(struct oplus_wrap_chip *chip);
	int (*fw_check_then_recover)(struct oplus_wrap_chip *chip);
	int (*fw_check_then_recover_fix)(struct oplus_wrap_chip *chip);
	void (*eint_regist)(struct oplus_wrap_chip *chip);
	void (*eint_unregist)(struct oplus_wrap_chip *chip);
	void (*set_data_active)(struct oplus_wrap_chip *chip);
	void (*set_data_sleep)(struct oplus_wrap_chip *chip);
	void (*set_clock_active)(struct oplus_wrap_chip *chip);
	void (*set_clock_sleep)(struct oplus_wrap_chip *chip);
	void (*set_switch_mode)(struct oplus_wrap_chip *chip, int mode);
	int (*get_gpio_ap_data)(struct oplus_wrap_chip *chip);
	int (*read_ap_data)(struct oplus_wrap_chip *chip);
	void (*reply_mcu_data)(struct oplus_wrap_chip *chip, int ret_info, int device_type);
	void (*reply_mcu_data_4bits)(struct oplus_wrap_chip *chip,
		int ret_info, int device_type);
	void (*reset_fastchg_after_usbout)(struct oplus_wrap_chip *chip);
	void (*switch_fast_chg)(struct oplus_wrap_chip *chip);
	void (*reset_mcu)(struct oplus_wrap_chip *chip);
	void (*set_mcu_sleep)(struct oplus_wrap_chip *chip);
	void (*set_wrap_chargerid_switch_val)(struct oplus_wrap_chip *chip, int value);
	bool (*is_power_off_charging)(struct oplus_wrap_chip *chip);
	int (*get_reset_gpio_val)(struct oplus_wrap_chip *chip);
	int (*get_switch_gpio_val)(struct oplus_wrap_chip *chip);
	int (*get_ap_clk_gpio_val)(struct oplus_wrap_chip *chip);
	int (*get_fw_version)(struct oplus_wrap_chip *chip);
	int (*get_clk_gpio_num)(struct oplus_wrap_chip *chip);
	int (*get_data_gpio_num)(struct oplus_wrap_chip *chip);
	void (*update_temperature_soc)(void);
	int (*check_asic_fw_status)(struct oplus_wrap_chip *chip);
};

void oplus_wrap_init(struct oplus_wrap_chip *chip);
void oplus_wrap_shedule_fastchg_work(void);
void oplus_wrap_read_fw_version_init(struct oplus_wrap_chip *chip);
void oplus_wrap_fw_update_work_init(struct oplus_wrap_chip *chip);
bool oplus_wrap_wake_fastchg_work(struct oplus_wrap_chip *chip);
void oplus_wrap_print_log(void);
void oplus_wrap_switch_mode(int mode);
bool oplus_wrap_get_allow_reading(void);
bool oplus_wrap_get_fastchg_started(void);
bool oplus_wrap_get_fastchg_ing(void);
bool oplus_wrap_get_fastchg_allow(void);
void oplus_wrap_set_fastchg_allow(int enable);
bool oplus_wrap_get_fastchg_to_normal(void);
void oplus_wrap_set_fastchg_to_normal_false(void);
bool oplus_wrap_get_fastchg_to_warm(void);
void oplus_wrap_set_fastchg_to_warm_false(void);
void oplus_wrap_set_fastchg_type_unknow(void);
bool oplus_wrap_get_fastchg_low_temp_full(void);
void oplus_wrap_set_fastchg_low_temp_full_false(void);
bool oplus_wrap_get_wrap_multistep_adjust_current_support(void);
bool oplus_wrap_get_fastchg_dummy_started(void);
void oplus_wrap_set_fastchg_dummy_started_false(void);
int oplus_wrap_get_adapter_update_status(void);
int oplus_wrap_get_adapter_update_real_status(void);
bool oplus_wrap_get_btb_temp_over(void);
void oplus_wrap_reset_fastchg_after_usbout(void);
void oplus_wrap_switch_fast_chg(void);
void oplus_wrap_reset_mcu(void);
void oplus_wrap_set_mcu_sleep(void);
void oplus_wrap_set_wrap_chargerid_switch_val(int value);
void oplus_wrap_set_ap_clk_high(void);
int is_wrap_support_single_batt_swrap(void);
int oplus_wrap_get_wrap_switch_val(void);
bool oplus_wrap_check_chip_is_null(void);
void oplus_wrap_battery_update(void);

int oplus_wrap_get_uart_tx(void);
int oplus_wrap_get_uart_rx(void);
void oplus_wrap_uart_init(void);
void oplus_wrap_uart_reset(void);
void oplus_wrap_set_adapter_update_real_status(int real);
void oplus_wrap_set_adapter_update_report_status(int report);
int oplus_wrap_get_fast_chg_type(void);
int oplus_wrap_get_reply_bits(void);
void oplus_wrap_set_disable_adapter_output(bool disable);
void oplus_wrap_set_wrap_max_current_limit(int current_level);
bool oplus_wrap_get_detach_unexpectly(void);
void oplus_wrap_set_detach_unexpectly(bool val);
void oplus_wrap_set_disable_real_fast_chg(bool val);
void oplus_wrap_turn_off_fastchg(void);
int oplus_wrap_get_reply_bits(void);
extern int get_wrap_mcu_type(struct oplus_wrap_chip *chip);
bool opchg_get_mcu_update_state(void);
void oplus_wrap_get_wrap_chip_handle(struct oplus_wrap_chip **chip);
void oplus_wrap_reset_temp_range(struct oplus_wrap_chip *chip);
bool oplus_wrap_get_reset_adapter_st(void);
int oplus_wrap_get_reset_active_status(void);
void oplus_wrap_fw_update_work_plug_in(void);
int oplus_wrap_check_asic_fw_status(void);
int oplus_wrap_get_abnormal_adapter_current_cnt(void);

#endif /* _OPLUS_WRAP_H */
