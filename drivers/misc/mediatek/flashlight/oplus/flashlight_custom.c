#include "flashlight_custom.h"
#include "mt6360_custom.h"

#define ONLY_FIRST_FLASHLIGHT  2
#define ONLY_SECOND_FLASHLIGHT  3

void Oplus2Channel_set_timeout_custom(unsigned int* cur_timeout, int channel, int timeout, int *ch1_timeout, int *ch2_timeout)
{
	if (!cur_timeout || !ch1_timeout || !ch2_timeout)
		pr_info("[%s] no cur_timeout || ch1_timeout || ch2_timeout!!!\n", __func__);

	if (channel != 0 && channel != 1)
		pr_info("[%s] channel error!!!  channel:%d\n", __func__, channel);

	//set default timeout
	*ch1_timeout = cur_timeout[0];
	*ch2_timeout = cur_timeout[1];

	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)
		|| is_project(21127) || is_project(21305)) {
		*ch1_timeout = timeout;
		*ch2_timeout = timeout;
		pr_info("[%s] set ch1 ch2 timeout = %d.\n", __func__, timeout);
	} else {
		if (channel == 0)
			*ch1_timeout = timeout;
		else
			*ch2_timeout = timeout;
		pr_info("[%s] set ch%d timeout = %d.\n", __func__, channel, timeout);
	}
}

void Oplus2Channel_set_duty_custom(int cur_ch1_duty, int cur_ch2_duty, int channel, int duty, int *ch1_duty, int *ch2_duty)
{
	if (!ch1_duty || !ch2_duty)
		pr_info("[%s] no ch1_duty || ch2_duty!!!\n", __func__);

	if (channel != 0 && channel != 1)
		pr_info("[%s] channel error!!!  channel:%d\n", __func__, channel);

	//set default duty
	*ch1_duty = cur_ch1_duty;
	*ch2_duty = cur_ch2_duty;

	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)
		|| is_project(21127) || is_project(21305)) {
		*ch1_duty = duty;
		*ch2_duty = duty;
		pr_info("[%s] set ch1 ch2 duty = %d.\n", __func__, duty);
	} else {
		if (channel == 0)
			*ch1_duty = duty;
		else
			*ch2_duty = duty;
		pr_info("[%s] set ch%d duty = %d.\n", __func__, channel, duty);
	}
}

void Oplus2Channel_set_onoff_custom(int channel, int param, int *ch1_status, int *ch2_status, int *need_op_ch_number) 
{
	if (!ch1_status || !ch2_status)
		pr_info("[%s] ch1_status || ch2_status!!!\n", __func__);

	if (channel != 0 && channel != 1) {
		pr_info("[%s] channel error!!! channel: %d\n", __func__, channel);
		return;
	}

	// set default
	*need_op_ch_number = -1;
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)
		|| is_project(21127) || is_project(21305)) {
		*need_op_ch_number = 2;
		if (param == ONLY_FIRST_FLASHLIGHT) {
			*ch1_status = 1;
			*ch2_status = 0;
		} else if (param == ONLY_SECOND_FLASHLIGHT) {
			*ch1_status = 0;
			*ch2_status = 1;
		} else {
			*ch1_status = param;
			*ch2_status = param;
		}
		pr_info("[%s] set ch1 status = %d ch2 status = %d param= %d\n",
			__func__, *ch1_status, *ch2_status, param);
	} else {
		if (channel == 0) {
			*need_op_ch_number = 0;
			*ch1_status = param;
		}else {
			*need_op_ch_number = 1;
 			*ch2_status = param;
		}
		pr_info("[%s] set ch%d status = %d.\n", __func__, channel, param);
	}
}

int Oplusflashlight_set_strobe_timeout_custom(struct flashlight_device* flashlight_dev, int default_timeout)
{
	if (!flashlight_dev)
		return -EINVAL;

	pr_info("Oplusflashlight_set_strobe_timeout_custom");
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		return flashlight_set_strobe_timeout(flashlight_dev,
					MT6360_HW_TIMEOUT_20817, MT6360_HW_TIMEOUT_20817 + 200);
	} else {
		return flashlight_set_strobe_timeout(flashlight_dev,
					default_timeout, default_timeout + 200);
	}
}

int Oplusflashlight_get_scenairo_custom(int decouple)
{
	int ret = 0;

	pr_info("Oplusflashlight_get_scenairo_custom");
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)
		|| is_project(21127) || is_project(21305)) {
		if (decouple) {
			ret = FLASHLIGHT_SCENARIO_FLASHLIGHT | FLASHLIGHT_SCENARIO_DECOUPLE;
		}else {
			ret = FLASHLIGHT_SCENARIO_FLASHLIGHT | FLASHLIGHT_SCENARIO_COUPLE;
		}
	}else {
		if (decouple) {
			ret = FLASHLIGHT_SCENARIO_CAMERA | FLASHLIGHT_SCENARIO_DECOUPLE;
		}else {
			ret = FLASHLIGHT_SCENARIO_CAMERA | FLASHLIGHT_SCENARIO_COUPLE;
		}
	}

	return ret;
}

int Oplus_get_current_duty_custom(const int *mt6360_current, int idx)
{
	pr_info("Oplus_get_current_duty_custom");

	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		return mt6360_current_20817[idx];
	}else {
		return mt6360_current[idx];
	}
}

int Oplus_get_max_duty_torch_custom(int default_max_level)
{
	pr_info("Oplus_get_max_duty_custom");
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		return MT6360_LEVEL_TORCH_20817;
	}else {
		return default_max_level;
	}
}

int Oplus_get_max_duty_num_custom(int default_max_level)
{
	pr_info("Oplus_get_max_duty_custom");
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		return MT6360_LEVEL_NUM_20817;
	}else {
		return default_max_level;
	}
}

int Oplus_get_hw_timeout_custom(int default_timeout)
{
	pr_info("Oplus_get_hw_timeout_custom");
	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		return MT6360_HW_TIMEOUT_20817;
	}else {
		return default_timeout;
	}
}

void Oplus_get_torch_and_strobe_level_custom(int default_torch_level, int default_strobe_level, int idx, int *torch_level, int *strobe_level)
{
	pr_info("Oplus_get_torch_and_strobe_level_custom");
	if (!torch_level || !strobe_level) {
		pr_info("[%s] torch_level || strobe_level!!!\n", __func__);
		return;
	}

	if (is_project(20817) || is_project(20827)
		|| is_project(20831) || is_project(21881) || is_project(21882)) {
		*torch_level = mt6360_torch_level_20817[idx];
		*strobe_level = mt6360_strobe_level_20817[idx];
	} else if (is_project(21015) || is_project(21217)){
		*torch_level = mt6360_torch_level_21015[idx];
		*strobe_level = mt6360_strobe_level_21015[idx];
	} else {
		*torch_level = default_torch_level;
		*strobe_level = default_strobe_level;
	}

}
