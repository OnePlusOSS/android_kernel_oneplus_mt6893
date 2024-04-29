// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "gpio.h"
#include <soc/oplus/system/oplus_project.h>

struct GPIO_PINCTRL gpio_pinctrl_list_cam[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_dufu[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif

};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_23031[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_22277[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_23231[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_moss[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
        {"pnd1"},
        {"pnd0"},
        {"rst1"},
        {"rst0"},
        {"ldo_vcama_1"},
        {"ldo_vcama_0"},
        {"ldo_vcamd_1"},
        {"ldo_vcamd_0"},
        {"ldo_vcamio_1"},
        {"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		{"ldo_vcamaf_1"},
		{"ldo_vcamaf_0"},
        {"ldo_vcama1_1"},
        {"ldo_vcama1_0"},
        {"ldo_vcamd1_1"},
        {"ldo_vcamd1_0"},
        {"ldo_vcamois_1"},
        {"ldo_vcamois_0"},
        {"ldo_vcamois1_1"},
        {"ldo_vcamois1_0"},
#endif

};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_lijing[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_lijinga[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_fanli[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_22693[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif

};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_22629[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_23687[
                        GPIO_CTRL_STATE_MAX_NUM_CAM] = {
        /* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamois_1"},
	{"ldo_vcamois_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_blade[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_hawaii[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"},
	{"ldo_vcamd1_1"},
	{"ldo_vcamd1_0"},
	{"ldo_vcamiso_1"},
	{"ldo_vcamiso_0"},
	{"ldo_vcamiso1_1"},
	{"ldo_vcamiso1_0"},
#endif
};

struct GPIO_PINCTRL gpio_pinctrl_list_cam_212a1[
			GPIO_CTRL_STATE_MAX_NUM_CAM] = {
	/* Main */
	{"pnd1"},
	{"pnd0"},
	{"rst1"},
	{"rst0"},
	{"ldo_vcama_1"},
	{"ldo_vcama_0"},
	{"ldo_vcamaf_1"},
	{"ldo_vcamaf_0"},
	{"ldo_vcamd_1"},
	{"ldo_vcamd_0"},
	{"ldo_vcamio_1"},
	{"ldo_vcamio_0"},
	{"ldo_vcama1_1"},
	{"ldo_vcama1_0"}
};
#ifdef MIPI_SWITCH
struct GPIO_PINCTRL gpio_pinctrl_list_switch[
			GPIO_CTRL_STATE_MAX_NUM_SWITCH] = {
	{"cam_mipi_switch_en_1"},
	{"cam_mipi_switch_en_0"},
	{"cam_mipi_switch_sel_1"},
	{"cam_mipi_switch_sel_0"}
};
#endif

extern void gpio_dump_regs(void);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
struct GPIO_PINCTRL gpio_pinctrl_list_ldo_enable[1] = {
	{"fan53870_chip_enable"}
};

struct GPIO_PINCTRL gpio_pinctrl_list_gpio_power[2] = {
	{"cam0_gpio_power_enable"},
	{"cam0_gpio_power_disable"}
};
#endif

static struct GPIO gpio_instance;

static enum IMGSENSOR_RETURN gpio_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	int    i, j;
	struct GPIO            *pgpio            = (struct GPIO *)pinstance;
	enum   IMGSENSOR_RETURN ret              = IMGSENSOR_RETURN_SUCCESS;
	char str_pinctrl_name[LENGTH_FOR_SNPRINTF];
	char *lookup_names = NULL;

	pgpio->pgpio_mutex = &pcommon->pinctrl_mutex;

	pgpio->ppinctrl = devm_pinctrl_get(&pcommon->pplatform_device->dev);
	if (IS_ERR(pgpio->ppinctrl)) {
		PK_PR_ERR("%s : Cannot find camera pinctrl!", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {
		for (i = 0 ; i < GPIO_CTRL_STATE_MAX_NUM_CAM; i++) {
			if(is_project(0x2169E) || is_project(0x2169F) || is_project(0x216C9) || is_project(0x216CA)
                          || is_project(21711) || is_project(21712)) {
				lookup_names =
				gpio_pinctrl_list_cam_moss[i].ppinctrl_lookup_names;
			} else if(is_project(22083) || is_project(22084) || is_project(22291) || is_project(22292)) {
				lookup_names =
				gpio_pinctrl_list_cam_lijing[i].ppinctrl_lookup_names;
			} else if(is_project(23053) || is_project(23054) || is_project(23253) || is_project(23055)) {
				lookup_names =
				gpio_pinctrl_list_cam_lijinga[i].ppinctrl_lookup_names;
			} else if(is_project(22087) || is_project(22088) || is_project(22331) || is_project(22332) || is_project(22333)
			    || is_project(22334) || is_project(22869)) {
				lookup_names =
				gpio_pinctrl_list_cam_fanli[i].ppinctrl_lookup_names;
			} else if(is_project(22693) || is_project(22694) || is_project(22612) || is_project(0x226B1)) {
				lookup_names =
				gpio_pinctrl_list_cam_22693[i].ppinctrl_lookup_names;
			} else if(is_project(22629) || is_project(22710) || is_project(22711)
			   || is_project(22633) || is_project(22712) || is_project(22713)) {
				lookup_names =
				gpio_pinctrl_list_cam_22629[i].ppinctrl_lookup_names;
			} else if(is_project(23613) || is_project(23686) || is_project(23687) || is_project(23689)
			   || is_project(23707) || is_project(23709)) {
				lookup_names =
				gpio_pinctrl_list_cam_23687[i].ppinctrl_lookup_names;
			} else if(is_project(21081)) {
				lookup_names =
				gpio_pinctrl_list_cam_dufu[i].ppinctrl_lookup_names;
			} else if(is_project(23031)) {
				lookup_names =
				gpio_pinctrl_list_cam_23031[i].ppinctrl_lookup_names;
			} else if(is_project(22277)) {
				lookup_names =
				gpio_pinctrl_list_cam_22277[i].ppinctrl_lookup_names;
			} else if(is_project(23231)) {
				lookup_names =
				gpio_pinctrl_list_cam_23231[i].ppinctrl_lookup_names;
			} else if(is_project(0x212A1)) {
				lookup_names =
				gpio_pinctrl_list_cam_212a1[i].ppinctrl_lookup_names;
			} else if(is_project(0x2266A) || is_project(22604) || is_project(22603) || is_project(22631)
				|| is_project(22632) || is_project(0x2266B)|| is_project(0x2266C) || is_project(0x2260A)
				|| is_project(0x2260B) || is_project(22609)|| is_project(22669) || is_project(23602)) {
				lookup_names =
				gpio_pinctrl_list_cam_blade[i].ppinctrl_lookup_names;
			} else if(is_project(22610) || is_project(22705) || is_project(22706)) {
				lookup_names =
				gpio_pinctrl_list_cam_hawaii[i].ppinctrl_lookup_names;
			}
			else {
				lookup_names =
				gpio_pinctrl_list_cam[i].ppinctrl_lookup_names;
			}
			if (lookup_names) {
				ret = snprintf(str_pinctrl_name,
				sizeof(str_pinctrl_name),
				"cam%d_%s",
				j,
				lookup_names);
				if (ret < 0)
					pr_info(
						"ERROR:%s, snprintf err, %d\n",
						__func__,
						ret);

				pgpio->ppinctrl_state_cam[j][i] =
					pinctrl_lookup_state(
						pgpio->ppinctrl,
						str_pinctrl_name);
			}

			if (pgpio->ppinctrl_state_cam[j][i] == NULL ||
				IS_ERR(pgpio->ppinctrl_state_cam[j][i])) {
				pr_info(
					"ERROR: %s : pinctrl err, %s\n",
					__func__,
					str_pinctrl_name);
				ret = IMGSENSOR_RETURN_ERROR;
			}
		}
	}
	#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (1) {
		if (gpio_pinctrl_list_ldo_enable[0].ppinctrl_lookup_names) {
			pgpio->pinctrl_state_ldo_enable = pinctrl_lookup_state(
				pgpio->ppinctrl,
				gpio_pinctrl_list_ldo_enable[0].ppinctrl_lookup_names);
		}
		if (pgpio->pinctrl_state_ldo_enable == NULL) {
			PK_PR_ERR("%s : pinctrl err, %s\n", __func__,
				gpio_pinctrl_list_ldo_enable[0].ppinctrl_lookup_names);
			ret = IMGSENSOR_RETURN_ERROR;
		}

		if (gpio_pinctrl_list_gpio_power[0].ppinctrl_lookup_names) {
			pgpio->pinctrl_state_gpio_power_enable = pinctrl_lookup_state(
				pgpio->ppinctrl,
				gpio_pinctrl_list_gpio_power[0].ppinctrl_lookup_names);
		}
		if (pgpio->pinctrl_state_gpio_power_enable == NULL) {
			PK_PR_ERR("%s : pinctrl err, %s\n", __func__,
				gpio_pinctrl_list_gpio_power[0].ppinctrl_lookup_names);
			ret = IMGSENSOR_RETURN_ERROR;
		}

		if (gpio_pinctrl_list_gpio_power[1].ppinctrl_lookup_names) {
			pgpio->pinctrl_state_gpio_power_disable = pinctrl_lookup_state(
				pgpio->ppinctrl,
				gpio_pinctrl_list_gpio_power[1].ppinctrl_lookup_names);
		}
		if (pgpio->pinctrl_state_gpio_power_disable == NULL) {
			PK_PR_ERR("%s : pinctrl err, %s\n", __func__,
				gpio_pinctrl_list_gpio_power[1].ppinctrl_lookup_names);
			ret = IMGSENSOR_RETURN_ERROR;
		}
	}
	#endif
#ifdef MIPI_SWITCH
	for (i = 0; i < GPIO_CTRL_STATE_MAX_NUM_SWITCH; i++) {
		if (gpio_pinctrl_list_switch[i].ppinctrl_lookup_names) {
			pgpio->ppinctrl_state_switch[i] =
				pinctrl_lookup_state(
					pgpio->ppinctrl,
			gpio_pinctrl_list_switch[i].ppinctrl_lookup_names);
		}

		if (pgpio->ppinctrl_state_switch[i] == NULL ||
			IS_ERR(pgpio->ppinctrl_state_switch[i])) {
			PK_PR_ERR("%s : pinctrl err, %s\n", __func__,
			gpio_pinctrl_list_switch[i].ppinctrl_lookup_names);
			ret = IMGSENSOR_RETURN_ERROR;
		}
	}
#endif

	return ret;
}

static enum IMGSENSOR_RETURN gpio_release(void *pinstance)
{
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN gpio_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct pinctrl_state  *ppinctrl_state;
	struct GPIO           *pgpio = (struct GPIO *)pinstance;
	enum   GPIO_STATE      gpio_state;

	PK_DBG("%s :debug pinctrl ENABLE, PinIdx %d, Val %d\n",__func__, pin, pin_state);

	if (is_project(19165)) {
                if (pin < IMGSENSOR_HW_PIN_PDN ||
                        pin > IMGSENSOR_HW_PIN_AVDD_1 ||
                        pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
                        pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
                        sensor_idx < 0)
                                return IMGSENSOR_RETURN_ERROR;
	} else {
	if (pin < IMGSENSOR_HW_PIN_PDN ||
#ifdef MIPI_SWITCH
	    pin > IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL ||
#else
		#ifdef OPLUS_FEATURE_CAMERA_COMMON
		pin > IMGSENSOR_HW_PIN_GPIO_POWER_ENABLE ||
		#else
		pin > IMGSENSOR_HW_PIN_DOVDD ||
		#endif
#endif
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;
	}
	gpio_state = (pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0)
		? GPIO_STATE_H : GPIO_STATE_L;

#ifdef MIPI_SWITCH
	if (pin == IMGSENSOR_HW_PIN_MIPI_SWITCH_EN)
		ppinctrl_state = pgpio->ppinctrl_state_switch[
			GPIO_CTRL_STATE_MIPI_SWITCH_EN_H + gpio_state];
	else if (pin == IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL)
		ppinctrl_state = pgpio->ppinctrl_state_switch[
			GPIO_CTRL_STATE_MIPI_SWITCH_SEL_H + gpio_state];
	else
#endif
	{
		#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (pin == IMGSENSOR_HW_PIN_FAN53870_ENABLE) {
			ppinctrl_state = pgpio->pinctrl_state_ldo_enable;
		} else if(pin == IMGSENSOR_HW_PIN_GPIO_POWER_ENABLE) {
			if (gpio_state == GPIO_STATE_H)
				ppinctrl_state = pgpio->pinctrl_state_gpio_power_enable;
			else
				ppinctrl_state = pgpio->pinctrl_state_gpio_power_disable;
		} else {
			ppinctrl_state =
				pgpio->ppinctrl_state_cam[sensor_idx][
				((pin - IMGSENSOR_HW_PIN_PDN) << 1) + gpio_state];
		}
		#else
		ppinctrl_state =
			pgpio->ppinctrl_state_cam[(unsigned int)sensor_idx][
			((pin - IMGSENSOR_HW_PIN_PDN) << 1) + gpio_state];
		#endif
	}

	mutex_lock(pgpio->pgpio_mutex);

	if (ppinctrl_state != NULL && !IS_ERR(ppinctrl_state))
		pinctrl_select_state(pgpio->ppinctrl, ppinctrl_state);
	else
		PK_PR_ERR("%s : pinctrl err, PinIdx %d, Val %d\n",
			__func__, pin, pin_state);

	mutex_unlock(pgpio->pgpio_mutex);

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN gpio_dump(void *pintance)
{
#ifdef DUMP_GPIO
	PK_DBG("[sensor_dump][gpio]\n");
	gpio_dump_regs();
	PK_DBG("[sensor_dump][gpio] finish\n");
#endif
	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_GPIO,
	.pinstance = (void *)&gpio_instance,
	.init      = gpio_init,
	.set       = gpio_set,
	.release   = gpio_release,
	.dump      = gpio_dump
};

enum IMGSENSOR_RETURN imgsensor_hw_gpio_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

