// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

//#ifdef OPLUS_ARCH_EXTENDS
#include <linux/regulator/consumer.h>
//#endif /*OPLUS_ARCH_EXTENDS*/

#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#include "fsa4480-i2c.h"
#include "../../../../drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define HL5280_DEVICE_REG_VALUE 0x49

#define FSA4480_DEVICE_ID  0x00
#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS0  0x06
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_FUN_EN          0x12
#define FSA4480_JACK_STATUS     0x17
#define FSA4480_RESET           0x1E
#define FSA4480_MASK            0x30

#undef dev_dbg
#define dev_dbg dev_info

enum switch_vendor {
    FSA4480 = 0,
    HL5280,
    DIO4480
};

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
	unsigned int hs_det_pin;
	//#ifdef OPLUS_ARCH_EXTENDS
	int hs_det_level;
	//#endif /*OPLUS_ARCH_EXTENDS*/
	enum switch_vendor vendor;
	bool plug_state;

	//#ifdef OPLUS_ARCH_EXTENDS
	struct work_struct usbc_digit_work;
	uint8_t  new_state;
	uint8_t  old_state;
	//#endif /*OPLUS_ARCH_EXTENDS*/
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_MASK,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SWITCH_CONTROL, 0x18},
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

#ifdef OPLUS_ARCH_EXTENDS
struct fsa4480_priv *g_fsa_priv = NULL;
static int dio_status = 0;
#endif

//#ifdef OPLUS_ARCH_EXTENDS
static void fsa4480_usbc_digit_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_digit_work);
	unsigned int reg04,reg05,reg06,reg07;
	char buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};
	struct device *dev;

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}

	dev = fsa_priv->dev;
	if (!dev)
		return ;

	mutex_lock(&fsa_priv->notification_lock);

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &reg04);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &reg05);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &reg06);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &reg07);
	dev_info(dev, "%s: is not audio jack,reg04:%02x,reg05:%02x,reg06:%02x,reg07:%02x\n",
	__func__, reg04, reg05, reg06, reg07);
	if ((reg04 != 0x98) || (reg05 != 0x18) || (reg06 != 0x05) || (reg07 != 0x00)) {
		dev_info(dev, "%s: error status\n",__func__);
		scnprintf(buf, sizeof(buf) - 1, "func@@%s$$typec_mode@@%d,%d$$regs@@%02x,%02x,%02x,%02x", \
			__func__, fsa_priv->old_state, fsa_priv->new_state,reg04,reg05,reg06,reg07);
		upload_mm_fb_kevent_to_atlas_limit(OPLUS_AUDIO_EVENTID_HEADSET_DET, buf, MM_FB_KEY_RATELIMIT_1MIN);

		if (fsa_priv->new_state == TYPEC_UNATTACHED) {
			regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
			usleep_range(1000, 1005);
			dev_info(dev, "%s: error status,usb plug out,reset type switch\n",__func__);
		}
	}
	mutex_unlock(&fsa_priv->notification_lock);
	pm_relax(fsa_priv->dev);
}
//#endif


static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

    if (fsa_priv->vendor == DIO4480) {
        return;
    }
	
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb, struct fsa4480_priv, pd_nb);
	struct device *dev;
	struct tcp_notify *noti = ptr;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (fsa_priv->vendor == HL5280) {
		dev_err(dev, "%s: switch chip is HL5280\n", __func__);
	}

	dev_info(dev, "%s: typeC event: %d plug_state: %d\n", __func__, evt, fsa_priv->plug_state);

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:
		dev_info(dev, "%s: old_state: %d, new_state: %d\n",
			__func__, noti->typec_state.old_state, noti->typec_state.new_state);
		fsa_priv->new_state = noti->typec_state.new_state;
		fsa_priv->old_state = noti->typec_state.old_state;
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* AUDIO plug in */
			dev_err(dev, "%s: audio plug in\n", __func__);
			fsa_priv->plug_state = true;
			pm_stay_awake(fsa_priv->dev);
			cancel_work(&fsa_priv->usbc_analog_work);
			schedule_work(&fsa_priv->usbc_analog_work);
		} else if (fsa_priv->plug_state == true
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* AUDIO plug out */
			dev_err(dev, "%s: audio plug out\n", __func__);
			fsa_priv->plug_state = false;
			pm_stay_awake(fsa_priv->dev);
			cancel_work(&fsa_priv->usbc_analog_work);
			schedule_work(&fsa_priv->usbc_analog_work);
		} else if(fsa_priv->vendor == DIO4480){
			pm_stay_awake(fsa_priv->dev);
			cancel_work(&fsa_priv->usbc_digit_work);
			schedule_work(&fsa_priv->usbc_digit_work);
		}
		break;
	case TCP_NOTIFY_PLUG_OUT:
		dev_err(dev, "%s: typec plug out\n", __func__);
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	struct device *dev;
	unsigned int switch_status = 0;
	unsigned int jack_status = 0;
	int state;
	int i = 0;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);

	dev_info(dev, "%s: plug_state %d\n", __func__, fsa_priv->plug_state);

	if (fsa_priv->plug_state) {
		if (fsa_priv->vendor == DIO4480) {
			/* activate switches */
			regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x45);

			for (i = 0;i < 100 ;i++) {
				usleep_range(10*1000, 10*1005);
				regmap_read(fsa_priv->regmap, 0x18, &switch_status);
				if (switch_status & 0x4) {
					dev_info(dev, "%s: Audio jack detection and configuration has occurred.\n", __func__);
					i = 100;
				}
			}

			dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);
			dio_status = 0;

		} else {
			/* activate switches */
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x45);
			usleep_range(4000, 4005);
			dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);
		}

		regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &jack_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_JACK_STATUS, jack_status);
		if ((jack_status & 0x2)&&(fsa_priv->vendor != DIO4480)) {
			//for 3 pole, mic switch to SBU2
			dev_info(dev, "%s: set mic to sbu2 for 3 pole.\n", __func__);
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			usleep_range(4000, 4005);
		}

		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS0, switch_status);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS1, switch_status);

		if (fsa_priv->vendor == DIO4480) {
			regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &switch_status);
			dev_info(dev, "%s: reg[0x17]=0x%x.\n", __func__, switch_status);

			if (switch_status == 0x01) {
				dev_info(dev, "%s: error status,swap MIC_GND\n", __func__);
				regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
				usleep_range(1000, 1005);
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x00);//GND - GSBU1, MIC - SBU2
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
				usleep_range(10000, 10005);
				dio_status = 1;
			}

		}
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			dev_info(dev, "%s: set hs_det_pin to enable.\n", __func__);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			usleep_range(1000, 1005);
			//dev_info(dev, "%s: panzhao test sleep 300ms\n", __func__);
			gpio_direction_output(fsa_priv->hs_det_pin, fsa_priv->hs_det_level);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}
	} else {
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			dev_info(dev, "%s: set hs_det_pin to disable.\n", __func__);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			gpio_direction_output(fsa_priv->hs_det_pin, !fsa_priv->hs_det_level);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}

		if (fsa_priv->vendor == DIO4480) {
			regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x18);
			regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x98);
			dev_info(dev, "%s: plugout. set to usb mode\n", __func__);
			dio_status = 0;
		} else {
			/* deactivate switches */
			fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
			//#ifdef OPLUS_ARCH_EXTENDS
			regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9D);
			//#endif /*OPLUS_ARCH_EXTENDS*/
		}
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(fsa_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	rc = fsa4480_usbc_analog_setup_switches(fsa_priv);

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	return blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int switch_control = 0;
#ifndef OPLUS_ARCH_EXTENDS
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
#else
	struct fsa4480_priv *fsa_priv = g_fsa_priv;
	if (!fsa_priv) {
		pr_err("fsa_priv is NULL\n");
		return -EINVAL;
	}
#endif
	if (!fsa_priv->regmap)
		return -EINVAL;

	pr_info("%s - switch event: %d\n", __func__, event);
	if(fsa_priv->vendor != DIO4480) {
		return -EINVAL;
	}

	switch (event) {
	case FSA_MIC_GND_SWAP:
		if (fsa_priv->vendor == DIO4480) {
			if (dio_status) {
				pr_info("%s - switch event: %d delay 0ms\n", __func__, event);
				regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
				usleep_range(1000, 1005);
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x07); //GND - GSBU2, MIC - SBU1
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
				dio_status = 0;
			}
		} else {
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
					&switch_control);
			if ((switch_control & 0x07) == 0x07)
				switch_control = 0x0;
			else
				switch_control = 0x7;
		}
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

static int fsa4480_parse_dt(struct fsa4480_priv *fsa_priv,
	struct device *dev)
{
    struct device_node *dNode = dev->of_node;
    int ret = 0;
    int hs_det_level = 0;
    int state = 0;

//#ifdef OPLUS_ARCH_EXTENDS
    static struct regulator *vio28_reg;
    vio28_reg = regulator_get(NULL, "vio28");

    if(!vio28_reg)
    {
        pr_err("%s: VIO28_reg is not NULL\n", __func__);
    }

    /*set 3v*/
    ret = regulator_set_voltage(vio28_reg, 3000000, 3000000);
    if(!ret)
   {
      pr_warning("regulator_set_voltage(%d) failed!\n", ret);
   }
//#endif /*OPLUS_ARCH_EXTENDS*/

    if (dNode == NULL) {
        return -ENODEV;
    }

	if (!fsa_priv) {
		pr_err("%s: fsa_priv is NULL\n", __func__);
		return -ENOMEM;
	}

	fsa_priv->hs_det_pin = of_get_named_gpio(dNode,
	        "fsa4480,hs-det-gpio", 0);
	if (!gpio_is_valid(fsa_priv->hs_det_pin)) {
	    pr_warning("%s: hs-det-gpio in dt node is missing\n", __func__);
	    return -ENODEV;
	}

	//#ifdef OPLUS_ARCH_EXTENDS
	ret = of_property_read_u32(dNode,
			"fsa4480,hs-det-level", &hs_det_level);
	if (ret) {
		pr_warning("%s: hs-det-level request fail\n", __func__);
		fsa_priv->hs_det_level = 0;
	} else {
		fsa_priv->hs_det_level = hs_det_level;
	}
	//#endif /*OPLUS_ARCH_EXTENDS*/

	ret = gpio_request(fsa_priv->hs_det_pin, "fsa4480_hs_det");
	if (ret) {
		pr_warning("%s: hs-det-gpio request fail\n", __func__);
		return ret;
	}
	dev_info(dev, "%s: hs_det_pin gpio = %d.\n", __func__, fsa_priv->hs_det_pin);

	//#ifdef OPLUS_ARCH_EXTENDS
	gpio_direction_output(fsa_priv->hs_det_pin, !fsa_priv->hs_det_level);
	//#else /*OPLUS_ARCH_EXTENDS*/
	//gpio_direction_output(fsa_priv->hs_det_pin, 1);
	//#endif /*OPLUS_ARCH_EXTENDS*/

	state = gpio_get_value(fsa_priv->hs_det_pin);
	dev_info(dev, "%s: init hs_det_pin state = %d.\n", __func__, state);

	return ret;
}

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	int rc = 0;
	unsigned int reg_value = 0;

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	fsa_priv->dev = &i2c->dev;

	fsa4480_parse_dt(fsa_priv, &i2c->dev);

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}


	regmap_read(fsa_priv->regmap, FSA4480_DEVICE_ID, &reg_value);
	dev_err(fsa_priv->dev, "%s: device id reg value: 0x%x\n", __func__, reg_value);
	if (HL5280_DEVICE_REG_VALUE == reg_value) {
		dev_err(fsa_priv->dev, "%s: switch chip is HL5280\n", __func__);
		fsa_priv->vendor = HL5280;
        } else if (0xF1 == reg_value) {
		dev_err(fsa_priv->dev, "%s: switch chip is DIO4480\n", __func__);
		fsa_priv->vendor = DIO4480;
	} else {
		dev_err(fsa_priv->dev, "%s: switch chip is FSA4480\n", __func__);
		fsa_priv->vendor = FSA4480;
	}

    if (fsa_priv->vendor != DIO4480) {
	    fsa4480_update_reg_defaults(fsa_priv->regmap);
		//#ifdef OPLUS_ARCH_EXTENDS
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9D);
		//#endif /*OPLUS_ARCH_EXTENDS*/

	}

	if (fsa_priv->vendor == DIO4480) {
		regmap_write(fsa_priv->regmap, 0x1e, 0x01);//reset DIO4480
		usleep_range(1*1000, 1*1005);
	}

	fsa_priv->plug_state = false;
	fsa_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!fsa_priv->tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		goto err_data;
	}

	fsa_priv->pd_nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->pd_nb.priority = 0;
	rc = register_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (rc < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		goto err_data;
	}

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	INIT_WORK(&fsa_priv->usbc_digit_work,
		  fsa4480_usbc_digit_work_fn);

	fsa_priv->fsa4480_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((fsa_priv->fsa4480_notifier).rwsem);
	fsa_priv->fsa4480_notifier.head = NULL;
#ifdef OPLUS_ARCH_EXTENDS
	g_fsa_priv = fsa_priv;
#endif

	return 0;

err_data:
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	cancel_work_sync(&fsa_priv->usbc_digit_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
#ifdef OPLUS_ARCH_EXTENDS
	register_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, fsa_priv);
#endif

	dev_set_drvdata(&i2c->dev, NULL);
	return 0;
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}
late_initcall_sync(fsa4480_init);
//module_init(fsa4480_init);

/*static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);
*/
MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
