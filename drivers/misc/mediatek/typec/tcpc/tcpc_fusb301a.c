/*
 * Copyright (c) 2021, LGE Inc. All rights reserved.
 *
 * fusb301 USB TYPE-C Configuration Controller driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/pm_wakeup.h>

#include <linux/workqueue.h>
#include <inc/tcpci.h>
#include <inc/tcpc_fusb301a.h>
/*#include <linux/switch.h>*/
#include <inc/tcpm.h>

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))
#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)
#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))
#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
        (((_byte) & (_mask)) >> (_shift))
#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
        __BITS_GET(_byte, _bit, FFS(_bit))
#undef  __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
        (((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))
#undef  BITS_SET
#define BITS_SET(_byte, _bit, _val) \
        __BITS_SET(_byte, _bit, FFS(_bit), _val)
#undef  BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
        (((_byte) & (_bit)) == (_bit))
#define IS_ERR_VALUE_FUSB301(x) unlikely((unsigned long)(x) >= (unsigned long)-MAX_ERRNO)
/* Register Map */
#define FUSB301_REG_DEVICEID            0x01
#define FUSB301_REG_MODES               0x02
#define FUSB301_REG_CONTROL             0x03
#define FUSB301_REG_MANUAL              0x04
#define FUSB301_REG_RESET               0x05
#define FUSB301_REG_MASK                0x10
#define FUSB301_REG_STATUS              0x11
#define FUSB301_REG_TYPE                0x12
#define FUSB301_REG_INT                 0x13
/* Register Vaules */
#define FUSB301_DRP_ACC                 BIT(5)
#define FUSB301_DRP                     BIT(4)
#define FUSB301_SNK_ACC                 BIT(3)
#define FUSB301_SNK                     BIT(2)
#define FUSB301_SRC_ACC                 BIT(1)
#define FUSB301_SRC                     BIT(0)
#define FUSB301_TGL_35MS                0
#define FUSB301_TGL_30MS                1
#define FUSB301_TGL_25MS                2
#define FUSB301_TGL_20MS                3
#define FUSB301_HOST_0MA                0
#define FUSB301_HOST_DEFAULT            1
#define FUSB301_HOST_1500MA             2
#define FUSB301_HOST_3000MA             3
#define FUSB301_INT_ENABLE              0x00
#define FUSB301_INT_DISABLE             0x01
#define FUSB301_UNATT_SNK               BIT(3)
#define FUSB301_UNATT_SRC               BIT(2)
#define FUSB301_DISABLED                BIT(1)
#define FUSB301_ERR_REC                 BIT(0)
#define FUSB301_DISABLED_CLEAR          0x00
#define FUSB301_SW_RESET                BIT(0)
#define FUSB301_M_ACC_CH                BIT(3)
#define FUSB301_M_BCLVL                 BIT(2)
#define FUSB301_M_DETACH                BIT(1)
#define FUSB301_M_ATTACH                BIT(0)
#define FUSB301_FAULT_CC                0x30
#define FUSB301_CC2                     0x20
#define FUSB301_CC1                     0x10
#define FUSB301_NO_CONN                 0x00
#define FUSB301_VBUS_OK                 0x08
#define FUSB301_SNK_0MA                 0x00
#define FUSB301_SNK_DEFAULT             0x02
#define FUSB301_SNK_1500MA              0x04
#define FUSB301_SNK_3000MA              0x06
#define FUSB301_ATTACH                  0x01
#define FUSB301_TYPE_SNK                BIT(4)
#define FUSB301_TYPE_SRC                BIT(3)
#define FUSB301_TYPE_PWR_ACC            BIT(2)
#define FUSB301_TYPE_DBG_ACC            BIT(1)
#define FUSB301_TYPE_AUD_ACC            BIT(0)
#define FUSB301_TYPE_PWR_DBG_ACC       (FUSB301_TYPE_PWR_ACC|\
                                        FUSB301_TYPE_DBG_ACC)
#define FUSB301_TYPE_PWR_AUD_ACC       (FUSB301_TYPE_PWR_ACC|\
                                        FUSB301_TYPE_AUD_ACC)
#define FUSB301_TYPE_INVALID            0x00
#define FUSB301_INT_ACC_CH              BIT(3)
#define FUSB301_INT_BCLVL               BIT(2)
#define FUSB301_INT_DETACH              BIT(1)
#define FUSB301_INT_ATTACH              BIT(0)
#define FUSB301_REV10                   0x10
#define FUSB301_REV11                   0x11
#define FUSB301_REV12                   0x12
/* Mask */
#define FUSB301_TGL_MASK                0x30
#define FUSB301_HOST_CUR_MASK           0x06
#define FUSB301_INT_MASK                0x01
#define FUSB301_BCLVL_MASK              0x06
#define FUSB301_TYPE_MASK               0x1F
#define FUSB301_MODE_MASK               0x3F
#define FUSB301_INT_STS_MASK            0x0F
#define FUSB301_MAX_TRY_COUNT           10
/* FUSB STATES */
#define FUSB_STATE_DISABLED             0x00
#define FUSB_STATE_ERROR_RECOVERY       0x01
#define FUSB_STATE_UNATTACHED_SNK       0x02
#define FUSB_STATE_UNATTACHED_SRC       0x03
#define FUSB_STATE_ATTACHWAIT_SNK       0x04
#define FUSB_STATE_ATTACHWAIT_SRC       0x05
#define FUSB_STATE_ATTACHED_SNK         0x06
#define FUSB_STATE_ATTACHED_SRC         0x07
#define FUSB_STATE_AUDIO_ACCESSORY      0x08
#define FUSB_STATE_DEBUG_ACCESSORY      0x09
#define FUSB_STATE_TRY_SNK              0x0A
#define FUSB_STATE_TRYWAIT_SRC          0x0B
#define FUSB_STATE_TRY_SRC              0x0C
#define FUSB_STATE_TRYWAIT_SNK          0x0D
/* wake lock timeout in ms */
#define FUSB301_WAKE_LOCK_TIMEOUT       1000
#define ROLE_SWITCH_TIMEOUT		1500
#define FUSB301_TRY_TIMEOUT		600
#define FUSB301_CC_DEBOUNCE_TIMEOUT	200
/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 start */
#define FUSB301_HUB_STATUS_1	29
#define FUSB301_HUB_STATUS_2	45
#define TCPEC_POLARITY_VALUE_0	0
#define TCPEC_POLARITY_VALUE_1	1
#define TCPEC_POLARITY_VALUE_2	2
/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 end */
struct fusb301_data {
	u32 int_gpio;
	u32 init_mode;
	u32 dfp_power;
	u32 dttime;
	bool try_snk_emulation;
	u32 ttry_timeout;
	u32 ccdebounce_timeout;
};
struct fusb301_chip {
	struct i2c_client *client;
	struct fusb301_data *pdata;
	struct workqueue_struct  *cc_wq;
	struct tcpc_device *tcpc;
	struct tcpc_desc *tcpc_desc;
	int irq_gpio;
	int ufp_power;
	u8 mode;
	u8 dev_id;
	u8 type;
	u8 state;
	u8 orient;
	u8 bc_lvl;
	u8 dfp_power;
	u8 dttime;
	bool triedsnk;
	int try_attcnt;
	struct work_struct dwork;
	struct delayed_work twork;
	struct wakeup_source *wlock;
	struct mutex mlock;
	//struct power_supply *usb_psy;
	struct dual_role_phy_instance *dual_role;
	bool role_switch;
	struct dual_role_phy_desc *desc;
};

struct fusb301_chip *g_fusb301_chip;

enum typec_cc_status {
	TYPEC_CC_RP_DEF,
};

#define fusb_update_state(chip, st) \
	if(chip && st < FUSB_STATE_TRY_SRC) { \
		chip->state = st; \
		dev_info(&chip->client->dev, "%s: %s\n", __func__, #st); \
		wake_up_interruptible(&mode_switch); \
	}
#define STR(s)    #s
#define STRV(s)   STR(s)
static void fusb301_detach(struct fusb301_chip *chip);
DECLARE_WAIT_QUEUE_HEAD(mode_switch);
int fusb301_set_mode(struct fusb301_chip *chip, u8 mode);
static int fusb301_write_masked_byte(struct i2c_client *client,
					u8 addr, u8 mask, u8 val)
{
	int rc;
	if (!mask) {
		/* no actual access */
		rc = -EINVAL;
		goto out;
	}
	rc = i2c_smbus_read_byte_data(client, addr);
	if (!IS_ERR_VALUE_FUSB301(rc)) {
		rc = i2c_smbus_write_byte_data(client,
			addr, BITS_SET((u8)rc, mask, val));
		if (IS_ERR_VALUE_FUSB301(rc))
			pr_err("%s : write iic failed.\n", __func__);
	} else {
		pr_err("%s : read iic failed.\n", __func__);
		return rc;
	}
out:
	return rc;
}
static int fusb301_read_device_id(struct i2c_client *client)
{
	struct device *cdev = &client->dev;
	int rc = 0;
	dev_info(cdev, "fusb301_read_device_id entry open\n");
	rc = i2c_smbus_read_byte_data(client,
				FUSB301_REG_DEVICEID);

	dev_info(cdev, "fusb301a usb device id: %d\n", rc);

	if ((IS_ERR_VALUE_FUSB301(rc)) || (rc != 0x12)) {
		dev_info(cdev, "IS_ERR_VALUE_FUSB301 device id: 0x%2x\n", rc);
		return (rc != 0x12) ? (-1) : (-2);
	}
	// chip->dev_id = rc;
	dev_info(cdev, "device id: 0x%2x\n", rc);

	// return 0;
	return rc;
}
static int fusb301_update_status(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	u16 control_now;
	/* read mode & control register */
	rc = i2c_smbus_read_word_data(chip->client, FUSB301_REG_MODES);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}
	chip->mode = rc & FUSB301_MODE_MASK;
	control_now = (rc >> 8) & 0xFF;
	chip->dfp_power = BITS_GET(control_now, FUSB301_HOST_CUR_MASK);
	chip->dttime = BITS_GET(control_now, FUSB301_TGL_MASK);
	return 0;
}
/*
 * spec lets transitioning to below states from any state
 *  FUSB_STATE_DISABLED
 *  FUSB_STATE_ERROR_RECOVERY
 *  FUSB_STATE_UNATTACHED_SNK
 *  FUSB_STATE_UNATTACHED_SRC
 */
static int fusb301_set_chip_state(struct fusb301_chip *chip, u8 state)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	if(state > FUSB_STATE_UNATTACHED_SRC)
		return -EINVAL;
	rc = i2c_smbus_write_byte_data(chip->client, FUSB301_REG_MANUAL,
				state == FUSB_STATE_DISABLED ? FUSB301_DISABLED:
				state == FUSB_STATE_ERROR_RECOVERY ? FUSB301_ERR_REC:
				state == FUSB_STATE_UNATTACHED_SNK ? FUSB301_UNATT_SNK:
				FUSB301_UNATT_SRC);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "failed to write manual(%d)\n", rc);
	}
	return rc;
}
int fusb301_set_mode(struct fusb301_chip *chip, u8 mode)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	if (mode > FUSB301_DRP_ACC) {
		dev_err(cdev, "mode(%d) is unavailable\n", mode);
		return -EINVAL;
	}
	if (mode != chip->mode) {
		rc = i2c_smbus_write_byte_data(chip->client,
				FUSB301_REG_MODES, mode);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			dev_err(cdev, "%s: failed to write mode\n", __func__);
			return rc;
		}
		chip->mode = mode;
	}
	dev_err(cdev, "%s: mode (%d)(%d)\n", __func__, chip->mode , mode);
	return rc;
}
static int fusb301_set_dfp_power(struct fusb301_chip *chip, u8 hcurrent)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	dev_info(cdev, "fusb301_set_dfp_power entry open\n");
	if (hcurrent > FUSB301_HOST_3000MA) {
		dev_err(cdev, "hcurrent(%d) is unavailable\n",
					hcurrent);
		return -EINVAL;
	}
	if (hcurrent == chip->dfp_power) {
		dev_dbg(cdev, "vaule is not updated(%d)\n",
					hcurrent);
		return rc;
	}
	rc = fusb301_write_masked_byte(chip->client,
					FUSB301_REG_CONTROL,
					FUSB301_HOST_CUR_MASK,
					hcurrent);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "failed to write current(%d)\n", rc);
		return rc;
	}
	chip->dfp_power = hcurrent;
	dev_dbg(cdev, "%s: host current(%d)\n", __func__, hcurrent);

	return rc;
}
/*
 * When 3A capable DRP device is connected without VBUS,
 * DRP always detect it as SINK device erroneously.
 * Since USB Type-C specification 1.0 and 1.1 doesn't
 * consider this corner case, apply workaround for this case.
 * Set host mode current to 1.5A initially, and then change
 * it to default USB current right after detection SINK port.
 */
static int fusb301_init_force_dfp_power(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = fusb301_write_masked_byte(chip->client,
					FUSB301_REG_CONTROL,
					FUSB301_HOST_CUR_MASK,
					FUSB301_HOST_1500MA);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "failed to write current\n");
		return rc;
	}
	chip->dfp_power = FUSB301_HOST_1500MA;
	dev_dbg(cdev, "%s: host current (%d)\n", __func__, rc);

	return rc;
}
static int fusb301_set_toggle_time(struct fusb301_chip *chip, u8 toggle_time)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	if (toggle_time > FUSB301_TGL_20MS) {
		dev_err(cdev, "toggle_time(%d) is unavailable\n", toggle_time);
		return -EINVAL;
	}
	if (toggle_time == chip->dttime) {
		dev_dbg(cdev, "vaule is not updated (%d)\n", toggle_time);
		return rc;
	}
	rc = fusb301_write_masked_byte(chip->client,
					FUSB301_REG_CONTROL,
					FUSB301_TGL_MASK,
					toggle_time);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "failed to write toggle time\n");
		return rc;
	}
	chip->dttime = toggle_time;
	dev_dbg(cdev, "%s: host current (%d)\n", __func__, rc);
	return rc;
}
static int fusb301_init_reg(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	dev_info(cdev, "fusb301_init_reg open\n");
	/* change current */
	rc = fusb301_init_force_dfp_power(chip);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "%s: failed to force dfp power\n",
				__func__);
	/* change toggle time */
	rc = fusb301_set_toggle_time(chip, chip->pdata->dttime);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "%s: failed to set toggle time\n",
				__func__);
	/* change mode */
	/*
	 * force to DRP+ACC,chip->pdata->init_mode
	 */
	rc = fusb301_set_mode(chip, FUSB301_DRP_ACC);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "%s: failed to set mode\n",
				__func__);
	/* set error recovery state */
	rc = fusb301_set_chip_state(chip,
				FUSB_STATE_ERROR_RECOVERY);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "%s: failed to set error recovery state\n",
				__func__);
	dev_info(cdev, "fusb301_init_reg ok\n");

	return rc;
}

static int fusb301_reset_device(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	rc = i2c_smbus_write_byte_data(chip->client,
					FUSB301_REG_RESET,
					FUSB301_SW_RESET);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "reset fails\n");
		return rc;
	}
	msleep(10);
	rc = fusb301_update_status(chip);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "fail to read status\n");
	rc = fusb301_init_reg(chip);
	if (IS_ERR_VALUE_FUSB301(rc))
		dev_err(cdev, "fail to init reg\n");
	fusb301_detach(chip);
	/* clear global interrupt mask */
	rc = fusb301_write_masked_byte(chip->client,
				FUSB301_REG_CONTROL,
				FUSB301_INT_MASK,
				FUSB301_INT_ENABLE);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: fail to init\n", __func__);
		return rc;
	}
	dev_info(cdev, "mode[0x%02x], host_cur[0x%02x], dttime[0x%02x]\n",
			chip->mode, chip->dfp_power, chip->dttime);
	return rc;
}
static ssize_t fregdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	u8 start_reg[] = {0x01, 0x02,
			  0x03, 0x04,
			  0x05, 0x06,
			  0x10, 0x11,
			  0x12, 0x13}; /* reserved 0x06 */
	int i = 0;
	int rc = 0;
	int ret = 0;
	mutex_lock(&chip->mlock);
	for (i = 0 ; i < 5; i++) {
		rc = i2c_smbus_read_word_data(chip->client, start_reg[(i*2)]);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			pr_err("cannot read 0x%02x\n", start_reg[(i*2)]);
			rc = 0;
			goto dump_unlock;
		}
		ret += snprintf(buf + ret, 1024, "from 0x%02x read 0x%02x\n"
						"from 0x%02x read 0x%02x\n",
							start_reg[(i*2)],
							(rc & 0xFF),
							start_reg[(i*2)+1],
							((rc >> 8) & 0xFF));
	}
dump_unlock:
	mutex_unlock(&chip->mlock);
	return ret;
}
DEVICE_ATTR(fregdump, S_IRUGO, fregdump_show, NULL);
static ssize_t ftype_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	switch (chip->type) {
	case FUSB301_TYPE_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SINK(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SOURCE(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_PWR_ACC:
		ret = snprintf(buf, PAGE_SIZE, "PWRACC(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_DBG_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DEBUGACC(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_PWR_DBG_ACC:
		ret = snprintf(buf, PAGE_SIZE, "POWEREDDEBUGACC(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_AUD_ACC:
		ret = snprintf(buf, PAGE_SIZE, "AUDIOACC(%d)\n", chip->type);
		break;
	case FUSB301_TYPE_PWR_AUD_ACC:
		ret = snprintf(buf, PAGE_SIZE, "POWEREDAUDIOACC(%d)\n", chip->type);
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "NOTYPE(%d)\n", chip->type);
		break;
	}
	mutex_unlock(&chip->mlock);
	return ret;
}
DEVICE_ATTR(ftype, S_IRUGO , ftype_show, NULL);
static ssize_t fchip_state_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE,
			STRV(FUSB_STATE_DISABLED) " - FUSB_STATE_DISABLED\n"
			STRV(FUSB_STATE_ERROR_RECOVERY) " - FUSB_STATE_ERROR_RECOVERY\n"
			STRV(FUSB_STATE_UNATTACHED_SNK) " - FUSB_STATE_UNATTACHED_SNK\n"
			STRV(FUSB_STATE_UNATTACHED_SRC) " - FUSB_STATE_UNATTACHED_SRC\n");
}
static ssize_t fchip_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int state = 0;
	int rc = 0;
	if (sscanf(buff, "%d", &state) == 1) {
		mutex_lock(&chip->mlock);
		if(((state == FUSB_STATE_UNATTACHED_SNK) &&
			(chip->mode & (FUSB301_SRC | FUSB301_SRC_ACC))) ||
			((state == FUSB_STATE_UNATTACHED_SRC) &&
			(chip->mode & (FUSB301_SNK | FUSB301_SNK_ACC)))) {
			mutex_unlock(&chip->mlock);
			return -EINVAL;
		}
		rc = fusb301_set_chip_state(chip, (u8)state);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}
		fusb301_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fchip_state, S_IRUGO | S_IWUSR, fchip_state_show, fchip_state_store);
static ssize_t fmode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	switch (chip->mode) {
	case FUSB301_DRP_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DRP+ACC(%d)\n", chip->mode);
		break;
	case FUSB301_DRP:
		ret = snprintf(buf, PAGE_SIZE, "DRP(%d)\n", chip->mode);
		break;
        case FUSB301_SNK_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SNK+ACC(%d)\n", chip->mode);
		break;
	case FUSB301_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SNK(%d)\n", chip->mode);
		break;
	case FUSB301_SRC_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SRC+ACC(%d)\n", chip->mode);
		break;
	case FUSB301_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SRC(%d)\n", chip->mode);
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "UNKNOWN(%d)\n", chip->mode);
		break;
	}
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t fmode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int mode = 0;
	int rc = 0;
	if (sscanf(buff, "%d", &mode) == 1) {
		mutex_lock(&chip->mlock);
		/*
		 * since device trigger to usb happens independent
		 * from charger based on vbus, setting SRC modes
		 * doesn't prevent usb enumeration as device
		 * KNOWN LIMITATION
		 */
		rc = fusb301_set_mode(chip, (u8)mode);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}
		rc = fusb301_set_chip_state(chip,
					FUSB_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}
		fusb301_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fmode, S_IRUGO | S_IWUSR, fmode_show, fmode_store);
static ssize_t fdttime_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dttime);
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t fdttime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int dttime = 0;
	int rc = 0;
	if (sscanf(buff, "%d", &dttime) == 1) {
		mutex_lock(&chip->mlock);
		rc = fusb301_set_toggle_time(chip, (u8)dttime);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_FUSB301(rc))
			return rc;
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fdttime, S_IRUGO | S_IWUSR, fdttime_show, fdttime_store);
static ssize_t fhostcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dfp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t fhostcur_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	int rc = 0;
	if (sscanf(buff, "%d", &buf) == 1) {
		mutex_lock(&chip->mlock);
		rc = fusb301_set_dfp_power(chip, (u8)buf);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_FUSB301(rc))
			return rc;
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fhostcur, S_IRUGO | S_IWUSR, fhostcur_show, fhostcur_store);
static ssize_t fclientcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->ufp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}
DEVICE_ATTR(fclientcur, S_IRUGO, fclientcur_show, NULL);
static ssize_t freset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	u32 reset = 0;
	int rc = 0;
	if (sscanf(buff, "%u", &reset) == 1) {
		mutex_lock(&chip->mlock);
		rc = fusb301_reset_device(chip);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE_FUSB301(rc))
			return rc;
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(freset, S_IWUSR, NULL, freset_store);
static ssize_t fsw_trysnk_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->pdata->try_snk_emulation);
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t fsw_trysnk_store(struct device *dev,
			struct device_attribute *attr,
			const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	if ((sscanf(buff, "%d", &buf) == 1) && (buf == 0 || buf == 1)) {
		mutex_lock(&chip->mlock);
		chip->pdata->try_snk_emulation = buf;
		if (chip->state == FUSB_STATE_ERROR_RECOVERY)
			chip->triedsnk = !chip->pdata->try_snk_emulation;
		mutex_unlock(&chip->mlock);
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fsw_trysnk, S_IRUGO | S_IWUSR,\
			fsw_trysnk_show, fsw_trysnk_store);
static ssize_t ftry_timeout_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->pdata->ttry_timeout);
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t ftry_timeout_store(struct device *dev,
			struct device_attribute *attr,
			const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	if ((sscanf(buff, "%d", &buf) == 1) && (buf >= 0)) {
		mutex_lock(&chip->mlock);
		chip->pdata->ttry_timeout = buf;
		mutex_unlock(&chip->mlock);
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(ftry_timeout, S_IRUGO | S_IWUSR,\
		ftry_timeout_show, ftry_timeout_store);
static ssize_t fccdebounce_timeout_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n",
			chip->pdata->ccdebounce_timeout);
	mutex_unlock(&chip->mlock);
	return ret;
}
static ssize_t fccdebounce_timeout_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	if ((sscanf(buff, "%d", &buf) == 1) && (buf >= 0)) {
		mutex_lock(&chip->mlock);
		chip->pdata->ccdebounce_timeout = buf;
		mutex_unlock(&chip->mlock);
		return size;
	}
	return -EINVAL;
}
DEVICE_ATTR(fccdebounce_timeout, S_IRUGO | S_IWUSR,\
                fccdebounce_timeout_show, fccdebounce_timeout_store);
static int fusb301_create_devices(struct device *cdev)
{
	int ret = 0;
	ret = device_create_file(cdev, &dev_attr_fchip_state);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fchip_state\n");
		ret = -ENODEV;
		goto err0;
	}
	ret = device_create_file(cdev, &dev_attr_ftype);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_ftype\n");
		ret = -ENODEV;
		goto err1;
	}
	ret = device_create_file(cdev, &dev_attr_fmode);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fmode\n");
		ret = -ENODEV;
		goto err2;
	}
	ret = device_create_file(cdev, &dev_attr_freset);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_freset\n");
		ret = -ENODEV;
		goto err3;
	}
	ret = device_create_file(cdev, &dev_attr_fdttime);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fdttime\n");
		ret = -ENODEV;
		goto err4;
	}
	ret = device_create_file(cdev, &dev_attr_fhostcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fhostcur\n");
		ret = -ENODEV;
		goto err5;
	}
	ret = device_create_file(cdev, &dev_attr_fclientcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fufpcur\n");
		ret = -ENODEV;
		goto err6;
	}
	ret = device_create_file(cdev, &dev_attr_fregdump);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fregdump\n");
		ret = -ENODEV;
		goto err7;
	}
	ret = device_create_file(cdev, &dev_attr_fsw_trysnk);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fsw_trysnk\n");
		ret = -ENODEV;
		goto err8;
	}
	ret = device_create_file(cdev, &dev_attr_ftry_timeout);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_ftry_timeout\n");;
		ret = -ENODEV;
		goto err9;
	}
	ret = device_create_file(cdev, &dev_attr_fccdebounce_timeout);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fccdebounce_timeout\n");
		ret = -ENODEV;
		goto err10;
	}
	return ret;
err10:
	device_remove_file(cdev, &dev_attr_ftry_timeout);
err9:
	device_remove_file(cdev, &dev_attr_fsw_trysnk);
err8:
	device_remove_file(cdev, &dev_attr_fregdump);
err7:
	device_remove_file(cdev, &dev_attr_fclientcur);
err6:
	device_remove_file(cdev, &dev_attr_fhostcur);
err5:
	device_remove_file(cdev, &dev_attr_fdttime);
err4:
	device_remove_file(cdev, &dev_attr_freset);
err3:
	device_remove_file(cdev, &dev_attr_fmode);
err2:
	device_remove_file(cdev, &dev_attr_ftype);
err1:
	device_remove_file(cdev, &dev_attr_fchip_state);
err0:
	return ret;
}
static void fusb301_destory_device(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_ftype);
	device_remove_file(cdev, &dev_attr_fmode);
	device_remove_file(cdev, &dev_attr_freset);
	device_remove_file(cdev, &dev_attr_fdttime);
	device_remove_file(cdev, &dev_attr_fhostcur);
	device_remove_file(cdev, &dev_attr_fclientcur);
	device_remove_file(cdev, &dev_attr_fregdump);
}

static void fusb301_get_cc_status(struct fusb301_chip *chip)
{
	struct tcpc_device *tcpc_dev = chip->tcpc;
	struct device *cdev = &chip->client->dev;
	int  rc = 0;
	u8  status;

	rc = i2c_smbus_read_word_data(chip->client,
				FUSB301_REG_STATUS);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: failed to read\n", __func__);
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
	status = rc & 0xFF;

	chip->orient = status & FUSB301_FAULT_CC;
	if (chip->orient == FUSB301_CC1){
		chip->orient = TCPEC_POLARITY_VALUE_0;
	} else if (chip->orient == FUSB301_CC2) {
		chip->orient = TCPEC_POLARITY_VALUE_1;
	} else {
		chip->orient = TCPEC_POLARITY_VALUE_2;
	}
	tcpc_dev->typec_polarity = chip->orient;
	printk("tcpc->typec_polarity= %d\n",tcpc_dev->typec_polarity);
}

static int fusb301_power_set_icurrent_max(struct fusb301_chip *chip,
						int icurrent)
{
	return -ENXIO;
}
/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 start */
/*extern bool hub_plugin_flag;zjc*/
/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 end */
static void fusb301_bclvl_changed(struct fusb301_chip *chip)
{
	struct tcpc_device *tcpc_dev = chip->tcpc;
	struct device *cdev = &chip->client->dev;
	int limit;
    int  rc = 0;
	u8  status,type;
	rc = i2c_smbus_read_word_data(chip->client,
				FUSB301_REG_STATUS);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: failed to read\n", __func__);
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
	status = rc & 0xFF;
	/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 start */
	/*if (status == FUSB301_HUB_STATUS_1 || status == FUSB301_HUB_STATUS_2) {
		hub_plugin_flag = true;
	} else {
		hub_plugin_flag = false;
	}
	pr_err("status = %d, hub_plugin_flag = %d\n", status, hub_plugin_flag);*/
	/* HS03s for DEVAL5626-623 by shixuanxuan at 20210927 end */
	type = (status & FUSB301_ATTACH) ?
			(rc >> 8) & FUSB301_TYPE_MASK : FUSB301_TYPE_INVALID;

	dev_dbg(cdev, "sts[0x%02x], type[0x%02x]\n", status, type);
	if (type == FUSB301_TYPE_SRC ||
			type == FUSB301_TYPE_PWR_AUD_ACC ||
			type == FUSB301_TYPE_PWR_DBG_ACC ||
			type == FUSB301_TYPE_PWR_ACC) {
		chip->orient = (status & 0x30) >> 4;
		if (chip->orient == 0x01){
			chip->orient = 0;
		} else if (chip->orient == 0x02) {
			chip->orient = 1;
		} else {
			chip->orient = 2;
		}
		tcpc_dev->typec_polarity = chip->orient;
		chip->bc_lvl = status & 0x06;
		limit = (chip->bc_lvl == FUSB301_SNK_3000MA ? 3000 :
				(chip->bc_lvl == FUSB301_SNK_1500MA ? 1500 : 0));
		fusb301_power_set_icurrent_max(chip, limit);
		chip->bc_lvl = (status & 0x06) >> 1;
	}
}
static void fusb301_acc_changed(struct fusb301_chip *chip)
{
	/* TODO */
	/* implement acc changed work */
	return;
}
static void fusb301_src_detected(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if (chip->mode & (FUSB301_SRC | FUSB301_SRC_ACC)) {
		dev_err(cdev, "not support in source mode\n");
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
	if (chip->state == FUSB_STATE_TRY_SNK)
		cancel_delayed_work(&chip->twork);
	fusb_update_state(chip, FUSB_STATE_ATTACHED_SNK);
	chip->type = FUSB301_TYPE_SRC;
	dev_info(cdev, "fusb301_src_detected OK\n");
}
static void fusb301_snk_detected(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if (chip->mode & (FUSB301_SNK | FUSB301_SNK_ACC)) {
		dev_err(cdev, "not support in sink mode\n");
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}

	/*
	 * disable try sink
	 */
	chip->triedsnk = true;

	/* SW Try.SNK Workaround below Rev 1.2 */
	if ((!chip->triedsnk) &&
		(chip->mode & (FUSB301_DRP | FUSB301_DRP_ACC))) {
		if (IS_ERR_VALUE_FUSB301(fusb301_set_mode(chip, FUSB301_SNK)) ||
			IS_ERR_VALUE_FUSB301(fusb301_set_chip_state(chip,
						FUSB_STATE_UNATTACHED_SNK))) {
			dev_err(cdev, "%s: failed to config trySnk\n", __func__);
			if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
				dev_err(cdev, "%s: failed to reset\n", __func__);
		} else {
			fusb_update_state(chip, FUSB_STATE_TRY_SNK);
			chip->triedsnk = true;
			queue_delayed_work(chip->cc_wq, &chip->twork,
					msecs_to_jiffies(chip->pdata->ttry_timeout));
		}
	} else {
		/*
		 * chip->triedsnk == true
		 * or
		 * mode == FUSB301_SRC/FUSB301_SRC_ACC
		 */
		fusb301_set_dfp_power(chip, chip->pdata->dfp_power);
		if (chip->state == FUSB_STATE_TRYWAIT_SRC)
			cancel_delayed_work(&chip->twork);
		fusb_update_state(chip, FUSB_STATE_ATTACHED_SRC);
		chip->type = FUSB301_TYPE_SNK;
	}
}
static void fusb301_dbg_acc_detected(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if (chip->mode & (FUSB301_SRC | FUSB301_SNK | FUSB301_DRP)) {
		dev_err(cdev, "not support accessory mode\n");
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
	/*
	 * TODO
	 * need to implement
	 */
	fusb_update_state(chip, FUSB_STATE_DEBUG_ACCESSORY);
}
static void fusb301_aud_acc_detected(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if (chip->mode & (FUSB301_SRC | FUSB301_SNK | FUSB301_DRP)) {
		dev_err(cdev, "not support accessory mode\n");
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
	/*
	 * TODO
	 * need to implement
	 */
	fusb_update_state(chip, FUSB_STATE_AUDIO_ACCESSORY);
}
static void fusb301_timer_try_expired(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	if (IS_ERR_VALUE_FUSB301(fusb301_set_mode(chip, FUSB301_SRC)) ||
		IS_ERR_VALUE_FUSB301(fusb301_set_chip_state(chip,
					FUSB_STATE_UNATTACHED_SRC))) {
		dev_err(cdev, "%s: failed to config tryWaitSrc\n", __func__);
		if(IS_ERR_VALUE_FUSB301(fusb301_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
	} else {
		fusb_update_state(chip, FUSB_STATE_TRYWAIT_SRC);
		queue_delayed_work(chip->cc_wq, &chip->twork,
			msecs_to_jiffies(chip->pdata->ccdebounce_timeout));
	}
}
static void fusb301_detach(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_dbg(cdev, "%s: type[0x%02x] chipstate[0x%02x]\n",
			__func__, chip->type, chip->state);
	dev_info(cdev, "fusb301_detach entry open\n");
	switch (chip->state) {
	case FUSB_STATE_ATTACHED_SRC:
		fusb301_init_force_dfp_power(chip);
		break;
	case FUSB_STATE_ATTACHED_SNK:
		fusb301_power_set_icurrent_max(chip, 0);
		break;
	case FUSB_STATE_DEBUG_ACCESSORY:
	case FUSB_STATE_AUDIO_ACCESSORY:
		break;
	case FUSB_STATE_TRY_SNK:
	case FUSB_STATE_TRYWAIT_SRC:
		cancel_delayed_work(&chip->twork);
		break;
	case FUSB_STATE_DISABLED:
	case FUSB_STATE_ERROR_RECOVERY:
		break;
	case FUSB_STATE_TRY_SRC:
	case FUSB_STATE_TRYWAIT_SNK:
	default:
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		break;
	}

	/*
	 * disable set to DRP+ACC mode, control by fusb251.
	 */
	chip->triedsnk = false;

	if ((chip->triedsnk && chip->pdata->try_snk_emulation)
					|| chip->role_switch) {
		chip->role_switch = false;
		if (IS_ERR_VALUE_FUSB301(fusb301_set_mode(chip,
						chip->pdata->init_mode)) ||
			IS_ERR_VALUE_FUSB301(fusb301_set_chip_state(chip,
						FUSB_STATE_ERROR_RECOVERY))) {
			dev_err(cdev, "%s: failed to set init mode\n", __func__);
		}
	}
	chip->type = FUSB301_TYPE_INVALID;
	chip->bc_lvl = FUSB301_SNK_0MA;
	chip->ufp_power = 0;
	/* Try.Snk in HW ? */
	chip->triedsnk = !chip->pdata->try_snk_emulation;
	chip->try_attcnt = 0;
	fusb_update_state(chip, FUSB_STATE_ERROR_RECOVERY);
	dev_err(cdev, "%s: typec_attach_old= %d\n",__func__, chip->tcpc->typec_attach_old);
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;

	tcpci_notify_typec_state(chip->tcpc);
	if (chip->tcpc->typec_attach_old == TYPEC_ATTACHED_SRC) {
		tcpci_source_vbus(chip->tcpc,
			TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
	}
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
}
static bool fusb301_is_vbus_off(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	rc = i2c_smbus_read_byte_data(chip->client,
				FUSB301_REG_STATUS);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return false;
	}

	return !((rc & FUSB301_ATTACH) && (rc & FUSB301_VBUS_OK));
}
static bool fusb301_is_vbus_on(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	rc = i2c_smbus_read_byte_data(chip->client, FUSB301_REG_STATUS);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return false;
	}
	dev_info(cdev, "fusb301_is_vbus_on OPEN\n");
	return !!(rc & FUSB301_VBUS_OK);
}
/* workaround BC Level detection plugging slowly with C ot A on Rev1.0 */
static bool fusb301_bclvl_detect_wa(struct fusb301_chip *chip,
							u8 status, u8 type)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	if (((type == FUSB301_TYPE_SRC) ||
		((type == FUSB301_TYPE_INVALID) && (status & FUSB301_VBUS_OK))) &&
		!(status & FUSB301_BCLVL_MASK) &&
		(chip->try_attcnt < FUSB301_MAX_TRY_COUNT)) {
		rc = fusb301_set_chip_state(chip,
					FUSB_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_FUSB301(rc)) {
			dev_err(cdev, "%s: failed to set error recovery state\n",
					__func__);
			goto err;
		}
		chip->try_attcnt++;
		msleep(100);
		/*
		 * when cable is unplug during bc level workaournd,
		 * detach interrupt does not occur
		 */
		if (fusb301_is_vbus_off(chip)) {
			chip->try_attcnt = 0;
			dev_dbg(cdev, "%s: vbus is off\n", __func__);
		}
		return true;
	}
err:
	chip->try_attcnt = 0;
	return false;
}
static void fusb301_attach(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	u8 status, type;
	/* get status and type */
	rc = i2c_smbus_read_word_data(chip->client,
			FUSB301_REG_STATUS);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return;
	}
	status = rc & 0xFF;
	type = (status & FUSB301_ATTACH) ?
			(rc >> 8) & FUSB301_TYPE_MASK : FUSB301_TYPE_INVALID;
	dev_info(cdev, "sts[0x%02x], type[0x%02x]\n", status, type);
	if ((chip->state != FUSB_STATE_ERROR_RECOVERY) &&
		(chip->state != FUSB_STATE_TRY_SNK) &&
		(chip->state != FUSB_STATE_TRYWAIT_SRC)) {
		rc = fusb301_set_chip_state(chip,
				FUSB_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_FUSB301(rc))
			dev_err(cdev, "%s: failed to set error recovery\n",
					__func__);
		fusb301_detach(chip);
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		return;
	}
	if((chip->dev_id == FUSB301_REV10) &&
		fusb301_bclvl_detect_wa(chip, status, type)) {
		return;
	}
	switch (type) {
	case FUSB301_TYPE_SRC:
		dev_info(cdev, "FUSB301_TYPE_SRC into FUSB301_TYPE_SRC OPEN\n");
		fusb301_src_detected(chip);
		if (chip->tcpc->typec_attach_old != TYPEC_UNATTACHED) {
			chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
			fusb301_detach(chip);
		} else if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SNK) {
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SNK;
			dev_info(cdev, "FUSB301_TYPE_SRC into tcpci_sink_vbus Ok\n");
			tcpci_sink_vbus(chip->tcpc, TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SINK_5V, 1500);
			dev_info(cdev, "FUSB301_TYPE_SRC into FUSB301_TYPE_SRC Ok\n");
		}
		break;
	case FUSB301_TYPE_SNK:
	    if(fusb301_is_vbus_on(chip)) {
			dev_err(cdev, "%s: vbus voltage was high\n", __func__);
			tcpci_source_vbus(chip->tcpc,
				TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
			fusb301_detach(chip);
			break;
		}
		fusb301_snk_detected(chip);
		if (chip->tcpc->typec_attach_old != TYPEC_UNATTACHED) {
			chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
			fusb301_detach(chip);
		} else if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
			chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_3_0;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
			tcpci_source_vbus(chip->tcpc,
				TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 3000);
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			dev_info(cdev, "FUSB301_TYPE_SNK into FUSB301_TYPE_SNK Ok\n");
		}
		break;
	case FUSB301_TYPE_PWR_ACC:
		/*
		 * just power without functional dbg/aud determination
		 * ideally should not happen
		 */
		chip->type = type;
		break;
	case FUSB301_TYPE_DBG_ACC:
	case FUSB301_TYPE_PWR_DBG_ACC:
		dev_info(cdev, "fusb301_attach into FUSB301_TYPE_PWR_DBG_ACC  case \n");
		fusb301_dbg_acc_detected(chip);
		chip->type = type;
		if(fusb301_is_vbus_on(chip)) {
			dev_err(cdev, "%s: vbus voltage was high\n", __func__);
			break;
		}
		if (chip->tcpc->typec_attach_old != TYPEC_UNATTACHED) {
			chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
			fusb301_detach(chip);
		} else if (chip->tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
			chip->tcpc_desc->rp_lvl = TYPEC_CC_RP_3_0;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
			tcpci_source_vbus(chip->tcpc,
				TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 3000);
			tcpci_notify_typec_state(chip->tcpc);
			chip->tcpc->typec_attach_old = TYPEC_ATTACHED_SRC;
			dev_info(cdev, "FUSB301_TYPE_SNK into FUSB301_TYPE_SNK Ok\n");
		}
		break;
	case FUSB301_TYPE_AUD_ACC:
	case FUSB301_TYPE_PWR_AUD_ACC:
		fusb301_aud_acc_detected(chip);
		chip->type = type;
		break;
	case FUSB301_TYPE_INVALID:
		fusb301_detach(chip);
		dev_err(cdev, "%s: Invaild type[0x%02x]\n", __func__, type);
		break;
	default:
		rc = fusb301_set_chip_state(chip,
				FUSB_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE_FUSB301(rc))
			dev_err(cdev, "%s: failed to set error recovery\n",
					__func__);
		fusb301_detach(chip);
		dev_err(cdev, "%s: Unknwon type[0x%02x]\n", __func__, type);
		break;
	}
}
static void fusb301_timer_work_handler(struct work_struct *work)
{
	struct fusb301_chip *chip =
			container_of(work, struct fusb301_chip, twork.work);
	struct device *cdev = &chip->client->dev;
	mutex_lock(&chip->mlock);
	if (chip->state == FUSB_STATE_TRY_SNK) {
		if (fusb301_is_vbus_on(chip)) {
			if (IS_ERR_VALUE_FUSB301(fusb301_set_mode(chip,	chip->pdata->init_mode))) {
				dev_err(cdev, "%s: failed to set init mode\n", __func__);
			}
			chip->triedsnk = !chip->pdata->try_snk_emulation;
			mutex_unlock(&chip->mlock);
			return;
		}
		fusb301_timer_try_expired(chip);
	} else if (chip->state == FUSB_STATE_TRYWAIT_SRC) {
		fusb301_detach(chip);
	}
	dev_info(cdev, "fusb301 ok to fusb301_timer_work_handler \n");
	mutex_unlock(&chip->mlock);
}
static void fusb301_work_handler(struct work_struct *work)
{
	struct fusb301_chip *chip =
			container_of(work, struct fusb301_chip, dwork);
	struct device *cdev = &chip->client->dev;
	int rc;
	u8 int_sts;

	dev_info(cdev, "fusb301 to fusb301_work_handler open \n");
	__pm_stay_awake(chip->wlock);
	mutex_lock(&chip->mlock);
	/* get interrupt */

	fusb301_get_cc_status(chip);
	rc = i2c_smbus_read_byte_data(chip->client, FUSB301_REG_INT);
	if (IS_ERR_VALUE_FUSB301(rc)) {
		dev_err(cdev, "%s: fusb301 failed to read interrupt\n", __func__);
		goto work_unlock;
	}
	int_sts = rc & FUSB301_INT_STS_MASK;
	dev_info(cdev, "%s: int_sts[0x%02x]\n", __func__, int_sts);
	if (int_sts & FUSB301_INT_DETACH) {
		dev_info(cdev, "fusb301 to fusb301_work_handler 000\n");
		fusb301_detach(chip);
	} else {
		if (int_sts & FUSB301_INT_ATTACH) {
			dev_info(cdev, "fusb301 to fusb301_work_handler 001 \n");
			fusb301_attach(chip);
		}
		if (int_sts & FUSB301_INT_BCLVL) {
			dev_info(cdev, "fusb301 to fusb301_work_handler 002 \n");
			fusb301_bclvl_changed(chip);
		}
		if (int_sts & FUSB301_INT_ACC_CH) {
			dev_info(cdev, "fusb301 to fusb301_work_handler 003 \n");
			fusb301_acc_changed(chip);
		}
	}
work_unlock:
	mutex_unlock(&chip->mlock);
	__pm_relax(chip->wlock);
}
static irqreturn_t fusb301_interrupt(int irq, void *data)
{
	struct fusb301_chip *chip = (struct fusb301_chip *)data;
	if (!chip) {
		pr_err("%s : called before init.\n", __func__);
		return IRQ_HANDLED;
	}
	/*
	 * wake_lock_timeout, prevents multiple suspend entries
	 * before charger gets chance to trigger usb core for device
	 */
	__pm_wakeup_event(chip->wlock, jiffies_to_msecs(FUSB301_WAKE_LOCK_TIMEOUT));
	if (!queue_work(chip->cc_wq, &chip->dwork))
		dev_err(&chip->client->dev, "%s: can't alloc work\n", __func__);
	return IRQ_HANDLED;
}

static int fusb301_init_gpio(struct fusb301_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	dev_err(cdev, "fusb301_init_gpio\n");
	/* Start to enable fusb301 Chip */
	if (gpio_is_valid(chip->pdata->int_gpio)) {
		rc = gpio_request_one(chip->pdata->int_gpio,
				GPIOF_DIR_IN, "fusb301_int_gpio");
		if (rc)
			dev_err(cdev, "unable to request int_gpio %d\n",
					chip->pdata->int_gpio);
	} else {
		dev_err(cdev, "int_gpio %d is not valid\n",
				chip->pdata->int_gpio);
		rc = -EINVAL;
	}
	return rc;
}
static void fusb301_free_gpio(struct fusb301_chip *chip)
{
	if (gpio_is_valid(chip->pdata->int_gpio))
		gpio_free(chip->pdata->int_gpio);
}

static int fusb301_parse_dt(struct fusb301_chip *chip)

{
	struct device *cdev = &chip->client->dev;
	struct device_node *np = cdev->of_node;
	struct fusb301_data *data = chip->pdata;
	int ret;

	if (!np)
		return -EINVAL;

	pr_info("%s\n", __func__);

	np = of_find_node_by_name(NULL, "usb_type_c_fusb301");
	if (!np) {
		pr_err("%s find node type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	ret = of_get_named_gpio(np, "fusb301,intr_gpio", 0);
	if (ret < 0) {
		pr_err("%s no intr_gpio info\n", __func__);
		goto out;
	}
	data->int_gpio = ret;

	data->init_mode=FUSB301_DRP_ACC;
	data->dfp_power = FUSB301_HOST_DEFAULT;
	data->dttime = FUSB301_TGL_35MS;
	data->try_snk_emulation = true;
	data->ttry_timeout = FUSB301_TRY_TIMEOUT;
	data->ccdebounce_timeout = FUSB301_CC_DEBOUNCE_TIMEOUT;

	dev_info(cdev, "fusb301_parse_dt entry OK!\n");
out:
      return ret;
}

int fusb_alert_status_clear(struct tcpc_device *tcpc,
						uint32_t mask)
{
	pr_info("%s: enter alert not support\n",__func__);
	return 0;
}

static int fusb_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int ret;
	struct fusb301_chip *chip = g_fusb301_chip;
	struct device *cdev = &chip->client->dev;
	pr_info("%s: enter fusb301a tcpc init\n",__func__);
	ret = fusb301_reset_device(chip);
	if (ret) {
		dev_err(cdev, "failed to initialize\n");
		return ret;
	}
	return 0;
}

int fusb_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	pr_info("%s: enter fault not support\n",__func__);
	return 0;
}

int fusb_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	pr_info("%s: enter get alert mask not support\n",__func__);
	return 0;
}

int fusb_get_alert_status(struct tcpc_device *tcpc,
						uint32_t *alert)
{
	pr_info("%s: enter get alert status not support\n",__func__);
	return 0;
}

static int fusb_get_power_status(struct tcpc_device *tcpc,
						uint16_t *pwr_status)
{
	pr_info("%s: enter get fault not support\n",__func__);
	return 0;
}
int fusb_get_fault_status(struct tcpc_device *tcpc,
						uint8_t *status)
{
	pr_info("%s: enter get fault not support\n",__func__);
	return 0;
}
static int fusb_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
 {
	struct fusb301_chip *chip = NULL;
	unsigned int cc, cc_polarity, sink;

	chip = g_fusb301_chip;

 	pr_info("%s: enter \n",__func__);
	pr_info("%s: cc_polarity=%d, partner_type=%d,bc_lvl=%d\n",
		__func__, chip->orient, chip->type, chip->bc_lvl);

	cc_polarity = chip->orient;

	if (cc_polarity == 0x01){
		*cc2 = TYPEC_CC_OPEN;
	} else if (cc_polarity == 0x02) {
		*cc1 = TYPEC_CC_OPEN;
	} else {
		*cc1 = TYPEC_CC_OPEN;
		*cc2 = TYPEC_CC_OPEN;
	}

	cc = chip->bc_lvl;
	/* FUSB301_TYPE_SRC means src attached, 301a is sink */
	sink = chip->type;
	switch (cc) {
	case 0x1:
		if (*cc1 == TYPEC_CC_OPEN && *cc2 != TYPEC_CC_OPEN) {
			*cc2 = (sink == FUSB301_TYPE_SNK)? TYPEC_CC_RP_DEF : TYPEC_CC_RA;
		}
		else if (*cc2 == TYPEC_CC_OPEN && *cc1 != TYPEC_CC_OPEN){
			*cc1 = (sink == FUSB301_TYPE_SNK)? TYPEC_CC_RP_DEF : TYPEC_CC_RA;
		}
	case 0x2:
		if (*cc1 == TYPEC_CC_OPEN && *cc2 != TYPEC_CC_OPEN) {
			*cc2 = (sink == FUSB301_TYPE_SNK) ? TYPEC_CC_RP_1_5 : TYPEC_CC_RD;
		}
		else if (*cc2 == TYPEC_CC_OPEN && *cc1 != TYPEC_CC_OPEN){
			*cc1 = (sink == FUSB301_TYPE_SNK) ? TYPEC_CC_RP_1_5 : TYPEC_CC_RD;
		}
	case 0x3:
		if (*cc1 == TYPEC_CC_OPEN && *cc2 != TYPEC_CC_OPEN) {
			if (sink == FUSB301_TYPE_SNK)
				*cc2 = TYPEC_CC_RP_3_0;
		}
		else if (*cc2 == TYPEC_CC_OPEN && *cc1 != TYPEC_CC_OPEN){
			if (sink == FUSB301_TYPE_SNK)
				*cc1 = TYPEC_CC_RP_3_0;
		}
	case 0x0:
	default:
		*cc1 = TYPEC_CC_OPEN;
		*cc2 = TYPEC_CC_OPEN;
	}

	return 0;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int fusb301_set_msg_header(struct tcpc_device *tcpc,
					uint8_t power_role, uint8_t data_role)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_protocol_reset(struct tcpc_device *tcpc_dev)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_get_message(struct tcpc_device *tcpc, uint32_t *payload,
			uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int fusb301_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	pr_err("%s\n", __func__);
	return 0;
}
#endif


static int fusb_set_cc(struct tcpc_device *tcpc, int pull)
{
	struct fusb301_chip *chip = g_fusb301_chip;
	int ret;
	u8 value=0;

	pr_info("%s enter \n", __func__);

	pull = TYPEC_CC_PULL_GET_RES(pull);
	pr_info("%s enter pull = %d\n", __func__, pull);
	if (pull == TYPEC_CC_RP)
		value = FUSB301_SRC;
	else if (pull == TYPEC_CC_RD)
		value = FUSB301_SNK;
	else if (pull == TYPEC_CC_DRP)
		value = FUSB301_DRP;

	/*value = value << SET_MODE_SELECT_SHIFT;*/
	
	ret = fusb301_set_mode(chip, value);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	return 0;
}

static int fusb_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	pr_info("%s: enter \n",__func__);
	return 0;
}
static int fusb_set_low_rp_duty(struct tcpc_device *tcpc,
						bool low_rp)
{
	pr_info("%s: enter \n",__func__);
	return 0;
}
static int fusb_set_vconn(struct tcpc_device *tcpc, int enable)
{
	pr_info("%s: enter \n",__func__);
	return 0;
}
static int fusb_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
	pr_info("%s: enter \n",__func__);
	return 0;
}

static struct tcpc_ops fusb_tcpc_ops = {
	.init = fusb_tcpc_init,
	.alert_status_clear = fusb_alert_status_clear,
	.fault_status_clear = fusb_fault_status_clear,
	.get_alert_mask = fusb_get_alert_mask,
	.get_alert_status = fusb_get_alert_status,
	.get_power_status = fusb_get_power_status,
	.get_fault_status = fusb_get_fault_status,
	.get_cc = fusb_get_cc,
	.set_cc = fusb_set_cc,
	.set_polarity = fusb_set_polarity,
	.set_low_rp_duty = fusb_set_low_rp_duty,
	.set_vconn = fusb_set_vconn,
	.deinit = fusb_tcpc_deinit,
#ifdef CONFIG_USB_POWER_DELIVERY
	.set_msg_header = fusb301_set_msg_header,
	.set_rx_enable = fusb301_set_rx_enable,
	.protocol_reset = fusb301_protocol_reset,
	.get_message = fusb301_get_message,
	.transmit = fusb301_transmit,
	.set_bist_test_mode = fusb301_set_bist_test_mode,
	.set_bist_carrier_mode = fusb301_set_bist_carrier_mode,
#endif	/* CONFIG_USB_POWER_DELIVERY */
};

enum {
	WAKE_LOCK_SUSPEND, /* Prevent suspend */
	WAKE_LOCK_TYPE_COUNT
};

static int fusb301a_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(chip->irq_gpio);
	disable_irq(chip->irq_gpio);
	fusb301_set_mode(chip, FUSB301_SNK);
	pr_info("fusb301_s:%s\n", __func__);
	return 0;
}

static int fusb301a_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip  = (struct fusb301_chip *)i2c_get_clientdata(client);

	enable_irq(chip->irq_gpio);
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(chip->irq_gpio);
	fusb301_set_mode(chip, FUSB301_DRP_ACC);
	pr_info("fusb301_s: %s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(fusb301a_dev_pm_ops,
			fusb301a_pm_suspend, fusb301a_pm_resume);

/************************************************************************
 *
 *       fcc_status_show
 *
 *  Description :
 *  -------------
 *  show cc_status
 *
 ************************************************************************/
static ssize_t typec_cc_orientation_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	int typec_cc_orientation = -1;
	int rc, ret = 0;

	rc = i2c_smbus_read_byte_data(chip->client, FUSB301_REG_STATUS);

	if (rc < 0) {
		pr_err("cannot read FUSB301_REG_STATUS\n");
		rc = 0xFF;
		ret = snprintf (buf, PAGE_SIZE, "%d\n", rc);
	} else {
		chip->orient = (rc & 0x30) >> 4;
		typec_cc_orientation = chip->orient;
		ret = snprintf (buf, PAGE_SIZE, "%d\n", typec_cc_orientation);
	}
	dev_info(dev, "typec_cc_orientation = %d\n", typec_cc_orientation);
	return ret;
}
struct device_attribute dev_attr_typec_cc_orientation_fusb =
					    __ATTR(typec_cc_orientation, S_IRUGO, typec_cc_orientation_show, NULL);
static int fusb301_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct fusb301_chip *chip = NULL;
	struct device *cdev = &client->dev;
	struct tcpc_desc *desc_tcpc;
    struct fusb301_data *data;
	int ret = 0, chip_vid;

	dev_info(cdev, "fusb301_probe open\n");

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA |
				I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(cdev, "smbus data not supported!\n");
		return -EIO;
	}

	chip_vid = fusb301_read_device_id(client);
	/*HS03s for SR-AL5625-01-541 by wangzikang at 20210614 start*/
	if (chip_vid < 0) {
	/*HS03s for SR-AL5625-01-541 by wangzikang at 20210614 end*/
		dev_err(cdev, "fusb301 not support\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(cdev, sizeof(struct fusb301_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(cdev, "can't alloc fusb301_chip\n");
		return -ENOMEM;
	}
	chip->dev_id = chip_vid;
	chip->client = client;
	dev_err(cdev, "i2c_set_clientdata\n");
	i2c_set_clientdata(client, chip);
	// ret = fusb301_read_device_id(chip);
	// if (ret) {
	// 	dev_err(cdev, "fusb301 not support\n");
	// 	goto err1;
	// }
    data = devm_kzalloc(cdev,
				sizeof(struct fusb301_data), GFP_KERNEL);
	if (!data) {
		dev_err(cdev, "can't alloc fusb301_data\n");
		ret = -ENOMEM;
		goto err1;
	}
	chip->pdata = data;

	ret = fusb301_parse_dt(chip);
	if (ret < 0) {
		dev_err(cdev, "can't parse dt\n");
		goto err2;
	}
	dev_err(cdev, "fusb301 parse dt in probe OK\n");

	ret = fusb301_init_gpio(chip);
	if (ret < 0) {
		dev_err(cdev, "fail to init gpio\n");
		goto err2;
	}
	dev_err(cdev, "ok to init gpio\n");
	chip->type = FUSB301_TYPE_INVALID;
	chip->state = FUSB_STATE_ERROR_RECOVERY;
	chip->bc_lvl = FUSB301_SNK_0MA;
	chip->ufp_power = 0;
	/* Try.Snk in HW? */
	chip->triedsnk = !chip->pdata->try_snk_emulation;
	chip->try_attcnt = 0;

	/*WORK init*/
	chip->cc_wq = alloc_ordered_workqueue("fusb301-wq", WQ_HIGHPRI);
	if (!chip->cc_wq) {
		dev_err(cdev, "unable to create workqueue fuxb301-wq\n");
		goto err2;
	}
	INIT_WORK(&chip->dwork, fusb301_work_handler);
	INIT_DELAYED_WORK(&chip->twork, fusb301_timer_work_handler);
	chip->wlock = wakeup_source_register(cdev, "fusb301_wake");
	mutex_init(&chip->mlock);

	/*Creat Device*/
	ret = fusb301_create_devices(cdev);
	if (IS_ERR_VALUE_FUSB301(ret)) {
		dev_err(cdev, "could not create devices\n");
		goto err3;
	}

	ret = device_create_file(&client->dev, &dev_attr_typec_cc_orientation_fusb);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create dev_attr_typec_cc_orientation_fusb\n");
		ret = -ENODEV;
		goto err_create_file;
	}

	/*TCPC_DEV INIT*/
	desc_tcpc = devm_kzalloc(cdev, sizeof(*desc_tcpc), GFP_KERNEL);
	if (!desc_tcpc) {
		dev_err(cdev, "can't alloc desc_tcpc\n");
		goto err4;
	}
	desc_tcpc->name = kzalloc(13, GFP_KERNEL);
	if (!desc_tcpc->name) {
		goto err4;
	}
	strcpy((char *)desc_tcpc->name, "type_c_port0");
	desc_tcpc->role_def = TYPEC_ROLE_TRY_SRC;
	desc_tcpc->rp_lvl = TYPEC_CC_RP_1_5;
	chip->tcpc_desc = desc_tcpc;
	dev_info(cdev, "%s: type_c_port0, role=%d\n",
		__func__, desc_tcpc->role_def);
	chip->tcpc = tcpc_device_register(cdev, desc_tcpc, &fusb_tcpc_ops, chip);
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;

	/*Request IRQ*/
	chip->irq_gpio = gpio_to_irq(chip->pdata->int_gpio);
	if (chip->irq_gpio < 0) {
		dev_err(cdev, "could not register int_gpio\n");
		ret = -ENXIO;
		goto err4;
	}
	ret = devm_request_irq(cdev, chip->irq_gpio,
				fusb301_interrupt,
				IRQF_TRIGGER_FALLING,
				"fusb301_int_irq", chip);
	if (ret) {
		dev_err(cdev, "failed to reqeust IRQ\n");
		goto err4;
	}
	dev_info(cdev, "fusb301_probe  irq_gpio (0x%2x)\n", chip->irq_gpio);

	g_fusb301_chip = chip;

	enable_irq_wake(chip->irq_gpio);
	dev_info(cdev, "fusb301_probe SUCESS!\n");

	return 0;

err4:
	tcpc_device_unregister(cdev,chip->tcpc);
	devm_kfree(cdev, chip->tcpc);
err_create_file:
	device_remove_file(&client->dev, &dev_attr_typec_cc_orientation_fusb);
err3:
	fusb301_destory_device(cdev);
	destroy_workqueue(chip->cc_wq);
	mutex_destroy(&chip->mlock);
	wakeup_source_unregister(chip->wlock);
	fusb301_free_gpio(chip);
err2:
	devm_kfree(cdev, chip->pdata);

err1:
	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);
	return ret;
}

static int fusb301_remove(struct i2c_client *client)
{
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;
	if (!chip) {
		pr_err("%s : chip is null\n", __func__);
		return -ENODEV;
	}
	if (chip->irq_gpio > 0){
		devm_free_irq(cdev, chip->irq_gpio, chip);
	}
	fusb301_destory_device(cdev);
	destroy_workqueue(chip->cc_wq);
	mutex_destroy(&chip->mlock);
	wakeup_source_unregister(chip->wlock);
	fusb301_free_gpio(chip);
	devm_kfree(cdev, chip->pdata);
	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);
	device_remove_file(&client->dev, &dev_attr_typec_cc_orientation_fusb);
	return 0;
}

static void fusb301_shutdown(struct i2c_client *client)
{
	struct fusb301_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;
	disable_irq(chip->irq_gpio);
	if (IS_ERR_VALUE_FUSB301(fusb301_set_mode(chip, FUSB301_SNK)) ||
			IS_ERR_VALUE_FUSB301(fusb301_set_chip_state(chip,
					FUSB_STATE_ERROR_RECOVERY)))
		dev_err(cdev, "%s: failed to set sink mode\n", __func__);
}

static const struct i2c_device_id fusb301_id_table[] = {
	{"fusb301", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, fusb301_id_table);

static struct of_device_id fusb301_match_table[] = {
	{ .compatible = "fusb,usb_type_c0",},
	{ },
};

static struct i2c_driver fusb301_i2c_driver = {

	.driver = {
		.name = "fusb301",
		.owner = THIS_MODULE,
		.of_match_table = fusb301_match_table,
		.pm = &fusb301a_dev_pm_ops,
	},
		.probe = fusb301_probe,
		.remove = fusb301_remove,
		.shutdown = fusb301_shutdown,
		.id_table = fusb301_id_table,

};
static __init int fusb301_i2c_init(void)
{
	pr_info("%s: fusb301_i2c_init ok\n", __func__);
	return i2c_add_driver(&fusb301_i2c_driver);
}
static __exit void fusb301_i2c_exit(void)
{
	i2c_del_driver(&fusb301_i2c_driver);
}
module_init(fusb301_i2c_init);
module_exit(fusb301_i2c_exit);
MODULE_AUTHOR("jude84.kim@lge.com");
MODULE_DESCRIPTION("I2C bus driver for fusb301 USB Type-C");
MODULE_LICENSE("GPL v2");
