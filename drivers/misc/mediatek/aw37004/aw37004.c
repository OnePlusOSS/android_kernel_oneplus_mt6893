#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <soc/oplus/system/oplus_project.h>
#include "aw37004.h"

#define AW37004_ID  0x64
#define AW37004_ID1  0x55
#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2

#define AW37004_NAME "awinic,aw37004"
#define AW37004_DRIVER_VERSION		"V0.1.0"

struct aw37004_chip *camera_chip;

static const struct  aw37004_map  aw37004_on_config[] = {
	{0x03, 0x55},
	{0x04, 0x55},
	{0x05, 0x80},
	{0x06, 0x80},
	{0x0E, 0x0F},
	{0x0E, 0x00},
	{0x02, 0x8F},
	{0x02, 0x00},
};


static int aw37004_i2c_write(struct aw37004_chip *chip,
				unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
			i2c_smbus_write_byte_data(chip->client, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
				   ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw37004_i2c_read(struct aw37004_chip *chip,
			   unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(chip->client, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
				ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

int aw37004_camera_power_down_all(void)
{
	int ret = -1;
	ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, 0);//bit1
	if (ret < 0) {
		pr_err("AW37004 set enable failed\n");
		return ret;
	}
	return ret;
}

int aw37004_camera_power_up(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;
	struct mutex aw37004_mutex;
	mutex_init(&aw37004_mutex);

	if (camera_chip == NULL) {
		pr_info("AW37004 probe fail the camera_chip is NULL\n");
		return ret;
	}
	switch (out_iotype) {
	case OUT_DVDD1:
		//macro ov5675
		pr_info("AW37004 ultra & macro: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		if (is_project(23707) || is_project(23709)) {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_DVDD1].reg, 0x4B);
		}
		else {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_DVDD1].reg, 0x55);//bit0
		}
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set dvdd1 failed\n");
			break;
		}

		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 read enable failed\n");
			break;
		}

		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0001;//bit 0
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		pr_info("AW37004 set OUT_DVDD1 success!");
		break;

	case OUT_DVDD2:

		pr_info("AW37004 imx471: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		if (is_project(23707) || is_project(23709)) {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_DVDD2].reg, 0x55);
		}
		else {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_DVDD2].reg, 0x55);//bit1
		}

		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set dvdd2 failed\n");
			break;
		}

		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("xyz AW37004 read enable failed\n");
			break;
		}
		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0010;//bit1
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		pr_info("AW37004 set OUT_DVDD2 success!");
		break;

	case OUT_AVDD1:
		//wide s5kgw3/s5khm2
		pr_info("AW37004 wide: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		if (is_project(23707) || is_project(23709)) {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_AVDD1].reg, 0x80);
		}
		else {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_AVDD1].reg, 0x80);//bit2
		}
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set avdd1 failed\n");
			break;
		}

		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 read enable failed\n");
			break;
		}

		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0100;//bit2
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		pr_info("AW37004 set OUT_AVDD1 success!");
		break;

	case OUT_AVDD2:
		//ultra imx355
		pr_info("AW37004 ultra: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		if (is_project(23707) || is_project(23709)) {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_AVDD2].reg, 0x80);
		}
		else {
			ret = aw37004_i2c_write(camera_chip, aw37004_on_config[OUT_AVDD2].reg, 0x80);//bit3
		}
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set avdd1 failed\n");
			break;
		}

		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 read enable failed\n");
			break;
		}

		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b1000;//bit3
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit3
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		pr_info("AW37004 set OUT_AVDD2 success!");
		break;
	default:
		pr_err("AW37004 unknown port!!!\n");
		break;
	}
	pr_info("AW37004 result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(aw37004_camera_power_up);

int aw37004_camera_power_down(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;
	struct mutex aw37004_mutex;
	mutex_init(&aw37004_mutex);
	if (camera_chip == NULL) {
		pr_info("AW37004 probe fail the camera_chip is NULL\n");
		return ret;
	}
	switch (out_iotype) {
	case OUT_DVDD1:
		//macro ov5675
		pr_info("aw37004 ultra & macro: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("aw37004 read enable failed\n");
			break;
		}
		pr_info("aw37004 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b1110;//bit0
		pr_info("aw37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("aw37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		break;

	case OUT_DVDD2:
		//front imx471
		pr_info("aw37004 imx471: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("aw37004 read enable failed\n");
			break;
		}

		pr_info("aw37004 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b1101;//bit1
		pr_info("aw37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("aw37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		break;

	case OUT_AVDD1:
		//wide s5kgw3/s5khm2
		pr_info("AW37004 wide: aw37004 out put type is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 read enable failed\n");
			break;
		}

		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b1011;//bit2
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		break;

	case OUT_AVDD2:
		//ultra imx355
		pr_info("AW37004 ultra: aw37004 output iotype is [%d]", out_iotype);
		mutex_lock(&aw37004_mutex);
		ret = aw37004_i2c_read(camera_chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 read enable failed\n");
			break;
		}

		pr_info("AW37004 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b0111;//bit3
		pr_info("AW37004 after  set enable value = 0x%x\n", reg_val);

		ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&aw37004_mutex);
			pr_err("AW37004 set enable failed\n");
			break;
		}
		mutex_unlock(&aw37004_mutex);
		break;
	default:
		pr_info("AW37004 unknown camera!!!\n");
		break;
	}
	pr_info("AW37004 result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(aw37004_camera_power_down);


static ssize_t aw37004_vol_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw37004_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	aw37004_i2c_read(chip, aw37004_on_config[VOL_ENABLE].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t aw37004_vol_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct aw37004_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = aw37004_i2c_write(chip, aw37004_on_config[VOL_ENABLE].reg, reg_val);
	if (ret < 0) {
		pr_err("AW37004 set enable failed\n");
		return ret;
	}
	return count;
}


static ssize_t aw37004_out_avdd2_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw37004_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	aw37004_i2c_read(chip, aw37004_on_config[OUT_AVDD2].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t aw37004_out_avdd2_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct aw37004_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = aw37004_i2c_write(chip, aw37004_on_config[OUT_AVDD2].reg, reg_val);
	if (ret < 0) {
		pr_err("AW37004 open avdd2 failed\n");
		return ret;
	}
	return count;
}


static ssize_t aw37004_out_avdd1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw37004_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	aw37004_i2c_read(chip, aw37004_on_config[OUT_AVDD1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t aw37004_out_avdd1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct aw37004_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = aw37004_i2c_write(chip, aw37004_on_config[OUT_AVDD1].reg, reg_val);
	if (ret < 0) {
		pr_err("AW37004 open avdd1 failed\n");
		return ret;
	}
	return count;
}


static ssize_t aw37004_out_dvdd2_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw37004_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	aw37004_i2c_read(chip, aw37004_on_config[OUT_DVDD2].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t aw37004_out_dvdd2_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct aw37004_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = aw37004_i2c_write(chip, aw37004_on_config[OUT_DVDD2].reg, reg_val);
	if (ret < 0)	{
		pr_err("AW37004 open dvdd2 failed\n");
		return ret;
	}
	return count;
}


static ssize_t aw37004_out_dvdd1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct aw37004_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	aw37004_i2c_read(chip, aw37004_on_config[OUT_DVDD1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t aw37004_out_dvdd1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct aw37004_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = aw37004_i2c_write(chip, aw37004_on_config[OUT_DVDD1].reg, reg_val);
	if (ret < 0)	{
		pr_err("AW37004 open dvdd1 failed\n");
		return ret;
	}
	return count;
}

static DEVICE_ATTR(vol_enable, 0664, aw37004_vol_enable_show, aw37004_vol_enable_store);
static DEVICE_ATTR(out_avdd2, 0664, aw37004_out_avdd2_show, aw37004_out_avdd2_store);
static DEVICE_ATTR(out_avdd1, 0664, aw37004_out_avdd1_show, aw37004_out_avdd1_store);
static DEVICE_ATTR(out_dvdd2, 0664, aw37004_out_dvdd2_show, aw37004_out_dvdd2_store);
static DEVICE_ATTR(out_dvdd1, 0664, aw37004_out_dvdd1_show, aw37004_out_dvdd1_store);

static struct attribute *aw37004_attributes[] = {
	&dev_attr_out_dvdd1.attr,
	&dev_attr_out_dvdd2.attr,
	&dev_attr_out_avdd1.attr,
	&dev_attr_out_avdd2.attr,
	&dev_attr_vol_enable.attr,
	NULL
};

static struct attribute_group aw37004_attribute_group = {
	.attrs = aw37004_attributes
};



static int aw37004_get_id(struct  aw37004_chip *chip)
{
	unsigned char reg_val = 0;
	int ret = 0;
	ret = aw37004_i2c_write(chip, aw37004_on_config[OUT_DVDD1].reg, 0x64);
	if (ret < 0) {
		pr_err("AW37004_get_id failed\n");
		return ret;
	}

	aw37004_i2c_read(chip, aw37004_on_config[OUT_DVDD1].reg, &reg_val);
	pr_info("%s:AW37004 id is %d\n", __func__, reg_val);

	if ((reg_val != AW37004_ID) && (reg_val != AW37004_ID1)) {
		ret = -1;
		return ret;
	}
	return 0;
}

static int set_init_voltage(struct aw37004_chip *chip)
{
	int ret = 0;
	int i;

	for (i = 0 ; i < (ARRAY_SIZE(aw37004_on_config) - 4); i++) {
		ret = aw37004_i2c_write(chip, aw37004_on_config[i].reg, aw37004_on_config[i].value);
		if (ret < 0) {
			pr_err("AW37004 init voltage failed\n");
			return ret;
		}
	}
	//enable dischager function
	ret = aw37004_i2c_write(chip, aw37004_on_config[DISCHARGE_ENABLE].reg, aw37004_on_config[DISCHARGE_ENABLE].value);
	if (ret < 0) {
		pr_err("AW37004  dischager function enable failed\n");
		return ret;
	}
	return 0;
}

int aw37004_camera_power_up_eeprom(void)
{
	int ret = -1;
	if (camera_chip == NULL) {
		pr_err("AW37004 probe fail the camera_chip is NULL\n");
		return ret;
	}
	ret = set_init_voltage(camera_chip);
	if (ret < 0) {
		pr_err("set_init_voltage failed\n");
		return ret;
	}
	ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, aw37004_on_config[VOL_ENABLE].value);
	if (ret < 0) {
		pr_err("AW37004 set VOL_ENABLE failed\n");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(aw37004_camera_power_up_eeprom);

int aw37004_camera_power_up_all(void)
{
	int ret = -1;
	if (camera_chip == NULL) {
		pr_err("AW37004 probe fail the camera_chip is NULL\n");
		return ret;
	}
	ret = set_init_voltage(camera_chip);
	if (ret < 0) {
		pr_err("set_init_voltage failed\n");
		return ret;
	}
	ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, aw37004_on_config[VOL_ENABLE].value);
	if (ret < 0) {
		pr_err("AW37004 set VOL_ENABLE failed\n");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(aw37004_camera_power_up_all);

int aw37004_camera_power_down_eeprom(void)
{
	int ret = -1;
	ret = aw37004_i2c_write(camera_chip, aw37004_on_config[VOL_ENABLE].reg, 0);//bit1
	return ret;
}
EXPORT_SYMBOL(aw37004_camera_power_down_eeprom);

void aw37004_print_reg(struct  aw37004_chip *chip)
{
	int i;
	unsigned char reg_val = 0;

	for (i = 0 ; i < ARRAY_SIZE(aw37004_on_config); i++) {
		aw37004_i2c_read(chip, aw37004_on_config[i].reg, &reg_val);
		pr_info("%s:AW37004 info is reg %d, value %d\n", __func__, aw37004_on_config[i].reg, reg_val);
	}

}

static int aw37004_init(struct  aw37004_chip *chip)
{
	int ret = 0;

	/*chip->en_gpio = of_get_named_gpio(chip->dev->of_node,
			 "en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		pr_err("%s:%d, en gpio not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	pr_info("%s: en_gpio is %d\n", __func__, chip->en_gpio);
	ret = gpio_request(chip->en_gpio, "AW37004_en");
	if (ret < 0) {
			pr_err("AW37004 enable gpio request failed\n");
			return ret;
	}

	gpio_direction_output(chip->en_gpio, 0);*/

	msleep(10);

	ret = aw37004_get_id(chip);
	if (ret < 0) {
		pr_err("AW37004 read id failed\n");
		return ret;
	}

	ret = set_init_voltage(chip);
	if (ret < 0)
		pr_err("AW37004 init failed\n");

	if (AW37004_DEBUG) {
		msleep(10);
		aw37004_print_reg(chip);
	}

	return 0;
}

/*
static int aw37004_disable_power(struct  aw37004_chip *chip)
{
	int ret = 0;

	ret = regulator_disable(chip->vin1);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin1:%d\n", ret);

	if (!regulator_is_enabled(chip->vin1)) {
		ret = regulator_set_voltage(chip->vin1, 0, VIN1_1P35_VOL_MAX);
		if (ret)
			dev_err(chip->dev,
				"Unable to set (0) voltage for vin1:%d\n", ret);
	}

	ret = regulator_disable(chip->vin2);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin2: %d\n", ret);

	if (!regulator_is_enabled(chip->vin2)) {
		ret = regulator_set_voltage(chip->vin2, 0, VIN2_3P3_VOL_MAX);
		if (ret)
			dev_err(chip->dev,
				"Unable to set (0) voltage for vin2:%d\n", ret);
	}

	return 0;
}
*/

/*
static int aw37004_enable_power(struct  aw37004_chip *chip)
{
	int ret = 0;

	ret = regulator_set_voltage(chip->vin1, VIN1_1P35_VOL_MIN,
						VIN1_1P35_VOL_MAX);
	if (ret) {
		dev_err(chip->dev, "Unable to set voltage for vin1:%d\n", ret);
		goto put_vin1;
	}

	ret = regulator_enable(chip->vin1);
	if (ret) {
		dev_err(chip->dev, "Unable to enable vin1:%d\n", ret);
		goto unset_vin1;
	}

	ret = regulator_set_voltage(chip->vin2, VIN2_3P3_VOL_MIN,
						VIN2_3P3_VOL_MAX);
	if (ret) {
		dev_err(chip->dev, "Unable to set voltage for vin2:%d\n", ret);
		goto disable_vin1;
	}

	ret = regulator_enable(chip->vin2);
	if (ret) {
		dev_err(chip->dev, "Unable to enable vin2:%d\n", ret);
		goto unset_vin2;
	}
	return 0;

unset_vin2:
	ret = regulator_set_voltage(chip->vin2, 0, VIN2_3P3_VOL_MAX);
	if (ret)
		dev_err(chip->dev, "Unable to set (0) voltage for vin2:%d\n", ret);

disable_vin1:
	ret = regulator_disable(chip->vin1);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin1:%d\n", ret);

unset_vin1:
	ret = regulator_set_voltage(chip->vin1, 0, VIN1_1P35_VOL_MAX);
	if (ret)
		dev_err(chip->dev,
			"Unable to set (0) voltage for vin1:%d\n", ret);

put_vin1:
	return ret;
}
*/
/*
static int aw37004_vreg_init(struct  aw37004_chip *chip)
{
	int ret = 0;

	chip->vin1 = devm_regulator_get(chip->dev, "vin1");

	if (IS_ERR(chip->vin1)) {
		ret = PTR_ERR(chip->vin1);
		dev_err(chip->dev, "%s: can't get VIN1,%d\n", __func__, ret);
		goto err_vin1;
	}

	chip->vin2 = devm_regulator_get(chip->dev, "vin2");

	if (IS_ERR(chip->vin2)) {
		ret = PTR_ERR(chip->vin2);
		dev_err(chip->dev, "%s: can't get VIN2,%d\n", __func__, ret);
		goto err_vin2;
	}

	return 0;

err_vin2:
	//devm_regulator_put(chip->vin1);
err_vin1:
	return ret;
}
*/
static int aw37004_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw37004_chip *chip;

	pr_info("%s,enrty\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(struct aw37004_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_mem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check_functionality failed\n");
		ret = -EIO;
		goto  init_err;
	}

	chip->client = client;

	chip->dev = &client->dev;
	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	/*ret = aw37004_vreg_init(chip);
	if (ret < 0)	{
		dev_err(&client->dev, "get vreg failed\n");
		goto vreg_init_err;
	}

	ret = aw37004_enable_power(chip);
	if (ret) {
		dev_err(&client->dev, "enable power failed\n");
		ret = -1;
		goto vreg_enable_err;
	}*/
	ret = aw37004_init(chip);

	if (ret < 0) {
		dev_err(&client->dev, "AW37004 init fail!\n");
		ret = -ENODEV;
		goto init_err;
	}

	ret = sysfs_create_group(&client->dev.kobj,
					   &aw37004_attribute_group);

	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	camera_chip = chip;

	pr_info("%s,successfully\n", __func__);
	return 0;
err_sysfs:
init_err:
//vreg_enable_err:
	//devm_regulator_put(chip->vin1);
	//devm_regulator_put(chip->vin2);
//vreg_init_err:
	//devm_kfree(chip->dev, chip);
	//chip = NULL;
err_mem:
	return ret;
}

static int aw37004_remove(struct i2c_client *client)
{
	struct aw37004_chip *chip = i2c_get_clientdata(client);

	devm_kfree(chip->dev, chip);
	chip = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int aw37004_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw37004_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	pr_info("%s\n", __func__);
	ret = aw37004_i2c_write(chip, aw37004_on_config[VOL_DISABLE].reg, aw37004_on_config[VOL_DISABLE].value);
	if (ret < 0)
	   pr_err("aw37004 close voltage failed\n");

	//aw37004_disable_power(chip);
	return 0;
}

static int aw37004_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct aw37004_chip *chip = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	//aw37004_enable_power(chip);
	//gpio_direction_output(chip->en_gpio, 0);
	/*
	ret = aw37004_i2c_write(chip, aw37004_on_config[VOL_ENABLE].reg, aw37004_on_config[VOL_ENABLE].value);
	if (ret < 0) {
		pr_err("aw37004 set enable failed\n");
		return ret;
	}
	*/
	return 0;
}
#endif

static const struct dev_pm_ops aw37004_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(aw37004_suspend, aw37004_resume)
};

static const struct i2c_device_id aw37004_id_table[] = {
	{AW37004_NAME, 0},
	{} /* NULL terminated */
};

MODULE_DEVICE_TABLE(i2c, aw37004_id_table);


#ifdef CONFIG_OF
static const struct of_device_id aw37004_i2c_of_match_table[] = {
		{ .compatible = AW37004_NAME },
		{},
};
MODULE_DEVICE_TABLE(of, aw37004_i2c_of_match_table);
#endif

static struct i2c_driver aw37004_driver = {
	.driver = {
		.name = AW37004_NAME,
		.pm = &aw37004_pm_ops,
		.of_match_table = of_match_ptr(aw37004_i2c_of_match_table),
		},
	.probe = aw37004_probe,
	.remove = aw37004_remove,
	.id_table = aw37004_id_table,
};

static int __init aw37004_i2c_init(void)
{
	pr_info("aw37004 driver version %s\n", AW37004_DRIVER_VERSION);
	return i2c_add_driver(&aw37004_driver);
}
subsys_initcall(aw37004_i2c_init);

static void __exit aw37004_i2c_exit(void)
{
	i2c_del_driver(&aw37004_driver);
}
module_exit(aw37004_i2c_exit);

MODULE_DESCRIPTION("AW37004 driver");
MODULE_AUTHOR("awinic,lnc.");
MODULE_LICENSE("GPL");
