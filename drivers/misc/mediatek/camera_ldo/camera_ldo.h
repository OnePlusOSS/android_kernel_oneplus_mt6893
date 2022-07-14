#ifndef _CAMERA_LDO_H_
#define _CAMERA_LDO_H_

#define CAMERA_LDO_CHIP_REV_ADDR    0x00
#define CAMERA_LDO_CL_ADDR          0x01
#define CAMERA_LDO_DIC_ADDR         0x02

#define CAMERA_LDO_LDO1_OUT_ADDR    0x03 //DVDD1
#define CAMERA_LDO_LDO2_OUT_ADDR    0x04 //DVDD2
#define CAMERA_LDO_LDO3_OUT_ADDR    0x05 //AVDD1
#define CAMERA_LDO_LDO4_OUT_ADDR    0x06 //AVDD2
#define CAMERA_LDO_LDO5_OUT_ADDR    0x07 //VDDAF
#define CAMERA_LDO_LDO6_OUT_ADDR    0x08 //VDDOIS
#define CAMERA_LDO_LDO7_OUT_ADDR    0x09 //VDDIO

#define CAMERA_LDO_LDO21_SEQ_ADDR    0x0A
#define CAMERA_LDO_LDO43_SEQ_ADDR    0x0B
#define CAMERA_LDO_LDO65_SEQ_ADDR    0x0C
#define CAMERA_LDO_LDO07_SEQ_ADDR    0x0D

#define CAMERA_LDO_LDO_EN_ADDR          0x0E  //bit0:LDO1 ~ bit6:LDO7
#define CAMERA_LDO_LDO_SEQ_CTRL_ADDR    0x0F

#define CAMERA_LDO_PRINT printk


typedef enum {
	CAMERA_LDO_DVDD1=0,
	CAMERA_LDO_DVDD2,
	CAMERA_LDO_AVDD1,
	CAMERA_LDO_AVDD2,
	CAMERA_LDO_VDDAF,
	CAMERA_LDO_VDDOIS,
	CAMERA_LDO_VDDIO,
	CAMERA_LDO_MAX
} CAMERA_LDO_SELECT;


/* DTS state */
typedef enum {
	CAMERA_LDO_GPIO_STATE_ENP0,
	CAMERA_LDO_GPIO_STATE_ENP1,
	CAMERA_LDO_GPIO_STATE_MAX,/* for array size */
} CAMERA_LDO_GPIO_STATE;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
extern void camera_ldo_gpio_select_state(CAMERA_LDO_GPIO_STATE s);
extern void camera_ldo_set_ldo_value(CAMERA_LDO_SELECT ldonum,unsigned int value);
extern void camera_ldo_set_en_ldo(CAMERA_LDO_SELECT ldonum,unsigned int en);



#endif /* _CAMERA_LDO_H_*/
