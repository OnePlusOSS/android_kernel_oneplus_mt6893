/********************************************
 ** Copyright (C) 2018 OPLUS Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: lcd_bias.h
 ** Description: Source file for LCD bias
 **          To Control LCD bias voltage
 ** Version:1.0
 ** Date : 2018/10/03
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2018/10/03   Liyan@ODM_HQ   Source file for LCD bias
 ********************************************/

#ifndef _LCD_BIAS_H
#define _LCD_BIAS_H


#define LCD_BIAS_VPOS_ADDR    0x00
#define LCD_BIAS_VNEG_ADDR    0x01
#define LCD_BIAS_APPS_ADDR    0x03
#define LCD_BIAS_APPS_MTP_ADDR 0xFF
#define NEG_OUTPUT_APPS 0x40

#ifdef BUILD_LK
#ifdef CONFIG_MACH_MT6771
#define LCD_BIAS_I2C_BUSNUM   3	/* for I2C channel 3 */
#define LCD_BIAS_I2C_ADDR       0x3E /*for I2C slave dev addr*/

#define LCD_BIAS_ST_MODE         0
#define LCD_BIAS_MAX_ST_MODE_SPEED 100  /* khz */

#define GPIO_LCD_BIAS_ENP_PIN (GPIO23 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN (GPIO21 | 0x80000000)
#endif
#ifdef CONFIG_MACH_MT6769
#define LCD_BIAS_I2C_BUSNUM   0	/* for I2C channel 0 */
#define LCD_BIAS_I2C_ADDR       0x3E /*for I2C slave dev addr*/

#define LCD_BIAS_ST_MODE         0
#define LCD_BIAS_MAX_ST_MODE_SPEED 100  /* khz */

#define GPIO_LCD_BIAS_ENP_PIN (GPIO169 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN (GPIO165 | 0x80000000)
#else
#define LCD_BIAS_I2C_BUSNUM   6	/* for I2C channel 6 */
#define LCD_BIAS_I2C_ADDR       0x3E /*for I2C slave dev addr*/

#define LCD_BIAS_ST_MODE         0
#define LCD_BIAS_MAX_ST_MODE_SPEED 100  /* khz */

#define GPIO_LCD_BIAS_ENP_PIN (GPIO23 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN (GPIO202 | 0x80000000)
#endif

#define LCD_BIAS_PRINT printf

#else

#define LCD_BIAS_PRINT printk
/* #define LCD_BIAS_PRINT(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args) */

/* DTS state */
typedef enum {
	LCD_BIAS_GPIO_STATE_ENP0,
	LCD_BIAS_GPIO_STATE_ENP1,
	LCD_BIAS_GPIO_STATE_ENN0,
	LCD_BIAS_GPIO_STATE_ENN1,
	LCD_BIAS_GPIO_STATE_MAX,	/* for array size */
} LCD_BIAS_GPIO_STATE;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
void lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE s);
#endif

typedef enum {
	FIRST_VSP_AFTER_VSN = 0,
	FIRST_VSN_AFTER_VSP = 1
} LCD_BIAS_POWER_ON_SEQUENCE;

#ifndef CONFIG_EXT_LCD_BIAS_ODM_HQ
extern void pmi_lcd_bias_set_vspn_vol(unsigned int value);
extern void pmi_lcd_bias_set_vspn_en(unsigned int en, unsigned int seq);
#endif

void lcd_bias_set_vspn(unsigned int en, unsigned int seq, unsigned int value);

#endif /* _LCD_BIAS_H */
