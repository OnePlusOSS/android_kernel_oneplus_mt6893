/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __MT6360_CUSTOM_CTRL_H__
#define __MT6360_CUSTOM_CTRL_H__

#define MT6360_LEVEL_NUM_20817 33
#define MT6360_LEVEL_TORCH_20817 17
#define MT6360_LEVEL_FLASH_20817 MT6360_LEVEL_NUM_20817
#define MT6360_HW_TIMEOUT_20817 2400 /* ms */
#define MT6360_LEVEL_TORCH_21015 (16)
#define MT6360_LEVEL_FLASH_21015 (32)

/******************************************************************************
 * mt6360 20817 custom operations
 *****************************************************************************/
const int mt6360_current_20817[MT6360_LEVEL_NUM_20817] = {
	  25,   50,   63,  75, 100, 125, 150, 175,  200,  225,
	 250,  275,  300, 325, 350, 375, 400, 450,  500,  550,
	 600,  650,  700, 750, 800, 850, 900, 950, 1000, 1050,
	1100, 1150, 1200
};

const unsigned char mt6360_torch_level_20817[MT6360_LEVEL_TORCH_20817] = {
	0x00, 0x02, 0x03, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
	0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};

/* 0x00~0x74 6.25mA/step 0x75~0xB1 12.5mA/step */
const unsigned char mt6360_strobe_level_20817[MT6360_LEVEL_FLASH_20817] = {
	0x00, 0x04, 0x06, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20,
	0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C, 0x44, 0x4C, 0x54,
	0x5C, 0x64, 0x6C, 0x74, 0x78, 0x7C, 0x80, 0x84, 0x88, 0x8C,
	0x90, 0x94, 0x98
};

static const unsigned char mt6360_torch_level_21015[MT6360_LEVEL_TORCH_21015] = {
	0x00, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14,
	0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20
};

/* 0x00~0x74 6.25mA/step 0x75~0xB1 12.5mA/step */
static const unsigned char mt6360_strobe_level_21015[MT6360_LEVEL_FLASH_21015] = {
	0x00, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30,
	0x34, 0x38, 0x3C, 0x44, 0x4C, 0x54, 0x5C, 0x64, 0x6C, 0x74,
	0x78, 0x7C, 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C,
	0xA0, 0xA4
};

#endif
