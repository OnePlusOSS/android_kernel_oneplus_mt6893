/*
 * Copyright (C) 2016 MediaTek Inc.
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


unsigned short imx709_21015_seamless_normal_video[] = {
//Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
//Frame Length Lines Setting
    0x0340, 0x0F,
    0x0341, 0x30,
//Other Setting
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A97, 0x34,
    0x3A9B, 0x34,
    0x3AA1, 0x8B,
    0x3AA5, 0x8B,
//Integration Setting
    0x0202, 0x0F,
    0x0203, 0x18,
    0x0224, 0x01,
    0x0225, 0xF4,
//DOL Setting
    0x3170, 0x00,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
};

unsigned short imx709_21015_seamless_custom4[] = {
//Line Length PCK Setting
    0x0342, 0x1D,
    0x0343, 0x20,
//Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x88,
//Other Setting
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A97, 0x88,
    0x3A9B, 0x88,
    0x3AA1, 0xAC,
    0x3AA5, 0xAC,
//Integration Setting
    0x0202, 0x06,
    0x0203, 0x80,
    0x0224, 0x00,
    0x0225, 0xD0,
//DOL Setting
    0x3170, 0x01,
//Global setting
    0x3207, 0x00, //Turn on the QSC Setting
};
