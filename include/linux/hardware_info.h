/***********************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - hardware_info.h
** Description: headerfile  for hardware infomation
**
** Version: 1.0
** Date : 2018/08/11
**
** ------------------------------- Revision History: -------------------------------
**  	<author>		<data> 	   <version >	       <desc>
**      Jinfan.Hu       2018/08/11     1.0               headerfile  for hardware infomation
**
****************************************************************/
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#define HARDWARE_MAX_ITEM_LONGTH		64

enum{
	HARDWARE_LCD = 0,
	HARDWARE_TP,
	HARDWARE_FLASH,
	HARDWARE_FRONT_CAM,
	HARDWARE_FRONT_SUB_CAM,
	HARDWARE_BACK_CAM,
	HARDWARE_BACK_SUB_CAM,
#ifdef ODM_WT_EDIT
	HARDWARE_WIDE_ANGLE_CAM,
	HARDWARE_MONO_CAM,
#endif /* ODM_WT_EDIT */
	HARDWARE_BT,
	HARDWARE_WIFI,
	HARDWARE_ACCELEROMETER,
	HARDWARE_ALSPS,
	HARDWARE_GYROSCOPE,
	HARDWARE_MAGNETOMETER,
	HARDWARE_GPS,
	HARDWARE_FM,
	HARDWARE_NFC,
	HARDWARE_BATTERY_ID,
	HARDWARE_BACK_CAM_MOUDULE_ID,
	HARDWARE_FRONT_CAM_MOUDULE_ID,
	HARDWARE_BACK_SUB_CAM_MOUDULE_ID,
	HARDWARE_FRONT_SUB_CAM_MOUDULE_ID,
#ifdef ODM_WT_EDIT
	HARDWARE_WIDE_ANGLE_CAM_MOUDULE_ID,
	HARDWARE_MONO_CAM_MOUDULE_ID,
#endif /* ODM_WT_EDIT */
	HARDWARE_BACK_CAM_EFUSEID,
	HARDWARE_BCAK_SUBCAM_EFUSEID,
	HARDWARE_FRONT_CAME_EFUSEID,
	HARDWARE_BACK_CAM_SENSORID,
	HARDWARE_BACK_SUBCAM_SENSORID,
	HARDWARE_FRONT_CAM_SENSORID,
#ifdef ODM_WT_EDIT
	HARDWARE_MONO_CAM_SENSORID,
	HARDWARE_WIDE_ANGLE_CAM_SENSORID,
#endif /* ODM_WT_EDIT */
	HARDWARE_BOARD_ID,
	HARDWARE_HARDWARE_ID,
	HARDWARE_MAX_ITEM
};


#define HARDWARE_ID                                 'H'
#define HARDWARE_LCD_GET                            _IOWR(HARDWARE_ID, 0x01, char[HARDWARE_MAX_ITEM_LONGTH])            /*  LCD*/
#define HARDWARE_TP_GET                             _IOWR(HARDWARE_ID, 0x02, char[HARDWARE_MAX_ITEM_LONGTH])            /*  TP */
#define HARDWARE_FLASH_GET                          _IOWR(HARDWARE_ID, 0x03, char[HARDWARE_MAX_ITEM_LONGTH])            /*  FLASH */
#define HARDWARE_FRONT_CAM_GET                      _IOWR(HARDWARE_ID, 0x04, char[HARDWARE_MAX_ITEM_LONGTH])            /*  sub camera */
#define HARDWARE_BACK_CAM_GET                       _IOWR(HARDWARE_ID, 0x05, char[HARDWARE_MAX_ITEM_LONGTH])            /*  main camera */
#define HARDWARE_ACCELEROMETER_GET                  _IOWR(HARDWARE_ID, 0x06, char[HARDWARE_MAX_ITEM_LONGTH])            /*  accel */
#define HARDWARE_ALSPS_GET                          _IOWR(HARDWARE_ID, 0x07, char[HARDWARE_MAX_ITEM_LONGTH])            /*  romixity */
#define HARDWARE_GYROSCOPE_GET                      _IOWR(HARDWARE_ID, 0x08, char[HARDWARE_MAX_ITEM_LONGTH])            /*  gyro */
#define HARDWARE_MAGNETOMETER_GET                   _IOWR(HARDWARE_ID, 0x09, char[HARDWARE_MAX_ITEM_LONGTH])            /*  magicmate */
#define HARDWARE_BT_GET                             _IOWR(HARDWARE_ID, 0x10, char[HARDWARE_MAX_ITEM_LONGTH])            /*  bt */
#define HARDWARE_WIFI_GET                           _IOWR(HARDWARE_ID, 0x11, char[HARDWARE_MAX_ITEM_LONGTH])            /*  WIFI */
#define HARDWARE_GPS_GET                            _IOWR(HARDWARE_ID, 0x12, char[HARDWARE_MAX_ITEM_LONGTH])            /*  GPS */
#define HARDWARE_FM_GET                             _IOWR(HARDWARE_ID, 0x13, char[HARDWARE_MAX_ITEM_LONGTH])            /*  FM */
#define HARDWARE_BATTERY_ID_GET                     _IOWR(HARDWARE_ID, 0x15, char[HARDWARE_MAX_ITEM_LONGTH])            /*  battery */
#define HARDWARE_BACK_CAM_MOUDULE_ID_GET            _IOWR(HARDWARE_ID, 0x16, char[HARDWARE_MAX_ITEM_LONGTH])            /*  main camera module */
#define HARDWARE_FRONT_CAM_MODULE_ID_GET            _IOWR(HARDWARE_ID, 0x17, char[HARDWARE_MAX_ITEM_LONGTH])            /*  subcamera module */
#define HARDWARE_BOARD_ID_GET                       _IOWR(HARDWARE_ID, 0x18, char[HARDWARE_MAX_ITEM_LONGTH])            /*  board id */
#define HARDWARE_FRONT_FISH_CAM_GET                 _IOWR(HARDWARE_ID, 0x19, char[HARDWARE_MAX_ITEM_LONGTH])            /*  front fisheye camera */
#define HARDWARE_BACK_FISH_CAM_GET                  _IOWR(HARDWARE_ID, 0x1A, char[HARDWARE_MAX_ITEM_LONGTH])            /*  back fish eye camera */
#define HARDWARE_HALL_GET                           _IOWR(HARDWARE_ID, 0x1B, char[HARDWARE_MAX_ITEM_LONGTH])            /*  hall sensor */
#define HARDWARE_PRESSURE_GET                       _IOWR(HARDWARE_ID, 0x1C, char[HARDWARE_MAX_ITEM_LONGTH])            /*  presure */
#define HARDWARE_NFC_GET                            _IOWR(HARDWARE_ID, 0x1D, char[HARDWARE_MAX_ITEM_LONGTH])            /*  NFC */
#define HARDWARE_FRONT_SUBCAM_GET                   _IOWR(HARDWARE_ID, 0x1E, char[HARDWARE_MAX_ITEM_LONGTH]) /*  dual front camera，front sec-camera */
#define HARDWARE_BACK_SUBCAM_GET                    _IOWR(HARDWARE_ID, 0x1F, char[HARDWARE_MAX_ITEM_LONGTH])            /*  dual back camera，back sec-camera */
#define HARDWARE_FRONT_FISH_CAM_MOUDULE_ID_GET      _IOWR(HARDWARE_ID, 0x20, char[HARDWARE_MAX_ITEM_LONGTH])            /*  fisheye camera module */
#define HARDWARE_BACK_FISH_CAM_MOUDULE_ID_GET       _IOWR(HARDWARE_ID, 0x21, char[HARDWARE_MAX_ITEM_LONGTH])		/*  back fisheye camera module */
#define HARDWARE_HARDWARE_ID_GET                    _IOWR(HARDWARE_ID, 0x22, char[HARDWARE_MAX_ITEM_LONGTH])		/*  hardwareid */

#define HARDWARE_FRONT_SUBCAM_MODULEID_GET          _IOWR(HARDWARE_ID, 0x23, char[HARDWARE_MAX_ITEM_LONGTH]) /*  dual front camera，front sec-camera module*/
#define HARDWARE_BACK_SUBCAM_MODULEID_GET           _IOWR(HARDWARE_ID, 0x24, char[HARDWARE_MAX_ITEM_LONGTH]) /*  dual back camera，back sec-camera module */
#define HARDWARE_BACK_CAM_EFUSEID_GET               _IOWR(HARDWARE_ID, 0x25, char[HARDWARE_MAX_ITEM_LONGTH])            /*  back camera efuseId*/
#define HARDWARE_BCAK_SUBCAM_EFUSEID_GET            _IOWR(HARDWARE_ID, 0x26, char[HARDWARE_MAX_ITEM_LONGTH])            /*  back sec-camera efuseId*/
#define HARDWARE_FRONT_CAME_EFUSEID_GET             _IOWR(HARDWARE_ID, 0x27, char[HARDWARE_MAX_ITEM_LONGTH])			/*  fron camera efuseId*/
#define HARDWARE_BACK_CAM_SENSORID_GET        		_IOWR(HARDWARE_ID, 0x29, char[HARDWARE_MAX_ITEM_LONGTH])        	/*  back camera sensro_ID*/
#define HARDWARE_BACK_SUBCAM_SENSORID_GET           _IOWR(HARDWARE_ID, 0x30, char[HARDWARE_MAX_ITEM_LONGTH])            /*  back subcamera efuseId*/
#define HARDWARE_FRONT_CAM_SENSORID_GET             _IOWR(HARDWARE_ID, 0x31, char[HARDWARE_MAX_ITEM_LONGTH])        	/*  front camera efuseId*/
#ifdef ODM_WT_EDIT
#define HARDWARE_WIDE_ANGLE_CAM_GET                 _IOWR(HARDWARE_ID, 0x34, char[HARDWARE_MAX_ITEM_LONGTH])            /*  wide_angle*/
#define HARDWARE_WIDE_ANGLE_CAM_MOUDULE_ID_GET      _IOWR(HARDWARE_ID, 0x35, char[HARDWARE_MAX_ITEM_LONGTH])            /*  wide_angle module*/
#define HARDWARE_WIDE_ANGLE_CAM_SENSORID_GET        _IOWR(HARDWARE_ID, 0x37, char[HARDWARE_MAX_ITEM_LONGTH])            /*  wide_angle sensorId*/
#define HARDWARE_MONO_CAM_GET                       _IOWR(HARDWARE_ID, 0x3C, char[HARDWARE_MAX_ITEM_LONGTH])            /*  mono CAM*/
#define HARDWARE_MONO_CAM_MOUDULE_ID_GET            _IOWR(HARDWARE_ID, 0x3D, char[HARDWARE_MAX_ITEM_LONGTH])            /*  mono cam module*/
#define HARDWARE_MONO_CAM_SENSORID_GET              _IOWR(HARDWARE_ID, 0x3F, char[HARDWARE_MAX_ITEM_LONGTH])            /*  mono cam sensor*/
#endif /* ODM_WT_EDIT */
#define HARDWARE_BACK_CAM_MOUDULE_ID_SET            _IOWR(HARDWARE_ID, 0x81, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FRONT_CAM_MODULE_ID_SET            _IOWR(HARDWARE_ID, 0x82, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BACK_SUBCAM_MODULE_ID_SET          _IOWR(HARDWARE_ID, 0x83, char[HARDWARE_MAX_ITEM_LONGTH])



int hardwareinfo_set_prop(int cmd, const char *name);
void hardwareinfo_tp_register(void (*fn)(void *), void *driver_data);
#endif /*__HARDWARE_H__*/
