/* OPLUS Mobile Comm Corp
 *
 * yixue.ge add for oplus project
 *
 *
 */
#ifndef _OPLUS_CUSTOME_H_
#define _OPLUS_CUSTOME_H_

//must same as perload define
#define D_OPLUS_CUST_PART_MAGIC_NUM              (0x6F70706F)
#define D_OPLUS_CUST_PART_CONFIG_MAGIC_NUM       (0x636F6E66)

#define PROJECT_BUF_SIZE 64
#define SENSOR_BUF_SIZE 256
#define DOWNLOADTIME_BUF_SIZE 256
#define OPLUSCAMERA_BUF_SIZE 25600

#define DIGEST_ID_IN_PHONE_SIZE 64
#define DIGEST_ID_IN_PHONE_CHECKSUM_DATA_SIZE 2
#define UNLOCK_SYNBOL_SIZE 2048
#define UNLOCK_SYNBOL_CHECKSUM_SIZE  2

#define RPMB_ENABLE_MAGIC 0x9621
#define RPMB_KEY_PROVISIONED 0x594553
#define RPMB_KEY_NOT_PROVISIONED 0x4e4f

typedef struct
{
	unsigned int		nMagicNum1;
	unsigned int		nMagicNum2;
	unsigned int		nPlUsbEnumEnabled;
	unsigned int		nUsbAutoSwitch;
	unsigned char 		project[PROJECT_BUF_SIZE];
	unsigned char		Sensor[SENSOR_BUF_SIZE];
	unsigned char		DownloadTime[DOWNLOADTIME_BUF_SIZE];
	unsigned int		rpmb_enable;
	unsigned int 		rpmb_key_provisioned;
	unsigned char		OplusCamera[OPLUSCAMERA_BUF_SIZE];
}TOplusCustConfigInf;

#endif
