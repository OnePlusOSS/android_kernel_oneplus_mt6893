#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
//#include <asm/atomic.h>
#include <linux/slab.h>


//#include "kd_camera_hw.h"
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "s5k4h7otp.h"
#include "s5k4h7mipiraw_Sensor.h"



static struct  HLT_S5K4H7_otp_struct *S5K4H7_HLT_OTP;
static void HLT_S5K4H7_read_OTP(struct HLT_S5K4H7_otp_struct *S5K4H7_OTP)
{
	unsigned char val=0;
	int R_temp =0 ,Gr_temp = 0,Gb_temp = 0,B_temp = 0;
	unsigned int year_temp = 0,month_temp = 0,day_temp = 0;

	S5K4H7_HLT_write_cmos_sensor(0x0100,0x01);
	mdelay(5);
	S5K4H7_HLT_write_cmos_sensor(0x0A02,0x15);
	S5K4H7_HLT_write_cmos_sensor(0x3B41,0x01);
	S5K4H7_HLT_write_cmos_sensor(0x3B42,0x03);
	S5K4H7_HLT_write_cmos_sensor(0x3B40,0x01);
	S5K4H7_HLT_write_cmos_sensor(0x0A00,0x01);
	mdelay(5);

	val=S5K4H7_HLT_read_cmos_sensor(0x0A10);//flag of info and awb
	printk("HLT_S5K4H7_read_OTP group1 0x0A10 val=%x\n",val);

	if(val==0x01)
	{
		S5K4H7_OTP->flag = 0x01;
		S5K4H7_OTP->MID = S5K4H7_HLT_read_cmos_sensor(0x0A04);
		S5K4H7_OTP->LID = S5K4H7_HLT_read_cmos_sensor(0x0A0A);
		R_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A12)|(S5K4H7_HLT_read_cmos_sensor(0x0A13)<<8));
		Gr_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A14)|(S5K4H7_HLT_read_cmos_sensor(0x0A15)<<8));
		Gb_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A16)|(S5K4H7_HLT_read_cmos_sensor(0x0A17)<<8));
		B_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A18)|(S5K4H7_HLT_read_cmos_sensor(0x0A19)<<8));
		S5K4H7_OTP->RGr_ratio = R_temp*512/Gr_temp;
		S5K4H7_OTP->BGr_ratio = B_temp*512/Gr_temp;
		S5K4H7_OTP->GbGr_ratio = Gb_temp*512/Gr_temp;
		year_temp = S5K4H7_HLT_read_cmos_sensor(0x0A08);
		month_temp = S5K4H7_HLT_read_cmos_sensor(0x0A07);
		day_temp = S5K4H7_HLT_read_cmos_sensor(0x0A06);
	}else{

		S5K4H7_HLT_write_cmos_sensor(0x0A02,0x17);
		S5K4H7_HLT_write_cmos_sensor(0x3B41,0x01);
		S5K4H7_HLT_write_cmos_sensor(0x3B42,0x03);
		S5K4H7_HLT_write_cmos_sensor(0x3B40,0x01);
		S5K4H7_HLT_write_cmos_sensor(0x0A00,0x01);
		mdelay(5);
		val=S5K4H7_HLT_read_cmos_sensor(0x0A10);//flag of info and awb
		printk("HLT_S5K4H7_read_OTP group2 0x0A10 val=%x\n",val);

		if(val==0x01){

			S5K4H7_OTP->flag = 0x01;
			S5K4H7_OTP->MID = S5K4H7_HLT_read_cmos_sensor(0x0A04);
			S5K4H7_OTP->LID = S5K4H7_HLT_read_cmos_sensor(0x0A0A);
			R_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A12)|(S5K4H7_HLT_read_cmos_sensor(0x0A13)<<8));
			Gr_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A14)|(S5K4H7_HLT_read_cmos_sensor(0x0A15)<<8));
			Gb_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A16)|(S5K4H7_HLT_read_cmos_sensor(0x0A17)<<8));
			B_temp = (S5K4H7_HLT_read_cmos_sensor(0x0A18)|(S5K4H7_HLT_read_cmos_sensor(0x0A19)<<8));
			S5K4H7_OTP->RGr_ratio = R_temp*512/Gr_temp;
			S5K4H7_OTP->BGr_ratio = B_temp*512/Gr_temp;
			S5K4H7_OTP->GbGr_ratio = Gb_temp*512/Gr_temp;
			year_temp = S5K4H7_HLT_read_cmos_sensor(0x0A08);
			month_temp = S5K4H7_HLT_read_cmos_sensor(0x0A07);
			day_temp = S5K4H7_HLT_read_cmos_sensor(0x0A06);
		}else{
			S5K4H7_OTP->flag=0x00;
			S5K4H7_OTP->MID =0x00;
			S5K4H7_OTP->LID = 0x00;
			S5K4H7_OTP->RGr_ratio = 0x00;
			S5K4H7_OTP->BGr_ratio = 0x00;
			S5K4H7_OTP->GbGr_ratio = 0x00;
		}
	}

	printk("HLT_S5K4H7_read_OTP S5K4H7_OTP->MID=0x%x,S5K4H7_OTP->LID=0x%x\r\n",S5K4H7_OTP->MID,S5K4H7_OTP->LID);
	printk("HLT_S5K4H7_read_OTP R_temp=%d,Gr_temp=%d,Gb_temp=%d,B_temp=%d\r\n",R_temp,Gr_temp,Gb_temp,B_temp);
	printk("HLT_S5K4H7_read_OTP S5K4H7_OTP->RGr_ratio=%d,S5K4H7_OTP->BGr_ratio=%d S5K4H7_OTP->GbGr_ratio=%d\r\n",S5K4H7_OTP->RGr_ratio,S5K4H7_OTP->BGr_ratio,S5K4H7_OTP->GbGr_ratio);

	S5K4H7_HLT_write_cmos_sensor(0x0A00,0x00);
	S5K4H7_HLT_write_cmos_sensor(0x0100,0x00);
}
static void HLT_S5K4H7_apply_OTP(struct HLT_S5K4H7_otp_struct *S5K4H7_OTP)
{
	int R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;
	if(((S5K4H7_OTP->flag)&0x03)!=0x01)
		return;

	R_gain = (RGr_ratio_Typical*1000)/S5K4H7_OTP->RGr_ratio;
	B_gain = (BGr_ratio_Typical*1000)/S5K4H7_OTP->BGr_ratio;
	Gb_gain = (GbGr_ratio_Typical*1000)/S5K4H7_OTP->GbGr_ratio;
	Gr_gain = 1000;
    Base_gain = R_gain;
	if(Base_gain>B_gain) Base_gain=B_gain;
	if(Base_gain>Gb_gain) Base_gain=Gb_gain;
	if(Base_gain>Gr_gain) Base_gain=Gr_gain;
	R_gain = 0x100 * R_gain /Base_gain;
	B_gain = 0x100 * B_gain /Base_gain;
	Gb_gain = 0x100 * Gb_gain /Base_gain;
	Gr_gain = 0x100 * Gr_gain /Base_gain;
	printk("HLT_S5K5e8_OTP R_gain=%x,B_gain=%x Gb_gain=%x Gr_gain=%x\n",R_gain,B_gain,Gb_gain,Gr_gain);
	if(Gr_gain>0x100)
		{
			S5K4H7_HLT_write_cmos_sensor(0x020e,Gr_gain>>8);
			S5K4H7_HLT_write_cmos_sensor(0x020f,Gr_gain&0xff);
		}
	if(R_gain>0x100)
		{
			S5K4H7_HLT_write_cmos_sensor(0x0210,R_gain>>8);
			S5K4H7_HLT_write_cmos_sensor(0x0211,R_gain&0xff);
		}
	if(B_gain>0x100)
		{
			S5K4H7_HLT_write_cmos_sensor(0x0212,B_gain>>8);
			S5K4H7_HLT_write_cmos_sensor(0x0213,B_gain&0xff);
		}
	if(Gb_gain>0x100)
		{
			S5K4H7_HLT_write_cmos_sensor(0x0214,Gb_gain>>8);
			S5K4H7_HLT_write_cmos_sensor(0x0215,Gb_gain&0xff);
		}

	//LSC on
	S5K4H7_HLT_write_cmos_sensor(0x3400,0x00);
	S5K4H7_HLT_write_cmos_sensor(0x0b00,0x01);
}
char HLT_S5K4H7_OTP_CheckID(void)
{
	S5K4H7_HLT_OTP = kzalloc(sizeof(struct HLT_S5K4H7_otp_struct), GFP_KERNEL);
	HLT_S5K4H7_read_OTP(S5K4H7_HLT_OTP);
	return S5K4H7_HLT_OTP->MID;
}
void HLT_S5K4H7_OTP_Release(void)
{
	kfree(S5K4H7_HLT_OTP);
}
void HLT_S5K4H7_OTP_Setting(void)
{
	HLT_S5K4H7_apply_OTP(S5K4H7_HLT_OTP);
}


