/*****************************************************************************
 *
 * Filename:
 * ---------
 *   imx135mipiraw_sensor.c for sunwingroup module
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx135sunwinmipiraw_Sensor.h"
#include "imx135sunwinmipiraw_Camera_Sensor_para.h"
#include "imx135sunwinmipiraw_CameraCustomized.h"

#define IMX135SUNWIN_MID 			0x33
#define IMX135SUNWIN_LENSID 		0xF1
#define IMX135SUNWIN_ID_CHECKSUM 	0xAC


kal_bool  IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool IMX135SUNWINMIPI_Auto_Flicker_mode = KAL_FALSE;

kal_uint8 IMX135SUNWINMIPI_sensor_write_I2C_address = IMX135SUNWINMIPI_WRITE_ID;
kal_uint8 IMX135SUNWINMIPI_sensor_read_I2C_address = IMX135SUNWINMIPI_READ_ID;

//jrd_cam@tcl 20130917 for module SUNWIN bug 	521820+
#define CHECKSUM_ENABLE
//jrd_cam@tcl 20130917 for module SUNWIN bug 	521820-
static struct IMX135SUNWINMIPI_sensor_STRUCT IMX135SUNWINMIPI_sensor={IMX135SUNWINMIPI_WRITE_ID,IMX135SUNWINMIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
KAL_FALSE,KAL_FALSE,KAL_FALSE,231270000,231270000,259200000,0,0,0,64,64,64,IMX135SUNWINMIPI_PV_LINE_LENGTH_PIXELS,
IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES,IMX135SUNWINMIPI_VIDEO_LINE_LENGTH_PIXELS,IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES,IMX135SUNWINMIPI_FULL_LINE_LENGTH_PIXELS,IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES,0,0,0,0,0,0,30};
MSDK_SCENARIO_ID_ENUM  IMX135SUNWINMIPI_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 IMX135SUNWINMIPI_MAX_EXPOSURE_LINES = IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES-5;
kal_uint8  IMX135SUNWINMIPI_MIN_EXPOSURE_LINES = 2;
kal_uint32 IMX135SUNWINMIPI_isp_master_clock;

static DEFINE_SPINLOCK(imx111_drv_lock);

static kal_bool debugFlag = KAL_FALSE;
static kal_bool otp_debugFlag = KAL_FALSE;
#define SENSORDB(fmt, arg...) 	   if (debugFlag) {printk("[JRD_CAM][IMX135SUNWIN]"  fmt, ##arg);}
#define SENSORDB_ERR(fmt, arg...)  {printk("[JRD_CAM_ERR][IMX135SUNWIN]"  fmt, ##arg);}


#define IMX135SUNWIN_OTP
#ifdef IMX135SUNWIN_OTP
#define JRDSENSORDB(fmt, arg...) if (otp_debugFlag){printk( "[JRD_CAM][IMX135SUNWIN]"  fmt, ##arg);}
#define JRDSENSORDB_ERROR(fmt, arg...) {printk( "[JRD_CAM_ERR][IMX135SUNWIN]"  fmt, ##arg);}

#define PAGE_MAX 19
#define PAGE_FULL_SIZE 64
#define DECOMPRESSSIZE 504
#define Sleep(ms) mdelay(ms)

#define G1G2G3_WRITTERN_FLAG_NOT_INIT 0xff
#define ID_WB_G1G2G3_WRITTERN_FLAG  0x40
#define ID_LSC_WRITTERN_FLAG 0x40
#define IMX135SUNWIN_ID 0x32

#define IMX135SUNWINOTP_LSC_FLAG_PAGE  0x09,0x03
#define IMX135SUNWINOTP_LSC_FLAG_SIZE 1
#define IMX135SUNWINOTP_LSC_FLAG_ADDRESS 0x3b04

#define IMX135SUNWINOTP_LSC_START_PAGE  0x03,0x09
#define IMX135SUNWINOTP_LSC_SIZE 504
#define IMX135SUNWINOTP_LSC_SIZE_FOR_READ 384   //read 384 beyte and make these 
#define IMX135SUNWINOTP_LSC_DATA_START_OFFSET 3
#define IMX135SUNWINOTP_LSC_CHECEK_SUM_OFFSET 1
// to 504 
#define IMX135SUNWINOTP_LSC_START_ADDRESS 0x3b04


#define IMX135SUNWINOTP_MDI_START_PAGE 0x02,0x01
#define IMX135SUNWINOTP_MDI_SIZE 4
#define IMX135SUNWINOTP_MDI_START_ADDRESS 0x3b0a

#define IMX135SUNWINOTP_AWB_START_PAGE 0x02,0x01
#define IMX135SUNWINOTP_AWB_SIZE 23
#define IMX135SUNWINOTP_AWB_START_ADDRESS 0x3b10

static kal_uint8 IMX135SUNWINOTP_Page1_Page2_Page3_which_valid=(kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT;
static kal_uint8 IMX135SUNWINOTP_MID_valid_group = (kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT;

static kal_uint8 decompressData[DECOMPRESSSIZE]; //restore the decompressed data after reading out from otp.
static kal_bool otpRdStatus = KAL_FALSE;
#endif

//extern 			tcl_camera_id;

UINT8 IMX135SUNWINMIPIPixelClockDivider=0;
kal_uint16 IMX135SUNWINMIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT IMX135SUNWINMIPISensorConfigData;
kal_uint32 IMX135SUNWINMIPI_FAC_SENSOR_REG;
kal_uint16 IMX135SUNWINMIPI_sensor_flip_value; 

// Gain Index
#define IMX135SUNWINMIPI_MaxGainIndex (71)
kal_uint16 IMX135SUNWINMIPI_sensorGainMapping[IMX135SUNWINMIPI_MaxGainIndex][2] ={
	{71  ,25 },
	{76  ,42 },
	{83  ,59 },
	{89  ,73 },
	{96  ,85 },
	{102 ,96 },
	{108 ,105},
	{115 ,114},
	{121 ,121},
	{128 ,128},
	{134 ,134},
	{140 ,140},
	{147 ,145},
	{153 ,149},
	{160 ,154},
	{166 ,158},
	{172 ,161},
	{179 ,164},
	{185 ,168},
	{192 ,171},
	{200 ,174},
	{208 ,177},
	{216 ,180},
	{224 ,183},
	{232 ,185},
	{240 ,188},
	{248 ,190},
	{256 ,192},
	{264 ,194},
	{272 ,196},
	{280 ,197},
	{288 ,199},
	{296 ,201},
	{304 ,202},
	{312 ,203},
	{320 ,205},
	{328 ,206},
	{336 ,207},
	{344 ,208},
	{352 ,209},
	{360 ,210},
	{368 ,211},
	{376 ,212},
	{384 ,213},
	{390 ,214},
	{399 ,215},
	{409 ,216},
	{419 ,217},
	{431 ,218},
	{442 ,219},
	{455 ,220},
	{467 ,221},
	{481 ,222},
	{496 ,223},
	{512 ,224},
	{528 ,225},
	{545 ,226},
	{565 ,227},
	{584 ,228},
	{606 ,229},
	{630 ,230},
	{655 ,231},
	{682 ,232},
	{712 ,233},
	{744 ,234},
	{780 ,235},
	{819 ,236},
	{862 ,237},
	{910 ,238},
	{963 ,239},
	{1024,240} 

};

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT IMX135SUNWINMIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT IMX135SUNWINMIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define IMX135SUNWINMIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX135SUNWINMIPI_WRITE_ID)

kal_uint16 IMX135SUNWINMIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,IMX135SUNWINMIPI_WRITE_ID);
    return get_byte;
}

#ifdef IMX135SUNWIN_OTP
static void IMX135SUNWINOTPWriteSensor(kal_uint16 address, kal_uint16 para);
static kal_uint8 IMX135SUNWINOTPReadSensor(kal_uint16 address);
static void IMX135SUNWIN_update_awb_gain(kal_uint16 G_gain, kal_uint16 R_gain, kal_uint16 B_gain);
static void IMX135SUNWINLsc(kal_uint8 * dataBuf);
static kal_bool IMX135SUNWIN_ReadOtp(kal_uint8 Bank,kal_uint16 address,kal_uint8 *iBuffer,kal_uint16 buffersize);
static kal_bool check_IMX135SUNWIN_otp_valid_group(kal_uint8 *groupBank, kal_uint8 groupBankArrayLength, kal_uint8 *groupBankIndex);
static kal_uint8 IMX135SUNWINOTPWbPregain(void);
static kal_bool IMX135SUNWINOTPShading(void);
static void IMX135SUNWIN_LSC_DUMP(void);
static void Dump_OTP(void);
#endif

//add for jrd camera debug log open/close begin
//echo 1 sys to enable log. echo 0 to disable log. 
//echo 10 for dump otp value. echo 20 for dump LSC value
static int IMX135SUNWINDebugLog(kal_bool value)
{
    printk(KERN_INFO "[JRD_CAM]IMX135DebugLog value is %d\n", value);
    if( 1 == value ){ //enable sensor log
        debugFlag = KAL_TRUE;
    }
    else if( 0 == value ){//disable sensor log
        debugFlag = KAL_FALSE;
    }
	else if( 10 == value ){
		otp_debugFlag = KAL_TRUE;
		Dump_OTP();
	}else if( 20==value ){
		otp_debugFlag = KAL_TRUE;
		IMX135SUNWIN_LSC_DUMP();
	}
}
//add for jrd camera debug log open/close end

int IMX135SUNWINGetTemperature(void)
{
    int temp;
    IMX135SUNWINMIPI_write_cmos_sensor(0x0220, 0x01);
    temp = IMX135SUNWINMIPI_read_cmos_sensor(0x0222);
    JRDSENSORDB("IMX135SUNWINGetTemperature: %d\n", temp);
    return temp;
}

void IMX135SUNWINMIPISetCNRIntensity(int cnrVal)
{
    switch(cnrVal)
    {
        case 1:
            IMX135SUNWINMIPI_write_cmos_sensor(0x428B,0x0F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x428F,0x0F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4298,0x0E);
            IMX135SUNWINMIPI_write_cmos_sensor(0x429A,0x0E);
            break;
        case 2:
            IMX135SUNWINMIPI_write_cmos_sensor(0x428B,0x3F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x428F,0x3F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4298,0x3E);
            IMX135SUNWINMIPI_write_cmos_sensor(0x429A,0x3E);
            break;
        case 3:
            IMX135SUNWINMIPI_write_cmos_sensor(0x428B,0x7F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x428F,0x7F);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4298,0x7E);  
            IMX135SUNWINMIPI_write_cmos_sensor(0x429A,0x7E);
            break;
        default:
            break;
    }
}

void IMX135SUNWINMIPISetARNRIntensity(int arnrVal)
{
    switch(arnrVal)
    {
        case 1:
            IMX135SUNWINMIPI_write_cmos_sensor(0x4216,0x00);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4217,0x08);
            break;
        case 2:
            IMX135SUNWINMIPI_write_cmos_sensor(0x4216,0x08);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4217,0x08);
            break;
        case 3:
            IMX135SUNWINMIPI_write_cmos_sensor(0x4216,0x08);
            IMX135SUNWINMIPI_write_cmos_sensor(0x4217,0x10);
            break;
        default:
            break;
    }
}

void IMX135SUNWINMIPISetNRIntensity(int cnrVal, int arnrVal)
{
    JRDSENSORDB("IMX135SUNWINMIPISetNRIntensity enter: %d %d\n", cnrVal, arnrVal);
    if(0 != cnrVal)
    {
        IMX135SUNWINMIPISetCNRIntensity(cnrVal);
    }
    if(0 != arnrVal)
    {
        IMX135SUNWINMIPISetARNRIntensity(arnrVal);
    }
}

void IMX135SUNWINMIPISetNR(int cnrVal, int arnrVal)
{
    IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop
    JRDSENSORDB("IMX135SUNWINMIPISetNR enter:\n");
    if((0 == cnrVal)&&(0 == arnrVal))
    {
        JRDSENSORDB("NR off!\n");
        IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xFF);
    }
    else if((0 != cnrVal)&&(0 == arnrVal))
    {
        JRDSENSORDB("CNR on: %d ARNR off!\n", cnrVal);
        IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xF9);
        IMX135SUNWINMIPISetNRIntensity(cnrVal, arnrVal);
    }
    else if((0 == cnrVal)&&(0 != arnrVal))
    {
        JRDSENSORDB("CNR off! ARNR on: %d\n", arnrVal);
        IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xF3);
        IMX135SUNWINMIPISetNRIntensity(cnrVal, arnrVal);
    }
    else if((0 != cnrVal)&&(0 != arnrVal))
    {
        JRDSENSORDB("CNR on: %d ARNR on: %d\n", cnrVal, arnrVal);
        IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xF1);
        IMX135SUNWINMIPISetNRIntensity(cnrVal, arnrVal);
    }

    IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x01);// STREAM STop
}

void IMX135SUNWINMIPI_write_shutter(kal_uint16 shutter)
{
	kal_uint32 frame_length = 0,line_length=0;
    kal_uint32 extra_lines = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;
	
	SENSORDB("IMX135SUNWINMIPI_write_shutter shutter = %d\n",shutter);
	
	if (IMX135SUNWINMIPI_sensor.pv_mode == KAL_TRUE) 
	{
		max_exp_shutter = IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES + IMX135SUNWINMIPI_sensor.pv_dummy_lines-4;
	}
	else if (IMX135SUNWINMIPI_sensor.video_mode== KAL_TRUE) 
	{
		max_exp_shutter = IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES + IMX135SUNWINMIPI_sensor.video_dummy_lines-4;
	}	 
	else 
	{
		max_exp_shutter = IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES + IMX135SUNWINMIPI_sensor.cp_dummy_lines-4;
	}	 
	 
	if(shutter > max_exp_shutter)
		extra_lines = shutter - max_exp_shutter;
	else 
		extra_lines = 0;
	
	if (IMX135SUNWINMIPI_sensor.pv_mode == KAL_TRUE) 
	{
		frame_length =IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES+ IMX135SUNWINMIPI_sensor.pv_dummy_lines + extra_lines;
		line_length = IMX135SUNWINMIPI_PV_LINE_LENGTH_PIXELS+ IMX135SUNWINMIPI_sensor.pv_dummy_pixels;
		spin_lock_irqsave(&imx111_drv_lock,flags);
		IMX135SUNWINMIPI_sensor.pv_line_length = line_length;
		IMX135SUNWINMIPI_sensor.pv_frame_length = frame_length;
		spin_unlock_irqrestore(&imx111_drv_lock,flags);
	}
	else if (IMX135SUNWINMIPI_sensor.video_mode== KAL_TRUE) 
	{
		frame_length = IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES+ IMX135SUNWINMIPI_sensor.video_dummy_lines + extra_lines;
		line_length =IMX135SUNWINMIPI_VIDEO_LINE_LENGTH_PIXELS + IMX135SUNWINMIPI_sensor.video_dummy_pixels;
		spin_lock_irqsave(&imx111_drv_lock,flags);
		IMX135SUNWINMIPI_sensor.video_line_length = line_length;
		IMX135SUNWINMIPI_sensor.video_frame_length = frame_length;
		spin_unlock_irqrestore(&imx111_drv_lock,flags);
	} 
	else
	{
		frame_length = IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES+ IMX135SUNWINMIPI_sensor.cp_dummy_lines + extra_lines;
		line_length =IMX135SUNWINMIPI_FULL_LINE_LENGTH_PIXELS + IMX135SUNWINMIPI_sensor.cp_dummy_pixels;
		spin_lock_irqsave(&imx111_drv_lock,flags);
		IMX135SUNWINMIPI_sensor.cp_line_length = line_length;
		IMX135SUNWINMIPI_sensor.cp_frame_length = frame_length;
		spin_unlock_irqrestore(&imx111_drv_lock,flags);
	}
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 1);        
	IMX135SUNWINMIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	

	IMX135SUNWINMIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0203, shutter  & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 0);    
}   /* write_IMX135SUNWINMIPI_shutter */


static kal_uint16 IMX135SUNWINMIPIReg2Gain(const kal_uint8 iReg)
{

    kal_uint8 iI;
    // Range: 1x to 16x
    for (iI = 0; iI < IMX135SUNWINMIPI_MaxGainIndex; iI++) {
        if(iReg <= IMX135SUNWINMIPI_sensorGainMapping[iI][1]){
            break;
        }
    }
    return IMX135SUNWINMIPI_sensorGainMapping[iI][0];

}
static kal_uint8 IMX135SUNWINMIPIGain2Reg(const kal_uint16 iGain)
{

	kal_uint8 iI;
    
    for (iI = 0; iI < (IMX135SUNWINMIPI_MaxGainIndex-1); iI++) {
        if(iGain <= IMX135SUNWINMIPI_sensorGainMapping[iI][0]){    
            break;
        }
    }
    if(iGain != IMX135SUNWINMIPI_sensorGainMapping[iI][0])
    {
         SENSORDB("[IMX135SUNWINMIPIGain2Reg] Gain mapping don't correctly:%d %d \n", iGain, IMX135SUNWINMIPI_sensorGainMapping[iI][0]);
    }
    return IMX135SUNWINMIPI_sensorGainMapping[iI][1];
}


/*************************************************************************
* FUNCTION
*    IMX135SUNWINMIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX135SUNWINMIPI_SetGain(UINT16 iGain)
{
    kal_uint8 iReg;
	SENSORDB("[IMX135SUNWINMIPI_SetGain] SetGain=%d\n",  iGain);
    iReg = IMX135SUNWINMIPIGain2Reg(iGain);
	SENSORDB("[IMX135SUNWINMIPI_SetGain ] RegisterGain:%d\n", iReg);
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 1);
    IMX135SUNWINMIPI_write_cmos_sensor(0x0205, (kal_uint8)iReg);
    IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 0);

}   /*  IMX135SUNWINMIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_IMX135SUNWINMIPI_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_IMX135SUNWINMIPI_gain(void)
{
    return (kal_uint16)(IMX135SUNWINMIPI_read_cmos_sensor(0x0205)) ;
}  /* read_IMX135SUNWINMIPI_gain */

void write_IMX135SUNWINMIPI_gain(kal_uint16 gain)
{
    IMX135SUNWINMIPI_SetGain(gain);
}
void IMX135SUNWINMIPI_camera_para_to_sensor(void)
{

	kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=IMX135SUNWINMIPISensorReg[i].Addr; i++)
    {
        IMX135SUNWINMIPI_write_cmos_sensor(IMX135SUNWINMIPISensorReg[i].Addr, IMX135SUNWINMIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX135SUNWINMIPISensorReg[i].Addr; i++)
    {
        IMX135SUNWINMIPI_write_cmos_sensor(IMX135SUNWINMIPISensorReg[i].Addr, IMX135SUNWINMIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        IMX135SUNWINMIPI_write_cmos_sensor(IMX135SUNWINMIPISensorCCT[i].Addr, IMX135SUNWINMIPISensorCCT[i].Para);
    }

}


/*************************************************************************
* FUNCTION
*    IMX135SUNWINMIPI_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX135SUNWINMIPI_sensor_to_camera_para(void)
{

	kal_uint32    i,temp_data;
    for(i=0; 0xFFFFFFFF!=IMX135SUNWINMIPISensorReg[i].Addr; i++)
    {
		temp_data=IMX135SUNWINMIPI_read_cmos_sensor(IMX135SUNWINMIPISensorReg[i].Addr);
		spin_lock(&imx111_drv_lock);
		IMX135SUNWINMIPISensorReg[i].Para = temp_data;
		spin_unlock(&imx111_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX135SUNWINMIPISensorReg[i].Addr; i++)
    {
    	temp_data=IMX135SUNWINMIPI_read_cmos_sensor(IMX135SUNWINMIPISensorReg[i].Addr);
         spin_lock(&imx111_drv_lock);
        IMX135SUNWINMIPISensorReg[i].Para = temp_data;
		spin_unlock(&imx111_drv_lock);
    }

}

/*************************************************************************
* FUNCTION
*    IMX135SUNWINMIPI_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  IMX135SUNWINMIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void IMX135SUNWINMIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void IMX135SUNWINMIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 SENSORDB("[IMX105MIPI][Error]get_sensor_item_info error!!!\n");
          }
           	spin_lock(&imx111_drv_lock);    
            temp_para=IMX135SUNWINMIPISensorCCT[temp_addr].Para;	
			spin_unlock(&imx111_drv_lock);
            temp_gain = IMX135SUNWINMIPIReg2Gain(temp_para);
            temp_gain=(temp_gain*1000)/BASEGAIN;
            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1000;
            info_ptr->Max=15875;//why
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=IMX135SUNWINMIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }
                
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=IMX135SUNWINMIPI_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool IMX135SUNWINMIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
   {
	case PRE_GAIN:
		switch (item_idx)
		{
		  case 0:
		    temp_addr = PRE_GAIN_R_INDEX;
		  break;
		  case 1:
		    temp_addr = PRE_GAIN_Gr_INDEX;
		  break;
		  case 2:
		    temp_addr = PRE_GAIN_Gb_INDEX;
		  break;
		  case 3:
		    temp_addr = PRE_GAIN_B_INDEX;
		  break;
		  case 4:
		    temp_addr = SENSOR_BASEGAIN;
		  break;
		  default:
		     SENSORDB("[IMX135SUNWINMIPI][Error]set_sensor_item_info error!!!\n");
		}
		temp_para = IMX135SUNWINMIPIGain2Reg(ItemValue);
		spin_lock(&imx111_drv_lock);    
		IMX135SUNWINMIPISensorCCT[temp_addr].Para = temp_para;
		spin_unlock(&imx111_drv_lock);
		IMX135SUNWINMIPI_write_cmos_sensor(IMX135SUNWINMIPISensorCCT[temp_addr].Addr,temp_para);
		temp_para=read_IMX135SUNWINMIPI_gain();	

	break;
    case CMMCLK_CURRENT:
        switch (item_idx)
        {
            case 0:
                if(ItemValue==2)
                {			
                spin_lock(&imx111_drv_lock);    
                    IMX135SUNWINMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
				spin_unlock(&imx111_drv_lock);
                }
                else if(ItemValue==3 || ItemValue==4)
                {
                	spin_lock(&imx111_drv_lock);    
                    IMX135SUNWINMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
					spin_unlock(&imx111_drv_lock);
                }
                else if(ItemValue==5 || ItemValue==6)
                {
                	spin_lock(&imx111_drv_lock);    
                    IMX135SUNWINMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
					spin_unlock(&imx111_drv_lock);
                }
                else
                {
                	spin_lock(&imx111_drv_lock);    
                    IMX135SUNWINMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
					spin_unlock(&imx111_drv_lock);
                }
                break;
            default:
                ASSERT(0);
        }
        break;
    case FRAME_RATE_LIMITATION:
        ASSERT(0);
        break;
    case REGISTER_EDITOR:
        switch (item_idx)
        {
            case 0:
				spin_lock(&imx111_drv_lock);    
                IMX135SUNWINMIPI_FAC_SENSOR_REG=ItemValue;
				spin_unlock(&imx111_drv_lock);
                break;
            case 1:
                IMX135SUNWINMIPI_write_cmos_sensor(IMX135SUNWINMIPI_FAC_SENSOR_REG,ItemValue);
                break;
            default:
                ASSERT(0);
        }
        break;
    default:
        ASSERT(0);
   }
   return TRUE;
}

#ifdef IMX135SUNWIN_OTP
static void IMX135SUNWINOTPWriteSensor(kal_uint16 address, kal_uint16 para)
{
    IMX135SUNWINMIPI_write_cmos_sensor(address, para);
}

static kal_uint8 IMX135SUNWINOTPReadSensor(kal_uint16 address)
{
    kal_uint8 reVal;
    reVal = (kal_uint8)IMX135SUNWINMIPI_read_cmos_sensor(address);	
//    JRDSENSORDB("!!!!!address=0x%x, reVal= 0x%x!!!!!\n", address, reVal);
    return reVal;
}

#if 0
static void IMX135SUNWIN_update_awb_gain(kal_uint16 G_gain, kal_uint16 R_gain, kal_uint16 B_gain)
{
    JRDSENSORDB("IMX135SUNWIN_update_awb_gain ENTER :\n ");
    JRDSENSORDB("IMX135SUNWIN_update_awb_gain G_gain =%d, R_gain=%d, B_gain=%d\n",G_gain, R_gain, B_gain);

    IMX135SUNWINOTPWriteSensor(0x020E, G_gain>>8);
    IMX135SUNWINOTPWriteSensor(0x020F, G_gain & 0x00ff);
    IMX135SUNWINOTPWriteSensor(0x0210, R_gain>>8);
    IMX135SUNWINOTPWriteSensor(0x0211, R_gain & 0x00ff);
    IMX135SUNWINOTPWriteSensor(0x0212, B_gain>>8);
    IMX135SUNWINOTPWriteSensor(0x0213, B_gain & 0x00ff);
    IMX135SUNWINOTPWriteSensor(0x0214, G_gain>>8);
    IMX135SUNWINOTPWriteSensor(0x0215, G_gain & 0x00ff);
}
#endif

//read data from the corresponding bank
static kal_bool IMX135SUNWIN_ReadOtp(kal_uint8 Bank,kal_uint16 address,kal_uint8 *iBuffer,kal_uint16 buffersize)
{
    kal_uint8 reVal;
    kal_uint16 j = 0;
    kal_uint16 i = 0;
    kal_uint8 rdrdy_status;
    kal_uint16 k = buffersize;
    kal_uint16 x = 0;
    JRDSENSORDB("IMX135SUNWIN_ReadOtp ENTER Bank:0x%x address:0x%x buffersize:%d\n ",Bank, address, buffersize);

    while (i<buffersize)
    {
        j=0;
        IMX135SUNWINOTPWriteSensor(0x3b02, Bank);//select bank
        IMX135SUNWINOTPWriteSensor(0x3b00, 0x01);//select bank
        Sleep(1);
        rdrdy_status = IMX135SUNWINOTPReadSensor(0x3b01);

        if(0x01 & rdrdy_status)
        {
            JRDSENSORDB("Ready to read OTP!\n");
        }
        else
        {
            for(x=0;x<10;x++)
			{          
				rdrdy_status = IMX135SUNWINOTPReadSensor(0x3b01);
				if(0x01 & rdrdy_status)
				{
					JRDSENSORDB("Ready to read OTP! try =%d \n",x);
					break;
				}
				Sleep(1);
			}
            if(x==10)
            {
            	JRDSENSORDB_ERROR("Haven't been Ready to read OTP!\n");
            	return KAL_FALSE;
            }
        }
		
        while (j<PAGE_FULL_SIZE)
        {
            reVal= IMX135SUNWINOTPReadSensor(address+j);
            *(iBuffer+i) =(kal_uint8)reVal;
            i++;
            j++;
            if (i>=buffersize)
            {
                break;
            }
        }
        Bank++;
    }
    return KAL_TRUE;
}
//jrd_cam@tcl 20130917 for module SUNWIN bug 	521820+
#ifdef CHECKSUM_ENABLE
#define IMX135TRLYOTP_P1P2P3_START_ADDRESS 0x3b04
#define IMX135TRLYOTP_CHECKSUM_P1P2P3_START_ADDRESS 0x3b06
#define IMX135TRLYOTP_CHECKSUM_P1P2P3_CHECKSUM_ID_OFFSET 1
#define IMX135TRLYOTP_CHECKSUM_P1P2P3_LENGTH 40
#define IMX135TRLYOTP_CHECKSUM 256

static int CHECKSUM_IMX135SUNWINOTP(u8 Page)
{
    int i=0;
    int temp=0;
    int i4RetValue=0;
    kal_uint8 otpChecksumBuffer[PAGE_FULL_SIZE];
	
    i4RetValue = IMX135SUNWIN_ReadOtp(Page,  IMX135TRLYOTP_P1P2P3_START_ADDRESS, otpChecksumBuffer, IMX135TRLYOTP_CHECKSUM_P1P2P3_LENGTH);
    for(i=IMX135TRLYOTP_CHECKSUM_P1P2P3_START_ADDRESS-IMX135TRLYOTP_P1P2P3_START_ADDRESS;i<IMX135TRLYOTP_CHECKSUM_P1P2P3_LENGTH;i++)
    {
           temp+=otpChecksumBuffer[i];
    }
    JRDSENSORDB_ERROR("[JRD_CAM][checksum]temp =0x%x,,,%d,,,checksum=0x%x,\n",temp,temp,otpChecksumBuffer[IMX135TRLYOTP_CHECKSUM_P1P2P3_CHECKSUM_ID_OFFSET]);
    if(temp%IMX135TRLYOTP_CHECKSUM!=otpChecksumBuffer[IMX135TRLYOTP_CHECKSUM_P1P2P3_CHECKSUM_ID_OFFSET])
    {
           JRDSENSORDB_ERROR("[JRD_CAM][checksum]temp =0x%x,,,%d,line=%d\n",temp,temp,__LINE__);
           return -2;
    }
	return KAL_TRUE;
}
#endif

static int IMX135SUNWINOTP_MID_checksum(u8 Page)
{
    int i = 0;
    int temp = 0;
	kal_uint8 reVal;
	kal_uint8 rdrdy_status;
    kal_uint8 otpChecksumBuffer[PAGE_FULL_SIZE];
	
	IMX135SUNWINOTPWriteSensor(0x3b02, Page);//select bank
	IMX135SUNWINOTPWriteSensor(0x3b00, 0x01);//select bank
	Sleep(1);
	rdrdy_status = IMX135SUNWINOTPReadSensor(0x3b01);

	if(0x01 & rdrdy_status)
	{
		JRDSENSORDB("Ready to read OTP!\n");
	}else
	{
		for (i = 0; i < 10; i++)
		{		   
			rdrdy_status = IMX135SUNWINOTPReadSensor(0x3b01);
			if(0x01 & rdrdy_status)
			{
				JRDSENSORDB("Ready to read OTP! try =%d \n",i);
				break;
			}
			Sleep(1);
		}
		if ( i == 10 )
		{
			JRDSENSORDB_ERROR("Haven't been Ready to read OTP!\n");
			return KAL_FALSE;
		}
	}
	
	otpChecksumBuffer[0] = IMX135SUNWINOTPReadSensor(0x3B05);//checksum value
	
	otpChecksumBuffer[1] = IMX135SUNWINOTPReadSensor(0x3B0A);
	otpChecksumBuffer[2] = IMX135SUNWINOTPReadSensor(0x3B0B);
	otpChecksumBuffer[3] = IMX135SUNWINOTPReadSensor(0x3B0C);
	otpChecksumBuffer[4] = IMX135SUNWINOTPReadSensor(0x3B0D);

	printk("[JRD_CAM][IMX135SUNWIN]otp checksum value=%d %d %d %d %d\n", 
			otpChecksumBuffer[0], otpChecksumBuffer[1], otpChecksumBuffer[2], otpChecksumBuffer[3], otpChecksumBuffer[4]);

	if (otpChecksumBuffer[0] == (otpChecksumBuffer[1]+otpChecksumBuffer[2]+otpChecksumBuffer[3]+otpChecksumBuffer[4])%256)
	{
		printk("[JRD_CAM][IMX135SUNWIN]otp checksum successfully\n");
		return otpChecksumBuffer[0];
	}else
	{
		printk("[JRD_CAM][IMX135SUNWIN]error otp checksum failed\n");
		return KAL_FALSE;
	}
}

static kal_bool check_IMX135SUNWIN_otp_valid_group(kal_uint8 *groupBank, kal_uint8 groupBankArrayLength, kal_uint8 *groupBankIndex)
{
    kal_uint8 i, programflag[64];
    kal_uint16 programflag_regaddr = 0x3b04;
    kal_bool ret;

    *groupBankIndex = (kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT;

    for (i = 0 ; i < groupBankArrayLength; i++)
    {
        IMX135SUNWIN_ReadOtp(groupBank[i], programflag_regaddr, programflag, 1);
        if(ID_WB_G1G2G3_WRITTERN_FLAG == programflag[0])
        {
            JRDSENSORDB("[JRD_CAM]OPT group select,programflag=%d\n",programflag[0]);
            *groupBankIndex = i;
            break;
        }else
		{
	        JRDSENSORDB("[JRD_CAM]OPT group change,programflag=%d\n",programflag[0]);
		}
    }

    if(*groupBankIndex !=(kal_uint8) G1G2G3_WRITTERN_FLAG_NOT_INIT)
    {
        ret = KAL_TRUE;
    }else
    {
        ret = KAL_FALSE;
    }
    return ret;
}

//read and process AWB data
static kal_uint8 IMX135SUNWINOTPWbPregain(void)
{
    kal_uint8 awbGroupBank[] = {IMX135SUNWINOTP_MDI_START_PAGE};
    kal_uint8 rdRdyStatus, awbGroupBankArrayLength;
    kal_uint8 awbGroupBankValidIndex;
    kal_bool ret=KAL_FALSE;
    kal_uint8 otpDataBuffer[IMX135SUNWINOTP_AWB_SIZE];
    int i=0;
    kal_uint8 Vendor_ID = 0x0;

    JRDSENSORDB("[JRD_CAM]IMX135SUNWINOTPWbPregain ENTER :\n ");

    //1. check which group is valid
    awbGroupBankArrayLength = sizeof(awbGroupBank) / sizeof(awbGroupBank[0]);
	if(IMX135SUNWINOTP_Page1_Page2_Page3_which_valid==(kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT)
	{
	    ret = check_IMX135SUNWIN_otp_valid_group(&awbGroupBank[0], awbGroupBankArrayLength, &awbGroupBankValidIndex);
	    if (KAL_TRUE == ret)
	    {
	        JRDSENSORDB("[JRD_CAM]in%s line=%d\n",__func__,__LINE__);
	        JRDSENSORDB("[JRD_CAM]AWB OTP exist! Valid group is awbGroupBank[%d]: %x\n", awbGroupBankValidIndex, awbGroupBank[awbGroupBankValidIndex]);
	        IMX135SUNWINOTP_Page1_Page2_Page3_which_valid=awbGroupBankValidIndex;
	    }
	    else
	    {
	        JRDSENSORDB("[JRD_CAM]in%s line=%d\n",__func__,__LINE__);
	        JRDSENSORDB_ERROR("[JRD_CAM]No AWB OTP Data!\n");
	        IMX135SUNWINOTP_Page1_Page2_Page3_which_valid=(kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT;
	        return -1;
	    }
	}
	else if((0<=IMX135SUNWINOTP_Page1_Page2_Page3_which_valid)&&(sizeof(awbGroupBank)>IMX135SUNWINOTP_Page1_Page2_Page3_which_valid))
	{
	          JRDSENSORDB("[JRD_CAM]Aleady get valid Group %d\n",__LINE__);
	     awbGroupBankValidIndex=IMX135SUNWINOTP_Page1_Page2_Page3_which_valid;
	}
	else
	{
	      JRDSENSORDB("[JRD_CAM]in%s line=%d\n",__func__,__LINE__);
	      JRDSENSORDB_ERROR("[JRD_CAM]errorIMX135SUNWINOTP_Page1_Page2_Page3_which_valid=%d,G1G2G3_WRITTERN_FLAG_NOT_INIT=%dthere       must be some error\n",IMX135SUNWINOTP_Page1_Page2_Page3_which_valid,G1G2G3_WRITTERN_FLAG_NOT_INIT);           
	      return -1;
	}
	//jrd_cam@tcl 20130917 for module SUNWIN bug 	521820+
#ifdef CHECKSUM_ENABLE
	if(CHECKSUM_IMX135SUNWINOTP(awbGroupBank[awbGroupBankValidIndex])!=KAL_TRUE)
	{
		JRDSENSORDB_ERROR("[JRD_CAM] checksum error\n");           
		return -2;
	}
	JRDSENSORDB_ERROR("[JRD_CAM] checksum ok\n");           
#endif
	//jrd_cam@tcl 20130917 for module SUNWIN bug 	521820-
	//2. read awb from otp
	if(KAL_FALSE == IMX135SUNWIN_ReadOtp(awbGroupBank[awbGroupBankValidIndex],IMX135SUNWINOTP_AWB_START_ADDRESS,otpDataBuffer,IMX135SUNWINOTP_AWB_SIZE))
	{
	    JRDSENSORDB_ERROR("[JRD_CAM]Read awb OTP error!\n");
	    return -1;
	}

	Vendor_ID = otpDataBuffer[1];
	JRDSENSORDB("[JRD_CAM]Module lens ID: %d\n", Vendor_ID);

    return KAL_TRUE;
}

static kal_bool IMX135SUNWINOTP_Get_Module_Info(kal_uint8 *Vendor_ID)
{
    kal_uint8 ModuleinforGroupBank[] = {IMX135SUNWINOTP_MDI_START_PAGE};
    kal_uint8 P1P2P3BankValidIndex = 0;
	kal_bool ret = 0;
	
	if ( IMX135SUNWINOTP_MID_valid_group == (kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT )
    {
        ret = check_IMX135SUNWIN_otp_valid_group(&ModuleinforGroupBank[0],sizeof(ModuleinforGroupBank) / sizeof(ModuleinforGroupBank[0]) , &P1P2P3BankValidIndex);
        if (KAL_TRUE == ret)
        {
            IMX135SUNWINOTP_MID_valid_group = P1P2P3BankValidIndex;
            JRDSENSORDB("[JRD_CAM]AWB OTP exist! Valid group is awbGroupBank[%d]: %x\n", P1P2P3BankValidIndex, ModuleinforGroupBank[P1P2P3BankValidIndex]);
        }else
        {
            printk("[JRD_CAM]No AWB OTP Data failed\n");
            IMX135SUNWINOTP_MID_valid_group=(kal_uint8)G1G2G3_WRITTERN_FLAG_NOT_INIT;
            return -1;
        }
    }
    else if(( 0 <= IMX135SUNWINOTP_MID_valid_group ) && ( sizeof(ModuleinforGroupBank) > IMX135SUNWINOTP_MID_valid_group ))
    {
		printk("[JRD_CAM]Aleady get valid Group\n");
		P1P2P3BankValidIndex = IMX135SUNWINOTP_MID_valid_group;
    }
    else
    {
		printk("[JRD_CAM]error IMX135SUNWINOTP_MID_valid_group=%d,there must be some error\n",IMX135SUNWINOTP_MID_valid_group);           
		return -1;
    }
    printk("[JRD_CAM]IMX135SUNWINOTP_MID_valid_group=%d\n", ModuleinforGroupBank[IMX135SUNWINOTP_MID_valid_group]);		
    JRDSENSORDB("[JRD_CAM]P1P2P3BankValidIndex=%d,ModuleinforGroupBank[%d]=%d\n",P1P2P3BankValidIndex,P1P2P3BankValidIndex,ModuleinforGroupBank[P1P2P3BankValidIndex]);

	//ret = IMX135SUNWINOTP_MID_checksum(ModuleinforGroupBank[P1P2P3BankValidIndex]);
 	
	if(KAL_FALSE == IMX135SUNWIN_ReadOtp(ModuleinforGroupBank[P1P2P3BankValidIndex], IMX135SUNWINOTP_MDI_START_ADDRESS, Vendor_ID, IMX135SUNWINOTP_MDI_SIZE))
    {
        printk("[JRD_CAM][IMX135SUNWIN]Read OTP Get_Module_Info error!\n");
        return -1;
    }
	
    printk("[JRD_CAM][IMX135SUNWIN]VendorID: 0x%x 0x%x 0x%x 0x%x\n", *Vendor_ID, *(Vendor_ID+1), *(Vendor_ID+2), *(Vendor_ID+3));
	
    return ret;
}

static void Dump_OTP(void)
{
    kal_uint16 startAddress = 0x3b04;
    kal_uint8 OTPBufferVendor[PAGE_FULL_SIZE];
    kal_int8 P1P2P3BankValidIndex;
    kal_int8 VendorID;
    kal_int8 i,j;
	kal_bool ret;
	
    JRDSENSORDB("[JRD_CAM]----------------DumpOTP------------------\n");
	for(i=0;i<PAGE_MAX;i++)
	{
		JRDSENSORDB("[JRD_CAM]Page=%d\n",i);
		for(j=0;j<PAGE_FULL_SIZE;j++)
		{
			OTPBufferVendor[j]=0;
		}
		if(KAL_FALSE == IMX135SUNWIN_ReadOtp(i,startAddress,OTPBufferVendor,PAGE_FULL_SIZE))
		{
			JRDSENSORDB_ERROR("[JRD_CAM]Read awb OTP error!\n");
		//       return -1;
		}
		for(j=0;j<PAGE_FULL_SIZE;j++)
		{
			JRDSENSORDB("[JRD_CAM]addr:%d 0x%x=%x\n",i*PAGE_FULL_SIZE+j,startAddress+j,OTPBufferVendor[j]);
		}
		Sleep(1);
	}
    JRDSENSORDB("[JRD_CAM]----------------end DumpOTP------------------\n");

}

static kal_bool check_is_IMX135SUNWIN_otp_lsc_valid(kal_uint8 *Page)
{ 
    kal_uint8 LSC_flag_Page[]={IMX135SUNWINOTP_LSC_FLAG_PAGE}; 
    kal_uint8 otpDataBuffer[IMX135SUNWINOTP_LSC_FLAG_SIZE];
    kal_uint8 ret=KAL_FALSE,i;
    JRDSENSORDB("[JRD_CAM]in %s line=%d\n",__func__,__LINE__);
    
    for(i=0;i<sizeof(LSC_flag_Page);i++)
    {
        ret=IMX135SUNWIN_ReadOtp(LSC_flag_Page[i], IMX135SUNWINOTP_LSC_FLAG_ADDRESS, &otpDataBuffer, IMX135SUNWINOTP_LSC_FLAG_SIZE);
        if((otpDataBuffer[0]&0xc0)==ID_LSC_WRITTERN_FLAG)
        {
        	JRDSENSORDB_ERROR("[JRD_CAM]in %s line=%d LSC FLAG is read 0x40,i=%d!!\n",__func__,__LINE__,i);
            *Page=LSC_flag_Page[i];
	        return KAL_TRUE;
        }
    
    }
    if(i==sizeof(LSC_flag_Page))
    {
    	JRDSENSORDB_ERROR("[JRD_CAM]in %s line=%d LSC FLAG is read=%d!\n",__func__,__LINE__,otpDataBuffer[0]);
        return KAL_FALSE;
    }    
    if(ret!=KAL_TRUE)
    {
        JRDSENSORDB_ERROR("[JRD_CAM]in %s line=%dREAD LSC FLAG error\n",__func__,__LINE__);
        return KAL_FALSE;
    } 

    return KAL_FALSE;
}

static kal_bool IMX135SUNWINOTPShading1(void)
{
    kal_uint8 rdRdyStatus, lscGroupBankArrayLength;
    kal_int8 lscGroupBankValidIndex;
    kal_bool ret=KAL_FALSE;
    kal_uint8 otpDataBuffer[IMX135SUNWINOTP_LSC_SIZE_FOR_READ];
    int i;
    int y =IMX135SUNWINOTP_LSC_DATA_START_OFFSET;
    kal_uint32 checksum = 0;
    kal_uint8 LSC_PAGE=0;
	
    printk("[JRD_CAM][IMX135SUNWINOTP]start to read LSC data\n");
    //1. check is LSC valid and which Page is vaild return by LSC_PAGE
#if 1//JRD_CAM no flag in current module so underconstruction
    ret = check_is_IMX135SUNWIN_otp_lsc_valid(&LSC_PAGE);
     JRDSENSORDB("[JRD_CAM]in %s line=%d ret =%d\n",__func__,__LINE__,ret);
    if(ret==KAL_FALSE)
    {
        JRDSENSORDB_ERROR("[JRD_CAM]in %s line=%d no LSC data!!\n",__func__,__LINE__);
	    return KAL_FALSE;
    }
	#endif
    //2. read shading from otp
    if(KAL_FALSE == 
    IMX135SUNWIN_ReadOtp(LSC_PAGE,IMX135SUNWINOTP_LSC_START_ADDRESS,&otpDataBuffer[0],IMX135SUNWINOTP_LSC_SIZE_FOR_READ))
    {
        JRDSENSORDB_ERROR("Read lsc OTP error!\n");
        return KAL_FALSE;
    }
    //3. check sum and decompress the lsc data
    checksum += otpDataBuffer[y-1];//special for IMX135 sungsamg
    JRDSENSORDB("[JRD_CAM][%d]----------------DumpOTP LSC level 1 ------------------\n",LSC_PAGE);

	for(i=0;i<IMX135SUNWINOTP_LSC_SIZE_FOR_READ;i++)
	{
	     JRDSENSORDB("[JRD_CAM][0x%x]=[%x]\n",0x3b04+i,otpDataBuffer[i]);
	}
    JRDSENSORDB("[JRD_CAM][%d]----------------DumpOTP LSC level 1 end------------------\n",LSC_PAGE);
	for(i = 0; i < IMX135SUNWINOTP_LSC_SIZE; i+=4)
    {
    //    JRDSENSORDB("otpDataBuffer[%d] = %d\n", y, otpDataBuffer[y]);
        decompressData[i] = otpDataBuffer[y]>>4;
        checksum += otpDataBuffer[y];
        decompressData[i + 1] = otpDataBuffer[y + 1];
        checksum += otpDataBuffer[y + 1];
        decompressData[i + 2] = otpDataBuffer[y] & 0x0f;
        checksum += otpDataBuffer[y + 2];
        decompressData[i + 3] = otpDataBuffer[y + 2];
        y += 3;
    }	
	for(;i<IMX135SUNWINOTP_LSC_SIZE_FOR_READ;i++)
	{
        checksum+=otpDataBuffer[i];
	}
    JRDSENSORDB("[JRD_CAM]checksum==%x,\n",checksum);
	IMX135SUNWIN_LSC_DUMP();
    if((checksum%256)==otpDataBuffer[IMX135SUNWINOTP_LSC_CHECEK_SUM_OFFSET])
    {
        printk("[JRD_CAM][JRD_CAM]OTP CHECKSUM OK checksum==%x,,OTP[1]=%x\n",checksum,otpDataBuffer[IMX135SUNWINOTP_LSC_CHECEK_SUM_OFFSET]);
    }
    else
    {
        JRDSENSORDB_ERROR("[JRD_CAM]OTP CHECKSUM error checksum==%x,,OTP[1]=%x\n",checksum,otpDataBuffer[IMX135SUNWINOTP_LSC_CHECEK_SUM_OFFSET]);
    	return KAL_FALSE;
	}
    
    return KAL_TRUE;
}

static void IMX135SUNWIN_LSC_DUMP(void)
{
    int i;
    JRDSENSORDB("[JRD_CAM]----------------DumpOTP LSC------------------\n");
    for(i = 0; i <DECOMPRESSSIZE; i++)
    {
        JRDSENSORDB("decompressData[%d] = %x\n", i, decompressData[i]);
    }
    JRDSENSORDB("[JRD_CAM]----------------DumpOTP LSC end------------------\n");
}

//write shading parameters to sensor
static void IMX135SUNWINLsc(kal_uint8 * dataBuf)
{
    int i;

    for(i = 0; i < DECOMPRESSSIZE; i++)
    {
        //JRDSENSORDB("decompressData[%d] = %d\n", i, dataBuf[i]);
        IMX135SUNWINOTPWriteSensor((0x4800+i),dataBuf[i]);
    }

    IMX135SUNWINOTPWriteSensor(0x4500, 0x1f);
    IMX135SUNWINOTPWriteSensor(0x0700, 0x01);
    IMX135SUNWINOTPWriteSensor(0x3a63, 0x01);
}
#endif
//[End]

static void IMX135SUNWINMIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
	kal_uint32 frame_length = 0, line_length = 0;
	if(IMX135SUNWINMIPI_sensor.pv_mode == KAL_TRUE)
	{
		spin_lock(&imx111_drv_lock);    
		IMX135SUNWINMIPI_sensor.pv_dummy_pixels = iPixels;
		IMX135SUNWINMIPI_sensor.pv_dummy_lines = iLines;
		IMX135SUNWINMIPI_sensor.pv_line_length = IMX135SUNWINMIPI_PV_LINE_LENGTH_PIXELS + iPixels;
		IMX135SUNWINMIPI_sensor.pv_frame_length = IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES + iLines;
		spin_unlock(&imx111_drv_lock);
		line_length = IMX135SUNWINMIPI_sensor.pv_line_length;
		frame_length = IMX135SUNWINMIPI_sensor.pv_frame_length;

	}
    else if(IMX135SUNWINMIPI_sensor.video_mode == KAL_TRUE)
   	{
		spin_lock(&imx111_drv_lock);    
		IMX135SUNWINMIPI_sensor.video_dummy_pixels = iPixels;
		IMX135SUNWINMIPI_sensor.video_dummy_lines = iLines;
		IMX135SUNWINMIPI_sensor.video_line_length = IMX135SUNWINMIPI_VIDEO_LINE_LENGTH_PIXELS + iPixels;
		IMX135SUNWINMIPI_sensor.video_frame_length = IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES + iLines;
		spin_unlock(&imx111_drv_lock);
		line_length = IMX135SUNWINMIPI_sensor.video_line_length;
		frame_length = IMX135SUNWINMIPI_sensor.video_frame_length;
   	}
	else
	{
		spin_lock(&imx111_drv_lock);	
		IMX135SUNWINMIPI_sensor.cp_dummy_pixels = iPixels;
		IMX135SUNWINMIPI_sensor.cp_dummy_lines = iLines;
		IMX135SUNWINMIPI_sensor.cp_line_length = IMX135SUNWINMIPI_FULL_LINE_LENGTH_PIXELS + iPixels;
		IMX135SUNWINMIPI_sensor.cp_frame_length = IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES + iLines;
		spin_unlock(&imx111_drv_lock);
		line_length = IMX135SUNWINMIPI_sensor.cp_line_length;
		frame_length = IMX135SUNWINMIPI_sensor.cp_frame_length;
    }

	IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 1);        
	IMX135SUNWINMIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0342, (line_length >>8) & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0343, line_length & 0xFF);
	IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 0);

	SENSORDB("IMX135SUNWINMIPI_SetDummy,dumy_pixel=%d,dumy_line=%d,\n",iPixels,iLines);
  
}   /*  IMX135SUNWINMIPI_SetDummy */

static void IMX135SUNWINMIPI_Sensor_Init(void)
{
    SENSORDB("IMX135SUNWINMIPI_Sensor_Init enter:");
	IMX135SUNWINMIPI_write_cmos_sensor(0x0101, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0105, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0110, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0220, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3302, 0x11);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3833, 0x20);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3893, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3906, 0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3907, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x391B, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3C09, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x600A, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3008, 0xB0);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x320A, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x320D, 0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3216, 0x2E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x322C, 0x02);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3409, 0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x340C, 0x2D);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3411, 0x39);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3414, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3427, 0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3480, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3484, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3488, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x348C, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3490, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3494, 0x1E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3511, 0x8F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x364F, 0x2D);//

//quality
	//defect forrection recommended setting
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x380A, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x380B, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4103, 0x00);//

	//color artifact recommended setting
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x4243, 0x9A);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4330, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4331, 0x90);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4332, 0x02);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4333, 0x58);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4334, 0x03);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4335, 0x20);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4336, 0x03);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4337, 0x84);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x433C, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4340, 0x02);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4341, 0x58);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4342, 0x03);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4343, 0x52);//

	/////Moire reduction parameter setting
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x4364, 0x0B);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4368, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4369, 0x0F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436A, 0x03);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436B, 0xA8);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436C, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436D, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436E, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x436F, 0x06);//

	//CNR parameter setting

	IMX135SUNWINMIPI_write_cmos_sensor(0x4281, 0x21);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4282, 0x18);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4283, 0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4284, 0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4287, 0x7F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4288, 0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x428B, 0x7F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x428C, 0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x428F, 0x7F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4297, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4298, 0x7E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4299, 0x7E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x429A, 0x7E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x42A4, 0xFB);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x42A5, 0x7E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x42A6, 0xDF);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x42A7, 0xB7);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x42AF, 0x03);//
	
   // ARNR Parameter setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x4207, 0x03);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4216, 0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4217, 0x08);//

	//DLC Parammeter setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x4218, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x421B, 0x20);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x421F, 0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4222, 0x02);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4223, 0x22);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x422E, 0x54);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x422F, 0xFB);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4230, 0xFF);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4231, 0xFE);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4232, 0xFF);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4235, 0x58);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4236, 0xF7);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4237, 0xFD);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x4239, 0x4E);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x423A, 0xFC);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x423B, 0xFD);//
	
	//HDR
	//LSC setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x452A, 0x02);//


	//white balance setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0712, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0713, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0714, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0715, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0716, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0717, 0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0718, 0x01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0719, 0x00);//

	//shading setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x4500, 0x1F);//
	SENSORDB("IMX135SUNWINMIPI_Sensor_Init exit");

    // The register only need to enable 1 time.    
    spin_lock(&imx111_drv_lock);  
    IMX135SUNWINMIPI_Auto_Flicker_mode = KAL_FALSE;     // reset the flicker status    
	spin_unlock(&imx111_drv_lock);

#ifdef IMX135SUNWIN_OTP
    /*if(KAL_TRUE == otpRdStatus)
    {
		printk("[JRD_CAM]IMX135SUNWINMIPI_Sensor_Init LSC otp enter\n");
        IMX135SUNWINLsc(&decompressData[0]);
		printk("[JRD_CAM]IMX135SUNWINMIPI_Sensor_Init LSC otp end:\n");
    }
	else
	{
		printk("[JRD_CAM]IMX135SUNWINMIPI_Sensor_Init LSC OTP otpRdStatus=%d\n", otpRdStatus);
	}*/
#endif
}   /*  IMX135SUNWINMIPI_Sensor_Init  */
void IMX135SUNWINMIPI_Sensor_VideoFullSizeSetting(void)//16:9   6M

{	
	SENSORDB("IMX135SUNWINMIPI_Sensor_VideoFullSizeSetting enter:");

 	//PLL setting 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop

	//PLL setting
    IMX135SUNWINMIPI_write_cmos_sensor(0x011E,0x18);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x011F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0301,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0303,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0305,0x0B);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0309,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030B,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030C,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030D,0x09);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3A06,0x11);//   

	//Mode setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0108,0x03);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0112,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0113,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0381,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0383,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0385,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0387,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0390,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0391,0x22);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0392,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0401,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0404,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0405,0x10);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4082,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4083,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x7006,0x04);//

	//Optionnal function setting
    //[Begin]zhfan for  module otp PR 450709
	//IMX135SUNWINMIPI_write_cmos_sensor(0x0700,0x00);//   
	//IMX135SUNWINMIPI_write_cmos_sensor(0x3A63,0x00);//
    //[End]   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4100,0xF8);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xFF);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4344,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x441C,0x01);//

	//Size setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0340,0x06);//   0A
	IMX135SUNWINMIPI_write_cmos_sensor(0x0341,0x68);//   40
	IMX135SUNWINMIPI_write_cmos_sensor(0x0342,0x11);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0343,0xDC);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0344,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0345,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0346,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0347,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0348,0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0349,0x6F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034A,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034B,0x2F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034C,0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034D,0x38);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034E,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034F,0x18);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0350,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0351,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0352,0x00);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0353,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0354,0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0355,0x38);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0356,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0357,0x18);//	
	IMX135SUNWINMIPI_write_cmos_sensor(0x301D,0x30);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3310,0x08);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3311,0x38);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3312,0x06);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3313,0x18);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331C,0x04);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331D,0xAB);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4084,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4085,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4086,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4087,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4400,0x00);//

	//global timing setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0830,0x6F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0831,0x27);//  
	IMX135SUNWINMIPI_write_cmos_sensor(0x0832,0x4F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0833,0x2F);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0834,0x2F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0835,0x2F);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0836,0x9F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0837,0x37);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0839,0x1F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x083A,0x17);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x083B,0x02);// 

	// integration time setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0202,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0203,0x64);//

	//gain setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0205,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0210,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0211,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0212,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0213,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0214,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0215,0x00);//

#if 0
	//hdr setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0230,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0231,0x00);//     
	IMX135SUNWINMIPI_write_cmos_sensor(0x0233,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0234,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0235,0x40);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0236,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0238,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0239,0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x023B,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x023C,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B0,0x04);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B1,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B3,0X00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B4,0X01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3800,0X00);//

	#endif
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x01);// STREAM START

    SENSORDB("IMX135SUNWINMIPI_Sensor_VideoFullSizeSetting exit");

}

void IMX135SUNWINMIPI_PreviewSetting(void)
{	
    SENSORDB("IMX135SUNWINMIPI_PreviewSetting enter:");

    //PLL setting 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop

	//PLL setting
    IMX135SUNWINMIPI_write_cmos_sensor(0x011E,0x18);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x011F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0301,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0303,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0305,0x0B);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0309,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030B,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030C,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030D,0x09);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3A06,0x11);//   

	//Mode setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0108,0x03);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0112,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0113,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0381,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0383,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0385,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0387,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0390,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0391,0x22);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0392,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0401,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0404,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0405,0x10);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4082,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4083,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x7006,0x04);//

	//Optionnal function setting
    //[Begin]zhfan for  module otp PR 450709
	//IMX135SUNWINMIPI_write_cmos_sensor(0x0700,0x00);//   
	//IMX135SUNWINMIPI_write_cmos_sensor(0x3A63,0x00);//
    //[End]   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4100,0xF8);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xFF);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4344,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x441C,0x01);//

	//Size setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0340,0x06);//   0A
	IMX135SUNWINMIPI_write_cmos_sensor(0x0341,0x68);//   40
	IMX135SUNWINMIPI_write_cmos_sensor(0x0342,0x11);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0343,0xDC);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0344,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0345,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0346,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0347,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0348,0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0349,0x6F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034A,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034B,0x2F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034C,0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034D,0x38);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034E,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034F,0x18);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0350,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0351,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0352,0x00);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0353,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0354,0x08);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0355,0x38);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0356,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0357,0x18);//	
	IMX135SUNWINMIPI_write_cmos_sensor(0x301D,0x30);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3310,0x08);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3311,0x38);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3312,0x06);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3313,0x18);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331C,0x04);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331D,0xAB);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4084,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4085,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4086,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4087,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4400,0x00);//

	//global timing setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0830,0x6F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0831,0x27);//  
	IMX135SUNWINMIPI_write_cmos_sensor(0x0832,0x4F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0833,0x2F);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0834,0x2F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0835,0x2F);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0836,0x9F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0837,0x37);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0839,0x1F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x083A,0x17);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x083B,0x02);// 

	// integration time setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0202,0x06);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0203,0x64);//

	//gain setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0205,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0210,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0211,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0212,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0213,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0214,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0215,0x00);//

#if 0
	//hdr setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0230,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0231,0x00);//     
	IMX135SUNWINMIPI_write_cmos_sensor(0x0233,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0234,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0235,0x40);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0236,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0238,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0239,0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x023B,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x023C,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B0,0x04);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B1,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B3,0X00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B4,0X01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3800,0X00);//

	#endif
	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x01);// STREAM START


     SENSORDB("IMX135SUNWINMIPI_PreviewSetting exit");
}

void IMX135SUNWINMIPI_set_13M(void)
{	
	SENSORDB("IMX135SUNWINMIPI_set_13M Capture setting enter:");

	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop
	//PLL setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x011E,0x18);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x011F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0301,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0303,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0305,0x0B);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0309,0x05);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030B,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030C,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030D,0x29);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x030E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3A06,0x11);//   

	//Mode setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0108,0x03);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0112,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0113,0x0A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0381,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0383,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0385,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0387,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0390,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0391,0x11);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0392,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0401,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0404,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0405,0x10);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4082,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4083,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x7006,0x04);//

	//Optionnal function setting
    //[Begin]zhfan for  module otp PR 450709
	//IMX135SUNWINMIPI_write_cmos_sensor(0x0700,0x00);//   
	//IMX135SUNWINMIPI_write_cmos_sensor(0x3A63,0x00);//
    //[End]   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4100,0xF8);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4203,0xFF);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4344,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x441C,0x01);//

	//Size setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0340,0x0C);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0341,0x4A);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0342,0x15);//    //0x11
	IMX135SUNWINMIPI_write_cmos_sensor(0x0343,0x50);//    //DC
	IMX135SUNWINMIPI_write_cmos_sensor(0x0344,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0345,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0346,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0347,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0348,0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0349,0x6F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034A,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034B,0x2F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034C,0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034D,0x70);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034E,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x034F,0x30);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0350,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0351,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0352,0x00);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0353,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0354,0x10);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0355,0x70);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0356,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0357,0x30);//	
	IMX135SUNWINMIPI_write_cmos_sensor(0x301D,0x30);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3310,0x10);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3311,0x70);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3312,0x0C);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x3313,0x30);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331C,0x09);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x331D,0x77);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4084,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4085,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4086,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4087,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x4400,0x00);//

	//global timing setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0830,0x77);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0831,0x2F);//  
	IMX135SUNWINMIPI_write_cmos_sensor(0x0832,0x4F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0833,0x2F);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0834,0x2F);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0835,0x37);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0836,0xA7);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0837,0x37);// 
	IMX135SUNWINMIPI_write_cmos_sensor(0x0839,0x1F);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x083A,0x17);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x083B,0x02);// 

	// integration time setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0202,0x0C);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0203,0x46);//

	//gain setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0205,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020E,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x020F,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0210,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0211,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0212,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0213,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0214,0x01);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0215,0x00);//
#if 0
	//hdr setting
	IMX135SUNWINMIPI_write_cmos_sensor(0x0230,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0231,0x00);//     
	IMX135SUNWINMIPI_write_cmos_sensor(0x0233,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0234,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0235,0x40);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x0236,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0238,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x0239,0x04);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x023B,0x00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x023C,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B0,0x04);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B1,0x00);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B3,0X00);//   
	IMX135SUNWINMIPI_write_cmos_sensor(0x33B4,0X01);//
	IMX135SUNWINMIPI_write_cmos_sensor(0x3800,0X00);//

#endif	
	IMX135SUNWINMIPI_write_cmos_sensor(0x0100,0x01);//STREAM START
    SENSORDB("IMX135SUNWINMIPI_set_13M Capture setting exit");
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPIOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX135SUNWINMIPIOpen(void)
{
    int  retry = 0; 
    // check if sensor ID correct
    retry = 3; 
	kal_uint16 sensorid; 
	
	SENSORDB("[JRD_CAM]IMX135SUNWINMIPIOpen enter:\n"); 
	
    do {
       SENSORDB("Read ID in the Open function\n"); 
	   sensorid=(kal_uint16)((IMX135SUNWINMIPI_read_cmos_sensor(0x0016)<<8) | IMX135SUNWINMIPI_read_cmos_sensor(0x0017));  
	   spin_lock(&imx111_drv_lock);    
	   IMX135SUNWINMIPI_sensor_id =sensorid;  
	   spin_unlock(&imx111_drv_lock);
	   if (IMX135SUNWINMIPI_sensor_id == (IMX135SUNWIN_SENSOR_ID - 3))
			break; 
	   SENSORDB_ERR("Read Sensor ID Fail = 0x%04x\n", IMX135SUNWINMIPI_sensor_id);
	   retry--; 
	}
	while (retry > 0);
	
    printk("[JRD_CAM]IMX135SUNWINMIPIOpen Read Sensor ID = 0x%04x\n", IMX135SUNWINMIPI_sensor_id); 
    if (IMX135SUNWINMIPI_sensor_id != (IMX135SUNWIN_SENSOR_ID - 3))
        return ERROR_SENSOR_CONNECT_FAIL;
	
    IMX135SUNWINMIPI_Sensor_Init();
	SENSORDB("IMX135SUNWINMIPIOpen exit:\n"); 
	
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPIGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX135SUNWINMIPIGetSensorID(UINT32 *sensorID) 
{
	kal_uint16 sensorIDH = 0;
	kal_uint16 sensorIDL = 0;
    kal_uint8  Vendor_ID[IMX135SUNWINOTP_MDI_SIZE];
	kal_uint8  retry = 2; 
	kal_uint8  ret;
   
    printk("[JRD_CAM]IMX135SUNWINMIPIGetSensorID\n"); 
	
    do {		
		sensorIDH = (IMX135SUNWINMIPI_read_cmos_sensor(0x0016)<<8)&0xFF00;
		sensorIDL = IMX135SUNWINMIPI_read_cmos_sensor(0x0017) ;
		*sensorID = sensorIDH | sensorIDL;
		printk("[JRD_CAM]IMX135SUNWIN sensor ID =  0x%04x\n", *sensorID);
		if (*sensorID == (IMX135SUNWIN_SENSOR_ID - 3))
		{
			//otpRdStatus = IMX135SUNWINOTPShading1();//get OTP LSC data
			
			ret = IMX135SUNWINOTP_Get_Module_Info(Vendor_ID); //ret is checksum value
			printk("[JRD_CAM]IMX135SUNWIN Vendor_ID:0x%x 0x%x 0x%x 0x%x ret=0x%x\n", Vendor_ID[0], Vendor_ID[1], Vendor_ID[2], Vendor_ID[3], ret);
			
			if ( (IMX135SUNWIN_MID == Vendor_ID[0]) && (IMX135SUNWIN_LENSID == Vendor_ID[1]) )
			{
				printk("[JRD_CAM]IMX135SUNWIN sensor has been found\n");

			}else
			{
				*sensorID = 0;
				printk("[JRD_CAM]IMX135SUNWIN fail to check MID\n");
			}
			
			break;
		}
        retry--; 
	} while (retry > 0);

    if (*sensorID != (IMX135SUNWIN_SENSOR_ID - 3)) {
		printk("[JRD_CAM]IMX135SUNWIN Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of IMX135SUNWINMIPI to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX135SUNWINMIPI_SetShutter(kal_uint16 iShutter)
{
	SENSORDB("IMX135SUNWINMIPI_SetShutter:shutter=%d\n",iShutter);
   
    if (iShutter < 1)
        iShutter = 1; 
	else if(iShutter > 0xffff)
		iShutter = 0xffff;
	unsigned long flags;
	spin_lock_irqsave(&imx111_drv_lock,flags);
    IMX135SUNWINMIPI_sensor.pv_shutter = iShutter;	
	spin_unlock_irqrestore(&imx111_drv_lock,flags);
    IMX135SUNWINMIPI_write_shutter(iShutter);
}   /*  IMX135SUNWINMIPI_SetShutter   */

/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 IMX135SUNWINMIPI_read_shutter(void)
{
    return (UINT16)( (IMX135SUNWINMIPI_read_cmos_sensor(0x0202)<<8) | IMX135SUNWINMIPI_read_cmos_sensor(0x0203) );
}

/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPI_night_mode
*
* DESCRIPTION
*   This function night mode of IMX135SUNWINMIPI.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

///is not use in raw sensor
void IMX135SUNWINMIPI_NightMode(kal_bool bEnable)
{
#if 0
    /************************************************************************/
    /*                      Auto Mode: 30fps                                                                                          */
    /*                      Night Mode:15fps                                                                                          */
    /************************************************************************/
    if(bEnable)
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/15)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
            OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
            OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            OV5642_MAX_EXPOSURE_LINES = OV5642_CURRENT_FRAME_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
    else// Fix video framerate 30 fps
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/30)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            if(OV5642_pv_exposure_lines < (OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP)) // for avoid the shutter > frame_lines,move the frame lines setting to shutter function
            {
                OV5642_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
                OV5642_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
                OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            }
            OV5642_MAX_EXPOSURE_LINES = OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
#endif	
}/*	IMX135SUNWINMIPI_NightMode */

/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX135SUNWINMIPIClose(void)
{
    return ERROR_NONE;
}	/* IMX135SUNWINMIPIClose() */

void IMX135SUNWINMIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	
    iTemp = IMX135SUNWINMIPI_read_cmos_sensor(0x0101) & 0xFB;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            IMX135SUNWINMIPI_write_cmos_sensor(0x0101, iTemp);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            IMX135SUNWINMIPI_write_cmos_sensor(0x0101, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            IMX135SUNWINMIPI_write_cmos_sensor(0x0101, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            IMX135SUNWINMIPI_write_cmos_sensor(0x0101, iTemp | 0x03);	//Set mirror and flip
            break;
    }
}


/*************************************************************************
* FUNCTION
*   IMX135SUNWINMIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX135SUNWINMIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {   
        SENSORDB("IMX135SUNWINMIPIVideo enter:\n");
    	spin_lock(&imx111_drv_lock);    
        IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_TRUE;  
		IMX135SUNWINMIPI_sensor.video_mode=KAL_TRUE;
		IMX135SUNWINMIPI_sensor.pv_mode=KAL_FALSE;
		IMX135SUNWINMIPI_sensor.capture_mode=KAL_FALSE;
		spin_unlock(&imx111_drv_lock);
		IMX135SUNWINMIPI_Sensor_VideoFullSizeSetting();
		
		iStartX += IMX135SUNWINMIPI_IMAGE_SENSOR_VIDEO_STARTX;
		iStartY += IMX135SUNWINMIPI_IMAGE_SENSOR_VIDEO_STARTY;
		spin_lock(&imx111_drv_lock);	
		IMX135SUNWINMIPI_sensor.cp_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.cp_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.pv_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.pv_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.video_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.video_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.video_line_length = IMX135SUNWINMIPI_VIDEO_LINE_LENGTH_PIXELS+IMX135SUNWINMIPI_sensor.video_dummy_pixels; 
		IMX135SUNWINMIPI_sensor.video_frame_length = IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES+IMX135SUNWINMIPI_sensor.video_dummy_lines;
		spin_unlock(&imx111_drv_lock);
		
		IMX135SUNWINMIPI_SetDummy(IMX135SUNWINMIPI_sensor.video_dummy_pixels,IMX135SUNWINMIPI_sensor.video_dummy_lines);
		IMX135SUNWINMIPI_SetShutter(IMX135SUNWINMIPI_sensor.video_shutter);
		memcpy(&IMX135SUNWINMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
		image_window->GrabStartX= iStartX;
		image_window->GrabStartY= iStartY;
		SENSORDB("IMX135SUNWINMIPIVideo exit:\n");
	//	image_window->ExposureWindowWidth= IMX135SUNWINMIPI_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
	//	image_window->ExposureWindowHeight= IMX135SUNWINMIPI_IMAGE_SENSOR_PV_HEIGHT - 2*iStartY;
    }
    else
    {
		SENSORDB("IMX135SUNWINMIPIPreview enter:\n");
    	spin_lock(&imx111_drv_lock);    
        IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_FALSE;
		IMX135SUNWINMIPI_sensor.video_mode=KAL_FALSE;
		IMX135SUNWINMIPI_sensor.pv_mode=KAL_TRUE;
		IMX135SUNWINMIPI_sensor.capture_mode=KAL_FALSE;
		spin_unlock(&imx111_drv_lock);
             IMX135SUNWINMIPI_PreviewSetting();
		iStartX += IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTX;
		iStartY += IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTY;
		spin_lock(&imx111_drv_lock);	
		IMX135SUNWINMIPI_sensor.cp_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.cp_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.pv_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.pv_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.video_dummy_pixels = 0;
		IMX135SUNWINMIPI_sensor.video_dummy_lines = 0;
		IMX135SUNWINMIPI_sensor.pv_line_length = IMX135SUNWINMIPI_PV_LINE_LENGTH_PIXELS+IMX135SUNWINMIPI_sensor.pv_dummy_pixels; 
		IMX135SUNWINMIPI_sensor.pv_frame_length = IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES+IMX135SUNWINMIPI_sensor.pv_dummy_lines;
		spin_unlock(&imx111_drv_lock);
		
		IMX135SUNWINMIPI_SetDummy(IMX135SUNWINMIPI_sensor.pv_dummy_pixels,IMX135SUNWINMIPI_sensor.pv_dummy_lines);
		IMX135SUNWINMIPI_SetShutter(IMX135SUNWINMIPI_sensor.pv_shutter);
		memcpy(&IMX135SUNWINMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
		image_window->GrabStartX= iStartX;
		image_window->GrabStartY= iStartY;
		image_window->ExposureWindowWidth= IMX135SUNWINMIPI_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
		image_window->ExposureWindowHeight= IMX135SUNWINMIPI_IMAGE_SENSOR_PV_HEIGHT - 2*iStartY;
		SENSORDB("IMX135SUNWINMIPIPreview exit:\n");

    }
    
    IMX135SUNWINMIPISetFlipMirror(IMAGE_HV_MIRROR);
	
	return ERROR_NONE;
}	/* IMX135SUNWINMIPIPreview() */

UINT32 IMX135SUNWINMIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
	
	SENSORDB("IMX135SUNWINMIPICapture enter:\n"); 
	spin_lock(&imx111_drv_lock);	
	IMX135SUNWINMIPI_sensor.video_mode=KAL_FALSE;
	IMX135SUNWINMIPI_sensor.pv_mode=KAL_FALSE;
	IMX135SUNWINMIPI_sensor.capture_mode=KAL_TRUE;
    IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_FALSE; 
    IMX135SUNWINMIPI_Auto_Flicker_mode = KAL_FALSE;   
	spin_unlock(&imx111_drv_lock);

		spin_lock(&imx111_drv_lock);    
    IMX135SUNWINMIPI_sensor.cp_dummy_pixels= 0;
    IMX135SUNWINMIPI_sensor.cp_dummy_lines = 0;   
		spin_unlock(&imx111_drv_lock);
        IMX135SUNWINMIPI_set_13M();

    IMX135SUNWINMIPISetFlipMirror(IMAGE_HV_MIRROR);
    
	spin_lock(&imx111_drv_lock);    
	IMX135SUNWINMIPI_sensor.cp_line_length=IMX135SUNWINMIPI_FULL_LINE_LENGTH_PIXELS+IMX135SUNWINMIPI_sensor.cp_dummy_pixels;
	IMX135SUNWINMIPI_sensor.cp_frame_length=IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES+IMX135SUNWINMIPI_sensor.cp_dummy_lines;
	spin_unlock(&imx111_drv_lock);
	iStartX = IMX135SUNWINMIPI_IMAGE_SENSOR_CAP_STARTX;
	iStartY = IMX135SUNWINMIPI_IMAGE_SENSOR_CAP_STARTY;
	image_window->ExposureWindowWidth=IMX135SUNWINMIPI_IMAGE_SENSOR_FULL_WIDTH -2*iStartX;
	image_window->ExposureWindowHeight=IMX135SUNWINMIPI_IMAGE_SENSOR_FULL_HEIGHT-2*iStartY;
	spin_lock(&imx111_drv_lock);	
    memcpy(&IMX135SUNWINMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&imx111_drv_lock);
	SENSORDB("IMX135SUNWINMIPICapture exit:\n"); 

    return ERROR_NONE;
}	/* IMX135SUNWINMIPICapture() */

UINT32 IMX135SUNWINMIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorPreviewWidth	= IMX135SUNWINMIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= IMX135SUNWINMIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= IMX135SUNWINMIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= IMX135SUNWINMIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= IMX135SUNWINMIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = IMX135SUNWINMIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("IMX135SUNWINMIPIGetResolution\n");    

    return ERROR_NONE;
}   /* IMX135SUNWINMIPIGetResolution() */

UINT32 IMX135SUNWINMIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("IMX135SUNWINMIPIGetInfo start\n");
	switch(ScenarioId){
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
				pSensorInfo->SensorPreviewResolutionX=IMX135SUNWINMIPI_REAL_CAP_WIDTH;
				pSensorInfo->SensorPreviewResolutionY=IMX135SUNWINMIPI_REAL_CAP_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorPreviewResolutionX=IMX135SUNWINMIPI_REAL_VIDEO_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=IMX135SUNWINMIPI_REAL_VIDEO_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
		default:
        	pSensorInfo->SensorPreviewResolutionX=IMX135SUNWINMIPI_REAL_PV_WIDTH;
        	pSensorInfo->SensorPreviewResolutionY=IMX135SUNWINMIPI_REAL_PV_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}

    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=15;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R; 
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2; 
    pSensorInfo->PreviewDelayFrame = 2; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
     //   case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTY;           		
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
		
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		   pSensorInfo->SensorClockFreq=24;
		   pSensorInfo->SensorClockDividCount= 5;
		   pSensorInfo->SensorClockRisingCount= 0;
		   pSensorInfo->SensorClockFallingCount= 2;
		   pSensorInfo->SensorPixelClockCount= 3;
		   pSensorInfo->SensorDataLatchCount= 2;
		   pSensorInfo->SensorGrabStartX = IMX135SUNWINMIPI_IMAGE_SENSOR_VIDEO_STARTX; 
		   pSensorInfo->SensorGrabStartY = IMX135SUNWINMIPI_IMAGE_SENSOR_VIDEO_STARTY;				   
		   pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;		   
		   pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		   pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		   pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
		   pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
		   pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
		   pSensorInfo->SensorPacketECCOrder = 1;

			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
       // case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = IMX135SUNWINMIPI_IMAGE_SENSOR_CAP_STARTX;	//2*IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = IMX135SUNWINMIPI_IMAGE_SENSOR_CAP_STARTY;	//2*IMX135SUNWINMIPI_IMAGE_SENSOR_PV_STARTY;          			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 1;             
            break;
    }
	spin_lock(&imx111_drv_lock);	

    IMX135SUNWINMIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
	
	spin_unlock(&imx111_drv_lock);
    memcpy(pSensorConfigData, &IMX135SUNWINMIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* IMX135SUNWINMIPIGetInfo() */


UINT32 IMX135SUNWINMIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock(&imx111_drv_lock);	
	IMX135SUNWINMIPI_CurrentScenarioId = ScenarioId;
	spin_unlock(&imx111_drv_lock);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            IMX135SUNWINMIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            IMX135SUNWINMIPICapture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
} /* IMX135SUNWINMIPIControl() */

UINT32 IMX135SUNWINMIPISetVideoMode(UINT16 u2FrameRate)
{
	kal_uint16 IMX135SUNWINMIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("IMX135SUNWINMIPISetVideoMode  u2FrameRate = %d\n", u2FrameRate);
	/*
	if(u2FrameRate>=25)
		return ERROR_NONE;
	*/
	if(u2FrameRate==0)
	{
		spin_lock(&imx111_drv_lock);
		IMX135SUNWINMIPI_sensor.fix_video_fps = KAL_FALSE;
		IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_FALSE;
		spin_unlock(&imx111_drv_lock);
		return ERROR_NONE;
	}

	spin_lock(&imx111_drv_lock);
	IMX135SUNWINMIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&imx111_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[Enter Fix_fps func] IMX135SUNWINMIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);

	IMX135SUNWINMIPI_Video_Max_Expourse_Time = (kal_uint16)((IMX135SUNWINMIPI_sensor.video_pclk*10/u2FrameRate)/IMX135SUNWINMIPI_sensor.video_line_length);

	if (IMX135SUNWINMIPI_Video_Max_Expourse_Time > IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES/*IMX135SUNWINMIPI_sensor.pv_frame_length*/) 
	{
		spin_lock(&imx111_drv_lock);    
		IMX135SUNWINMIPI_sensor.video_frame_length = IMX135SUNWINMIPI_Video_Max_Expourse_Time;
		IMX135SUNWINMIPI_sensor.video_dummy_lines = IMX135SUNWINMIPI_sensor.video_frame_length-IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES;
		spin_unlock(&imx111_drv_lock);
		SENSORDB("%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,IMX135SUNWINMIPI_sensor.video_frame_length,IMX135SUNWINMIPI_sensor.video_dummy_lines);
		IMX135SUNWINMIPI_SetDummy(IMX135SUNWINMIPI_sensor.video_dummy_pixels,IMX135SUNWINMIPI_sensor.video_dummy_lines);
	}

	IMX135SUNWINMIPISetFlipMirror(IMAGE_HV_MIRROR);
		
	spin_lock(&imx111_drv_lock);    
	IMX135SUNWINMIPI_MPEG4_encode_mode = KAL_TRUE; 
	spin_unlock(&imx111_drv_lock);
	
    return ERROR_NONE;
}

UINT32 IMX135SUNWINMIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32 pv_max_frame_rate_lines=0;
#if 0
if(IMX135SUNWINMIPI_sensor.pv_mode==TRUE)
	pv_max_frame_rate_lines=IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES;
else
    pv_max_frame_rate_lines=IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES	;

    SENSORDB("[IMX135SUNWINMIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) {   // enable auto flicker   
    	spin_lock(&imx111_drv_lock);    
        IMX135SUNWINMIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&imx111_drv_lock);
        if(IMX135SUNWINMIPI_MPEG4_encode_mode == KAL_TRUE) {    // in the video mode, reset the frame rate
            pv_max_frame_rate_lines = IMX135SUNWINMIPI_MAX_EXPOSURE_LINES + (IMX135SUNWINMIPI_MAX_EXPOSURE_LINES>>7);            
            IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 1);        
            IMX135SUNWINMIPI_write_cmos_sensor(0x0340, (pv_max_frame_rate_lines >>8) & 0xFF);
            IMX135SUNWINMIPI_write_cmos_sensor(0x0341, pv_max_frame_rate_lines & 0xFF);	
            IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 0);        	
        }
    } else {
    	spin_lock(&imx111_drv_lock);    
        IMX135SUNWINMIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&imx111_drv_lock);
        if(IMX135SUNWINMIPI_MPEG4_encode_mode == KAL_TRUE) {    // in the video mode, restore the frame rate
            IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 1);        
            IMX135SUNWINMIPI_write_cmos_sensor(0x0340, (IMX135SUNWINMIPI_MAX_EXPOSURE_LINES >>8) & 0xFF);
            IMX135SUNWINMIPI_write_cmos_sensor(0x0341, IMX135SUNWINMIPI_MAX_EXPOSURE_LINES & 0xFF);	
            IMX135SUNWINMIPI_write_cmos_sensor(0x0104, 0);        	
        }
        SENSORDB("Disable Auto flicker\n");    
    }
	#endif
    return ERROR_NONE;
}

UINT32 IMX135SUNWINMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("IMX135SUNWINMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =IMX135SUNWINMIPI_sensor.pv_pclk;
			lineLength = IMX135SUNWINMIPI_PV_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX135SUNWINMIPI_PV_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			IMX135SUNWINMIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = IMX135SUNWINMIPI_sensor.cp_pclk;
			lineLength = IMX135SUNWINMIPI_VIDEO_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX135SUNWINMIPI_VIDEO_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			IMX135SUNWINMIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = IMX135SUNWINMIPI_sensor.video_pclk;
			lineLength = IMX135SUNWINMIPI_FULL_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX135SUNWINMIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			IMX135SUNWINMIPI_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}

UINT32 IMX135SUNWINMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	SENSORDB("IMX135SUNWINMIPIGetDefaultFramerateByScenario Test pattern enable\n");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 150;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

UINT32 IMX135SUNWINMIPISetTestPatternMode(kal_bool bEnable)  //daikan
{
    SENSORDB("IMX135SUNWINMIPISetTestPatternMode Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        IMX135SUNWINMIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        IMX135SUNWINMIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        IMX135SUNWINMIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        IMX135SUNWINMIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 IMX135SUNWINMIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        	switch(IMX135SUNWINMIPI_CurrentScenarioId)
    		{
    			case MSDK_SCENARIO_ID_CAMERA_ZSD:
    		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara16++=IMX135SUNWINMIPI_sensor.cp_line_length;  
		            *pFeatureReturnPara16=IMX135SUNWINMIPI_sensor.cp_frame_length;
	            	SENSORDB("Sensor period:%d %d\n",IMX135SUNWINMIPI_sensor.cp_line_length, IMX135SUNWINMIPI_sensor.cp_frame_length); 
	            	*pFeatureParaLen=4;        				
    				break;
    			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=IMX135SUNWINMIPI_sensor.video_line_length;  
					*pFeatureReturnPara16=IMX135SUNWINMIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", IMX135SUNWINMIPI_sensor.video_line_length, IMX135SUNWINMIPI_sensor.video_frame_length); 
					 *pFeatureParaLen=4;
					break;
    			default:	
					*pFeatureReturnPara16++=IMX135SUNWINMIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=IMX135SUNWINMIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", IMX135SUNWINMIPI_sensor.pv_line_length, IMX135SUNWINMIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
            		break;
      		}
        	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        	switch(IMX135SUNWINMIPI_CurrentScenarioId)
    		{
    			case MSDK_SCENARIO_ID_CAMERA_ZSD:
    			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	            *pFeatureReturnPara32 = IMX135SUNWINMIPI_sensor.cp_pclk; 
	            *pFeatureParaLen=4;		         	
	         		break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = IMX135SUNWINMIPI_sensor.video_pclk;
					*pFeatureParaLen=4;
					break;
	         		default:
	            *pFeatureReturnPara32 = IMX135SUNWINMIPI_sensor.pv_pclk;
	            *pFeatureParaLen=4;
	            break;
	         }
		     break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            IMX135SUNWINMIPI_SetShutter(*pFeatureData16);
			SENSORDB("shutter&gain test by hhl:IMX135SUNWINMIPI_SetShutter in feature ctrl\n"); 
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC:
			SENSORDB("hhl'test the function of the sync cased\n"); 
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            IMX135SUNWINMIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           IMX135SUNWINMIPI_SetGain((UINT16) *pFeatureData16);
            
			SENSORDB("shutter&gain test by hhl:IMX135SUNWINMIPI_SetGain in feature ctrl\n"); 
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&imx111_drv_lock);    
            IMX135SUNWINMIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&imx111_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			IMX135SUNWINMIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = IMX135SUNWINMIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&imx111_drv_lock);    
                IMX135SUNWINMIPISensorCCT[i].Addr=*pFeatureData32++;
                IMX135SUNWINMIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&imx111_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX135SUNWINMIPISensorCCT[i].Addr;
                *pFeatureData32++=IMX135SUNWINMIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&imx111_drv_lock);    
                IMX135SUNWINMIPISensorReg[i].Addr=*pFeatureData32++;
                IMX135SUNWINMIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&imx111_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX135SUNWINMIPISensorReg[i].Addr;
                *pFeatureData32++=IMX135SUNWINMIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=IMX135SUNWIN_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, IMX135SUNWINMIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, IMX135SUNWINMIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &IMX135SUNWINMIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            IMX135SUNWINMIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            IMX135SUNWINMIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=IMX135SUNWINMIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            IMX135SUNWINMIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            IMX135SUNWINMIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            IMX135SUNWINMIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gb;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            IMX135SUNWINMIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            IMX135SUNWINMIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            IMX135SUNWINMIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            IMX135SUNWINMIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			IMX135SUNWINMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			IMX135SUNWINMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		//add for jrd camera debug log begin
		case SENSOR_FEATURE_DEBUG_LOG:
			IMX135SUNWINDebugLog((kal_bool)*pFeatureData16); 
			break;
		//add for jrd camera debug log end

        default:
            break;
    }
    return ERROR_NONE;
}	/* IMX135SUNWINMIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncIMX135SUNWINMIPI=
{
    IMX135SUNWINMIPIOpen,
    IMX135SUNWINMIPIGetInfo,
    IMX135SUNWINMIPIGetResolution,
    IMX135SUNWINMIPIFeatureControl,
    IMX135SUNWINMIPIControl,
    IMX135SUNWINMIPIClose
};

UINT32 IMX135SUNWIN_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncIMX135SUNWINMIPI;

    return ERROR_NONE;
}   /* SensorInit() */
