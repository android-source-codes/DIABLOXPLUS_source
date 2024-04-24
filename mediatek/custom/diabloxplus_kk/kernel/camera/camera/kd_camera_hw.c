/*JRD BSP TCT SH, please don't merge MTK patch in this file*/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/kernel.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX 	"[JRD_CAM][kd_camera_hw]"
#define PFX_ERR "[JRD_CAM_ERR][kd_camera_hw]"

#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERROR, PFX_ERR , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

kal_bool searchMainSensor = KAL_TRUE;
extern void ISP_MCLK1_EN(BOOL En);
static void setCameraPin(u16 pinNum, u8 mode, u8 dir, u8 level)
{
    if(mt_set_gpio_mode(pinNum, mode)){PK_DBG("set gpio mode failed!! \n");}
    if(mt_set_gpio_out(pinNum, level)){PK_DBG("set gpio failed!! \n");}
    if(mt_set_gpio_dir(pinNum, dir)){PK_DBG("set gpio dir failed!! \n");}
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

	u32 pinSet[2][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                     GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                     GPIO_OUT_ONE,                   /* ON state */
                     GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                     GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                     GPIO_OUT_ONE,
                     GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                     GPIO_OUT_ONE,
                     GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                     GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                     GPIO_OUT_ONE,
                     GPIO_OUT_ZERO,
                    },
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
		searchMainSensor = KAL_TRUE;
    }else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
		searchMainSensor = KAL_FALSE;
    }
   
    //power ON
    if ( On ) {
		if (currSensorName && (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) && ((0 == strcmp(SENSOR_DRVNAME_IMX135TRULY_MIPI_RAW, currSensorName))
			|| (0 == strcmp(SENSOR_DRVNAME_IMX135SUNWIN_MIPI_RAW, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_IMX135BLX_MIPI_RAW, currSensorName))))
		{
			PK_DBG("PowerOn kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);	 

            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                //PDN pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
            }
			
			if(GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                //Reset pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
            }           

            //disable inactive sensor
            if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
                //PDN pin
                if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
            }
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
                //Reset pin
                if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST], pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
            }

            mdelay(1);
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))//AVDD=2.8v
            {
              PK_DBG("Fail to enable analog power\n");
              goto _kdCISModulePowerOn_exit_;
            }

            //Power on DVDD 1.05V
            if(mt_set_gpio_mode(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN, GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN_M_GPIO)){PK_DBG("set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN, GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN, GPIO_OUT_ONE)){PK_DBG("set gpio failed!! \n");}
         
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))//IOVDD=1.8 V
            {
              PK_DBG("Fail to enable IO power\n");
              goto _kdCISModulePowerOn_exit_;
            }

			//power on AF
		    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
			
            mdelay(5);

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                //PDN pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
            }
            
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                //RST pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
            }

			//Enable AF VCM
            if(mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN, GPIO_CAMERA_AF_EN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ONE)){PK_DBG("set gpio failed!! \n");}

            mdelay(10);
        }		
		else if (currSensorName && (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_OV13850_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOn kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);	

			//First Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
			}
			
			if(GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				//Reset pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
			}			

			//disable inactive sensor
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
				//Reset pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST], pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG(" set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}
			}
			
			//just for temp  diable 1.05v DVDD
			if(mt_set_gpio_mode(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN_M_GPIO)){PK_DBG("[ set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			mdelay(1);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))//AVDD=2.8v
			{
			  PK_DBG("Fail to enable analog power\n");
			  goto _kdCISModulePowerOn_exit_;
			}
			
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))//IOVDD=1.8 V
			{
			  PK_DBG("Fail to enable IO power\n");
			  goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))//DVDD=1.2v
			{
			  PK_DBG("Fail to enable analog power\n");
			  goto _kdCISModulePowerOn_exit_;
			}
		 
			//power on AF
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			
			//Enable AF VCM
			if(mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN, GPIO_CAMERA_AF_EN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ONE)){PK_DBG("set gpio failed!! \n");}
			
			mdelay(2);

			//enable active sensor
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}
			
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				//RST pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			mdelay(10);
		}
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_IMX132_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOn kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//First Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {	
				//Reset pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}

			//disable inactive sensor
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
				//Reset pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST], pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
			 PK_DBG("Fail to enable digital power\n");
			 goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
			 PK_DBG("Fail to enable analog power\n");
			 goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D_SUB, VOL_1300,mode_name))
			{
			  PK_DBG("Fail to enable digital power\n");
			  goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			//enable active sensor
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				//RST pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			mdelay(2);
		}
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOn kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//First Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {	
				//Reset pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}

			//disable inactive sensor
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
				//Reset pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST], pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
			 PK_DBG("Fail to enable digital power\n");
			 goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
			 PK_DBG("Fail to enable analog power\n");
			 goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			//enable active sensor
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				//RST pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			mdelay(10);
		}
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOn kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//First Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {	
				//Reset pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}

			//disable inactive sensor
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
				//Reset pin
				if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST], pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],  GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			mdelay(1);
			
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))//DOVDD
			{
			 PK_DBG("Fail to enable digital power\n");
			 goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D_SUB, VOL_1800,mode_name))//DVDD
			{
			  PK_DBG("Fail to enable digital power\n");
			  goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
			
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))//AVDD
			{
			 PK_DBG("Fail to enable analog power\n");
			 goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			//enable active sensor
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_ERR("set gpio failed!! \n");}
			}
			udelay(100);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				//RST pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_ERR("set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_ERR("set gpio failed!! \n");}
			}

			mdelay(2);
		}
		else{
			PK_DBG("PowerOn don't match any sensor sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);
		} 
				
	}
  	else {//power OFF
		if (currSensorName && (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) && ((0 == strcmp(SENSOR_DRVNAME_IMX135TRULY_MIPI_RAW, currSensorName))
			|| (0 == strcmp(SENSOR_DRVNAME_IMX135SUNWIN_MIPI_RAW, currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_IMX135BLX_MIPI_RAW, currSensorName))))
		{
			PK_DBG("PowerOff kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);	
			
			//disable main--RESET pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMRST_PIN,GPIO_CAMERA_CMRST_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMRST_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			//disable main--PDN pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}

			//AF PWD Disable
			if(mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN, GPIO_CAMERA_AF_EN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[ set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
			
			mdelay(5);

			//power off AF
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("Fail to OFF analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)){//IOVDD=1.8V
				PK_DBG("Fail to enable digital power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			//Power down DVDD
			if(mt_set_gpio_mode(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN_M_GPIO)){PK_DBG("[ set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_MAIN_DVDD_ENADBLE_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {//AVDD=2.8v
				PK_DBG("Fail to OFF analog power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);
		}
		else if (currSensorName && (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_OV13850_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOff kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);	
			
			//disable main--RESET pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMRST_PIN,GPIO_CAMERA_CMRST_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMRST_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			//disable main--PDN pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}

			//AF PWD Disable
			if(mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN, GPIO_CAMERA_AF_EN_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[ set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
			
			mdelay(5);

			//power off AF
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			{
				PK_DBG("Fail to OFF analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)){//DVDD=1.2V
				PK_DBG("Fail to enable digital power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)){//IOVDD=1.8V
				PK_DBG("Fail to enable digital power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {//AVDD=2.8v
				PK_DBG("Fail to OFF analog power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);
		}
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_IMX132_MIPI_RAW, currSensorName)))
		{
            PK_DBG("PowerOff kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//mdelay(1);
			//disable sub--PDN pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}

			//disable sub--RESET pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			mdelay(5);//>500ns

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_ERR("Fail to OFF camera analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D_SUB, mode_name)) {
               PK_ERR("Fail to OFF camera digital power\n");
               goto _kdCISModulePowerOn_exit_;
          	}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_ERR("Fail to OFF camera analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
			mdelay(2);
        }
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName)))
		{
            PK_DBG("PowerOff kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//disable sub--PDN pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){PK_DBG("set gpio failed!! \n");}

			//disable sub--RESET pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			mdelay(5);//>500ns

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_ERR("Fail to OFF camera analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            //if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D_SUB, mode_name)) {
            //   PK_ERR("Fail to OFF camera digital power\n");
            //   goto _kdCISModulePowerOn_exit_;
          	//}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_ERR("Fail to OFF camera analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
			mdelay(10);
        }
		else if (currSensorName && (DUAL_CAMERA_SUB_SENSOR == SensorIdx) && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName)))
		{
			PK_DBG("PowerOff kdCISModulePowerOn sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);

			//disable sub--PDN pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){PK_DBG("set gpio failed!! \n");}
			udelay(50);
			//disable sub--RESET pin to low
			if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO)){PK_DBG(" set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}

			mdelay(1);//>500ns

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
				PK_ERR("Fail to OFF camera analog power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
			{
				PK_ERR("Fail to OFF camera analog power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D_SUB, mode_name)) {
			   PK_ERR("Fail to OFF camera digital power\n");
			   goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);
		}
		else{
			PK_DBG("PowerOff don't match any sensor sensorIdx:%d currSensorName=%s\n",SensorIdx, currSensorName);
		} 
  }
return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);

